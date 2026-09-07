/* hal_sim.cpp: seam 1 as a fake -- the host test double and the on-device sim. Filtered
   out of the bench env by build_src_filter. */
#include "hal.h"
#include "sim.h"
#include "config.h"
#include "pins.h"
#include "noinit.h"
#include "pulses.h"
#include <string.h>
#ifndef PB_NATIVE
#include "sim_console.h"
#endif

#if PB_SIM

/* PIN_PUMP_EN exists only in the pump owner's translation unit, so this file names D6
   itself. A4 == 18 and A5 == 19 in the UNO R4 WiFi variant. */
#define SIM_PUMP_PIN 6
#define SIM_PIN_SDA  18
#define SIM_PIN_SCL  19
#define SIM_WDT_RELOAD 16384u
/* The PCF8575's P4 bit, mirrored from sensors.cpp's EXP_HOME_BIT: the fake models what the
   pin physically does; sensors.cpp alone says what it means. */
#define SIM_EXP_HOME_BIT (1u << 4)

/* ---- clock ---- */
static uint32_t g_us, g_ms;
/* g_ms is its own counter, never derived from g_us: g_us is a real microsecond register
   with its own ~71.6 minute wrap, like a chip's micros() against millis()'s ~49.7 days, so
   g_us / 1000 could never exceed ~4.29 million. g_ms_frac_us is the sub-millisecond
   remainder hal_delay_us() carries between calls. */
static uint32_t g_ms_frac_us;

/* ---- D6 ---- */
static bool     g_pump_on;
static uint32_t g_pump_on_us;              /* cumulative time asserted */
static uint32_t g_pump_on_at_ms;           /* when the current assertion started */

/* ---- watchdog ---- */
static bool     g_wdt_running;
static uint32_t g_wdt_counter = SIM_WDT_RELOAD;
static uint32_t g_wdt_rate_hz = 2929;      /* PCLKB/8192 = 2929.7 Hz */
static uint32_t g_wdt_frac;                /* carried remainder, in counts * 1e6 */
static uint32_t g_wdt_delta;
static uint32_t g_feeds;

/* ---- injector state ---- */
static bool     g_float_ok = true;
/* Per-sample float pattern. Empty means "use g_float_ok". A non-empty pattern advances one
   character per read of PIN_HALL_FLOAT and sticks at the last character rather than
   wrapping. */
static char     g_float_pat[16];
static size_t   g_float_pat_len;
static size_t   g_float_pat_idx;
static uint16_t g_flow_ml_s;
static uint32_t g_storm_hz;
/* The pump-relative injectors: each is an (armed, offset, payload) triple plus a separate
   live deadline, so a second dose in the same test re-schedules from ITS pump-on. */
static bool     g_storm_on_armed; static uint32_t g_storm_on_hz;
static uint32_t g_burst_left;                     /* flow pulses still owed */
static bool     g_float_at_armed; static uint32_t g_float_at_off_ms; static bool g_float_at_ok;
static bool     g_float_due;      static uint32_t g_float_due_us;
static bool     g_rx_at_armed;    static uint32_t g_rx_at_off_ms;
static char     g_rx_at_buf[64];
static bool     g_rx_due;         static uint32_t g_rx_due_us;
static void   (*g_on_pump_on)(void);   /* fires once, from inside the next pump-on
                                          write, then clears itself */
static bool     g_i2c_fail;
static bool     g_mux_stuck;
static bool     g_stall;
static bool     g_leak;
static uint16_t g_chan[16];
static uint16_t g_exp_port = 0xFFFFu;      /* the PCF8575's latch */
static uint8_t  g_mux_sel;
static bool     g_adc_settled;
static uint16_t g_adc_prev;
static uint16_t g_servo_us = 1500u;        /* 1500 == stopped */
static uint32_t g_servo_stops;             /* writes of 1500 */

/* ---- the screw and home region. g_screw_pos is the fake's own physical position, in
   pulses -- separate from pulses.cpp's directionless total of accepted edges, exactly like
   the real hall. Saturates at 0: the threadless start of the screw is a hard stop. ---- */
static uint32_t g_screw_pulse_ms;          /* 0 == the screw does not turn at all */
static uint32_t g_home_lo, g_home_hi;      /* sim_reset() sets [0, 40] */
static uint32_t g_screw_pos;

/* ---- capacities: a host default and a device default. The host suites capture whole
   console lines and dose-length event traces; the sim binary has to fit these inside a
   32 KB part that also carries the framework and the WiFi headers. Nothing on the device
   reads g_rx/g_tx back (its console goes through sim_console.cpp's real UART), but the
   injectors are still compiled, so the arrays cannot be zero. ---- */
#ifdef PB_NATIVE
#define SIM_RX_CAP 256
#define SIM_TX_CAP 4096
#define SIM_EV_CAP 1024
#else
#define SIM_RX_CAP 32
#define SIM_TX_CAP 64
#define SIM_EV_CAP 64
#endif

/* ---- serial ---- */
static char   g_rx[SIM_RX_CAP]; static size_t g_rx_len, g_rx_pos;
static char   g_tx[SIM_TX_CAP]; static size_t g_tx_len;

/* ---- call trace ---- */
static sim_ev_t g_ev[SIM_EV_CAP];
static size_t   g_ev_n;

static void ev_(sim_ev_kind_t k, uint8_t pin, uint32_t arg) {
  if (g_ev_n >= sizeof g_ev / sizeof g_ev[0]) return;
  g_ev[g_ev_n].kind = k; g_ev[g_ev_n].pin = pin;
  g_ev[g_ev_n].arg = arg; g_ev[g_ev_n].at_ms = g_ms;
  g_ev_n++;
}
size_t sim_events(const sim_ev_t **out) { *out = g_ev; return g_ev_n; }
void   sim_events_clear(void) { g_ev_n = 0; }

/* ---- the clock, and every model that runs off it ---- */
static void tick_models_(uint32_t us) {
  if (g_wdt_running && g_wdt_rate_hz) {
    g_wdt_frac += g_wdt_rate_hz * us;                  /* counts * 1e6 */
    uint32_t ticks = g_wdt_frac / 1000000u;
    g_wdt_frac %= 1000000u;
    g_wdt_counter = (g_wdt_counter > ticks) ? (g_wdt_counter - ticks) : 0u;
  }
  if (g_pump_on) g_pump_on_us += us;
}

static uint32_t g_next_flow_us, g_next_screw_us;

/* now_ms is the millisecond this step reaches by its end (g_ms + 1): emit_() runs between
   tick_models_() and the clock assignment, so the bare global is one step stale against
   what hal_millis() is about to return, and an edge gated on it would land a whole
   millisecond late against any `>=` deadline.

   The onset ramps rather than stepping: a pure step at PB_PRIME_MS_DEFAULT fires in the
   same millisecond the prime rule samples the count, so a healthy dose could never show
   PB_PRIME_MIN_PULSES in time. A real pump spins up, and the water standing in a wet line
   accelerates over a short span; SIM_FLOW_ONSET_MS models that as a linear ramp from
   pump-on. A dry line is g_flow_ml_s staying 0. A storm is not ramped: it models
   electrical noise on a floating D2, which has no motor and starts at full rate the instant
   the pump leg beside it is energised. */
#define SIM_FLOW_ONSET_MS 150u
static uint32_t sim_flow_hz_(uint32_t now_ms) {
  if (g_storm_hz) return g_storm_hz;
  if (g_pump_on && g_flow_ml_s) {
    uint32_t full_hz  = ((uint32_t)g_flow_ml_s * (uint32_t)PB_PULSES_PER_L_DEFAULT) / 1000u;
    uint32_t since_on = now_ms - g_pump_on_at_ms;
    if (since_on >= SIM_FLOW_ONSET_MS) return full_hz;
    return (full_hz * since_on) / SIM_FLOW_ONSET_MS;   /* linear ramp, integer only */
  }
  if (!g_pump_on && g_leak) return 1u;      /* a slow weep past a closed gate */
  return 0u;
}

/* A period of sim_set_screw_pulse_ms(), converted to Hz. 0 (the default) means the screw
   does not turn: a silent fallback rate would give the screw two disagreeing speeds if a
   case forgot to set one. */
static uint32_t sim_screw_hz_(void) {
  if (g_stall || g_servo_us == 1500u || g_servo_us == 0u || g_screw_pulse_ms == 0u) return 0u;
  return 1000u / g_screw_pulse_ms;
}

/* true only while the physical position sits inside the home REGION -- a point would
   never be found by a bounded traverse landing one pulse either side of it. */
static bool screw_home_(void) { return g_screw_pos >= g_home_lo && g_screw_pos <= g_home_hi; }

/* The screw ISR: counts the edge like the real hall (directionless), raises SIM_EV_SCREW,
   and walks the fake's physical position in the direction the servo was last commanded. */
static void screw_isr_(void) {
  pulses_isr_screw();
  ev_(SIM_EV_SCREW, PIN_HALL_SCREW, g_servo_us);
  if (g_servo_us > 1500u)      { g_screw_pos++; }
  else if (g_servo_us < 1500u) { if (g_screw_pos > 0u) g_screw_pos--; }
}

/* An edge lands at its OWN microsecond inside the step, so the ISR's gap reject measures
   the interval the fake scheduled and not the reading of it. */
static void emit_(uint32_t hz, uint32_t *next_us, uint32_t target_us, void (*isr)(void)) {
  if (hz == 0u) { *next_us = target_us; return; }
  uint32_t period = 1000000u / hz;
  if (period == 0u) period = 1u;
  /* Signed-cast differences, like every ms comparison in this file: g_us wraps on its own
     ~71.6-minute period, and a plain unsigned compare would leave a stalled next_us that
     never again satisfies `<= target_us` once target_us has wrapped past it, starving the
     emitters mid-test. Correct for any gap under 2^31 us. */
  if ((int32_t)(g_us - *next_us) > 0) *next_us = g_us;
  while ((int32_t)(target_us - *next_us) >= 0) {
    g_us = *next_us;
    *next_us += period;
    isr();
  }
}

/* one edge per 1 ms step, at the step's own microsecond, so the ISR's gap reject accepts
   every one of them. Only while the pump is on: a burst is what the meter saw, not what
   the fake felt like emitting. */
static void emit_burst_(uint32_t target_us) {
  if (!g_pump_on || g_burst_left == 0u) return;
  g_us = target_us;
  --g_burst_left;
  pulses_isr_flow();
}

/* One millisecond of rig time. The edge emitters run between tick_models_() and the final
   clock assignment, so an edge can land at its own microsecond inside the step. */
static void advance_1ms_(void) {
  const uint32_t target = g_us + 1000u;
  tick_models_(1000u);
  emit_(sim_flow_hz_(g_ms + 1u),       &g_next_flow_us,  target, pulses_isr_flow);
  emit_(sim_screw_hz_(),              &g_next_screw_us, target, screw_isr_);
  emit_burst_(target);
  g_us = target;
  g_ms = g_ms + 1u;   /* its own counter, see its declaration */
  /* the two scheduled injectors, applied at the step that reaches their deadline */
  if (g_float_due && g_us >= g_float_due_us) { g_float_due = false; g_float_ok = g_float_at_ok; }
  if (g_rx_due    && g_us >= g_rx_due_us)    { g_rx_due    = false; sim_serial_rx(g_rx_at_buf); }
}

void     sim_advance(uint32_t ms) { for (uint32_t i = 0; i < ms; ++i) advance_1ms_(); }
uint32_t hal_millis(void) { advance_1ms_(); return g_ms; }   /* PB_SIM_TICK_US == 1000 */
uint32_t hal_micros(void) { return g_us; }                   /* reads; never advances */
void     hal_delay_us(uint16_t us) {
  tick_models_(us);
  g_us += us;
  g_ms_frac_us += us;             /* folded into g_ms below, never derived from g_us itself */
  g_ms += g_ms_frac_us / 1000u;
  g_ms_frac_us %= 1000u;
}

/* Jumps the clock without walking every millisecond between, which is the only way a host
   test can start near a rollover in bounded time. emit_()'s "next_us is behind g_us" guard
   keeps the next step from catching up the edge schedule across the jump. The remainder
   is cleared because g_us cannot represent an arbitrary millisecond count without wrapping. */
void sim_set_clock_ms(uint32_t ms) {
  g_us = ms * 1000u;
  g_ms = ms;
  g_ms_frac_us = 0u;
  tick_models_(0u);
}

/* ---- D6. One event per write, carrying the same whole word the board writes. Polarity is
   the board's business; the fake models active-high. ---- */
void hal_boot_pump_off(void) {
  ev_(SIM_EV_PIN_CFG, SIM_PUMP_PIN, SIM_PFS_DIR_OUT);
  g_pump_on = false;
}
void hal_pump_write(bool on) {
  ev_(SIM_EV_PUMP_WRITE, SIM_PUMP_PIN, SIM_PFS_DIR_OUT | (on ? SIM_PFS_LEVEL_HI : 0u));
  if (on && !g_pump_on) {
    g_pump_on_at_ms = g_ms;                       /* the prime window runs from here */
    if (g_storm_on_armed) g_storm_hz = g_storm_on_hz;   /* the storm begins WITH the pump */
    if (g_float_at_armed) { g_float_due = true; g_float_due_us = g_us + g_float_at_off_ms * 1000u; }
    if (g_rx_at_armed)    { g_rx_due    = true; g_rx_due_us    = g_us + g_rx_at_off_ms * 1000u; }
  }
  if (!on && g_pump_on) {
    if (g_storm_on_armed) g_storm_hz = 0u;        /* and ends with it, as the 12 V leg does */
    g_burst_left = 0u;
  }
  g_pump_on = on;
  if (on && g_on_pump_on) { void (*cb)(void) = g_on_pump_on; g_on_pump_on = 0; cb(); }
}
void sim_on_pump_on(void (*cb)(void)) { g_on_pump_on = cb; }
bool     hal_pump_level_on(void) { return true; }
bool     sim_pump_is_on(void)    { return g_pump_on; }
uint32_t sim_pump_on_ms(void)    { return g_pump_on_us / 1000u; }

/* ---- ordinary pins ---- */
void hal_pin_write(uint8_t pin, uint8_t level) {
  ev_(SIM_EV_PIN_CFG, pin, SIM_PFS_DIR_OUT | (level ? SIM_PFS_LEVEL_HI : 0u));
}
int  hal_pin_read(uint8_t pin) {
  if (pin == PIN_HALL_FLOAT) {
    if (g_float_pat_len > 0u) {
      char c = g_float_pat[g_float_pat_idx];
      if (g_float_pat_idx + 1u < g_float_pat_len) g_float_pat_idx++;  /* sticks at the last */
      return (c == '1') ? PB_LOW : PB_HIGH;
    }
    return g_float_ok ? PB_LOW : PB_HIGH;   /* LOW == OK */
  }
  return PB_HIGH;
}
void sim_set_float(bool ok) { g_float_ok = ok; g_float_pat_len = 0u; g_float_pat_idx = 0u; }
void sim_set_float_pattern(const char *pattern) {
  size_t n = strlen(pattern);
  if (n >= sizeof g_float_pat) n = sizeof g_float_pat - 1u;
  memcpy(g_float_pat, pattern, n);
  g_float_pat[n] = '\0';
  g_float_pat_len = n;
  g_float_pat_idx = 0u;
}

/* ---- ADC and I2C ---- */
uint16_t hal_adc_read(void) {
  uint8_t ch = g_mux_stuck ? (uint8_t)PB_CANARY_CHANNEL : g_mux_sel;
  uint16_t settled = g_chan[ch & 0x0Fu];
  /* the first conversion after a select still carries the previous channel: a 10 k
     source into the ADC's sample cap does not settle inside one conversion */
  uint16_t v = g_adc_settled ? settled : g_adc_prev;
  g_adc_settled = true;
  g_adc_prev = settled;
  ev_(SIM_EV_ADC, ch, v);
  return v;
}
void sim_set_channel(uint8_t ch, uint16_t raw) { g_chan[ch & 0x0Fu] = raw; }
uint8_t hal_adc_bits(void)  { return (uint8_t)PB_ADC_BITS; }  /* the host's width IS the constant */
bool hal_adc_width_ok(void) { return true; }   /* no ADC to mis-configure on the host */

bool hal_i2c_write16(uint8_t addr, uint16_t bits) {
  ev_(SIM_EV_I2C_WRITE, addr, bits);
  if (g_i2c_fail) return false;
  g_exp_port = bits;
  g_mux_sel = (uint8_t)(bits & 0x0Fu);
  g_adc_settled = false;               /* a select un-settles the ADC */
  return true;
}
bool hal_i2c_read16(uint8_t addr, uint16_t *bits) {
  ev_(SIM_EV_I2C_READ, addr, 0);
  if (g_i2c_fail) return false;
  uint16_t v = g_exp_port;
  /* P4 is quasi-bidirectional: written HIGH, the external circuit drives what a read sees.
     Both the select and the home-hall read write the select bits then read straight back,
     so the fake overrides bit 4 on every expander read with the physical home state --
     otherwise "home" would be "P4 was written HIGH", true on every select, and no traverse
     would ever terminate. */
  if (addr == I2C_ADDR_EXPANDER) {
    if (screw_home_()) v &= (uint16_t)~SIM_EXP_HOME_BIT;
    else                v |= (uint16_t)SIM_EXP_HOME_BIT;
  }
  *bits = v;
  return true;
}
bool hal_i2c_probe(uint8_t addr) {
  if (g_i2c_fail) return false;
  return addr == I2C_ADDR_EXPANDER || addr == I2C_ADDR_LCD || addr == I2C_ADDR_OLED;
}
bool hal_i2c_recover(void) {
  /* Exactly PB_I2C_RECOVER_CLOCKS clocks, a fixed count, never "until SDA releases". The
     refusal-while-dosing guard is sensors.cpp's. */
  hal_pin_write(SIM_PIN_SDA, PB_HIGH);
  for (uint8_t i = 0; i < PB_I2C_RECOVER_CLOCKS; ++i) {
    hal_pin_write(SIM_PIN_SCL, PB_HIGH); hal_delay_us(5);
    hal_pin_write(SIM_PIN_SCL, PB_LOW);  hal_delay_us(5);
  }
  return true;
}
void sim_set_i2c_fail(bool fail)  { g_i2c_fail = fail; }
void sim_set_mux_stuck(bool stuck) { g_mux_stuck = stuck; }

void hal_servo_us(uint16_t us) {
  g_servo_us = us;
  if (us == 1500u) g_servo_stops++;
  ev_(SIM_EV_SERVO, PIN_SERVO, us);
}
void sim_set_stall(bool on) { g_stall = on; }
void sim_set_leak(bool on)  { g_leak = on; }
void sim_set_flow_ml_s(uint16_t ml_s) { g_flow_ml_s = ml_s; }
void sim_flow_storm(uint32_t hz)      { g_storm_hz = hz; }

void sim_flow_storm_at_pump_on(uint32_t hz) {
  g_storm_on_armed = (hz != 0u);
  g_storm_on_hz    = hz;
}
void sim_set_flow_burst_pulses(uint32_t n) { g_burst_left = n; }

void sim_set_float_at_ms(uint32_t ms, bool ok) {
  g_float_at_armed  = true;
  g_float_at_off_ms = ms;
  g_float_at_ok     = ok;
}
void sim_serial_rx_at_ms(uint32_t ms, const char *s) {
  size_t n = strlen(s);
  if (n >= sizeof g_rx_at_buf) n = sizeof g_rx_at_buf - 1u;
  memcpy(g_rx_at_buf, s, n);
  g_rx_at_buf[n]   = '\0';
  g_rx_at_armed    = true;
  g_rx_at_off_ms   = ms;
}

uint16_t sim_servo_us(void)   { return g_servo_us; }
uint32_t sim_servo_stops(void) { return g_servo_stops; }
void sim_set_screw_pulse_ms(uint32_t ms)          { g_screw_pulse_ms = ms; }
void sim_set_home_region(uint32_t lo, uint32_t hi) { g_home_lo = lo; g_home_hi = hi; }
void sim_set_cart_at(uint32_t pulses)              { g_screw_pos = pulses; }

/* ---- the watchdog ---- */
bool     hal_wdt_start(void) { g_wdt_running = true; g_wdt_counter = SIM_WDT_RELOAD; g_wdt_frac = 0; return true; }
uint32_t hal_wdt_granted(void) { return g_wdt_running ? (uint32_t)PB_WDT_GRANTED_MS : 0u; }
uint32_t hal_wdt_counter(void) { return g_wdt_counter; }
uint32_t hal_wdt_last_delta(void) { return g_wdt_delta; }
uint32_t sim_feeds(void) { return g_feeds; }
void     sim_wdt_stop(void) { g_wdt_rate_hz = 0; }
void     sim_wdt_rate_hz(uint32_t hz) { g_wdt_rate_hz = hz; }

void hal_wdt_feed(void) {
  g_wdt_counter = SIM_WDT_RELOAD;
  g_wdt_frac = 0;
  g_feeds++;
  ev_(SIM_EV_WDT_FEED, 0, g_wdt_counter);
}

/* The one place in the program that deliberately does not feed. Precondition: not dosing;
   hal_uno.cpp carries the same body. */
bool hal_wdt_alive(void) {
  hal_wdt_feed();                              /* start from a known reload */
  uint32_t a  = hal_wdt_counter();
  uint32_t t0 = hal_millis();
  while (hal_millis() - t0 < PB_WDT_PROBE_MS)  /* 40 ms, UNFED, pump already idle-OFF */
    hal_pump_write(false);                     /* the safety half of safety_tick(), without the feed */
  uint32_t b = hal_wdt_counter();
  hal_wdt_feed();                              /* and immediately back in the window */
  g_wdt_delta = (a > b) ? (a - b) : 0;         /* a DOWN-counter: b must be smaller */
  return g_wdt_delta >= PB_WDT_PROBE_MIN_COUNTS;
}

/* ---- interrupts: the fake models correctly armed, filtered pins ---- */
bool hal_irq_armed(uint8_t pin)    { return pin == PIN_FLOW || pin == PIN_HALL_SCREW; }
bool hal_irq_filtered(uint8_t pin) { return pin == PIN_FLOW || pin == PIN_HALL_SCREW; }

/* ---- the console, two arms. On the host, a fake UART; in the sim binary on a board,
   src/sim_console.cpp's real one. ---- */
#ifdef PB_NATIVE
size_t hal_serial_read(char *buf, size_t cap) {
  size_t n = 0;
  while (n < cap && g_rx_pos < g_rx_len) buf[n++] = g_rx[g_rx_pos++];
  return n;
}
void hal_serial_write(const char *s) {
  while (*s && g_tx_len < sizeof g_tx - 1) g_tx[g_tx_len++] = *s++;
  g_tx[g_tx_len] = '\0';
}
void hal_serial_drain(void) { g_rx_pos = g_rx_len; }
#else
size_t hal_serial_read(char *buf, size_t cap) { return sim_console_read(buf, cap); }
void   hal_serial_write(const char *s)        { sim_console_write(s); }
void   hal_serial_drain(void) {
  char b[32];
  for (uint8_t i = 0; i < 8u; ++i)            /* BOUNDED: 256 bytes, never "until empty" */
    if (sim_console_read(b, sizeof b) == 0) return;
}
#endif
void sim_serial_rx(const char *s) {
  size_t n = strlen(s);
  if (g_rx_len + n > sizeof g_rx) n = sizeof g_rx - g_rx_len;
  memcpy(g_rx + g_rx_len, s, n);
  g_rx_len += n;
}
size_t sim_serial_tx(char *buf, size_t cap) {
  size_t n = g_tx_len < cap - 1 ? g_tx_len : cap - 1;
  memcpy(buf, g_tx, n);
  buf[n] = '\0';
  g_tx_len = 0;
  return n;
}

/* ---- memory. The host has no __StackLimit; report a break inside the margin so the heap
   check runs without faking a failure. g_heap_break is a static so a test can move it;
   sim_reset() restores it. ---- */
static uint32_t g_heap_break = 0x20001800u;
uint32_t hal_heap_arena(void)   { return 2048u; }
uint32_t hal_heap_ordblks(void) { return 3u; }
uint32_t hal_heap_break(void)   { return g_heap_break; }
void     sim_set_heap_break(uint32_t addr) { g_heap_break = addr; }
uint32_t hal_stack_limit(void)  { return 0x20007b00u; }
uint32_t hal_stack_hwm(void)    { return 384u; }
uint32_t hal_boot_salt(void) { return g_nv.boots * PB_BOOT_SALT_STRIDE; }

void hal_begin(void) {
  g_mux_sel = 0;
  g_exp_port = 0xFFFFu;
  g_servo_us = 1500u;
#ifndef PB_NATIVE
  sim_console_begin();     /* the real UART, in the one binary that has one */
#endif
}

/* A partial clobber: the magic survives, the checksum does not -- the shape the
   bootloader's own .data/.bss leaves behind. */
void sim_noinit_clobber(void) { g_nv.pattern ^= 0xA5A5A5A5u; }

void sim_reset(bool warm) {
  g_us = g_ms = 0; g_ms_frac_us = 0;
  g_pump_on = false; g_pump_on_us = 0; g_pump_on_at_ms = 0;
  g_wdt_running = false; g_wdt_counter = SIM_WDT_RELOAD; g_wdt_rate_hz = 2929;
  g_wdt_frac = 0; g_wdt_delta = 0; g_feeds = 0;
  g_float_ok = true; g_float_pat_len = 0; g_float_pat_idx = 0; g_flow_ml_s = 0; g_storm_hz = 0;
  g_storm_on_armed = false; g_storm_on_hz = 0u; g_burst_left = 0u;
  g_float_at_armed = false; g_float_due = false;
  g_rx_at_armed    = false; g_rx_due    = false; g_rx_at_buf[0] = '\0';
  g_on_pump_on     = 0;
  g_i2c_fail = false; g_mux_stuck = false; g_stall = false; g_leak = false;
  g_adc_settled = false; g_adc_prev = 0;
  memset(g_chan, 0, sizeof g_chan);
  g_rx_len = g_rx_pos = 0; g_tx_len = 0;
  g_ev_n = 0;
  g_next_flow_us = g_next_screw_us = 0;
  g_servo_stops = 0;
  g_screw_pulse_ms = 0; g_home_lo = 0; g_home_hi = 40; g_screw_pos = 0;
  g_heap_break = 0x20001800u;
  if (!warm) memset(&g_nv, 0, sizeof g_nv);   /* a power cycle clears SRAM */
  noinit_begin();                             /* what setup() does, in the same order */
  hal_begin();
}

#endif /* PB_SIM */
