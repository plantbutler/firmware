/* src/hal_sim.cpp — one fake, two jobs: the on-device sim and the host test double.
   Filtered out of [env:uno_r4_wifi] by build_src_filter. */
#include "hal.h"
#include "sim.h"
#include "config.h"
#include "pins.h"
#include "noinit.h"
#include "pulses.h"
#include <string.h>
#ifndef PB_NATIVE
#include "sim_console.h"      /* the device-only shim (task 29); native never links it */
#endif

#if PB_SIM

/* PB_PUMP_OWNER belongs to hal_uno.cpp alone (§2.2), so pins.h does not define
   PIN_PUMP_EN here and this file must name D6 itself. Same for the I2C pair:
   A4 == 18 (variants/UNOWIFIR4/pins_arduino.h:18), A5 == 19 (:19). */
#define SIM_PUMP_PIN 6
#define SIM_PIN_SDA  18
#define SIM_PIN_SCL  19
#define SIM_WDT_RELOAD 16384u
/* The PCF8575's P4 bit, mirrored from sensors.cpp's own EXP_HOME_BIT (sensors.cpp is the
   only file allowed to give the wiring meaning; this is the fake modelling what the pin
   physically does, not a second source of truth for what it means). */
#define SIM_EXP_HOME_BIT (1u << 4)

/* ---- clock ---- */
static uint32_t g_us, g_ms;
/* g_ms is tracked as its OWN counter, never derived from g_us by division (task 14 found
   why: g_us is a real microsecond register and wraps on its own ~71.6 minute period --
   exactly like a real chip's micros(), which is independent of millis()'s ~49.7 day one --
   so g_us / 1000u can never exceed roughly 4.295 million regardless of how far g_ms has
   actually counted. sim_set_clock_ms(0xFFFFF000) landing there and the very next
   hal_millis() call silently resyncing g_ms down to ~4.29 MILLION is exactly that bug,
   caught by test before it could poison task 22's identical rollover fixture: see the
   task 14 report for the reproduction. g_ms_frac_us is the sub-millisecond remainder
   hal_delay_us() carries between calls so a run of sub-ms waits still folds into g_ms
   correctly however large g_ms already is. */
static uint32_t g_ms_frac_us;

/* ---- D6 ---- */
static bool     g_pump_on;
static uint32_t g_pump_on_us;              /* cumulative time asserted */
static uint32_t g_pump_on_at_ms;           /* when the current assertion started */

/* ---- watchdog ---- */
static bool     g_wdt_running;
static uint32_t g_wdt_counter = SIM_WDT_RELOAD;
static uint32_t g_wdt_rate_hz = 2929;      /* PCLKB/8192 = 2929.7 Hz (§7) */
static uint32_t g_wdt_frac;                /* carried remainder, in counts * 1e6 */
static uint32_t g_wdt_delta;
static uint32_t g_feeds;

/* ---- injector state; the task that consumes each one is named in sim.h ---- */
static bool     g_float_ok = true;
/* task 15's per-sample float. Empty (len 0) means "no pattern, use g_float_ok" -- the
   pre-task-15 sim_set_float() behaviour, kept so every earlier caller of it still works
   unchanged. A non-empty pattern advances one character per hal_pin_read(PIN_HALL_FLOAT)
   and STICKS at the last character rather than wrapping, matching sim.h's contract. */
static char     g_float_pat[16];
static size_t   g_float_pat_len;
static size_t   g_float_pat_idx;
static uint16_t g_flow_ml_s;
static uint32_t g_storm_hz;
/* The four pump-relative injectors (task 18 step 1). Each scheduled one is an (armed,
   offset, payload) triple PLUS a separate live deadline, so that a second dose in the
   same test re-schedules from ITS pump-on rather than from the first one's. */
static bool     g_storm_on_armed; static uint32_t g_storm_on_hz;
static uint32_t g_burst_left;                     /* flow pulses still owed */
static bool     g_float_at_armed; static uint32_t g_float_at_off_ms; static bool g_float_at_ok;
static bool     g_float_due;      static uint32_t g_float_due_us;
static bool     g_rx_at_armed;    static uint32_t g_rx_at_off_ms;
static char     g_rx_at_buf[64];
static bool     g_rx_due;         static uint32_t g_rx_due_us;
static void   (*g_on_pump_on)(void);   /* sim_on_pump_on(): fires ONCE, from inside the next
                                          hal_pump_write(true), then clears itself (task 24) */
static bool     g_i2c_fail;
static bool     g_mux_stuck;
static bool     g_stall;
static bool     g_leak;
static uint16_t g_chan[16];
static uint16_t g_exp_port = 0xFFFFu;      /* the PCF8575's latch; task 7 gives it meaning */
static uint8_t  g_mux_sel;
static bool     g_adc_settled;
static uint16_t g_adc_prev;
static uint16_t g_servo_us = 1500u;        /* 1500 == stopped; task 6 drives the screw off it */
static uint32_t g_servo_stops;             /* count of writes of 1500 (task 14) */

/* ---- the screw + home region model (task 14). g_screw_pos is the fake's own notion of
   the cart's PHYSICAL position, in pulses -- separate from pulses.cpp's g_screw, which is
   a directionless total of accepted edges since boot, exactly like the real hall sensor.
   Saturates at 0 rather than going negative: the threadless start of the screw is a hard
   stop the fake gives itself, for the same reason a physical screw has one. ---- */
static uint32_t g_screw_pulse_ms;          /* 0 == the screw does not turn at all */
static uint32_t g_home_lo, g_home_hi;      /* sim_reset() sets [0, 40] */
static uint32_t g_screw_pos;

/* ---- capacities for the three buffers below: a HOST default and a DEVICE default, not
   one shrunk constant. The host suites capture whole console lines and dose-length event
   traces for assertions, on a machine where 12 KB is noise; [env:uno_r4_wifi_sim] has to
   find these same three inside a 32 KB part that also carries the framework, WiFiS3's
   headers (unused code, still linked) and every other static in the image. Nothing on the
   device ever reads g_rx/g_tx back — hal_serial_read()/hal_serial_write()'s device arm
   (below) goes through sim_console.cpp's real UART instead, and sim_serial_rx()/
   sim_serial_tx() (task 28: the injectors are never gated) have no caller in a device
   build; they are still compiled, so the arrays cannot be dropped to zero, but the choice
   of "how much" no longer answers to anything but "not zero". g_ev is the one of the three
   with an on-device reader in prospect (a future `sim events` dump), so it gets the larger
   of the three device numbers: enough for one bring-up step's worth of pin/I2C/servo/WDT
   admin (a single sensors_sweep() or safety_tick() pass is a few dozen events; 64 is
   several such passes, not one). See this task's commit message for the host figure that
   justifies leaving 1024 alone rather than trimming it too. */
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

/* now_ms is the millisecond THIS step reaches by its end (g_ms + 1), not g_ms, which
   at the call site below is still last step's value: emit_() runs between tick_models_()
   and the g_us/g_ms assignment, so a read of the bare global here is one step STALE
   against what hal_millis() is about to return to the caller. (Task 14's fix: gating on the
   stale value pushed an edge one whole millisecond late against any deadline dose_run()
   checks with `>=` -- see the note by g_ms's declaration for why `now_ms` is passed rather
   than read here.)

   Task 18: task 14's fix above made the fake's onset LAND ON TIME; it did not make the
   onset REALISTIC. Until this task the model was a pure step function -- silent, then at
   full rate the instant `now_ms - g_pump_on_at_ms` reached PB_PRIME_MS_DEFAULT (3000) --
   and PB_PRIME_MS_DEFAULT is ALSO the exact instant dose_run()'s prime rule samples `got`
   (`el >= prime_ms && got < PB_PRIME_MIN_PULSES`). Those two facts collided: the step fired
   in the SAME millisecond the rule read it, so at most one pulse could ever be on the
   counter when the rule checked, never PB_PRIME_MIN_PULSES (5) -- a healthy dose on the
   untouched default window could not pass a correct rule, because the rig, not the rule,
   was unrealistic. A fake "dead until the deadline and instantly at full rate after" was
   never an honest stand-in for a YF-S401: the sensor's own paddle wheel responds within a
   couple of pulses of water reaching it, but the WATER reaching it ramps -- the small pump
   motor takes real, if short, time to spin up, and the water already standing in a WET
   line (the ordinary case; a genuinely DRY line is what PB_PRIME_MS_DEFAULT budgets for
   and is modelled here as g_flow_ml_s staying 0, not as a delayed full rate) accelerates
   over that same short span. SIM_FLOW_ONSET_MS below models that spin-up as a linear ramp
   from 0 to the steady rate, starting AT pump-on rather than waiting out the deadline: a
   healthy wet-line dose now has pulses on the counter within tens of milliseconds, with
   the whole PB_PRIME_MS_DEFAULT budget still free as margin, exactly as the real hardware
   the prime rule is written against would leave it. A storm (g_storm_hz, including
   sim_flow_storm_at_pump_on()) is deliberately NOT ramped: it models electrical noise on a
   floating D2, which has no motor to spin up and starts at full rate the instant the pump
   leg beside it is energised. */
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

/* Task 6 step 6's placeholder returned a flat 20 Hz whenever the servo was off its stop
   point. This is task 14's real model: a period of sim_set_screw_pulse_ms(), converted to
   Hz for emit_(). 0 (the default) means the screw does not turn -- there is no calibrated
   rate to fall back on, and a silent flat rate would give the screw two disagreeing
   speeds if a case forgot to set one. */
static uint32_t sim_screw_hz_(void) {
  if (g_stall || g_servo_us == 1500u || g_servo_us == 0u || g_screw_pulse_ms == 0u) return 0u;
  return 1000u / g_screw_pulse_ms;
}

/* true only while the physical position sits inside the home REGION -- a point would
   never be found by a bounded traverse landing one pulse either side of it. */
static bool screw_home_(void) { return g_screw_pos >= g_home_lo && g_screw_pos <= g_home_hi; }

/* The ISR wrapper emit_() drives for the screw: counts the accepted edge exactly like the
   real hall (pulses_isr_screw(), directionless), raises SIM_EV_SCREW, and ALSO walks the
   fake's own physical position in the direction the servo was last commanded -- which is
   what makes sim_set_home_region() and sensors_home_hall() agree with each other. */
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
  /* Both comparisons were plain unsigned `<`/`<=` until task 14: harmless for every test
     before this one, since next_us/g_us/target_us always sat close together near 0 -- but
     g_us itself is a real 32-bit microsecond register with its OWN ~71.6-minute wrap
     (see the note by g_ms's declaration), and sim_set_clock_ms() is the first thing that
     ever runs it that close to that wrap inside one test. A stalled next_us left behind
     by the wrap would then never again satisfy a plain `<= target_us` once target_us
     itself had wrapped past it, silently starving the screw/flow emitters mid-test --
     caught by test_move_deadline_holds_across_a_millis_rollover reporting "stall" instead
     of "timeout" once the wrap arrived a few thousand ms into a 45 s move. Signed-cast
     unsigned differences, exactly like every ms comparison elsewhere in this file, fix it
     for a true gap under 2^31 us -- true of every gap this program ever schedules. */
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

/* One millisecond of rig time. Task 6 hangs the flow and screw edge emitters here,
   between tick_models_() and the final clock assignment, so an edge can land at its own
   microsecond inside the step. */
static void advance_1ms_(void) {
  const uint32_t target = g_us + 1000u;
  tick_models_(1000u);
  emit_(sim_flow_hz_(g_ms + 1u),       &g_next_flow_us,  target, pulses_isr_flow);
  emit_(sim_screw_hz_(),              &g_next_screw_us, target, screw_isr_);
  emit_burst_(target);
  g_us = target;
  g_ms = g_ms + 1u;   /* its OWN counter -- see the note by its declaration */
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

/* task 14 step 1's injector: not sim_advance()/hal_millis() by another name -- it jumps
   the clock to an arbitrary value without walking through every millisecond between here
   and there, which is the only way a host test can start near a rollover in bounded time.
   emit_()'s own wraparound-safe "next_us is behind g_us" guard is what keeps the next
   advance_1ms_() from trying to "catch up" the edge schedule across the jump.

   The brief's own draft body was `g_us = ms * 1000u; g_ms = ms;` and nothing else. That
   is unsafe with g_ms independently tracked (see the note by its declaration): g_us's own
   32-bit register cannot represent an arbitrary millisecond count as microseconds without
   wrapping, so g_ms_frac_us's stale remainder from before the jump would otherwise carry
   forward into the value returned after it. Clearing it here is the one addition beyond
   the draft; it is still "one bounded tick_models_() call; nothing else" as far as the
   MODELS go. */
void sim_set_clock_ms(uint32_t ms) {
  g_us = ms * 1000u;
  g_ms = ms;
  g_ms_frac_us = 0u;
  tick_models_(0u);
}

/* ---- D6. ONE event per write, carrying the same whole word the board writes (§2.1).
   Polarity is the board's business and lives only in hal_uno.cpp; the fake models
   PB_RELAY_ACTIVE_HIGH. ---- */
void hal_boot_pump_off(void) {
  ev_(SIM_EV_PIN_CFG, SIM_PUMP_PIN, SIM_PFS_DIR_OUT);
  g_pump_on = false;
}
void hal_pump_write(bool on) {
  ev_(SIM_EV_PUMP_WRITE, SIM_PUMP_PIN, SIM_PFS_DIR_OUT | (on ? SIM_PFS_LEVEL_HI : 0u));
  if (on && !g_pump_on) {
    g_pump_on_at_ms = g_ms;                       /* task 6's prime delay runs from here */
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
void hal_pin_mode(uint8_t pin, uint8_t mode) { ev_(SIM_EV_PIN_MODE, pin, mode); }
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
    return g_float_ok ? PB_LOW : PB_HIGH;   /* §2.10: LOW == OK */
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

/* ---- ADC and I2C. Task 7 gives the expander its mux/home-hall meaning. ---- */
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
  /* P4 is quasi-bidirectional (§2.10): writing it HIGH (EXP_INPUTS_HI, always) lets the
     external circuit drive what a READ sees. sensors_select()/sensors_home_hall() both
     write the select bits and then read them straight back, so the fake has to override
     bit 4 on every expander read with the PHYSICAL home state rather than echo back
     whatever a select last wrote there -- otherwise "home" would just be "P4 was written
     HIGH", which is true on every select and would never terminate a traverse. */
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
  /* EXACTLY PB_I2C_RECOVER_CLOCKS clocks, a fixed loop count, never "until SDA releases"
     (§2.13). The refusal-while-dosing guard is sensors.cpp's — see task 7. */
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

/* The ONE place in the program that deliberately does not feed. Precondition: not dosing —
   it is called from setup() and as dose_run()'s first guard, after the g_dosing check.
   Spec §2.5, verbatim; hal_uno.cpp carries the same body. */
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

/* ---- the console. ONE file, two arms. On the host this is the fake UART, byte for byte
   as task 3 wrote it. In the sim binary on a real board it is src/sim_console.cpp, the
   device-only shim, and hal_sim.cpp itself is unchanged between the two. ---- */
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

/* ---- memory. The host has no __StackLimit; report a break comfortably inside the
   margin so §12's check is exercised without faking a failure nobody asked for.
   g_heap_break is a file static so a test (task 22's report_heap_ok() cases) can move it
   without pretending a real allocator ran; sim_reset() restores it to this same default. ---- */
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

/* A partial clobber: the magic survives, the checksum does not. That is the shape
   the bootloader's own .data/.bss would leave behind (§2.3). */
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
  if (!warm) memset(&g_nv, 0, sizeof g_nv);   /* a power cycle clears SRAM (§2.3) */
  noinit_begin();                             /* what setup() does, in the same order */
  hal_begin();
}

#endif /* PB_SIM */
