/* src/hal_sim.cpp — one fake, two jobs: the on-device sim and the host test double.
   Filtered out of [env:uno_r4_wifi] by build_src_filter. */
#include "hal.h"
#include "sim.h"
#include "config.h"
#include "pins.h"
#include "noinit.h"
#include <string.h>

#if PB_SIM

/* PB_PUMP_OWNER belongs to hal_uno.cpp alone (§2.2), so pins.h does not define
   PIN_PUMP_EN here and this file must name D6 itself. Same for the I2C pair:
   A4 == 18 (variants/UNOWIFIR4/pins_arduino.h:18), A5 == 19 (:19). */
#define SIM_PUMP_PIN 6
#define SIM_PIN_SDA  18
#define SIM_PIN_SCL  19
#define SIM_WDT_RELOAD 16384u

/* ---- clock ---- */
static uint32_t g_us, g_ms;

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
static uint16_t g_flow_ml_s;
static uint32_t g_storm_hz;
static bool     g_i2c_fail;
static bool     g_mux_stuck;
static bool     g_stall;
static bool     g_leak;
static uint16_t g_chan[16];
static uint16_t g_exp_port = 0xFFFFu;      /* the PCF8575's latch; task 7 gives it meaning */
static uint8_t  g_mux_sel;
static uint16_t g_servo_us = 1500u;        /* 1500 == stopped; task 6 drives the screw off it */

/* ---- serial ---- */
static char   g_rx[256]; static size_t g_rx_len, g_rx_pos;
static char   g_tx[4096]; static size_t g_tx_len;

/* ---- call trace ---- */
static sim_ev_t g_ev[1024];
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

/* One millisecond of rig time. Task 6 hangs the flow and screw edge emitters here,
   between tick_models_() and the final clock assignment, so an edge can land at its own
   microsecond inside the step. */
static void advance_1ms_(void) {
  const uint32_t target = g_us + 1000u;
  tick_models_(1000u);
  g_us = target;
  g_ms = g_us / 1000u;
}

void     sim_advance(uint32_t ms) { for (uint32_t i = 0; i < ms; ++i) advance_1ms_(); }
uint32_t hal_millis(void) { advance_1ms_(); return g_ms; }   /* PB_SIM_TICK_US == 1000 */
uint32_t hal_micros(void) { return g_us; }                   /* reads; never advances */
void     hal_delay_us(uint16_t us) { tick_models_(us); g_us += us; g_ms = g_us / 1000u; }

/* ---- D6. ONE event per write, carrying the same whole word the board writes (§2.1).
   Polarity is the board's business and lives only in hal_uno.cpp; the fake models
   PB_RELAY_ACTIVE_HIGH. ---- */
void hal_boot_pump_off(void) {
  ev_(SIM_EV_PIN_CFG, SIM_PUMP_PIN, SIM_PFS_DIR_OUT);
  g_pump_on = false;
}
void hal_pump_write(bool on) {
  ev_(SIM_EV_PUMP_WRITE, SIM_PUMP_PIN, SIM_PFS_DIR_OUT | (on ? SIM_PFS_LEVEL_HI : 0u));
  if (on && !g_pump_on) g_pump_on_at_ms = g_ms;    /* task 6's prime delay runs from here */
  g_pump_on = on;
}
bool     hal_pump_level_on(void) { return true; }
bool     sim_pump_is_on(void)    { return g_pump_on; }
uint32_t sim_pump_on_ms(void)    { return g_pump_on_us / 1000u; }

/* ---- ordinary pins ---- */
void hal_pin_mode(uint8_t pin, uint8_t mode) { ev_(SIM_EV_PIN_MODE, pin, mode); }
void hal_pin_write(uint8_t pin, uint8_t level) {
  ev_(SIM_EV_PIN_CFG, pin, SIM_PFS_DIR_OUT | (level ? SIM_PFS_LEVEL_HI : 0u));
}
int  hal_pin_read(uint8_t pin) {
  if (pin == PIN_HALL_FLOAT) return g_float_ok ? PB_LOW : PB_HIGH;   /* §2.10: LOW == OK */
  return PB_HIGH;
}
void sim_set_float(bool ok) { g_float_ok = ok; }

/* ---- ADC and I2C. Task 7 gives the expander its mux/home-hall meaning. ---- */
uint16_t hal_adc_read(void) {
  uint8_t ch = g_mux_stuck ? PB_CANARY_CHANNEL : g_mux_sel;
  uint16_t v = g_chan[ch & 0x0Fu];
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
  return true;
}
bool hal_i2c_read16(uint8_t addr, uint16_t *bits) {
  ev_(SIM_EV_I2C_READ, addr, 0);
  if (g_i2c_fail) return false;
  *bits = g_exp_port;
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

void hal_servo_us(uint16_t us) { g_servo_us = us; ev_(SIM_EV_SERVO, PIN_SERVO, us); }
void sim_set_stall(bool on) { g_stall = on; }
void sim_set_leak(bool on)  { g_leak = on; }
void sim_set_flow_ml_s(uint16_t ml_s) { g_flow_ml_s = ml_s; }
void sim_flow_storm(uint32_t hz)      { g_storm_hz = hz; }

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

/* ---- serial ---- */
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
   margin so §12's check is exercised without faking a failure nobody asked for. ---- */
uint32_t hal_heap_arena(void)   { return 2048u; }
uint32_t hal_heap_ordblks(void) { return 3u; }
uint32_t hal_heap_break(void)   { return 0x20001800u; }
uint32_t hal_stack_limit(void)  { return 0x20007b00u; }
uint32_t hal_stack_hwm(void)    { return 384u; }
uint32_t hal_boot_salt(void) { return g_nv.boots * PB_BOOT_SALT_STRIDE; }

void hal_begin(void) {
  g_mux_sel = 0;
  g_exp_port = 0xFFFFu;
  g_servo_us = 1500u;
}

/* A partial clobber: the magic survives, the checksum does not. That is the shape
   the bootloader's own .data/.bss would leave behind (§2.3). */
void sim_noinit_clobber(void) { g_nv.pattern ^= 0xA5A5A5A5u; }

void sim_reset(bool warm) {
  g_us = g_ms = 0;
  g_pump_on = false; g_pump_on_us = 0; g_pump_on_at_ms = 0;
  g_wdt_running = false; g_wdt_counter = SIM_WDT_RELOAD; g_wdt_rate_hz = 2929;
  g_wdt_frac = 0; g_wdt_delta = 0; g_feeds = 0;
  g_float_ok = true; g_flow_ml_s = 0; g_storm_hz = 0;
  g_i2c_fail = false; g_mux_stuck = false; g_stall = false; g_leak = false;
  memset(g_chan, 0, sizeof g_chan);
  g_rx_len = g_rx_pos = 0; g_tx_len = 0;
  g_ev_n = 0;
  if (!warm) memset(&g_nv, 0, sizeof g_nv);   /* a power cycle clears SRAM (§2.3) */
  noinit_begin();                             /* what setup() does, in the same order */
  hal_begin();
}

#endif /* PB_SIM */
