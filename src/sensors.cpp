/* src/sensors.cpp — the expander, the mux discipline, the canary, the home hall,
   I2C health with back-off and the bounded nine-clock recovery. */
#include "sensors.h"
#include "hal.h"
#include "safety.h"
#include "config.h"
#include "pins.h"
#include <stdio.h>
#include <string.h>

/* P4..P15 written HIGH on EVERY select: the PCF8575 is quasi-bidirectional and P4 is the
   home hall, so a select that dropped it would make the input that gates the pump
   unreadable (cad/wiring/nets.py, P4: "write P4 HIGH before reading"). */
#define EXP_INPUTS_HI 0xFFF0u
#define EXP_HOME_BIT  (1u << 4)

static uint16_t g_val[PB_CHANNELS];
static bool     g_valid[PB_CHANNELS];
static bool     g_stuck;
static uint8_t  g_cur_ch;

static uint32_t g_errors, g_fails, g_backoff_until;
static bool     g_healthy = true;
static uint32_t g_txn, g_txn_t0, g_txn_prev;

static bool     g_float_seen;
static int      g_float_last;
static uint32_t g_float_change_ms;

static void note_(bool ok) {
  g_txn++;
  if (ok) { g_fails = 0; return; }
  g_errors++;
  if (++g_fails >= (uint32_t)PB_I2C_FAIL_LIMIT) {
    g_healthy = false;
    g_backoff_until = hal_millis() + (uint32_t)PB_I2C_BACKOFF_MS;
  }
}

/* false == do not touch the bus this pass. */
static bool gate_(void) {
  if (g_healthy) return true;
  if ((int32_t)(hal_millis() - g_backoff_until) < 0) return false;   /* still backing off */
  if (safety_dosing()) return false;      /* §2.13: an expired back-off STAYS expired */
  safety_tick();
  (void)hal_i2c_recover();                /* exactly PB_I2C_RECOVER_CLOCKS clocks */
  safety_tick();
  g_healthy = true;
  g_fails = 0;
  return true;
}

static void float_track_(void) {
  int now = hal_pin_read(PIN_HALL_FLOAT);
  if (now == g_float_last) return;
  g_float_last = now;
  g_float_change_ms = hal_millis();
  g_float_seen = true;
}

bool sensors_begin(void) {
  memset(g_val, 0, sizeof g_val);
  memset(g_valid, 0, sizeof g_valid);
  g_stuck = false; g_cur_ch = 0;
  g_errors = 0; g_fails = 0; g_backoff_until = 0; g_healthy = true;
  g_txn = 0; g_txn_prev = 0; g_txn_t0 = hal_millis();
  g_float_seen = false;
  g_float_last = hal_pin_read(PIN_HALL_FLOAT);
  g_float_change_ms = g_txn_t0;
  if (!safety_dosing()) (void)hal_i2c_recover();   /* at boot, outside a dose (§2.13) */
  return hal_i2c_probe(I2C_ADDR_EXPANDER);
}

bool sensors_select(uint8_t ch) {
  if (!gate_()) return false;
  uint16_t bits = (uint16_t)((ch & 0x0Fu) | EXP_INPUTS_HI);
  bool ok = hal_i2c_write16(I2C_ADDR_EXPANDER, bits);
  note_(ok);
  if (ok) g_cur_ch = (uint8_t)(ch & 0x0Fu);
  return ok;
}

bool sensors_read_raw(uint8_t ch, uint16_t *raw) {
  if (!sensors_select(ch)) return false;
  safety_wait_ms(1);              /* >= 1 ms to settle, fed */
  (void)hal_adc_read();           /* discard the first conversion */
  *raw = hal_adc_read();          /* keep the second */
  return true;
}

bool sensors_sweep(void) {
  safety_tick();
  uint16_t canary = 0;
  if (!sensors_read_raw(PB_CANARY_CHANNEL, &canary)) {
    for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) g_valid[ch] = false;
    g_stuck = false;
    float_track_();
    return false;
  }
  bool ok = true, all_equal = true;
  for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) {
    safety_tick();
    uint16_t v = 0;
    if (sensors_read_raw(ch, &v)) {
      g_val[ch] = v; g_valid[ch] = true;
      if (v != canary) all_equal = false;
    } else {
      g_valid[ch] = false; all_equal = false; ok = false;
    }
  }
  g_stuck = all_equal;
  if (g_stuck) {                 /* omit the wired channels; the diagnostics keep the
                                    report legal and err=stuck is what the phone sees */
    for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) g_valid[ch] = false;
    ok = false;
  }
  float_track_();
  return ok;
}

uint16_t sensors_value(uint8_t ch) { return ch < PB_CHANNELS ? g_val[ch] : 0u; }
bool     sensors_valid(uint8_t ch) { return ch < PB_CHANNELS ? g_valid[ch] : false; }
bool     sensors_stuck(void)       { return g_stuck; }

bool sensors_home_hall(bool *home) {
  if (!gate_()) return false;
  uint16_t bits = (uint16_t)(g_cur_ch | EXP_INPUTS_HI);
  if (!hal_i2c_write16(I2C_ADDR_EXPANDER, bits)) { note_(false); return false; }
  note_(true);
  uint16_t got = 0;
  if (!hal_i2c_read16(I2C_ADDR_EXPANDER, &got)) { note_(false); return false; }
  note_(true);
  /* WPSE313 open collector with R3 pulling P4 up: LOW == the magnet is present. */
  *home = ((got & EXP_HOME_BIT) == 0u);
  return true;
}

bool     sensors_i2c_healthy(void) { return g_healthy; }
uint32_t sensors_i2c_errors(void)  { return g_errors; }

uint32_t sensors_i2c_txn_per_min(void) {
  uint32_t now = hal_millis();
  uint32_t elapsed = now - g_txn_t0;
  if (elapsed >= 60000u) {
    g_txn_prev = (g_txn * 60000u) / elapsed;
    g_txn = 0;
    g_txn_t0 = now;
  }
  return g_txn_prev;
}

uint32_t sensors_float_change_age_s(void) {
  if (!g_float_seen) return 0u;
  return (hal_millis() - g_float_change_ms) / 1000u;
}

void sensors_scan(char *out, size_t cap) {
  if (cap == 0u) return;
  out[0] = '\0';
  size_t at = 0;
  for (uint8_t a = 0x08u; a <= 0x77u; ++a) {
    safety_tick();
    if (!hal_i2c_probe(a)) continue;
    if (at + 6u >= cap) break;
    at += (size_t)snprintf(out + at, cap - at, "0x%02X ", (unsigned)a);
  }
}
