/* src/safety.cpp — safety_tick, safety_wait_ms, and (from drop 2) the float debounce,
   the dry latch, the contradiction latch and dose_run(). Spec §2. */
#include "safety.h"
#include "config.h"
#include "hal.h"
#include "noinit.h"
#include "pins.h"

static bool g_dosing;                    /* true only between the ON and OFF writes */

bool safety_dosing(void) { return g_dosing; }
void safety_set_dosing(bool on) { g_dosing = on; }

void safety_tick(void) {
  if (!g_dosing) hal_pump_write(false);  /* idle ACTIVELY re-asserts OFF, every pass */
  hal_wdt_feed();                        /* the ONE feeder in the program */
}

void safety_wait_ms(uint32_t ms) {
  uint32_t t0 = hal_millis();
  while (hal_millis() - t0 < ms) safety_tick();
}

/* §2.10. N consecutive OK to GRANT; ONE bad sample refuses immediately. The wait between
   samples is safety_wait_ms(), which calls safety_tick() on every iteration -- so the pump
   is idle-re-asserted and the dog fed throughout. */
bool safety_float_ok_debounced(void) {
  for (uint8_t i = 0; i < PB_FLOAT_OK_SAMPLES; ++i) {
    if (hal_pin_read(PIN_HALL_FLOAT) != PB_LOW) return false;
    if (i + 1u < PB_FLOAT_OK_SAMPLES) safety_wait_ms(PB_FLOAT_SAMPLE_MS);
  }
  return true;
}

/* §2.11. The checksum is recomputed on EVERY write (§2.3), so a partial clobber cannot
   read back as a valid latch. */
void safety_dry_set(bool on) {
  g_nv.dry_latched = on;
  noinit_commit();
}
bool safety_dry(void) { return g_nv.dry_latched; }

/* §2.10's second consequence: the flap counter. Not persisted -- process-lifetime state,
   exactly like g_dosing above. TWO call sites, both in task 17's dose_run() exit helpers;
   see the header for the exact contract. */
static uint8_t g_float_refusals;

void safety_float_refusal_count(bool refused_for_float) {
  if (refused_for_float) { if (g_float_refusals < 255u) g_float_refusals++; }
  else                     g_float_refusals = 0u;
}
bool safety_float_flap(void) { return g_float_refusals >= PB_FLOAT_FLAP_LIMIT; }
