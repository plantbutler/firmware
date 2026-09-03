/* test/support/harness.h — the Unity fixture over hal_sim. A header, not a suite. */
#pragma once
#include <unity.h>
#include "hal.h"
#include "sim.h"
#include "safety.h"

static inline void pb_test_setup(void) {
  sim_reset(false);          /* a cold boot: clock at 0, .noinit cleared */
  hal_begin();
  hal_boot_pump_off();
  (void)hal_wdt_start();
  sim_events_clear();
}

/* safety_set_dosing(false) belongs HERE, not at the end of whichever test happens to set it
   true: Unity aborts a failing TEST_ASSERT_* with a longjmp straight into tearDown(), skipping
   every line after it in the case body. g_dosing is process-lifetime state in safety.cpp, and
   sim_reset() (called from pb_test_setup(), not here) only resets hal_sim.cpp's own statics —
   a different translation unit. tearDown() is the one place Unity guarantees runs regardless
   of how the case ended, so it is the only place that can actually promise every case starts
   with g_dosing == false.

   safety_float_refusal_count(false) belongs here for the identical reason (task 15 fix round
   1): g_float_refusals is the same shape of process-lifetime static in safety.cpp, and a test
   that only clears it as its own last line -- as test_the_flap_counter_trips_after_three_
   consecutive_float_refusals did before this fix -- leaves it dirty for every following case
   in the binary the moment an assertion earlier in that body fails and longjmps past the
   clear. `false` is not a magic reset value here: it is the SAME call dose_end_ml_() (task 17)
   will make on every granted dose, so this line asks the real accessor for "cleared", not a
   parallel reset path. test_dose.cpp carries a g_float_refusals proof pair, the same shape as
   its existing g_dosing pair, to guard this teardown line the same way. */
static inline void pb_test_teardown(void) {
  sim_events_clear();
  safety_set_dosing(false);
  safety_float_refusal_count(false);
}

static inline void pb_advance(uint32_t ms) { sim_advance(ms); }

static inline uint32_t pb_count(sim_ev_kind_t kind) {
  const sim_ev_t *ev; size_t n = sim_events(&ev); uint32_t hits = 0;
  for (size_t i = 0; i < n; ++i) if (ev[i].kind == kind) hits++;
  return hits;
}

/* Strictly inside: the two feeds that BRACKET hal_wdt_alive()'s probe are legal, and
   anything between them is the bug this exists to catch (§2.5). */
static inline void pb_expect_no_feed_between(uint32_t from_ms, uint32_t to_ms) {
  const sim_ev_t *ev; size_t n = sim_events(&ev); uint32_t hits = 0;
  for (size_t i = 0; i < n; ++i)
    if (ev[i].kind == SIM_EV_WDT_FEED && ev[i].at_ms > from_ms && ev[i].at_ms < to_ms) hits++;
  TEST_ASSERT_EQUAL_UINT32(0u, hits);
}
