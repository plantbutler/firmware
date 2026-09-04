/* test/support/harness.h — the Unity fixture over hal_sim. A header, not a suite. */
#pragma once
#include <unity.h>
#include "cli.h"
#include "config.h"
#include "hal.h"
#include "sensors.h"
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
   its existing g_dosing pair, to guard this teardown line the same way.

   cli_stop_clear() belongs here for the same reason again (task 16): g_stop_req, the
   partial-match prefix and the pushback buffer are process-lifetime statics in cli.cpp,
   and every test that exercises cli_stop_requested() only clears them as ITS OWN first
   line, which is no protection at all against a PRIOR case that longjmped out mid-body
   with bytes still parked in the pushback. Without this line, a case whose earlier
   assertion fails leaves the next case in the binary reading a stale prefix or a stale
   stop request instead of the console traffic it just sent.

   safety_reset_dose_cooldown_() belongs here for the identical reason a third time (task 17
   fix round 2): g_last_end_ms is process-lifetime state in safety.cpp exactly like the two
   above, and it has no ordinary reset path -- only a granted-reaching dose_run() call ever
   writes it, always to a fresh non-zero value, never to zero. Several cases in test_dose.cpp
   are safe today only because a stale leftover g_last_end_ms happens to sit ABOVE their own
   fresh clock (an unsigned "current minus stale" wraps to a huge number rather than a small
   one), which is a real argument but a fragile one: it depends on PB_FLOAT_SAMPLE_MS and
   PB_FLOAT_OK_SAMPLES keeping their current values, and the margin is about 38 ms. Resetting
   it here removes the dependency on that margin entirely -- with g_last_end_ms at 0, the
   cooldown rung's own `g_last_end_ms != 0u` guard skips the check outright, exactly as a
   real "no dose has ended yet" boot would read it, rather than every case having to land its
   own clock in the one safe zone above or below whatever the previous case left behind.
   test_dose.cpp carries a g_last_end_ms proof pair, the same shape as the two above, to
   guard this teardown line the same way.

   sensors_test_reset_health_() belongs here for the identical reason a fourth time (task 18
   fix round 1, finding 2): g_healthy, g_fails and g_backoff_until are process-lifetime
   statics in sensors.cpp, and a test that calls sim_set_i2c_fail(true) and cleans up with
   `sim_set_i2c_fail(false); sensors_begin();` as its own LAST lines loses that cleanup to the
   same longjmp as every static above the moment an earlier assertion in that body fails —
   leaving the bus reading unhealthy for every later test in the binary that never happens to
   call sensors_begin() itself. Seen directly: one mutation's real diagnostic buried under two
   unrelated downstream DOSE_REFUSED_I2C failures. */
static inline void pb_test_teardown(void) {
  sim_events_clear();
  safety_set_dosing(false);
  safety_float_refusal_count(false);
  cli_stop_clear();
  safety_reset_dose_cooldown_();
  sensors_test_reset_health_();
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

/* Drive a REAL latching dose: the float says OK, nothing ever flows, the dose runs past
   its own prime window, and it is not a console prime. That is §2.7's five conditions,
   and it is the ONLY way the latch can be set -- there is no setter, on purpose. */
static inline void pb_latch_contra(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true);
  sim_set_flow_ml_s(0);                  /* float OK, no flow: the contradiction */
  dose_req_t q = {0};
  q.by_time    = true;
  q.cap_ms     = PB_PRIME_MS_DEFAULT + PB_STALL_MS_DEFAULT + 1000u;
  q.long_prime = false;                  /* a console prime is EXEMPT (§2.7) */
  (void)dose_run(&q);
  TEST_ASSERT_TRUE_MESSAGE(safety_contra(), "pb_latch_contra did not latch");
}
