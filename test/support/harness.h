/* test/support/harness.h — the Unity fixture over hal_sim. A header, not a suite. */
#pragma once
#include <unity.h>
#include "cli.h"
#include "config.h"
#include "hal.h"
#include "pulses.h"
#include "report.h"
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
   unrelated downstream DOSE_REFUSED_I2C failures.

   pulses_test_reset_leak_() belongs here for the identical reason a fifth time (task 22 fix
   round 2): g_leak_count is a process-lifetime static in pulses.cpp with no reset of its own,
   and a case that storms the meter and never calls pulses_begin() itself — pulses_begin() would
   also zero g_flow/g_screw and the rate window, which is not this cleanup's business — leaves
   every later case in the binary reading a nonzero leak count for a reason that has nothing to
   do with what it tests. Proved vacuous, not just latent: a review reproduced
   test_ch205_counts_leak_pulses_and_err_leak_reaches_the_wire (test_report.cpp) still passing
   with its OWN storm-and-poll stimulus removed entirely, asserting purely on the previous
   case's residue.

   report_clear_ack() belongs here for the same reason a sixth time: g_ack/g_ack_set are
   process-lifetime statics in report.cpp, and every current case happens to set or clear the
   ack slot itself before asserting on it — but a failing assertion mid-case longjmps past that
   self-cleanup exactly like every static above, leaving the slot dirty for whatever runs next.
   Unlike the others this one needs no dedicated test-only wrapper: report_clear_ack() is
   already the production entry point exec_pending() will call, and it already does exactly
   what teardown needs. */
static inline void pb_test_teardown(void) {
  sim_events_clear();
  safety_set_dosing(false);
  safety_float_refusal_count(false);
  cli_stop_clear();
  safety_reset_dose_cooldown_();
  sensors_test_reset_health_();
  pulses_test_reset_leak_();
  report_clear_ack();
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
/* Drive n whole network passes, advancing the fake clock ms between them. THE ONE
   SPELLING: task 24's own cases, task 25's retry cases and task 26's ack-cycle cases all
   use it, so a change to what "a pass" means lands once. Guarded #ifdef PB_SIM -- it names
   the fake link, which the device test env filters out. */
#ifdef PB_SIM
#  include "netfsm.h"
static inline void pb_net_passes(uint16_t n, uint32_t ms) {
  for (uint16_t i = 0; i < n; ++i) {
    link_fake_pass_begin();
    net_poll(false);            /* not dosing: dose_run() blocks, so a pass cannot overlap one */
    if (ms) pb_advance(ms);
  }
}
#endif

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
