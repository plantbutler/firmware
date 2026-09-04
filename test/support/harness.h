/* test/support/harness.h -- the Unity fixture. A HEADER (spec §10).
   Host arm ([env:native] and friends): hal_sim's injectors and a driven clock.
   Device arm ([env:uno_r4_wifi_test]): real hardware, real time, no injectors -- a case
   needing one is #ifdef PB_SIM'd out. config.h and safety.h are NOT optional: pb_latch_contra()
   names dose_req_t, dose_run(), safety_contra(), PB_BOOT_GAP_MS, PB_PRIME_MS_DEFAULT and
   PB_STALL_MS_DEFAULT, and nothing else this header includes reaches any of them (task 28). */
#pragma once
#include "config.h"
#include "hal.h"
#include "safety.h"
#include <unity.h>

#ifdef PB_SIM
#include "cli.h"
#include "exec.h"
#include "netfsm.h"
#include "pulses.h"
#include "report.h"
#include "sensors.h"
#include "sim.h"

static inline void pb_test_setup(void) {
  sim_reset(false);          /* a cold boot: clock at 0, .noinit cleared */
  hal_begin();
  hal_boot_pump_off();
  (void)hal_wdt_start();     /* KEEP THIS: without it hal_wdt_alive() is false and the
                                 dose ladder refuses every dose in test_dose/test_contra */
  sim_events_clear();
}

/* safety_set_dosing(false) belongs HERE, not at the end of whichever test happens to set it
   true: Unity aborts a failing TEST_ASSERT_* with a longjmp straight into tearDown(), skipping
   every line after it in the case body. g_dosing is process-lifetime state in safety.cpp, and
   sim_reset() (called from pb_test_setup(), not here) only resets hal_sim.cpp's own statics —
   a different translation unit. tearDown() is the one place Unity guarantees runs regardless
   of how the case ended, so it is the only place that can actually promise every case starts
   with g_dosing == false.

   safety_float_refusal_count(false) belongs here for the identical reason (task 15 fix
   round 1): g_float_refusals is the same shape of process-lifetime static in safety.cpp, and a
   test that only clears it as its own last line -- as test_the_flap_counter_trips_after_three_
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
   what teardown needs.

   netfsm_test_reset_retry_() is this list's NINTH entry, for the identical reason (task 25
   fix round 1, finding 3): g_retried and g_connect_starved are process-lifetime statics in
   netfsm.cpp with no reset path but net_begin() and NET_IDLE's own per-report reset. Every
   case in test_netfsm.cpp today calls net_begin() as its own first or second statement,
   which is exactly why this gap produced no failure on its own — the same shape as every
   entry above it before ITS test existed. See netfsm.h's own declaration.

   exec_begin() belongs here as the TENTH entry, for the identical reason again (task 26):
   g_boot_home_due, g_pending, g_cmd, g_last_id and g_last_text are process-lifetime statics
   in exec.cpp, and every case that touches this file calls exec_begin() itself — but a case
   that longjmps out before reaching its own call (or a future case that never calls it at
   all) would otherwise read a PRIOR case's pending command, boot-home flag or last-ack text.
   Unlike the dedicated _test_reset_ wrappers above, exec_begin() needs no second, test-only
   copy: it is already the production entry point and it already has no side effect beyond
   resetting exactly these statics (same shape as report_clear_ack() above) — it does not
   touch the cart, the network or the pump.

   ALL of the above is host-only (task 28): sensors_test_reset_health_() and
   netfsm_test_reset_retry_() are declared under #ifdef PB_NATIVE, and pulses_test_reset_leak_()
   under #if PB_SIM -- none of the three exist in a device build, where PB_NATIVE and PB_SIM
   are both undefined. The device arm's teardown below is a no-op instead. */
static inline void pb_test_teardown(void) {
  sim_events_clear();
  safety_set_dosing(false);
  safety_float_refusal_count(false);
  cli_stop_clear();
  safety_reset_dose_cooldown_();
  sensors_test_reset_health_();
  pulses_test_reset_leak_();
  report_clear_ack();
  netfsm_test_reset_retry_();
  exec_begin();
}

static inline void pb_advance(uint32_t ms) { sim_advance(ms); }

/* task 3: counts call-trace events of one kind. Used five times by test_sensors. */
static inline uint32_t pb_count(sim_ev_kind_t kind) {
  const sim_ev_t *ev; size_t n = sim_events(&ev); uint32_t hits = 0;
  for (size_t i = 0; i < n; ++i) if (ev[i].kind == kind) hits++;
  return hits;
}

/* task 3: the ONE deliberately unfed window in the program is hal_wdt_alive()'s probe.
   Strictly inside: the two feeds that BRACKET the probe are legal. A static inline
   DEFINITION, not a declaration - the definition reads sim_events() and there is nowhere
   else it could live without every suite that includes this header owning a copy. */
static inline void pb_expect_no_feed_between(uint32_t from_ms, uint32_t to_ms) {
  const sim_ev_t *ev; size_t n = sim_events(&ev); uint32_t hits = 0;
  for (size_t i = 0; i < n; ++i)
    if (ev[i].kind == SIM_EV_WDT_FEED && ev[i].at_ms > from_ms && ev[i].at_ms < to_ms) hits++;
  TEST_ASSERT_EQUAL_UINT32(0u, hits);
}

/* task 19: there is no safety_contra_set_(). The latch is settable in exactly one place,
   so the fixture EARNS it by driving a real latching dose. */
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

/* task 24: drive n whole network passes, advancing the fake clock ms between them.
   THE ONE SPELLING of "a pass" -- task 24's cases, task 25's retries and task 26's ack
   cycles all use it. Guarded here (rather than just at the call site) because it calls
   link_fake_pass_begin(), which [env:uno_r4_wifi_test] filters link_fake.cpp out of. */
static inline void pb_net_passes(uint16_t n, uint32_t ms) {
  for (uint16_t i = 0; i < n; ++i) {
    link_fake_pass_begin();
    net_poll(false);         /* not dosing: the dosing loop blocks, so no pass overlaps one */
    if (ms) pb_advance(ms);
  }
}

#else   /* ---- the device arm ([env:uno_r4_wifi_test]): real hardware, real time, no
             injectors. Only the three helpers above that genuinely differ between host and
             board get a body here; the rest (pb_count, pb_expect_no_feed_between,
             pb_latch_contra, pb_net_passes) read sim_events() or the fake link and have no
             device meaning -- the device suite does not call them. ---- */
static inline void pb_test_setup(void)     { hal_begin(); hal_boot_pump_off(); }
static inline void pb_test_teardown(void)  {}
static inline void pb_advance(uint32_t ms) { safety_wait_ms(ms); }   /* fed, on real time */
#endif

/* There is no pb_begin_fake_dose() / pb_end_fake_dose() pair (and there never was one in the
   tree): task 7's two I2C-recovery cases call safety_set_dosing() -- task 5's declared seam,
   spelled exactly that way -- directly. */
