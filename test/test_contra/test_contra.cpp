/* test/test_contra/test_contra.cpp -- the float/flow contradiction latch, spec §2.7.
   Task 19. The float said OK -- permission granted by the one input whose whole design
   (DECISIONS #12) is that failure reads as refusal -- and the dose that permission
   authorised produced no flow at all. Two independent sensors contradict each other.
   The safe reading is that the tank is empty and the float is stuck, and the safe
   response is not "end this dose and let the next one start" (all a no-flow abort does)
   but refuse everything until a human looks.

   There is deliberately no safety_contra_set_(): pb_latch_contra() (harness.h) is the
   ONLY way any test earns the latch, because it drives a real latching dose through the
   real dose_run() -- exactly what an operator's rig would do. A test hook that set the
   flag directly would be a second setter, which is the very thing this design exists to
   prevent. */
#include <unity.h>
#include <string.h>
#include "../support/harness.h"
#include "cart.h"
#include "cli.h"
#include "config.h"
#include "safety.h"
#include "sim.h"

void setUp(void)    { pb_test_setup(); }
void tearDown(void) { pb_test_teardown(); }

/* §2.7. The float said OK - permission granted by the one input whose whole design is that
   failure reads as refusal - and the dose that permission authorised produced no flow at
   all. Two independent sensors contradict each other. The safe reading is that the tank is
   empty and the float is stuck, and the safe response is not "end this dose and let the
   next one start" but refuse everything until a human looks. */
void test_latch_sets_when_the_float_said_ok_and_no_pulse_ever_arrived(void) {
  TEST_ASSERT_FALSE(safety_contra());
  pb_latch_contra();
  TEST_ASSERT_TRUE(safety_contra());
}

/* Fix round 1 (review finding 1). The setter's own comment says `safety_set_err("contra")`
   runs AFTER `g_last_err = err_of(r)` "because the latch is the louder fact": the dose that
   SETS the latch aborts for an ordinary reason first (here DOSE_ABORT_NOFLOW, "noflow"),
   and the override is what replaces that with "contra" before dose_run() ever returns.
   Every other "contra" assertion in this suite (test_dose_refused_when_the_contradiction_
   latch_is_set, test_the_ladder_reports_contra_above_dry) reads safety_last_err() after a
   SECOND, already-refused dose, where DOSE_REFUSED_CONTRA maps to "contra" through the
   ordinary err_of() switch regardless of whether this override line exists at all -- they
   would keep passing even with the override deleted outright. This is the only case that
   reads safety_last_err() on the dose that actually sets the latch. */
void test_latch_overrides_err_to_contra_on_the_dose_that_sets_it(void) {
  pb_latch_contra();
  TEST_ASSERT_EQUAL_STRING("contra", safety_last_err());
}

/* The float dropped: the two sensors AGREE that the tank ran out. Ordinary abort. */
void test_latch_does_not_set_when_the_float_dropped_mid_dose(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true); sim_set_flow_ml_s(0);
  sim_set_float_at_ms(500u, false);          /* the fake drops D5 mid-dose */
  dose_req_t q = {0}; q.by_time = true;
  q.cap_ms = PB_PRIME_MS_DEFAULT + PB_STALL_MS_DEFAULT + 1000u;
  TEST_ASSERT_EQUAL(DOSE_ABORT_FLOAT, dose_run(&q));
  TEST_ASSERT_FALSE(safety_contra());
}

/* got > 0: the meter and the float agree that water WAS moving and then stopped - a hose
   off a pot, a tank sucked dry mid-dose. DOSE_ABORT_NOFLOW, ordinary, no latch. */
void test_latch_does_not_set_when_flow_started_and_then_stalled(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true);
  sim_set_flow_burst_pulses(20u);            /* flows, then stops */
  dose_req_t q = {0}; q.by_time = true;
  q.cap_ms = PB_PRIME_MS_DEFAULT + PB_STALL_MS_DEFAULT + 1000u;
  TEST_ASSERT_EQUAL(DOSE_ABORT_NOFLOW, dose_run(&q));
  TEST_ASSERT_FALSE(safety_contra());
}

/* Stopped before its own prime window: no evidence either way. */
void test_latch_does_not_set_when_the_dose_was_stopped_before_the_prime_window(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true); sim_set_flow_ml_s(0);
  sim_serial_rx_at_ms(200u, "stop\n");
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 30000u;
  TEST_ASSERT_EQUAL(DOSE_ABORT_STOP, dose_run(&q));
  TEST_ASSERT_FALSE(safety_contra());
}

/* THIS dose's window, not the configured default. A prime dose that ran 5 s - past
   PB_PRIME_MS_DEFAULT (3 s) but inside PB_PRIME_LONG_MS (15 s) - has not yet had its own
   window elapse, so even without the long_prime exemption it must not latch. */
void test_latch_uses_the_doses_own_prime_window_not_the_configured_default(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true); sim_set_flow_ml_s(0);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 5000u; q.long_prime = true;
  (void)dose_run(&q);
  TEST_ASSERT_TRUE(dose_last_ms() > PB_PRIME_MS_DEFAULT);      /* past the DEFAULT window */
  TEST_ASSERT_TRUE(dose_last_ms() < PB_PRIME_LONG_MS);         /* inside its OWN */
  TEST_ASSERT_FALSE(safety_contra());
}

/* Bring-up 7a's own command, on a line that has never held water, satisfies every other
   condition on its FIRST attempt. Without this exemption 7a latches immediately and §13's
   instruction to run it again would be wrong. */
void test_latch_does_not_set_for_a_console_prime_dose(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true); sim_set_flow_ml_s(0);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_PRIME_CAP_MS; q.long_prime = true;
  TEST_ASSERT_EQUAL(DOSE_ABORT_NOFLOW, dose_run(&q));
  TEST_ASSERT_FALSE(safety_contra());
}

/* A refused dose never reaches dose_end_ml_(), so it can never latch - which is also why
   the float-flap counter of task 15 exists: repeated float REFUSALS are invisible here. */
void test_latch_does_not_set_for_a_dose_that_was_refused(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(false);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 5000u;
  TEST_ASSERT_EQUAL(DOSE_REFUSED_FLOAT, dose_run(&q));
  TEST_ASSERT_FALSE(safety_contra());
}

void test_dose_refused_when_the_contradiction_latch_is_set(void) {
  pb_latch_contra();
  pb_advance(PB_DOSE_MIN_GAP_MS + 1u);
  sim_set_float(true); sim_set_flow_ml_s(30);
  dose_req_t q = {0}; q.ml = 100u; q.cap_ms = 10000u; q.need_pos = false;
  TEST_ASSERT_EQUAL(DOSE_REFUSED_CONTRA, dose_run(&q));
  TEST_ASSERT_EQUAL_STRING("contra", safety_last_err());
}

/* including a CONSOLE one: the exemption is about setting the latch, never about
   escaping it. `pump 20000 prime` under a latch is refused like everything else. */
void test_latch_refuses_every_subsequent_dose_including_a_console_one(void) {
  pb_latch_contra();
  pb_advance(PB_DOSE_MIN_GAP_MS + 1u);
  sim_set_float(true); sim_set_flow_ml_s(30);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_PRIME_CAP_MS; q.long_prime = true;
  TEST_ASSERT_EQUAL(DOSE_REFUSED_CONTRA, dose_run(&q));
}

/* §2.0, §2.9, §2.11: homing is not watering. A cart left over outlet N holds that gate
   open under the reservoir head for as long as the latch stands, which may be days.
   Parking is MORE wanted after a latch, not less. */
void test_latch_does_not_refuse_homing(void) {
  pb_latch_contra();
  safety_dry_set(true);                       /* both latches, at once */
  sim_set_screw_pulse_ms(2);
  sim_set_home_region(0u, 40u);
  sim_set_cart_at(600u);
  TEST_ASSERT_TRUE(cart_home());
  TEST_ASSERT_TRUE(cart_parked());
  TEST_ASSERT_TRUE(safety_contra());          /* and homing did NOT clear it */
}

/* §2.3: .noinit survives a warm reset - the watchdog reset is precisely the event that
   would otherwise erase the latch - and a cold boot starts clean. */
void test_latch_survives_a_warm_reset_and_not_a_cold_one(void) {
  pb_latch_contra();
  /* sim_reset() re-enters the boot path (task 3), so the .noinit verify has already run
     when it returns; calling noinit_begin() again would advance the boot counter twice. */
  sim_reset(true);
  TEST_ASSERT_TRUE_MESSAGE(safety_contra(), "the latch did not survive a warm reset");
  sim_reset(false);
  TEST_ASSERT_FALSE_MESSAGE(safety_contra(), "the latch survived a COLD boot");
}

void test_latch_clears_only_on_the_literal_two_token_command(void) {
  pb_latch_contra();
  TEST_ASSERT_FALSE(cli_dispatch("clear"));
  TEST_ASSERT_FALSE(cli_dispatch("clearcontra"));
  TEST_ASSERT_TRUE(safety_contra());
  TEST_ASSERT_TRUE(cli_dispatch("clear contra"));
  TEST_ASSERT_FALSE(safety_contra());
}

/* It clears on NOTHING else. `dry off` is a different latch, and a successful home is not
   evidence about water. A reader who adds either as a clear has removed the feature. */
void test_latch_is_not_cleared_by_dry_off_or_by_a_successful_home(void) {
  pb_latch_contra();
  TEST_ASSERT_TRUE(cli_dispatch("dry off"));
  TEST_ASSERT_TRUE(safety_contra());
  sim_set_screw_pulse_ms(2); sim_set_home_region(0u, 40u); sim_set_cart_at(600u);
  TEST_ASSERT_TRUE(cart_home());
  TEST_ASSERT_TRUE(safety_contra());
}

/* Rung 3, actually discriminated. test_latch_does_not_set_when_the_float_dropped_mid_dose
   drops the float at 500 ms -- well inside the 3 s prime window -- so elapsed_ms >=
   prime_ms is ALREADY false on its own there, and mutation testing confirms it: with the
   "float still OK" read deleted, that case still passes. This one drops the float exactly
   at the prime window's own boundary, so the loop's rule 7 (the float check), not rule 5
   (prime/noflow), is what ends the dose -- DOSE_ABORT_FLOAT, with elapsed_ms comfortably
   past prime_ms and got_pulses == 0, so all of the OTHER four conditions hold. The ONLY
   thing standing between this dose and a latch is dose_end_ml_()'s fresh, right-now read
   of the float pin -- delete it and this same case reports contra=1. */
void test_latch_does_not_set_when_the_float_drops_at_the_prime_boundary(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true); sim_set_flow_ml_s(0);
  sim_set_float_at_ms(PB_PRIME_MS_DEFAULT, false);
  dose_req_t q = {0}; q.by_time = true;
  q.cap_ms = PB_PRIME_MS_DEFAULT + PB_STALL_MS_DEFAULT + 1000u;
  dose_result_t r = dose_run(&q);
  TEST_ASSERT_EQUAL(DOSE_ABORT_FLOAT, r);
  TEST_ASSERT_TRUE(dose_last_ms() >= PB_PRIME_MS_DEFAULT);
  TEST_ASSERT_EQUAL_UINT32(0u, dose_last_pulses());
  TEST_ASSERT_FALSE(safety_contra());
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_latch_sets_when_the_float_said_ok_and_no_pulse_ever_arrived);
  RUN_TEST(test_latch_overrides_err_to_contra_on_the_dose_that_sets_it);
  RUN_TEST(test_latch_does_not_set_when_the_float_dropped_mid_dose);
  RUN_TEST(test_latch_does_not_set_when_flow_started_and_then_stalled);
  RUN_TEST(test_latch_does_not_set_when_the_dose_was_stopped_before_the_prime_window);
  RUN_TEST(test_latch_uses_the_doses_own_prime_window_not_the_configured_default);
  RUN_TEST(test_latch_does_not_set_for_a_console_prime_dose);
  RUN_TEST(test_latch_does_not_set_for_a_dose_that_was_refused);
  RUN_TEST(test_dose_refused_when_the_contradiction_latch_is_set);
  RUN_TEST(test_latch_refuses_every_subsequent_dose_including_a_console_one);
  RUN_TEST(test_latch_does_not_refuse_homing);
  RUN_TEST(test_latch_survives_a_warm_reset_and_not_a_cold_one);
  RUN_TEST(test_latch_clears_only_on_the_literal_two_token_command);
  RUN_TEST(test_latch_is_not_cleared_by_dry_off_or_by_a_successful_home);
  RUN_TEST(test_latch_does_not_set_when_the_float_drops_at_the_prime_boundary);
  return UNITY_END();
}
