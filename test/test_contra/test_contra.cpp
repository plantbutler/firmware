/* test_contra.cpp: the float/flow contradiction latch -- when it sets, what it refuses, what survives a reset, and how it reaches the wire. */
#include <unity.h>
#include <string.h>
#include "../support/bodies.h"
#include "../support/harness.h"
#include "cart.h"
#include "cli.h"
#include "config.h"
#include "exec.h"
#include "netfsm.h"
#include "report.h"
#include "safety.h"
#include "sim.h"

void setUp(void)    { pb_test_setup(); }
void tearDown(void) { pb_test_teardown(); }

/* The float said OK and the dose it authorised produced no flow at all: two independent
   sensors contradict each other. The safe reading is an empty tank and a stuck float; the
   safe response is not "end this dose and let the next one start" but refuse everything
   until a human looks. */
void test_latch_sets_when_the_float_said_ok_and_no_pulse_ever_arrived(void) {
  TEST_ASSERT_FALSE(safety_contra());
  pb_latch_contra();
  TEST_ASSERT_TRUE(safety_contra());
}

/* The dose that sets the latch aborts for an ordinary reason ("noflow") first, and the
   override replaces that with "contra" before the call returns. Every other "contra"
   assertion here reads a second, already-refused dose and would pass with the override
   deleted; this is the only case that reads the setting dose's own error. */
void test_latch_overrides_err_to_contra_on_the_dose_that_sets_it(void) {
  pb_latch_contra();
  TEST_ASSERT_EQUAL_STRING("contra", safety_last_err());
}

/* The float dropped: the two sensors agree that the tank ran out. Ordinary abort. */
void test_latch_does_not_set_when_the_float_dropped_mid_dose(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true); sim_set_flow_ml_s(0);
  sim_set_float_at_ms(500u, false);          /* the fake drops D5 mid-dose */
  dose_req_t q = {0}; q.by_time = true;
  q.cap_ms = PB_PRIME_MS_DEFAULT + PB_STALL_MS_DEFAULT + 1000u;
  TEST_ASSERT_EQUAL(DOSE_ABORT_FLOAT, dose_run(&q));
  TEST_ASSERT_FALSE(safety_contra());
}

/* Water was moving and then stopped -- a hose off a pot, a tank sucked dry mid-dose: the
   meter and the float agree. Ordinary abort, no latch. */
void test_latch_does_not_set_when_flow_started_and_then_stalled(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true);
  sim_set_flow_burst_pulses(20u);            /* flows, then stops */
  dose_req_t q = {0}; q.by_time = true;
  q.cap_ms = PB_PRIME_MS_DEFAULT + PB_STALL_MS_DEFAULT + 1000u;
  TEST_ASSERT_EQUAL(DOSE_ABORT_NOFLOW, dose_run(&q));
  TEST_ASSERT_FALSE(safety_contra());
}

/* No evidence either way. */
void test_latch_does_not_set_when_the_dose_was_stopped_before_the_prime_window(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true); sim_set_flow_ml_s(0);
  sim_serial_rx_at_ms(200u, "stop\n");
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 30000u;
  TEST_ASSERT_EQUAL(DOSE_ABORT_STOP, dose_run(&q));
  TEST_ASSERT_FALSE(safety_contra());
}

/* 5 s is past PB_PRIME_MS_DEFAULT (3 s) but inside PB_PRIME_LONG_MS (15 s): this dose's own
   window has not elapsed, so even without the long_prime exemption it must not latch. */
void test_latch_uses_the_doses_own_prime_window_not_the_configured_default(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true); sim_set_flow_ml_s(0);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 5000u; q.long_prime = true;
  (void)dose_run(&q);
  TEST_ASSERT_TRUE(dose_last_ms() > PB_PRIME_MS_DEFAULT);      /* past the default window */
  TEST_ASSERT_TRUE(dose_last_ms() < PB_PRIME_LONG_MS);         /* inside its own */
  TEST_ASSERT_FALSE(safety_contra());
}

/* A console prime on a line that has never held water satisfies every other condition on
   its first attempt; without the exemption it would latch immediately. */
void test_latch_does_not_set_for_a_console_prime_dose(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true); sim_set_flow_ml_s(0);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_PRIME_CAP_MS; q.long_prime = true;
  TEST_ASSERT_EQUAL(DOSE_ABORT_NOFLOW, dose_run(&q));
  TEST_ASSERT_FALSE(safety_contra());
}

/* A refused dose never reaches the latch's setter; repeated float refusals are the flap
   counter's business, invisible here. */
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

/* The exemption is about setting the latch, never about escaping it. */
void test_latch_refuses_every_subsequent_dose_including_a_console_one(void) {
  pb_latch_contra();
  pb_advance(PB_DOSE_MIN_GAP_MS + 1u);
  sim_set_float(true); sim_set_flow_ml_s(30);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_PRIME_CAP_MS; q.long_prime = true;
  TEST_ASSERT_EQUAL(DOSE_REFUSED_CONTRA, dose_run(&q));
}

/* Homing is not watering: a cart left over outlet N holds that gate open under the
   reservoir head for as long as the latch stands, which may be days. */
void test_latch_does_not_refuse_homing(void) {
  pb_latch_contra();
  safety_dry_set(true);                       /* both latches, at once */
  sim_set_screw_pulse_ms(2);
  sim_set_home_region(0u, 40u);
  sim_set_cart_at(600u);
  TEST_ASSERT_TRUE(cart_home());
  TEST_ASSERT_TRUE(cart_parked());
  TEST_ASSERT_TRUE(safety_contra());          /* and homing did not clear it */
}

/* The watchdog reset is precisely the event that would otherwise erase the latch. */
void test_latch_survives_a_warm_reset_and_not_a_cold_one(void) {
  pb_latch_contra();
  /* sim_reset() re-enters the boot path, so the .noinit verify has already run when it
     returns; a second noinit_begin() would advance the boot counter twice. */
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

/* `dry off` is a different latch, and a successful home is not evidence about water. */
void test_latch_is_not_cleared_by_dry_off_or_by_a_successful_home(void) {
  pb_latch_contra();
  TEST_ASSERT_TRUE(cli_dispatch("dry off"));
  TEST_ASSERT_TRUE(safety_contra());
  sim_set_screw_pulse_ms(2); sim_set_home_region(0u, 40u); sim_set_cart_at(600u);
  TEST_ASSERT_TRUE(cart_home());
  TEST_ASSERT_TRUE(safety_contra());
}

/* Dropping the float at 500 ms, as the mid-dose case above does, leaves elapsed < prime_ms,
   so that case passes even without the latch's own fresh float read. Dropping it exactly at
   the prime boundary makes every other latch condition hold -- DOSE_ABORT_FLOAT, elapsed
   past prime_ms, zero pulses -- so that fresh read is the only thing between this dose and
   contra=1. */
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

static void test_boot_self_home_runs_under_both_latches(void) {
  /* The cart's position statics have no teardown reset: an earlier case in this binary that
     homed for real would leave cart_parked() true before exec_pending() ever runs here.
     cart_begin() is the only reset. */
  cart_begin();
  /* Contra first, then dry: the ladder refuses a dry dose rungs above where a granted dose
     could ever set contra, so the other order never latches at all. */
  pb_latch_contra();
  safety_dry_set(true);
  TEST_ASSERT_TRUE(safety_contra());
  exec_begin();
  pb_advance(PB_BOOT_HOME_MS + 1);
  exec_pending();
  TEST_ASSERT_TRUE(cart_parked());        /* parking is more wanted after a latch, not less */
  TEST_ASSERT_TRUE(safety_contra());      /* and homing clears nothing */
}

/* err=contra, ch207=1 and float=0 are the only channel by which the backend and the phone
   ever learn about the latch at all. */
static void test_latch_reports_err_contra_and_ch207_and_float_zero(void) {
  sensors_begin();
  sim_set_float(true);                    /* the tank samples fine; the latch outranks it */
  pb_latch_contra();
  report_clear_ack();                     /* no ack, so err= falls through to the latch */
  report_stamp();
  char b[PB_BODY_CAP];
  TEST_ASSERT_TRUE(report_build(b, sizeof b) > 0);
  TEST_ASSERT_NOT_NULL_MESSAGE(strstr(b, " err=contra"), b);
  TEST_ASSERT_NOT_NULL_MESSAGE(strstr(b, " ch207=1"),    b);
  TEST_ASSERT_NOT_NULL_MESSAGE(strstr(b, " float=0"),    b);
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
  RUN_TEST(test_boot_self_home_runs_under_both_latches);
  RUN_TEST(test_latch_reports_err_contra_and_ch207_and_float_zero);
  return UNITY_END();
}
