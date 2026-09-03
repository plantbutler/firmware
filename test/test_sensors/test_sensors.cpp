#include <unity.h>
#include "../support/harness.h"
#include "config.h"
#include "pulses.h"

void setUp(void)    { pb_test_setup(); }
void tearDown(void) { pb_test_teardown(); }

/* §2.14: PB_FLOW_MIN_GAP_US is honest about only biting above 2 kHz, which is why the
   two rate rules exist above it. PB_SCREW_MIN_GAP_US is four times wider. */
static void test_edges_closer_than_the_minimum_gap_are_rejected(void) {
  pulses_begin();
  pulses_isr_flow();
  pulses_isr_flow();                     /* same microsecond: rejected */
  TEST_ASSERT_EQUAL_UINT32(1u, pulses_flow());
  pb_advance(1);                         /* 1000 us > 500 us */
  pulses_isr_flow();
  TEST_ASSERT_EQUAL_UINT32(2u, pulses_flow());

  pulses_isr_screw();
  pulses_isr_screw();
  TEST_ASSERT_EQUAL_UINT32(1u, pulses_screw());
  pb_advance(1);                         /* 1000 us < 2000 us: still rejected */
  pulses_isr_screw();
  TEST_ASSERT_EQUAL_UINT32(1u, pulses_screw());
  pb_advance(2);                         /* 3000 us since the accepted edge */
  pulses_isr_screw();
  TEST_ASSERT_EQUAL_UINT32(2u, pulses_screw());
}

/* §2.14: at the ISR's own 2 kHz ceiling a 250 ml target is reached in ~625 ms, so any
   estimator slower than 100 ms loses the race the in-dose rate rules have to win. */
static void test_the_rate_estimator_reports_over_a_hundred_millisecond_window(void) {
  pulses_begin();
  sim_flow_storm(2000);                  /* edges 500 us apart: at the reject's boundary */
  pb_advance(100);
  uint32_t hz = pulses_flow_rate();
  TEST_ASSERT_TRUE(hz >= 1500u);
  TEST_ASSERT_TRUE(hz <= 2500u);
  TEST_ASSERT_TRUE(hz > (uint32_t)PB_FLOW_MAX_HZ);   /* what the dose-loop rule needs */
  /* the previous window's value stands while the current one fills */
  TEST_ASSERT_EQUAL_UINT32(hz, pulses_flow_rate());
}

static void test_a_counter_snapshot_is_never_torn_by_an_edge(void) {
  pulses_begin();
  pb_advance(1);
  pulses_isr_flow();
  TEST_ASSERT_EQUAL_UINT32(1u, pulses_flow());
  pulses_test_tear_next(1u);             /* an edge lands BETWEEN the snapshot's two reads */
  pb_advance(1);
  TEST_ASSERT_EQUAL_UINT32(2u, pulses_flow());
  TEST_ASSERT_EQUAL_UINT32(2u, pulses_flow());   /* and it settled */
}

/* §7: PB_COAST_MS — impeller spin-down is not a leak. And §1: there is no leak LATCH. */
static void test_leak_does_not_latch_from_coast_down_pulses_after_a_dose(void) {
  pulses_begin();
  pulses_leak_rearm_at(hal_millis() + PB_COAST_MS);
  sim_flow_storm(50);                    /* the impeller coasting down */
  pb_advance(1000);
  sim_flow_storm(0);
  pulses_leak_poll(false);
  TEST_ASSERT_EQUAL_UINT32(0u, pulses_leak_count());
  TEST_ASSERT_FALSE(pulses_leak_seen());

  pb_advance(1500);                      /* past the blanking window */
  pulses_leak_poll(false);               /* arms and rebases */
  sim_flow_storm(50);
  pb_advance(1000);
  sim_flow_storm(0);
  pulses_leak_poll(false);
  TEST_ASSERT_TRUE(pulses_leak_count() > 0u);
  TEST_ASSERT_TRUE(pulses_leak_seen());
}

/* CARRIED DEFECT (task 6 brief): the fake's flow model must not delay the first pulse
   past dose_run()'s prime deadline. Task 18's prime rule (spec §2.8) aborts a dose when
   `el >= prime_ms && got < PB_PRIME_MIN_PULSES`, where `el` and `got` are both read from
   hal_millis()/pulses_flow() at the SAME loop iteration. This is the behavioural property
   that rule depends on: with a healthy nonzero flow rate and the default prime window, the
   first pulse must be counted by the time hal_millis() first reports elapsed ==
   PB_PRIME_MS_DEFAULT, not one millisecond later. Against the model that gates its edge
   emitter on the stale (pre-increment) g_ms read inside advance_1ms_(), the first edge
   lands at elapsed 3001 ms and this case fails at elapsed 3000 ms with pulses_flow() == 0. */
static void test_the_first_flow_edge_lands_at_or_before_the_default_prime_deadline(void) {
  pulses_begin();
  sim_set_flow_ml_s(50);                 /* a healthy nonzero delivery rate */
  hal_pump_write(true);                  /* starts the fake's prime clock (g_pump_on_at_ms) */
  pb_advance((uint32_t)PB_PRIME_MS_DEFAULT);
  TEST_ASSERT_TRUE(pulses_flow() > 0u);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_edges_closer_than_the_minimum_gap_are_rejected);
  RUN_TEST(test_the_rate_estimator_reports_over_a_hundred_millisecond_window);
  RUN_TEST(test_a_counter_snapshot_is_never_torn_by_an_edge);
  RUN_TEST(test_leak_does_not_latch_from_coast_down_pulses_after_a_dose);
  RUN_TEST(test_the_first_flow_edge_lands_at_or_before_the_default_prime_deadline);
  return UNITY_END();
}
