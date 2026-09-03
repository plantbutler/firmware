#include <unity.h>
#include "../support/harness.h"
#include "config.h"
#include "pulses.h"
#include "sensors.h"
#include "safety.h"
#include "pins.h"

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

/* The PCF8575 is quasi-bidirectional: an input must be written HIGH to be readable, and
   P4 is the home hall (cad/wiring/nets.py P4: "10 k pull-up (R3); write P4 HIGH before
   reading"). So every select writes P4..P15 HIGH and only P0..P3 carry the channel. */
static void test_select_holds_p4_high_so_the_home_hall_stays_readable(void) {
  (void)sensors_begin();
  sim_events_clear();
  TEST_ASSERT_TRUE(sensors_select(3));
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  uint32_t writes = 0;
  for (size_t i = 0; i < n; ++i) {
    if (ev[i].kind != SIM_EV_I2C_WRITE || ev[i].pin != I2C_ADDR_EXPANDER) continue;
    TEST_ASSERT_EQUAL_UINT32(0xFFF3u, ev[i].arg);
    writes++;
  }
  TEST_ASSERT_EQUAL_UINT32(1u, writes);
}

/* Bring-up 2's recipe verbatim: select, >= 1 ms, read twice, keep the SECOND. The first
   conversion after a select still carries the previous channel on a 10 k source. */
static void test_read_discards_the_first_conversion_and_keeps_the_second(void) {
  (void)sensors_begin();
  sim_set_channel(0, 1111);
  sim_set_channel(1, 2222);
  uint16_t v = 0;
  TEST_ASSERT_TRUE(sensors_read_raw(0, &v));
  sim_events_clear();
  TEST_ASSERT_TRUE(sensors_read_raw(1, &v));
  TEST_ASSERT_EQUAL_UINT16(2222u, v);
  TEST_ASSERT_EQUAL_UINT32(2u, pb_count(SIM_EV_ADC));
}

static void test_an_i2c_error_is_reported_as_error_not_as_zero(void) {
  (void)sensors_begin();
  sim_set_channel(2, 9999);
  sim_set_i2c_fail(true);
  uint16_t v = 0x5A5Au;
  TEST_ASSERT_FALSE(sensors_read_raw(2, &v));
  TEST_ASSERT_EQUAL_UINT16(0x5A5Au, v);        /* untouched: never a value on failure */
  bool home = true;
  TEST_ASSERT_FALSE(sensors_home_hall(&home)); /* an error, never "not home" */
}

/* §3: a healthy sweep is ~18 ms and a wedged one is 7 s at the core's fixed 1000 ms
   transfer timeout. The dog's window is 5592 ms. */
static void test_sweep_feeds_the_watchdog_between_channels(void) {
  (void)sensors_begin();
  sim_events_clear();
  (void)sensors_sweep();
  TEST_ASSERT_TRUE(pb_count(SIM_EV_WDT_FEED) >= (uint32_t)PB_CHANNELS + 1u);
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  uint32_t writes_since_feed = 0;
  for (size_t i = 0; i < n; ++i) {
    if (ev[i].kind == SIM_EV_WDT_FEED) writes_since_feed = 0;
    else if (ev[i].kind == SIM_EV_I2C_WRITE) {
      writes_since_feed++;
      TEST_ASSERT_TRUE(writes_since_feed <= 1u);   /* never two transfers un-fed */
    }
  }
}

static void test_sweep_reads_the_open_canary_channel_every_time(void) {
  (void)sensors_begin();
  for (uint8_t s = 0; s < 2; ++s) {
    sim_events_clear();
    (void)sensors_sweep();
    const sim_ev_t *ev; size_t n = sim_events(&ev);
    uint32_t canary_selects = 0;
    for (size_t i = 0; i < n; ++i)
      if (ev[i].kind == SIM_EV_I2C_WRITE && (ev[i].arg & 0x0Fu) == (uint32_t)PB_CANARY_CHANNEL)
        canary_selects++;
    TEST_ASSERT_EQUAL_UINT32(1u, canary_selects);
  }
}

/* §5: an unpowered mux, a broken S-line or a floating EN gives the SAME ADC value on
   every channel with no error raised anywhere, and from butler's side that is
   byte-identical to five equally-dry pots. So the wired channels are omitted, not sent. */
static void test_a_stuck_mux_is_reported_as_an_error_not_as_readings(void) {
  (void)sensors_begin();
  for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) sim_set_channel(ch, (uint16_t)(1000u + ch));
  sim_set_channel(PB_CANARY_CHANNEL, 4321u);
  TEST_ASSERT_TRUE(sensors_sweep());
  TEST_ASSERT_FALSE(sensors_stuck());
  TEST_ASSERT_TRUE(sensors_valid(0));
  TEST_ASSERT_EQUAL_UINT16(1000u, sensors_value(0));

  sim_set_mux_stuck(true);
  TEST_ASSERT_FALSE(sensors_sweep());
  TEST_ASSERT_TRUE(sensors_stuck());
  for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) TEST_ASSERT_FALSE(sensors_valid(ch));
}

static void test_three_consecutive_failures_back_off_and_mark_the_bus_unhealthy(void) {
  (void)sensors_begin();
  TEST_ASSERT_TRUE(sensors_i2c_healthy());
  sim_set_i2c_fail(true);
  for (uint8_t i = 0; i < PB_I2C_FAIL_LIMIT; ++i) (void)sensors_select(0);
  TEST_ASSERT_FALSE(sensors_i2c_healthy());
  TEST_ASSERT_TRUE(sensors_i2c_errors() >= (uint32_t)PB_I2C_FAIL_LIMIT);

  sim_events_clear();                       /* while backing off, the bus is not touched */
  TEST_ASSERT_FALSE(sensors_select(0));
  TEST_ASSERT_EQUAL_UINT32(0u, pb_count(SIM_EV_I2C_WRITE));

  sim_set_i2c_fail(false);
  pb_advance((uint32_t)PB_I2C_BACKOFF_MS + 1u);
  TEST_ASSERT_TRUE(sensors_select(0));
  TEST_ASSERT_TRUE(sensors_i2c_healthy());
}

/* §2.13: A4/A5 are the mux select lines and the home hall — the input that gates the
   pump. A back-off that expires mid-dose simply stays expired until the dose ends. */
static void test_i2c_recovery_never_runs_while_the_pump_is_asserted(void) {
  (void)sensors_begin();
  sim_set_i2c_fail(true);
  for (uint8_t i = 0; i < PB_I2C_FAIL_LIMIT; ++i) (void)sensors_select(0);
  TEST_ASSERT_FALSE(sensors_i2c_healthy());
  sim_set_i2c_fail(false);
  pb_advance((uint32_t)PB_I2C_BACKOFF_MS + 1u);

  safety_set_dosing(true);
  sim_events_clear();
  TEST_ASSERT_FALSE(sensors_select(0));
  TEST_ASSERT_EQUAL_UINT32(0u, pb_count(SIM_EV_I2C_WRITE));
  TEST_ASSERT_EQUAL_UINT32(0u, pb_count(SIM_EV_PIN_CFG));   /* no bit-bang either */
  TEST_ASSERT_FALSE(sensors_i2c_healthy());

  safety_set_dosing(false);
  TEST_ASSERT_TRUE(sensors_select(0));      /* and it runs the moment the dose ends */
}

/* A FIXED count, never an "until SDA releases" condition — that condition is an
   unbounded loop on a bus a stuck device is holding down (§2.13). */
static void test_recovery_is_a_fixed_nine_clocks_with_sda_held_low(void) {
  (void)sensors_begin();
  sim_set_i2c_fail(true);
  for (uint8_t i = 0; i < PB_I2C_FAIL_LIMIT; ++i) (void)sensors_select(0);
  sim_set_i2c_fail(false);
  pb_advance((uint32_t)PB_I2C_BACKOFF_MS + 1u);
  sim_events_clear();
  TEST_ASSERT_TRUE(sensors_select(0));

  const sim_ev_t *ev; size_t n = sim_events(&ev);
  uint32_t scl = 0, sda = 0;
  for (size_t i = 0; i < n; ++i) {
    if (ev[i].kind != SIM_EV_PIN_CFG) continue;
    if (ev[i].pin == 19u) scl++;            /* A5 == PIN_A0 + 5 == 19 */
    if (ev[i].pin == 18u) sda++;            /* A4 == 18 */
  }
  TEST_ASSERT_EQUAL_UINT32(2u * (uint32_t)PB_I2C_RECOVER_CLOCKS, scl);  /* nine high/low pairs */
  TEST_ASSERT_EQUAL_UINT32(1u, sda);        /* SDA released once, then the fixed clocks */
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_edges_closer_than_the_minimum_gap_are_rejected);
  RUN_TEST(test_the_rate_estimator_reports_over_a_hundred_millisecond_window);
  RUN_TEST(test_a_counter_snapshot_is_never_torn_by_an_edge);
  RUN_TEST(test_leak_does_not_latch_from_coast_down_pulses_after_a_dose);
  RUN_TEST(test_the_first_flow_edge_lands_at_or_before_the_default_prime_deadline);
  RUN_TEST(test_select_holds_p4_high_so_the_home_hall_stays_readable);
  RUN_TEST(test_read_discards_the_first_conversion_and_keeps_the_second);
  RUN_TEST(test_an_i2c_error_is_reported_as_error_not_as_zero);
  RUN_TEST(test_sweep_feeds_the_watchdog_between_channels);
  RUN_TEST(test_sweep_reads_the_open_canary_channel_every_time);
  RUN_TEST(test_a_stuck_mux_is_reported_as_an_error_not_as_readings);
  RUN_TEST(test_three_consecutive_failures_back_off_and_mark_the_bus_unhealthy);
  RUN_TEST(test_i2c_recovery_never_runs_while_the_pump_is_asserted);
  RUN_TEST(test_recovery_is_a_fixed_nine_clocks_with_sda_held_low);
  return UNITY_END();
}
