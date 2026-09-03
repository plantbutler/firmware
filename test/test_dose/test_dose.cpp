/* test/test_dose/test_dose.cpp
   The dose suite. Today it carries one case: that a native Unity runner links and runs.
   Tasks 3-6 and 15-19 fill it with the pump-pin, watchdog and refusal-ladder cases. */
#include <unity.h>
#include "../support/harness.h"
#include "config.h"

void setUp(void)    { pb_test_setup(); }
void tearDown(void) { pb_test_teardown(); }

static void test_the_native_runner_links_and_runs(void) {
  /* PB_CONTROLLER comes from [env:native]'s build_flags, so this also proves the flag
     reached the compiler. */
  TEST_ASSERT_EQUAL_STRING("test1", PB_CONTROLLER);
}

/* spec §7: RL_16384 * PR_8192 / (PCLKB/1000), PCLKB = 24 MHz (bsp_clock_cfg.h:8,14).
   Re-derived here so a copied-in constant cannot drift from its own arithmetic. */
static void test_the_watchdog_grant_arithmetic_matches_the_constant(void) {
  TEST_ASSERT_EQUAL_UINT32(5592u, (16384u * 8192u) / (24000000u / 1000u));
  TEST_ASSERT_EQUAL_UINT32((16384u * 8192u) / (24000000u / 1000u), (uint32_t)PB_WDT_GRANTED_MS);
  TEST_ASSERT_EQUAL_UINT32(58u, (uint32_t)PB_WDT_PROBE_MIN_COUNTS);
  /* the probe window must be two orders of magnitude inside the grant (§2.5) */
  TEST_ASSERT_TRUE(PB_WDT_PROBE_MS * 100u < PB_WDT_GRANTED_MS);
}

/* spec §7: the body is assembled into PB_BODY_CAP bytes; `c=` plus its value is the only
   term not counted in PB_BODY_WORST_FIXED. An empty c= is a permanent 400 (butler.py
   parse_report: c must be non-empty). */
static void test_the_body_worst_case_sum_fits_the_body_cap(void) {
  TEST_ASSERT_TRUE(sizeof(PB_CONTROLLER) > 1u);
  TEST_ASSERT_TRUE(sizeof(PB_CONTROLLER) + 2u + PB_BODY_WORST_FIXED <= PB_BODY_CAP);
  TEST_ASSERT_TRUE(PB_HDR_FIXED + PB_BODY_CAP <= PB_TX_CAP);
}

/* spec §4.6: this ships DEFINED, so no backend water command is ever queued until a
   deliberate later commit. */
static void test_the_going_live_flag_ships_defined(void) {
  TEST_ASSERT_EQUAL_INT(1, PB_REPORT_POS_UNKNOWN);
}

/* §2.1: pinMode(D6, OUTPUT) latches PODR = 0 and drives the pin LOW, discarding a
   preceding digitalWrite. The correct sequence is ONE PFS write carrying direction and
   level together, and pinMode never touches D6 at all. */
static void test_boot_configures_d6_with_one_pfs_write_carrying_direction_and_level(void) {
  sim_events_clear();
  hal_boot_pump_off();
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  TEST_ASSERT_EQUAL_UINT32(1u, (uint32_t)n);
  TEST_ASSERT_EQUAL_INT(SIM_EV_PIN_CFG, (int)ev[0].kind);
  TEST_ASSERT_EQUAL_UINT8(6, ev[0].pin);
  TEST_ASSERT_TRUE(ev[0].arg & SIM_PFS_DIR_OUT);     /* the direction */
  TEST_ASSERT_FALSE(ev[0].arg & SIM_PFS_LEVEL_HI);   /* and the OFF level, in the same word */
}

static void test_pinmode_is_never_called_on_the_pump_pin(void) {
  hal_begin();
  hal_boot_pump_off();
  hal_pump_write(true);
  hal_pump_write(false);
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  uint32_t hits = 0;
  for (size_t i = 0; i < n; ++i)
    if (ev[i].kind == SIM_EV_PIN_MODE && ev[i].pin == 6) hits++;
  TEST_ASSERT_EQUAL_UINT32(0u, hits);
}

/* §2.1: R_IOPORT_PinCfg -> R_BSP_PinCfg is one unconditional `PmnPFS = cfg`, so every
   pump write re-states the DIRECTION as well as the level. That is what makes
   safety_tick()'s idle re-assert a REPAIR of a stray pinMode on D6. */
static void test_every_pump_write_restates_the_direction_as_well_as_the_level(void) {
  sim_events_clear();
  hal_pump_write(true);
  hal_pump_write(false);
  hal_pump_write(false);
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  uint32_t writes = 0;
  for (size_t i = 0; i < n; ++i) {
    if (ev[i].kind != SIM_EV_PUMP_WRITE) continue;
    writes++;
    TEST_ASSERT_TRUE(ev[i].arg & SIM_PFS_DIR_OUT);
  }
  TEST_ASSERT_EQUAL_UINT32(3u, writes);
}

static void test_wdt_alive_is_false_only_when_the_counter_is_frozen(void) {
  TEST_ASSERT_TRUE(hal_wdt_alive());              /* healthy: 2929 Hz */
  sim_wdt_stop();                                 /* frozen */
  TEST_ASSERT_FALSE(hal_wdt_alive());
  TEST_ASSERT_EQUAL_UINT32(0u, hal_wdt_last_delta());
  sim_wdt_rate_hz(1000);                          /* moves, but 40 counts < 58 */
  TEST_ASSERT_FALSE(hal_wdt_alive());
}

/* Without this case the suite passes against a probe that feeds — which is exactly how
   this bug survived review the first time (§2.5). */
static void test_wdt_alive_does_not_feed_inside_its_probe_window(void) {
  sim_events_clear();
  TEST_ASSERT_TRUE(hal_wdt_alive());
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  uint32_t feeds = 0, first = 0, last = 0;
  for (size_t i = 0; i < n; ++i) {
    if (ev[i].kind != SIM_EV_WDT_FEED) continue;
    if (feeds == 0) first = ev[i].at_ms;
    last = ev[i].at_ms;
    feeds++;
  }
  TEST_ASSERT_EQUAL_UINT32(2u, feeds);                        /* only the bracketing pair */
  TEST_ASSERT_TRUE(last - first >= PB_WDT_PROBE_MS);          /* the window really was 40 ms */
  pb_expect_no_feed_between(first, last);
}

static void test_wdt_alive_is_true_on_a_counter_that_moves_at_the_real_2929_hz(void) {
  sim_wdt_rate_hz(2929);                          /* PCLKB/8192 = 2929.7 Hz (§7) */
  TEST_ASSERT_TRUE(hal_wdt_alive());
  /* 41 ms of advance (the probe's own hal_millis() reads included) x 2.929 = 120 counts */
  TEST_ASSERT_TRUE(hal_wdt_last_delta() >= PB_WDT_PROBE_MIN_COUNTS);
  TEST_ASSERT_TRUE(hal_wdt_last_delta() <= 130u);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_the_native_runner_links_and_runs);
  RUN_TEST(test_the_watchdog_grant_arithmetic_matches_the_constant);
  RUN_TEST(test_the_body_worst_case_sum_fits_the_body_cap);
  RUN_TEST(test_the_going_live_flag_ships_defined);
  RUN_TEST(test_boot_configures_d6_with_one_pfs_write_carrying_direction_and_level);
  RUN_TEST(test_pinmode_is_never_called_on_the_pump_pin);
  RUN_TEST(test_every_pump_write_restates_the_direction_as_well_as_the_level);
  RUN_TEST(test_wdt_alive_is_false_only_when_the_counter_is_frozen);
  RUN_TEST(test_wdt_alive_does_not_feed_inside_its_probe_window);
  RUN_TEST(test_wdt_alive_is_true_on_a_counter_that_moves_at_the_real_2929_hz);
  return UNITY_END();
}
