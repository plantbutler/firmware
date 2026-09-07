/* test_dose.cpp: the dose ladder, the float debounce, the latches and the safety loop, on the host. */
#include <unity.h>
#include <string.h>
#include "../support/harness.h"
#include "cart.h"
#include "cli.h"
#include "config.h"
#include "noinit.h"
#include "safety.h"
#include "pulses.h"
#include "sensors.h"

void setUp(void)    { pb_test_setup(); }
void tearDown(void) { pb_test_teardown(); }

/* How many dose_result_t arms this build can drive, counted: all nineteen, CONTRA included,
   through pb_latch_contra() (harness.h), the only way a test may earn that latch. */
#define PB_DRIVABLE_RESULTS 19

static void test_the_native_runner_links_and_runs(void) {
  /* PB_CONTROLLER is an [env:native] build flag, so this also proves the flag reached
     the compiler. */
  TEST_ASSERT_EQUAL_UINT(7u, (unsigned)PB_CONTROLLER);
}

/* RL_16384 * PR_8192 / (PCLKB/1000), PCLKB = 24 MHz (bsp_clock_cfg.h). Re-derived so a
   copied-in constant cannot drift from its own arithmetic. */
static void test_the_watchdog_grant_arithmetic_matches_the_constant(void) {
  TEST_ASSERT_EQUAL_UINT32(5592u, (16384u * 8192u) / (24000000u / 1000u));
  TEST_ASSERT_EQUAL_UINT32((16384u * 8192u) / (24000000u / 1000u), (uint32_t)PB_WDT_GRANTED_MS);
  TEST_ASSERT_EQUAL_UINT32(58u, (uint32_t)PB_WDT_PROBE_MIN_COUNTS);
  /* the probe window must sit two orders of magnitude inside the grant */
  TEST_ASSERT_TRUE(PB_WDT_PROBE_MS * 100u < PB_WDT_GRANTED_MS);
}

/* The body is assembled into PB_BODY_CAP bytes; `c=` plus its value is the only term not
   counted in PB_BODY_WORST_FIXED. The backend answers an empty or out-of-range c= with a
   400 for good. */
static void test_the_body_worst_case_sum_fits_the_body_cap(void) {
  TEST_ASSERT_TRUE(PB_CONTROLLER >= 0 && PB_CONTROLLER <= 255);
  TEST_ASSERT_TRUE(PB_CONTROLLER_WIRE + 2u + PB_BODY_WORST_FIXED <= PB_BODY_CAP);
  TEST_ASSERT_TRUE(PB_HDR_FIXED + PB_BODY_CAP <= PB_TX_CAP);
}

/* Ships DEFINED: no backend water command is queued until somebody flips it on purpose. */
static void test_the_going_live_flag_ships_defined(void) {
#if PB_REPORT_POS_UNKNOWN
  TEST_ASSERT_EQUAL_INT(1, PB_REPORT_POS_UNKNOWN);
#else
  TEST_IGNORE_MESSAGE("going-live arm: [env:native_live] sets the flag to 0; see native");
#endif
}

/* The Arduino pin-mode call latches PODR = 0 and drives D6 LOW, discarding a level written
   before it. So: one PFS write carrying direction and level together, and no pin-mode call
   on D6 at all. */
static void test_boot_configures_d6_with_one_pfs_write_carrying_direction_and_level(void) {
  sim_events_clear();
  hal_boot_pump_off();
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  TEST_ASSERT_EQUAL_UINT32(1u, (uint32_t)n);
  TEST_ASSERT_EQUAL_INT(SIM_EV_PIN_CFG, (int)ev[0].kind);
  TEST_ASSERT_EQUAL_UINT8(6, ev[0].pin);
  TEST_ASSERT_TRUE(ev[0].arg & SIM_PFS_DIR_OUT);
  TEST_ASSERT_FALSE(ev[0].arg & SIM_PFS_LEVEL_HI);   /* the OFF level, in the same word */
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

/* R_IOPORT_PinCfg -> R_BSP_PinCfg is one unconditional `PmnPFS = cfg`, so every pump write
   restates the direction with the level: that is what lets the idle re-assert repair a
   stray pin-mode call on D6. */
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
  sim_wdt_stop();
  TEST_ASSERT_FALSE(hal_wdt_alive());
  TEST_ASSERT_EQUAL_UINT32(0u, hal_wdt_last_delta());
  sim_wdt_rate_hz(1000);                          /* moves, but 40 counts < 58 */
  TEST_ASSERT_FALSE(hal_wdt_alive());
}

/* A probe that fed inside its own window would never see a frozen counter. */
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
  sim_wdt_rate_hz(2929);                          /* PCLKB/8192 = 2929.7 Hz */
  TEST_ASSERT_TRUE(hal_wdt_alive());
  /* 41 ms of advance (the probe's own hal_millis() reads included) x 2.929 = 120 counts */
  TEST_ASSERT_TRUE(hal_wdt_last_delta() >= PB_WDT_PROBE_MIN_COUNTS);
  TEST_ASSERT_TRUE(hal_wdt_last_delta() <= 130u);
}

/* Idle re-asserts OFF every pass in the whole-word form, so it repairs a stray pin-mode
   call on D6 as well as a stray level; only then is the dog fed. */
static void test_idle_safety_tick_rewrites_the_off_level(void) {
  sim_events_clear();
  safety_tick();
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  TEST_ASSERT_EQUAL_UINT32(2u, (uint32_t)n);
  TEST_ASSERT_EQUAL_INT(SIM_EV_PUMP_WRITE, (int)ev[0].kind);
  TEST_ASSERT_TRUE(ev[0].arg & SIM_PFS_DIR_OUT);
  TEST_ASSERT_FALSE(ev[0].arg & SIM_PFS_LEVEL_HI);
  TEST_ASSERT_EQUAL_INT(SIM_EV_WDT_FEED, (int)ev[1].kind);   /* the feed comes SECOND */

  /* mid-dose the pump write is skipped but the feed is not: that is what makes a 60 s dose
     legal under a 5592 ms window */
  safety_set_dosing(true);
  sim_events_clear();
  safety_tick();
  n = sim_events(&ev);
  TEST_ASSERT_EQUAL_UINT32(1u, (uint32_t)n);
  TEST_ASSERT_EQUAL_INT(SIM_EV_WDT_FEED, (int)ev[0].kind);
  /* no reset of g_dosing here on purpose: teardown owns it, since a failed assertion above
     would longjmp past a reset written here; the pair below proves teardown does it */
}

/* Half of a pair run back-to-back (see main()): leaves g_dosing true and does not reset it,
   so if teardown stops resetting it the next case fails instead of the leak going quiet. */
static void test_g_dosing_leaks_here_if_teardown_does_not_reset_it(void) {
  safety_set_dosing(true);
}

/* The other half: a correct teardown reset leaves the idle tick emitting both events; a
   leaked g_dosing leaves only the feed. */
static void test_g_dosing_does_not_leak_between_cases(void) {
  sim_events_clear();
  safety_tick();
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  TEST_ASSERT_EQUAL_UINT32(2u, (uint32_t)n);
  TEST_ASSERT_EQUAL_INT(SIM_EV_PUMP_WRITE, (int)ev[0].kind);
  TEST_ASSERT_EQUAL_INT(SIM_EV_WDT_FEED, (int)ev[1].kind);
}

static void test_safety_wait_ms_feeds_on_every_iteration(void) {
  sim_events_clear();
  uint32_t t0 = hal_millis();
  safety_wait_ms(100);
  uint32_t t1 = hal_millis();
  TEST_ASSERT_TRUE(t1 - t0 >= 100u);

  const sim_ev_t *ev; size_t n = sim_events(&ev);
  uint32_t feeds = 0, prev = 0;
  bool have_prev = false;
  for (size_t i = 0; i < n; ++i) {
    if (ev[i].kind != SIM_EV_WDT_FEED) continue;
    if (have_prev) TEST_ASSERT_TRUE(ev[i].at_ms - prev <= 1u);   /* no gap wider than a tick */
    prev = ev[i].at_ms; have_prev = true; feeds++;
  }
  /* the fake advances 1 ms per hal_millis() call, so a 100 ms wait is 99 iterations */
  TEST_ASSERT_TRUE(feeds >= 99u);
}

static void test_a_cold_boot_zeroes_the_noinit_struct(void) {
  g_nv.dry_latched = true; g_nv.contra_latched = true; g_nv.cmd_high_water = 42u;
  noinit_commit();
  sim_reset(false);                        /* a power cycle: SRAM is cleared */
  TEST_ASSERT_TRUE(noinit_was_cold());
  TEST_ASSERT_FALSE(g_nv.dry_latched);
  TEST_ASSERT_FALSE(g_nv.contra_latched);
  TEST_ASSERT_EQUAL_UINT32(0u, g_nv.cmd_high_water);
  TEST_ASSERT_EQUAL_UINT32((uint32_t)PB_NOINIT_MAGIC, g_nv.magic);
  TEST_ASSERT_EQUAL_UINT32(1u, g_nv.boots);
}

/* The bootloader's own .data/.bss sit where __noinit_start does, so a partial clobber
   with the magic intact is a real case, and only the checksum catches it. */
static void test_a_bad_checksum_reads_as_a_cold_boot(void) {
  g_nv.dry_latched = true; g_nv.cmd_high_water = 7u;
  noinit_commit();
  sim_noinit_clobber();                    /* magic survives; the sum does not */
  TEST_ASSERT_EQUAL_UINT32((uint32_t)PB_NOINIT_MAGIC, g_nv.magic);
  sim_reset(true);                         /* a WARM reset: SRAM kept */
  TEST_ASSERT_TRUE(noinit_was_cold());
  TEST_ASSERT_FALSE(g_nv.dry_latched);
  TEST_ASSERT_EQUAL_UINT32(0u, g_nv.cmd_high_water);
}

static void test_a_warm_boot_restores_the_latches_and_the_high_water_mark(void) {
  g_nv.dry_latched = true; g_nv.contra_latched = true;
  g_nv.cmd_high_water = 65540u;            /* above 2^16: the ack id is a uint32 */
  g_nv.pattern = 0xDEADBEEFu;              /* the `noinit pattern` console word */
  noinit_commit();
  uint32_t before = g_nv.boots;
  sim_reset(true);
  TEST_ASSERT_FALSE(noinit_was_cold());
  TEST_ASSERT_TRUE(g_nv.dry_latched);
  TEST_ASSERT_TRUE(g_nv.contra_latched);
  TEST_ASSERT_EQUAL_UINT32(65540u, g_nv.cmd_high_water);
  TEST_ASSERT_EQUAL_UINT32(0xDEADBEEFu, g_nv.pattern);
  TEST_ASSERT_EQUAL_UINT32(before + 1u, g_nv.boots);
}

/* A reset with the pump asserted latches dry, and the flag stays set for setup() to turn
   into err=resetmid before clearing it. */
static void test_a_dose_in_flight_across_a_warm_boot_latches_dry(void) {
  g_nv.dry_latched = false; g_nv.dose_in_flight = true;
  noinit_commit();
  sim_reset(true);
  TEST_ASSERT_FALSE(noinit_was_cold());
  TEST_ASSERT_TRUE(g_nv.dry_latched);
  TEST_ASSERT_TRUE(g_nv.dose_in_flight);
}

/* The same condition, as the accessor setup() and safety_last_err() consume: without it
   `err=resetmid` has no producer. */
static void test_a_dose_in_flight_across_a_warm_boot_raises_resetmid(void) {
  g_nv.dose_in_flight = true;
  noinit_commit();
  sim_reset(true);
  TEST_ASSERT_TRUE(noinit_reset_mid());
  /* a COLD boot is not a reset mid-dose, whatever SRAM happened to hold */
  sim_reset(false);
  TEST_ASSERT_FALSE(noinit_reset_mid());
}

/* Without the salt a watchdog reset loop reports at t ~= 15000 every boot, and the backend
   discards each repeat as a retry of the same (controller, t). */
static void test_boot_salt_differs_across_two_warm_boots(void) {
  sim_reset(true); uint32_t a = hal_boot_salt();
  sim_reset(true); uint32_t b = hal_boot_salt();
  TEST_ASSERT_TRUE(a != b);
  TEST_ASSERT_TRUE(a != 0u && b != 0u);
  /* and it puts t above 2^31 on ordinary boots, which a signed %d would print negative */
  TEST_ASSERT_TRUE(a > 0x80000000u || b > 0x80000000u);
}

/* Multiply first: ml * cfg / 1000. Dividing first truncates the calibration to whole pulses
   per millilitre and under-delivers 15% at the nominal 5880. And cfg 0 must not divide by
   zero: the Cortex-M4's UDIV returns 0 without DIV_0_TRP, which would report a flood as
   nothing delivered. */
static void test_ml_from_pulses_rounds_down_and_does_not_overflow(void) {
  TEST_ASSERT_EQUAL_UINT32(100u, pulses_to_ml(588u, 5880u));
  TEST_ASSERT_EQUAL_UINT32(9u,   pulses_to_ml(58u, 5880u));    /* 9.86 ml, rounded DOWN */
  TEST_ASSERT_EQUAL_UINT32(250u, pulses_to_ml(1470u, 5880u));
  TEST_ASSERT_EQUAL_UINT32(0u,   pulses_to_ml(0u, 5880u));
  TEST_ASSERT_EQUAL_UINT32(0u,   pulses_to_ml(1000u, 0u));     /* never a UDIV-returns-0 lie */
  /* past UINT32_MAX/1000 the multiply-first form would wrap; the split form does not */
  TEST_ASSERT_EQUAL_UINT32(850340u,     pulses_to_ml(5000000u, 5880u));
  TEST_ASSERT_EQUAL_UINT32(2147483647u, pulses_to_ml(2147483647u, 1000u));
}

/* A float bouncing at the waterline satisfies one sample and fails three; on raw samples
   the board would report float=1, refuse the queued dose, and repeat every cooldown. */
static void test_three_consecutive_ok_samples_are_needed_to_grant(void) {
  pb_test_setup();
  sim_set_float_pattern("1101111");        /* one bad sample inside the first window */
  TEST_ASSERT_FALSE(safety_float_ok_debounced());
  pb_test_setup();
  sim_set_float_pattern("111");
  TEST_ASSERT_TRUE(safety_float_ok_debounced());
}

/* Refusing on one bad sample is safe, granting on one is not: D5 runs up to a metre to the
   reservoir beside a 12 V pump lead. */
static void test_one_bad_sample_refuses_immediately(void) {
  pb_test_setup();
  sim_set_float_pattern("0111111");
  uint32_t t0 = hal_millis();
  TEST_ASSERT_FALSE(safety_float_ok_debounced());
  /* it returned on the FIRST sample: no PB_FLOAT_SAMPLE_MS wait was paid */
  TEST_ASSERT_LESS_THAN_UINT32(PB_FLOAT_SAMPLE_MS, hal_millis() - t0);
}

/* The debounce waits 2 x PB_FLOAT_SAMPLE_MS, so at least two feeds. */
static void test_the_float_debounce_feeds_the_watchdog_between_samples(void) {
  pb_test_setup();
  sim_set_float(true);
  sim_events_clear();
  TEST_ASSERT_TRUE(safety_float_ok_debounced());
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  uint32_t feeds = 0, prev = 0; bool first = true;
  for (size_t i = 0; i < n; ++i) {
    if (ev[i].kind != SIM_EV_WDT_FEED) continue;
    if (!first) TEST_ASSERT_TRUE(ev[i].at_ms - prev <= 3u);
    prev = ev[i].at_ms; first = false; feeds++;
  }
  TEST_ASSERT_TRUE(feeds >= 2u);
}

/* Warm means watchdog or RESET button. A cold boot must clear it: a power cycle starts
   clean, and PB_BOOT_GAP_MS refuses for the first 10 s anyway. */
static void test_the_dry_latch_survives_a_warm_reset_and_not_a_cold_one(void) {
  pb_test_setup();
  TEST_ASSERT_FALSE(safety_dry());
  safety_dry_set(true);
  TEST_ASSERT_TRUE(safety_dry());

  /* sim_reset() re-enters the boot path, so noinit_begin() has already run: calling it
     again would advance the boot counter, and the salt, twice per reset. */
  sim_reset(true);                     /* warm: SRAM intact, .noinit verifies */
  TEST_ASSERT_TRUE_MESSAGE(safety_dry(), "the latch did not survive a warm reset");

  sim_reset(false);                    /* cold: SRAM cleared, magic mismatches */
  TEST_ASSERT_FALSE_MESSAGE(safety_dry(), "the latch survived a COLD boot");
}

/* The report's debounce and the dose's are separate samples minutes apart, so a float
   flapping at the waterline can grant in one and refuse in the other; three refusals in a
   row trips the flap: the report forces float=0 and err=float, and the backend's rules go
   dark. The final count(false) is an assertion --
   cleared by any granted dose -- not cleanup; teardown owns the reset (pair below). */
static void test_the_flap_counter_trips_after_three_consecutive_float_refusals(void) {
  pb_test_setup();
  TEST_ASSERT_FALSE(safety_float_flap());
  safety_float_refusal_count(true);
  safety_float_refusal_count(true);
  TEST_ASSERT_FALSE(safety_float_flap());          /* two is not yet a pattern */
  safety_float_refusal_count(true);
  TEST_ASSERT_TRUE(safety_float_flap());           /* the third trips it */
  safety_float_refusal_count(false);               /* a GRANTED dose clears it */
  TEST_ASSERT_FALSE(safety_float_flap());
}

/* Half of a pair run back-to-back (see main()), like the g_dosing pair: g_float_refusals is
   process-lifetime state in safety.cpp, left tripped here on purpose. */
static void test_g_float_refusals_leaks_here_if_teardown_does_not_reset_it(void) {
  safety_float_refusal_count(true);
  safety_float_refusal_count(true);
  safety_float_refusal_count(true);        /* trips the flap; left dirty on purpose */
}

/* The other half: reads true only if teardown stopped clearing g_float_refusals. */
static void test_g_float_refusals_does_not_leak_between_cases(void) {
  TEST_ASSERT_FALSE(safety_float_flap());
}

/* Takes `unsigned`, not dose_result_t: the exit-path loop counts over the enum as unsigned
   so DOSE_RESULT_COUNT is a legal bound, and passes the raw loop variable here. */
static const char *pb_result_name(unsigned rv) {
  switch ((dose_result_t)rv) {
    case DOSE_OK:               return "DOSE_OK";
    case DOSE_REFUSED_WDT:      return "DOSE_REFUSED_WDT";
    case DOSE_REFUSED_DRY:      return "DOSE_REFUSED_DRY";
    case DOSE_REFUSED_CONTRA:   return "DOSE_REFUSED_CONTRA";
    case DOSE_REFUSED_BOOT:     return "DOSE_REFUSED_BOOT";
    case DOSE_REFUSED_RANGE:    return "DOSE_REFUSED_RANGE";
    case DOSE_REFUSED_CAL:      return "DOSE_REFUSED_CAL";
    case DOSE_REFUSED_FLOAT:    return "DOSE_REFUSED_FLOAT";
    case DOSE_REFUSED_POS:      return "DOSE_REFUSED_POS";
    case DOSE_REFUSED_I2C:      return "DOSE_REFUSED_I2C";
    case DOSE_REFUSED_BUSY:     return "DOSE_REFUSED_BUSY";
    case DOSE_REFUSED_COOLDOWN: return "DOSE_REFUSED_COOLDOWN";
    case DOSE_REFUSED_NOISE:    return "DOSE_REFUSED_NOISE";
    case DOSE_ABORT_CAP:        return "DOSE_ABORT_CAP";
    case DOSE_ABORT_NOFLOW:     return "DOSE_ABORT_NOFLOW";
    case DOSE_ABORT_NOISE:      return "DOSE_ABORT_NOISE";
    case DOSE_ABORT_FLOAT:      return "DOSE_ABORT_FLOAT";
    case DOSE_ABORT_POS:        return "DOSE_ABORT_POS";
    case DOSE_ABORT_STOP:       return "DOSE_ABORT_STOP";
    case DOSE_RESULT_COUNT:     break;
  }
  return "?";
}

/* Arranges the one condition each result needs, runs the dose, and returns true; returns
   false without running anything for a result this build cannot reach. No `default:` arm,
   so -Wall -Wextra reports an added enum value as a missing case rather than a vacuous pass.

   Every arm whose dose reaches the pump loop advances the clock past PB_BOOT_GAP_MS first:
   the cooldown rung reads g_last_end_ms, a safety.cpp static that pb_test_setup() does not
   reset, and a dose ending above 10 s of this iteration's clock keeps every later
   iteration's `hal_millis() - g_last_end_ms` a huge wrapped difference, never a small one. */
static bool pb_drive_dose_to_result(dose_result_t want) {
  switch (want) {
    case DOSE_OK: {
      pb_advance(PB_BOOT_GAP_MS + 1u);
      pulses_begin();
      (void)sensors_begin();
      sim_set_float(true);
      sim_set_flow_ml_s(85u);
      dose_req_t q = {0};
      q.ml = (uint16_t)PB_DOSE_RIG_MAX_ML;
      q.cap_ms = PB_DOSE_CAP_MS_MAX;
      q.long_prime = true;
      (void)dose_run(&q);
      return true;
    }
    case DOSE_REFUSED_WDT: {
      sim_wdt_stop();                                   /* the counter FREEZES */
      dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
      (void)dose_run(&q);
      return true;
    }
    case DOSE_REFUSED_DRY: {
      safety_dry_set(true);
      dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
      (void)dose_run(&q);
      return true;
    }
    case DOSE_REFUSED_CONTRA: {
      pb_latch_contra();
      pb_advance(PB_DOSE_MIN_GAP_MS + 1u);
      dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
      (void)dose_run(&q);
      return true;
    }
    case DOSE_REFUSED_BOOT: {
      /* the clock is still at 0 -- the ABSENCE of an advance is the arrangement */
      dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
      (void)dose_run(&q);
      return true;
    }
    case DOSE_REFUSED_RANGE: {
      pb_advance(PB_BOOT_GAP_MS + 1u);
      dose_req_t q = {0};
      q.ml = (uint16_t)(PB_DOSE_RIG_MAX_ML + 1u);       /* inside the protocol, outside the rig */
      q.cap_ms = 10000u;
      (void)dose_run(&q);
      return true;
    }
    case DOSE_REFUSED_CAL: {
      pb_advance(PB_BOOT_GAP_MS + 1u);
      safety_force_bad_cal_();
      dose_req_t q = {0}; q.ml = 100u; q.cap_ms = 10000u;
      (void)dose_run(&q);
      (void)cfg_pulses_per_l_set(PB_PULSES_PER_L_DEFAULT);  /* put it back for later arms */
      return true;
    }
    case DOSE_REFUSED_FLOAT: {
      pb_advance(PB_BOOT_GAP_MS + 1u);
      pulses_begin();
      (void)sensors_begin();
      sim_set_float(false);
      dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
      (void)dose_run(&q);
      return true;
    }
    case DOSE_REFUSED_POS: {
      pb_advance(PB_BOOT_GAP_MS + 1u);
      pulses_begin();
      (void)sensors_begin();
      (void)cart_begin();                                /* position UNKNOWN, every build */
      dose_req_t q = {0};
      q.outlet = 1u; q.ml = 100u; q.cap_ms = 10000u; q.need_pos = true;
      (void)dose_run(&q);
      return true;
    }
    case DOSE_REFUSED_I2C: {
      pb_advance(PB_BOOT_GAP_MS + 1u);
      pulses_begin();
      (void)sensors_begin();
      sim_set_i2c_fail(true);
      for (uint8_t i = 0; i < PB_I2C_FAIL_LIMIT; ++i) (void)sensors_select(0u);
      dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
      (void)dose_run(&q);
      sim_set_i2c_fail(false);
      (void)sensors_begin();                             /* leave the bus healthy behind us */
      return true;
    }
    case DOSE_REFUSED_BUSY: {
      dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
      safety_set_dosing(true);       /* never touched by a refusal this early -- reset it below */
      (void)dose_run(&q);
      safety_set_dosing(false);
      return true;
    }
    case DOSE_REFUSED_COOLDOWN: {
      pb_advance(PB_BOOT_GAP_MS + 1u);
      pulses_begin();
      (void)sensors_begin();
      dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
      (void)dose_run(&q);                                /* ends on its cap, stamps g_last_end_ms */
      (void)dose_run(&q);                                /* immediately again: cooldown */
      return true;
    }
    case DOSE_REFUSED_NOISE: {
      pb_advance(PB_BOOT_GAP_MS + 1u);
      pulses_begin();
      (void)sensors_begin();
      sim_flow_storm(100u);                              /* D2 counting with the pump OFF */
      pb_advance(500u);
      dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
      (void)dose_run(&q);
      sim_flow_storm(0u);                                /* quiet for whatever runs next */
      return true;
    }
    case DOSE_ABORT_CAP: {
      pb_advance(PB_BOOT_GAP_MS + 1u);
      pulses_begin();
      (void)sensors_begin();
      sim_set_flow_ml_s(0u);                             /* the pump runs; nothing moves */
      dose_req_t q = {0}; q.ml = 100u; q.cap_ms = 1000u;
      (void)dose_run(&q);
      return true;
    }
    case DOSE_ABORT_NOFLOW: {
      pb_advance(PB_BOOT_GAP_MS + 1u);
      pulses_begin();
      (void)sensors_begin();
      sim_set_float(true);
      sim_set_flow_ml_s(0u);                             /* the pump runs; nothing ever moves */
      dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
      (void)dose_run(&q);
      return true;
    }
    case DOSE_ABORT_NOISE: {
      pb_advance(PB_BOOT_GAP_MS + 1u);
      pulses_begin();
      (void)sensors_begin();
      sim_set_float(true);
      sim_flow_storm_at_pump_on(2000u);       /* a storm that begins WITH the pump */
      dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
      (void)dose_run(&q);
      return true;
    }
    case DOSE_ABORT_FLOAT: {
      pb_advance(PB_BOOT_GAP_MS + 1u);
      pulses_begin();
      (void)sensors_begin();
      sim_set_float(true);
      sim_set_flow_ml_s(30u);
      sim_set_float_at_ms(500u, false);       /* drops mid-dose, well inside the prime window */
      dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
      (void)dose_run(&q);
      return true;
    }
    case DOSE_ABORT_POS: {
      pb_advance(PB_BOOT_GAP_MS + 1u);
      pulses_begin();
      (void)sensors_begin();
      sim_set_float(true);
      sim_set_flow_ml_s(30u);
      sim_set_i2c_fail(true);                 /* bites inside the loop: the ladder reads the
                                                  cached healthy flag, so this dose starts */
      dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
      (void)dose_run(&q);
      sim_set_i2c_fail(false);
      (void)sensors_begin();                  /* leave the bus healthy for whatever runs next */
      return true;
    }
    case DOSE_ABORT_STOP: {
      pb_advance(PB_BOOT_GAP_MS + 1u);
      pulses_begin();
      (void)sensors_begin();
      sim_serial_rx("stop\n");        /* sits in the fake's UART ring; the entry-time
                                          cli_stop_clear() does not touch that ring */
      dose_req_t q = {0}; q.by_time = true; q.cap_ms = 5000u;
      (void)dose_run(&q);
      return true;
    }
    case DOSE_RESULT_COUNT:
      return false;   /* the sentinel is never a result to drive */
  }
  return false;
}

/* Nineteen results, nineteen exits, D6 OFF at every one. The loop runs against
   DOSE_RESULT_COUNT, so a result added without a way to reach it fails here. */
void test_pump_is_off_on_every_exit_path(void) {
  char skipped[256] = {0};
  unsigned driven = 0;
  for (unsigned r = 0; r < (unsigned)DOSE_RESULT_COUNT; ++r) {
    pb_test_setup();
    if (!pb_drive_dose_to_result((dose_result_t)r)) {  /* not reachable in THIS build */
      if (skipped[0]) strncat(skipped, ", ", sizeof skipped - strlen(skipped) - 1);
      strncat(skipped, pb_result_name(r), sizeof skipped - strlen(skipped) - 1);
      continue;                     /* NEVER TEST_IGNORE in this loop -- see below */
    }
    ++driven;
    TEST_ASSERT_EQUAL_MESSAGE((int)r, (int)dose_last_result(), pb_result_name(r));
    TEST_ASSERT_FALSE_MESSAGE(sim_pump_is_on(),  pb_result_name(r));
    TEST_ASSERT_FALSE_MESSAGE(safety_dosing(),   pb_result_name(r));
  }
  /* Asserted, so a build that quietly stops driving an arm fails HERE. */
  TEST_ASSERT_EQUAL_UINT_MESSAGE(PB_DRIVABLE_RESULTS, driven, skipped);
  if (skipped[0]) TEST_MESSAGE(skipped);   /* after the loop, and MESSAGE, not IGNORE */
}

/* Ordering pair, half 1: cal above range. */
void test_the_ladder_reports_the_more_specific_reason(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true);
  safety_force_bad_cal_();                          /* see below */
  dose_req_t r = {0}; r.ml = 9999u; r.cap_ms = 1000u;
  dose_result_t got = dose_run(&r);
  /* g_pulses_per_l is a safety.cpp static pb_test_setup() cannot reach: put it back before
     the assertion, or a failure here leaves every later case refusing with cal */
  (void)cfg_pulses_per_l_set(PB_PULSES_PER_L_DEFAULT);
  TEST_ASSERT_EQUAL(DOSE_REFUSED_CAL, got);               /* cal above range */
}

/* Ordering pair, half 2. An operator reading err=dry when the real reason was the
   contradiction latch pulls the tank apart looking for water that is already there. */
void test_the_ladder_reports_contra_above_dry(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pb_latch_contra();
  safety_dry_set(true);                             /* BOTH latches stand */
  pb_advance(PB_DOSE_MIN_GAP_MS + 1u);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL(DOSE_REFUSED_CONTRA, dose_run(&q));   /* contra above dry */
  TEST_ASSERT_EQUAL_STRING("contra", safety_last_err());
}

/* Seen from a dose that ran, not from the exit helpers directly. */
void test_refusal_reports_zero_millilitres_not_the_previous_dose(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();          /* own tumbling window: the storm case leaves ~99 Hz behind */
  sim_set_float(true); sim_set_flow_ml_s(30);
  /* long_prime with a 6000 ms cap: PB_PRIME_CAP_MS (20000) leaves the cap alone, and the
     dose delivers real millilitres for the refusal below to not ack */
  dose_req_t ok = {0}; ok.by_time = true; ok.cap_ms = 6000u; ok.long_prime = true;
  (void)dose_run(&ok);
  TEST_ASSERT_TRUE(dose_flow_ml() > 0u);
  pb_advance(PB_DOSE_MIN_GAP_MS + 1u);
  /* pulses_flow_rate()'s window only advances when called, and the ladder calls it once per
     dose; left unrebased since the first dose, it would divide that dose's whole flow by
     the whole gap and read ~30 Hz, refusing NOISE instead of FLOAT. On the board loop()
     rebases it every pass; a synchronous test has to do so by hand. */
  pulses_begin();
  sim_set_float(false);
  TEST_ASSERT_EQUAL(DOSE_REFUSED_FLOAT, dose_run(&ok));
  TEST_ASSERT_EQUAL_UINT16(0u, dose_flow_ml());
}

/* First rung after busy: a counter that is not moving is a dog that will never bite, and
   every bound below it is then the only thing between a stuck loop and a running pump. */
void test_dose_refused_when_the_watchdog_counter_is_not_moving(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_wdt_stop();                                 /* the counter FREEZES */
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_WDT, dose_run(&q),
      "a frozen watchdog counter must refuse with wdt");
}

/* Nothing is latched above it here, so `dry` is the token, not `contra`. */
void test_dose_refused_when_the_dry_latch_is_set(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  safety_dry_set(true);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_DRY, dose_run(&q),
      "the dry latch must refuse with dry");
}

/* A power cycle clears the dry latch, and PB_BOOT_GAP_MS stands in for it for ten
   seconds. The fake's clock starts at zero and the watchdog probe advances it ~41 ms, so
   the absence of an advance is the arrangement. */
void test_dose_refused_inside_the_boot_gap(void) {
  pb_test_setup();                                /* the clock is at zero: a fresh boot */
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_BOOT, dose_run(&q),
      "a dose inside PB_BOOT_GAP_MS must be refused with boot");
}

/* For every caller, console and backend alike. The first dose's 1000 ms cap is under
   PB_PRIME_MS_DEFAULT, so it ends on its cap, arms no prime rule and latches nothing; the
   second follows at once and only the cooldown stands between them. */
void test_dose_refused_inside_the_minimum_gap_since_the_last_dose(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  (void)dose_run(&q);                             /* ends on its cap and stamps g_last_end_ms */
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_COOLDOWN, dose_run(&q),
      "a second dose inside PB_DOSE_MIN_GAP_MS must be refused with cooldown");
}

/* D5 reads LOW for OK, so HIGH is a tank at the waterline or a broken wire: the same
   answer by design. */
void test_dose_refused_when_the_float_reads_not_ok(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(false);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_FLOAT, dose_run(&q),
      "a float reading not-OK must refuse with float");
}

/* The debounce asymmetry seen from the dose: PB_FLOAT_OK_SAMPLES consecutive OK readings
   grant, one bad sample in the window refuses. */
void test_dose_refused_when_a_single_float_sample_is_bad(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float_pattern("1101111");               /* OK, OK, BAD, then OK forever */
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_FLOAT, dose_run(&q),
      "one bad sample inside the debounce window must refuse with float");
}

/* A need_pos dose with the cart position unknown would dead-head the pump against a closed
   manifold. The obvious arrangement proves nothing: cart_begin() leaves g_pos at 0 with
   q.outlet 1, so the rung's second line (known but wrong outlet) refuses even with the first
   deleted. The calibrated arm therefore loses the position while leaving cart_pos() equal
   to the outlet -- which is the field case too, since a failed cart_goto() clears the
   validity but not g_pos. */
void test_dose_refused_when_position_is_unknown(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  /* Well clear of the dose cooldown on both arms: the calibrated arm burns ~5400 ms of
     fake clock before the dose (1450 pulses at 2 ms to gate 1, then a 2500 ms stall
     window). */
  pb_advance(4u * PB_DOSE_MIN_GAP_MS);
#if PB_PULSES_PER_GATE == 0
  (void)cart_begin();                    /* cart_pos_known() is compiled to false here, so
                                            only the first line can answer */
  dose_req_t q = {0};
  q.outlet = 1u; q.ml = 100u; q.cap_ms = 10000u; q.need_pos = true;
#else
  (void)cart_begin();
  sim_set_screw_pulse_ms(2u); sim_set_home_region(0u, 40u); sim_set_cart_at(0u);
  TEST_ASSERT_TRUE(cart_goto(1u));       /* position known, and equal to 1 */
  sim_set_stall(true);
  (void)cart_goto(2u);                   /* fails: pos_valid false, cart_pos() still 1 */
  sim_set_stall(false);
  TEST_ASSERT_FALSE(cart_pos_known());
  TEST_ASSERT_EQUAL_UINT(1u, cart_pos());
  dose_req_t q = {0};
  q.outlet = 1u; q.ml = 100u; q.cap_ms = 10000u; q.need_pos = true;   /* outlet == pos */
#endif
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_POS, dose_run(&q),
      "a need_pos dose with an unknown cart position must be refused with pos");
}

/* The rung's second line: the position is known and wrong. Only the calibrated arm can
   reach it, since cart_pos_known() is compiled to false while PB_PULSES_PER_GATE is 0.
   PB_PULSES_HOME_TO_1 is 0 there, so cart_goto(1) lands from home without turning the
   screw, costing ~3 ms of fake clock and leaving the cooldown read where every other
   case's is. */
void test_dose_refused_when_the_cart_is_at_another_outlet(void) {
#if PB_PULSES_PER_GATE == 0
  TEST_IGNORE_MESSAGE("uncalibrated arm: cart_pos_known() is compiled out to false (spec 2.15), "
                      "so the ladder's second position line cannot be reached; native_cal runs it");
#else
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  (void)cart_begin();
  sim_set_home_region(0u, 40u);
  sim_set_cart_at(0u);                            /* already home, so cart_home() lands at once */
  TEST_ASSERT_TRUE_MESSAGE(cart_goto(1u), "arrange: the cart must be KNOWN at outlet 1");
  dose_req_t q = {0};
  q.outlet = 2u; q.ml = 100u; q.cap_ms = 10000u; q.need_pos = true;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_POS, dose_run(&q),
      "a dose for outlet 2 with the cart standing at outlet 1 must be refused with pos");
#endif
}

/* The I2C bus carries the mux select lines and the home hall, so an unhealthy bus is a rig
   that cannot say where its cart is; PB_I2C_FAIL_LIMIT consecutive failed transfers is
   unhealthy. sensors.cpp's counters are statics, so the bus goes back before the
   assertion, on the failing path too. */
void test_dose_refused_when_i2c_is_unhealthy(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  (void)sensors_begin();
  sim_set_i2c_fail(true);
  for (uint8_t i = 0; i < PB_I2C_FAIL_LIMIT; ++i) (void)sensors_select(0u);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  dose_result_t r = dose_run(&q);
  sim_set_i2c_fail(false);
  (void)sensors_begin();                          /* leave the bus healthy for the next case */
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_I2C, r,
      "an unhealthy I2C bus must refuse with i2c");
}

/* The protocol ceiling is PB_DOSE_MAX_ML (1000); the rig ceiling is PB_DOSE_RIG_MAX_ML
   (250), a reservoir whose full dump is a mop-up. The backend knows only the first, so
   251 ml is a legal command on the wire and this rung is the only thing that refuses it. */
void test_dose_refused_when_ml_exceeds_the_rig_ceiling(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  dose_req_t q = {0};
  q.ml = (uint16_t)(PB_DOSE_RIG_MAX_ML + 1u);     /* 251: inside the protocol, outside the rig */
  q.cap_ms = 10000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_RANGE, dose_run(&q),
      "a millilitre target above PB_DOSE_RIG_MAX_ML must be refused with range");
}

/* A zero cap_ms is a request with no bound at all, refused before the ml and need_pos
   checks that follow it. */
void test_dose_refused_when_the_cap_is_zero(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 0u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_RANGE, dose_run(&q),
      "a zero cap_ms must be refused with range");
}

/* Outlet is never a sentinel: water=0 is a legal backend command that the backend's
   `outlet is None` guard does not catch, so 0 arrives from the wire and must be refused. */
void test_dose_refused_when_a_need_pos_dose_names_outlet_zero(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  dose_req_t q = {0};
  q.outlet = 0u; q.ml = 100u; q.cap_ms = 10000u; q.need_pos = true;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_RANGE, dose_run(&q),
      "outlet 0 with need_pos must be refused with range, not treated as a sentinel");
}

/* Pulses with D6 OFF are not water: a floating D2 would otherwise reach target in
   milliseconds. 100 Hz is an order under PB_FLOW_MAX_HZ (1200) and two over
   PB_FLOW_IDLE_MAX_HZ (2), so it is the idle ceiling that answers, not the in-dose one;
   the 500 ms of storm after pulses_begin() fills the window the ladder reads. */
void test_dose_refused_when_the_idle_pulse_rate_is_nonzero(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_flow_storm(100u);                           /* D2 counting with the pump OFF */
  pb_advance(500u);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  dose_result_t r = dose_run(&q);
  sim_flow_storm(0u);                             /* and the meter is quiet for the next case */
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_NOISE, r,
      "a non-zero idle pulse rate must refuse with noise, before D6 is ever asserted");
}

/* With target == 0 the loop's `target && got >= target` can never fire, so the only exit
   left would be the cap: D6 asserted for the whole cap for a request of no water. The
   `target == 0` guard below the caps is belt and braces -- ml >= 1 and cfg >= 1000 make the
   smallest legal product 1 pulse -- so the reachable zero target is ml == 0, which the
   backend can send, and the range rung is what answers it. */
void test_metered_dose_with_a_zero_target_is_refused_not_run_to_cap(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  dose_req_t q = {0};
  q.ml = 0u; q.by_time = false;                   /* metered, and asking for nothing */
  q.cap_ms = PB_DOSE_CAP_MS_MAX;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_RANGE, dose_run(&q),
      "a metered dose with a zero target must be refused with range, not run to its cap");
}

/* DOSE_OK is reachable through one line only, `target && got >= target`, so the result is
   the assertion. target = 250 * 5880 / 1000 = 1470 pulses; the fake delivers
   85 ml/s * 5880 / 1000 = 499 pulses/s, first pulse ~1 ms after the ON write, so the dose
   measures ~3279 ms: inside PB_PRIME_CAP_MS (20000) under long_prime, inside the measured
   clamp's 250 * 1000 / 30 * 2 = 16666 ms, and inside the 3000 ms default window too.
   long_prime is kept for margin, not need. */
void test_dose_stops_at_the_millilitre_target(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_flow_ml_s(85u);
  dose_req_t q = {0};
  q.ml = (uint16_t)PB_DOSE_RIG_MAX_ML;            /* 250 ml at the default calibration */
  q.cap_ms = PB_DOSE_CAP_MS_MAX;
  q.long_prime = true;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_OK, dose_run(&q),
      "a metered dose that reaches its pulse target must end DOSE_OK, not on its cap");
}

/* When the meter never reaches the target the cap ends the dose, and the cap is a bound on
   time, never a second target. target = 100 * 5880 / 1000 = 588 pulses, delivered 0; cap
   the typed 1000 ms (the measured clamp computes 100 * 1000 / 30 * 2 = 6666 and leaves it
   alone). Under PB_PRIME_MS_DEFAULT, so the prime and stall rules never arm and nothing
   latches. */
void test_dose_stops_at_the_cap_when_flow_never_reaches_target(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_flow_ml_s(0u);                          /* the pump runs; nothing moves */
  dose_req_t q = {0}; q.ml = 100u; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_CAP, dose_run(&q),
      "a metered dose that never reaches its target must end on its cap");
}

/* The cap bounds how long D6 is asserted, not how long the call takes: sim_pump_on_ms()
   counts milliseconds with the pin high. Twenty ms of slack is the loop's granularity --
   the fake advances 1 ms per clock read, and the OFF write is the statement after the
   break. */
void test_pump_on_time_never_exceeds_the_cap(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_flow_ml_s(0u);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 2000u;  /* under PB_PRIME_MS_DEFAULT */
  (void)dose_run(&q);
  TEST_ASSERT_TRUE_MESSAGE(sim_pump_on_ms() <= 2000u + 20u,
      "D6 was asserted for longer than cap_ms");
}

/* The case above bounds D6 by the caller's cap; this one bounds the cap itself. Every
   other cap_ms in this file is <= PB_DOSE_CAP_MS_MAX, so the clamp line in safety.cpp could
   be deleted with everything else green -- and nothing upstream narrows a cap: the parser
   lets cap_s ride to 65535 and exec multiplies by 1000, so that line is all that stands
   between a hostile cap_s and a 65,535-second run. by_time, so only the cap can end this;
   real flow, so neither prime nor stall fires first; sim_pump_on_ms() measures the pin. */
void test_a_cap_over_the_firmware_ceiling_is_clamped_to_the_ceiling(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_flow_ml_s(30u);                    /* flowing: prime and stall can never fire */
  dose_req_t q = {0};
  q.by_time = true;                          /* no target: only the cap can end this */
  q.cap_ms  = PB_DOSE_CAP_MS_MAX + 5000u;    /* 65 s, over the ceiling, from a cap_s of 65 */
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_CAP, dose_run(&q),
      "a flowing by-time dose can only end on its cap");
  TEST_ASSERT_TRUE_MESSAGE(dose_last_ms() >= PB_DOSE_CAP_MS_MAX,
      "the dose must actually reach the ceiling, or this case proves nothing");
  TEST_ASSERT_TRUE_MESSAGE(dose_last_ms() < PB_DOSE_CAP_MS_MAX + 1000u,
      "the cap was NOT clamped: the dose ran on past the firmware's hard maximum");
  TEST_ASSERT_TRUE_MESSAGE(sim_pump_on_ms() <= PB_DOSE_CAP_MS_MAX + 20u,
      "D6 was asserted past the firmware's hard maximum run time");
}

/* Divide-first would truncate the calibration to whole pulses per millilitre. Runs at the
   cal floor, an ugly value, the nominal and the ceiling, within one pulse of the
   multiply-first arithmetic: at most one edge lands per 1 ms tick (the fake's period at
   85 ml/s is ~2 ms).

   ml is 150 so one request works in all three native environments: under native_measured
   the cap is also clamped to twice the requested water, and at cfg 20000 the 3000-pulse
   target takes ~9012 ms of fake flow, inside 150 ml's 10000 ms clamp.

   Under native_measured the plausibility rule also bounds which (cfg, flow) pairs can
   honestly reach DOSE_OK. The fake emits flow_ml_s * 5880 / 1000 Hz whatever cfg it is
   told, so elapsed = ml*cfg*1000/(flow*5880) against a floor of ml*8.33 ms at
   PB_ML_PER_S_MEASURED 30; ml cancels, leaving cfg >= flow * 49, so at 85 ml/s cfg must be
   >= 4165. The floor and the ugly value fail that honestly -- a calibration that far under
   the sensor's rating would make 85 ml/s read as hundreds -- so the skip below is the rule
   working, not a test bug; native proves the arithmetic at all four points. */
void test_target_pulses_match_the_calibration_within_one_pulse(void) {
  const uint16_t cfgs[] = { 1000u, 1999u, 5880u, 20000u };   /* floor, ugly, nominal, ceiling */
  for (size_t i = 0; i < sizeof cfgs / sizeof cfgs[0]; ++i) {
#if PB_ML_PER_S_MEASURED > 0
    if (cfgs[i] < 4200u) continue;   /* see the comment above: not reachable honestly here */
#endif
    pb_test_setup();
    pb_advance(PB_BOOT_GAP_MS + 1u);
    pulses_begin();
    TEST_ASSERT_TRUE(cfg_pulses_per_l_set(cfgs[i]));
    sim_set_float(true);
    sim_set_flow_ml_s(85u);
    dose_req_t q = {0};
    q.ml = 150u; q.cap_ms = PB_DOSE_CAP_MS_MAX; q.long_prime = true;
    dose_result_t r = dose_run(&q);
    TEST_ASSERT_EQUAL_MESSAGE(DOSE_OK, r,
        "the dose must reach its pulse target, not its cap, at every calibration");
    uint32_t want = (uint32_t)q.ml * (uint32_t)cfgs[i] / 1000u;   /* MULTIPLY FIRST */
    uint32_t got  = dose_last_pulses();
    TEST_ASSERT_TRUE_MESSAGE(got >= want && got <= want + 1u,
        "delivered pulses must be within one pulse of ml * cfg / 1000");
  }
  (void)cfg_pulses_per_l_set(PB_PULSES_PER_L_DEFAULT);   /* put it back for later cases */
}

/* Every bound in the loop is an unsigned difference, and the cap must straddle the wrap or
   this case is vacuous: 0xFFFFF000 is 4095 ms below UINT32_MAX, so the 1000-2000 ms caps
   the rest of the file uses would finish before it; 6000 ms under long_prime wraps ~4095
   in. dose_last_ms() reads past cap_ms by design: g_last_end_ms is a fresh clock read made
   after the break, and pulses_flow_rate() reads the clock a second time every pass, so the
   overshoot is +2 or +3 by parity -- a fact about the fake's bookkeeping. What must hold
   is that the cap ends the dose within a handful of ms of the typed bound. */
void test_dose_cap_holds_across_a_millis_rollover(void) {
  pb_test_setup();
  sim_set_clock_ms(0xFFFFF000u);
  pulses_begin();                                 /* rebase the tumbling window at the jump */
  sim_set_float(true);
  sim_set_flow_ml_s(0u);                           /* the CAP, not the meter, must end this */
  dose_req_t q = {0};
  q.by_time = true; q.cap_ms = 6000u; q.long_prime = true;
  dose_result_t r = dose_run(&q);
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_CAP, r, "the cap must still fire across the wrap");
  TEST_ASSERT_TRUE_MESSAGE(dose_last_ms() > 6000u && dose_last_ms() <= 6000u + 5u,
      "dose_last_ms() must land within a few ticks of the cap, on the far side of the wrap");
}

/* Bring-up runs before the cart is calibrated, so a console `pump` that demanded a known
   position would be unrunnable: need_pos = false must never reach either position rung. */
void test_console_pump_does_not_require_a_known_position(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  (void)cart_begin();                              /* position UNKNOWN, and stays that way */
  sim_set_float(true);
  sim_set_flow_ml_s(0u);
  TEST_ASSERT_FALSE(cart_pos_known());
  dose_req_t q = {0};
  q.by_time = true; q.cap_ms = 1000u; q.need_pos = false;   /* console pump: no position needed */
  dose_result_t r = dose_run(&q);
  TEST_ASSERT_FALSE_MESSAGE(cart_pos_known(),
      "the dose must not have touched cart position at all");
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_CAP, r,
      "need_pos=false must never refuse DOSE_REFUSED_POS regardless of cart state");
}

/* `status` is a dispatched command, so its absence from the console's output after the
   dose is what proves the discard. Bytes typed while the console looks frozen sit in the
   fake's raw UART ring, are consumed by cli_stop_requested() inside the loop (matching
   neither `stop` nor `dry on`) and reconstructed into cli.cpp's own pushback buffer; the
   closing cli_stop_clear() throws that away and hal_serial_drain() discards the ring.
   Either alone leaves something to replay after the dose. */
void test_bytes_buffered_during_a_dose_are_discarded_not_executed(void) {
  char tx[512];

  /* Half 1: hal_serial_drain() alone. A 1 ms cap fires `el >= cap_ms` on the first
     iteration -- t0 is read one clock tick before the first `now` -- above the
     cli_stop_requested() check, so "status\n" sits unread in the raw ring for the whole
     dose and only the drain can clear it. */
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(0u);
  sim_serial_rx("status\n");
  dose_req_t short_q = {0}; short_q.by_time = true; short_q.cap_ms = 1u;
  (void)dose_run(&short_q);
  (void)sim_serial_tx(tx, sizeof tx);     /* drain whatever the dose itself may have printed */
  cli_poll();
  size_t n = sim_serial_tx(tx, sizeof tx);
  TEST_ASSERT_TRUE_MESSAGE(n == 0u || strstr(tx, "granted=") == NULL,
      "bytes never reached by cli_stop_requested() during a very short dose must still be "
      "discarded, by hal_serial_drain()");

  /* Half 2: cli_stop_clear() alone. A longer cap lets the loop's own cli_stop_requested()
     read "status\n" from the raw ring and reconstruct it into cli.cpp's pushback, a
     separate buffer the drain cannot see; only cli_stop_clear()'s discard of that pushback
     stops read_console_() from replaying it after the dose. */
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(0u);
  sim_serial_rx("status\n");             /* impatience typed while the console looks frozen */
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  (void)dose_run(&q);
  (void)sim_serial_tx(tx, sizeof tx);     /* drain whatever the dose itself may have printed */
  cli_poll();                             /* the buffered "status" must NOT reach cli_dispatch() */
  n = sim_serial_tx(tx, sizeof tx);
  TEST_ASSERT_TRUE_MESSAGE(n == 0u || strstr(tx, "granted=") == NULL,
      "the matcher's reconstructed pushback must not be replayed into cli_poll() after the "
      "dose, by cli_stop_clear()");
  TEST_ASSERT_FALSE(sim_pump_is_on());
}

/* Needs PB_ML_PER_S_MEASURED > 0, which native_measured alone defines; the other arm only
   proves the clamp is compiled out. */
void test_cap_is_clamped_to_twice_the_requested_millilitres(void) {
#if PB_ML_PER_S_MEASURED > 0
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(0u);                  /* nothing must arrive: the CLAMP ends this dose */
  dose_req_t q = {0};
  q.ml = 200u; q.cap_ms = PB_DOSE_CAP_MS_MAX; q.long_prime = true;
  dose_result_t r = dose_run(&q);
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_CAP, r,
      "the measured clamp, not the 60 s/20 s ceiling above it, must end this dose");
  uint32_t want = (uint32_t)q.ml * 1000u / (uint32_t)PB_ML_PER_S_MEASURED
                  * PB_CAP_SLACK_NUM / PB_CAP_SLACK_DEN;
  TEST_ASSERT_EQUAL_UINT32(13332u, want);            /* derived, then checked against itself */
  /* dose_last_ms() lands a few ticks past `want`, never exactly on it: g_last_end_ms is a
     fresh clock read after the loop breaks, and the loop reads the clock twice per pass
     (see the rollover case). */
  TEST_ASSERT_TRUE_MESSAGE(dose_last_ms() > want && dose_last_ms() <= want + 5u,
      "dose_last_ms() must land within a few ticks of the measured clamp");
#else
  TEST_IGNORE_MESSAGE("PB_ML_PER_S_MEASURED == 0: the measured cap clamp is compiled out; "
                       "native_measured runs this case");
#endif
}

/* The counter has two call sites: dose_end_() increments only on DOSE_REFUSED_FLOAT and
   dose_end_ml_() resets unconditionally. The other flap cases drive the setter and bypass
   the ladder, so a guard widened to count a non-float refusal would pass them all. Here a
   priming dose resets the counter and stamps g_last_end_ms, the next call inside the gap is
   a genuine cooldown refusal, and after two further float refusals the flap must still read
   false -- which is where a widened guard breaks first. */
void test_the_flap_counter_is_driven_by_dose_run_not_the_setter(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(0u);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;

  /* Prime: resets the counter through dose_end_ml_() and stamps g_last_end_ms fresh. */
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_CAP, dose_run(&q), "arrange: the priming dose");
  TEST_ASSERT_FALSE(safety_float_flap());

  /* The interleaved non-float refusal, still inside PB_DOSE_MIN_GAP_MS: correct code
     leaves the counter at 0. */
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_COOLDOWN, dose_run(&q),
      "arrange: a genuine cooldown refusal, still inside the gap");
  TEST_ASSERT_FALSE(safety_float_flap());

  /* past the gap, so what follows reaches the float rung */
  pb_advance(PB_DOSE_MIN_GAP_MS + 1u);
  sim_set_float(false);

  TEST_ASSERT_EQUAL(DOSE_REFUSED_FLOAT, dose_run(&q));
  TEST_ASSERT_FALSE_MESSAGE(safety_float_flap(), "one genuine float refusal must not trip it");

  TEST_ASSERT_EQUAL(DOSE_REFUSED_FLOAT, dose_run(&q));
  TEST_ASSERT_FALSE_MESSAGE(safety_float_flap(),
      "two genuine float refusals must not trip it EITHER -- if the interleaved cooldown "
      "above had counted, the total would already be three here");

  TEST_ASSERT_EQUAL(DOSE_REFUSED_FLOAT, dose_run(&q));
  TEST_ASSERT_TRUE_MESSAGE(safety_float_flap(), "the third GENUINE float refusal must trip it");
}

/* The single-rung dry and boot cases both pass with the two rungs swapped; only a case
   that latches dry inside the boot gap can tell the orderings apart. dry is the actionable
   fact: err=boot says wait and retry, err=dry sends the operator to `dry off`. */
void test_the_ladder_reports_dry_above_boot(void) {
  pb_test_setup();                                /* clock at zero: inside the boot gap */
  safety_dry_set(true);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_DRY, dose_run(&q),
      "the dry latch must be reported even while still inside the boot gap");
}

/* The second half of the counter's contract: dose_end_ml_()'s reset is unconditional on
   every path that reaches it, not only DOSE_OK. Made conditional, stale float refusals
   would accumulate across unrelated doses and trip the flap for no reason a status line
   could point to. Two float refusals, an ABORT_CAP dose that ran, then one more float
   refusal: with the reset skipped that third would be the count's third. */
void test_the_flap_counter_is_cleared_by_dose_run_on_any_granted_path(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(0u);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;

  /* Baseline: a granted-reaching dose; it also stamps g_last_end_ms in this case's own
     clock domain, which the cooldown arithmetic below relies on. */
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_CAP, dose_run(&q), "arrange: the baseline dose");
  TEST_ASSERT_FALSE(safety_float_flap());
  pb_advance(PB_DOSE_MIN_GAP_MS + 1u);      /* past the gap, so the doses below reach their
                                                own rungs rather than refusing on cooldown */

  /* Two genuine float refusals: count = 2, one short of the limit. */
  sim_set_float(false);
  TEST_ASSERT_EQUAL(DOSE_REFUSED_FLOAT, dose_run(&q));
  TEST_ASSERT_EQUAL(DOSE_REFUSED_FLOAT, dose_run(&q));
  TEST_ASSERT_FALSE(safety_float_flap());

  /* The dose under test reaches dose_end_ml_() on a non-DOSE_OK path. */
  sim_set_float(true);
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_CAP, dose_run(&q),
      "arrange: a granted dose that is NOT DOSE_OK");
  TEST_ASSERT_FALSE(safety_float_flap());

  /* Past the gap again, then one more float refusal: a stale count of 2 would trip here. */
  pb_advance(PB_DOSE_MIN_GAP_MS + 1u);
  sim_set_float(false);
  TEST_ASSERT_EQUAL(DOSE_REFUSED_FLOAT, dose_run(&q));
  TEST_ASSERT_FALSE_MESSAGE(safety_float_flap(),
      "the counter must have been cleared by the ABORT_CAP dose above, not only by DOSE_OK");
}

/* Half of a pair run back-to-back (see main()), like the two pairs above: g_last_end_ms is
   process-lifetime state in safety.cpp, left non-zero here on purpose. Leaked, the next
   case's dose reads "a dose ended moments ago" and refuses cooldown instead of ABORT_CAP. */
void test_g_last_end_ms_leaks_here_if_teardown_does_not_reset_it(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(0u);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  (void)dose_run(&q);            /* reaches the loop; stamps g_last_end_ms, left dirty on
                                     purpose */
}

/* The other half. Advances to 2 x PB_BOOT_GAP_MS, not the usual PB_BOOT_GAP_MS + 1: a
   leaked g_last_end_ms sits near 11 s in the previous case's clock, so the usual advance
   would land below it, wrap the unsigned difference huge and miss the leak; twice the gap
   lands above it but inside its ten-second cooldown, the one zone that tells a leak from a
   reset. */
void test_g_last_end_ms_does_not_leak_between_cases(void) {
  pb_advance(2u * PB_BOOT_GAP_MS);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(0u);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_CAP, dose_run(&q),
      "a leaked g_last_end_ms from the previous case would read as a fresh dose that ended "
      "moments ago and refuse this one with cooldown instead");
}

/* With the target rule first, a D2 at the ISR's own 2 kHz ceiling reaches a 250 ml target
   in ~625 ms and the dose returns DOSE_OK with flow_ml=250 for water that never moved. */
void test_the_rate_rules_are_evaluated_above_the_target_rule(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true);
  TEST_ASSERT_TRUE(cfg_pulses_per_l_set(5000u));
  sim_flow_storm_at_pump_on(2000u);
  dose_req_t q = {0}; q.ml = 250u; q.cap_ms = PB_DOSE_CAP_MS_MAX; q.need_pos = false;
  TEST_ASSERT_EQUAL(DOSE_ABORT_NOISE, dose_run(&q));
  TEST_ASSERT_NOT_EQUAL(DOSE_OK, dose_last_result());
  (void)cfg_pulses_per_l_set(PB_PULSES_PER_L_DEFAULT);   /* put it back for later cases */
}

/* The same storm, stated as the consequence: no target is ever reached by noise. */
void test_a_storm_that_begins_AT_PUMP_ON_aborts_before_the_target_is_reached(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true);
  TEST_ASSERT_TRUE(cfg_pulses_per_l_set(5000u));
  TEST_ASSERT_EQUAL_UINT32(0u, pulses_flow_rate());   /* NOT storming before the dose: the
                                                         idle guard must not be what fires */
  sim_flow_storm_at_pump_on(2000u);
  dose_req_t q = {0}; q.ml = 250u; q.cap_ms = PB_DOSE_CAP_MS_MAX;
  TEST_ASSERT_EQUAL(DOSE_ABORT_NOISE, dose_run(&q));
  TEST_ASSERT_EQUAL_UINT16(0u, dose_flow_ml() > 250u ? 1u : 0u);   /* nothing was acked */
  (void)cfg_pulses_per_l_set(PB_PULSES_PER_L_DEFAULT);   /* put it back for later cases */
}

/* The two cases above prove the storm aborts, not the order: their 1250-pulse target needs
   ~625 ms of a 2000 Hz storm, six 100 ms estimator windows after the storm is first
   visible, so either order exits on the rate ceiling long before the target. This fixture
   lands the target inside the iteration the first window closes, so both conditions turn
   true at once and only the order decides. 45 ml (225 pulses at cfg 5000) was found
   empirically: the window closes at el ~115 ms with 227 pulses; 44 reaches its 220 at
   ~112 ms before the window closes and neither order catches it; 46 is reached after the
   window has already broken the loop. Moving the rate-ceiling check below the target flips
   this fixture to DOSE_OK. */
void test_the_rate_ceiling_alone_wins_the_race_against_the_target(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  TEST_ASSERT_TRUE(cfg_pulses_per_l_set(5000u));
  sim_flow_storm_at_pump_on(2000u);
  dose_req_t q = {0}; q.ml = 45u; q.cap_ms = PB_DOSE_CAP_MS_MAX;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_NOISE, dose_run(&q),
      "the rate ceiling must win the race against the target rule, not lose it");
  (void)cfg_pulses_per_l_set(PB_PULSES_PER_L_DEFAULT);   /* put it back for later cases */
}

/* The no-flow abort, half 1: a line that never primes. The pump runs the whole default
   window and nothing comes out. */
void test_prime_abort_fires_when_nothing_flows_in_the_prime_window(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true); sim_set_flow_ml_s(0);         /* the pump runs; nothing moves */
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
  TEST_ASSERT_EQUAL(DOSE_ABORT_NOFLOW, dose_run(&q));
  TEST_ASSERT_TRUE(sim_pump_on_ms() < PB_PRIME_MS_DEFAULT + 200u);
}

/* `prime` extends the window; a dose that never flows still aborts, at PB_PRIME_LONG_MS
   rather than never. */
void test_prime_flag_still_aborts_when_nothing_ever_flows(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true); sim_set_flow_ml_s(0);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 60000u; q.long_prime = true;
  TEST_ASSERT_EQUAL(DOSE_ABORT_NOFLOW, dose_run(&q));
  TEST_ASSERT_TRUE(sim_pump_on_ms() >= PB_PRIME_LONG_MS);        /* the window extended */
  TEST_ASSERT_TRUE(sim_pump_on_ms() <  PB_PRIME_CAP_MS + 500u);  /* and it still ended */
}

void test_prime_flag_caps_the_dose_at_the_prime_cap(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true); sim_set_flow_ml_s(30);        /* flowing, so no no-flow abort */
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 60000u; q.long_prime = true;
  TEST_ASSERT_EQUAL(DOSE_ABORT_CAP, dose_run(&q));
  TEST_ASSERT_TRUE(sim_pump_on_ms() <= PB_PRIME_CAP_MS + 200u);  /* NOT the typed 60 s */
}

/* A dose that delivered a few pulses and then stopped must still abort: arming on `got`
   would let zero flow disarm the rule entirely. */
void test_stall_abort_is_armed_on_time_not_on_pulses(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_burst_pulses(PB_PRIME_MIN_PULSES + 2u);   /* then nothing, forever */
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
  TEST_ASSERT_EQUAL(DOSE_ABORT_NOFLOW, dose_run(&q));
  TEST_ASSERT_TRUE(sim_pump_on_ms() <
                   PB_PRIME_MS_DEFAULT + PB_STALL_MS_DEFAULT + 500u);
}

/* The prime rule's boundary, from the other side: exactly one pulse short of the
   threshold must NOT be read as "flow started". */
void test_five_spurious_edges_at_start_do_not_disable_the_abort(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_burst_pulses(PB_PRIME_MIN_PULSES - 1u);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
  TEST_ASSERT_EQUAL(DOSE_ABORT_NOFLOW, dose_run(&q));
}

/* A healthy dose on the untouched default prime window, no long_prime: 100 ml at 85 ml/s
   takes a few hundred milliseconds, well inside the 3 s window. */
void test_a_healthy_metered_dose_completes_on_the_default_prime_window(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(85u);
  dose_req_t q = {0};
  q.ml = 100u; q.cap_ms = PB_DOSE_CAP_MS_MAX;   /* NOT long_prime: the default window */
  dose_result_t r = dose_run(&q);
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_OK, r,
      "a healthy dose on the default prime window must complete, not abort noflow");
  TEST_ASSERT_TRUE(dose_last_pulses() >= (uint32_t)PB_PRIME_MIN_PULSES);
  TEST_ASSERT_TRUE_MESSAGE(dose_last_ms() < PB_PRIME_MS_DEFAULT,
      "a healthy dose must reach its target well inside the prime window, with room to spare");
}

/* The rate ceiling against the YF-S401's rating: above 1200 Hz is not a fast pump but a
   meter not measuring water. The 100 ms estimator window is what brings the verdict inside
   a second. */
void test_dose_aborts_when_the_pulse_rate_exceeds_the_meter_rating(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_flow_storm_at_pump_on((uint32_t)PB_FLOW_MAX_HZ + 200u);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
  TEST_ASSERT_EQUAL(DOSE_ABORT_NOISE, dose_run(&q));
  TEST_ASSERT_TRUE(sim_pump_on_ms() < 1000u);
}

/* The plausibility rule on the DOSE_OK path: at native_measured's 30 ml/s a 120 ml dose
   honestly needs 4000 ms; this one delivers it in under 900 ms, clearing the floor
   (ml*1000/(PLAUS_NUM*30) = 1000 ms) while its ~882 Hz stays under PB_FLOW_MAX_HZ (1200),
   so only this rule can fire. 882 Hz is a rate a real meter can produce; the elapsed time
   against the volume is what is impossible. */
void test_a_dose_that_reaches_target_implausibly_fast_is_noise_not_ok(void) {
#if PB_ML_PER_S_MEASURED > 0
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(150u);      /* ~882 pulses/s: under PB_FLOW_MAX_HZ, so rule 1 is silent */
  dose_req_t q = {0};
  q.ml = 120u; q.cap_ms = PB_DOSE_CAP_MS_MAX;
  dose_result_t r = dose_run(&q);
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_NOISE, r,
      "reaching the target in well under a quarter of the honest 4 s must be noise, not ok");
  TEST_ASSERT_NOT_EQUAL(DOSE_OK, dose_last_result());
#else
  TEST_IGNORE_MESSAGE("PB_ML_PER_S_MEASURED == 0: the rule is compiled out");
#endif
}

/* Pulling the float mid-dose stops it within one loop iteration, and must not latch the
   contradiction: the float dropping is the two sensors agreeing (real flow, tank ran dry). */
void test_dose_stops_within_one_iteration_when_the_float_drops(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(30u);
  sim_set_float_at_ms(500u, false);      /* drops mid-dose, well inside the prime window */
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
  dose_result_t r = dose_run(&q);
  TEST_ASSERT_EQUAL(DOSE_ABORT_FLOAT, r);
  TEST_ASSERT_TRUE_MESSAGE(sim_pump_on_ms() < 500u + 10u,
      "the float drop must stop the dose within about one loop iteration");
  TEST_ASSERT_FALSE_MESSAGE(safety_contra(),
      "the float and the meter agreeing (flow, then dry) must never latch the contradiction");
}

/* The bus wedges after the dose has started: sim_set_i2c_fail(true) does not fail the
   cached healthy flag (that takes PB_I2C_FAIL_LIMIT failed transfers), so the ladder grants
   and the live cart_bus_check() inside the loop, at most once per PB_POS_RECHECK_MS, is
   what finds the bus gone. Recovery refuses while dosing, so it stays broken. */
void test_dose_aborts_when_the_expander_read_fails_mid_dose(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  (void)sensors_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(30u);
  sim_set_i2c_fail(true);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
  dose_result_t r = dose_run(&q);
  TEST_ASSERT_EQUAL(DOSE_ABORT_POS, r);
  TEST_ASSERT_TRUE_MESSAGE(sim_pump_on_ms() <= (uint32_t)PB_POS_RECHECK_MS + 10u,
      "the bus failure must be caught within PB_POS_RECHECK_MS plus one iteration");
  sim_set_i2c_fail(false);
  (void)sensors_begin();                  /* leave the bus healthy for whatever runs next */
}

/* The last-resort abort, driven by real bytes through sim_serial_rx() while the loop spins,
   never by poking cli.cpp's flag. */
void test_stop_typed_mid_dose_stops_it_within_one_iteration(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(0u);          /* nothing must arrive: `stop` alone must end this */
  sim_serial_rx_at_ms(500u, "stop\n");
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
  dose_result_t r = dose_run(&q);
  TEST_ASSERT_EQUAL(DOSE_ABORT_STOP, r);
  TEST_ASSERT_TRUE_MESSAGE(sim_pump_on_ms() < 500u + 10u,
      "a typed stop must end the dose within about one loop iteration");
}

/* `dry on` mid-dose both aborts the dose, through the same matcher `stop` uses, and
   latches the dry flag in the same pass. */
void test_dry_on_typed_mid_dose_stops_it(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(0u);
  sim_serial_rx_at_ms(500u, "dry on\n");
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
  dose_result_t r = dose_run(&q);
  TEST_ASSERT_EQUAL(DOSE_ABORT_STOP, r);
  TEST_ASSERT_TRUE_MESSAGE(safety_dry(), "`dry on` typed mid-dose must latch the dry flag");
  safety_dry_set(false);            /* leave it clean for whatever runs next */
}

/* What makes a 60 s dose legal under a 5592 ms grant, and the first assertion to fail if a
   `continue` is ever added to the loop body. Anchored on the ON write: the ladder's own
   watchdog probe feeds twice around a deliberate unfed window before the loop starts, so
   scanning from the first feed after the pump's ON write is what isolates the loop. */
void test_watchdog_is_fed_on_every_iteration_of_the_dose_loop(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  (void)sensors_begin();
  sim_set_float(true); sim_set_flow_ml_s(30);
  sim_events_clear();
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 5000u;
  (void)dose_run(&q);
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  size_t loop_start = n;
  for (size_t i = 0; i < n; ++i)
    if (ev[i].kind == SIM_EV_PUMP_WRITE && (ev[i].arg & SIM_PFS_LEVEL_HI)) loop_start = i;
  TEST_ASSERT_TRUE_MESSAGE(loop_start < n, "arrange: the dose must have asserted D6");
  uint32_t prev = 0, feeds = 0; bool first = true;
  for (size_t i = loop_start; i < n; ++i) {
    if (ev[i].kind != SIM_EV_WDT_FEED) continue;
    /* pulses_flow_rate() reads the clock a second time every pass, so an iteration costs
       two fake ticks; 3 ms is that plus one tick of slack. */
    if (!first) TEST_ASSERT_TRUE_MESSAGE(ev[i].at_ms - prev <= 3u, "unfed span in the dose loop");
    prev = ev[i].at_ms; first = false; feeds++;
  }
  TEST_ASSERT_TRUE(feeds > 100u);
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
  RUN_TEST(test_idle_safety_tick_rewrites_the_off_level);
  /* This pair MUST run back-to-back, in this order: the guarantee under test is that
     pb_test_teardown() resets g_dosing between them. */
  RUN_TEST(test_g_dosing_leaks_here_if_teardown_does_not_reset_it);
  RUN_TEST(test_g_dosing_does_not_leak_between_cases);
  RUN_TEST(test_safety_wait_ms_feeds_on_every_iteration);
  RUN_TEST(test_a_cold_boot_zeroes_the_noinit_struct);
  RUN_TEST(test_a_bad_checksum_reads_as_a_cold_boot);
  RUN_TEST(test_a_warm_boot_restores_the_latches_and_the_high_water_mark);
  RUN_TEST(test_a_dose_in_flight_across_a_warm_boot_latches_dry);
  RUN_TEST(test_a_dose_in_flight_across_a_warm_boot_raises_resetmid);
  RUN_TEST(test_boot_salt_differs_across_two_warm_boots);
  RUN_TEST(test_ml_from_pulses_rounds_down_and_does_not_overflow);
  RUN_TEST(test_three_consecutive_ok_samples_are_needed_to_grant);
  RUN_TEST(test_one_bad_sample_refuses_immediately);
  RUN_TEST(test_the_float_debounce_feeds_the_watchdog_between_samples);
  RUN_TEST(test_the_dry_latch_survives_a_warm_reset_and_not_a_cold_one);
  RUN_TEST(test_the_flap_counter_trips_after_three_consecutive_float_refusals);
  /* This pair MUST run back-to-back, in this order: the guarantee under test is that
     pb_test_teardown() resets g_float_refusals between them. */
  RUN_TEST(test_g_float_refusals_leaks_here_if_teardown_does_not_reset_it);
  RUN_TEST(test_g_float_refusals_does_not_leak_between_cases);
  RUN_TEST(test_dose_refused_when_the_watchdog_counter_is_not_moving);
  RUN_TEST(test_dose_refused_when_the_dry_latch_is_set);
  RUN_TEST(test_dose_refused_inside_the_boot_gap);
  RUN_TEST(test_dose_refused_inside_the_minimum_gap_since_the_last_dose);
  RUN_TEST(test_dose_refused_when_the_float_reads_not_ok);
  RUN_TEST(test_dose_refused_when_a_single_float_sample_is_bad);
  RUN_TEST(test_dose_refused_when_position_is_unknown);
  RUN_TEST(test_dose_refused_when_the_cart_is_at_another_outlet);
  RUN_TEST(test_dose_refused_when_i2c_is_unhealthy);
  RUN_TEST(test_dose_refused_when_ml_exceeds_the_rig_ceiling);
  RUN_TEST(test_dose_refused_when_the_cap_is_zero);
  RUN_TEST(test_dose_refused_when_a_need_pos_dose_names_outlet_zero);
  RUN_TEST(test_dose_refused_when_the_idle_pulse_rate_is_nonzero);
  RUN_TEST(test_the_rate_rules_are_evaluated_above_the_target_rule);
  RUN_TEST(test_a_storm_that_begins_AT_PUMP_ON_aborts_before_the_target_is_reached);
  RUN_TEST(test_the_rate_ceiling_alone_wins_the_race_against_the_target);
  RUN_TEST(test_metered_dose_with_a_zero_target_is_refused_not_run_to_cap);
  RUN_TEST(test_dose_stops_at_the_millilitre_target);
  RUN_TEST(test_dose_stops_at_the_cap_when_flow_never_reaches_target);
  RUN_TEST(test_pump_on_time_never_exceeds_the_cap);
  RUN_TEST(test_a_cap_over_the_firmware_ceiling_is_clamped_to_the_ceiling);
  RUN_TEST(test_prime_abort_fires_when_nothing_flows_in_the_prime_window);
  RUN_TEST(test_prime_flag_still_aborts_when_nothing_ever_flows);
  RUN_TEST(test_prime_flag_caps_the_dose_at_the_prime_cap);
  RUN_TEST(test_stall_abort_is_armed_on_time_not_on_pulses);
  RUN_TEST(test_five_spurious_edges_at_start_do_not_disable_the_abort);
  RUN_TEST(test_a_healthy_metered_dose_completes_on_the_default_prime_window);
  RUN_TEST(test_dose_aborts_when_the_pulse_rate_exceeds_the_meter_rating);
  RUN_TEST(test_a_dose_that_reaches_target_implausibly_fast_is_noise_not_ok);
  RUN_TEST(test_dose_stops_within_one_iteration_when_the_float_drops);
  RUN_TEST(test_dose_aborts_when_the_expander_read_fails_mid_dose);
  RUN_TEST(test_stop_typed_mid_dose_stops_it_within_one_iteration);
  RUN_TEST(test_dry_on_typed_mid_dose_stops_it);
  RUN_TEST(test_watchdog_is_fed_on_every_iteration_of_the_dose_loop);
  RUN_TEST(test_pump_is_off_on_every_exit_path);
  RUN_TEST(test_the_ladder_reports_the_more_specific_reason);
  RUN_TEST(test_the_ladder_reports_contra_above_dry);
  RUN_TEST(test_refusal_reports_zero_millilitres_not_the_previous_dose);
  RUN_TEST(test_target_pulses_match_the_calibration_within_one_pulse);
  RUN_TEST(test_console_pump_does_not_require_a_known_position);
  RUN_TEST(test_bytes_buffered_during_a_dose_are_discarded_not_executed);
  RUN_TEST(test_cap_is_clamped_to_twice_the_requested_millilitres);
  RUN_TEST(test_the_flap_counter_is_driven_by_dose_run_not_the_setter);
  RUN_TEST(test_the_ladder_reports_dry_above_boot);
  RUN_TEST(test_the_flap_counter_is_cleared_by_dose_run_on_any_granted_path);
  /* This pair MUST run back-to-back, in this order: the guarantee under test is that
     pb_test_teardown() resets g_last_end_ms between them. */
  RUN_TEST(test_g_last_end_ms_leaks_here_if_teardown_does_not_reset_it);
  RUN_TEST(test_g_last_end_ms_does_not_leak_between_cases);
  /* LAST: it leaves g_last_end_ms at a small wrapped value that would poison every later
     case's cooldown arithmetic; teardown resets it too, but this is the belt to those
     braces. */
  RUN_TEST(test_dose_cap_holds_across_a_millis_rollover);
  return UNITY_END();
}
