/* test/test_dose/test_dose.cpp
   The dose suite. Today it carries one case: that a native Unity runner links and runs.
   Tasks 3-6 and 15-19 fill it with the pump-pin, watchdog and refusal-ladder cases. */
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

/* Which arms of dose_result_t this BUILD can actually drive dose_run() to, counted rather
   than guessed. 19 here, after task 19: DOSE_REFUSED_CONTRA is now reachable through
   pb_latch_contra() (harness.h) -- the only way any test may earn the latch -- followed
   by an ordinary request. All nineteen results are driven; the switch has no ignored
   arm left. */
#define PB_DRIVABLE_RESULTS 19

static void test_the_native_runner_links_and_runs(void) {
  /* PB_CONTROLLER comes from [env:native]'s build_flags, so this also proves the flag
     reached the compiler. */
  TEST_ASSERT_EQUAL_UINT(7u, (unsigned)PB_CONTROLLER);
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
  TEST_ASSERT_TRUE(PB_CONTROLLER >= 0 && PB_CONTROLLER <= 255);
  TEST_ASSERT_TRUE(PB_CONTROLLER_WIRE + 2u + PB_BODY_WORST_FIXED <= PB_BODY_CAP);
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

/* §2.4: idle ACTIVELY re-asserts OFF, every pass, using the whole-word form of §2.1 —
   so the re-assert repairs a stray pinMode on D6 as well as a stray level. Then, and
   only then, the dog is fed. */
static void test_idle_safety_tick_rewrites_the_off_level(void) {
  sim_events_clear();
  safety_tick();
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  TEST_ASSERT_EQUAL_UINT32(2u, (uint32_t)n);
  TEST_ASSERT_EQUAL_INT(SIM_EV_PUMP_WRITE, (int)ev[0].kind);
  TEST_ASSERT_TRUE(ev[0].arg & SIM_PFS_DIR_OUT);      /* direction restated */
  TEST_ASSERT_FALSE(ev[0].arg & SIM_PFS_LEVEL_HI);    /* at the OFF level */
  TEST_ASSERT_EQUAL_INT(SIM_EV_WDT_FEED, (int)ev[1].kind);   /* and the feed comes SECOND */

  /* mid-dose the pump write is skipped — but the feed is not, which is what makes a
     60 s dose legal under a 5592 ms window (§3). */
  safety_set_dosing(true);
  sim_events_clear();
  safety_tick();
  n = sim_events(&ev);
  TEST_ASSERT_EQUAL_UINT32(1u, (uint32_t)n);
  TEST_ASSERT_EQUAL_INT(SIM_EV_WDT_FEED, (int)ev[0].kind);
  /* No safety_set_dosing(false) here on purpose: pb_test_teardown() is what has to
     guarantee g_dosing == false for the next case, not this line. If a TEST_ASSERT_*
     above this point had failed, Unity's longjmp would have skipped a reset written here —
     see the pair of cases below, which exists to catch exactly that regression. */
}

/* This case and the next are a deliberate pair, run back-to-back in that order (see
   main()): this one leaves g_dosing == true and does NOT reset it. If
   pb_test_teardown() ever stops resetting it, the NEXT case's assertion fails instead
   of the leak going quiet — proving the guarantee rather than assuming it. */
static void test_g_dosing_leaks_here_if_teardown_does_not_reset_it(void) {
  safety_set_dosing(true);
}

/* The other half of the pair above. If pb_test_teardown() correctly reset g_dosing to
   false after the previous case, an idle safety_tick() here still re-asserts the pump
   OFF write (two events); if the reset regressed, g_dosing is still true here and
   safety_tick() emits only the feed (one event) — the exact failure mode the finding
   describes, made to fail loudly instead of going quiet. */
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
  sim_reset(false);                        /* a power cycle: SRAM is cleared (§2.3) */
  TEST_ASSERT_TRUE(noinit_was_cold());
  TEST_ASSERT_FALSE(g_nv.dry_latched);
  TEST_ASSERT_FALSE(g_nv.contra_latched);
  TEST_ASSERT_EQUAL_UINT32(0u, g_nv.cmd_high_water);
  TEST_ASSERT_EQUAL_UINT32((uint32_t)PB_NOINIT_MAGIC, g_nv.magic);
  TEST_ASSERT_EQUAL_UINT32(1u, g_nv.boots);
}

/* The checksum is what keeps a PARTIAL clobber from reading as a valid latch — the
   bootloader's own .data/.bss sit exactly where __noinit_start does (§2.3). */
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
  g_nv.cmd_high_water = 65540u;            /* above 2^16: the ack id is a uint32 (§4.3) */
  g_nv.pattern = 0xDEADBEEFu;              /* bring-up 7c's `noinit pattern` word */
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

/* §2.3: a reset with the pump asserted is the single loudest thing this rig can discover
   about itself. It latches dry, and the flag stays set for setup() to turn into
   err=resetmid before clearing it. */
static void test_a_dose_in_flight_across_a_warm_boot_latches_dry(void) {
  g_nv.dry_latched = false; g_nv.dose_in_flight = true;
  noinit_commit();
  sim_reset(true);
  TEST_ASSERT_FALSE(noinit_was_cold());
  TEST_ASSERT_TRUE(g_nv.dry_latched);
  TEST_ASSERT_TRUE(g_nv.dose_in_flight);
}

/* The same condition, as the fact setup() and safety_last_err() actually consume. Without
   this accessor `err=resetmid` has no producer anywhere and bring-up 7c's pass criterion
   (`status` says dry=1 and last=resetmid) is unreachable. */
static void test_a_dose_in_flight_across_a_warm_boot_raises_resetmid(void) {
  g_nv.dose_in_flight = true;
  noinit_commit();
  sim_reset(true);
  TEST_ASSERT_TRUE(noinit_reset_mid());
  /* a COLD boot is not a reset mid-dose, whatever SRAM happened to hold */
  sim_reset(false);
  TEST_ASSERT_FALSE(noinit_reset_mid());
}

/* §15.2: without the salt, a watchdog reset loop reports at t ~= 15000 every boot, and
   butler silently discards each repeat as a retry of the same (controller, t). */
static void test_boot_salt_differs_across_two_warm_boots(void) {
  sim_reset(true); uint32_t a = hal_boot_salt();
  sim_reset(true); uint32_t b = hal_boot_salt();
  TEST_ASSERT_TRUE(a != b);
  TEST_ASSERT_TRUE(a != 0u && b != 0u);
  /* and it puts t above 2^31 on ordinary boots, which is why §9 greps for %d */
  TEST_ASSERT_TRUE(a > 0x80000000u || b > 0x80000000u);
}

/* §6: target is ml * cfg / 1000 — MULTIPLY FIRST. The reverse order truncates the
   calibration to whole pulses per millilitre and under-delivers 15% at the nominal 5880.
   And `cal 0` used to make pulses_to_ml divide by zero: the Cortex-M4's UDIV returns 0
   without DIV_0_TRP, so the flood happened and the report said nothing came out. */
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

/* §2.10. Three consecutive OK samples to GRANT. The float bouncing at the waterline
   satisfies one sample and fails three; a raw-sample implementation reports float=1, the
   backend queues, the board refuses, the acked refusal pages HIGH and sets the pot's
   cooldown - and repeats every cooldown period, forever. */
static void test_three_consecutive_ok_samples_are_needed_to_grant(void) {
  pb_test_setup();
  sim_set_float_pattern("1101111");        /* one bad sample inside the first window */
  TEST_ASSERT_FALSE(safety_float_ok_debounced());
  pb_test_setup();
  sim_set_float_pattern("111");
  TEST_ASSERT_TRUE(safety_float_ok_debounced());
}

/* The asymmetry is deliberate: refusing on one bad sample is safe, GRANTING on one is
   not, because D5 runs up to a metre to the reservoir beside a 12 V pump lead. */
static void test_one_bad_sample_refuses_immediately(void) {
  pb_test_setup();
  sim_set_float_pattern("0111111");
  uint32_t t0 = hal_millis();
  TEST_ASSERT_FALSE(safety_float_ok_debounced());
  /* it returned on the FIRST sample: no PB_FLOAT_SAMPLE_MS wait was paid */
  TEST_ASSERT_LESS_THAN_UINT32(PB_FLOAT_SAMPLE_MS, hal_millis() - t0);
}

/* Every loop that can iterate over a millisecond of wall clock feeds the dog (§3). This
   one waits 2 x PB_FLOAT_SAMPLE_MS and is called from dose_run()'s refusal ladder. */
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

/* §2.11. `dry on` survives a WARM reset - watchdog or RESET button - because a brown-out
   at pump start (the wiring README warns of 3-5x inrush on a sagging brick) used to clear
   it silently while the operator's hands were in the plumbing. It does NOT survive a cold
   boot, and it must not: a power cycle starts clean and PB_BOOT_GAP_MS refuses for the
   first 10 s anyway. */
static void test_the_dry_latch_survives_a_warm_reset_and_not_a_cold_one(void) {
  pb_test_setup();
  TEST_ASSERT_FALSE(safety_dry());
  safety_dry_set(true);
  TEST_ASSERT_TRUE(safety_dry());

  /* sim_reset() re-enters the boot path (task 3), so noinit_begin() has already run by
     the time it returns -- do not call it a second time or the boot counter, and with it
     the salt, advances twice per reset. */
  sim_reset(true);                     /* warm: SRAM intact, .noinit verifies */
  TEST_ASSERT_TRUE_MESSAGE(safety_dry(), "the latch did not survive a warm reset");

  sim_reset(false);                    /* cold: SRAM cleared, magic mismatches */
  TEST_ASSERT_FALSE_MESSAGE(safety_dry(), "the latch survived a COLD boot");
}

/* §2.10's second consequence. The report's debounce and the dose's debounce are separate
   samples taken minutes apart, so a float flapping at the waterline can grant in the
   report and refuse in the dose. Above the limit the report forces float=0 and err=float
   regardless of the report-time debounce (task 22), and butler's rules ladder goes dark.

   The final safety_float_refusal_count(false) below is a real assertion under test --
   it proves "cleared by any GRANTED dose", not merely a courtesy reset -- so it stays.
   It is not what keeps the counter clean for the NEXT case, though: pb_test_teardown()
   is (task 15 fix round 1), for the identical reason g_dosing is reset there rather than
   inline. See the proof pair immediately below this case. */
static void test_the_flap_counter_trips_after_three_consecutive_float_refusals(void) {
  pb_test_setup();
  TEST_ASSERT_FALSE(safety_float_flap());
  safety_float_refusal_count(true);
  safety_float_refusal_count(true);
  TEST_ASSERT_FALSE(safety_float_flap());          /* two is not yet a pattern */
  safety_float_refusal_count(true);
  TEST_ASSERT_TRUE(safety_float_flap());           /* the third trips it */
  safety_float_refusal_count(false);               /* a GRANTED dose clears it -- asserted, */
  TEST_ASSERT_FALSE(safety_float_flap());           /* not merely relied on for cleanup */
}

/* This case and the next are a deliberate pair, run back-to-back in that order (see
   main()) -- the same shape as the g_dosing pair above, guarding the same class of bug
   (task 15 fix round 1): g_float_refusals is process-lifetime state in safety.cpp, and
   this case leaves it dirty (flap tripped) and does NOT reset it inline. If
   pb_test_teardown() ever stops resetting it, the NEXT case's assertion fails instead of
   the leak going quiet. */
static void test_g_float_refusals_leaks_here_if_teardown_does_not_reset_it(void) {
  safety_float_refusal_count(true);
  safety_float_refusal_count(true);
  safety_float_refusal_count(true);        /* trips the flap; left dirty on purpose */
}

/* The other half of the pair. If pb_test_teardown() correctly cleared g_float_refusals
   after the previous case, safety_float_flap() reads false here; if the reset
   regressed, it reads true -- the exact failure mode the finding described, made to
   fail loudly instead of going quiet. */
static void test_g_float_refusals_does_not_leak_between_cases(void) {
  TEST_ASSERT_FALSE(safety_float_flap());
}

/* Statics of THIS SUITE, not of harness.h: harness.h is a fixture over hal_sim shared by
   every test file, and this switch is specific to dose_run()'s own enum. Takes `unsigned`,
   not dose_result_t: the exit-path loop below counts over the enum as `unsigned` (so that
   DOSE_RESULT_COUNT itself, one past the last named value, is a legal loop bound), and
   this is called with that raw loop variable. */
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

/* Sets up the ONE condition each result needs, calls dose_run(), and returns true --
   or returns false, WITHOUT calling dose_run() at all, for a result this build cannot
   reach yet. No `default:` arm: -Wall -Wextra then reports a newly added enum value as a
   missing case rather than letting it fall through to a vacuous pass.

   Every arm that runs a dose that reaches the pump loop (OK, BUSY is the exception --
   see below, COOLDOWN's first call, NOISE, ABORT_CAP, ABORT_STOP) advances the clock past
   PB_BOOT_GAP_MS first, for the same reason every standalone case in this file does: the
   ladder's cooldown rung is g_last_end_ms, a safety.cpp file static this loop's repeated
   pb_test_setup() calls do NOT reset (pb_test_setup() only resets the fake). Landing every
   such dose's start just above 10 s and its end therefore above 11 s of THIS iteration's
   own fresh clock is what keeps every LATER iteration's `hal_millis() - g_last_end_ms`
   read a huge (wrapped) unsigned difference rather than an accidental small one -- the
   same arrangement the brief's own standalone cases rely on, extended across the
   iterations of one loop instead of across separate test functions. */
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
      sim_set_i2c_fail(true);                 /* bites INSIDE the loop, at the live bus check --
                                                  the pre-dose ladder reads the cached healthy
                                                  flag, not a live transfer, so this dose starts */
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
      sim_serial_rx("stop\n");        /* sits in the fake's UART ring; dose_run()'s own
                                          cli_stop_clear() at entry does not touch that ring */
      dose_req_t q = {0}; q.by_time = true; q.cap_ms = 5000u;
      (void)dose_run(&q);
      return true;
    }
    case DOSE_RESULT_COUNT:
      return false;   /* the sentinel is never a result to drive */
  }
  return false;
}

/* §2.8. Nineteen results, nineteen exits, and D6 must be OFF at every one of them. The
   loop runs over the enum against DOSE_RESULT_COUNT, so a result added without a way to
   reach it fails HERE rather than in six months on a bench with 12 V on COM. */
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

/* §2.8's ordering pair, half 1: cal above range. */
void test_the_ladder_reports_the_more_specific_reason(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true);
  safety_force_bad_cal_();                          /* see below */
  dose_req_t r = {0}; r.ml = 9999u; r.cap_ms = 1000u;
  dose_result_t got = dose_run(&r);
  /* g_pulses_per_l is a file static of safety.cpp and pb_test_setup() cannot reach it, so
     the calibration is put back HERE and before the assertion - a suite that left it at
     zero would refuse every later case with DOSE_REFUSED_CAL, on the failing path too. */
  (void)cfg_pulses_per_l_set(PB_PULSES_PER_L_DEFAULT);
  TEST_ASSERT_EQUAL(DOSE_REFUSED_CAL, got);               /* cal above range */
}

/* The second half of task 17 step 7's ordering pair. It could not be written there: the
   only way to earn the latch is pb_latch_contra(), which this task's step 1 creates.
   An operator reading `err=dry` when the real reason was the contradiction latch pulls
   the tank apart looking for water that is already there. */
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

/* §2.8's second eye-checkable property, seen from a dose that ran rather than from the
   exit helpers directly: a refusal must never ack the previous dose's millilitres. */
void test_refusal_reports_zero_millilitres_not_the_previous_dose(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();          /* own tumbling window: the storm case leaves ~99 Hz behind */
  sim_set_float(true); sim_set_flow_ml_s(30);
  /* long_prime, and a cap ABOVE the fake's first edge. sim_flow_hz_() emits nothing until
     el >= PB_PRIME_MS_DEFAULT (3000), so a 2000 ms dose delivers zero pulses and this
     assertion fails; and once task 18's prime rule lands, any dose on the default window
     aborts NOFLOW at el == 3000. long_prime moves the window to PB_PRIME_LONG_MS (15000)
     and PB_PRIME_CAP_MS (20000) leaves a 6000 ms cap alone: ~3000 ms of flow, ~88 ml. */
  dose_req_t ok = {0}; ok.by_time = true; ok.cap_ms = 6000u; ok.long_prime = true;
  (void)dose_run(&ok);
  TEST_ASSERT_TRUE(dose_flow_ml() > 0u);
  pb_advance(PB_DOSE_MIN_GAP_MS + 1u);
  /* pulses_flow_rate()'s tumbling window is only ever advanced by a CALL to it, and
     dose_run()'s ladder calls it exactly once per invocation (the NOISE-idle rung). The
     first dose above left the window's base at 0 pulses / 0 ms, unmoved (that call landed
     inside its own first 100 ms and never rearmed it), so a call now -- tens of thousands
     of fake-ms later, after ~89 ml of REAL flow the first dose delivered -- would divide
     that whole historical total by the whole elapsed gap and read back a bogus non-zero
     rate (~30 Hz here), well above PB_FLOW_IDLE_MAX_HZ. That reads as DOSE_REFUSED_NOISE,
     not DOSE_REFUSED_FLOAT, and proves nothing about the float. Rebasing here is what a
     board actually gets for free -- main.cpp's loop() calls pulses_flow_rate() every pass,
     so the window is never this stale in the field; a synchronous host test that calls
     dose_run() twice with nothing in between has to do by hand what many loop() passes
     would have done anyway. */
  pulses_begin();
  sim_set_float(false);
  TEST_ASSERT_EQUAL(DOSE_REFUSED_FLOAT, dose_run(&ok));
  TEST_ASSERT_EQUAL_UINT16(0u, dose_flow_ml());
}

/* §2.6 guard 1, and it is the FIRST rung after busy for a reason: a counter that is not
   moving is a dog that will never bite, and every bound below it is then the only thing
   between a stuck loop and a running pump. */
void test_dose_refused_when_the_watchdog_counter_is_not_moving(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_wdt_stop();                                 /* the counter FREEZES (task 3) */
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_WDT, dose_run(&q),
      "a frozen watchdog counter must refuse with wdt");
}

/* §2.11. `dry on` is strictly more refusing, and it is the one thing an operator with
   their hands in the plumbing can rely on. Nothing is latched above it here, so `dry` is
   the token that has to come back rather than `contra`. */
void test_dose_refused_when_the_dry_latch_is_set(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  safety_dry_set(true);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_DRY, dose_run(&q),
      "the dry latch must refuse with dry");
}

/* DECISIONS #5's minimum gap since boot, and §2.11's cover for the one thing .noinit
   cannot hold: a power cycle clears the dry latch, and PB_BOOT_GAP_MS is what stands in
   its place for the first ten seconds. pb_test_setup() leaves the fake's clock at zero and
   hal_wdt_alive()'s probe advances it by about 41 ms, so no advance is needed here - the
   ABSENCE of one is the arrangement. */
void test_dose_refused_inside_the_boot_gap(void) {
  pb_test_setup();                                /* the clock is at zero: a fresh boot */
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_BOOT, dose_run(&q),
      "a dose inside PB_BOOT_GAP_MS must be refused with boot");
}

/* §2.6 guard 7, for EVERY caller - queued console impatience and backend adjacency alike.
   The first dose is by_time with a 1000 ms cap, deliberately UNDER PB_PRIME_MS_DEFAULT, so
   it ends on its own cap without arming task 18's prime rule and without satisfying §2.7's
   `elapsed_ms >= prime_ms`; it latches nothing. The second follows it immediately, and the
   cooldown is the only thing standing between them. */
void test_dose_refused_inside_the_minimum_gap_since_the_last_dose(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  (void)dose_run(&q);                             /* ends on its cap and stamps g_last_end_ms */
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_COOLDOWN, dose_run(&q),
      "a second dose inside PB_DOSE_MIN_GAP_MS must be refused with cooldown");
}

/* §2.10 and bring-up 5a. D5 reads PB_LOW for OK, so HIGH is a tank at or below the
   waterline - or a broken wire, which by DECISIONS #12's whole design is the same answer. */
void test_dose_refused_when_the_float_reads_not_ok(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(false);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_FLOAT, dose_run(&q),
      "a float reading not-OK must refuse with float");
}

/* §2.10's asymmetry, seen from the dose rather than from the debounce: PB_FLOAT_OK_SAMPLES
   consecutive OK readings GRANT, and one bad sample anywhere in that window refuses. The
   pattern below grants twice and fails on the third - the waterline bounce that a
   raw-sample implementation would report to the backend as float=1. */
void test_dose_refused_when_a_single_float_sample_is_bad(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float_pattern("1101111");               /* OK, OK, BAD, then OK forever (task 15) */
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_FLOAT, dose_run(&q),
      "one bad sample inside the debounce window must refuse with float");
}

/* §2.6 guard 4, FIRST line. A backend water command carries need_pos, and a cart whose
   position is unknown means the pump would dead-head against a closed manifold.

   The arrangement matters, because the obvious one tests nothing. cart_begin() leaves
   g_pos at 0 while q.outlet is 1, so guard 4's SECOND line (need_pos && cart_pos() !=
   outlet) refuses with the same DOSE_REFUSED_POS whether or not the first line exists:
   delete the rung this case is named for and it still passes. The calibrated arm below
   therefore loses the position while LEAVING cart_pos() equal to the outlet, so only the
   first line can answer. That is the case that matters in the field too: a failed
   cart_goto(N) clears g_home_seen and g_pos_valid but leaves g_pos where it was, so with
   only the second line a re-issued dose for that same outlet would run the pump against a
   cart nobody knows the position of. */
void test_dose_refused_when_position_is_unknown(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  /* native_cal ONLY: below, the calibrated arm drives the cart to gate 1 (1450 pulses at
     2 ms/pulse, ~2900 ms) and then into a stall on the way to gate 2 (PB_STALL_WINDOW_MS,
     2500 ms) before this function ever calls dose_run() -- burning ~5400 ms of fake clock
     that this file's usual "advance PB_BOOT_GAP_MS + 1 and go" arrangement does not budget
     for. g_last_end_ms is a safety.cpp file static this suite's pb_test_setup() cannot
     reach, and by this point in the file it holds a value close to 11000 (the cooldown
     case's own dose, still fresh at only ~1000-2000 ms past PB_BOOT_GAP_MS).

     Getting this margin right took two tries, and the FIRST one is worth recording because
     it is the trap this whole file's convention exists to dodge: adding exactly one more
     PB_DOSE_MIN_GAP_MS here pushed the uncalibrated arm's clock from ~10043 (BELOW the
     stale g_last_end_ms of ~11082, so the unsigned subtraction WRAPS to a huge number and
     the cooldown rung passes) to ~20045 -- now ABOVE g_last_end_ms, but by only ~8963 ms,
     which is a perfectly ordinary, non-wrapped, LESS-THAN-10000 difference: it broke the
     arm this test was already passing on while fixing the one it was not. There are only
     two safe zones for "current clock minus a stale g_last_end_ms" -- comfortably below it
     (wraps huge) or at least PB_DOSE_MIN_GAP_MS above it (an honest pass) -- and a single
     extra gap landed exactly in the unsafe strip between them. A wide margin, well clear of
     that strip for BOTH arms (the calibrated one's own ~5400 ms burn included), is what
     this file's rhythm relies on elsewhere too: found by running native_cal AND native, not
     by inspection of either alone. */
  pb_advance(4u * PB_DOSE_MIN_GAP_MS);
#if PB_PULSES_PER_GATE == 0
  (void)cart_begin();                    /* §2.15: cart_pos_known() is hard false here, so
                                            the first line is the only one that can answer */
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

/* Guard 4's SECOND line, which is a different fact from the first: the position is known,
   and it is the wrong one. Only the calibrated arm can reach it - §2.15 compiles
   cart_pos_known() out to hard false while PB_PULSES_PER_GATE is 0, so under [env:native]
   the first line answers and this one is unreachable. PB_PULSES_HOME_TO_1 is still 0 in the
   calibrated arm, so the cart is already at outlet 1 when it is at home: cart_goto(1)
   lands without turning the screw and the arrangement costs about three milliseconds of
   fake clock, which is what keeps this case's cooldown read in the same place as every
   other case's. */
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

/* §2.13's reason, stated as a refusal: A4/A5 are the mux select lines AND the home hall -
   the input that gates the pump - so an unhealthy bus is a rig that cannot say where its
   cart is. PB_I2C_FAIL_LIMIT consecutive failed transfers is what task 7 calls unhealthy.
   sensors.cpp's counters are file statics, so this case puts the bus back BEFORE it
   asserts, and therefore puts it back on the failing path too. */
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

/* DECISIONS #7 and §7. The PROTOCOL ceiling is PB_DOSE_MAX_ML (1000, == butler's
   MAX_DOSE_ML); the RIG ceiling is PB_DOSE_RIG_MAX_ML (250), "a reservoir small enough that
   a full dump is a mop-up". butler does not know about the smaller one - that is §4.6's
   going-live precondition - so 251 ml is a command the wire can legally carry and this rung
   is the only thing in the system that refuses it. */
void test_dose_refused_when_ml_exceeds_the_rig_ceiling(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  dose_req_t q = {0};
  q.ml = (uint16_t)(PB_DOSE_RIG_MAX_ML + 1u);     /* 251: inside the protocol, outside the rig */
  q.cap_ms = 10000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_RANGE, dose_run(&q),
      "a millilitre target above PB_DOSE_RIG_MAX_ML must be refused with range");
}

/* §2.8's second range rung: a cap_ms of zero is a request with no bound at all, and it is
   refused before the ml/need_pos checks that follow it in the ladder. */
void test_dose_refused_when_the_cap_is_zero(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 0u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_RANGE, dose_run(&q),
      "a zero cap_ms must be refused with range");
}

/* §2.8's third range rung, and the struct comment's own point: outlet is NEVER a sentinel.
   water=0 is a legal backend command (_int_in(v,"water",0,256), and butler's `outlet is
   None` guard does not catch 0), so 0 arrives here from the wire and must be refused
   rather than treated as "no outlet named". */
void test_dose_refused_when_a_need_pos_dose_names_outlet_zero(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  dose_req_t q = {0};
  q.outlet = 0u; q.ml = 100u; q.cap_ms = 10000u; q.need_pos = true;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_RANGE, dose_run(&q),
      "outlet 0 with need_pos must be refused with range, not treated as a sentinel");
}

/* §2.6 guard 6, the pre-dose half. Pulses arriving with D6 OFF are not water: an unplugged
   or floating D2 counting garbage would otherwise "reach target" in milliseconds. The storm
   runs at 100 Hz - an order under PB_FLOW_MAX_HZ (1200) and two orders over
   PB_FLOW_IDLE_MAX_HZ (2) - so it is unmistakably the IDLE ceiling that answers and not the
   in-dose one task 18 adds. pulses_begin() rebases the tumbling window on this case's own
   clock, and the 500 ms of storm after it is what fills the window the ladder then reads. */
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

/* A metered dose whose target rounds to zero pulses must be REFUSED, never run to its cap.
   With target == 0 the loop's `target && got >= target` can never fire, so the only exit
   left is `el >= cap_ms`: a request for no water that asserts D6 for the whole cap, and
   DOSE_ABORT_CAP here is exactly that failure.
   The arithmetic says the `target == 0` guard below the caps is belt-and-braces rather than
   the rung under test. target = ml * cfg / 1000, and the two rungs above it bound
   ml >= 1 and cfg >= PB_PULSES_PER_L_MIN (1000), so the smallest legal product is
   1 * 1000 / 1000 = 1 pulse: it can only be reached through a corrupted cfg that got past
   the cal rung. The REACHABLE zero target is ml == 0 - which butler can send - and the
   range rung is what answers it. */
void test_metered_dose_with_a_zero_target_is_refused_not_run_to_cap(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  dose_req_t q = {0};
  q.ml = 0u; q.by_time = false;                   /* metered, and asking for nothing */
  q.cap_ms = PB_DOSE_CAP_MS_MAX;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_RANGE, dose_run(&q),
      "a metered dose with a zero target must be refused with range, not run to its cap");
}

/* TARGET RULE 1: a metered dose stops when the METER says so, not when the clock does.
   DOSE_OK is reachable through one line and one line only - `target && got >= target` - so
   the result IS the assertion. The arithmetic, from §7's constants and nothing else:

       target       = ml * cfg / 1000 = 250 * 5880 / 1000 = 1470 pulses   (MULTIPLY FIRST)
       fake's rate  = 85 ml/s * PB_PULSES_PER_L_DEFAULT / 1000 = 499 pulses/s

   Under task 18's onset ramp (SIM_FLOW_ONSET_MS, hal_sim.cpp), the fake's first pulse lands
   about 1 ms after the ON write, not at PB_PRIME_MS_DEFAULT + 1 (3001 ms) the way it did
   before that fix -- measured directly (a 1 ms cap already shows one pulse landed), not
   estimated. dose_last_ms() for THIS exact request measures 3279 ms, not the pre-fix ~5945:
   still governed by the meter's own 499 pulses/s, plus the ramp's own ~150 ms of reduced
   early rate, not by any prime-window arithmetic at all.

   3279 ms is inside every cap that applies: PB_PRIME_CAP_MS (20000) under long_prime, and
   250 * 1000 / 30 * 2 = 16666 ms under native_measured -- and, unlike before this task's
   fix, inside the UNTOUCHED default prime window (3000 ms) too, with margin to spare, since
   flow is continuous throughout and neither the prime nor the stall rule ever finds a gap to
   fire on. `long_prime` is kept here anyway, for stability against exactly the kind of
   arithmetic drift this comment itself is being corrected for -- confirmed empirically:
   this case still passes with `q.long_prime` commented out -- but it is no longer load-
   bearing the way it was pre-fix, when the fake's own onset model made the default window
   fail by construction. See test_a_healthy_metered_dose_completes_on_the_default_prime_
   window for the case that exercises the default window on purpose. */
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

/* TARGET RULE 2: when the meter never reaches the target, the CAP ends the dose - and the
   cap is a bound on time, never a second target. The arithmetic:

       target = ml * cfg / 1000 = 100 * 5880 / 1000 = 588 pulses, and the meter delivers 0
       cap    = the typed 1000 ms; under native_measured the measured clamp computes
                100 * 1000 / 30 * 2 = 6666 ms and leaves the typed value alone

   cap_ms is deliberately under PB_PRIME_MS_DEFAULT so that the cap is the only rule that
   can fire: task 18's prime and stall rules both arm at `el >= prime_ms`, and §2.7's latch
   needs `elapsed_ms >= prime_ms` too, so this dose ends without latching anything. */
void test_dose_stops_at_the_cap_when_flow_never_reaches_target(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_flow_ml_s(0u);                          /* the pump runs; nothing moves */
  dose_req_t q = {0}; q.ml = 100u; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_CAP, dose_run(&q),
      "a metered dose that never reaches its target must end on its cap");
}

/* The cap bounds how long D6 is ASSERTED, which is the only thing that puts water on the
   floor - not how long dose_run() takes to return. sim_pump_on_ms() is the fake's
   cumulative count of milliseconds with the pin high, so it measures D6 and not the
   function. The twenty milliseconds of slack are the loop's own granularity: the fake
   advances one millisecond per hal_millis() call, and the unconditional OFF write is the
   statement after the break. */
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

/* The case above bounds D6 by the cap the CALLER asked for. This one bounds the cap itself,
   which is a different guarantee and, until this case, an untested one: every cap_ms in this
   file was <= PB_DOSE_CAP_MS_MAX, so `if (cap_ms > PB_DOSE_CAP_MS_MAX) cap_ms =
   PB_DOSE_CAP_MS_MAX;` (safety.cpp) could be DELETED with all 259 cases still green.

   That one line is the whole of the "hard maximum run time in the same code path that asserts
   the pump pin", which is one of the three firmware measures standing in for the hardware
   interlock this rig does not have. Nothing upstream narrows a cap for it: report.cpp's parser
   deliberately lets cap_s ride to 65535 unclamped and says so, and exec.cpp multiplies it by
   1000. So a buggy or hostile backend sending cap_s=65535 authorises a 65,535-SECOND pump run,
   and this line is the only thing between that number and D6.

   by_time, so there is no pulse target and the cap is the ONLY rule that can end the dose; real
   flow, so neither the prime nor the stall rule can end it first; and the assertion is on
   dose_last_ms() AND on sim_pump_on_ms(), because the second measures the pin rather than the
   function. 65 s asked for, 60 s allowed: a value far enough over the ceiling to be
   unmistakable and small enough that the fake clock walks it in well under a second. */
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

/* §6: divide-first truncates the calibration to whole pulses per millilitre. Runs at the
   cal range's floor, a legal-but-ugly value, the nominal default and the ceiling, and
   checks dose_last_pulses() against the MULTIPLY-FIRST arithmetic within one pulse -- the
   loop's own granularity, since at most one flow edge lands per 1 ms tick (the fake's
   pulse period at 85 ml/s is ~2 ms, well over the 1 ms step).

   ml is 150, not the rig ceiling's 250: under native_measured the cap is ALSO clamped to
   2x the requested water (ml * 1000 / PB_ML_PER_S_MEASURED * 2), and at cfg = 20000 (the
   ceiling) target = 3000 pulses takes ~9012 ms of simulated flow -- inside the 150 ml
   clamp's 10000 ms bound, but NOT inside 250 ml's would-be 16666 ms/2=... the smaller ml
   was chosen so one dose_req_t works in all three native environments without a second,
   per-environment version of this case.

   Under native_measured only, task 18's plausibility rule bounds which (cfg, flow_ml_s)
   pairs can honestly reach DOSE_OK at all, and the bound is NOT loose. The fake's emitted
   RATE is always flow_ml_s * PB_PULSES_PER_L_DEFAULT / 1000 Hz -- fixed at the sensor's own
   5880 pulses/L rating, deliberately independent of whatever cfg the firmware is TOLD (a
   miscalibration is exactly what cfg=1000/1999 model). Reaching `target = ml*cfg/1000`
   pulses then takes elapsed_ms = ml*cfg*1000/(flow_ml_s*5880); the plausibility rule aborts
   unless elapsed_ms >= ml*1000*PLAUS_DEN/(PLAUS_NUM*PB_ML_PER_S_MEASURED) = ml*8.33 (at
   PB_ML_PER_S_MEASURED=30). The `ml` cancels on both sides, leaving cfg >= flow_ml_s*49 as
   the ONLY thing that decides it -- at flow_ml_s=85 that needs cfg >= 4165, which the floor
   (1000) and the "ugly" value (1999) both fail. That is the rule doing its job, not a test
   bug: a calibration this far below the sensor's true rating really would make an honest
   85 ml/s look like several hundred ml/s once run through it, and that is what the
   plausibility ceiling exists to catch. Lowering flow_ml_s to clear it at the floor would
   just as surely blow the ceiling cfg's cap (the two constraints pull in opposite
   directions across a 20x calibration span -- there is no single flow_ml_s satisfying
   both). native (PB_ML_PER_S_MEASURED == 0, plausibility compiled out) already proves the
   MULTIPLY-FIRST arithmetic at all four points; native_measured proves it only where the
   fixture's own flow rate is physically honest for that calibration. */
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

/* Every bound in dose_run() is computed as an unsigned difference, and this is the case
   that proves it rather than assumes it: the cap must straddle the millis() wrap or the
   case is vacuous. 0xFFFFF000 is 4095 ms below UINT32_MAX, so a cap of 1000-2000 ms (the
   pattern every other case in this file uses to dodge task 18's prime rule) would finish
   BEFORE the wrap. long_prime with a 6000 ms cap wraps ~4095 ms in and still has to land
   on the far side of it.

   dose_last_ms() reads MORE than cap_ms, and that is dose_run()'s own arithmetic, not a
   fuzz margin this case invented: the loop's break condition (`el >= cap_ms`) fires with
   the loop's OWN `el` at or just past cap_ms, but dose_end_ml_() is not handed that value --
   it is handed `g_last_end_ms - t0`, and g_last_end_ms comes from a FRESH `hal_millis()`
   call made after the break, for the end-of-dose bookkeeping. That call advances the fake's
   clock by its own one more millisecond before it reads it back, exactly like every other
   hal_millis() call in this file's contract. Before task 18 that was the WHOLE story and the
   offset was an exact, unconditional +1: one hal_millis() call per iteration inside the
   loop, one more in the epilogue.

   Task 18's rule 1 (`pulses_flow_rate() > PB_FLOW_MAX_HZ`) is now evaluated EVERY iteration,
   above every other rule, and pulses_flow_rate() itself calls hal_millis() to read its own
   100 ms tumbling window -- a SECOND clock-advancing call per pass, beyond the loop's own
   `now = hal_millis()`. The loop's `el` only ever sees the FIRST of the two, so `el` and the
   fake's global clock drift apart by one tick every iteration; `el` still breaks at or just
   past cap_ms, but which side of cap_ms it lands on now depends on parity (whether cap_ms's
   own position lines up with an even or odd count of these paired ticks), which is a fact
   about the fake's bookkeeping, not about dose_run(). The exact `+1` this case used to check
   is therefore no longer a safe universal constant -- it is `+3` for THIS cap_ms and clock
   position, empirically, and a different cap could legally read `+2`. What still has to
   hold, and is what this case actually cares about, is that the cap ends the dose within a
   handful of milliseconds of the typed bound, straddling the millis() wrap or not. */
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

/* Bring-up 4a, 5a and 5b all run BEFORE the cart is calibrated, so a console `pump` that
   demanded a known position would make every one of them unrunnable. need_pos = false must
   never reach either position rung, whatever cart_pos_known() says. */
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

/* §2.8, §15.3. At task 17, `pump` is not yet a console command (task 20 adds it), so a
   buffered `pump 60000` would reach `? unknown; type help` and this case would pass whether
   or not the bytes were discarded -- it proves nothing. `status` is dispatched today, so
   its ABSENCE from the console's output after the dose is what proves the discard: bytes
   typed while the console looked frozen sit in the fake's raw UART ring, are consumed by
   cli_stop_requested() inside the loop (matching neither `stop` nor `dry on`) and
   reconstructed into cli.cpp's OWN pushback buffer -- which dose_run()'s closing
   cli_stop_clear() throws away, together with hal_serial_drain()'s discard of the ring
   itself. Carry 1: drain alone would leave the reconstructed pushback for cli_poll() to
   replay after the dose; clear alone would leave raw bytes still in the ring to fall
   through the fallback read and be dispatched. Only both together discard it. */
void test_bytes_buffered_during_a_dose_are_discarded_not_executed(void) {
  char tx[512];

  /* Half 1, proving hal_serial_drain() alone is not enough to have caught this: a 1 ms cap
     means the loop's `el >= cap_ms` fires on its FIRST iteration -- t0 is captured one
     hal_millis() call before the first `now` read inside the loop, so el is already 1 by
     then -- which is ABOVE the cli_stop_requested() check in the loop body, so that check
     is never reached and "status\n" sits UNREAD in the fake's raw UART ring for the whole
     dose. Only hal_serial_drain()'s discard of the ring can clear it; cli_stop_clear() has
     nothing to do here, because cli.cpp's own pushback buffer was never touched. */
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

  /* Half 2, proving cli_stop_clear() alone is not enough either: a longer cap lets the
     loop's OWN cli_stop_requested() call read "status\n" from the raw ring (it matches
     neither `stop` nor `dry on`) and reconstruct it, byte for byte, into cli.cpp's own
     pushback buffer -- a SEPARATE buffer hal_serial_drain() cannot see, because it only
     ever touches the raw ring. Only cli_stop_clear()'s own discard of that pushback stops
     read_console_() from replaying it into cli_poll() once the dose ends. */
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

/* The measured clamp half of §7's two-arm cap: `test_dose_cap_is_clamped_to_sixty_seconds`
   (the unconditional 60 s clamp) needs a meter that stays live for the WHOLE dose --
   task 18's sim_set_flow_burst_pulses() injector, which does not exist yet -- and reaching
   60 s any other way either finishes on task 18's not-yet-written prime rule (~3001 ms) or,
   once task 19's latch lands, satisfies every one of §2.7's five conditions and poisons
   every later case in the file with a persistent .noinit contra latch. That case is task
   18's. This one only needs PB_ML_PER_S_MEASURED > 0, which native_measured alone defines,
   so the other arm just proves it is compiled OUT. */
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
  /* dose_last_ms() lands a few ticks past `want`, never exactly at it: dose_last_ms() is
     g_last_end_ms - t0, and g_last_end_ms is a FRESH hal_millis() call made after the loop
     breaks, past the loop's own last `el` -- see test_dose_cap_holds_across_a_millis_
     rollover's comment for the full derivation of why this is no longer an exact `+1` since
     task 18's rule 1 added a SECOND hal_millis()-calling site (pulses_flow_rate()) inside
     the loop. This case is what first caught the underlying arithmetic FOR REAL:
     PB_ML_PER_S_MEASURED was silently 0 in every native_measured run before include/
     config.h's own #ifndef fix landed alongside this test (found here), so this exact
     arithmetic had never actually run before. */
  TEST_ASSERT_TRUE_MESSAGE(dose_last_ms() > want && dose_last_ms() <= want + 5u,
      "dose_last_ms() must land within a few ticks of the measured clamp");
#else
  TEST_IGNORE_MESSAGE("PB_ML_PER_S_MEASURED == 0: the measured cap clamp is compiled out; "
                       "native_measured runs this case");
#endif
}

/* Fix round 1, Important finding (review of this task): the flap counter's "exactly two
   call sites" guarantee -- dose_end_() increments ONLY on DOSE_REFUSED_FLOAT, dose_end_ml_()
   resets UNCONDITIONALLY -- had no test that drove the counter THROUGH dose_run() and its
   ladder. Every existing case that touches safety_float_flap() (see the task-15 block
   above) calls safety_float_refusal_count() directly, bypassing dose_end_() entirely, so a
   guard broadened to also count a non-float reason would pass the whole suite unnoticed.
   The reviewer proved this by mutating dose_end_()'s guard to `r == DOSE_REFUSED_FLOAT ||
   r == DOSE_REFUSED_COOLDOWN` and watching test_dose pass unchanged.

   This case reproduces that exact scenario through dose_run() rather than assuming it:
   a priming dose (ABORT_CAP, no flow) resets the counter via dose_end_ml_() and stamps
   g_last_end_ms; the very next call, still inside PB_DOSE_MIN_GAP_MS of it, is refused
   DOSE_REFUSED_COOLDOWN through the REAL ladder rung, not stood in for. Correct code
   leaves the counter untouched by that refusal. The reviewer's mutation would let it
   through, and the effect is deliberately made to surface EARLY rather than only at the
   end: after the interleaved cooldown refusal, only two genuine float refusals are needed
   to trip the flap instead of three, so the assertion after the SECOND float refusal below
   -- which this file's other float-flap case (via the setter) already proves must still
   read false at two -- is what a widened guard actually breaks first. */
void test_the_flap_counter_is_driven_by_dose_run_not_the_setter(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(0u);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;

  /* Prime: a GRANTED-reaching dose, which resets the counter through dose_end_ml_()'s
     unconditional call and stamps g_last_end_ms fresh. */
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_CAP, dose_run(&q), "arrange: the priming dose");
  TEST_ASSERT_FALSE(safety_float_flap());

  /* The interleaved NON-float refusal, reached through the real ladder: still inside
     PB_DOSE_MIN_GAP_MS of the priming dose above, so this is a genuine DOSE_REFUSED_
     COOLDOWN, not a stand-in for one. Correct code -- dose_end_()'s guard testing only
     `r == DOSE_REFUSED_FLOAT` -- leaves the counter at 0 here. */
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_COOLDOWN, dose_run(&q),
      "arrange: a genuine cooldown refusal, still inside the gap");
  TEST_ASSERT_FALSE(safety_float_flap());

  /* Past the gap now, so what follows really does reach the float rung rather than the
     cooldown rung above it. */
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

/* Fix round 1, Minor finding (review of this task): DRY above BOOT is proven today only
   ACCIDENTALLY, by test_pump_is_off_on_every_exit_path's DRY fixture, which happens to run
   at clock zero (inside the boot gap) without meaning to test the order at all. The two
   DEDICATED single-rung cases (test_dose_refused_when_the_dry_latch_is_set,
   test_dose_refused_inside_the_boot_gap) both still pass if the two rungs are swapped,
   because neither one combines the two conditions -- only a case that latches dry AND
   stays inside the boot gap can tell the orderings apart, and this is that case.

   dry is the more actionable fact for an operator: err=boot within the first ten seconds
   after a reset says "wait and retry"; err=dry sends them to `dry off` instead, which is
   what is actually true here and boot's own advice would hide. Added as the one extra
   adjacent-pair case worth its keep, alongside cal-above-range (already in this file) and
   contra-above-dry (task 19) -- not a case per rung, which the review did not ask for. */
void test_the_ladder_reports_dry_above_boot(void) {
  pb_test_setup();                                /* clock at zero: inside the boot gap */
  safety_dry_set(true);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_DRY, dose_run(&q),
      "the dry latch must be reported even while still inside the boot gap");
}

/* Fix round 2, Important finding (review of this task): the flap counter's contract has
   TWO halves. Fix round 1 proved the first -- dose_end_()'s guard increments ONLY on
   DOSE_REFUSED_FLOAT -- through real dose_run() calls. Nothing proved the second:
   dose_end_ml_()'s reset call is UNCONDITIONAL, on every path that reaches it, not only
   DOSE_OK. The reviewer's own proof: make the reset conditional --
   `if (r == DOSE_OK) safety_float_refusal_count(false);` -- and the whole native
   environment (98 cases across all four suites) passes unchanged. A dose that reaches
   dose_end_ml_() by ABORT_CAP, ABORT_NOFLOW or ABORT_STOP RAN -- water moved, nothing about
   the float was wrong -- and under that mutation it would silently stop clearing stale
   float refusals from earlier, unrelated doses, which then accumulate across doses that
   have nothing to do with the float and eventually trip the flap for no proximate reason a
   status line can point to.

   This case is the mirror of the round-1 one: accumulate two genuine float refusals
   (one short of PB_FLOAT_FLAP_LIMIT), run a dose that reaches dose_end_ml_() on a
   NON-DOSE_OK path (ABORT_CAP -- the pump asserted, the cap ended it, nothing about the
   float), then show a single FURTHER float refusal does not trip the flap. If the ABORT_CAP
   dose had not cleared the counter, that third-in-total float refusal would be the count's
   third -- exactly at the flap limit -- so this is where a conditional reset is caught. */
void test_the_flap_counter_is_cleared_by_dose_run_on_any_granted_path(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(0u);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;

  /* Baseline: a granted-reaching dose, clearing whatever the counter held before this case
     (teardown already guarantees 0, but this also stamps g_last_end_ms fresh in THIS
     case's own clock domain, which the cooldown arithmetic below depends on). */
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_CAP, dose_run(&q), "arrange: the baseline dose");
  TEST_ASSERT_FALSE(safety_float_flap());
  pb_advance(PB_DOSE_MIN_GAP_MS + 1u);      /* past the gap, so the doses below reach their
                                                own rungs rather than refusing on cooldown */

  /* Two genuine float refusals -- count = 2, one short of the flap limit. */
  sim_set_float(false);
  TEST_ASSERT_EQUAL(DOSE_REFUSED_FLOAT, dose_run(&q));
  TEST_ASSERT_EQUAL(DOSE_REFUSED_FLOAT, dose_run(&q));
  TEST_ASSERT_FALSE(safety_float_flap());

  /* The dose under test: reaches dose_end_ml_() on a NON-DOSE_OK path. Correct code --
     dose_end_ml_()'s reset call is UNCONDITIONAL -- clears the counter here regardless of
     what r is. */
  sim_set_float(true);
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_CAP, dose_run(&q),
      "arrange: a granted dose that is NOT DOSE_OK");
  TEST_ASSERT_FALSE(safety_float_flap());

  /* Past the gap again, then ONE more genuine float refusal. A conditional reset above
     would have left the stale count of 2 in place, and this single refusal would push it
     to 3 -- tripping the flap here instead of leaving it false. */
  pb_advance(PB_DOSE_MIN_GAP_MS + 1u);
  sim_set_float(false);
  TEST_ASSERT_EQUAL(DOSE_REFUSED_FLOAT, dose_run(&q));
  TEST_ASSERT_FALSE_MESSAGE(safety_float_flap(),
      "the counter must have been cleared by the ABORT_CAP dose above, not only by DOSE_OK");
}

/* This case and the next are a deliberate pair, run back-to-back in that order (see
   main()) -- the same shape as the g_dosing and g_float_refusals pairs above, guarding the
   same class of bug for the third static of it in this file (fix round 2):
   g_last_end_ms is process-lifetime state in safety.cpp, and this case leaves it non-zero
   and does NOT reset it inline. If pb_test_teardown() ever stops resetting it, the NEXT
   case's own dose reads a leftover "a dose ended moments ago" and is refused
   DOSE_REFUSED_COOLDOWN instead of the DOSE_ABORT_CAP it actually arranges for. */
void test_g_last_end_ms_leaks_here_if_teardown_does_not_reset_it(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true);
  sim_set_flow_ml_s(0u);
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
  (void)dose_run(&q);            /* reaches the loop; stamps g_last_end_ms non-zero, left
                                     dirty on purpose -- see the next case */
}

/* The other half of the pair. Advances FURTHER than the usual PB_BOOT_GAP_MS + 1: a leaked
   g_last_end_ms from the previous case sits at roughly PB_BOOT_GAP_MS + a dose's own cost
   (about 11 s) in that case's own clock domain, and this case's clock starts back at zero
   too -- so a plain PB_BOOT_GAP_MS + 1 advance would land BELOW the leaked value, wrapping
   the unsigned "current minus stale" difference to a harmless huge number and missing the
   leak entirely (the exact trap several standalone cases in this file were found to have
   hit the OTHER way around, earlier in this task). Advancing to 2 x PB_BOOT_GAP_MS instead
   lands ABOVE the leaked value but still inside its ten-second cooldown window, which is
   the one zone that actually tells a leak apart from a correct reset. */
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

/* §2.14. The rate rules are evaluated ABOVE the target rule, always. With the target rule
   first, a D2 at the ISR's own 2 kHz ceiling reaches a 250 ml target in about 625 ms and
   dose_run() returns DOSE_OK with flow_ml=250 for water that never moved. */
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

/* The same storm, stated as the consequence rather than the mechanism, because this is
   the sentence that has to stay true: no target is ever reached by noise. */
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

/* Fix round 1, Important finding (review of this task): the two cases above do not actually
   prove rule 1 is evaluated ABOVE the target rule -- they prove the storm aborts the dose,
   which is a weaker claim. Their target (250 ml -> 1250 pulses at cfg 5000) needs ~625 ms of
   a 2000 Hz storm, six rate-estimator windows (100 ms each) after the storm is first visible
   at ~100 ms -- so under EITHER ordering the loop has already exited via the rate ceiling
   long before the target could ever be reached, and swapping which of the two checks runs
   FIRST changes nothing observable. The reviewer proved this by mutation: moving ONLY the
   rate-ceiling check below the target check (leaving the plausibility check where it
   belongs) passed the entire suite unchanged, under BOTH native and native_measured.

   This fixture is built to land the target INSIDE the same loop iteration the rate
   estimator's first 100 ms window closes and reports the storm, so both conditions
   (`got >= target` and `pulses_flow_rate() > PB_FLOW_MAX_HZ`) become true at once and the
   ORDER of the two checks is what actually decides the result. 45 ml (-> 225 pulses at
   cfg 5000) was found empirically, not derived: at a 2000 Hz storm with cfg 5000 the window
   closes at el ~= 115 ms with 227 pulses on the counter (one ml lower, 44, reaches its own
   220-pulse target at ~112 ms, BEFORE the window has closed at all -- the estimator has not
   measured anything yet, so neither ordering can catch it, which is a real, different
   limitation of a 100 ms window and not what this case is testing; one ml higher, 46, is not
   reached until AFTER the window has already closed and broken the loop via rate1, so it is
   back to "order does not matter" territory). Reversing only the rate-ceiling check below
   the target check flips THIS exact fixture to DOSE_OK at the same 227 pulses -- proof
   below. */
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
   window and nothing at all comes out -- the pitch-28a903 scenario, bring-up 7a's own
   "run it dry the first time" case. */
void test_prime_abort_fires_when_nothing_flows_in_the_prime_window(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_float(true); sim_set_flow_ml_s(0);         /* the pump runs; nothing moves */
  dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
  TEST_ASSERT_EQUAL(DOSE_ABORT_NOFLOW, dose_run(&q));
  TEST_ASSERT_TRUE(sim_pump_on_ms() < PB_PRIME_MS_DEFAULT + 200u);
}

/* The case that replaces the design's inverted one. `prime` EXTENDS the window; a dose
   that never flows still aborts, at PB_PRIME_LONG_MS rather than never. */
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

/* Armed on TIME. A dose that delivered four pulses and then stopped must still abort:
   arming on `got` is what let zero flow disarm the rule entirely. */
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

/* THE OUTCOME THIS TASK HAS TO SETTLE: a HEALTHY dose, on the untouched DEFAULT prime
   window (no long_prime), with real continuous flow, reaches its millilitre target and
   returns DOSE_OK. Before this task's fix to sim_flow_hz_()'s onset (see hal_sim.cpp), the
   fake stayed silent until elapsed reached PB_PRIME_MS_DEFAULT and then jumped straight to
   full rate -- the SAME millisecond dose_run()'s prime rule samples `got`, so at most one
   pulse could ever be on the counter when the rule checked, never PB_PRIME_MIN_PULSES. A
   healthy dose could not complete on the default window at all; every other case in this
   file that needed real flow worked around it with `long_prime`. This case takes none of
   that workaround: 100 ml at 85 ml/s needs on the order of a few hundred milliseconds,
   comfortably inside the untouched 3 s window, with the window itself still free as margin. */
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

/* Rule 1's own ceiling, against the YF-S401's rating rather than against the target rule's
   order (that is what the two storm cases above already prove). A rate above 1200 Hz is
   not a fast pump; it is a meter that is not measuring water. The 100 ms estimator window
   is what makes the verdict arrive inside a second. */
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

/* Rule 2, on the DOSE_OK path itself: at 30 ml/s (native_measured's PB_ML_PER_S_MEASURED) a
   120 ml dose honestly needs 4000 ms. This one delivers it in well under 900 ms -- fast
   enough to clear the plausibility floor (el < ml*1000/(PLAUS_NUM*PB_ML_PER_S_MEASURED) =
   1000 ms here) but its own pulse RATE (~882 Hz) stays comfortably under PB_FLOW_MAX_HZ
   (1200), so rule 1 cannot be what fires -- only rule 2 can, and that is the point: this is
   noise a rate ceiling alone would miss, because 882 Hz is a rate a real YF-S401 could
   produce; it is the ELAPSED time against the DELIVERED volume that is impossible. */
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

/* Bring-up 5b in software: pulling the float mid-dose must stop it within one loop
   iteration. 5b's own pass criterion also includes contra=0 afterwards, because the float
   dropping is the two sensors AGREEING (the meter was genuinely flowing and the tank ran
   dry), not contradicting -- task 19 owns the latch's setter, but the reader exists now and
   this is the case that has to keep reading false once that setter lands. */
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

/* §2.13's live half. The bus wedges AFTER the dose has started: sim_set_i2c_fail(true) does
   not retroactively fail sensors.cpp's cached healthy flag (that needs PB_I2C_FAIL_LIMIT
   consecutive failed transfers through gate_()), so the pre-dose ladder's !sensors_i2c_
   healthy() rung reads clean and this dose is granted -- it is the LIVE cart_bus_check()
   inside the loop, at most once per PB_POS_RECHECK_MS, that discovers the bus is gone.
   hal_i2c_recover() refuses while g_dosing (§2.13), so the bus stays broken for the rest of
   the dose, which is the point: A4/A5 carry the mux select lines and the home hall. */
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

/* §2.12's last-resort abort, driven by real bytes through sim_serial_rx() WHILE the loop is
   spinning -- never by poking cli.cpp's flag directly. */
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

/* `dry on` means the same thing during a dose as before one (cli.cpp's own comment): it
   both aborts the dose (through the identical DOSE_ABORT_STOP matcher `stop` uses) AND
   latches the dry flag, in the same byte-wise pass. */
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

/* This is what makes a 60 s dose legal under a 5592 ms grant (§3), and it is the first
   assertion that fails if anyone ever adds a `continue` to the loop body.

   Anchored on the ON write, not on "skip the first N feeds": the refusal ladder's OWN
   hal_wdt_alive() feeds twice around a deliberate ~PB_WDT_PROBE_MS UNFED window (§2.5)
   before the loop even starts, and safety_float_ok_debounced() feeds again, closely,
   while sampling -- both correct, neither the loop. Scanning from the first WDT_FEED
   after the pump's ON write is what actually isolates "every iteration of the dose loop"
   rather than assuming a fixed feed count for everything that runs ahead of it. */
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
    /* Rule 1 (pulses_flow_rate()) calls hal_millis() a second time on every pass, so a
       loop iteration now costs two fake ticks, not one -- verified empirically at exactly
       2 ms per iteration once anchored on the ON write; 3 ms is that plus one tick of
       slack, not a re-derivation of task 17's original (single-hal_millis()-call) bound. */
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
     pb_test_teardown() resets g_float_refusals between them (task 15 fix round 1). */
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
     pb_test_teardown() resets g_last_end_ms between them (task 17 fix round 2). */
  RUN_TEST(test_g_last_end_ms_leaks_here_if_teardown_does_not_reset_it);
  RUN_TEST(test_g_last_end_ms_does_not_leak_between_cases);
  /* LAST: it deliberately leaves g_last_end_ms at a small wrapped value, which would
     otherwise poison every later case's cooldown-avoidance arithmetic (see its own
     comment) -- redundant with teardown's own reset after fix round 2, but left in place
     as the belt to teardown's braces, and to minimise churn on an already-passing case. */
  RUN_TEST(test_dose_cap_holds_across_a_millis_rollover);
  return UNITY_END();
}
