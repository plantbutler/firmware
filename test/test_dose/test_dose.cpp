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
   than guessed. 14 here: DOSE_REFUSED_CONTRA (task 19's latch) and the four DOSE_ABORT_*
   results that task 18's mid-dose rules produce (NOFLOW, NOISE, FLOAT, POS) do not exist
   yet -- the breaks that would produce them are not in the loop. 19 - 5 = 14. Each later
   task raises this as it makes another arm reachable: 18 after task 18 step 9, 19 after
   task 19 step 8. */
#define PB_DRIVABLE_RESULTS 14

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
    case DOSE_REFUSED_CONTRA:
      return false;   /* task 19's latch: pb_latch_contra() does not exist yet */
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
    case DOSE_ABORT_NOFLOW:
      return false;   /* task 18's prime/stall rules: the break does not exist yet */
    case DOSE_ABORT_NOISE:
      return false;   /* task 18's in-dose rate/plausibility rules */
    case DOSE_ABORT_FLOAT:
      return false;   /* task 18's mid-dose float check */
    case DOSE_ABORT_POS:
      return false;   /* task 18's PB_POS_RECHECK_MS live bus check */
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

/* §2.8's ordering pair, half 1: cal above range. The other half, contra above dry, needs
   pb_latch_contra(), which does not exist until task 19 step 1 -- task 19 step 5 appends
   test_the_ladder_reports_contra_above_dry to this same file once that fixture exists. */
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
       first pulse  = PB_PRIME_MS_DEFAULT + 1 = 3001 ms into the dose
       target at    = 3001 + 1469 * 1000/499 = ~5945 ms

   and 5945 ms is inside every cap that applies: PB_PRIME_CAP_MS (20000) under long_prime,
   and 250 * 1000 / 30 * 2 = 16666 ms under native_measured.

   `long_prime` is not decoration here, and the reason is a property of the FIXTURE rather
   than of dose_run(): the fake's pump model delivers its first pulse at
   PB_PRIME_MS_DEFAULT + 1 ms after the ON write, which is one millisecond AFTER task 18's
   prime rule fires at `el >= prime_ms && got < PB_PRIME_MIN_PULSES`. A metered dose on the
   default window therefore aborts NOFLOW at exactly 3000 ms with nothing wrong with it.
   PB_PRIME_LONG_MS is the only printed lever that moves that window, and the
   PB_PRIME_CAP_MS it brings with it is still three times the time this dose needs. */
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
   per-environment version of this case. */
void test_target_pulses_match_the_calibration_within_one_pulse(void) {
  const uint16_t cfgs[] = { 1000u, 1999u, 5880u, 20000u };   /* floor, ugly, nominal, ceiling */
  for (size_t i = 0; i < sizeof cfgs / sizeof cfgs[0]; ++i) {
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

   dose_last_ms() reads cap_ms + 1, not cap_ms, and that is dose_run()'s own arithmetic, not
   a fuzz margin this case invented: the loop's break condition (`el >= cap_ms`) fires with
   the loop's OWN `el` at exactly cap_ms, but dose_end_ml_() is not handed that value -- it
   is handed `g_last_end_ms - t0`, and g_last_end_ms comes from a FRESH `hal_millis()` call
   made after the break, for the end-of-dose bookkeeping. That call advances the fake's
   clock by its own one more millisecond before it reads it back, exactly like every other
   hal_millis() call in this file's contract. No earlier case in this file catches it:
   every other cap-driven case checks dose_result_t or a loose (+20 ms) bound on
   sim_pump_on_ms(), never dose_last_ms() itself. This is the ONLY case exact enough to
   surface it, discovered by running it rather than by reading the design doc's prose
   ("asserts the dose still ends at cap_ms"), which does not mention the extra tick.

   This case is registered LAST in main(), after every other case in this file: the wrap
   it engineers lands g_last_end_ms at a SMALL absolute value (the whole point), which would
   otherwise poison the cooldown rung's "g_last_end_ms always stays comfortably below THIS
   case's own post-boot-gap clock, so the unsigned difference wraps huge" convention every
   later case in the file relies on -- found by running the suite, not by inspection. */
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
  TEST_ASSERT_EQUAL_UINT32(6000u + 1u, dose_last_ms());
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
  /* +1: dose_last_ms() is g_last_end_ms - t0, and g_last_end_ms is a FRESH hal_millis()
     call made after the loop breaks, one tick past the loop's own last `el` -- see
     test_dose_cap_holds_across_a_millis_rollover's comment for the full derivation. This
     case is what first caught it FOR REAL: PB_ML_PER_S_MEASURED was silently 0 in every
     native_measured run before include/config.h's own #ifndef fix landed alongside this
     test (found here), so this exact arithmetic had never actually run before. */
  TEST_ASSERT_EQUAL_UINT32(want + 1u, dose_last_ms());
#else
  TEST_IGNORE_MESSAGE("PB_ML_PER_S_MEASURED == 0: the measured cap clamp is compiled out; "
                       "native_measured runs this case");
#endif
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
  RUN_TEST(test_metered_dose_with_a_zero_target_is_refused_not_run_to_cap);
  RUN_TEST(test_dose_stops_at_the_millilitre_target);
  RUN_TEST(test_dose_stops_at_the_cap_when_flow_never_reaches_target);
  RUN_TEST(test_pump_on_time_never_exceeds_the_cap);
  RUN_TEST(test_pump_is_off_on_every_exit_path);
  RUN_TEST(test_the_ladder_reports_the_more_specific_reason);
  RUN_TEST(test_refusal_reports_zero_millilitres_not_the_previous_dose);
  RUN_TEST(test_target_pulses_match_the_calibration_within_one_pulse);
  RUN_TEST(test_console_pump_does_not_require_a_known_position);
  RUN_TEST(test_bytes_buffered_during_a_dose_are_discarded_not_executed);
  RUN_TEST(test_cap_is_clamped_to_twice_the_requested_millilitres);
  /* LAST: it deliberately leaves g_last_end_ms at a small wrapped value, which would
     otherwise poison every later case's cooldown-avoidance arithmetic (see its own
     comment). */
  RUN_TEST(test_dose_cap_holds_across_a_millis_rollover);
  return UNITY_END();
}
