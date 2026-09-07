/* test_report.cpp: the k=v report body, its byte budget, and the response parser, on the host. */
#include <unity.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../support/bodies.h"
#include "../support/harness.h"
#include "config.h"
#include "hal.h"
#include "sim.h"
#include "sensors.h"
#include "pulses.h"
#include "safety.h"
#include "report.h"
#include "cart.h"
#include "noinit.h"

static char g_buf[PB_BODY_CAP];
static char g_blk[PB_BODY_CAP];     /* the diagnostic block alone, at its clamp width */

/* cart.cpp's statics outlive sim_reset(); without this a homed cart leaks pos=ok into
   later cases. */
void setUp(void) {
  pb_test_setup(); sensors_begin(); (void)cart_begin();
  memset(g_buf, 0, sizeof g_buf);
}
void tearDown(void) { pb_test_teardown(); }

/* A clean sweep: six wired channels with distinct values, and a canary that matches none. */
static void fresh_sweep(void) {
  for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) sim_set_channel(ch, (uint16_t)(8000 + ch));
  sim_set_channel(PB_CANARY_CHANNEL, 1);
  TEST_ASSERT_TRUE(sensors_sweep());
}

static uint16_t build(void) { report_stamp(); return report_build(g_buf, sizeof g_buf); }

static void test_report_carries_c_t_and_the_valid_channels(void) {
  fresh_sweep();
  TEST_ASSERT_TRUE(build() > 0);
  char t[32];
  snprintf(t, sizeof t, "t=%lu", (unsigned long)report_t_wire());
  char c[16];
  snprintf(c, sizeof c, "c=%u", (unsigned)PB_CONTROLLER);
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, c));
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, t));
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, "ch0=8000"));
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, "ch5=8005"));
  TEST_ASSERT_EQUAL_CHAR('\n', g_buf[strlen(g_buf) - 1]);
}

static void test_report_always_carries_at_least_one_diagnostic_channel(void) {
  sim_set_i2c_fail(true);                 /* a wedged bus empties the mux mask entirely */
  (void)sensors_sweep();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(pb_has_key(g_buf, "ch0="));
  TEST_ASSERT_TRUE(pb_has_key(g_buf, "ch203="));    /* butler 400s a report with no chN= at all */
}

static void test_report_omits_a_channel_whose_read_failed_rather_than_sending_zero(void) {
  fresh_sweep();
  sim_set_i2c_fail(true);
  (void)sensors_sweep();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(pb_has_key(g_buf, "ch2="));
  TEST_ASSERT_FALSE(pb_has_tok(g_buf, "ch2=0"));
}

static void test_report_omits_the_wired_channels_and_says_stuck_when_the_canary_matches(void) {
  for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) sim_set_channel(ch, 7777);
  sim_set_channel(PB_CANARY_CHANNEL, 7777);      /* unpowered mux / floating EN / broken S-line */
  sim_set_mux_stuck(true);
  /* every failed sweep returns false, and the report must still be legal on one */
  TEST_ASSERT_FALSE(sensors_sweep());
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(pb_has_key(g_buf, "ch0="));
  TEST_ASSERT_FALSE(pb_has_key(g_buf, "ch4="));
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, "err=stuck"));
  TEST_ASSERT_TRUE(pb_has_key(g_buf, "ch200="));
}

static void test_report_float_is_the_debounced_tank_verdict_anded_with_not_contra(void) {
  fresh_sweep();
  sim_set_float(true);
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, "float=1"));
  sim_set_float(false);
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, "float=0"));
}

/* pb_latch_contra() is a real contradicting dose, the latch's only setter, and leaves the
   float reading OK, so only the AND with !contra can be why the wire says 0. */
static void test_report_float_is_zero_under_the_contradiction_latch_even_though_the_tank_reads_ok(void) {
  fresh_sweep();
  pb_latch_contra();
  sim_set_float(true);
  TEST_ASSERT_TRUE(safety_float_ok_debounced());   /* the raw debounce alone says OK */
  TEST_ASSERT_TRUE(safety_contra());
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, "float=0"));            /* ANDed with !contra */
}

static void test_report_float_is_only_ever_zero_or_one(void) {
  fresh_sweep();
  for (int i = 0; i < 6; ++i) {                  /* a float flapping at the waterline */
    sim_set_float(i % 2 == 0);
    TEST_ASSERT_TRUE(build() > 0);
    TEST_ASSERT_TRUE(pb_has_tok(g_buf, "float=0") || pb_has_tok(g_buf, "float=1"));
    TEST_ASSERT_FALSE(pb_has_tok(g_buf, "float=2"));       /* _int_in(v,"float",0,2) is HALF-open */
    TEST_ASSERT_FALSE(pb_has_tok(g_buf, "float=-1"));
  }
}

/* The flap trips on the PB_FLOAT_FLAP_LIMIT-th consecutive refusal; a further refusal cannot
   untrip it, which is why two rows below run past the limit. */
static void trip_float_flap_(unsigned refusals) {
  for (unsigned i = 0; i < refusals; ++i) safety_float_refusal_count(true);
}

/* A dose that reaches its target: the only thing besides the counter's own clear that puts
   the flap back down. */
static void grant_a_dose_(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  pulses_begin();
  sim_set_flow_ml_s(85u);
  dose_req_t q = {0};
  q.ml = (uint16_t)PB_DOSE_RIG_MAX_ML;
  q.cap_ms = PB_DOSE_CAP_MS_MAX;
  q.long_prime = true;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_OK, dose_run(&q), "arrange: a granted dose");
}

static void latch_nothing_(void)              { }
static void latch_flap_(void)                 { trip_float_flap_(PB_FLOAT_FLAP_LIMIT); }
static void latch_flap_past_the_limit_(void)  { trip_float_flap_(PB_FLOAT_FLAP_LIMIT + 1u); }
static void latch_flap_then_the_clear_(void)  { trip_float_flap_(PB_FLOAT_FLAP_LIMIT + 1u);
                                                safety_float_refusal_count(false); }
static void latch_flap_then_a_dose_(void)     { trip_float_flap_(PB_FLOAT_FLAP_LIMIT);
                                                grant_a_dose_(); }
static void latch_dry_(void)                  { safety_dry_set(true); }
static void latch_dry_then_off_(void)         { safety_dry_set(true); safety_dry_set(false); }
static void latch_contra_(void)               { pb_latch_contra(); }
static void latch_flap_and_dry_(void)         { trip_float_flap_(PB_FLOAT_FLAP_LIMIT);
                                                safety_dry_set(true); }
static void latch_flap_and_dry_off_(void)     { trip_float_flap_(PB_FLOAT_FLAP_LIMIT);
                                                safety_dry_set(true); safety_dry_set(false); }

/* ---- the three latches on the wire: ch207 contra, ch210 the float flap, ch211 dry. Each
   reaches its own channel and no other: an emitter reading flap || contra on ch210, or
   dry || contra on ch211, or flap || dry on both, satisfies some rows and not the rest.
   Absent reads as 0 to the backend, so all three ride at 0 on a clean boot. contra and the
   flap are float= terms and force it to 0 while they stand; the dry latch is no float= term
   at all -- it rides beside float=1, and is what forces pos=unknown instead. Every row leaves
   the tank reading OK, so a 0 in the float column can only be a latch. ---- */
static void test_each_latch_reaches_its_own_channel_and_only_two_of_them_move_float(void) {
  static const struct {
    const char *why;
    void (*arrange)(void);
    int contra, flap, dry, wire_float;
  } rows[] = {
    { "a clean boot: no latch stands",                     latch_nothing_,             0,0,0, 1 },
    { "the flap, on the limit-th consecutive refusal",     latch_flap_,                0,1,0, 0 },
    { "the flap, one refusal past the limit",              latch_flap_past_the_limit_, 0,1,0, 0 },
    { "the flap, dropped by the counter's own clear",      latch_flap_then_the_clear_, 0,0,0, 1 },
    { "the flap, dropped by a granted dose",               latch_flap_then_a_dose_,    0,0,0, 1 },
    { "the dry latch alone, beside a float the tank likes",latch_dry_,                 0,0,1, 1 },
    { "the dry latch released by `dry off`",               latch_dry_then_off_,        0,0,0, 1 },
    { "the contradiction alone, on a tank reading OK",     latch_contra_,              1,0,0, 0 },
    { "both latches on one report, neither masking the other", latch_flap_and_dry_,    0,1,1, 0 },
    { "`dry off` drops ch211 and leaves the flap standing",latch_flap_and_dry_off_,    0,1,0, 0 },
  };
  for (unsigned i = 0; i < sizeof rows / sizeof rows[0]; ++i) {
    tearDown(); setUp();                    /* a cold fixture per row, as a case gets */
    fresh_sweep();
    sim_set_float(true);
    rows[i].arrange();
    TEST_ASSERT_EQUAL_INT_MESSAGE(rows[i].contra, safety_contra() ? 1 : 0, rows[i].why);
    TEST_ASSERT_EQUAL_INT_MESSAGE(rows[i].flap, safety_float_flap() ? 1 : 0, rows[i].why);
    TEST_ASSERT_EQUAL_INT_MESSAGE(rows[i].dry, safety_dry() ? 1 : 0, rows[i].why);
    TEST_ASSERT_TRUE_MESSAGE(build() > 0, rows[i].why);
    char msg[PB_BODY_CAP + 96];
    snprintf(msg, sizeof msg, "%s | %s", rows[i].why, g_buf);
    char tok[16];
    snprintf(tok, sizeof tok, "ch207=%d", rows[i].contra);
    TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, tok), msg);
    snprintf(tok, sizeof tok, "ch210=%d", rows[i].flap);
    TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, tok), msg);
    snprintf(tok, sizeof tok, "ch211=%d", rows[i].dry);
    TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, tok), msg);
    snprintf(tok, sizeof tok, "float=%d", rows[i].wire_float);
    TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, tok), msg);
  }
}

static void test_report_pos_is_unknown_while_the_going_live_flag_is_set(void) {
#if PB_REPORT_POS_UNKNOWN
  fresh_sweep();
  TEST_ASSERT_EQUAL_INT(1, PB_REPORT_POS_UNKNOWN);   /* ships at 1 */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, "pos=unknown"));
  TEST_ASSERT_FALSE(pb_has_tok(g_buf, "pos=ok"));
#else
  TEST_IGNORE_MESSAGE("going-live arm: [env:native_live] sets the flag to 0; see native");
#endif
}

/* pos= is ok only while !dry, the cart knows its position and the expander is healthy: a
   pos=ok the backend trusts while dry queues doses the board refuses and acks, paging once
   per cooldown forever. Reachable only under native_live; elsewhere the flag says unknown
   before the formula, and an uncalibrated cart never knows where it is. */
static void test_report_pos_is_unknown_while_the_dry_latch_is_set(void) {
#if PB_REPORT_POS_UNKNOWN || PB_PULSES_PER_GATE == 0
  TEST_IGNORE_MESSAGE("pos=ok is unreachable here (flag on, or uncalibrated); see native_live");
#else
  fresh_sweep();
  TEST_ASSERT_TRUE(cart_begin());
  TEST_ASSERT_FALSE(cart_pos_known());
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "pos=unknown"), g_buf);   /* no home seen since boot */

  sim_set_screw_pulse_ms(2);
  sim_set_home_region(0, 40);
  sim_set_cart_at(PB_PULSES_HOME_TO_1 + 4u * PB_PULSES_PER_GATE);   /* over gate five */
  TEST_ASSERT_TRUE_MESSAGE(cart_home(), "arrange: a homed cart");
  TEST_ASSERT_TRUE(cart_pos_known());
  TEST_ASSERT_TRUE(sensors_i2c_healthy());
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "pos=ok"), g_buf);

  safety_dry_set(true);
  TEST_ASSERT_TRUE(cart_pos_known());              /* dry moves nothing: only the word changes */
  TEST_ASSERT_TRUE(sensors_i2c_healthy());
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "pos=unknown"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "ch211=1"), g_buf);
  TEST_ASSERT_FALSE_MESSAGE(pb_has_tok(g_buf, "pos=ok"), g_buf);

  safety_dry_set(false);                           /* `dry off`: the only way back */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "pos=ok"), g_buf);
#endif
}

/* The last term: PB_I2C_FAIL_LIMIT failed expander transfers take the bus unhealthy without
   moving the cart, and pos= follows. */
static void test_report_pos_is_unknown_after_the_expander_goes_unhealthy(void) {
#if PB_REPORT_POS_UNKNOWN || PB_PULSES_PER_GATE == 0
  TEST_IGNORE_MESSAGE("pos=ok is unreachable here (flag on, or uncalibrated); see native_live");
#else
  fresh_sweep();
  TEST_ASSERT_TRUE(cart_begin());
  sim_set_screw_pulse_ms(2);
  sim_set_home_region(0, 40);
  sim_set_cart_at(PB_PULSES_HOME_TO_1 + 4u * PB_PULSES_PER_GATE);
  TEST_ASSERT_TRUE_MESSAGE(cart_home(), "arrange: a homed cart");
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "pos=ok"), g_buf);

  sim_set_i2c_fail(true);
  for (uint8_t i = 0; i < PB_I2C_FAIL_LIMIT; ++i) (void)sensors_select(0);
  TEST_ASSERT_FALSE(sensors_i2c_healthy());
  TEST_ASSERT_TRUE(cart_pos_known());              /* the cart has not moved: only the bus is gone */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "pos=unknown"), g_buf);
#endif
}

static void test_report_pos_is_unknown_when_the_gate_pitch_is_uncalibrated(void) {
#if PB_PULSES_PER_GATE == 0
  fresh_sweep();
  TEST_ASSERT_EQUAL_INT(0, PB_PULSES_PER_GATE);
  TEST_ASSERT_FALSE(cart_pos_known());
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, "pos=unknown"));
#else
  TEST_IGNORE_MESSAGE("calibrated arm: PB_PULSES_PER_GATE != 0; see native");
#endif
}

static void test_report_omits_flow_ml_when_there_is_no_ack(void) {
  fresh_sweep();
  report_clear_ack();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(pb_has_key(g_buf, "ack="));
  TEST_ASSERT_FALSE(pb_has_key(g_buf, "flow_ml="));
}

static void test_report_never_emits_ack_without_flow_ml(void) {
  fresh_sweep();
  report_set_ack(17, 0, "float");        /* a refusal: flow_ml is 0, and MUST be present */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, "ack=17"));
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, "flow_ml=0"));
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, "err=float"));
}

static void test_report_never_emits_ack_zero(void) {
  fresh_sweep();
  report_set_ack(0, 0, "none");          /* ack is _int_in(v,"ack",1,2**63): 0 400s the report */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(pb_has_key(g_buf, "ack="));
}

/* No report may be built while the ack slot reads err=recv: butler would mark the command
   acked with flow_ml=0, page an under-delivery, start the cooldown and charge the daily cap,
   and only then would the board run the dose. */
static void test_report_build_refuses_while_the_ack_slot_still_reads_recv(void) {
  fresh_sweep();
  report_set_ack(23, 0, "recv");
  TEST_ASSERT_TRUE(report_ack_is_recv());
  TEST_ASSERT_FALSE(report_may_build());
  report_stamp();
  TEST_ASSERT_EQUAL_UINT16(0, report_build(g_buf, sizeof g_buf));
  report_set_ack(23, 248, "none");           /* exec_pending() overwrites the slot with
                                                the real result */
  TEST_ASSERT_TRUE(report_may_build());
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, "ack=23"));
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, "flow_ml=248"));
  TEST_ASSERT_FALSE(pb_has_tok(g_buf, "err=recv"));
}

static void test_report_t_is_unsigned_at_and_above_two_to_the_thirty_one(void) {
  fresh_sweep();
  const uint32_t targets[3] = { 0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFFu };
  for (int i = 0; i < 3; ++i) {
    /* Jump the clock, never step it 2^31 times. The -1: the stamp's own hal_millis() read
       advances the fake one step before it reads. */
    sim_set_clock_ms((uint32_t)(targets[i] - hal_boot_salt() - 1u));
    TEST_ASSERT_TRUE(build() > 0);
    TEST_ASSERT_EQUAL_UINT32(targets[i], report_t_wire());
    char t[32];
    snprintf(t, sizeof t, "t=%lu", (unsigned long)targets[i]);
    TEST_ASSERT_TRUE(pb_has_tok(g_buf, t));
    TEST_ASSERT_NULL(strstr(g_buf, "t=-"));   /* a signed conversion here 400s every
                                                 report, forever */
  }
}

static void test_report_t_differs_across_two_boots_fifteen_seconds_apart(void) {
  sim_reset(true);                    /* warm: the .noinit boot counter advances */
  sensors_begin(); fresh_sweep();
  sim_advance(15000);
  TEST_ASSERT_TRUE(build() > 0);
  const uint32_t first = report_t_wire();
  sim_reset(true);
  sensors_begin(); fresh_sweep();
  sim_advance(15000);
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_NOT_EQUAL(first, report_t_wire());   /* else butler swallows the 2nd as a retry */
}

static void test_report_never_repeats_a_key(void) {
  fresh_sweep();
  report_set_ack(9, 5, "none");
  TEST_ASSERT_TRUE(build() > 0);
  char copy[PB_BODY_CAP]; memcpy(copy, g_buf, sizeof copy);
  char *keys[48]; int nk = 0;
  for (char *tok = strtok(copy, " \n"); tok && nk < 48; tok = strtok(NULL, " \n")) {
    char *eq = strchr(tok, '=');
    TEST_ASSERT_NOT_NULL(eq);            /* every token is k=v or the whole report 400s */
    *eq = '\0';
    for (int i = 0; i < nk; ++i) TEST_ASSERT_TRUE(strcmp(keys[i], tok) != 0);
    keys[nk++] = tok;
  }
  TEST_ASSERT_TRUE(nk >= 13);
}

/* ch205 is the leak count, advanced only by pulses_leak_poll(), which loop() calls once per
   pass. A 2 kHz storm for 60 s is ~120,000 pulses, an order short of PB_DIAG_CLAMP, so storm
   in ten-second bursts, polling as loop() would. */
static void test_a_saturated_diagnostic_counter_stays_inside_max_raw(void) {
  fresh_sweep();
  pulses_leak_poll(false);               /* arm the watch (the rearm window is long past) */
  sim_flow_storm(2000);
  for (int i = 0; i < 100 && pulses_leak_count() <= (uint32_t)PB_DIAG_CLAMP; ++i) {
    sim_advance(10000);
    pulses_leak_poll(false);             /* pump OFF: every one of these pulses is a leak */
  }
  sim_flow_storm(0);
  TEST_ASSERT_TRUE_MESSAGE(pulses_leak_count() > (uint32_t)PB_DIAG_CLAMP,
                           "the leak watch never reached the clamp: is pulses_leak_poll() "
                           "being called at all?");
  TEST_ASSERT_TRUE(build() > 0);
  char clamp[24];
  snprintf(clamp, sizeof clamp, "ch205=%lu", (unsigned long)PB_DIAG_CLAMP);
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, clamp));
}

/* Every index, ch210 and ch211 included: through report_build() the latches are booleans and
   never sit above the clamp, so an emitter that skipped them passed. The block's byte count
   is config.h's " chNNN=999999" x 12 row; unclamped, ten digits make each field 17. The
   table's factor was hand-typed; this case measures it. */
static void test_every_diagnostic_channel_is_clamped_on_the_wire_the_two_latches_included(void) {
  uint32_t above[PB_DIAG_CHANNELS];
  for (uint32_t i = 0; i < (uint32_t)PB_DIAG_CHANNELS; ++i) above[i] = 0xFFFFFFFFu;
  uint16_t n = 0;
  TEST_ASSERT_TRUE(report_put_diags(g_buf, sizeof g_buf, &n, above));
  g_buf[n] = '\0';
  for (uint32_t i = 0; i < (uint32_t)PB_DIAG_CHANNELS; ++i) {
    char tok[24];
    snprintf(tok, sizeof tok, "ch%lu=%lu", (unsigned long)(200u + i), (unsigned long)PB_DIAG_CLAMP);
    TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, tok), g_buf);
  }
  TEST_ASSERT_EQUAL_UINT16((uint16_t)(PB_DIAG_CHANNELS * 13), n);   /* " chNNN=999999" x 12 */

  uint16_t unclamped = 0;                            /* " chNNN=4294967295" x 12: 17 each */
  for (uint32_t i = 0; i < (uint32_t)PB_DIAG_CHANNELS; ++i) {
    char raw[24];
    unclamped = (uint16_t)(unclamped + snprintf(raw, sizeof raw, " ch%lu=%lu",
                                                (unsigned long)(200u + i), (unsigned long)above[i]));
  }
  TEST_ASSERT_EQUAL_UINT16((uint16_t)(PB_DIAG_CHANNELS * 17), unclamped);
  TEST_ASSERT_EQUAL_UINT16((uint16_t)(PB_DIAG_CHANNELS * 4), (uint16_t)(unclamped - n));
}

/* One leaked pulse must reach the wire as both ch205 and err=leak; leak has no latch, so
   this is its only surface. */
static void test_ch205_counts_leak_pulses_and_err_leak_reaches_the_wire(void) {
  fresh_sweep();
  report_clear_ack();                    /* no ack, so err= falls through to the leak watch */
  pulses_leak_poll(false);
  sim_flow_storm(50);
  sim_advance(1000);
  sim_flow_storm(0);
  pulses_leak_poll(false);
  TEST_ASSERT_TRUE(pulses_leak_count() > 0u);
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(pb_has_tok(g_buf, "ch205=0"));
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "err=leak"), g_buf);
}

static void test_ch204_is_zero_before_d5_has_ever_changed_not_a_sentinel(void) {
  fresh_sweep();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, "ch204=0"));  /* never -1, "unknown" or "never": _int_in 400s those */
}

static void test_report_err_token_never_contains_whitespace(void) {
  static const char *every_producer[] = {
    "none","float","pos","noflow","noise","cap","stop","wdt","dry","contra","boot","range",
    "cal","i2c","busy","cooldown","leak","adc","stuck","txcap","resetmid","heap","goto","recv"
  };
  /* Whitespace-freedom, not a-z only: "i2c" is a real token and carries a digit. A space,
     tab, CR or LF is what turns one k=v token into two on the wire. */
  for (unsigned i = 0; i < sizeof every_producer / sizeof every_producer[0]; ++i)
    TEST_ASSERT_NULL(strpbrk(every_producer[i], " \t\r\n"));
  fresh_sweep();
  safety_set_err("resetmid");
  report_clear_ack();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(pb_has_tok(g_buf, "err=resetmid"));
}

static void test_report_refuses_to_send_on_truncation_and_says_txcap(void) {
  fresh_sweep();
  const uint32_t before = report_txcap_drops();
  char small[40];
  report_stamp();
  TEST_ASSERT_EQUAL_UINT16(0, report_build(small, sizeof small));
  TEST_ASSERT_EQUAL_UINT32(before + 1, report_txcap_drops());
  TEST_ASSERT_EQUAL_STRING("txcap", safety_last_err());
  /* the next body that fits clears it; nothing else ever does */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_EQUAL_STRING("none", safety_last_err());
}

/* The only heap bound that exists during the 48-hour run: the break is unchecked, nothing
   references the heap limit, and the run is when the network stack, the largest allocator,
   is active. */
static void test_a_break_inside_the_stack_margin_latches_err_heap(void) {
  fresh_sweep();
  TEST_ASSERT_TRUE(report_heap_ok());                  /* the fake starts well clear */
  sim_set_heap_break(hal_stack_limit() - (uint32_t)PB_STACK_MARGIN + 4u);
  TEST_ASSERT_FALSE(report_heap_ok());
  report_clear_ack();
  TEST_ASSERT_TRUE(build() > 0);                       /* a report saying heap beats no report */
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "err=heap"), g_buf);
}

/* Every field the host can drive to its widest at once. ch200..ch202 are constants in the
   fake, ch203 has no setter, ch206's only host writer is kept out of this file and ch208 is a
   boolean, so the diagnostic block still falls some forty bytes short of its clamp width; the
   rest of the body is at full stretch. t= is parked one step short of UINT32_MAX because the
   stamp's own clock read advances the fake first, so nothing that reads the clock may run
   between this and build(). */
static void arrange_widest_body_(void) {
  for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) sim_set_channel(ch, 16383);  /* 14-bit maximum */
  sim_set_channel(PB_CANARY_CHANNEL, 1);
  sim_set_float(false);
  TEST_ASSERT_TRUE(sensors_sweep());
  sim_set_float(true);
  TEST_ASSERT_TRUE(sensors_sweep());             /* a float change is on record: ch204 counts */

  pb_latch_contra();                             /* ch207: float OK, no flow, a real dose */
  trip_float_flap_(PB_FLOAT_FLAP_LIMIT);                                           /* ch210 */
  safety_dry_set(true);                                                            /* ch211 */

  pulses_leak_poll(false);                       /* ch205: past the clamp, pump off, as loop() polls */
  sim_flow_storm(2000);
  for (int i = 0; i < 100 && pulses_leak_count() <= (uint32_t)PB_DIAG_CLAMP; ++i) {
    sim_advance(10000);
    pulses_leak_poll(false);
  }
  sim_flow_storm(0);
  TEST_ASSERT_TRUE(pulses_leak_count() > (uint32_t)PB_DIAG_CLAMP);

  sim_wdt_rate_hz(1000000u);                     /* ch209: the counter drains to 0 inside the probe */
  (void)hal_wdt_alive();
  sim_wdt_rate_hz(2929u);
  TEST_ASSERT_TRUE(hal_wdt_last_delta() >= 10000u);   /* five digits: the whole reload */

  sim_set_clock_ms((uint32_t)(0xFFFFFFFFu - hal_boot_salt() - 1u));   /* jump, never 2^31 steps */
  report_set_ack(4294967295u, PB_DOSE_MAX_ML, "resetmid");
}

/* PB_BODY_WORST_SUM is a hand sum of every field at its widest; this builds that body and
   checks the number. The diagnostic block cannot be driven to width through report_build(),
   so the block as built is swapped for the emitter's own at twelve values above the clamp. */
static void test_report_fits_the_buffer_at_maximum_field_widths(void) {
  arrange_widest_body_();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "t=4294967295"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "ch0=16383"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "ch5=16383"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "pos=unknown"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "ack=4294967295"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "flow_ml=1000"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "err=resetmid"), g_buf);
  TEST_ASSERT_TRUE(strlen(g_buf) < PB_BODY_CAP);

  /* the diagnostic block as built: from the first diagnostic to the space before float= */
  const char *d0 = strstr(g_buf, " ch200=");
  const char *d1 = strstr(g_buf, " float=");
  TEST_ASSERT_NOT_NULL(d0);
  TEST_ASSERT_NOT_NULL(d1);
  TEST_ASSERT_TRUE(d1 > d0);
  const size_t diag_built = (size_t)(d1 - d0);

  /* the same block at its clamp width, from the emitter itself */
  uint32_t above[PB_DIAG_CHANNELS];
  for (uint32_t i = 0; i < (uint32_t)PB_DIAG_CHANNELS; ++i) above[i] = 0xFFFFFFFFu;
  uint16_t diag_worst = 0;
  TEST_ASSERT_TRUE(report_put_diags(g_blk, sizeof g_blk, &diag_worst, above));

  char c[16];
  const int cw = snprintf(c, sizeof c, "c=%u", (unsigned)PB_CONTROLLER);   /* the excluded term */
  const size_t worst = strlen(g_buf) - (size_t)cw - diag_built + diag_worst;
  TEST_ASSERT_EQUAL_UINT32((uint32_t)PB_BODY_WORST_SUM, (uint32_t)worst);
  TEST_ASSERT_TRUE(worst <= (size_t)PB_BODY_WORST_FIXED);
  TEST_ASSERT_TRUE((size_t)PB_BODY_WORST_FIXED - worst < 32u);   /* rounded up, never loosened */
  TEST_ASSERT_TRUE(PB_CONTROLLER_WIRE + 2u + PB_BODY_WORST_FIXED <= PB_BODY_CAP);
}

/* The runtime half: the widest body report_build() itself can produce from the real producers
   must reach the wire, under the cap and no longer than the table. The block falls short of
   its clamp width for the reasons the arrangement gives, so the bound is an inequality. */
static void test_report_build_fits_the_buffer_with_every_host_drivable_field_at_its_widest(void) {
  arrange_widest_body_();
  const uint16_t n = build();
  TEST_ASSERT_TRUE_MESSAGE(n > 0, "the widest host body was dropped as txcap");
  TEST_ASSERT_EQUAL_UINT16(n, (uint16_t)strlen(g_buf));
  TEST_ASSERT_TRUE(n < PB_BODY_CAP);
  char c[16];
  const int cw = snprintf(c, sizeof c, "c=%u", (unsigned)PB_CONTROLLER);
  TEST_ASSERT_TRUE_MESSAGE(n <= (size_t)cw + PB_BODY_WORST_SUM, g_buf);   /* never longer than the table */

  char clamp[24];
  snprintf(clamp, sizeof clamp, "ch204=%lu", (unsigned long)PB_DIAG_CLAMP);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, clamp), g_buf);
  snprintf(clamp, sizeof clamp, "ch205=%lu", (unsigned long)PB_DIAG_CLAMP);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, clamp), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "t=4294967295"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "ch0=16383"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "ch5=16383"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "ch207=1"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "ch210=1"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "ch211=1"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "float=0"), g_buf);
  /* the wider word on every arm: the flag forces it here, the dry latch and no home seen do
     under native_live */
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "pos=unknown"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "ack=4294967295"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "flow_ml=1000"), g_buf);
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(g_buf, "err=resetmid"), g_buf);
}

/* backend/fake_device.py's build_report() is the shape butler was written against:
   "c= t= chN=... float= pos= ack= flow_ml=", space-joined, one trailing newline. Ours adds
   ch200..ch211 and err=; strip those and the two must be byte-identical. */
static void test_report_matches_the_fake_device_shape(void) {
  fresh_sweep();
  sim_set_float(true);
  report_set_ack(17, 248, "none");
  TEST_ASSERT_TRUE(build() > 0);

  char spine[PB_BODY_CAP] = {0};
  char copy[PB_BODY_CAP]; memcpy(copy, g_buf, sizeof copy);
  for (char *tok = strtok(copy, " \n"); tok; tok = strtok(NULL, " \n")) {
    /* the diagnostic range by name: ch2= is a wired channel and starts with the same three
       characters; two prefixes, because ch210 and ch211 start "ch21" */
    if (strncmp(tok, "ch20", 4) == 0 || strncmp(tok, "ch21", 4) == 0 ||
        strncmp(tok, "err=", 4) == 0) continue;
    if (spine[0]) strncat(spine, " ", sizeof spine - strlen(spine) - 1);
    strncat(spine, tok, sizeof spine - strlen(spine) - 1);
  }
  char golden[PB_BODY_CAP];
  snprintf(golden, sizeof golden,
           "c=%u t=%lu ch0=8000 ch1=8001 ch2=8002 ch3=8003 ch4=8004 ch5=8005 "
           "float=1 pos=unknown ack=17 flow_ml=248",
           (unsigned)PB_CONTROLLER, (unsigned long)report_t_wire());
  TEST_ASSERT_EQUAL_STRING(golden, spine);
}

/* ---- response_parse(): the half of the wire where a fault becomes water ---- */

static void test_response_parses_next_only(void) {
  response_t r;
  const char *b = "next=60\n";
  TEST_ASSERT_FALSE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL_UINT16(60, r.next_s);
  TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
}

static void test_response_parses_a_water_command(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 water=3 ml=250 cap_s=30\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL_UINT16(60, r.next_s);
  TEST_ASSERT_EQUAL(CMD_WATER, r.cmd.kind);
  TEST_ASSERT_EQUAL_UINT32(17, r.cmd.id);
  TEST_ASSERT_EQUAL_UINT8(3, r.cmd.outlet);
  TEST_ASSERT_EQUAL_UINT16(250, r.cmd.ml);
  TEST_ASSERT_EQUAL_UINT16(30, r.cmd.cap_s);
}

static void test_response_parses_a_stop_command(void) {
  response_t r;
  const char *b = "next=60\ncmd=18 stop=1\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL(CMD_STOP, r.cmd.kind);
  TEST_ASSERT_EQUAL_UINT32(18, r.cmd.id);
}

static void test_response_ignores_unknown_keys(void) {
  response_t r;
  const char *b = "next=60 note=hello\ncmd=19 water=2 ml=100 cap_s=10 spare=7\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL(CMD_WATER, r.cmd.kind);
  TEST_ASSERT_EQUAL_UINT16(100, r.cmd.ml);
}

/* ml= matches only at a token start: flow_ml= is the shape the report body carries the other
   way. */
static void test_response_a_key_embedded_in_a_longer_key_is_not_matched(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 water=3 flow_ml=999 ml=250 cap_s=30\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL_UINT16(250, r.cmd.ml);   /* the real ml=, not flow_ml=999's tail */
}

/* Four shapes a corrupted body can take that each yield no command AND leave the replay mark
   where it was: a refused line must not burn the id it named, or a later well-formed response
   carrying that id would be turned away as a replay. */
static void test_a_malformed_command_line_yields_no_command_and_never_moves_the_mark(void) {
  static const struct { const char *body; const char *why; } rows[] = {
    { "next=60\ncmd=0 water=3 ml=250 cap_s=30\n",
      "cmd=0 is not an id" },
    { "next=60\ncmd=21\n",
      "neither water= nor stop=1: a shape butler never sends and a corrupted body might, and "
      "neither branch's field checks succeed" },
    { "next=60\ncmd=17 water=256 ml=250 cap_s=30\n",
      "an outlet too wide for outlet's own uint8_t: 256 cast down is 0, a LEGAL-looking "
      "outlet rather than the obviously-bogus field it was" },
    { "next=60\ncmd=17 water=4294967297 ml=250 cap_s=30\n",
      "2^32 + 1 wraps modulo 2^32 to outlet=1 under a rounded-down single-threshold guard; "
      "field_u32's per-digit check is what refuses it instead" },
  };
  response_t r;
  for (unsigned i = 0; i < sizeof rows / sizeof rows[0]; ++i) {
    TEST_ASSERT_FALSE_MESSAGE(response_parse(rows[i].body, (uint16_t)strlen(rows[i].body), &r),
                              rows[i].why);
    TEST_ASSERT_EQUAL_MESSAGE(CMD_NONE, r.cmd.kind, rows[i].why);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, g_nv.cmd_high_water, rows[i].why);
  }
}

static void test_response_rejects_a_repeated_or_lower_command_id(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 water=3 ml=250 cap_s=30\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL_UINT32(17, g_nv.cmd_high_water);
  TEST_ASSERT_FALSE(response_parse(b, (uint16_t)strlen(b), &r));   /* the same body again */
  const char *lower = "next=60\ncmd=9 water=3 ml=250 cap_s=30\n";
  TEST_ASSERT_FALSE(response_parse(lower, (uint16_t)strlen(lower), &r));
  const char *higher = "next=60\ncmd=18 water=3 ml=250 cap_s=30\n";
  TEST_ASSERT_TRUE(response_parse(higher, (uint16_t)strlen(higher), &r));
}

static void test_response_rejects_water_without_ml_or_without_cap_s(void) {
  response_t r;
  const char *no_ml = "next=60\ncmd=17 water=3 cap_s=30\n";
  TEST_ASSERT_FALSE(response_parse(no_ml, (uint16_t)strlen(no_ml), &r));
  const char *no_cap = "next=60\ncmd=17 water=3 ml=250\n";     /* an absent cap is unbounded */
  TEST_ASSERT_FALSE(response_parse(no_cap, (uint16_t)strlen(no_cap), &r));
  TEST_ASSERT_EQUAL_UINT32(0, g_nv.cmd_high_water);
}

static void test_response_rejects_ml_zero(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 water=3 ml=0 cap_s=30\n";
  TEST_ASSERT_FALSE(response_parse(b, (uint16_t)strlen(b), &r));
}

static void test_response_truncated_body_yields_no_command(void) {
  response_t r;
  const char *full = "next=60\ncmd=17 water=3 ml=250 cap_s=30\n";
  for (uint16_t cut = 9; cut < strlen(full); ++cut) {        /* every mid-token truncation */
    TEST_ASSERT_FALSE(response_parse(full, cut, &r));
    TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
  }
  TEST_ASSERT_EQUAL_UINT32(0, g_nv.cmd_high_water);
}

static void test_response_next_out_of_range_keeps_the_previous_interval(void) {
  response_t r;
  const char *lo = "next=4\n";
  TEST_ASSERT_FALSE(response_parse(lo, (uint16_t)strlen(lo), &r));
  TEST_ASSERT_EQUAL_UINT16(0, r.next_s);                    /* 0 == keep what we had */
  const char *hi = "next=3601\n";
  TEST_ASSERT_FALSE(response_parse(hi, (uint16_t)strlen(hi), &r));
  TEST_ASSERT_EQUAL_UINT16(0, r.next_s);
  const char *edge_lo = "next=5\n";
  TEST_ASSERT_FALSE(response_parse(edge_lo, (uint16_t)strlen(edge_lo), &r));
  TEST_ASSERT_EQUAL_UINT16(5, r.next_s);
  const char *edge_hi = "next=3600\n";
  TEST_ASSERT_FALSE(response_parse(edge_hi, (uint16_t)strlen(edge_hi), &r));
  TEST_ASSERT_EQUAL_UINT16(3600, r.next_s);
}


/* response_parse only ever sees the body, so "a header with no body" is len == 0. NULL with
   len 0 passes without the guard (the loop bound stops it); NULL with a nonzero len is the
   one that must not walk into memchr. */
static void test_response_empty_or_null_body_yields_no_command(void) {
  response_t r;
  TEST_ASSERT_FALSE(response_parse("", 0, &r));
  TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
  TEST_ASSERT_EQUAL_UINT16(0, r.next_s);
  TEST_ASSERT_FALSE(response_parse(NULL, 0, &r));
  TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
  TEST_ASSERT_FALSE(response_parse(NULL, 40, &r));   /* a length with no buffer to match */
  TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
}

/* Butler always sends next= first, but a body missing it entirely must not refuse the
   command that follows -- next_s simply stays 0 ("keep the previous interval"). */
static void test_response_body_with_no_next_line_still_parses_the_command(void) {
  response_t r;
  const char *b = "cmd=17 water=3 ml=250 cap_s=30\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL_UINT16(0, r.next_s);
  TEST_ASSERT_EQUAL(CMD_WATER, r.cmd.kind);
}

/* field_u32 requires the first character after '=' to be a digit -- a leading '-' or letter
   fails that test immediately, so ml= reads as ABSENT, not as some salvaged magnitude. */
static void test_response_rejects_negative_or_non_numeric_ml(void) {
  response_t r;
  const char *neg = "next=60\ncmd=17 water=3 ml=-5 cap_s=30\n";
  TEST_ASSERT_FALSE(response_parse(neg, (uint16_t)strlen(neg), &r));
  const char *nan = "next=60\ncmd=17 water=3 ml=abc cap_s=30\n";
  TEST_ASSERT_FALSE(response_parse(nan, (uint16_t)strlen(nan), &r));
  TEST_ASSERT_EQUAL_UINT32(0, g_nv.cmd_high_water);
}

/* An outlet outside 1..PB_OUTLETS is accepted here at either end -- water=0 below the gates
   and a number well above them -- because it fits outlet's own field and nothing here drives
   hardware. exec_pending() refuses both with err=range above cart_goto(), so the backend
   learns the real reason. The ids ascend: the mark moves on the first row. */
static void test_response_an_outlet_outside_the_real_gates_is_accepted_structurally(void) {
  TEST_ASSERT_TRUE_MESSAGE(PB_OUTLETS < 200, "fixture assumes PB_OUTLETS stays small");
  static const struct { const char *body; uint8_t outlet; } rows[] = {
    { "next=60\ncmd=17 water=0 ml=250 cap_s=30\n",     0 },
    { "next=60\ncmd=18 water=200 ml=250 cap_s=30\n", 200 },
  };
  response_t r;
  for (unsigned i = 0; i < sizeof rows / sizeof rows[0]; ++i) {
    TEST_ASSERT_TRUE_MESSAGE(response_parse(rows[i].body, (uint16_t)strlen(rows[i].body), &r),
                             rows[i].body);
    TEST_ASSERT_EQUAL_MESSAGE(CMD_WATER, r.cmd.kind, rows[i].body);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(rows[i].outlet, r.cmd.outlet, rows[i].body);
  }
}

/* The same width rule, the other two fields: a value well within uint32_t (no overflow, so
   field_u32 itself is not what stops it) but too wide for ml/cap_s's own uint16_t must be
   refused here, not truncated by the (uint16_t) cast into a small, wrong, accepted number. */
static void test_response_rejects_ml_or_cap_s_too_wide_for_their_fields(void) {
  response_t r;
  const char *wide_ml = "next=60\ncmd=17 water=3 ml=70000 cap_s=30\n";
  TEST_ASSERT_FALSE(response_parse(wide_ml, (uint16_t)strlen(wide_ml), &r));
  TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
  const char *wide_cap = "next=60\ncmd=17 water=3 ml=250 cap_s=70000\n";
  TEST_ASSERT_FALSE(response_parse(wide_cap, (uint16_t)strlen(wide_cap), &r));
  TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
  TEST_ASSERT_EQUAL_UINT32(0, g_nv.cmd_high_water);
}

/* "ml=250x" must not read as ml=250 with the tail ignored: the whole field is absent. */
static void test_response_a_trailing_non_digit_does_not_truncate_to_a_smaller_number(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 water=3 ml=250x cap_s=30\n";
  TEST_ASSERT_FALSE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
}

/* Butler only ever sends stop=1. stop=0 must not silently fall through as a water command
   either (there is no water= on the line) -- it is simply nothing. */
static void test_response_stop_zero_is_neither_stop_nor_water(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 stop=0\n";
  TEST_ASSERT_FALSE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
}

/* Two cmd= tokens on ONE line: field_u32 returns the FIRST match it finds scanning left to
   right, so the first id wins deterministically and the second is inert, exactly like any
   other unrecognised token on the line -- never a double-parse, never the larger of the two. */
static void test_response_two_cmd_fields_on_one_line_the_first_wins(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 cmd=99 water=3 ml=250 cap_s=30\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL_UINT32(17, r.cmd.id);
}

/* A replayed id on one line does not abort the whole parse -- it disqualifies only that
   line, and the loop tries the next one. This is what makes the replay guard survive a body
   that (through some future bug, or a poisoned AT session's leftover bytes) carries a stale
   command ahead of a fresh one: the stale line is skipped, never re-executed, and the fresh
   one is still reachable in the SAME call. */
static void test_response_skips_a_replayed_line_and_accepts_a_fresh_one_after_it(void) {
  response_t r;
  const char *first = "next=60\ncmd=17 water=3 ml=250 cap_s=30\n";
  TEST_ASSERT_TRUE(response_parse(first, (uint16_t)strlen(first), &r));
  TEST_ASSERT_EQUAL_UINT32(17, g_nv.cmd_high_water);

  const char *both = "next=60\ncmd=17 water=3 ml=250 cap_s=30\ncmd=18 water=2 ml=100 cap_s=10\n";
  TEST_ASSERT_TRUE(response_parse(both, (uint16_t)strlen(both), &r));
  TEST_ASSERT_EQUAL_UINT32(18, r.cmd.id);
  TEST_ASSERT_EQUAL_UINT8(2, r.cmd.outlet);
  TEST_ASSERT_EQUAL_UINT32(18, g_nv.cmd_high_water);
}

/* The exact representable boundary (2^32 - 1, UINT32_MAX) is legal for a uint32_t field and
   must still parse: field_u32's per-digit overflow guard must reject everything ABOVE the
   boundary without also rejecting the boundary itself. cmd.id has no narrower width check
   (unlike outlet/ml/cap_s), so it is the field that isolates this from the width-truncation
   rule proved separately above. */
static void test_response_the_exact_uint32_boundary_still_parses(void) {
  response_t r;
  const char *b = "next=60\ncmd=4294967295 water=3 ml=250 cap_s=30\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL_UINT32(4294967295u, r.cmd.id);
  TEST_ASSERT_EQUAL_UINT32(4294967295u, g_nv.cmd_high_water);
}

/* cap_s and PB_DOSE_CAP_MS_MAX are two ceilings with two owners. The parser carries cap_s
   through untouched and leaves the clamp to the dose ladder: a buggy backend cannot widen the
   firmware's cap, and there is one clamp to keep, not two. */
static void test_response_carries_cap_s_through_unclamped(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 water=3 ml=250 cap_s=5000\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL_UINT16(5000, r.cmd.cap_s);
  TEST_ASSERT_TRUE_MESSAGE(5000u * 1000u > (uint32_t)PB_DOSE_CAP_MS_MAX,
                            "fixture must exceed the firmware's own cap to prove nothing here "
                            "narrows it");
}

/* A warm reset (watchdog, RESET button, a reset mid-dose) must not reopen the replay window,
   which holds only if the mark's bump is committed; otherwise the next boot's checksum fails
   and reads the block as cold. */
static void test_response_cmd_high_water_survives_a_warm_reset(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 water=3 ml=250 cap_s=30\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL_UINT32(17, g_nv.cmd_high_water);

  sim_reset(true);                                    /* warm: watchdog or RESET, not power */
  TEST_ASSERT_FALSE_MESSAGE(noinit_was_cold(), "a warm reset must not be read as a cold one");
  TEST_ASSERT_EQUAL_UINT32(17, g_nv.cmd_high_water);   /* the mark survived the reset */
  TEST_ASSERT_FALSE(response_parse(b, (uint16_t)strlen(b), &r));   /* still a replay */
}

static void test_every_canned_body_declares_its_own_true_content_length(void) {
  const char *const raw[] = { k_cmd_200, k_stop_200, k_out_of_range_200 };
  for (unsigned i = 0; i < 3u; ++i) {
    const char *hdr  = strstr(raw[i], "Content-Length: ");
    const char *body = strstr(raw[i], "\r\n\r\n") + 4;
    TEST_ASSERT_NOT_NULL(hdr);
    TEST_ASSERT_EQUAL_UINT32((uint32_t)strtoul(hdr + 16, NULL, 10), (uint32_t)strlen(body));
  }
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_every_canned_body_declares_its_own_true_content_length);
  RUN_TEST(test_report_carries_c_t_and_the_valid_channels);
  RUN_TEST(test_report_always_carries_at_least_one_diagnostic_channel);
  RUN_TEST(test_report_omits_a_channel_whose_read_failed_rather_than_sending_zero);
  RUN_TEST(test_report_omits_the_wired_channels_and_says_stuck_when_the_canary_matches);
  RUN_TEST(test_report_float_is_the_debounced_tank_verdict_anded_with_not_contra);
  RUN_TEST(test_report_float_is_zero_under_the_contradiction_latch_even_though_the_tank_reads_ok);
  RUN_TEST(test_report_float_is_only_ever_zero_or_one);
  RUN_TEST(test_each_latch_reaches_its_own_channel_and_only_two_of_them_move_float);
  RUN_TEST(test_report_pos_is_unknown_while_the_going_live_flag_is_set);
  RUN_TEST(test_report_pos_is_unknown_while_the_dry_latch_is_set);
  RUN_TEST(test_report_pos_is_unknown_after_the_expander_goes_unhealthy);
  RUN_TEST(test_report_pos_is_unknown_when_the_gate_pitch_is_uncalibrated);
  RUN_TEST(test_report_omits_flow_ml_when_there_is_no_ack);
  RUN_TEST(test_report_never_emits_ack_without_flow_ml);
  RUN_TEST(test_report_never_emits_ack_zero);
  RUN_TEST(test_report_build_refuses_while_the_ack_slot_still_reads_recv);
  RUN_TEST(test_report_t_is_unsigned_at_and_above_two_to_the_thirty_one);
  RUN_TEST(test_report_t_differs_across_two_boots_fifteen_seconds_apart);
  RUN_TEST(test_report_never_repeats_a_key);
  RUN_TEST(test_a_saturated_diagnostic_counter_stays_inside_max_raw);
  RUN_TEST(test_every_diagnostic_channel_is_clamped_on_the_wire_the_two_latches_included);
  RUN_TEST(test_ch205_counts_leak_pulses_and_err_leak_reaches_the_wire);
  RUN_TEST(test_ch204_is_zero_before_d5_has_ever_changed_not_a_sentinel);
  RUN_TEST(test_report_err_token_never_contains_whitespace);
  RUN_TEST(test_report_refuses_to_send_on_truncation_and_says_txcap);
  RUN_TEST(test_a_break_inside_the_stack_margin_latches_err_heap);
  RUN_TEST(test_report_fits_the_buffer_at_maximum_field_widths);
  RUN_TEST(test_report_build_fits_the_buffer_with_every_host_drivable_field_at_its_widest);
  RUN_TEST(test_report_matches_the_fake_device_shape);
  RUN_TEST(test_response_parses_next_only);
  RUN_TEST(test_response_parses_a_water_command);
  RUN_TEST(test_response_parses_a_stop_command);
  RUN_TEST(test_response_ignores_unknown_keys);
  RUN_TEST(test_response_a_key_embedded_in_a_longer_key_is_not_matched);
  RUN_TEST(test_a_malformed_command_line_yields_no_command_and_never_moves_the_mark);
  RUN_TEST(test_response_rejects_a_repeated_or_lower_command_id);
  RUN_TEST(test_response_rejects_water_without_ml_or_without_cap_s);
  RUN_TEST(test_response_rejects_ml_zero);
  RUN_TEST(test_response_truncated_body_yields_no_command);
  RUN_TEST(test_response_next_out_of_range_keeps_the_previous_interval);
  RUN_TEST(test_response_empty_or_null_body_yields_no_command);
  RUN_TEST(test_response_body_with_no_next_line_still_parses_the_command);
  RUN_TEST(test_response_rejects_negative_or_non_numeric_ml);
  RUN_TEST(test_response_an_outlet_outside_the_real_gates_is_accepted_structurally);
  RUN_TEST(test_response_rejects_ml_or_cap_s_too_wide_for_their_fields);
  RUN_TEST(test_response_a_trailing_non_digit_does_not_truncate_to_a_smaller_number);
  RUN_TEST(test_response_stop_zero_is_neither_stop_nor_water);
  RUN_TEST(test_response_two_cmd_fields_on_one_line_the_first_wins);
  RUN_TEST(test_response_skips_a_replayed_line_and_accepts_a_fresh_one_after_it);
  RUN_TEST(test_response_the_exact_uint32_boundary_still_parses);
  RUN_TEST(test_response_carries_cap_s_through_unclamped);
  RUN_TEST(test_response_cmd_high_water_survives_a_warm_reset);
  return UNITY_END();
}
