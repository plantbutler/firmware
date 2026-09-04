/* test/test_report/test_report.cpp — report_build() and response_parse(). */
#include <unity.h>
#include <stdio.h>
#include <string.h>
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

void setUp(void)    { pb_test_setup(); sensors_begin(); memset(g_buf, 0, sizeof g_buf); }
void tearDown(void) { pb_test_teardown(); }

/* A clean sweep: six wired channels with distinct values, and a canary that matches none. */
static void fresh_sweep(void) {
  for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) sim_set_channel(ch, (uint16_t)(8000 + ch));
  sim_set_channel(PB_CANARY_CHANNEL, 1);
  TEST_ASSERT_TRUE(sensors_sweep());
}

static uint16_t build(void) { report_stamp(); return report_build(g_buf, sizeof g_buf); }

/* whole-token match: "ch1=8001" must not be found inside "ch11=8001" */
static bool has_tok(const char *tok) {
  size_t n = strlen(tok);
  for (const char *p = strstr(g_buf, tok); p; p = strstr(p + n, tok)) {
    bool left  = (p == g_buf) || p[-1] == ' ';
    bool right = (p[n] == ' ' || p[n] == '\n' || p[n] == '\0');
    if (left && right) return true;
  }
  return false;
}

static bool has_key(const char *key) {   /* key includes the '=' */
  size_t n = strlen(key);
  for (const char *p = strstr(g_buf, key); p; p = strstr(p + n, key))
    if (p == g_buf || p[-1] == ' ') return true;
  return false;
}

static void test_report_carries_c_t_and_the_valid_channels(void) {
  fresh_sweep();
  TEST_ASSERT_TRUE(build() > 0);
  char t[32];
  snprintf(t, sizeof t, "t=%lu", (unsigned long)report_t_wire());
  TEST_ASSERT_TRUE(has_tok("c=" PB_CONTROLLER));
  TEST_ASSERT_TRUE(has_tok(t));
  TEST_ASSERT_TRUE(has_tok("ch0=8000"));
  TEST_ASSERT_TRUE(has_tok("ch5=8005"));
  TEST_ASSERT_EQUAL_CHAR('\n', g_buf[strlen(g_buf) - 1]);
}

static void test_report_always_carries_at_least_one_diagnostic_channel(void) {
  sim_set_i2c_fail(true);                 /* a wedged bus empties the mux mask entirely */
  (void)sensors_sweep();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(has_key("ch0="));
  TEST_ASSERT_TRUE(has_key("ch203="));    /* butler 400s a report with no chN= at all */
}

static void test_report_omits_a_channel_whose_read_failed_rather_than_sending_zero(void) {
  fresh_sweep();
  sim_set_i2c_fail(true);
  (void)sensors_sweep();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(has_key("ch2="));
  TEST_ASSERT_FALSE(has_tok("ch2=0"));
}

static void test_report_omits_the_wired_channels_and_says_stuck_when_the_canary_matches(void) {
  for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) sim_set_channel(ch, 7777);
  sim_set_channel(PB_CANARY_CHANNEL, 7777);      /* unpowered mux / floating EN / broken S-line */
  sim_set_mux_stuck(true);
  /* FALSE, not TRUE: task 7's contract is that every failure returns false, and the canary
     matching every wired channel is one. The report must still be LEGAL on a false sweep -
     that is the whole point of the diagnostics - which is what the assertions below check. */
  TEST_ASSERT_FALSE(sensors_sweep());
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(has_key("ch0="));
  TEST_ASSERT_FALSE(has_key("ch4="));
  TEST_ASSERT_TRUE(has_tok("err=stuck"));
  TEST_ASSERT_TRUE(has_key("ch200="));
}

static void test_report_float_is_the_debounced_tank_verdict_anded_with_not_contra(void) {
  fresh_sweep();
  sim_set_float(true);
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("float=1"));
  sim_set_float(false);
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("float=0"));
}

/* The "anded_with_not_contra" half of the test above it: despite its name, that case never
   latches the contradiction and so never proves the AND term at all -- found by mutating
   report.cpp's `fl` expression to drop `&& !safety_contra()` entirely and re-running this
   whole file: nothing failed. pb_latch_contra() (harness.h) drives a REAL dose_run() call
   with the float OK and no flow, the only way §2.7's latch is ever set (there is
   deliberately no setter), and leaves the float pin reading OK afterward — so the raw
   debounce alone would say float=1, and only the AND with !safety_contra() can be the
   reason the wire still says 0. */
static void test_report_float_is_zero_under_the_contradiction_latch_even_though_the_tank_reads_ok(void) {
  fresh_sweep();
  pb_latch_contra();
  sim_set_float(true);
  TEST_ASSERT_TRUE(safety_float_ok_debounced());   /* the raw debounce alone says OK */
  TEST_ASSERT_TRUE(safety_contra());
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("float=0"));            /* §2.10, §4.1: ANDed with !contra */
}

static void test_report_float_is_only_ever_zero_or_one(void) {
  fresh_sweep();
  for (int i = 0; i < 6; ++i) {                  /* a float flapping at the waterline */
    sim_set_float(i % 2 == 0);
    TEST_ASSERT_TRUE(build() > 0);
    TEST_ASSERT_TRUE(has_tok("float=0") || has_tok("float=1"));
    TEST_ASSERT_FALSE(has_tok("float=2"));       /* _int_in(v,"float",0,2) is HALF-open */
    TEST_ASSERT_FALSE(has_tok("float=-1"));
  }
}

static void test_repeated_float_refusals_drive_float_to_zero_on_the_wire(void) {
  fresh_sweep();
  sim_set_float(true);
  for (int i = 0; i < PB_FLOAT_FLAP_LIMIT + 1; ++i) safety_float_refusal_count(true);
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("float=0"));          /* even though the tank samples OK */
}

static void test_a_granted_dose_clears_the_float_refusal_counter(void) {
  fresh_sweep();
  sim_set_float(true);
  for (int i = 0; i < PB_FLOAT_FLAP_LIMIT + 1; ++i) safety_float_refusal_count(true);
  safety_float_refusal_count(false);             /* any granted dose clears it */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("float=1"));
}

static void test_report_pos_is_unknown_while_the_going_live_flag_is_set(void) {
  fresh_sweep();
  TEST_ASSERT_EQUAL_INT(1, PB_REPORT_POS_UNKNOWN);   /* ships defined — §4.6 */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("pos=unknown"));
  TEST_ASSERT_FALSE(has_tok("pos=ok"));
}

static void test_report_pos_is_unknown_while_the_dry_latch_is_set(void) {
  fresh_sweep();
  safety_dry_set(true);
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("pos=unknown"));
  safety_dry_set(false);
}

static void test_report_pos_is_unknown_when_the_gate_pitch_is_uncalibrated(void) {
#if PB_PULSES_PER_GATE == 0
  fresh_sweep();
  TEST_ASSERT_EQUAL_INT(0, PB_PULSES_PER_GATE);      /* bring-up 6 has not run */
  TEST_ASSERT_FALSE(cart_pos_known());
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("pos=unknown"));
#else
  /* [env:native_cal] (task 14) builds this whole tree a second time with
     PB_PULSES_PER_GATE=1450, the same idiom test_cart.cpp and test_dose.cpp already use
     throughout for a case that is only meaningful on the uncalibrated arm. Found by running
     `pio test -e native_cal -f test_report`: this case is about proving pos=unknown holds
     WHILE the gate pitch is uncalibrated, and under native_cal it no longer is. */
  TEST_IGNORE_MESSAGE("calibrated arm: PB_PULSES_PER_GATE != 0; see native");
#endif
}

static void test_report_omits_flow_ml_when_there_is_no_ack(void) {
  fresh_sweep();
  report_clear_ack();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(has_key("ack="));
  TEST_ASSERT_FALSE(has_key("flow_ml="));
}

static void test_report_never_emits_ack_without_flow_ml(void) {
  fresh_sweep();
  report_set_ack(17, 0, "float");        /* a refusal: flow_ml is 0, and MUST be present */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("ack=17"));
  TEST_ASSERT_TRUE(has_tok("flow_ml=0"));
  TEST_ASSERT_TRUE(has_tok("err=float"));
}

static void test_report_never_emits_ack_zero(void) {
  fresh_sweep();
  report_set_ack(0, 0, "none");          /* ack is _int_in(v,"ack",1,2**63): 0 400s the report */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(has_key("ack="));
}

static void test_report_ack_id_survives_above_sixty_five_thousand(void) {
  fresh_sweep();
  report_set_ack(4294967295u, 1000, "none");
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("ack=4294967295"));
  TEST_ASSERT_TRUE(has_tok("flow_ml=1000"));
}

/* §4.3: "No report may be built while the ack slot still carries err=recv." netfsm sets the
   slot to (id, flow_ml=0, err="recv") the MOMENT response_parse() yields a command (task 24's
   job, out of this file's reach), but the refusal itself lives in report_build() via
   report_may_build()/report_ack_is_recv() (this file, step 12) and is directly testable here
   without netfsm existing yet. Found genuinely uncovered by mutation: deleting the
   `if (!report_may_build()) return 0;` guard at the top of report_build() left every other
   case in this file passing. Without this guard butler would mark the command acked with
   flow_ml=0 (`:829-833`), page a HIGH "the meter counted 0 of N ml" (`:1367-1369`), set the
   pot's cooldown from acked_ts (`:736-742`) and charge 0 ml against the daily cap - and only
   THEN would the board run the dose it had already told the backend it refused. */
static void test_report_build_refuses_while_the_ack_slot_still_reads_recv(void) {
  fresh_sweep();
  report_set_ack(23, 0, "recv");
  TEST_ASSERT_TRUE(report_ack_is_recv());
  TEST_ASSERT_FALSE(report_may_build());
  report_stamp();
  TEST_ASSERT_EQUAL_UINT16(0, report_build(g_buf, sizeof g_buf));
  report_set_ack(23, 248, "none");           /* exec_pending() overwrites the slot with the
                                                 real result (§4.3 step 2) */
  TEST_ASSERT_TRUE(report_may_build());
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("ack=23"));
  TEST_ASSERT_TRUE(has_tok("flow_ml=248"));
  TEST_ASSERT_FALSE(has_tok("err=recv"));
}

static void test_report_t_is_unsigned_at_and_above_two_to_the_thirty_one(void) {
  fresh_sweep();
  const uint32_t targets[3] = { 0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFFu };
  for (int i = 0; i < 3; ++i) {
    /* JUMP the clock; do NOT sim_advance() 2^31 times. sim_set_clock_ms() is step 13's
       injector and it deliberately does not run the edge emitters.
       -1: hal_millis() ALWAYS advances the fake by one step before it reads (sim.h's clock
       contract), and report_stamp()'s one call is what report_t_ms()/report_t_wire() are
       built from — so arming the clock at exactly the target leaves the stamped value one
       past it. Confirmed by running this case unadjusted: it failed EQUAL_UINT32 by
       exactly +1 at every one of the three targets, never by more, which is what a
       one-off-by-the-single-hal_millis()-call bug looks like and what a genuine
       hal_boot_salt() defect would not. */
    sim_set_clock_ms((uint32_t)(targets[i] - hal_boot_salt() - 1u));
    TEST_ASSERT_TRUE(build() > 0);
    TEST_ASSERT_EQUAL_UINT32(targets[i], report_t_wire());
    char t[32];
    snprintf(t, sizeof t, "t=%lu", (unsigned long)targets[i]);
    TEST_ASSERT_TRUE(has_tok(t));
    TEST_ASSERT_NULL(strstr(g_buf, "t=-"));   /* a single %d here 400s EVERY report, forever */
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

/* ch205 is pulses_leak_count(), and NOTHING advances it except pulses_leak_poll() — which
   loop() calls once per pass (task 12 step 4) and no test harness calls for free. So the case
   has to drive the poller itself, exactly as loop() does, and it has to reach PB_DIAG_CLAMP:
   a 2 kHz storm for 60 s is ~120,000 pulses, an order of magnitude short. Storm the meter in
   ten-second bursts, polling as loop() would, until the count is past the clamp. */
static void test_a_saturated_diagnostic_counter_stays_inside_max_raw(void) {
  fresh_sweep();
  /* pb_test_teardown() (harness.h) resets g_leak_count via pulses_test_reset_leak_() at the
     end of EVERY case, so this one starts clean without depending on running first. */
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
  TEST_ASSERT_TRUE(has_tok(clamp));
}

/* The same producer, at the other end of its range: one leaked pulse must reach the wire as
   BOTH ch205 and err=leak. §4.1 carries `leak` in its fixed enum and §1 says there is no
   latch, so this is the only surface the token has. */
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
  TEST_ASSERT_FALSE(has_tok("ch205=0"));
  TEST_ASSERT_TRUE_MESSAGE(has_tok("err=leak"), g_buf);
}

static void test_ch204_is_zero_before_d5_has_ever_changed_not_a_sentinel(void) {
  fresh_sweep();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("ch204=0"));  /* never -1, "unknown" or "never": _int_in 400s those */
}

static void test_report_err_token_never_contains_whitespace(void) {
  static const char *every_producer[] = {
    "none","float","pos","noflow","noise","cap","stop","wdt","dry","contra","boot","range",
    "cal","i2c","busy","cooldown","leak","adc","stuck","txcap","resetmid","heap","goto","recv"
  };
  /* The wire requirement is whitespace-freedom, not a-z-only: "i2c" is a real token in this
     very enum (spec §4.1, and cart_err()'s own "i2c") and contains a digit. A brief draft of
     this case asserted every character was 'a'..'z', which 400s "i2c" against the spec that
     put it in the enum -- caught by running it: the loop aborted at i2c's '2' before ever
     reaching the fresh_sweep()/build() half of the case below, so THIS half of the case had
     never actually run under `pio test`. strpbrk() alone is the real, sufficient check: a
     space, tab, CR or LF is what turns one k=v token into two on the wire. */
  for (unsigned i = 0; i < sizeof every_producer / sizeof every_producer[0]; ++i)
    TEST_ASSERT_NULL(strpbrk(every_producer[i], " \t\r\n"));
  fresh_sweep();
  /* pb_test_teardown() resets g_leak_count between every case (see harness.h), so a leak
     storm from an EARLIER test in this binary cannot make pulses_leak_seen() true here and
     mask err=resetmid behind err=leak via err='s precedence. */
  safety_set_err("resetmid");
  report_clear_ack();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("err=resetmid"));
}

static void test_report_refuses_to_send_on_truncation_and_says_txcap(void) {
  fresh_sweep();
  const uint32_t before = report_txcap_drops();
  char small[40];
  report_stamp();
  TEST_ASSERT_EQUAL_UINT16(0, report_build(small, sizeof small));
  TEST_ASSERT_EQUAL_UINT32(before + 1, report_txcap_drops());
  TEST_ASSERT_EQUAL_STRING("txcap", safety_last_err());
  /* ...and the NEXT body that fits clears it. Nothing else in the program ever does, so one
     384-byte report would otherwise put err=txcap on every later report forever. */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_EQUAL_STRING("none", safety_last_err());
}

/* Spec §12 item 0: "hal_begin() and EVERY REPORT check the break against the stack, because
   nothing else will." _sbrk is unchecked and __HeapLimit is referenced by nothing in the
   image, so this is the only bound that exists during the 48-hour run - and the run is
   exactly when the network stack, the largest allocator in the program, is active. */
static void test_a_break_inside_the_stack_margin_latches_err_heap(void) {
  fresh_sweep();
  /* pb_test_teardown() resets g_leak_count between every case (see harness.h): without that,
     a prior test's leak storm would mask err=heap behind err=leak on the wire here. */
  TEST_ASSERT_TRUE(report_heap_ok());                  /* the fake starts well clear */
  sim_set_heap_break(hal_stack_limit() - (uint32_t)PB_STACK_MARGIN + 4u);
  TEST_ASSERT_FALSE(report_heap_ok());
  report_clear_ack();
  TEST_ASSERT_TRUE(build() > 0);                       /* a report saying heap beats no report */
  TEST_ASSERT_TRUE_MESSAGE(has_tok("err=heap"), g_buf);
}

static void test_report_fits_the_buffer_at_maximum_field_widths(void) {
  for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) sim_set_channel(ch, 16383);  /* 14-bit maximum */
  sim_set_channel(PB_CANARY_CHANNEL, 1);
  TEST_ASSERT_TRUE(sensors_sweep());
  sim_set_clock_ms((uint32_t)(0xFFFFFFFFu - hal_boot_salt()));   /* jump, never 2^31 steps */
  report_set_ack(4294967295u, PB_DOSE_MAX_ML, "resetmid");
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(strlen(g_buf) < PB_BODY_CAP);
  TEST_ASSERT_TRUE(strlen(g_buf) <= sizeof(PB_CONTROLLER) + 2 + PB_BODY_WORST_FIXED);
}

/* backend/fake_device.py's build_report() is the shape butler was written against:
   "c= t= chN=... float= pos= ack= flow_ml=", space-joined, one trailing newline. Ours adds
   ch200..ch209 and err=; strip those and the two must be byte-identical. */
static void test_report_matches_the_fake_device_shape(void) {
  fresh_sweep();
  sim_set_float(true);
  report_set_ack(17, 248, "none");
  TEST_ASSERT_TRUE(build() > 0);

  char spine[PB_BODY_CAP] = {0};
  char copy[PB_BODY_CAP]; memcpy(copy, g_buf, sizeof copy);
  for (char *tok = strtok(copy, " \n"); tok; tok = strtok(NULL, " \n")) {
    /* The diagnostic RANGE by name, never the prefix "ch2": `ch2=8002` is a WIRED channel and
       starts with the same three characters, so a prefix filter deletes a token the golden
       string keeps and this case can never pass. */
    if (strncmp(tok, "ch20", 4) == 0 || strncmp(tok, "err=", 4) == 0) continue;
    if (spine[0]) strncat(spine, " ", sizeof spine - strlen(spine) - 1);
    strncat(spine, tok, sizeof spine - strlen(spine) - 1);
  }
  char golden[PB_BODY_CAP];
  snprintf(golden, sizeof golden,
           "c=%s t=%lu ch0=8000 ch1=8001 ch2=8002 ch3=8003 ch4=8004 ch5=8005 "
           "float=1 pos=unknown ack=17 flow_ml=248",
           PB_CONTROLLER, (unsigned long)report_t_wire());
  TEST_ASSERT_EQUAL_STRING(golden, spine);
}

/* ---- response_parse() — the half of the wire where a fault becomes water (spec §4.5) ---- */

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

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_report_carries_c_t_and_the_valid_channels);
  RUN_TEST(test_report_always_carries_at_least_one_diagnostic_channel);
  RUN_TEST(test_report_omits_a_channel_whose_read_failed_rather_than_sending_zero);
  RUN_TEST(test_report_omits_the_wired_channels_and_says_stuck_when_the_canary_matches);
  RUN_TEST(test_report_float_is_the_debounced_tank_verdict_anded_with_not_contra);
  RUN_TEST(test_report_float_is_zero_under_the_contradiction_latch_even_though_the_tank_reads_ok);
  RUN_TEST(test_report_float_is_only_ever_zero_or_one);
  RUN_TEST(test_repeated_float_refusals_drive_float_to_zero_on_the_wire);
  RUN_TEST(test_a_granted_dose_clears_the_float_refusal_counter);
  RUN_TEST(test_report_pos_is_unknown_while_the_going_live_flag_is_set);
  RUN_TEST(test_report_pos_is_unknown_while_the_dry_latch_is_set);
  RUN_TEST(test_report_pos_is_unknown_when_the_gate_pitch_is_uncalibrated);
  RUN_TEST(test_report_omits_flow_ml_when_there_is_no_ack);
  RUN_TEST(test_report_never_emits_ack_without_flow_ml);
  RUN_TEST(test_report_never_emits_ack_zero);
  RUN_TEST(test_report_ack_id_survives_above_sixty_five_thousand);
  RUN_TEST(test_report_build_refuses_while_the_ack_slot_still_reads_recv);
  RUN_TEST(test_report_t_is_unsigned_at_and_above_two_to_the_thirty_one);
  RUN_TEST(test_report_t_differs_across_two_boots_fifteen_seconds_apart);
  RUN_TEST(test_report_never_repeats_a_key);
  RUN_TEST(test_a_saturated_diagnostic_counter_stays_inside_max_raw);
  RUN_TEST(test_ch205_counts_leak_pulses_and_err_leak_reaches_the_wire);
  RUN_TEST(test_ch204_is_zero_before_d5_has_ever_changed_not_a_sentinel);
  RUN_TEST(test_report_err_token_never_contains_whitespace);
  RUN_TEST(test_report_refuses_to_send_on_truncation_and_says_txcap);
  RUN_TEST(test_a_break_inside_the_stack_margin_latches_err_heap);
  RUN_TEST(test_report_fits_the_buffer_at_maximum_field_widths);
  RUN_TEST(test_report_matches_the_fake_device_shape);
  RUN_TEST(test_response_parses_next_only);
  RUN_TEST(test_response_parses_a_water_command);
  RUN_TEST(test_response_parses_a_stop_command);
  RUN_TEST(test_response_ignores_unknown_keys);
  return UNITY_END();
}
