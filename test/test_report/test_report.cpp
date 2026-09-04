/* test/test_report/test_report.cpp — report_build() and response_parse(). */
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

/* field_u32 only matches "ml=" at the start of the line or right after a space -- without that
   guard it would also match the tail of a key like "flow_ml=", which is not a shape butler's
   response ever carries today but is exactly the shape its OWN report body carries the other
   direction (§4.1's flow_ml=). One shared parsing habit away from a real body someday. */
static void test_response_a_key_embedded_in_a_longer_key_is_not_matched(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 water=3 flow_ml=999 ml=250 cap_s=30\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL_UINT16(250, r.cmd.ml);   /* the real ml=, not flow_ml=999's tail */
}

static void test_response_rejects_command_id_zero(void) {
  response_t r;
  const char *b = "next=60\ncmd=0 water=3 ml=250 cap_s=30\n";
  TEST_ASSERT_FALSE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
  TEST_ASSERT_EQUAL_UINT32(0, g_nv.cmd_high_water);   /* and it never moves the mark */
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

/* ---- the rest of the enumeration: shapes the brief's ten cases do not cover ---- */

/* "a header with no body": at this function's boundary that is exactly len==0, since
   response_parse only ever sees the BODY (netfsm scans past the CRLFCRLF itself). A NULL
   body must be equally inert -- a truncated read that produced no buffer at all.

   NULL with len==0 alone would pass even without the `!body` guard -- the while(pos<len)
   loop bound stops it before any dereference -- so that pairing does not actually pin the
   guard down (found by mutating it away: nothing failed). NULL with a NONZERO len is the
   case that matters: a caller bug handing this function a length without a buffer to match
   must not walk into memchr(NULL, ...) and crash the one path between a fault and water. */
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

/* A cmd= with neither water= nor stop=1 is a shape butler never sends and a corrupted body
   might: neither branch's field checks succeed, so the line yields nothing and the mark
   does not move -- the id stays available for a LATER, well-formed response to use. */
static void test_response_cmd_with_no_water_or_stop_yields_no_command(void) {
  response_t r;
  const char *b = "next=60\ncmd=21\n";
  TEST_ASSERT_FALSE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
  TEST_ASSERT_EQUAL_UINT32(0, g_nv.cmd_high_water);
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

/* §4.5, verbatim: an outlet outside 1..PB_OUTLETS -- water=0 included -- is ACCEPTED here.
   exec_pending() (task 26) refuses it with err=range, above cart_goto(), so the backend
   learns the real reason instead of whichever step happened to fail first. Rejecting it
   HERE would be the more "obviously safe" instinct and would be wrong: the outlet never
   drives hardware from this function, and the backend needs the honest refusal reason. */
static void test_response_water_zero_is_accepted_structurally(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 water=0 ml=250 cap_s=30\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL(CMD_WATER, r.cmd.kind);
  TEST_ASSERT_EQUAL_UINT8(0, r.cmd.outlet);
}

/* Same point, the other side of PB_OUTLETS: an outlet that fits the uint8_t field but is
   well above the five real gates is ALSO accepted here for the identical reason. */
static void test_response_an_outlet_above_pb_outlets_is_accepted_structurally(void) {
  response_t r;
  TEST_ASSERT_TRUE_MESSAGE(PB_OUTLETS < 200, "fixture assumes PB_OUTLETS stays small");
  const char *b = "next=60\ncmd=17 water=200 ml=250 cap_s=30\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL(CMD_WATER, r.cmd.kind);
  TEST_ASSERT_EQUAL_UINT8(200, r.cmd.outlet);
}

/* An outlet that does NOT fit outlet's own uint8_t field is a different case from the two
   above and must be rejected outright here, not truncated by the (uint8_t) cast: 256 cast
   to uint8_t is 0, which is a LEGAL-looking outlet, not the obviously-bogus field it was. */
static void test_response_an_outlet_too_wide_for_the_field_yields_no_command(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 water=256 ml=250 cap_s=30\n";
  TEST_ASSERT_FALSE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
  TEST_ASSERT_EQUAL_UINT32(0, g_nv.cmd_high_water);
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

/* "ml=250x" must not be read as ml=250 with a stray trailing character ignored -- that
   would be exactly the "partial" acceptance rule 1 forbids. field_u32's trailing-character
   check makes the whole field absent instead. */
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

/* The overflow-wrap hazard field_u32's exact per-digit check exists to close: with a
   rounded-down single-threshold guard, "water=4294967297" (2^32 + 1) wraps modulo 2^32 to
   outlet=1 and is SILENTLY ACCEPTED as a legitimate small outlet -- a malformed field
   masquerading as a valid command instead of yielding no command at all. */
static void test_response_an_overflowing_numeric_field_is_rejected_not_wrapped(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 water=4294967297 ml=250 cap_s=30\n";
  TEST_ASSERT_FALSE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
  TEST_ASSERT_EQUAL_UINT32(0, g_nv.cmd_high_water);
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

/* Requirement 3: the backend's cap_s and the firmware's PB_DOSE_CAP_MS_MAX are two different
   ceilings with two different owners. response_parse decides nothing (requirement 4) -- it
   carries cap_s straight through, however large, and leaves the clamp to dose_run() (task 17,
   safety.cpp:222), which the caller reaches only after this struct is handed to exec_pending().
   A cap_s of 5000 s is nowhere near butler's own MAX_CAP_S=60, but that is exactly the point:
   a hostile or buggy backend cannot WIDEN the firmware's cap by asking for a bigger one, and
   this function proves that not by narrowing it here (there would then be two clamps to keep
   in sync) but by not touching it at all. */
static void test_response_carries_cap_s_through_unclamped(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 water=3 ml=250 cap_s=5000\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL_UINT16(5000, r.cmd.cap_s);
  TEST_ASSERT_TRUE_MESSAGE(5000u * 1000u > (uint32_t)PB_DOSE_CAP_MS_MAX,
                            "fixture must exceed the firmware's own cap to prove nothing here "
                            "narrows it");
}

/* The reason the mark lives in .noinit at all: a WARM reset (watchdog, RESET button -- the
   board resetting mid-dose is the exact physical scenario requirement 2 names) must not
   reopen the replay window. This is only true if g_nv.cmd_high_water's own bump is
   noinit_commit()ed -- without that call, noinit_begin()'s sum check fails on the next boot,
   the struct is read as garbage-since-last-cold-boot, and sim_reset(true)'s "!warm" branch
   never runs to explain why: the .noinit block would simply be judged COLD when it is not,
   silently wiping cmd_high_water back to 0 on a reset the operator never asked for. */
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
  RUN_TEST(test_response_a_key_embedded_in_a_longer_key_is_not_matched);
  RUN_TEST(test_response_rejects_command_id_zero);
  RUN_TEST(test_response_rejects_a_repeated_or_lower_command_id);
  RUN_TEST(test_response_rejects_water_without_ml_or_without_cap_s);
  RUN_TEST(test_response_rejects_ml_zero);
  RUN_TEST(test_response_truncated_body_yields_no_command);
  RUN_TEST(test_response_next_out_of_range_keeps_the_previous_interval);
  RUN_TEST(test_response_empty_or_null_body_yields_no_command);
  RUN_TEST(test_response_body_with_no_next_line_still_parses_the_command);
  RUN_TEST(test_response_cmd_with_no_water_or_stop_yields_no_command);
  RUN_TEST(test_response_rejects_negative_or_non_numeric_ml);
  RUN_TEST(test_response_water_zero_is_accepted_structurally);
  RUN_TEST(test_response_an_outlet_above_pb_outlets_is_accepted_structurally);
  RUN_TEST(test_response_an_outlet_too_wide_for_the_field_yields_no_command);
  RUN_TEST(test_response_rejects_ml_or_cap_s_too_wide_for_their_fields);
  RUN_TEST(test_response_a_trailing_non_digit_does_not_truncate_to_a_smaller_number);
  RUN_TEST(test_response_stop_zero_is_neither_stop_nor_water);
  RUN_TEST(test_response_two_cmd_fields_on_one_line_the_first_wins);
  RUN_TEST(test_response_skips_a_replayed_line_and_accepts_a_fresh_one_after_it);
  RUN_TEST(test_response_an_overflowing_numeric_field_is_rejected_not_wrapped);
  RUN_TEST(test_response_the_exact_uint32_boundary_still_parses);
  RUN_TEST(test_response_carries_cap_s_through_unclamped);
  RUN_TEST(test_response_cmd_high_water_survives_a_warm_reset);
  return UNITY_END();
}
