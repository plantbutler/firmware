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
  fresh_sweep();
  TEST_ASSERT_EQUAL_INT(0, PB_PULSES_PER_GATE);      /* bring-up 6 has not run */
  TEST_ASSERT_FALSE(cart_pos_known());
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("pos=unknown"));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_report_carries_c_t_and_the_valid_channels);
  RUN_TEST(test_report_always_carries_at_least_one_diagnostic_channel);
  RUN_TEST(test_report_omits_a_channel_whose_read_failed_rather_than_sending_zero);
  RUN_TEST(test_report_omits_the_wired_channels_and_says_stuck_when_the_canary_matches);
  RUN_TEST(test_report_float_is_the_debounced_tank_verdict_anded_with_not_contra);
  RUN_TEST(test_report_float_is_only_ever_zero_or_one);
  RUN_TEST(test_repeated_float_refusals_drive_float_to_zero_on_the_wire);
  RUN_TEST(test_a_granted_dose_clears_the_float_refusal_counter);
  RUN_TEST(test_report_pos_is_unknown_while_the_going_live_flag_is_set);
  RUN_TEST(test_report_pos_is_unknown_while_the_dry_latch_is_set);
  RUN_TEST(test_report_pos_is_unknown_when_the_gate_pitch_is_uncalibrated);
  return UNITY_END();
}
