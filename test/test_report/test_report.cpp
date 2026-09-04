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

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_report_carries_c_t_and_the_valid_channels);
  RUN_TEST(test_report_always_carries_at_least_one_diagnostic_channel);
  RUN_TEST(test_report_omits_a_channel_whose_read_failed_rather_than_sending_zero);
  return UNITY_END();
}
