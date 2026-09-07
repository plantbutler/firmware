/* test_device.cpp: the checks that cannot be simulated -- wall-clock AT budgets the fake link cannot see, and the real watchdog; device only. */
#include "../support/harness.h"
#include "config.h"
#include "hal.h"
#include "link.h"
#include <unity.h>
#include <stdio.h>

void setUp(void)    { pb_test_setup(); }
void tearDown(void) { pb_test_teardown(); }

static void test_wifi_begin_returns_within_two_seconds(void) {
  /* WiFi.setTimeout(0) keeps CWifi::begin() from polling; a platform bump that breaks that
     must be loud here, not a mysterious ten-second stall. */
  uint32_t t0 = hal_millis();
  link_begin(PB_NET_STEP_MS);
  link_join();
  TEST_ASSERT_LESS_THAN_UINT32(2000u, hal_millis() - t0);
}

static void test_sock_open_from_a_stale_socket_completes_within_the_wdt_window(void) {
  while (link_state() != LINK_UP && hal_millis() < 30000u) { safety_tick(); }
  TEST_ASSERT_EQUAL_INT(LINK_UP, link_state());
  TEST_ASSERT_TRUE(sock_open());          /* leave it open and abandon it on purpose */
  /* No safety_tick() between the close and the re-open, deliberately: the device setup does
     not start the dog. Do not move hal_wdt_start() into it -- these two ATs (up to
     2 x PB_NET_STEP_MS) would then run unfed, and a slow round trip would reset the board
     mid-suite with no diagnostic. The last case starts the dog, on purpose. */
  uint32_t t0 = hal_millis();
  sock_close();
  bool again = sock_open();
  uint32_t took = hal_millis() - t0;
  sock_close();
  TEST_ASSERT_TRUE(again);
  TEST_ASSERT_LESS_THAN_UINT32(PB_WDT_GRANTED_MS, took);
}

/* Needs a deliberately slow responder on HOST_NAME:HTTP_PORT -- e.g. a listener that
   accepts, waits three seconds, then answers -- running on the laptop before this suite. */
static void test_a_recv_pass_against_a_slow_responder_completes_within_the_wdt_window(void) {
  TEST_ASSERT_TRUE(sock_open());
  static const char req[] =
    "GET / HTTP/1.1\r\nHost: slow\r\nConnection: close\r\n\r\n";
  TEST_ASSERT_TRUE(sock_write((const uint8_t *)req, sizeof req - 1) > 0);
  uint8_t rx[64];
  uint32_t t0 = hal_millis();
  int n = 0;
  while (n == 0 && hal_millis() - t0 < PB_NET_DEADLINE_MS) {
    safety_tick();
    n = sock_read(rx, sizeof rx);
    TEST_ASSERT_LESS_THAN_UINT32(PB_WDT_GRANTED_MS, hal_millis() - t0);
  }
  sock_close();
}

static void test_wdt_alive_returns_true_on_real_silicon(void) {
  /* The only place the probe meets the real down-counter. Once started the dog cannot be
     stopped: this case runs last, and the board resets a few seconds after the summary
     prints. That reset is expected. */
  TEST_ASSERT_TRUE(hal_wdt_start());
  TEST_ASSERT_EQUAL_UINT32(PB_WDT_GRANTED_MS, hal_wdt_granted());
  TEST_ASSERT_TRUE(hal_wdt_alive());
  /* Not a second assertion: hal_wdt_alive() returns delta >= PB_WDT_PROBE_MIN_COUNTS, so it
     could not fail independently of the line above. The number itself is what the bench
     wants to see: how much margin the probe really had. */
  {
    char m[64];
    snprintf(m, sizeof m, "wdt probe: delta=%lu min=%lu",
             (unsigned long)hal_wdt_last_delta(), (unsigned long)PB_WDT_PROBE_MIN_COUNTS);
    TEST_MESSAGE(m);
  }
}

/* The framework declares setup()/loop() extern "C": without it this TU mangles the names
   and the link fails with "undefined reference to setup". */
extern "C" void setup(void) {
  while (hal_millis() < 2000u) { }        /* let the USB CDC bridge come up */
  UNITY_BEGIN();
  RUN_TEST(test_wifi_begin_returns_within_two_seconds);
  RUN_TEST(test_sock_open_from_a_stale_socket_completes_within_the_wdt_window);
  RUN_TEST(test_a_recv_pass_against_a_slow_responder_completes_within_the_wdt_window);
  RUN_TEST(test_wdt_alive_returns_true_on_real_silicon);
  UNITY_END();
}

extern "C" void loop(void) {}
