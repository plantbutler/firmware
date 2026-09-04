/* test/test_device/test_device.cpp -- DEVICE ONLY. The four checks spec §9 says cannot
   be simulated. link_fake cannot see WiFiClient's internal modem writes, so the AT budget
   of spec §3 is only half-proved by the host suite; these two wall-clock cases are the
   other half.

   test_a_recv_pass_against_a_slow_responder_completes_within_the_wdt_window needs a
   deliberately slow responder on HOST_NAME:HTTP_PORT. The spec names no fixture; run one
   on the laptop before this suite -- e.g. a listener that accepts, waits three seconds and
   then answers -- and record which one you used in the commit message. */
#include "../support/harness.h"
#include "config.h"
#include "hal.h"
#include "link.h"
#include <unity.h>
#include <stdio.h>   /* snprintf, for the wdt probe's TEST_MESSAGE */

void setUp(void)    { pb_test_setup(); }
void tearDown(void) { pb_test_teardown(); }

static void test_wifi_begin_returns_within_two_seconds(void) {
  /* WiFi.setTimeout(0) makes CWifi::begin()'s poll loop body never run. A platform bump
     that breaks that trick must be loud here rather than a mysterious ten-second stall. */
  uint32_t t0 = hal_millis();
  link_begin(PB_NET_STEP_MS);
  link_join();
  TEST_ASSERT_LESS_THAN_UINT32(2000u, hal_millis() - t0);
}

static void test_sock_open_from_a_stale_socket_completes_within_the_wdt_window(void) {
  while (link_state() != LINK_UP && hal_millis() < 30000u) { safety_tick(); }
  TEST_ASSERT_EQUAL_INT(LINK_UP, link_state());
  TEST_ASSERT_TRUE(sock_open());          /* leave it open and abandon it on purpose */
  /* No safety_tick() between the close and the re-open, and that is deliberate: the device
     pb_test_setup() does NOT start the dog, so nothing is counting down here. Do not "fix"
     that by moving hal_wdt_start() into the device setup the way the host arm does -- the
     two ATs this pair costs (up to 2 x PB_NET_STEP_MS) would then run unfed, and a slow
     round-trip would reset the board mid-suite with no diagnostic and no failing assert.
     The dog is started by the LAST case, on purpose. */
  uint32_t t0 = hal_millis();
  sock_close();
  bool again = sock_open();
  uint32_t took = hal_millis() - t0;
  sock_close();
  TEST_ASSERT_TRUE(again);
  TEST_ASSERT_LESS_THAN_UINT32(PB_WDT_GRANTED_MS, took);
}

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
  /* The sim's counter is a fake by construction, so this is the only place the probe of
     spec §2.5 meets the actual down-counter. Once started the dog cannot be stopped:
     this case runs LAST, and the board resets a few seconds after the summary prints.
     That reset is expected. */
  TEST_ASSERT_TRUE(hal_wdt_start());
  TEST_ASSERT_EQUAL_UINT32(PB_WDT_GRANTED_MS, hal_wdt_granted());
  TEST_ASSERT_TRUE(hal_wdt_alive());
  /* NOT a second assertion: hal_wdt_alive() RETURNS delta >= PB_WDT_PROBE_MIN_COUNTS
     (hal_uno.cpp:160), so asserting that again cannot fail independently of the line above.
     What is actually wanted from a case that has never executed is the number itself, so
     the operator running this on the bench sees how much margin the probe really had. */
  {
    char m[64];
    snprintf(m, sizeof m, "wdt probe: delta=%lu min=%lu",
             (unsigned long)hal_wdt_last_delta(), (unsigned long)PB_WDT_PROBE_MIN_COUNTS);
    TEST_MESSAGE(m);
  }
}

/* setup()/loop() are declared inside an extern "C" block by arduino_main() (api/Common.h,
   same as src/main.cpp): without it this TU's C++ linkage mangles the names and the
   framework's own main.cpp.o link fails with "undefined reference to setup"/"loop". No
   Arduino header needed for that -- extern "C" is core C++, not an Arduino facility. */
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
