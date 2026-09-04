/* test/test_net/test_netfsm.cpp — seam 2, the FSM, the AT budget, the retry policy. */
#include <unity.h>
#include <string.h>
#include "../support/harness.h"
#include "config.h"
#include "hal.h"
#include "link.h"
#include "sim.h"

void setUp(void)    { pb_test_setup(); link_fake_reset(); link_begin(PB_NET_STEP_MS); }
void tearDown(void) { pb_test_teardown(); }

static void up(void) {           /* drive the fake link to LINK_UP */
  link_join();
  TEST_ASSERT_EQUAL(LINK_UP, link_state());
}

static void test_sock_close_is_idempotent_and_leaves_the_socket_unallocated(void) {
  up();
  TEST_ASSERT_TRUE(sock_open());
  sock_close();
  link_fake_pass_begin();
  sock_close();                  /* the second close costs nothing and must not fault */
  TEST_ASSERT_EQUAL_UINT16(0, link_fake_at_count());
  TEST_ASSERT_TRUE(sock_open()); /* the precondition holds again only because _sock == -1 */
  sock_close();
}

static void test_the_fake_counts_at_commands_per_pass(void) {
  link_fake_pass_begin();
  link_join();                                    /* WiFi.cpp:43-67 — 2 ATs, never 3 */
  TEST_ASSERT_EQUAL_UINT16(2, link_fake_at_count());
  link_fake_pass_begin();
  TEST_ASSERT_EQUAL(LINK_UP, link_state());       /* status() — 1 AT */
  TEST_ASSERT_EQUAL_UINT16(1, link_fake_at_count());
  link_fake_pass_begin();
  TEST_ASSERT_TRUE(sock_open());                  /* _BEGINCLIENT + _CLIENTCONNECT — 2 ATs */
  TEST_ASSERT_EQUAL_UINT16(2, link_fake_at_count());
  link_fake_pass_begin();
  sock_close();                                   /* _CLIENTCLOSE — 1 AT */
  TEST_ASSERT_EQUAL_UINT16(1, link_fake_at_count());
  link_fake_pass_begin();
  sock_close();                                   /* _sock == -1 — 0 ATs */
  TEST_ASSERT_EQUAL_UINT16(0, link_fake_at_count());
}

static void test_a_second_link_reset_still_produces_a_working_at_round_trip(void) {
  up();
  link_reset();
  TEST_ASSERT_EQUAL_UINT16(1, link_fake_reset_count());
  TEST_ASSERT_EQUAL_UINT16(1, link_desyncs());
  link_join();                          /* must resync: 2 ATs into a REOPENED UART */
  TEST_ASSERT_EQUAL(LINK_UP, link_state());
  link_reset();                         /* and again — this is the one that used to die */
  link_join();
  TEST_ASSERT_EQUAL(LINK_UP, link_state());
  TEST_ASSERT_EQUAL_UINT16(2, link_fake_reset_count());
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_sock_close_is_idempotent_and_leaves_the_socket_unallocated);
  RUN_TEST(test_the_fake_counts_at_commands_per_pass);
  RUN_TEST(test_a_second_link_reset_still_produces_a_working_at_round_trip);
  return UNITY_END();
}
