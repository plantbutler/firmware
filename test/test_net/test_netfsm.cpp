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

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_sock_close_is_idempotent_and_leaves_the_socket_unallocated);
  return UNITY_END();
}
