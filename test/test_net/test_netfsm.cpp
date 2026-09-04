/* test/test_net/test_netfsm.cpp — seam 2, the FSM, the AT budget, the retry policy. */
#include <unity.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../support/harness.h"
#include "config.h"
#include "hal.h"
#include "link.h"
#include "netfsm.h"
#include "report.h"
#include "safety.h"
#include "secrets.h"
#include "sensors.h"
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

/* §3 change 1: a FAILED connect still ALLOCATES the socket (_sock >= 0), because
   getSocket() allocates before the connect runs. That is the arithmetic that keeps a
   CONNECT pass at 2 ATs rather than 3 -- if a failed open instead left _sock == -1, the
   next sock_close() would see the precondition already satisfied and cost 0 ATs, silently
   hiding the fact that a real failed WiFiClient::connect() still owns a socket the caller
   must close. Proving "left allocated" here means proving the ONE observable consequence
   of allocation this seam exposes: the next sock_close() must still cost its AT. */
static void test_a_failed_connect_leaves_the_socket_allocated(void) {
  up();
  link_fake_fail_open(true);
  TEST_ASSERT_FALSE(sock_open());       /* _BEGINCLIENT + _CLIENTCONNECT both ran; refused */
  link_fake_pass_begin();
  sock_close();                          /* if _sock were -1 already, this would cost 0 ATs */
  TEST_ASSERT_EQUAL_UINT16(1, link_fake_at_count());
}

/* No case in this file called sock_write() before this one -- link_fake_sent() and
   link_fake_write_count() are additions to the skeleton's ten primitives (see the commit
   that adds them), and an accessor nothing has ever called is an accessor whose first bug
   report comes from task 24/25. Exercise it once and read back both what it reports and
   what actually crossed the seam. */
static void test_sock_write_records_the_bytes_and_the_write_count(void) {
  up();
  TEST_ASSERT_TRUE(sock_open());
  static const uint8_t body[] = "GET / HTTP/1.1\r\nHost: x\r\n\r\n";
  const size_t n = sizeof(body) - 1;     /* exclude the trailing NUL */
  link_fake_pass_begin();
  TEST_ASSERT_EQUAL_INT((int)n, sock_write(body, n));
  TEST_ASSERT_EQUAL_UINT16(1, link_fake_at_count());     /* SEND — one AT, per §3's table */
  TEST_ASSERT_EQUAL_UINT16(1, link_fake_write_count());
  uint16_t len = 0;
  const uint8_t *sent = link_fake_sent(&len);
  TEST_ASSERT_EQUAL_UINT16((uint16_t)n, len);
  TEST_ASSERT_EQUAL_MEMORY(body, sent, n);
  TEST_ASSERT_EQUAL_INT((int)n, sock_write(body, n));    /* a second write bumps the count */
  TEST_ASSERT_EQUAL_UINT16(2, link_fake_write_count());
  sock_close();
}

/* pump_passes(n) is pb_net_passes(n, 0) and nothing else -- one spelling, so the cases
   below and task 25's cannot drift apart. */
static void pump_passes(uint8_t n) { pb_net_passes(n, 0u); }
static const char *k200 =
  "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: 8\r\n\r\nnext=60\n";

static void test_http_post_carries_host_token_and_content_length(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  pump_passes(8);
  uint16_t n = 0;
  const char *tx = (const char *)link_fake_sent(&n);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_TRUE(strstr(tx, "POST /report HTTP/1.1\r\n") == tx);
  /* HOST_NAME and BUTLER_TOKEN are `const char[]` OBJECTS in secrets.h, not string-literal
     macros the way PB_CONTROLLER is, so `"Host: " HOST_NAME` would not compile. Build the
     needles instead; the c= assertion below juxtaposes because PB_CONTROLLER really is one. */
  char want[128];
  snprintf(want, sizeof want, "\r\nHost: %s\r\n", HOST_NAME);
  TEST_ASSERT_NOT_NULL(strstr(tx, want));
  snprintf(want, sizeof want, "\r\nX-Token: %s\r\n", BUTLER_TOKEN);
  TEST_ASSERT_NOT_NULL(strstr(tx, want));
  TEST_ASSERT_NOT_NULL(strstr(tx, "\r\nContent-Type: text/plain\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(tx, "\r\nConnection: close\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(tx, "\r\n\r\nc=" PB_CONTROLLER " t="));
}

static void test_report_content_length_matches_the_bytes_actually_written(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  pump_passes(8);
  uint16_t n = 0;
  const char *tx = (const char *)link_fake_sent(&n);
  const char *hdr = strstr(tx, "Content-Length: ");
  TEST_ASSERT_NOT_NULL(hdr);
  unsigned long claimed = strtoul(hdr + strlen("Content-Length: "), NULL, 10);
  const char *body = strstr(tx, "\r\n\r\n") + 4;
  TEST_ASSERT_EQUAL_UINT32((uint32_t)claimed, (uint32_t)(n - (uint16_t)(body - tx)));
}

/* A full first-ever round trip is DOWN -> JOIN_ISSUE -> JOIN_WAIT -> IDLE -> SOCK_CLOSE ->
   CONNECT -> SEND -> RECV -> CLOSE -> SOCK_CLOSE -> IDLE: ten net_poll() calls, one state
   transition per call (spec's own per-pass table, §3/§4.2), never eight -- confirmed by
   tracing link_fake_at_count() and net_state() pass by pass against this file's own
   reference netfsm.cpp. pump_passes(12) leaves two calls of margin once IDLE is reached
   (idle passes are no-ops until g_next_s elapses, so the margin costs nothing). */
static void test_socket_is_closed_on_success_error_timeout_and_a_failed_open(void) {
  sensors_begin();
  /* success */
  net_begin(); link_fake_queue_response(k200, strlen(k200));
  pump_passes(12);
  TEST_ASSERT_EQUAL(NET_IDLE, net_state());
  TEST_ASSERT_TRUE(sock_open());          /* the precondition holds: _sock was left -1 */
  sock_close();
  /* a failed open */
  net_begin(); link_fake_fail_open(true);
  pump_passes(12);
  link_fake_fail_open(false);
  TEST_ASSERT_TRUE(sock_open());          /* would be false if the failed open had not closed */
  sock_close();
  /* a timeout in RECV: no response was ever queued. Each pass's own AT round trips only
     advance the fake clock by a couple of ms (sim.h's "hal_millis() advances the rig by
     exactly 1 ms" contract), so reaching the 5 s PB_NET_DEADLINE_MS needs real elapsed time
     between passes, not just more of them -- pb_net_passes()'s second argument, which
     pump_passes() (this file's alias for it with ms hardwired to 0) cannot supply. 20 passes
     x 500 ms comfortably crosses the deadline with passes to spare for the SOCK_CLOSE/IDLE
     cleanup that follows it. */
  net_begin();
  pb_net_passes(20, 500u);
  TEST_ASSERT_TRUE(sock_open());
  sock_close();
}

static void test_connect_is_never_issued_without_a_close_in_a_prior_pass(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  net_state_t prev = net_state();
  for (int i = 0; i < 24; ++i) {
    link_fake_pass_begin();
    net_poll(false);
    if (net_state() == NET_CONNECT) TEST_ASSERT_EQUAL(NET_SOCK_CLOSE, prev);
    prev = net_state();
  }
}

static void test_no_pass_issues_more_than_two_at_commands(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  for (int i = 0; i < 40; ++i) {
    link_fake_pass_begin();
    net_poll(false);
    TEST_ASSERT_TRUE(link_fake_at_count() <= 2);
  }
}

static void test_every_error_exit_transitions_to_sock_close_rather_than_closing_inline(void) {
  sensors_begin();
  net_begin();
  link_fake_fail_open(true);
  for (int i = 0; i < 24; ++i) {
    link_fake_pass_begin();
    net_state_t before = net_state();
    net_poll(false);
    if (before == NET_CONNECT) {
      TEST_ASSERT_EQUAL(NET_SOCK_CLOSE, net_state());
      TEST_ASSERT_EQUAL_UINT16(2, link_fake_at_count());  /* NOT 3: no inline _CLIENTCLOSE */
      link_fake_fail_open(false);
      return;
    }
  }
  TEST_FAIL_MESSAGE("the FSM never reached NET_CONNECT");
}

static void test_sock_read_calls_neither_available_nor_connected(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  for (int i = 0; i < 40; ++i) {
    link_fake_pass_begin();
    net_state_t before = net_state();
    net_poll(false);
    if (before == NET_RECV) TEST_ASSERT_EQUAL_UINT16(1, link_fake_at_count());
  }
  TEST_ASSERT_FALSE(link_fake_saw_available());
  TEST_ASSERT_FALSE(link_fake_saw_connected());
}

static const char *k400 =
  "HTTP/1.1 400 Bad Request\r\nContent-Length: 38\r\n\r\n"
  "next=60\ncmd=1 water=3 ml=250 cap_s=30\n";

static void test_response_is_never_parsed_from_a_four_hundred_body(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k400, strlen(k400));
  pump_passes(10);
  cmd_t c;
  TEST_ASSERT_FALSE(net_take_command(&c));      /* a 400 body echoes OUR tokens back at us */
  TEST_ASSERT_EQUAL_UINT16(400, net_last_status());
  TEST_ASSERT_EQUAL_UINT32(0, net_reports_ok());
}

/* pump_passes(n) (== pb_net_passes(n, 0)) cannot drive a SECOND round trip on its own: with
   g_next_s left at 60 by round 1's own "next=60", NET_IDLE's due check never lets the FSM
   leave IDLE again until 60 s of fake-clock time have actually elapsed, and pump_passes()'s
   zero ms step advances the clock by only a couple of ms per pass (sim.h's "hal_millis()
   advances the rig by exactly 1 ms" contract). Without real elapsed time between passes,
   round 2 never leaves IDLE, SOCK_CLOSE's rx-buffer reset is never re-exercised, and this
   case would pass for a reason that has nothing to do with the bug it names -- so this uses
   pb_net_passes() directly, with a step big enough to cross both the 60 s report interval
   and, once round 2's own RECV starts, the 5 s PB_NET_DEADLINE_MS its intentionally-empty
   response never answers. */
static void test_stale_bytes_in_the_rx_buffer_cannot_become_a_command(void) {
  sensors_begin();
  net_begin();
  /* round 1: a complete 200 carrying a command */
  const char *with_cmd =
    "HTTP/1.1 200 OK\r\nContent-Length: 38\r\n\r\nnext=60\ncmd=5 water=3 ml=250 cap_s=30\n";
  link_fake_queue_response(with_cmd, strlen(with_cmd));
  pump_passes(10);
  cmd_t c;
  TEST_ASSERT_TRUE(net_take_command(&c));
  TEST_ASSERT_EQUAL_UINT32(5, c.id);
  TEST_ASSERT_EQUAL_UINT32(1, net_reports_ok());
  report_clear_ack();                            /* stand in for exec_pending()'s real ack */
  /* round 2: the server answers with nothing at all. The old bytes must not be re-parsed --
     and must not even be mistaken for a completed response, which is the guard-independent
     half of the proof: the replay guard (task 23) would refuse a re-executed cmd=5 anyway,
     but a stale, un-cleared g_rx would still let rx_complete() see an already-complete
     response the instant RECV starts, short-circuiting the real (empty) exchange and
     crediting a 200 that never happened. */
  link_fake_queue_response("", 0);
  pb_net_passes(40, 2000u);
  TEST_ASSERT_FALSE(net_take_command(&c));
  TEST_ASSERT_EQUAL_UINT32(1, net_reports_ok());       /* round 2 must NOT count as a second 200 */
  TEST_ASSERT_EQUAL_UINT32(1, net_reports_failed());   /* it must count as the timeout it is */
}

static uint16_t g_at_in_dose;
static net_state_t g_state_in_dose;
static void poke_net_from_inside_the_dose(void) {
  link_fake_pass_begin();
  /* safety_dosing() is TRUE here — we are inside hal_pump_write(true) — and this is the one
     call site in the suite that must pass it, because it is the guard under test. */
  net_poll(safety_dosing());
  g_at_in_dose = link_fake_at_count();
  g_state_in_dose = net_state();
}

static void test_poll_is_a_noop_while_the_pump_is_asserted(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  pump_passes(4);                              /* park the FSM somewhere with work to do */
  const net_state_t before = net_state();
  sim_set_float(true);
  sim_set_flow_ml_s(30);
  sim_on_pump_on(poke_net_from_inside_the_dose);
  /* dose_run()'s ladder refuses DOSE_REFUSED_BOOT below PB_BOOT_GAP_MS (safety.cpp), and
     pump_passes(4) alone leaves the fake clock nowhere near that -- without this the pump
     never asserts, poke_net_from_inside_the_dose() never runs, and both asserts below pass
     vacuously on g_at_in_dose/g_state_in_dose's zero-initialised defaults. Same shape as
     harness.h's own pb_latch_contra(). */
  pb_advance(PB_BOOT_GAP_MS + 1u);
  dose_req_t q = { 0, 0, true, 1500, false, false };   /* by_time, no position needed */
  (void)dose_run(&q);
  TEST_ASSERT_EQUAL_UINT16(0, g_at_in_dose);   /* not one AT command while D6 is hot */
  TEST_ASSERT_EQUAL(before, g_state_in_dose);  /* and not one state transition either */
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_sock_close_is_idempotent_and_leaves_the_socket_unallocated);
  RUN_TEST(test_the_fake_counts_at_commands_per_pass);
  RUN_TEST(test_a_second_link_reset_still_produces_a_working_at_round_trip);
  RUN_TEST(test_a_failed_connect_leaves_the_socket_allocated);
  RUN_TEST(test_sock_write_records_the_bytes_and_the_write_count);
  RUN_TEST(test_http_post_carries_host_token_and_content_length);
  RUN_TEST(test_report_content_length_matches_the_bytes_actually_written);
  RUN_TEST(test_socket_is_closed_on_success_error_timeout_and_a_failed_open);
  RUN_TEST(test_connect_is_never_issued_without_a_close_in_a_prior_pass);
  RUN_TEST(test_no_pass_issues_more_than_two_at_commands);
  RUN_TEST(test_every_error_exit_transitions_to_sock_close_rather_than_closing_inline);
  RUN_TEST(test_sock_read_calls_neither_available_nor_connected);
  RUN_TEST(test_response_is_never_parsed_from_a_four_hundred_body);
  RUN_TEST(test_stale_bytes_in_the_rx_buffer_cannot_become_a_command);
  RUN_TEST(test_poll_is_a_noop_while_the_pump_is_asserted);
  return UNITY_END();
}
