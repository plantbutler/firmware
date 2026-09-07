/* test_netfsm.cpp: the network FSM, its AT budget per pass and its retry policy, on the host. */
#include <unity.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../support/bodies.h"
#include "../support/harness.h"
#include "cart.h"
#include "config.h"
#include "exec.h"
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
  sock_close();
  TEST_ASSERT_EQUAL_UINT16(0, link_fake_at_count());
  TEST_ASSERT_TRUE(sock_open()); /* opens only because nothing is allocated */
  sock_close();
}

static void test_the_fake_counts_at_commands_per_pass(void) {
  link_fake_pass_begin();
  link_join();                                    /* a join is 2 ATs in the driver, never 3 */
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
  sock_close();                                   /* nothing allocated — 0 ATs */
  TEST_ASSERT_EQUAL_UINT16(0, link_fake_at_count());
}

static void test_a_second_link_reset_still_produces_a_working_at_round_trip(void) {
  up();
  link_reset();
  TEST_ASSERT_EQUAL_UINT16(1, link_fake_reset_count());
  TEST_ASSERT_EQUAL_UINT16(1, link_desyncs());
  link_join();                          /* resyncs: 2 ATs into a reopened UART */
  TEST_ASSERT_EQUAL(LINK_UP, link_state());
  link_reset();
  link_join();
  TEST_ASSERT_EQUAL(LINK_UP, link_state());
  TEST_ASSERT_EQUAL_UINT16(2, link_fake_reset_count());
}

/* The driver allocates the socket before the connect runs, so a refused connect still owns
   one, and the only sign of that this seam shows is that the next sock_close() costs its AT. */
static void test_a_failed_connect_leaves_the_socket_allocated(void) {
  up();
  link_fake_fail_open(true);
  TEST_ASSERT_FALSE(sock_open());       /* _BEGINCLIENT + _CLIENTCONNECT both ran; refused */
  link_fake_pass_begin();
  sock_close();                          /* 0 ATs if nothing were allocated */
  TEST_ASSERT_EQUAL_UINT16(1, link_fake_at_count());
}

static void test_sock_write_records_the_bytes_and_the_write_count(void) {
  up();
  TEST_ASSERT_TRUE(sock_open());
  static const uint8_t body[] = "GET / HTTP/1.1\r\nHost: x\r\n\r\n";
  const size_t n = sizeof(body) - 1;     /* exclude the trailing NUL */
  link_fake_pass_begin();
  TEST_ASSERT_EQUAL_INT((int)n, sock_write(body, n));
  TEST_ASSERT_EQUAL_UINT16(1, link_fake_at_count());     /* SEND — one AT */
  TEST_ASSERT_EQUAL_UINT16(1, link_fake_write_count());
  uint16_t len = 0;
  const uint8_t *sent = link_fake_sent(&len);
  TEST_ASSERT_EQUAL_UINT16((uint16_t)n, len);
  TEST_ASSERT_EQUAL_MEMORY(body, sent, n);
  TEST_ASSERT_EQUAL_INT((int)n, sock_write(body, n));
  TEST_ASSERT_EQUAL_UINT16(2, link_fake_write_count());
  sock_close();
}

static void pump_passes(uint8_t n) { pb_net_passes(n, 0u); }
static const char *k200 =
  "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: 8\r\n\r\nnext=60\n";

static void test_http_post_carries_host_token_and_content_length(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  pump_passes(10);   /* ten passes reach SEND: two refresh-only passes (signal, then address)
                        precede NET_IDLE's first real one */
  uint16_t n = 0;
  const char *tx = (const char *)link_fake_sent(&n);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_TRUE(strstr(tx, "POST /report HTTP/1.1\r\n") == tx);
  char want[128];
  snprintf(want, sizeof want, "\r\nHost: %s\r\n", HOST_NAME);
  TEST_ASSERT_NOT_NULL(strstr(tx, want));
  snprintf(want, sizeof want, "\r\nX-Token: %s\r\n", BUTLER_TOKEN);
  TEST_ASSERT_NOT_NULL(strstr(tx, want));
  TEST_ASSERT_NOT_NULL(strstr(tx, "\r\nContent-Type: text/plain\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(tx, "\r\nConnection: close\r\n"));
  snprintf(want, sizeof want, "\r\n\r\nc=%u t=", (unsigned)PB_CONTROLLER);
  TEST_ASSERT_NOT_NULL(strstr(tx, want));
}

static void test_report_content_length_matches_the_bytes_actually_written(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  pump_passes(10);   /* ten passes reach SEND, as above */
  uint16_t n = 0;
  const char *tx = (const char *)link_fake_sent(&n);
  const char *hdr = strstr(tx, "Content-Length: ");
  TEST_ASSERT_NOT_NULL(hdr);
  unsigned long claimed = strtoul(hdr + strlen("Content-Length: "), NULL, 10);
  const char *body = strstr(tx, "\r\n\r\n") + 4;
  TEST_ASSERT_EQUAL_UINT32((uint32_t)claimed, (uint32_t)(n - (uint16_t)(body - tx)));
}

/* A first round trip is twelve passes: DOWN, JOIN_ISSUE, JOIN_WAIT, IDLE and its two
   refresh-only passes, then SOCK_CLOSE, CONNECT, SEND, RECV, CLOSE, SOCK_CLOSE back to IDLE.
   Fourteen leaves margin; idle passes are no-ops until the interval elapses. */
static void test_socket_is_closed_on_success_error_timeout_and_a_failed_open(void) {
  sensors_begin();
  /* success */
  net_begin(); link_fake_queue_response(k200, strlen(k200));
  pump_passes(14);
  TEST_ASSERT_EQUAL(NET_IDLE, net_state());
  TEST_ASSERT_TRUE(sock_open());          /* opens only because the socket was closed */
  sock_close();
  /* a failed open */
  net_begin(); link_fake_fail_open(true);
  pump_passes(14);
  link_fake_fail_open(false);
  TEST_ASSERT_TRUE(sock_open());          /* false if the failed open had not closed */
  sock_close();
  /* a timeout in RECV: nothing queued. An empty response is retried once, so two 5 s RECV
     deadlines must elapse; 40 passes x 500 ms crosses both with passes to spare. */
  net_begin();
  pb_net_passes(40, 500u);
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
  pump_passes(12);   /* twelve: a full first round trip */
  cmd_t c;
  TEST_ASSERT_FALSE(net_take_command(&c));      /* a 400 body echoes our own tokens back */
  TEST_ASSERT_EQUAL_UINT16(400, net_last_status());
  TEST_ASSERT_EQUAL_UINT32(0, net_reports_ok());
}

/* Round 2 is due only after the 60 s interval round 1's next=60 set, and its empty response
   then runs out a 5 s RECV deadline; only passes with a real time step cross both. */
static void test_stale_bytes_in_the_rx_buffer_cannot_become_a_command(void) {
  sensors_begin();
  net_begin();
  /* round 1: a complete 200 carrying a command */
  const char *with_cmd =
    "HTTP/1.1 200 OK\r\nContent-Length: 38\r\n\r\nnext=60\ncmd=5 water=3 ml=250 cap_s=30\n";
  link_fake_queue_response(with_cmd, strlen(with_cmd));
  pump_passes(12);   /* twelve: a full first round trip */
  cmd_t c;
  TEST_ASSERT_TRUE(net_take_command(&c));
  TEST_ASSERT_EQUAL_UINT32(5, c.id);
  TEST_ASSERT_EQUAL_UINT32(1, net_reports_ok());
  report_clear_ack();                            /* stand in for exec_pending()'s real ack */
  /* round 2: the server answers with nothing. The stale bytes must not be re-parsed, nor read
     as an already-complete response that credits a 200 which never happened. */
  link_fake_queue_response("", 0);
  pb_net_passes(40, 2000u);   /* round 2 reuses round 1's join: no extra refresh passes */
  TEST_ASSERT_FALSE(net_take_command(&c));
  TEST_ASSERT_EQUAL_UINT32(1, net_reports_ok());       /* round 2 must NOT count as a second 200 */
  TEST_ASSERT_EQUAL_UINT32(1, net_reports_failed());   /* it must count as the timeout it is */
}

static uint16_t g_at_in_dose;
static net_state_t g_state_in_dose;
static void poke_net_from_inside_the_dose(void) {
  link_fake_pass_begin();
  /* inside the dose, so safety_dosing() is true: the guard under test */
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
  /* the ladder refuses any dose before PB_BOOT_GAP_MS; without this the pump never asserts
     and both asserts below pass vacuously on zero-initialised defaults */
  pb_advance(PB_BOOT_GAP_MS + 1u);
  dose_req_t q = { 0, 0, true, 1500, false, false };   /* by_time, no position needed */
  (void)dose_run(&q);
  TEST_ASSERT_EQUAL_UINT16(0, g_at_in_dose);   /* not one AT command while D6 is hot */
  TEST_ASSERT_EQUAL(before, g_state_in_dose);  /* and not one state transition either */
}

static void test_net_begin_clears_a_standing_disable_latch(void) {
  net_disable("heap");
  TEST_ASSERT_NOT_NULL(net_disabled());
  net_begin();
  TEST_ASSERT_NULL(net_disabled());
  net_poll(false);
  TEST_ASSERT_NOT_EQUAL(NET_DOWN, net_state());   /* it actually runs again */
}

/* setup() must latch the boot verdict after net_begin(), which clears the disable latch
   unconditionally; in the other order the board reports for 48 hours with a failed watchdog,
   ADC or heap assertion behind it. net_boot() is the one copy of that order, and main.cpp is
   not in this binary. */
static void test_a_failed_boot_assertion_survives_net_begin(void) {
  net_boot("wdt");
  TEST_ASSERT_NOT_NULL(net_disabled());
  TEST_ASSERT_EQUAL_STRING("wdt", net_disabled());

  /* four passes must move nothing: honouring the latch is net_poll()'s first act */
  pb_net_passes(4, PB_NET_STEP_MS);
  TEST_ASSERT_EQUAL(NET_DOWN, net_state());
  TEST_ASSERT_EQUAL_UINT32(0, net_reports_ok());
  TEST_ASSERT_EQUAL_UINT32(0, net_reports_failed());
}

static void test_a_join_deadline_is_not_expired_early_by_the_clock_rollover(void) {
  /* set before net_begin(): a board 49.7 days up has every FSM timestamp near the wrap */
  sim_set_clock_ms(0xFFFFF000u);
  net_begin();
  link_fake_drop_link();

  net_poll(false);                             /* NET_DOWN -> NET_JOIN_ISSUE */
  net_poll(false);                             /* issues the join; the 5 s deadline WRAPS */
  TEST_ASSERT_EQUAL(NET_JOIN_WAIT, net_state());

  /* An unsigned compare of a pre-wrap clock against a post-wrap deadline calls the join
     expired on the first pass. A timed-out status query would poison the link instead, so
     the drop keeps the join pending at zero ATs and pb_advance() supplies the elapsed time. */
  link_fake_drop_link();
  pb_advance(1300);
  net_poll(false);
  TEST_ASSERT_EQUAL(NET_JOIN_WAIT, net_state());
}

static void test_a_recv_deadline_is_not_expired_early_by_the_clock_rollover(void) {
  sim_set_clock_ms(0xF0000000u);
  sensors_begin();
  net_begin();
  for (int i = 0; i < 24 && net_state() != NET_SEND; ++i) net_poll(false);
  TEST_ASSERT_EQUAL(NET_SEND, net_state());

  /* jump to just under the wrap before the RECV deadline is armed, so it straddles the wrap */
  sim_set_clock_ms(0xFFFFF830u);
  net_poll(false);                             /* NET_SEND -> NET_RECV, deadline WRAPS */
  TEST_ASSERT_EQUAL(NET_RECV, net_state());

  /* nothing is queued, so only the deadline can end this pass; an unsigned compare would call
     it expired at once */
  pb_advance(100);
  net_poll(false);
  TEST_ASSERT_EQUAL(NET_RECV, net_state());
}

static void test_a_backoff_wait_still_waits_across_the_clock_rollover(void) {
  sim_set_clock_ms(0xFFFF0000u);
  net_begin();
  link_fake_drop_link();

  net_poll(false);                             /* NET_DOWN -> NET_JOIN_ISSUE */
  net_poll(false);                             /* issues the join, arms the deadline pre-wrap */
  TEST_ASSERT_EQUAL(NET_JOIN_WAIT, net_state());

  /* Just under the wrap: the join deadline is past, and link_down() must arm its 2 s backoff
     while the clock is still below the wrap or it never straddles it. The timed-out query
     below advances the clock by PB_NET_STEP_MS; 0xFFFFF830 leaves ~800 ms to spare. */
  sim_set_clock_ms(0xFFFFF830u);
  link_fake_timeout_next();
  net_poll(false);
  TEST_ASSERT_EQUAL(NET_DOWN, net_state());

  /* an unsigned compare would read the wrapped backoff as already past and rejoin at once */
  net_poll(false);
  TEST_ASSERT_EQUAL(NET_DOWN, net_state());

  pb_advance(2500);                            /* now the 2 s backoff really has elapsed */
  net_poll(false);                             /* the deferred tear-down spends this pass */
  TEST_ASSERT_EQUAL(NET_DOWN, net_state());
  net_poll(false);
  TEST_ASSERT_EQUAL(NET_JOIN_ISSUE, net_state());
}

/* one pass at a time, so the send counter sees the state each pass started in */
static int run_passes(int n, uint32_t ms_each) {
  int sends = 0;
  for (int i = 0; i < n; ++i) {
    const net_state_t before = net_state();
    pb_net_passes(1u, ms_each);
    if (before == NET_SEND) ++sends;
  }
  return sends;
}

static void test_an_exchange_that_produced_no_bytes_is_retried_exactly_once(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response("", 0);        /* the server says nothing at all */
  const int sends = run_passes(120, 200); /* 24 s: two RECV deadlines, inside the 30 s window */
  TEST_ASSERT_EQUAL_INT(2, sends);        /* the original and ONE retry */
  TEST_ASSERT_EQUAL_UINT32(0, net_reports_ok());
}

static void test_a_retry_is_abandoned_rather_than_sent_outside_the_dedup_window(void) {
  sim_reset(true);                        /* warm: the boot counter advances, so the salt
                                             is non-zero and t= is above 2^31 */
  sensors_begin();
  TEST_ASSERT_NOT_EQUAL(0, hal_boot_salt());
  net_begin();
  link_fake_queue_response("", 0);
  /* inside the window: the retry IS sent */
  TEST_ASSERT_EQUAL_INT(2, run_passes(120, 200));
  /* a fresh report, walked one pass at a time to where the retry is armed but not yet sent
     (NET_SOCK_CLOSE, between the timeout and the redial): that pass re-checks the window, so
     the clock must pass the deadline before the walk, not after */
  net_begin();
  link_fake_queue_response("", 0);
  int sends = 0;
  net_state_t st = NET_DOWN;
  for (int i = 0; i < 60; ++i) {
    const net_state_t before = net_state();
    pb_net_passes(1u, 200u);
    if (before == NET_SEND) ++sends;
    st = net_state();
    if (sends == 1 && st == NET_SOCK_CLOSE) break;
  }
  TEST_ASSERT_EQUAL_INT(1, sends);
  TEST_ASSERT_EQUAL(NET_SOCK_CLOSE, st);  /* the retry is armed, sitting right before CONNECT */
  sim_advance(PB_RETRY_DEADLINE_MS + 1000);
  sends += run_passes(40, 200);
  TEST_ASSERT_EQUAL_INT(1, sends);        /* ABANDONED: never sent outside the dedup window */
}

static void test_a_response_that_produced_any_bytes_is_never_retried(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response("HTTP/1.1 2", 10);      /* bytes arrived; the answer never completed */
  TEST_ASSERT_EQUAL_INT(1, run_passes(120, 200));
}

static void test_a_truncated_reply_is_never_retried(void) {
  sensors_begin();
  net_begin();
  const char *cut = "HTTP/1.1 200 OK\r\nContent-Length: 38\r\n\r\nnext=60\ncmd=5 wat";
  link_fake_queue_response(cut, strlen(cut));
  TEST_ASSERT_EQUAL_INT(1, run_passes(120, 200));  /* a truncation is bytes that ARRIVED */
  cmd_t c;
  TEST_ASSERT_FALSE(net_take_command(&c));         /* and a half-read reply never waters */
}

static void test_a_four_hundred_is_never_retried(void) {
  sensors_begin();
  net_begin();
  const char *b = "HTTP/1.1 400 Bad Request\r\nContent-Length: 5\r\n\r\nnope\n";
  link_fake_queue_response(b, strlen(b));
  TEST_ASSERT_EQUAL_INT(1, run_passes(60, 200));
  TEST_ASSERT_EQUAL_UINT16(400, net_last_status());
}

static void test_a_five_hundred_is_not_retried(void) {
  sensors_begin();
  net_begin();
  const char *b = "HTTP/1.1 500 Internal Server Error\r\nContent-Length: 5\r\n\r\noops\n";
  link_fake_queue_response(b, strlen(b));
  TEST_ASSERT_EQUAL_INT(1, run_passes(60, 200));   /* no rollback guarantee: not a 503 */
  TEST_ASSERT_EQUAL_UINT16(500, net_last_status());
}

static void test_a_five_oh_three_is_retried_once(void) {
  sensors_begin();
  net_begin();
  const char *b = "HTTP/1.1 503 Service Unavailable\r\nContent-Length: 5\r\n\r\nbusy\n";
  link_fake_queue_response(b, strlen(b));
  TEST_ASSERT_EQUAL_INT(2, run_passes(60, 200));   /* the backend answers 503 when it
                                                      rolled everything back */
  TEST_ASSERT_EQUAL_UINT16(503, net_last_status());
}

static void test_report_body_is_byte_identical_on_the_retry(void) {
  sensors_begin();
  net_begin();
  const char *b = "HTTP/1.1 503 Service Unavailable\r\nContent-Length: 5\r\n\r\nbusy\n";
  link_fake_queue_response(b, strlen(b));
  static uint8_t first[PB_TX_CAP];
  uint16_t first_len = 0, len = 0;
  int sends = 0;
  for (int i = 0; i < 60; ++i) {
    link_fake_pass_begin();
    const net_state_t before = net_state();
    net_poll(false);
    if (before == NET_SEND) {
      const uint8_t *tx = link_fake_sent(&len);
      if (++sends == 1) { memcpy(first, tx, len); first_len = len; }
      else { TEST_ASSERT_EQUAL_UINT16(first_len, len);
             TEST_ASSERT_EQUAL_MEMORY(first, tx, len); }
      link_fake_queue_response(b, strlen(b));      /* the same 503 again */
    }
    sim_advance(200);
  }
  TEST_ASSERT_EQUAL_INT(2, sends);
}

static void test_a_modem_timeout_poisons_the_link_and_counts_a_desync(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  /* walk to the first state that issues an AT, then time it out */
  for (int i = 0; i < 40; ++i) {
    link_fake_pass_begin();
    const net_state_t before = net_state();
    if (before == NET_CONNECT) {
      link_fake_timeout_next();
      const uint16_t desyncs_before = link_desyncs();
      const uint16_t resets_before = link_fake_reset_count();
      net_poll(false);
      /* the tear-down is deferred: this pass spent two round trips and a third would exceed the
         watchdog grant */
      TEST_ASSERT_EQUAL_UINT16(desyncs_before, link_desyncs());
      TEST_ASSERT_EQUAL_UINT16(resets_before, link_fake_reset_count());
      /* it lands on the pass after the backoff, before any rejoin */
      pb_advance(2500);   /* past the first backoff rung */
      net_poll(false);
      TEST_ASSERT_EQUAL_UINT16(desyncs_before + 1, link_desyncs());   /* rides out as ch206 */
      TEST_ASSERT_EQUAL_UINT16(resets_before + 1, link_fake_reset_count());
      TEST_ASSERT_EQUAL(NET_DOWN, net_state());       /* still down: the rejoin is the NEXT pass */
      TEST_ASSERT_EQUAL(NET_DOWN, net_state());
      /* and the next pass issues nothing at all while the backoff runs */
      link_fake_pass_begin();
      net_poll(false);
      TEST_ASSERT_EQUAL_UINT16(0, link_fake_at_count());
      return;
    }
    net_poll(false);
    sim_advance(50);
  }
  TEST_FAIL_MESSAGE("the FSM never reached NET_CONNECT");
}

/* One case per state that issues an AT: a modem timeout there must fire both halves of the
   poison, the reset (a desync counted) and the drop (NET_DOWN); either alone leaves a bug. */

static void test_a_modem_timeout_in_join_wait_poisons_the_link(void) {
  sensors_begin();
  net_begin();
  net_poll(false);                              /* NET_DOWN -> NET_JOIN_ISSUE */
  net_poll(false);                              /* issues the join -> NET_JOIN_WAIT */
  TEST_ASSERT_EQUAL(NET_JOIN_WAIT, net_state());
  link_fake_timeout_next();                     /* times out link_state()'s OWN AT this pass */
  const uint16_t desyncs_before = link_desyncs();
  const uint16_t resets_before = link_fake_reset_count();
  net_poll(false);
  /* deferred: the tear-down would put this pass over the watchdog grant */
  TEST_ASSERT_EQUAL_UINT16(desyncs_before, link_desyncs());
  TEST_ASSERT_EQUAL_UINT16(resets_before, link_fake_reset_count());
  pb_advance(2500);                              /* past the first backoff rung */
  net_poll(false);
  TEST_ASSERT_EQUAL_UINT16(desyncs_before + 1, link_desyncs());
  TEST_ASSERT_EQUAL_UINT16(resets_before + 1, link_fake_reset_count());
  TEST_ASSERT_EQUAL(NET_DOWN, net_state());      /* not left in JOIN_WAIT for the 5 s
                                                    deadline to catch later */
}

static void test_a_modem_timeout_in_sock_close_poisons_the_link(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  /* the first NET_SOCK_CLOSE of a report never opened a socket and costs 0 ATs, so it cannot
     time out; the one after NET_CLOSE, with the socket still allocated, can */
  bool seen_close = false;
  for (int i = 0; i < 40; ++i) {
    link_fake_pass_begin();
    const net_state_t before = net_state();
    if (before == NET_CLOSE) seen_close = true;
    if (seen_close && before == NET_SOCK_CLOSE) {
      link_fake_timeout_next();
      const uint16_t desyncs_before = link_desyncs();
      const uint16_t resets_before = link_fake_reset_count();
      net_poll(false);
      /* deferred: the tear-down would put this pass over the watchdog grant */
      TEST_ASSERT_EQUAL_UINT16(desyncs_before, link_desyncs());
      TEST_ASSERT_EQUAL_UINT16(resets_before, link_fake_reset_count());
      TEST_ASSERT_EQUAL(NET_DOWN, net_state());
      pb_advance(2500);                            /* past the first backoff rung */
      net_poll(false);
      TEST_ASSERT_EQUAL_UINT16(desyncs_before + 1, link_desyncs());
      TEST_ASSERT_EQUAL_UINT16(resets_before + 1, link_fake_reset_count());
      TEST_ASSERT_EQUAL(NET_DOWN, net_state());  /* still down: the rejoin is the NEXT pass */
      return;
    }
    net_poll(false);
  }
  TEST_FAIL_MESSAGE("never reached the socket-allocated NET_SOCK_CLOSE");
}

static void test_a_modem_timeout_in_send_poisons_the_link(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  for (int i = 0; i < 40; ++i) {
    link_fake_pass_begin();
    const net_state_t before = net_state();
    if (before == NET_SEND) {
      link_fake_timeout_next();
      const uint16_t desyncs_before = link_desyncs();
      const uint16_t resets_before = link_fake_reset_count();
      net_poll(false);
      /* deferred: the tear-down would put this pass over the watchdog grant */
      TEST_ASSERT_EQUAL_UINT16(desyncs_before, link_desyncs());
      TEST_ASSERT_EQUAL_UINT16(resets_before, link_fake_reset_count());
      pb_advance(2500);                            /* past the first backoff rung */
      net_poll(false);
      TEST_ASSERT_EQUAL_UINT16(desyncs_before + 1, link_desyncs());
      TEST_ASSERT_EQUAL_UINT16(resets_before + 1, link_fake_reset_count());
      TEST_ASSERT_EQUAL(NET_DOWN, net_state());   /* not NET_SOCK_CLOSE with a retry armed --
                                                      that is what a bare send failure does */
      return;
    }
    net_poll(false);
  }
  TEST_FAIL_MESSAGE("the FSM never reached NET_SEND");
}

static void test_a_modem_timeout_in_recv_poisons_the_link(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  for (int i = 0; i < 40; ++i) {
    link_fake_pass_begin();
    const net_state_t before = net_state();
    if (before == NET_RECV) {
      link_fake_timeout_next();     /* sock_read()'s own AT times out: r < 0, not r == 0 */
      const uint16_t desyncs_before = link_desyncs();
      const uint16_t resets_before = link_fake_reset_count();
      net_poll(false);
      /* deferred: the tear-down would put this pass over the watchdog grant */
      TEST_ASSERT_EQUAL_UINT16(desyncs_before, link_desyncs());
      TEST_ASSERT_EQUAL_UINT16(resets_before, link_fake_reset_count());
      pb_advance(2500);                            /* past the first backoff rung */
      net_poll(false);
      TEST_ASSERT_EQUAL_UINT16(desyncs_before + 1, link_desyncs());
      TEST_ASSERT_EQUAL_UINT16(resets_before + 1, link_fake_reset_count());
      TEST_ASSERT_EQUAL(NET_DOWN, net_state());   /* not NET_SOCK_CLOSE with a retry armed --
                                                      that is what the deadline-expiry exit does */
      return;
    }
    net_poll(false);
  }
  TEST_FAIL_MESSAGE("the FSM never reached NET_RECV");
}

/* Two report cycles with no net_begin() between them: NET_IDLE must reset the spent retry,
   or the second report is abandoned after one send. */
static void test_each_report_gets_its_own_single_retry(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response("", 0);              /* round 1: the server never answers at all */
  const int sends1 = run_passes(120, 200);      /* 24 s: two RECV deadlines, inside the
                                                   30 s retry window */
  TEST_ASSERT_EQUAL_INT(2, sends1);             /* the original and its one retry */
  TEST_ASSERT_EQUAL(NET_IDLE, net_state());

  pb_advance(60000);            /* the interval is still 60 s: no 200 ever changed it */
  link_fake_queue_response("", 0);              /* round 2: also nothing, ever */
  const int sends2 = run_passes(120, 200);
  TEST_ASSERT_EQUAL_INT(2, sends2);             /* round 2 gets its OWN retry, not zero */
}

/* Every fake hal_millis() read costs a tick, so through net_poll() a timeout always reads
   PB_NET_STEP_MS + 1, where >= and > agree; only a direct call lands on the boundary itself. */
static void test_was_timeout_boundary_is_inclusive(void) {
  const uint32_t t0 = hal_millis();
  sim_advance(PB_NET_STEP_MS - 1u);   /* + netfsm_test_was_timeout_()'s own hal_millis() tick
                                          == exactly PB_NET_STEP_MS elapsed */
  TEST_ASSERT_TRUE(netfsm_test_was_timeout_(t0));

  const uint32_t t1 = hal_millis();
  sim_advance(PB_NET_STEP_MS - 2u);   /* + the same tick == PB_NET_STEP_MS - 1: one short */
  TEST_ASSERT_FALSE(netfsm_test_was_timeout_(t1));
}

static void test_link_drop_returns_to_joining_with_exponential_backoff(void) {
  static const uint32_t ladder[] = PB_NET_BACKOFF_MS;
  sensors_begin();
  net_begin();
  /* join, then pull the AP out from under it */
  for (int i = 0; i < 8 && net_state() != NET_IDLE; ++i) { link_fake_pass_begin(); net_poll(false); }
  TEST_ASSERT_EQUAL(NET_IDLE, net_state());
  link_fake_drop_link();
  uint32_t seen[3] = {0, 0, 0};
  for (int rung = 0; rung < 3; ++rung) {
    /* drive until the FSM parks in NET_DOWN, then measure how long it waits */
    for (int i = 0; i < 60 && net_state() != NET_DOWN; ++i) {
      link_fake_pass_begin(); net_poll(false); sim_advance(50);
    }
    TEST_ASSERT_EQUAL(NET_DOWN, net_state());
    uint32_t waited = 0;
    while (net_state() == NET_DOWN && waited < 60000) {
      link_fake_pass_begin(); net_poll(false); sim_advance(100); waited += 100;
    }
    seen[rung] = waited;
    /* still down: the fake's join succeeds unconditionally, so a dropped link alone would
       reassociate on the next status query and reset the backoff; a timed-out AT is the only
       "the join itself failed" the fake has, and it climbs the same ladder */
    link_fake_timeout_next();
  }
  TEST_ASSERT_TRUE(seen[0] <= ladder[0] + 200);
  TEST_ASSERT_TRUE(seen[1] > seen[0]);
  TEST_ASSERT_TRUE(seen[2] > seen[1]);
}

static void test_a_poisoned_close_does_not_leave_starvation_armed_for_the_next_report(void) {
  sensors_begin();
  net_begin();

  /* arm the starvation flag: a clean, no-timeout open failure on the first attempt and on
     the retry is the only way NET_CONNECT sets it */
  link_fake_fail_open(true);
  int guard = 0;
  while (net_reports_failed() < 2u && guard++ < 200) net_poll(false);
  TEST_ASSERT_EQUAL_UINT32(2u, net_reports_failed());   /* the open AND its retry both failed */
  TEST_ASSERT_EQUAL(NET_SOCK_CLOSE, net_state());       /* and the socket is still allocated */

  /* poison the NET_SOCK_CLOSE pass that would have consumed the flag: the timeout check
     returns early above the consumer, and the JOIN_WAIT exit never touches it */
  link_fake_timeout_next();
  net_poll(false);
  TEST_ASSERT_EQUAL(NET_DOWN, net_state());

  /* a clean report must now complete; a stale flag would drop a working link in the next
     SOCK_CLOSE pass, once per report, forever */
  link_fake_fail_open(false);
  link_fake_queue_response(k200, strlen(k200));
  int g2 = 0;
  while (net_reports_ok() == 0u && g2++ < 300) pb_net_passes(1, 1000u);
  TEST_ASSERT_EQUAL_UINT32(1u, net_reports_ok());
}

static void test_an_ack_is_set_the_moment_a_command_is_received(void) {
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, strlen(k_cmd_200));
  net_begin(); exec_begin();
  pb_net_passes(12, 100);
  TEST_ASSERT_TRUE(report_ack_is_recv());     /* (id, flow_ml = 0, err = "recv") on RECEIPT */
  TEST_ASSERT_FALSE(report_may_build());
}

static void test_command_is_not_executed_in_the_pass_that_received_it(void) {
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, strlen(k_cmd_200));
  net_begin(); exec_begin();
  for (int i = 0; i < 40; ++i) {
    link_fake_pass_begin();
    net_poll(false);
    if (report_ack_is_recv()) {               /* the pass that received it */
      TEST_ASSERT_EQUAL_UINT32(0, sim_pump_on_ms());
      break;
    }
    pb_advance(100);
  }
}

static void test_command_is_surfaced_only_once_per_round_trip(void) {
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, strlen(k_cmd_200));
  net_begin(); exec_begin();
  pb_net_passes(12, 100);
  cmd_t c;
  TEST_ASSERT_TRUE(net_take_command(&c));
  TEST_ASSERT_EQUAL_UINT32(17, c.id);
  TEST_ASSERT_FALSE(net_take_command(&c));
}

static void test_no_report_is_built_between_receiving_a_command_and_executing_it(void) {
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, strlen(k_cmd_200));
  net_begin(); exec_begin();
  pb_net_passes(12, 100);
  /* the write count, not the last buffer: the question is whether anything at all was sent */
  uint16_t writes = link_fake_write_count();
  pb_advance(120000);                          /* two report intervals go by */
  pb_net_passes(20, 100);
  TEST_ASSERT_EQUAL_UINT16(writes, link_fake_write_count());   /* the report WAITS */
}

static void test_a_stop_command_is_acked(void) {
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_stop_200, strlen(k_stop_200));
  net_begin(); exec_begin();
  pb_net_passes(14, 100);
  exec_pending();
  TEST_ASSERT_FALSE(report_ack_is_recv());
  report_stamp();
  char b[PB_BODY_CAP]; (void)report_build(b, sizeof b);
  TEST_ASSERT_NOT_NULL(strstr(b, " ack=31 flow_ml=0 err=stop"));
}

static void test_a_failed_goto_still_acks(void) {
  /* PB_PULSES_PER_GATE == 0 compiles cart_goto() to a refusal: the shipped configuration */
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, strlen(k_cmd_200));
  net_begin(); exec_begin();
  pb_net_passes(14, 100);
  exec_pending();
  report_stamp();
  char b[PB_BODY_CAP]; (void)report_build(b, sizeof b);
  TEST_ASSERT_NOT_NULL(strstr(b, " ack=17 flow_ml=0 err=goto"));
}

static void test_an_out_of_range_outlet_acks_range_and_never_reaches_the_cart(void) {
  /* The range check sits above cart_goto() so the backend is told which refusal it got; both
     bounds, because water=0 is legal for butler to send and 0 and 6 fail different halves of
     the compare. The check runs before the cart is consulted, so it holds on every arm. */
  static const struct { const char *body; const char *want; } k[] = {
    { "HTTP/1.1 200 OK\r\nContent-Length: 39\r\n\r\nnext=60\ncmd=51 water=0 ml=100 cap_s=10\n",
      " ack=51 flow_ml=0 err=range" },
    { "HTTP/1.1 200 OK\r\nContent-Length: 39\r\n\r\nnext=60\ncmd=52 water=6 ml=100 cap_s=10\n",
      " ack=52 flow_ml=0 err=range" },
  };
  for (unsigned i = 0; i < sizeof k / sizeof k[0]; ++i) {
    pb_test_setup();
    link_fake_reset(); link_fake_set_state(LINK_UP);
    link_fake_queue_response(k[i].body, strlen(k[i].body));
    net_begin(); exec_begin();
    pb_net_passes(14, 100);
    exec_pending();
    report_stamp();
    char b[PB_BODY_CAP]; (void)report_build(b, sizeof b);
    TEST_ASSERT_NOT_NULL(strstr(b, k[i].want));
    TEST_ASSERT_NULL(strstr(b, "err=goto"));   /* it never got as far as the cart */
  }
}

static void test_every_terminal_path_in_exec_pending_sets_an_ack(void) {
  static const char *bodies[] = {
    "HTTP/1.1 200 OK\r\nContent-Length: 39\r\n\r\nnext=60\ncmd=41 water=0 ml=100 cap_s=10\n",
    "HTTP/1.1 200 OK\r\nContent-Length: 39\r\n\r\nnext=60\ncmd=42 water=9 ml=100 cap_s=10\n",
    "HTTP/1.1 200 OK\r\nContent-Length: 39\r\n\r\nnext=60\ncmd=43 water=3 ml=100 cap_s=10\n",
    "HTTP/1.1 200 OK\r\nContent-Length: 22\r\n\r\nnext=60\ncmd=44 stop=1\n"
  };
  for (unsigned i = 0; i < 4; ++i) {
    pb_test_setup();
    link_fake_reset(); link_fake_set_state(LINK_UP);
    link_fake_queue_response(bodies[i], strlen(bodies[i]));
    net_begin(); exec_begin();
    pb_net_passes(14, 100);
    exec_pending();
    TEST_ASSERT_FALSE(report_ack_is_recv());
    report_stamp();
    char b[PB_BODY_CAP]; (void)report_build(b, sizeof b);
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(b, " ack="), "a terminal path left no ack");
    TEST_ASSERT_NOT_NULL(strstr(b, " flow_ml="));
  }
}

static void test_refused_dose_acks_with_flow_ml_zero_and_an_err_token(void) {
  static const char k[] =
    "HTTP/1.1 200 OK\r\nContent-Length: 39\r\n\r\nnext=60\ncmd=51 water=3 ml=999 cap_s=10\n";
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k, strlen(k));
  net_begin(); exec_begin();
  pb_net_passes(14, 100);
  exec_pending();
  report_stamp();
  char b[PB_BODY_CAP]; (void)report_build(b, sizeof b);
  TEST_ASSERT_NOT_NULL(strstr(b, "flow_ml=0"));   /* an ACKED refusal charges the pot 0 ml */
}

static void test_pending_ack_rides_the_next_report_after_every_discard_path(void) {
  static const char k_400[] = "HTTP/1.1 400 Bad Request\r\nContent-Length: 3\r\n\r\nno\n";
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, strlen(k_cmd_200));
  net_begin(); exec_begin();
  pb_net_passes(14, 100);
  exec_pending();                       /* ack=17 is now real */
  link_fake_queue_response(k_400, strlen(k_400));
  pb_net_passes(30, 2500);                 /* that report 400s and is discarded */
  uint16_t n = 0;
  const char *tx = (const char *)link_fake_sent(&n);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_NOT_NULL(strstr(tx, " ack=17 "));                 /* still on the next one */
}

static void test_err_recv_never_reaches_the_wire(void) {
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, strlen(k_cmd_200));
  net_begin(); exec_begin();
  pb_net_passes(60, 2500);                 /* many intervals; exec runs, then reports resume */
  uint16_t n = 0;
  const char *tx = (const char *)link_fake_sent(&n);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_NULL(strstr(tx, "err=recv"));
}

/* Under PB_PULSES_PER_GATE == 0 cart_goto() always fails, so the dose and its summary line
   are never reached; proven under native_cal. */
static void test_a_backend_dose_prints_the_per_dose_summary_line(void) {
#if PB_PULSES_PER_GATE == 0
  TEST_IGNORE_MESSAGE("uncalibrated arm: cart_goto() always fails, so dose_run() is never "
                       "reached; see native_cal");
#else
  pb_test_setup();
  /* cart_goto() self-homes first, which needs a turning screw and a home region; cart_begin()
     also resets cart.cpp's process-lifetime statics an earlier case may have dirtied. */
  cart_begin();
  sim_set_screw_pulse_ms(2);
  sim_set_home_region(0, 40);
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, sizeof k_cmd_200 - 1u);
  net_begin(); exec_begin();
  pb_net_passes(14, 100);
  char out[512]; (void)sim_serial_tx(out, sizeof out);
  exec_pending();
  size_t n = sim_serial_tx(out, sizeof out); out[n] = 0;
  TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, "dose outlet="), out);
#endif
}

/* A garbage hang field plus a long enough dose puts a backend command into the loop that
   deliberately starves the watchdog; zero-initialising the request is the only defence.
   Under the uncalibrated arm the dose is never reached and the case would pass vacuously. */
static void test_a_backend_command_never_sets_hang(void) {
#if PB_PULSES_PER_GATE == 0
  TEST_IGNORE_MESSAGE("uncalibrated arm: cart_goto() always fails, so dose_run() is never "
                       "reached; see native_cal");
#else
  pb_test_setup();
  cart_begin();                    /* same reason as the summary-line case immediately above */
  sim_set_screw_pulse_ms(2);
  sim_set_home_region(0, 40);
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, sizeof k_cmd_200 - 1u);
  net_begin(); exec_begin();
  pb_net_passes(14, 100);
  uint32_t f0 = sim_feeds();
  pb_advance(PB_HANG_MS * 3u);
  exec_pending();
  TEST_ASSERT_GREATER_THAN_UINT32(f0, sim_feeds());   /* the dog was fed throughout */

  /* Feeds alone cannot prove it: PB_HANG_MS and PB_PRIME_MS_DEFAULT are both 3000, so the
     no-flow abort ends the dose on the millisecond starvation would begin. */
  TEST_ASSERT_FALSE(exec_test_last_req_().hang);
#endif
}

/* Every other exec case here ends in a refusal or an abort, where 0 ml is honest either way.
   The backend writes the acked flow_ml, alarms when it is under half the request and charges
   the daily cap with it, so a constant zero would report every good watering as a failure.
   The dose needs flow and a clock past PB_BOOT_GAP_MS, or the ladder refuses with boot and
   the summary line looks the same. */
static void test_a_granted_backend_dose_acks_the_millilitres_that_actually_flowed(void) {
#if PB_PULSES_PER_GATE == 0
  TEST_IGNORE_MESSAGE("uncalibrated arm: cart_goto() always fails, so a granted dose is never "
                       "reached; see native_cal");
#else
  pb_test_setup();
  cart_begin();                    /* same reason as the two cases immediately above */
  sim_set_screw_pulse_ms(2);
  sim_set_home_region(0, 40);
  sim_set_float(true);
  /* 85 ml/s is 499 pulses/s at the meter's 5880/L, so ml=100 (588 pulses) lands in about
     1.2 s, inside cap_s=10 and under PB_FLOW_MAX_HZ */
  sim_set_flow_ml_s(85);
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, sizeof k_cmd_200 - 1u);
  net_begin(); exec_begin();
  pb_net_passes(14, 100);
  pb_advance(PB_BOOT_GAP_MS + 1u);           /* or the ladder answers boot, not water */
  exec_pending();

  TEST_ASSERT_EQUAL_MESSAGE(DOSE_OK, dose_last_result(),
      "arrange: this case is worthless unless the dose was actually granted and reached target");
  TEST_ASSERT_TRUE_MESSAGE(dose_flow_ml() >= 95u && dose_flow_ml() <= 110u,
      "arrange: about 100 ml must have moved, or the number under test is not a real one");

  report_stamp();
  char b[PB_BODY_CAP]; (void)report_build(b, sizeof b);
  char want[24];
  snprintf(want, sizeof want, " flow_ml=%u", (unsigned)dose_flow_ml());
  TEST_ASSERT_NOT_NULL_MESSAGE(strstr(b, want), b);      /* the HONEST millilitres, on the wire */
  TEST_ASSERT_NOT_NULL_MESSAGE(strstr(b, " ack=17"), b);
#endif
}

static void test_the_cached_accessors_fill_after_a_join_and_cost_nothing(void) {
  sensors_begin();
  net_begin();
  /* Before any join the cache must read as "no link" rather than as leftovers. */
  TEST_ASSERT_EQUAL_UINT8(0, net_link());
  TEST_ASSERT_EQUAL_INT8(0, net_rssi());
  TEST_ASSERT_EQUAL_STRING("0.0.0.0", net_ip());

  link_fake_queue_response(k200, strlen(k200));
  pb_net_passes(20, 100);
  TEST_ASSERT_EQUAL_UINT8(2, net_link());                    /* LINK_UP, cached in JOIN_WAIT */
  TEST_ASSERT_EQUAL_INT8(-52, net_rssi());                   /* the fake's fixed answers, so */
  TEST_ASSERT_EQUAL_STRING("192.168.1.42", net_ip());        /* the refresh passes really ran */

  /* reading the cache is free: ui_fill_() calls all three every loop pass, and the real
     driver would stack ~5 ATs on the FSM's own */
  link_fake_pass_begin();
  (void)net_link(); (void)net_rssi(); (void)net_ip(); (void)net_desyncs();
  TEST_ASSERT_EQUAL_UINT16(0, link_fake_at_count());
}

static void test_a_dropped_link_clears_the_cached_signal_and_address(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  pb_net_passes(20, 100);
  TEST_ASSERT_EQUAL_STRING("192.168.1.42", net_ip());        /* armed, so the clear is visible */

  /* a join the FSM gives up on must not leave status showing a dead link's address */
  link_fake_drop_link();
  int guard = 0;
  while (net_state() != NET_DOWN && guard++ < 200) {
    link_fake_timeout_next();
    pb_net_passes(1, 1000);
  }
  TEST_ASSERT_EQUAL(NET_DOWN, net_state());
  TEST_ASSERT_EQUAL_UINT8(0, net_link());
  TEST_ASSERT_EQUAL_INT8(0, net_rssi());
  TEST_ASSERT_EQUAL_STRING("0.0.0.0", net_ip());
}

/* The cache fills, but nothing above says in how many passes. netfsm.cpp keeps the two
   refreshes in separate 1-AT passes -- 3 x 1200 + PB_NET_SLACK_MS = 5600 ms in one pass
   would sit against the 5592 ms watchdog grant -- and the only thing
   implementing that is the return that ends the RSSI pass. Each refresh writes its own fixed
   value, so which pass each answer arrived in is observable. */
static void test_the_signal_and_address_refreshes_never_share_a_pass(void) {
  sensors_begin();
  net_begin();

  /* one pass at a time, to the transition into NET_IDLE: it arms both refreshes and does
     neither */
  int guard = 0;
  while (net_state() != NET_IDLE && guard++ < 40) pb_net_passes(1, 100);
  TEST_ASSERT_EQUAL_MESSAGE(NET_IDLE, net_state(), "arrange: the FSM never joined");
  TEST_ASSERT_EQUAL_INT8_MESSAGE(0, net_rssi(), "arrange: nothing may be refreshed yet");
  TEST_ASSERT_EQUAL_STRING_MESSAGE("0.0.0.0", net_ip(), "arrange: nothing may be refreshed yet");

  pb_net_passes(1, 100);                       /* the signal refresh, and nothing else */
  TEST_ASSERT_EQUAL_INT8_MESSAGE(-52, net_rssi(), "the signal refresh did not run in its pass");
  TEST_ASSERT_EQUAL_STRING_MESSAGE("0.0.0.0", net_ip(),
      "the address refresh ran in the SAME pass as the signal refresh: netfsm.cpp keeps "
      "each refresh in a 1-AT pass of its own, for the margin under the watchdog grant");

  pb_net_passes(1, 100);                       /* the address refresh, in a pass of its own */
  TEST_ASSERT_EQUAL_STRING_MESSAGE("192.168.1.42", net_ip(),
      "the address refresh did not run in the pass after the signal refresh");
}

/* Both refresh passes carry the timeout-then-poison pairing every AT-issuing pass has; the
   fake charges one AT for each, so a timeout in either must tear the session down like any
   other. */
static void arrange_idle_(void) {
  sensors_begin();
  net_begin();
  int guard = 0;
  while (net_state() != NET_IDLE && guard++ < 40) pb_net_passes(1, 100);
  TEST_ASSERT_EQUAL_MESSAGE(NET_IDLE, net_state(), "arrange: the FSM never joined");
}

static void test_a_timeout_in_the_signal_refresh_poisons_the_link(void) {
  arrange_idle_();
  const uint16_t desyncs = link_desyncs();
  link_fake_timeout_next();
  pb_net_passes(1, 100);                       /* the signal refresh, and it times out */
  TEST_ASSERT_EQUAL_MESSAGE(NET_DOWN, net_state(),
      "a modem timeout in the signal refresh did not poison the link");
  TEST_ASSERT_EQUAL_INT8(0, net_rssi());
  pb_advance(2500);                            /* past the first backoff rung */
  pb_net_passes(1, 100);                       /* the deferred tear-down */
  TEST_ASSERT_EQUAL_UINT16_MESSAGE(desyncs + 1, link_desyncs(), "the session was not torn down");
}

static void test_a_timeout_in_the_address_refresh_poisons_the_link(void) {
  arrange_idle_();
  pb_net_passes(1, 100);                       /* the signal refresh, cleanly */
  TEST_ASSERT_EQUAL_INT8_MESSAGE(-52, net_rssi(), "arrange: the signal refresh did not run");
  const uint16_t desyncs = link_desyncs();
  link_fake_timeout_next();
  pb_net_passes(1, 100);                       /* the address refresh, and it times out */
  TEST_ASSERT_EQUAL_MESSAGE(NET_DOWN, net_state(),
      "a modem timeout in the address refresh did not poison the link: on the board this is "
      "the pass that used to spend up to 125 s inside WiFi.localIP() against a 5592 ms grant");
  TEST_ASSERT_EQUAL_STRING("0.0.0.0", net_ip());
  pb_advance(2500);
  pb_net_passes(1, 100);
  TEST_ASSERT_EQUAL_UINT16_MESSAGE(desyncs + 1, link_desyncs(), "the session was not torn down");
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
  RUN_TEST(test_net_begin_clears_a_standing_disable_latch);
  RUN_TEST(test_a_failed_boot_assertion_survives_net_begin);
  RUN_TEST(test_a_join_deadline_is_not_expired_early_by_the_clock_rollover);
  RUN_TEST(test_a_backoff_wait_still_waits_across_the_clock_rollover);
  RUN_TEST(test_a_recv_deadline_is_not_expired_early_by_the_clock_rollover);
  RUN_TEST(test_an_exchange_that_produced_no_bytes_is_retried_exactly_once);
  RUN_TEST(test_a_retry_is_abandoned_rather_than_sent_outside_the_dedup_window);
  RUN_TEST(test_a_response_that_produced_any_bytes_is_never_retried);
  RUN_TEST(test_a_truncated_reply_is_never_retried);
  RUN_TEST(test_a_four_hundred_is_never_retried);
  RUN_TEST(test_a_five_hundred_is_not_retried);
  RUN_TEST(test_a_five_oh_three_is_retried_once);
  RUN_TEST(test_report_body_is_byte_identical_on_the_retry);
  RUN_TEST(test_a_modem_timeout_poisons_the_link_and_counts_a_desync);
  RUN_TEST(test_a_modem_timeout_in_join_wait_poisons_the_link);
  RUN_TEST(test_a_modem_timeout_in_sock_close_poisons_the_link);
  RUN_TEST(test_a_modem_timeout_in_send_poisons_the_link);
  RUN_TEST(test_a_modem_timeout_in_recv_poisons_the_link);
  RUN_TEST(test_each_report_gets_its_own_single_retry);
  RUN_TEST(test_a_poisoned_close_does_not_leave_starvation_armed_for_the_next_report);
  RUN_TEST(test_was_timeout_boundary_is_inclusive);
  RUN_TEST(test_link_drop_returns_to_joining_with_exponential_backoff);
  RUN_TEST(test_an_ack_is_set_the_moment_a_command_is_received);
  RUN_TEST(test_command_is_not_executed_in_the_pass_that_received_it);
  RUN_TEST(test_command_is_surfaced_only_once_per_round_trip);
  RUN_TEST(test_no_report_is_built_between_receiving_a_command_and_executing_it);
  RUN_TEST(test_a_stop_command_is_acked);
  RUN_TEST(test_a_failed_goto_still_acks);
  RUN_TEST(test_an_out_of_range_outlet_acks_range_and_never_reaches_the_cart);
  RUN_TEST(test_every_terminal_path_in_exec_pending_sets_an_ack);
  RUN_TEST(test_refused_dose_acks_with_flow_ml_zero_and_an_err_token);
  RUN_TEST(test_pending_ack_rides_the_next_report_after_every_discard_path);
  RUN_TEST(test_err_recv_never_reaches_the_wire);
  RUN_TEST(test_a_backend_dose_prints_the_per_dose_summary_line);
  RUN_TEST(test_a_backend_command_never_sets_hang);
  RUN_TEST(test_a_granted_backend_dose_acks_the_millilitres_that_actually_flowed);
  RUN_TEST(test_the_cached_accessors_fill_after_a_join_and_cost_nothing);
  RUN_TEST(test_a_dropped_link_clears_the_cached_signal_and_address);
  RUN_TEST(test_the_signal_and_address_refreshes_never_share_a_pass);
  RUN_TEST(test_a_timeout_in_the_signal_refresh_poisons_the_link);
  RUN_TEST(test_a_timeout_in_the_address_refresh_poisons_the_link);
  return UNITY_END();
}
