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
     pump_passes() (this file's alias for it with ms hardwired to 0) cannot supply. Task 25:
     zero response bytes is retry-eligible, so this scenario now runs the SEND/RECV leg twice
     (the original, then the one retry) before the report is finally abandoned -- two 5 s
     PB_NET_DEADLINE_MS waits, not one. 40 passes x 500 ms (20 s) comfortably crosses both with
     passes to spare for the SOCK_CLOSE/IDLE cleanup that follows. */
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

static void test_net_begin_clears_a_standing_disable_latch(void) {
  net_disable("heap");
  TEST_ASSERT_NOT_NULL(net_disabled());
  net_begin();
  /* Without this every later case in the file would poll a dead FSM and pass on a no-op.
     g_disabled is the one static net_begin() did not clear, and net_disable() is the only
     writer, so nothing before task 25 could have caught it. */
  TEST_ASSERT_NULL(net_disabled());
  net_poll(false);
  TEST_ASSERT_NOT_EQUAL(NET_DOWN, net_state());   /* it actually runs again */
}

static void test_a_join_deadline_is_not_expired_early_by_the_clock_rollover(void) {
  /* Armed BEFORE net_begin(): every timestamp in the FSM is relative to the one before it, and
     a board 49.7 days up has all of them up here together. */
  sim_set_clock_ms(0xFFFFF000u);
  net_begin();
  link_fake_drop_link();

  net_poll(false);                             /* NET_DOWN -> NET_JOIN_ISSUE */
  net_poll(false);                             /* issues the join; the 5 s deadline WRAPS */
  TEST_ASSERT_EQUAL(NET_JOIN_WAIT, net_state());

  /* An unsigned `hal_millis() >= g_deadline` compares a pre-wrap clock against a post-wrap
     deadline and calls it expired on the first pass -- abandoning a join that had 5 s to run.

     link_join() just set g_join_pending true; task 25 makes EVERY timed-out AT call poison the
     link (spec's "any modem timeout"), so the old trick of forcing a timeout on link_state()'s
     own AT to advance the clock without flipping to LINK_UP no longer works here -- it would
     poison straight back to NET_DOWN and never reach the deadline check this test exists to
     pin. link_fake_drop_link() clears g_join_pending again with ZERO ATs, so the next
     link_state() call is a normal, FAST, non-timeout round trip that genuinely reports
     LINK_DOWN -- exactly how a real board sees a join still in flight -- while pb_advance()
     supplies the same elapsed time the old (1200 + 100) ms combination did. */
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

  /* Jump to just under the wrap BEFORE the RECV deadline is armed -- ~3.1 days, inside the
     idiom's 2^31 validity. Arming it after the jump is what makes it straddle the wrap. */
  sim_set_clock_ms(0xFFFFF830u);
  net_poll(false);                             /* NET_SEND -> NET_RECV, deadline WRAPS */
  TEST_ASSERT_EQUAL(NET_RECV, net_state());

  /* Nothing is queued, so sock_read() returns 0 and only the deadline can end this pass. An
     unsigned compare calls the wrapped deadline expired at once and discards a report that
     still had 5 s to arrive. */
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

  /* Jump to just under the wrap -- well inside the idiom's 2^31 validity -- so the join
     deadline is genuinely past and link_down() arms its 2 s backoff across the wrap. The exact
     value matters: the timed-out status query below advances the fake clock by PB_NET_STEP_MS,
     and link_down() has to run while hal_millis() is STILL below the wrap or the backoff it
     arms never straddles it and this test proves nothing. 0xFFFFF830 leaves ~800 ms to spare. */
  sim_set_clock_ms(0xFFFFF830u);
  link_fake_timeout_next();
  net_poll(false);
  TEST_ASSERT_EQUAL(NET_DOWN, net_state());

  /* An unsigned `hal_millis() < g_wait_until` reads the wrapped deadline as already past and
     re-joins at once, hammering the modem exactly when the link is worst. */
  net_poll(false);
  TEST_ASSERT_EQUAL(NET_DOWN, net_state());

  pb_advance(2500);                            /* now the 2 s backoff really has elapsed */
  net_poll(false);
  TEST_ASSERT_EQUAL(NET_JOIN_ISSUE, net_state());
}

/* Passes with wall clock, because the RECV deadline and the retry deadline are both in ms.
   This is pb_net_passes(1, ms_each) with a send counter wrapped round it — one pass at a
   time, so that the counter can see the state the pass STARTED in. Do not re-derive the pass
   itself here: harness.h's helper is the one spelling (task 24 step 1). */
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
  sim_reset(true);                        /* WARM: the boot counter advances, so the salt is
                                             non-zero and t= is above 2^31 (spec §15.2) */
  sensors_begin();
  TEST_ASSERT_NOT_EQUAL(0, hal_boot_salt());
  net_begin();
  link_fake_queue_response("", 0);
  /* inside the window: the retry IS sent */
  TEST_ASSERT_EQUAL_INT(2, run_passes(120, 200));
  /* a fresh report, then let the retry deadline expire before the FSM can resend. Walk until
     the ORIGINAL send has gone out, RECV has timed out and the retry is ARMED but not yet
     re-transmitted (net_state() == NET_SOCK_CLOSE, the one pass between "finish() decided to
     retry" and "CONNECT dials again") -- a fixed pass count here would be guessing the exact
     boundary between "timeout just detected" and "already walked into CONNECT for the retry",
     and NET_SOCK_CLOSE's own re-check of the window (the point of this case) only has
     something to prove if the clock is pushed past the deadline BEFORE that walk, not after. */
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
  TEST_ASSERT_EQUAL_INT(2, run_passes(60, 200));   /* sqlite3.OperationalError rolls it all back */
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
      TEST_ASSERT_EQUAL_UINT16(desyncs_before + 1, link_desyncs());   /* rides out as ch206 */
      TEST_ASSERT_EQUAL_UINT16(resets_before + 1, link_fake_reset_count());
      TEST_ASSERT_EQUAL(NET_DOWN, net_state());        /* the NEXT command is NOT issued */
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

/* Fix round 1, finding 2: the review disabled each of the six "any modem timeout poisons"
   brackets in turn and found four of them uncaught by anything in this suite --
   NET_JOIN_WAIT, NET_SOCK_CLOSE, NET_SEND and NET_RECV's r<0 branch. Only NET_CONNECT
   (above) and NET_JOIN_ISSUE (test_link_drop_returns_to_joining_with_exponential_backoff,
   below) were proven. One case per unproven site, same shape as the one above: force a
   timeout on that site's own AT round trip and prove BOTH halves of poison() fired
   (link_reset()'s desync count, link_down()'s NET_DOWN) -- either alone leaves a live bug
   this bracket exists to catch. */

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
  TEST_ASSERT_EQUAL_UINT16(desyncs_before + 1, link_desyncs());
  TEST_ASSERT_EQUAL_UINT16(resets_before + 1, link_fake_reset_count());
  TEST_ASSERT_EQUAL(NET_DOWN, net_state());      /* not left sitting in JOIN_WAIT for the
                                                     5 s deadline to (much later) also catch */
}

static void test_a_modem_timeout_in_sock_close_poisons_the_link(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  /* the FIRST NET_SOCK_CLOSE of a report (right after IDLE) never opened a socket, so its
     own sock_close() costs 0 ATs and can never time out (§3's table: 0 or 1 AT). Walk past
     a full successful round trip to the SECOND one, entered from NET_CLOSE with the socket
     CONNECT opened still allocated -- that is the one sock_close() actually has an AT to
     lose. */
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
      TEST_ASSERT_EQUAL_UINT16(desyncs_before + 1, link_desyncs());
      TEST_ASSERT_EQUAL_UINT16(resets_before + 1, link_fake_reset_count());
      TEST_ASSERT_EQUAL(NET_DOWN, net_state());
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

/* Fix round 1, finding 3: NET_IDLE's own `g_retried = false; g_connect_starved = false;`
   has no test that can fail, because every existing case drives exactly one report cycle
   and net_begin()'s OWN reset (called once, at the top of the case) masks the gap. Drive
   TWO report cycles back to back with no intervening net_begin() -- if the second report
   inherited the first's spent retry, it would send once and be abandoned instead of
   retrying, because finish()'s `!g_retried` would already read false walking in. */
static void test_each_report_gets_its_own_single_retry(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response("", 0);              /* round 1: the server never answers at all */
  const int sends1 = run_passes(120, 200);      /* 24 s: two RECV deadlines, inside the 30 s
                                                    retry window -- same budget as
                                                    test_an_exchange_that_produced_no_bytes_
                                                    is_retried_exactly_once, above */
  TEST_ASSERT_EQUAL_INT(2, sends1);             /* the original and its one retry */
  TEST_ASSERT_EQUAL(NET_IDLE, net_state());

  pb_advance(60000);            /* g_next_s is still 60 (no 200 ever arrived to change it),
                                    so this alone makes round 2 due */
  link_fake_queue_response("", 0);              /* round 2: also nothing, ever */
  const int sends2 = run_passes(120, 200);
  TEST_ASSERT_EQUAL_INT(2, sends2);             /* round 2 gets its OWN retry, not zero */
}

/* Fix round 1, finding 4: was_timeout()'s `(int32_t)(hal_millis() - t0) >= PB_NET_STEP_MS`
   idiom is never exercised at the exact boundary through net_poll() -- hal_sim.cpp's own
   "every hal_millis() read advances the rig by 1 ms" contract means capturing t0 costs one
   tick and was_timeout()'s own hal_millis() call costs a second, so every timeout net_poll()
   can ever manufacture reads PB_NET_STEP_MS + 1, never PB_NET_STEP_MS itself -- verified by
   hand-tracing every poison test above. >= and > agree on PB_NET_STEP_MS + 1, so none of
   the four poison tests just added above, nor the pre-existing ones, can tell the two
   forms apart.
   netfsm_test_was_timeout_() calls the real (only) copy of the comparison directly against a
   clock landed on the boundary by hand, via sim_advance(), so the +1 never happens. */
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
    /* still down: the join fails again. link_fake_drop_link() alone cannot express that here --
       link_join() succeeds UNCONDITIONALLY in the fake (it never consults g_state), so a drop
       applied before JOIN_ISSUE runs is silently undone the moment link_join()'s own two ATs
       set g_join_pending, and the very next link_state() call flips straight to LINK_UP,
       resetting g_backoff_i and erasing the exponential progression this test exists to pin.
       A timed-out AT is the fake's only real "the join itself failed" primitive, and task 25
       makes that poison() -> link_reset() + the SAME link_down() ladder -- exactly the
       repeated-failure shape a real dropped AP produces, and the only one that keeps
       g_backoff_i climbing instead of being reset by an accidental reassociation. */
    link_fake_timeout_next();
  }
  TEST_ASSERT_TRUE(seen[0] <= ladder[0] + 200);
  TEST_ASSERT_TRUE(seen[1] > seen[0]);
  TEST_ASSERT_TRUE(seen[2] > seen[1]);
}

static void test_a_poisoned_close_does_not_leave_starvation_armed_for_the_next_report(void) {
  sensors_begin();
  net_begin();

  /* Arm g_connect_starved: sock_open() fails cleanly (no timeout) on the first attempt AND on
     the retry, which is the only way NET_CONNECT sets it. */
  link_fake_fail_open(true);
  int guard = 0;
  while (net_reports_failed() < 2u && guard++ < 200) net_poll(false);
  TEST_ASSERT_EQUAL_UINT32(2u, net_reports_failed());   /* the open AND its retry both failed */
  TEST_ASSERT_EQUAL(NET_SOCK_CLOSE, net_state());       /* and the socket is still allocated */

  /* Now poison the very NET_SOCK_CLOSE pass that would have CONSUMED the flag. The timeout
     check sits above the consumer and returns early, so g_connect_starved survives the pass.
     NET_JOIN_WAIT's LINK_UP exit then goes straight to NET_IDLE without ever touching it --
     which is why NET_IDLE's own reset is the only thing that clears it. */
  link_fake_timeout_next();
  net_poll(false);
  TEST_ASSERT_EQUAL(NET_DOWN, net_state());

  /* A clean, fully successful report from here must end parked in NET_IDLE. With NET_IDLE's
     g_connect_starved reset deleted, the stale flag fires in the NEXT report's SOCK_CLOSE pass
     and calls link_down() on a link that never misbehaved -- the board drops a working
     connection once per report, forever. */
  link_fake_fail_open(false);
  link_fake_queue_response(k200, strlen(k200));
  int g2 = 0;
  while (net_reports_ok() == 0u && g2++ < 300) pb_net_passes(1, 1000u);
  /* With NET_IDLE's g_connect_starved reset deleted, this never becomes true: the stale flag
     fires in every subsequent SOCK_CLOSE pass, link_down()s a link that never misbehaved, and
     the board can no longer complete a report at all. */
  TEST_ASSERT_EQUAL_UINT32(1u, net_reports_ok());
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
  return UNITY_END();
}
