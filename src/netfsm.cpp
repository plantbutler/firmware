/* src/netfsm.cpp — the report state machine and the HTTP framing, above seam 2.
   §9 greps this file for ZERO hits of the safety header, the dosing entry point and the pump
   write. That is the direction that matters: one include of the safety header added here
   during a later change puts a call that asserts D6 one edit away from a state whose socket
   is open, with the build and make check both green.
   NO signed integer conversion in any format string (§9): every numeric site is %lu with an
   explicit cast. */
#include "netfsm.h"
#include "link.h"
#include "report.h"
#include "sensors.h"
#include "cart.h"
#include "config.h"
#include "hal.h"
#include "ui.h"          /* ui_modem_ran() ONLY. §9's grep over this file names the safety
                            header, the dosing entry point and the pump write, and ui.h is
                            none of those. */
#include "secrets.h"
#include <stdio.h>
#include <string.h>

/* THE DOSING FLAG IS A PARAMETER, not a forward declaration. §9's grep keeps the safety header
   out of this file, and an `extern bool safety_dosing(void);` here would be that include by
   another name; loop() supplies the flag instead (task 12 step 4, task 26 step 13b). Spec §3,
   "include hygiene, in both directions". */

static_assert(sizeof(HOST_NAME) + sizeof(BUTLER_TOKEN) + PB_HDR_FIXED + PB_BODY_CAP <= PB_TX_CAP,
              "HOST_NAME + BUTLER_TOKEN + headers + body do not fit PB_TX_CAP: snprintf would "
              "truncate silently while Content-Length still claimed the full length (spec §4.2)");
static_assert(sizeof(PB_CONTROLLER) + 2 + PB_BODY_WORST_FIXED <= PB_BODY_CAP, "body cap (§7)");
static_assert(sizeof(PB_CONTROLLER) > 1, "an empty c= is a permanent 400 (butler.py:252-253)");

static const uint32_t k_backoff[] = PB_NET_BACKOFF_MS;

static net_state_t g_state;
static const char *g_disabled;
static uint16_t g_status;
static uint16_t g_next_s = 60;
static uint32_t g_ok, g_failed;
static bool     g_modem_ran;
static uint8_t  g_backoff_i;
static uint32_t g_wait_until, g_deadline, g_last_report_ms;
static bool     g_first_report_due;
static bool     g_retried;                  /* each report gets its own single retry */
/* Set only inside NET_CONNECT's failure branch, and only once the retry is already exhausted
   (g_retried was already true): TWO consecutive attempts that cannot even OPEN a socket,
   neither of them a modem timeout, is the fake's own model of a link that silently dropped
   between reports (link_fake_drop_link()) -- sock_open() answers instantly and cleanly with
   "not associated", so there is no desync for poison()/link_reset() to fix, but nothing else
   in this FSM ever re-polls link_state() once IDLE is first reached. Left unhandled, the board
   would retry-then-abandon every report forever without ever rejoining -- exactly the 48-hour
   "survives a WiFi drop" bar this file exists to meet. NET_SOCK_CLOSE is the one place that
   acts on it, via the SAME link_down() a JOIN_WAIT deadline expiry uses; a RECV-side failure
   (the server present but silent) never sets this and is not treated as a link problem. */
static bool     g_connect_starved;

static char     g_body[PB_BODY_CAP];
static uint16_t g_body_len;                 /* != 0 == a report is pending on the wire */
static char     g_tx[PB_TX_CAP];
static uint16_t g_tx_len;
static char     g_rx[PB_RX_CAP];
static uint16_t g_rx_len;

static cmd_t    g_cmd;
static bool     g_have_cmd;

net_state_t net_state(void)           { return g_state; }
uint16_t    net_last_status(void)     { return g_status; }
uint16_t    net_next_s(void)          { return g_next_s; }
uint32_t    net_reports_ok(void)      { return g_ok; }
uint32_t    net_reports_failed(void)  { return g_failed; }
bool        net_modem_ran_this_pass(void) { return g_modem_ran; }
const char *net_disabled(void)        { return g_disabled; }
void        net_disable(const char *why) { g_disabled = why; }

/* EVERY pass that issues an AT command goes through this, never through a bare
   `g_modem_ran = true;`. ui.cpp's own flag has to be raised in the SAME pass, or spec §3's and
   §5's rule that neither screen is painted after a modem pass is not implemented at all — and
   the cost of not implementing it is up to 102 s of wedged-bus LCD painting stacked on top of
   a 2.4 s modem pass, inside a 5592 ms grant. net_modem_ran_this_pass() stays as the readable
   fact for `status` and for tests; ui_modem_ran() is the consumer that matters. */
static void modem_ran_(void) { g_modem_ran = true; ui_modem_ran(); }

bool net_take_command(cmd_t *out) {
  if (!g_have_cmd) return false;
  *out = g_cmd;
  g_have_cmd = false;                       /* surfaced ONCE per round trip */
  return true;
}

void net_begin(void) {
  link_begin(PB_NET_STEP_MS);
  g_state = NET_DOWN; g_status = 0; g_next_s = 60; g_ok = 0; g_failed = 0;
  g_backoff_i = 0; g_deadline = 0;
  g_wait_until = hal_millis();   /* NOT 0: the wait is a subtraction, so a zero sentinel
                                    would read as "not yet" for a clock past 2^31. */
  g_last_report_ms = hal_millis(); g_first_report_due = true;
  g_body_len = 0; g_tx_len = 0; g_rx_len = 0; g_have_cmd = false;
  g_retried = false; g_connect_starved = false;
  g_disabled = NULL;    /* a latch left standing here would make net_poll() a silent no-op */
}

/* PB_RETRY_DEADLINE_MS = 30000, well inside butler's RETRY_WINDOW_S = 300 (butler.py:86).
   g_t_ms is the UNSALTED hal_millis() stamped alongside g_t_wire (§4.1). Measuring against the
   wire value gives `elapsed - salt` mod 2^32. Two variables, one purpose each (§4.4). */
static bool retry_window_open(void) {
  return (int32_t)(hal_millis() - report_t_ms()) < (int32_t)PB_RETRY_DEADLINE_MS;
}

/* Every error exit routes THROUGH NET_SOCK_CLOSE via this one function, and it does NOT call
   sock_close() itself: a failed CONNECT that closed inline would be _BEGINCLIENT +
   _CLIENTCONNECT + _CLIENTCLOSE = 3 ATs = 3600 ms, and 3600 + PB_NET_SLACK_MS = 5600 > 5592.
   Load-bearing arithmetic, not tidiness (spec §3 change 2).

   The retry-eligible set is exactly two cases and nothing else (spec §4.4):
   (a) zero response bytes arrived, and (b) a COMPLETE 503, which is raised only on
   sqlite3.OperationalError (butler.py:1638-1639) and rolls the whole BEGIN IMMEDIATE back.
   Everything else is discarded: a 4xx (the backend answered; the same body cannot get better),
   a truncated reply, a parse failure, a 500, any other non-200.

   If ANY response bytes arrived, do not retry. When the request lands and the RESPONSE is lost,
   the backend has already moved a command queued -> sent (:870-873); the retry then hits the
   unconditional expire (:837-841) and kills a command the board never saw — a HIGH "never
   acknowledged" page for a dose that never existed, and the pot charged the full ml because
   flow_ml is NULL. Retrying is what destroys it, which is why a TRUNCATION is on the
   never-retry side: a truncation is bytes that arrived. */
static void finish(uint16_t status, bool ok, bool retry_eligible) {
  if (status) g_status = status;
  if (ok) ++g_ok; else ++g_failed;
  const bool retry = retry_eligible && !g_retried && g_body_len != 0 && retry_window_open();
  if (retry) g_retried = true;      /* g_body_len KEPT: SOCK_CLOSE routes back to CONNECT */
  else       g_body_len = 0;        /* past the deadline the report is ABANDONED, never sent */
  g_state = NET_SOCK_CLOSE;
}

static void link_down(void) {
  g_wait_until = hal_millis() + k_backoff[g_backoff_i];
  if (g_backoff_i + 1 < sizeof k_backoff / sizeof k_backoff[0]) ++g_backoff_i;
  g_state = NET_DOWN;
}

static bool assemble(void) {
  int w = snprintf(g_tx, sizeof g_tx,
                   "POST /report HTTP/1.1\r\nHost: %s\r\nX-Token: %s\r\n"
                   "Content-Type: text/plain\r\nContent-Length: %lu\r\n"
                   "Connection: close\r\n\r\n",
                   HOST_NAME, BUTLER_TOKEN, (unsigned long)g_body_len);
  if (w < 0 || (size_t)w + g_body_len >= sizeof g_tx) return false;
  memcpy(g_tx + w, g_body, g_body_len);
  g_tx_len = (uint16_t)((uint16_t)w + g_body_len);
  return true;
}

static uint16_t rx_status(void) {
  if (g_rx_len < 12 || memcmp(g_rx, "HTTP/1.", 7) != 0) return 0;
  for (int i = 9; i < 12; ++i) if (g_rx[i] < '0' || g_rx[i] > '9') return 0;
  return (uint16_t)((g_rx[9] - '0') * 100 + (g_rx[10] - '0') * 10 + (g_rx[11] - '0'));
}

/* strncasecmp() is POSIX (<strings.h>), not standard C; the Renesas newlib carries it in
   <string.h> instead. Hand-rolled here rather than including both headers behind a guess at
   which one the device toolchain provides — five lines, no portability question left open. */
static int ci_starts_with_(const char *s, const char *left, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    char a = s[i], b = left[i];
    if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');
    if (b >= 'A' && b <= 'Z') b = (char)(b - 'A' + 'a');
    if (a != b) return 0;
  }
  return 1;
}

static bool rx_complete(const char **body, uint16_t *blen) {
  const char *end = NULL;
  for (uint16_t i = 0; i + 4 <= g_rx_len; ++i)
    if (memcmp(g_rx + i, "\r\n\r\n", 4) == 0) { end = g_rx + i + 4; break; }
  if (!end) return false;
  uint32_t cl = 0;
  for (uint16_t i = 0; i + 15 <= (uint16_t)(end - g_rx); ++i) {
    if (!ci_starts_with_(g_rx + i, "content-length:", 15)) continue;
    uint16_t j = (uint16_t)(i + 15);
    while (j < g_rx_len && g_rx[j] == ' ') ++j;
    while (j < g_rx_len && g_rx[j] >= '0' && g_rx[j] <= '9') cl = cl * 10u + (uint32_t)(g_rx[j++] - '0');
    break;
  }
  const uint16_t have = (uint16_t)(g_rx_len - (uint16_t)(end - g_rx));
  if (have < cl) return false;
  *body = end;
  *blen = (uint16_t)cl;
  return true;
}

/* buf_read breaks out on Timeout (Modem.cpp:185-187) and leaves the late answer sitting in
   Serial2's RX FIFO; write() clears its result string (:100) but does not drain the UART, and
   the FSM's restart logic (:241,:267) resyncs some shapes and not others. So ANY modem timeout
   is treated as link poisoned: do not issue the next command. link_reset() is end();
   beginned = false; begin(); ++desyncs — and the middle line is the one the 48-hour run depends
   on. Never the ping helper: it resets modem.timeout() to 10000 ms (WiFi.cpp:585-593). */
static void poison(void) {
  link_reset();
  link_down();     /* same backoff arithmetic a JOIN_WAIT deadline expiry uses; one spelling */
}

/* A timeout always costs a full step and always yields the failure value; a SUCCESSFUL two-AT
   pass never does. So the test lives inside the failure branch, never around the whole pass. */
static bool was_timeout(uint32_t t0) {
  return (int32_t)(hal_millis() - t0) >= (int32_t)PB_NET_STEP_MS;
}

void net_poll(bool dosing) {
  g_modem_ran = false;
  if (g_disabled) return;              /* the boot assertion's consumer (§3) */
  if (dosing || cart_busy()) return;   /* §3's runtime guard. The flag arrives as a PARAMETER
                                          because §9 keeps the safety header out of this
                                          file. */

  switch (g_state) {
    case NET_DOWN:
      if ((int32_t)(hal_millis() - g_wait_until) < 0) return;
      g_state = NET_JOIN_ISSUE;
      return;

    case NET_JOIN_ISSUE: {
      modem_ran_();
      const uint32_t t0 = hal_millis();
      link_join();                              /* 2 ATs (WiFi.cpp:43-67) */
      if (was_timeout(t0)) { poison(); return; }
      g_deadline = hal_millis() + PB_NET_DEADLINE_MS;
      g_state = NET_JOIN_WAIT;
      return;
    }

    case NET_JOIN_WAIT: {
      modem_ran_();
      const uint32_t t0 = hal_millis();
      link_state_t s = link_state();            /* 1 AT */
      if (s == LINK_UP) { g_backoff_i = 0; g_state = NET_IDLE; return; }
      if (was_timeout(t0)) { poison(); return; }
      if ((int32_t)(hal_millis() - g_deadline) >= 0) link_down();
      return;
    }

    case NET_IDLE: {
      const uint32_t due = (uint32_t)g_next_s * 1000u;
      if (!g_first_report_due && hal_millis() - g_last_report_ms < due) return;
      if (!report_may_build()) return;   /* §4.3: the report WAITS while the ack reads recv */
      /* §12 item 0's per-report break check. report_heap_ok() latches err=heap; disabling the
         network is this file's half, because the flag lives here. A board that stops reporting
         is the right answer once the break is inside the stack margin: the network stack is
         the largest allocator in the program, and continuing is how the corruption reaches a
         water command. */
      if (!report_heap_ok()) { net_disable("heap"); return; }
      /* This pass issues no AT command at all, which is what makes the sweep legal here:
         §3 skips sensors_sweep() in any pass where a modem command ran. It is the sweep's
         ONLY caller in the whole program and its cadence is one per report cycle; loop()
         does not call it and there is no sensors_poll() anywhere. */
      (void)sensors_sweep();
      report_stamp();
      g_body_len = report_build(g_body, sizeof g_body);
      g_last_report_ms = hal_millis();
      g_first_report_due = false;
      if (g_body_len == 0) { ++g_failed; return; }   /* err=txcap: DROPPED, never sent (§4.2) */
      g_retried = false;              /* each report gets its own single retry */
      g_connect_starved = false;
      g_state = NET_SOCK_CLOSE;
      return;
    }

    case NET_SOCK_CLOSE: {
      modem_ran_();
      memset(g_rx, 0, sizeof g_rx);
      g_rx_len = 0;                     /* no byte of an earlier round trip survives into this one */
      const uint32_t t0 = hal_millis();
      sock_close();                     /* 1 AT, or 0 when _sock == -1 */
      if (was_timeout(t0)) { poison(); return; }
      /* An armed retry (g_retried already true, body kept) is re-checked against the window
         HERE, not only once back in finish(): the retry can sit ARMED across a backoff/rejoin
         before this pass ever runs again, and a window that was open at failure time can have
         since closed. Abandoning it here, rather than letting SOCK_CLOSE blindly walk it into
         CONNECT, is what makes "abandoned rather than sent outside the dedup window" true for
         every path back to this state, not just the one finish() takes on the failure itself. */
      if (g_retried && g_body_len != 0 && !retry_window_open()) g_body_len = 0;
      if (g_body_len) { g_state = NET_CONNECT; return; }
      /* Two straight non-timeout CONNECT failures (see g_connect_starved's own comment):
         re-join instead of parking in IDLE forever with no way back to JOIN_ISSUE. Plain
         link_down() -- the SAME backoff a JOIN_WAIT deadline expiry uses -- not poison(): the
         modem answered cleanly both times, so there is no UART desync to fix with a reset. */
      if (g_connect_starved) { g_connect_starved = false; link_down(); return; }
      g_state = NET_IDLE;
      return;
    }

    case NET_CONNECT: {
      modem_ran_();
      const uint32_t t0 = hal_millis();
      if (!sock_open()) {               /* 2 ATs; a failed open leaves _sock >= 0 */
        if (was_timeout(t0)) { poison(); return; }
        if (g_retried) g_connect_starved = true;   /* this is the retry -- and it ALSO failed */
        finish(0, false, true);
        return;
      }
      g_state = NET_SEND;
      return;
    }

    case NET_SEND: {
      modem_ran_();
      if (!assemble()) { finish(0, false, false); return; }
      const uint32_t t0 = hal_millis();
      if (sock_write((const uint8_t *)g_tx, g_tx_len) != (int)g_tx_len) {
        if (was_timeout(t0)) { poison(); return; }
        finish(0, false, true);
        return;
      }
      g_deadline = hal_millis() + PB_NET_DEADLINE_MS;
      g_state = NET_RECV;
      return;
    }

    case NET_RECV: {
      modem_ran_();
      /* client.read(buf, cap) and NOTHING else: available() would add an _AVAILABLE, and
         connected() costs TWO more. The PB_NET_DEADLINE_MS deadline is the closed-socket
         detector instead, for zero AT commands (spec §3 change 3). */
      const uint32_t t0 = hal_millis();
      int r = sock_read((uint8_t *)g_rx + g_rx_len, (size_t)(sizeof g_rx - g_rx_len));
      if (r > 0) g_rx_len = (uint16_t)(g_rx_len + r);
      const char *body; uint16_t blen;
      if (rx_complete(&body, &blen)) { g_state = NET_CLOSE; return; }
      if (r < 0 && was_timeout(t0)) { poison(); return; }
      if (r < 0 || (int32_t)(hal_millis() - g_deadline) >= 0) {
        finish(0, false, g_rx_len == 0);   /* retry-eligible only if NOTHING at all arrived */
        return;
      }
      return;
    }

    case NET_CLOSE: {                   /* 0 ATs: interpretation only */
      const uint16_t st = rx_status();
      const char *body; uint16_t blen;
      if (st == 200 && rx_complete(&body, &blen)) {
        /* The previous report's ack was delivered. Clear it BEFORE the next command can set
           the receipt placeholder, or every report repeats the same ack forever (§4.3). */
        report_clear_ack();
        response_t rs;
        const bool got = response_parse(body, blen, &rs);
        if (rs.next_s) g_next_s = rs.next_s;
        if (got) {
          g_cmd = rs.cmd; g_have_cmd = true;
          report_set_ack(rs.cmd.id, 0, "recv");   /* the ack exists from RECEIPT, not from a dose */
        }
        finish(200, true, false);
      } else {
        /* Only a 200 body reaches response_parse: butler's 400 body echoes the board's own
           tokens (f"{key}= out of range: {value}"), so a 4xx body could otherwise be parsed
           for cmd=/ml= (§4.2). A complete 503 is the one non-200 that is retry-eligible —
           sqlite3.OperationalError rolls the whole BEGIN IMMEDIATE back (spec §4.4). */
        finish(st, false, st == 503);
      }
      return;
    }
  }
}
