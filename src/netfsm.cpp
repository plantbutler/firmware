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
  g_disabled = NULL;    /* a latch left standing here would make net_poll() a silent no-op */
}

/* Every error exit routes THROUGH NET_SOCK_CLOSE via this one function, and it does NOT call
   sock_close() itself: a failed CONNECT that closed inline would be _BEGINCLIENT +
   _CLIENTCONNECT + _CLIENTCLOSE = 3 ATs = 3600 ms, and 3600 + PB_NET_SLACK_MS = 5600 > 5592.
   Load-bearing arithmetic, not tidiness (spec §3 change 2). */
static void finish(uint16_t status, bool ok) {
  if (status) g_status = status;
  if (ok) ++g_ok; else ++g_failed;
  g_body_len = 0;                 /* task 25 keeps it when a retry is armed */
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

    case NET_JOIN_ISSUE:
      modem_ran_();
      link_join();                              /* 2 ATs (WiFi.cpp:43-67) */
      g_deadline = hal_millis() + PB_NET_DEADLINE_MS;
      g_state = NET_JOIN_WAIT;
      return;

    case NET_JOIN_WAIT: {
      modem_ran_();
      link_state_t s = link_state();            /* 1 AT */
      if (s == LINK_UP) { g_backoff_i = 0; g_state = NET_IDLE; return; }
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
      g_state = NET_SOCK_CLOSE;
      return;
    }

    case NET_SOCK_CLOSE:
      modem_ran_();
      memset(g_rx, 0, sizeof g_rx);
      g_rx_len = 0;                     /* no byte of an earlier round trip survives into this one */
      sock_close();                     /* 1 AT, or 0 when _sock == -1 */
      g_state = g_body_len ? NET_CONNECT : NET_IDLE;
      return;

    case NET_CONNECT:
      modem_ran_();
      if (!sock_open()) { finish(0, false); return; }   /* 2 ATs; a failed open leaves _sock >= 0 */
      g_state = NET_SEND;
      return;

    case NET_SEND:
      modem_ran_();
      if (!assemble()) { finish(0, false); return; }
      if (sock_write((const uint8_t *)g_tx, g_tx_len) != (int)g_tx_len) { finish(0, false); return; }
      g_deadline = hal_millis() + PB_NET_DEADLINE_MS;
      g_state = NET_RECV;
      return;

    case NET_RECV: {
      modem_ran_();
      /* client.read(buf, cap) and NOTHING else: available() would add an _AVAILABLE, and
         connected() costs TWO more. The PB_NET_DEADLINE_MS deadline is the closed-socket
         detector instead, for zero AT commands (spec §3 change 3). */
      int r = sock_read((uint8_t *)g_rx + g_rx_len, (size_t)(sizeof g_rx - g_rx_len));
      if (r > 0) g_rx_len = (uint16_t)(g_rx_len + r);
      const char *body; uint16_t blen;
      if (rx_complete(&body, &blen)) { g_state = NET_CLOSE; return; }
      if (r < 0 || (int32_t)(hal_millis() - g_deadline) >= 0) { finish(0, false); return; }
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
        finish(200, true);
      } else {
        /* Only a 200 body reaches response_parse: butler's 400 body echoes the board's own
           tokens (f"{key}= out of range: {value}"), so a 4xx body could otherwise be parsed
           for cmd=/ml= (§4.2). */
        finish(st, false);
      }
      return;
    }
  }
}
