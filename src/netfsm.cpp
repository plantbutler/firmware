/* netfsm.cpp: the report/response state machine and HTTP framing over the network seam.
   Names neither the safety header, the dosing entry point nor the pump write: a state
   whose socket is open must never be one edit away from a call that asserts D6. No signed
   conversion in any format string. */
#include "netfsm.h"
#include "link.h"
#include "report.h"
#include "sensors.h"
#include "cart.h"
#include "config.h"
#include "hal.h"
#include "ui.h"
#include "secrets.h"
#include <stdio.h>
#include <string.h>

/* The dosing flag arrives as a net_poll() parameter: an extern declaration here would be
   the safety header by another name. */

static_assert(sizeof(HOST_NAME) + sizeof(BUTLER_TOKEN) + PB_HDR_FIXED + PB_BODY_CAP <= PB_TX_CAP,
              "HOST_NAME + BUTLER_TOKEN + headers + body do not fit PB_TX_CAP: snprintf would "
              "truncate silently while Content-Length still claimed the full length (spec §4.2)");
static_assert(PB_CONTROLLER_WIRE + 2 + PB_BODY_WORST_FIXED <= PB_BODY_CAP, "body cap (§7)");
static_assert(PB_CONTROLLER >= 0 && PB_CONTROLLER <= 255, "c= is 0..255 (butler.MAX_CONTROLLER)");

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
/* Set only in NET_CONNECT's failure branch once the retry is exhausted: two consecutive
   opens that fail cleanly, neither a modem timeout, is a link that silently dropped between
   reports. There is no desync to reset, and nothing else re-polls the link once IDLE is
   reached, so without this the board would retry-then-abandon every report forever without
   rejoining. NET_SOCK_CLOSE acts on it through the same link_down() a JOIN_WAIT expiry uses;
   a RECV-side failure (server present but silent) never sets it. */
static bool     g_connect_starved;

static char     g_body[PB_BODY_CAP];
static uint16_t g_body_len;                 /* != 0 == a report is pending on the wire */
static char     g_tx[PB_TX_CAP];
static uint16_t g_tx_len;
static char     g_rx[PB_RX_CAP];
static uint16_t g_rx_len;

static cmd_t    g_cmd;
static bool     g_have_cmd;

/* This file's own cache of what the seam last reported, so every other caller reads a
   zero-AT accessor. g_link is refreshed every NET_JOIN_WAIT pass; g_rssi/g_ip at most once
   per join, one whole NET_IDLE pass each; all three reset when link_down() gives up. */
static uint8_t  g_link;
static int8_t   g_rssi;
static char     g_ip[16] = "0.0.0.0";
static bool     g_need_rssi, g_need_ip;      /* set on the JOIN_WAIT -> IDLE transition */
static bool     g_need_reset;                /* a poisoned session, torn down in its own pass */

net_state_t net_state(void)           { return g_state; }
uint16_t    net_last_status(void)     { return g_status; }
uint16_t    net_next_s(void)          { return g_next_s; }
uint32_t    net_reports_ok(void)      { return g_ok; }
uint32_t    net_reports_failed(void)  { return g_failed; }
bool        net_modem_ran_this_pass(void) { return g_modem_ran; }
const char *net_disabled(void)        { return g_disabled; }
void        net_disable(const char *why) { g_disabled = why; }
uint8_t     net_link(void)            { return g_link; }
int8_t      net_rssi(void)            { return g_rssi; }
const char *net_ip(void)              { return g_ip; }
uint16_t    net_desyncs(void)         { return link_desyncs(); }

/* Every pass that issues an AT command goes through this: ui.cpp's flag must be raised in
   the same pass, or a screen is painted after a modem pass -- up to 102 s of wedged-bus LCD
   painting on top of a 2.4 s modem pass, inside a 5592 ms watchdog grant. */
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
  g_link = 0; g_rssi = 0; snprintf(g_ip, sizeof g_ip, "0.0.0.0");
  g_need_rssi = false; g_need_ip = false;
  g_need_reset = false;
}

/* net_begin() must run first: it clears g_disabled unconditionally so a latch cannot
   survive a restart, and a verdict latched before it would be thrown away one statement
   later -- the banner would still print net=DISABLED from main.cpp's own flag while the
   board reported rescaled raw counts for 48 hours. */
void net_boot(const char *boot_err) {
  net_begin();
  if (boot_err) net_disable(boot_err);
}

/* PB_RETRY_DEADLINE_MS is well inside butler's 300 s dedup window. Measured against the
   unsalted hal_millis() stamp, not the wire value, which carries the boot salt. */
static bool retry_window_open(void) {
  return (int32_t)(hal_millis() - report_t_ms()) < (int32_t)PB_RETRY_DEADLINE_MS;
}

/* Every error exit routes through NET_SOCK_CLOSE, and this never closes inline: a failed
   CONNECT closing here would be 3 ATs = 3600 ms, and 3600 + PB_NET_SLACK_MS exceeds the
   5592 ms grant.

   Retry-eligible is exactly two cases: zero response bytes arrived, or a complete 503
   (raised only on sqlite3.OperationalError, which rolls the whole transaction back).
   Everything else is discarded -- a 4xx, a truncated reply, a parse failure, a 500. If any
   bytes arrived the request landed: the backend has already moved the command queued ->
   sent, and a retry would hit its unconditional expire and kill a command the board never
   saw, paging and charging the pot the full ml. A truncation is bytes that arrived. */
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
  /* `status` must never show an RSSI or IP from a link that has ended; the next join
     re-arms both on its own JOIN_WAIT -> IDLE transition. */
  g_link = 0; g_rssi = 0; snprintf(g_ip, sizeof g_ip, "0.0.0.0");
  g_need_rssi = false; g_need_ip = false;
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

/* strncasecmp() is POSIX, and the Renesas newlib keeps it in a different header than the
   host does; hand-rolled rather than guessing. */
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

/* A modem timeout leaves the late answer in Serial2's RX FIFO: the next write clears its
   result but not the UART, and the modem's restart logic resyncs some shapes and not
   others. So ANY timeout poisons the link: issue no further command, and reset the modem
   (end, clear its begun flag, begin again -- the middle step is the one a long run depends
   on). Never the ping helper: it resets the modem timeout to 10000 ms.
   The tear-down is deferred to its own pass: poison() is reached from passes that already
   spent two round trips, and the reset costs a third (3 x PB_NET_STEP_MS = 3600 ms, over
   the grant once slack is added). The fake charges the reset nothing, so the host suite
   cannot see this. The link is down either way, so nothing talks to the modem meanwhile. */
static void poison(void) {
  g_need_reset = true;
  link_down();     /* the same backoff a JOIN_WAIT deadline expiry uses */
}

/* A timeout always costs a full step and always yields the failure value; a successful
   two-AT pass never does, so the test lives inside the failure branch. */
static bool was_timeout(uint32_t t0) {
  return (int32_t)(hal_millis() - t0) >= (int32_t)PB_NET_STEP_MS;
}

#ifdef PB_NATIVE
void netfsm_test_reset_retry_(void) { g_retried = false; g_connect_starved = false; }
bool netfsm_test_was_timeout_(uint32_t t0) { return was_timeout(t0); }
#endif

void net_poll(bool dosing) {
  g_modem_ran = false;
  if (g_disabled) return;              /* the boot assertion's consumer */
  if (dosing || cart_busy()) return;   /* a parameter, so the safety header stays out
                                          of this file */

  switch (g_state) {
    case NET_DOWN:
      if ((int32_t)(hal_millis() - g_wait_until) < 0) return;
      if (g_need_reset) {        /* its own pass, for the reason above poison() */
        modem_ran_();
        link_reset();
        g_need_reset = false;
        return;
      }
      g_state = NET_JOIN_ISSUE;
      return;

    case NET_JOIN_ISSUE: {
      modem_ran_();
      const uint32_t t0 = hal_millis();
      link_join();                              /* 2 ATs */
      if (was_timeout(t0)) { poison(); return; }
      g_deadline = hal_millis() + PB_NET_DEADLINE_MS;
      g_state = NET_JOIN_WAIT;
      return;
    }

    case NET_JOIN_WAIT: {
      modem_ran_();
      const uint32_t t0 = hal_millis();
      link_state_t s = link_state();            /* 1 AT */
      /* The one place g_link is written; JOIN_WAIT already has the answer, at zero extra
         ATs. 0 down, 1 joining, 2 up, as ui and `status` read it. */
      g_link = (uint8_t)(s == LINK_UP ? 2 : (s == LINK_JOINING ? 1 : 0));
      if (s == LINK_UP) {
        g_backoff_i = 0;
        g_need_rssi = true; g_need_ip = true;   /* NET_IDLE spends one whole pass on each */
        g_state = NET_IDLE;
        return;
      }
      if (was_timeout(t0)) { poison(); return; }
      if ((int32_t)(hal_millis() - g_deadline) >= 0) link_down();
      return;
    }

    case NET_IDLE: {
      /* The refresh passes come first, one AT each and one whole pass each. At 1 + 1 they
         would fit in one pass and stay apart anyway: a 1-AT pass keeps ~3.2 s of margin
         under the grant, a 2-AT pass ~1.2 s. Every 2-AT pass in this file is 2 ATs
         because it must be. */
      if (g_need_rssi) {
        modem_ran_();
        const uint32_t t0 = hal_millis();
        g_rssi = link_rssi();                   /* 1 AT */
        if (was_timeout(t0)) { poison(); return; }
        g_need_rssi = false;
        return;
      }
      if (g_need_ip) {
        /* The same was_timeout()/poison() pairing as every other AT pass: the driver's
           address query is one bounded AT, not the core's localIP() with its ~2.5 s
           success and ~125 s failure, so a slow answer here means the modem timed out. */
        modem_ran_();
        const uint32_t t0 = hal_millis();
        const char *ip = link_ip();              /* 1 AT, once per join */
        if (was_timeout(t0)) { poison(); return; }
        snprintf(g_ip, sizeof g_ip, "%s", ip);
        g_need_ip = false;
        return;
      }

      const uint32_t due = (uint32_t)g_next_s * 1000u;
      if (!g_first_report_due && hal_millis() - g_last_report_ms < due) return;
      if (!report_may_build()) return;   /* the report waits while the ack still reads recv */
      /* report_heap_ok() latches err=heap; disabling the network is this file's half. A
         board that stops reporting is the right answer once the break is inside the stack
         margin: the network stack is the largest allocator, and continuing is how the
         corruption reaches a water command. */
      if (!report_heap_ok()) { net_disable("heap"); return; }
      /* This pass issues no AT command, which is what makes the sweep legal here: no pass
         may both run the modem and sweep the sensors. This is the sweep's only caller. */
      (void)sensors_sweep();
      report_stamp();
      g_body_len = report_build(g_body, sizeof g_body);
      g_last_report_ms = hal_millis();
      g_first_report_due = false;
      if (g_body_len == 0) { ++g_failed; return; }   /* err=txcap: DROPPED, never sent */
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
      /* An armed retry can sit across a backoff and rejoin before this pass runs again, so
         the window is re-checked here: a retry outside the dedup window is abandoned,
         never sent. */
      if (g_retried && g_body_len != 0 && !retry_window_open()) g_body_len = 0;
      if (g_body_len) { g_state = NET_CONNECT; return; }
      /* Two straight clean CONNECT failures: re-join rather than park in IDLE forever.
         link_down(), not poison(): the modem answered both times, there is no desync. */
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
      /* Only the socket read: available() would add an AT and connected() two more. The
         deadline is the closed-socket detector, at zero ATs. */
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
        /* Clear the delivered ack BEFORE the next command can set the receipt placeholder,
           or every report repeats the same ack forever. */
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
           tokens, so a 4xx body could otherwise be parsed for cmd=/ml=. A complete 503 is
           the one retry-eligible non-200. */
        finish(st, false, st == 503);
      }
      return;
    }
  }
}
