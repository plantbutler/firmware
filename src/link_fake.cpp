/* link_fake.cpp: seam 2 against a script, for the native and sim envs; the bench env's
   build_src_filter excludes it. */
#include "link.h"
#include "sim.h"
#include "config.h"
#include <string.h>
#include <stdio.h>

#define FAKE_RESP_CAP 512

static link_state_t g_state;
static bool     g_beginned;      /* models ModemClass::beginned */
static bool     g_serial_open;   /* Serial2: opened by begin(), closed by end() */
static bool     g_join_pending;
static int      g_sock;          /* -1 == unallocated, as WiFiClient::_sock */
static bool     g_fail_open;
static bool     g_timeout_next;
static uint16_t g_at;
static uint16_t g_desyncs;
static uint16_t g_resets;
static uint8_t  g_resp[FAKE_RESP_CAP];
static uint16_t g_resp_len, g_resp_pos;
static uint8_t  g_sent[PB_TX_CAP];
static uint16_t g_sent_len;
static uint16_t g_writes;        /* sock_write() calls: "was anything sent at all?" */

/* One AT round trip. false == the modem timed out, and a timeout costs the full step: that
   elapsed time is the only signal netfsm has, because link.h has no timeout primitive. */
static bool at_(void) {
  ++g_at;
  if (!g_serial_open || g_timeout_next) {
    g_timeout_next = false;
    sim_advance(PB_NET_STEP_MS);
    return false;
  }
  sim_advance(1);
  return true;
}

void link_begin(uint32_t step_ms) {
  (void)step_ms;
  if (!g_beginned) { g_beginned = true; g_serial_open = true; }   /* modem.begin() */
}

void link_join(void) { if (at_() && at_()) g_join_pending = true; }   /* 2 ATs */

link_state_t link_state(void) {
  if (!at_()) return g_state;
  if (g_join_pending) { g_join_pending = false; g_state = LINK_UP; }
  return g_state;
}

/* One AT each, like the driver: 1 for the signal and 1 for the address, at most once per
   join. Charging at_() is what pins each refresh pass's timeout/poison pairing; a free call
   would let either poison() be deleted with every net case green. Each returns its own
   distinctive fixed value so "which pass did this arrive in" is answerable. */
int8_t      link_rssi(void) { return at_() ? -52 : 0; }
const char *link_ip(void) {
  static char ip[16];
  snprintf(ip, sizeof ip, "%s", at_() ? "192.168.1.42" : "0.0.0.0");
  return ip;
}
uint16_t    link_desyncs(void) { return g_desyncs; }

bool sock_open(void) {
  if (g_sock >= 0) return false;      /* PRECONDITION: sock_close() ran */
  if (g_state != LINK_UP) return false;
  if (!at_()) return false;           /* _BEGINCLIENT */
  g_sock = 1;                         /* getSocket() ALLOCATES before the connect runs */
  if (!at_()) return false;           /* _CLIENTCONNECT */
  return !g_fail_open;                /* a failed connect leaves _sock >= 0 */
}

int sock_write(const uint8_t *b, size_t n) {
  if (g_sock < 0) return -1;
  if (!at_()) return -1;
  ++g_writes;                   /* the COUNT, not just the last buffer: "nothing was
                                   sent across two report intervals" needs it */
  if (n > sizeof g_sent) n = sizeof g_sent;
  memcpy(g_sent, b, n);
  g_sent_len = (uint16_t)n;
  return (int)n;
}

int sock_read(uint8_t *b, size_t cap) {
  if (g_sock < 0) return -1;
  if (!at_()) return -1;
  uint16_t left = (uint16_t)(g_resp_len - g_resp_pos);
  if (left == 0) return 0;
  if (cap < left) left = (uint16_t)cap;
  memcpy(b, g_resp + g_resp_pos, left);
  g_resp_pos = (uint16_t)(g_resp_pos + left);
  return (int)left;
}

void sock_close(void) { if (g_sock >= 0) { (void)at_(); g_sock = -1; } }

void link_reset(void) {
  g_serial_open = false;        /* modem.end() */
  g_beginned = false;           /* the core's end() never clears this; without it begin() no-ops */
  if (!g_beginned) { g_beginned = true; g_serial_open = true; }   /* modem.begin() */
  g_state = LINK_DOWN; g_join_pending = false; g_sock = -1;
  ++g_desyncs; ++g_resets;
}

void link_fake_reset(void) {
  g_state = LINK_DOWN; g_beginned = false; g_serial_open = false; g_join_pending = false;
  g_sock = -1; g_fail_open = false; g_timeout_next = false;
  g_at = 0; g_desyncs = 0; g_resets = 0;
  g_resp_len = 0; g_resp_pos = 0; g_sent_len = 0; g_writes = 0;
}
void link_fake_set_state(link_state_t s) { g_state = s; if (s == LINK_UP) g_join_pending = false; }
void link_fake_queue_response(const char *raw, size_t n) {
  if (n > sizeof g_resp) n = sizeof g_resp;
  memcpy(g_resp, raw, n); g_resp_len = (uint16_t)n; g_resp_pos = 0;
}
void link_fake_fail_open(bool fail) { g_fail_open = fail; }
void link_fake_timeout_next(void)   { g_timeout_next = true; }
void link_fake_drop_link(void)      { g_state = LINK_DOWN; g_join_pending = false; g_sock = -1; }
void link_fake_pass_begin(void)     { g_at = 0; }
uint16_t link_fake_at_count(void)   { return g_at; }
uint16_t link_fake_reset_count(void){ return g_resets; }
/* link.h has no available()/connected() primitive, so these are hard-coded false and the
   case asserting them is a tautology that documents the seam's shape; the real check is the
   AT budget beside it. The driver's half only a wall-clock case on real silicon can see. */
bool link_fake_saw_available(void)  { return false; }
bool link_fake_saw_connected(void)  { return false; }
const uint8_t *link_fake_sent(uint16_t *len) { if (len) *len = g_sent_len; return g_sent; }
uint16_t link_fake_write_count(void) { return g_writes; }
