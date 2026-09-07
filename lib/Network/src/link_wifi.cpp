/* link_wifi.cpp: the WiFiS3 driver behind the link seam -- the only file that names WiFiS3; device only, no host coverage; must not reach the safety header, the dosing entry point or the pump write. */
#include <WiFiS3.h>
#include <string>
#include "link.h"
#include "config.h"
#include "secrets.h"

/* Exactly one WiFiClient for the life of the program: its constructor heap-allocates a
   1 KB FIFO, so a file static pays that once instead of once per report. */
static WiFiClient g_client;
static uint16_t g_desyncs;

/* link_ip() does not call WiFi.localIP(): that is a do/while of up to 50 passes, each a
   100 ms sleep and two modem queries, spinning while the answer reads 0.0.0.0 -- which is
   exactly the state of a link that dropped between the pass that saw WL_CONNECTED and the
   pass that asks. Up to 2500 ms per spin, ~125 s worst case, against a 5592 ms watchdog
   grant: a guaranteed reset, repeated after every rejoin while the AP flaps. So link_ip()
   issues the one _IPSTA query itself, bounded by modem.timeout() like every primitive
   here, and skips the mode read because this file only ever joins as a station. No answer,
   or 0.0.0.0, leaves the cache invalid; netfsm.cpp asks once per join, so that is one pass
   without an address, not a spin.
   g_last_state remembers link_state()'s last answer at no extra AT, so link_ip() can gate
   on "up right now" without asking; g_ip_valid makes the query happen at most once per
   join, after which link_ip() is a pure accessor and cannot block. */
static link_state_t g_last_state = LINK_DOWN;
static bool         g_ip_valid;
static char         g_ip[16] = "0.0.0.0";   /* returned until the first successful query,
                                                and again after any invalidation */

static void ip_invalidate_(void) {
  g_ip_valid = false;
  snprintf(g_ip, sizeof g_ip, "0.0.0.0");
}

void link_begin(uint32_t step_ms) {
  modem.timeout((int)step_ms);          /* the library default is 10000 ms */
  modem.begin();                        /* once, here: the one-time soft reset lands in
                                           setup(), which is what makes link_join() 2 ATs */
  WiFi.setTimeout(0);                   /* CWifi::begin()'s poll loop body never runs */
  g_client.setConnectionTimeout((int)PB_NET_STEP_MS);
                                        /* defaults to 0: without this nothing bounds the
                                           ESP32's own connect inside the 1200 ms window */
}

void link_reset(void) {
  modem.end();
  modem.beginned = false;   /* end() never clears this and begin() is guarded by it. Without
                               this line the reset closes Serial2 and never reopens it: every
                               AT then times out at PB_NET_STEP_MS, ch206 climbs, and the
                               board silently stops reporting for the rest of the run. */
  modem.begin();            /* re-issues the soft reset, which is the point */
  g_desyncs++;
  ip_invalidate_();          /* the cached address belongs to a join that no longer exists */
  g_last_state = LINK_DOWN;  /* explicit: link_ip() must never see a stale LINK_UP before
                                 the next link_state() call */
}
uint16_t link_desyncs(void) { return g_desyncs; }

void link_join(void) { (void)WiFi.begin(WIFI_SSID, WIFI_PASS); }   /* 2 ATs; does not spin */

link_state_t link_state(void) {
  int s = WiFi.status();                                            /* one bounded query */
  link_state_t ls;
  if (s == WL_CONNECTED) ls = LINK_UP;
  else if (s == WL_IDLE_STATUS || s == WL_SCAN_COMPLETED) ls = LINK_JOINING;
  else ls = LINK_DOWN;
  if (g_last_state == LINK_UP && ls != LINK_UP) ip_invalidate_();   /* dropped: the cached
                                                                        address is from the
                                                                        join that just ended */
  g_last_state = ls;
  return ls;
}
int8_t link_rssi(void) { return (int8_t)WiFi.RSSI(); }

/* Not up: 0 ATs. Up and not yet cached: exactly one round trip, `AT+IPSTA=0`, which returns
   within modem.timeout() whatever the modem does -- 1 AT on netfsm.cpp's budget. Up and
   cached: a pure accessor. */
const char *link_ip(void) {
  if (!g_ip_valid && g_last_state == LINK_UP) {
    std::string res;
    if (modem.write(std::string(PROMPT(_IPSTA)), res, "%s%d\r\n", CMD_WRITE(_IPSTA), IP_ADDR)) {
      IPAddress a;
      if (a.fromString(res.c_str()) && a != IPAddress(0, 0, 0, 0)) {
        snprintf(g_ip, sizeof g_ip, "%u.%u.%u.%u",
                 (unsigned)a[0], (unsigned)a[1], (unsigned)a[2], (unsigned)a[3]);
        g_ip_valid = true;
      }
    }
  }
  return g_ip;
}

bool sock_open(void) { return g_client.connect(HOST_NAME, (uint16_t)HTTP_PORT) == 1; }

int sock_write(const uint8_t *b, size_t n) {
  size_t w = g_client.write(b, n);      /* one AT plus one passthrough */
  return (int)w;
}

/* read() and nothing else: one AT per RECV pass. available() would add one; connected()
   costs two because it calls available() itself, so the FSM never calls it and
   PB_NET_DEADLINE_MS is the closed-socket detector instead. The library's ping helper
   resets the modem timeout to 10000 ms and would undo the whole margin; it is never
   called, and a build check greps for its name. */
int sock_read(uint8_t *b, size_t cap) { return g_client.read(b, (size_t)cap); }

void sock_close(void) { g_client.stop(); }   /* idempotent: the library just sets its socket slot to -1 */
