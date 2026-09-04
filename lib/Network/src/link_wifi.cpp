/* link_wifi.cpp — the WiFiS3 driver behind seam 2, and nothing else. The ONLY file that names
   WiFiS3. DEVICE ONLY: lib/Network is in [env:native]'s lib_ignore and this file has no host
   coverage at all (spec §9). Zero hits for the safety header, the dosing entry point or the
   pump write are a make check invariant over this directory. */
#include <WiFiS3.h>
#include "link.h"
#include "config.h"
#include "secrets.h"

/* Exactly ONE WiFiClient for the life of the program: its ctor does
   `new FifoBuffer<uint8_t,1024>` on every construction (WiFiClient.cpp:6-8), so a file static
   heap-allocates that 1 KB once instead of once per report. */
static WiFiClient g_client;
static uint16_t g_desyncs;

/* WiFi.localIP() (WiFi.cpp:328-357) is a do/while loop, up to 50 times: a 100 ms sleep then up
   to 2 modem.write() calls, while the IP still reads 0.0.0.0 -- NOT the bounded single query
   the rest of this file's primitives are. When there is genuinely no IP (not joined, or just
   dropped), that is PRECISELY the condition that runs all 50 iterations: up to 100 + 2*1200 =
   2500 ms per spin, ~125000 ms worst case, against a 5592 ms watchdog grant -- a guaranteed
   reset from one call.
   g_last_state is link_state()'s own last answer, kept here with zero extra ATs (link_state()
   already computes it; this just remembers it) so link_ip() can gate on "is the link UP right
   now" without paying an AT to ask. g_ip_valid makes the one WiFi.localIP() call per join
   happen AT MOST ONCE: after that, link_ip() is a pure accessor and cannot block at all, which
   is the only shape safe to call from cli_print_status() -- the console command that runs
   synchronously inside loop() with nothing else feeding the dog while it does. */
static link_state_t g_last_state = LINK_DOWN;
static bool         g_ip_valid;
static char         g_ip[16] = "0.0.0.0";   /* what link_ip() returns before the first
                                                successful population, or after any
                                                invalidation -- honest, not stale, not empty */

static void ip_invalidate_(void) {
  g_ip_valid = false;
  snprintf(g_ip, sizeof g_ip, "0.0.0.0");
}

void link_begin(uint32_t step_ms) {
  modem.timeout((int)step_ms);          /* MODEM_TIMEOUT defaults to 10000 (Modem.h:12) */
  modem.begin();                        /* EXPLICITLY, ONCE: drains the one-time _SOFTRESETWIFI
                                           into setup(), which is what makes link_join() 2 ATs */
  WiFi.setTimeout(0);                   /* CWifi::begin()'s poll loop body never runs
                                           (WiFi.cpp:61,563) */
  g_client.setConnectionTimeout((int)PB_NET_STEP_MS);
                                        /* selects _CLIENTCONNECT and appends the value
                                           (WiFiClient.cpp:57-61); _connectionTimeout defaults
                                           to 0, so without this nothing bounds the ESP32's own
                                           connect inside our 1200 ms window */
}

void link_reset(void) {
  modem.end();
  modem.beginned = false;   /* Modem.cpp:45-48 never does this, and begin() is guarded by it
                               (:35). Without this line link_reset() closes Serial2 and declines
                               to reopen it: every subsequent AT command writes into a closed
                               UART, buf_read times out at PB_NET_STEP_MS forever, ch206 climbs,
                               and the board silently stops reporting for the rest of the run. */
  modem.begin();            /* re-issues _SOFTRESETWIFI, which is the point of the reset */
  g_desyncs++;
  ip_invalidate_();          /* we just tore the modem session down: not up, and the cached
                                 address (if any) belongs to a join that no longer exists */
  g_last_state = LINK_DOWN;  /* explicit, not left to the next link_state() call: link_ip()
                                 must never see a stale LINK_UP between this call and that one */
}
uint16_t link_desyncs(void) { return g_desyncs; }

void link_join(void) { (void)WiFi.begin(WIFI_SSID, WIFI_PASS); }   /* 2 ATs; does NOT spin */

link_state_t link_state(void) {
  int s = WiFi.status();                                            /* ONE bounded query */
  link_state_t ls;
  if (s == WL_CONNECTED) ls = LINK_UP;
  else if (s == WL_IDLE_STATUS || s == WL_SCAN_COMPLETED) ls = LINK_JOINING;
  else ls = LINK_DOWN;
  if (g_last_state == LINK_UP && ls != LINK_UP) ip_invalidate_();   /* dropped: a cached
                                                                        address from the join
                                                                        that just ended is a
                                                                        stale one, not a wrong
                                                                        one -- don't show it */
  g_last_state = ls;
  return ls;
}
int8_t link_rssi(void) { return (int8_t)WiFi.RSSI(); }

/* Guarded and cached, not a bare pass-through -- see the comment beside g_last_state above.
   NOT UP: never touches WiFi.localIP() at all (0 ATs); the answer would be worthless anyway.
   UP, not yet cached: one WiFi.localIP() call, which on a genuinely joined link is expected to
   resolve on its first internal iteration (a 100 ms sleep plus up to 2 ATs, ~2500 ms worst case
   -- survivable inside the 5592 ms grant, but not free, which is why this runs at most once per
   join rather than once per `status`). UP, already cached: a pure accessor, cannot block. */
const char *link_ip(void) {
  if (!g_ip_valid && g_last_state == LINK_UP) {
    IPAddress a = WiFi.localIP();
    snprintf(g_ip, sizeof g_ip, "%u.%u.%u.%u",
             (unsigned)a[0], (unsigned)a[1], (unsigned)a[2], (unsigned)a[3]);
    g_ip_valid = true;
  }
  return g_ip;
}

bool sock_open(void) { return g_client.connect(HOST_NAME, (uint16_t)HTTP_PORT) == 1; }

int sock_write(const uint8_t *b, size_t n) {
  size_t w = g_client.write(b, n);      /* write_nowait(_CLIENTSEND) + one passthrough */
  return (int)w;
}

/* client.read(buf, cap) and NOTHING else — one _CLIENTRECEIVE per RECV pass
   (WiFiClient.cpp:145-182). available() would add an _AVAILABLE; connected() costs TWO because
   it calls available() itself (:224-238), and the FSM never calls it: PB_NET_DEADLINE_MS is the
   closed-socket detector instead (§3 change 3). The ping helper is never called anywhere: it
   resets the modem timeout to 10000 ms (WiFi.cpp:585-593) and would undo the whole margin.
   make check greps its name to zero over lib/, comments included, which is why it is not
   spelled here. */
int sock_read(uint8_t *b, size_t cap) { return g_client.read(b, (size_t)cap); }

void sock_close(void) { g_client.stop(); }   /* sets _sock = -1 (:217): idempotent by design */
