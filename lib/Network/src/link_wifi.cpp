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
static char g_ip[16];

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
}
uint16_t link_desyncs(void) { return g_desyncs; }

void link_join(void) { (void)WiFi.begin(WIFI_SSID, WIFI_PASS); }   /* 2 ATs; does NOT spin */

link_state_t link_state(void) {
  int s = WiFi.status();                                            /* ONE bounded query */
  if (s == WL_CONNECTED) return LINK_UP;
  if (s == WL_IDLE_STATUS || s == WL_SCAN_COMPLETED) return LINK_JOINING;
  return LINK_DOWN;
}
int8_t link_rssi(void) { return (int8_t)WiFi.RSSI(); }

const char *link_ip(void) {
  IPAddress a = WiFi.localIP();
  snprintf(g_ip, sizeof g_ip, "%u.%u.%u.%u",
           (unsigned)a[0], (unsigned)a[1], (unsigned)a[2], (unsigned)a[3]);
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
