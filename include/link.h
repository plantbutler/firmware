/* link.h: the network seam; nothing here names WiFiS3. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef enum { LINK_DOWN, LINK_JOINING, LINK_UP } link_state_t;

void         link_begin(uint32_t step_ms);  /* modem.timeout(); modem.begin(); WiFi.setTimeout(0) */
void         link_join(void);               /* 2 ATs, returns; does NOT spin */
link_state_t link_state(void);              /* ONE bounded status query */
int8_t       link_rssi(void);
const char  *link_ip(void);                 /* into a static char[16] */
void         link_reset(void);              /* end(); beginned = false; begin(); counts a desync */
uint16_t     link_desyncs(void);
bool         sock_open(void);               /* HOST_NAME:HTTP_PORT; PRECONDITION: the close ran first */
int          sock_write(const uint8_t *b, size_t n);  /* one write, one modem round trip */
int          sock_read(uint8_t *b, size_t cap);       /* -1 closed, 0 nothing yet */
void         sock_close(void);              /* idempotent; EVERY exit, failed open included */
