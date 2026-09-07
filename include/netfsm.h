/* netfsm.h: the report state machine, the HTTP framing and the two-AT budget.
   One bounded link/socket step per net_poll(), at most 2 AT commands (2400 ms against a
   5592 ms grant) — which holds only because every exit closes the socket in its own pass. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "report.h"

typedef enum { NET_DOWN, NET_JOIN_ISSUE, NET_JOIN_WAIT, NET_IDLE,
               NET_SOCK_CLOSE, NET_CONNECT, NET_SEND, NET_RECV, NET_CLOSE } net_state_t;

void        net_begin(void);
void        net_boot(const char *boot_err);  /* net_begin(), then net_disable() only if
                                                boot_err is non-NULL. The order lives HERE,
                                                not in main.cpp, because the host build
                                                filters main.cpp out: an order written there
                                                is one no host test can fail on. */
void        net_poll(bool dosing);          /* passed IN: this file may not include safety.h */
net_state_t net_state(void);
uint16_t    net_last_status(void);
uint16_t    net_next_s(void);
uint32_t    net_reports_ok(void);
uint32_t    net_reports_failed(void);
bool        net_modem_ran_this_pass(void);  /* printed by `status` as modem_ran= */
bool        net_take_command(cmd_t *out);
void        net_disable(const char *why);   /* setup()'s watchdog-grant assertion */
const char *net_disabled(void);

/* This file owns the AT budget, so nothing outside it and the driver behind the seam may
   call INTO the seam: an unconditional read of the seam from the UI fill costs up to ~5 AT
   commands per loop() pass against a 5592 ms grant, a guaranteed watchdog reset. These four
   are plain cached accessors and issue ZERO AT commands. The state is refreshed every
   NET_JOIN_WAIT pass; the signal strength and address at most once per successful join,
   each in its OWN NET_IDLE pass, and cleared back to their "no link" defaults the moment
   the FSM gives up on the link. None of the four carries the seam's own name prefix,
   because the invariant check scans comments too. */
uint8_t     net_link(void);      /* 0 down, 1 joining, 2 up -- cached, matches ui_state_t::link */
int8_t      net_rssi(void);      /* cached */
const char *net_ip(void);        /* cached, into this file's own static buffer, "0.0.0.0"
                                     before the first successful join or after a drop */
uint16_t    net_desyncs(void);   /* wraps the seam's own zero-AT counter so nothing outside
                                     this file names it */

#ifdef PB_NATIVE
/* Host-suite seam: g_retried and g_connect_starved are process-lifetime statics with no
   reset path but net_begin(); a case that longjmps out mid-body before its own net_begin()
   would read the previous case's retry/starvation state. pb_test_teardown() is the only
   caller. */
void netfsm_test_reset_retry_(void);

/* Host-suite seam: the fake clock advances 1 ms on every hal_millis() read, so net_poll()
   can never present was_timeout() with an elapsed time of EXACTLY PB_NET_STEP_MS -- every
   timeout it manufactures reads one past the boundary. Calling the same comparison with a
   controlled t0 is the only way to tell >= from >. */
bool netfsm_test_was_timeout_(uint32_t t0);
#endif
