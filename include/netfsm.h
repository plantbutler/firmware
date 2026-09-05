/* include/netfsm.h — seam 2's state machine: the report state machine, the HTTP framing
   and the two-AT budget. Spec §3.
   ONE bounded link/socket step per net_poll(), AT MOST 2 AT commands, each with its own
   deadline. Worst pass is CONNECT = _BEGINCLIENT + _CLIENTCONNECT = 2 * PB_NET_STEP_MS =
   2400 ms against a granted 5592 ms — and that holds ONLY because sock_close ran in a PRIOR
   pass and left _sock == -1, and because every error exit routes through NET_SOCK_CLOSE
   instead of closing inline. Spec §3's per-pass AT table. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "report.h"      /* cmd_t */

typedef enum { NET_DOWN, NET_JOIN_ISSUE, NET_JOIN_WAIT, NET_IDLE,
               NET_SOCK_CLOSE, NET_CONNECT, NET_SEND, NET_RECV, NET_CLOSE } net_state_t;

void        net_begin(void);
void        net_boot(const char *boot_err);  /* net_begin() and then, only if boot_err is
                                                non-NULL, net_disable(). setup()'s two-line
                                                order lives HERE and not in main.cpp, because
                                                [env:native] filters main.cpp out: an order
                                                written there is an order no host test can
                                                fail on. See net_boot()'s own comment. */
void        net_poll(bool dosing);          /* the flag is passed IN — see netfsm.cpp */
net_state_t net_state(void);
uint16_t    net_last_status(void);
uint16_t    net_next_s(void);
uint32_t    net_reports_ok(void);
uint32_t    net_reports_failed(void);
bool        net_modem_ran_this_pass(void);  /* printed by `status` as modem_ran= */
bool        net_take_command(cmd_t *out);
void        net_disable(const char *why);   /* setup()'s watchdog-grant assertion (spec §3) */
const char *net_disabled(void);

/* This file owns the AT budget, so it owns when the seam's link information is refreshed:
   nothing outside this file and the WiFiS3 driver behind it may call INTO the seam directly
   (fix round, task 27) -- src/main.cpp's UI fill used to read the seam's state query twice,
   its signal-strength query once and its address query once, UNCONDITIONALLY, every loop()
   pass, on top of whatever this file's own poll had already spent -- against the real driver
   that is up to ~5 AT commands in one pass against a 5592 ms grant, a guaranteed watchdog
   reset in normal operation. These four are the only sanctioned way to read link information
   from outside this file: each is a plain cached accessor and issues ZERO AT commands. The
   state is refreshed every NET_JOIN_WAIT pass (this file already queries it there); the
   signal strength and address are each refreshed at most once per successful join, in their
   OWN separate NET_IDLE pass, and are cleared back to their "no link" defaults the moment the
   FSM gives up on the link. None of these four names embeds the seam's own function-name
   prefix, on purpose, so that prefix never leaks past this file's boundary -- the reason is
   the same one make check already applies to the modem's ping helper elsewhere in this tree:
   said in words, not spelled, because the invariant scans comments too. */
uint8_t     net_link(void);      /* 0 down, 1 joining, 2 up -- cached, matches ui_state_t::link */
int8_t      net_rssi(void);      /* cached */
const char *net_ip(void);        /* cached, into this file's own static buffer, "0.0.0.0"
                                     before the first successful join or after a drop */
uint16_t    net_desyncs(void);   /* the seam's own desync counter is already a zero-AT
                                     accessor; this wrapper exists only so nothing outside
                                     this file and the driver behind the seam names it either */

#ifdef PB_NATIVE
/* Host-suite seam, same shape as sensors.h's sensors_test_reset_health_() (task 18) and
   pulses.h's pulses_test_reset_leak_() (task 22): g_retried and g_connect_starved are
   process-lifetime statics in netfsm.cpp with no reset path but net_begin() and NET_IDLE's
   own per-report reset. Every case in test_netfsm.cpp today calls net_begin() as its own
   first or second statement, which is why no teardown entry has been needed before this —
   but that is exactly the trap the other six entries above it exist to close, and a case
   that longjmps out mid-body before reaching ITS OWN net_begin() (or a future case that
   never calls it at all) would otherwise read the previous case's retry/starvation state.
   pb_test_teardown() is the only caller. */
void netfsm_test_reset_retry_(void);

/* Host-suite seam: was_timeout()'s subtract-and-test-sign idiom is a file-static, and the
   fake clock's own "every hal_millis() read advances the rig by 1 ms" contract means net_poll()
   can never present it with an elapsed time of EXACTLY PB_NET_STEP_MS -- capturing t0 costs one
   tick, was_timeout()'s own hal_millis() call costs a second, so every timeout net_poll() can
   ever manufacture reads PB_NET_STEP_MS + 1, one past the boundary this idiom's >= is supposed
   to pin. Calling the real comparison directly against a clock landed on the exact boundary is
   the only way to tell >= apart from >. Not a second copy of the policy — the same static,
   exposed for a controlled t0. */
bool netfsm_test_was_timeout_(uint32_t t0);
#endif
