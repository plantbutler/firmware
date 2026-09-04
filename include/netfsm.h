/* include/netfsm.h — seam 2's state machine: the report state machine, the HTTP framing
   and the two-AT budget. Spec §3.
   ONE bounded link/socket step per net_poll(), AT MOST 2 AT commands, each with its own
   deadline. Worst pass is CONNECT = _BEGINCLIENT + _CLIENTCONNECT = 2 * PB_NET_STEP_MS =
   2400 ms against a granted 5592 ms — and that holds ONLY because sock_close() ran in a PRIOR
   pass and left _sock == -1, and because every error exit routes through NET_SOCK_CLOSE
   instead of closing inline. Spec §3's per-pass AT table. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "report.h"      /* cmd_t */

typedef enum { NET_DOWN, NET_JOIN_ISSUE, NET_JOIN_WAIT, NET_IDLE,
               NET_SOCK_CLOSE, NET_CONNECT, NET_SEND, NET_RECV, NET_CLOSE } net_state_t;

void        net_begin(void);
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
