/* exec.cpp: the boot self-home, and at most one backend command per pass, run only when the socket is closed.
   Not in main.cpp, which no host test can reach, and not in netfsm.cpp, which may not dose. */
#include "exec.h"
#include "cart.h"
#include "cli.h"
#include "config.h"
#include "hal.h"
#include "netfsm.h"
#include "report.h"
#include "safety.h"
#include <stdio.h>
#include <string.h>

static bool     g_boot_home_due;
static bool     g_pending;
static cmd_t    g_cmd;
static uint32_t g_last_id;
static char     g_last_text[16];

void exec_begin(void) {
  g_boot_home_due = true; g_pending = false;
  g_last_id = 0u; g_last_text[0] = '\0';
}

uint32_t    exec_last_cmd_id(void)   { return g_last_id; }
const char *exec_last_cmd_text(void) { return g_last_text[0] ? g_last_text : 0; }

/* Every terminal path ends here, so the OLED's row-7 text is filled once and not at four
   call sites that would drift: `cmd 17 ok 248ml` or `cmd 17 REF float`, in 16 columns.
   No float conversion anywhere. */
static void ack(uint32_t id, uint16_t flow_ml, const char *err) {
  report_set_ack(id, flow_ml, err);
  g_last_id = id;
  if (err && strcmp(err, "none") == 0)
    snprintf(g_last_text, sizeof g_last_text, "ok %luml", (unsigned long)flow_ml);
  else
    snprintf(g_last_text, sizeof g_last_text, "REF %s", err ? err : "?");
}

#ifdef PB_NATIVE
/* Host-suite seam. PB_HANG_MS and PB_PRIME_MS_DEFAULT are both 3000, so the no-flow abort
   fires on the millisecond the hang would begin; only the FIELD tells them apart. */
static dose_req_t g_last_req;
dose_req_t exec_test_last_req_(void) { return g_last_req; }
#endif

void exec_pending(void) {
  /* The boot self-home runs under BOTH latches: a mid-dose reset latches dry, and gating
     on it would leave the cart holding a gate open under the reservoir head until a human
     types `dry off`. It drives the servo, not D6; pump OFF is re-asserted on every pass. */
  if (g_boot_home_due && hal_millis() >= PB_BOOT_HOME_MS) {
    g_boot_home_due = false;
    (void)cart_home();
  }

  if (!g_pending && !net_take_command(&g_cmd)) return;
  g_pending = true;
  /* Nothing has been consumed yet, so no park is owed if the socket is still open. */
  if (net_state() != NET_IDLE) return;

  /* From here the command is CONSUMED. The ack already exists — netfsm set (id, 0, "recv") on
     receipt — and every path below OVERWRITES it. */

  if (g_cmd.kind == CMD_STOP) { ack(g_cmd.id, 0, "stop"); goto park; }

  /* ABOVE cart_goto() on purpose: an out-of-range outlet is acked err=range rather than
     whichever cart error happened first. Also what handles outlet 0, which butler accepts. */
  if (g_cmd.outlet < 1 || g_cmd.outlet > PB_OUTLETS) { ack(g_cmd.id, 0, "range"); goto park; }

  if (!cart_goto(g_cmd.outlet)) { ack(g_cmd.id, 0, "goto"); goto park; }

  {
    /* `= {0}`, NOT a bare declaration: an uninitialised `hang` member would put a BACKEND
       water command into the loop that deliberately starves the watchdog. */
    dose_req_t q = {0};
    q.outlet = g_cmd.outlet;
    q.ml = g_cmd.ml;
    q.by_time = false;
    q.cap_ms = (uint32_t)g_cmd.cap_s * 1000u;
    q.need_pos = true;                   /* a backend water command: position must be known */
    q.long_prime = false;                /* never from the wire: `prime` is a console token */
#if defined(PB_DOSE_BY_TIME) && PB_DOSE_BY_TIME
    /* The by-time fallback, against the SAME constant the cap clamp uses; config.h #errors
       if PB_ML_PER_S_MEASURED is 0: a by-time dose on an unmeasured rate is an unbounded run. */
    q.by_time = true;
    {
      uint32_t byt = (uint32_t)g_cmd.ml * 1000u / PB_ML_PER_S_MEASURED;
      if (byt < q.cap_ms) q.cap_ms = byt;
    }
#endif
#ifdef PB_NATIVE
    g_last_req = q;      /* the request AS BUILT, for the host suite to assert on directly */
#endif
    dose_result_t r = dose_run(&q);
    /* the summary line prints at the end of EVERY dose from every path: cli.cpp covers
       `pump` and `calib`, this is the one that runs unattended */
    cli_print_dose_summary();
    ack(g_cmd.id, dose_flow_ml(), err_of(r));   /* the HONEST millilitres, 0 for a refusal */
  }

park:
  /* EVERY consumed command parks, goto failures included. The magnet cart lifts the gate
     it sits over and the reservoir sits above the pump inlet, so a cart left over outlet N
     holds that gate open under a head of water until the next command — six hours, or never. */
  g_pending = false;
  (void)cart_home();
}
