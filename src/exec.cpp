/* exec.cpp — at most one command per pass, executed only when the socket is closed.
   §1's module table puts this in main.cpp; it lives here because [env:native] filters main.cpp
   out and netfsm.cpp is grepped for dose_run. See the commit message. */
#include "exec.h"
#include "cart.h"
#include "cli.h"        /* cli_print_dose_summary() -- §6 wants the summary line from EVERY
                           dose path, and this file is the path that runs unattended */
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

/* THE one function every terminal path goes through, which is why the OLED's row-7 pair is
   filled here and not at four call sites that would drift. "ok <n>ml" when water actually
   moved, "REF <err>" otherwise: §5's row 7 reads `cmd 17 ok 248ml` or `cmd 17 REF float`,
   and 16 characters is all there is. No float conversion anywhere (§12 item 1). */
static void ack(uint32_t id, uint16_t flow_ml, const char *err) {
  report_set_ack(id, flow_ml, err);
  g_last_id = id;
  if (err && strcmp(err, "none") == 0)
    snprintf(g_last_text, sizeof g_last_text, "ok %luml", (unsigned long)flow_ml);
  else
    snprintf(g_last_text, sizeof g_last_text, "REF %s", err ? err : "?");
}

void exec_pending(void) {
  /* §2.11: the boot self-home runs under BOTH latches. Gating it on !dry_latched would leave
     the cart wherever a mid-dose watchdog reset stopped it — and §2.3 latches dry on exactly
     that case — holding gate N open under the reservoir head until a human types `dry off`.
     It drives the servo, not D6; safety_tick() re-asserts pump-OFF on every pass of the move. */
  if (g_boot_home_due && hal_millis() >= PB_BOOT_HOME_MS) {
    g_boot_home_due = false;
    (void)cart_home();
  }

  if (!g_pending && !net_take_command(&g_cmd)) return;
  g_pending = true;
  /* Nothing has been consumed yet, so no park is owed if the socket is still open. */
  if (net_state() != NET_IDLE) return;

  /* From here the command is CONSUMED. The ack already exists — netfsm set (id, 0, "recv") on
     receipt — and every path below OVERWRITES it (§4.3). */

  if (g_cmd.kind == CMD_STOP) { ack(g_cmd.id, 0, "stop"); goto park; }

  /* ABOVE cart_goto() on purpose: this is what makes §4.5's promise true — an out-of-range
     outlet is refused with err=range and acked, rather than the backend receiving whichever
     cart error happened first. It is also what handles water=0, which butler accepts. */
  if (g_cmd.outlet < 1 || g_cmd.outlet > PB_OUTLETS) { ack(g_cmd.id, 0, "range"); goto park; }

  if (!cart_goto(g_cmd.outlet)) { ack(g_cmd.id, 0, "goto"); goto park; }

  {
    /* `= {0}`, NOT a bare declaration. dose_req_t carries an unconditional `hang` member
       (task 20 step 7), and an uninitialised one plus `el >= PB_HANG_MS` puts a BACKEND
       water command into the loop that deliberately starves the watchdog for bring-up 7c.
       cli_run_dose_() zero-initialises for the same reason; this is the path that runs
       unattended for 48 hours. */
    dose_req_t q = {0};
    q.outlet = g_cmd.outlet;
    q.ml = g_cmd.ml;
    q.by_time = false;
    q.cap_ms = (uint32_t)g_cmd.cap_s * 1000u;
    q.need_pos = true;                   /* a backend water command: position must be known */
    q.long_prime = false;                /* never from the wire: `prime` is a console token */
#if defined(PB_DOSE_BY_TIME) && PB_DOSE_BY_TIME
    /* §6's stated 7b fallback, against the SAME constant the cap clamp uses; config.h #errors
       if PB_ML_PER_S_MEASURED is 0, because a by-time dose against an unmeasured rate is an
       unbounded run in a costume. */
    q.by_time = true;
    {
      uint32_t byt = (uint32_t)g_cmd.ml * 1000u / PB_ML_PER_S_MEASURED;
      if (byt < q.cap_ms) q.cap_ms = byt;
    }
#endif
    dose_result_t r = dose_run(&q);
    /* §6: the per-dose summary line is printed "at the end of every dose, from EVERY path".
       cli.cpp's own helper covers `pump` and `calib`; this is the other path, and it is the
       one that runs unattended. This is cli.cpp's SECOND exported entry point into this
       file, and it does not affect the single-`dose_run(`-call-site grep, which is scoped
       to cli.cpp on purpose. */
    cli_print_dose_summary();
    ack(g_cmd.id, dose_flow_ml(), err_of(r));   /* the HONEST millilitres, 0 for a refusal */
  }

park:
  /* §2.9: EVERY consumed command parks, goto failures included. The magnet cart lifts the gate
     it sits over and the reservoir sits above the pump inlet, so a cart left over outlet N
     holds that gate open under a head of water until the next command — six hours, or never. */
  g_pending = false;
  (void)cart_home();
}
