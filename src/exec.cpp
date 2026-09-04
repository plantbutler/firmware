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

  /* PLACEHOLDER, replaced whole by step 8. Written out rather than elided so that the file
     as printed compiles and step 4's four cases can actually run. */
  g_pending = false;
}
