/* include/cli.h -- the console. Bench commands always; bring-up commands under
   #if PB_BRINGUP (task 20). A flat if-chain, so cli_dispatch() is testable. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

void cli_begin(void);
void cli_poll(void);
bool cli_dispatch(const char *line);
void cli_print_status(void);
void cli_printf_u32(const char *fmt, uint32_t v);
/* Printed at the end of EVERY dose, from EVERY path (spec §6's pitch deliverable) --
   cli.cpp's own pump/calib helper calls it, and task 26's exec.cpp calls it again for the
   backend's doses in the bench build. Declared here, OUTSIDE #if PB_BRINGUP, because both
   binaries print it; defined in src/cli.cpp, also outside the #if -- a body in this header
   would multiply-define across cli.cpp, exec.cpp and main.cpp. */
void cli_print_dose_summary(void);
/* RSSI is the one signed figure `status` prints; %d is banned in report.cpp and
   netfsm.cpp only, and cli.cpp is outside that grep (task 24 step 17). */
void cli_printf_i32(const char *fmt, int32_t v);

/* §2.12. The last-resort abort, called from dose_run()'s loop (task 17) once per
   iteration. A byte-at-a-time matcher over its own state, with its OWN hal_serial_read():
   it consumes only the bytes of an exact `stop\n` or `dry on\n` and pushes every other
   byte into cli.cpp's line buffer unread, so `status` typed mid-dose is still `status`
   once the dose ends and cli_poll() gets a turn. Latches until cli_stop_clear(). */
bool cli_stop_requested(void);
/* Forgets the pending request, the partial match AND the pushback -- task 17's
   end-of-dose sequence is hal_serial_drain(); cli_stop_clear();, which discards the UART
   ring and this file's pushback together (§2.8, §15.3): bytes typed during a dose are
   DISCARDED, never queued into a command after it. */
void cli_stop_clear(void);

#if PB_BRINGUP && defined(PB_NATIVE)
/* Host-suite seam, same shape as safety.h's: `pump`'s [prime] [hang] flag parser is a
   pure string inspection with no side effect, forwarded here so a host case can prove
   the literal-token requirement (spec §6) directly -- without ever setting hang=true on
   a dose_req_t and reaching the loop that deliberately starves the watchdog, which would
   hang the suite by construction. Only exists where the bring-up console itself does. */
void cli_pump_flags_for_test_(const char *args, bool *prime, bool *hang);
#endif
