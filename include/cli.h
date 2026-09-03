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
