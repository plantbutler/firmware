/* cli.h: the console. Bench commands always; bring-up commands only in the bring-up build. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

void cli_begin(void);
void cli_poll(void);
bool cli_dispatch(const char *line);
void cli_print_status(void);
void cli_printf_u32(const char *fmt, uint32_t v);
/* Printed at the end of EVERY dose, from every path (the console's own pump helper and
   exec.cpp's backend doses). Declared outside the bring-up guard because both binaries
   print it. */
void cli_print_dose_summary(void);
/* RSSI is the one signed figure `status` prints; the %d ban covers the wire, not the console. */
void cli_printf_i32(const char *fmt, int32_t v);

/* The last-resort abort, polled once per dose-loop iteration. A byte-at-a-time matcher
   with its OWN serial read: it consumes only the bytes of an exact `stop\n` or `dry on\n`
   and pushes every other byte back into the line buffer unread, so `status` typed
   mid-dose is still `status` once the dose ends. Latches until cli_stop_clear(). */
bool cli_stop_requested(void);
/* Forgets the pending request, the partial match AND the pushback. The end of a dose
   drains the UART ring and calls this together: bytes typed during a dose are DISCARDED,
   never queued into a command after it. */
void cli_stop_clear(void);

#if PB_BRINGUP && defined(PB_NATIVE)
/* Host-suite seam: `pump`'s [prime] [hang] flag parser is a pure string inspection,
   exposed so a host case can prove the literal-token rule without ever setting hang=true
   on a request and reaching the loop that deliberately starves the watchdog. */
void cli_pump_flags_for_test_(const char *args, bool *prime, bool *hang);
#endif
