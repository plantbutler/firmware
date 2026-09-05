/* include/pulses.h — the two interrupt counters, the per-pin gap reject, torn-read-safe
   snapshots, the rate estimator, pulses->ml, and the leak watch. Spec §1, §2.14, §7. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

void     pulses_begin(void);

/* The ISR bodies. hal_uno.cpp's two ISRs call these and do nothing else; each rejects an
   edge closer than its own minimum gap, using hal_micros(). */
void     pulses_isr_flow(void);
void     pulses_isr_screw(void);

/* Read, re-read, repeat until two reads agree. NO interrupt masking: masking is what
   drops edges, and a dropped screw edge is lost cart position, silently. */
uint32_t pulses_flow(void);
uint32_t pulses_screw(void);

/* Hz over a PB_FLOW_RATE_WINDOW_MS tumbling window; the previous window's value stands
   while the current one fills. No floats, no ring buffer. */
uint32_t pulses_flow_rate(void);

/* Multiply first, divide second (§6). Returns 0 for a zero calibration rather than
   trusting UDIV. */
uint32_t pulses_to_ml(uint32_t pulses, uint16_t pulses_per_l);

/* The leak watch. dose_run() calls pulses_leak_rearm_at(hal_millis() + PB_COAST_MS) at
   the end of every dose, so impeller spin-down is not a leak. There is NO latch: these
   pulses raise ch205 and err=leak, they never block a dose (§1). */
void     pulses_leak_rearm_at(uint32_t at_ms);
void     pulses_leak_poll(bool pump_on);
uint32_t pulses_leak_count(void);
bool     pulses_leak_seen(void);

#if PB_SIM
/* Host-only: inject `edges` flow edges between the two reads of the next snapshot. The
   host has no preemption, so this is the only way to prove the double-read loop. Never
   compiled into the bench binary. */
void     pulses_test_tear_next(uint32_t edges);

/* Same shape, for pulses_screw()'s identical retry loop -- added by task 14, which owns
   the screw model, because the loop had no injector anywhere and was asserted only by
   the always-agrees-on-the-first-pass happy path. */
void     pulses_test_tear_screw_next(uint32_t edges);

/* Host-suite seam, same shape as safety.cpp's g_dosing/g_float_refusals/g_last_end_ms trio and
   sensors.cpp's g_healthy/g_fails/g_backoff_until trio: g_leak_count (and the armed/base/rearm
   state around it) is a process-lifetime static in pulses.cpp with no reset path of its own --
   pulses_begin() resets it, but pulses_begin() ALSO zeroes g_flow/g_screw and the rate window,
   which no test wants as a side effect of a leak-count-only cleanup. A case that storms the
   meter and never calls pulses_begin() itself (found leaving `err=leak` reachable purely by
   residue from an earlier test, task 22 fix round 2) leaves every later case in the binary
   reading a nonzero count for a reason that has nothing to do with what it tests.
   pb_test_teardown() is the only caller. */
void     pulses_test_reset_leak_(void);
#endif
