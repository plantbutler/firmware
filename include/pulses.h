/* pulses.h: the two interrupt counters, the per-pin gap reject, torn-read-safe snapshots,
   the rate estimator, pulses->ml, and the leak watch. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

void     pulses_begin(void);

/* The ISR bodies: hal_uno.cpp's two ISRs call these and do nothing else. Each rejects an
   edge closer than its own minimum gap, by hal_micros(). */
void     pulses_isr_flow(void);
void     pulses_isr_screw(void);

/* Read, re-read, repeat until two reads agree. NO interrupt masking: masking is what
   drops edges, and a dropped screw edge is lost cart position, silently. */
uint32_t pulses_flow(void);
uint32_t pulses_screw(void);

/* Hz over a PB_FLOW_RATE_WINDOW_MS tumbling window; the previous window's value stands
   while the current one fills. No floats, no ring buffer. */
uint32_t pulses_flow_rate(void);

/* Multiply first, divide second. Returns 0 for a zero calibration rather than trusting
   UDIV, which returns 0 on divide by zero. */
uint32_t pulses_to_ml(uint32_t pulses, uint16_t pulses_per_l);

/* The leak watch. The dose loop rearms it at hal_millis() + PB_COAST_MS at the end of
   every dose, so impeller spin-down is not a leak. There is NO latch: these pulses raise
   ch205 and err=leak, they never block a dose. */
void     pulses_leak_rearm_at(uint32_t at_ms);
void     pulses_leak_poll(bool pump_on);
uint32_t pulses_leak_count(void);
bool     pulses_leak_seen(void);

#if PB_SIM
/* Host-only: inject `edges` flow edges between the two reads of the next snapshot. The
   host has no preemption, so this is the only way to prove the double-read loop. */
void     pulses_test_tear_next(uint32_t edges);

/* Same, for pulses_screw()'s identical retry loop. */
void     pulses_test_tear_screw_next(uint32_t edges);

/* Host-suite seam: g_leak_count and the armed/base/rearm state around it are
   process-lifetime statics with no reset path of their own -- pulses_begin() resets them
   but ALSO zeroes g_flow/g_screw and the rate window, which no leak-only cleanup wants.
   Without this, a case that storms the meter leaves err=leak reachable in every later
   case by residue. pb_test_teardown() is the only caller. */
void     pulses_test_reset_leak_(void);
#endif
