/* sim.h — the fake rig's fault injectors and its call-trace log. sim + native only. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "link.h"       /* link_state_t, for the seam-2 fake's control surface at EOF */

/* THE FAKE'S CLOCK CONTRACT, and every later task depends on it:
     hal_millis()   advances the rig by exactly 1 ms and runs every model (watchdog,
                    pump-on time, and from task 6 the flow and screw edges), then returns
                    the millisecond it has just reached. That is what lets a bounded spin loop —
                    hal_wdt_alive()'s probe, safety_wait_ms(), dose_run()'s loop — make
                    progress on a host with no real time.
     hal_micros()   READS the clock. It never advances it, so an ISR's minimum-gap reject
                    measures the interval the fake scheduled and not the reading of it.
     hal_delay_us() advances by the microseconds asked for, watchdog and pump time only.
     sim_advance(n) is n of the same 1 ms step. */
#define PB_SIM_TICK_US 1000u

/* The whole-word PmnPFS shape of §2.1, mirrored so the host can assert it:
   IOPORT_CFG_PORT_DIRECTION_OUTPUT = 0x4, IOPORT_CFG_PORT_OUTPUT_HIGH = 0x1
   (r_ioport_api.h:184-186). */
#define SIM_PFS_DIR_OUT  0x4u
#define SIM_PFS_LEVEL_HI 0x1u

void     sim_reset(bool warm);             /* re-enter setup(); warm keeps .noinit (task 4) */
void     sim_advance(uint32_t ms);
void     sim_set_float(bool ok);           /* D5; consumed by the debounce, task 15 */
/* Each character is consumed by ONE hal_pin_read(PIN_HALL_FLOAT): '1' == OK (the pin reads
   PB_LOW), '0' == not OK. The last character repeats forever, so "100" is a float that
   grants one sample and fails the next two -- exactly the waterline bounce of §2.10.
   sim_set_float() clears any pattern. */
void     sim_set_float_pattern(const char *pattern);
void     sim_set_flow_ml_s(uint16_t ml_s); /* the pump's delivery rate; task 6 */
void     sim_flow_storm(uint32_t hz);      /* an edge storm on D2; task 6 */
/* A storm that begins on the ON write, not before it. The pre-dose PB_FLOW_IDLE_MAX_HZ
   guard cannot see this one, and it is the scenario that matters: a floating D2 running
   beside the 12 V pump leg starts counting garbage WHEN THE PUMP DOES. */
void     sim_flow_storm_at_pump_on(uint32_t hz);
/* Deliver exactly n flow pulses immediately after the ON write, then nothing, ever. The
   stall rule's fixture: a dose that flowed briefly and stopped (task 18 step 7). */
void     sim_set_flow_burst_pulses(uint32_t n);
/* Scheduled injectors, both measured from the next pump-on: apply the change when the
   fake's clock reaches `ms` into the dose. Task 18 step 8's two mid-dose console cases use
   the second; task 19 step 4 uses both, and must not add a second spelling of either. */
void     sim_set_float_at_ms(uint32_t ms, bool ok);
void     sim_serial_rx_at_ms(uint32_t ms, const char *s);
void     sim_set_i2c_fail(bool fail);      /* every expander transfer fails; task 7 */
void     sim_set_mux_stuck(bool stuck);    /* every channel returns the canary's value; task 7 */
void     sim_set_stall(bool on);           /* the screw stops pulsing while driven; task 14 */
void     sim_set_leak(bool on);            /* pulses with D6 off; task 6 */
void     sim_wdt_stop(void);               /* the counter FREEZES */
void     sim_wdt_rate_hz(uint32_t hz);     /* the counter moves, at this rate */
void     sim_noinit_clobber(void);         /* scramble the .noinit struct; task 4 */
void     sim_set_channel(uint8_t ch, uint16_t raw);
void     sim_serial_rx(const char *s);     /* push bytes at the console */
size_t   sim_serial_tx(char *buf, size_t cap);   /* drain what the console printed */
bool     sim_pump_is_on(void);
uint32_t sim_pump_on_ms(void);             /* cumulative ms with D6 asserted */
uint32_t sim_feeds(void);

/* The screw: one D3 pulse every sim_set_screw_pulse_ms() of commanded rotation, counted
   up or down according to the servo microseconds last written through seam 1. The home
   REGION, not a point: the hall reads asserted anywhere in [lo, hi] pulses, which is what
   a magnet over a hall actually does and what makes "drive until home" terminate. */
void     sim_set_screw_pulse_ms(uint32_t ms);      /* 0 == the screw does not turn at all */
void     sim_set_home_region(uint32_t lo, uint32_t hi);
void     sim_set_cart_at(uint32_t pulses);         /* place the cart without moving it */
uint16_t sim_servo_us(void);                       /* what cart.cpp last commanded */
uint32_t sim_servo_stops(void);                    /* count of writes of 1500 */

/* The clock injector. It lands HERE, not in task 22, because task 14 step 9's rollover
   case is its first consumer and a case that has to reach eight tasks forward for its
   fixture is a case that gets written twice. Body: g_us = ms * 1000u; g_ms = ms;
   g_ms_frac_us = 0u; one bounded tick_models_() call; nothing else. The frac reset is not
   optional: g_ms is tracked as its own counter (not derived from g_us — see the note by
   its declaration in hal_sim.cpp), so a stale sub-millisecond remainder from before the
   jump would otherwise carry into the value hal_delay_us() next reports. Task 22 step 13
   adds sim_set_heap_break() ONLY. */
void     sim_set_clock_ms(uint32_t ms);

/* Move the fake's break, so §12 item 0's per-report check has something to fail against.
   hal_heap_break() otherwise returns a fixed address comfortably inside the margin (task 3
   step 6), which exercises the happy path and nothing else. Task 22 step 13. */
void     sim_set_heap_break(uint32_t addr);

typedef enum {
  SIM_EV_PIN_CFG,      /* a whole-word direction+level write: hal_boot_pump_off, hal_pin_write */
  SIM_EV_PIN_MODE,
  SIM_EV_PUMP_WRITE,   /* arg carries SIM_PFS_DIR_OUT | level, the same word the board writes */
  SIM_EV_WDT_FEED,
  SIM_EV_I2C_WRITE,
  SIM_EV_I2C_READ,
  SIM_EV_SERVO,
  SIM_EV_ADC,
  SIM_EV_SCREW         /* ADDED HERE: one per accepted screw pulse (task 14) */
} sim_ev_kind_t;

typedef struct {
  sim_ev_kind_t kind;
  uint8_t       pin;
  uint32_t      arg;
  uint32_t      at_ms;
} sim_ev_t;

size_t sim_events(const sim_ev_t **out);
void   sim_events_clear(void);

/* ---- seam 2's fake (src/link_fake.cpp). sim + native only. ---- */
void        link_fake_reset(void);
void        link_fake_set_state(link_state_t s);
void        link_fake_queue_response(const char *raw, size_t n);
void        link_fake_fail_open(bool fail);
void        link_fake_timeout_next(void);   /* next AT round trip burns PB_NET_STEP_MS and fails */
void        link_fake_drop_link(void);
void        link_fake_pass_begin(void);     /* zero the per-pass AT counter */
uint16_t    link_fake_at_count(void);
bool        link_fake_saw_available(void);
bool        link_fake_saw_connected(void);
uint16_t    link_fake_reset_count(void);
const uint8_t *link_fake_sent(uint16_t *len);
uint16_t    link_fake_write_count(void);    /* sock_write() calls since link_fake_reset() */
