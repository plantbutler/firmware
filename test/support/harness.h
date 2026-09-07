/* harness.h: the Unity fixture, a header because PlatformIO builds one binary per test/ subdirectory; the host arm has hal_sim's injectors and a driven clock, the device arm real hardware and real time. */
#pragma once
#include "config.h"
#include "hal.h"
#include "safety.h"
#include <unity.h>

#ifdef PB_SIM
#include "cli.h"
#include "exec.h"
#include "netfsm.h"
#include "pulses.h"
#include "report.h"
#include "sensors.h"
#include "sim.h"

static inline void pb_test_setup(void) {
  sim_reset(false);          /* a cold boot: clock at 0, .noinit cleared */
  hal_begin();
  hal_boot_pump_off();
  (void)hal_wdt_start();     /* without it hal_wdt_alive() is false and the dose ladder
                                 refuses every dose */
  sim_events_clear();
}

/* Every reset belongs here, not at the end of whichever case dirtied the state: Unity
   aborts a failing TEST_ASSERT_* with a longjmp straight into tearDown(), skipping every
   line after it in the case body, and each of these is a process-lifetime static in another
   translation unit that sim_reset() (hal_sim.cpp's own statics only) never touches.
   tearDown() is the one place Unity guarantees runs however the case ended. The dose
   cooldown reset to 0 reads as "no dose has ended yet", so the cooldown rung skips its check
   outright instead of every case having to land its clock clear of the previous case's
   leftover. report_clear_ack() and exec_begin() are production entry points that reset
   exactly these statics and nothing else, so they need no test-only twin.
   Host-only: the _test_reset_ helpers do not exist in a device build. */
static inline void pb_test_teardown(void) {
  sim_events_clear();
  safety_set_dosing(false);
  safety_float_refusal_count(false);
  cli_stop_clear();
  safety_reset_dose_cooldown_();
  sensors_test_reset_health_();
  pulses_test_reset_leak_();
  report_clear_ack();
  netfsm_test_reset_retry_();
  exec_begin();
}

static inline void pb_advance(uint32_t ms) { sim_advance(ms); }

static inline uint32_t pb_count(sim_ev_kind_t kind) {
  const sim_ev_t *ev; size_t n = sim_events(&ev); uint32_t hits = 0;
  for (size_t i = 0; i < n; ++i) if (ev[i].kind == kind) hits++;
  return hits;
}

/* Strictly inside: the two feeds that bracket hal_wdt_alive()'s probe -- the one
   deliberately unfed window in the program -- are legal. */
static inline void pb_expect_no_feed_between(uint32_t from_ms, uint32_t to_ms) {
  const sim_ev_t *ev; size_t n = sim_events(&ev); uint32_t hits = 0;
  for (size_t i = 0; i < n; ++i)
    if (ev[i].kind == SIM_EV_WDT_FEED && ev[i].at_ms > from_ms && ev[i].at_ms < to_ms) hits++;
  TEST_ASSERT_EQUAL_UINT32(0u, hits);
}

/* There is no safety_contra_set_(): the latch is settable in exactly one place, so the
   fixture earns it by driving a real latching dose. */
static inline void pb_latch_contra(void) {
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true);
  sim_set_flow_ml_s(0);                  /* float OK, no flow: the contradiction */
  dose_req_t q = {0};
  q.by_time    = true;
  q.cap_ms     = PB_PRIME_MS_DEFAULT + PB_STALL_MS_DEFAULT + 1000u;
  q.long_prime = false;                  /* a console prime is exempt */
  (void)dose_run(&q);
  TEST_ASSERT_TRUE_MESSAGE(safety_contra(), "pb_latch_contra did not latch");
}

/* n whole network passes, ms of fake clock between them: the one spelling of "a pass".
   Under PB_SIM because the device build filters link_fake.cpp out. */
static inline void pb_net_passes(uint16_t n, uint32_t ms) {
  for (uint16_t i = 0; i < n; ++i) {
    link_fake_pass_begin();
    net_poll(false);         /* not dosing: the dosing loop blocks, so no pass overlaps one */
    if (ms) pb_advance(ms);
  }
}

#else   /* the device arm: real hardware, real time, no injectors. The other helpers read
           the sim or the fake link and have no device meaning. */
static inline void pb_test_setup(void)     { hal_begin(); hal_boot_pump_off(); }
static inline void pb_test_teardown(void)  {}
static inline void pb_advance(uint32_t ms) { safety_wait_ms(ms); }   /* fed, on real time */
#endif
