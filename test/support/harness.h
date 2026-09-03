/* test/support/harness.h — the Unity fixture over hal_sim. A header, not a suite. */
#pragma once
#include <unity.h>
#include "hal.h"
#include "sim.h"

static inline void pb_test_setup(void) {
  sim_reset(false);          /* a cold boot: clock at 0, .noinit cleared */
  hal_begin();
  hal_boot_pump_off();
  (void)hal_wdt_start();
  sim_events_clear();
}

static inline void pb_test_teardown(void) { sim_events_clear(); }

static inline void pb_advance(uint32_t ms) { sim_advance(ms); }

static inline uint32_t pb_count(sim_ev_kind_t kind) {
  const sim_ev_t *ev; size_t n = sim_events(&ev); uint32_t hits = 0;
  for (size_t i = 0; i < n; ++i) if (ev[i].kind == kind) hits++;
  return hits;
}

/* Strictly inside: the two feeds that BRACKET hal_wdt_alive()'s probe are legal, and
   anything between them is the bug this exists to catch (§2.5). */
static inline void pb_expect_no_feed_between(uint32_t from_ms, uint32_t to_ms) {
  const sim_ev_t *ev; size_t n = sim_events(&ev); uint32_t hits = 0;
  for (size_t i = 0; i < n; ++i)
    if (ev[i].kind == SIM_EV_WDT_FEED && ev[i].at_ms > from_ms && ev[i].at_ms < to_ms) hits++;
  TEST_ASSERT_EQUAL_UINT32(0u, hits);
}
