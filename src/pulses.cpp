/* src/pulses.cpp — D2/D3 ISR bodies, gap reject, snapshots, rate, ml, leak watch. */
#include "pulses.h"
#include "hal.h"
#include "config.h"

static volatile uint32_t g_flow, g_screw;
static volatile uint32_t g_flow_last_us, g_screw_last_us;

static uint32_t g_win_t0, g_win_base, g_rate_prev;

static uint32_t g_leak_rearm_ms, g_leak_base, g_leak_count;
static bool     g_leak_armed;

#if PB_SIM
static uint32_t g_tear_pending;
void pulses_test_tear_next(uint32_t edges) { g_tear_pending = edges; }
static void tear_(void) {
  if (!g_tear_pending) return;
  uint32_t n = g_tear_pending;
  g_tear_pending = 0;
  while (n--) {
    g_flow_last_us = hal_micros() - (uint32_t)PB_FLOW_MIN_GAP_US - 1u;   /* force acceptance */
    pulses_isr_flow();
  }
}
#else
static void tear_(void) { }
#endif

void pulses_begin(void) {
  g_flow = g_screw = 0;
  /* back-date both, so the FIRST edge after boot is never rejected as too close */
  g_flow_last_us  = hal_micros() - (uint32_t)PB_FLOW_MIN_GAP_US  - 1u;
  g_screw_last_us = hal_micros() - (uint32_t)PB_SCREW_MIN_GAP_US - 1u;
  g_win_t0 = hal_millis();
  g_win_base = 0; g_rate_prev = 0;
  g_leak_rearm_ms = g_win_t0; g_leak_base = 0; g_leak_count = 0; g_leak_armed = false;
}

void pulses_isr_flow(void) {
  uint32_t now = hal_micros();
  if ((uint32_t)(now - g_flow_last_us) < (uint32_t)PB_FLOW_MIN_GAP_US) return;
  g_flow_last_us = now;
  g_flow++;
}

void pulses_isr_screw(void) {
  uint32_t now = hal_micros();
  if ((uint32_t)(now - g_screw_last_us) < (uint32_t)PB_SCREW_MIN_GAP_US) return;
  g_screw_last_us = now;
  g_screw++;
}

uint32_t pulses_flow(void) {
  uint32_t a, b;
  do { a = g_flow; tear_(); b = g_flow; } while (a != b);
  return a;
}

uint32_t pulses_screw(void) {
  uint32_t a, b;
  do { a = g_screw; b = g_screw; } while (a != b);
  return a;
}

uint32_t pulses_flow_rate(void) {
  uint32_t now = hal_millis();
  uint32_t elapsed = now - g_win_t0;
  if (elapsed >= (uint32_t)PB_FLOW_RATE_WINDOW_MS) {
    uint32_t n = pulses_flow();
    g_rate_prev = ((n - g_win_base) * 1000u) / elapsed;
    g_win_base = n;
    g_win_t0 = now;
  }
  return g_rate_prev;
}

uint32_t pulses_to_ml(uint32_t pulses, uint16_t pulses_per_l) {
  if (pulses_per_l == 0u) return 0u;
  if (pulses <= (0xFFFFFFFFu / 1000u)) return (pulses * 1000u) / pulses_per_l;
  return (pulses / pulses_per_l) * 1000u + ((pulses % pulses_per_l) * 1000u) / pulses_per_l;
}

void pulses_leak_rearm_at(uint32_t at_ms) {
  g_leak_rearm_ms = at_ms;
  g_leak_armed = false;
  g_leak_base = pulses_flow();
}

void pulses_leak_poll(bool pump_on) {
  if (pump_on) {                       /* pulses with the pump ON are just the dose */
    g_leak_armed = false;
    g_leak_base = pulses_flow();
    return;
  }
  if (!g_leak_armed) {
    if ((int32_t)(hal_millis() - g_leak_rearm_ms) < 0) {   /* still coasting down */
      g_leak_base = pulses_flow();
      return;
    }
    g_leak_armed = true;
    g_leak_base = pulses_flow();
    return;
  }
  uint32_t n = pulses_flow();
  if (n > g_leak_base) {
    g_leak_count += n - g_leak_base;
    g_leak_base = n;
  }
}

uint32_t pulses_leak_count(void) { return g_leak_count; }
bool     pulses_leak_seen(void)  { return g_leak_count > 0u; }
