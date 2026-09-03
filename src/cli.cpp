/* src/cli.cpp -- the console (spec §6). Bench commands here; task 20 adds the
   #if PB_BRINGUP block, and tasks 15/16/19 add dry, stop and the latch release.
   secrets.h is here for PB_CONTROLLER, which `status` prints: neither [env:uno_r4_wifi]
   nor [env:uno_r4_wifi_bringup] passes it in build_flags, and secrets.h is the only
   header that defines it. Task 14 adds cart.h, task 15/16 safety.h, task 24 netfsm.h
   and link.h -- each at TOP LEVEL, never inside the #if PB_BRINGUP block, because
   cli_print_status() calls into all of them unconditionally. */
#include "cart.h"
#include "cli.h"
#include "config.h"
#include "hal.h"
#include "noinit.h"
#include "pins.h"
#include "pulses.h"
#include "secrets.h"
#include "sensors.h"
#include <stdio.h>
#include <string.h>

#if defined(PB_SIM)
#  define PB_BUILD_NAME "sim"
#elif defined(PB_BRINGUP)
#  define PB_BUILD_NAME "bringup"
#else
#  define PB_BUILD_NAME "bench"
#endif

static char     g_line[PB_LINE_CAP];
static uint16_t g_len;
static bool     g_overlong;
static bool     g_hall_stream;
static uint32_t g_hall_next_ms;
static uint32_t g_arena_min, g_arena_max, g_hwm_max;

void cli_printf_u32(const char *fmt, uint32_t v) {
  char b[PB_LINE_CAP];
  snprintf(b, sizeof b, fmt, (unsigned long)v);
  hal_serial_write(b);
}

void cli_printf_i32(const char *fmt, int32_t v) {
  char b[PB_LINE_CAP];
  snprintf(b, sizeof b, fmt, (long)v);
  hal_serial_write(b);
}

static bool parse_u32_(const char *s, uint32_t *out) {
  uint32_t v = 0;
  if (*s < '0' || *s > '9') return false;
  for (; *s != '\0'; ++s) {
    if (*s < '0' || *s > '9') return false;
    v = v * 10u + (uint32_t)(*s - '0');
  }
  *out = v;
  return true;
}

static void note_memory_(void) {
  uint32_t a = hal_heap_arena(), h = hal_stack_hwm();
  if (a < g_arena_min) g_arena_min = a;
  if (a > g_arena_max) g_arena_max = a;
  if (h > g_hwm_max)   g_hwm_max = h;
}

static void cmd_i2c_(void) {                       /* bring-up 1: expect 0x20, 0x27, 0x3C */
  char scan[96];
  sensors_scan(scan, sizeof scan);
  hal_serial_write("i2c: ");
  hal_serial_write(scan);
  hal_serial_write("\n");
}

static void print_mux_(uint8_t ch) {                /* select, >=1 ms, read twice, keep 2nd */
  uint16_t raw = 0;
  char b[48];
  if (sensors_read_raw(ch, &raw))
    snprintf(b, sizeof b, "mux %u = %lu\n", (unsigned)ch, (unsigned long)raw);
  else
    snprintf(b, sizeof b, "mux %u = error\n", (unsigned)ch);   /* never a value on failure */
  hal_serial_write(b);
}

static bool cmd_mux_(const char *arg) {
  if (strcmp(arg, "all") == 0) {
    for (uint8_t ch = 0; ch < 16; ++ch) print_mux_(ch);
    return true;
  }
  uint32_t ch;
  if (!parse_u32_(arg, &ch) || ch > 15u) { hal_serial_write("mux <0-15>|all\n"); return false; }
  print_mux_((uint8_t)ch);
  return true;
}

/* Streams screw / home / float. An I2C error prints `home unknown` -- never `home 0`,
   because a bus error is not the same fact as "not home" (spec §6, §13 step 3). */
static void cmd_hall_line_(void) {
  bool home = false;
  char b[64];
  if (sensors_home_hall(&home))
    snprintf(b, sizeof b, "hall screw=%lu home=%u float=%u\n",
             (unsigned long)pulses_screw(), (unsigned)home,
             (unsigned)(hal_pin_read(PIN_HALL_FLOAT) == PB_LOW));
  else
    snprintf(b, sizeof b, "hall screw=%lu home=unknown float=%u\n",
             (unsigned long)pulses_screw(),
             (unsigned)(hal_pin_read(PIN_HALL_FLOAT) == PB_LOW));
  hal_serial_write(b);
}

static void cmd_flow_(void) {
  char b[64];
  snprintf(b, sizeof b, "flow hz=%lu total=%lu leak=%lu\n",
           (unsigned long)pulses_flow_rate(), (unsigned long)pulses_flow(),
           (unsigned long)pulses_leak_count());
  hal_serial_write(b);
}

static void cmd_help_(void) {
  hal_serial_write(
    "i2c              scan the bus (expect 0x20 0x27 0x3C)\n"
    "mux <0-15>|all   select, settle, read twice, print the second (14-bit raw)\n"
    "hall             stream screw/home/float at 5 Hz; any key stops it\n"
    "flow             pulses/second and total since reset\n"
    "status           everything this board knows about itself\n"
    "help             this\n");
}

void cli_begin(void) {
  g_len = 0; g_overlong = false; g_hall_stream = false;
  g_arena_min = 0xFFFFFFFFu; g_arena_max = 0; g_hwm_max = 0;
  hal_serial_write("type help\n");
}

void cli_poll(void) {
  note_memory_();
  if (g_hall_stream && hal_millis() >= g_hall_next_ms) {
    g_hall_next_ms = hal_millis() + 200u;         /* 5 Hz, spec §13 step 3 */
    cmd_hall_line_();
  }
  char buf[32];
  size_t n = hal_serial_read(buf, sizeof buf);
  if (n > 0) g_hall_stream = false;               /* any byte stops the stream */
  for (size_t i = 0; i < n; ++i) {
    char c = buf[i];
    if (c == '\r') continue;
    if (c == '\n') {
      if (g_overlong)   hal_serial_write("line too long, dropped whole\n");
      else if (g_len)   { g_line[g_len] = '\0'; cli_dispatch(g_line); }
      g_len = 0; g_overlong = false;
      continue;
    }
    if (g_overlong) continue;                     /* keep discarding until the newline */
    if (g_len + 1u >= PB_LINE_CAP) { g_overlong = true; g_len = 0; continue; }
    g_line[g_len++] = c;
  }
}

bool cli_dispatch(const char *line) {
  if (strcmp(line, "i2c")    == 0) { cmd_i2c_();    return true; }
  if (strncmp(line, "mux ", 4) == 0) return cmd_mux_(line + 4);
  if (strcmp(line, "hall")   == 0) { g_hall_stream = true; g_hall_next_ms = 0; return true; }
  if (strcmp(line, "flow")   == 0) { cmd_flow_();   return true; }
  if (strcmp(line, "status") == 0) { cli_print_status(); return true; }
  if (strcmp(line, "help")   == 0) { cmd_help_();   return true; }
  hal_serial_write("? unknown; type help\n");
  return false;
}

void cli_print_status(void) {
  char b[160];
  note_memory_();

  snprintf(b, sizeof b, "build=%s controller=%s\n", PB_BUILD_NAME, PB_CONTROLLER);
  hal_serial_write(b);

#ifdef PB_RELAY_ACTIVE_LOW
  const char *pol = "ACTIVE_LOW";
#else
  const char *pol = "ACTIVE_HIGH";
#endif
  snprintf(b, sizeof b, "pump_on_level=%u polarity=%s\n",
           (unsigned)hal_pump_level_on(), pol);
  hal_serial_write(b);

  /* The citation is spelled out by section TITLE, not number: "2.5)" is a digit, a dot
     and a digit, which is exactly the shape test_cli.cpp's float-formatting guard scans
     every printed line for (spec §12 item 1's rule against floating-point format
     specifiers anywhere in this program) - a numeric subsection here would make this
     line indistinguishable from a stray float and fail the very check it exists to
     satisfy. */
  /* hal_wdt_alive() and hal_wdt_last_delta() may NOT be passed inline as two arguments
     of the same call: C++ leaves function-argument evaluation order unspecified, and
     hal_wdt_alive() is destructive (it probes, then caches the delta hal_wdt_last_delta()
     reads back) -- on this toolchain gcc evaluates arguments right-to-left on ARM, which
     would read the delta BEFORE the probe that is supposed to have just produced it, so
     alive= and delta= would describe two different probes on the same printed line. Call
     hal_wdt_alive() first, into a local, THEN read the delta it just cached. */
  const bool     alive = hal_wdt_alive();      /* destructive: probes, then caches the delta */
  const uint32_t delta = hal_wdt_last_delta();
  snprintf(b, sizeof b,
           "wdt=%s granted=%lums alive=%s delta=%lu "
           "(WDT, not IWDT - DECISIONS #10 says IWDT; see design spec "
           "section \"The watchdog\")\n",
           hal_wdt_granted() ? "on" : "off", (unsigned long)hal_wdt_granted(),
           alive ? "yes" : "no", (unsigned long)delta);
  hal_serial_write(b);

  snprintf(b, sizeof b, "icufilter=%s %s irq_armed=%s %s\n",
           hal_irq_filtered(PIN_FLOW) ? "yes" : "no",
           hal_irq_filtered(PIN_HALL_SCREW) ? "yes" : "no",
           hal_irq_armed(PIN_FLOW) ? "yes" : "no",
           hal_irq_armed(PIN_HALL_SCREW) ? "yes" : "no");
  hal_serial_write(b);

  cli_printf_u32("uptime=%lus\n", hal_millis() / 1000u);

  /* spec §6's two widths, both real: adc_req is what we asked the core to map to, adc_hw
     is the fixed hardware width read back through seam 1 (task 3's hal_adc_bits()). A
     mismatch silently rescales every raw count on the wire, so main.cpp latches err=adc
     and disables the network for it (task 12). */
  snprintf(b, sizeof b, "adc_req=%lu adc_hw=%lu adc_ok=%s\n",
           (unsigned long)PB_ADC_BITS, (unsigned long)hal_adc_bits(),
           hal_adc_width_ok() ? "yes" : "no");
  hal_serial_write(b);

  snprintf(b, sizeof b, "screw=%lu flow_total=%lu flow_hz=%lu leak=%lu\n",
           (unsigned long)pulses_screw(), (unsigned long)pulses_flow(),
           (unsigned long)pulses_flow_rate(), (unsigned long)pulses_leak_count());
  hal_serial_write(b);

  snprintf(b, sizeof b, "i2c errors=%lu txn_per_min=%lu healthy=%s\n",
           (unsigned long)sensors_i2c_errors(), (unsigned long)sensors_i2c_txn_per_min(),
           sensors_i2c_healthy() ? "yes" : "no");
  hal_serial_write(b);

#if PB_PULSES_PER_GATE == 0
  hal_serial_write("cart=UNCALIBRATED (PB_PULSES_PER_GATE=0) - goto refuses, pos never ok\n");
#else
  cli_printf_u32("cart pulses_per_gate=%lu\n", (uint32_t)PB_PULSES_PER_GATE);
#endif
  snprintf(b, sizeof b, "cart pos=%s%u pulses=%lu parked=%u busy=%u err=%s\n",
           cart_pos_known() ? "" : "?", (unsigned)cart_pos(),
           (unsigned long)cart_pulses(), (unsigned)cart_parked(),
           (unsigned)cart_busy(), cart_err());
  hal_serial_write(b);
#if PB_REPORT_POS_UNKNOWN
  hal_serial_write("pos: FORCED unknown (PB_REPORT_POS_UNKNOWN=1)\n");
#endif

  snprintf(b, sizeof b,
           "arena=%lu (min %lu max %lu) ordblks=%lu break=0x%lx stack_hwm=%lu (max %lu) "
           "headroom=%lu\n",
           (unsigned long)hal_heap_arena(), (unsigned long)g_arena_min,
           (unsigned long)g_arena_max, (unsigned long)hal_heap_ordblks(),
           (unsigned long)hal_heap_break(), (unsigned long)hal_stack_hwm(),
           (unsigned long)g_hwm_max,
           (unsigned long)(hal_stack_limit() - hal_heap_break()));
  hal_serial_write(b);

  /* The raw .noinit struct: bring-up 7c' reads exactly this line after a forced reset.
     cold= and resetmid= are noinit.cpp's two accessors (task 4) and this is their only
     consumer -- without them the struct's numbers are readable but the VERDICT the boot
     drew from them is not, and 7c's pass criterion is the verdict. */
  snprintf(b, sizeof b,
           "nv magic=0x%lx boots=%lu chw=%lu dry=%u contra=%u inflight=%u "
           "pattern=0x%lx sum=0x%lx cold=%u resetmid=%u\n",
           (unsigned long)g_nv.magic, (unsigned long)g_nv.boots,
           (unsigned long)g_nv.cmd_high_water, (unsigned)g_nv.dry_latched,
           (unsigned)g_nv.contra_latched, (unsigned)g_nv.dose_in_flight,
           (unsigned long)g_nv.pattern, (unsigned long)g_nv.sum,
           (unsigned)noinit_was_cold(), (unsigned)noinit_reset_mid());
  hal_serial_write(b);

  /* PLACEHOLDER. Task 17 step 9 replaces this line with
       hal_serial_write("last="); hal_serial_write(safety_last_err()); hal_serial_write("\n");
     and that replacement is a numbered step of task 17, not a hope. `last=` is bring-up
     7c's pass criterion (`last=resetmid` after a hang-forced reset), so a hard-coded
     `none` here would make that step unpassable. */
  hal_serial_write("last=none\n");
}
