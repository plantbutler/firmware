/* src/main.cpp -- DEVICE ONLY ([env:native] filters this file out).
   setup()'s ORDER is load-bearing: spec §2.1 pins the first statement, spec §5 pins the
   panels-before-sensors rule, spec §2.5 the three assertions and spec §12 the break check.
   No Arduino header here: spec §9 allows it only in hal_uno.cpp, lib/Network, lib/Screen.
   secrets.h is here for PB_CONTROLLER, which ui_fill_() copies into ui_state_t: the two
   device envs do not pass it in build_flags and secrets.h is its only other definition. */
#include "Screen.h"
#include "cli.h"
#include "config.h"
#include "hal.h"
#include "noinit.h"
#include "pins.h"
#include "pulses.h"
#include "safety.h"
#include "secrets.h"
#include "sensors.h"
#include "ui.h"
#include <stdio.h>
#include <string.h>

#if defined(PB_SIM)
#  define PB_BUILD_NAME "sim"
#elif defined(PB_BRINGUP)
#  define PB_BUILD_NAME "bringup"
#else
#  define PB_BUILD_NAME "bench"
#endif

Screen g_oled_screen(ScreenType::Oled);
Screen g_lcd_screen(ScreenType::Lcd);

static bool        g_net_disabled;
static const char *g_boot_err = "none";
static ui_state_t  g_ui;                 /* file-static: the main stack is 1024 B (spec §12) */

bool        main_net_disabled(void) { return g_net_disabled; }
const char *main_boot_err(void)     { return g_boot_err; }

/* setup()/loop() are declared inside an extern "C" block by arduino_main() (api/Common.h);
   a plain C++ definition here would mangle and never link (task 1 found this the hard way). */
extern "C" void setup(void) {
  hal_boot_pump_off();   /* FIRST. One PFS write: direction AND level, atomically (spec §2.1) */
  noinit_begin();        /* magic + checksum; a dose in flight across the reset latches dry */
  hal_begin();           /* opens the console at 115200, then ADC width, pins, ISRs, Wire,
                            servo, stack paint */

  hal_i2c_probe(I2C_ADDR_OLED);
  hal_i2c_probe(I2C_ADDR_LCD);
  g_oled_screen.probe();
  g_oled_screen.begin();
  g_lcd_screen.probe();
  g_lcd_screen.begin();  /* BEFORE sensors_begin(): init_priv() re-opens the bus (spec §5) */
  g_oled_screen.clear();  /* a panel that did not answer probe() is a no-op here (task 9) */
  g_lcd_screen.clear();

  sensors_begin();
  pulses_begin();

  /* spec §2.3: a reset taken with the pump asserted is the single loudest thing this rig
     can discover about itself. noinit_begin() has already latched the verdict and the dry
     latch; this is where the token is raised and the flag CLEARED — exactly once per boot,
     and here rather than anywhere else, because a dose_in_flight nobody clears re-latches
     dry on every subsequent warm boot forever. Bring-up 7c's pass criterion is that
     `status` then says dry=1 and last=resetmid. */
  if (noinit_reset_mid()) {
    g_boot_err = "resetmid";              /* the network stays ENABLED: this is a report,
                                             not a reason to stop reporting */
    g_nv.dose_in_flight = false;
    noinit_commit();
  }

  /* Must run AFTER both screens' begin() above: their mandatory init chains (44 Wire
     transactions for the LCD, 16 for the OLED — lib/Screen/src/Screen.cpp's Screen::begin()
     comment traces both) are ~60 s of unfed bus traffic in the worst case, against a
     5592 ms grant. That is safe ONLY while the watchdog is not yet armed. Moving this call
     earlier is an entirely reasonable-looking edit that would reboot the board during boot. */
  if (!hal_wdt_start()) { g_net_disabled = true; g_boot_err = "wdt"; }

  /* spec §2.5: the library's timeout getter returns 0 under the wdt_cfg_t overload even
     on a running dog, so hal_wdt_granted() computes the grant itself and this asserts it
     against the constant. (Do not name that getter here: make check greps it to zero
     across the tree, comments included.) */
  if (hal_wdt_granted() != PB_WDT_GRANTED_MS) { g_net_disabled = true; g_boot_err = "wdt"; }

  /* spec §3: the worst net step is 2 AT commands = 2400 ms; 2400 + slack must fit. */
  if (hal_wdt_granted() < 2u * PB_NET_STEP_MS + PB_NET_SLACK_MS) {
    g_net_disabled = true; g_boot_err = "wdt";
  }

  /* spec §2.5: liveness, not a constant. The counter must DECREASE across a 40 ms UNFED
     window. A failure here also latches dry. (Task 15 replaces the two lines below with
     safety_dry_set(true).)
     hal_wdt_alive() is DESTRUCTIVE, not a getter (hal_uno.cpp:151-160): it feeds the dog,
     spins 40 ms deliberately UNFED, measures the delta, then feeds again. Call it EXACTLY
     ONCE per boot and reuse the result -- a second call is a second, independent 40 ms
     probe, not a re-read of this one's verdict, and could disagree with it near the
     threshold. wdt_alive is what the banner below prints as alive=. */
  const bool wdt_alive = hal_wdt_alive();
  if (!wdt_alive) {
    g_net_disabled = true; g_boot_err = "wdt";
    g_nv.dry_latched = true;
    noinit_commit();
  }

  /* spec §7: the hardware ADC width is fixed and analogReadResolution() only stores the
     REQUESTED one, so a core bump that changed the fixed width would silently rescale
     every raw count on the wire with no error anywhere. hal_begin() computed the answer;
     this is the only producer of err=adc, a token §4.1's fixed enum already carries. */
  if (!hal_adc_width_ok()) { g_net_disabled = true; g_boot_err = "adc"; }

  /* spec §12 item 0: _sbrk is the unchecked libnosys version and __HeapLimit is referenced
     by nothing in the image, so the break against the stack is the ONLY heap bound that
     exists. The network stack is the largest allocator in the program, so crossing this
     is the one case where continuing is how the corruption reaches a water command. */
  if (hal_heap_break() >= hal_stack_limit() - PB_STACK_MARGIN) {
    g_net_disabled = true; g_boot_err = "heap";
  }

  /* Bring-up step 0's pass criterion, read BEFORE 12 V goes onto COM (spec §13). Printed
     BEFORE cli_begin() below, so the console shows the banner first and "type help"
     second -- the order spec §13's own transcript documents. cli_begin() writes "type
     help" as its own first action, so the reverse order was a one-line ordering bug,
     not a missing feature. */
  {
    char b[160];
    snprintf(b, sizeof b,
             "\nPB bench sketch build=%s dry=%u contra=%u pump_on_level=%u "
             "wdt=%s granted=%lums alive=%s adc=%lu/%lu oled=%u lcd=%u net=%s last=%s\n",
             PB_BUILD_NAME, (unsigned)g_nv.dry_latched, (unsigned)g_nv.contra_latched,
             (unsigned)hal_pump_level_on(), hal_wdt_granted() ? "on" : "off",
             (unsigned long)hal_wdt_granted(), wdt_alive ? "yes" : "no",
             (unsigned long)PB_ADC_BITS, (unsigned long)hal_adc_bits(),
             /* Screen::present()'s one consumer. Bring-up step 0 reads the banner before
                step 1 scans the bus, so a panel that did not answer probe() is named here
                rather than discovered later as a screen that simply never updates. */
             (unsigned)g_oled_screen.present(), (unsigned)g_lcd_screen.present(),
             g_net_disabled ? "DISABLED" : "enabled", g_boot_err);
    hal_serial_write(b);
  }

  cli_begin();
}

static void ui_fill_(ui_state_t *s) {
  memset(s, 0, sizeof *s);
  strncpy(s->build, PB_BUILD_NAME, sizeof s->build - 1);
  strncpy(s->controller, PB_CONTROLLER, sizeof s->controller - 1);
  s->uptime_min   = hal_millis() / 60000u;      /* MINUTES: spec §5's bus rule */
  s->pump_on      = safety_dosing();            /* ui.cpp may not include safety.h itself */
  s->float_ok     = (hal_pin_read(PIN_HALL_FLOAT) == PB_LOW);
  s->screw_pulses = pulses_screw();
  s->flow_hz      = pulses_flow_rate();
  s->flow_total   = pulses_flow();
  s->dry          = g_nv.dry_latched;
  s->contra       = g_nv.contra_latched;
#ifdef PB_SIM
  s->sim = true;
#endif
  s->lcd_state  = s->pump_on ? "PUMP" : "IDLE";
  s->lcd_detail = s->float_ok ? "float ok" : "float NOT OK";
  /* TASK 26 OWNS THE REST OF THIS FUNCTION and rewrites it whole: pos/pos_known/parked
     from the cart, link/rssi/ip from seam 2, http_status/next_s from the report FSM,
     cmd_id/cmd_text from exec.cpp, and spec §5's real lcd_state selection. Until then
     rows 1 and 4-7 of the OLED read pos ?, wifi -- 0 dBm, no link, next 0s and cmd -,
     which is correct for a tree with no cart and no network in it. */
}

extern "C" void loop(void) {
  safety_tick();     /* pump idle re-asserted, D6's direction repaired, then the dog fed */
  cli_poll();        /* one whole line; may block, but only through safety_wait_ms() */
  /* net_poll(safety_dosing()); <- task 26 adds this line (spec §3) */
  /* exec_pending();            <- task 26 adds this line (spec §3) */
  pulses_leak_poll(safety_dosing());   /* the leak watch, EVERY pass: ch205's only driver.
                                          report_build() (task 22) is what turns a non-zero
                                          count into err=leak; nothing here does. */
  ui_fill_(&g_ui);
  ui_poll(&g_ui);    /* no-ops while the pump is asserted, the cart moves, or a modem ran */
}
