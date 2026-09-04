/* src/main.cpp -- DEVICE ONLY ([env:native] filters this file out).
   setup()'s ORDER is load-bearing: spec §2.1 pins the first statement, spec §5 pins the
   panels-before-sensors rule, spec §2.5 the three assertions and spec §12 the break check.
   No Arduino header here: spec §9 allows it only in hal_uno.cpp, lib/Network, lib/Screen.
   secrets.h is here for PB_CONTROLLER, which ui_fill_() copies into ui_state_t: the two
   device envs do not pass it in build_flags and secrets.h is its only other definition. */
#include "Screen.h"
#include "cart.h"     /* cart_begin/pos_known/pos/parked/busy */
#include "cli.h"
#include "config.h"
#include "exec.h"     /* exec_begin/exec_pending/exec_last_cmd_id/exec_last_cmd_text */
#include "hal.h"
#include "netfsm.h"   /* net_disable/net_begin/net_poll/net_last_status/net_next_s, and
                          (fix round, task 27) net_link/net_rssi/net_ip: netfsm.cpp owns the
                          AT budget, so it owns the seam, and this file may not call into it
                          directly any more -- see netfsm.h's own comment on why. */
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
  cart_begin();                                    /* beside pulses_begin(), in setup()'s first half */

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
     window. A failure here also latches dry, through safety_dry_set() (task 15) rather
     than a second, hand-rolled g_nv.dry_latched write -- one route to the latch, not two.
     hal_wdt_alive() is DESTRUCTIVE, not a getter (hal_uno.cpp:151-160): it feeds the dog,
     spins 40 ms deliberately UNFED, measures the delta, then feeds again. Call it EXACTLY
     ONCE per boot and reuse the result -- a second call is a second, independent 40 ms
     probe, not a re-read of this one's verdict, and could disagree with it near the
     threshold. wdt_alive is what the banner below prints as alive=. */
  const bool wdt_alive = hal_wdt_alive();
  if (!wdt_alive) {
    g_net_disabled = true; g_boot_err = "wdt";
    safety_dry_set(true);
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

#if PB_SIM
  hal_pin_mode(PIN_LED, PB_OUT);   /* hal_pin_mode()'s ONE caller in the tree: seam 1
                                      declares it and nothing else in the program
                                      configures an ordinary output pin */
  hal_serial_write("SIM *** D6 NOT DRIVEN. This binary has no pump driver and no network stack.\n");
#endif

  /* spec §2.5: a failed watchdog, ADC or heap assertion disables the network and says why in
     status. main.cpp holds the verdict; netfsm.cpp holds the flag, because [env:native]
     filters main.cpp out and no host test could otherwise reach it. */
  if (main_net_disabled()) net_disable(main_boot_err());
  net_begin();
  exec_begin();

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
  s->dry          = safety_dry();
  s->contra       = safety_contra();
  s->pos_known    = cart_pos_known();
  s->pos          = cart_pos();
  s->parked       = cart_parked();
  /* Fix round, task 27: this USED to call into the seam directly -- twice for the state
     alone, once each for signal strength and address, UNCONDITIONALLY every loop() pass, on
     top of whatever net_poll() had already spent that same pass. Against the fake that cost
     nothing; against the real driver it was up to ~5 AT commands in one pass against a
     5592 ms grant, a guaranteed watchdog reset in normal operation (task 27 fix-round
     report). netfsm.cpp now owns the seam and refreshes these on its own schedule, at most
     once per join for the ones that cost an AT; the three accessors below are cached and
     issue none, so nothing here can ever repeat that mistake. */
  s->link         = net_link();
  s->rssi         = net_rssi();
  strncpy(s->ip, net_ip(), sizeof s->ip - 1);
  s->http_status  = net_last_status();          /* a 400/401 loop is otherwise invisible
                                                   to anyone not on the serial port */
  s->next_s       = net_next_s();
  s->cmd_id       = exec_last_cmd_id();
  s->cmd_text     = exec_last_cmd_text();
#ifdef PB_SIM
  s->sim = true;
#endif

  /* spec §5's LCD state selection, most-urgent first. Row 1 is human prose and is tested
     (task 10) never to equal a wire err= token. NOTE what this does NOT decide: the
     renderer itself overrides row 1 with `HTTP <n>` whenever http_status is a non-200
     (task 10 step 4, §4.2), and overrides row 0 with the contra banner and then the sim
     banner (task 19 step 7). A 400/401 loop is therefore visible on the panel whichever
     branch below happened to run, which is the point - it must not depend on this
     function choosing the right prose. */
  static char detail[17];
  if (s->contra)        { s->lcd_state = "CONTRA LATCH"; s->lcd_detail = "float ok,no flow"; }
  else if (s->dry)      { s->lcd_state = "REFUSED";      s->lcd_detail = "dry latch set"; }
  else if (s->pump_on)  { snprintf(detail, sizeof detail, "PUMP o%u", (unsigned)s->pos);
                          s->lcd_state = detail;         s->lcd_detail = "dosing"; }
  else if (cart_busy()) { snprintf(detail, sizeof detail, "MOVE o%u", (unsigned)s->pos);
                          s->lcd_state = detail;         s->lcd_detail = "cart moving"; }
  else if (s->link != 2){ s->lcd_state = "WIFI?";        s->lcd_detail = "no link"; }
  else                  { s->lcd_state = "IDLE";
                          snprintf(detail, sizeof detail, "next %us", (unsigned)s->next_s);
                          s->lcd_detail = detail; }
}

#if PB_SIM
/* 200 on / 200 off / 200 on / 1400 off: a rhythm no bench binary produces, readable across
   a room. `PIN_LED` is include/pins.h's (task 2 step 5, the value 13) -- NOT
   `LED_BUILTIN_PIN`, which is in no header, and NOT `LED_BUILTIN`, which is an
   Arduino-header name main.cpp may not have (spec §9). */
static void sim_blink_(void) {
  const uint32_t p = hal_millis() % 2000u;
  hal_pin_write(PIN_LED, (p < 200u || (p >= 400u && p < 600u)) ? PB_HIGH : PB_LOW);
}
#endif

extern "C" void loop(void) {
  safety_tick();               /* pump idle re-asserted (D6's direction repaired), then fed */
  cli_poll();                  /* one whole line; may block, but only through safety_wait_ms() */
  net_poll(safety_dosing());   /* ONE bounded link/socket step. The flag is passed IN: netfsm.cpp
                                  may not include safety.h (§9), so the caller supplies it. */
  exec_pending();              /* at most one command; runs only when the socket is closed */
  pulses_leak_poll(safety_dosing());          /* the leak watch, EVERY pass: ch205's only
                                                 driver, and report_build() turns a non-zero
                                                 count into err=leak (task 22 step 12).
                                                 sensors_sweep() is NOT here - task 24's
                                                 NET_IDLE pass owns it, once per report cycle,
                                                 in the one pass with no AT command. */
  ui_fill_(&g_ui);
  ui_poll(&g_ui);              /* no-ops while dosing, while the cart moves, or after a modem pass */
#if PB_SIM
  sim_blink_();
#endif
}
