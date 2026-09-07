/* main.cpp: setup() in a load-bearing order, the boot banner, ui_fill_() and loop().
   Device only, no board header; secrets.h is here for PB_CONTROLLER alone. */
#include "Screen.h"
#include "cart.h"
#include "cli.h"
#include "config.h"
#include "exec.h"
#include "hal.h"
#include "netfsm.h"
#include "noinit.h"
#include "pins.h"
#include "pulses.h"
#include "safety.h"
#include "secrets.h"
#include "sensors.h"
#include "sim_console.h"
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
static ui_state_t  g_ui;                 /* file-static: the main stack is 1024 B */

bool        main_net_disabled(void) { return g_net_disabled; }
const char *main_boot_err(void)     { return g_boot_err; }

/* setup()/loop() are declared extern "C" by the core (api/Common.h); a plain C++
   definition here would mangle and never link. */
extern "C" void setup(void) {
  hal_boot_pump_off();   /* FIRST. One PFS write: direction AND level, atomically */
  noinit_begin();        /* magic + checksum; a dose in flight across the reset latches dry */
  hal_begin();           /* console at 115200, then ADC width, pins, ISRs, Wire, servo,
                            stack paint */

  hal_i2c_probe(I2C_ADDR_OLED);
  hal_i2c_probe(I2C_ADDR_LCD);
  g_oled_screen.probe();
  g_oled_screen.begin();
  g_lcd_screen.probe();
  g_lcd_screen.begin();  /* BEFORE sensors_begin(): init_priv() re-opens the bus */
  g_oled_screen.clear();  /* a panel that did not answer probe() is a no-op here */
  g_lcd_screen.clear();

  sensors_begin();
  pulses_begin();
  cart_begin();

  /* noinit_begin() has already latched the verdict and dry; this raises the token and
     CLEARS the flag, once per boot, because a dose_in_flight nobody clears re-latches dry
     on every warm boot forever. */
  if (noinit_reset_mid()) {
    g_boot_err = "resetmid";              /* the network stays ENABLED: this is a report,
                                             not a reason to stop reporting */
    g_nv.dose_in_flight = false;
    noinit_commit();
  }

  /* AFTER both screens' begin(): their init chains (44 Wire transactions for the LCD, 16
     for the OLED) are up to ~60 s of unfed bus traffic against a 5592 ms grant, safe ONLY
     while the dog is not yet armed. Moved earlier, this reboots the board during boot. */
  if (!hal_wdt_start()) { g_net_disabled = true; g_boot_err = "wdt"; }

  /* The library's timeout getter returns 0 under the wdt_cfg_t overload even on a running
     dog, so hal_wdt_granted() computes the grant itself. (Do not name that getter here:
     the build check greps it to zero, comments included.) */
  if (hal_wdt_granted() != PB_WDT_GRANTED_MS) { g_net_disabled = true; g_boot_err = "wdt"; }

  /* the worst net step is 2 AT commands = 2400 ms; that plus slack must fit the grant. */
  if (hal_wdt_granted() < 2u * PB_NET_STEP_MS + PB_NET_SLACK_MS) {
    g_net_disabled = true; g_boot_err = "wdt";
  }

  /* Liveness, not a constant: the counter must DECREASE across a 40 ms UNFED window. A
     failure also latches dry, through safety_dry_set() -- one route to the latch, not two.
     hal_wdt_alive() is DESTRUCTIVE, not a getter: it feeds, spins 40 ms unfed, measures,
     feeds again. Call it EXACTLY ONCE per boot and reuse the result; a second call is a
     second probe that could disagree near the threshold. The banner prints it as alive=. */
  const bool wdt_alive = hal_wdt_alive();
  if (!wdt_alive) {
    g_net_disabled = true; g_boot_err = "wdt";
    safety_dry_set(true);
  }

  /* The hardware ADC width is fixed and analogReadResolution() only stores the REQUESTED
     one, so a core bump that changed the width would silently rescale every raw count on
     the wire. The only producer of err=adc. */
  if (!hal_adc_width_ok()) { g_net_disabled = true; g_boot_err = "adc"; }

  /* _sbrk is the unchecked libnosys version and nothing references __HeapLimit, so the
     break against the stack is the ONLY heap bound there is. The network stack is the
     largest allocator, so past this a water command is what the corruption reaches. */
  if (hal_heap_break() >= hal_stack_limit() - PB_STACK_MARGIN) {
    g_net_disabled = true; g_boot_err = "heap";
  }

  /* The boot banner, read BEFORE 12 V goes onto COM. Printed BEFORE cli_begin(), which
     writes "type help" as its own first action. */
  {
    char b[160];
    snprintf(b, sizeof b,
             "\nPB bench sketch build=%s dry=%u contra=%u pump_on_level=%u "
             "wdt=%s granted=%lums alive=%s adc=%lu/%lu oled=%u lcd=%u net=%s last=%s\n",
             PB_BUILD_NAME, (unsigned)g_nv.dry_latched, (unsigned)g_nv.contra_latched,
             (unsigned)hal_pump_level_on(), hal_wdt_granted() ? "on" : "off",
             (unsigned long)hal_wdt_granted(), wdt_alive ? "yes" : "no",
             (unsigned long)PB_ADC_BITS, (unsigned long)hal_adc_bits(),
             /* a panel that did not answer probe() is named here rather than discovered
                later as a screen that never updates */
             (unsigned)g_oled_screen.present(), (unsigned)g_lcd_screen.present(),
             g_net_disabled ? "DISABLED" : "enabled", g_boot_err);
    hal_serial_write(b);
  }

#if PB_SIM
  /* the LED pin is configured by sim_console_begin(), inside hal_begin() above: in this
     build hal_pin_mode() writes the fake rig's event log, not a pin */
  hal_serial_write("SIM *** D6 NOT DRIVEN. This binary has no pump driver and no network stack.\n");
#endif

  /* A failed watchdog, ADC or heap assertion disables the network and says why in status.
     net_boot() runs net_begin() FIRST and latches the verdict SECOND, and that order is
     the whole mechanism: net_begin() clears the latch unconditionally (one left standing
     across a restart would make net_poll() a silent no-op forever), so a verdict latched
     before it is thrown away one statement later -- the banner still prints net=DISABLED,
     because it reads this file's own flag, and the board reports rescaled counts for 48
     hours. The order lives in netfsm.cpp because no host test can reach this file. */
  net_boot(main_net_disabled() ? main_boot_err() : NULL);
  exec_begin();

  cli_begin();
}

static void ui_fill_(ui_state_t *s) {
  memset(s, 0, sizeof *s);
  strncpy(s->build, PB_BUILD_NAME, sizeof s->build - 1);
  /* the field stays char[]: the screens draw text and 0..255 is three characters */
  snprintf(s->controller, sizeof s->controller, "%u", (unsigned)PB_CONTROLLER);
  s->uptime_min   = hal_millis() / 60000u;      /* minutes: rarer repaints, see the screen bus budget in config.h */
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
  /* Cached accessors that issue no AT command: netfsm.cpp owns the seam and refreshes
     these on its own schedule. Asking the driver here every pass would cost up to ~5 AT
     commands against a 5592 ms grant, a guaranteed watchdog reset. */
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

  /* LCD state, most urgent first. Row 1 is human prose, never a wire err= token. The
     renderer overrides row 1 with `HTTP <n>` on any non-200 and row 0 with the contra and
     sim banners, so neither depends on this function choosing the right prose. */
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

extern "C" void loop(void) {
  safety_tick();               /* pump idle re-asserted (D6's direction repaired), then fed */
  cli_poll();                  /* one whole line; may block, but only through safety_wait_ms() */
  net_poll(safety_dosing());   /* ONE bounded link/socket step. The flag is passed IN: netfsm.cpp
                                  may not include safety.h, so the caller supplies it. */
  exec_pending();              /* at most one command; runs only when the socket is closed */
  pulses_leak_poll(safety_dosing());          /* the leak watch, EVERY pass: ch205's only
                                                 driver. sensors_sweep() is NOT here: the
                                                 NET_IDLE pass owns it, once per report
                                                 cycle, the one pass with no AT command. */
  ui_fill_(&g_ui);
  ui_poll(&g_ui);              /* no-ops while dosing, while the cart moves, or after a modem pass */
#if PB_SIM
  sim_console_blink_tick();    /* real pin and real clock, in sim_console.cpp: in this
                                   build hal_pin_write()/hal_millis() drive neither */
#endif
}
