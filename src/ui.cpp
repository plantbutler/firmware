/* ui.cpp: two PURE renderers and the change-only painter for the OLED and the LCD.
   Includes no safety header: the dosing guard reads s->pump_on, which main.cpp fills. */
#include "ui.h"
#include "cart.h"
#include "config.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static void row_(char *dst, const char *text) {
  size_t n = 0;
  while (n < 16 && text[n] != '\0') { dst[n] = text[n]; ++n; }
  while (n < 16) dst[n++] = ' ';
  dst[16] = '\0';
}

static void rowf_(char *dst, const char *fmt, ...) {
  char tmp[32];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(tmp, sizeof tmp, fmt, ap);
  va_end(ap);
  row_(dst, tmp);
}

/* The OLED, at the panel's real 16 columns. */
void ui_render(const ui_state_t *s, char rows[8][17]) {
  rowf_(rows[0], "PB %s  %luh%02lum", s->controller,
        (unsigned long)(s->uptime_min / 60u), (unsigned long)(s->uptime_min % 60u));
  if (s->pos_known) rowf_(rows[1], "pos %u  p %lu", (unsigned)s->pos, (unsigned long)s->screw_pulses);
  else              rowf_(rows[1], "pos ?  p %lu", (unsigned long)s->screw_pulses);
  rowf_(rows[2], "flt %s  pump %s", s->float_ok ? "OK" : "NO", s->pump_on ? "ON " : "off");
  rowf_(rows[3], "flow %lu/s t %lu", (unsigned long)s->flow_hz, (unsigned long)s->flow_total);
  rowf_(rows[4], "wifi %s  %d dBm",
        s->link == 2 ? "UP" : (s->link == 1 ? "JN" : "--"), (int)s->rssi);
  row_(rows[5], s->link == 2 ? s->ip : (s->link == 1 ? "joining..." : "no link"));
  rowf_(rows[6], "rpt %u next %us", (unsigned)s->http_status,
        (unsigned)((s->next_s / 5u) * 5u));                       /* 5 s steps: rarer repaints, see the screen bus budget in config.h */
  if (s->cmd_id == 0) row_(rows[7], "cmd -");
  else rowf_(rows[7], "cmd %lu %s", (unsigned long)s->cmd_id,
             s->cmd_text ? s->cmd_text : "");
  if (s->sim) row_(rows[0], "*** SIM NO D6 **");  /* EXACTLY 16 chars; the host suite asserts it */
}

void ui_render_lcd(const ui_state_t *s, char rows[2][17]) {
  if (s->contra) {                    /* OVERRIDES the caller's choice: a latch may not be
                                         hidden behind whatever state the fill function
                                         picked this pass, nor behind an HTTP status */
    row_(rows[0], "CONTRA LATCH");
    row_(rows[1], "float ok,no flow");   /* 16 columns */
  } else {
    row_(rows[0], s->lcd_state ? s->lcd_state : "IDLE");
    /* The last HTTP status goes on the LCD: a 400/401 loop is otherwise invisible off the
       serial port. The RENDERER decides, not ui_fill_(), which picks lcd_detail for a
       dozen other reasons. 0 means "no exchange yet". */
    if (s->http_status != 0u && s->http_status != 200u)
      rowf_(rows[1], "HTTP %u", (unsigned)s->http_status);
    else
      row_(rows[1], s->lcd_detail ? s->lcd_detail : "");
  }
  /* LAST, and outside the branch: SIM outranks CONTRA on row 0. Exactly sixteen
     characters, asserted in both renderers. */
  if (s->sim) row_(rows[0], "*** SIM NO D6 **");
}

/* --- the painter. Both panels freeze for the length of a dose, and the LCD says so. --- */
static char     g_oled_shadow[8][17];
static char     g_lcd_shadow[2][17];
static bool     g_shadow_valid;
static bool     g_modem_ran;

void ui_modem_ran(void) { g_modem_ran = true; }

#ifdef PB_NATIVE
static uint16_t g_paints;
uint16_t ui_paints_for_test(void) { return g_paints; }
static void paint_oled_(uint8_t r, const char *t) { (void)r; (void)t; ++g_paints; }
static void paint_lcd_(uint8_t r, const char *t)  { (void)r; (void)t; ++g_paints; }
#else
#include "Screen.h"
extern Screen g_oled_screen;
extern Screen g_lcd_screen;
static void paint_oled_(uint8_t r, const char *t) { g_oled_screen.row(r, t); }
static void paint_lcd_(uint8_t r, const char *t)  { g_lcd_screen.row(r, t); }
#endif

void ui_poll(const ui_state_t *s) {
  bool modem = g_modem_ran;
  g_modem_ran = false;
  if (s->pump_on || cart_busy() || modem) return;

  char oled[8][17], lcd[2][17];
  ui_render(s, oled);
  ui_render_lcd(s, lcd);

  for (uint8_t r = 0; r < 8; ++r)
    if (!g_shadow_valid || strcmp(oled[r], g_oled_shadow[r]) != 0) {
      paint_oled_(r, oled[r]);
      strcpy(g_oled_shadow[r], oled[r]);
    }
  for (uint8_t r = 0; r < 2; ++r)
    if (!g_shadow_valid || strcmp(lcd[r], g_lcd_shadow[r]) != 0) {
      paint_lcd_(r, lcd[r]);
      strcpy(g_lcd_shadow[r], lcd[r]);
    }
  g_shadow_valid = true;
}
