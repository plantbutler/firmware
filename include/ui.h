/* include/ui.h -- the two pure renderers and the coarsened painter (spec §5).
   ui.cpp includes neither safety.h nor anything that can assert D6: spec §9 greps for it. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  char        build[8];        /* "bench" | "bringup" | "sim" */
  char        controller[16];
  uint32_t    uptime_min;      /* MINUTES, not seconds -- spec §5's bus rule */
  bool        pos_known;
  uint8_t     pos;
  uint32_t    screw_pulses;
  bool        float_ok;
  bool        pump_on;         /* main.cpp fills this from safety_dosing() */
  bool        parked;
  uint32_t    flow_hz;
  uint32_t    flow_total;
  uint8_t     link;            /* 0 down, 1 joining, 2 up */
  int8_t      rssi;
  char        ip[16];
  uint16_t    http_status;
  uint16_t    next_s;          /* painted in 5 s steps */
  uint32_t    cmd_id;
  const char *cmd_text;        /* "ok 248ml" | "REF float" | 0 */
  bool        contra;
  bool        dry;
  bool        sim;
  const char *lcd_state;       /* row 0: IDLE | MOVE o3 | PUMP o3 | WIFI? | REFUSED | ... */
  const char *lcd_detail;      /* row 1: human prose, NEVER the err= wire token */
} ui_state_t;

void ui_render(const ui_state_t *s, char rows[8][17]);
void ui_render_lcd(const ui_state_t *s, char rows[2][17]);
void ui_poll(const ui_state_t *s);
/* net_poll() calls this DIRECTLY, in any pass that issued a modem command (task 24).
   netfsm.cpp may include ui.h -- §9's grep over netfsm.cpp is safety.h|dose_run|
   hal_pump_write, and ui.h is none of those. The alternative, main.cpp reading
   net_modem_ran_this_pass() and forwarding it, was rejected: it puts a rule that exists
   to bound ONE pass into a different translation unit from the pass that broke it. */
void ui_modem_ran(void);

#ifdef PB_NATIVE
uint16_t ui_paints_for_test(void);   /* the host counts row paints instead of driving a panel */
#endif
