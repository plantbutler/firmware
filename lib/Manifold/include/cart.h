/* cart.h: the cart, positioned by counted screw pulses plus the home hall; names no network header and no panel pointer -- ui.cpp reads the cart, the cart writes to nothing. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

bool        cart_begin(void);          /* servo stopped, position unknown. No movement. */
bool        cart_home(void);           /* drive toward home until the hall asserts; bounded */
bool        cart_goto(uint8_t outlet); /* 1..PB_OUTLETS, by pulses; bounded, stall-aborted */
bool        cart_pos_known(void);
uint8_t     cart_pos(void);            /* 0 == home/parked; 1..PB_OUTLETS == over that gate */
bool        cart_busy(void);           /* true only inside a move; net_poll/ui_poll read it */
bool        cart_parked(void);         /* at home, i.e. over no gate -- rides out as ch208 */
bool        cart_bus_check(void);      /* one live expander read; false on a bus error */
uint32_t    cart_pulses(void);         /* screw pulses since the last successful home */
const char *cart_err(void);            /* "none" | "stall" | "timeout" | "i2c" | "range" | "uncal" */
void        cart_jog(int16_t us, uint32_t ms);   /* bounded console jog, <= PB_SERVO_CAP_MS */
