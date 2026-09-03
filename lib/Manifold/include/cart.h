/* lib/Manifold/include/cart.h -- the cart, positioned by COUNTED SCREW PULSES plus the
   home hall. Every blocking wait in the old Manifold became a (target pulses, deadline,
   stall window) triple. This file includes no network header of any kind -- not the seam,
   not the library wrapper, not the modem driver -- and no panel pointer either: ui.cpp
   reads the cart; the cart writes to nothing. (task 14 step 11 greps this directory for
   all four of those names and expects zero, so do not spell one here.) */
#pragma once
#include <stdbool.h>
#include <stdint.h>

bool        cart_begin(void);          /* servo stopped, position UNKNOWN. No movement. */
bool        cart_home(void);           /* drive toward home until the hall asserts; bounded */
bool        cart_goto(uint8_t outlet); /* 1..PB_OUTLETS, by pulses; bounded, stall-aborted */
bool        cart_pos_known(void);
uint8_t     cart_pos(void);            /* 0 == home/parked; 1..PB_OUTLETS == over that gate */
bool        cart_busy(void);           /* true only inside a move; net_poll/ui_poll read it */
bool        cart_parked(void);         /* at home, i.e. over NO gate -- rides out as ch208 */
bool        cart_bus_check(void);      /* ONE live expander read; false on a bus error */
uint32_t    cart_pulses(void);         /* screw pulses since the last successful home */
const char *cart_err(void);            /* "none" | "stall" | "timeout" | "i2c" | "range" | "uncal" */
void        cart_jog(int16_t us, uint32_t ms);   /* bounded console jog, <= PB_SERVO_CAP_MS */
