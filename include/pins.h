/* include/pins.h -- the wiring contract as constants.
   Today: the polarity gate only. Task 2 adds the pin numbers and the PB_PUMP_OWNER gate. */
#pragma once

#if !defined(PB_RELAY_ACTIVE_LOW) && !defined(PB_RELAY_ACTIVE_HIGH)
#  error "Relay polarity is unknown until bring-up 4a. Define PB_RELAY_ACTIVE_LOW or \
PB_RELAY_ACTIVE_HIGH in platformio.ini build_flags after you have READ THE MODULE."
#endif
