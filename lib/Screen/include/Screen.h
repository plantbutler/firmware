/* lib/Screen/include/Screen.h -- DEVICE ONLY. [env:native] lib_ignores this library.
   The only place in the tree that names LiquidCrystal_I2C or u8x8 (spec §1). */
#ifndef SCREEN_H
#define SCREEN_H

#include "Arduino_SensorKit.h"
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <stdint.h>

enum class ScreenType { Oled, Lcd };

class Screen {
private:
  ScreenType type;
  LiquidCrystal_I2C lcd;
  bool present_;

public:
  explicit Screen(ScreenType type);

  /* One bounded probe at boot. A panel that does not answer becomes a permanent no-op
     rather than wedging inside Oled.begin() or LiquidCrystal_I2C::init(). Spec §5. */
  bool probe();
  bool present() const { return present_; }

  void begin();                              /* no-op unless probe() said yes */
  void clear();
  void row(uint8_t r, const char *text);     /* one 16-column row; text is padded by ui.cpp */
};

#endif /* SCREEN_H */
