/* Screen.h: the LCD/OLED panel driver; device only ([env:native] ignores this library) and the only place in the tree that names LiquidCrystal_I2C or u8x8. */
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
  /* The backpack answers at 0x27 or 0x3F depending on its solder jumpers, and the library
     takes the address in its constructor: two driver objects, and probe() points `lcd` at
     the one that answered. Neither touches the bus until begin(). */
  LiquidCrystal_I2C lcd_a, lcd_b;
  LiquidCrystal_I2C *lcd;
  bool present_;

  /* Feeds the watchdog, then judges the unit that started at unit_start_ms. One LCD
     command()/write()/setCursor() (6 Wire transactions) or one OLED drawGlyph() (3) is a
     unit -- the finest granularity either library's public surface exposes. A unit over
     PB_SCREEN_PAINT_BUDGET_MS marks the panel permanently not-present and the caller must
     stop: the watchdog is a safety device, the screen a debugging aid, and when the two
     conflict the screen loses. config.h derives the transaction counts. */
  bool paint_ok_(uint32_t unit_start_ms);

public:
  explicit Screen(ScreenType type);

  /* One bounded probe at boot: a panel that does not answer becomes a permanent no-op
     rather than wedging inside the library's own init. */
  bool probe();
  bool present() const { return present_; }

  void begin();                              /* no-op unless probe() said yes */
  void clear();
  void row(uint8_t r, const char *text);     /* one 16-column row; text is padded by ui.cpp */
};

#endif /* SCREEN_H */
