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

  /* Feeds the watchdog, then judges the unit that just returned (it started at
     unit_start_ms). Every LCD command()/write()/setCursor() (6 Wire transactions) and
     every OLED drawGlyph() (3 Wire transactions) is ONE such unit -- the finest
     granularity either library's PUBLIC surface exposes; neither can be split further
     without reaching into private internals ("forking" them). If the unit ran longer
     than PB_SCREEN_PAINT_BUDGET_MS, the panel is marked permanently not-present and the
     caller must stop: the watchdog is a safety device, the screen is a debugging aid,
     and when a paint and the grant conflict the screen loses (spec §5). config.h has
     the full transaction-count derivation for both panels. */
  bool paint_ok_(uint32_t unit_start_ms);

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
