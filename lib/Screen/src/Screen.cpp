/* lib/Screen/src/Screen.cpp -- DEVICE ONLY. No host test covers this file (spec §9). */
#include "Screen.h"
#include "config.h"
#include "hal.h"
#include "pins.h"
#include "safety.h"

Screen::Screen(ScreenType type) : type(type), lcd(I2C_ADDR_LCD, 16, 2), present_(false) {}

bool Screen::probe() {
  present_ = hal_i2c_probe(type == ScreenType::Oled ? I2C_ADDR_OLED : I2C_ADDR_LCD);
  return present_;
}

void Screen::begin() {
  if (!present_) return;
  if (type == ScreenType::Oled) {
    Oled.begin();
    Oled.setFlipMode(true);
    Oled.setFont(u8x8_font_chroma48medium8_r);
  } else {
    /* init() opens Wire and blocks for a whole second inside init_priv(), which is why
       spec §5 fixes the order: both panels come up BEFORE sensors_begin(). */
    lcd.init();
    lcd.backlight();
  }
}

void Screen::clear() {
  if (!present_) return;
  if (type == ScreenType::Oled) Oled.clearDisplay();
  else lcd.clear();
}

/* Spec §5: 16 characters x 6 Wire transactions, plus a setCursor that is itself 6 more.
   On a wedged bus, at the core's fixed 1000 ms transfer timeout, that is up to 102
   seconds. The watchdog bites at 5592 ms. So: one character, one safety_tick(), and
   never the library's own row printer, which has no hook between characters. */
void Screen::row(uint8_t r, const char *text) {
  if (!present_ || text == 0) return;
  if (type == ScreenType::Oled) {
    Oled.drawString(0, r, text);       /* u8x8 pushes 8-byte tiles: ~17 transactions */
    return;
  }
  lcd.setCursor(0, r);
  for (uint8_t i = 0; i < 16 && text[i] != '\0'; ++i) {
    lcd.write((uint8_t)text[i]);
    safety_tick();
  }
}
