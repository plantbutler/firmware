/* Screen.cpp: drives the LCD or OLED through opaque library calls, each budgeted against the watchdog; device only, no host test covers it. */
#include "Screen.h"
#include "config.h"
#include "hal.h"
#include "pins.h"
#include "safety.h"

Screen::Screen(ScreenType type)
    : type(type), lcd_a(I2C_ADDR_LCD, 16, 2), lcd_b(I2C_ADDR_LCD_ALT, 16, 2), lcd(&lcd_a),
      present_(false) {}

bool Screen::probe() {
  if (type == ScreenType::Oled) {
    present_ = hal_i2c_probe(I2C_ADDR_OLED);
    return present_;
  }
  /* 0x27 first, then 0x3F: the backpack ships at either, and a panel probed at the wrong
     one would sit dark for the whole run with nothing saying why. */
  if (hal_i2c_probe(I2C_ADDR_LCD))          { lcd = &lcd_a; present_ = true; }
  else if (hal_i2c_probe(I2C_ADDR_LCD_ALT)) { lcd = &lcd_b; present_ = true; }
  else                                       present_ = false;
  return present_;
}

/* One opaque library call can, on a wedged bus, run unfed for its own worst case, because
   safety_tick() cannot run until it returns. So this feeds first, then judges the unit that
   just ran against PB_SCREEN_PAINT_BUDGET_MS: over budget is treated as a wedge and the
   panel goes dark for the rest of this boot -- only a fresh probe() brings it back. A wedge
   is detected after the fact, but cannot happen twice. */
bool Screen::paint_ok_(uint32_t unit_start_ms) {
  safety_tick();
  if (hal_millis() - unit_start_ms > PB_SCREEN_PAINT_BUDGET_MS) {
    present_ = false;
    return false;
  }
  return true;
}

/* begin() is not budget-guarded on either panel, and both chains exceed the 5592 ms
   watchdog grant on their own at Wire's fixed 1000 ms per transaction: the LCD's mandatory
   HD44780 4-bit init is 44 Wire transactions (~44 s worst case), the OLED's SSD1306 init
   sequence about 16 (~16 s, a lower bound). Nothing in either is optional and neither
   feeds. This is safe only because the watchdog is started after both panels' begin() --
   nothing in this file enforces that ordering; setup() must. Before the watchdog is
   started its refresh is a no-op, so the feeds inside clear() below feed nothing yet. */
void Screen::begin() {
  if (!present_) return;
  if (type == ScreenType::Oled) {
    Oled.initDisplay();
    Oled.setFlipMode(true);
    Oled.setFont(u8x8_font_chroma48medium8_r);
    /* Not Oled.begin(): that would run the library's whole-panel clear, 144 unguarded Wire
       transactions, for nothing the bounded clear() below does not already give. An MCU
       reset does not necessarily reset the OLED controller, so stale GDRAM may show for the
       length of this call -- cosmetic, the first paint overwrites it. setPowerSave(0) runs
       last, so the panel is not driven visibly until it is already blank. */
    clear();
    Oled.setPowerSave(0);
  } else {
    /* init() opens Wire and blocks a whole second: the HD44780's documented power-up wait,
       not bus trouble, so it is deliberately not run under PB_SCREEN_PAINT_BUDGET_MS --
       budgeting it would mark a healthy LCD not-present on every boot. That second is also
       why both panels come up before sensors_begin(). */
    lcd->init();
    lcd->backlight();
  }
}

/* Called on every UI transition, so it gets the per-unit budget guard. The OLED path
   paints 16 blank glyphs per row through row(), not the library's per-row or whole-panel
   clear: those look cheap, but under the fast I2C cad each SendData opens and closes its
   own transfer, so a per-row clear is 2 address transactions plus 16 data ones, 18 in all
   (18 s worst case), and the whole-panel clear runs that for each of 8 tile rows, 144. */
void Screen::clear() {
  if (!present_) return;
  if (type == ScreenType::Oled) {
    static const char blank[17] = "                ";     /* 16 spaces */
    uint8_t rows = Oled.getRows();
    for (uint8_t y = 0; y < rows && present_; ++y) row(y, blank);
    return;
  }
  uint32_t t0 = hal_millis();
  lcd->clear();
  paint_ok_(t0);
}

/* The finest unit either library's public surface allows is one opaque call, which can be
   several Wire transactions at a fixed 1000 ms each. LCD: one character or one setCursor()
   is 6 transactions, 6000 ms worst case -- already over the 5592 ms grant, and the calls
   underneath are private, so paint_ok_() can only stop the NEXT unit: the guarantee is "at
   most one such unit per boot", not "never exceeds the grant". OLED: one drawGlyph() per
   character, instead of the library's string draw that has no hook between glyphs, is 3
   transactions under the fast I2C cad this display class selects -- 3000 ms worst case, a
   real closed bound under the grant. */
void Screen::row(uint8_t r, const char *text) {
  if (!present_ || text == 0) return;
  if (type == ScreenType::Oled) {
    for (uint8_t i = 0; i < 16 && text[i] != '\0'; ++i) {
      uint32_t t0 = hal_millis();
      Oled.drawGlyph(i, r, (uint8_t)text[i]);
      if (!paint_ok_(t0)) return;
    }
    return;
  }
  uint32_t t0 = hal_millis();
  lcd->setCursor(0, r);
  if (!paint_ok_(t0)) return;
  for (uint8_t i = 0; i < 16 && text[i] != '\0'; ++i) {
    t0 = hal_millis();
    lcd->write((uint8_t)text[i]);
    if (!paint_ok_(t0)) return;
  }
}
