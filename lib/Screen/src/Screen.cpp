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

/* Fix round 1 (Critical): a single opaque library call -- one LCD command()/write()/
   setCursor() (6 Wire transactions) or one OLED drawGlyph() (3 Wire transactions, traced
   below) -- can, on a wedged bus, run unfed for as long as that call's own worst case,
   because safety_tick() cannot run until the call returns control to us. This function
   is what turns "detected after the fact" into "cannot happen twice": it feeds first (so
   the dog is fed as promptly as the library's opacity allows), then judges the unit that
   just ran against PB_SCREEN_PAINT_BUDGET_MS. A unit inside budget is normal bus
   traffic. A unit over budget is treated as a wedge, and the panel goes dark for the
   rest of this boot -- present_ only ever becomes true again via a fresh probe(). */
bool Screen::paint_ok_(uint32_t unit_start_ms) {
  safety_tick();
  if (hal_millis() - unit_start_ms > PB_SCREEN_PAINT_BUDGET_MS) {
    present_ = false;
    return false;
  }
  return true;
}

void Screen::begin() {
  if (!present_) return;
  if (type == ScreenType::Oled) {
    Oled.begin();
    Oled.setFlipMode(true);
    Oled.setFont(u8x8_font_chroma48medium8_r);
  } else {
    /* init() opens Wire and blocks for a whole second inside init_priv(), which is why
       spec §5 fixes the order: both panels come up BEFORE sensors_begin(). That second
       is a documented, expected HD44780 power-up wait, not bus trouble, so this call is
       deliberately NOT run under PB_SCREEN_PAINT_BUDGET_MS -- budgeting it would mark a
       healthy LCD not-present on every single boot. It is on spec §9's not-tested-on-
       the-host list, same as today. */
    lcd.init();
    lcd.backlight();
  }
}

/* clear() has the same shape as row() -- opaque bus calls, no built-in feed -- and,
   unlike begin(), is called repeatedly across the run (task 10: on every UI state
   transition), so it gets the same per-unit budget guard. The OLED path clears by
   painting 16 blank glyphs per row through row() itself, reusing its already-bounded
   per-glyph unit (see row()'s comment) -- NOT Oled.clearDisplay() or Oled.clearLine().
   Both of those are far more expensive than they look: u8x8's SSD1306 DRAW_TILE handler
   (u8x8_d_ssd1306_128x64_noname.c's U8X8_MSG_DISPLAY_DRAW_TILE case) issues 2 Wire
   transactions for the column/page address (two SendCmd calls, the second of which
   forces the fast I2C cad, u8x8_cad_ssd13xx_fast_i2c, to close and reopen the transfer;
   two more SendArg bytes ride along on the open one) plus ONE MORE Wire transaction per
   SendData call. clearLine() asks for 16 tile-columns in one DRAW_TILE call, so its
   inner `do { SendData(...); } while(arg_int>0)` loop calls SendData 16 times -- one
   fresh Wire transaction each, because u8x8_cad_ssd13xx_fast_i2c's SEND_DATA case always
   routes through u8x8_i2c_data_transfer(), which opens and closes its own transfer
   every time regardless of the "fast" merge. That is 2 + 16 = 18 Wire transactions for
   ONE clearLine() call -- an 18000 ms worst case, and clearDisplay() runs that 8 times
   (once per tile row, u8x8_display.c's u8x8_ClearDisplayWithTile()) for 144 total. */
void Screen::clear() {
  if (!present_) return;
  if (type == ScreenType::Oled) {
    static const char blank[17] = "                ";     /* 16 spaces */
    uint8_t rows = Oled.getRows();
    for (uint8_t y = 0; y < rows && present_; ++y) row(y, blank);
    return;
  }
  uint32_t t0 = hal_millis();
  lcd.clear();
  paint_ok_(t0);
}

/* Spec §5, corrected in fix round 1: the finest granularity either library's PUBLIC
   surface allows is not "one Wire transaction" but "one opaque call", and that call can
   itself be more than one transaction. A tick after every such call (never the
   library's own row/string printer) is therefore necessary but not sufficient:
   paint_ok_() adds the budget check that makes a stalled unit self-limiting instead of
   merely fed.

   LCD: one command()/write()/setCursor() is send() -> 2x write4bits() -> 3x
   expanderWrite() = 6 Wire.endTransmission() calls (LiquidCrystal_I2C.cpp:9-21,247-269),
   each independently capped at Wire's fixed 1000 ms (Wire.cpp:194) -- worst case 6000 ms
   for ONE character or ONE setCursor(), which already exceeds PB_WDT_GRANTED_MS on its
   own. send/write4bits/expanderWrite are private, so this floor cannot be lowered
   without forking the library; paint_ok_() cannot stop a call already in flight, only
   the NEXT one -- so the guarantee here is "at most one such unit, ever, per boot", not
   "never exceeds the grant". See config.h's citation for the full accounting.

   OLED: the opaque drawString() (u8x8_8x8.c's u8x8_draw_string() calls u8x8_DrawGlyph()
   once per character with no hook between them) is replaced by drawGlyph() per
   character, called directly. Traced through the ACTUAL cad callback this OLED class
   uses -- U8X8_SSD1306_128X64_NONAME_HW_I2C selects u8x8_cad_ssd13xx_FAST_i2c
   (U8x8lib.h:806-808), not the "classic" u8x8_cad_ssd13xx_i2c -- one drawGlyph() is
   3 Wire.endTransmission() calls: the DRAW_TILE handler's two SendCmd calls collapse to
   2 transactions (the fast cad merges a SendCmd immediately followed by SendArg/SendArg
   into the SAME transaction, but a SECOND SendCmd forces the first to close), and its
   one SendData call is a 3rd, separate transaction (u8x8_cad_ssd13xx_fast_i2c's
   SEND_DATA case always calls u8x8_i2c_data_transfer(), which opens and closes its own
   transfer). Worst case 3 x 1000 ms = 3000 ms per character -- comfortably under the
   5592 ms grant, a real closed bound, not just a kill-switch mitigation. */
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
  lcd.setCursor(0, r);
  if (!paint_ok_(t0)) return;
  for (uint8_t i = 0; i < 16 && text[i] != '\0'; ++i) {
    t0 = hal_millis();
    lcd.write((uint8_t)text[i]);
    if (!paint_ok_(t0)) return;
  }
}
