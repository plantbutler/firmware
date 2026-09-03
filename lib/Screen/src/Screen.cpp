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

/* Fix round 2 (Important): begin() is NOT budget-guarded, on either panel, and both
   chains are already past PB_WDT_GRANTED_MS (5592 ms) on their own:

   LCD -- lcd.init() (-> init_priv() -> begin()) + lcd.backlight(), traced against
   LiquidCrystal_I2C.cpp:63-129,152-159,233-241: one expanderWrite() (1 tx, the
   backlight-off reset) + four write4bits() 4-bit-mode-select attempts (4x3 = 12 tx) +
   five command()-based calls -- FUNCTIONSET, display(), clear(), ENTRYMODESET, home()
   (5x6 = 30 tx) + backlight()'s own expanderWrite() (1 tx) = 44 Wire transactions,
   ~44000 ms worst case. This is the mandatory HD44780 4-bit init sequence from the
   datasheet (LiquidCrystal_I2C.cpp:30-43's own comment cites it) -- nothing in it is
   optional, so unlike clear()'s old Oled.clearDisplay() below, there is no bounded
   alternative to swap in here.

   OLED -- Oled.initDisplay(): the SSD1306 init byte sequence
   (u8x8_d_ssd1306_128x64_noname.c's u8x8_d_ssd1306_128x64_noname_init_seq[]) is 16
   controller commands; under the fast I2C cad's merge rule (a SendCmd always
   force-closes whatever transfer is open) each is its own Wire transaction, so this is
   on the order of 16 Wire transactions and ~16000 ms worst case (traced from the
   visible byte sequence; the generic dispatch wrapper that sends it was not chased
   further, so treat this as a lower bound, not a certified count the way the LCD's 44
   and the old clearDisplay()'s 144 are). Also mandatory: the controller's own required
   power-up configuration.

   Both together: at least ~60000 ms of unguarded boot-time bus exposure, non-optional,
   with zero feed anywhere inside either chain. This is safe TODAY only because
   hal_wdt_start() (task 12, not written yet) is drafted to run AFTER both screens'
   begin() calls -- the dog is not yet armed while this runs, and WDTimer::refresh()
   (WDT.cpp:80-84) is a no-op until WDTimer::begin() has set _is_initialized, so even
   the safety_tick() calls inside clear() below (see the OLED branch) feed nothing
   real yet. THE MOMENT hal_wdt_start() runs before this function, that ~60 s becomes
   a live unfed span against a 5592 ms grant. Nothing in this file enforces that
   ordering -- it is task 12's setup() to get right and this comment to be read before
   changing it. */
void Screen::begin() {
  if (!present_) return;
  if (type == ScreenType::Oled) {
    Oled.initDisplay();
    Oled.setFlipMode(true);
    Oled.setFont(u8x8_font_chroma48medium8_r);
    /* Oled.begin() would be initDisplay();clearDisplay();setPowerSave(0)
       (U8x8lib.h:258-259) -- clearDisplay() is the SAME 144-transaction function
       clear() below was rewritten specifically to stop calling at runtime. Decision:
       it is not worth it here either. It buys nothing over the bounded path clear()
       already provides (a blank screen), at 144 unguarded transactions clear() no
       longer pays. So this calls clear() instead: bounded to <= 3000 ms per unit, and
       the panel goes not-present on overrun instead of the boot hanging. Trade-off
       accepted: the OLED's own GDRAM may still hold stale content from before a warm
       reset (an MCU reset does not necessarily reset the OLED controller), so that
       content stays visible for the length of this call rather than a fixed
       unconditional clear -- cosmetic only, and the first real paint overwrites it
       regardless. setPowerSave(0) runs last, so the panel is not driven visibly until
       it is already blank. */
    clear();
    Oled.setPowerSave(0);
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
