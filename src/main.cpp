/* src/main.cpp -- DEVICE ONLY. Task 12 writes the full setup order of spec §2.5/§5/§12. */
#include "Screen.h"

Screen g_oled_screen(ScreenType::Oled);
Screen g_lcd_screen(ScreenType::Lcd);

extern "C" void setup(void) {
  /* Order is load-bearing and lands in task 12. Both panels are probed and opened BEFORE
     sensors_begin(), because init_priv() re-opens the IIC peripheral (spec §5). */
  g_oled_screen.probe();
  g_oled_screen.begin();
  g_lcd_screen.probe();
  g_lcd_screen.begin();
}

extern "C" void loop(void) {}
