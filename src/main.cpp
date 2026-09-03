/* src/main.cpp -- DEVICE ONLY. [env:native] filters this file out (no main()).
   Task 12 fills in the setup order of spec §2.5/§5/§12 and the loop of spec §3.
   No Arduino header here, ever: spec §9 allows it only in hal_uno.cpp, lib/Network
   and lib/Screen. Everything this file needs arrives through include/hal.h.

   setup()/loop() need extern "C" linkage: cores/arduino/main.cpp's arduino_main()
   calls them through the extern "C" declaration in api/Common.h (pulled in via
   Arduino.h), so a plain C++ definition here links under a mangled name and the
   framework's arduino_main() fails to find it. */

extern "C" void setup(void) {}
extern "C" void loop(void) {}
