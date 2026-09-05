# Working on the firmware

PlatformIO project for an **Arduino UNO R4 WiFi** (Renesas RA4M1 + ESP32-S3 for WiFi, 32 KB RAM).
Read the umbrella's [AGENTS.md](https://github.com/plantbutler/plantbutler/blob/main/AGENTS.md) (on this machine: `~/projects/plant-butler/AGENTS.md`) and
[DECISIONS.md](https://github.com/plantbutler/plantbutler/blob/main/DECISIONS.md) first: decisions
#4 (protocol), #5 (what the firmware may decide) and #7 (safety) are what this code has to keep.

## Build, upload, tooling

```bash
make                 # build the BENCH binary (env uno_r4_wifi) - this is what runs unattended
make upload          # flash it
make bringup         # flash the BRING-UP binary: pump/cal/servo/home/goto/hang compiled in
make sim             # flash the SIM binary: no pump driver, no network stack
make test            # host suites: pio test -e native
make check           # the mechanical invariants of the spec's section 9
make monitor         # serial at 115200 (NOT 9600 - an 80-char hall line is 83 ms at 9600)
make compiledb       # compile_commands.json for clangd
```

Five environments. `uno_r4_wifi` is the one left running; `uno_r4_wifi_bringup` is for
bring-up 0-7d and is never left running; `uno_r4_wifi_test` runs the on-device suites;
`uno_r4_wifi_sim` has no pump driver and no network stack; `native` runs the host suites.
Four further environments — `native_bench`, `native_cal`, `native_measured`,
`native_nosimcli` — are `native` plus exactly one flag each, and exist only so that four
suites can be compiled a second time. Nothing uses `PLATFORMIO_BUILD_FLAGS`.

A fresh clone does not build until you create `include/secrets.h` (gitignored) defining
`WIFI_SSID`, `WIFI_PASS` (`const char[]`), `HTTP_PORT` (`const int`) and `HOST_NAME`
(`const char[]`). Never commit it. Library deps come from `platformio.ini` (Servo,
Arduino_SensorKit, LiquidCrystal_I2C, Network).

## The two seams

`include/hal.h` is seam 1: Arduino-free free functions, implemented by `src/hal_uno.cpp`
on the board and `src/hal_sim.cpp` on the host. `include/link.h` is seam 2: ten network
primitives, implemented by `lib/Network/src/link_wifi.cpp` and `src/link_fake.cpp`.
Implementations are selected by `build_src_filter`, never by a runtime flag.

These four clauses no longer share one answer, so they are kept separate rather than
folded into one sentence about `hal_uno.cpp` alone. **Only `src/hal_uno.cpp` arms an ISR
or writes D6** -- that half is still true without qualification. **The Arduino header has
three homes**: `src/hal_uno.cpp`, `src/sim_console.cpp` and `lib/Screen` (confirmed:
`grep -rln 'Arduino\.h' include src lib test` names exactly these three files).
`lib/Network` names none of it -- `lib/Network/src/link_wifi.cpp` includes `<WiFiS3.h>`,
not `<Arduino.h>`, and nothing else in that library mentions Arduino in any form.
**A pin number has two**: `src/hal_uno.cpp` (the real board) and `src/sim_console.cpp`,
which calls `pinMode(LED_BUILTIN, OUTPUT)` and `digitalWrite(LED_BUILTIN, ...)` directly
-- the sim binary's `build_src_filter` excludes `hal_uno.cpp` entirely, so a real LED
write has nowhere else in that binary to live, and `hal_sim.cpp`'s own `hal_pin_write()`
only appends to the fake rig's event trace rather than touching real GPIO. `lib/Network/
src/link_wifi.cpp` is the only file that names WiFiS3; `lib/Screen` the only one that
names the LCD or u8x8 libraries.

## tools/check.sh

`make check` runs it. Each grep protects one thing prose cannot hold: that D6 is written
by exactly two functions in the whole-word PFS form; that `pinMode` never touches D6, D2
or D3 outside `hal_arm_pulse_pins_`; that `safety_tick()` is the only feeder of the
watchdog; that nothing outside `safety.cpp` spins unbounded; that the network layer
cannot assert D6 and the safety layer cannot make a network call; that no report field is
printed with a signed conversion; that the bench binary has no bring-up console; and that
`safety.o` and `hal_uno.o` are byte-identical between the bench and bringup envs.

**Read `docs/superpowers/specs/2026-09-03-bench-sketch-design.md` before touching
`src/safety.cpp`.** It is the only file where a mistake puts water on the floor.

## Running the bench

These three sentences are requirements of the spec (§2.7, §2.9, §15.2), not advice, and
they are written here because there is nowhere else in the delivered tree they can live:

**A power cycle after a latch silently rearms the rig.** The dry latch and the
contradiction latch live in `.noinit`, which survives a warm reset and does not survive a
power cycle or a brown-out. Pulling the plug on a latched rig and plugging it back in
clears the latch and lets the next backend command water. Until the backend keeps the
durable half of the latch, the only safe way to end a latched session is to leave it
latched and read `status`.

**A `next` below about 60 s will visibly stutter while doses are live.** `dose_run()`
blocks for up to 60 s and `net_poll()` cannot run while it does, so a report interval
shorter than a dose is an interval the board cannot keep. The reports are not lost; they
are late, and the lateness is proportional to how much watering is happening.

**After any power event, look for gaps in `readings`.** The boot salt covers a watchdog
reset and the RESET button, not a brown-out or a power-cycle loop — those clear SRAM, so
the boot counter restarts and two boots can collide on `(controller, t)` inside the 300 s
dedup window, which shows up as a missing row rather than as an error anywhere.

## What is here (2026-08-30)

- `src/main.cpp` — inits both screens, `initialize()`s and `test()`s the manifold (cycles valves
  1-5, minutes of blocking delays), then loops once a second reading A0-A3 onto the screens.
  Network is commented out.
- `lib/Manifold` — a continuous-rotation servo on pin 8 turning a lead screw (through reduction
  gears) that moves a magnet cart over 5 gates; the magnet lifts the gate under it. Purely
  time-based: 21.7 s between gates, 12.5 s from home to gate 1, `delay()` throughout. `reset()`
  drives backwards for one gate-width and declares position 0 — from gate 5 that is wrong by about
  80 s (the threadless start of the screw is the real home; the cart parks there only if driven
  back far enough). Logs to a `Screen*` unconditionally.
- `lib/Screen` — one wrapper over the SensorKit OLED (u8x8) and a 16x2 I2C LCD at 0x27.
- `lib/Network` — WiFiS3 connect and a blocking HTTP GET `?potnr=…&k=v` to a PHP page that no
  longer matters. Leaks the socket and spins forever on a lost connection.
- A4/A5 are the I2C pins, so only A0-A3 are free while the screens are attached.
- No tests, no CI. README TODO: `String` → `char[]`.

## What happens next

In the plan, under the project "Board that reports and waters":

1. **Readings up the wire** (cycle 1, Jacopo, Claude reviews) — drop both screens (A4 becomes
   channel 5; `Manifold::log` needs a null-safe screen), pick 14-bit ADC once, post five raw
   channels once a minute as `k=v` lines over plain HTTP, reconnect on drop, retry once, discard.
   No TLS, no JSON, no OTA, no commands, no other manifold changes. Done when the board survives
   a WiFi drop, a router reboot and 48 hours unattended.
2. **Pump on command** — execute one bounded water command from the backend's response; strip
   the test-at-boot; every long delay becomes a watchdog-fed bounded wait.
3. **Manifold that knows where it is** — a hall sensor counting screw revolutions plus a home
   hall per manifold, 50-cycle endurance.
4. **Don't flood the flat** — put the hardware gate back between the pump pin, the float and the
   relay's input; refuse when position is unknown; status fields in every report.

Cycle 1's line "A4 becomes channel 5" is **dead**. The bench wiring supersedes it: A4/A5
are I2C, carrying the expander (which carries the mux select lines and the home hall) and
both screens. The five channels arrive through the mux on A0.

The board reports `(controller, channel)` raw counts and accepts a valve index; it never knows
what a pot or a plant is. The backend decides when to water; the firmware only enforces the caps.
