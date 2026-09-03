# Working on the firmware

PlatformIO project for an **Arduino UNO R4 WiFi** (Renesas RA4M1 + ESP32-S3 for WiFi, 32 KB RAM).
Read the umbrella's [AGENTS.md](https://github.com/plantbutler/plantbutler/blob/main/AGENTS.md) (on this machine: `~/projects/plant-butler/AGENTS.md`) and
[DECISIONS.md](https://github.com/plantbutler/plantbutler/blob/main/DECISIONS.md) first: decisions
#4 (protocol), #5 (what the firmware may decide) and #7 (safety) are what this code has to keep.

## Build, upload, tooling

```bash
pio run                     # build; env is uno_r4_wifi (platformio.ini)
pio run -t upload           # flash over USB
pio device monitor -b 9600  # serial
make / make upload / make compiledb   # the same, plus compile_commands.json for clangd
```

A fresh clone does not build until you create `include/secrets.h` (gitignored) defining
`WIFI_SSID`, `WIFI_PASS` (`const char[]`), `HTTP_PORT` (`const int`) and `HOST_NAME`
(`const char[]`). Never commit it. Library deps come from `platformio.ini` (Servo,
Arduino_SensorKit, LiquidCrystal_I2C).

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

The bench wiring is drawn and generated in `cad/wiring` (pin map, what switches the pump, power,
bring-up order). Two things there bind this code from cycle 2 on. The pump is switched by a relay
module, so nothing in hardware ANDs "firmware says pump" with "the tank has water": the IWDT, a
hard maximum run time in the same code path that asserts the pump pin, and a no-flow abort from
the meter are mandatory, not nice to have. And the bench uses A4/A5 for I2C (the expander that
carries the mux select lines and the home hall), so cycle 1's "A4 becomes channel 5" holds only
until the mux is wired; after that the five channels arrive through the mux on A0.

The board reports `(controller, channel)` raw counts and accepts a valve index; it never knows
what a pot or a plant is. The backend decides when to water; the firmware only enforces the caps.
