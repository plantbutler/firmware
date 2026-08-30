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
- `lib/Manifold` — a continuous-rotation servo on pin 8 stepping a rotary valve between 5
  outputs. Purely time-based: 21.7 s between valves, 12.5 s from home to valve 1, `delay()`
  throughout. `reset()` drives backwards for one valve-width and declares position 0 — from
  valve 5 that is wrong by about 80 s. Logs to a `Screen*` unconditionally.
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
3. **Manifold that knows where it is** — homing (hard stop or a microswitch), 50-cycle endurance.
4. **Don't flood the flat** — float switch on a sense pin and in the driver circuit, refuse when
   position is unknown, status fields in every report.

The board reports `(controller, channel)` raw counts and accepts a valve index; it never knows
what a pot or a plant is. The backend decides when to water; the firmware only enforces the caps.
