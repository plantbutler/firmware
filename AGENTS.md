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

A fresh clone does not build until you create `include/secrets.h` (gitignored). Copy
`include/secrets.h.example` and fill it in — it is the list, and it says "six names, no
more": `WIFI_SSID`, `WIFI_PASS`, `HOST_NAME` and `BUTLER_TOKEN` (`const char[]`),
`HTTP_PORT` (`const int`), and the `PB_CONTROLLER` string macro, which the example defaults
to `"bench1"`. `BUTLER_TOKEN` is not optional: `src/netfsm.cpp` names it in a
`static_assert` over `PB_TX_CAP`, so omitting it fails the compile rather than showing up
later as a missing header field. Never commit the file. Library deps come from
`platformio.ini` (Servo, Arduino_SensorKit, LiquidCrystal_I2C, Network).

## The two seams

`include/hal.h` is seam 1: Arduino-free free functions, implemented by `src/hal_uno.cpp`
on the board and `src/hal_sim.cpp` on the host. `include/link.h` is seam 2: eleven network
primitives (`link_begin`, `link_join`, `link_state`, `link_rssi`, `link_ip`, `link_reset`,
`link_desyncs`, `sock_open`, `sock_write`, `sock_read`, `sock_close`), implemented by
`lib/Network/src/link_wifi.cpp` and `src/link_fake.cpp`. `tools/check.sh` names all eleven
and grants the callers an explicit exclusion list, so adding a twelfth means editing it.
Implementations are selected by `build_src_filter`, never by a runtime flag.

These four clauses no longer share one answer, so they are kept separate rather than
folded into one sentence about `hal_uno.cpp` alone. **Only `src/hal_uno.cpp` arms an ISR
or writes D6** -- that half is still true without qualification. **The Arduino header has
three homes**: `src/hal_uno.cpp`, `src/sim_console.cpp` and `lib/Screen` (confirmed:
`grep -rln 'Arduino\.h' include src lib test` names exactly these three files).
`lib/Network` names none of it -- `lib/Network/src/link_wifi.cpp` includes `<WiFiS3.h>`,
not `<Arduino.h>`, and nothing else in that library mentions Arduino in any form.
**A literal pin number has two homes, and `src/hal_uno.cpp` is not one of them**:
`include/pins.h` is the wiring contract and holds every number the board uses; `hal_uno.cpp`
holds none of its own, reaching every pin through a macro — `PIN_*` from `pins.h`, or the
core's `SDA`/`SCL` in the nine-clock recovery. The second home is `src/hal_sim.cpp`
(`SIM_PUMP_PIN 6`, `SIM_PIN_SDA 18`, `SIM_PIN_SCL 19`), and it says why in its own comment:
`PIN_PUMP_EN` is defined only in the translation unit that defines `PB_PUMP_OWNER` -- which
is `hal_uno.cpp` and nothing else -- so the fake rig cannot reach D6 through `pins.h` and
has to spell it. `src/sim_console.cpp` is not a third home: it writes `LED_BUILTIN`, the
core's own name for pin 13, and `pins.h`'s `PIN_LED` was deleted for exactly that reason.
`lib/Network/src/link_wifi.cpp` is the only file that includes `<WiFiS3.h>` (several others
name it in prose, which is not the same thing and is what the greps are careful about);
`lib/Screen` the only one that names the LCD or u8x8 libraries.

## tools/check.sh

`make check` runs 34 invariants, each protecting one thing prose cannot hold: that D6 is
written by exactly two functions in the whole-word PFS form; that `pinMode` never touches
D6 **at all** (it would latch PODR=0 and drive the pin LOW), which is a separate and
stricter rule from the D2/D3 one beside it — every `pinMode` on D2 or D3 must sit inside
`hal_arm_pulse_pins_`; that `safety_tick()` is the only feeder of the watchdog **in
`safety.cpp`**, and that nothing outside `safety.cpp` and the two HALs feeds it at all —
`hal_wdt_alive()` feeds twice, deliberately, bracketing the one probe window in the program
that is meant to go unfed, so "the only feeder" without the qualification is not true and
`tools/check.sh` does not claim it; that nothing outside `safety.cpp` spins unbounded; that
the network layer cannot assert D6 and the safety layer cannot make a network call; that no
report field is printed with a signed conversion; that the bench binary has no bring-up
console; and that `safety.o` and `hal_uno.o` are byte-identical between the bench and
bringup envs.

Eight of the 34 read object files, or a preprocessed translation unit, out of `.pio/build`
rather than source text. On a fresh clone seven of them cannot run — the eighth builds the
env it needs itself — and the script now says which, and **exits 2** rather than reporting
a green gate: a skip is not a violation, but a gate that could not run part of itself has
not passed. The output names the `pio run -e ...` that makes each one run. Those seven are
also the ones that matter most to a stranger: all three `lib/Network` build checks and the
`safety.o`/`hal_uno.o` pair exist BECAUSE `link_wifi.cpp` went uncompiled into every binary
for five tasks, which no grep over source text could see.

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

## What is here

- `src/main.cpp` — `setup()` in a load-bearing order (spec §2.1 pins the first statement,
  §5 the screen/bus order), a boot banner, then a `loop()` that calls `safety_tick()`,
  `cli_poll()`, `net_poll()`, `exec_pending()`, `pulses_leak_poll()` and the UI pair. No
  unbounded wait in any of them; the one that can wait does it through `safety_wait_ms()`,
  which feeds the dog. `net_begin()` runs BEFORE the boot verdict is latched, and that
  order is the mechanism rather than a detail — see the comment there.
- `src/safety.cpp` — the dose ladder and `dose_run()`, the one function that asserts D6.
  Read the spec before touching it.
- `src/netfsm.cpp` — the network state machine: one bounded link or socket step per pass,
  at most two AT commands in any of them.
- `src/report.cpp` / `src/exec.cpp` — the `k=v` body, and the one bounded command per
  round trip that comes back in the response.
- `src/sensors.cpp`, `src/pulses.cpp`, `src/cli.cpp`, `src/ui.cpp`, `src/noinit.cpp` — the
  mux and I2C health, the flow and screw meters, the console, the two panels, the latches.
- `src/hal_uno.cpp` / `src/hal_sim.cpp` — seam 1's two implementations. The board one owns
  the only ISR, the only D6 write and the only `delay()` in the tree (one 50 ms USB settle
  at power-on, which `tools/check.sh` exempts by name and forbids everywhere else).
- `lib/Manifold` — `cart.cpp`: the cart positioned by COUNTED SCREW PULSES plus the home
  hall, on the `PIN_SERVO` (D9) continuous servo. Every blocking wait in the old
  time-based Manifold became a (target pulses, deadline, stall window) triple. It writes to
  no panel and names no network header.
- `lib/Screen` — one wrapper over the SensorKit OLED (u8x8) and a 16x2 I2C LCD at 0x27.
- `lib/Network` — `link_wifi.cpp`, seam 2's driver, plus `include/Network.h`, which nothing
  includes and which **must not be deleted**: its comment is the only surviving account of
  why this library once failed to build at all. Device only: `[env:native]` puts it in
  `lib_ignore`, and `[env:uno_r4_wifi]` has to name `Network` in `lib_deps` explicitly or
  PlatformIO's dependency finder never compiles it — which is exactly what happened for
  five tasks, undetectably, until a device build finally linked.
- A4/A5 are the I2C pins. The six channels do not use the other analogue pins: they arrive
  through the mux on A0 (`PIN_MUX_ADC`), whose select lines are on the expander.
- Tests are the only evidence any of this works, because no board is attached: seven host
  suites under `[env:native]` (`test_cart`, `test_cli`, `test_contra`, `test_dose`,
  `test_net`, `test_report`, `test_sensors`), each compiled again under the four one-flag
  variants, plus `test_device`, which `[env:native]`'s `test_ignore` keeps out and which
  only ever gets LINKED here — it needs the board to run. No CI.

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

Cycle 1's line "A4 becomes channel 5" is **dead**, and so is its "five raw channels".
The bench wiring supersedes both: A4/A5 are I2C, carrying the expander (which carries the
mux select lines and the home hall) and both screens, and `PB_CHANNELS` is **6** — C0-C4
moisture plus C5, the LDR — all six arriving through the mux on A0. `src/report.cpp` emits
`ch0`..`ch5`.

The board reports `(controller, channel)` raw counts and accepts a valve index; it never knows
what a pot or a plant is. The backend decides when to water; the firmware only enforces the caps.
