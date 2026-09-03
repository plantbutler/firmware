# Bench Sketch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (- [ ]) syntax for tracking.

**Goal:** Build the bench-rig firmware specified in `docs/superpowers/specs/2026-09-03-bench-sketch-design.md`: an Arduino UNO R4 WiFi sketch that runs the nine serial bring-up commands live, posts six raw sensor channels plus ten diagnostic channels once a minute to `butler.py` over plain HTTP, executes and acks at most one bounded water command, paints two I2C screens, and enforces — in firmware — the four measures that stand in for the hardware interlock the wiring package removed: a watchdog whose counter is proved to move, a hard cap in the same code path that asserts the pump pin, two time-armed no-flow aborts, and a float/flow contradiction latch. It ships with `PB_REPORT_POS_UNKNOWN=1` defined, so no backend water command is ever queued until a deliberate later commit (spec §4.6).

**How many channels, once, so nobody has to work it out twice.** `PB_CHANNELS` is **6** and `sensors_sweep()` reads `ch0`..`ch5` — five moisture probes on C0-C4 plus one LDR on C5 (spec §7's comment on `PB_CHANNELS`). **All six reach the wire** as `ch0=`..`ch5=`; a channel whose select or read failed is omitted rather than sent as 0 (spec §4.1's `chN=` row). Where this plan or the spec says "five raw sensor channels" it means the five moisture probes the backend waters from; the sixth is the light reading and is carried on the same terms. `PB_BODY_WORST_FIXED` (288) is derived in spec §7 against six `chN=` fields plus the ten diagnostics, so the buffer arithmetic already accounts for it.

**Architecture:** Two seams, both plain C free functions, selected by `build_src_filter` and not by any runtime flag (spec §1). Seam 1 is `include/hal.h` — 35 Arduino-free functions (spec §1's 32, plus `hal_micros()`, `hal_adc_bits()` and `hal_adc_width_ok()`, all three recorded as deviations in task 3) implemented by `src/hal_uno.cpp` on the board and by `src/hal_sim.cpp` on the host and in the sim binary. Seam 2 is `include/link.h` — 10 network primitives implemented by `lib/Network/src/link_wifi.cpp` on the board and `src/link_fake.cpp` on the host. Everything else — `safety`, `pulses`, `sensors`, `cart`, `report`, `netfsm`, `exec`, `cli`, `ui` — is logic that compiles and is unit-tested on the host under `pio test -e native`. `src/hal_uno.cpp` is the only translation unit that includes `<Arduino.h>`, names a pin, owns an ISR or writes D6; `lib/Network/src/link_wifi.cpp` is the only one that names `WiFiS3`; `lib/Screen` is the only one that names `LiquidCrystal_I2C` or `u8x8`. Mechanical invariants that prose cannot hold are greps in `tools/check.sh`, run by `make check`.

**Tech Stack:** PlatformIO Core 6.1.19; `platform = renesas-ra@1.6.0`, `framework = arduino`, `board = uno_r4_wifi` (Renesas RA4M1 + ESP32-S3, 32 KB SRAM, 1 KB main stack); C++ compiled `-std=gnu++17` by the renesas-ra builder; Unity via `pio test -e native` on the host and `pio test -e uno_r4_wifi_test` on the board; libraries `arduino-libraries/Servo@^1.2.2`, `arduino-libraries/Arduino_Sensorkit@^1.4.0`, `marcoschwartz/LiquidCrystal_I2C@^1.1.4`; serial at 115200; backend is `butler.py` (FastAPI + SQLite) speaking plain-HTTP `k=v` lines.

**Spec:** docs/superpowers/specs/2026-09-03-bench-sketch-design.md

## Global Constraints

- Repository root for every path in this plan is `/Users/jcanton/projects/plant-butler/firmware` (the umbrella submodule). The retired clone at `~/projects/plant_butler` is not this repo.
- The spec travels with the plan: `docs/superpowers/specs/2026-09-03-bench-sketch-design.md`. Every constant, pin number, line-number citation and test name in this plan comes from it. Do not invent a value; if a task says the spec does not give one, say so in the commit message rather than guessing.
- Before a single module is written, prove `pio test -e native --without-testing` links (spec, "Read this first", item 7). Everything else depends on the answer.
- `platform = renesas-ra@1.6.0`, `framework = arduino` and `board = uno_r4_wifi` belong in `[env:uno_r4_wifi]`, NEVER in the global `[env]`: PlatformIO inherits `[env]` into every environment, and a global `framework = arduino` makes `pio test -e native` abort before compiling anything (spec §10, verified empirically on PlatformIO Core 6.1.19).
- The global `[env]` carries `build_flags = -Wall -Wextra` and nothing else. Never put `-std=gnu++*` in the global `[env]` or in any renesas-ra environment: that builder already supplies `-std=gnu++17` for C++ and `-std=gnu11` for C (builder/frameworks/arduino.py:100,123), and a `-std=gnu++` in build_flags lands in SCons CCFLAGS and warns once per core .c file on every build (spec §1, §10). `[env:native]` is the exception and needs its own `-std=gnu++17`, because `platform = native` supplies none.
- Library version floors, exactly: `arduino-libraries/Servo@^1.2.2`, `arduino-libraries/Arduino_Sensorkit@^1.4.0`, `marcoschwartz/LiquidCrystal_I2C@^1.1.4` (spec §10).
- `test_build_src = yes` is required in BOTH `[env:native]` and `[env:uno_r4_wifi_test]`, or none of `src/report.cpp`, `src/safety.cpp`, `src/netfsm.cpp` links into a test and every suite fails on unresolved symbols (spec §10).
- `monitor_speed = 115200`. Baud moves from 9600 to 115200 project-wide; at 9600 an 80-character line of the `hall` stream is 83 ms of blocking (spec §7).
- Every module is a `.cpp`, not a `.c` — the seams are plain C free functions, the files are not. A `.c` file gets `-std=gnu11` from the renesas-ra builder, so it would have no `static_assert` (spec §1).
- Citations belong in `include/config.h` next to the constants they justify — a constant without one is a bug report waiting to be written (spec preamble).
- Pin the backend citations by NAME, not by line number: every `butler.py:` number in the spec was derived mechanically with grep on 2026-09-03 against a file still being edited. Re-grep before trusting one (spec, "Read this first", item 5).
- The pin contract: `include/pins.h` defines `PIN_PUMP_EN` ONLY when `PB_PUMP_OWNER` is defined, and `src/hal_uno.cpp` is the only file in the tree that defines `PB_PUMP_OWNER` (spec §2.2, §8).
- The pin contract: there is NO default relay polarity. `pins.h` `#error`s unless exactly one of `PB_RELAY_ACTIVE_LOW` / `PB_RELAY_ACTIVE_HIGH` is defined in `platformio.ini` build_flags. A board cannot be flashed before someone has read the relay module (spec §2.2).
- The pin contract: D6 is written by exactly two functions, `hal_boot_pump_off()` and `hal_pump_write()`, both using the whole-word `R_IOPORT_PinCfg(NULL, g_pin_cfg[PIN_PUMP_EN].pin, cfg)` form that carries direction AND level in one PmnPFS write. `pinMode` must never touch D6: on this core `pinMode(pin, OUTPUT)` latches PODR = 0 and drives the pin LOW, discarding a preceding `digitalWrite` (spec §2.1 — this corrects `cad/wiring/README.md`).
- The pin contract: D2 (`PIN_FLOW`) and D3 (`PIN_HALL_SCREW`) are configured in exactly one function, `hal_arm_pulse_pins_()`: `pinMode(INPUT_PULLUP)` then `attachInterrupt` then the direct `R_ICU->IRQCR[ch]` FLTEN write. Any later `pinMode` on those pins silently detaches the interrupt (spec §2.14).
- `safety_tick()` is the only caller of `hal_wdt_feed()`, and `hal_wdt_alive()`'s 40 ms probe is the one place in the program where feeding is deliberately suspended. Nothing else may skip a `safety_tick()` (spec §2.4, §2.5).
- Every loop that can iterate over an I2C transfer, a modem call or a millisecond of wall clock calls `safety_tick()` on each iteration. That is what makes a 60 s dose legal under a 5592 ms window (spec §3).
- The `make check` invariants are listed below **in the exact POSIX-class form `tools/check.sh` uses**. `/usr/bin/grep` on macOS is BSD grep, which does not honour `\b` or `\s`, so a reader comparing this list against the script must not "fix" the script back to those escapes. The authoritative table is spec §9's; this list is its transcription, and the two prose sentences that count the invariants (task 13's commit and task 30's commit) name the table rather than a number.
- `make check` invariant: exactly 1 file **defining** `PB_PUMP_OWNER` (the pattern is `define[[:space:]]\+PB_PUMP_OWNER`, not the bare token: `include/pins.h` necessarily *tests* it with `#ifdef`, so a bare-token file count would be 2 forever).
- `make check` invariant: exactly 0 hits for `pinMode\(PIN_PUMP_EN`.
- `make check` invariant: exactly 2 hits for `R_IOPORT_PinCfg.*PIN_PUMP_EN` (hal_boot_pump_off, hal_pump_write).
- `make check` invariant: exactly 0 hits for `R_IOPORT_PinWrite.*PIN_PUMP_EN|digitalWrite.*PIN_PUMP_EN`.
- `make check` invariant: every hit of `pinMode\((PIN_FLOW|PIN_HALL_SCREW)` in the tree is inside `hal_arm_pulse_pins_`'s body — an `awk` range over `src/hal_uno.cpp` must count the same number the whole-tree grep does (task 13 step 2 is the only place this is checked; task 8 does not attempt it).
- `make check` invariant: exactly 1 hit for `WDT\.refresh` (inside `hal_wdt_feed`).
- `make check` invariant: exactly 1 hit for `hal_wdt_feed\(` in safety.cpp (`safety_tick`).
- `make check` invariant: exactly 0 hits for `hal_wdt_feed\(` outside safety.cpp, hal_uno.cpp, hal_sim.cpp, hal.h.
- `make check` invariant: exactly 0 hits for `WDT\.getTimeout` (it lies under the cfg overload).
- `make check` invariant: exactly 0 hits for `(^|[^[:alnum:]_])delay\(` outside hal_uno.cpp.
- `make check` invariant: exactly 0 hits for `Wire\.flush`.
- `make check` invariant: exactly 0 hits for `lcd\.print|lcd\.println` (no hook between characters).
- `make check` invariant: exactly 0 hits for `WiFi\.ping` (it resets modem.timeout to 10 s).
- `make check` invariant: exactly 0 hits for `%[0-9.]*[fgeFGE]([^[:alnum:]]|$)` in any format string (newlib float printf is the deepest stack consumer in the program).
- `make check` invariant: exactly 0 hits for `%[0-9.]*[di]([^[:alnum:]]|$)` in report.cpp and netfsm.cpp format strings — every `t=`/`ack=`/`chN=` site is `%lu` with an explicit `(unsigned long)` cast.
- `make check` invariant: exactly 0 hits for `WiFiS3|link\.h|Network\.h` in safety.cpp and lib/Manifold.
- `make check` invariant: exactly 0 hits for `safety\.h|dose_run|hal_pump_write` in netfsm.cpp, lib/Network/, ui.cpp.
- `make check` invariant: exactly 1 hit for `dose_run\(` in cli.cpp. `src/exec.cpp` carries the tree's expected **second** call site; the invariant is scoped to `cli.cpp` and is never widened.
- `make check` invariant: exactly 0 hits for `for[[:space:]]*\([[:space:]]*;[[:space:]]*;[[:space:]]*\)|while[[:space:]]*\([[:space:]]*(true|1)[[:space:]]*\)` outside safety.cpp.
- `make check` invariant: exactly 0 hits for `String|std::map|std::string|(^|[^[:alnum:]_])new([^[:alnum:]_]|$)|malloc[[:space:]]*\(` outside lib/Network, lib/Screen. **The allocation half of the pattern is `malloc[[:space:]]*\(`, a call and not the bare word**, because `src/hal_uno.cpp` legitimately writes `#include <malloc.h>` for `mallinfo()` and a bare `malloc` would match the include forever (task 8 step 8, task 13 step 4).
- `make check` invariant: exactly 0 hits for `Arduino\.h` outside src/hal_uno.cpp, lib/Network, lib/Screen — **and, from task 29 onward, `src/sim_console.cpp`**, the device-only console shim the sim binary needs. That widening is a recorded deviation from spec §9's table; it is made in task 29 step 6, in the same commit that creates the file, and nowhere else.
- `make check` invariant: exactly 0 hits for `"hang"|"prime"|"cal "` — the C **string literals**, never the bare words — in the bench env's preprocessed src/cli.cpp. A bare `hang` matches the substring inside `sensors_float_change_age_s`, `exchange` and `unchanged`, all of which reach that translation unit through its includes.
- `make check` invariant: exactly 2 files containing `PB_BRINGUP` **under `src` and `lib`** (cli.cpp, main.cpp). The scan is deliberately NOT `include` or `test`, and that is a recorded deviation from spec §9's table, made in task 13 step 5 and named again in task 20 step 13: `include/config.h` gains a `!defined(PB_BRINGUP)` arm in the uncalibrated-build guard (task 20 step 13) and `test/test_cli/test_cli.cpp` compiles **both** arms of the bench-vs-bringup cases on purpose (task 20). Neither is a console affordance, which is what the invariant exists to bound; widening the scan back to `include test` would make `make check` fail on every task from 20 onward.
- `make check` invariant: `.pio/build/uno_r4_wifi_sim` contains a `hal_sim.cpp.o` and **no** `hal_uno.cpp.o`.
- `make check` invariant: identical sha for safety.o and hal_uno.o between `uno_r4_wifi` and `uno_r4_wifi_bringup`.
- **Comment hygiene, from task 8 onward and not optional.** These greps scan comments as well as code, in `include/`, `src/`, `lib/` and `test/`. The following are poison in a comment as much as in a statement: `delay(`, `lcd.print`, `Wire.flush`, `WiFi.ping`, `pinMode(PIN_PUMP_EN`, `WDT.getTimeout`, `Arduino.h` (outside the three allowed files), the bare word `new`, `String`, `std::map`, `std::string`, `malloc(`, `for(;;)`, `while(true)`, and any `%f`/`%g`-shaped token — **including inside a string literal in a test**, which is why task 11's float-formatting case builds its needles character by character. Write "a one-second blocking wait", "the library's own row printer", "the unbounded flush helper of TwoWire", "never the timeout getter", "the ping helper" instead. A failing `make check` on a comment is the check working; the fix is to reword the comment, never to weaken the pattern.
- Naming: `err=` is a bare lowercase token from a fixed enum, tested to contain no whitespace — `none float pos noflow noise cap stop wdt dry contra boot range cal i2c busy cooldown leak adc stuck txcap resetmid heap goto recv`. A space would split into a non-`k=v` token and 400 the whole report (spec §4.1).
- Naming: `err=recv` is in the enum because `g_last_err` can hold it, and is the one token that must never reach the wire (spec §4.1, §4.3).
- Naming: `float=` is never 2 and never negative; `ack=` is never 0; `chN` values are bare non-negative integers with no sentinels — `_int_in` bounds are half-open and ASCII-digits-only, so any `-1`, `unknown` or `never` 400s the whole report (spec §0, §2.10, §4.1).
- Naming: every key appears at most once in a report body. The body is assembled from four independent sources into one buffer, so this is a rule the builder holds, not a property it gets for free (spec §4.1).
- Naming: host test functions use the exact names in spec §9 where the spec gives one. Test directories are `test/test_dose/`, `test/test_contra/`, `test/test_report/`, `test/test_net/`, `test/test_cart/`, `test/test_sensors/`, `test/test_cli/`, and `test/test_device/` (created by task 28; DEVICE ONLY — it never runs under `[env:native]`, which has no board); shared fixtures live in `test/support/harness.h` and the shared canned response bodies in `test/support/bodies.h`, both of which are HEADERS — `test/support/` must not become a suite directory with no runner (spec §9, §10).
- Naming: every suite includes the shared headers by the **relative** form — `#include "../support/harness.h"`, `#include "../support/bodies.h"` — because `#include "x"` searches the including file's own directory first, and that resolves from any suite directory without a `-I test` on the command line. Do not write `"support/harness.h"`: no environment in this plan sets an include path that would resolve it.
- TDD throughout: write the failing test, run it and see it fail, write the minimal code, run it and see it pass, commit. Each step is one action of 2-5 minutes.
- **The running `N Tests` totals printed after each `pio test` are ADVISORY; `0 Failures` plus the named cases actually appearing in the output is the gate.** The `test_dose` chain (1, 4, 10, 16, 18, 19 through tasks 1-6) was recounted top to bottom and is right. The `test_report` and `test_net` chains were **not** — they are internally inconsistent and the Assembly notes name the exact lines. If a total disagrees with what you see, count the `RUN_TEST` lines in the suite and correct the plan's number in that task's commit message; never delete a case to reach a number.
- Ordering rule: the tree must build and its tests must pass at the end of every task. `pio test -e native` and `pio run -e uno_r4_wifi` are the gate on every commit; `make check` is the gate from task 13 onward.
- Do not commit `include/secrets.h` (gitignored). `include/secrets.h.example` is checked in and defines WIFI_SSID, WIFI_PASS, HTTP_PORT, HOST_NAME, BUTLER_TOKEN and PB_CONTROLLER (spec §11). **`include/secrets.h` itself is created by task 1 step 8, by copying the example and filling it in, and by no other step.** It is a build input from task 11 onward — `cli.cpp`, `main.cpp`, `report.cpp`, `netfsm.cpp` and `link_wifi.cpp` all include it — so a clone without it does not compile, on the host or on the board.
- `HOST_NAME` and `BUTLER_TOKEN` are `const char[]` OBJECTS, not string-literal macros; only `PB_CONTROLLER` is a macro. Never write `"Host: " HOST_NAME` — juxtaposition needs literals on both sides and will not compile. Format them with `%s`, or build the needle with `snprintf` in a test.

## File Structure

```
plant-butler/firmware/
├── platformio.ini                          five envs in full: uno_r4_wifi (the unattended binary),
│                                           uno_r4_wifi_bringup, uno_r4_wifi_test, uno_r4_wifi_sim, native
│                                           -- plus four one-flag variants of native (native_bench,
│                                           native_cal, native_measured, native_nosimcli) that exist
│                                           only to compile a host suite a second time
├── Makefile                                all / upload / test / sim / bringup / check / calib / compiledb,
│                                           monitor at 115200
├── tools/check.sh                          the mechanical invariants of spec §9, as greps; exit non-zero on any miss
├── AGENTS.md                       (edit)  the five envs, the two seams, the greps, 115200 baud,
│                                           and the three "running the bench" sentences of §2.7/§2.9/§15.2
├── README.md                       (edit)  the `String -> char[]` TODO closes by construction
├── include/
│   ├── pins.h                              the wiring contract as constants; the PB_PUMP_OWNER gate;
│   │                                       the mandatory relay-polarity #error
│   ├── config.h                            every tunable and measured constant with the citation that justifies it
│   ├── hal.h                               seam 1: Arduino-free free functions, own PB_LOW/PB_HIGH/PB_IN/PB_OUT
│   ├── link.h                              seam 2: the 10 network primitives and link_state_t
│   ├── noinit.h                            the warm-reset struct, its magic and its checksum
│   ├── safety.h                            safety_tick/safety_wait_ms, the float debounce, both latches,
│   │                                       dose_req_t/dose_result_t and dose_run()
│   ├── pulses.h                            the two edge counters, the rate estimator, pulses->ml, the leak watch
│   ├── sensors.h                           the expander, the mux discipline, the canary, the home hall, I2C health
│   ├── report.h                            report_in_t, report_build(), the t= stamps
│   ├── netfsm.h                            net_state_t and the report FSM's public surface
│   ├── exec.h                              cmd_t/response_t and exec_pending() — the ack cycle's owner
│   ├── cli.h                               the console's public surface, including cli_stop_requested()
│   ├── ui.h                                ui_state_t and the two pure renderers
│   ├── sim.h                               the fake rig's fault injectors and its call-trace log,
│   │                                      AND the scripted link's link_fake_* control surface,
│   │                                      which task 21 appends HERE. There is no link_fake.h.
│   ├── secrets.h            (gitignored)   WIFI_SSID, WIFI_PASS, HTTP_PORT, HOST_NAME, BUTLER_TOKEN, PB_CONTROLLER
│   └── secrets.h.example    (checked in)   the same six names with placeholder values
├── src/
│   ├── main.cpp                            setup order, the five-line loop; DEVICE ONLY (filtered out on native)
│   ├── hal_uno.cpp                         DEVICE ONLY. The only file with <Arduino.h>, a pin number, an ISR or a D6 write
│   ├── hal_sim.cpp                         sim + native: the fake rig — tank, pump, flow pulses, screw, expander,
│   │                                       six channels, a settable WDT counter, a settable UART — and its injectors
│   ├── sim_console.h                       SIM BINARY ONLY (task 29): the two-line internal seam between
│   │                                       hal_sim.cpp and the device console shim. Carries no Arduino header
│   ├── sim_console.cpp                     SIM BINARY ONLY (task 29). DEVICE ONLY. The fourth and last home of
│   │                                       <Arduino.h>: three console functions and nothing else
│   ├── link_fake.cpp                       sim + native: canned backend responses and scripted link failures
│   ├── safety.cpp                          safety_tick, safety_wait_ms, the float debounce, the dry latch,
│   │                                       the contradiction latch, and dose_run() — the only caller of hal_pump_write()
│   ├── pulses.cpp                          the D2/D3 ISR bodies, the per-pin minimum-gap reject, torn-read-safe
│   │                                       snapshots, the 100 ms rate estimator, pulses->ml, the leak watch
│   ├── sensors.cpp                         PCF8575 + mux + open-channel canary + home hall + I2C health,
│   │                                       back-off and the bounded nine-clock recovery
│   ├── report.cpp                          report_build() and response_parse() — the wire protocol, pure
│   ├── netfsm.cpp                          the report state machine and the HTTP framing, above seam 2
│   ├── exec.cpp                            exec_pending(): the boot self-home, the ack cycle, and the park
│   ├── cli.cpp                             the bench commands always; the bring-up commands under #if PB_BRINGUP
│   └── ui.cpp                              the two pure renderers and the coarsened painter
├── lib/
│   ├── Manifold/include/cart.h             the cart's public surface: position by counted screw pulses
│   ├── Manifold/src/cart.cpp               home / goto / jog / bus_check; one servo start, one servo stop
│   ├── Network/include/Network.h           reworked to a single `#include "link.h"`
│   ├── Network/src/link_wifi.cpp           DEVICE ONLY. The WiFiS3 driver behind seam 2, and nothing else
│   ├── Screen/include/Screen.h             the OLED/LCD wrapper, plus probe() and present()
│   └── Screen/src/Screen.cpp               DEVICE ONLY. A panel that does not answer becomes a permanent no-op
└── test/
    ├── support/harness.h                   the Unity fixture over hal_sim (a HEADER, not a suite)
    ├── support/bodies.h                    the canned HTTP responses shared by test_net, test_cart
    │                                       and test_contra (a HEADER: separate suite directories are
    │                                       separate binaries, so a file static cannot be shared)
    ├── test_dose/test_dose.cpp             the pump pin, the watchdog probe, the refusal ladder, the caps, the aborts
    ├── test_contra/test_contra.cpp         the float/flow contradiction latch, on its own
    ├── test_report/test_report.cpp         report_build() and response_parse()
    ├── test_net/test_netfsm.cpp            the FSM, the AT budget, the retry policy, the ack cycle
    ├── test_cart/test_cart.cpp             position, homing, stalls, the park
    ├── test_sensors/test_sensors.cpp       the mux discipline, the canary, I2C health, the leak watch
    ├── test_cli/test_cli.cpp               the line reader, the two command sets, `status`, the dose summary line
    └── test_device/test_device.cpp         DEVICE ONLY (task 28): the four checks that cannot be
                                            simulated. Never runs under [env:native].
```

## Drops

The spec stages this work into three drops (spec §11). Every task below carries its drop; the tree builds and its tests pass at the end of every task, so the drop boundaries are release points, not merge points.

- **Drop 1 — tasks 1 to 13.** `platformio.ini` and the native link gate, `pins.h`, `config.h`, seam 1 (`hal.h`, `hal_sim.cpp`, `hal_uno.cpp`), the `.noinit` block, `safety_tick()`/`safety_wait_ms()`, `pulses`, `sensors`, `lib/Screen`, `ui`, `cli` with the bench commands (`i2c`, `mux`, `hall`, `flow`, `status`), `main.cpp`, and every `make check` invariant that is decidable on a drop-1 tree. **Unblocks bring-up steps 0 to 3** — the boot banner read before 12 V goes onto COM, the bus scan, the mux sweep and the hall stream.
- **Drop 2 — tasks 14 to 20.** The cart that counts screw pulses, the float debounce and the dry latch, `cli_stop_requested()`, `dose_run()` entire — the refusal ladder, both caps, the target arithmetic and the two pump writes — the dose loop's abort rules, the float/flow contradiction latch, and the bring-up console (`servo`, `home`, `goto`, `pump`, `calib`, `cal`, `hang`, `noinit pattern`, `clear contra`) with the per-dose summary line. **Unblocks bring-up steps 4a to 7d.** THIS IS THE SAFETY SPINE: if the appetite runs short, what gives is drop 3's diagnostic channels and drop 1's UI coarsening — never a guard, never the latch, never a `make check` grep.
- **Drop 3 — tasks 21 to 30.** Seam 2 (`link.h`, `link_fake.cpp`, `link_wifi.cpp`), `report_build()` and `response_parse()`, the report FSM and its HTTP framing, the retry policy and link poisoning, `exec.cpp`'s ack cycle and park, the on-device test environment, the sim binary's fault-injection console, and the remaining `make check` invariants plus the documentation this work owes the other repos. **Unblocks bring-up step 7e and the 48-hour unattended run.**

## Tasks

---

### Task 1: Project skeleton and the native link gate

**Drop 1.**

**Files:**
- Create: `platformio.ini` (rewrite of the existing 17-line file), `Makefile` (rewrite of the existing 24-line file), `tools/check.sh`, `include/secrets.h.example`, `include/pins.h` (stub: the polarity `#error` only), `test/support/harness.h`, `test/test_dose/test_dose.cpp`
- Modify: `src/main.cpp` (rewrite the existing 80-line sketch to a bare `setup`/`loop`), `.gitignore`
- Test: `test/test_dose/test_dose.cpp`

**Interfaces:**
- Consumes: nothing. This is the first task.
- Produces:
  - Five PlatformIO environments exactly as spec §10 prints them — `[env]` carrying only `build_flags = -Wall -Wextra`; `[env:uno_r4_wifi]` (the unattended binary) with `platform = renesas-ra@1.6.0`, `framework = arduino`, `board = uno_r4_wifi`, `-DPB_RELAY_ACTIVE_HIGH`, `-DPB_REPORT_POS_UNKNOWN=1`, a commented `-DPB_DOSE_BY_TIME=1`, `build_src_flags = -Wformat-signedness`, `build_src_filter = +<*> -<hal_sim.cpp> -<link_fake.cpp>`, `monitor_speed = 115200`, three `lib_deps`; `[env:uno_r4_wifi_bringup]` extending it with `-DPB_BRINGUP=1`; `[env:uno_r4_wifi_test]` extending bringup with `test_framework = unity`, `test_build_src = yes`, `build_src_filter = +<*> -<link_fake.cpp>`; `[env:uno_r4_wifi_sim]` extending bringup with `-DPB_SIM=1 -DPB_SIM_CLI=1 -DPB_CONTROLLER='"bench1sim"'`, `build_src_filter = +<*> -<hal_uno.cpp>`, `lib_ignore = Network`; `[env:native]` with `platform = native`, `test_framework = unity`, `test_build_src = yes`, `build_src_filter = +<*> -<main.cpp> -<hal_uno.cpp>`, `lib_ignore = Network, Screen, Servo, Arduino_Sensorkit, LiquidCrystal_I2C`.
  - **Four one-flag variants of `[env:native]`**, each `extends = env:native` plus exactly one flag, so that every suite this plan compiles twice has a named environment to be compiled twice *in* — `native_bench` (`-UPB_BRINGUP`, task 20's bench-vs-bringup cases), `native_cal` (`-DPB_PULSES_PER_GATE=1450`, task 14's calibrated arm), `native_measured` (`-DPB_ML_PER_S_MEASURED=30`, task 17's two cap-clamp cases) and `native_nosimcli` (`-UPB_SIM_CLI`, task 29's absence case). **This settles the assembly note's open question in favour of named environments; no task in this plan uses `PLATFORMIO_BUILD_FLAGS`.** `native_nosimcli` undefines `PB_SIM_CLI` and NOT `PB_SIM`, because `PB_SIM` also gates every `hal_*` body in `hal_sim.cpp` (task 28) and `-UPB_SIM` would leave the host suite linking against no HAL at all.
  - `Makefile` targets `all upload monitor test sim bringup check calib clean compiledb`, monitor at 115200.
  - `tools/check.sh`: a runnable script with `fail()`, `count()`, `files()`, `expect()` helpers and an exit code, carrying no greps yet (task 13 fills it).
  - `include/secrets.h.example`: `WIFI_SSID`, `WIFI_PASS` (`const char[]`), `HTTP_PORT` (`const int`), `HOST_NAME`, `BUTLER_TOKEN` (`const char[]`), `PB_CONTROLLER` (a string-literal macro).
  - `test/support/harness.h`: a HEADER (`test/support/` must never become a suite directory with no runner — spec §10), declaring `pb_test_setup()` / `pb_test_teardown()`; task 3 fills it.

**Spec sections to read in full before starting:** "Read this first" item 7; §10 in full; §11's tree; §16.2.

---

1. - [ ] Write the failing test that proves the host runner links at all. This is spec §0's thirty-minute experiment: everything downstream depends on the answer.

   ```cpp
   /* test/test_dose/test_dose.cpp
      The dose suite. Today it carries one case: that a native Unity runner links and runs.
      Tasks 3-6 and 15-19 fill it with the pump-pin, watchdog and refusal-ladder cases. */
   #include <unity.h>

   void setUp(void) {}
   void tearDown(void) {}

   static void test_the_native_runner_links_and_runs(void) {
     /* PB_CONTROLLER comes from [env:native]'s build_flags, so this also proves the flag
        reached the compiler. */
     TEST_ASSERT_EQUAL_STRING("test1", PB_CONTROLLER);
   }

   int main(void) {
     UNITY_BEGIN();
     RUN_TEST(test_the_native_runner_links_and_runs);
     return UNITY_END();
   }
   ```

2. - [ ] Run it and watch it fail for the right reason — there is no `native` environment yet:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native
   ```
   expected, verbatim:
   ```
   UnknownEnvNamesError: Unknown environment names 'native'. Valid names are 'uno_r4_wifi'
   ```

3. - [ ] Write `platformio.ini` in full. Three things here are load-bearing and each has cost someone a day: `platform`/`framework`/`board` are in `[env:uno_r4_wifi]` and **never** in `[env]` (a global `framework = arduino` makes `pio test -e native` abort before compiling anything); `[env]` carries no `-std=gnu++*` (the renesas-ra builder already supplies it, and one in `build_flags` warns once per core `.c` file); and `test_build_src = yes` appears in both test envs or every suite fails on unresolved symbols.

   ```ini
   ; NOTE: the global [env] section carries NO `platform` and NO `framework`. PlatformIO
   ; inherits [env] into EVERY environment, and BuildFrameworks then exits with "Please
   ; specify `board`" for [env:native], which has none -- i.e. a global framework kills
   ; this project's own first gate. Verified on PlatformIO Core 6.1.19. See spec §10.
   ;
   ; Nor does [env] carry -std=gnu++17: builder/frameworks/arduino.py:100,123 already
   ; supplies -std=gnu++17 for C++ and -std=gnu11 for C, and a -std=gnu++ in build_flags
   ; lands in SCons CCFLAGS and warns once per core .c file on every build (spec §1).

   [env]
   build_flags = -Wall -Wextra

   [env:uno_r4_wifi]                                  ; THE UNATTENDED BINARY
   platform = renesas-ra@1.6.0
   framework = arduino
   board = uno_r4_wifi
   build_flags =
       ${env.build_flags}
       -DPB_RELAY_ACTIVE_HIGH        ; SET BY BRING-UP 4a. No default exists; pins.h errors out.
       -DPB_REPORT_POS_UNKNOWN=1     ; SHIPS DEFINED. Flipping it turns on watering -- spec §4.6.
   ;   -DPB_DOSE_BY_TIME=1           ; 7b's stated fallback (spec §6). Uncomment ONLY after
   ;                                 ; PB_ML_PER_S_MEASURED is committed non-zero; config.h
   ;                                 ; errors out if it is not.
   build_src_flags = -Wformat-signedness   ; a %d against a uint32_t t= is a first-report 400 (§9)
   build_src_filter = +<*> -<hal_sim.cpp> -<link_fake.cpp>
   monitor_speed = 115200
   lib_deps =
       arduino-libraries/Servo@^1.2.2
       arduino-libraries/Arduino_Sensorkit@^1.4.0
       marcoschwartz/LiquidCrystal_I2C@^1.1.4

   [env:uno_r4_wifi_bringup]                          ; BRING-UP 0-7d ONLY. Never left running.
   extends = env:uno_r4_wifi
   build_flags = ${env:uno_r4_wifi.build_flags} -DPB_BRINGUP=1

   [env:uno_r4_wifi_test]                             ; spec §9's on-device suites. Never left running.
   extends = env:uno_r4_wifi_bringup
   test_framework = unity
   test_build_src = yes                               ; or src/report.cpp never links into a test
   build_src_filter = +<*> -<link_fake.cpp>           ; keep hal_uno.cpp (the real board) AND
                                                      ; hal_sim.cpp (harness.h is a fixture over it)

   [env:uno_r4_wifi_sim]
   extends = env:uno_r4_wifi_bringup
   ; PB_SIM gates the fake rig's hal_* bodies; PB_SIM_CLI gates only the `sim ...` console
   ; family. They are two flags because task 29 must compile the console family OUT while
   ; leaving the HAL in -- see [env:native_nosimcli].
   build_flags = ${env:uno_r4_wifi_bringup.build_flags} -DPB_SIM=1 -DPB_SIM_CLI=1 -DPB_CONTROLLER='"bench1sim"'
   build_src_filter = +<*> -<hal_uno.cpp>
   lib_ignore = Network

   [env:native]
   platform = native
   test_framework = unity
   ; -DPB_RELAY_ACTIVE_HIGH is NOT in spec §10's printed [env:native], but pins.h (§2.2)
   ; errors out unless a polarity is defined and every native TU includes pins.h. It is inert
   ; on the host: PB_PUMP_OWNER is defined only by hal_uno.cpp, which native does not compile,
   ; so PIN_PUMP_EN and the PUMP_*_PFS_LEVEL macros do not exist here at all.
   build_flags = ${env.build_flags} -std=gnu++17 -DPB_NATIVE=1 -DPB_SIM=1 -DPB_SIM_CLI=1 -DPB_BRINGUP=1 -DPB_RELAY_ACTIVE_HIGH -DPB_CONTROLLER='"test1"' -I include
   test_build_src = yes
   build_src_filter = +<*> -<main.cpp> -<hal_uno.cpp>
   lib_ignore = Network, Screen, Servo, Arduino_Sensorkit, LiquidCrystal_I2C

   ; ---- The four one-flag variants of [env:native]. Four suites in this plan must be
   ; compiled a SECOND time under one different flag, and a named environment is how they
   ; do it: no task uses PLATFORMIO_BUILD_FLAGS. Each is three lines and inherits
   ; platform/test_framework/test_build_src/build_src_filter/lib_ignore from env:native.
   ; A later -U or -D wins over an earlier one on the same command line, which is what
   ; makes the -U forms work. ----

   [env:native_bench]                                 ; task 20: the bench-vs-bringup cases
   extends = env:native
   build_flags = ${env:native.build_flags} -UPB_BRINGUP

   [env:native_cal]                                   ; task 14: the calibrated arm of the #if
   extends = env:native
   build_flags = ${env:native.build_flags} -DPB_PULSES_PER_GATE=1450

   [env:native_measured]                              ; task 17: the two cap-clamp cases
   extends = env:native
   build_flags = ${env:native.build_flags} -DPB_ML_PER_S_MEASURED=30

   [env:native_nosimcli]                              ; task 29: `sim ...` is not a command here
   extends = env:native
   ; -UPB_SIM_CLI, NOT -UPB_SIM: PB_SIM also gates every hal_* body in hal_sim.cpp (task 28),
   ; so -UPB_SIM would leave this suite linking against no HAL at all.
   build_flags = ${env:native.build_flags} -UPB_SIM_CLI
   ```

4. - [ ] Run the gate and watch it go green:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native
   ```
   expected:
   ```
   test/test_dose/test_dose.cpp:5: test_the_native_runner_links_and_runs	[PASSED]
   ----------------- native:test_dose [PASSED] Took 2.35 seconds -----------------
   ================== 1 test cases: 1 succeeded in 00:00:02.350 ==================
   ```
   **If this cannot be made to link, stop and report.** The two seams are the whole architecture and this is the experiment that validates them.

5. - [ ] Commit the gate.

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && git add platformio.ini test/test_dose/test_dose.cpp && git commit -m "platformio.ini: five environments, and the native gate that proves the seams

   platform/framework/board stay out of [env]: PlatformIO inherits [env] into every
   environment and [env:native] has no board, so a global framework aborts pio test
   before it compiles anything. [env:native] gains -DPB_RELAY_ACTIVE_HIGH, which spec
   §10 does not print, because pins.h errors out without a polarity and every native
   translation unit includes it; it is inert on the host.

   Four one-flag variants of [env:native] are added beyond spec §10's five: native_bench,
   native_cal, native_measured and native_nosimcli. Four suites in this plan must be
   compiled a second time under one different flag, and a named environment is how they do
   it -- PLATFORMIO_BUILD_FLAGS is used nowhere. PB_SIM_CLI is split out of PB_SIM for the
   same reason: PB_SIM gates the fake rig's HAL bodies, so it cannot be the flag that
   compiles the sim console away."
   ```

6. - [ ] Reduce `src/main.cpp` to a bare sketch so `pio run -e uno_r4_wifi` builds without `secrets.h` and without the old `std::map`/`Manifold`/`Screen` sketch. Note there is no Arduino header here and there must never be one: spec §9 greps for it outside `src/hal_uno.cpp`, `lib/Network` and `lib/Screen`. `setup()` and `loop()` are declared in that header but a definition does not need the declaration.

   ```cpp
   /* src/main.cpp -- DEVICE ONLY. [env:native] filters this file out (no main()).
      Task 12 fills in the setup order of spec §2.5/§5/§12 and the loop of spec §3.
      No Arduino header here, ever: spec §9 allows it only in hal_uno.cpp, lib/Network
      and lib/Screen. Everything this file needs arrives through include/hal.h. */

   void setup(void) {}
   void loop(void) {}
   ```

7. - [ ] Confirm the device build is green, then commit:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio run -e uno_r4_wifi 2>&1 | tail -5
   ```
   expected: `SUCCESS`, and no `secrets.h` in the compile log.
   ```
   cd /Users/jcanton/projects/plant-butler/firmware && git add src/main.cpp && git commit -m "main.cpp: a bare sketch, so the device build stands on its own again

   The old sketch's std::map, Manifold::test() and both Screen* users go with it; task 12
   writes the real setup order. lib/Manifold and lib/Network are still on disk and still
   compile, but nothing includes them any more, so the LDF does not build them."
   ```

8. - [ ] Add the pin-contract stub and the secrets example. `pins.h` carries only the polarity gate today; task 2 fills in the rest. There is **no default polarity**: a board cannot be flashed before someone has read the relay module (spec §2.2).

   ```c
   /* include/pins.h -- the wiring contract as constants.
      Today: the polarity gate only. Task 2 adds the pin numbers and the PB_PUMP_OWNER gate. */
   #pragma once

   #if !defined(PB_RELAY_ACTIVE_LOW) && !defined(PB_RELAY_ACTIVE_HIGH)
   #  error "Relay polarity is unknown until bring-up 4a. Define PB_RELAY_ACTIVE_LOW or \
   PB_RELAY_ACTIVE_HIGH in platformio.ini build_flags after you have READ THE MODULE."
   #endif
   ```

   ```c
   /* include/secrets.h.example -- copy to include/secrets.h (gitignored) and fill in.
      Six names, no more. spec §11. */
   #pragma once

   const char WIFI_SSID[]    = "your-ssid";
   const char WIFI_PASS[]    = "your-passphrase";
   const int  HTTP_PORT      = 8000;
   const char HOST_NAME[]    = "butler.lan";
   const char BUTLER_TOKEN[] = "replace-me";

   /* A string literal, not a char[]: report.cpp asserts sizeof(PB_CONTROLLER) > 1 at compile
      time, because an empty c= is a permanent 400 (butler.py parse_report's "no c= in the
      report" branch). The sim and native envs set it from build_flags instead. */
   #ifndef PB_CONTROLLER
   #  define PB_CONTROLLER "bench1"
   #endif
   ```

   **Then create the real, gitignored header from it — nothing later in this plan does, and
   without it nothing builds.** `[env:native]` carries `test_build_src = yes` and filters out
   only `main.cpp` and `hal_uno.cpp`, so it compiles `src/netfsm.cpp`, which includes
   `secrets.h` from task 24 onward; `pio run -e uno_r4_wifi` compiles `link_wifi.cpp`,
   `cli.cpp`, `main.cpp` and `report.cpp`, all four of which include it. A clean clone that
   skips this line fails from task 24 on with `fatal error: secrets.h: No such file or
   directory`, on the host and on the board alike.

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && cp include/secrets.h.example include/secrets.h && printf '%s\n' "--- edit include/secrets.h now: real SSID, real passphrase, the backend's host and port, the token butler.py expects, and this board's PB_CONTROLLER ---"
   ```
   `include/secrets.h` is the ONE file in this plan that a task creates and no commit ever
   contains (step 9 gitignores it). It is the operator's file, not the repository's: a second
   machine building this tree copies the example again rather than pulling it. Leave
   `PB_CONTROLLER` at its `#ifndef` default until the bench board has a name; the host suites
   and the sim binary override it from `build_flags` regardless.

9. - [ ] Make the gitignore explicit about the real path and about the sim artefact, then commit:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && printf 'include/secrets.h\nfirmware-SIM.bin\n' >> .gitignore && git add include/pins.h include/secrets.h.example .gitignore && git commit -m "pins.h: the relay polarity has no default, and secrets.h gains a checked-in example

   A board cannot be flashed before someone has read the relay module (spec §2.2). The
   example carries the six names a fresh clone needs, PB_CONTROLLER among them."
   ```

10. - [ ] Write the `Makefile`. **Recipes are indented with tabs** (this repo has already lost a day to that). `monitor` is 115200 project-wide: at 9600 an 80-character line of the `hall` stream is 83 ms of blocking (spec §7, §16.2).

    ```make
    PIO ?= pio
    MONITOR_SPEED = 115200

    all:
    	$(PIO) run -e uno_r4_wifi

    upload:
    	$(PIO) run -e uno_r4_wifi -t upload

    monitor:
    	$(PIO) device monitor -b $(MONITOR_SPEED)

    test:
    	$(PIO) test -e native

    bringup:
    	@echo "BRING-UP BUILD - pump/cal/servo/home/goto/hang are compiled in. This is NOT the binary left running."
    	$(PIO) run -e uno_r4_wifi_bringup -t upload

    sim:
    	@echo "SIM BUILD - the 12 V brick must be unplugged"
    	$(PIO) run -e uno_r4_wifi_sim -t upload

    calib:
    	@echo "BRING-UP 7b: upload the bringup binary, then type calib in the monitor"
    	$(PIO) run -e uno_r4_wifi_bringup -t upload
    	$(PIO) device monitor -b $(MONITOR_SPEED)

    check:
    	./tools/check.sh

    clean:
    	$(PIO) run -t clean

    compiledb:
    	$(PIO) run -t compiledb

    .PHONY: all upload monitor test bringup sim calib check clean compiledb
    ```

11. - [ ] Write the `tools/check.sh` skeleton — helpers and an exit code, no greps yet, and a header saying so, so nobody reads the short list as the whole contract. Use POSIX character classes rather than `\b`/`\s`: this repo is built on macOS, where `/usr/bin/grep` is BSD grep.

    ```bash
    #!/usr/bin/env bash
    # tools/check.sh -- the mechanical invariants of the bench-sketch spec §9, as greps.
    #
    # THIS IS NOT THE WHOLE CONTRACT YET. The invariants that need drop-2 and drop-3 code
    # (the safety.h/dose_run/hal_pump_write greps over netfsm and ui, the single dose_run(
    # in cli.cpp, the %d grep over report.cpp and netfsm.cpp, the preprocessed-cli grep and
    # the safety.o/hal_uno.o hash equality) are added by task 30. See
    # docs/superpowers/specs/2026-09-03-bench-sketch-design.md §9.
    #
    # Patterns use POSIX classes, never \b or \s: BSD grep is the default on macOS.
    set -uo pipefail
    cd "$(dirname "$0")/.."

    SCAN=(include src lib test)      # deliberately NOT tools/: this file contains the patterns
    fails=0

    fail() { printf 'FAIL  %s\n' "$*" >&2; fails=$((fails + 1)); }
    ok()   { printf 'ok    %s\n' "$*"; }

    # count <pattern> [paths...] -> matching LINES
    count() { local pat="$1"; shift; grep -rEn "$pat" "$@" 2>/dev/null | wc -l | tr -d ' '; }
    # files <pattern> [paths...] -> matching FILES
    files() { local pat="$1"; shift; grep -rlE "$pat" "$@" 2>/dev/null | wc -l | tr -d ' '; }
    # expect <want> <got> <description>
    expect() { if [ "$2" = "$1" ]; then ok "$3 ($1)"; else fail "$3: expected $1, found $2"; fi; }

    # ---- invariants land here (task 13, then task 30) ----

    if [ "$fails" -gt 0 ]; then
      printf '\n%s invariant(s) FAILED\n' "$fails" >&2
      exit 1
    fi
    printf '\nall invariants hold\n'
    exit 0
    ```

12. - [ ] Add the harness header stub — a header, never a suite directory:

    ```c
    /* test/support/harness.h -- the Unity fixture over hal_sim.
       This is a HEADER. test/support/ must not become a suite directory with no runner
       (spec §10). Task 3 fills it in; task 19 adds the contradiction bracket, task 24
       pb_net_passes(), and task 28 the on-device arm. */
    #pragma once
    #include <unity.h>

    static inline void pb_test_setup(void)    {}
    static inline void pb_test_teardown(void) {}
    ```

    The full member list this header ends up carrying, so that no later task silently drops
    one when it rewrites the file: `pb_test_setup`, `pb_test_teardown`, `pb_advance`,
    `pb_count`, `pb_expect_no_feed_between` (task 3); `pb_latch_contra` (task 19);
    `pb_net_passes` (task 24). Task 28 adds a device arm **around** the three that differ
    between host and board, and deletes none of the others. There is deliberately no
    `pb_begin_fake_dose` / `pb_end_fake_dose` pair: the two cases that need a dose in flight
    (task 7's I2C-recovery pair) call `safety_set_dosing()` directly, which is the seam task 5
    exists to provide, and a second spelling of the same thing is how one of them goes stale.

13. - [ ] Prove all five environments resolve their options, then commit:

    ```
    cd /Users/jcanton/projects/plant-butler/firmware && chmod +x tools/check.sh && make check && pio test -e native && pio test -e native_bench && pio test -e native_cal && pio test -e native_measured && pio test -e native_nosimcli && pio run -e uno_r4_wifi -e uno_r4_wifi_bringup 2>&1 | tail -3 && pio run -e uno_r4_wifi_sim -e uno_r4_wifi_test -t idedata 2>&1 | tail -3
    ```
    `make check` prints `all invariants hold`; the five host environments run task 1's one case; the two device builds print `SUCCESS`. The sim and test envs will not fully build until their sources exist (`hal_sim.cpp`, `link_fake.cpp`) — what this step proves is that PlatformIO accepts their options; a missing-file error is expected and fine, an *option* error is not.
    ```
    cd /Users/jcanton/projects/plant-butler/firmware && git add Makefile tools/check.sh test/support/harness.h && git commit -m "Makefile at 115200, and the skeleton of tools/check.sh

    Targets: all upload monitor test sim bringup check calib clean compiledb. make sim and
    make bringup warn before they upload. check.sh carries the helpers and the exit code;
    task 13 fills in the drop-1 greps and task 30 the rest. Recipes are tab-indented."
    ```

---

---

### Task 2: pins.h and config.h — the wiring contract and every constant with its citation

**Drop 1.**

**Files:**
- Create: `include/config.h`
- Modify: `include/pins.h` (task 1 left a stub carrying only the relay-polarity `#error`; this task grows it into the whole contract), `platformio.ini` (one line: `[env:native]`'s `build_flags`), `test/test_dose/test_dose.cpp` (add a `test_config` group beside task 1's link-gate case)
- Test: `test/test_dose/test_dose.cpp`

**Interfaces:**

*Consumes.* Nothing at compile time. From task 1's `platformio.ini`: `[env:uno_r4_wifi]` carries `-DPB_RELAY_ACTIVE_HIGH` and `-DPB_REPORT_POS_UNKNOWN=1`; `[env:native]` carries `-DPB_NATIVE=1 -DPB_SIM=1 -DPB_SIM_CLI=1 -DPB_BRINGUP=1 -DPB_CONTROLLER='"test1"' -I include`. **`-DPB_SIM_CLI=1` is part of that line and step 7 must not drop it**: it gates the whole `sim ...` console family (task 29), and `[env:native_nosimcli]`'s `-UPB_SIM_CLI` proves nothing if the flag was never defined.

*Produces.*
- `include/pins.h`: `PIN_FLOW` 2, `PIN_HALL_SCREW` 3, `PIN_HALL_FLOAT` 5, `PIN_SERVO`, `PIN_LED` 13, `PIN_MUX_ADC` 14, `I2C_ADDR_EXPANDER` 0x20, `I2C_ADDR_LCD` 0x27, `I2C_ADDR_OLED` 0x3C; `PIN_PUMP_EN` 6 and `PUMP_ON_PFS_LEVEL`/`PUMP_OFF_PFS_LEVEL` defined **only** under `#ifdef PB_PUMP_OWNER`; an unconditional `#error` when neither `PB_RELAY_ACTIVE_LOW` nor `PB_RELAY_ACTIVE_HIGH` is defined.
- `include/config.h`: every macro of spec §7, verbatim, each with the citation comment that justifies it; plus an `#error` when `PB_DOSE_BY_TIME` is defined and `PB_ML_PER_S_MEASURED == 0`.

Everything from task 3 onward includes one or both of these headers. No other file may hold a pin number or a tunable.

---

1. [ ] **Write the three failing config cases.** Append to `test/test_dose/test_dose.cpp`, above its `main()`, and register them in `main()` with `RUN_TEST`:

```c
#include "config.h"

/* spec §7: RL_16384 * PR_8192 / (PCLKB/1000), PCLKB = 24 MHz (bsp_clock_cfg.h:8,14).
   Re-derived here so a copied-in constant cannot drift from its own arithmetic. */
static void test_the_watchdog_grant_arithmetic_matches_the_constant(void) {
  TEST_ASSERT_EQUAL_UINT32(5592u, (16384u * 8192u) / (24000000u / 1000u));
  TEST_ASSERT_EQUAL_UINT32((16384u * 8192u) / (24000000u / 1000u), (uint32_t)PB_WDT_GRANTED_MS);
  TEST_ASSERT_EQUAL_UINT32(58u, (uint32_t)PB_WDT_PROBE_MIN_COUNTS);
  /* the probe window must be two orders of magnitude inside the grant (§2.5) */
  TEST_ASSERT_TRUE(PB_WDT_PROBE_MS * 100u < PB_WDT_GRANTED_MS);
}

/* spec §7: the body is assembled into PB_BODY_CAP bytes; `c=` plus its value is the only
   term not counted in PB_BODY_WORST_FIXED. An empty c= is a permanent 400 (butler.py
   parse_report: c must be non-empty). */
static void test_the_body_worst_case_sum_fits_the_body_cap(void) {
  TEST_ASSERT_TRUE(sizeof(PB_CONTROLLER) > 1u);
  TEST_ASSERT_TRUE(sizeof(PB_CONTROLLER) + 2u + PB_BODY_WORST_FIXED <= PB_BODY_CAP);
  TEST_ASSERT_TRUE(PB_HDR_FIXED + PB_BODY_CAP <= PB_TX_CAP);
}

/* spec §4.6: this ships DEFINED, so no backend water command is ever queued until a
   deliberate later commit. */
static void test_the_going_live_flag_ships_defined(void) {
  TEST_ASSERT_EQUAL_INT(1, PB_REPORT_POS_UNKNOWN);
}
```

2. [ ] **Run it and watch it fail on the missing header.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose
```

Expected: `test/test_dose/test_dose.cpp:N:10: fatal error: config.h: No such file or directory` and `*** [.pio/build/native/test/test_dose/test_dose.o] Error 1`.

3. [ ] **Write `include/config.h`.** This is spec §7 transcribed whole — the citations are the deliverable, not decoration.

```c
/* include/config.h — every tunable and measured constant, with the citation that
   justifies it. A constant without one is a bug report waiting to be written.
   Source: docs/superpowers/specs/2026-09-03-bench-sketch-design.md §7. */
#pragma once

/* ---- fixed by the wiring package ---- */
#define PB_OUTLETS               5
#define PB_CHANNELS              6      /* C0-C4 moisture, C5 LDR */
#define PB_CANARY_CHANNEL       15      /* unwired by the mux table: the stuck-mux canary (§5) */
/* analogReadResolution(bits) does NOT change the hardware width: ANALOG_READ_HARDWARE_
   RESOLUTION_FIXED is defined (analog.cpp:599), the hardware is fixed at open time to
   BSP_FEATURE_ADC_MAX_RESOLUTION_BITS = 14 (:34-45), and adcConvert map()s hw -> requested
   (:495). So 14 is an identity map TODAY -- but the default requested value is 10 (:11), and
   a core bump that changed the fixed width would silently rescale every raw count on the
   wire. hal_begin() therefore ASSERTS analogReadResolution() == PB_ADC_BITS after setting it
   (:698), and `status` prints both the requested and the hardware width. */
#define PB_ADC_BITS             14

/* ---- the watchdog. PCLKB = 24 MHz (bsp_clock_cfg.h:8,14: HOCO 48 / PCLKB_DIV 2).
   RL_16384 * PR_8192 / (PCLKB/1000) = 16384*8192/24000 = 5592 ms (WDT.cpp:105-113).
   We use the wdt_cfg_t overload for stop_control = DISABLE (WDT.cpp:67, r_wdt_api.h:115-116),
   and that overload NEVER assigns _timeout (WDT.cpp:32-46), so getTimeout() would
   return 0 on a running dog. hal_wdt_granted() computes this number instead.
   The counter is a DOWN-counter at PCLKB/8192 = 2929.7 Hz = 2.93 counts/ms, which is
   what hal_wdt_alive() measures across an UNFED window (§2.5). ---- */
#define PB_WDT_GRANTED_MS     5592
#define PB_WDT_PROBE_MS         40     /* the ONE unfed window in the program. 40/5592 = 0.7%. */
#define PB_WDT_PROBE_MIN_COUNTS 58     /* half of 40 * 2929.7/1000 = 117: no false negative on
                                          jitter, no false positive on a frozen counter */

/* ---- the network. MODEM_TIMEOUT default is 10000 (Modem.h:12), nearly twice the
   WDT window. The worst net_poll() pass is CONNECT = 2 AT commands (_BEGINCLIENT +
   _CLIENTCONNECT), which holds ONLY because (a) sock_close() always ran in a PRIOR
   pass and left _sock == -1 (WiFiClient.cpp:31,217), (b) every error exit routes
   through the SOCK_CLOSE state instead of closing inline, and (c) sock_read() is a
   bare client.read() -- no available(), no connected(). See §3's per-pass table.
   2 * 1200 = 2400 < 5592, and 2400 + SLACK = 4400 is what setup() asserts. ---- */
#define PB_NET_STEP_MS        1200
#define PB_NET_SLACK_MS       2000     /* setup() asserts granted >= 2*STEP + SLACK */
#define PB_NET_DEADLINE_MS    5000     /* RECV: also the closed-socket detector, since the FSM
                                          never calls connected() (2 ATs, WiFiClient.cpp:224-238) */
#define PB_NET_BACKOFF_MS     { 2000, 4000, 8000, 16000, 30000 }
#define PB_RETRY_DEADLINE_MS 30000     /* << RETRY_WINDOW_S = 300 (butler.py RETRY_WINDOW_S).
                                          Measured on g_t_ms (RAW millis), never on g_t_wire. */

/* ---- the dose. Protocol ceilings MATCH butler.py: MAX_DOSE_ML 1000, MAX_CAP_S 60.
   The RIG ceiling is smaller, per DECISIONS #7's "a reservoir small enough that a full
   dump is a mop-up" -- and butler does NOT know about it, which is a going-live
   precondition (§4.6) and an owed backend change (§16.5.8). ---- */
#define PB_DOSE_MAX_ML        1000     /* == MAX_DOSE_ML: protocol parity */
#define PB_DOSE_RIG_MAX_ML     250     /* what dose_run() actually enforces */
#define PB_DOSE_CAP_MS_MAX   60000     /* == MAX_CAP_S * 1000 */
#define PB_DOSE_MIN_GAP_MS   10000     /* every caller. See §15.3 for why not 60 s. */
#define PB_BOOT_GAP_MS       10000     /* DECISIONS #5 "minimum gap since boot" */
#define PB_BOOT_HOME_MS      15000
#define PB_POS_RECHECK_MS     1000     /* live expander read inside the dose loop */
#define PB_COAST_MS           2000     /* impeller spin-down is not a leak */
/* cap_for(ml) = min(60, ml//FLOW_FLOOR_ML_S + 5) with FLOW_FLOOR_ML_S = 20 (butler.py) is a
   GUESS. At a real 30 ml/s, cap_for(500) authorises 1.8x the requested water, so the cap is
   not a bound exactly when the meter -- the thing it stands in for -- has failed. Once 7b
   measures the rate, this clamps the cap to 2x the requested millilitres. It is ALSO the
   constant -DPB_DOSE_BY_TIME=1 uses; there is no second ml/s constant (§6). */
#define PB_ML_PER_S_MEASURED     0     /* 7b fills this in; 0 == clamp disabled, status says so */
#define PB_CAP_SLACK_NUM         2
#define PB_CAP_SLACK_DEN         1
/* Delivered-vs-elapsed plausibility on the DOSE_OK path (§2.8): a dose that reaches its
   target in less than 1/4 of the time the measured rate says it needs is noise, not a fast
   pump. Armed only when PB_ML_PER_S_MEASURED > 0. */
#define PB_PLAUS_NUM             4
#define PB_PLAUS_DEN             1

/* ---- flow. GUESSES until bring-up 7b. YF-S401 nominal ~5880 pulses/L; the
   -0207 floor is 0.2 L/min and the -3507 ceiling 6 L/min = 588 pulses/s.
   `cal` sets pulses_per_l at runtime so 7b needs no reflash; 7b's numbers get
   COMMITTED here. ---- */
#define PB_PULSES_PER_L_DEFAULT 5880
#define PB_PULSES_PER_L_MIN     1000   /* `cal` and dose_run() both enforce this range */
#define PB_PULSES_PER_L_MAX    20000
#define PB_PRIME_MS_DEFAULT     3000
#define PB_PRIME_LONG_MS       15000   /* `prime` EXTENDS the window; it never removes it */
#define PB_PRIME_CAP_MS        20000   /* and caps the whole dose regardless of the typed ms */
#define PB_STALL_MS_DEFAULT     1500
#define PB_PRIME_MIN_PULSES        5
#define PB_FLOW_MIN_GAP_US       500   /* ISR reject; honest: only bites above 2 kHz */
#define PB_FLOW_MAX_HZ          1200   /* 2x the meter's 588/s ceiling -> DOSE_ABORT_NOISE */
#define PB_FLOW_IDLE_MAX_HZ        2   /* non-zero with the pump off -> DOSE_REFUSED_NOISE */
/* pulses_flow_rate()'s window, and it has to be STATED or the rate rules are unfalsifiable:
   at the ISR's own 2 kHz ceiling a 250 ml target (1250 pulses at cfg=5000) is reached in
   ~625 ms, so any estimator slower than that loses the race. 100 ms wins it 6x over. */
#define PB_FLOW_RATE_WINDOW_MS   100
#define PB_HANG_MS              3000   /* `pump <ms> hang`: run the dose this long, THEN stop
                                          feeding (bring-up 7c). PB_BRINGUP only. */

/* ---- the cart. Pulses-per-gate DOES NOT EXIST YET: today's Manifold is purely
   time-based, so there is nothing to port. Bring-up 6 measures it. Until then
   cart_goto() and cart_pos_known() are COMPILED OUT to hard false (§2.15). ---- */
#define PB_PULSES_PER_GATE       0     /* 6 fills this in; 0 == goto compiles to `return false` */
#define PB_PULSES_HOME_TO_1      0
#define PB_MOVE_CAP_MS       45000
#define PB_STALL_WINDOW_MS    2500
#define PB_SERVO_CAP_MS      10000
#define PB_SCREW_MIN_GAP_US   2000

/* ---- the float ---- */
#define PB_FLOAT_OK_SAMPLES      3
#define PB_FLOAT_SAMPLE_MS      20
/* After this many CONSECUTIVE DOSE_REFUSED_FLOAT results, the report forces float=0 and
   err=float regardless of the report-time debounce, and water_rules goes dark. Cleared by
   any granted dose. Without it a float flapping at the waterline can grant in the report
   and refuse in the dose -- separate samples, minutes apart -- and the acked refusal sets
   the pot's cooldown and pages HIGH, forever (§2.10). */
#define PB_FLOAT_FLAP_LIMIT      3

/* ---- I2C. TwoWire's transfer timeout is a fixed 1000 ms (Wire.cpp:194, private,
   no setter); TwoWire::flush() (Wire.cpp:833) spins forever and is never called. ---- */
#define PB_I2C_FAIL_LIMIT        3
#define PB_I2C_BACKOFF_MS     5000
#define PB_I2C_RECOVER_CLOCKS    9     /* a FIXED count, never "until SDA releases" */

/* ---- going live. See §4.6. Ships DEFINED; flipping it turns on backend watering. ---- */
#ifndef PB_REPORT_POS_UNKNOWN
#  define PB_REPORT_POS_UNKNOWN  1
#endif

/* ---- buffers. Main stack is 1024 B (bsp_cfg.h:26) -- every one of these is
   FILE-STATIC, never a stack local.

   PB_BODY_WORST_FIXED is the report body's worst case with `c=` and its value EXCLUDED,
   summed term by term at the maximum width the grammar permits, with every diagnostic
   clamped to six digits (§4.1):

       t=4294967295                        13
       six wired channels, chN=16383     6*10 =  60   (14-bit ADC: 5 digits)
       ten diagnostics, chNNN=999999    10*13 = 130   (clamped; unclamped it is 10*16 = 160)
       float=1                              8
       pos=unknown                         12
       ack=4294967295                      15
       flow_ml=1000                        13   (bounded by PB_DOSE_MAX_ML)
       err=resetmid                        13   (longest token is 8 chars)
                                          ---
                                          264, rounded up to 288

   The old PB_BODY_CAP of 288 therefore had NO margin at all: any PB_CONTROLLER longer
   than six characters overflowed, and the failure mode is err=txcap with the report
   DROPPED -- a silent reporting blackout, not a 400. netfsm.cpp static_asserts
   sizeof(PB_CONTROLLER) + 2 + PB_BODY_WORST_FIXED <= PB_BODY_CAP, and that
   sizeof(PB_CONTROLLER) > 1 (an empty c= is a permanent 400). ---- */
#define PB_BODY_WORST_FIXED    288
#define PB_BODY_CAP            384
#define PB_DIAG_CLAMP       999999     /* every chN diagnostic is min(v, this) on the way out:
                                          chN must be < MAX_RAW = 2**31, and a storming D2
                                          pushes ch205 past 2**31 in ~12.4 days */
#define PB_TX_CAP              768     /* PB_HDR_FIXED + HOST_NAME + BUTLER_TOKEN + PB_BODY_CAP */
#define PB_RX_CAP              256
#define PB_HDR_FIXED           128     /* the fixed part of the request line + headers */
#define PB_LINE_CAP             96
#define PB_STACK_MARGIN       2048     /* the break must stay this far below __StackLimit.
                                          _sbrk is UNCHECKED (§12), so this is the only bound
                                          that exists; crossing it latches err=heap. */

/* ---- .noinit ---- */
#define PB_NOINIT_MAGIC   0x50423031u  /* "PB01" */

/* ---- §6's seconds fallback. A by-time dose against an unmeasured rate is an unbounded
   run in a costume, so the flag and the measurement are welded together. ---- */
#if defined(PB_DOSE_BY_TIME) && (PB_ML_PER_S_MEASURED == 0)
#  error "PB_DOSE_BY_TIME needs PB_ML_PER_S_MEASURED committed non-zero by bring-up 7b \
(spec §6). Measure the rate, commit it in config.h, THEN uncomment the build flag."
#endif
```

4. [ ] **Run the suite and watch the three cases pass.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose
```

Expected: `test_the_watchdog_grant_arithmetic_matches_the_constant:PASS`, `test_the_body_worst_case_sum_fits_the_body_cap:PASS`, `test_the_going_live_flag_ships_defined:PASS`, alongside task 1's link-gate case, and a `4 Tests 0 Failures 0 Ignored / OK` summary.

5. [ ] **Write `include/pins.h`** — the whole contract, replacing task 1's `#error`-only stub. `PB_PUMP_OWNER` appears in exactly one file in the tree (`src/hal_uno.cpp`, task 8) and this header is what makes that gate mean something.

```c
/* include/pins.h — the wiring contract as constants. Source of truth for the numbers:
   cad/wiring/nets.py and the pin table it generates in cad/wiring/README.md.
   Spec §2.2. Nothing outside this header may hold a pin number. */
#pragma once

/* There is NO default relay polarity: a board cannot be flashed before someone has READ
   THE MODULE (bring-up 4a). `status` prints the compiled level so 4a can confirm it. */
#if !defined(PB_RELAY_ACTIVE_LOW) && !defined(PB_RELAY_ACTIVE_HIGH)
#  error "Relay polarity is unknown until bring-up 4a. Define PB_RELAY_ACTIVE_LOW or \
PB_RELAY_ACTIVE_HIGH in platformio.ini build_flags after you have READ THE MODULE."
#endif
#if defined(PB_RELAY_ACTIVE_LOW) && defined(PB_RELAY_ACTIVE_HIGH)
#  error "Define exactly one of PB_RELAY_ACTIVE_LOW / PB_RELAY_ACTIVE_HIGH."
#endif

/* ---- direct pins (cad/wiring/README.md pin table) ---- */
#define PIN_FLOW           2   /* YF-S401 pulse. 1 k series at the board; INPUT_PULLUP is the
                                  software half of the missing pull, R4 is the other (§2.14) */
#define PIN_HALL_SCREW     3   /* WPSE313 screw hall, interrupt pin: an edge missed while a mux
                                  sits on another channel is lost cart position */
#define PIN_HALL_FLOAT     5   /* WPSE313 float hall, 10 k pull-up R2. Direct pin, NOT the
                                  expander: shortest path for the safety input */
#define PIN_SERVO          9   /* SG90 continuous servo, manifold 1. A servo needs its 50 Hz
                                  train unbroken, so it is a direct pin, never the expander.
                                  SEE THE NOTE IN STEP 6 BEFORE YOU TRUST THIS NUMBER. */
#define PIN_LED           13   /* NOT FROM THE SPEC, and not from cad/wiring either: it is the
                                  board's own LED_BUILTIN. §8's sim double-blink is the only
                                  consumer (task 29), and it must not spell `LED_BUILTIN` —
                                  that name comes from the Arduino header, which task 29's
                                  callers may not include (spec §9). Named here so exactly one
                                  file in the tree holds the number. */
#define PIN_MUX_ADC       14   /* A0 == PIN_A0 == 14u
                                  (framework-arduinorenesas-uno/variants/UNOWIFIR4/
                                   pins_arduino.h:15). MUX1 SIG lands here. */

/* ---- I2C addresses (A4/A5; the bus that gates the pump) ---- */
#define I2C_ADDR_EXPANDER  0x20  /* PCF8575, A0-A2 low. P0..P3 = MUX1 S0..S3,
                                    P4 = HALL_HOME (input, 10 k pull-up R3) */
#define I2C_ADDR_LCD       0x27  /* LCD1602 backpack */
#define I2C_ADDR_OLED      0x3C  /* SensorKit OLED (u8x8) */

/* ---- D6, and only for its owner ----
   PIN_PUMP_EN exists ONLY in the translation unit that defines PB_PUMP_OWNER, which is
   src/hal_uno.cpp and nothing else. In a sim or host build no translation unit can even
   NAME the pin: D6 is never made an output, stays an input from reset, and R1 holds the
   relay's OFF level in hardware even with 12 V on COM (§8). */
#ifdef PB_PUMP_OWNER
#  define PIN_PUMP_EN 6
#  ifdef PB_RELAY_ACTIVE_LOW
     /* PFS level bits, not BSP_IO_LEVEL_*: these are ORed into a whole-word PmnPFS write.
        IOPORT_CFG_PORT_OUTPUT_HIGH = 0x1, IOPORT_CFG_PORT_OUTPUT_LOW = 0
        (r_ioport_api.h:185-186). */
#    define PUMP_ON_PFS_LEVEL  0
#    define PUMP_OFF_PFS_LEVEL IOPORT_CFG_PORT_OUTPUT_HIGH
#  else
#    define PUMP_ON_PFS_LEVEL  IOPORT_CFG_PORT_OUTPUT_HIGH
#    define PUMP_OFF_PFS_LEVEL 0
#  endif
#endif
```

6. [ ] **Resolve the servo pin against the wiring package, in this step, before committing.** The spec never names a servo pin. Two sources disagree and one of them is retired:
   - `cad/wiring/nets.py:233` and the generated pin table in `cad/wiring/README.md:32` both say **D9** for manifold 1 (`{"manifold": "1 (bench)", "servo": "D9", ...}`).
   - The retired clone at `~/projects/plant_butler` drove the servo from **pin 8**, and the plan skeleton carried that number forward.

   `pins.h` is an implementation of the wiring package, so the code above ships **9**. Confirm it by eye now:

```bash
grep -n 'SERVO' /Users/jcanton/projects/plant-butler/cad/wiring/nets.py | head
```

   Expected: the `Wire("UNO", "D9", "SERVO", ...)` row and the manifold table row naming `D9`. If the bench is in fact wired to D8, change the constant here and open an issue against `cad/wiring/nets.py` — do not leave the two disagreeing. Record whichever you confirmed in the commit message.

7. [ ] **Let `[env:native]` compile a header that carries a mandatory `#error`.** `src/safety.cpp` (`PIN_HALL_FLOAT`) and `src/sensors.cpp` (`I2C_ADDR_EXPANDER`, `PIN_MUX_ADC`) both include `pins.h` and both compile on the host, so the host needs a polarity defined or every native build stops at the `#error`. Defining it there costs nothing: without `PB_PUMP_OWNER` the host build has no `PIN_PUMP_EN` and no `PUMP_*_PFS_LEVEL` at all, so the value is inert. In `platformio.ini`, `[env:native]`:

```ini
build_flags = ${env.build_flags} -std=gnu++17 -DPB_NATIVE=1 -DPB_SIM=1 -DPB_SIM_CLI=1 -DPB_BRINGUP=1 -DPB_RELAY_ACTIVE_HIGH -DPB_CONTROLLER='"test1"' -I include
```

   **This line is task 1's line plus `-DPB_RELAY_ACTIVE_HIGH` and nothing else. `-DPB_SIM_CLI=1` stays.** Dropping it would silently compile the whole `sim ...` console family out of the host suite, fail task 29's `test_every_sim_command_is_parsed_and_dispatched`, and turn `[env:native_nosimcli]`'s `-UPB_SIM_CLI` into a no-op that proves nothing.

   with this comment above the env, so nobody deletes it later:

```ini
; -DPB_RELAY_ACTIVE_HIGH is required here only because safety.cpp and sensors.cpp include
; pins.h and pins.h #errors without a polarity. It is INERT on the host: PB_PUMP_OWNER is
; never defined off the board, so PIN_PUMP_EN and the PFS level macros do not exist here.
; The board-side guard is unweakened — hal_uno.cpp is the only PB_PUMP_OWNER in the tree.
```

8. [ ] **Prove the `#error` fires on the board env, by hand, once.** Temporarily delete the `-DPB_RELAY_ACTIVE_HIGH` line from `[env:uno_r4_wifi]`'s `build_flags` and build:

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio run -e uno_r4_wifi
```

Expected: `include/pins.h:9:4: error: #error "Relay polarity is unknown until bring-up 4a. Define PB_RELAY_ACTIVE_LOW or PB_RELAY_ACTIVE_HIGH in platformio.ini build_flags after you have READ THE MODULE."` and no other diagnostic. **Restore the flag**, rebuild, and confirm the build is clean again.

9. [ ] **Run both gates and commit.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose && pio run -e uno_r4_wifi
git add include/config.h include/pins.h platformio.ini test/test_dose/test_dose.cpp
git commit -m "config.h and pins.h: every constant with the citation that justifies it

Spec §7 transcribed whole, plus §2.2's pin contract: PIN_PUMP_EN and the PFS level
macros exist only under PB_PUMP_OWNER, and there is no default relay polarity.

Three things a reader should know. The servo is PIN_SERVO 9, from cad/wiring/nets.py
and the pin table it generates; the spec names no servo pin and the retired clone used
8. PIN_LED is 13 and is NOT from the spec or from cad/wiring - it is the board's own
LED_BUILTIN, named here so that task 29's sim double-blink does not have to spell an
Arduino-header name in a file that may not include one. And [env:native] now defines
PB_RELAY_ACTIVE_HIGH, because safety.cpp and sensors.cpp include pins.h on the host; it
is inert there, since PB_PUMP_OWNER is never defined off the board."
```

---

### Task 3: Seam 1: hal.h, the fake rig in hal_sim.cpp, and the Unity harness

**Drop 1.**

**Files:**
- Create: `include/hal.h`, `include/sim.h`, `src/hal_sim.cpp`
- Modify: `test/support/harness.h` (task 1 created it empty), `test/test_dose/test_dose.cpp`
- Test: `test/test_dose/test_dose.cpp`

**Interfaces:**

*Consumes.* `include/pins.h` and `include/config.h` from task 2 — in particular `PB_WDT_GRANTED_MS` 5592, `PB_WDT_PROBE_MS` 40, `PB_WDT_PROBE_MIN_COUNTS` 58, `PIN_HALL_FLOAT` 5, `PB_CHANNELS` 6, `PB_ADC_BITS` 14. `[env:native]` compiles this file (`build_src_filter = +<*> -<main.cpp> -<hal_uno.cpp>`).

*Produces.* `include/hal.h` — seam 1, the 35 Arduino-free free functions listed in step 3 below, plus `PB_LOW`/`PB_HIGH`/`PB_IN`/`PB_OUT`. `include/sim.h` — the fake's injectors and call-trace log, listed in step 5. `src/hal_sim.cpp` — the host and sim implementation of every one of them. `test/support/harness.h` — `pb_test_setup()`, `pb_test_teardown()`, `pb_advance(uint32_t ms)`, `pb_expect_no_feed_between(uint32_t from_ms, uint32_t to_ms)`, `pb_count(sim_ev_kind_t)`.

Tasks 4, 6 and 7 each extend `src/hal_sim.cpp` (the `.noinit` model, the flow/screw pulse drive, the expander and six-channel model). Task 8 implements the same `hal.h` for the board.

**Three deviations from the printed seam, all deliberate, all recorded in the commit message:**
- **Three functions are added: `hal_micros()`, `hal_adc_bits()` and `hal_adc_width_ok()`.** Spec §1's printed `hal.h` has no microsecond source, but §1's module table gives `pulses.cpp` a "per-pin minimum-gap reject in the ISR" against `PB_FLOW_MIN_GAP_US = 500` and `PB_SCREW_MIN_GAP_US = 2000` (§7), which cannot be expressed in milliseconds. And §7 requires `hal_begin()` to **assert** `analogReadResolution() == PB_ADC_BITS` and `status` to print `adc_req=14 adc_hw=14` — that is *two* accessors the printed seam does not have, not one: `hal_adc_bits()` is the hardware width `adc_hw=` reads back, and `hal_adc_width_ok()` is the verdict `hal_begin()` computed. **Both are consumed — `hal_adc_bits()` by `cli_print_status()` (task 11 step 6) and by `main.cpp`'s boot banner (task 12 step 3) — so leaving either out is a link failure, not a tidiness question.** That makes 35 functions, not the 32 the module table counts.
- **`hal_pump_write()` and `hal_boot_pump_off()` each emit exactly one call-trace event** whose `arg` carries the same whole-word shape the board writes (`SIM_PFS_DIR_OUT | level`). `sim.h` therefore defines `SIM_PFS_DIR_OUT` and `SIM_PFS_LEVEL_HI` mirroring `IOPORT_CFG_PORT_DIRECTION_OUTPUT` (0x4) and `IOPORT_CFG_PORT_OUTPUT_HIGH` (0x1) (`r_ioport_api.h:184-186`), so §2.1's property is testable on a host with no PFS registers.
- **`src/hal_sim.cpp` is wrapped whole in `#if PB_SIM`.** Spec §10 wants it internally gated so `[env:uno_r4_wifi_test]` can compile both HALs; that env does not define `PB_SIM`, so under this gate the device test env gets no fake. Task 28 owns that env and decides between finer gating and §10's accepted fallback of two device test envs, one per HAL. Do not attempt it here.

---

1. [ ] **Write the six failing cases.** Replace `test/test_dose/test_dose.cpp`'s includes with the harness and append these, registering each with `RUN_TEST` in `main()`. Keep task 2's three config cases.

```c
#include <unity.h>
#include "../support/harness.h"
#include "config.h"

void setUp(void)    { pb_test_setup(); }
void tearDown(void) { pb_test_teardown(); }

/* §2.1: pinMode(D6, OUTPUT) latches PODR = 0 and drives the pin LOW, discarding a
   preceding digitalWrite. The correct sequence is ONE PFS write carrying direction and
   level together, and pinMode never touches D6 at all. */
static void test_boot_configures_d6_with_one_pfs_write_carrying_direction_and_level(void) {
  sim_events_clear();
  hal_boot_pump_off();
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  TEST_ASSERT_EQUAL_UINT32(1u, (uint32_t)n);
  TEST_ASSERT_EQUAL_INT(SIM_EV_PIN_CFG, (int)ev[0].kind);
  TEST_ASSERT_EQUAL_UINT8(6, ev[0].pin);
  TEST_ASSERT_TRUE(ev[0].arg & SIM_PFS_DIR_OUT);     /* the direction */
  TEST_ASSERT_FALSE(ev[0].arg & SIM_PFS_LEVEL_HI);   /* and the OFF level, in the same word */
}

static void test_pinmode_is_never_called_on_the_pump_pin(void) {
  hal_begin();
  hal_boot_pump_off();
  hal_pump_write(true);
  hal_pump_write(false);
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  uint32_t hits = 0;
  for (size_t i = 0; i < n; ++i)
    if (ev[i].kind == SIM_EV_PIN_MODE && ev[i].pin == 6) hits++;
  TEST_ASSERT_EQUAL_UINT32(0u, hits);
}

/* §2.1: R_IOPORT_PinCfg -> R_BSP_PinCfg is one unconditional `PmnPFS = cfg`, so every
   pump write re-states the DIRECTION as well as the level. That is what makes
   safety_tick()'s idle re-assert a REPAIR of a stray pinMode on D6. */
static void test_every_pump_write_restates_the_direction_as_well_as_the_level(void) {
  sim_events_clear();
  hal_pump_write(true);
  hal_pump_write(false);
  hal_pump_write(false);
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  uint32_t writes = 0;
  for (size_t i = 0; i < n; ++i) {
    if (ev[i].kind != SIM_EV_PUMP_WRITE) continue;
    writes++;
    TEST_ASSERT_TRUE(ev[i].arg & SIM_PFS_DIR_OUT);
  }
  TEST_ASSERT_EQUAL_UINT32(3u, writes);
}

static void test_wdt_alive_is_false_only_when_the_counter_is_frozen(void) {
  TEST_ASSERT_TRUE(hal_wdt_alive());              /* healthy: 2929 Hz */
  sim_wdt_stop();                                 /* frozen */
  TEST_ASSERT_FALSE(hal_wdt_alive());
  TEST_ASSERT_EQUAL_UINT32(0u, hal_wdt_last_delta());
  sim_wdt_rate_hz(1000);                          /* moves, but 40 counts < 58 */
  TEST_ASSERT_FALSE(hal_wdt_alive());
}

/* Without this case the suite passes against a probe that feeds — which is exactly how
   this bug survived review the first time (§2.5). */
static void test_wdt_alive_does_not_feed_inside_its_probe_window(void) {
  sim_events_clear();
  TEST_ASSERT_TRUE(hal_wdt_alive());
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  uint32_t feeds = 0, first = 0, last = 0;
  for (size_t i = 0; i < n; ++i) {
    if (ev[i].kind != SIM_EV_WDT_FEED) continue;
    if (feeds == 0) first = ev[i].at_ms;
    last = ev[i].at_ms;
    feeds++;
  }
  TEST_ASSERT_EQUAL_UINT32(2u, feeds);                        /* only the bracketing pair */
  TEST_ASSERT_TRUE(last - first >= PB_WDT_PROBE_MS);          /* the window really was 40 ms */
  pb_expect_no_feed_between(first, last);
}

static void test_wdt_alive_is_true_on_a_counter_that_moves_at_the_real_2929_hz(void) {
  sim_wdt_rate_hz(2929);                          /* PCLKB/8192 = 2929.7 Hz (§7) */
  TEST_ASSERT_TRUE(hal_wdt_alive());
  /* 41 ms of advance (the probe's own hal_millis() reads included) x 2.929 = 120 counts */
  TEST_ASSERT_TRUE(hal_wdt_last_delta() >= PB_WDT_PROBE_MIN_COUNTS);
  TEST_ASSERT_TRUE(hal_wdt_last_delta() <= 130u);
}
```

2. [ ] **Run it and watch it fail on the missing seam.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose
```

Expected: `test/support/harness.h:N:10: fatal error: hal.h: No such file or directory`.

3. [ ] **Write `include/hal.h`** — seam 1, verbatim from spec §1, plus `hal_micros()`.

```c
/* hal.h — the hardware seam. NOTHING here names an Arduino type. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define PB_LOW 0
#define PB_HIGH 1
#define PB_IN 0
#define PB_OUT 1

uint32_t hal_millis(void);
uint32_t hal_micros(void);                 /* ADDED to the printed seam: the ISR gap rejects of
                                              §2.14 are 500 us and 2000 us, not milliseconds */
void     hal_delay_us(uint16_t us);        /* the ONLY sub-ms wait; never > 200 us */
void     hal_pin_mode(uint8_t pin, uint8_t mode);   /* NEVER for D2, D3 or D6 */
int      hal_pin_read(uint8_t pin);
void     hal_pin_write(uint8_t pin, uint8_t level);

void     hal_boot_pump_off(void);          /* ONE PFS write: direction AND level. setup()'s 1st stmt */
void     hal_pump_write(bool on);          /* the ONLY route to D6 after boot */
bool     hal_pump_level_on(void);          /* what PUMP_ON compiled to; `status` prints it */

uint16_t hal_adc_read(void);               /* A0, 14-bit */
uint8_t  hal_adc_bits(void);               /* ADDED: the HARDWARE width, read back. `status`
                                              prints adc_hw= from it and main.cpp's boot banner
                                              prints adc=%lu/%lu from it (§6, §7) */
bool     hal_adc_width_ok(void);           /* ADDED: the verdict hal_begin() drew from the
                                              analogReadResolution() readback; `status` prints
                                              adc_ok= and main.cpp latches err=adc on it (§7) */
bool     hal_i2c_write16(uint8_t addr, uint16_t bits);  /* false == bus error */
bool     hal_i2c_read16(uint8_t addr, uint16_t *bits);  /* false == bus error, NOT zero */
bool     hal_i2c_probe(uint8_t addr);
bool     hal_i2c_recover(void);            /* EXACTLY nine clocks, fixed count */
void     hal_servo_us(uint16_t us);        /* 1500 == stop; 0 == detach */

bool     hal_wdt_start(void);              /* wdt_cfg_t overload; false if the core rejected it */
uint32_t hal_wdt_granted(void);            /* OUR computed grant, never the timeout getter — §2.5 */
uint32_t hal_wdt_counter(void);            /* the raw down-counter; the sim makes it settable */
bool     hal_wdt_alive(void);              /* counter DECREASED across an UNFED window — §2.5 */
uint32_t hal_wdt_last_delta(void);         /* what the last probe measured; rides out as ch209 */
void     hal_wdt_feed(void);               /* ONE caller: safety_tick(). NOT called by the probe. */

bool     hal_irq_armed(uint8_t pin);       /* IELSR scan + NVIC enable for the pin's ICU channel */
bool     hal_irq_filtered(uint8_t pin);    /* IRQCR[ch] FLTEN; `status` prints icufilter= from it */

size_t   hal_serial_read(char *buf, size_t cap);
void     hal_serial_write(const char *s);
void     hal_serial_drain(void);           /* discard the RX ring */
uint32_t hal_heap_arena(void);             /* mallinfo().arena  — break growth */
uint32_t hal_heap_ordblks(void);           /* mallinfo().ordblks — free-chunk count */
uint32_t hal_heap_break(void);             /* (uint32_t)sbrk(0) — the ONLY real heap bound (§12) */
uint32_t hal_stack_limit(void);            /* (uint32_t)&__StackLimit */
uint32_t hal_stack_hwm(void);              /* bytes of the 1024 used, from the boot paint */
uint32_t hal_boot_salt(void);              /* per-boot, from the .noinit boot counter */
void     hal_begin(void);                  /* ADC width, input pins, ISRs, Wire, servo, stack paint */
```

4. [ ] **Run it and watch the failure move to the fake.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose
```

Expected: `test/support/harness.h:N:10: fatal error: sim.h: No such file or directory`.

5. [ ] **Write `include/sim.h`** — the fake's control surface. Every injector here is a state setter with a real consumer; the task that gives each one teeth is named beside it.

```c
/* sim.h — the fake rig's fault injectors and its call-trace log. sim + native only. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* THE FAKE'S CLOCK CONTRACT, and every later task depends on it:
     hal_millis()   advances the rig by exactly 1 ms and runs every model (watchdog,
                    pump-on time, and from task 6 the flow and screw edges), then returns
                    the millisecond it has just reached. That is what lets a bounded spin loop —
                    hal_wdt_alive()'s probe, safety_wait_ms(), dose_run()'s loop — make
                    progress on a host with no real time.
     hal_micros()   READS the clock. It never advances it, so an ISR's minimum-gap reject
                    measures the interval the fake scheduled and not the reading of it.
     hal_delay_us() advances by the microseconds asked for, watchdog and pump time only.
     sim_advance(n) is n of the same 1 ms step. */
#define PB_SIM_TICK_US 1000u

/* The whole-word PmnPFS shape of §2.1, mirrored so the host can assert it:
   IOPORT_CFG_PORT_DIRECTION_OUTPUT = 0x4, IOPORT_CFG_PORT_OUTPUT_HIGH = 0x1
   (r_ioport_api.h:184-186). */
#define SIM_PFS_DIR_OUT  0x4u
#define SIM_PFS_LEVEL_HI 0x1u

void     sim_reset(bool warm);             /* re-enter setup(); warm keeps .noinit (task 4) */
void     sim_advance(uint32_t ms);
void     sim_set_float(bool ok);           /* D5; consumed by the debounce, task 15 */
void     sim_set_flow_ml_s(uint16_t ml_s); /* the pump's delivery rate; task 6 */
void     sim_flow_storm(uint32_t hz);      /* an edge storm on D2; task 6 */
void     sim_set_i2c_fail(bool fail);      /* every expander transfer fails; task 7 */
void     sim_set_mux_stuck(bool stuck);    /* every channel returns the canary's value; task 7 */
void     sim_set_stall(bool on);           /* the screw stops pulsing while driven; task 14 */
void     sim_set_leak(bool on);            /* pulses with D6 off; task 6 */
void     sim_wdt_stop(void);               /* the counter FREEZES */
void     sim_wdt_rate_hz(uint32_t hz);     /* the counter moves, at this rate */
void     sim_noinit_clobber(void);         /* scramble the .noinit struct; task 4 */
void     sim_set_channel(uint8_t ch, uint16_t raw);
void     sim_serial_rx(const char *s);     /* push bytes at the console */
size_t   sim_serial_tx(char *buf, size_t cap);   /* drain what the console printed */
bool     sim_pump_is_on(void);
uint32_t sim_pump_on_ms(void);             /* cumulative ms with D6 asserted */
uint32_t sim_feeds(void);

typedef enum {
  SIM_EV_PIN_CFG,      /* a whole-word direction+level write: hal_boot_pump_off, hal_pin_write */
  SIM_EV_PIN_MODE,
  SIM_EV_PUMP_WRITE,   /* arg carries SIM_PFS_DIR_OUT | level, the same word the board writes */
  SIM_EV_WDT_FEED,
  SIM_EV_I2C_WRITE,
  SIM_EV_I2C_READ,
  SIM_EV_SERVO,
  SIM_EV_ADC
} sim_ev_kind_t;

typedef struct {
  sim_ev_kind_t kind;
  uint8_t       pin;
  uint32_t      arg;
  uint32_t      at_ms;
} sim_ev_t;

size_t sim_events(const sim_ev_t **out);
void   sim_events_clear(void);
```

6. [ ] **Write `src/hal_sim.cpp`.** The `hal_wdt_alive()` body is spec §2.5 verbatim — **it is the one place in the program where feeding is deliberately suspended, and it must call `hal_pump_write(false)` and nothing else inside the window.**

```c
/* src/hal_sim.cpp — one fake, two jobs: the on-device sim and the host test double.
   Filtered out of [env:uno_r4_wifi] by build_src_filter. */
#include "hal.h"
#include "sim.h"
#include "config.h"
#include "pins.h"
#include <string.h>

#if PB_SIM

/* PB_PUMP_OWNER belongs to hal_uno.cpp alone (§2.2), so pins.h does not define
   PIN_PUMP_EN here and this file must name D6 itself. Same for the I2C pair:
   A4 == 18, A5 == 19 (variants/UNOWIFIR4/pins_arduino.h:15). */
#define SIM_PUMP_PIN 6
#define SIM_PIN_SDA  18
#define SIM_PIN_SCL  19
#define SIM_WDT_RELOAD 16384u

/* ---- clock ---- */
static uint32_t g_us, g_ms;

/* ---- D6 ---- */
static bool     g_pump_on;
static uint32_t g_pump_on_us;              /* cumulative time asserted */
static uint32_t g_pump_on_at_ms;           /* when the current assertion started */

/* ---- watchdog ---- */
static bool     g_wdt_running;
static uint32_t g_wdt_counter = SIM_WDT_RELOAD;
static uint32_t g_wdt_rate_hz = 2929;      /* PCLKB/8192 = 2929.7 Hz (§7) */
static uint32_t g_wdt_frac;                /* carried remainder, in counts * 1e6 */
static uint32_t g_wdt_delta;
static uint32_t g_feeds;

/* ---- injector state; the task that consumes each one is named in sim.h ---- */
static bool     g_float_ok = true;
static uint16_t g_flow_ml_s;
static uint32_t g_storm_hz;
static bool     g_i2c_fail;
static bool     g_mux_stuck;
static bool     g_stall;
static bool     g_leak;
static uint16_t g_chan[16];
static uint16_t g_exp_port = 0xFFFFu;      /* the PCF8575's latch; task 7 gives it meaning */
static uint8_t  g_mux_sel;
static uint16_t g_servo_us = 1500u;        /* 1500 == stopped; task 6 drives the screw off it */
static uint32_t g_boots;                   /* task 4 replaces this with g_nv.boots */

/* ---- serial ---- */
static char   g_rx[256]; static size_t g_rx_len, g_rx_pos;
static char   g_tx[4096]; static size_t g_tx_len;

/* ---- call trace ---- */
static sim_ev_t g_ev[1024];
static size_t   g_ev_n;

static void ev_(sim_ev_kind_t k, uint8_t pin, uint32_t arg) {
  if (g_ev_n >= sizeof g_ev / sizeof g_ev[0]) return;
  g_ev[g_ev_n].kind = k; g_ev[g_ev_n].pin = pin;
  g_ev[g_ev_n].arg = arg; g_ev[g_ev_n].at_ms = g_ms;
  g_ev_n++;
}
size_t sim_events(const sim_ev_t **out) { *out = g_ev; return g_ev_n; }
void   sim_events_clear(void) { g_ev_n = 0; }

/* ---- the clock, and every model that runs off it ---- */
static void tick_models_(uint32_t us) {
  if (g_wdt_running && g_wdt_rate_hz) {
    g_wdt_frac += g_wdt_rate_hz * us;                  /* counts * 1e6 */
    uint32_t ticks = g_wdt_frac / 1000000u;
    g_wdt_frac %= 1000000u;
    g_wdt_counter = (g_wdt_counter > ticks) ? (g_wdt_counter - ticks) : 0u;
  }
  if (g_pump_on) g_pump_on_us += us;
}

/* One millisecond of rig time. Task 6 hangs the flow and screw edge emitters here,
   between tick_models_() and the final clock assignment, so an edge can land at its own
   microsecond inside the step. */
static void advance_1ms_(void) {
  const uint32_t target = g_us + 1000u;
  tick_models_(1000u);
  g_us = target;
  g_ms = g_us / 1000u;
}

void     sim_advance(uint32_t ms) { for (uint32_t i = 0; i < ms; ++i) advance_1ms_(); }
uint32_t hal_millis(void) { advance_1ms_(); return g_ms; }   /* PB_SIM_TICK_US == 1000 */
uint32_t hal_micros(void) { return g_us; }                   /* reads; never advances */
void     hal_delay_us(uint16_t us) { tick_models_(us); g_us += us; g_ms = g_us / 1000u; }

/* ---- D6. ONE event per write, carrying the same whole word the board writes (§2.1).
   Polarity is the board's business and lives only in hal_uno.cpp; the fake models
   PB_RELAY_ACTIVE_HIGH. ---- */
void hal_boot_pump_off(void) {
  ev_(SIM_EV_PIN_CFG, SIM_PUMP_PIN, SIM_PFS_DIR_OUT);
  g_pump_on = false;
}
void hal_pump_write(bool on) {
  ev_(SIM_EV_PUMP_WRITE, SIM_PUMP_PIN, SIM_PFS_DIR_OUT | (on ? SIM_PFS_LEVEL_HI : 0u));
  if (on && !g_pump_on) g_pump_on_at_ms = g_ms;    /* task 6's prime delay runs from here */
  g_pump_on = on;
}
bool     hal_pump_level_on(void) { return true; }
bool     sim_pump_is_on(void)    { return g_pump_on; }
uint32_t sim_pump_on_ms(void)    { return g_pump_on_us / 1000u; }

/* ---- ordinary pins ---- */
void hal_pin_mode(uint8_t pin, uint8_t mode) { ev_(SIM_EV_PIN_MODE, pin, mode); }
void hal_pin_write(uint8_t pin, uint8_t level) {
  ev_(SIM_EV_PIN_CFG, pin, SIM_PFS_DIR_OUT | (level ? SIM_PFS_LEVEL_HI : 0u));
}
int  hal_pin_read(uint8_t pin) {
  if (pin == PIN_HALL_FLOAT) return g_float_ok ? PB_LOW : PB_HIGH;   /* §2.10: LOW == OK */
  return PB_HIGH;
}
void sim_set_float(bool ok) { g_float_ok = ok; }

/* ---- ADC and I2C. Task 7 gives the expander its mux/home-hall meaning. ---- */
uint16_t hal_adc_read(void) {
  uint8_t ch = g_mux_stuck ? PB_CANARY_CHANNEL : g_mux_sel;
  uint16_t v = g_chan[ch & 0x0Fu];
  ev_(SIM_EV_ADC, ch, v);
  return v;
}
void sim_set_channel(uint8_t ch, uint16_t raw) { g_chan[ch & 0x0Fu] = raw; }
uint8_t hal_adc_bits(void)  { return (uint8_t)PB_ADC_BITS; }  /* the host's width IS the constant */
bool hal_adc_width_ok(void) { return true; }   /* no ADC to mis-configure on the host */

bool hal_i2c_write16(uint8_t addr, uint16_t bits) {
  ev_(SIM_EV_I2C_WRITE, addr, bits);
  if (g_i2c_fail) return false;
  g_exp_port = bits;
  g_mux_sel = (uint8_t)(bits & 0x0Fu);
  return true;
}
bool hal_i2c_read16(uint8_t addr, uint16_t *bits) {
  ev_(SIM_EV_I2C_READ, addr, 0);
  if (g_i2c_fail) return false;
  *bits = g_exp_port;
  return true;
}
bool hal_i2c_probe(uint8_t addr) {
  if (g_i2c_fail) return false;
  return addr == I2C_ADDR_EXPANDER || addr == I2C_ADDR_LCD || addr == I2C_ADDR_OLED;
}
bool hal_i2c_recover(void) {
  /* EXACTLY PB_I2C_RECOVER_CLOCKS clocks, a fixed loop count, never "until SDA releases"
     (§2.13). The refusal-while-dosing guard is sensors.cpp's — see task 7. */
  hal_pin_write(SIM_PIN_SDA, PB_HIGH);
  for (uint8_t i = 0; i < PB_I2C_RECOVER_CLOCKS; ++i) {
    hal_pin_write(SIM_PIN_SCL, PB_HIGH); hal_delay_us(5);
    hal_pin_write(SIM_PIN_SCL, PB_LOW);  hal_delay_us(5);
  }
  return true;
}
void sim_set_i2c_fail(bool fail)  { g_i2c_fail = fail; }
void sim_set_mux_stuck(bool stuck) { g_mux_stuck = stuck; }

void hal_servo_us(uint16_t us) { g_servo_us = us; ev_(SIM_EV_SERVO, PIN_SERVO, us); }
void sim_set_stall(bool on) { g_stall = on; }
void sim_set_leak(bool on)  { g_leak = on; }
void sim_set_flow_ml_s(uint16_t ml_s) { g_flow_ml_s = ml_s; }
void sim_flow_storm(uint32_t hz)      { g_storm_hz = hz; }

/* ---- the watchdog ---- */
bool     hal_wdt_start(void) { g_wdt_running = true; g_wdt_counter = SIM_WDT_RELOAD; g_wdt_frac = 0; return true; }
uint32_t hal_wdt_granted(void) { return g_wdt_running ? (uint32_t)PB_WDT_GRANTED_MS : 0u; }
uint32_t hal_wdt_counter(void) { return g_wdt_counter; }
uint32_t hal_wdt_last_delta(void) { return g_wdt_delta; }
uint32_t sim_feeds(void) { return g_feeds; }
void     sim_wdt_stop(void) { g_wdt_rate_hz = 0; }
void     sim_wdt_rate_hz(uint32_t hz) { g_wdt_rate_hz = hz; }

void hal_wdt_feed(void) {
  g_wdt_counter = SIM_WDT_RELOAD;
  g_wdt_frac = 0;
  g_feeds++;
  ev_(SIM_EV_WDT_FEED, 0, g_wdt_counter);
}

/* The ONE place in the program that deliberately does not feed. Precondition: not dosing —
   it is called from setup() and as dose_run()'s first guard, after the g_dosing check.
   Spec §2.5, verbatim; hal_uno.cpp carries the same body. */
bool hal_wdt_alive(void) {
  hal_wdt_feed();                              /* start from a known reload */
  uint32_t a  = hal_wdt_counter();
  uint32_t t0 = hal_millis();
  while (hal_millis() - t0 < PB_WDT_PROBE_MS)  /* 40 ms, UNFED, pump already idle-OFF */
    hal_pump_write(false);                     /* the safety half of safety_tick(), without the feed */
  uint32_t b = hal_wdt_counter();
  hal_wdt_feed();                              /* and immediately back in the window */
  g_wdt_delta = (a > b) ? (a - b) : 0;         /* a DOWN-counter: b must be smaller */
  return g_wdt_delta >= PB_WDT_PROBE_MIN_COUNTS;
}

/* ---- interrupts: the fake models correctly armed, filtered pins ---- */
bool hal_irq_armed(uint8_t pin)    { return pin == PIN_FLOW || pin == PIN_HALL_SCREW; }
bool hal_irq_filtered(uint8_t pin) { return pin == PIN_FLOW || pin == PIN_HALL_SCREW; }

/* ---- serial ---- */
size_t hal_serial_read(char *buf, size_t cap) {
  size_t n = 0;
  while (n < cap && g_rx_pos < g_rx_len) buf[n++] = g_rx[g_rx_pos++];
  return n;
}
void hal_serial_write(const char *s) {
  while (*s && g_tx_len < sizeof g_tx - 1) g_tx[g_tx_len++] = *s++;
  g_tx[g_tx_len] = '\0';
}
void hal_serial_drain(void) { g_rx_pos = g_rx_len; }
void sim_serial_rx(const char *s) {
  size_t n = strlen(s);
  if (g_rx_len + n > sizeof g_rx) n = sizeof g_rx - g_rx_len;
  memcpy(g_rx + g_rx_len, s, n);
  g_rx_len += n;
}
size_t sim_serial_tx(char *buf, size_t cap) {
  size_t n = g_tx_len < cap - 1 ? g_tx_len : cap - 1;
  memcpy(buf, g_tx, n);
  buf[n] = '\0';
  g_tx_len = 0;
  return n;
}

/* ---- memory. The host has no __StackLimit; report a break comfortably inside the
   margin so §12's check is exercised without faking a failure nobody asked for. ---- */
uint32_t hal_heap_arena(void)   { return 2048u; }
uint32_t hal_heap_ordblks(void) { return 3u; }
uint32_t hal_heap_break(void)   { return 0x20001800u; }
uint32_t hal_stack_limit(void)  { return 0x20007b00u; }
uint32_t hal_stack_hwm(void)    { return 384u; }
uint32_t hal_boot_salt(void)    { return g_boots * 2654435761u; }   /* task 4 rewires to g_nv.boots */

void hal_begin(void) {
  g_mux_sel = 0;
  g_exp_port = 0xFFFFu;
  g_servo_us = 1500u;
}

void sim_noinit_clobber(void) { }   /* task 4 gives this the .noinit struct to scramble */

void sim_reset(bool warm) {
  g_us = g_ms = 0;
  g_pump_on = false; g_pump_on_us = 0; g_pump_on_at_ms = 0;
  g_wdt_running = false; g_wdt_counter = SIM_WDT_RELOAD; g_wdt_rate_hz = 2929;
  g_wdt_frac = 0; g_wdt_delta = 0; g_feeds = 0;
  g_float_ok = true; g_flow_ml_s = 0; g_storm_hz = 0;
  g_i2c_fail = false; g_mux_stuck = false; g_stall = false; g_leak = false;
  memset(g_chan, 0, sizeof g_chan);
  g_rx_len = g_rx_pos = 0; g_tx_len = 0;
  g_ev_n = 0;
  if (warm) g_boots++; else g_boots = 1;     /* task 4 moves this into the .noinit struct */
  hal_begin();
}

#endif /* PB_SIM */
```

7. [ ] **Write `test/support/harness.h`** — a HEADER, with `static inline` bodies. `test/support/` must not become a suite directory with no runner (spec §10).

```c
/* test/support/harness.h — the Unity fixture over hal_sim. A header, not a suite. */
#pragma once
#include <unity.h>
#include "hal.h"
#include "sim.h"

static inline void pb_test_setup(void) {
  sim_reset(false);          /* a cold boot: clock at 0, .noinit cleared */
  hal_begin();
  hal_boot_pump_off();
  (void)hal_wdt_start();
  sim_events_clear();
}

static inline void pb_test_teardown(void) { sim_events_clear(); }

static inline void pb_advance(uint32_t ms) { sim_advance(ms); }

static inline uint32_t pb_count(sim_ev_kind_t kind) {
  const sim_ev_t *ev; size_t n = sim_events(&ev); uint32_t hits = 0;
  for (size_t i = 0; i < n; ++i) if (ev[i].kind == kind) hits++;
  return hits;
}

/* Strictly inside: the two feeds that BRACKET hal_wdt_alive()'s probe are legal, and
   anything between them is the bug this exists to catch (§2.5). */
static inline void pb_expect_no_feed_between(uint32_t from_ms, uint32_t to_ms) {
  const sim_ev_t *ev; size_t n = sim_events(&ev); uint32_t hits = 0;
  for (size_t i = 0; i < n; ++i)
    if (ev[i].kind == SIM_EV_WDT_FEED && ev[i].at_ms > from_ms && ev[i].at_ms < to_ms) hits++;
  TEST_ASSERT_EQUAL_UINT32(0u, hits);
}
```

8. [ ] **Run the suite and watch all nine cases pass.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose
```

Expected: task 1's link-gate case, task 2's three and these six — `10 Tests 0 Failures 0 Ignored / OK`. (An earlier draft said 9 and silently dropped task 1's case; nothing deletes it.) If `test_wdt_alive_is_true_on_a_counter_that_moves_at_the_real_2929_hz` reports a delta of 0, the probe is feeding inside its window — re-read step 6's `hal_wdt_alive()` and check that nothing but `hal_pump_write(false)` is called in the loop body.

9. [ ] **Check the board env still builds** (it must not compile the fake) **and commit.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio run -e uno_r4_wifi && pio test -e native -f test_dose
git add include/hal.h include/sim.h src/hal_sim.cpp test/support/harness.h test/test_dose/test_dose.cpp
git commit -m "Seam 1: hal.h, the fake rig, and the Unity fixture over it

Three deviations from the seam spec §1 prints, all deliberate:

hal_micros(), hal_adc_bits() and hal_adc_width_ok() are added. The printed hal.h has no
microsecond source, and the per-pin ISR gap rejects of §2.14 are 500 us and 2000 us. And
§7 makes hal_begin() assert the analogReadResolution() readback and status print
adc_req/adc_hw, which needs TWO accessors - the hardware width and the verdict. That
makes 35 functions, not 32.

The fake records a pump write as ONE event whose arg carries SIM_PFS_DIR_OUT | level —
the same whole word R_IOPORT_PinCfg puts in PmnPFS — so §2.1's 'every write restates the
direction' is testable on a host with no PFS registers.

hal_sim.cpp is wrapped whole in #if PB_SIM, so [env:uno_r4_wifi_test] currently gets no
fake. That env is task 28's, and §10 already names the fallback: two device test envs,
one per HAL.

hal_wdt_alive() is §2.5 verbatim in both HALs, and the suite asserts that no feed occurs
strictly between the two that bracket the probe. Without that assertion the case passes
against a feeding probe."
```

---

### Task 4: The .noinit block: magic, checksum, cold-versus-warm boot, and the boot salt

**Drop 1.**

**Files:**
- Create: `include/noinit.h`, `src/noinit.cpp`
- Modify: `src/hal_sim.cpp` (`sim_reset(bool warm)`, `sim_noinit_clobber()`, `hal_boot_salt()`), `test/test_dose/test_dose.cpp`
- Test: `test/test_dose/test_dose.cpp`

**Interfaces:**

*Consumes.* `PB_NOINIT_MAGIC` (0x50423031u, "PB01") from `include/config.h` (task 2). From `src/hal_sim.cpp` (task 3): `sim_reset(bool warm)`, `sim_noinit_clobber(void)`, and `hal_boot_salt(void)`, which currently multiplies a file-static boot counter and is rewired here.

*Produces.*

```c
typedef struct {
  uint32_t magic; uint32_t boots; uint32_t cmd_high_water;
  bool dry_latched; bool contra_latched; bool dose_in_flight; uint8_t _pad;
  uint32_t pattern; uint32_t sum;
} pb_noinit_t;
extern pb_noinit_t g_nv;
uint32_t noinit_sum(const pb_noinit_t *n);
void     noinit_begin(void);
void     noinit_commit(void);
bool     noinit_was_cold(void);
bool     noinit_reset_mid(void);
```

`noinit_reset_mid()` is the fifth, and it is not optional: it is the **only** producer of the `resetmid` token in spec §4.1's fixed enum. `cli_print_status()` prints `resetmid=%u` from it (task 11 step 6) and `safety_last_err()`'s lazy initialisation turns it into `err=resetmid` (task 17 step 3), which is bring-up 7c's stated pass criterion. Without it both of those fail to link and 7c has no pass criterion at all.

Two contracts later tasks depend on and must not rediscover:
- **`noinit_commit()` is called after EVERY write to `g_nv`.** A write without it leaves the struct reading as a partial clobber on the next boot, which is a silently lost `dry on`.
- **`noinit_begin()` records the *reset taken with the pump asserted* condition in a program-wide flag, and leaves `dose_in_flight` for `setup()` to clear.** `(!noinit_was_cold() && g_nv.dose_in_flight)` is exactly §2.3's condition; `noinit_begin()` latches it into `g_reset_mid` (which `noinit_reset_mid()` returns) at the same moment it latches dry, so the fact survives `setup()` clearing the flag. **Task 12** (`main.cpp`'s `setup()`) is the consumer: it reads `noinit_reset_mid()`, sets `g_boot_err = "resetmid"`, then clears `g_nv.dose_in_flight` and calls `noinit_commit()` — **exactly once per boot**. Without that clear the flag latches dry on every warm boot forever. **Task 17** (`dose_run()`) is the only other writer of `dose_in_flight`: set before the ON write, cleared after the OFF write.

**One deviation, recorded in the commit message.** Spec §11's file tree lists `include/noinit.h` and no `noinit.cpp`, but the logic cannot live in the header: `noinit_was_cold()` needs one program-wide flag, and a `static inline` in a header gives every translation unit its own. It cannot live in the two HALs either, because then the checksum rule is written twice and can drift. So `src/noinit.cpp` is added: 35 lines, compiled by every env, holding `g_nv` (with the `.noinit` section attribute off the host) and the four functions.

---

1. [ ] **Write the six failing cases** in `test/test_dose/test_dose.cpp`, registered with `RUN_TEST` in `main()`.

```c
#include "noinit.h"

static void test_a_cold_boot_zeroes_the_noinit_struct(void) {
  g_nv.dry_latched = true; g_nv.contra_latched = true; g_nv.cmd_high_water = 42u;
  noinit_commit();
  sim_reset(false);                        /* a power cycle: SRAM is cleared (§2.3) */
  TEST_ASSERT_TRUE(noinit_was_cold());
  TEST_ASSERT_FALSE(g_nv.dry_latched);
  TEST_ASSERT_FALSE(g_nv.contra_latched);
  TEST_ASSERT_EQUAL_UINT32(0u, g_nv.cmd_high_water);
  TEST_ASSERT_EQUAL_UINT32((uint32_t)PB_NOINIT_MAGIC, g_nv.magic);
  TEST_ASSERT_EQUAL_UINT32(1u, g_nv.boots);
}

/* The checksum is what keeps a PARTIAL clobber from reading as a valid latch — the
   bootloader's own .data/.bss sit exactly where __noinit_start does (§2.3). */
static void test_a_bad_checksum_reads_as_a_cold_boot(void) {
  g_nv.dry_latched = true; g_nv.cmd_high_water = 7u;
  noinit_commit();
  sim_noinit_clobber();                    /* magic survives; the sum does not */
  TEST_ASSERT_EQUAL_UINT32((uint32_t)PB_NOINIT_MAGIC, g_nv.magic);
  sim_reset(true);                         /* a WARM reset: SRAM kept */
  TEST_ASSERT_TRUE(noinit_was_cold());
  TEST_ASSERT_FALSE(g_nv.dry_latched);
  TEST_ASSERT_EQUAL_UINT32(0u, g_nv.cmd_high_water);
}

static void test_a_warm_boot_restores_the_latches_and_the_high_water_mark(void) {
  g_nv.dry_latched = true; g_nv.contra_latched = true;
  g_nv.cmd_high_water = 65540u;            /* above 2^16: the ack id is a uint32 (§4.3) */
  g_nv.pattern = 0xDEADBEEFu;              /* bring-up 7c's `noinit pattern` word */
  noinit_commit();
  uint32_t before = g_nv.boots;
  sim_reset(true);
  TEST_ASSERT_FALSE(noinit_was_cold());
  TEST_ASSERT_TRUE(g_nv.dry_latched);
  TEST_ASSERT_TRUE(g_nv.contra_latched);
  TEST_ASSERT_EQUAL_UINT32(65540u, g_nv.cmd_high_water);
  TEST_ASSERT_EQUAL_UINT32(0xDEADBEEFu, g_nv.pattern);
  TEST_ASSERT_EQUAL_UINT32(before + 1u, g_nv.boots);
}

/* §2.3: a reset with the pump asserted is the single loudest thing this rig can discover
   about itself. It latches dry, and the flag stays set for setup() to turn into
   err=resetmid before clearing it. */
static void test_a_dose_in_flight_across_a_warm_boot_latches_dry(void) {
  g_nv.dry_latched = false; g_nv.dose_in_flight = true;
  noinit_commit();
  sim_reset(true);
  TEST_ASSERT_FALSE(noinit_was_cold());
  TEST_ASSERT_TRUE(g_nv.dry_latched);
  TEST_ASSERT_TRUE(g_nv.dose_in_flight);
}

/* The same condition, as the fact setup() and safety_last_err() actually consume. Without
   this accessor `err=resetmid` has no producer anywhere and bring-up 7c's pass criterion
   (`status` says dry=1 and last=resetmid) is unreachable. */
static void test_a_dose_in_flight_across_a_warm_boot_raises_resetmid(void) {
  g_nv.dose_in_flight = true;
  noinit_commit();
  sim_reset(true);
  TEST_ASSERT_TRUE(noinit_reset_mid());
  /* a COLD boot is not a reset mid-dose, whatever SRAM happened to hold */
  sim_reset(false);
  TEST_ASSERT_FALSE(noinit_reset_mid());
}

/* §15.2: without the salt, a watchdog reset loop reports at t ~= 15000 every boot, and
   butler silently discards each repeat as a retry of the same (controller, t). */
static void test_boot_salt_differs_across_two_warm_boots(void) {
  sim_reset(true); uint32_t a = hal_boot_salt();
  sim_reset(true); uint32_t b = hal_boot_salt();
  TEST_ASSERT_TRUE(a != b);
  TEST_ASSERT_TRUE(a != 0u && b != 0u);
  /* and it puts t above 2^31 on ordinary boots, which is why §9 greps for %d */
  TEST_ASSERT_TRUE(a > 0x80000000u || b > 0x80000000u);
}
```

2. [ ] **Run it and watch it fail on the missing header.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose
```

Expected: `test/test_dose/test_dose.cpp:N:10: fatal error: noinit.h: No such file or directory`.

3. [ ] **Write `include/noinit.h`.**

```c
/* include/noinit.h — the ~44 bytes that survive a warm reset. Spec §2.3.
   SRAM survives a watchdog or RESET-button reset; only the startup code clears .bss, and
   fsp.ld provides a .noinit NOLOAD section (:222-231) that it does not clear. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "config.h"

/* The salt's stride. §15.2 requires "a large odd stride" and names no value; this is the
   32-bit golden-ratio multiplier 0x9E3779B1. Odd, so distinct boot counts never collide,
   and large enough that t = hal_boot_salt() + hal_millis() is above 2^31 on ordinary
   boots — which is exactly why the %d grep of §9 is load-bearing. */
#define PB_BOOT_SALT_STRIDE 2654435761u

typedef struct {
  uint32_t magic;            /* PB_NOINIT_MAGIC; anything else == cold boot, zero the struct */
  uint32_t boots;            /* incremented every warm boot; feeds hal_boot_salt() */
  uint32_t cmd_high_water;   /* highest command id ever accepted (§4.3 replay guard) */
  bool     dry_latched;      /* the operator's `dry on` */
  bool     contra_latched;   /* the float/flow contradiction latch, §2.7 */
  bool     dose_in_flight;   /* set before the ON write, cleared after the OFF write */
  uint8_t  _pad;
  uint32_t pattern;          /* bring-up 7c's `noinit pattern` spare word */
  uint32_t sum;              /* XOR of every field above; recomputed on EVERY write */
} pb_noinit_t;

extern pb_noinit_t g_nv;

uint32_t noinit_sum(const pb_noinit_t *n);
void     noinit_begin(void);     /* setup() calls this once, before anything reads a latch */
void     noinit_commit(void);    /* call after EVERY write to g_nv. No exceptions. */
bool     noinit_was_cold(void);  /* what noinit_begin() decided this boot was */

/* §2.3's reset-taken-with-the-pump-asserted verdict, latched by noinit_begin() BEFORE
   setup() clears dose_in_flight, so the fact outlives the flag. The ONLY producer of the
   `resetmid` token of §4.1's enum: status prints resetmid= from it (task 11) and
   safety_last_err() turns it into err=resetmid (task 17). */
bool     noinit_reset_mid(void);
```

4. [ ] **Write `src/noinit.cpp`.**

```c
/* src/noinit.cpp — the warm-reset struct's storage and its four rules. Spec §2.3. */
#include "noinit.h"
#include <string.h>

/* On the board the struct must land in fsp.ld's uncleared NOLOAD section. On the host
   there is no such section and an ordinary definition is the whole model — which is why
   survival across a REAL warm reset is a measurement bring-up 7c owes, not a claim this
   file makes. */
#ifdef PB_NATIVE
pb_noinit_t g_nv;
#else
pb_noinit_t g_nv __attribute__((section(".noinit")));
#endif

static bool g_was_cold;
static bool g_reset_mid;      /* §2.3's verdict, latched once per boot by noinit_begin() */

uint32_t noinit_sum(const pb_noinit_t *n) {
  return n->magic ^ n->boots ^ n->cmd_high_water ^ n->pattern
       ^ ((uint32_t)n->dry_latched)
       ^ ((uint32_t)n->contra_latched   << 8)
       ^ ((uint32_t)n->dose_in_flight   << 16);
}

void noinit_commit(void) { g_nv.sum = noinit_sum(&g_nv); }

bool noinit_was_cold(void)  { return g_was_cold; }
bool noinit_reset_mid(void) { return g_reset_mid; }

void noinit_begin(void) {
  g_reset_mid = false;
  if (g_nv.magic != PB_NOINIT_MAGIC || g_nv.sum != noinit_sum(&g_nv)) {
    /* a cold boot, or a partial clobber: start clean */
    memset(&g_nv, 0, sizeof g_nv);
    g_nv.magic = PB_NOINIT_MAGIC;
    g_nv.boots = 1u;
    g_was_cold = true;
  } else {
    /* dry_latched, contra_latched, cmd_high_water and pattern are KEPT untouched.
       dose_in_flight is kept SET as well, and the VERDICT is latched here rather than left
       for setup() to re-derive: setup() clears the flag (task 12), and g_reset_mid is what
       survives that clear so `status` and safety_last_err() can still see the fact. */
    if (g_nv.dose_in_flight) { g_nv.dry_latched = true; g_reset_mid = true; }
    g_nv.boots++;
    g_was_cold = false;
  }
  noinit_commit();
}
```

5. [ ] **Give the fake a `.noinit` block to model.** In `src/hal_sim.cpp`: add `#include "noinit.h"`, delete the `static uint32_t g_boots;` declaration and the `if (warm) g_boots++; else g_boots = 1;` line, and replace these three functions:

```c
uint32_t hal_boot_salt(void) { return g_nv.boots * PB_BOOT_SALT_STRIDE; }

/* A partial clobber: the magic survives, the checksum does not. That is the shape
   the bootloader's own .data/.bss would leave behind (§2.3). */
void sim_noinit_clobber(void) { g_nv.pattern ^= 0xA5A5A5A5u; }

void sim_reset(bool warm) {
  g_us = g_ms = 0;
  g_pump_on = false; g_pump_on_us = 0; g_pump_on_at_ms = 0;
  g_wdt_running = false; g_wdt_counter = SIM_WDT_RELOAD; g_wdt_rate_hz = 2929;
  g_wdt_frac = 0; g_wdt_delta = 0; g_feeds = 0;
  g_float_ok = true; g_flow_ml_s = 0; g_storm_hz = 0;
  g_i2c_fail = false; g_mux_stuck = false; g_stall = false; g_leak = false;
  memset(g_chan, 0, sizeof g_chan);
  g_rx_len = g_rx_pos = 0; g_tx_len = 0;
  g_ev_n = 0;
  if (!warm) memset(&g_nv, 0, sizeof g_nv);   /* a power cycle clears SRAM (§2.3) */
  noinit_begin();                             /* what setup() does, in the same order */
  hal_begin();
}
```

6. [ ] **Run the suite and watch the six cases pass.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose
```

Expected: `16 Tests 0 Failures 0 Ignored / OK` (task 1's one, task 2's three, task 3's six, these six). If `test_a_bad_checksum_reads_as_a_cold_boot` fails, check that `sim_noinit_clobber()` changes a field `noinit_sum()` actually reads — clobbering `sum` alone would also work, but clobbering a *payload* field is the case that matters.

7. [ ] **Check the board env builds and commit.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio run -e uno_r4_wifi && pio test -e native -f test_dose
git add include/noinit.h src/noinit.cpp src/hal_sim.cpp test/test_dose/test_dose.cpp
git commit -m "The .noinit block: magic, checksum, cold-versus-warm boot, and the boot salt

Survival across a real warm reset is UNPROVEN and stays unproven until bring-up 7c'
measures it (§2.3): __noinit_start sits at the very bottom of SRAM, immediately above
.data, which is exactly where a second-stage image puts its own .data/.bss, and nothing
in the installed package says what SRAM the R4's bootloader uses. Until 7c' passes,
every guarantee here is best-effort and the checksum is the whole defence — it is what
keeps a partial clobber from reading as a valid dry latch.

noinit_begin() deliberately does NOT clear dose_in_flight on a warm boot; it latches the
verdict into noinit_reset_mid() and leaves the flag for setup() (task 12) to clear exactly
once per boot. The pair (!noinit_was_cold() && g_nv.dose_in_flight) is §2.3's
reset-with-the-pump-asserted case, and noinit_reset_mid() is the ONLY producer of the
resetmid token in §4.1's enum - status prints it and safety_last_err() turns it into
err=resetmid, which is bring-up 7c's pass criterion.

Deviations: src/noinit.cpp is added to §11's tree, because noinit_was_cold() needs one
program-wide flag that a header cannot give it and the checksum rule must not be written
twice in the two HALs. And PB_BOOT_SALT_STRIDE is 0x9E3779B1: §15.2 asks for a large odd
stride and names no value."
```

---

### Task 5: safety_tick() and safety_wait_ms() — the feed primitive everything else is built on

**Drop 1.**

**Files:**
- Create: `include/safety.h`, `src/safety.cpp`
- Modify: `test/test_dose/test_dose.cpp`
- Test: `test/test_dose/test_dose.cpp`

**Interfaces:**

*Consumes.* From `include/hal.h` (task 3): `void hal_pump_write(bool on);`, `void hal_wdt_feed(void);`, `uint32_t hal_millis(void);`.

*Produces.*

```c
void safety_tick(void);            /* pump idle re-asserted, THEN the dog fed. That order. */
void safety_wait_ms(uint32_t ms);  /* a bounded wait that calls safety_tick() every iteration */
bool safety_dosing(void);          /* true only between dose_run()'s ON and OFF writes */
void safety_set_dosing(bool on);   /* the ONLY production writer is dose_run(), task 17 */
```

`src/safety.cpp` is created here, ahead of drop 2, because `sensors_sweep()` (task 7) must feed between channels and `ui_poll()` (task 10) must feed between LCD characters. Tasks 15–19 fill in the rest of the file: the float debounce, the dry latch, the contradiction latch and `dose_run()`.

**One deviation, recorded in the commit message.** The skeleton lists three declarations; `safety_set_dosing(bool)` is a fourth. `g_dosing` is a file-static that `dose_run()` (task 17) writes directly, so production needs no setter — but task 7's `test_i2c_recovery_never_runs_while_the_pump_is_asserted` is a spec-named case (§9) that has to reproduce a dose in flight two drops before `dose_run()` exists, and a Unity suite cannot reach a file-static. The setter is that seam, and it is declared next to a comment saying so.

**The two invariants this file exists to hold, quoted so they cannot be got subtly wrong:**
- §2.4 — *"You cannot feed the dog without having just re-asserted the pump's idle state."* The pump write comes **first**, the feed **second**, in one function, with nothing between them.
- §2.4 — *"There is exactly one place in the program where feeding is deliberately suspended, and it is `hal_wdt_alive()`'s probe (§2.5). Nothing else may skip a `safety_tick()`."*

---

1. [ ] **Write the two failing cases** in `test/test_dose/test_dose.cpp`, registered with `RUN_TEST` in `main()`.

```c
#include "safety.h"

/* §2.4: idle ACTIVELY re-asserts OFF, every pass, using the whole-word form of §2.1 —
   so the re-assert repairs a stray pinMode on D6 as well as a stray level. Then, and
   only then, the dog is fed. */
static void test_idle_safety_tick_rewrites_the_off_level(void) {
  sim_events_clear();
  safety_tick();
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  TEST_ASSERT_EQUAL_UINT32(2u, (uint32_t)n);
  TEST_ASSERT_EQUAL_INT(SIM_EV_PUMP_WRITE, (int)ev[0].kind);
  TEST_ASSERT_TRUE(ev[0].arg & SIM_PFS_DIR_OUT);      /* direction restated */
  TEST_ASSERT_FALSE(ev[0].arg & SIM_PFS_LEVEL_HI);    /* at the OFF level */
  TEST_ASSERT_EQUAL_INT(SIM_EV_WDT_FEED, (int)ev[1].kind);   /* and the feed comes SECOND */

  /* mid-dose the pump write is skipped — but the feed is not, which is what makes a
     60 s dose legal under a 5592 ms window (§3). */
  safety_set_dosing(true);
  sim_events_clear();
  safety_tick();
  n = sim_events(&ev);
  TEST_ASSERT_EQUAL_UINT32(1u, (uint32_t)n);
  TEST_ASSERT_EQUAL_INT(SIM_EV_WDT_FEED, (int)ev[0].kind);
  safety_set_dosing(false);
}

static void test_safety_wait_ms_feeds_on_every_iteration(void) {
  sim_events_clear();
  uint32_t t0 = hal_millis();
  safety_wait_ms(100);
  uint32_t t1 = hal_millis();
  TEST_ASSERT_TRUE(t1 - t0 >= 100u);

  const sim_ev_t *ev; size_t n = sim_events(&ev);
  uint32_t feeds = 0, prev = 0;
  bool have_prev = false;
  for (size_t i = 0; i < n; ++i) {
    if (ev[i].kind != SIM_EV_WDT_FEED) continue;
    if (have_prev) TEST_ASSERT_TRUE(ev[i].at_ms - prev <= 1u);   /* no gap wider than a tick */
    prev = ev[i].at_ms; have_prev = true; feeds++;
  }
  /* the fake advances 1 ms per hal_millis() call, so a 100 ms wait is 99 iterations */
  TEST_ASSERT_TRUE(feeds >= 99u);
}
```

2. [ ] **Run it and watch it fail on the missing header.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose
```

Expected: `test/test_dose/test_dose.cpp:N:10: fatal error: safety.h: No such file or directory`.

3. [ ] **Write `include/safety.h`.** Tasks 15–19 append to this header; nothing here is removed.

```c
/* include/safety.h — D6's whole story. READ THIS FILE FIRST.
   Includes neither link.h, Network.h nor WiFiS3.h: this layer *cannot* make a network
   call, and tools/check.sh greps for it (§3, §9). */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Pump idle re-asserted, then the dog fed — in that order, in one function, with nothing
   between them (§2.4). Called at the top of loop(), inside safety_wait_ms()'s loop, and
   inside dose_run()'s loop. Nowhere else. */
void safety_tick(void);

/* A bounded wait that calls safety_tick() on every iteration. Every loop in the program
   that can iterate over an I2C transfer, a modem call or a millisecond of wall clock
   uses this or its own safety_tick() (§3). */
void safety_wait_ms(uint32_t ms);

bool safety_dosing(void);

/* The only production writer is dose_run() (task 17), which sets g_dosing directly; this
   exists so the host suites can reproduce a dose in flight — sensors.cpp's recovery guard
   (§2.13) has to be testable before dose_run() is written. */
void safety_set_dosing(bool on);
```

4. [ ] **Write `src/safety.cpp`** — §2.4 verbatim.

```c
/* src/safety.cpp — safety_tick, safety_wait_ms, and (from drop 2) the float debounce,
   the dry latch, the contradiction latch and dose_run(). Spec §2. */
#include "safety.h"
#include "hal.h"

static bool g_dosing;                    /* true only between the ON and OFF writes */

bool safety_dosing(void) { return g_dosing; }
void safety_set_dosing(bool on) { g_dosing = on; }

void safety_tick(void) {
  if (!g_dosing) hal_pump_write(false);  /* idle ACTIVELY re-asserts OFF, every pass */
  hal_wdt_feed();                        /* the ONE feeder in the program */
}

void safety_wait_ms(uint32_t ms) {
  uint32_t t0 = hal_millis();
  while (hal_millis() - t0 < ms) safety_tick();
}
```

5. [ ] **Run the suite and watch both cases pass.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose
```

Expected: `18 Tests 0 Failures 0 Ignored / OK` (task 4's sixteen plus this task's two). If `test_idle_safety_tick_rewrites_the_off_level` reports three events instead of two, `hal_pump_write()` in the fake is emitting more than one event per write — task 3 step 6 fixes that, not this file.

6. [ ] **Check the board env builds and commit.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio run -e uno_r4_wifi && pio test -e native -f test_dose
git add include/safety.h src/safety.cpp test/test_dose/test_dose.cpp
git commit -m "safety_tick(): the pump's idle state re-asserted, then the dog fed

The ordering carries the argument (§2.4). Because hal_pump_write() is §2.1's whole-word
R_IOPORT_PinCfg form — direction and level in one PmnPFS write — the idle re-assert
repairs a stray pinMode on D6 as well as a stray level, and you cannot reach the feed
without having just done it.

safety.cpp lands two drops before dose_run() because sensors_sweep() must feed between
channels and ui_poll() between LCD characters. Drop 2 fills in the rest.

Deviation: safety_set_dosing() is a fourth declaration the skeleton does not list. It
exists so §9's test_i2c_recovery_never_runs_while_the_pump_is_asserted can be written
in drop 1, before dose_run() exists to set g_dosing itself."
```

---

### Task 6: pulses.cpp — the two edge counters, the gap reject, the rate estimator and the leak watch

**Drop 1.**

**Files:**
- Create: `include/pulses.h`, `src/pulses.cpp`, `test/test_sensors/test_sensors.cpp` (the first task to touch this suite)
- Modify: `src/hal_sim.cpp` (drive `pulses_isr_flow()` / `pulses_isr_screw()` from the fake pump and screw models), `test/test_dose/test_dose.cpp`
- Test: `test/test_sensors/test_sensors.cpp`, `test/test_dose/test_dose.cpp`

**Interfaces:**

*Consumes.* From `include/hal.h` (task 3): `uint32_t hal_micros(void);` — which in the fake **reads** the clock and never advances it — and `uint32_t hal_millis(void);`, which advances the rig by 1 ms and runs every model. From `include/config.h` (task 2): `PB_FLOW_MIN_GAP_US` 500, `PB_SCREW_MIN_GAP_US` 2000, `PB_FLOW_RATE_WINDOW_MS` 100, `PB_COAST_MS` 2000, `PB_PULSES_PER_L_DEFAULT` 5880, `PB_PRIME_MS_DEFAULT` 3000, `PB_FLOW_MAX_HZ` 1200.

*Produces.*

```c
void     pulses_begin(void);
void     pulses_isr_flow(void);        /* the D2 ISR body */
void     pulses_isr_screw(void);       /* the D3 ISR body */
uint32_t pulses_flow(void);            /* torn-read-safe snapshot */
uint32_t pulses_screw(void);
uint32_t pulses_flow_rate(void);       /* Hz, over a 100 ms tumbling window */
uint32_t pulses_to_ml(uint32_t pulses, uint16_t pulses_per_l);
void     pulses_leak_rearm_at(uint32_t at_ms);
void     pulses_leak_poll(bool pump_on);
uint32_t pulses_leak_count(void);
bool     pulses_leak_seen(void);
```

Task 8 (`hal_uno.cpp`) calls `pulses_isr_flow` / `pulses_isr_screw` from the two real ISRs. Task 14 (`cart`) reads `pulses_screw()`. Tasks 17–18 (`dose_run()`) read `pulses_flow()`, `pulses_flow_rate()` and `pulses_to_ml()`, and call `pulses_leak_rearm_at(hal_millis() + PB_COAST_MS)` at the end of every dose. Task 22 (`report_build`) reads `pulses_leak_count()` for `ch205`.

**There is NO leak latch** (spec §1, *Two things that are deliberately NOT here*): pulses with the pump off raise the count and `err=leak`; they never block a dose. A latch with no clear bricks the rig on the first stray pulse during 7b priming.

**One deviation, recorded in the commit message.** `pulses.h` declares one extra symbol under `#if PB_SIM`: `void pulses_test_tear_next(uint32_t edges);`. The host has no preemption, so the double-read snapshot cannot be proved without injecting an edge *between* the two reads, and the injection point is inside `pulses.cpp`. `PB_SIM` is defined in `[env:native]` and `[env:uno_r4_wifi_sim]` and never in the bench binary, so the seam does not ship.

---

1. [ ] **Write the four failing pulse cases** in a new `test/test_sensors/test_sensors.cpp`.

```c
#include <unity.h>
#include "../support/harness.h"
#include "config.h"
#include "pulses.h"

void setUp(void)    { pb_test_setup(); }
void tearDown(void) { pb_test_teardown(); }

/* §2.14: PB_FLOW_MIN_GAP_US is honest about only biting above 2 kHz, which is why the
   two rate rules exist above it. PB_SCREW_MIN_GAP_US is four times wider. */
static void test_edges_closer_than_the_minimum_gap_are_rejected(void) {
  pulses_begin();
  pulses_isr_flow();
  pulses_isr_flow();                     /* same microsecond: rejected */
  TEST_ASSERT_EQUAL_UINT32(1u, pulses_flow());
  pb_advance(1);                         /* 1000 us > 500 us */
  pulses_isr_flow();
  TEST_ASSERT_EQUAL_UINT32(2u, pulses_flow());

  pulses_isr_screw();
  pulses_isr_screw();
  TEST_ASSERT_EQUAL_UINT32(1u, pulses_screw());
  pb_advance(1);                         /* 1000 us < 2000 us: still rejected */
  pulses_isr_screw();
  TEST_ASSERT_EQUAL_UINT32(1u, pulses_screw());
  pb_advance(2);                         /* 3000 us since the accepted edge */
  pulses_isr_screw();
  TEST_ASSERT_EQUAL_UINT32(2u, pulses_screw());
}

/* §2.14: at the ISR's own 2 kHz ceiling a 250 ml target is reached in ~625 ms, so any
   estimator slower than 100 ms loses the race the in-dose rate rules have to win. */
static void test_the_rate_estimator_reports_over_a_hundred_millisecond_window(void) {
  pulses_begin();
  sim_flow_storm(2000);                  /* edges 500 us apart: at the reject's boundary */
  pb_advance(100);
  uint32_t hz = pulses_flow_rate();
  TEST_ASSERT_TRUE(hz >= 1500u);
  TEST_ASSERT_TRUE(hz <= 2500u);
  TEST_ASSERT_TRUE(hz > (uint32_t)PB_FLOW_MAX_HZ);   /* what the dose-loop rule needs */
  /* the previous window's value stands while the current one fills */
  TEST_ASSERT_EQUAL_UINT32(hz, pulses_flow_rate());
}

static void test_a_counter_snapshot_is_never_torn_by_an_edge(void) {
  pulses_begin();
  pb_advance(1);
  pulses_isr_flow();
  TEST_ASSERT_EQUAL_UINT32(1u, pulses_flow());
  pulses_test_tear_next(1u);             /* an edge lands BETWEEN the snapshot's two reads */
  pb_advance(1);
  TEST_ASSERT_EQUAL_UINT32(2u, pulses_flow());
  TEST_ASSERT_EQUAL_UINT32(2u, pulses_flow());   /* and it settled */
}

/* §7: PB_COAST_MS — impeller spin-down is not a leak. And §1: there is no leak LATCH. */
static void test_leak_does_not_latch_from_coast_down_pulses_after_a_dose(void) {
  pulses_begin();
  pulses_leak_rearm_at(hal_millis() + PB_COAST_MS);
  sim_flow_storm(50);                    /* the impeller coasting down */
  pb_advance(1000);
  sim_flow_storm(0);
  pulses_leak_poll(false);
  TEST_ASSERT_EQUAL_UINT32(0u, pulses_leak_count());
  TEST_ASSERT_FALSE(pulses_leak_seen());

  pb_advance(1500);                      /* past the blanking window */
  pulses_leak_poll(false);               /* arms and rebases */
  sim_flow_storm(50);
  pb_advance(1000);
  sim_flow_storm(0);
  pulses_leak_poll(false);
  TEST_ASSERT_TRUE(pulses_leak_count() > 0u);
  TEST_ASSERT_TRUE(pulses_leak_seen());
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_edges_closer_than_the_minimum_gap_are_rejected);
  RUN_TEST(test_the_rate_estimator_reports_over_a_hundred_millisecond_window);
  RUN_TEST(test_a_counter_snapshot_is_never_torn_by_an_edge);
  RUN_TEST(test_leak_does_not_latch_from_coast_down_pulses_after_a_dose);
  return UNITY_END();
}
```

2. [ ] **Write the arithmetic case** in `test/test_dose/test_dose.cpp`, registered with `RUN_TEST` in `main()`. Add `#include "pulses.h"`.

```c
/* §6: target is ml * cfg / 1000 — MULTIPLY FIRST. The reverse order truncates the
   calibration to whole pulses per millilitre and under-delivers 15% at the nominal 5880.
   And `cal 0` used to make pulses_to_ml divide by zero: the Cortex-M4's UDIV returns 0
   without DIV_0_TRP, so the flood happened and the report said nothing came out. */
static void test_ml_from_pulses_rounds_down_and_does_not_overflow(void) {
  TEST_ASSERT_EQUAL_UINT32(100u, pulses_to_ml(588u, 5880u));
  TEST_ASSERT_EQUAL_UINT32(9u,   pulses_to_ml(58u, 5880u));    /* 9.86 ml, rounded DOWN */
  TEST_ASSERT_EQUAL_UINT32(250u, pulses_to_ml(1470u, 5880u));
  TEST_ASSERT_EQUAL_UINT32(0u,   pulses_to_ml(0u, 5880u));
  TEST_ASSERT_EQUAL_UINT32(0u,   pulses_to_ml(1000u, 0u));     /* never a UDIV-returns-0 lie */
  /* past UINT32_MAX/1000 the multiply-first form would wrap; the split form does not */
  TEST_ASSERT_EQUAL_UINT32(850340u,     pulses_to_ml(5000000u, 5880u));
  TEST_ASSERT_EQUAL_UINT32(2147483647u, pulses_to_ml(2147483647u, 1000u));
}
```

3. [ ] **Run both suites and watch them fail on the missing header.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_sensors -f test_dose
```

Expected: `test/test_sensors/test_sensors.cpp:4:10: fatal error: pulses.h: No such file or directory` for both suites.

4. [ ] **Write `include/pulses.h`.**

```c
/* include/pulses.h — the two interrupt counters, the per-pin gap reject, torn-read-safe
   snapshots, the rate estimator, pulses->ml, and the leak watch. Spec §1, §2.14, §7. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

void     pulses_begin(void);

/* The ISR bodies. hal_uno.cpp's two ISRs call these and do nothing else; each rejects an
   edge closer than its own minimum gap, using hal_micros(). */
void     pulses_isr_flow(void);
void     pulses_isr_screw(void);

/* Read, re-read, repeat until two reads agree. NO interrupt masking: masking is what
   drops edges, and a dropped screw edge is lost cart position, silently. */
uint32_t pulses_flow(void);
uint32_t pulses_screw(void);

/* Hz over a PB_FLOW_RATE_WINDOW_MS tumbling window; the previous window's value stands
   while the current one fills. No floats, no ring buffer. */
uint32_t pulses_flow_rate(void);

/* Multiply first, divide second (§6). Returns 0 for a zero calibration rather than
   trusting UDIV. */
uint32_t pulses_to_ml(uint32_t pulses, uint16_t pulses_per_l);

/* The leak watch. dose_run() calls pulses_leak_rearm_at(hal_millis() + PB_COAST_MS) at
   the end of every dose, so impeller spin-down is not a leak. There is NO latch: these
   pulses raise ch205 and err=leak, they never block a dose (§1). */
void     pulses_leak_rearm_at(uint32_t at_ms);
void     pulses_leak_poll(bool pump_on);
uint32_t pulses_leak_count(void);
bool     pulses_leak_seen(void);

#if PB_SIM
/* Host-only: inject `edges` flow edges between the two reads of the next snapshot. The
   host has no preemption, so this is the only way to prove the double-read loop. Never
   compiled into the bench binary. */
void     pulses_test_tear_next(uint32_t edges);
#endif
```

5. [ ] **Write `src/pulses.cpp`.**

```c
/* src/pulses.cpp — D2/D3 ISR bodies, gap reject, snapshots, rate, ml, leak watch. */
#include "pulses.h"
#include "hal.h"
#include "config.h"

static volatile uint32_t g_flow, g_screw;
static volatile uint32_t g_flow_last_us, g_screw_last_us;

static uint32_t g_win_t0, g_win_base, g_rate_prev;

static uint32_t g_leak_rearm_ms, g_leak_base, g_leak_count;
static bool     g_leak_armed;

#if PB_SIM
static uint32_t g_tear_pending;
void pulses_test_tear_next(uint32_t edges) { g_tear_pending = edges; }
static void tear_(void) {
  if (!g_tear_pending) return;
  uint32_t n = g_tear_pending;
  g_tear_pending = 0;
  while (n--) {
    g_flow_last_us = hal_micros() - (uint32_t)PB_FLOW_MIN_GAP_US - 1u;   /* force acceptance */
    pulses_isr_flow();
  }
}
#else
static void tear_(void) { }
#endif

void pulses_begin(void) {
  g_flow = g_screw = 0;
  /* back-date both, so the FIRST edge after boot is never rejected as too close */
  g_flow_last_us  = hal_micros() - (uint32_t)PB_FLOW_MIN_GAP_US  - 1u;
  g_screw_last_us = hal_micros() - (uint32_t)PB_SCREW_MIN_GAP_US - 1u;
  g_win_t0 = hal_millis();
  g_win_base = 0; g_rate_prev = 0;
  g_leak_rearm_ms = g_win_t0; g_leak_base = 0; g_leak_count = 0; g_leak_armed = false;
}

void pulses_isr_flow(void) {
  uint32_t now = hal_micros();
  if ((uint32_t)(now - g_flow_last_us) < (uint32_t)PB_FLOW_MIN_GAP_US) return;
  g_flow_last_us = now;
  g_flow++;
}

void pulses_isr_screw(void) {
  uint32_t now = hal_micros();
  if ((uint32_t)(now - g_screw_last_us) < (uint32_t)PB_SCREW_MIN_GAP_US) return;
  g_screw_last_us = now;
  g_screw++;
}

uint32_t pulses_flow(void) {
  uint32_t a, b;
  do { a = g_flow; tear_(); b = g_flow; } while (a != b);
  return a;
}

uint32_t pulses_screw(void) {
  uint32_t a, b;
  do { a = g_screw; b = g_screw; } while (a != b);
  return a;
}

uint32_t pulses_flow_rate(void) {
  uint32_t now = hal_millis();
  uint32_t elapsed = now - g_win_t0;
  if (elapsed >= (uint32_t)PB_FLOW_RATE_WINDOW_MS) {
    uint32_t n = pulses_flow();
    g_rate_prev = ((n - g_win_base) * 1000u) / elapsed;
    g_win_base = n;
    g_win_t0 = now;
  }
  return g_rate_prev;
}

uint32_t pulses_to_ml(uint32_t pulses, uint16_t pulses_per_l) {
  if (pulses_per_l == 0u) return 0u;
  if (pulses <= (0xFFFFFFFFu / 1000u)) return (pulses * 1000u) / pulses_per_l;
  return (pulses / pulses_per_l) * 1000u + ((pulses % pulses_per_l) * 1000u) / pulses_per_l;
}

void pulses_leak_rearm_at(uint32_t at_ms) {
  g_leak_rearm_ms = at_ms;
  g_leak_armed = false;
  g_leak_base = pulses_flow();
}

void pulses_leak_poll(bool pump_on) {
  if (pump_on) {                       /* pulses with the pump ON are just the dose */
    g_leak_armed = false;
    g_leak_base = pulses_flow();
    return;
  }
  if (!g_leak_armed) {
    if ((int32_t)(hal_millis() - g_leak_rearm_ms) < 0) {   /* still coasting down */
      g_leak_base = pulses_flow();
      return;
    }
    g_leak_armed = true;
    g_leak_base = pulses_flow();
    return;
  }
  uint32_t n = pulses_flow();
  if (n > g_leak_base) {
    g_leak_count += n - g_leak_base;
    g_leak_base = n;
  }
}

uint32_t pulses_leak_count(void) { return g_leak_count; }
bool     pulses_leak_seen(void)  { return g_leak_count > 0u; }
```

6. [ ] **Drive the two ISRs from the fake.** In `src/hal_sim.cpp`: add `#include "pulses.h"` and, just above `advance_1ms_()`, the two rate models and the emitter; then replace `advance_1ms_()` itself.

```c
static uint32_t g_next_flow_us, g_next_screw_us;

static uint32_t sim_flow_hz_(void) {
  if (g_storm_hz) return g_storm_hz;
  if (g_pump_on && (g_ms - g_pump_on_at_ms) >= (uint32_t)PB_PRIME_MS_DEFAULT && g_flow_ml_s)
    return ((uint32_t)g_flow_ml_s * (uint32_t)PB_PULSES_PER_L_DEFAULT) / 1000u;
  if (!g_pump_on && g_leak) return 1u;      /* a slow weep past a closed gate */
  return 0u;
}

static uint32_t sim_screw_hz_(void) {
  if (g_stall || g_servo_us == 1500u || g_servo_us == 0u) return 0u;
  return 20u;                               /* task 14 owns the bench's real screw rate */
}

/* An edge lands at its OWN microsecond inside the step, so the ISR's gap reject measures
   the interval the fake scheduled and not the reading of it. */
static void emit_(uint32_t hz, uint32_t *next_us, uint32_t target_us, void (*isr)(void)) {
  if (hz == 0u) { *next_us = target_us; return; }
  uint32_t period = 1000000u / hz;
  if (period == 0u) period = 1u;
  if (*next_us < g_us) *next_us = g_us;
  while (*next_us <= target_us) {
    g_us = *next_us;
    *next_us += period;
    isr();
  }
}

static void advance_1ms_(void) {
  const uint32_t target = g_us + 1000u;
  tick_models_(1000u);
  emit_(sim_flow_hz_(),  &g_next_flow_us,  target, pulses_isr_flow);
  emit_(sim_screw_hz_(), &g_next_screw_us, target, pulses_isr_screw);
  g_us = target;
  g_ms = g_us / 1000u;
}
```

   And in `sim_reset()`, beside the other counters: `g_next_flow_us = g_next_screw_us = 0;`.

7. [ ] **Run both suites and watch the five cases pass.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_sensors -f test_dose
```

Expected: `test_sensors` reports `4 Tests 0 Failures 0 Ignored / OK` and `test_dose` `19 Tests 0 Failures 0 Ignored / OK` (task 5's eighteen plus the one this task adds there). If the rate case reports a number near 1000 rather than 2000, `emit_()` is spacing edges by a whole millisecond instead of by `period` — re-read step 6.

8. [ ] **Check the board env builds and commit.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio run -e uno_r4_wifi && pio test -e native
git add include/pulses.h src/pulses.cpp src/hal_sim.cpp test/test_sensors/test_sensors.cpp test/test_dose/test_dose.cpp
git commit -m "pulses: two counters, a per-pin gap reject, a 100 ms rate window, a leak watch

The snapshots read and re-read until two reads agree, and mask nothing: masking is what
drops edges, and a dropped screw edge is lost cart position with no symptom.

pulses_to_ml multiplies first and divides second — the reverse order truncates the
calibration to whole pulses per millilitre and under-delivers 15% at the nominal 5880 —
and returns 0 for a zero calibration rather than trusting UDIV, which returns 0 without
DIV_0_TRP and would have acked flow_ml=0 while the flood happened.

There is deliberately NO leak latch (§1): pulses with the pump off raise the count and
err=leak, they never block a dose. A latch with no clear bricks the rig on the first
stray pulse during 7b priming.

Deviation: pulses.h declares pulses_test_tear_next() under #if PB_SIM. The host has no
preemption, so the double-read loop cannot be proved without injecting an edge between
the two reads, and PB_SIM is never defined in the bench binary."
```

---

### Task 7: sensors.cpp — the expander, the mux discipline, the open-channel canary and I2C health

**Drop 1.**

**Files:**
- Create: `include/sensors.h`, `src/sensors.cpp`
- Modify: `src/hal_sim.cpp` (the ADC settling model behind "discard the first conversion"), `test/test_sensors/test_sensors.cpp`
- Test: `test/test_sensors/test_sensors.cpp`

**Interfaces:**

*Consumes.* From `include/hal.h` (task 3): `bool hal_i2c_write16(uint8_t addr, uint16_t bits);`, `bool hal_i2c_read16(uint8_t addr, uint16_t *bits);`, `bool hal_i2c_probe(uint8_t addr);`, `bool hal_i2c_recover(void);`, `uint16_t hal_adc_read(void);`, `void hal_delay_us(uint16_t us);`, `int hal_pin_read(uint8_t pin);`. From `include/safety.h` (task 5): `void safety_tick(void);`, `void safety_wait_ms(uint32_t ms);`, `bool safety_dosing(void);`, `void safety_set_dosing(bool on);`. From `include/config.h` and `include/pins.h` (task 2): `PB_CHANNELS` 6, `PB_CANARY_CHANNEL` 15, `PB_I2C_FAIL_LIMIT` 3, `PB_I2C_BACKOFF_MS` 5000, `PB_I2C_RECOVER_CLOCKS` 9, `PIN_HALL_FLOAT` 5, `I2C_ADDR_EXPANDER` 0x20.

*Produces.*

```c
bool     sensors_begin(void);
bool     sensors_select(uint8_t ch);
bool     sensors_read_raw(uint8_t ch, uint16_t *raw);
bool     sensors_sweep(void);
uint16_t sensors_value(uint8_t ch);
bool     sensors_valid(uint8_t ch);
bool     sensors_stuck(void);
bool     sensors_home_hall(bool *home);
bool     sensors_i2c_healthy(void);
uint32_t sensors_i2c_errors(void);
uint32_t sensors_i2c_txn_per_min(void);
uint32_t sensors_float_change_age_s(void);
void     sensors_scan(char *out, size_t cap);
```

**`sensors_sweep()` has exactly one named owner and a stated cadence, settled here so no later task can leave it uncalled or call it twice.** The owner is **`src/netfsm.cpp`'s `NET_IDLE` pass (task 24 step 4)**: it runs one sweep per report cycle, immediately before `report_stamp()` and `report_build()` (task 22) read `sensors_value()` / `sensors_valid()`. That pass issues **zero** AT commands, which is how spec §3's rule — *"`ui_poll()` and `sensors_sweep()` are both skipped in any pass where a modem command ran, so the loop pass cannot stack a 7 s wedged sweep on top of 2.4 s of modem"* — holds **by construction** rather than by a flag `main.cpp` would have to remember to consult. **`main.cpp`'s `loop()` does NOT call `sensors_sweep()`, and there is no `sensors_poll()` anywhere in this plan.** What `loop()` does carry is `pulses_leak_poll(safety_dosing())` (task 12 step 4), which is a different function in a different module and is `ch205`'s only driver. `cart` (task 14) and `dose_run()` (tasks 17–18) call `sensors_home_hall()` directly; nothing else calls `sensors_sweep()`.

**Every failure returns `false` and never a value.** `sensors_home_hall()` in particular reports a bus error as `false`, never as "not home" — `cad/wiring/README.md`: *"A failed or timed-out I2C read is not a zero: the firmware must treat it as 'home unknown' and refuse to move or pump."*

**One deviation, recorded in the commit message.** `hal.h`'s comment says `hal_i2c_recover()` "refuses while dosing". The guard is implemented **here**, in `sensors.cpp`, one line above the call, because the HAL must not call up into the safety layer. §2.13's invariant is unchanged: *"`hal_i2c_recover()` bit-bangs A4/A5 — the mux select lines and the home hall, the input that gates the pump. It therefore begins `if (g_dosing) return false;`, and a back-off that expires mid-dose simply stays expired until the dose ends."*

---

1. [ ] **Write the nine failing cases** in `test/test_sensors/test_sensors.cpp`, registered with `RUN_TEST` in `main()` beside task 6's four.

```c
#include "sensors.h"
#include "safety.h"
#include "pins.h"

/* The PCF8575 is quasi-bidirectional: an input must be written HIGH to be readable, and
   P4 is the home hall (cad/wiring/nets.py P4: "10 k pull-up (R3); write P4 HIGH before
   reading"). So every select writes P4..P15 HIGH and only P0..P3 carry the channel. */
static void test_select_holds_p4_high_so_the_home_hall_stays_readable(void) {
  (void)sensors_begin();
  sim_events_clear();
  TEST_ASSERT_TRUE(sensors_select(3));
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  uint32_t writes = 0;
  for (size_t i = 0; i < n; ++i) {
    if (ev[i].kind != SIM_EV_I2C_WRITE || ev[i].pin != I2C_ADDR_EXPANDER) continue;
    TEST_ASSERT_EQUAL_UINT32(0xFFF3u, ev[i].arg);
    writes++;
  }
  TEST_ASSERT_EQUAL_UINT32(1u, writes);
}

/* Bring-up 2's recipe verbatim: select, >= 1 ms, read twice, keep the SECOND. The first
   conversion after a select still carries the previous channel on a 10 k source. */
static void test_read_discards_the_first_conversion_and_keeps_the_second(void) {
  (void)sensors_begin();
  sim_set_channel(0, 1111);
  sim_set_channel(1, 2222);
  uint16_t v = 0;
  TEST_ASSERT_TRUE(sensors_read_raw(0, &v));
  sim_events_clear();
  TEST_ASSERT_TRUE(sensors_read_raw(1, &v));
  TEST_ASSERT_EQUAL_UINT16(2222u, v);
  TEST_ASSERT_EQUAL_UINT32(2u, pb_count(SIM_EV_ADC));
}

static void test_an_i2c_error_is_reported_as_error_not_as_zero(void) {
  (void)sensors_begin();
  sim_set_channel(2, 9999);
  sim_set_i2c_fail(true);
  uint16_t v = 0x5A5Au;
  TEST_ASSERT_FALSE(sensors_read_raw(2, &v));
  TEST_ASSERT_EQUAL_UINT16(0x5A5Au, v);        /* untouched: never a value on failure */
  bool home = true;
  TEST_ASSERT_FALSE(sensors_home_hall(&home)); /* an error, never "not home" */
}

/* §3: a healthy sweep is ~18 ms and a wedged one is 7 s at the core's fixed 1000 ms
   transfer timeout. The dog's window is 5592 ms. */
static void test_sweep_feeds_the_watchdog_between_channels(void) {
  (void)sensors_begin();
  sim_events_clear();
  (void)sensors_sweep();
  TEST_ASSERT_TRUE(pb_count(SIM_EV_WDT_FEED) >= (uint32_t)PB_CHANNELS + 1u);
  const sim_ev_t *ev; size_t n = sim_events(&ev);
  uint32_t writes_since_feed = 0;
  for (size_t i = 0; i < n; ++i) {
    if (ev[i].kind == SIM_EV_WDT_FEED) writes_since_feed = 0;
    else if (ev[i].kind == SIM_EV_I2C_WRITE) {
      writes_since_feed++;
      TEST_ASSERT_TRUE(writes_since_feed <= 1u);   /* never two transfers un-fed */
    }
  }
}

static void test_sweep_reads_the_open_canary_channel_every_time(void) {
  (void)sensors_begin();
  for (uint8_t s = 0; s < 2; ++s) {
    sim_events_clear();
    (void)sensors_sweep();
    const sim_ev_t *ev; size_t n = sim_events(&ev);
    uint32_t canary_selects = 0;
    for (size_t i = 0; i < n; ++i)
      if (ev[i].kind == SIM_EV_I2C_WRITE && (ev[i].arg & 0x0Fu) == (uint32_t)PB_CANARY_CHANNEL)
        canary_selects++;
    TEST_ASSERT_EQUAL_UINT32(1u, canary_selects);
  }
}

/* §5: an unpowered mux, a broken S-line or a floating EN gives the SAME ADC value on
   every channel with no error raised anywhere, and from butler's side that is
   byte-identical to five equally-dry pots. So the wired channels are omitted, not sent. */
static void test_a_stuck_mux_is_reported_as_an_error_not_as_readings(void) {
  (void)sensors_begin();
  for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) sim_set_channel(ch, (uint16_t)(1000u + ch));
  sim_set_channel(PB_CANARY_CHANNEL, 4321u);
  TEST_ASSERT_TRUE(sensors_sweep());
  TEST_ASSERT_FALSE(sensors_stuck());
  TEST_ASSERT_TRUE(sensors_valid(0));
  TEST_ASSERT_EQUAL_UINT16(1000u, sensors_value(0));

  sim_set_mux_stuck(true);
  TEST_ASSERT_FALSE(sensors_sweep());
  TEST_ASSERT_TRUE(sensors_stuck());
  for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) TEST_ASSERT_FALSE(sensors_valid(ch));
}

static void test_three_consecutive_failures_back_off_and_mark_the_bus_unhealthy(void) {
  (void)sensors_begin();
  TEST_ASSERT_TRUE(sensors_i2c_healthy());
  sim_set_i2c_fail(true);
  for (uint8_t i = 0; i < PB_I2C_FAIL_LIMIT; ++i) (void)sensors_select(0);
  TEST_ASSERT_FALSE(sensors_i2c_healthy());
  TEST_ASSERT_TRUE(sensors_i2c_errors() >= (uint32_t)PB_I2C_FAIL_LIMIT);

  sim_events_clear();                       /* while backing off, the bus is not touched */
  TEST_ASSERT_FALSE(sensors_select(0));
  TEST_ASSERT_EQUAL_UINT32(0u, pb_count(SIM_EV_I2C_WRITE));

  sim_set_i2c_fail(false);
  pb_advance((uint32_t)PB_I2C_BACKOFF_MS + 1u);
  TEST_ASSERT_TRUE(sensors_select(0));
  TEST_ASSERT_TRUE(sensors_i2c_healthy());
}

/* §2.13: A4/A5 are the mux select lines and the home hall — the input that gates the
   pump. A back-off that expires mid-dose simply stays expired until the dose ends. */
static void test_i2c_recovery_never_runs_while_the_pump_is_asserted(void) {
  (void)sensors_begin();
  sim_set_i2c_fail(true);
  for (uint8_t i = 0; i < PB_I2C_FAIL_LIMIT; ++i) (void)sensors_select(0);
  TEST_ASSERT_FALSE(sensors_i2c_healthy());
  sim_set_i2c_fail(false);
  pb_advance((uint32_t)PB_I2C_BACKOFF_MS + 1u);

  safety_set_dosing(true);
  sim_events_clear();
  TEST_ASSERT_FALSE(sensors_select(0));
  TEST_ASSERT_EQUAL_UINT32(0u, pb_count(SIM_EV_I2C_WRITE));
  TEST_ASSERT_EQUAL_UINT32(0u, pb_count(SIM_EV_PIN_CFG));   /* no bit-bang either */
  TEST_ASSERT_FALSE(sensors_i2c_healthy());

  safety_set_dosing(false);
  TEST_ASSERT_TRUE(sensors_select(0));      /* and it runs the moment the dose ends */
}

/* A FIXED count, never an "until SDA releases" condition — that condition is an
   unbounded loop on a bus a stuck device is holding down (§2.13). */
static void test_recovery_is_a_fixed_nine_clocks_with_sda_held_low(void) {
  (void)sensors_begin();
  sim_set_i2c_fail(true);
  for (uint8_t i = 0; i < PB_I2C_FAIL_LIMIT; ++i) (void)sensors_select(0);
  sim_set_i2c_fail(false);
  pb_advance((uint32_t)PB_I2C_BACKOFF_MS + 1u);
  sim_events_clear();
  TEST_ASSERT_TRUE(sensors_select(0));

  const sim_ev_t *ev; size_t n = sim_events(&ev);
  uint32_t scl = 0, sda = 0;
  for (size_t i = 0; i < n; ++i) {
    if (ev[i].kind != SIM_EV_PIN_CFG) continue;
    if (ev[i].pin == 19u) scl++;            /* A5 == PIN_A0 + 5 == 19 */
    if (ev[i].pin == 18u) sda++;            /* A4 == 18 */
  }
  TEST_ASSERT_EQUAL_UINT32(2u * (uint32_t)PB_I2C_RECOVER_CLOCKS, scl);  /* nine high/low pairs */
  TEST_ASSERT_EQUAL_UINT32(1u, sda);        /* SDA released once, then the fixed clocks */
}
```

2. [ ] **Run it and watch it fail on the missing header.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_sensors
```

Expected: `test/test_sensors/test_sensors.cpp:N:10: fatal error: sensors.h: No such file or directory`.

3. [ ] **Write `include/sensors.h`.**

```c
/* include/sensors.h — PCF8575 + mux + the open-channel canary + the home hall + I2C
   health. Spec §1's module table, §2.10, §2.13, §5, §7.
   EVERY failure returns false and never a value. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

bool     sensors_begin(void);                   /* probe, one recovery, reset the counters */
bool     sensors_select(uint8_t ch);            /* P0..P3 = ch, P4..P15 HIGH, always */
bool     sensors_read_raw(uint8_t ch, uint16_t *raw);  /* select, >= 1 ms, read twice, keep 2nd */

/* ONE caller: netfsm.cpp's NET_IDLE pass (task 24), once per report cycle, immediately
   before report_build() reads the channels. That pass issues zero AT commands, which is how
   §3's "skipped in any pass where a modem command ran" holds by construction. main.cpp's
   loop() does NOT call this. Reads ch0..ch5 AND the unwired canary channel, feeding between
   channels. Every failure returns false; the canary matching every wired channel is one. */
bool     sensors_sweep(void);

uint16_t sensors_value(uint8_t ch);
bool     sensors_valid(uint8_t ch);
bool     sensors_stuck(void);                   /* the canary equalled every wired channel */

/* false == bus error, NEVER "not home". A failed read must refuse to move or pump. */
bool     sensors_home_hall(bool *home);

bool     sensors_i2c_healthy(void);
uint32_t sensors_i2c_errors(void);
uint32_t sensors_i2c_txn_per_min(void);         /* `status` prints it so the cost stays visible */

/* Seconds since D5 last changed state: a bare non-negative integer, ALWAYS. 0 before D5
   has ever moved, never a sentinel — _int_in rejects a leading '-' and any non-digit, so
   a "-1" or "never" in a chN would 400 the whole report (§2.10). Staleness is REPORTED,
   never enforced. */
uint32_t sensors_float_change_age_s(void);

void     sensors_scan(char *out, size_t cap);   /* the `i2c` console command */
```

4. [ ] **Write `src/sensors.cpp`.**

```c
/* src/sensors.cpp — the expander, the mux discipline, the canary, the home hall,
   I2C health with back-off and the bounded nine-clock recovery. */
#include "sensors.h"
#include "hal.h"
#include "safety.h"
#include "config.h"
#include "pins.h"
#include <stdio.h>
#include <string.h>

/* P4..P15 written HIGH on EVERY select: the PCF8575 is quasi-bidirectional and P4 is the
   home hall, so a select that dropped it would make the input that gates the pump
   unreadable (cad/wiring/nets.py, P4: "write P4 HIGH before reading"). */
#define EXP_INPUTS_HI 0xFFF0u
#define EXP_HOME_BIT  (1u << 4)

static uint16_t g_val[PB_CHANNELS];
static bool     g_valid[PB_CHANNELS];
static bool     g_stuck;
static uint8_t  g_cur_ch;

static uint32_t g_errors, g_fails, g_backoff_until;
static bool     g_healthy = true;
static uint32_t g_txn, g_txn_t0, g_txn_prev;

static bool     g_float_seen;
static int      g_float_last;
static uint32_t g_float_change_ms;

static void note_(bool ok) {
  g_txn++;
  if (ok) { g_fails = 0; return; }
  g_errors++;
  if (++g_fails >= (uint32_t)PB_I2C_FAIL_LIMIT) {
    g_healthy = false;
    g_backoff_until = hal_millis() + (uint32_t)PB_I2C_BACKOFF_MS;
  }
}

/* false == do not touch the bus this pass. */
static bool gate_(void) {
  if (g_healthy) return true;
  if ((int32_t)(hal_millis() - g_backoff_until) < 0) return false;   /* still backing off */
  if (safety_dosing()) return false;      /* §2.13: an expired back-off STAYS expired */
  safety_tick();
  (void)hal_i2c_recover();                /* exactly PB_I2C_RECOVER_CLOCKS clocks */
  safety_tick();
  g_healthy = true;
  g_fails = 0;
  return true;
}

static void float_track_(void) {
  int now = hal_pin_read(PIN_HALL_FLOAT);
  if (now == g_float_last) return;
  g_float_last = now;
  g_float_change_ms = hal_millis();
  g_float_seen = true;
}

bool sensors_begin(void) {
  memset(g_val, 0, sizeof g_val);
  memset(g_valid, 0, sizeof g_valid);
  g_stuck = false; g_cur_ch = 0;
  g_errors = 0; g_fails = 0; g_backoff_until = 0; g_healthy = true;
  g_txn = 0; g_txn_prev = 0; g_txn_t0 = hal_millis();
  g_float_seen = false;
  g_float_last = hal_pin_read(PIN_HALL_FLOAT);
  g_float_change_ms = g_txn_t0;
  if (!safety_dosing()) (void)hal_i2c_recover();   /* at boot, outside a dose (§2.13) */
  return hal_i2c_probe(I2C_ADDR_EXPANDER);
}

bool sensors_select(uint8_t ch) {
  if (!gate_()) return false;
  uint16_t bits = (uint16_t)((ch & 0x0Fu) | EXP_INPUTS_HI);
  bool ok = hal_i2c_write16(I2C_ADDR_EXPANDER, bits);
  note_(ok);
  if (ok) g_cur_ch = (uint8_t)(ch & 0x0Fu);
  return ok;
}

bool sensors_read_raw(uint8_t ch, uint16_t *raw) {
  if (!sensors_select(ch)) return false;
  safety_wait_ms(1);              /* >= 1 ms to settle, fed */
  (void)hal_adc_read();           /* discard the first conversion */
  *raw = hal_adc_read();          /* keep the second */
  return true;
}

bool sensors_sweep(void) {
  safety_tick();
  uint16_t canary = 0;
  if (!sensors_read_raw(PB_CANARY_CHANNEL, &canary)) {
    for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) g_valid[ch] = false;
    g_stuck = false;
    float_track_();
    return false;
  }
  bool ok = true, all_equal = true;
  for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) {
    safety_tick();
    uint16_t v = 0;
    if (sensors_read_raw(ch, &v)) {
      g_val[ch] = v; g_valid[ch] = true;
      if (v != canary) all_equal = false;
    } else {
      g_valid[ch] = false; all_equal = false; ok = false;
    }
  }
  g_stuck = all_equal;
  if (g_stuck) {                 /* omit the wired channels; the diagnostics keep the
                                    report legal and err=stuck is what the phone sees */
    for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) g_valid[ch] = false;
    ok = false;
  }
  float_track_();
  return ok;
}

uint16_t sensors_value(uint8_t ch) { return ch < PB_CHANNELS ? g_val[ch] : 0u; }
bool     sensors_valid(uint8_t ch) { return ch < PB_CHANNELS ? g_valid[ch] : false; }
bool     sensors_stuck(void)       { return g_stuck; }

bool sensors_home_hall(bool *home) {
  if (!gate_()) return false;
  uint16_t bits = (uint16_t)(g_cur_ch | EXP_INPUTS_HI);
  if (!hal_i2c_write16(I2C_ADDR_EXPANDER, bits)) { note_(false); return false; }
  note_(true);
  uint16_t got = 0;
  if (!hal_i2c_read16(I2C_ADDR_EXPANDER, &got)) { note_(false); return false; }
  note_(true);
  /* WPSE313 open collector with R3 pulling P4 up: LOW == the magnet is present. */
  *home = ((got & EXP_HOME_BIT) == 0u);
  return true;
}

bool     sensors_i2c_healthy(void) { return g_healthy; }
uint32_t sensors_i2c_errors(void)  { return g_errors; }

uint32_t sensors_i2c_txn_per_min(void) {
  uint32_t now = hal_millis();
  uint32_t elapsed = now - g_txn_t0;
  if (elapsed >= 60000u) {
    g_txn_prev = (g_txn * 60000u) / elapsed;
    g_txn = 0;
    g_txn_t0 = now;
  }
  return g_txn_prev;
}

uint32_t sensors_float_change_age_s(void) {
  if (!g_float_seen) return 0u;
  return (hal_millis() - g_float_change_ms) / 1000u;
}

void sensors_scan(char *out, size_t cap) {
  if (cap == 0u) return;
  out[0] = '\0';
  size_t at = 0;
  for (uint8_t a = 0x08u; a <= 0x77u; ++a) {
    safety_tick();
    if (!hal_i2c_probe(a)) continue;
    if (at + 6u >= cap) break;
    at += (size_t)snprintf(out + at, cap - at, "0x%02X ", (unsigned)a);
  }
}
```

5. [ ] **Give the fake the ADC settling the discard rule exists for.** In `src/hal_sim.cpp`, add `static bool g_adc_settled; static uint16_t g_adc_prev;` beside the other channel state, set `g_adc_settled = false;` in `sim_reset()`, and replace `hal_adc_read()` and the select half of `hal_i2c_write16()`:

```c
uint16_t hal_adc_read(void) {
  uint8_t ch = g_mux_stuck ? (uint8_t)PB_CANARY_CHANNEL : g_mux_sel;
  uint16_t settled = g_chan[ch & 0x0Fu];
  /* the first conversion after a select still carries the previous channel: a 10 k
     source into the ADC's sample cap does not settle inside one conversion */
  uint16_t v = g_adc_settled ? settled : g_adc_prev;
  g_adc_settled = true;
  g_adc_prev = settled;
  ev_(SIM_EV_ADC, ch, v);
  return v;
}

bool hal_i2c_write16(uint8_t addr, uint16_t bits) {
  ev_(SIM_EV_I2C_WRITE, addr, bits);
  if (g_i2c_fail) return false;
  g_exp_port = bits;
  g_mux_sel = (uint8_t)(bits & 0x0Fu);
  g_adc_settled = false;               /* a select un-settles the ADC */
  return true;
}
```

6. [ ] **Run the suite and watch all thirteen cases pass.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_sensors
```

Expected: `13 Tests 0 Failures 0 Ignored / OK` (task 6's four plus these nine). If `test_recovery_is_a_fixed_nine_clocks_with_sda_held_low` reports 0 SCL events, `gate_()` returned early — check that `pb_advance(PB_I2C_BACKOFF_MS + 1)` really moved the clock past `g_backoff_until`.

7. [ ] **Check the board env builds and commit.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio run -e uno_r4_wifi && pio test -e native
git add include/sensors.h src/sensors.cpp src/hal_sim.cpp test/test_sensors/test_sensors.cpp
git commit -m "sensors: the mux discipline, the open-channel canary, and I2C health

The canary is the case that matters most. sensors_select() returns true whenever the
PCF8575 ACKs the port write, so an unpowered mux, a broken S-line or a floating EN gives
the same ADC value on every channel with no error raised anywhere — and from butler's
side that is byte-identical to five equally-dry pots, which is the one failure the 'raw
on the wire, calibration in the backend' split cannot catch. So every sweep reads one
UNWIRED channel, and when it equals every wired channel the wired channels are omitted
and err=stuck rides out instead.

sensors_sweep() has exactly one caller, and it is netfsm.cpp's NET_IDLE pass (task 24),
once per report cycle. That pass issues zero AT commands, so §3's 'skipped in any pass
where a modem command ran' holds by construction and a 7 s wedged sweep cannot stack on
2.4 s of modem. main.cpp's loop() does not call it. sensors_home_hall() reports a bus
error as false and never as 'not home'.

Deviation: the refuse-while-dosing guard on the nine-clock recovery lives here, one line
above the hal_i2c_recover() call, rather than inside the HAL as hal.h's comment implies —
the HAL must not call up into the safety layer. §2.13's rule is unchanged: a back-off
that expires mid-dose stays expired until the dose ends."
```

---

### Task 8: hal_uno.cpp — the real board behind seam 1

**Drop 1.**

**Files:**
- Create: `src/hal_uno.cpp`
- Modify: `tools/check.sh` (task 1 left it a runnable script with a `fail()` helper and no greps; this task adds the D6, watchdog and pulse-pin ones)
- Test: **none on the host.** This file is on spec §9's *"NOT tested on the host"* list. It is verified by `pio run -e uno_r4_wifi`, by the greps in `tools/check.sh`, and on the bench by bring-up 1, 3, 4a, 4b and 4c (§13).

**Interfaces:**

*Consumes.* `include/hal.h` (task 3) — this file implements all 35 of its functions for the board, `hal_adc_bits()` included. `include/pins.h` and `include/config.h` (task 2). `include/noinit.h` (task 4) for `g_nv.boots`. `include/pulses.h` (task 6) for `pulses_isr_flow()` / `pulses_isr_screw()`.

*Produces.* No new declarations. Five things it owns that nothing else in the tree may:
1. `hal_boot_pump_off()` and `hal_pump_write(bool)` — the two, and only two, `R_IOPORT_PinCfg(NULL, g_pin_cfg[PIN_PUMP_EN].pin, …)` call sites.
2. `hal_arm_pulse_pins_()` — the only place D2 and D3 are configured.
3. `hal_wdt_start()` on the `wdt_cfg_t` overload, and `hal_wdt_granted()` computing the grant itself.
4. `hal_irq_armed()` / `hal_irq_filtered()`.
5. `hal_begin()` — the ADC width and its assertion, the stack paint, `Wire`, the servo, and the pulse-pin arming.

`#define PB_PUMP_OWNER` appears at the top of this file and **nowhere else in the tree**, which is what makes `pins.h` define `PIN_PUMP_EN` for this translation unit and no other.

**The invariants this file exists to hold, quoted so they cannot be got subtly wrong:**
- §2.1 — *"`pinMode(pin, OUTPUT)` is `R_IOPORT_PinCfg(NULL, g_pin_cfg[pin].pin, IOPORT_CFG_PORT_DIRECTION_OUTPUT)` (`digital.cpp:12-14`) … an unconditional whole-register assignment: `R_PFS->PORT[..].PIN[..].PmnPFS = cfg;` (`bsp_io.h:391-395`). `cfg` is `0x4` … So `pinMode(D6, OUTPUT)` latches PODR = 0 and drives D6 LOW, discarding whatever `digitalWrite` had just put there."* The wiring README's recipe is wrong for this silicon; the correct sequence is one PFS write carrying direction and level together, and `pinMode` never touches D6 at all.
- §2.1 — *"Do NOT re-declare `g_pin_cfg` here: `Arduino.h:60-66` already declares it, INSIDE an `extern "C"` block, and a second declaration in a `.cpp` without `extern "C"` is a hard error."*
- §2.14 — *"`Interrupts.cpp:183` configures the pin with `IOPORT_CFG_IRQ_ENABLE | … | pullup`; `digital.cpp`'s `pinMode` writes the same register without `IOPORT_CFG_IRQ_ENABLE`. Any subsequent `pinMode` on D2 or D3 … detaches it with no error and no symptom."*
- §2.14 — the design's `IRQManager::addPeripheral` route **is not reachable and must not be used**: a fresh cfg takes the `if (p_cfg->irq == FSP_INVALID_VECTOR)` branch (`IRQManager.cpp:747-756`) and allocates a second NVIC vector on the same ICU channel, with **no** `last_interrupt_index > PROG_IRQ_NUM` bounds check. Set the `FLTEN` bit directly instead, after `attachInterrupt` has opened the channel.
- §2.5 — *"`WDT.getTimeout()` cannot be used here … under this overload `getTimeout()` returns 0 even on a perfectly running dog — which, in the design as written, would have made `dose_run()`'s first guard refuse every dose forever."*

---

1. [ ] **Add the greps first, and watch them fail.** Append to `tools/check.sh`, immediately under the `# ---- invariants land here (task 13, then task 30) ----` marker, **in the `expect` / `count` / `files` style task 1 built the helpers for** — task 13 step 1 reads this file back and counts `expect ` lines, so a second style here would make that step wrong. **Exactly six `expect` lines, and the D2/D3 `awk` range check is NOT one of them: task 13 step 2 owns that one, and adding it here as well is the doubled invariant task 13 step 1 warns about.**

```bash
# ---- D6: spec §2.1, §2.2 ----
# PB_PUMP_OWNER is DEFINED in exactly one file. The pattern is the DEFINITION, never the
# bare token: include/pins.h necessarily *tests* it with #ifdef, so a bare-token file count
# would be 2 forever and this invariant could never hold.
expect 1 "$(files 'define[[:space:]]+PB_PUMP_OWNER' "${SCAN[@]}")" \
  "exactly one file defines PB_PUMP_OWNER, so exactly one file gets PIN_PUMP_EN"
expect 0 "$(count 'pinMode\(PIN_PUMP_EN' "${SCAN[@]}")" \
  "pinMode never touches D6 (it would latch PODR=0 and drive the pin LOW)"
expect 2 "$(count 'R_IOPORT_PinCfg.*PIN_PUMP_EN' "${SCAN[@]}")" \
  "exactly two whole-word PFS writes to D6 (hal_boot_pump_off, hal_pump_write)"
expect 0 "$(count 'R_IOPORT_PinWrite.*PIN_PUMP_EN|digitalWrite.*PIN_PUMP_EN' "${SCAN[@]}")" \
  "no unverifiable write form on D6"

# ---- the watchdog, first two of four: spec §2.5. The other two are task 13 step 3's. ----
expect 1 "$(count 'WDT\.refresh' "${SCAN[@]}")" \
  "one refresh call site, inside hal_wdt_feed"
expect 0 "$(count 'WDT\.getTimeout' "${SCAN[@]}")" \
  "the timeout getter is never used (it returns 0 under the wdt_cfg_t overload)"
```

```bash
cd /Users/jcanton/projects/plant-butler/firmware && make check
```

Expected: `FAIL  exactly one file defines PB_PUMP_OWNER, so exactly one file gets PIN_PUMP_EN: expected 1, found 0`, a `FAIL` for the two-PFS-writes line, and a non-zero exit — `src/hal_uno.cpp` does not exist yet.

2. [ ] **Write `src/hal_uno.cpp`.** Keep it deliberately stupid: every function one to six lines, no arithmetic beyond the grant, no state machine. It is the ~230 lines no host test covers.

```c
/* src/hal_uno.cpp — DEVICE ONLY. The only translation unit in the project that includes
   <Arduino.h>, names a pin, owns an ISR, or writes D6. Filtered out of [env:native] and
   [env:uno_r4_wifi_sim] by build_src_filter. Spec §2.1, §2.5, §2.13, §2.14, §12. */

#define PB_PUMP_OWNER 1     /* the ONLY definition in the tree. pins.h defines
                               PIN_PUMP_EN for this translation unit and no other. */

#include <Arduino.h>        /* declares g_pin_cfg at :60-66, inside extern "C". Do NOT
                               re-declare it: a second declaration in a .cpp without
                               extern "C" is a hard linkage error (§2.1). */
#include <Wire.h>
#include <Servo.h>
#include <WDT.h>
#include <malloc.h>
#include <unistd.h>

#include "hal.h"
#include "pins.h"
#include "config.h"
#include "noinit.h"
#include "pulses.h"

extern uint32_t __StackLimit;
extern uint32_t __StackTop;

static Servo    g_servo;
static bool     g_wdt_running;
static uint32_t g_wdt_delta;
static bool     g_adc_ok;

/* ---------------------------------------------------------------- D6, and only here */
#define PB_PUMP_PFS_OFF ((uint32_t)(IOPORT_CFG_PORT_DIRECTION_OUTPUT | PUMP_OFF_PFS_LEVEL))
#define PB_PUMP_PFS_ON  ((uint32_t)(IOPORT_CFG_PORT_DIRECTION_OUTPUT | PUMP_ON_PFS_LEVEL))

void hal_boot_pump_off(void) {          /* setup()'s FIRST statement */
  /* ONE register write. Direction = output AND level = the module's OFF level, atomically.
     BSP_CFG_PARAM_CHECKING_ENABLE is 0 (bsp_cfg.h:28), so the NULL p_ctrl digital.cpp
     passes is safe, and we match it. */
  R_IOPORT_PinCfg(NULL, g_pin_cfg[PIN_PUMP_EN].pin, PB_PUMP_PFS_OFF);
}

void hal_pump_write(bool on) {
  /* THE SAME whole-word form as the boot write, deliberately. R_IOPORT_PinCfg ->
     R_BSP_PinCfg is one unconditional `PmnPFS = cfg` (bsp_io.h:391-395), so every pump
     write re-states the DIRECTION as well as the level — which is what makes
     safety_tick()'s idle re-assert a REPAIR of a stray pinMode on D6. */
  R_IOPORT_PinCfg(NULL, g_pin_cfg[PIN_PUMP_EN].pin, on ? PB_PUMP_PFS_ON : PB_PUMP_PFS_OFF);
}

bool hal_pump_level_on(void) { return PUMP_ON_PFS_LEVEL != 0; }

/* ------------------------------------------------------------------------ the clock */
uint32_t hal_millis(void) { return (uint32_t)millis(); }
uint32_t hal_micros(void) { return (uint32_t)micros(); }
void     hal_delay_us(uint16_t us) { delayMicroseconds(us); }

/* ------------------------------------------------------------------- ordinary pins */
void hal_pin_mode(uint8_t pin, uint8_t mode) {
  pinMode(pin, mode == PB_OUT ? OUTPUT : INPUT_PULLUP);
}
int  hal_pin_read(uint8_t pin) { return digitalRead(pin) == HIGH ? PB_HIGH : PB_LOW; }
void hal_pin_write(uint8_t pin, uint8_t level) { digitalWrite(pin, level ? HIGH : LOW); }

/* -------------------------------------------------------------------- ADC and I2C */
uint16_t hal_adc_read(void) { return (uint16_t)analogRead(PIN_MUX_ADC); }

bool hal_i2c_write16(uint8_t addr, uint16_t bits) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)(bits & 0xFFu));        /* PCF8575: P0..P7 first, then P8..P15 */
  Wire.write((uint8_t)(bits >> 8));
  return Wire.endTransmission() == 0;
}

bool hal_i2c_read16(uint8_t addr, uint16_t *bits) {
  if (Wire.requestFrom((int)addr, 2) != 2) return false;
  uint16_t lo = (uint16_t)Wire.read();
  uint16_t hi = (uint16_t)Wire.read();
  *bits = (uint16_t)(lo | (uint16_t)(hi << 8));
  return true;
}

bool hal_i2c_probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool hal_i2c_recover(void) {
  /* EXACTLY PB_I2C_RECOVER_CLOCKS clocks, a FIXED loop count, never an "until SDA
     releases" condition — that condition is an unbounded loop on a bus a stuck device is
     holding down. The refuse-while-dosing guard is sensors.cpp's, one line above the
     call (§2.13). Nothing here calls the unbounded flush helper of TwoWire: Wire.cpp:833
     spins with no bound. */
  Wire.end();
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT);
  for (uint8_t i = 0; i < PB_I2C_RECOVER_CLOCKS; ++i) {
    digitalWrite(SCL, HIGH); hal_delay_us(5);
    digitalWrite(SCL, LOW);  hal_delay_us(5);
  }
  digitalWrite(SCL, HIGH);
  Wire.begin();
  return true;
}

/* ---------------------------------------------------------------------- the servo */
void hal_servo_us(uint16_t us) {
  if (us == 0u) { g_servo.detach(); return; }
  if (!g_servo.attached()) g_servo.attach(PIN_SERVO);
  g_servo.writeMicroseconds(us);
}

/* ------------------------------------------------------------------- the watchdog */
bool hal_wdt_start(void) {
  wdt_cfg_t cfg = {};                             /* NINE members (r_wdt_api.h:147-160):
                                                     p_callback, p_context and p_extend
                                                     follow stop_control. `= {}` so no
                                                     stack garbage reaches R_WDT_Open. */
  cfg.timeout        = WDT_TIMEOUT_16384;
  cfg.clock_division = WDT_CLOCK_DIVISION_8192;
  cfg.window_start   = WDT_WINDOW_START_100;
  cfg.window_end     = WDT_WINDOW_END_0;          /* refresh legal at any point */
  cfg.reset_control  = WDT_RESET_CONTROL_RESET;
  cfg.stop_control   = WDT_STOP_CONTROL_DISABLE;  /* the reason for this overload:
                                                     the uint32_t one hardcodes ENABLE
                                                     (WDT.cpp:67) — a dog a future __WFI
                                                     could silently stop */
  g_wdt_running = (WDT.begin(cfg) == 1);
  return g_wdt_running;
}

uint32_t hal_wdt_granted(void) {
  /* NEVER the timeout getter: _timeout is assigned only inside getReload(), which only
     begin(uint32_t) calls (WDT.cpp:59,153), so under that overload the getter returns 0
     on a perfectly running dog. Compute the grant instead. */
  if (!g_wdt_running) return 0u;
  return (16384u * 8192u) / (R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKB) / 1000u);
}

uint32_t hal_wdt_counter(void) {
  /* the raw 14-bit down-counter (R7FA4M1AB.h:17811-17812, CNTVAL mask 0x3FFF) */
  return (uint32_t)(R_WDT->WDTSR & R_WDT_WDTSR_CNTVAL_Msk);
}

void hal_wdt_feed(void) { WDT.refresh(); }

uint32_t hal_wdt_last_delta(void) { return g_wdt_delta; }

/* The ONE place in the program that deliberately does not feed. Precondition: not dosing
   — it is called from setup() and as dose_run()'s first guard, after the g_dosing check.
   Spec §2.5, verbatim; hal_sim.cpp carries the same body. */
bool hal_wdt_alive(void) {
  hal_wdt_feed();                              /* start from a known reload */
  uint32_t a  = hal_wdt_counter();
  uint32_t t0 = hal_millis();
  while (hal_millis() - t0 < PB_WDT_PROBE_MS)  /* 40 ms, UNFED, pump already idle-OFF */
    hal_pump_write(false);                     /* the safety half of safety_tick(), without the feed */
  uint32_t b = hal_wdt_counter();
  hal_wdt_feed();                              /* and immediately back in the window */
  g_wdt_delta = (a > b) ? (a - b) : 0u;        /* a DOWN-counter: b must be smaller */
  return g_wdt_delta >= PB_WDT_PROBE_MIN_COUNTS;
}

/* -------------------------------------------------- D2 and D3, configured ONCE */
static void isr_flow_(void)  { pulses_isr_flow(); }
static void isr_screw_(void) { pulses_isr_screw(); }

static void hal_icu_enable_filter_(uint8_t pin) {
  /* attachInterrupt hardcodes filter_enable = false (Interrupts.cpp:151) but ALREADY
     sets pclk_div = EXTERNAL_IRQ_PCLK_DIV_BY_64 (:150), so the only thing missing is
     FLTEN. Set the bit directly: IRQManager::addPeripheral would allocate a SECOND NVIC
     vector on the same ICU channel, unbounded (§2.14). */
  auto cfg = getPinCfgs(pin, PIN_CFG_REQ_INTERRUPT);   /* variant.h:32 — public */
  if (cfg[0] == 0) return;                             /* not an IRQ-capable pin */
  uint8_t ch = GET_CHANNEL(cfg[0]);                    /* variant.h:120 */
  R_ICU->IRQCR[ch] |= (uint8_t)(R_ICU_IRQCR_FLTEN_Msk
                     | (EXTERNAL_IRQ_PCLK_DIV_BY_64 << R_ICU_IRQCR_FCLKSEL_Pos));
}

static void hal_arm_pulse_pins_(void) {          /* the ONLY place D2 and D3 are configured */
  pinMode(PIN_FLOW,       INPUT_PULLUP);         /* attachInterrupt PRESERVES this */
  pinMode(PIN_HALL_SCREW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW),       isr_flow_,  FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_SCREW), isr_screw_, FALLING);
  hal_icu_enable_filter_(PIN_FLOW);
  hal_icu_enable_filter_(PIN_HALL_SCREW);
}

bool hal_irq_armed(uint8_t pin) {
  /* There is no per-channel enable bit reachable from a pin number — the file-static
     IrqPool owns the allocated vector — so find the vector: scan IELSR for the pin's ICU
     event, then ask the NVIC. PROG_IRQ_NUM is BSP_ICU_VECTOR_MAX_ENTRIES
     (IRQManager.cpp:7); the ICU IRQ events are contiguous (bsp_elc.h:51-66). */
  auto cfg = getPinCfgs(pin, PIN_CFG_REQ_INTERRUPT);
  if (cfg[0] == 0) return false;
  uint8_t ch = GET_CHANNEL(cfg[0]);
  for (uint8_t i = 0; i < 32u; ++i) {
    if ((R_ICU->IELSR[i] & 0xFFu) != (uint32_t)(ELC_EVENT_ICU_IRQ0 + ch)) continue;
    return NVIC_GetEnableIRQ((IRQn_Type)i) != 0u;
  }
  return false;
}

bool hal_irq_filtered(uint8_t pin) {
  auto cfg = getPinCfgs(pin, PIN_CFG_REQ_INTERRUPT);
  if (cfg[0] == 0) return false;
  uint8_t ch = GET_CHANNEL(cfg[0]);
  return (R_ICU->IRQCR[ch] & R_ICU_IRQCR_FLTEN_Msk) != 0u;
}

/* --------------------------------------------------------------------- the console */
size_t hal_serial_read(char *buf, size_t cap) {
  size_t n = 0;
  while (n < cap && Serial.available() > 0) buf[n++] = (char)Serial.read();
  return n;
}
void hal_serial_write(const char *s) { Serial.write(s); }
void hal_serial_drain(void) { while (Serial.available() > 0) (void)Serial.read(); }

/* ------------------------------------------------------------ memory, and the paint */
uint32_t hal_heap_arena(void)   { struct mallinfo m = mallinfo(); return (uint32_t)m.arena; }
uint32_t hal_heap_ordblks(void) { struct mallinfo m = mallinfo(); return (uint32_t)m.ordblks; }
uint32_t hal_heap_break(void)   { return (uint32_t)sbrk(0); }
uint32_t hal_stack_limit(void)  { return (uint32_t)&__StackLimit; }

static void hal_paint_stack_(void) {
  /* 0xA5 from __StackLimit up to just below the live frame — painting through our own
     frame would corrupt the return address. §12: nothing else bounds this stack. */
  uint32_t sp;
  __asm volatile ("mov %0, sp" : "=r" (sp));
  uint32_t *p = &__StackLimit;
  uint32_t *stop = (uint32_t *)(sp - 64u);
  while (p < stop) *p++ = 0xA5A5A5A5u;
}

uint32_t hal_stack_hwm(void) {
  uint32_t *p = &__StackLimit;
  while (p < &__StackTop && *p == 0xA5A5A5A5u) p++;
  return (uint32_t)((uint8_t *)&__StackTop - (uint8_t *)p);
}

uint32_t hal_boot_salt(void) { return g_nv.boots * PB_BOOT_SALT_STRIDE; }

/* The HARDWARE width, which analogReadResolution() does not change: the core fixes it at
   open time to BSP_FEATURE_ADC_MAX_RESOLUTION_BITS (analog.cpp:34-45). `status` prints
   adc_hw= from this and main.cpp's boot banner prints adc=14/14 from it beside PB_ADC_BITS.
   ACCEPTED FALLBACK (§9's not-tested-on-the-host list): if that feature macro does not
   resolve on the installed core, return PB_ADC_BITS here and make task 11's status line read
   `adc_hw=unverifiable` instead of a number — and say which shipped in the commit message. */
uint8_t hal_adc_bits(void) { return (uint8_t)BSP_FEATURE_ADC_MAX_RESOLUTION_BITS; }

bool hal_adc_width_ok(void) { return g_adc_ok; }   /* `status` prints adc_req/adc_hw/adc_ok */

void hal_begin(void) {
  /* FIRST, and nothing before it: main.cpp may not include the Arduino header (spec §9) and
     hal.h has no serial-begin, so this is where spec §12's "Serial at 115200" lands. Without
     this line nothing on the console prints and bring-up step 0 has no banner at all. The
     short settle is the USB CDC bridge coming up; it is bounded and it is not a delay() in
     the sense §9 greps for -- see the invariant's hal_uno.cpp exemption. */
  Serial.begin(115200);
  delay(50);
  analogReadResolution(PB_ADC_BITS);
  /* §7: analogReadResolution() does NOT change the hardware width, so 14 is an identity
     map TODAY — but the default requested value is 10 (analog.cpp:11) and a core bump
     that changed the fixed width would silently rescale every raw count on the wire.
     Assert the readback (analog.cpp:698) instead of trusting it. */
  g_adc_ok = (analogReadResolution() == PB_ADC_BITS);
  hal_paint_stack_();
  pinMode(PIN_HALL_FLOAT, INPUT_PULLUP);
  Wire.begin();
  g_servo.attach(PIN_SERVO);
  hal_arm_pulse_pins_();
}
```

3. [ ] **Build both board envs.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio run -e uno_r4_wifi && pio run -e uno_r4_wifi_bringup
```

Expected: both `SUCCESS`. Two things can go wrong here, and both have an accepted fallback the spec already names — take the fallback, do not invent a third route:
- **`sbrk` will not link.** Replace `hal_heap_break()` with `(uint32_t)&__HeapBase + hal_heap_arena()` — nano-malloc's `arena` is the total obtained from `sbrk` — and make `status` (task 11) say which form shipped.
- **`hal_irq_armed()`'s `IELSR` scan will not build.** Report only the filter state and have `status` say which shipped; `icufilter=` is then the whole answer.

   If either fallback is taken, say so in the commit message and in `status`. A build error in `hal_icu_enable_filter_()` has its own accepted fallback (§2.14): the pull-up plus the plausibility ceiling alone, with `icufilter=no` saying so.

4. [ ] **Run the greps and watch them pass.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && make check && echo "check: clean"
```

Expected: `check: clean` and exit 0, with six `ok` lines: 1 file defines `PB_PUMP_OWNER`; 0 hits for `pinMode(PIN_PUMP_EN`; **2** hits for `R_IOPORT_PinCfg.*PIN_PUMP_EN` (`hal_boot_pump_off`, `hal_pump_write`); 0 for `R_IOPORT_PinWrite.*PIN_PUMP_EN|digitalWrite.*PIN_PUMP_EN`; 1 `WDT.refresh`; 0 `WDT.getTimeout`. The D2/D3 `awk` range check is **not** here — task 13 step 2 adds it, once.

5. [ ] **Confirm the host suites are untouched, and commit.**

```bash
cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native && make check
git add src/hal_uno.cpp tools/check.sh
git commit -m "hal_uno.cpp: the real board behind seam 1

These ~230 lines have NO host coverage and are on §9's 'NOT tested on the host' list —
the pin numbers, the R_IOPORT_PinCfg boot write, the two ISRs, the R_ICU->IRQCR filter
write, the IELSR scan, analogReadResolution(14), WDT.begin(cfg) and the stack paint.
tools/check.sh holds their shape; bring-up 1, 3, 4a, 4b and 4c prove them, and 4a's pass
criterion now includes COM-NO staying OPEN across a power cycle AND across a hang-forced
watchdog reset. That is what proves the boot write, and the wiring README's own recipe
would have failed it on an active-LOW module.

D6 is written by exactly two functions, both the whole-word R_IOPORT_PinCfg form that
carries direction and level in one PmnPFS write. pinMode never touches it: on this core
pinMode(pin, OUTPUT) latches PODR = 0 and drives the pin LOW, discarding a preceding
digitalWrite.

The ICU filter is set by writing IRQCR[ch] directly after attachInterrupt has opened the
channel. IRQManager::addPeripheral is NOT used: with a fresh cfg it allocates a second
NVIC vector on the same channel and, unlike the SPI branch, has no bounds check.

hal_wdt_granted() computes the grant rather than calling WDT.getTimeout(), which returns
0 under the wdt_cfg_t overload and would have refused every dose forever.

hal_begin()'s first statement is Serial.begin(115200). main.cpp may not include the Arduino
header (spec 9) and hal.h has no serial-begin, so the console is opened one function later
than spec 12's order prints it; without that line bring-up step 0 has no banner at all.

hal_adc_bits() returns BSP_FEATURE_ADC_MAX_RESOLUTION_BITS - the HARDWARE width, which
analogReadResolution() does not change. status prints adc_hw= from it and the boot banner
prints adc=14/14. If the feature macro does not resolve, the accepted fallback is to return
PB_ADC_BITS and print adc_hw=unverifiable; say which shipped."
```

---

### Task 9: `lib/Screen` — `probe()`, `present()`, and a panel that never hangs

**Drop 1.**

**Files:**
- Modify: `lib/Screen/include/Screen.h` (all 21 lines), `lib/Screen/src/Screen.cpp` (all 33 lines), `src/main.cpp` (add the two panel objects so the library is actually compiled), `tools/check.sh` (two greps)
- Test: **none on the host.** `lib/Screen` is in `[env:native]`'s `lib_ignore` and is on spec §9's "NOT tested on the host" list, together with the one-second blocking wait inside `LiquidCrystal_I2C::init_priv()`. What proves this file is `pio run -e uno_r4_wifi`, the two greps, and **bring-up step 1** (spec §13: `i2c` must see 0x20, 0x27 and 0x3C) plus **bring-up step 0** (the banner is legible on both panels).

**Interfaces:**
- Consumes: `bool hal_i2c_probe(uint8_t addr)` (task 3); `I2C_ADDR_LCD` = 0x27, `I2C_ADDR_OLED` = 0x3C (task 2); `void safety_tick(void)` (task 5).
- Produces, for task 10's painter and task 12's setup:
  - `enum class ScreenType { Oled, Lcd };` (unchanged)
  - `explicit Screen(ScreenType type);`
  - `bool probe();` — one `hal_i2c_probe()`; a device that does not answer becomes a permanent no-op
  - `bool present() const;`
  - `void begin();` — no-op unless `probe()` said yes
  - `void clear();`
  - `void row(uint8_t r, const char *text);` — for the LCD, `setCursor` then a loop of single-character writes with `safety_tick()` between them
  - `extern Screen g_oled_screen, g_lcd_screen;` defined in `src/main.cpp`
  - The old `initialize()` and `print(col,row,str)` are **deleted**. `print()`'s body used the library's own row printer, which spec §9 greps to zero, and nothing needs a column offset: `ui.cpp` paints whole 16-character rows.
  - The `Screen*` the old `Manifold` logged through is **deleted, not replaced**. The dependency is inverted: `ui.cpp` reads the cart, the cart no longer writes to a screen (spec §5).

**Spec sections to read in full before starting:** §1's module table (the `lib/Screen` row); §5 in full, especially "The bus rule, and the honest cost of keeping the screens".

---

1. - [ ] Read the two facts this file exists to respect, and keep them in view for the whole task. From spec §0's platform table: one LCD character is 6 Wire transactions and a 16-character row plus its `setCursor` is **102**; `TwoWire`'s transfer timeout is a fixed 1000 ms with no setter; and the library's row printer offers **no hook between characters**. So a row painted with it is one unfed span of up to 102 seconds on a wedged bus, against a granted 5592 ms watchdog window.

2. - [ ] Rewrite the header:

   ```cpp
   /* lib/Screen/include/Screen.h -- DEVICE ONLY. [env:native] lib_ignores this library.
      The only place in the tree that names LiquidCrystal_I2C or u8x8 (spec §1). */
   #ifndef SCREEN_H
   #define SCREEN_H

   #include "Arduino_SensorKit.h"
   #include <Arduino.h>
   #include <LiquidCrystal_I2C.h>
   #include <stdint.h>

   enum class ScreenType { Oled, Lcd };

   class Screen {
   private:
     ScreenType type;
     LiquidCrystal_I2C lcd;
     bool present_;

   public:
     explicit Screen(ScreenType type);

     /* One bounded probe at boot. A panel that does not answer becomes a permanent no-op
        rather than wedging inside Oled.begin() or LiquidCrystal_I2C::init(). Spec §5. */
     bool probe();
     bool present() const { return present_; }

     void begin();                              /* no-op unless probe() said yes */
     void clear();
     void row(uint8_t r, const char *text);     /* one 16-column row; text is padded by ui.cpp */
   };

   #endif /* SCREEN_H */
   ```

3. - [ ] Rewrite the implementation. **Spec §5's invariant, and it is the whole reason this method exists: an LCD row is written as a loop of single-character writes with `safety_tick()` between characters — never through the library's row printer, which offers no hook between characters.**

   ```cpp
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
   ```

   Two notes to carry forward: `row()` writes only as far as the terminator, so a caller passing a short string leaves stale glyphs to its right — `ui.cpp` (task 10) always passes a string padded to 16. And the unbounded `flush()` helper of `TwoWire` is called nowhere in this class; task 13's grep keeps it that way.

4. - [ ] Give `src/main.cpp` the two panel objects, so that `lib/Screen` is actually compiled (the LDF builds a library only when something includes it, and until now nothing did). Task 12 fills the rest of `setup()`; `ui.cpp` (task 10) refers to these two by `extern`.

   ```cpp
   /* src/main.cpp -- DEVICE ONLY. Task 12 writes the full setup order of spec §2.5/§5/§12. */
   #include "Screen.h"

   Screen g_oled_screen(ScreenType::Oled);
   Screen g_lcd_screen(ScreenType::Lcd);

   void setup(void) {
     /* Order is load-bearing and lands in task 12. Both panels are probed and opened BEFORE
        sensors_begin(), because init_priv() re-opens the IIC peripheral (spec §5). */
     g_oled_screen.probe();
     g_oled_screen.begin();
     g_lcd_screen.probe();
     g_lcd_screen.begin();
   }

   void loop(void) {}
   ```

5. - [ ] Build for the board and confirm `lib/Screen` really was compiled this time:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio run -e uno_r4_wifi 2>&1 | grep -E 'Screen|SUCCESS'
   ```
   expected: a `Compiling .pio/build/uno_r4_wifi/lib*/Screen/Screen.cpp.o` line, then `SUCCESS`. If `Screen.cpp.o` does not appear, the LDF has not picked the library up and the greps below would be proving nothing.

6. - [ ] Add this task's two greps to `tools/check.sh`, immediately under the greps task 8 put beneath the `# ---- invariants land here (task 13, then task 30) ----` marker:

   ```bash
   # spec §5: TwoWire::flush() spins with no bound and is never called by us.
   expect 0 "$(count 'Wire\.flush' "${SCAN[@]}")" \
     "no Wire flush anywhere"

   # spec §5: the library's row printer offers no hook between characters, so an LCD row is
   # painted one character at a time with safety_tick() between them.
   expect 0 "$(count 'lcd\.print|lcd\.println' "${SCAN[@]}")" \
     "no library row printer on the LCD"
   ```

7. - [ ] Run the checks and the build together:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && make check && pio run -e uno_r4_wifi 2>&1 | tail -3
   ```
   expected: `ok    no Wire flush anywhere (0)`, `ok    no library row printer on the LCD (0)`, `all invariants hold`, `SUCCESS`.

8. - [ ] Re-grep the installed libraries by hand — the script only scans our source, and any library later added to this bus must be re-grepped:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && grep -rn 'Wire\.flush' .pio/libdeps/uno_r4_wifi | wc -l
   ```
   expected today: `0`. Record the number you actually got in the commit message.

9. - [ ] Commit.

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && git add lib/Screen src/main.cpp tools/check.sh && git commit -m "Screen: a panel that does not answer is a no-op, and a row is painted one character at a time

   probe() at boot replaces wedging inside Oled.begin() or LiquidCrystal_I2C::init().
   row() writes single characters with safety_tick() between them: a 16-character row is
   102 Wire transactions and the core's transfer timeout is a fixed 1000 ms, so the
   library's own row printer is up to 102 unfed seconds against a 5592 ms window.
   initialize() and print(col,row,str) are deleted, and so is the Screen* the old Manifold
   logged through - ui.cpp reads the cart, the cart no longer writes to a screen.

   No host test covers this file: lib/Screen is lib_ignored on native and is on spec §9's
   not-tested-on-the-host list, the one-second wait inside init_priv() included. Bring-up
   step 1 (i2c sees 0x20, 0x27, 0x3C) and step 0 (a legible banner) are what prove it.
   .pio/libdeps/uno_r4_wifi re-grepped for the unbounded flush helper by hand: 0 hits."
   ```

---

---

### Task 10: `ui.h` and `ui.cpp` — two pure renderers and the coarsened painter

**Drop 1.**

**Files:**
- Create: `include/ui.h`, `src/ui.cpp`
- Modify: `test/test_cli/test_cli.cpp` (create it if task 11 has not yet run; the two orders are independent)
- Test: `test/test_cli/test_cli.cpp`

**Interfaces:**
- Consumes: `bool cart_busy(void)` from `lib/Manifold/include/cart.h` (task 14 — until then, the declaration is satisfied by a weak stub, see step 3); `Screen::row()` and the two panel objects `g_oled_screen` / `g_lcd_screen` from task 9 (device only).
- Produces:
  ```c
  typedef struct {
    char     build[8]; char controller[16]; uint32_t uptime_min;
    bool     pos_known; uint8_t pos; uint32_t screw_pulses;
    bool     float_ok;  bool pump_on; bool parked;
    uint32_t flow_hz;   uint32_t flow_total;
    uint8_t  link;      int8_t rssi; char ip[16];
    uint16_t http_status; uint16_t next_s;
    uint32_t cmd_id;    const char *cmd_text;
    bool     contra;    bool dry; bool sim;
    const char *lcd_state; const char *lcd_detail;
  } ui_state_t;
  void ui_render(const ui_state_t *s, char rows[8][17]);
  void ui_render_lcd(const ui_state_t *s, char rows[2][17]);
  void ui_poll(const ui_state_t *s);
  void ui_modem_ran(void);
  #ifdef PB_NATIVE
  uint16_t ui_paints_for_test(void);
  #endif
  ```

**Spec sections to read in full before starting:** §1's module table (the `ui.cpp` row); §5 in full.

**Two decisions this task records, both forced by spec §9's grep list:**

- **`ui_poll()` does not call `safety_dosing()`.** The skeleton's sketch has it begin `if (safety_dosing() || cart_busy() || modem_ran) return;`, but §9 requires **zero hits for `safety\.h|dose_run|hal_pump_write` in `ui.cpp`**, and `safety_dosing()` is declared in `safety.h`. The guard therefore reads the already-present `s->pump_on` field, which `main.cpp` (task 12) fills from `safety_dosing()`. That is exactly what the grep is for: one `#include "safety.h"` added to `ui.cpp` during a later change puts a `dose_run()` call one edit away from a painter.
- **The renderers are pure and the painter is not.** `ui_render()` / `ui_render_lcd()` take a filled `ui_state_t` and touch nothing else, which is what makes them host-testable while `lib/Screen` is `lib_ignore`d on native. `ui_poll()`'s panel writes are compiled out under `PB_NATIVE` and replaced by a paint counter the tests read.

**The sim banner is one 16-character string, `"*** SIM NO D6 **"`, used verbatim in BOTH renderers.** Task 29 asserts it exactly, not by substring: an earlier draft had `"*** SIM: D6 NOT"` on the OLED and `"*** SIM: NO D6 *"` on the LCD, and a substring check for `"SIM"` would have shipped the truncation.

**And one deviation from spec §5's printed layouts, to record in the commit message:** two of §5's sample rows are wider than the panels. OLED row 2 (`float OK   pump off`) is 19 characters and row 3 (`flow 0/s  tot 5881`) is 18, against u8x8's 16 columns; LCD row 1's contradiction detail (`float ok, no flow`) is 17 against the LCD's 16. The renderings below are the same information at the real widths — `flt OK  pump off`, `flow 0/s t 5881`, `float ok,no flow`. Every other row is §5's verbatim.

---

1. - [ ] Write the header:

   ```c
   /* include/ui.h -- the two pure renderers and the coarsened painter (spec §5).
      ui.cpp includes neither safety.h nor anything that can assert D6: spec §9 greps for it. */
   #pragma once
   #include <stdbool.h>
   #include <stdint.h>

   typedef struct {
     char        build[8];        /* "bench" | "bringup" | "sim" */
     char        controller[16];
     uint32_t    uptime_min;      /* MINUTES, not seconds -- spec §5's bus rule */
     bool        pos_known;
     uint8_t     pos;
     uint32_t    screw_pulses;
     bool        float_ok;
     bool        pump_on;         /* main.cpp fills this from safety_dosing() */
     bool        parked;
     uint32_t    flow_hz;
     uint32_t    flow_total;
     uint8_t     link;            /* 0 down, 1 joining, 2 up */
     int8_t      rssi;
     char        ip[16];
     uint16_t    http_status;
     uint16_t    next_s;          /* painted in 5 s steps */
     uint32_t    cmd_id;
     const char *cmd_text;        /* "ok 248ml" | "REF float" | 0 */
     bool        contra;
     bool        dry;
     bool        sim;
     const char *lcd_state;       /* row 0: IDLE | MOVE o3 | PUMP o3 | WIFI? | REFUSED | ... */
     const char *lcd_detail;      /* row 1: human prose, NEVER the err= wire token */
   } ui_state_t;

   void ui_render(const ui_state_t *s, char rows[8][17]);
   void ui_render_lcd(const ui_state_t *s, char rows[2][17]);
   void ui_poll(const ui_state_t *s);
   /* net_poll() calls this DIRECTLY, in any pass that issued a modem command (task 24).
      netfsm.cpp may include ui.h -- §9's grep over netfsm.cpp is safety.h|dose_run|
      hal_pump_write, and ui.h is none of those. The alternative, main.cpp reading
      net_modem_ran_this_pass() and forwarding it, was rejected: it puts a rule that exists
      to bound ONE pass into a different translation unit from the pass that broke it. */
   void ui_modem_ran(void);

   #ifdef PB_NATIVE
   uint16_t ui_paints_for_test(void);   /* the host counts row paints instead of driving a panel */
   #endif
   ```

2. - [ ] Write the four failing tests. The first three drive the pure renderers; the fourth drives the guard.

   ```cpp
   /* test/test_cli/test_cli.cpp -- the console and the two renderers.
      Task 11 adds the line-reader and command cases to this same file; tasks 15, 16, 19, 20
      and 29 add more. The fixture is the SHARED one from the first line: every later case in
      this file calls pb_test_setup() (which starts the watchdog), and a suite whose setUp is
      empty makes task 11's granted=/alive= case unpassable. */
   #include "../support/harness.h"
   #include "cart.h"
   #include "config.h"
   #include "ui.h"
   #include <string.h>
   #include <unity.h>

   void setUp(void)    { pb_test_setup(); }
   void tearDown(void) { pb_test_teardown(); }

   static ui_state_t base_state(void) {
     ui_state_t s;
     memset(&s, 0, sizeof s);
     strcpy(s.build, "bench");
     strcpy(s.controller, "bench1");
     strcpy(s.ip, "192.168.1.42");
     s.uptime_min = 83; s.pos_known = false; s.screw_pulses = 1290;
     s.float_ok = true; s.pump_on = false; s.parked = true;
     s.flow_hz = 0; s.flow_total = 5881;
     s.link = 2; s.rssi = -52; s.http_status = 200; s.next_s = 60;
     s.cmd_id = 17; s.cmd_text = "ok 248ml";
     s.lcd_state = "IDLE"; s.lcd_detail = "next 35s";
     return s;
   }

   static void test_ui_render_fills_eight_rows_of_sixteen_characters(void) {
     ui_state_t s = base_state();
     char rows[8][17];
     memset(rows, 'X', sizeof rows);
     ui_render(&s, rows);
     for (int r = 0; r < 8; ++r) {
       TEST_ASSERT_EQUAL_CHAR('\0', rows[r][16]);      /* terminated AT index 16 */
       TEST_ASSERT_EQUAL_UINT(16, strlen(rows[r]));    /* padded, so no stale glyphs remain */
     }
     TEST_ASSERT_EQUAL_STRING("PB bench1  1h23m", rows[0]);
   }

   static void test_ui_render_lcd_shows_the_contradiction_banner(void) {
     ui_state_t s = base_state();
     s.contra = true;
     s.lcd_state = "CONTRA LATCH";
     s.lcd_detail = "float ok,no flow";
     char rows[2][17];
     ui_render_lcd(&s, rows);
     TEST_ASSERT_EQUAL_STRING("CONTRA LATCH    ", rows[0]);
     TEST_ASSERT_EQUAL_STRING("float ok,no flow", rows[1]);
   }

   static void test_ui_render_lcd_prose_is_never_the_wire_error_token(void) {
     /* spec §4.1's fixed err= enum. Row 1 is human prose and must never be one of these. */
     static const char *const tokens[] = {
       "none", "float", "pos", "noflow", "noise", "cap", "stop", "wdt", "dry", "contra",
       "boot", "range", "cal", "i2c", "busy", "cooldown", "leak", "adc", "stuck", "txcap",
       "resetmid", "heap", "goto", "recv"
     };
     static const char *const details[] = {
       "float NOT OK", "float ok,no flow", "HTTP 400", "next 35s", "p 1290/1450"
     };
     char rows[2][17];
     for (unsigned d = 0; d < sizeof details / sizeof details[0]; ++d) {
       ui_state_t s = base_state();
       s.lcd_detail = details[d];
       ui_render_lcd(&s, rows);
       char trimmed[17];
       strcpy(trimmed, rows[1]);
       for (int i = 15; i >= 0 && trimmed[i] == ' '; --i) trimmed[i] = '\0';
       for (unsigned t = 0; t < sizeof tokens / sizeof tokens[0]; ++t)
         TEST_ASSERT_NOT_EQUAL(0, strcmp(trimmed, tokens[t]));
     }
   }

   /* spec §4.2: "the last HTTP status is on the LCD, not only in `status`: a 400/401 loop is
      otherwise invisible to anyone not on the serial port". The renderer decides this, not the
      caller: main.cpp's ui_fill_() selects lcd_detail for a dozen other reasons, and a rule
      that depended on it happening to choose the right one would be a rule in name only. */
   static void test_ui_render_lcd_shows_the_last_http_status_on_a_four_hundred(void) {
     ui_state_t s = base_state();
     s.http_status = 400;
     s.lcd_detail = "next 35s";
     char rows[2][17];
     ui_render_lcd(&s, rows);
     TEST_ASSERT_EQUAL_STRING("HTTP 400        ", rows[1]);
     s.http_status = 200;                           /* a healthy exchange leaves the prose */
     ui_render_lcd(&s, rows);
     TEST_ASSERT_EQUAL_STRING("next 35s        ", rows[1]);
     s.http_status = 0;                             /* and so does "nothing has happened yet" */
     ui_render_lcd(&s, rows);
     TEST_ASSERT_EQUAL_STRING("next 35s        ", rows[1]);
   }

   static void test_ui_poll_is_a_noop_while_the_pump_is_asserted(void) {
     ui_state_t s = base_state();
     s.pump_on = false;
     ui_poll(&s);                                   /* first paint: every row changes */
     uint16_t after_first = ui_paints_for_test();
     TEST_ASSERT_TRUE(after_first > 0);

     s.pump_on = true;
     s.uptime_min = 99; s.flow_total = 6000;        /* plenty changed... */
     ui_poll(&s);
     TEST_ASSERT_EQUAL_UINT16(after_first, ui_paints_for_test());   /* ...and nothing painted */
   }

   /* spec §3, §5: a pass that issued a modem command has already spent up to 2.4 s of a
      5592 ms grant, and one wedged LCD row is up to 102 s. net_poll() calls ui_modem_ran()
      directly (task 24); this is the assertion that keeps that call from being deleted. */
   static void test_ui_poll_is_a_noop_in_a_pass_where_a_modem_command_ran(void) {
     ui_state_t s = base_state();
     ui_poll(&s);                                   /* first paint fills the shadow */
     uint16_t after_first = ui_paints_for_test();
     s.uptime_min = 99; s.flow_total = 6000;        /* plenty changed... */
     ui_modem_ran();
     ui_poll(&s);
     TEST_ASSERT_EQUAL_UINT16(after_first, ui_paints_for_test());   /* ...and nothing painted */
     ui_poll(&s);                                   /* the flag is consumed, not sticky */
     TEST_ASSERT_TRUE(ui_paints_for_test() > after_first);
   }

   int main(void) {
     UNITY_BEGIN();
     RUN_TEST(test_ui_render_fills_eight_rows_of_sixteen_characters);
     RUN_TEST(test_ui_render_lcd_shows_the_contradiction_banner);
     RUN_TEST(test_ui_render_lcd_prose_is_never_the_wire_error_token);
     RUN_TEST(test_ui_render_lcd_shows_the_last_http_status_on_a_four_hundred);
     RUN_TEST(test_ui_poll_is_a_noop_while_the_pump_is_asserted);
     RUN_TEST(test_ui_poll_is_a_noop_in_a_pass_where_a_modem_command_ran);
     return UNITY_END();
   }
   ```

3. - [ ] Run them and watch them fail to compile — `ui.h` exists but `ui.cpp` does not, and `cart.h` does not either until task 14:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cli
   ```
   expected: `fatal error: cart.h: No such file or directory` (if task 14 has not run) or, once that is past, `undefined reference to 'ui_render(...)'`.

   If `cart.h` does not yet exist, create the one declaration `ui.cpp` needs and nothing else, so the two tasks stay independent:

   ```c
   /* lib/Manifold/include/cart.h -- task 14 writes the rest of the cart's surface. */
   #pragma once
   #include <stdbool.h>
   #include <stdint.h>
   bool cart_busy(void);
   ```
   and a stub definition in `lib/Manifold/src/cart.cpp`: `#include "cart.h"` / `bool cart_busy(void) { return false; }`. Delete both when task 14 lands (it rewrites the same two files).

4. - [ ] Write `ui.cpp`'s renderers. `snprintf` truncates to 16 and then the row is space-padded, so no row can ever exceed its panel or leave stale glyphs.

   ```cpp
   /* src/ui.cpp -- two PURE renderers and the coarsened painter (spec §5).
      This file includes neither the safety header nor cart-independent state: spec §9 greps
      it for the safety header, the dosing entry point and the pump write, and expects zero.
      The dosing guard reads the already-present s->pump_on, which main.cpp fills from
      safety_dosing(). */
   #include "ui.h"
   #include "cart.h"
   #include "config.h"
   #include <stdarg.h>
   #include <stdio.h>
   #include <string.h>

   static void row_(char *dst, const char *text) {
     size_t n = 0;
     while (n < 16 && text[n] != '\0') { dst[n] = text[n]; ++n; }
     while (n < 16) dst[n++] = ' ';
     dst[16] = '\0';
   }

   static void rowf_(char *dst, const char *fmt, ...) {
     char tmp[32];
     va_list ap;
     va_start(ap, fmt);
     vsnprintf(tmp, sizeof tmp, fmt, ap);
     va_end(ap);
     row_(dst, tmp);
   }

   /* spec §5's OLED, at the panel's real 16 columns. Rows 2 and 3 are abbreviated from §5's
      sample, which prints 19 and 18 characters wide. */
   void ui_render(const ui_state_t *s, char rows[8][17]) {
     rowf_(rows[0], "PB %s  %luh%02lum", s->controller,
           (unsigned long)(s->uptime_min / 60u), (unsigned long)(s->uptime_min % 60u));
     if (s->pos_known) rowf_(rows[1], "pos %u  p %lu", (unsigned)s->pos, (unsigned long)s->screw_pulses);
     else              rowf_(rows[1], "pos ?  p %lu", (unsigned long)s->screw_pulses);
     rowf_(rows[2], "flt %s  pump %s", s->float_ok ? "OK" : "NO", s->pump_on ? "ON " : "off");
     rowf_(rows[3], "flow %lu/s t %lu", (unsigned long)s->flow_hz, (unsigned long)s->flow_total);
     rowf_(rows[4], "wifi %s  %d dBm",
           s->link == 2 ? "UP" : (s->link == 1 ? "JN" : "--"), (int)s->rssi);
     row_(rows[5], s->link == 2 ? s->ip : (s->link == 1 ? "joining..." : "no link"));
     rowf_(rows[6], "rpt %u next %us", (unsigned)s->http_status,
           (unsigned)((s->next_s / 5u) * 5u));                       /* 5 s steps, spec §5 */
     if (s->cmd_id == 0) row_(rows[7], "cmd -");
     else rowf_(rows[7], "cmd %lu %s", (unsigned long)s->cmd_id,
                s->cmd_text ? s->cmd_text : "");
     if (s->sim) row_(rows[0], "*** SIM NO D6 **");  /* EXACTLY 16 chars; task 29 asserts it */
   }

   void ui_render_lcd(const ui_state_t *s, char rows[2][17]) {
     row_(rows[0], s->lcd_state  ? s->lcd_state  : "IDLE");
     /* spec §4.2: the last HTTP status is on the LCD, not only in `status` — a 400/401 loop is
        otherwise invisible to anyone not on the serial port. The RENDERER decides it, not the
        caller: ui_fill_() (task 26) picks lcd_detail for a dozen other reasons and a rule that
        relied on it choosing this one would not be a rule. 0 means "no exchange yet". */
     if (s->http_status != 0u && s->http_status != 200u)
       rowf_(rows[1], "HTTP %u", (unsigned)s->http_status);
     else
       row_(rows[1], s->lcd_detail ? s->lcd_detail : "");
     if (s->sim) row_(rows[0], "*** SIM NO D6 **");  /* the SAME 16 chars as the OLED's */
   }
   ```

5. - [ ] Write the painter. Nothing is painted while the pump is asserted, the cart is moving, or a modem command ran this pass: A4/A5 carry the mux select lines and the home hall, the input that gates the pump (spec §5). Only changed rows are pushed.

   ```cpp
   /* --- the painter. Both panels freeze for the length of a dose; that is the visible price
      of keeping them, and the LCD says so. spec §5. --- */
   static char     g_oled_shadow[8][17];
   static char     g_lcd_shadow[2][17];
   static bool     g_shadow_valid;
   static bool     g_modem_ran;

   void ui_modem_ran(void) { g_modem_ran = true; }

   #ifdef PB_NATIVE
   static uint16_t g_paints;
   uint16_t ui_paints_for_test(void) { return g_paints; }
   static void paint_oled_(uint8_t r, const char *t) { (void)r; (void)t; ++g_paints; }
   static void paint_lcd_(uint8_t r, const char *t)  { (void)r; (void)t; ++g_paints; }
   #else
   #include "Screen.h"
   extern Screen g_oled_screen;
   extern Screen g_lcd_screen;
   static void paint_oled_(uint8_t r, const char *t) { g_oled_screen.row(r, t); }
   static void paint_lcd_(uint8_t r, const char *t)  { g_lcd_screen.row(r, t); }
   #endif

   void ui_poll(const ui_state_t *s) {
     bool modem = g_modem_ran;
     g_modem_ran = false;
     if (s->pump_on || cart_busy() || modem) return;

     char oled[8][17], lcd[2][17];
     ui_render(s, oled);
     ui_render_lcd(s, lcd);

     for (uint8_t r = 0; r < 8; ++r)
       if (!g_shadow_valid || strcmp(oled[r], g_oled_shadow[r]) != 0) {
         paint_oled_(r, oled[r]);
         strcpy(g_oled_shadow[r], oled[r]);
       }
     for (uint8_t r = 0; r < 2; ++r)
       if (!g_shadow_valid || strcmp(lcd[r], g_lcd_shadow[r]) != 0) {
         paint_lcd_(r, lcd[r]);
         strcpy(g_lcd_shadow[r], lcd[r]);
       }
     g_shadow_valid = true;
   }
   ```

6. - [ ] Run the suite and watch it pass:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cli
   ```
   expected: six `[PASSED]` lines and `6 test cases: 6 succeeded`.

7. - [ ] Confirm the device build still links (the `extern Screen` arm compiles only there) and that the grep this file exists to satisfy is clean:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio run -e uno_r4_wifi 2>&1 | tail -3 && grep -cE 'safety\.h|dose_run|hal_pump_write' src/ui.cpp
   ```
   expected: `SUCCESS`, then `0`.

8. - [ ] Commit.

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && git add include/ui.h src/ui.cpp test/test_cli/test_cli.cpp && git commit -m "ui: two pure renderers, and a painter that stops while the pump is asserted

   ui_poll() guards on s->pump_on rather than safety_dosing(), because spec §9 greps
   ui.cpp for safety.h|dose_run|hal_pump_write and expects zero - main.cpp fills the field
   from safety_dosing(). The panel writes are compiled out under PB_NATIVE and replaced by
   a paint counter, so the guard is testable on the host while lib/Screen is lib_ignored.

   Three rows deviate from spec §5's samples, which are wider than the panels: OLED row 2
   is 19 characters there and row 3 is 18, against u8x8's 16, and the LCD contradiction
   detail is 17 against 16. Rendered as flt OK  pump off / flow 0/s t 5881 /
   float ok,no flow. Row 1 of the LCD stays human prose and is tested never to equal a
   wire err= token.

   ui_render_lcd() puts the last non-200 HTTP status on row 1 itself rather than trusting
   the caller to select it (spec 4.2): main.cpp's ui_fill_() chooses lcd_detail for a dozen
   other reasons, and a 400/401 loop is invisible to anyone not on the serial port.

   ui_modem_ran() is called DIRECTLY by net_poll() (task 24), not forwarded by main.cpp -
   9's grep over netfsm.cpp is safety.h|dose_run|hal_pump_write and ui.h is none of those.
   The pass-skip rule has its own case here, so the call cannot quietly be dropped."
   ```

---

---

### Task 11: `cli.cpp` — the line reader, the six bench commands and `status`

**Drop 1.**

**Files:**
- Create: `include/cli.h`, `src/cli.cpp`
- Modify: `test/test_cli/test_cli.cpp` (add four cases and their `RUN_TEST` lines)
- Test: `test/test_cli/test_cli.cpp`

**Interfaces:**
- Consumes: `size_t hal_serial_read(char*, size_t)`, `void hal_serial_write(const char*)`, `void hal_serial_drain(void)`, `uint32_t hal_millis(void)`, `int hal_pin_read(uint8_t)`, `bool hal_pump_level_on(void)`, `uint32_t hal_wdt_granted(void)`, `bool hal_wdt_alive(void)`, `uint32_t hal_wdt_last_delta(void)`, `uint8_t hal_adc_bits(void)`, `bool hal_adc_width_ok(void)`, `bool hal_irq_armed(uint8_t)`, `bool hal_irq_filtered(uint8_t)`, `uint32_t hal_heap_arena/ordblks/break(void)`, `uint32_t hal_stack_limit/hwm(void)` (task 3); `pb_noinit_t g_nv`, `noinit_was_cold()`, `noinit_reset_mid()` (task 4); `uint32_t pulses_flow/pulses_flow_rate/pulses_screw/pulses_leak_count(void)` (task 6); `bool sensors_read_raw(uint8_t, uint16_t*)`, `bool sensors_home_hall(bool*)`, `bool sensors_i2c_healthy(void)`, `uint32_t sensors_i2c_errors/sensors_i2c_txn_per_min(void)`, `void sensors_scan(char*, size_t)` (task 7); `PIN_FLOW`, `PIN_HALL_SCREW`, `PIN_HALL_FLOAT` (task 2).
- Produces:
  ```c
  void cli_begin(void);
  void cli_poll(void);
  bool cli_dispatch(const char *line);   /* flat if-chain; the testable entry point */
  void cli_print_status(void);
  void cli_printf_u32(const char *fmt, uint32_t v);
  void cli_printf_i32(const char *fmt, int32_t v);   /* RSSI is signed; task 24 step 17 needs it */
  ```
  Tasks 15, 16, 19 and 20 add `dry on|off`, `stop`, `clear contra` and the whole `#if PB_BRINGUP` block to this same file; tasks 14, 17, 22, 24, 25, 26 and 27 each add their own `status` lines.

**Spec sections to read in full before starting:** §6 (the command table and the `status` list); §7's `PB_LINE_CAP`; §12 items 0-2; §13 steps 1-3.

**The `adc_hw=` value, resolved.** §6 wants `status` to print `adc_req=14 adc_hw=14`, and spec §1's printed `hal.h` exposes no accessor for the hardware width. Task 3 adds one — `hal_adc_bits()`, declared in `include/hal.h` step 3 and defined in `hal_sim.cpp` step 6 — precisely so this line is a readback and not a label, and task 8 step 2 implements it for the board from the BSP's fixed-width feature macro. Print `adc_req=` from `PB_ADC_BITS` and `adc_hw=` from `hal_adc_bits()`, and add `adc_ok=` from `hal_adc_width_ok()`. If task 8 had to take its documented fallback (the feature macro would not resolve), `hal_adc_bits()` returns `PB_ADC_BITS` and this line must read `adc_hw=unverifiable` instead — say which shipped in that commit.

---

1. - [ ] Write the header:

   ```c
   /* include/cli.h -- the console. Bench commands always; bring-up commands under
      #if PB_BRINGUP (task 20). A flat if-chain, so cli_dispatch() is testable. */
   #pragma once
   #include <stdbool.h>
   #include <stdint.h>

   void cli_begin(void);
   void cli_poll(void);
   bool cli_dispatch(const char *line);
   void cli_print_status(void);
   void cli_printf_u32(const char *fmt, uint32_t v);
   /* RSSI is the one signed figure `status` prints; %d is banned in report.cpp and
      netfsm.cpp only, and cli.cpp is outside that grep (task 24 step 17). */
   void cli_printf_i32(const char *fmt, int32_t v);
   ```

2. - [ ] Add the four failing cases to `test/test_cli/test_cli.cpp`, above `main()`, and their `RUN_TEST` lines inside it. They drive the real serial fake in both directions.

   ```cpp
   #include "cli.h"
   #include "hal.h"
   #include "sim.h"

   static void drain_tx(void) { char b[2048]; sim_serial_tx(b, sizeof b); }

   /* cli_poll() reads at most sizeof(buf) == 32 bytes per call (step 4), and the overlong-line
      case below pushes ~136 bytes at it. ONE cli_poll() would consume 32 of them and never
      reach the newline, so "line too long" would never be printed and the case would fail for
      a reason that has nothing to do with the line reader. Loop, with a fixed bound so that a
      bug here cannot hang the suite. */
   static size_t feed(const char *line, char *out, size_t cap) {
     drain_tx();
     sim_serial_rx(line);
     for (unsigned i = 0; i < 16u; ++i) cli_poll();
     return sim_serial_tx(out, cap);
   }

   /* THE BENCH COMMAND SET of spec §6, and this case must end up carrying ALL of it.
      Six commands exist today. Four more arrive later and each is added to THIS case by the
      task that adds the command: `dry on` / `dry off` (task 15), `stop` (task 16),
      `clear contra` (task 19). Every one of them is present in the BENCH binary as well as
      the bringup one, so none of them may be wrapped in #if PB_BRINGUP here or there. */
   static void test_parses_every_bench_command(void) {
     pb_test_setup();
     TEST_ASSERT_TRUE(cli_dispatch("i2c"));
     TEST_ASSERT_TRUE(cli_dispatch("mux 3"));
     TEST_ASSERT_TRUE(cli_dispatch("mux all"));
     TEST_ASSERT_TRUE(cli_dispatch("hall"));
     TEST_ASSERT_TRUE(cli_dispatch("flow"));
     TEST_ASSERT_TRUE(cli_dispatch("status"));
     TEST_ASSERT_TRUE(cli_dispatch("help"));
     TEST_ASSERT_FALSE(cli_dispatch("mux 16"));      /* out of range */
     TEST_ASSERT_FALSE(cli_dispatch("nonsense"));
   }

   static void test_an_overlong_line_is_dropped_whole_not_truncated_into_a_command(void) {
     pb_test_setup();
     char line[PB_LINE_CAP + 40];
     memset(line, 'x', sizeof line);
     memcpy(line, "flow ", 5);                        /* a real command hiding at the front */
     line[sizeof line - 2] = '\n';
     line[sizeof line - 1] = '\0';
     char out[2048];
     size_t n = feed(line, out, sizeof out);
     out[n] = '\0';
     TEST_ASSERT_NOT_NULL(strstr(out, "line too long"));
     TEST_ASSERT_NULL(strstr(out, "flow hz="));       /* the prefix did NOT become a command */
   }

   static void test_status_reports_the_watchdog_grant_liveness_and_the_pump_active_level(void) {
     pb_test_setup();
     char out[2048];
     size_t n = feed("status\n", out, sizeof out);
     out[n] = '\0';
     TEST_ASSERT_NOT_NULL(strstr(out, "granted=5592ms"));
     TEST_ASSERT_NOT_NULL(strstr(out, "alive=yes"));
     TEST_ASSERT_NOT_NULL(strstr(out, "WDT, not IWDT"));
     TEST_ASSERT_NOT_NULL(strstr(out, "pump_on_level="));
     sim_wdt_stop();
     n = feed("status\n", out, sizeof out);
     out[n] = '\0';
     TEST_ASSERT_NOT_NULL(strstr(out, "alive=no"));
   }

   static void test_no_float_formatting_appears_in_any_printed_line(void) {
     /* newlib's float formatting is the deepest stack consumer in the program (spec §12),
        so the float conversions are banned. A float-formatted number shows as
        <digit>.<digit>. Two exemptions, and only two: the ip= line's dotted quad, and -
        from task 20 - the mls= field of the dose summary line, computed in integer tenths.

        THE TWO NEEDLES ARE BUILT CHARACTER BY CHARACTER ON PURPOSE. make check greps this
        tree for a percent sign followed by a float conversion letter, and it scans string
        literals in test/ exactly as it scans code; writing the needles out would make this
        file the one hit that fails the check it exists to defend. */
     pb_test_setup();
     char out[4096];
     size_t n = feed("status\n", out, sizeof out);
     out[n] = '\0';
     const char pct = '%';
     char needle[3] = { pct, 'f', '\0' };
     TEST_ASSERT_NULL(strstr(out, needle));
     needle[1] = 'g';
     TEST_ASSERT_NULL(strstr(out, needle));
     char *line = strtok(out, "\n");
     while (line) {
       if (strncmp(line, "ip=", 3) != 0)
         for (size_t i = 1; line[i] != '\0' && line[i + 1] != '\0'; ++i)
           if (line[i] == '.' && line[i - 1] >= '0' && line[i - 1] <= '9' &&
               line[i + 1] >= '0' && line[i + 1] <= '9')
             TEST_FAIL_MESSAGE(line);
       line = strtok(0, "\n");
     }
   }
   ```

3. - [ ] Run and watch it fail on the missing implementation:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cli
   ```
   expected: `undefined reference to 'cli_dispatch(char const*)'` and friends.

4. - [ ] Write the line reader and the dispatcher. Six commands land here — `i2c`, `mux`, `hall`, `flow`, `status`, `help` — implemented by five statics plus one streamer. An overlong line is **dropped whole**, never truncated into a command — a serial-monitor reconnect or a `cat` of the wrong file into `/dev/cu.*` must not be able to leave a valid prefix behind (spec §6). Move the five `cmd_*_()` statics and `print_mux_()` of step 5 above `cli_poll()` when you paste them in; C++ needs them declared before use, and `cmd_hall_line_()` in particular is called from `cli_poll()` above the dispatcher.

   ```cpp
   /* src/cli.cpp -- the console (spec §6). Bench commands here; task 20 adds the
      #if PB_BRINGUP block, and tasks 15/16/19 add dry, stop and the latch release.
      secrets.h is here for PB_CONTROLLER, which `status` prints: neither [env:uno_r4_wifi]
      nor [env:uno_r4_wifi_bringup] passes it in build_flags, and secrets.h is the only
      header that defines it. Task 14 adds cart.h, task 15/16 safety.h, task 24 netfsm.h
      and link.h -- each at TOP LEVEL, never inside the #if PB_BRINGUP block, because
      cli_print_status() calls into all of them unconditionally. */
   #include "cli.h"
   #include "config.h"
   #include "hal.h"
   #include "noinit.h"
   #include "pins.h"
   #include "pulses.h"
   #include "secrets.h"
   #include "sensors.h"
   #include <stdio.h>
   #include <string.h>

   #if defined(PB_SIM)
   #  define PB_BUILD_NAME "sim"
   #elif defined(PB_BRINGUP)
   #  define PB_BUILD_NAME "bringup"
   #else
   #  define PB_BUILD_NAME "bench"
   #endif

   static char     g_line[PB_LINE_CAP];
   static uint16_t g_len;
   static bool     g_overlong;
   static bool     g_hall_stream;
   static uint32_t g_hall_next_ms;
   static uint32_t g_arena_min, g_arena_max, g_hwm_max;

   void cli_printf_u32(const char *fmt, uint32_t v) {
     char b[PB_LINE_CAP];
     snprintf(b, sizeof b, fmt, (unsigned long)v);
     hal_serial_write(b);
   }

   void cli_printf_i32(const char *fmt, int32_t v) {
     char b[PB_LINE_CAP];
     snprintf(b, sizeof b, fmt, (long)v);
     hal_serial_write(b);
   }

   static bool parse_u32_(const char *s, uint32_t *out) {
     uint32_t v = 0;
     if (*s < '0' || *s > '9') return false;
     for (; *s != '\0'; ++s) {
       if (*s < '0' || *s > '9') return false;
       v = v * 10u + (uint32_t)(*s - '0');
     }
     *out = v;
     return true;
   }

   void cli_begin(void) {
     g_len = 0; g_overlong = false; g_hall_stream = false;
     g_arena_min = 0xFFFFFFFFu; g_arena_max = 0; g_hwm_max = 0;
     hal_serial_write("type help\n");
   }

   static void note_memory_(void) {
     uint32_t a = hal_heap_arena(), h = hal_stack_hwm();
     if (a < g_arena_min) g_arena_min = a;
     if (a > g_arena_max) g_arena_max = a;
     if (h > g_hwm_max)   g_hwm_max = h;
   }

   void cli_poll(void) {
     note_memory_();
     if (g_hall_stream && hal_millis() >= g_hall_next_ms) {
       g_hall_next_ms = hal_millis() + 200u;         /* 5 Hz, spec §13 step 3 */
       cmd_hall_line_();
     }
     char buf[32];
     size_t n = hal_serial_read(buf, sizeof buf);
     if (n > 0) g_hall_stream = false;               /* any byte stops the stream */
     for (size_t i = 0; i < n; ++i) {
       char c = buf[i];
       if (c == '\r') continue;
       if (c == '\n') {
         if (g_overlong)   hal_serial_write("line too long, dropped whole\n");
         else if (g_len)   { g_line[g_len] = '\0'; cli_dispatch(g_line); }
         g_len = 0; g_overlong = false;
         continue;
       }
       if (g_overlong) continue;                     /* keep discarding until the newline */
       if (g_len + 1u >= PB_LINE_CAP) { g_overlong = true; g_len = 0; continue; }
       g_line[g_len++] = c;
     }
   }

   bool cli_dispatch(const char *line) {
     if (strcmp(line, "i2c")    == 0) { cmd_i2c_();    return true; }
     if (strncmp(line, "mux ", 4) == 0) return cmd_mux_(line + 4);
     if (strcmp(line, "hall")   == 0) { g_hall_stream = true; g_hall_next_ms = 0; return true; }
     if (strcmp(line, "flow")   == 0) { cmd_flow_();   return true; }
     if (strcmp(line, "status") == 0) { cli_print_status(); return true; }
     if (strcmp(line, "help")   == 0) { cmd_help_();   return true; }
     hal_serial_write("? unknown; type help\n");
     return false;
   }
   ```

5. - [ ] Write the four bench commands. `hall` never calls `pinMode` on D2/D3 — spec §2.14: a later `pinMode` on an IRQ pin drops `IOPORT_CFG_IRQ_ENABLE` and silently detaches the interrupt, and `make check` greps for exactly that.

   ```cpp
   static void cmd_i2c_(void) {                       /* bring-up 1: expect 0x20, 0x27, 0x3C */
     char scan[96];
     sensors_scan(scan, sizeof scan);
     hal_serial_write("i2c: ");
     hal_serial_write(scan);
     hal_serial_write("\n");
   }

   static void print_mux_(uint8_t ch) {               /* select, >=1 ms, read twice, keep 2nd */
     uint16_t raw = 0;
     char b[48];
     if (sensors_read_raw(ch, &raw))
       snprintf(b, sizeof b, "mux %u = %lu\n", (unsigned)ch, (unsigned long)raw);
     else
       snprintf(b, sizeof b, "mux %u = error\n", (unsigned)ch);   /* never a value on failure */
     hal_serial_write(b);
   }

   static bool cmd_mux_(const char *arg) {
     if (strcmp(arg, "all") == 0) {
       for (uint8_t ch = 0; ch < 16; ++ch) print_mux_(ch);
       return true;
     }
     uint32_t ch;
     if (!parse_u32_(arg, &ch) || ch > 15u) { hal_serial_write("mux <0-15>|all\n"); return false; }
     print_mux_((uint8_t)ch);
     return true;
   }

   /* Streams screw / home / float. An I2C error prints `home unknown` -- never `home 0`,
      because a bus error is not the same fact as "not home" (spec §6, §13 step 3). */
   static void cmd_hall_line_(void) {
     bool home = false;
     char b[64];
     if (sensors_home_hall(&home))
       snprintf(b, sizeof b, "hall screw=%lu home=%u float=%u\n",
                (unsigned long)pulses_screw(), (unsigned)home,
                (unsigned)(hal_pin_read(PIN_HALL_FLOAT) == PB_LOW));
     else
       snprintf(b, sizeof b, "hall screw=%lu home=unknown float=%u\n",
                (unsigned long)pulses_screw(),
                (unsigned)(hal_pin_read(PIN_HALL_FLOAT) == PB_LOW));
     hal_serial_write(b);
   }

   static void cmd_flow_(void) {
     char b[64];
     snprintf(b, sizeof b, "flow hz=%lu total=%lu leak=%lu\n",
              (unsigned long)pulses_flow_rate(), (unsigned long)pulses_flow(),
              (unsigned long)pulses_leak_count());
     hal_serial_write(b);
   }

   static void cmd_help_(void) {
     hal_serial_write(
       "i2c              scan the bus (expect 0x20 0x27 0x3C)\n"
       "mux <0-15>|all   select, settle, read twice, print the second (14-bit raw)\n"
       "hall             stream screw/home/float at 5 Hz; any key stops it\n"
       "flow             pulses/second and total since reset\n"
       "status           everything this board knows about itself\n"
       "help             this\n");
   }
   ```

6. - [ ] Write `status`. Every fractional figure is integer tenths printed `%lu.%lu`; there is no `%f` anywhere in this program (spec §12 item 1). `build=` here is one of the only two places `PB_BRINGUP` may appear in the tree (the other is `main.cpp`).

   ```cpp
   void cli_print_status(void) {
     char b[160];
     note_memory_();

     snprintf(b, sizeof b, "build=%s controller=%s\n", PB_BUILD_NAME, PB_CONTROLLER);
     hal_serial_write(b);

   #ifdef PB_RELAY_ACTIVE_LOW
     const char *pol = "ACTIVE_LOW";
   #else
     const char *pol = "ACTIVE_HIGH";
   #endif
     snprintf(b, sizeof b, "pump_on_level=%u polarity=%s\n",
              (unsigned)hal_pump_level_on(), pol);
     hal_serial_write(b);

     snprintf(b, sizeof b,
              "wdt=%s granted=%lums alive=%s delta=%lu "
              "(WDT, not IWDT - DECISIONS #10 says IWDT; see "
              "docs/superpowers/specs/2026-09-03-bench-sketch-design.md 2.5)\n",
              hal_wdt_granted() ? "on" : "off", (unsigned long)hal_wdt_granted(),
              hal_wdt_alive() ? "yes" : "no", (unsigned long)hal_wdt_last_delta());
     hal_serial_write(b);

     snprintf(b, sizeof b, "icufilter=%s %s irq_armed=%s %s\n",
              hal_irq_filtered(PIN_FLOW) ? "yes" : "no",
              hal_irq_filtered(PIN_HALL_SCREW) ? "yes" : "no",
              hal_irq_armed(PIN_FLOW) ? "yes" : "no",
              hal_irq_armed(PIN_HALL_SCREW) ? "yes" : "no");
     hal_serial_write(b);

     cli_printf_u32("uptime=%lus\n", hal_millis() / 1000u);

     /* spec §6's two widths, both real: adc_req is what we asked the core to map to, adc_hw
        is the fixed hardware width read back through seam 1 (task 3's hal_adc_bits()). A
        mismatch silently rescales every raw count on the wire, so main.cpp latches err=adc
        and disables the network for it (task 12). */
     snprintf(b, sizeof b, "adc_req=%lu adc_hw=%lu adc_ok=%s\n",
              (unsigned long)PB_ADC_BITS, (unsigned long)hal_adc_bits(),
              hal_adc_width_ok() ? "yes" : "no");
     hal_serial_write(b);

     snprintf(b, sizeof b, "screw=%lu flow_total=%lu flow_hz=%lu leak=%lu\n",
              (unsigned long)pulses_screw(), (unsigned long)pulses_flow(),
              (unsigned long)pulses_flow_rate(), (unsigned long)pulses_leak_count());
     hal_serial_write(b);

     snprintf(b, sizeof b, "i2c errors=%lu txn_per_min=%lu healthy=%s\n",
              (unsigned long)sensors_i2c_errors(), (unsigned long)sensors_i2c_txn_per_min(),
              sensors_i2c_healthy() ? "yes" : "no");
     hal_serial_write(b);

   #if PB_PULSES_PER_GATE == 0
     hal_serial_write("cart=UNCALIBRATED (PB_PULSES_PER_GATE=0) - goto refuses, pos never ok\n");
   #else
     cli_printf_u32("cart pulses_per_gate=%lu\n", (uint32_t)PB_PULSES_PER_GATE);
   #endif
   #if PB_REPORT_POS_UNKNOWN
     hal_serial_write("pos: FORCED unknown (PB_REPORT_POS_UNKNOWN=1)\n");
   #endif

     snprintf(b, sizeof b,
              "arena=%lu (min %lu max %lu) ordblks=%lu break=0x%lx stack_hwm=%lu (max %lu) "
              "headroom=%lu\n",
              (unsigned long)hal_heap_arena(), (unsigned long)g_arena_min,
              (unsigned long)g_arena_max, (unsigned long)hal_heap_ordblks(),
              (unsigned long)hal_heap_break(), (unsigned long)hal_stack_hwm(),
              (unsigned long)g_hwm_max,
              (unsigned long)(hal_stack_limit() - hal_heap_break()));
     hal_serial_write(b);

     /* The raw .noinit struct: bring-up 7c' reads exactly this line after a forced reset.
        cold= and resetmid= are noinit.cpp's two accessors (task 4) and this is their only
        consumer -- without them the struct's numbers are readable but the VERDICT the boot
        drew from them is not, and 7c's pass criterion is the verdict. */
     snprintf(b, sizeof b,
              "nv magic=0x%lx boots=%lu chw=%lu dry=%u contra=%u inflight=%u "
              "pattern=0x%lx sum=0x%lx cold=%u resetmid=%u\n",
              (unsigned long)g_nv.magic, (unsigned long)g_nv.boots,
              (unsigned long)g_nv.cmd_high_water, (unsigned)g_nv.dry_latched,
              (unsigned)g_nv.contra_latched, (unsigned)g_nv.dose_in_flight,
              (unsigned long)g_nv.pattern, (unsigned long)g_nv.sum,
              (unsigned)noinit_was_cold(), (unsigned)noinit_reset_mid());
     hal_serial_write(b);

     /* PLACEHOLDER. Task 17 step 9 replaces this line with
          hal_serial_write("last="); hal_serial_write(safety_last_err()); hal_serial_write("\n");
        and that replacement is a numbered step of task 17, not a hope. `last=` is bring-up
        7c's pass criterion (`last=resetmid` after a hang-forced reset), so a hard-coded
        `none` here would make that step unpassable. */
     hal_serial_write("last=none\n");
   }
   ```

7. - [ ] Run the suite and watch all of it pass:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cli
   ```
   expected: ten `[PASSED]` lines (task 10's six plus these four).

8. - [ ] Confirm the two device builds still link and that `PB_BRINGUP` now lives in exactly the files it is allowed to:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio run -e uno_r4_wifi -e uno_r4_wifi_bringup 2>&1 | tail -3 && grep -rl 'PB_BRINGUP' include src lib test
   ```
   expected: `SUCCESS`, then `src/cli.cpp` alone until task 12 adds `src/main.cpp`.

9. - [ ] Commit.

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && git add include/cli.h src/cli.cpp test/test_cli/test_cli.cpp && git commit -m "cli: the line reader, the six bench commands, and status

   An overlong line is dropped whole rather than truncated into a command: a stray line
   from a serial reconnect must not be able to leave a valid prefix behind. hall never
   calls pinMode on D2 or D3 - that would drop IOPORT_CFG_IRQ_ENABLE and silently detach
   the interrupt - and an I2C error prints home unknown, never home 0.

   status prints adc_req from the constant and adc_hw from hal_adc_bits(), the readback
   task 3 added to seam 1 because the core's own resolution setter returns void and the
   assertion spec §6 asks for has no getter to assert against. last=none is a placeholder
   that task 17 step 9 replaces with safety_last_err(); it is a numbered step there, not a
   hope, because last=resetmid is bring-up 7c's pass criterion."
   ```

---

---

### Task 12: `main.cpp` — the setup order and the loop

**Drop 1.**

**Files:**
- Modify: `src/main.cpp` (from task 9's four-line stub to the full setup order and loop)
- Test: **none on the host.** `[env:native]` filters `main.cpp` out (it has `setup`/`loop` and no `main()`). What proves this file is `pio run -e uno_r4_wifi`, and **bring-up step 0** (spec §13): flash the bringup binary, open the monitor at 115200, and read the banner *before 12 V goes onto COM*.

**Interfaces:**
- Consumes: `hal_boot_pump_off()`, `hal_begin()`, `hal_i2c_probe()`, `hal_wdt_start()`, `hal_wdt_granted()`, `hal_wdt_alive()`, `hal_adc_bits()`, `hal_adc_width_ok()`, `hal_heap_break()`, `hal_stack_limit()`, `hal_serial_write()` (tasks 3, 8); `noinit_begin()`, `noinit_commit()`, `noinit_reset_mid()`, `g_nv` (task 4); `safety_tick()`, `safety_dosing()` (task 5); `pulses_begin()`, `pulses_flow()`, `pulses_flow_rate()`, `pulses_screw()`, `pulses_leak_poll()` (task 6); `sensors_begin()` (task 7 — **not** `sensors_sweep()`: task 24's `NET_IDLE` pass is its only caller, and there is no `sensors_poll()` in this plan); `Screen::probe/begin/clear` and the two panel objects (task 9); `ui_poll()`, `ui_state_t` (task 10); `cli_begin()`, `cli_poll()` (task 11).
- Produces: `void setup(void)`, `void loop(void)`, `Screen g_oled_screen`, `Screen g_lcd_screen` (referenced `extern` by `ui.cpp`), and `const char *main_boot_err(void)` / `bool main_net_disabled(void)`.

  **What consumes `main_net_disabled()`, named here so it is not left dead.** Task 26 step 13 reads it once, in `setup()`, and forwards it: `if (main_net_disabled()) net_disable(main_boot_err());`. `netfsm.cpp` owns the disabled flag from there on (task 24 produces `net_disable(const char *why)` and `net_disabled(void)` — those two spellings and no others), because `main.cpp` is filtered out of `[env:native]` and nothing in a host test could otherwise reach a flag that lives here. Spec §2.5 says three times that a failed boot assertion "disables the network and says why in `status`"; this is the whole of the mechanism, and neither half works without the other.

  `PB_BRINGUP` appears in this file and in `cli.cpp`, and nowhere else in the tree.

**Spec sections to read in full before starting:** §2.5 (the setup assertions); §3 (the loop); §5 (init ordering is fixed); §12 item 0; §13 step 0.

**Two deviations, to record in the commit message.**

- Spec §12's order lists "Serial at 115200" between `noinit_begin()` and `hal_begin()`, but `main.cpp` may not include the Arduino header (spec §9 allows it only in `hal_uno.cpp`, `lib/Network` and `lib/Screen`) and `hal.h` has no `hal_serial_begin()`. So `hal_begin()` opens the console at 115200 as **its first statement** — the same order, one function later. **Task 8 step 2 writes that line, as the first statement of `hal_begin()`**; if it is not there, nothing on this page prints and bring-up step 0 has no banner. Check it before flashing.
- **`loop()` is six lines, not spec §3's five.** The sixth is `pulses_leak_poll(safety_dosing())`. Spec §3's printed loop has no leak poll, yet §1 makes `pulses_leak_poll()` the only thing that advances the leak count, §4.1 puts that count on the wire as `ch205 = leak pulses since boot`, and §4.1's fixed `err=` enum carries `leak`. It is cheap (two counter reads), it must run on **every** pass rather than once per report cycle, and without it `ch205` is permanently 0 and `err=leak` has no producer anywhere. Record it as a deviation with that reasoning; do not add a seventh. **`sensors_sweep()` is NOT in the loop**: task 24's `NET_IDLE` pass owns it, once per report cycle, and that pass issues zero AT commands, which is how §3's "skipped in any pass where a modem command ran" holds by construction rather than by a flag this file would have to consult.

---

1. - [ ] Read the two invariants this file's first two statements exist to satisfy, and do not reorder them:
   - **spec §2.1:** `hal_boot_pump_off()` is `setup()`'s FIRST statement, and it is one whole-word PFS write carrying direction AND level. `pinMode` must never touch D6: on this core `pinMode(pin, OUTPUT)` latches PODR = 0 and drives the pin LOW, discarding a preceding level write. On an active-LOW module the wiring README's own recipe would assert the pump for the whole of `setup()` — through a one-second blocking wait inside the LCD library — on **every** boot, watchdog resets included.
   - **spec §5:** both panels are probed and opened **before** `sensors_begin()`, because `LiquidCrystal_I2C::init_priv()` re-opens the IIC peripheral and would tear down the bus underneath the expander.

2. - [ ] Write `setup()`'s first half — the pin, the warm-reset struct, the HAL, the panels, the sensors and the pulse counters, in that order:

   ```cpp
   /* src/main.cpp -- DEVICE ONLY ([env:native] filters this file out).
      setup()'s ORDER is load-bearing: spec §2.1 pins the first statement, spec §5 pins the
      panels-before-sensors rule, spec §2.5 the three assertions and spec §12 the break check.
      No Arduino header here: spec §9 allows it only in hal_uno.cpp, lib/Network, lib/Screen.
      secrets.h is here for PB_CONTROLLER, which ui_fill_() copies into ui_state_t: the two
      device envs do not pass it in build_flags and secrets.h is its only other definition. */
   #include "Screen.h"
   #include "cli.h"
   #include "config.h"
   #include "hal.h"
   #include "noinit.h"
   #include "pins.h"
   #include "pulses.h"
   #include "safety.h"
   #include "secrets.h"
   #include "sensors.h"
   #include "ui.h"
   #include <stdio.h>
   #include <string.h>

   #if defined(PB_SIM)
   #  define PB_BUILD_NAME "sim"
   #elif defined(PB_BRINGUP)
   #  define PB_BUILD_NAME "bringup"
   #else
   #  define PB_BUILD_NAME "bench"
   #endif

   Screen g_oled_screen(ScreenType::Oled);
   Screen g_lcd_screen(ScreenType::Lcd);

   static bool        g_net_disabled;
   static const char *g_boot_err = "none";
   static ui_state_t  g_ui;                 /* file-static: the main stack is 1024 B (spec §12) */

   bool        main_net_disabled(void) { return g_net_disabled; }
   const char *main_boot_err(void)     { return g_boot_err; }

   void setup(void) {
     hal_boot_pump_off();   /* FIRST. One PFS write: direction AND level, atomically (spec §2.1) */
     noinit_begin();        /* magic + checksum; a dose in flight across the reset latches dry */
     hal_begin();           /* opens the console at 115200, then ADC width, pins, ISRs, Wire,
                               servo, stack paint */

     hal_i2c_probe(I2C_ADDR_OLED);
     hal_i2c_probe(I2C_ADDR_LCD);
     g_oled_screen.probe();
     g_oled_screen.begin();
     g_lcd_screen.probe();
     g_lcd_screen.begin();  /* BEFORE sensors_begin(): init_priv() re-opens the bus (spec §5) */
     g_oled_screen.clear();  /* a panel that did not answer probe() is a no-op here (task 9) */
     g_lcd_screen.clear();

     sensors_begin();
     pulses_begin();
   ```

3. - [ ] Write `setup()`'s second half — the watchdog, its three assertions and the break check. **Each assertion disables the network and says why in `status`; none of them reset-loops. A board that reports nothing is better than one that reset-loops through a router reboot (spec §2.5).**

   ```cpp
     /* spec §2.3: a reset taken with the pump asserted is the single loudest thing this rig
        can discover about itself. noinit_begin() has already latched the verdict and the dry
        latch; this is where the token is raised and the flag CLEARED — exactly once per boot,
        and here rather than anywhere else, because a dose_in_flight nobody clears re-latches
        dry on every subsequent warm boot forever. Bring-up 7c's pass criterion is that
        `status` then says dry=1 and last=resetmid. */
     if (noinit_reset_mid()) {
       g_boot_err = "resetmid";              /* the network stays ENABLED: this is a report,
                                                not a reason to stop reporting */
       g_nv.dose_in_flight = false;
       noinit_commit();
     }

     if (!hal_wdt_start()) { g_net_disabled = true; g_boot_err = "wdt"; }

     /* spec §2.5: the library's timeout getter returns 0 under the wdt_cfg_t overload even
        on a running dog, so hal_wdt_granted() computes the grant itself and this asserts it
        against the constant. (Do not name that getter here: make check greps it to zero
        across the tree, comments included.) */
     if (hal_wdt_granted() != PB_WDT_GRANTED_MS) { g_net_disabled = true; g_boot_err = "wdt"; }

     /* spec §3: the worst net step is 2 AT commands = 2400 ms; 2400 + slack must fit. */
     if (hal_wdt_granted() < 2u * PB_NET_STEP_MS + PB_NET_SLACK_MS) {
       g_net_disabled = true; g_boot_err = "wdt";
     }

     /* spec §2.5: liveness, not a constant. The counter must DECREASE across a 40 ms UNFED
        window. A failure here also latches dry. (Task 15 replaces the two lines below with
        safety_dry_set(true).) */
     if (!hal_wdt_alive()) {
       g_net_disabled = true; g_boot_err = "wdt";
       g_nv.dry_latched = true;
       noinit_commit();
     }

     /* spec §7: the hardware ADC width is fixed and analogReadResolution() only stores the
        REQUESTED one, so a core bump that changed the fixed width would silently rescale
        every raw count on the wire with no error anywhere. hal_begin() computed the answer;
        this is the only producer of err=adc, a token §4.1's fixed enum already carries. */
     if (!hal_adc_width_ok()) { g_net_disabled = true; g_boot_err = "adc"; }

     /* spec §12 item 0: _sbrk is the unchecked libnosys version and __HeapLimit is referenced
        by nothing in the image, so the break against the stack is the ONLY heap bound that
        exists. The network stack is the largest allocator in the program, so crossing this
        is the one case where continuing is how the corruption reaches a water command. */
     if (hal_heap_break() >= hal_stack_limit() - PB_STACK_MARGIN) {
       g_net_disabled = true; g_boot_err = "heap";
     }

     cli_begin();

     /* Bring-up step 0's pass criterion, read BEFORE 12 V goes onto COM (spec §13). */
     {
       char b[160];
       snprintf(b, sizeof b,
                "\nPB bench sketch build=%s dry=%u contra=%u pump_on_level=%u "
                "wdt=%s granted=%lums alive=%s adc=%lu/%lu oled=%u lcd=%u net=%s last=%s\n",
                PB_BUILD_NAME, (unsigned)g_nv.dry_latched, (unsigned)g_nv.contra_latched,
                (unsigned)hal_pump_level_on(), hal_wdt_granted() ? "on" : "off",
                (unsigned long)hal_wdt_granted(), hal_wdt_alive() ? "yes" : "no",
                (unsigned long)PB_ADC_BITS, (unsigned long)hal_adc_bits(),
                /* Screen::present()'s one consumer. Bring-up step 0 reads the banner before
                   step 1 scans the bus, so a panel that did not answer probe() is named here
                   rather than discovered later as a screen that simply never updates. */
                (unsigned)g_oled_screen.present(), (unsigned)g_lcd_screen.present(),
                g_net_disabled ? "DISABLED" : "enabled", g_boot_err);
       hal_serial_write(b);
     }
   }
   ```

4. - [ ] Write the loop — spec §3's five lines plus `pulses_leak_poll()`, of which four exist today (`net_poll` and `exec_pending` arrive in task 26).

   ```cpp
   static void ui_fill_(ui_state_t *s) {
     memset(s, 0, sizeof *s);
     strncpy(s->build, PB_BUILD_NAME, sizeof s->build - 1);
     strncpy(s->controller, PB_CONTROLLER, sizeof s->controller - 1);
     s->uptime_min   = hal_millis() / 60000u;      /* MINUTES: spec §5's bus rule */
     s->pump_on      = safety_dosing();            /* ui.cpp may not include safety.h itself */
     s->float_ok     = (hal_pin_read(PIN_HALL_FLOAT) == PB_LOW);
     s->screw_pulses = pulses_screw();
     s->flow_hz      = pulses_flow_rate();
     s->flow_total   = pulses_flow();
     s->dry          = g_nv.dry_latched;
     s->contra       = g_nv.contra_latched;
   #ifdef PB_SIM
     s->sim = true;
   #endif
     s->lcd_state  = s->pump_on ? "PUMP" : "IDLE";
     s->lcd_detail = s->float_ok ? "float ok" : "float NOT OK";
     /* TASK 26 OWNS THE REST OF THIS FUNCTION and rewrites it whole: pos/pos_known/parked
        from the cart, link/rssi/ip from seam 2, http_status/next_s from the report FSM,
        cmd_id/cmd_text from exec.cpp, and spec §5's real lcd_state selection. Until then
        rows 1 and 4-7 of the OLED read pos ?, wifi -- 0 dBm, no link, next 0s and cmd -,
        which is correct for a tree with no cart and no network in it. */
   }

   void loop(void) {
     safety_tick();     /* pump idle re-asserted, D6's direction repaired, then the dog fed */
     cli_poll();        /* one whole line; may block, but only through safety_wait_ms() */
     /* net_poll(safety_dosing()); <- task 26 adds this line (spec §3) */
     /* exec_pending();            <- task 26 adds this line (spec §3) */
     pulses_leak_poll(safety_dosing());   /* the leak watch, EVERY pass: ch205's only driver.
                                             report_build() (task 22) is what turns a non-zero
                                             count into err=leak; nothing here does. */
     ui_fill_(&g_ui);
     ui_poll(&g_ui);    /* no-ops while the pump is asserted, the cart moves, or a modem ran */
   }
   ```

   `ui_fill_()` is a helper of this file and not a sixth loop *responsibility*;
   `pulses_leak_poll()` is the sixth, and the deviation note above is where it is justified.
   **There is no `sensors_poll()` and no `sensors_sweep()` here**: task 24's `NET_IDLE` pass
   owns the sweep, once per report cycle, in the one pass that issues no AT command. Do not
   add a seventh line: the rule the whole design rests on is that every loop which can iterate
   over an I2C transfer, a modem call or a millisecond of wall clock calls `safety_tick()` on
   each iteration, and that rule lives inside the callees, not here.

5. - [ ] Build both device envs:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio run -e uno_r4_wifi -e uno_r4_wifi_bringup 2>&1 | tail -4
   ```
   expected: two `SUCCESS` lines, and RAM/Flash figures that you should note — `.bss` should be a few hundred bytes above today's 5480 (spec §12 predicts ~1.9 KB of our own by the end of drop 3).

6. - [ ] Confirm `PB_BRINGUP` is now in exactly two files, which is what task 13's invariant will assert:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && grep -rl 'PB_BRINGUP' include src lib test
   ```
   expected, exactly: `src/cli.cpp` and `src/main.cpp`.

7. - [ ] Flash the bring-up binary and read the banner. **This is bring-up step 0, and it happens before 12 V goes onto COM.**

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && make bringup && make monitor
   ```
   expected, on the monitor at 115200:
   ```
   PB bench sketch build=bringup dry=0 contra=0 pump_on_level=1 wdt=on granted=5592ms alive=yes adc=14/14 oled=1 lcd=1 net=enabled last=none
   type help
   ```
   `oled=1 lcd=1` says both panels answered `probe()`. A 0 there is not fatal — that panel becomes a permanent no-op (task 9) — but it is the first thing bring-up step 1's bus scan will explain.
   `granted=5592ms` and `alive=yes` are the numbers, not adjectives: 5592 is `16384 × 8192 / 24000` and `alive` means the down-counter actually decreased across a 40 ms unfed window. If `alive=no`, stop — nothing downstream is safe. If `pump_on_level` does not match the relay module you read in bring-up 4a, fix the build flag before going further.

8. - [ ] Commit.

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && git add src/main.cpp && git commit -m "main: the setup order, the three boot assertions, and three of the five loop lines

   hal_boot_pump_off() is the first statement and it is one PFS write carrying direction
   and level; both panels are probed and opened before sensors_begin(), because the LCD
   library re-opens the IIC peripheral. Each of the three watchdog assertions and the
   break-against-stack check disables the network and names itself in status rather than
   reset-looping - a board that reports nothing beats one that reset-loops through a
   router reboot.

   Two deviations, recorded. Serial is opened by hal_begin()'s first statement rather than
   by main.cpp, because main.cpp may not include the Arduino header (spec §9) and hal.h has
   no serial-begin: same order, one function later, and task 8 step 2 owns the line. And
   loop() is six lines rather than spec §3's five - pulses_leak_poll(safety_dosing()) is the
   sixth, because it is the only thing that advances the leak count, §4.1 puts that count on
   the wire as ch205, and it has to run on every pass rather than once per report cycle.
   report_build() (task 22) is what turns a non-zero count into err=leak, the token in §4.1's
   fixed enum that would otherwise have no producer at all. sensors_sweep() is deliberately NOT here: task 24's
   NET_IDLE pass owns it, and that pass issues zero AT commands, which is how §3's
   'skipped in any pass where a modem command ran' holds by construction.

   setup() also consumes noinit_reset_mid(): it raises err=resetmid and CLEARS dose_in_flight,
   exactly once per boot. Without that clear the flag re-latches dry on every warm boot for
   the rest of the board's life, and bring-up 7c's pass criterion is unreachable.

   The ADC width is asserted at boot and latches err=adc, which until now was a token in
   §4.1's enum with no producer at all.

   No host test covers this file - [env:native] filters it out. Bring-up step 0 is what
   proves it: build=bringup dry=0 contra=0 and the expected pump level, read before 12 V
   goes onto COM."
   ```

---

---

### Task 13: `tools/check.sh` and `make check` — the drop-1 mechanical invariants

**Drop 1.**

**Files:**
- Modify: `tools/check.sh` (consolidate task 8's and task 9's greps and complete the drop-1 set), `Makefile` (the `check` target already calls it — confirm)
- Test: `make check` is itself the test.

**Interfaces:**
- Consumes: the whole drop-1 tree.
- Produces: `make check`, green on the drop-1 tree and non-zero with a named invariant when any of them is broken. Task 30 adds the invariants of spec §9's table that need drop-2 and drop-3 code to be decidable. Do not carry a count of the invariants in prose anywhere: spec §9's table is the authority and the count drifts every time a task adds one.

**Spec sections to read in full before starting:** §9's "Mechanical invariants" table; §16.2.

**Comment hygiene** is stated in full in this plan's Global Constraints and starts at task 8, which is the first file that would trip it. Nothing new is introduced here; if a grep added below fires on a comment, reword the comment and never weaken the pattern.

---

1. - [ ] **Confirm — do not re-add — the six greps already in the file.** Task 8 step 1 wrote the four D6 invariants of spec §2.1/§2.2 and two of the four watchdog ones, in exactly the `expect`/`count`/`files` style used below, immediately under the `# ---- invariants land here (task 13, then task 30) ----` marker; task 9 step 6 added the two `lib/Screen` greps beneath them. Read the file and check they are there, once each:

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && grep -cE '^[[:space:]]*expect ' tools/check.sh
   ```
   expected: `8` — six from task 8 step 1 and two from task 9 step 6. **The anchor matters:
   a bare `grep -c 'expect '` returns 9, because task 1 step 11's skeleton carries the
   helper's own documentation line `# expect <want> <got> <description>`, which matches the
   pattern and is not an invariant.** Count invocations, not mentions. **If any of the six
   D6/watchdog greps appears twice, or if task 8's D2/D3 `awk` range check is already present,
   delete the duplicate now** — the whole point of this task is that `make check` prints one line per invariant, and a doubled grep is a doubled `ok` line that reads as two invariants where there is one. Step 2 below is the D2/D3 check's only home. For reference, the six that must already be present are:

   ```bash
   # ---- D6: spec §2.1, §2.2 (task 8 step 1 — already present) ----
   # The pattern is the DEFINITION, never the bare token: include/pins.h necessarily *tests*
   # PB_PUMP_OWNER with #ifdef, so a bare-token file count would be 2 forever.
   expect 1 "$(files 'define[[:space:]]+PB_PUMP_OWNER' "${SCAN[@]}")" \
     "exactly one file defines PB_PUMP_OWNER, so exactly one file gets PIN_PUMP_EN"
   expect 0 "$(count 'pinMode\(PIN_PUMP_EN' "${SCAN[@]}")" \
     "pinMode never touches D6 (it would latch PODR=0 and drive the pin LOW)"
   expect 2 "$(count 'R_IOPORT_PinCfg.*PIN_PUMP_EN' "${SCAN[@]}")" \
     "exactly two whole-word PFS writes to D6 (hal_boot_pump_off, hal_pump_write)"
   expect 0 "$(count 'R_IOPORT_PinWrite.*PIN_PUMP_EN|digitalWrite.*PIN_PUMP_EN' "${SCAN[@]}")" \
     "no unverifiable write form on D6"
   # ---- and two of the four watchdog greps (task 8 step 1 — already present) ----
   expect 1 "$(count 'WDT\.refresh' "${SCAN[@]}")" \
     "one refresh call site, inside hal_wdt_feed"
   expect 0 "$(count 'WDT\.getTimeout' "${SCAN[@]}")" \
     "the timeout getter is never used (it returns 0 under the wdt_cfg_t overload)"
   ```

2. - [ ] Add the pulse-pin invariant. D2 and D3 are configured in exactly one function; **any later `pinMode` on those pins drops `IOPORT_CFG_IRQ_ENABLE` and silently detaches the interrupt** (spec §2.14), which on D3 is lost cart position and on D2 is a flow meter that has stopped counting without saying so.

   ```bash
   # ---- D2/D3: spec §2.14. Both hits must be inside hal_arm_pulse_pins_. ----
   pp_all=$(count 'pinMode\((PIN_FLOW|PIN_HALL_SCREW)' "${SCAN[@]}")
   pp_fn=$(awk '/hal_arm_pulse_pins_/ {f=1}
                f && /pinMode\((PIN_FLOW|PIN_HALL_SCREW)/ {c++}
                f && /^}/ {f=0}
                END {print c+0}' src/hal_uno.cpp 2>/dev/null)
   expect "$pp_all" "$pp_fn" \
     "every pinMode on D2/D3 is inside hal_arm_pulse_pins_ (a later one detaches the interrupt)"
   ```

3. - [ ] Add the **two remaining** watchdog invariants — the refresh-call-site and timeout-getter greps are already in the file from task 8, so adding them again here would print each twice. **`safety_tick()` is the only caller of `hal_wdt_feed()`, and `hal_wdt_alive()`'s 40 ms probe is the one place in the program where feeding is deliberately suspended** (spec §2.4, §2.5).

   ```bash
   # ---- the watchdog, continued: spec §2.4, §2.5 ----
   expect 1 "$(count 'hal_wdt_feed\(' src/safety.cpp)" \
     "safety_tick is the ONLY feeder in safety.cpp"
   # hal.h's declaration is excluded: a seam has to declare what it seams.
   expect 0 "$(grep -rEn 'hal_wdt_feed\(' "${SCAN[@]}" \
                 --exclude=safety.cpp --exclude=hal_uno.cpp --exclude=hal_sim.cpp \
                 --exclude=hal.h 2>/dev/null | wc -l | tr -d ' ')" \
     "nothing outside safety.cpp and the two HALs feeds the dog"
   ```

4. - [ ] Add the blocking-and-formatting invariants. The `\b`-free forms below are deliberate: `/usr/bin/grep` on macOS is BSD grep.

   ```bash
   # ---- blocking, buffers and formatting: spec §3, §9, §12 ----
   expect 0 "$(grep -rEn '(^|[^[:alnum:]_])delay\(' "${SCAN[@]}" --exclude=hal_uno.cpp \
                 2>/dev/null | wc -l | tr -d ' ')" \
     "no unbounded blocking wait outside hal_uno.cpp's power-on settles"
   expect 0 "$(count '%[0-9.]*[fgeFGE]([^[:alnum:]]|$)' "${SCAN[@]}")" \
     "no float formatting anywhere (newlib float printf is the deepest stack consumer)"
   expect 0 "$(grep -rEn 'for[[:space:]]*\([[:space:]]*;[[:space:]]*;[[:space:]]*\)|while[[:space:]]*\([[:space:]]*(true|1)[[:space:]]*\)' \
                 "${SCAN[@]}" --exclude=safety.cpp 2>/dev/null | wc -l | tr -d ' ')" \
     "the program's only intentional unbounded loop is in the function that owns D6"
   # `malloc[[:space:]]*\(` -- the CALL, never the bare word. src/hal_uno.cpp writes
   # `#include <malloc.h>` for mallinfo(), which is how the heap diagnostics of ch200/ch201
   # exist at all; a bare-word pattern would match that include and fail this check forever.
   expect 0 "$(grep -rEn 'String|std::map|std::string|(^|[^[:alnum:]_])new([^[:alnum:]_]|$)|malloc[[:space:]]*\(' \
                 include src test lib/Manifold 2>/dev/null | wc -l | tr -d ' ')" \
     "no dynamic allocation outside lib/Network and lib/Screen"
   # Task 29 widens this ONE exclusion to sim_console.cpp, the device-only console shim the
   # sim binary needs, and records it as a deviation from §9's table in its own commit. It is
   # the only widening this line ever takes.
   expect 0 "$(grep -rEn 'Arduino\.h' include src test lib/Manifold --exclude=hal_uno.cpp \
                 2>/dev/null | wc -l | tr -d ' ')" \
     "the Arduino header lives only in hal_uno.cpp, lib/Network and lib/Screen"
   expect 0 "$(count 'WiFi\.ping' "${SCAN[@]}")" \
     "ping is never called (it resets the modem timeout to 10 s)"
   ```

5. - [ ] Add the two-binaries invariant — the one that keeps `pump 60000 prime hang` out of the binary that runs unattended:

   ```bash
   # ---- the two binaries: spec §6 ----
   # SCOPED TO src AND lib, not "${SCAN[@]}", and that is a deliberate deviation from §9's
   # table recorded in this task's commit message and again in task 20 step 13. Two files
   # outside src/ and lib/ name PB_BRINGUP on purpose and neither is a console affordance,
   # which is what this invariant exists to bound: include/config.h's uncalibrated-build
   # guard (task 20 step 13) tests !defined(PB_BRINGUP), and test/test_cli/test_cli.cpp
   # compiles BOTH arms of the bench-vs-bringup cases. Widening this back to include/ and
   # test/ makes make check fail on every task from 20 onward.
   expect 2 "$(files 'PB_BRINGUP' src lib)" \
     "PB_BRINGUP appears in src/cli.cpp and src/main.cpp, and in no other source file"
   ```

6. - [ ] Run it and read every line:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && make check
   ```
   expected: one `ok` line per invariant and `all invariants hold`, exit 0. If the `new`/`String`/`delay(` greps fire on a comment, reword the comment — see the hygiene note above — and do not weaken the pattern.

7. - [ ] Prove the check can actually fail, which is the only way to know it is wired up:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && printf '\nstatic void ui_break_the_check_(void) { delay(1); }\n' >> src/ui.cpp && make check; echo "exit=$?"
   ```
   expected:
   ```
   FAIL  no unbounded blocking wait outside hal_uno.cpp's power-on settles: expected 0, found 1
   1 invariant(s) FAILED
   exit=1
   ```
   Then revert:
   ```
   cd /Users/jcanton/projects/plant-butler/firmware && git checkout -- src/ui.cpp && make check
   ```

8. - [ ] Commit. **Drop 1 is complete: bring-up steps 0-3 can now be run on the bringup binary.**

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && git add tools/check.sh Makefile && git commit -m "make check: the drop-1 invariants, as greps

Every invariant in spec 9's table that is decidable on a drop-1 tree is now a grep: the
   four D6 rules, the D2/D3 rule scoped by awk to hal_arm_pulse_pins_'s body, the four
   watchdog rules, the blocking/allocation/formatting rules and the two-binaries rule. The
   ones that need drop-2 or drop-3 code are named in the file header so nobody reads the
   short list as the whole contract; task 30 adds them. No count is written down: spec 9's
   table is the authority, and a number in a comment goes stale the first time a task adds
   an invariant.

   The allocation grep matches malloc followed by an open parenthesis, not the bare word:
   hal_uno.cpp legitimately includes the C allocator header for mallinfo(), and a bare-word
   pattern would fail this check forever. Two greps are scoped more narrowly than spec 9's
   table prints them, and both are deviations rather than accidents. PB_PUMP_OWNER is counted
   by its DEFINITION, because pins.h necessarily tests it with #ifdef and a bare-token file
   count would be 2 forever. And PB_BRINGUP is counted over src and lib only: include/config.h
   gains a !defined(PB_BRINGUP) arm in the uncalibrated-build guard (task 20) and test_cli.cpp
   compiles both arms of the bench-vs-bringup cases, and neither is a console affordance.

   Patterns use POSIX classes rather than backslash-b, because BSD grep is the default on
   macOS. The greps scan comments too - that is deliberate, and the fix for a false hit is
   to reword the comment. Verified failing by adding a blocking wait to ui.cpp: exit 1, and
   it names the invariant."
   ```

---

---

### Task 14: `lib/Manifold` reworked — the cart that counts screw pulses

**Drop 2.**

**Files:**
- Create: `lib/Manifold/include/cart.h`, `lib/Manifold/src/cart.cpp`
- Delete: `lib/Manifold/include/Manifold.h`, `lib/Manifold/src/Manifold.cpp`
- Modify: `include/sim.h` and `src/hal_sim.cpp` (the screw and home-region model, `sim_set_stall`), `include/config.h` (the two servo drive microsecond constants — see the note below), `src/cli.cpp` (`status` gains the cart lines)
- Test: `test/test_cart/test_cart.cpp`

**Interfaces:**
- Consumes: `void hal_servo_us(uint16_t us)` (1500 == stop, 0 == detach) and `uint32_t hal_millis(void)` from task 3; `uint32_t pulses_screw(void)` from task 6; `bool sensors_home_hall(bool *home)` (false == bus error, never a value) from task 7 — and **not** `sensors_i2c_healthy()`: `cart_bus_check()` and `move_()` both take a live `sensors_home_hall()` read as the bus fact, and the health accessor is the dose ladder's, not the cart's; `void safety_tick(void)` and `void safety_wait_ms(uint32_t)` from task 5; `PB_PULSES_PER_GATE`, `PB_PULSES_HOME_TO_1`, `PB_MOVE_CAP_MS` (45000), `PB_STALL_WINDOW_MS` (2500), `PB_SERVO_CAP_MS` (10000), `PB_OUTLETS` (5) from task 2.
- Produces — `lib/Manifold/include/cart.h`, in full:

  ```c
  /* lib/Manifold/include/cart.h -- the cart, positioned by COUNTED SCREW PULSES plus the
     home hall. Every blocking wait in the old Manifold became a (target pulses, deadline,
     stall window) triple. This file includes no network header of any kind -- not the seam,
     not the library wrapper, not the modem driver -- and no panel pointer either: ui.cpp
     reads the cart; the cart writes to nothing. (task 14 step 11 greps this directory for
     all four of those names and expects zero, so do not spell one here.) */
  #pragma once
  #include <stdbool.h>
  #include <stdint.h>

  bool        cart_begin(void);          /* servo stopped, position UNKNOWN. No movement. */
  bool        cart_home(void);           /* drive toward home until the hall asserts; bounded */
  bool        cart_goto(uint8_t outlet); /* 1..PB_OUTLETS, by pulses; bounded, stall-aborted */
  bool        cart_pos_known(void);
  uint8_t     cart_pos(void);            /* 0 == home/parked; 1..PB_OUTLETS == over that gate */
  bool        cart_busy(void);           /* true only inside a move; net_poll/ui_poll read it */
  bool        cart_parked(void);         /* at home, i.e. over NO gate -- rides out as ch208 */
  bool        cart_bus_check(void);      /* ONE live expander read; false on a bus error */
  uint32_t    cart_pulses(void);         /* screw pulses since the last successful home */
  const char *cart_err(void);            /* "none" | "stall" | "timeout" | "i2c" | "range" | "uncal" */
  void        cart_jog(int16_t us, uint32_t ms);   /* bounded console jog, <= PB_SERVO_CAP_MS */
  ```

  `Manifold::test()`, `Manifold::reset()`'s one-gate-width guess (about 80 s wrong from gate 5 — the threadless start of the screw is the real home, and the cart parks there only if driven back far enough) and the `Screen*` the manifold logged through are **DELETED, not replaced**. `cart_goto()` and `cart_pos_known()` are compiled out structurally when the pitch is unknown, exactly as spec §2.15 prints it. `cart_bus_check()` is one live expander read returning false on a bus error — task 17's dose loop calls it every `PB_POS_RECHECK_MS`. The servo is stopped on **every** exit path, timeouts and stalls included. `cart_home()` and the boot self-home run under **both** latches (spec §2.11; task 19 and task 26 pin it).

**Spec sections to read in full before starting:** §1's module table row for `lib/Manifold`; §2.9; §2.11; §2.15 in full; §7 (the cart constants); §13 step 6.

**Two numbers the spec does not give, and what this task does about them.** §7 fixes `PB_MOVE_CAP_MS`, `PB_STALL_WINDOW_MS`, `PB_SERVO_CAP_MS` and `PB_SCREW_MIN_GAP_US`, but **nothing anywhere gives the servo's drive microseconds** — the continuous-rotation servo needs a value each side of the 1500 µs stop point and the spec names none. Step 3 adds `PB_SERVO_FWD_US` and `PB_SERVO_REV_US` to `include/config.h` **marked in the file as NOT from the spec**, at 1600/1400 (a deliberate slow crawl, so a wrong sign is a slow wrong direction rather than a fast one), and bring-up 6 commits the real pair alongside `PB_PULSES_PER_GATE`. Say so in the commit message; do not let two invented numbers pass as measured ones. Which direction is "toward the gates" is likewise unknown until bring-up 6 turns the screw and watches: `PB_SERVO_FWD_US > 1500` is an assumption, and step 3's comment says the fix is to swap the two constants, never to add a sign flip somewhere in `cart.cpp`.

**Tests:** `test_position_is_unknown_after_boot_until_homed`, `test_goto_refuses_when_pulses_per_gate_is_zero`, `test_pos_is_never_ok_before_calibration`, `test_goto_counts_pulses_not_milliseconds`, `test_home_zeroes_the_count_only_when_the_hall_asserts`, `test_home_from_outlet_five_actually_reaches_home`, `test_home_that_times_out_leaves_position_unknown`, `test_stall_aborts_within_the_stall_window_and_loses_position`, `test_an_i2c_error_on_the_home_hall_is_unknown_not_not_home`, `test_goto_rejects_an_outlet_outside_one_to_five`, `test_servo_is_stopped_on_every_exit_path`, `test_move_deadline_holds_across_a_millis_rollover`. **These nine are now the largest single block of unwritten test code in this plan.**

**Deliverable:** `pio test -e native -f test_cart` and `pio test -e native_cal -f test_cart` both pass twelve cases — the second is task 1's `[env:native_cal]`, `extends = env:native` plus `-DPB_PULSES_PER_GATE=1450`, so **both arms of the `#if` are exercised**, which is the whole point of making the refusal structural. Two of the twelve are regressions on code that is being deleted: homing from outlet five actually reaches home, and `cart_goto` refuses rather than silently succeeding while the pitch is zero. The `WiFiS3|link\.h|Network\.h` grep over `lib/Manifold` is task 30's and does not exist in `tools/check.sh` yet; check it by hand in step 10.

---

1. - [ ] **Add the screw and home-region model to the fake, and its injectors.** `test_cart` cannot be written before the fake can turn a screw. In `include/sim.h`:

   ```c
   /* The screw: one D3 pulse every sim_set_screw_pulse_ms() of commanded rotation, counted
      up or down according to the servo microseconds last written through seam 1. The home
      REGION, not a point: the hall reads asserted anywhere in [lo, hi] pulses, which is what
      a magnet over a hall actually does and what makes "drive until home" terminate. */
   void sim_set_screw_pulse_ms(uint32_t ms);      /* 0 == the screw does not turn at all */
   void sim_set_home_region(uint32_t lo, uint32_t hi);
   void sim_set_cart_at(uint32_t pulses);         /* place the cart without moving it */
   void sim_set_stall(bool on);                   /* the screw stops; the servo keeps driving */
   uint16_t sim_servo_us(void);                   /* what cart.cpp last commanded */
   uint32_t sim_servo_stops(void);                /* count of writes of 1500 */

   /* The clock injector. It lands HERE, not in task 22, because step 9's rollover case is
      its first consumer and a case that has to reach eight tasks forward for its fixture is
      a case that gets written twice. Body: g_us = ms * 1000u; g_ms = ms; one bounded
      tick_models_() call; nothing else. Task 22 step 13 adds sim_set_heap_break() ONLY. */
   void sim_set_clock_ms(uint32_t ms);
   ```

   and, in the same edit, add `SIM_EV_SCREW` to `sim_ev_kind_t` — task 3 step 5's enum ends at
   `SIM_EV_ADC` and the emitter below raises this one, so it has to exist before the model
   compiles. **Append it, never insert it**: `pb_count()` and every case in the tree compare
   enumerators by name, but a reader diffing two versions of `sim.h` should not have to check
   whether the numbering moved.

   ```c
   typedef enum {
     SIM_EV_PIN_CFG, SIM_EV_PIN_MODE, SIM_EV_PUMP_WRITE, SIM_EV_WDT_FEED,
     SIM_EV_I2C_WRITE, SIM_EV_I2C_READ, SIM_EV_SERVO, SIM_EV_ADC,
     SIM_EV_SCREW            /* ADDED HERE: one per accepted screw pulse (task 14) */
   } sim_ev_kind_t;
   ```

   This step also **supersedes task 6 step 6's `sim_screw_hz_()` placeholder**, which returned a
   flat 20 Hz whenever the servo was off its stop point. Delete that function; the model below
   replaces it, and leaving both would give the screw two rates that disagree.

   and in `src/hal_sim.cpp`, drive the model from the fake's clock: on every `sim_advance()` /
   `hal_millis()` step, if the commanded microseconds differ from 1500 and `!g_stall` and
   `g_screw_pulse_ms != 0`, advance an internal position by one pulse per `g_screw_pulse_ms`
   in the commanded direction, raise a `SIM_EV_SCREW` event and increment the counter
   `pulses_screw()` reads. `hal_servo_us()` records the value, counts a stop when it is 1500,
   and logs `SIM_EV_SERVO`. The home hall is read through the expander, so
   `sensors_home_hall()` sees `pos >= lo && pos <= hi` — set the default region to
   `[0, 40]` in `sim_reset()`.

2. - [ ] **Write the first two failing cases**, which are the two the deleted code got wrong. Create `test/test_cart/test_cart.cpp`:

   ```cpp
   #include <unity.h>
   #include <string.h>
   #include "../support/harness.h"
   #include "cart.h"
   #include "config.h"
   #include "sim.h"

   void setUp(void)    { pb_test_setup(); }
   void tearDown(void) { pb_test_teardown(); }

   /* §2.15. With the pitch unknown the natural arithmetic finds the target already satisfied
      at home, returns true and sets g_pos = outlet; both position guards then pass and the
      pump dead-heads against a CLOSED manifold with pos=ok on the wire. A comment saying
      "0 means always refuse" is not a mechanism. */
   void test_goto_refuses_when_pulses_per_gate_is_zero(void) {
     TEST_ASSERT_TRUE(cart_begin());
   #if PB_PULSES_PER_GATE == 0
     TEST_ASSERT_FALSE(cart_goto(1));
     TEST_ASSERT_FALSE(cart_goto(3));
     TEST_ASSERT_EQUAL_STRING("uncal", cart_err());
     TEST_ASSERT_FALSE(cart_pos_known());
   #else
     TEST_IGNORE_MESSAGE("calibrated arm: see native_cal");
   #endif
   }

   /* The regression on Manifold::reset(): it drove backwards for ONE gate-width and declared
      position 0, which from gate five is about 80 s short of the threadless start of the
      screw -- so the cart was left over gate four, holding it open under the reservoir head,
      while the firmware believed it was parked. */
   void test_home_from_outlet_five_actually_reaches_home(void) {
   #if PB_PULSES_PER_GATE == 0
     TEST_IGNORE_MESSAGE("uncalibrated arm: goto is compiled out");
   #else
     TEST_ASSERT_TRUE(cart_begin());
     sim_set_screw_pulse_ms(2);
     sim_set_home_region(0, 40);
     sim_set_cart_at(PB_PULSES_HOME_TO_1 + 4u * PB_PULSES_PER_GATE);   /* over gate five */
     TEST_ASSERT_TRUE(cart_home());
     TEST_ASSERT_TRUE(cart_parked());
     TEST_ASSERT_EQUAL_UINT32(0u, cart_pulses());
     TEST_ASSERT_EQUAL_UINT8(0u, cart_pos());
   #endif
   }

   int main(void) {
     UNITY_BEGIN();
     RUN_TEST(test_goto_refuses_when_pulses_per_gate_is_zero);
     RUN_TEST(test_home_from_outlet_five_actually_reaches_home);
     return UNITY_END();
   }
   ```

3. - [ ] **Run both arms and watch them fail to link.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cart; pio test -e native_cal -f test_cart
   ```
   expected: `fatal error: cart.h: No such file or directory` — the header does not exist yet.

   Then add the two servo constants to `include/config.h`, immediately under the cart block, and read the comment before you copy it:

   ```c
   /* NOT FROM THE SPEC. §7 gives the cart's cap, stall window and servo cap but no drive
      microseconds, and a continuous-rotation servo needs one value each side of the 1500 us
      stop point. 1600/1400 is a deliberate slow crawl: if the sign is wrong, bring-up 6 sees
      a slow wrong direction rather than a fast one. BRING-UP 6 COMMITS THE REAL PAIR, in the
      same commit as PB_PULSES_PER_GATE. If forward turns out to be the other way, SWAP THESE
      TWO VALUES -- do not add a sign flip inside cart.cpp, or the direction of travel stops
      being readable from one place. */
   #define PB_SERVO_FWD_US       1600   /* toward gate 1..5 */
   #define PB_SERVO_REV_US       1400   /* toward home */
   #define PB_SERVO_STOP_US      1500   /* hal_servo_us(1500) == stop (hal.h) */
   ```

4. - [ ] **Write `cart.cpp`'s state and its one move primitive.** Every bounded move in this file goes through `move_()`; there is no second loop that drives the servo.

   ```c
   /* lib/Manifold/src/cart.cpp -- position by counted screw pulses (§2.9, §2.15).
      Includes no network header of any kind -- not the seam, not the library wrapper, not
      the modem driver: this file CANNOT make a network call, and tools/check.sh proves it
      rather than this comment. */
   #include "cart.h"
   #include "config.h"
   #include "hal.h"
   #include "pulses.h"
   #include "safety.h"
   #include "sensors.h"

   static uint32_t    g_pulses;        /* screw pulses since the last successful home */
   static bool        g_home_seen;     /* a home hall assertion has been observed since boot */
   static uint8_t     g_pos;           /* 0 == home; 1..PB_OUTLETS == over that gate */
   static bool        g_pos_valid;
   static bool        g_busy;
   static const char *g_err = "none";

   bool        cart_busy(void)   { return g_busy; }
   uint32_t    cart_pulses(void) { return g_pulses; }
   const char *cart_err(void)    { return g_err; }
   uint8_t     cart_pos(void)    { return g_pos; }
   bool        cart_parked(void) { return g_home_seen && g_pos_valid && g_pos == 0u; }

   bool cart_begin(void) {
     hal_servo_us(PB_SERVO_STOP_US);   /* the servo is stopped before anything else is true */
     g_pulses = 0u; g_home_seen = false; g_pos = 0u; g_pos_valid = false;
     g_busy = false; g_err = "none";
     return true;                      /* NO MOVEMENT AT BOOT: the self-home is exec.cpp's,
                                          PB_BOOT_HOME_MS after reset (§3) */
   }

   bool cart_bus_check(void) {         /* ONE live expander read; false is a bus error */
     bool home = false;
     return sensors_home_hall(&home);
   }

   /* The one move primitive. A (deadline, stall window) pair, never a blocking wait, and
      safety_tick() on EVERY pass -- that is what keeps a 45 s traverse legal under a 5592 ms
      watchdog grant (§3). `stop_at` is a pulse count measured on pulses_screw(); `home_stops`
      makes the home hall a second terminating condition. Returns true only if a terminating
      condition was MET; every exit stops the servo. */
   static bool move_(uint16_t us, uint32_t stop_at_delta, bool home_stops) {
     uint32_t t0        = hal_millis();
     uint32_t s0        = pulses_screw();
     uint32_t last_edge = t0;
     uint32_t last_seen = s0;
     uint32_t now       = t0;
     bool     ok        = false;

     /* The move cap IS the loop condition, so this loop is bounded in its own head. §9's
        unbounded-loop grep pins the program's ONE deliberate never-terminating loop to
        dose_run() in safety.cpp -- the bring-up 7c hang -- and excludes no other file; the
        cart is not that loop and may not be written as though it were. `timeout` is armed
        BEFORE the loop so that falling out of the head carries the same reason the old
        in-body deadline check set, and a terminating condition clears it below. */
     g_err  = "timeout";
     g_busy = true;
     hal_servo_us(us);                      /* THE ONE SERVO START */
     while ((now - t0) < PB_MOVE_CAP_MS) {  /* unsigned: holds across a millis() rollover */
       safety_tick();                       /* pump idle re-asserted, then the dog fed */
       now           = hal_millis();
       uint32_t seen = pulses_screw();
       if (seen != last_seen) { last_seen = seen; last_edge = now; }

       if (home_stops) {
         bool home = false;
         if (!sensors_home_hall(&home)) {   /* a bus error is UNKNOWN, never "not home": */
           g_err = "i2c";                   /* driving on blind is what "not home" would do */
           break;
         }
         if (home) { ok = true; break; }
       }
       if (stop_at_delta != 0u && (seen - s0) >= stop_at_delta) { ok = true; break; }
       /* unsigned differences throughout: both bounds hold across a millis() rollover */
       if ((now - last_edge) >= PB_STALL_WINDOW_MS) { g_err = "stall"; break; }
     }
     hal_servo_us(PB_SERVO_STOP_US);        /* THE ONE SERVO STOP. Every exit path. */
     g_busy = false;
     if (ok) g_err = "none";
     return ok;
   }
   ```

5. - [ ] **Write `cart_home()` and `cart_jog()`.** Homing is not watering: it runs under the dry latch and under the contradiction latch alike (spec §2.11), so **there is no latch check in this file** — a reader who adds one leaves the cart holding gate N open under the reservoir head for as long as the latch stands, which after a mid-dose watchdog reset (which latches dry, §2.3) is until a human types `dry off`.

   ```c
   bool cart_home(void) {
     g_err = "none";
     g_pos_valid = false;                 /* unknown WHILE moving, so a failure leaves it so */
     if (!move_(PB_SERVO_REV_US, 0u, true)) return false;   /* g_err set by move_ */
     g_pulses = 0u;                       /* zeroed ONLY because the hall actually asserted */
     g_home_seen = true;
     g_pos = 0u;
     g_pos_valid = true;
     return true;
   }

   void cart_jog(int16_t us, uint32_t ms) {          /* console only; bounded (§6) */
     if (ms > PB_SERVO_CAP_MS) ms = PB_SERVO_CAP_MS; /* a typo may not run the screw forever */
     g_pos_valid = false;                            /* an untracked move loses the position */
     g_busy = true;
     hal_servo_us((uint16_t)us);
     safety_wait_ms(ms);                             /* fed on every iteration (task 5) */
     hal_servo_us(PB_SERVO_STOP_US);
     g_busy = false;
   }
   ```

6. - [ ] **Write `cart_goto()` and `cart_pos_known()` — both arms of the `#if`, spec §2.15 verbatim in shape.**

   ```c
   #if PB_PULSES_PER_GATE == 0
   /* §2.15: the refusal is STRUCTURAL. With the pitch unknown, the arithmetic below would
      find the target already satisfied at home and report pos=ok while the pump dead-heads
      against a closed manifold. Bring-up 6 measures the pitch and deletes this arm. */
   bool cart_goto(uint8_t outlet) { (void)outlet; g_err = "uncal"; return false; }
   bool cart_pos_known(void)      { return false; }
   #else
   bool cart_goto(uint8_t outlet) {
     g_err = "none";
     if (outlet < 1u || outlet > PB_OUTLETS) { g_err = "range"; return false; }
     if (!g_home_seen && !cart_home()) return false;      /* position must start from home */
     uint32_t want = (uint32_t)PB_PULSES_HOME_TO_1 +
                     (uint32_t)(outlet - 1u) * (uint32_t)PB_PULSES_PER_GATE;
     uint32_t have = g_pulses;
     if (want == have) { g_pos = outlet; g_pos_valid = true; return true; }
     bool     fwd   = want > have;
     uint32_t delta = fwd ? (want - have) : (have - want);
     g_pos_valid = false;                                  /* unknown until the move lands */
     uint32_t s0 = pulses_screw();
     bool ok = move_(fwd ? PB_SERVO_FWD_US : PB_SERVO_REV_US, delta, false);
     uint32_t moved = pulses_screw() - s0;                 /* what actually happened, not what
                                                              was asked for: a stall stops here */
     g_pulses = fwd ? (have + moved) : (have >= moved ? have - moved : 0u);
     if (!ok) { g_home_seen = false; return false; }       /* a stall LOSES the position */
     g_pos = outlet;
     g_pos_valid = true;
     return true;
   }
   bool cart_pos_known(void) { return g_home_seen && g_pos_valid; }
   #endif
   ```

7. - [ ] **Run both arms and watch the two cases pass.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cart && pio test -e native_cal -f test_cart
   ```
   expected: `2 Tests 0 Failures` in each, with one `IGNORE` in each — the arm that does not apply.

8. - [ ] **Delete the old manifold, and commit the two regressions with it.** Nothing includes `Manifold.h` any more: task 12's `main.cpp` was written against `cart.h`.

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && git rm lib/Manifold/include/Manifold.h lib/Manifold/src/Manifold.cpp && grep -rn 'Manifold' src include lib test --include=*.cpp --include=*.h
   ```
   expected: no hits at all. Then:

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native && pio run -e uno_r4_wifi && git add lib/Manifold include/config.h include/sim.h src/hal_sim.cpp test/test_cart/test_cart.cpp && git commit -m "cart: position by counted screw pulses, and the old Manifold deleted

   Manifold::reset() drove backwards for one gate-width and declared position 0. From gate
   five that is about 80 s short of the threadless start of the screw, which is the real
   home: the cart was left over gate four, holding that gate open under the reservoir head,
   while the firmware believed it was parked. test_home_from_outlet_five_actually_reaches_home
   is that bug as a test. Manifold::test(), its minutes of blocking delays and the Screen*
   it logged through are deleted, not replaced.

   cart_goto() and cart_pos_known() are COMPILED OUT to hard false while PB_PULSES_PER_GATE
   is 0 (spec 2.15). The arithmetic would otherwise find the target already satisfied at
   home, return true and report pos=ok while the pump dead-heads against a closed manifold.
   The suite is compiled twice - native and native_cal - so both arms are exercised.

   PB_SERVO_FWD_US and PB_SERVO_REV_US are NOT from the spec: 7 gives no servo drive
   microseconds. 1600/1400 is a slow crawl so that a wrong sign shows as a slow wrong
   direction; bring-up 6 commits the real pair with PB_PULSES_PER_GATE, and fixes a wrong
   direction by SWAPPING the two constants, never by a sign flip inside cart.cpp."
   ```

9. - [ ] **Write the remaining ten cases**, two at a time, each run before and after. Six of them need a word first, because each is a property that is easy to implement past:

   - `test_position_is_unknown_after_boot_until_homed` — `cart_begin()` then `TEST_ASSERT_FALSE(cart_pos_known())`, and the fake asserts the servo never left 1500: **`cart_begin()` does not move the cart.** The self-home is `exec_pending()`'s, `PB_BOOT_HOME_MS` after reset.
   - `test_pos_is_never_ok_before_calibration` — under plain `native` (pitch 0), drive a successful `cart_home()` and assert `cart_pos_known()` is still false. Seeing the home hall is not the same fact as being able to deliver to an outlet.
   - `test_goto_counts_pulses_not_milliseconds` — run the same `cart_goto(2)` twice with `sim_set_screw_pulse_ms(2)` and then `sim_set_screw_pulse_ms(8)`, and assert `cart_pulses()` is identical and the elapsed `hal_millis()` is not. That is the whole difference between this file and the one it replaces.
   - `test_home_zeroes_the_count_only_when_the_hall_asserts` — with `sim_set_home_region(0,0)` moved out of reach (`sim_set_home_region(9000, 9001)`), `cart_home()` returns false, `cart_pulses()` is **not** zeroed and `cart_pos_known()` is false.
   - `test_an_i2c_error_on_the_home_hall_is_unknown_not_not_home` — `sim_set_i2c_fail(true)` mid-home; assert `cart_home()` is false, `cart_err()` is `"i2c"` and `cart_pos_known()` is false. An unreadable hall must not read as "not home", which would drive the cart blind into the end of the screw.
   - `test_move_deadline_holds_across_a_millis_rollover` — `pb_test_setup()` then `sim_set_clock_ms(0xFFFFF000)` (step 1's injector, declared and defined in this task) before `cart_home()` with the home region out of reach; the move must still end at `PB_MOVE_CAP_MS`, proving the unsigned differences in `move_()`.

   The three the Tests list names and the paragraphs above do not, each with its setup and the
   exact assertion, so the step can be checked off from what is written:

   - `test_home_that_times_out_leaves_position_unknown` — `cart_begin(); sim_set_screw_pulse_ms(2); sim_set_home_region(9000u, 9001u); sim_set_cart_at(600u);` then `TEST_ASSERT_FALSE(cart_home()); TEST_ASSERT_EQUAL_STRING("timeout", cart_err()); TEST_ASSERT_FALSE(cart_pos_known()); TEST_ASSERT_FALSE(cart_parked());`. A traverse that never saw the hall has not found home, and must not be allowed to claim it did.
   - `test_stall_aborts_within_the_stall_window_and_loses_position` — under `native_cal` only (`#if PB_PULSES_PER_GATE == 0` → `TEST_IGNORE_MESSAGE`): home successfully, then `sim_set_stall(true);` and `cart_goto(3)`. Assert `TEST_ASSERT_FALSE(cart_goto(3)); TEST_ASSERT_EQUAL_STRING("stall", cart_err()); TEST_ASSERT_FALSE(cart_pos_known());` and that the elapsed `hal_millis()` across the call is under `PB_STALL_WINDOW_MS + PB_MOVE_CAP_MS / 10` — the stall window, not the move cap, is what ends it.
   - `test_goto_rejects_an_outlet_outside_one_to_five` — under `native_cal` only: `cart_begin();` then `TEST_ASSERT_FALSE(cart_goto(0)); TEST_ASSERT_EQUAL_STRING("range", cart_err()); TEST_ASSERT_FALSE(cart_goto(6)); TEST_ASSERT_EQUAL_STRING("range", cart_err());` and `TEST_ASSERT_EQUAL_UINT32(0u, sim_servo_stops() - stops_before)` — a rejected outlet must not start the servo at all, so `outlet == 0` (which butler accepts, §4.5) cannot cost a traverse.

   `test_servo_is_stopped_on_every_exit_path` is written as a loop over the four exits — success, stall (`sim_set_stall(true)`), timeout (home region out of reach), and bus error (`sim_set_i2c_fail(true)`) — asserting `sim_servo_us() == PB_SERVO_STOP_US` after each and that `sim_servo_stops()` increased by one:

   ```cpp
   void test_servo_is_stopped_on_every_exit_path(void) {
     const char *names[] = { "ok", "stall", "timeout", "i2c" };
     for (unsigned k = 0; k < 4u; ++k) {
       pb_test_setup();
       TEST_ASSERT_TRUE(cart_begin());
       sim_set_screw_pulse_ms(2);
       sim_set_home_region(0, 40);
       sim_set_cart_at(600u);
       if (k == 1u) sim_set_stall(true);
       if (k == 2u) sim_set_home_region(9000u, 9001u);
       if (k == 3u) sim_set_i2c_fail(true);
       uint32_t stops = sim_servo_stops();
       (void)cart_home();
       TEST_ASSERT_EQUAL_UINT16_MESSAGE(PB_SERVO_STOP_US, sim_servo_us(), names[k]);
       TEST_ASSERT_EQUAL_UINT32_MESSAGE(stops + 1u, sim_servo_stops(), names[k]);
     }
   }
   ```

10. - [ ] **Add the cart's `status` lines**, in `cli_print_status()` in `src/cli.cpp`, replacing task 11's `cart=UNCALIBRATED` block with one that also reports the live state.

    **First add `#include "cart.h"` to `src/cli.cpp`'s TOP-LEVEL include block**, in
    alphabetical order beside `cli.h` and `config.h`. It does not go inside task 20's
    `#if PB_BRINGUP` block: the six calls below run in `cli_print_status()`, which both
    binaries carry, so a bringup-only include leaves `cart_pos_known()` and its five
    neighbours undeclared in `pio run -e uno_r4_wifi` and in `pio test -e native_bench
    -f test_cli` from this step onward. Task 20 step 3 relies on this include and does not
    repeat it.

    ```c
    #if PB_PULSES_PER_GATE == 0
      hal_serial_write("cart=UNCALIBRATED (PB_PULSES_PER_GATE=0) - goto refuses, pos never ok\n");
    #else
      cli_printf_u32("cart pulses_per_gate=%lu\n", (uint32_t)PB_PULSES_PER_GATE);
    #endif
      snprintf(b, sizeof b, "cart pos=%s%u pulses=%lu parked=%u busy=%u err=%s\n",
               cart_pos_known() ? "" : "?", (unsigned)cart_pos(),
               (unsigned long)cart_pulses(), (unsigned)cart_parked(),
               (unsigned)cart_busy(), cart_err());
      hal_serial_write(b);
    ```

11. - [ ] **Run the whole gate, check the include grep by hand, and commit.** The `WiFiS3|link\.h|Network\.h` invariant over `lib/Manifold` is task 30's and is not in `tools/check.sh` yet, so it is a hand-check today:

    ```bash
    cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native && pio test -e native_cal -f test_cart && \
      grep -rEc 'WiFiS3|link\.h|Network\.h|Screen' lib/Manifold ; make check && pio run -e uno_r4_wifi -e uno_r4_wifi_bringup 2>&1 | tail -3
    ```
    expected: `0 Failures` from both suites, `0` from the grep over each file in `lib/Manifold`, `all invariants hold`, two `SUCCESS` lines.

    ```bash
    cd /Users/jcanton/projects/plant-butler/firmware && git add lib/Manifold src/cli.cpp test/test_cart/test_cart.cpp && git commit -m "cart: homing, goto, the stall window and the deadline, with status reporting all of it

    Every bound is a (target pulses, deadline, stall window) triple and every loop calls
    safety_tick(), which is what makes a 45 s traverse legal under a 5592 ms grant. The
    servo is stopped on all four exit paths - success, stall, timeout and a bus error - and
    the test loops over them rather than asserting the happy one.

    An unreadable home hall is UNKNOWN, never 'not home': treating a bus error as not-home
    is what drives the cart blind into the end of the screw. It aborts with err=i2c and
    leaves the position unknown.

    There is deliberately no latch check in this file. Homing is not watering: it runs under
    dry on and under a contradiction latch alike (spec 2.11), because parking the cart off
    every gate is MORE wanted after a latch, not less - a cart left over outlet N holds that
    gate open under the reservoir head for as long as the latch stands."
    ```

12. - [ ] **What this task does not prove, and which bring-up step does.** No host test here touches a real screw, real reduction gears or a real hall. **Bring-up 6** (spec §13) is what proves the cart: `dry on` first, then `servo`, `home` and `goto` against the rig, then the same with WiFi connected and the cart deliberately stalled — and it is bring-up 6 that **produces `PB_PULSES_PER_GATE` and `PB_PULSES_HOME_TO_1`** and commits them, along with the real `PB_SERVO_FWD_US`/`PB_SERVO_REV_US`. Until it lands, `cart_goto()` is compiled to `return false`, `pos` is never `ok`, and the fake's screw is the only screw this code has ever turned.

---

---

### Task 15: The float debounce, the dry latch and the float-flap counter

**Drop 2.**

**Files:**
- Modify: `include/safety.h`, `src/safety.cpp`, `src/cli.cpp` (`dry on|off` in the dispatcher, and `status` gains `dry=`), `include/sim.h` and `src/hal_sim.cpp` (`sim_set_float_pattern`), `test/test_dose/test_dose.cpp`, `test/test_cli/test_cli.cpp` (`dry on` / `dry off` join task 11's bench-command case)
- Test: `test/test_dose/test_dose.cpp`

**Interfaces:**
- Consumes: `int hal_pin_read(uint8_t)` and `PIN_HALL_FLOAT` from tasks 2-3; `void safety_wait_ms(uint32_t)` from task 5 (it calls `safety_tick()` on every iteration, so the wait is fed); `pb_noinit_t g_nv` and `void noinit_commit(void)` from task 4; `PB_FLOAT_OK_SAMPLES` (3), `PB_FLOAT_SAMPLE_MS` (20), `PB_FLOAT_FLAP_LIMIT` (3) from task 2.
- Produces — `include/safety.h` gains:

  ```c
  bool safety_float_ok_debounced(void);          /* N consecutive OK to GRANT (§2.10) */
  void safety_dry_set(bool on);                  /* the operator's `dry on|off`; .noinit-backed */
  bool safety_dry(void);
  bool safety_float_flap(void);                  /* >= PB_FLOAT_FLAP_LIMIT consecutive float
                                                    refusals; task 22 puts it on the wire */
  void safety_float_refusal_count(bool refused_for_float);
  ```

  `safety_float_ok_debounced()` is exactly the loop spec §2.10 prints: `PB_FLOAT_OK_SAMPLES` consecutive OK readings to GRANT, **one bad sample REFUSES immediately**, `safety_wait_ms(PB_FLOAT_SAMPLE_MS)` between samples so the wait is fed. The asymmetry is the point — refusing on one bad sample is safe, granting on one is not, because D5 runs up to a metre to the reservoir alongside a 12 V pump lead. `safety_dry_set()` writes `g_nv.dry_latched` and calls `noinit_commit()`, so the latch survives a warm reset — the case that mattered, because a brown-out at pump start used to silently clear it while the operator's hands were in the plumbing. `safety_float_flap()` is task 22's source for `float=0` on the wire and is cleared by any **granted** dose.

**`safety_float_refusal_count()` has exactly two call sites, and this task fixes their contract.** It is called `safety_float_refusal_count(true)` from `dose_end_()` **only on the `DOSE_REFUSED_FLOAT` arm**, and `safety_float_refusal_count(false)` from `dose_end_ml_()` **unconditionally** — because spec §2.10 says the counter is cleared "by any granted dose", and `dose_end_ml_()` is the function only a granted dose reaches. Refusals for cooldown, i2c, position and the rest must leave the counter alone: a rig refusing for a stalled cart must not quietly forget that the float has been flapping for an hour. Task 17 owns both call sites; write them there in exactly that shape.

**Spec sections to read in full before starting:** §2.10 in full; §2.11; §7's float block; §13 steps 5a and 6.

**Tests:** `test_three_consecutive_ok_samples_are_needed_to_grant`, `test_one_bad_sample_refuses_immediately`, `test_the_float_debounce_feeds_the_watchdog_between_samples`, `test_the_dry_latch_survives_a_warm_reset_and_not_a_cold_one`, `test_the_flap_counter_trips_after_three_consecutive_float_refusals`.

**Deliverable:** `pio test -e native -f test_dose` passes five cases driven through `sim_set_float()` / `sim_set_float_pattern()` and `sim_reset(warm|cold)`. The debounce is what stops the specific, expensive lie spec §2.10 describes: a float bouncing at the waterline satisfies one sample and fails three, so a raw-sample implementation reports `float=1`, butler queues, the board refuses, the acked refusal pages **HIGH** and sets the pot's cooldown, and the pattern repeats every cooldown period forever, with every field the phone-notification design rests on saying the rig is fine.

**One wording conflict, resolved and recorded.** Spec §2.10 says the report forces `float=0` "**above** `PB_FLOAT_FLAP_LIMIT` (3)" consecutive refusals, while §9's own test name is `test_the_flap_counter_trips_after_three_consecutive_float_refusals`. The test name wins: `safety_float_flap()` is `g_float_refusals >= PB_FLOAT_FLAP_LIMIT`, so the third consecutive refusal trips it. Say so in the commit message rather than leaving a reader to find the two sentences disagreeing.

---

1. - [ ] **Give the fake a per-sample float, so a bouncing float can actually be tested.** A single `sim_set_float(bool)` cannot express "OK once, bad twice", which is the whole failure mode. In `include/sim.h`:

   ```c
   /* Each character is consumed by ONE hal_pin_read(PIN_HALL_FLOAT): '1' == OK (the pin reads
      PB_LOW), '0' == not OK. The last character repeats forever, so "100" is a float that
      grants one sample and fails the next two -- exactly the waterline bounce of §2.10.
      sim_set_float() clears any pattern. */
   void sim_set_float_pattern(const char *pattern);
   ```

   and in `src/hal_sim.cpp`, hold the pattern in a `static char g_float_pat[16]` with an index
   that advances on each read of `PIN_HALL_FLOAT` and sticks at the last character.
   `sim_reset()` clears it and leaves the float OK.

2. - [ ] **Write the three debounce cases**, in `test/test_dose/test_dose.cpp`, with their `RUN_TEST` lines:

   ```cpp
   /* §2.10. Three consecutive OK samples to GRANT. The float bouncing at the waterline
      satisfies one sample and fails three; a raw-sample implementation reports float=1, the
      backend queues, the board refuses, the acked refusal pages HIGH and sets the pot's
      cooldown - and repeats every cooldown period, forever. */
   void test_three_consecutive_ok_samples_are_needed_to_grant(void) {
     pb_test_setup();
     sim_set_float_pattern("1101111");        /* one bad sample inside the first window */
     TEST_ASSERT_FALSE(safety_float_ok_debounced());
     pb_test_setup();
     sim_set_float_pattern("111");
     TEST_ASSERT_TRUE(safety_float_ok_debounced());
   }

   /* The asymmetry is deliberate: refusing on one bad sample is safe, GRANTING on one is
      not, because D5 runs up to a metre to the reservoir beside a 12 V pump lead. */
   void test_one_bad_sample_refuses_immediately(void) {
     pb_test_setup();
     sim_set_float_pattern("0111111");
     uint32_t t0 = hal_millis();
     TEST_ASSERT_FALSE(safety_float_ok_debounced());
     /* it returned on the FIRST sample: no PB_FLOAT_SAMPLE_MS wait was paid */
     TEST_ASSERT_LESS_THAN_UINT32(PB_FLOAT_SAMPLE_MS, hal_millis() - t0);
   }

   /* Every loop that can iterate over a millisecond of wall clock feeds the dog (§3). This
      one waits 2 x PB_FLOAT_SAMPLE_MS and is called from dose_run()'s refusal ladder. */
   void test_the_float_debounce_feeds_the_watchdog_between_samples(void) {
     pb_test_setup();
     sim_set_float(true);
     sim_events_clear();
     TEST_ASSERT_TRUE(safety_float_ok_debounced());
     const sim_ev_t *ev; size_t n = sim_events(&ev);
     uint32_t feeds = 0, prev = 0; bool first = true;
     for (size_t i = 0; i < n; ++i) {
       if (ev[i].kind != SIM_EV_WDT_FEED) continue;
       if (!first) TEST_ASSERT_TRUE(ev[i].at_ms - prev <= 3u);
       prev = ev[i].at_ms; first = false; feeds++;
     }
     TEST_ASSERT_TRUE(feeds >= 2u);
   }
   ```

3. - [ ] **Run them and watch them fail to link.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose
   ```
   expected: `undefined reference to 'safety_float_ok_debounced()'`.

4. - [ ] **Write the debounce, spec §2.10 verbatim in shape**, into `src/safety.cpp`:

   ```c
   /* §2.10. N consecutive OK to GRANT; ONE bad sample refuses immediately. The wait between
      samples is safety_wait_ms(), which calls safety_tick() on every iteration -- so the pump
      is idle-re-asserted and the dog fed throughout. */
   bool safety_float_ok_debounced(void) {
     for (uint8_t i = 0; i < PB_FLOAT_OK_SAMPLES; ++i) {
       if (hal_pin_read(PIN_HALL_FLOAT) != PB_LOW) return false;
       if (i + 1u < PB_FLOAT_OK_SAMPLES) safety_wait_ms(PB_FLOAT_SAMPLE_MS);
     }
     return true;
   }
   ```

5. - [ ] **Run and watch the three pass, then commit the debounce on its own.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose && git add include/safety.h src/safety.cpp include/sim.h src/hal_sim.cpp test/test_dose/test_dose.cpp && git commit -m "safety: the float debounce is asymmetric, and the fake can bounce a float

   Three consecutive OK samples to grant; one bad sample refuses immediately. Refusing on
   one sample is safe and granting on one is not: D5 runs up to a metre to the reservoir
   beside a 12 V pump lead. The wait between samples is safety_wait_ms(), so the dog is fed
   through it.

   sim_set_float_pattern() exists because a single boolean cannot express the failure this
   guards: a float bouncing at the waterline satisfies one sample and fails three, and the
   raw-sample version of this function reports float=1 to a backend that then queues a dose
   the board refuses - acked with flow_ml=0, paging HIGH and setting the pot's cooldown,
   every cooldown period, forever."
   ```

6. - [ ] **Write the dry-latch case. The latch surviving a warm reset is the point of the whole `.noinit` block.**

   ```cpp
   /* §2.11. `dry on` survives a WARM reset - watchdog or RESET button - because a brown-out
      at pump start (the wiring README warns of 3-5x inrush on a sagging brick) used to clear
      it silently while the operator's hands were in the plumbing. It does NOT survive a cold
      boot, and it must not: a power cycle starts clean and PB_BOOT_GAP_MS refuses for the
      first 10 s anyway. */
   void test_the_dry_latch_survives_a_warm_reset_and_not_a_cold_one(void) {
     pb_test_setup();
     TEST_ASSERT_FALSE(safety_dry());
     safety_dry_set(true);
     TEST_ASSERT_TRUE(safety_dry());

     /* sim_reset() re-enters the boot path (task 3), so noinit_begin() has already run by
        the time it returns -- do not call it a second time or the boot counter, and with it
        the salt, advances twice per reset. */
     sim_reset(true);                     /* warm: SRAM intact, .noinit verifies */
     TEST_ASSERT_TRUE_MESSAGE(safety_dry(), "the latch did not survive a warm reset");

     sim_reset(false);                    /* cold: SRAM cleared, magic mismatches */
     TEST_ASSERT_FALSE_MESSAGE(safety_dry(), "the latch survived a COLD boot");
   }
   ```

   Run it, watch it fail, then write the latch:

   ```c
   void safety_dry_set(bool on) {
     g_nv.dry_latched = on;
     noinit_commit();      /* the checksum is recomputed on EVERY write (§2.3) */
   }
   bool safety_dry(void) { return g_nv.dry_latched; }
   ```

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose
   ```

7. - [ ] **Write the flap-counter case**, which is the one that keeps the acked-refusal loop from coming back by a different door — the contradiction latch does **not** cover it, because a *refused* dose never reaches `dose_end_ml_()`:

   ```cpp
   /* §2.10's second consequence. The report's debounce and the dose's debounce are separate
      samples taken minutes apart, so a float flapping at the waterline can grant in the
      report and refuse in the dose. Above the limit the report forces float=0 and err=float
      regardless of the report-time debounce (task 22), and butler's rules ladder goes dark. */
   void test_the_flap_counter_trips_after_three_consecutive_float_refusals(void) {
     pb_test_setup();
     TEST_ASSERT_FALSE(safety_float_flap());
     safety_float_refusal_count(true);
     safety_float_refusal_count(true);
     TEST_ASSERT_FALSE(safety_float_flap());          /* two is not yet a pattern */
     safety_float_refusal_count(true);
     TEST_ASSERT_TRUE(safety_float_flap());           /* the third trips it */
     safety_float_refusal_count(false);               /* a GRANTED dose clears it */
     TEST_ASSERT_FALSE(safety_float_flap());
   }
   ```

   and the implementation:

   ```c
   static uint8_t g_float_refusals;

   /* TWO call sites, both in task 17, and the argument is not "was this dose refused":
        dose_end_()    calls this with true ONLY on the DOSE_REFUSED_FLOAT arm;
        dose_end_ml_() calls it with false unconditionally, because §2.10 says the counter is
                       cleared by any GRANTED dose and dose_end_ml_() is the function only a
                       granted dose reaches.
      A refusal for cooldown, i2c or position must leave the counter ALONE: a rig refusing
      for a stalled cart must not quietly forget that the float has been flapping for an
      hour. */
   void safety_float_refusal_count(bool refused_for_float) {
     if (refused_for_float) { if (g_float_refusals < 255u) g_float_refusals++; }
     else                     g_float_refusals = 0u;
   }
   bool safety_float_flap(void) { return g_float_refusals >= PB_FLOAT_FLAP_LIMIT; }
   ```

8. - [ ] **Add `dry on|off` to the dispatcher — in BOTH builds, outside any `#if PB_BRINGUP`.** Bring-up 6 types it before touching the plumbing, and the unattended binary must be able to be made safe over the same wire. In `src/cli.cpp`, in `cli_dispatch()`, above the unknown-command fallthrough:

   ```c
     if (strcmp(line, "dry on") == 0) {
       safety_dry_set(true);
       hal_serial_write("dry=1 - every dose refused until `dry off`\n");
       return true;
     }
     if (strcmp(line, "dry off") == 0) {
       safety_dry_set(false);
       hal_serial_write("dry=0\n");
       return true;
     }
   ```

   and in `cli_print_status()`:

   ```c
     cli_printf_u32("dry=%lu\n", (uint32_t)(safety_dry() ? 1u : 0u));
   ```

   Then add both spellings to task 11's `test_parses_every_bench_command`, which is where spec §6's full bench command set is asserted:

   ```cpp
     TEST_ASSERT_TRUE(cli_dispatch("dry on"));
     TEST_ASSERT_TRUE(cli_dispatch("dry off"));
     TEST_ASSERT_FALSE(cli_dispatch("dry"));          /* no bare form, and no abbreviation */
   ```

   **And add `#include "safety.h"` to the top of `test/test_cli/test_cli.cpp`.** That file has
   carried `cart.h`, `config.h`, `ui.h`, `cli.h`, `hal.h`, `sim.h` and `../support/harness.h`
   since tasks 10 and 11; from this task on it also names `safety_dry()` (here),
   `cli_stop_requested()` (task 16, through `cli.h`), `dose_req_t` and `cfg_pulses_per_l_get()`
   (task 20). One include, added once, rather than four tasks each discovering the same
   missing declaration.

9. - [ ] **Run the whole gate and commit.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native && pio test -e native_bench -f test_cli && make check && pio run -e uno_r4_wifi -e uno_r4_wifi_bringup 2>&1 | tail -3
   ```
   expected: `0 Failures`, `all invariants hold`, two `SUCCESS` lines. `native_bench` matters here: `dry on|off` must exist in the **bench** binary too, so a case that only ran under `PB_BRINGUP` would prove nothing.

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && git add include/safety.h src/safety.cpp src/cli.cpp test/test_dose/test_dose.cpp test/test_cli/test_cli.cpp && git commit -m "safety: the dry latch survives a warm reset, and the float-flap counter

   dry on|off writes .noinit and recomputes the checksum, so the latch survives a watchdog
   or RESET-button reset. That is the case that mattered: a brown-out at pump start used to
   clear it silently while the operator's hands were in the plumbing. It does not survive a
   power cycle - nothing in .noinit does - and PB_BOOT_GAP_MS is what covers the first 10 s
   after one.

   The flap counter closes the acked-refusal loop by its other door. The report's debounce
   and the dose's debounce are separate samples minutes apart, so a float flapping at the
   waterline can grant in the report and refuse in the dose; the contradiction latch cannot
   see it, because a refused dose never reaches dose_end_ml_(). Three consecutive
   DOSE_REFUSED_FLOAT results force float=0 on the wire (task 22) and butler's rules ladder
   goes dark.

   Spec 2.10 says 'above PB_FLOAT_FLAP_LIMIT' while 9's own test name says 'after three'.
   The test name wins: the predicate is >= PB_FLOAT_FLAP_LIMIT, so the third refusal trips
   it.

   safety_float_refusal_count() has exactly two call sites, both in dose_run()'s two exit
   helpers, and the false arm belongs to dose_end_ml_() alone - the counter is cleared by a
   GRANTED dose, never by a refusal for some unrelated reason."
   ```

10. - [ ] **What this task does not prove.** The host tests drive a fake pin. **Bring-up 5a** (spec §13) is what proves the debounce on the rig: with the float below the line, `pump 2000` must be refused and print `DOSE_REFUSED_FLOAT` by name — and three consecutive OK samples are needed to grant, so a float that is merely *near* the line does not authorise a dose. **Bring-up 6** types `dry on` before the 12 V brick goes on and confirms `status` says `dry=1`. Neither is simulated here, and a green `pio test -e native` is not evidence for either.

---

---

### Task 16: `cli_stop_requested()` — the byte-wise mid-dose abort matcher

**Drop 2.**

**Files:**
- Modify: `include/cli.h`, `src/cli.cpp` (the matcher, the pushback buffer, `cli_poll()`'s reader, and **the `stop` command in the dispatcher**), `test/test_cli/test_cli.cpp`
- Test: `test/test_cli/test_cli.cpp`

**Interfaces:**
- Consumes: `size_t hal_serial_read(char *buf, size_t cap)` and `void hal_serial_write(const char *)` from task 3; `void safety_dry_set(bool)` from task 15; `PB_LINE_CAP` from task 2.
- Produces — `include/cli.h` gains:

  ```c
  bool cli_stop_requested(void);   /* the last-resort abort, called from dose_run()'s loop */
  void cli_stop_clear(void);       /* forget the request, the partial match AND the pushback */
  ```

  `cli_stop_requested()` is a byte-at-a-time matcher over its own state, with **its own `hal_serial_read()`**, that consumes ONLY the bytes of an exact `stop\n` or `dry on\n` and pushes every other byte into `cli.cpp`'s line buffer unread. `dry on` typed mid-dose sets `g_nv.dry_latched` **and** raises the stop request, so the word means the same thing during a dose as before one. Spec §2.12 left this function undefined at the seam while advertising it as the last-resort abort; it is specified here, and its host test drives it through the real `hal_serial_read` fake, byte by byte, **split across two reads** — never by poking a flag.

**Spec sections to read in full before starting:** §2.12 in full; §6's command table (the `stop` and `dry on|off` rows, both ticked in the `bench` column as well as `bringup`).

**Two things the spec's own numbers do not survive contact with, both settled here.**

- **§2.12 calls it "a four-byte state".** `dry on` is six bytes, and the same matcher must recognise it. The partial-match buffer below is **eight** bytes — the longer word plus room — and `PB_STOP_MATCH_CAP` names it in `cli.cpp`. Four bytes would silently truncate `dry on` into a permanent non-match.
- **The skeleton's deliverable and spec §2.8 disagree about what happens to pushed-back bytes at the end of a dose.** The deliverable says `status` typed during a dose should still run afterwards; §2.8 says `hal_serial_drain()` at the end of every dose discards the impatience "rather than executing it", and §15.3 is explicit that three impatient `pump 60000` lines must not become 180 seconds of pumping. **The spec wins.** The pushback exists so that the *matcher* is honest byte-for-byte — `sta` must leave three bytes for the line buffer, not two, and `stopp` must not abort anything — and task 17's end-of-dose sequence is `hal_serial_drain(); cli_stop_clear();`, which discards the UART ring **and** this file's pushback together. Record it in the commit message; a reader who finds only the deliverable sentence will otherwise reintroduce the queue.

**The defect this task also fixes.** `stop` was specified as a mid-dose matcher and never added to `cli_dispatch()`, so a `stop` typed at an idle console answered `? unknown; type help` — the one command an operator reaches for in an emergency, absent from the console that prints the help. Step 7 adds it, in **both** binaries.

**Tests:** `test_stop_is_matched_byte_by_byte_across_two_reads`, `test_a_non_matching_byte_is_pushed_to_the_line_buffer_unread`, `test_dry_on_mid_dose_raises_the_stop_request_and_sets_the_latch`, `test_a_near_miss_token_does_not_raise_the_stop_request`.

**Deliverable:** `pio test -e native -f test_cli` and `pio test -e native_bench -f test_cli` pass four cases, at least one of which feeds `st` on the first `hal_serial_read()` and `op\n` on the second. A non-matching prefix such as `sta` must leave **all three** bytes available to `cli_poll()`'s line buffer, so `status` typed at an idle console is not eaten by a matcher that ran first.

---

1. - [ ] **Write the case that fixes the shape: the match spans two reads.** A matcher that only works when the whole word arrives in one `hal_serial_read()` is a matcher that works on the bench and not on a real CDC line. Add to `test/test_cli/test_cli.cpp`, with its `RUN_TEST` line:

   ```cpp
   /* §2.12. The console's last-resort abort. dose_run() calls this once per loop iteration,
      so the word arrives in whatever fragments the UART hands over -- here `st` and `op\n`. */
   static void test_stop_is_matched_byte_by_byte_across_two_reads(void) {
     pb_test_setup();
     cli_stop_clear();
     sim_serial_rx("st");
     TEST_ASSERT_FALSE(cli_stop_requested());     /* half a word is not a stop */
     sim_serial_rx("op\n");
     TEST_ASSERT_TRUE(cli_stop_requested());
     TEST_ASSERT_TRUE(cli_stop_requested());      /* it LATCHES until cli_stop_clear() */
     cli_stop_clear();
     TEST_ASSERT_FALSE(cli_stop_requested());
   }
   ```

2. - [ ] **Run it and watch it fail to link.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cli
   ```
   expected: `undefined reference to 'cli_stop_requested()'`.

3. - [ ] **Write the pushback buffer and re-route `cli_poll()`'s reader through it.** This comes first because the matcher is only safe once the bytes it declines have somewhere to go. In `src/cli.cpp`, above `cli_poll()`:

   ```c
   /* Bytes cli_stop_requested() read from the UART and did NOT consume. cli_poll() drains
      this before it touches the UART, so the two readers cannot lose or reorder a byte
      between them. It is not a queue of commands: the dosing loop clears it at the end of
      every dose (§2.8, §15.3), so impatience typed during a dose is discarded, not
      executed. */
   static char     g_push[PB_LINE_CAP];
   static uint16_t g_push_len;

   static void push_back_(const char *b, size_t n) {
     for (size_t i = 0; i < n && g_push_len < sizeof g_push; ++i) g_push[g_push_len++] = b[i];
   }

   static size_t read_console_(char *buf, size_t cap) {
     if (g_push_len > 0u) {                       /* pushback FIRST, in arrival order */
       size_t n = g_push_len < cap ? g_push_len : cap;
       for (size_t i = 0; i < n; ++i) buf[i] = g_push[i];
       for (size_t i = n; i < g_push_len; ++i) g_push[i - n] = g_push[i];
       g_push_len = (uint16_t)(g_push_len - n);
       return n;
     }
     return hal_serial_read(buf, cap);
   }
   ```

   and in `cli_poll()`, replace the single line

   ```c
     size_t n = hal_serial_read(buf, sizeof buf);
   ```

   with

   ```c
     size_t n = read_console_(buf, sizeof buf);
   ```

   `hal_serial_read` now has exactly two call sites in this file: `read_console_()` and the
   matcher of step 4. Nothing else in the tree reads the console.

4. - [ ] **Write the matcher.** It is line-exact: a line that is not *entirely* `stop` or `dry on` is pushed back whole, in order, including its newline.

   ```c
   /* §2.12. Consumes ONLY an exact `stop\n` or `dry on\n`. Every other byte is pushed back
      unread, so `status` typed at the console is still `status` when cli_poll() reads it.

      §2.12 calls this "a four-byte state"; `dry on` is six bytes, so the partial buffer is
      eight. Four would truncate `dry on` into a permanent non-match. */
   #define PB_STOP_MATCH_CAP 8

   static char     g_pfx[PB_STOP_MATCH_CAP];
   static uint8_t  g_pfx_len;
   static bool     g_line_dirty;    /* this line already broke the match: push everything back */
   static bool     g_stop_req;

   static bool pfx_is_prefix_of_(const char *word) {
     for (uint8_t i = 0; i < g_pfx_len; ++i) if (word[i] == '\0' || word[i] != g_pfx[i]) return false;
     return true;
   }

   void cli_stop_clear(void) {
     g_stop_req = false;
     g_pfx_len = 0u;
     g_line_dirty = false;
     g_push_len = 0u;      /* forget every byte the console has seen and not acted on */
   }

   bool cli_stop_requested(void) {
     char b[16];
     size_t n = hal_serial_read(b, sizeof b);
     for (size_t i = 0; i < n; ++i) {
       char c = b[i];
       if (c == '\r') continue;
       if (c == '\n') {
         if (!g_line_dirty && g_pfx_len == 4u && pfx_is_prefix_of_("stop")) {
           g_stop_req = true;
         } else if (!g_line_dirty && g_pfx_len == 6u && pfx_is_prefix_of_("dry on")) {
           safety_dry_set(true);       /* the word means the same during a dose as before one */
           g_stop_req = true;
         } else {
           push_back_(g_pfx, g_pfx_len);
           push_back_("\n", 1u);       /* the line is handed on WHOLE, newline included */
         }
         g_pfx_len = 0u; g_line_dirty = false;
         continue;
       }
       if (g_line_dirty) { push_back_(&c, 1u); continue; }
       if (g_pfx_len < PB_STOP_MATCH_CAP) g_pfx[g_pfx_len++] = c;
       if (!pfx_is_prefix_of_("stop") && !pfx_is_prefix_of_("dry on")) {
         push_back_(g_pfx, g_pfx_len);   /* the whole prefix, in order: `sta` is three bytes */
         g_pfx_len = 0u;
         g_line_dirty = true;            /* and the rest of THIS line is not a match either */
       }
     }
     return g_stop_req;
   }
   ```

   `#include "safety.h"` at the top of `cli.cpp` if it is not already there — `cli.cpp` is the
   one file outside `safety.cpp` that spec §9's grep permits to name it, and task 20 will add
   the file's single `dose_run(` call site.

5. - [ ] **Run and watch the first case pass.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cli
   ```

6. - [ ] **Write the three remaining cases**, then run them.

   ```cpp
   /* The deliverable's own example. `sta` must leave THREE bytes for the line buffer: a
      matcher that swallowed `st` would turn `status` into `atus` -- an unknown command that
      looks like a console fault rather than a matcher bug. */
   static void test_a_non_matching_byte_is_pushed_to_the_line_buffer_unread(void) {
     pb_test_setup();
     cli_stop_clear();
     char out[512];
     (void)sim_serial_tx(out, sizeof out);
     sim_serial_rx("status\n");
     TEST_ASSERT_FALSE(cli_stop_requested());   /* not a stop, and not consumed either */
     cli_poll();                                /* reads the pushback FIRST */
     size_t n = sim_serial_tx(out, sizeof out); out[n] = '\0';
     TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, "granted="), out);   /* status actually ran */
   }

   /* §2.12: `dry on` typed mid-dose sets the latch AND raises the stop request, so the word
      means the same thing during a dose as before one. */
   static void test_dry_on_mid_dose_raises_the_stop_request_and_sets_the_latch(void) {
     pb_test_setup();
     cli_stop_clear();
     TEST_ASSERT_FALSE(safety_dry());
     sim_serial_rx("dry ");
     TEST_ASSERT_FALSE(cli_stop_requested());
     sim_serial_rx("on\n");
     TEST_ASSERT_TRUE(cli_stop_requested());
     TEST_ASSERT_TRUE(safety_dry());
   }

   /* Near misses. `sto` is short, `stopp` is long, `xstop` is not the line, and `dry off` is
      a different command that must NOT abort a dose - it clears a latch, it does not stop
      water. All four leave the request down and the bytes recoverable. */
   static void test_a_near_miss_token_does_not_raise_the_stop_request(void) {
     const char *misses[] = { "sto\n", "stopp\n", "xstop\n", "dry off\n" };
     for (unsigned i = 0; i < 4u; ++i) {
       pb_test_setup();
       cli_stop_clear();
       sim_serial_rx(misses[i]);
       TEST_ASSERT_FALSE_MESSAGE(cli_stop_requested(), misses[i]);
       TEST_ASSERT_FALSE_MESSAGE(safety_dry(), misses[i]);
     }
   }
   ```

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cli
   ```
   expected: `0 Failures`. If `test_a_near_miss_token_does_not_raise_the_stop_request` fails on
   `stopp`, the matcher is matching a **prefix** rather than a whole line — fix the matcher,
   never the case: `pump 60000` typed by an operator whose terminal echoes oddly must not be
   able to end with a token that reads as an abort of something else.

7. - [ ] **Add `stop` to the dispatcher — in BOTH binaries.** This is the defect named at the top: `stop` existed only as a mid-dose matcher, so a `stop` typed at an idle console answered `? unknown; type help`. In `cli_dispatch()`, above the unknown-command fallthrough:

   ```c
     if (strcmp(line, "stop") == 0) {
       /* A dose in progress never reaches here: the dosing loop blocks and matches this
          word byte-wise itself (§2.12). This arm is the idle console's answer, and it exists
          so that `stop` is never `? unknown` -- it is the one command an operator reaches for
          in an emergency. It also clears a stale request left by a matched-but-unconsumed
          word, so the NEXT dose is not aborted by a stop typed before it. */
       cli_stop_clear();
       hal_serial_write("stop: no dose running\n");
       return true;
     }
   ```

   and add it to task 11's `test_parses_every_bench_command`, which is where spec §6's full
   bench command set is asserted:

   ```cpp
     TEST_ASSERT_TRUE(cli_dispatch("stop"));
   ```

8. - [ ] **Run the whole gate — both console builds — and commit.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native && pio test -e native_bench -f test_cli && \
     grep -c 'hal_serial_read(' src/cli.cpp && make check && pio run -e uno_r4_wifi -e uno_r4_wifi_bringup 2>&1 | tail -3
   ```
   expected: `0 Failures` from both suites, `2` from the grep (`read_console_()` and the
   matcher, and nothing else), `all invariants hold`, two `SUCCESS` lines. `native_bench`
   matters: `stop` and `dry on` ship in the **bench** binary, so a case that only ran under
   `PB_BRINGUP` would prove nothing about the binary that runs unattended.

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && git add include/cli.h src/cli.cpp test/test_cli/test_cli.cpp && git commit -m "cli: the byte-wise mid-dose abort matcher, and stop is finally a command

   Spec 2.12 advertised cli_stop_requested() as the last-resort abort and left it undefined
   at the seam. It is a line-exact, byte-at-a-time matcher with its own hal_serial_read():
   it consumes only an exact stop\\n or dry on\\n and pushes every other byte back for
   cli_poll()'s line buffer, in order, newline included. sta leaves three bytes, not two;
   stopp aborts nothing.

   2.12 calls the state four bytes. dry on is six, and the same matcher has to recognise it,
   so the partial buffer is eight - four would truncate dry on into a permanent non-match.

   stop was specified as a mid-dose matcher and never added to cli_dispatch(), so a stop
   typed at an idle console answered '? unknown; type help' - the one command an operator
   reaches for in an emergency, missing from the console that prints the help. It is a
   command now, in both binaries, and it also clears a stale request so a stop typed before
   a dose cannot abort the dose after it.

   The pushback is not a queue. Task 17's end-of-dose sequence is hal_serial_drain() then
   cli_stop_clear(), which discards the UART ring and this buffer together: 15.3's three
   impatient pump 60000 lines must not become 180 seconds of pumping."
   ```

9. - [ ] **What this task does not prove.** The fake's serial ring is not a CDC line. **Bring-up 7a and 7c** (spec §13) are where `stop` is typed at a real console during a real dose, over the USB CDC bridge that the ESP32 provides — and the property that matters there is latency, not parsing: `dose_run()` polls this function once per loop iteration, so an abort lands within one iteration, but nothing in the host suite measures the round trip through the modem's UART bridge. Note the result in the bring-up notes.

---

---

### Task 17: `dose_run()` — the refusal ladder, the two caps, the target arithmetic and the pump writes

**Drop 2.**

> **Read spec §2.8 in full before the first line. This is the file where a mistake puts water on the floor.**

**Files:**
- Modify: `include/safety.h`, `src/safety.cpp`, `src/cli.cpp` (`status` gains `pulses_per_l`, the prime and stall windows, the measured cap clamp or `cap=UNCLAMPED`, and the real `last=` line), `test/test_dose/test_dose.cpp`
- Test: `test/test_dose/test_dose.cpp`

**Interfaces:**
- Consumes: `bool hal_wdt_alive(void)`, `void hal_pump_write(bool)`, `uint32_t hal_millis(void)`, `void hal_serial_drain(void)`, `int hal_pin_read(uint8_t)` from task 3; `g_nv` and `noinit_commit()` from task 4; `void safety_tick(void)`, `bool safety_dosing(void)`, `void safety_set_dosing(bool)` from tasks 5 and 7; `uint32_t pulses_flow(void)`, `uint32_t pulses_flow_rate(void)`, `uint32_t pulses_to_ml(uint32_t pulses, uint16_t per_l)`, `void pulses_leak_rearm_at(uint32_t at_ms)` from task 6; `bool sensors_i2c_healthy(void)` from task 7; `bool cart_pos_known(void)`, `uint8_t cart_pos(void)`, `bool cart_bus_check(void)` from task 14; `bool safety_float_ok_debounced(void)`, `bool safety_dry(void)`, `void safety_float_refusal_count(bool)` from task 15; `bool cli_stop_requested(void)`, `void cli_stop_clear(void)` from task 16.
- Produces — `include/safety.h` gains, from spec §2.8:

  ```c
  /* THIS TASK DECLARES dose_result_t, and it is the first declaration in the tree: task 5's
     cut of safety.h carries only safety_tick/safety_wait_ms/safety_dosing/safety_set_dosing,
     and tasks 15 and 16 name DOSE_REFUSED_FLOAT only in prose and comments. Add it here, once.
     DOSE_RESULT_COUNT is an ADDITION to spec §2.8's
     printed enum, and the only one: it is what lets test_pump_is_off_on_every_exit_path loop
     over the enum, so a result added later without a way to reach it fails a test instead of
     going quietly unreachable. err_of() must never map it. */
  typedef enum { DOSE_OK = 0, DOSE_REFUSED_WDT, DOSE_REFUSED_DRY, DOSE_REFUSED_CONTRA,
                 DOSE_REFUSED_BOOT, DOSE_REFUSED_RANGE, DOSE_REFUSED_CAL, DOSE_REFUSED_FLOAT,
                 DOSE_REFUSED_POS, DOSE_REFUSED_I2C, DOSE_REFUSED_BUSY, DOSE_REFUSED_COOLDOWN,
                 DOSE_REFUSED_NOISE, DOSE_ABORT_CAP, DOSE_ABORT_NOFLOW, DOSE_ABORT_NOISE,
                 DOSE_ABORT_FLOAT, DOSE_ABORT_POS, DOSE_ABORT_STOP,
                 DOSE_RESULT_COUNT } dose_result_t;

  typedef struct { uint8_t outlet; uint16_t ml; bool by_time; uint32_t cap_ms;
                   bool need_pos; bool long_prime; } dose_req_t;

  dose_result_t dose_run(const dose_req_t *q);
  uint16_t      dose_flow_ml(void);
  dose_result_t dose_last_result(void);
  uint32_t      dose_last_ms(void);
  uint32_t      dose_last_pulses(void);
  uint8_t       dose_last_outlet(void);
  const char   *err_of(dose_result_t r);
  const char   *safety_last_err(void);
  void          safety_set_err(const char *tok);
  uint16_t      cfg_pulses_per_l_get(void);
  bool          cfg_pulses_per_l_set(uint16_t v);
  ```

  `dose_req_t.outlet` is **never a sentinel**: `water=0` is a legal backend command (`_int_in(v,"water",0,256)`, and butler's `outlet is None` guard does not catch 0), so 0 arrives from the wire and is refused here as well as by task 26's range check. There is **no `return` between the ON write and the OFF write**; the loop's only exit is a `break`. Task 20 adds the unconditional `bool hang;` member to `dose_req_t` and its loop hook; task 19 adds the contradiction latch to `dose_end_ml_()` and nowhere else; task 18 fills the rest of the loop body.

**`safety_last_err()` is the wire's `err=`, and this is where it acquires a value.** `g_last_err` is a `const char *` holding one bare lowercase token of spec §4.1's fixed enum — no whitespace, ever, because a space would split into a non-`k=v` token and 400 the whole report. Every token in that enum has exactly one named producer, and this is the roll-call, written against the functions that actually exist rather than against the ones an earlier draft assumed:

| token(s) | producer |
| --- | --- |
| `none float pos noflow noise cap stop wdt dry contra boot range cal i2c busy cooldown` | `dose_end_()` / `dose_end_ml_()`, **here**, through `err_of()` |
| `stuck`, `leak` | `report_build()` (task 22 step 12) — from `sensors_stuck()` and `pulses_leak_seen()`, in that precedence, above `safety_last_err()` |
| `txcap` | `report_build()`'s truncation path (task 22 step 4), which also clears it on the next body that fits |
| `heap` | `report_heap_ok()` (task 22), called at the top of `report_build()`, **and** `main.cpp`'s boot check (task 12 step 3) |
| `wdt`, `adc` | `main.cpp`'s boot assertions (task 12 step 3) |
| `resetmid` | `main.cpp`'s boot block (task 12 step 3) and this file's lazy initialisation, both from `noinit_reset_mid()` (task 4) |
| `recv` | `netfsm`'s receipt ack (task 24) — the one token that must never reach the wire |
| `goto`, `range` | `exec.cpp`'s terminal paths (task 26) |

There is no `sensors_poll()` and no `sensors_sweep()` in that table: `sensors_sweep()` (task 7) never calls `safety_set_err()`, and `err=stuck` is `report_build()`'s reading of `sensors_stuck()`.

**Spec sections to read in full before starting:** §2.6's guard table; §2.8 in full; §7's dose and flow blocks; §15.3.

**Tests:** `test_dose_refused_when_the_watchdog_counter_is_not_moving`, `test_dose_refused_when_the_dry_latch_is_set`, `test_dose_refused_inside_the_boot_gap`, `test_dose_refused_inside_the_minimum_gap_since_the_last_dose`, `test_dose_refused_when_the_float_reads_not_ok`, `test_dose_refused_when_a_single_float_sample_is_bad`, `test_dose_refused_when_position_is_unknown`, `test_dose_refused_when_the_cart_is_at_another_outlet`, `test_dose_refused_when_i2c_is_unhealthy`, `test_dose_refused_when_ml_exceeds_the_rig_ceiling`, `test_dose_refused_when_the_cap_is_zero`, `test_dose_refused_when_a_need_pos_dose_names_outlet_zero`, `test_dose_refused_when_the_idle_pulse_rate_is_nonzero`, `test_metered_dose_with_a_zero_target_is_refused_not_run_to_cap`, `test_console_pump_does_not_require_a_known_position`, `test_dose_stops_at_the_millilitre_target`, `test_dose_stops_at_the_cap_when_flow_never_reaches_target`, `test_dose_cap_is_clamped_to_sixty_seconds`, `test_cap_is_clamped_to_twice_the_requested_millilitres`, `test_target_pulses_match_the_calibration_within_one_pulse`, `test_pump_is_off_on_every_exit_path`, `test_pump_on_time_never_exceeds_the_cap`, `test_refusal_reports_zero_millilitres_not_the_previous_dose`, `test_the_ladder_reports_the_more_specific_reason`, `test_dose_cap_holds_across_a_millis_rollover`, `test_bytes_buffered_during_a_dose_are_discarded_not_executed`. (`test_the_ladder_reports_contra_above_dry` is the second half of the ordering pair and lands in this same file in **task 19 step 5**, because the latch fixture it needs does not exist until then.)

**Deliverable:** `pio test -e native -f test_dose` passes twenty-six cases (twenty-seven after task 19 step 5 appends the contra-above-dry half) and `pio test -e native_measured -f test_dose` (task 1's `extends = env:native` plus `-DPB_ML_PER_S_MEASURED=30`) passes the measured clamp case and the plausibility case that are compiled out at 0. **A third environment in which one case takes its other arm: `pio test -e native_cal -f test_dose`** (task 1's `extends = env:native` plus `-DPB_PULSES_PER_GATE=1450`) — the command runs the whole suite, and the point of it is that two cases (`test_dose_refused_when_the_cart_is_at_another_outlet` and the calibrated arm of `test_dose_refused_when_position_is_unknown`) compile to something different in it. `test_dose_refused_when_the_cart_is_at_another_outlet` needs `cart_pos_known()` to be *able* to answer true, which §2.15 forbids while the pitch is 0 — under `native` and `native_measured` it `TEST_IGNORE`s, and without that third command the ladder's second position line is never executed anywhere. Step 10's gate runs all three. `test_target_pulses_match_the_calibration_within_one_pulse` runs at cfg = 1000, 1999, 5880 and 20000: divide-first ordering truncates the calibration to whole pulses per millilitre, stopping every metered dose 15% short at the nominal 5880 and by 2x at 1999 — and butler's `2*flow_ml < ml` alert never fires on either, so the pot is quietly short-watered while every field reports healthy. **`test_pump_is_off_on_every_exit_path` covers all nineteen values of `dose_result_t`** — the skeleton says eighteen, which is the count without `DOSE_OK`; count them from the enum above, not from prose. The `WiFiS3|link\.h|Network\.h` grep over `safety.cpp` is task 30's and is not in `tools/check.sh` yet, so step 9 checks it by hand.

---

1. - [ ] **Write the case that is this whole task in one assertion: the pump is off on every exit path.** Add to `test/test_dose/test_dose.cpp` with its `RUN_TEST` line:

   ```cpp
   /* §2.8. Nineteen results, nineteen exits, and D6 must be OFF at every one of them. The
      loop runs over the enum against DOSE_RESULT_COUNT, so a result added without a way to
      reach it fails HERE rather than in six months on a bench with 12 V on COM. */
   void test_pump_is_off_on_every_exit_path(void) {
     char skipped[256] = {0};
     unsigned driven = 0;
     for (unsigned r = 0; r < (unsigned)DOSE_RESULT_COUNT; ++r) {
       pb_test_setup();
       if (!pb_drive_dose_to_result((dose_result_t)r)) {  /* not reachable in THIS build */
         if (skipped[0]) strncat(skipped, ", ", sizeof skipped - strlen(skipped) - 1);
         strncat(skipped, pb_result_name(r), sizeof skipped - strlen(skipped) - 1);
         continue;                     /* NEVER TEST_IGNORE in this loop -- see below */
       }
       ++driven;
       TEST_ASSERT_EQUAL_MESSAGE((int)r, (int)dose_last_result(), pb_result_name(r));
       TEST_ASSERT_FALSE_MESSAGE(sim_pump_is_on(),  pb_result_name(r));
       TEST_ASSERT_FALSE_MESSAGE(safety_dosing(),   pb_result_name(r));
     }
     /* Asserted, so a build that quietly stops driving an arm fails HERE. */
     TEST_ASSERT_EQUAL_UINT_MESSAGE(PB_DRIVABLE_RESULTS, driven, skipped);
     if (skipped[0]) TEST_MESSAGE(skipped);   /* after the loop, and MESSAGE, not IGNORE */
   }
   ```

   **`pb_drive_dose_to_result()` returns `bool`, and nothing inside this loop may call
   `TEST_IGNORE_MESSAGE`.** Unity's `TEST_IGNORE_MESSAGE` is `UNITY_IGNORE_AND_BAIL`: it
   sets the ignored flag and calls `TEST_ABORT()`, which longjmps out of the WHOLE test
   function, not out of one `switch` arm. An ignore inside this loop ends the case at the
   first unreachable result, every later result goes undriven, and `pio test` still prints
   `0 Failures` — a green test over the one invariant this rig has no hardware left to
   enforce, which is worse than no test at all. So the helper returns `false` for a result
   this build cannot reach, the loop continues and records the name, and the number of
   driven arms is ASSERTED against `PB_DRIVABLE_RESULTS`, a `#define` in this file that
   each later task raises as it makes an arm reachable.

   `pb_drive_dose_to_result()` and `pb_result_name()` are statics of **this suite**, not of
   `harness.h`: a `switch` over the enum that sets up the one condition each result needs
   (`sim_wdt_stop()` for WDT, `safety_dry_set(true)` for DRY, `sim_set_float(false)` for
   FLOAT, and so on), calls `dose_run()`, and returns `true`. Write the `switch` with **no
   `default:` arm**, so `-Wall -Wextra` reports a newly added enum value as a missing case
   rather than letting it fall through to a vacuous pass.

   **Which arms are drivable here, counted rather than guessed.** `DOSE_ABORT_STOP` IS
   reachable at task 17 and gets a real arm now: this task's loop already calls
   `cli_stop_requested()` (task 16), and bytes pushed with `sim_serial_rx("stop\n")` before
   `dose_run()` survive its opening `cli_stop_clear()` — which clears the pushback and the
   request, not the fake's UART ring — so the matcher consumes them inside the loop. Drive
   it with a `by_time` dose and a cap of a few seconds. Five arms are genuinely not
   drivable yet, because the breaks that produce them do not exist: `ABORT_NOFLOW`,
   `ABORT_NOISE`, `ABORT_FLOAT` and `ABORT_POS` are task 18's rules 5, 6 and 7, and
   `REFUSED_CONTRA` is task 19's latch. Hence `#define PB_DRIVABLE_RESULTS 14` here, 18
   after task 18 step 9, 19 after task 19 step 8.

2. - [ ] **Run it and watch it fail to link.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose
   ```
   expected: `undefined reference to 'dose_run(dose_req_t const*)'`.

3. - [ ] **Write the two exit helpers first — and first of all, `safety.cpp`'s FULL include
   block.** Task 5 step 4 wrote it as `safety.h` and `hal.h`, and this task, task 15, task 18
   and task 19 between them make the file consume five more headers. Print it once, here, so
   no later step has to guess:

   ```c
   /* src/safety.cpp -- safety_tick, safety_wait_ms, the float debounce, both latches and
      dose_run(). The include list is deliberately complete and deliberately SHORT: §9 greps
      this file for zero hits of the network seam, the library wrapper and the modem driver,
      and none of the seven below is any of those. */
   #include "safety.h"
   #include "cart.h"        /* cart_bus_check(), cart_pos_known(), cart_pos() -- task 18's rule 9 */
   #include "cli.h"         /* cli_stop_requested(), cli_stop_clear() -- task 16 */
   #include "config.h"      /* every PB_* the ladder, the caps and the loop read */
   #include "hal.h"
   #include "noinit.h"      /* g_nv, noinit_commit(), noinit_reset_mid() -- the two latches */
   #include "pins.h"        /* PIN_HALL_FLOAT -- the float debounce reads it directly */
   #include "pulses.h"      /* pulses_flow(), pulses_flow_rate(), pulses_to_ml(), the leak rearm */
   #include "sensors.h"     /* sensors_i2c_healthy() -- the ladder's i2c refusal */
   ```

   Every path out of `dose_run()` goes through one of the two helpers below, and both
   **always** set the whole result block — a refusal that left `g_last_flow_ml` alone would ack
   the previous dose's millilitres.

   ```c
   /* ---- the two exits. Both ALWAYS set every field, so a refusal can never ack the
      previous dose's millilitres (§2.8's second eye-checkable property). ---- */
   static dose_result_t g_last_result = DOSE_OK;
   static uint16_t      g_last_flow_ml;
   static uint32_t      g_last_ms, g_last_pulses, g_last_end_ms;
   static uint8_t       g_last_outlet;
   static const char   *g_last_err;
   static bool          g_float_granted;
   static uint16_t      g_pulses_per_l = PB_PULSES_PER_L_DEFAULT;
   static uint32_t      g_prime_ms     = PB_PRIME_MS_DEFAULT;
   static uint32_t      g_stall_ms     = PB_STALL_MS_DEFAULT;

   static dose_result_t dose_end_(dose_result_t r, const dose_req_t *q) {
     g_last_result = r;
     g_last_flow_ml = 0u;              /* a refusal delivered nothing. Never the last figure. */
     g_last_ms = 0u; g_last_pulses = 0u;
     g_last_outlet = q ? q->outlet : 0u;
     g_last_err = err_of(r);
     /* §2.10: the counter is incremented ONLY by a float refusal. Every other refusal leaves
        it alone -- a rig refusing for a stalled cart must not forget that the float has been
        flapping for an hour. dose_end_ml_() below is what clears it. */
     if (r == DOSE_REFUSED_FLOAT) safety_float_refusal_count(true);
     return r;
   }

   static dose_result_t dose_end_ml_(dose_result_t r, uint32_t got_pulses, uint32_t elapsed_ms,
                                     uint8_t outlet, uint32_t prime_ms, bool long_prime) {
     (void)prime_ms; (void)long_prime;      /* task 19 consumes both, and nothing else does */
     g_last_result  = r;
     g_last_pulses  = got_pulses;
     g_last_flow_ml = (uint16_t)pulses_to_ml(got_pulses, g_pulses_per_l);
     g_last_ms      = elapsed_ms;
     g_last_outlet  = outlet;
     g_last_err     = err_of(r);
     safety_float_refusal_count(false);     /* only a GRANTED dose reaches here (§2.10) */
     return r;
   }

   uint16_t      dose_flow_ml(void)     { return g_last_flow_ml; }
   dose_result_t dose_last_result(void) { return g_last_result; }
   uint32_t      dose_last_ms(void)     { return g_last_ms; }
   uint32_t      dose_last_pulses(void) { return g_last_pulses; }
   uint8_t       dose_last_outlet(void) { return g_last_outlet; }

   /* §4.1's fixed enum: bare lowercase tokens, no whitespace, ever. A space here splits into
      a non-k=v token and 400s the whole report at exactly the moment it matters. */
   const char *err_of(dose_result_t r) {
     switch (r) {
       case DOSE_OK:               return "none";
       case DOSE_REFUSED_WDT:      return "wdt";
       case DOSE_REFUSED_DRY:      return "dry";
       case DOSE_REFUSED_CONTRA:   return "contra";
       case DOSE_REFUSED_BOOT:     return "boot";
       case DOSE_REFUSED_RANGE:    return "range";
       case DOSE_REFUSED_CAL:      return "cal";
       case DOSE_REFUSED_FLOAT:    return "float";
       case DOSE_REFUSED_POS:      return "pos";
       case DOSE_REFUSED_I2C:      return "i2c";
       case DOSE_REFUSED_BUSY:     return "busy";
       case DOSE_REFUSED_COOLDOWN: return "cooldown";
       case DOSE_REFUSED_NOISE:    return "noise";
       case DOSE_ABORT_CAP:        return "cap";
       case DOSE_ABORT_NOFLOW:     return "noflow";
       case DOSE_ABORT_NOISE:      return "noise";
       case DOSE_ABORT_FLOAT:      return "float";
       case DOSE_ABORT_POS:        return "pos";
       case DOSE_ABORT_STOP:       return "stop";
       case DOSE_RESULT_COUNT:     break;      /* the sentinel is not a result */
     }
     return "none";
   }

   const char *safety_last_err(void) {
     /* The `resetmid` token's ONLY producer, and it is lazy on purpose: main.cpp does not
        have to remember to call anything for bring-up 7c's `last=resetmid` to be reachable
        after a hang-forced reset (§2.3). */
     if (g_last_err == 0 && noinit_reset_mid()) g_last_err = "resetmid";
     return g_last_err ? g_last_err : "none";
   }
   void safety_set_err(const char *tok) { g_last_err = tok; }

   uint16_t cfg_pulses_per_l_get(void) { return g_pulses_per_l; }
   bool cfg_pulses_per_l_set(uint16_t v) {
     if (v < PB_PULSES_PER_L_MIN || v > PB_PULSES_PER_L_MAX) return false;
     g_pulses_per_l = v;
     return true;
   }
   ```

4. - [ ] **Write `dose_run()`'s skeleton** — the ladder, the caps, the bracket, the one ON write and the one OFF write. Task 18 fills the loop; nothing about the shape below changes when it does.

   ```c
   /* ---- dose_run(). THE ONLY CALLER OF hal_pump_write(true) IN THE PROGRAM.
      Three properties are meant to be checkable by eye (§2.8):
        (a) exactly ONE hal_pump_write(true) and exactly ONE hal_pump_write(false) below it,
            with NO `return` between them -- the loop's only exit is a `break`;
        (b) every refusal is ABOVE the ON write, so a refused dose never asserts D6;
        (c) the loop body's first statement is safety_tick(), so the dog is fed on every
            iteration and a 60 s dose is legal under a 5592 ms grant (§3). ---- */
   dose_result_t dose_run(const dose_req_t *q) {
     cli_stop_clear();          /* a stop typed and answered BEFORE this dose is not its abort */
     g_float_granted = false;

     /* --- the ladder, in §2.8's printed order. The ORDER is the contract: the more specific
        reason must be the one reported, so contra sits above dry and cal above range. --- */
     if (safety_dosing())                       return dose_end_(DOSE_REFUSED_BUSY, q);
     if (!hal_wdt_alive())                      return dose_end_(DOSE_REFUSED_WDT, q);
     if (safety_contra())                       return dose_end_(DOSE_REFUSED_CONTRA, q);
     if (safety_dry())                          return dose_end_(DOSE_REFUSED_DRY, q);
     if (hal_millis() < PB_BOOT_GAP_MS)         return dose_end_(DOSE_REFUSED_BOOT, q);
     if (g_last_end_ms != 0u &&
         hal_millis() - g_last_end_ms < PB_DOSE_MIN_GAP_MS)
                                                return dose_end_(DOSE_REFUSED_COOLDOWN, q);
     if (g_pulses_per_l < PB_PULSES_PER_L_MIN ||
         g_pulses_per_l > PB_PULSES_PER_L_MAX)  return dose_end_(DOSE_REFUSED_CAL, q);
     if (!q->by_time && (q->ml == 0u || q->ml > PB_DOSE_RIG_MAX_ML))
                                                return dose_end_(DOSE_REFUSED_RANGE, q);
     if (q->cap_ms == 0u)                       return dose_end_(DOSE_REFUSED_RANGE, q);
     /* outlet 0 is a LEGAL backend command, so it arrives here from the wire and is refused
        here as well as by exec.cpp -- never a sentinel, never assumed non-zero. */
     if (q->need_pos && (q->outlet < 1u || q->outlet > PB_OUTLETS))
                                                return dose_end_(DOSE_REFUSED_RANGE, q);
     if (pulses_flow_rate() > PB_FLOW_IDLE_MAX_HZ)
                                                return dose_end_(DOSE_REFUSED_NOISE, q);
     if (!sensors_i2c_healthy())                return dose_end_(DOSE_REFUSED_I2C, q);
     if (!safety_float_ok_debounced())          return dose_end_(DOSE_REFUSED_FLOAT, q);
     if (q->need_pos && !cart_pos_known())      return dose_end_(DOSE_REFUSED_POS, q);
     if (q->need_pos && cart_pos() != q->outlet) return dose_end_(DOSE_REFUSED_POS, q);
     g_float_granted = true;                    /* consumed by dose_end_ml_() -- §2.7 */

     /* --- the two cap clamps, in §2.8's own arithmetic and its own order --- */
     uint32_t cap_ms = q->cap_ms;
     if (cap_ms > PB_DOSE_CAP_MS_MAX) cap_ms = PB_DOSE_CAP_MS_MAX;   /* == butler MAX_CAP_S */
   #if PB_ML_PER_S_MEASURED > 0
     if (!q->by_time) {           /* the cap may never authorise more than 2x the water asked for */
       uint32_t bound = (uint32_t)q->ml * 1000u / PB_ML_PER_S_MEASURED
                        * PB_CAP_SLACK_NUM / PB_CAP_SLACK_DEN;
       if (bound && cap_ms > bound) cap_ms = bound;
     }
   #endif
     uint32_t target = 0u;
     if (!q->by_time) {
       /* MULTIPLY FIRST, DIVIDE SECOND. `ml * (cfg/1000)` truncates the calibration to whole
          pulses per millilitre: at the nominal cfg = 5880 that is 5 instead of 5.88, so every
          metered dose stops 15% short forever, and at the legal cfg = 1999 it is a 2x error.
          Neither is visible to butler's 2*flow_ml < ml alert. Overflow is impossible and the
          range checks above are the proof: 250 x 20000 = 5e6, three orders below UINT32_MAX. */
       target = (uint32_t)q->ml * (uint32_t)g_pulses_per_l / 1000u;
       if (target == 0u) return dose_end_(DOSE_REFUSED_RANGE, q);  /* never "run to cap" */
     }
     uint32_t prime_ms = q->long_prime ? PB_PRIME_LONG_MS : g_prime_ms;
     if (q->long_prime && cap_ms > PB_PRIME_CAP_MS) cap_ms = PB_PRIME_CAP_MS;

     uint32_t flow0 = pulses_flow(), got = 0u, last_got = 0u;
     uint32_t t0 = hal_millis(), last_edge = t0, last_bus = t0, el = 0u;
     dose_result_t r = DOSE_ABORT_CAP;

     g_nv.dose_in_flight = true; noinit_commit();  /* a reset from here on latches dry (§2.3) */
     safety_set_dosing(true);                     /* safety_tick() now KEEPS the ON write */
     hal_pump_write(true);                         /* <-- THE ONLY ASSERTION OF D6 */
     for (;;) {
       safety_tick();                              /* fed on EVERY iteration */
       uint32_t now = hal_millis();
       el  = now - t0;                             /* unsigned diff: rollover-safe */
       got = pulses_flow() - flow0;
       if (got != last_got) { last_got = got; last_edge = now; }
       /* Task 18 inserts the two rate rules and the plausibility test ABOVE this line, */
       if (target && got >= target)  { r = DOSE_OK;        break; }
       if (el >= cap_ms)             { r = DOSE_ABORT_CAP; break; }
       /* and the prime, stall, float and bus rules HERE. */
       if (cli_stop_requested())     { r = DOSE_ABORT_STOP; break; }
       (void)last_bus;                             /* task 18's PB_POS_RECHECK_MS timer */
     }
     hal_pump_write(false);          /* unconditional, ONE exit, before any bookkeeping */

     safety_set_dosing(false);
     g_nv.dose_in_flight = false; noinit_commit();
     g_last_end_ms = hal_millis();
     pulses_leak_rearm_at(g_last_end_ms + PB_COAST_MS);  /* impeller coast-down is not a leak */
     hal_serial_drain();             /* the UART ring: impatience typed during the dose */
     cli_stop_clear();               /* and cli.cpp's pushback buffer, for the same reason
                                        (§2.8, §15.3): three impatient `pump 60000` lines must
                                        not become 180 seconds of pumping the moment this ends */
     return dose_end_ml_(r, got, g_last_end_ms - t0, q->outlet, prime_ms, q->long_prime);
   }
   ```

   `safety_contra()` does not exist yet. Add **both halves** now, not just the body: the line
   `bool safety_contra(void);` to `include/safety.h` beside `safety_dry()`, and the two-line
   definition `bool safety_contra(void) { return g_nv.contra_latched; }` to `safety.cpp`. A
   definition below `dose_run()` with no declaration above it compiles only if the implementer
   happens to order the file the one way that works, and task 19's own step 4 assumes the
   declaration is already there. So the ladder compiles in its final order today; task 19 adds
   the setter, the clear and the tests. Writing the ladder without the arm and inserting it
   later is how an ordering gets lost.

5. - [ ] **Run and watch the exit-path case pass.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose
   ```
   expected: `0 Failures`, with the four ignores task 18 and task 19 delete. If a result is
   unreachable, the failure names it — fix the fixture's `switch`, never the assertion.

6. - [ ] **Commit the spine on its own.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && git add include/safety.h src/safety.cpp test/test_dose/test_dose.cpp && git commit -m "dose_run(): the refusal ladder, the two caps, and the two pump writes

   The only caller of hal_pump_write(true) in the program. One ON write, one OFF write, no
   return between them, and the loop's only exit is a break - so the pump cannot be left on
   by a path nobody thought of. Every refusal is above the ON write. The loop body's first
   statement is safety_tick(), which is what makes a 60 s dose legal under a 5592 ms grant.

   The target arithmetic multiplies first. Dividing the calibration down to whole pulses per
   millilitre truncates it: every metered dose stops 15%% short at the nominal 5880 and by
   2x at the legal 1999, and butler's own 2*flow_ml < ml alert is silent on both - so the rig
   would have under-watered every pot indefinitely with nothing anywhere disagreeing.

   dose_end_() and dose_end_ml_() always set the whole result block, so a refusal can never
   ack the previous dose's millilitres. Only dose_end_ml_() clears the float-refusal counter,
   because 2.10 says a GRANTED dose clears it; a refusal for cooldown or a stalled cart must
   not make the board forget that the float has been flapping.

   DOSE_RESULT_COUNT is the one addition to 2.8's printed enum: it lets the exit-path test
   loop over every result, so one added without a way to reach it fails a test instead of
   going quietly unreachable."
   ```

7. - [ ] **Write the ladder's ORDER cases, not only its membership.** Membership is easy to get right; order is what carries the meaning. An operator reading `err=dry` when the real reason was the contradiction latch pulls the tank apart looking for water that is already there.

   **The ladder has two ordering pairs and only one of them can be tested today.** `contra`
   above `dry` needs a real latch, and the only way to earn one is `pb_latch_contra()`, which
   **task 19 step 1 creates**. Writing it here would leave this task's own gate
   (`pio test -e native -f test_dose`, step 8 and step 10) unable to link. So this step writes
   the `cal above range` half, and **task 19 step 5 appends
   `test_the_ladder_reports_contra_above_dry` to this same file** once its fixture exists.
   Both names are in this task's Tests list; the second is marked as task 19's.

   ```cpp
   void test_the_ladder_reports_the_more_specific_reason(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true);
     safety_force_bad_cal_();                          /* see below */
     dose_req_t r = {0}; r.ml = 9999u; r.cap_ms = 1000u;
     dose_result_t got = dose_run(&r);
     /* g_pulses_per_l is a file static of safety.cpp and pb_test_setup() cannot reach it, so
        the calibration is put back HERE and before the assertion - a suite that left it at
        zero would refuse every later case with DOSE_REFUSED_CAL, on the failing path too. */
     (void)cfg_pulses_per_l_set(PB_PULSES_PER_L_DEFAULT);
     TEST_ASSERT_EQUAL(DOSE_REFUSED_CAL, got);               /* cal above range */
   }

   void test_refusal_reports_zero_millilitres_not_the_previous_dose(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     pulses_begin();          /* own tumbling window: the storm case leaves ~99 Hz behind */
     sim_set_float(true); sim_set_flow_ml_s(30);
     /* long_prime, and a cap ABOVE the fake's first edge. sim_flow_hz_() emits nothing until
        el >= PB_PRIME_MS_DEFAULT (3000), so a 2000 ms dose delivers zero pulses and this
        assertion fails; and once task 18's prime rule lands, any dose on the default window
        aborts NOFLOW at el == 3000. long_prime moves the window to PB_PRIME_LONG_MS (15000)
        and PB_PRIME_CAP_MS (20000) leaves a 6000 ms cap alone: ~3000 ms of flow, ~88 ml. */
     dose_req_t ok = {0}; ok.by_time = true; ok.cap_ms = 6000u; ok.long_prime = true;
     (void)dose_run(&ok);
     TEST_ASSERT_TRUE(dose_flow_ml() > 0u);
     pb_advance(PB_DOSE_MIN_GAP_MS + 1u);
     sim_set_float(false);
     TEST_ASSERT_EQUAL(DOSE_REFUSED_FLOAT, dose_run(&ok));
     TEST_ASSERT_EQUAL_UINT16(0u, dose_flow_ml());
   }
   ```

   `cfg_pulses_per_l_set()` refuses out-of-range values by contract, so give `safety.cpp` a
   `void safety_force_bad_cal_(void)` under `#ifdef PB_NATIVE` that writes `g_pulses_per_l = 0`
   directly — **and DECLARE it in `include/safety.h`, under the same `#ifdef PB_NATIVE`, beside
   `safety_set_dosing()`, with the same "host-suite seam" comment.** No header the suite
   includes declares it otherwise, and a definition with no declaration is a link error the
   moment the compiler sees the call first. The `DOSE_REFUSED_CAL` arm exists to catch a value that got in some other way —
   through a corrupted `.noinit`, a future backend `cal=`, or a bug — and a test that cannot
   produce one is not testing it.

8. - [ ] **Write the remaining cases from this task's Tests list**, two at a time, in this plan's rhythm: write, run and see it fail, minimal change, run and see it pass. **The fifteen printed below are the refusal ladder and both target rules — the safety spine of drop 2.** Four more cases need a word rather than code, and the two clamp cases are a compile guard; both follow the block.

   **Four things every case below depends on, stated once here instead of fifteen times.**

   - **Two includes `test/test_dose/test_dose.cpp` does not carry yet**: `#include "cart.h"` (task 14) and `#include "sensors.h"` (task 7). `config.h`, `safety.h`, `hal.h`, `sim.h`, `noinit.h` and `pulses.h` are already there, through `../support/harness.h` and tasks 4-6.
   - **`pb_test_setup()` resets the FAKE, not the program.** `sim_reset(false)` clears the fake's rig and — because a cold boot fails the magic check — the whole `.noinit` struct, so both latches start down. It does **not** touch a file static of `pulses.cpp`, `sensors.cpp`, `cart.cpp` or `safety.cpp`. Four of those bite, and each case below deals with its own: `pulses_begin()` rebases the rate estimator and both counters; `sensors_begin()` puts the I2C health counters back; `cart_begin()` puts the position back to unknown; and `cfg_pulses_per_l_set()` puts the calibration back after step 7's `safety_force_bad_cal_()`.
   - **`g_last_end_ms` is the fourth, and nothing can reset it**, so the cooldown rung is what a stale one hits. Every case that runs a dose advances `PB_BOOT_GAP_MS + 1` first, which puts every dose's *end* above 11 s of fake clock while the next case's cooldown read happens at about 10 s — an unsigned difference that is always huge. Do not shorten that advance to save iterations; it is what keeps the suite order-independent.
   - **Every ladder case asserts the SPECIFIC `dose_result_t`, never merely that the dose was refused.** The order is the contract (§2.8), and an operator reading the wrong token takes the rig apart in the wrong place.

   ```cpp
   /* §2.6 guard 1, and it is the FIRST rung after busy for a reason: a counter that is not
      moving is a dog that will never bite, and every bound below it is then the only thing
      between a stuck loop and a running pump. */
   void test_dose_refused_when_the_watchdog_counter_is_not_moving(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_wdt_stop();                                 /* the counter FREEZES (task 3) */
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
     TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_WDT, dose_run(&q),
         "a frozen watchdog counter must refuse with wdt");
   }

   /* §2.11. `dry on` is strictly more refusing, and it is the one thing an operator with
      their hands in the plumbing can rely on. Nothing is latched above it here, so `dry` is
      the token that has to come back rather than `contra`. */
   void test_dose_refused_when_the_dry_latch_is_set(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     safety_dry_set(true);
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
     TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_DRY, dose_run(&q),
         "the dry latch must refuse with dry");
   }

   /* DECISIONS #5's minimum gap since boot, and §2.11's cover for the one thing .noinit
      cannot hold: a power cycle clears the dry latch, and PB_BOOT_GAP_MS is what stands in
      its place for the first ten seconds. pb_test_setup() leaves the fake's clock at zero and
      hal_wdt_alive()'s probe advances it by about 41 ms, so no advance is needed here - the
      ABSENCE of one is the arrangement. */
   void test_dose_refused_inside_the_boot_gap(void) {
     pb_test_setup();                                /* the clock is at zero: a fresh boot */
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
     TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_BOOT, dose_run(&q),
         "a dose inside PB_BOOT_GAP_MS must be refused with boot");
   }

   /* §2.6 guard 7, for EVERY caller - queued console impatience and backend adjacency alike.
      The first dose is by_time with a 1000 ms cap, deliberately UNDER PB_PRIME_MS_DEFAULT, so
      it ends on its own cap without arming task 18's prime rule and without satisfying §2.7's
      `elapsed_ms >= prime_ms`; it latches nothing. The second follows it immediately, and the
      cooldown is the only thing standing between them. */
   void test_dose_refused_inside_the_minimum_gap_since_the_last_dose(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     pulses_begin();
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
     (void)dose_run(&q);                             /* ends on its cap and stamps g_last_end_ms */
     TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_COOLDOWN, dose_run(&q),
         "a second dose inside PB_DOSE_MIN_GAP_MS must be refused with cooldown");
   }

   /* §2.10 and bring-up 5a. D5 reads PB_LOW for OK, so HIGH is a tank at or below the
      waterline - or a broken wire, which by DECISIONS #12's whole design is the same answer. */
   void test_dose_refused_when_the_float_reads_not_ok(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     pulses_begin();
     sim_set_float(false);
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
     TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_FLOAT, dose_run(&q),
         "a float reading not-OK must refuse with float");
   }

   /* §2.10's asymmetry, seen from the dose rather than from the debounce: PB_FLOAT_OK_SAMPLES
      consecutive OK readings GRANT, and one bad sample anywhere in that window refuses. The
      pattern below grants twice and fails on the third - the waterline bounce that a
      raw-sample implementation would report to the backend as float=1. */
   void test_dose_refused_when_a_single_float_sample_is_bad(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     pulses_begin();
     sim_set_float_pattern("1101111");               /* OK, OK, BAD, then OK forever (task 15) */
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
     TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_FLOAT, dose_run(&q),
         "one bad sample inside the debounce window must refuse with float");
   }

   /* §2.6 guard 4, FIRST line. A backend water command carries need_pos, and a cart whose
      position is unknown means the pump would dead-head against a closed manifold.

      The arrangement matters, because the obvious one tests nothing. cart_begin() leaves
      g_pos at 0 while q.outlet is 1, so guard 4's SECOND line (need_pos && cart_pos() !=
      outlet) refuses with the same DOSE_REFUSED_POS whether or not the first line exists:
      delete the rung this case is named for and it still passes. The calibrated arm below
      therefore loses the position while LEAVING cart_pos() equal to the outlet, so only the
      first line can answer. That is the case that matters in the field too: a failed
      cart_goto(N) clears g_home_seen and g_pos_valid but leaves g_pos where it was, so with
      only the second line a re-issued dose for that same outlet would run the pump against a
      cart nobody knows the position of. */
   void test_dose_refused_when_position_is_unknown(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     pulses_begin();
   #if PB_PULSES_PER_GATE == 0
     (void)cart_begin();                    /* §2.15: cart_pos_known() is hard false here, so
                                               the first line is the only one that can answer */
     dose_req_t q = {0};
     q.outlet = 1u; q.ml = 100u; q.cap_ms = 10000u; q.need_pos = true;
   #else
     (void)cart_begin();
     sim_set_screw_pulse_ms(2u); sim_set_home_region(0u, 40u); sim_set_cart_at(0u);
     TEST_ASSERT_TRUE(cart_goto(1u));       /* position known, and equal to 1 */
     sim_set_stall(true);
     (void)cart_goto(2u);                   /* fails: pos_valid false, cart_pos() still 1 */
     sim_set_stall(false);
     TEST_ASSERT_FALSE(cart_pos_known());
     TEST_ASSERT_EQUAL_UINT(1u, cart_pos());
     dose_req_t q = {0};
     q.outlet = 1u; q.ml = 100u; q.cap_ms = 10000u; q.need_pos = true;   /* outlet == pos */
   #endif
     TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_POS, dose_run(&q),
         "a need_pos dose with an unknown cart position must be refused with pos");
   }

   /* Guard 4's SECOND line, which is a different fact from the first: the position is known,
      and it is the wrong one. Only the calibrated arm can reach it - §2.15 compiles
      cart_pos_known() out to hard false while PB_PULSES_PER_GATE is 0, so under [env:native]
      the first line answers and this one is unreachable. PB_PULSES_HOME_TO_1 is still 0 in the
      calibrated arm, so the cart is already at outlet 1 when it is at home: cart_goto(1)
      lands without turning the screw and the arrangement costs about three milliseconds of
      fake clock, which is what keeps this case's cooldown read in the same place as every
      other case's. */
   void test_dose_refused_when_the_cart_is_at_another_outlet(void) {
   #if PB_PULSES_PER_GATE == 0
     TEST_IGNORE_MESSAGE("uncalibrated arm: cart_pos_known() is compiled out to false (spec 2.15), "
                         "so the ladder's second position line cannot be reached; native_cal runs it");
   #else
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     pulses_begin();
     (void)cart_begin();
     sim_set_home_region(0u, 40u);
     sim_set_cart_at(0u);                            /* already home, so cart_home() lands at once */
     TEST_ASSERT_TRUE_MESSAGE(cart_goto(1u), "arrange: the cart must be KNOWN at outlet 1");
     dose_req_t q = {0};
     q.outlet = 2u; q.ml = 100u; q.cap_ms = 10000u; q.need_pos = true;
     TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_POS, dose_run(&q),
         "a dose for outlet 2 with the cart standing at outlet 1 must be refused with pos");
   #endif
   }

   /* §2.13's reason, stated as a refusal: A4/A5 are the mux select lines AND the home hall -
      the input that gates the pump - so an unhealthy bus is a rig that cannot say where its
      cart is. PB_I2C_FAIL_LIMIT consecutive failed transfers is what task 7 calls unhealthy.
      sensors.cpp's counters are file statics, so this case puts the bus back BEFORE it
      asserts, and therefore puts it back on the failing path too. */
   void test_dose_refused_when_i2c_is_unhealthy(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     pulses_begin();
     (void)sensors_begin();
     sim_set_i2c_fail(true);
     for (uint8_t i = 0; i < PB_I2C_FAIL_LIMIT; ++i) (void)sensors_select(0u);
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
     dose_result_t r = dose_run(&q);
     sim_set_i2c_fail(false);
     (void)sensors_begin();                          /* leave the bus healthy for the next case */
     TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_I2C, r,
         "an unhealthy I2C bus must refuse with i2c");
   }

   /* DECISIONS #7 and §7. The PROTOCOL ceiling is PB_DOSE_MAX_ML (1000, == butler's
      MAX_DOSE_ML); the RIG ceiling is PB_DOSE_RIG_MAX_ML (250), "a reservoir small enough that
      a full dump is a mop-up". butler does not know about the smaller one - that is §4.6's
      going-live precondition - so 251 ml is a command the wire can legally carry and this rung
      is the only thing in the system that refuses it. */
   void test_dose_refused_when_ml_exceeds_the_rig_ceiling(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     dose_req_t q = {0};
     q.ml = (uint16_t)(PB_DOSE_RIG_MAX_ML + 1u);     /* 251: inside the protocol, outside the rig */
     q.cap_ms = 10000u;
     TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_RANGE, dose_run(&q),
         "a millilitre target above PB_DOSE_RIG_MAX_ML must be refused with range");
   }

   /* §2.6 guard 6, the pre-dose half. Pulses arriving with D6 OFF are not water: an unplugged
      or floating D2 counting garbage would otherwise "reach target" in milliseconds. The storm
      runs at 100 Hz - an order under PB_FLOW_MAX_HZ (1200) and two orders over
      PB_FLOW_IDLE_MAX_HZ (2) - so it is unmistakably the IDLE ceiling that answers and not the
      in-dose one task 18 adds. pulses_begin() rebases the tumbling window on this case's own
      clock, and the 500 ms of storm after it is what fills the window the ladder then reads. */
   void test_dose_refused_when_the_idle_pulse_rate_is_nonzero(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     pulses_begin();
     sim_flow_storm(100u);                           /* D2 counting with the pump OFF */
     pb_advance(500u);
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
     dose_result_t r = dose_run(&q);
     sim_flow_storm(0u);                             /* and the meter is quiet for the next case */
     TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_NOISE, r,
         "a non-zero idle pulse rate must refuse with noise, before D6 is ever asserted");
   }

   /* A metered dose whose target rounds to zero pulses must be REFUSED, never run to its cap.
      With target == 0 the loop's `target && got >= target` can never fire, so the only exit
      left is `el >= cap_ms`: a request for no water that asserts D6 for the whole cap, and
      DOSE_ABORT_CAP here is exactly that failure.
      The arithmetic says the `target == 0` guard below the caps is belt-and-braces rather than
      the rung under test. target = ml * cfg / 1000, and the two rungs above it bound
      ml >= 1 and cfg >= PB_PULSES_PER_L_MIN (1000), so the smallest legal product is
      1 * 1000 / 1000 = 1 pulse: it can only be reached through a corrupted cfg that got past
      the cal rung. The REACHABLE zero target is ml == 0 - which butler can send - and the
      range rung is what answers it. */
   void test_metered_dose_with_a_zero_target_is_refused_not_run_to_cap(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     dose_req_t q = {0};
     q.ml = 0u; q.by_time = false;                   /* metered, and asking for nothing */
     q.cap_ms = PB_DOSE_CAP_MS_MAX;
     TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_RANGE, dose_run(&q),
         "a metered dose with a zero target must be refused with range, not run to its cap");
   }

   /* TARGET RULE 1: a metered dose stops when the METER says so, not when the clock does.
      DOSE_OK is reachable through one line and one line only - `target && got >= target` - so
      the result IS the assertion. The arithmetic, from §7's constants and nothing else:

          target       = ml * cfg / 1000 = 250 * 5880 / 1000 = 1470 pulses   (MULTIPLY FIRST)
          fake's rate  = 85 ml/s * PB_PULSES_PER_L_DEFAULT / 1000 = 499 pulses/s
          first pulse  = PB_PRIME_MS_DEFAULT + 1 = 3001 ms into the dose
          target at    = 3001 + 1469 * 1000/499 = ~5945 ms

      and 5945 ms is inside every cap that applies: PB_PRIME_CAP_MS (20000) under long_prime,
      and 250 * 1000 / 30 * 2 = 16666 ms under native_measured.

      `long_prime` is not decoration here, and the reason is a property of the FIXTURE rather
      than of dose_run(): the fake's pump model delivers its first pulse at
      PB_PRIME_MS_DEFAULT + 1 ms after the ON write, which is one millisecond AFTER task 18's
      prime rule fires at `el >= prime_ms && got < PB_PRIME_MIN_PULSES`. A metered dose on the
      default window therefore aborts NOFLOW at exactly 3000 ms with nothing wrong with it.
      PB_PRIME_LONG_MS is the only printed lever that moves that window, and the
      PB_PRIME_CAP_MS it brings with it is still three times the time this dose needs. */
   void test_dose_stops_at_the_millilitre_target(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     pulses_begin();
     sim_set_flow_ml_s(85u);
     dose_req_t q = {0};
     q.ml = (uint16_t)PB_DOSE_RIG_MAX_ML;            /* 250 ml at the default calibration */
     q.cap_ms = PB_DOSE_CAP_MS_MAX;
     q.long_prime = true;
     TEST_ASSERT_EQUAL_MESSAGE(DOSE_OK, dose_run(&q),
         "a metered dose that reaches its pulse target must end DOSE_OK, not on its cap");
   }

   /* TARGET RULE 2: when the meter never reaches the target, the CAP ends the dose - and the
      cap is a bound on time, never a second target. The arithmetic:

          target = ml * cfg / 1000 = 100 * 5880 / 1000 = 588 pulses, and the meter delivers 0
          cap    = the typed 1000 ms; under native_measured the measured clamp computes
                   100 * 1000 / 30 * 2 = 6666 ms and leaves the typed value alone

      cap_ms is deliberately under PB_PRIME_MS_DEFAULT so that the cap is the only rule that
      can fire: task 18's prime and stall rules both arm at `el >= prime_ms`, and §2.7's latch
      needs `elapsed_ms >= prime_ms` too, so this dose ends without latching anything. */
   void test_dose_stops_at_the_cap_when_flow_never_reaches_target(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     pulses_begin();
     sim_set_flow_ml_s(0u);                          /* the pump runs; nothing moves */
     dose_req_t q = {0}; q.ml = 100u; q.cap_ms = 1000u;
     TEST_ASSERT_EQUAL_MESSAGE(DOSE_ABORT_CAP, dose_run(&q),
         "a metered dose that never reaches its target must end on its cap");
   }

   /* The cap bounds how long D6 is ASSERTED, which is the only thing that puts water on the
      floor - not how long dose_run() takes to return. sim_pump_on_ms() is the fake's
      cumulative count of milliseconds with the pin high, so it measures D6 and not the
      function. The twenty milliseconds of slack are the loop's own granularity: the fake
      advances one millisecond per hal_millis(), and the unconditional OFF write is the
      statement after the break. */
   void test_pump_on_time_never_exceeds_the_cap(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     pulses_begin();
     sim_set_flow_ml_s(0u);
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 2000u;  /* under PB_PRIME_MS_DEFAULT */
     (void)dose_run(&q);
     TEST_ASSERT_TRUE_MESSAGE(sim_pump_on_ms() <= 2000u + 20u,
         "D6 was asserted for longer than cap_ms");
   }
   ```

   and the seventeen `RUN_TEST` lines, in `main()` — the fifteen below plus step 7's two,
   which are declared non-static, so nothing warns if they are never registered and the
   suite silently reports twenty-two cases instead of twenty-four:

   ```cpp
     RUN_TEST(test_dose_refused_when_the_watchdog_counter_is_not_moving);
     RUN_TEST(test_dose_refused_when_the_dry_latch_is_set);
     RUN_TEST(test_dose_refused_inside_the_boot_gap);
     RUN_TEST(test_dose_refused_inside_the_minimum_gap_since_the_last_dose);
     RUN_TEST(test_dose_refused_when_the_float_reads_not_ok);
     RUN_TEST(test_dose_refused_when_a_single_float_sample_is_bad);
     RUN_TEST(test_dose_refused_when_position_is_unknown);
     RUN_TEST(test_dose_refused_when_the_cart_is_at_another_outlet);
     RUN_TEST(test_dose_refused_when_i2c_is_unhealthy);
     RUN_TEST(test_dose_refused_when_ml_exceeds_the_rig_ceiling);
     RUN_TEST(test_dose_refused_when_the_idle_pulse_rate_is_nonzero);
     RUN_TEST(test_metered_dose_with_a_zero_target_is_refused_not_run_to_cap);
     RUN_TEST(test_dose_stops_at_the_millilitre_target);
     RUN_TEST(test_dose_stops_at_the_cap_when_flow_never_reaches_target);
     RUN_TEST(test_pump_on_time_never_exceeds_the_cap);
     RUN_TEST(test_the_ladder_reports_the_more_specific_reason);             /* step 7 */
     RUN_TEST(test_refusal_reports_zero_millilitres_not_the_previous_dose);  /* step 7 */
   ```

   Four more cases need a word rather than code:

   - `test_target_pulses_match_the_calibration_within_one_pulse` runs at 1000, 1999, 5880 and 20000 and asserts `dose_last_pulses()` is within one pulse of `ml * cfg / 1000`. Write the **divide-first** version of the arithmetic first and watch it fail at 1999 by a factor of two — that failure is the finding.
   - `test_dose_cap_holds_across_a_millis_rollover` starts the fake's clock at `0xFFFFF000` with `sim_set_clock_ms()` and asserts the dose still ends at `cap_ms`. **The cap must straddle the wrap or the case is vacuous:** `0xFFFFF000` is 4095 ms below `UINT32_MAX`, so a cap of 1000 or 2000 ms — the pattern every other case here uses to dodge the prime rule — finishes before the wrap and never exercises an unsigned difference. Use `long_prime = true` with `cap_ms = 6000u`, so the clock wraps ~4095 ms in and the dose still ends at `el == 6000`. Every bound in the function is an unsigned difference and this is what proves it.
   - `test_console_pump_does_not_require_a_known_position` builds `need_pos = false` and asserts `DOSE_ABORT_CAP` rather than `DOSE_REFUSED_POS`, with `cart_pos_known()` false throughout. Bring-up 4a, 5a and 5b all run before the cart is calibrated; a `pump` that demanded a position would make them unrunnable.
   - `test_bytes_buffered_during_a_dose_are_discarded_not_executed` — **at task 17 assert the discard through `status`, not through `pump`.** `pump` is not a console command until task 20, so a buffered `pump 60000` reaches `? unknown; type help` and the case would pass whether or not the bytes were discarded. Push `status\n` during the dose and assert `sim_serial_tx()` carries no `granted=` afterwards, which the pushback-and-drain pair really does control today; task 20 owns the `pump 60000` form of this case, where §15.3's three-impatient-lines scenario can actually fail it. Add `#include "cli.h"` to this file — it is not reachable transitively, and `cli_poll()` and asserts nothing ran and `sim_pump_is_on()` is false. This is §15.3's scenario: three impatient lines typed at a console that looks frozen must not execute as 180 seconds of pumping the instant the first dose ends.

   **Only one of the two clamp cases is compiled out at `PB_ML_PER_S_MEASURED == 0`**, and
   getting that wrong loses a case rather than a line of prose. `test_cap_is_clamped_to_twice_the_requested_millilitres`
   tests the measured clamp, so it is guarded `#if PB_ML_PER_S_MEASURED > 0` with a
   `TEST_IGNORE_MESSAGE` on the other arm and `native_measured` is its arm.
   `test_dose_cap_is_clamped_to_sixty_seconds` tests `cap_ms > PB_DOSE_CAP_MS_MAX`, which is
   unconditional in `dose_run()`: no guard and no second environment. **But it cannot be written
   at task 17 and must not be attempted here.** A `by_time` dose with no flow ends at ~3001 ms
   once task 18's prime rule lands, so it never reaches the clamped 60000 ms cap; and once task
   19's latch lands the same dose satisfies all five of §2.7's conditions and latches `contra`,
   which is `.noinit`-backed and poisons every later case in the file. Reaching 60 s needs a
   meter that stays live for the whole dose — task 18 step 1's `sim_set_flow_burst_pulses(n)`
   with `n > 60000`, one edge per 1 ms step — and that injector does not exist yet. **Task 18
   owns this case.** The measured clamp case is writable here, but only inside a stated window:
   `long_prime = true`, metered, `ml` at or below 200, flow 0, typed `cap_ms =
   PB_DOSE_CAP_MS_MAX`; it asserts the dose ends at `ml * 1000 / PB_ML_PER_S_MEASURED * 2`
   (13332 ms at ml = 200) rather than at `PB_PRIME_CAP_MS` (20000), which is where the same
   request ends with the clamp compiled out. Then run all three environments:

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose && pio test -e native_measured -f test_dose && pio test -e native_cal -f test_dose
   ```

   expected: `0 Failures` from all three. Under `native` and `native_measured` one case is
   `IGNORE`d — `test_dose_refused_when_the_cart_is_at_another_outlet`, which `native_cal` runs
   — on top of the ignores task 18 and task 19 delete from step 1's fixture.

9. - [ ] **Wire `safety_last_err()` into `status`, replacing task 11's placeholder, and add the four lines this task owes.** This is a numbered step and not a hope: `last=resetmid` is bring-up 7c's pass criterion, and a hard-coded `last=none` makes that step unpassable. In `src/cli.cpp`, replace

    ```c
      /* PLACEHOLDER. Task 17 replaces this line ... */
      hal_serial_write("last=none\n");
    ```

    with

    ```c
      cli_printf_u32("pulses_per_l=%lu\n", (uint32_t)cfg_pulses_per_l_get());
      cli_printf_u32("prime_ms=%lu\n",     (uint32_t)PB_PRIME_MS_DEFAULT);
      cli_printf_u32("stall_ms=%lu\n",     (uint32_t)PB_STALL_MS_DEFAULT);
    #if PB_ML_PER_S_MEASURED > 0
      cli_printf_u32("cap=clamped to 2x the requested ml at %lu ml/s\n",
                     (uint32_t)PB_ML_PER_S_MEASURED);
    #else
      hal_serial_write("cap=UNCLAMPED (PB_ML_PER_S_MEASURED=0; bring-up 7b commits it)\n");
    #endif
      hal_serial_write("stop: `stop` and `dry on` abort a running dose; a backend stop=1 CANNOT "
                       "- net_poll() does not run while the pump is asserted (spec 2.12)\n");
      hal_serial_write("last=");
      hal_serial_write(safety_last_err());     /* one bare token of 4.1's fixed enum */
      hal_serial_write("\n");
    ```

10. - [ ] **Run everything, check the greps by hand, and commit.**

    ```bash
    cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native && pio test -e native_measured -f test_dose && \
      pio test -e native_cal -f test_dose && \
      grep -rEc 'WiFiS3|link\.h|Network\.h' src/safety.cpp ; \
      grep -c 'hal_pump_write(true)' src/safety.cpp && \
      grep -c 'hal_pump_write(false)' src/safety.cpp && make check && pio run -e uno_r4_wifi -e uno_r4_wifi_bringup 2>&1 | tail -3
    ```
    expected: `0 Failures` from all three runs; `0` from the network-header grep — `safety.cpp`
    **cannot** make a network call, and that is a property of its includes rather than of a
    comment; `1` for the ON write; **`2`** for the OFF write, and only two: `safety_tick()`'s
    idle re-assert and `dose_run()`'s single unconditional one. `hal_wdt_alive()`'s probe also
    writes the pump off, but it lives in `hal_uno.cpp` / `hal_sim.cpp` and is not in this file
    — read both call sites and confirm each is one of the two named here;
    `all invariants hold`; two `SUCCESS` lines.

    ```bash
    cd /Users/jcanton/projects/plant-butler/firmware && git add include/safety.h src/safety.cpp src/cli.cpp test/test_dose/test_dose.cpp && git commit -m "dose_run(): the ladder's order, the target arithmetic, and status tells the truth about last=

    The ladder's ORDER is the contract, not its membership: contra above dry, cal above
    range. An operator reading err=dry when the real reason was the contradiction latch
    pulls the tank apart looking for water that is already there.

    status's last= now prints safety_last_err() rather than a hard-coded none. That token is
    bring-up 7c's pass criterion - after a hang-forced watchdog reset the board must say
    dry=1 and last=resetmid - and it is produced by a lazy initialisation from
    noinit_reset_mid(), which is the only place that fact exists.

    cli_stop_clear() is dose_run()'s first statement and its last: first so that a stop typed
    and answered before a dose does not abort the dose after it, last so that the pushback
    buffer is discarded along with the UART ring. 15.3's three impatient pump 60000 lines
    must not become 180 seconds of pumping.

    status also states, in as many words, that a backend stop=1 can never interrupt a running
    dose - net_poll() does not run while D6 is asserted - so nobody reaches for it in an
    emergency."
    ```

11. - [ ] **What this task does not prove.** Every assertion here is against a fake pin and a fake meter. **Bring-up 4a** proves the relay is switched at all, dry, with no 12 V on COM, and that COM–NO stays open across a power cycle **and** a `hang`-forced watchdog reset. **Bring-up 5a** proves the float refusal on the rig, **7b** commits `PB_PULSES_PER_L_DEFAULT`, `PB_PRIME_MS_DEFAULT`, `PB_STALL_MS_DEFAULT` and `PB_ML_PER_S_MEASURED` — until that last one is committed the measured cap clamp does not exist and `status` says `cap=UNCLAMPED` — and **7c** proves the watchdog actually resets the chip, which cannot be simulated. The host tests prove the logic; bring-up proves the wiring, and neither substitutes for the other.

---

---

### Task 18: The dose loop's abort rules — prime, stall, both noise ceilings, plausibility, float, position

**Drop 2.**

**Files:**
- Modify: `src/safety.cpp` (**the `for(;;)` body only** — nothing above the ON write and nothing below the OFF write changes in this task), `include/sim.h` and `src/hal_sim.cpp` (the four pump-relative injectors of step 1: `sim_flow_storm_at_pump_on`, `sim_set_flow_burst_pulses`, `sim_set_float_at_ms`, `sim_serial_rx_at_ms`), `test/test_dose/test_dose.cpp`
- Test: `test/test_dose/test_dose.cpp`

**Interfaces:**
- Consumes: everything task 17 consumes, plus `PB_PRIME_MIN_PULSES` (5), `PB_PRIME_LONG_MS` (15000), `PB_PRIME_CAP_MS` (20000), `PB_FLOW_MAX_HZ` (1200), `PB_PLAUS_NUM` (4), `PB_PLAUS_DEN` (1), `PB_POS_RECHECK_MS` (1000) from task 2; `bool cart_bus_check(void)` from task 14; `int hal_pin_read(uint8_t)` and `PIN_HALL_FLOAT` from tasks 2-3.
- Produces: **no new production declarations** — step 1 adds four fake-rig injectors to `include/sim.h` (`sim_flow_storm_at_pump_on`, `sim_set_flow_burst_pulses`, `sim_set_float_at_ms`, `sim_serial_rx_at_ms`), which task 19 step 4 also consumes and must not respell. In production code the loop body reaches its final shape, and the ORDER is the whole fix:

  1. `pulses_flow_rate() > PB_FLOW_MAX_HZ` → `DOSE_ABORT_NOISE`
  2. the delivered-versus-elapsed plausibility test, under `#if PB_ML_PER_S_MEASURED > 0` → `DOSE_ABORT_NOISE`
  3. `target && got >= target` → `DOSE_OK`
  4. `el >= cap_ms` → `DOSE_ABORT_CAP`
  5. the prime rule, `el >= prime_ms && got < PB_PRIME_MIN_PULSES` → `DOSE_ABORT_NOFLOW`
  6. the stall rule, `el >= prime_ms && (now - last_edge) >= g_stall_ms` → `DOSE_ABORT_NOFLOW`
  7. the float single-sample abort → `DOSE_ABORT_FLOAT`
  8. `cli_stop_requested()` → `DOSE_ABORT_STOP`
  9. the `PB_POS_RECHECK_MS` `cart_bus_check()` → `DOSE_ABORT_POS`

  **Both flow rules are armed on ELAPSED TIME, never on `got`.** The design armed the stall rule on `got >= PB_PRIME_MIN_PULSES`, so with zero flow it never armed at all — and `prime` suppressed the other one. That made `pump 60000 prime`, which is bring-up 7a's own command and `calib`'s implementation, an unconditional sixty-second dry run with **no no-flow abort at all**: one of DECISIONS #10's three mandatory measures, entirely absent, from a single typed line. `prime` now **EXTENDS** the window to `PB_PRIME_LONG_MS` and caps the whole dose at `PB_PRIME_CAP_MS` regardless of the typed ms (task 17 wrote both clamps; this task is what makes them mean something).

  **And testing `got >= target` above the rate rules would ack water that never moved.** A D2 storming at the ISR's own 2 kHz ceiling reaches a 250 ml target (1250 pulses at cfg = 5000) in about 625 ms: `DOSE_OK`, `flow_ml=250` acked, the daily cap charged, `2*flow_ml < ml` never fires, and the pot silently gets nothing while every guard reports healthy. The rate estimator's window is `PB_FLOW_RATE_WINDOW_MS = 100 ms`, so a 2 kHz storm is visible inside ~0.1 s against a ≥ 0.6 s target — it wins the race by a factor of six, and that is why the window has a number rather than a description.

**Spec sections to read in full before starting:** §2.8's loop and its three eye-checkable properties; §2.14's plausibility ceiling in full; §15.1.

**Tests:** `test_prime_abort_fires_when_nothing_flows_in_the_prime_window`, `test_prime_flag_still_aborts_when_nothing_ever_flows`, `test_prime_flag_caps_the_dose_at_the_prime_cap`, `test_stall_abort_is_armed_on_time_not_on_pulses`, `test_dose_aborts_when_the_pulse_rate_exceeds_the_meter_rating`, `test_a_storm_that_begins_AT_PUMP_ON_aborts_before_the_target_is_reached`, `test_the_rate_rules_are_evaluated_above_the_target_rule`, `test_a_dose_that_reaches_target_implausibly_fast_is_noise_not_ok`, `test_five_spurious_edges_at_start_do_not_disable_the_abort`, `test_dose_stops_within_one_iteration_when_the_float_drops`, `test_dose_aborts_when_the_expander_read_fails_mid_dose`, `test_stop_typed_mid_dose_stops_it_within_one_iteration`, `test_dry_on_typed_mid_dose_stops_it`, `test_watchdog_is_fed_on_every_iteration_of_the_dose_loop`.

**Deliverable:** `pio test -e native -f test_dose` passes fourteen further cases, and `pio test -e native_measured -f test_dose` passes the plausibility case that is compiled out at `PB_ML_PER_S_MEASURED == 0`. `test_a_storm_that_begins_AT_PUMP_ON_aborts_before_the_target_is_reached` **must start its storm at pump-on, not before it**: the pre-dose `PB_FLOW_IDLE_MAX_HZ` guard only catches a storm that was already running, and the scenario that matters — a floating D2 running beside the 12 V pump leg — is a storm that begins **with** the pump. The two mid-dose console cases drive real bytes through `sim_serial_rx()` while the loop is spinning, never by poking a flag. This task also makes four more exit-path arms drivable and raises `PB_DRIVABLE_RESULTS` from 14 to 18 — `DOSE_ABORT_NOFLOW`, `DOSE_ABORT_NOISE`, `DOSE_ABORT_FLOAT` and `DOSE_ABORT_POS` all become reachable once this loop lands, while `DOSE_ABORT_STOP` was already drivable at task 17 and `DOSE_REFUSED_CONTRA` waits for task 19. There are no `TEST_IGNORE_MESSAGE` arms in that fixture to delete: an ignore inside its loop would longjmp out of the whole case (task 17 step 1).

---

1. - [ ] **Give the fake a storm that begins with the pump, and the two other pump-relative injectors this task and task 19 need.** `sim_flow_storm(hz)` starts immediately, which is the storm the pre-dose guard already catches — a test built on it proves the wrong thing. **All three declarations go into `include/sim.h` in this one edit**, so that no later task has to introduce a second spelling of one of them:

   ```c
   /* A storm that begins on the ON write, not before it. The pre-dose PB_FLOW_IDLE_MAX_HZ
      guard cannot see this one, and it is the scenario that matters: a floating D2 running
      beside the 12 V pump leg starts counting garbage WHEN THE PUMP DOES. */
   void sim_flow_storm_at_pump_on(uint32_t hz);

   /* Deliver exactly n flow pulses immediately after the ON write, then nothing, ever. The
      stall rule's fixture: a dose that flowed briefly and stopped (step 7). */
   void sim_set_flow_burst_pulses(uint32_t n);

   /* Scheduled injectors, both measured from the next pump-on: apply the change when the
      fake's clock reaches `ms` into the dose. Step 8's two mid-dose console cases use the
      second; task 19 step 4 uses both, and must not add a second spelling of either. */
   void sim_set_float_at_ms(uint32_t ms, bool ok);
   void sim_serial_rx_at_ms(uint32_t ms, const char *s);
   ```

   And the four bodies, in `src/hal_sim.cpp`. **All four are armed by the ON write and none of
   them can fire before a dose starts** — that is the whole point; a storm that was already
   running is what the pre-dose `PB_FLOW_IDLE_MAX_HZ` guard catches, and a fixture built on it
   proves the wrong thing.

   State, beside the other fake-rig statics:

   ```c
   /* The four pump-relative injectors (task 18 step 1). Each scheduled one is an (armed,
      offset, payload) triple PLUS a separate live deadline, so that a second dose in the
      same test re-schedules from ITS pump-on rather than from the first one's. */
   static bool     g_storm_on_armed; static uint32_t g_storm_on_hz;
   static uint32_t g_burst_left;                     /* flow pulses still owed */
   static bool     g_float_at_armed; static uint32_t g_float_at_off_ms; static bool g_float_at_ok;
   static bool     g_float_due;      static uint32_t g_float_due_us;
   static bool     g_rx_at_armed;    static uint32_t g_rx_at_off_ms;
   static char     g_rx_at_buf[64];
   static bool     g_rx_due;         static uint32_t g_rx_due_us;
   ```

   The injectors themselves, beside `sim_flow_storm()`:

   ```c
   void sim_flow_storm_at_pump_on(uint32_t hz) {
     g_storm_on_armed = (hz != 0u);
     g_storm_on_hz    = hz;
   }
   void sim_set_flow_burst_pulses(uint32_t n) { g_burst_left = n; }

   void sim_set_float_at_ms(uint32_t ms, bool ok) {
     g_float_at_armed  = true;
     g_float_at_off_ms = ms;
     g_float_at_ok     = ok;
   }
   void sim_serial_rx_at_ms(uint32_t ms, const char *s) {
     size_t n = strlen(s);
     if (n >= sizeof g_rx_at_buf) n = sizeof g_rx_at_buf - 1u;
     memcpy(g_rx_at_buf, s, n);
     g_rx_at_buf[n]   = '\0';
     g_rx_at_armed    = true;
     g_rx_at_off_ms   = ms;
   }
   ```

   The arming, inside `hal_pump_write()` — the ON arm is the one task 6 already uses for
   `g_pump_on_at_ms`, so this is two more lines in an `if` that exists:

   ```c
   void hal_pump_write(bool on) {
     ev_(SIM_EV_PUMP_WRITE, SIM_PUMP_PIN, SIM_PFS_DIR_OUT | (on ? SIM_PFS_LEVEL_HI : 0u));
     if (on && !g_pump_on) {
       g_pump_on_at_ms = g_ms;                       /* task 6's prime delay runs from here */
       if (g_storm_on_armed) g_storm_hz = g_storm_on_hz;   /* the storm begins WITH the pump */
       if (g_float_at_armed) { g_float_due = true; g_float_due_us = g_us + g_float_at_off_ms * 1000u; }
       if (g_rx_at_armed)    { g_rx_due    = true; g_rx_due_us    = g_us + g_rx_at_off_ms * 1000u; }
     }
     if (!on && g_pump_on) {
       if (g_storm_on_armed) g_storm_hz = 0u;        /* and ends with it, as the 12 V leg does */
       g_burst_left = 0u;
     }
     g_pump_on = on;
   }
   ```

   The burst and the two deadlines, inside `advance_1ms_()`. The burst is a **count**, not a
   rate, so it is emitted one edge per millisecond step: the flow ISR's per-pin minimum-gap
   reject would drop several edges landing in the same microsecond, and n pulses at 1 ms apart
   is well inside every prime window these cases use.

   ```c
   /* one edge per 1 ms step, at the step's own microsecond, so the ISR's gap reject accepts
      every one of them. Only while the pump is on: a burst is what the meter saw, not what
      the fake felt like emitting. */
   static void emit_burst_(uint32_t target_us) {
     if (!g_pump_on || g_burst_left == 0u) return;
     g_us = target_us;
     --g_burst_left;
     pulses_isr_flow();
   }

   static void advance_1ms_(void) {
     const uint32_t target = g_us + 1000u;
     tick_models_(1000u);
     emit_(sim_flow_hz_(),  &g_next_flow_us,  target, pulses_isr_flow);
     emit_(sim_screw_hz_(), &g_next_screw_us, target, pulses_isr_screw);
     emit_burst_(target);
     g_us = target;
     g_ms = g_us / 1000u;
     /* the two scheduled injectors, applied at the step that reaches their deadline */
     if (g_float_due && g_us >= g_float_due_us) { g_float_due = false; g_float_ok = g_float_at_ok; }
     if (g_rx_due    && g_us >= g_rx_due_us)    { g_rx_due    = false; sim_serial_rx(g_rx_at_buf); }
   }
   ```

   And in `sim_reset()`, beside the other counters — **all four, or a test leaves the next one
   armed**:

   ```c
     g_storm_on_armed = false; g_storm_on_hz = 0u; g_burst_left = 0u;
     g_float_at_armed = false; g_float_due = false;
     g_rx_at_armed    = false; g_rx_due    = false; g_rx_at_buf[0] = '\0';
   ```

2. - [ ] **Write the case that fixes the ORDER, and watch it fail against task 17's loop.**

   ```cpp
   /* §2.14. The rate rules are evaluated ABOVE the target rule, always. With the target rule
      first, a D2 at the ISR's own 2 kHz ceiling reaches a 250 ml target in about 625 ms and
      dose_run() returns DOSE_OK with flow_ml=250 for water that never moved. */
   void test_the_rate_rules_are_evaluated_above_the_target_rule(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true);
     TEST_ASSERT_TRUE(cfg_pulses_per_l_set(5000u));
     sim_flow_storm_at_pump_on(2000u);
     dose_req_t q = {0}; q.ml = 250u; q.cap_ms = PB_DOSE_CAP_MS_MAX; q.need_pos = false;
     TEST_ASSERT_EQUAL(DOSE_ABORT_NOISE, dose_run(&q));
     TEST_ASSERT_NOT_EQUAL(DOSE_OK, dose_last_result());
   }

   /* The same storm, stated as the consequence rather than the mechanism, because this is
      the sentence that has to stay true: no target is ever reached by noise. */
   void test_a_storm_that_begins_AT_PUMP_ON_aborts_before_the_target_is_reached(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true);
     TEST_ASSERT_TRUE(cfg_pulses_per_l_set(5000u));
     TEST_ASSERT_EQUAL_UINT32(0u, pulses_flow_rate());   /* NOT storming before the dose: the
                                                            idle guard must not be what fires */
     sim_flow_storm_at_pump_on(2000u);
     dose_req_t q = {0}; q.ml = 250u; q.cap_ms = PB_DOSE_CAP_MS_MAX;
     TEST_ASSERT_EQUAL(DOSE_ABORT_NOISE, dose_run(&q));
     TEST_ASSERT_EQUAL_UINT16(0u, dose_flow_ml() > 250u ? 1u : 0u);   /* nothing was acked */
   }
   ```

3. - [ ] **Run it and read the failure carefully — it is the finding.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose
   ```
   expected: `FAIL: Expected 15 Was 0` — `DOSE_ABORT_NOISE` against `DOSE_OK`. Two hundred and
   fifty millilitres reported as delivered and the pot charged for them, with nothing anywhere
   saying otherwise.

4. - [ ] **Write the loop body's final shape**, replacing the two comment lines and the `(void)last_bus;` that task 17 left inside `dose_run()`'s `for(;;)`. The ORDER below is the whole of this task; nothing above the ON write or below the OFF write changes.

   ```c
     for (;;) {
       safety_tick();                              /* fed on EVERY iteration */
       uint32_t now = hal_millis();
       el  = now - t0;                             /* unsigned diff: rollover-safe */
       got = pulses_flow() - flow0;
       if (got != last_got) { last_got = got; last_edge = now; }

       /* 1. BOTH NOISE RULES COME FIRST, ABOVE THE TARGET RULE. A D2 storming at the ISR's
             own 2 kHz ceiling reaches a 250 ml target (1250 pulses at cfg = 5000) in ~625 ms;
             testing `got >= target` first would ack DOSE_OK and flow_ml=250 for water that
             never moved. The estimator's window is 100 ms, so the storm is visible in ~0.1 s
             against a >= 0.6 s target. The pre-dose PB_FLOW_IDLE_MAX_HZ guard only catches a
             storm that was ALREADY running; this catches one that starts with the pump. */
       if (pulses_flow_rate() > PB_FLOW_MAX_HZ)           { r = DOSE_ABORT_NOISE;  break; }
   #if PB_ML_PER_S_MEASURED > 0
       /* 2. Delivered-vs-elapsed plausibility on the DOSE_OK path: reaching the target in far
             less time than the rig can physically deliver it is noise, not a fast pump. Armed
             only once bring-up 7b commits the rate. */
       if (target && got >= target &&
           el * PB_ML_PER_S_MEASURED * PB_PLAUS_NUM < (uint32_t)q->ml * 1000u * PB_PLAUS_DEN)
                                                          { r = DOSE_ABORT_NOISE;  break; }
   #endif
       if (target && got >= target)                       { r = DOSE_OK;           break; }
       if (el >= cap_ms)                                  { r = DOSE_ABORT_CAP;    break; }

       /* 5. PRIME: nothing at all came out in the prime window. `prime` EXTENDS it (task 17
             sets prime_ms to PB_PRIME_LONG_MS); it never removes it.
          6. STALL: armed on TIME, not on `got`, so zero flow can never disarm it. Arming it
             on got >= PB_PRIME_MIN_PULSES - the design's own form - meant that a dose with no
             flow at all never armed it, and `prime` suppressed rule 5, which made
             `pump 60000 prime` an unconditional sixty-second dry run. */
       if (el >= prime_ms && got < PB_PRIME_MIN_PULSES)    { r = DOSE_ABORT_NOFLOW; break; }
       if (el >= prime_ms && (now - last_edge) >= g_stall_ms)
                                                          { r = DOSE_ABORT_NOFLOW; break; }

       /* 7. the float: ONE bad sample aborts. That direction is dry (§2.10). */
       if (hal_pin_read(PIN_HALL_FLOAT) != PB_LOW)        { r = DOSE_ABORT_FLOAT;  break; }
       /* 8. the console's last-resort abort (§2.12). */
       if (cli_stop_requested())                          { r = DOSE_ABORT_STOP;   break; }
       /* 9. the README's "I2C hung, home hall unreadable" row: a LIVE expander read, at most
             once per PB_POS_RECHECK_MS, inside the dose. */
       if ((now - last_bus) >= PB_POS_RECHECK_MS) {
         last_bus = now;
         if (!cart_bus_check())                           { r = DOSE_ABORT_POS;    break; }
       }
       /* Task 20 appends the `hang` hook BELOW this line, so every abort rule above it is
          evaluated before the dog is deliberately starved. */
     }
   ```

5. - [ ] **Run and watch the two order cases pass.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_dose && pio test -e native_measured -f test_dose
   ```

6. - [ ] **Commit the order before writing the other twelve cases**, so the finding is a commit of its own rather than a line inside a larger one.

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && git add src/safety.cpp include/sim.h src/hal_sim.cpp test/test_dose/test_dose.cpp && git commit -m "dose loop: the rate rules are evaluated above the target rule

   A D2 storming at the ISR's own 2 kHz ceiling reaches a 250 ml target in about 625 ms.
   With the target rule first, dose_run() returned DOSE_OK with flow_ml=250 for water that
   never moved: the pot charged, the daily cap spent, the cooldown set, and butler's own
   2*flow_ml < ml alert silent because flow_ml said 250. The scenario is a floating D2
   beside the 12 V pump leg, and the pre-dose idle-rate guard cannot see it because the
   storm begins WITH the pump - sim_flow_storm_at_pump_on() exists for exactly that.

   Both flow rules are armed on elapsed time, never on pulses delivered. Arming the stall
   rule on got >= PB_PRIME_MIN_PULSES meant that with zero flow it never armed at all, and
   prime suppressed the other one - so pump 60000 prime, bring-up 7a's own command and
   calib's implementation, was an unconditional sixty-second dry run with the no-flow abort,
   one of DECISIONS #10's three mandatory measures, entirely absent. prime now extends the
   window to 15 s and caps the whole dose at 20 s."
   ```

7. - [ ] **Write the prime and stall cases.** These four are the no-flow abort, and the third of them is the one that used to be inverted.

   ```cpp
   void test_prime_abort_fires_when_nothing_flows_in_the_prime_window(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true); sim_set_flow_ml_s(0);         /* the pump runs; nothing moves */
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
     TEST_ASSERT_EQUAL(DOSE_ABORT_NOFLOW, dose_run(&q));
     TEST_ASSERT_TRUE(sim_pump_on_ms() < PB_PRIME_MS_DEFAULT + 200u);
   }

   /* The case that replaces the design's inverted one. `prime` EXTENDS the window; a dose
      that never flows still aborts, at PB_PRIME_LONG_MS rather than never. */
   void test_prime_flag_still_aborts_when_nothing_ever_flows(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true); sim_set_flow_ml_s(0);
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 60000u; q.long_prime = true;
     TEST_ASSERT_EQUAL(DOSE_ABORT_NOFLOW, dose_run(&q));
     TEST_ASSERT_TRUE(sim_pump_on_ms() >= PB_PRIME_LONG_MS);        /* the window extended */
     TEST_ASSERT_TRUE(sim_pump_on_ms() <  PB_PRIME_CAP_MS + 500u);  /* and it still ended */
   }

   void test_prime_flag_caps_the_dose_at_the_prime_cap(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true); sim_set_flow_ml_s(30);        /* flowing, so no no-flow abort */
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 60000u; q.long_prime = true;
     TEST_ASSERT_EQUAL(DOSE_ABORT_CAP, dose_run(&q));
     TEST_ASSERT_TRUE(sim_pump_on_ms() <= PB_PRIME_CAP_MS + 200u);  /* NOT the typed 60 s */
   }

   /* Armed on TIME. A dose that delivered four pulses and then stopped must still abort:
      arming on `got` is what let zero flow disarm the rule entirely. */
   void test_stall_abort_is_armed_on_time_not_on_pulses(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true);
     sim_set_flow_burst_pulses(PB_PRIME_MIN_PULSES + 2u);   /* then nothing, forever */
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
     TEST_ASSERT_EQUAL(DOSE_ABORT_NOFLOW, dose_run(&q));
     TEST_ASSERT_TRUE(sim_pump_on_ms() <
                      PB_PRIME_MS_DEFAULT + PB_STALL_MS_DEFAULT + 500u);
   }

   /* The prime rule's boundary, from the other side: exactly one pulse short of the
      threshold must NOT be read as "flow started". */
   void test_five_spurious_edges_at_start_do_not_disable_the_abort(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true);
     sim_set_flow_burst_pulses(PB_PRIME_MIN_PULSES - 1u);
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_DOSE_CAP_MS_MAX;
     TEST_ASSERT_EQUAL(DOSE_ABORT_NOFLOW, dose_run(&q));
   }
   ```

   `sim_set_flow_burst_pulses(n)` was declared in step 1 alongside `sim_flow_storm_at_pump_on()`;
   this is its first consumer. Mention it in the commit message.

8. - [ ] **Write the remaining cases**, two at a time, in the same rhythm. Five of them need a word first:

   - `test_dose_aborts_when_the_pulse_rate_exceeds_the_meter_rating` is the plain rule-1 case,
     and it is NOT one of the two storm cases of step 2 — those test the rule's *order* and a
     storm arriving at pump-on. This one tests the ceiling itself against the YF-S401's rating:
     `sim_flow_storm_at_pump_on(PB_FLOW_MAX_HZ + 200u)` (not 2 kHz), a `by_time` dose with
     `cap_ms = PB_DOSE_CAP_MS_MAX`, and `TEST_ASSERT_EQUAL(DOSE_ABORT_NOISE, dose_run(&q))`
     with `TEST_ASSERT_TRUE(sim_pump_on_ms() < 1000u)` — a rate above the meter's rating is
     not a fast pump, it is a meter that is not measuring water, and the estimator's 100 ms
     window is what makes that verdict arrive inside a second.
   - `test_a_dose_that_reaches_target_implausibly_fast_is_noise_not_ok` is `#if PB_ML_PER_S_MEASURED > 0` and runs under `native_measured`. At 30 ml/s a 120 ml dose honestly needs 4000 ms; deliver the target inside 900 ms and the result must be `DOSE_ABORT_NOISE`, not `DOSE_OK`. On the other arm, `TEST_IGNORE_MESSAGE("PB_ML_PER_S_MEASURED == 0: the rule is compiled out")` — and that is honest, because until bring-up 7b commits the rate the rule genuinely does not exist.
   - `test_dose_stops_within_one_iteration_when_the_float_drops` asserts `DOSE_ABORT_FLOAT` and that `sim_pump_on_ms()` is within one loop iteration of the moment `sim_set_float(false)` was applied. This is bring-up 5b in software; **5b's own pass criterion also includes `contra=0` afterwards**, because the float dropping is the two sensors *agreeing*, not contradicting (task 19).
   - `test_dose_aborts_when_the_expander_read_fails_mid_dose` sets `sim_set_i2c_fail(true)` after the dose has started and asserts `DOSE_ABORT_POS` within `PB_POS_RECHECK_MS` plus one iteration. `hal_i2c_recover()` refuses while `g_dosing` (§2.13), so the bus stays broken for the rest of the dose — which is the point: A4/A5 are the mux select lines **and** the home hall, the input that gates the pump.
   - `test_stop_typed_mid_dose_stops_it_within_one_iteration` and `test_dry_on_typed_mid_dose_stops_it` push real bytes through `sim_serial_rx()` while the loop is spinning — the fake's serial injector is time-driven, so schedule the bytes at a millisecond inside the dose. The second one additionally asserts `safety_dry()` afterwards: `dry on` means the same thing during a dose as before one.

   `test_watchdog_is_fed_on_every_iteration_of_the_dose_loop` reads `sim_events()` and asserts
   no gap between consecutive `SIM_EV_WDT_FEED` entries exceeds the fake's tick:

   ```cpp
   /* This is what makes a 60 s dose legal under a 5592 ms grant (§3), and it is the first
      assertion that fails if anyone ever adds a `continue` to the loop body. */
   void test_watchdog_is_fed_on_every_iteration_of_the_dose_loop(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true); sim_set_flow_ml_s(30);
     sim_events_clear();
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 5000u;
     (void)dose_run(&q);
     const sim_ev_t *ev; size_t n = sim_events(&ev);
     uint32_t prev = 0, feeds = 0; bool first = true;
     for (size_t i = 0; i < n; ++i) {
       if (ev[i].kind != SIM_EV_WDT_FEED) continue;
       if (!first) TEST_ASSERT_TRUE_MESSAGE(ev[i].at_ms - prev <= 3u, "unfed span in the dose loop");
       prev = ev[i].at_ms; first = false; feeds++;
     }
     TEST_ASSERT_TRUE(feeds > 100u);
   }
   ```

9. - [ ] **Make four more exit-path arms drivable, and raise `PB_DRIVABLE_RESULTS` from 14 to 18.** `DOSE_ABORT_NOFLOW`, `DOSE_ABORT_NOISE`, `DOSE_ABORT_FLOAT` and `DOSE_ABORT_POS` all become reachable with this task's rules 5, 6 and 7 (`DOSE_ABORT_STOP` was already drivable at task 17); `pb_drive_dose_to_result()` gets a real arm for each — `sim_flow_storm_at_pump_on(2000)`, `sim_set_i2c_fail(true)` after pump-on, and a scheduled `sim_serial_rx("stop\n")`. Only `DOSE_REFUSED_CONTRA` stays ignored, and task 19 deletes that one.

10. - [ ] **Run everything, run the gates, and commit.**

    ```bash
    cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native && pio test -e native_measured -f test_dose && \
      make check && pio run -e uno_r4_wifi -e uno_r4_wifi_bringup 2>&1 | tail -3
    ```
    expected: `0 Failures` from both, `all invariants hold` — including the
    `for\s*\(\s*;\s*;\s*\)` grep, which pins the program's only intentional unbounded loop to
    this one function — and two `SUCCESS` lines.

    ```bash
    cd /Users/jcanton/projects/plant-butler/firmware && git add src/safety.cpp include/sim.h src/hal_sim.cpp test/test_dose/test_dose.cpp && git commit -m "dose loop: prime, stall, the plausibility ceiling, the float, the console and the bus

    Nine rules, in one order, and the order is the specification. The two flow rules are
    armed on elapsed time so that zero flow cannot disarm them; the float aborts on a single
    bad sample because that direction is dry; the expander is read LIVE inside the loop
    every second, because A4/A5 carry the mux select lines and the home hall - the input
    that gates the pump - and a bus that wedges mid-dose is the README's own row.

    The plausibility test is compiled out until bring-up 7b commits PB_ML_PER_S_MEASURED,
    and its test says so rather than passing vacuously: reaching a target in under a
    quarter of the time the rig can physically deliver it is noise, not a fast pump.

    The exit-path test's ignores for ABORT_NOISE, ABORT_POS and ABORT_STOP are deleted -
    all three are reachable now, and each has a real arm in the fixture."
    ```

11. - [ ] **What this task does not prove.** Every rule here fires against a fake meter. **Bring-up 7a** is where the prime rule meets a line that has never held water; **7b** is where the stall window and the prime window get their real numbers (`PB_PRIME_MS_DEFAULT`, `PB_STALL_MS_DEFAULT`) and where `PB_ML_PER_S_MEASURED` arms the plausibility rule and the cap clamp at all; **5b** is where pulling the float hall mid-dose must abort within one iteration on real silicon. Nothing on the host tells you whether the YF-S401 pulses at the rig's actual flow — that is 7b's verdict, and `-DPB_DOSE_BY_TIME=1` is the stated fallback if it does not.

---

---

### Task 19: The float/flow contradiction latch

**Drop 2.**

> **Read spec §2.7 in full before the first line.** This is the strongest refusal in the program: two independent sensors contradicting each other, and the answer is to stop the rig until a human looks.

**Files:**
- Create: `test/test_contra/test_contra.cpp`
- Modify: `include/safety.h`, `src/safety.cpp` (`dose_end_ml_()` and the refusal ladder), `src/cli.cpp` (`clear contra`, and `status` gains `contra=` and the banner), `src/ui.cpp` (the LCD banner rows), `test/support/harness.h` (`pb_latch_contra()`), `test/test_cli/test_cli.cpp`, `test/test_dose/test_dose.cpp`
- Test: `test/test_contra/test_contra.cpp`, `test/test_cli/test_cli.cpp`

**Interfaces:**
- Consumes: `g_nv` and `void noinit_commit(void)`, `bool noinit_was_cold(void)` from task 4; `int hal_pin_read(uint8_t)` and `PIN_HALL_FLOAT` from tasks 2-3; `dose_end_ml_(r, got_pulses, elapsed_ms, outlet, prime_ms, long_prime)` from task 17; `bool cart_home(void)` from task 14; `void safety_set_err(const char *)` from task 17.
- Produces — `include/safety.h` gains:

  ```c
  bool safety_contra(void);        /* task 17's ladder already calls this against a stub */
  bool safety_contra_clear(void);  /* true if it WAS latched; the console's only way back */
  ```

  and `test/support/harness.h` gains `pb_latch_contra()`. **There is deliberately no `safety_contra_set_()`.** The latch is settable in exactly one place — `dose_end_ml_()` — and a test hook that set it directly would be a second setter, which is the very thing this design exists to prevent. The fixture *earns* the latch by driving a real latching dose through the real `dose_run()`, which is also what makes `test_dose_refused_when_the_contradiction_latch_is_set` and task 26's `test_boot_self_home_runs_under_both_latches` mean anything.

**Where it is set, cleared, checked and surfaced.**

- **SET** in exactly one place, inside `dose_end_ml_()`, under spec §2.7's five conditions, each doing one job.
- **CLEARED** in exactly one place: the console command `clear contra` — two literal tokens, no abbreviation, present in **both** the `bench` and `bringup` builds, because it is the only way back and the unattended binary can latch. It clears on nothing else: no timer, no successful anything, no backend command, no `dry off`.
- **CHECKED** in `dose_run()`'s ladder *above* the dry latch, so the more specific reason is the one reported.
- It refuses **every dose and nothing else**. `cart_home()` and the boot self-home run under it (spec §2.0, §2.9, §2.11): parking the cart off every gate is what you want after a latch, not something to withhold, because a cart left over outlet N holds that gate open under the reservoir head for as long as the latch stands.
- **SURFACED** as `err=contra`, `ch207=1` and `float=0` (task 22), a `status` banner, and LCD rows `CONTRA LATCH` / `float ok,no flow`.
- A kinked hose with a full tank latches too, **deliberately**: the latch does not diagnose, it refuses. A kinked hose and a stuck float both stop the rig until a human looks, and stopping the rig is the right answer to both.

**Spec sections to read in full before starting:** §2.0; §2.7 in full; §2.11; §4.1's `float=` row; §5's LCD rows; §13 steps 5b and 7a.

**Tests:** `test_latch_sets_when_the_float_said_ok_and_no_pulse_ever_arrived`, `test_latch_does_not_set_when_the_float_dropped_mid_dose`, `test_latch_does_not_set_when_flow_started_and_then_stalled`, `test_latch_does_not_set_when_the_dose_was_stopped_before_the_prime_window`, `test_latch_uses_the_doses_own_prime_window_not_the_configured_default`, `test_latch_does_not_set_for_a_console_prime_dose`, `test_latch_does_not_set_for_a_dose_that_was_refused`, `test_latch_refuses_every_subsequent_dose_including_a_console_one`, `test_latch_does_not_refuse_homing`, `test_latch_survives_a_warm_reset_and_not_a_cold_one`, **`test_latch_reports_err_contra_and_ch207_and_float_zero`**, `test_latch_clears_only_on_the_literal_two_token_command`, `test_latch_is_not_cleared_by_dry_off_or_by_a_successful_home`, `test_dose_refused_when_the_contradiction_latch_is_set`, `test_clear_requires_both_literal_tokens`.

Fourteen of those fifteen are written here. **`test_latch_reports_err_contra_and_ch207_and_float_zero` is written in task 26 step 11**, in this same file, because it asserts the latch's three *wire* surfaces and `report_build()` does not exist until task 22 — one drop later. It is named here so nobody reads this list as complete, and task 26 step 11 is where it is owed.

**Deliverable:** `pio test -e native -f test_contra` passes **thirteen** cases in a suite of its own (fourteen once task 26 step 11 adds the wire case). The fifteenth name in the list above, `test_clear_requires_both_literal_tokens`, is written by step 6 into `test/test_cli/test_cli.cpp` and not into this suite, because it exercises the console's line matcher and not the latch. This task also adds `test_the_ladder_reports_contra_above_dry` to `test/test_dose/test_dose.cpp` (step 5), which is task 17's twenty-fifth case rather than one of this suite's. And `pio test -e native_bench -f test_cli` proves `clear contra` is a command in the **bench** binary too. Record in the commit message that the firmware half of this latch is **not durable** — `.noinit` buys the warm-reset case only, a power cycle silently rearms the rig, and whether it survives even a watchdog reset is unproven until bring-up 7c′ — so spec §16.5.3, the backend's durable half, is a **requirement rather than a follow-up**. Record also that until §16.5.4 lands the phone says "reservoir empty" whatever the cause, because the `float:` alert text is fixed at `butler.py`'s `float:` rule, and that the alert arrives on the **second** report after the latch, because the raise needs two bad sightings inside the flap window.

---

1. - [ ] **Write the fixture that earns the latch, then the suite with one case.** In
   `test/support/harness.h` — **first extend its include list**, which task 3 step 7 wrote as
   `<unity.h>`, `hal.h`, `sim.h`. The helper below consumes `dose_req_t`, `dose_run()` and
   `safety_contra()` (safety.h) and `PB_BOOT_GAP_MS`, `PB_PRIME_MS_DEFAULT`,
   `PB_STALL_MS_DEFAULT` (config.h), and neither is reachable transitively: `safety.h`,
   `sim.h`, `netfsm.h` and `report.h` all include only `<stdbool.h>`, `<stdint.h>` and
   `<stddef.h>`. Every suite includes `harness.h` first, so a missing one here is a compile
   error in **all eight** suites at once.

   ```c
   #include "config.h"
   #include "safety.h"
   ```

   then, below the three helpers that differ between host and board:

   ```c
   /* Drive a REAL latching dose: the float says OK, nothing ever flows, the dose runs past
      its own prime window, and it is not a console prime. That is §2.7's five conditions,
      and it is the ONLY way the latch can be set -- there is no setter, on purpose. */
   static inline void pb_latch_contra(void) {
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true);
     sim_set_flow_ml_s(0);                  /* float OK, no flow: the contradiction */
     dose_req_t q = {0};
     q.by_time    = true;
     q.cap_ms     = PB_PRIME_MS_DEFAULT + PB_STALL_MS_DEFAULT + 1000u;
     q.long_prime = false;                  /* a console prime is EXEMPT (§2.7) */
     (void)dose_run(&q);
     TEST_ASSERT_TRUE_MESSAGE(safety_contra(), "pb_latch_contra did not latch");
   }
   ```

   and create `test/test_contra/test_contra.cpp`:

   ```cpp
   #include <unity.h>
   #include <string.h>
   #include "../support/harness.h"
   #include "cart.h"
   #include "cli.h"
   #include "config.h"
   #include "safety.h"
   #include "sim.h"

   void setUp(void)    { pb_test_setup(); }
   void tearDown(void) { pb_test_teardown(); }

   /* §2.7. The float said OK - permission granted by the one input whose whole design is that
      failure reads as refusal - and the dose that permission authorised produced no flow at
      all. Two independent sensors contradict each other. The safe reading is that the tank is
      empty and the float is stuck, and the safe response is not "end this dose and let the
      next one start" but refuse everything until a human looks. */
   void test_latch_sets_when_the_float_said_ok_and_no_pulse_ever_arrived(void) {
     TEST_ASSERT_FALSE(safety_contra());
     pb_latch_contra();
     TEST_ASSERT_TRUE(safety_contra());
   }

   int main(void) {
     UNITY_BEGIN();
     RUN_TEST(test_latch_sets_when_the_float_said_ok_and_no_pulse_ever_arrived);
     return UNITY_END();
   }
   ```

2. - [ ] **Run it and watch it fail.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_contra
   ```
   expected: `undefined reference to 'safety_contra_clear()'` if you added the declaration
   first, or `pb_latch_contra did not latch` against task 17's stub `safety_contra()`. Either
   failure is the right one to be starting from.

3. - [ ] **Write the latch's one setter, inside `dose_end_ml_()` and nowhere else.** Task 17 wrote that function with `(void)prime_ms; (void)long_prime;` — delete those two casts; this is what they were for. Write the comment beside each condition, because a reader deleting one of them will not otherwise know what it was for:

   ```c
     /* §2.7. THE ONE PLACE THE LATCH IS SET. Five conditions, each doing one job:
          g_float_granted      this was a PERMITTED dose -- a refusal proves nothing about flow;
          !long_prime          a console `pump <ms> prime` of a dry line is where "float OK, no
                               flow" is the EXPECTED result: priming a dry line IS running the
                               pump into air on purpose, and without this exemption bring-up 7a
                               would latch on its very first attempt and §13's "run it again"
                               would be wrong, because the second run would return
                               DOSE_REFUSED_CONTRA;
          float still OK       if it dropped, the two sensors AGREE (the tank ran out) and this
                               is an ordinary DOSE_ABORT_FLOAT;
          got_pulses == 0      NOTHING at all, ever -- a dose that flowed and then stalled has
                               got > 0, and that is an ordinary DOSE_ABORT_NOFLOW;
          elapsed >= prime_ms  THIS dose's effective prime window, not the configured default:
                               a `stop` typed 200 ms in is not evidence of anything. */
     if (g_float_granted && !long_prime &&
         hal_pin_read(PIN_HALL_FLOAT) == PB_LOW &&
         got_pulses == 0u && elapsed_ms >= prime_ms) {
       g_nv.contra_latched = true;
       noinit_commit();
       safety_set_err("contra");      /* AFTER err_of(r): the latch is the louder fact */
     }
   ```

   Note the placement: it runs **after** `g_last_err = err_of(r)`, so `err=contra` wins over
   the `noflow` the abort itself would have reported. That is deliberate — the abort is the
   symptom and the latch is the finding — and it is why the assignment is `safety_set_err()`
   rather than a second `g_last_err =` line.

   Then replace task 17's stub with the real pair:

   ```c
   bool safety_contra(void) { return g_nv.contra_latched; }

   /* THE ONLY CLEAR. No timer, no successful anything, no backend command, no `dry off`. */
   bool safety_contra_clear(void) {
     bool was = g_nv.contra_latched;
     g_nv.contra_latched = false;
     noinit_commit();
     return was;
   }
   ```

4. - [ ] **Run and watch the first case pass.** Then write the six "does not set" cases (the Tests list names six; an earlier draft said five), two at a time, each run before and after. Each is `pb_latch_contra()`'s scenario with exactly one condition changed, and each ends `TEST_ASSERT_FALSE(safety_contra())`:

   ```cpp
   /* The float dropped: the two sensors AGREE that the tank ran out. Ordinary abort. */
   void test_latch_does_not_set_when_the_float_dropped_mid_dose(void) {
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true); sim_set_flow_ml_s(0);
     sim_set_float_at_ms(500u, false);          /* the fake drops D5 mid-dose */
     dose_req_t q = {0}; q.by_time = true;
     q.cap_ms = PB_PRIME_MS_DEFAULT + PB_STALL_MS_DEFAULT + 1000u;
     TEST_ASSERT_EQUAL(DOSE_ABORT_FLOAT, dose_run(&q));
     TEST_ASSERT_FALSE(safety_contra());
   }

   /* got > 0: the meter and the float agree that water WAS moving and then stopped - a hose
      off a pot, a tank sucked dry mid-dose. DOSE_ABORT_NOFLOW, ordinary, no latch. */
   void test_latch_does_not_set_when_flow_started_and_then_stalled(void) {
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true);
     sim_set_flow_burst_pulses(20u);            /* flows, then stops */
     dose_req_t q = {0}; q.by_time = true;
     q.cap_ms = PB_PRIME_MS_DEFAULT + PB_STALL_MS_DEFAULT + 1000u;
     TEST_ASSERT_EQUAL(DOSE_ABORT_NOFLOW, dose_run(&q));
     TEST_ASSERT_FALSE(safety_contra());
   }

   /* Stopped before its own prime window: no evidence either way. */
   void test_latch_does_not_set_when_the_dose_was_stopped_before_the_prime_window(void) {
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true); sim_set_flow_ml_s(0);
     sim_serial_rx_at_ms(200u, "stop\n");
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 30000u;
     TEST_ASSERT_EQUAL(DOSE_ABORT_STOP, dose_run(&q));
     TEST_ASSERT_FALSE(safety_contra());
   }

   /* THIS dose's window, not the configured default. A prime dose that ran 5 s - past
      PB_PRIME_MS_DEFAULT (3 s) but inside PB_PRIME_LONG_MS (15 s) - has not yet had its own
      window elapse, so even without the long_prime exemption it must not latch. */
   void test_latch_uses_the_doses_own_prime_window_not_the_configured_default(void) {
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true); sim_set_flow_ml_s(0);
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 5000u; q.long_prime = true;
     (void)dose_run(&q);
     TEST_ASSERT_TRUE(dose_last_ms() > PB_PRIME_MS_DEFAULT);      /* past the DEFAULT window */
     TEST_ASSERT_TRUE(dose_last_ms() < PB_PRIME_LONG_MS);         /* inside its OWN */
     TEST_ASSERT_FALSE(safety_contra());
   }

   /* Bring-up 7a's own command, on a line that has never held water, satisfies every other
      condition on its FIRST attempt. Without this exemption 7a latches immediately and §13's
      instruction to run it again would be wrong. */
   void test_latch_does_not_set_for_a_console_prime_dose(void) {
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true); sim_set_flow_ml_s(0);
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_PRIME_CAP_MS; q.long_prime = true;
     TEST_ASSERT_EQUAL(DOSE_ABORT_NOFLOW, dose_run(&q));
     TEST_ASSERT_FALSE(safety_contra());
   }

   /* A refused dose never reaches dose_end_ml_(), so it can never latch - which is also why
      the float-flap counter of task 15 exists: repeated float REFUSALS are invisible here. */
   void test_latch_does_not_set_for_a_dose_that_was_refused(void) {
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(false);
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 5000u;
     TEST_ASSERT_EQUAL(DOSE_REFUSED_FLOAT, dose_run(&q));
     TEST_ASSERT_FALSE(safety_contra());
   }
   ```

   `sim_set_float_at_ms(ms, ok)` and `sim_serial_rx_at_ms(ms, s)` are **task 18 step 1's**
   scheduled injectors, declared in `include/sim.h` and implemented in `src/hal_sim.cpp` there:
   apply the change when the fake's clock reaches `ms` after the next pump-on. Consume them as
   they stand. Do not add a second spelling of either, and do not re-declare them here.

5. - [ ] **Write the four "what the latch refuses" cases, and the ordering half task 17 owed
   this task.** The most important one is the negative: it refuses **watering**, and nothing
   else.

   First, in `test/test_dose/test_dose.cpp` (not in this suite — it belongs beside the other
   half of the pair, and `test_dose` is in this task's Files list for exactly this), with its
   `RUN_TEST` line:

   ```cpp
   /* The second half of task 17 step 7's ordering pair. It could not be written there: the
      only way to earn the latch is pb_latch_contra(), which this task's step 1 creates.
      An operator reading `err=dry` when the real reason was the contradiction latch pulls
      the tank apart looking for water that is already there. */
   void test_the_ladder_reports_contra_above_dry(void) {
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     pb_latch_contra();
     safety_dry_set(true);                             /* BOTH latches stand */
     pb_advance(PB_DOSE_MIN_GAP_MS + 1u);
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = 1000u;
     TEST_ASSERT_EQUAL(DOSE_REFUSED_CONTRA, dose_run(&q));   /* contra above dry */
     TEST_ASSERT_EQUAL_STRING("contra", safety_last_err());
   }
   ```

   Then the four cases of this suite:

   ```cpp
   void test_dose_refused_when_the_contradiction_latch_is_set(void) {
     pb_latch_contra();
     pb_advance(PB_DOSE_MIN_GAP_MS + 1u);
     sim_set_float(true); sim_set_flow_ml_s(30);
     dose_req_t q = {0}; q.ml = 100u; q.cap_ms = 10000u; q.need_pos = false;
     TEST_ASSERT_EQUAL(DOSE_REFUSED_CONTRA, dose_run(&q));
     TEST_ASSERT_EQUAL_STRING("contra", safety_last_err());
   }

   /* including a CONSOLE one: the exemption is about setting the latch, never about
      escaping it. `pump 20000 prime` under a latch is refused like everything else. */
   void test_latch_refuses_every_subsequent_dose_including_a_console_one(void) {
     pb_latch_contra();
     pb_advance(PB_DOSE_MIN_GAP_MS + 1u);
     sim_set_float(true); sim_set_flow_ml_s(30);
     dose_req_t q = {0}; q.by_time = true; q.cap_ms = PB_PRIME_CAP_MS; q.long_prime = true;
     TEST_ASSERT_EQUAL(DOSE_REFUSED_CONTRA, dose_run(&q));
   }

   /* §2.0, §2.9, §2.11: homing is not watering. A cart left over outlet N holds that gate
      open under the reservoir head for as long as the latch stands, which may be days.
      Parking is MORE wanted after a latch, not less. */
   void test_latch_does_not_refuse_homing(void) {
     pb_latch_contra();
     safety_dry_set(true);                       /* both latches, at once */
     sim_set_screw_pulse_ms(2);
     sim_set_home_region(0u, 40u);
     sim_set_cart_at(600u);
     TEST_ASSERT_TRUE(cart_home());
     TEST_ASSERT_TRUE(cart_parked());
     TEST_ASSERT_TRUE(safety_contra());          /* and homing did NOT clear it */
   }

   /* §2.3: .noinit survives a warm reset - the watchdog reset is precisely the event that
      would otherwise erase the latch - and a cold boot starts clean. */
   void test_latch_survives_a_warm_reset_and_not_a_cold_one(void) {
     pb_latch_contra();
     /* sim_reset() re-enters the boot path (task 3), so the .noinit verify has already run
        when it returns; calling noinit_begin() again would advance the boot counter twice. */
     sim_reset(true);
     TEST_ASSERT_TRUE_MESSAGE(safety_contra(), "the latch did not survive a warm reset");
     sim_reset(false);
     TEST_ASSERT_FALSE_MESSAGE(safety_contra(), "the latch survived a COLD boot");
   }
   ```

6. - [ ] **Write `clear contra` into `src/cli.cpp`, OUTSIDE any `#if PB_BRINGUP`.** It ships in both binaries: the unattended binary can latch, and a rig that cannot be released except by a reflash is worse than one that can. Two literal tokens, no abbreviation; the residual risk — a stray serial line spelling exactly `clear contra` — is accepted and written down in spec §6.

   ```c
     if (strcmp(line, "clear contra") == 0) {     /* two literal tokens, no abbreviation */
       if (safety_contra_clear())
         hal_serial_write("contra cleared - the last dose said float OK and the meter saw "
                          "nothing. If you have not found out why, you have not fixed it.\n");
       else
         hal_serial_write("contra=0 already\n");
       return true;
     }
   ```

   and in `cli_print_status()`:

   ```c
     if (safety_contra())
       hal_serial_write("contra=1 *** CONTRADICTION LATCHED - float said OK, meter saw "
                        "nothing. `clear contra` to release.\n");
     else
       hal_serial_write("contra=0\n");
   ```

   Add `clear contra` to task 11's `test_parses_every_bench_command`, and write the two
   console cases in `test/test_cli/test_cli.cpp`:

   ```cpp
   static void test_clear_requires_both_literal_tokens(void) {
     const char *misses[] = { "clear", "contra", "clearcontra", "clear  contra", "CLEAR CONTRA" };
     for (unsigned i = 0; i < 5u; ++i)
       TEST_ASSERT_FALSE_MESSAGE(cli_dispatch(misses[i]), misses[i]);
     TEST_ASSERT_TRUE(cli_dispatch("clear contra"));
   }
   ```

   and, in `test_contra.cpp`:

   ```cpp
   void test_latch_clears_only_on_the_literal_two_token_command(void) {
     pb_latch_contra();
     TEST_ASSERT_FALSE(cli_dispatch("clear"));
     TEST_ASSERT_FALSE(cli_dispatch("clearcontra"));
     TEST_ASSERT_TRUE(safety_contra());
     TEST_ASSERT_TRUE(cli_dispatch("clear contra"));
     TEST_ASSERT_FALSE(safety_contra());
   }

   /* It clears on NOTHING else. `dry off` is a different latch, and a successful home is not
      evidence about water. A reader who adds either as a clear has removed the feature. */
   void test_latch_is_not_cleared_by_dry_off_or_by_a_successful_home(void) {
     pb_latch_contra();
     TEST_ASSERT_TRUE(cli_dispatch("dry off"));
     TEST_ASSERT_TRUE(safety_contra());
     sim_set_screw_pulse_ms(2); sim_set_home_region(0u, 40u); sim_set_cart_at(600u);
     TEST_ASSERT_TRUE(cart_home());
     TEST_ASSERT_TRUE(safety_contra());
   }
   ```

7. - [ ] **Make the LCD banner unconditional in `src/ui.cpp`.** Task 10's renderer paints whatever `lcd_state` / `lcd_detail` the caller selected, and task 26 is what selects them — which would leave drop 2 with a latch that no panel announces, on a bench where the operator is a metre from the board and not at the serial console. The latch is the loudest thing the rig can know about itself, so the renderer states it rather than trusting its caller:

   **This is an EDIT of task 10 step 4's renderer, not a replacement of it.** The sim banner
   and the HTTP-status row both stay: task 29 asserts `"*** SIM NO D6 **"` on row 0 *exactly*,
   and spec §4.2 requires the last non-200 HTTP status on the LCD. Dropping either while adding
   the contra branch is the shape of the bug this note exists to prevent. **Row 0's precedence
   is stated rather than left to the order of the lines: SIM outranks CONTRA**, because a sim
   binary showing `CONTRA LATCH` would be read as a latched *rig*, and the one thing row 0 has
   to say on the sim is that this board cannot pump at all.

   ```c
   void ui_render_lcd(const ui_state_t *s, char rows[2][17]) {
     if (s->contra) {                    /* §5's rows, and they OVERRIDE the caller's choice:
                                            a latch may not be hidden behind whatever state
                                            the fill function happened to select this pass -
                                            nor behind an HTTP status, which is the lesser
                                            fact of the two. */
       row_(rows[0], "CONTRA LATCH");
       row_(rows[1], "float ok,no flow");   /* 16 columns; §5's 17-char text will not fit */
     } else {
       row_(rows[0], s->lcd_state ? s->lcd_state : "IDLE");
       if (s->http_status != 0u && s->http_status != 200u)
         rowf_(rows[1], "HTTP %u", (unsigned)s->http_status);   /* §4.2, task 10 step 4 */
       else
         row_(rows[1], s->lcd_detail ? s->lcd_detail : "");
     }
     /* LAST, and outside the branch: SIM outranks CONTRA on row 0. Task 29 asserts these
        sixteen characters exactly, in both renderers. */
     if (s->sim) row_(rows[0], "*** SIM NO D6 **");
   }
   ```

   Task 10's `test_ui_render_lcd_shows_the_contradiction_banner` already sets `contra = true`
   and both strings, so it still passes; add two assertions to it — that the override holds
   when the caller has selected something else, and that SIM still wins row 0 over it:

   ```cpp
     s.contra = true; s.lcd_state = "IDLE"; s.lcd_detail = "next 35s"; s.http_status = 400;
     ui_render_lcd(&s, rows);
     TEST_ASSERT_EQUAL_STRING("CONTRA LATCH    ", rows[0]);
     TEST_ASSERT_EQUAL_STRING("float ok,no flow", rows[1]);   /* the latch outranks HTTP 400 */
     s.sim = true;
     ui_render_lcd(&s, rows);
     TEST_ASSERT_EQUAL_STRING("*** SIM NO D6 **", rows[0]);   /* and SIM outranks the latch */
   ```

8. - [ ] **Make the last exit-path arm drivable, and raise `PB_DRIVABLE_RESULTS` from 18 to 19.** `pb_drive_dose_to_result()`'s `DOSE_REFUSED_CONTRA` arm is now `pb_latch_contra(); pb_advance(PB_DOSE_MIN_GAP_MS + 1u);` followed by an ordinary request. All nineteen results are reachable, and the `switch` has no ignores left.

9. - [ ] **Run the whole gate and commit.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native && pio test -e native_bench -f test_cli && \
     grep -c 'contra_latched = true' src/safety.cpp && grep -rc 'contra_latched' src include lib && \
     make check && pio run -e uno_r4_wifi -e uno_r4_wifi_bringup 2>&1 | tail -3
   ```
   expected: `0 Failures`; **`1`** for the setter grep — one assignment of `true` in the whole
   tree, and if it is ever 2 the design has been undone; the second grep should show
   `contra_latched` only in `src/safety.cpp` and `include/noinit.h`; `all invariants hold`;
   two `SUCCESS` lines.

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && git add include/safety.h src/safety.cpp src/cli.cpp src/ui.cpp test/support/harness.h test/test_contra/test_contra.cpp test/test_cli/test_cli.cpp test/test_dose/test_dose.cpp && git commit -m "safety: the float/flow contradiction latch

   The float said OK - permission granted by the one input whose whole design is that
   failure reads as refusal - and the dose that permission authorised produced no flow at
   all. Two independent sensors contradict each other. The safe reading is that the tank is
   empty and the float is stuck, and the safe response is not 'end this dose and let the
   next one start', which is all a no-flow abort does, but refuse everything until a human
   looks.

   Set in one place, dose_end_ml_(), under five conditions each doing one job. Cleared in one
   place, the console's literal 'clear contra', which ships in BOTH binaries because the
   unattended one can latch and a rig releasable only by a reflash is worse. Checked above
   the dry latch so the more specific reason is reported. It refuses every dose and nothing
   else: cart_home() and the boot self-home run under it, because a cart left over outlet N
   holds that gate open under the reservoir head for as long as the latch stands.

   A kinked hose with a full tank latches too, deliberately. The latch does not diagnose; it
   refuses, and both causes should stop the rig until a human looks.

   NOT DURABLE, and this is a requirement rather than a caveat. .noinit buys the warm-reset
   case only: a power cycle, a brown-out and a reflash all clear SRAM and silently rearm the
   rig, and whether it survives even a watchdog reset is UNPROVEN until bring-up 7c'. Spec
   16.5.3 - the backend refusing to queue for a controller reporting err=contra or ch207=1
   until a human confirms in the app - is therefore a going-live requirement, not a
   follow-up. And until 16.5.4 stores err=, the phone says 'the reservoir is empty or at the
   waterline' whatever the cause, including a kinked hose with a full tank; the operator has
   to read status or ch207 before topping up. The alert also arrives on the SECOND report
   after the latch, not the first, because butler's raise needs two bad sightings inside its
   flap window."
   ```

10. - [ ] **What this task does not prove, and the two bring-up steps that do.** **Bring-up 5b** unplugs the float hall mid-dose and must show `DOSE_ABORT_FLOAT` **and `contra=0` afterwards** — the float dropping is agreement, not contradiction, and a rig that latches there has this condition inverted. **Bring-up 7a** primes a line that has never held water: a `pump 20000 prime` must **not** latch, and a plain `pump` on the same dry line **will** — at which point `status` says `contra=1`, and the recovery is `clear contra` and prime again. And **bring-up 7c′** is the only thing that turns `.noinit` survival from a claim into a measurement: `noinit pattern`, force the hang, and read the struct back. If 7c′ fails, every `.noinit` guarantee in the spec is void and this latch becomes cold-boot-only.

---

---

### Task 20: The bring-up console: servo, home, goto, pump, calib, cal, hang, noinit pattern, and the dose summary line

**Drop 2.**

**Files:**
- Modify: `src/cli.cpp` (the `#if PB_BRINGUP` block and the summary printer), `include/cli.h`, `include/safety.h` and `src/safety.cpp` (the unconditional `hang` member and its loop hook), `include/config.h` and `platformio.ini` (spec §2.15's `#error` on an uncalibrated bench build), `test/test_cli/test_cli.cpp`
- Test: `test/test_cli/test_cli.cpp`

**Interfaces:**
- Consumes: `dose_result_t dose_run(const dose_req_t *)`, `uint16_t dose_flow_ml(void)`, `dose_result_t dose_last_result(void)`, `uint32_t dose_last_ms(void)`, `uint32_t dose_last_pulses(void)`, `uint8_t dose_last_outlet(void)`, `const char *err_of(dose_result_t)`, `bool cfg_pulses_per_l_set(uint16_t)`, `uint16_t cfg_pulses_per_l_get(void)` from task 17; `bool cart_home(void)`, `bool cart_goto(uint8_t)`, `void cart_jog(int16_t, uint32_t)`, `const char *cart_err(void)` from task 14; `g_nv.pattern` and `noinit_commit()` from task 4; `PB_SERVO_CAP_MS` (10000), `PB_DOSE_CAP_MS_MAX` (60000), `PB_HANG_MS` (3000), `PB_PULSES_PER_L_MIN` (1000), `PB_PULSES_PER_L_MAX` (20000), `PB_OUTLETS` (5) from task 2.
- Produces: the `#if PB_BRINGUP` command block — `servo <±us> <ms>` (bounded jog, ≤ `PB_SERVO_CAP_MS`); `home`; `goto <1-5>`; `pump <ms> [prime] [hang]` building a `dose_req_t` with `need_pos = false`, `by_time = true`, `cap_ms = min(ms, PB_DOSE_CAP_MS_MAX)`, `long_prime` set only by the literal `prime` token, and `hang` starving the dog after `PB_HANG_MS` for bring-up 7c; `calib` running `pump 10000 prime` plus the summary line — the 7b workhorse; `cal <pulses_per_l>` accepting **only** 1000..20000; `noinit pattern` writing a known 32-bit word into `g_nv.pattern` for bring-up 7c′ — plus, in `include/cli.h` and **outside** the `#if`, `void cli_print_dose_summary(void);`, printed at the end of every dose from **every** path in the exact shape

  ```
  dose outlet=3 ms=4120 pulses=706 ml=120 mls=29.1 r=ok
  ```

  `dose_run(` appears **exactly once** in this file. `clear contra` (task 19), `stop` (task 16) and `dry on|off` (task 15) stay **outside** the `#if`, in both builds.

**`r=ok`, and why it is one conditional rather than a second mapping.** Spec §6 prints the summary line with `r=ok`, while `err_of(DOSE_OK)` is `"none"` — `none` is §4.1's wire token and `ok` is not in that enum. Do **not** add `ok` to `err_of()`: that enum is tested against `butler.py`'s own parser. Print `dose_last_result() == DOSE_OK ? "ok" : err_of(dose_last_result())` — one conditional, in one place, and the line matches the spec's printed shape while the wire enum stays exactly what butler accepts.

**`hang` is an unconditional member of `dose_req_t`.** A `#if PB_BRINGUP` on the field would break spec §6's strongest mechanical check — `safety.o` and `hal_uno.o` hashing **identically** between `uno_r4_wifi` and `uno_r4_wifi_bringup` — and that hash equality is the whole reason bring-up 7c can prove the watchdog on the `bringup` binary and have the result mean something for the `bench` one. The field is unconditional, exactly like `long_prime`, and only the `#if PB_BRINGUP` block ever sets it true.

**Spec sections to read in full before starting:** §6 in full (the command table, the `cal` argument, the summary line, the scoping of the bench grep); §2.15; §12 item 1; §13 steps 6, 7a, 7b, 7c and 7c′.

**Tests:** `test_bringup_commands_are_absent_from_the_bench_build`, `test_pump_without_an_argument_is_refused`, `test_pump_ms_is_clamped_to_the_hard_cap`, `test_pump_hang_requires_the_literal_third_token`, `test_goto_rejects_zero_and_six`, `test_cal_rejects_zero_and_absurd_values`, `test_dose_summary_line_carries_outlet_ms_pulses_ml_and_mls`.

**Deliverable:** `pio test -e native -f test_cli` and `pio test -e native_bench -f test_cli` both pass, with `test_bringup_commands_are_absent_from_the_bench_build` compiled **twice** — once as usual and once with `-UPB_BRINGUP` — so the bench arm is actually exercised rather than assumed. Drop 2 is then complete: the safety spine is whole and bring-up 4a–7d can be run. **Do not dilute it later**; if the appetite runs short, what gives is drop 3's diagnostic channels and drop 1's UI coarsening — never a guard, never the latch, never a `make check` grep.

---

1. - [ ] **Write the case the two-binaries design rests on, and write it first.** Add to `test/test_cli/test_cli.cpp` with its `RUN_TEST` line:

   ```cpp
   /* §6. `pump 60000 prime hang` was a single typed line that removed all three of DECISIONS
      #10's mandatory measures at once: it asserted D6, suppressed the no-flow abort and
      starved the watchdog. Over an unauthenticated USB CDC line a serial-monitor reconnect,
      a `cat` of the wrong file into /dev/cu.*, or an autocompleting terminal is enough.
      Gating on the spelling of a token is not a gate; a different binary is. */
   static void test_bringup_commands_are_absent_from_the_bench_build(void) {
     pb_test_setup();
   #if PB_BRINGUP
     TEST_ASSERT_TRUE(cli_dispatch("servo 1600 200"));
     TEST_ASSERT_TRUE(cli_dispatch("home"));
     TEST_ASSERT_TRUE(cli_dispatch("goto 3"));
     TEST_ASSERT_TRUE(cli_dispatch("pump"));          /* exists; refuses without an argument */
     TEST_ASSERT_TRUE(cli_dispatch("calib"));
     TEST_ASSERT_TRUE(cli_dispatch("cal 5880"));
     TEST_ASSERT_TRUE(cli_dispatch("noinit pattern"));
   #else
     /* Not refused - NOT A COMMAND. `? unknown; type help` is the only correct answer, and it
        is what bring-up 7e types at the bench binary to prove which binary is flashed. */
     TEST_ASSERT_FALSE(cli_dispatch("servo 1600 200"));
     TEST_ASSERT_FALSE(cli_dispatch("home"));
     TEST_ASSERT_FALSE(cli_dispatch("goto 3"));
     TEST_ASSERT_FALSE(cli_dispatch("pump 2000"));
     TEST_ASSERT_FALSE(cli_dispatch("calib"));
     TEST_ASSERT_FALSE(cli_dispatch("cal 5880"));
     TEST_ASSERT_FALSE(cli_dispatch("noinit pattern"));
     /* and the four that ship in BOTH binaries, asserted here so nobody moves them inside
        the #if: an unattended board must still be stoppable, dry-able and releasable. */
     TEST_ASSERT_TRUE(cli_dispatch("stop"));
     TEST_ASSERT_TRUE(cli_dispatch("dry on"));
     TEST_ASSERT_TRUE(cli_dispatch("dry off"));
     TEST_ASSERT_TRUE(cli_dispatch("clear contra"));
   #endif
   }
   ```

2. - [ ] **Run it both ways and watch the bringup arm fail.**

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cli; pio test -e native_bench -f test_cli
   ```
   expected: `native` fails with `Expected TRUE Was FALSE` on `servo` — none of these is a
   command yet — and `native_bench` **passes already**, which is exactly right: absence is the
   default and presence is what has to be built.

3. - [ ] **Write the `#if PB_BRINGUP` scaffold and `servo`.** One block near the bottom of `src/cli.cpp` and one line in `cli_dispatch()` above the unknown-command fallthrough. Everything bring-up-only lives inside the block; nothing else does.

   ```c
   #if PB_BRINGUP
   /* cart.h is ALREADY included at the top of this file (task 14 step 10) — cli_print_status()
      calls the cart in both binaries, so the include cannot live in here. Do not add a second
      one: this block is bringup-only and the cart is not. */

   /* spec §6. Every command here is compiled out of the binary that runs unattended, and
      make check proves it on the PREPROCESSED source of this file (task 30), not on this #if. */
   static bool cli_dispatch_bringup_(const char *line) {
     if (strncmp(line, "servo ", 6) == 0) {
       const char *sp = strchr(line + 6, ' ');
       uint32_t us = 0u, ms = 0u;
       if (!sp || !parse_u32_range_(line + 6, sp, &us) || !parse_u32_(sp + 1, &ms) ||
           us < 1000u || us > 2000u || ms == 0u) {
         hal_serial_write("usage: servo <1000-2000> <ms>\n");
         return true;
       }
       if (ms > PB_SERVO_CAP_MS) ms = PB_SERVO_CAP_MS;  /* a typo may not run the screw forever */
       cart_jog((int16_t)us, ms);
       hal_serial_write("servo done\n");
       return true;
     }
     return false;                                       /* steps 4, 7 and 11 add the rest */
   }
   #endif /* PB_BRINGUP */
   ```

   `parse_u32_range_(begin, end, out)` is `parse_u32_()`'s bounded sibling — task 11's
   `parse_u32_()` requires a NUL terminator, and the first argument here ends at a space. Add
   it beside `parse_u32_()`, outside the `#if`, and keep both digit-only: a `strtol` here would
   silently accept `servo 0x600 200`.

   In `cli_dispatch()`, immediately above `hal_serial_write("? unknown; type help\n")`:

   ```c
   #if PB_BRINGUP
     if (cli_dispatch_bringup_(line)) return true;
   #endif
   ```

4. - [ ] **Write `home` and `goto`.** `home` is **one** traverse: calling `cart_home()` twice — once to test and once to act — is two full traverses of up to `PB_MOVE_CAP_MS` each on a real screw, and two stalls before the operator is told about the first.

   ```c
     if (strcmp(line, "home") == 0) {
       if (cart_home()) hal_serial_write("home ok\n");                 /* ONE traverse */
       else { hal_serial_write("home FAILED: ");
              hal_serial_write(cart_err()); hal_serial_write("\n"); }
       return true;
     }
     if (strncmp(line, "goto ", 5) == 0) {
       uint32_t o = 0u;
       if (!parse_u32_(line + 5, &o) || o < 1u || o > PB_OUTLETS) {
         hal_serial_write("goto: outlet must be 1..5\n");              /* the range, by name */
         return true;
       }
       if (cart_goto((uint8_t)o)) hal_serial_write("goto ok\n");
       else { hal_serial_write("goto FAILED: ");
              hal_serial_write(cart_err()); hal_serial_write("\n"); }
       return true;
     }
   ```

   The refusal text carries the literal `1..5` because step 5's case greps for it: an operator
   who typed `goto 6` must be told the range, not merely that something failed.

5. - [ ] **Write the two range cases, run them, and see where each stands.**

   ```cpp
   static void test_goto_rejects_zero_and_six(void) {
   #if PB_BRINGUP
     pb_test_setup();
     char out[256];
     const char *bad[] = { "goto 0", "goto 6", "goto x" };
     for (unsigned i = 0; i < 3u; ++i) {
       (void)sim_serial_tx(out, sizeof out);
       TEST_ASSERT_TRUE(cli_dispatch(bad[i]));            /* the command exists... */
       size_t n = sim_serial_tx(out, sizeof out); out[n] = '\0';
       TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, "1..5"), bad[i]);   /* ...and says the range */
     }
   #else
     TEST_IGNORE_MESSAGE("bench build: goto is not a command");
   #endif
   }

   static void test_pump_without_an_argument_is_refused(void) {
   #if PB_BRINGUP
     pb_test_setup();
     char out[256];
     (void)sim_serial_tx(out, sizeof out);
     TEST_ASSERT_TRUE(cli_dispatch("pump"));
     size_t n = sim_serial_tx(out, sizeof out); out[n] = '\0';
     TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, "usage"), "a bare `pump` must never assert D6");
     TEST_ASSERT_FALSE(sim_pump_is_on());
   #else
     TEST_IGNORE_MESSAGE("bench build: pump is not a command");
   #endif
   }
   ```

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cli
   ```
   expected: `goto` passes already against step 4's range check — if it does not, the check was
   dropped, so put it back rather than relaxing the case — and `pump` fails on the missing
   `usage`, because it is not a command yet.

6. - [ ] **Add `hang` to `dose_req_t` and its hook to the dose loop.** In `include/safety.h`, below `long_prime`:

   ```c
     bool hang;   /* bring-up 7c: run the dose PB_HANG_MS, then STOP FEEDING. The field is
                     UNCONDITIONAL - a #if PB_BRINGUP here would break §6's safety.o hash
                     equality between uno_r4_wifi and uno_r4_wifi_bringup, which is what lets
                     7c prove the watchdog on one binary and mean it about the other. Only
                     cli.cpp's #if PB_BRINGUP block ever sets it true. */
   ```

   and in `src/safety.cpp`, as the **last** statement of `dose_run()`'s loop body — below the
   bus recheck — so every abort rule is evaluated before the dog is deliberately starved:

   ```c
       if (q->hang && el >= PB_HANG_MS) {
         /* bring-up 7c. D6 STAYS ASSERTED and the dog is NOT fed: the watchdog must bite, the
            reset must drop the pump, and the honest spill is 5592 ms x the flow rate 7b
            measured (~170 ml at 30 ml/s). Writing the pump off here would make the spill zero
            and 7c would prove nothing. This is the only loop in the program that is meant not
            to terminate, and it is in the one function §9's for(;;) grep exempts.
            hal_millis() is called so the loop is not an empty body with no forward progress. */
         for (;;) { (void)hal_millis(); }
       }
   ```

   **No host test ever sets `hang = true`** — it would hang the suite by construction, which is
   the point of the flag. `test_pump_hang_requires_the_literal_third_token` is the negative
   case, and bring-up 7c is the positive one.

7. - [ ] **Write `pump`, `calib`, and the file's one `dose_run()` call site.**

   ```c
   /* THE ONE CALL SITE OF THE DOSING ENTRY POINT IN THIS FILE -- §9 counts exactly one in
      cli.cpp, and this comment may not spell the token it counts.
      `pump` and `calib` both come through here, and so does the summary line. */
   static void cli_run_dose_(uint32_t ms, bool long_prime, bool hang) {
     dose_req_t q = {0};
     q.by_time    = true;
     q.need_pos   = false;                    /* bring-up 4a/5a/5b run before the cart is
                                                 calibrated; a pump that demanded a position
                                                 would make them unrunnable */
     q.cap_ms     = ms > PB_DOSE_CAP_MS_MAX ? PB_DOSE_CAP_MS_MAX : ms;
     q.long_prime = long_prime;
     q.hang       = hang;
     (void)dose_run(&q);
     cli_print_dose_summary();
   }

     if (strncmp(line, "pump", 4) == 0) {
       uint32_t ms = 0u;
       const char *arg = line + 4;
       if (*arg != ' ' || !parse_u32_range_(arg + 1, token_end_(arg + 1), &ms) || ms == 0u) {
         hal_serial_write("usage: pump <ms> [prime] [hang]\n");
         return true;
       }
       bool prime = false, hang = false;
       for (const char *t = arg + 1; *t != '\0'; ) {      /* WHOLE tokens: `hanging` is not
                                                             `hang`, and a substring match
                                                             would hang on a word nobody typed */
         while (*t == ' ') ++t;
         const char *e = t; while (*e != '\0' && *e != ' ') ++e;
         size_t n = (size_t)(e - t);
         if (n == 5u && strncmp(t, "prime", 5) == 0) prime = true;
         if (n == 4u && strncmp(t, "hang",  4) == 0) hang  = true;
         t = e;
       }
       cli_run_dose_(ms, prime, hang);
       return true;
     }
     if (strcmp(line, "calib") == 0) { cli_run_dose_(10000u, true, false); return true; }  /* 7b */
   ```

   `token_end_(s)` returns the pointer to the first space or NUL — one static beside
   `parse_u32_range_()`.

8. - [ ] **Write the two argument-cap cases and run them.** The second one is the reason the token walk above is a walk and not a `strstr`.

   ```cpp
   static void test_pump_ms_is_clamped_to_the_hard_cap(void) {
   #if PB_BRINGUP
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true); sim_set_flow_ml_s(30);
     TEST_ASSERT_TRUE(cli_dispatch("pump 600000"));                  /* ten minutes typed */
     TEST_ASSERT_LESS_OR_EQUAL_UINT32(PB_DOSE_CAP_MS_MAX + 200u, sim_pump_on_ms());
   #else
     TEST_IGNORE_MESSAGE("bench build");
   #endif
   }

   /* `" hanging"` contains `" hang"`, so a bare strstr passes this case wrongly - which is
      exactly what the case is for. §6's own words are "the literal third token". */
   static void test_pump_hang_requires_the_literal_third_token(void) {
   #if PB_BRINGUP
     pb_test_setup();
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true); sim_set_flow_ml_s(30);
     uint32_t f0 = sim_feeds();
     TEST_ASSERT_TRUE(cli_dispatch("pump 500 hanging"));
     TEST_ASSERT_GREATER_THAN_UINT32(f0, sim_feeds());   /* the dog was fed throughout */
   #else
     TEST_IGNORE_MESSAGE("bench build");
   #endif
   }
   ```

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cli && pio test -e native_bench -f test_cli && grep -c 'dose_run(' src/cli.cpp
   ```
   expected: `0 Failures` from both, and **exactly `1`** from the grep. That last one is a
   hand-check here on purpose: the `dose_run(` invariant belongs to task 30's `tools/check.sh`
   and does not exist yet, so a green `make check` at this point proves nothing about it.

   ```bash
   cd /Users/jcanton/projects/plant-butler/firmware && git add include/safety.h src/safety.cpp include/cli.h src/cli.cpp test/test_cli/test_cli.cpp && git commit -m "cli: servo, home, goto, pump and calib, behind #if PB_BRINGUP

   pump and calib share ONE dose_run() call site, which is also where the summary line is
   printed. pump's flags are whole tokens, not substrings: pump 500 hanging used to hang on
   a word nobody typed.

   dose_req_t.hang is an unconditional field. A #if PB_BRINGUP on it would break spec 6's
   safety.o hash equality between the bench and bringup envs - which is what lets bring-up
   7c prove the watchdog on the bringup binary and have that mean something for the bench
   one. The hang loop holds D6 ASSERTED and does not feed: writing the pump off there would
   make the spill zero and 7c would prove nothing. The honest spill is 5592 ms x the flow
   rate 7b measures."
   ```

9. - [ ] **Write the failing case for `cal`'s bounds.**

   ```cpp
   static void test_cal_rejects_zero_and_absurd_values(void) {
   #if PB_BRINGUP
     pb_test_setup();
     uint16_t before = cfg_pulses_per_l_get();
     char out[256];
     const char *bad[] = { "cal 0", "cal 999", "cal 20001", "cal 4294967295", "cal -5", "cal x" };
     for (unsigned i = 0; i < 6u; ++i) {
       (void)sim_serial_tx(out, sizeof out);
       TEST_ASSERT_TRUE(cli_dispatch(bad[i]));
       size_t n = sim_serial_tx(out, sizeof out); out[n] = '\0';
       TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, "1000..20000"), bad[i]);
       TEST_ASSERT_EQUAL_UINT16_MESSAGE(before, cfg_pulses_per_l_get(), bad[i]);
     }
     TEST_ASSERT_TRUE(cli_dispatch("cal 5880"));
     TEST_ASSERT_EQUAL_UINT16(5880u, cfg_pulses_per_l_get());
   #else
     TEST_IGNORE_MESSAGE("bench build");
   #endif
   }
   ```

10. - [ ] **Run it, watch it fail, then write `cal` and `noinit pattern`.**

    ```c
      if (strncmp(line, "cal ", 4) == 0) {
        /* `cal 0` - one token on the serial line, or a stray byte parsed as one - used to make
           target = 0 for EVERY subsequent command, so each dose ignored its millilitre target
           and ran the full cap_ms; pulses_to_ml then divided by zero, and the Cortex-M4's UDIV
           returns 0 without DIV_0_TRP - so the flood happened and the report said nothing came
           out. The dosing entry point re-checks the same range as DOSE_REFUSED_CAL (§6). */
        uint32_t v = 0u;
        if (!parse_u32_(line + 4, &v) || v < PB_PULSES_PER_L_MIN || v > PB_PULSES_PER_L_MAX ||
            !cfg_pulses_per_l_set((uint16_t)v)) {
          hal_serial_write("cal: pulses_per_l must be 1000..20000\n");
          return true;
        }
        cli_printf_u32("pulses_per_l=%lu\n", (uint32_t)cfg_pulses_per_l_get());
        return true;
      }
      if (strcmp(line, "noinit pattern") == 0) {        /* bring-up 7c' */
        g_nv.pattern = 0xC0FFEE01u;
        noinit_commit();
        hal_serial_write("noinit pattern written. Now: `pump 3000 hang`, wait for the reset, "
                         "then `status` - the pattern AND the checksum must both survive.\n");
        return true;
      }
    ```

    ```bash
    cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cli && pio test -e native_bench -f test_cli
    ```

11. - [ ] **Write the failing case for the summary line, which is the pitch's own deliverable.**

    ```cpp
    static void test_dose_summary_line_carries_outlet_ms_pulses_ml_and_mls(void) {
    #if PB_BRINGUP
      pb_test_setup();
      pb_advance(PB_BOOT_GAP_MS + 1u);
      sim_set_float(true); sim_set_flow_ml_s(30);
      TEST_ASSERT_TRUE(cli_dispatch("cal 5880"));
      char out[512];
      (void)sim_serial_tx(out, sizeof out);
      TEST_ASSERT_TRUE(cli_dispatch("pump 4000"));
      size_t n = sim_serial_tx(out, sizeof out); out[n] = '\0';
      TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, "dose outlet="), out);
      TEST_ASSERT_NOT_NULL(strstr(out, " ms="));
      TEST_ASSERT_NOT_NULL(strstr(out, " pulses="));
      TEST_ASSERT_NOT_NULL(strstr(out, " ml="));
      TEST_ASSERT_NOT_NULL(strstr(out, " mls="));
      TEST_ASSERT_NOT_NULL(strstr(out, " r="));
      /* mls is computed in integer TENTHS and printed as two unsigned longs around a literal
         dot: newlib's float formatting is the deepest stack consumer in the program, and the
         float conversions are banned and grepped for (§12 item 1). */
      const char *mls = strstr(out, " mls=");
      TEST_ASSERT_NOT_NULL(strchr(mls, '.'));
    #else
      TEST_IGNORE_MESSAGE("bench build");
    #endif
    }
    ```

12. - [ ] **Write the summary printer.** The DECLARATION `void cli_print_dose_summary(void);` goes in `include/cli.h`, outside the `#if`, because task 26's `exec.cpp` calls the same function for the backend's doses in the bench build. The DEFINITION below goes in `src/cli.cpp`, also outside the `#if` — a body in a header included by `cli.cpp`, `exec.cpp` and `main.cpp` is a multiple-definition link error:

    ```c
    /* src/cli.cpp, outside the #if PB_BRINGUP block. */
    void cli_print_dose_summary(void) {
      static char line[PB_LINE_CAP];
      uint32_t ms  = dose_last_ms();
      uint32_t ml  = dose_flow_ml();
      uint32_t t10 = ms ? (ml * 10000u) / ms : 0u;      /* ml/s x 10, in integer tenths */
      /* §6 prints r=ok for a successful dose while err_of(DOSE_OK) is the wire's "none".
         One conditional, here: `ok` must NOT be added to err_of()'s enum, which is tested
         against butler.py's own parser. */
      const char *r = (dose_last_result() == DOSE_OK) ? "ok" : err_of(dose_last_result());
      snprintf(line, sizeof line,
               "dose outlet=%lu ms=%lu pulses=%lu ml=%lu mls=%lu.%lu r=%s\n",
               (unsigned long)dose_last_outlet(), (unsigned long)ms,
               (unsigned long)dose_last_pulses(), (unsigned long)ml,
               (unsigned long)(t10 / 10u), (unsigned long)(t10 % 10u), r);
      hal_serial_write(line);
    }
    ```

    ```bash
    cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cli && pio test -e native_bench -f test_cli && make check
    ```
    `make check` must still report zero hits for the float-conversion grep across the tree, and
    task 11's `test_no_float_formatting_appears_in_any_printed_line` must still pass with this
    line in the output — `mls=29.1` is two integers and a literal dot, and that case's scanner
    knows about exactly two dotted fields: `ip=` and this one.

13. - [ ] **Make an uncalibrated unattended binary impossible to ship by accident.** Spec §2.15: "Once drop 2 lands, the `bench` env `#error`s on a zero gate constant, so the number cannot be forgotten." In `include/config.h`, below `PB_PULSES_PER_GATE`:

    ```c
    /* §2.15. The binary that runs unattended may not ship with an uncalibrated gate pitch.
       PB_ALLOW_UNCALIBRATED is set in [env:uno_r4_wifi]'s build_flags TODAY and is deleted by
       bring-up 6, in the same commit that writes the measured PB_PULSES_PER_GATE. It exists
       because this #error would otherwise block `pio run -e uno_r4_wifi`, which is this
       plan's per-commit gate for every remaining task -- an escape hatch that has to be
       deleted by hand, and that `status` prints, is the honest way to have both. */
    #if !defined(PB_BRINGUP) && !defined(PB_SIM) && !defined(PB_NATIVE) && \
        !defined(PB_ALLOW_UNCALIBRATED)
    #  if PB_PULSES_PER_GATE == 0
    #    error "PB_PULSES_PER_GATE is 0: bring-up 6 has not committed the gate pitch. Run it, \
    commit the number, and delete -DPB_ALLOW_UNCALIBRATED from [env:uno_r4_wifi]."
    #  endif
    #endif
    ```

    **This is the third file in the tree to name `PB_BRINGUP`, and the `make check` invariant already accounts for it.** Task 13 step 5's grep counts files under `src` and `lib` only, for exactly this reason and for `test/test_cli/test_cli.cpp`'s two-arm cases; the scoping is recorded there and in the Global Constraints list, and it is a deviation from spec §9's table. **Re-run `make check` before committing this step** and confirm it still reports `PB_BRINGUP appears in src/cli.cpp and src/main.cpp, and in no other source file (2)`. If it reports 3 or 4, the grep was widened back to `"${SCAN[@]}"` — restore the scope, do not delete the guard.

    in `[env:uno_r4_wifi]`'s `build_flags`, beside the `PB_REPORT_POS_UNKNOWN` line:

    ```ini
        -DPB_ALLOW_UNCALIBRATED       ; DELETED BY BRING-UP 6, together with the measured
                                      ; PB_PULSES_PER_GATE. config.h #errors without it.
    ```

    and one line in `cli_print_status()`:

    ```c
    #ifdef PB_ALLOW_UNCALIBRATED
      hal_serial_write("cart=UNCALIBRATED BUILD (PB_ALLOW_UNCALIBRATED set; bring-up 6 removes it)\n");
    #endif
    ```

    Prove both halves work:

    ```bash
    cd /Users/jcanton/projects/plant-butler/firmware && pio run -e uno_r4_wifi 2>&1 | tail -2 && \
      sed -i '' 's/^\( *\)-DPB_ALLOW_UNCALIBRATED/\1;-DPB_ALLOW_UNCALIBRATED/' platformio.ini && \
      pio run -e uno_r4_wifi 2>&1 | grep -c 'PB_PULSES_PER_GATE is 0' ; \
      sed -i '' 's/^\( *\);-DPB_ALLOW_UNCALIBRATED/\1-DPB_ALLOW_UNCALIBRATED/' platformio.ini && \
      pio run -e uno_r4_wifi 2>&1 | tail -2
    ```
    expected: `SUCCESS`, then a non-zero count (the guard bites), then `SUCCESS` again.

14. - [ ] **Run everything, build every environment, and commit — this is drop 2's close-out.**

    ```bash
    cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native && pio test -e native_bench -f test_cli && \
      pio test -e native_cal -f test_cart && pio test -e native_measured -f test_dose && \
      make check && pio run -e uno_r4_wifi -e uno_r4_wifi_bringup 2>&1 | tail -3
    ```
    expected: `0 Failures` from all four suites, `all invariants hold`, two `SUCCESS` lines.

    ```bash
    cd /Users/jcanton/projects/plant-butler/firmware && git add include/cli.h src/cli.cpp include/config.h platformio.ini test/test_cli/test_cli.cpp && git commit -m "cli: cal is bounded, noinit pattern, and the per-dose summary line

    cal 0 made target = 0 for every later command, so each dose ran its full cap and
    pulses_to_ml divided by zero; UDIV returns 0 without DIV_0_TRP, so the flood happened
    and the report said nothing came out. cal now accepts 1000..20000 only, digits only, and
    dose_run() re-checks the same range as DOSE_REFUSED_CAL.

    The summary line is printed at the end of every dose from every path - cli.cpp's one
    dose helper calls it, and exec.cpp (task 26) calls the same function for the backend's
    doses. mls is integer tenths printed as two unsigned longs around a literal dot: %f
    would link newlib's float formatting, the deepest stack consumer in the program, for one
    cosmetic figure. r=ok is one conditional in the printer, because 6 prints ok while the
    wire enum's no-error token is none and that enum is tested against butler's parser.

    config.h now refuses to build the unattended binary with a zero gate pitch unless
    -DPB_ALLOW_UNCALIBRATED is set - which it is today, and which bring-up 6 deletes in the
    same commit that writes the measured number. status says so on a board built that way.

    Drop 2 is complete: the safety spine is whole and bring-up 4a-7d can be run. If the
    appetite runs short from here, what gives is drop 3's diagnostic channels and drop 1's
    UI coarsening - never a guard, never the latch, never a make check grep."
    ```

15. - [ ] **Write the drop-2 close-out note into `docs/superpowers/notes/2026-09-03-drop-2-closeout.md`: what this drop has NOT proved, and which bring-up step proves each.** It is a **file**, committed here — add it to step 14's `git add`, or make one more commit for it; a checkbox step that leaves no artefact behind is a step nobody can tell was done. Nothing in steps 1–14 touches silicon, and no host test substitutes for any of the following. Each is a numbered step in spec §13, run on the **bringup** binary with `status` reporting `build=bringup`:

    - `servo`, `home` and `goto` against the real screw, the real reduction gears and the real home hall — **bring-up 6**, which also produces `PB_PULSES_PER_GATE` and `PB_PULSES_HOME_TO_1` and deletes `-DPB_ALLOW_UNCALIBRATED`. Until it lands, `cart_goto` is compiled to `return false` and `pos` is never `ok`.
    - `pump 2000` clicking a **dry** relay with no 12 V on COM, and COM–NO staying open across a power cycle **and** across a `hang`-forced watchdog reset — **bring-up 4a**, which is what proves spec §2.1's boot write; the wiring README's old level-then-direction recipe would have failed it on an active-LOW module.
    - `DOSE_REFUSED_FLOAT` with the float below the line, and `DOSE_ABORT_FLOAT` with the hall unplugged mid-dose, `contra=0` afterwards — **bring-up 5a and 5b**.
    - `pump 20000 prime` on a line that has never held water, and `clear contra` if a plain `pump` latched it first — **bring-up 7a**.
    - `calib` producing a real `mls=` figure and `cal <n>` retuning without a reflash — **bring-up 7b**, whose output commits `PB_PULSES_PER_L_DEFAULT`, `PB_PRIME_MS_DEFAULT`, `PB_STALL_MS_DEFAULT` and `PB_ML_PER_S_MEASURED`. Until that last one is committed, `status` says `cap=UNCLAMPED` and the measured cap clamp does not exist. If the meter does not pulse at the rig's real flow, `-DPB_DOSE_BY_TIME=1` is the stated fallback.
    - `pump 3000 hang` actually resetting the chip — **bring-up 7c**, which cannot be simulated. Afterwards `status` must say `dry=1` and `last=resetmid`.
    - pump start and dead-head current, measured with a clamp meter during `pump 5000 prime`, and F1 sized from the number — **bring-up 7d**. It is pure hardware: nothing in the firmware and nothing in the host suite bears on it, and no test anywhere in this plan comes near it.
    - `noinit pattern` surviving that reset — **bring-up 7c′**. If it fails, every `.noinit` guarantee in the spec is void: the dry latch, the contradiction latch, `err=resetmid` and the replay high-water mark all become cold-boot-only, and spec §16.5.3 stops being a follow-up and becomes a going-live blocker.

    Put this list in `docs/superpowers/notes/2026-09-03-drop-2-closeout.md`, commit it, and do not let a green `pio test -e native` be read as coverage of any line of it.

---

---

### Task 21: Seam 2: link.h and the scripted fake link

**Drop 3.**

**Files:**

- Create: `include/link.h`, `src/link_fake.cpp`, `test/test_net/test_netfsm.cpp`.
- Modify: `include/sim.h` (append the `link_fake_*` control surface at the end of the file, after
  the `sim_events` declarations), `lib/Network/include/Network.h` (replace the whole file — it is
  24 lines today, `NETWORK_H` guard around a `NetworkClient` class).
- Delete: `lib/Network/src/Network.cpp`.
- Test: `test/test_net/test_netfsm.cpp` (this task creates the suite; the skeleton says "modify",
  but no earlier task creates it).

**Interfaces:**

*Consumes.* From `include/hal.h` (task 3): `uint32_t hal_millis(void);`. From `include/sim.h`
(task 3): `void sim_advance(uint32_t ms);`, `void sim_reset(bool warm);`. From `include/config.h`
(task 2): `PB_NET_STEP_MS` (1200), `PB_TX_CAP` (768). From `test/support/harness.h` (task 3):
`void pb_test_setup(void);`, `void pb_test_teardown(void);`.

*Produces.* `include/link.h`, seam 2, exactly as spec §1 prints it:

```c
typedef enum { LINK_DOWN, LINK_JOINING, LINK_UP } link_state_t;
void         link_begin(uint32_t step_ms);
void         link_join(void);
link_state_t link_state(void);
int8_t       link_rssi(void);
const char  *link_ip(void);
void         link_reset(void);
uint16_t     link_desyncs(void);
bool         sock_open(void);
int          sock_write(const uint8_t *b, size_t n);
int          sock_read(uint8_t *b, size_t cap);
void         sock_close(void);
```

`sock_open()`'s precondition is that `sock_close()` ran. `sock_read()` returns -1 for a closed
socket and 0 for nothing yet. `sock_close()` is idempotent and is called on **every** exit, a
failed open included.

Plus, in `include/sim.h` (the fake rig's header — `link_fake.cpp` is a fake, and the skeleton names
these functions without naming a header; putting them in `link.h` would break "verbatim as spec §1
prints it"):

```c
void        link_fake_reset(void);
void        link_fake_set_state(link_state_t s);
void        link_fake_queue_response(const char *raw, size_t n);
void        link_fake_fail_open(bool fail);
void        link_fake_timeout_next(void);
void        link_fake_drop_link(void);
void        link_fake_pass_begin(void);
uint16_t    link_fake_at_count(void);
bool        link_fake_saw_available(void);
bool        link_fake_saw_connected(void);
uint16_t    link_fake_reset_count(void);
const uint8_t *link_fake_sent(uint16_t *len);   /* addition: the bytes the FSM last wrote */
uint16_t    link_fake_write_count(void);        /* addition: how many sock_write()s have run */
```

**There is no `include/link_fake.h`, and no task creates one.** Every consumer — tasks 24, 25, 26 and 29 — includes `include/sim.h`. If you find `#include "link_fake.h"` anywhere, it is a stale spelling of this decision; change the include, do not create the file.

`link_fake_sent()` and `link_fake_write_count()` are both additions to the skeleton's list. Tasks 24 and 25 need `link_fake_sent()` for `test_report_content_length_matches_the_bytes_actually_written` and `test_report_body_is_byte_identical_on_the_retry`, which cannot be written without reading back what crossed the seam. Task 26 needs `link_fake_write_count()` for `test_no_report_is_built_between_receiving_a_command_and_executing_it`, which has to prove that **nothing** was sent across two whole report intervals — a question `link_fake_sent()` structurally cannot answer, because it returns the last buffer whether or not it was written again. Record both in the commit message.

**A modem timeout has no primitive of its own, and must not get one.** `link.h` is fixed at ten
functions, so the fake signals a timeout the way the real modem does: the call *takes*
`PB_NET_STEP_MS` and returns its failure value. `link_fake_timeout_next()` makes the next AT round
trip advance the fake clock by `PB_NET_STEP_MS`; task 25's `netfsm.cpp` detects it by bracketing
every seam-2 call with `hal_millis()`. Say so in the commit message so nobody adds an eleventh
primitive later.

---

1. [ ] **Write the first failing test.** Create `test/test_net/test_netfsm.cpp`:

```c
/* test/test_net/test_netfsm.cpp — seam 2, the FSM, the AT budget, the retry policy. */
#include <unity.h>
#include <string.h>
#include "../support/harness.h"
#include "config.h"
#include "hal.h"
#include "link.h"
#include "sim.h"

void setUp(void)    { pb_test_setup(); link_fake_reset(); link_begin(PB_NET_STEP_MS); }
void tearDown(void) { pb_test_teardown(); }

static void up(void) {           /* drive the fake link to LINK_UP */
  link_join();
  TEST_ASSERT_EQUAL(LINK_UP, link_state());
}

static void test_sock_close_is_idempotent_and_leaves_the_socket_unallocated(void) {
  up();
  TEST_ASSERT_TRUE(sock_open());
  sock_close();
  link_fake_pass_begin();
  sock_close();                  /* the second close costs nothing and must not fault */
  TEST_ASSERT_EQUAL_UINT16(0, link_fake_at_count());
  TEST_ASSERT_TRUE(sock_open()); /* the precondition holds again only because _sock == -1 */
  sock_close();
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_sock_close_is_idempotent_and_leaves_the_socket_unallocated);
  return UNITY_END();
}
```

2. [ ] **Run it and see it fail to compile.**

```
pio test -e native -f test_net
```

Expected: `test/test_net/test_netfsm.cpp:8:10: fatal error: link.h: No such file or directory`.

3. [ ] **Create `include/link.h`** — seam 2, verbatim from spec §1, with the comments the spec
   prints:

```c
/* link.h — the network seam. NOTHING here names WiFiS3. See spec §1, §3's per-pass AT table. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef enum { LINK_DOWN, LINK_JOINING, LINK_UP } link_state_t;

void         link_begin(uint32_t step_ms);  /* modem.timeout(); modem.begin(); WiFi.setTimeout(0) */
void         link_join(void);               /* 2 ATs, returns; does NOT spin */
link_state_t link_state(void);              /* ONE bounded status query */
int8_t       link_rssi(void);
const char  *link_ip(void);                 /* into a static char[16] */
void         link_reset(void);              /* end(); beginned = false; begin(); counts a desync */
uint16_t     link_desyncs(void);
bool         sock_open(void);               /* HOST_NAME:HTTP_PORT; PRECONDITION: sock_close() ran */
int          sock_write(const uint8_t *b, size_t n);  /* one write, one modem round trip */
int          sock_read(uint8_t *b, size_t cap);       /* -1 closed, 0 nothing yet */
void         sock_close(void);              /* idempotent; EVERY exit, failed open included */
```

Run `pio test -e native -f test_net` again. Expected: it compiles and fails at link with
``undefined reference to `link_begin(unsigned long)'`` and one line per primitive.

4. [ ] **Append the control surface to `include/sim.h`**, at the end of the file, under a comment:

```c
/* ---- seam 2's fake (src/link_fake.cpp). sim + native only. ---- */
void        link_fake_reset(void);
void        link_fake_set_state(link_state_t s);
void        link_fake_queue_response(const char *raw, size_t n);
void        link_fake_fail_open(bool fail);
void        link_fake_timeout_next(void);   /* next AT round trip burns PB_NET_STEP_MS and fails */
void        link_fake_drop_link(void);
void        link_fake_pass_begin(void);     /* zero the per-pass AT counter */
uint16_t    link_fake_at_count(void);
bool        link_fake_saw_available(void);
bool        link_fake_saw_connected(void);
uint16_t    link_fake_reset_count(void);
const uint8_t *link_fake_sent(uint16_t *len);
uint16_t    link_fake_write_count(void);    /* sock_write() calls since link_fake_reset() */
```

`sim.h` must `#include "link.h"` above these for `link_state_t`. **This block is the fake's only header. `include/link_fake.h` is not created by this task or by any other; tasks 26 and 29 include `sim.h`.**

5. [ ] **Create `src/link_fake.cpp`** — everything except `link_reset()`, which step 7 gets wrong
   on purpose:

```c
/* src/link_fake.cpp — seam 2 against a script. [env:native] and [env:uno_r4_wifi_sim] only;
   [env:uno_r4_wifi]'s build_src_filter excludes it. */
#include "link.h"
#include "sim.h"
#include "config.h"
#include <string.h>

#define FAKE_RESP_CAP 512

static link_state_t g_state;
static bool     g_beginned;      /* models ModemClass::beginned (Modem.h:40) */
static bool     g_serial_open;   /* Serial2: opened by begin(), closed by end() */
static bool     g_join_pending;
static int      g_sock;          /* -1 == unallocated, as WiFiClient::_sock */
static bool     g_fail_open;
static bool     g_timeout_next;
static uint16_t g_at;
static uint16_t g_desyncs;
static uint16_t g_resets;
static uint8_t  g_resp[FAKE_RESP_CAP];
static uint16_t g_resp_len, g_resp_pos;
static uint8_t  g_sent[PB_TX_CAP];
static uint16_t g_sent_len;
static uint16_t g_writes;        /* sock_write() calls: "was anything sent at all?" */

/* One AT round trip. false == the modem timed out, and a timeout costs the full step: that
   elapsed time is the ONLY signal netfsm has, because link.h has no timeout primitive (§3). */
static bool at_(void) {
  ++g_at;
  if (!g_serial_open || g_timeout_next) {
    g_timeout_next = false;
    sim_advance(PB_NET_STEP_MS);
    return false;
  }
  sim_advance(1);
  return true;
}

void link_begin(uint32_t step_ms) {
  (void)step_ms;
  if (!g_beginned) { g_beginned = true; g_serial_open = true; }   /* modem.begin() */
}

void link_join(void) { if (at_() && at_()) g_join_pending = true; }   /* 2 ATs, §3's table */

link_state_t link_state(void) {
  if (!at_()) return g_state;
  if (g_join_pending) { g_join_pending = false; g_state = LINK_UP; }
  return g_state;
}

int8_t      link_rssi(void) { return -52; }
const char *link_ip(void)   { static char ip[16] = "192.168.1.42"; return ip; }
uint16_t    link_desyncs(void) { return g_desyncs; }

bool sock_open(void) {
  if (g_sock >= 0) return false;      /* PRECONDITION: sock_close() ran */
  if (g_state != LINK_UP) return false;
  if (!at_()) return false;           /* _BEGINCLIENT */
  g_sock = 1;                         /* getSocket() ALLOCATES before the connect runs */
  if (!at_()) return false;           /* _CLIENTCONNECT */
  return !g_fail_open;                /* a failed connect leaves _sock >= 0 — §3 change 1 */
}

int sock_write(const uint8_t *b, size_t n) {
  if (g_sock < 0) return -1;
  if (!at_()) return -1;
  ++g_writes;                   /* the COUNT, not just the last buffer: task 26's ack-cycle
                                   case has to prove that nothing was sent across two whole
                                   report intervals, and g_sent cannot answer that. */
  if (n > sizeof g_sent) n = sizeof g_sent;
  memcpy(g_sent, b, n);
  g_sent_len = (uint16_t)n;
  return (int)n;
}

int sock_read(uint8_t *b, size_t cap) {
  if (g_sock < 0) return -1;
  if (!at_()) return -1;
  uint16_t left = (uint16_t)(g_resp_len - g_resp_pos);
  if (left == 0) return 0;
  if (cap < left) left = (uint16_t)cap;
  memcpy(b, g_resp + g_resp_pos, left);
  g_resp_pos = (uint16_t)(g_resp_pos + left);
  return (int)left;
}

void sock_close(void) { if (g_sock >= 0) { (void)at_(); g_sock = -1; } }

void link_fake_reset(void) {
  g_state = LINK_DOWN; g_beginned = false; g_serial_open = false; g_join_pending = false;
  g_sock = -1; g_fail_open = false; g_timeout_next = false;
  g_at = 0; g_desyncs = 0; g_resets = 0;
  g_resp_len = 0; g_resp_pos = 0; g_sent_len = 0; g_writes = 0;
}
void link_fake_set_state(link_state_t s) { g_state = s; if (s == LINK_UP) g_join_pending = false; }
void link_fake_queue_response(const char *raw, size_t n) {
  if (n > sizeof g_resp) n = sizeof g_resp;
  memcpy(g_resp, raw, n); g_resp_len = (uint16_t)n; g_resp_pos = 0;
}
void link_fake_fail_open(bool fail) { g_fail_open = fail; }
void link_fake_timeout_next(void)   { g_timeout_next = true; }
void link_fake_drop_link(void)      { g_state = LINK_DOWN; g_join_pending = false; g_sock = -1; }
void link_fake_pass_begin(void)     { g_at = 0; }
uint16_t link_fake_at_count(void)   { return g_at; }
uint16_t link_fake_reset_count(void){ return g_resets; }
/* link.h has no available()/connected() primitive at all, so these can never become true from
   above the seam -- they are hard-coded false, and the case that asserts them
   (test_sock_read_calls_neither_available_nor_connected) is therefore asserting a TAUTOLOGY.
   Say so plainly rather than reading it as coverage: it documents the seam's shape, and the
   real check is the AT budget beside it. The half that could fail is the DRIVER's, and no
   host test can see it, because link_fake is not WiFiClient; task 28's two wall-clock cases
   time a RECV pass and a stale-socket open on real silicon, which is the closest anything in
   this plan comes to catching a driver that quietly started polling. */
bool link_fake_saw_available(void)  { return false; }
bool link_fake_saw_connected(void)  { return false; }
const uint8_t *link_fake_sent(uint16_t *len) { if (len) *len = g_sent_len; return g_sent; }
uint16_t link_fake_write_count(void) { return g_writes; }
```

Leave `link_reset()` out for now — step 7 adds it wrong first.

6. [ ] **Run and see it pass, then commit.**

```
pio test -e native -f test_net
```

Expected: `1 Tests 0 Failures 0 Ignored` and `OK`. Then:

```
git add include/link.h include/sim.h src/link_fake.cpp test/test_net/test_netfsm.cpp
git commit -m "Seam 2: link.h, and a fake link that scripts the backend

link_fake_sent() and link_fake_write_count() are additions to the ten primitives:
tasks 24 and 25 cannot assert Content-Length or byte-identical retries without
reading back what crossed the seam, and task 26 cannot prove that NOTHING was sent
across two report intervals without a send counter. A modem timeout deliberately
gets NO primitive — the fake burns PB_NET_STEP_MS and fails, which is the only
signal the real modem gives either.

The link_fake_* control surface lives at the end of include/sim.h, not in a header
of its own: link.h is seam 2 verbatim as spec 1 prints it, and a fake's injectors
are sim.h's business. There is no include/link_fake.h anywhere in this tree."
```

7. [ ] **Write the AT-counting test.** Append to `test/test_net/test_netfsm.cpp` and add its
   `RUN_TEST` line:

```c
static void test_the_fake_counts_at_commands_per_pass(void) {
  link_fake_pass_begin();
  link_join();                                    /* WiFi.cpp:43-67 — 2 ATs, never 3 */
  TEST_ASSERT_EQUAL_UINT16(2, link_fake_at_count());
  link_fake_pass_begin();
  TEST_ASSERT_EQUAL(LINK_UP, link_state());       /* status() — 1 AT */
  TEST_ASSERT_EQUAL_UINT16(1, link_fake_at_count());
  link_fake_pass_begin();
  TEST_ASSERT_TRUE(sock_open());                  /* _BEGINCLIENT + _CLIENTCONNECT — 2 ATs */
  TEST_ASSERT_EQUAL_UINT16(2, link_fake_at_count());
  link_fake_pass_begin();
  sock_close();                                   /* _CLIENTCLOSE — 1 AT */
  TEST_ASSERT_EQUAL_UINT16(1, link_fake_at_count());
  link_fake_pass_begin();
  sock_close();                                   /* _sock == -1 — 0 ATs */
  TEST_ASSERT_EQUAL_UINT16(0, link_fake_at_count());
}
```

Run `pio test -e native -f test_net`. It passes as written (the accounting was built in step 5) —
that is the point of the case: it pins the per-pass counts of spec §3's table so a later change to
the fake cannot quietly make the budget look affordable. If it does not pass, the counts in
`at_()`'s call sites are wrong; fix them, not the test.

8. [ ] **Commit.**

```
git commit -am "The fake counts AT commands per pass, to spec 3's table"
```

9. [ ] **Write the `beginned` test, and the buggy `link_reset()` it must catch.** Add to
   `src/link_fake.cpp` the version that omits the one line — this is deliberate, and step 11
   fixes it:

```c
void link_reset(void) {
  g_serial_open = false;        /* modem.end() — Modem.cpp:45-48 */
  g_beginned = true;            /* BUG (step 11): end() never clears it, and begin() is guarded */
  if (!g_beginned) { g_beginned = true; g_serial_open = true; }   /* modem.begin() */
  g_state = LINK_DOWN; g_join_pending = false; g_sock = -1;
  ++g_desyncs; ++g_resets;
}
```

and the test:

```c
static void test_a_second_link_reset_still_produces_a_working_at_round_trip(void) {
  up();
  link_reset();
  TEST_ASSERT_EQUAL_UINT16(1, link_fake_reset_count());
  TEST_ASSERT_EQUAL_UINT16(1, link_desyncs());
  link_join();                          /* must resync: 2 ATs into a REOPENED UART */
  TEST_ASSERT_EQUAL(LINK_UP, link_state());
  link_reset();                         /* and again — this is the one that used to die */
  link_join();
  TEST_ASSERT_EQUAL(LINK_UP, link_state());
  TEST_ASSERT_EQUAL_UINT16(2, link_fake_reset_count());
}
```

10. [ ] **Run it and see it fail.**

```
pio test -e native -f test_net
```

Expected:
`test/test_net/test_netfsm.cpp:NN:test_a_second_link_reset_still_produces_a_working_at_round_trip:FAIL: Expected 2 Was 0`
— `LINK_UP` is 2, and after the reset every AT writes into a UART `begin()` declined to reopen, so
`link_state()` returns `LINK_DOWN`. This is the 48-hour failure in one line: `ch206` climbs and the
board silently stops reporting.

11. [ ] **Fix the one line.** In `src/link_fake.cpp`:

```c
  g_beginned = false;           /* Modem.cpp:45-48 never does this. Without it, begin() no-ops. */
```

Run `pio test -e native -f test_net`. Expected: `3 Tests 0 Failures 0 Ignored`.

12. [ ] **Commit.**

```
git commit -am "link_reset() clears beginned, and a test that fails without it

ModemClass::end() closes Serial2 and never clears beginned; begin() is guarded by
it. The obvious end(); begin(); therefore closes the UART and declines to reopen
it: every later AT times out, ch206 climbs, and the board stops reporting with
status still printing wifi UP. Spec 1, 3, 15.4."
```

13. [ ] **Retire the old `lib/Network` API.** Replace `lib/Network/include/Network.h` in full:

```c
/* Network.h — kept only so an old include still resolves. The library's whole surface is
   seam 2 now; lib/Network/src/link_wifi.cpp (task 27) is its only implementation, and it is
   the only file in the tree that may name WiFiS3. See spec §1. */
#pragma once
#include "link.h"
```

and delete the implementation:

```
git rm lib/Network/src/Network.cpp
```

That removes `NetworkClient`, its `std::map`/`String` API, the two `while (true)` spins and the
leaked socket in one commit.

14. [ ] **Prove both gates.**

```
pio test -e native
pio run -e uno_r4_wifi
```

Both must pass. `pio run -e uno_r4_wifi` still links because nothing on the device references seam 2
yet: `main.cpp` gains `net_poll()` only in task 26, and `netfsm.cpp` does not exist until task 24.

15. [ ] **Commit.**

```
git add -A lib/Network
git commit -m "lib/Network is seam 2 and nothing else

NetworkClient, std::map<std::string,float>, String, the two while(true) spins and
the leaked socket are gone. link_wifi.cpp lands in task 27; until then the device
build simply has no caller."
```

---

### Task 22: report_build() — the body, the ten diagnostic channels, and every field butler can 400 on

**Drop 3.**

**Files:**

- Create: `include/report.h`, `src/report.cpp`, `test/test_report/test_report.cpp`.
- Modify: `src/cli.cpp` — one `#include "report.h"` plus **three** lines inside
  `cli_print_status()`, after the existing `pos: FORCED unknown (PB_REPORT_POS_UNKNOWN=1)`
  line (step 18).
- Modify: `include/sim.h` and `src/hal_sim.cpp` — one heap injector, `sim_set_heap_break()` (`sim_set_clock_ms()` already exists: task 14 step 1 declares and defines it, and this task's step 13 only consumes it)
  (step 13). The fake's one-millisecond-per-step contract is NOT changed.
- Test: `test/test_report/test_report.cpp`.

**Interfaces:**

*Consumes.*
From `include/sensors.h` (task 7): `uint16_t sensors_value(uint8_t ch);`,
`bool sensors_valid(uint8_t ch);`, `bool sensors_stuck(void);`,
`uint32_t sensors_i2c_errors(void);`, `uint32_t sensors_float_change_age_s(void);`,
`bool sensors_i2c_healthy(void);`, `bool sensors_begin(void);`, `bool sensors_sweep(void);`.
From `include/pulses.h` (task 6): `uint32_t pulses_leak_count(void);`, `bool pulses_leak_seen(void);`.
From `include/safety.h` (tasks 15, 17, 19): `bool safety_float_ok_debounced(void);`,
`bool safety_dry(void);`, `bool safety_float_flap(void);`,
`void safety_float_refusal_count(bool refused_for_float);`, `bool safety_contra(void);`,
`const char *safety_last_err(void);`, `void safety_set_err(const char *tok);`.
From `lib/Manifold/include/cart.h` (task 14): `bool cart_pos_known(void);`, `bool cart_parked(void);`.
From `include/hal.h` (task 3): `uint32_t hal_heap_arena(void);`, `uint32_t hal_heap_ordblks(void);`,
`uint32_t hal_stack_hwm(void);`, `uint32_t hal_wdt_last_delta(void);`, `uint32_t hal_boot_salt(void);`,
`uint32_t hal_millis(void);`, and — for `report_heap_ok()` — `uint32_t hal_heap_break(void);`,
`uint32_t hal_stack_limit(void);`.
From `include/link.h` (task 21): `uint16_t link_desyncs(void);`.
From `include/config.h` (task 2): `PB_CHANNELS` 6, `PB_CANARY_CHANNEL` 15, `PB_BODY_CAP` 384,
`PB_BODY_WORST_FIXED` 288, `PB_DIAG_CLAMP` 999999, `PB_REPORT_POS_UNKNOWN` 1, `PB_DOSE_MAX_ML` 1000.
`PB_CONTROLLER` is `"test1"` under `[env:native]`.

*Produces.* `include/report.h`:

```c
typedef struct { uint32_t id; uint16_t flow_ml; const char *err; } report_ack_t;
void     report_stamp(void);
uint32_t report_t_wire(void);
uint32_t report_t_ms(void);
uint16_t report_build(char *buf, uint16_t cap);   /* 0 == truncated: DROP, err=txcap */
void     report_set_ack(uint32_t id, uint16_t flow_ml, const char *err);
void     report_clear_ack(void);
bool     report_ack_is_recv(void);
bool     report_may_build(void);
uint16_t report_last_len(void);      /* addition: `status` prints it */
uint32_t report_txcap_drops(void);   /* addition: §4.2 asks status to say so LOUDLY */
bool     report_heap_ok(void);       /* addition: spec §12 item 0's per-report break check */
```

The last three are additions to the skeleton's list. Spec §4.2 requires `status` to announce a
truncation drop and there is no other way to reach the count. And **spec §12 item 0 is explicit
that "`hal_begin()` and **every report** check the break against the stack, because nothing else
will"** — `_sbrk` is the unchecked libnosys version and `__HeapLimit` is referenced by nothing in
the image, so this is the only heap bound that exists during the 48-hour run, and the run is
exactly when the largest allocator in the program (the network stack) is active. Task 12 step 3
does the boot half; `report_heap_ok()` is the per-report half, and without it §12 item 0 is half
implemented. Record all three in the commit message.

Three `static_assert`s live in this file (spec §4.2); the tx-buffer one lives in `netfsm.cpp`
(task 24) because that is where the tx buffer is.

**The `status` line "reports ok/failed" is NOT added here.** It needs `net_reports_ok()` /
`net_reports_failed()`, which task 24 produces; the skeleton put it on this task by mistake. Task 24
adds it.

---

1. [ ] **Write the first three failing cases.** Create `test/test_report/test_report.cpp`:

```c
/* test/test_report/test_report.cpp — report_build() and response_parse(). */
#include <unity.h>
#include <stdio.h>
#include <string.h>
#include "../support/harness.h"
#include "config.h"
#include "hal.h"
#include "sim.h"
#include "sensors.h"
#include "pulses.h"
#include "safety.h"
#include "report.h"

static char g_buf[PB_BODY_CAP];

void setUp(void)    { pb_test_setup(); sensors_begin(); memset(g_buf, 0, sizeof g_buf); }
void tearDown(void) { pb_test_teardown(); }

/* A clean sweep: six wired channels with distinct values, and a canary that matches none. */
static void fresh_sweep(void) {
  for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) sim_set_channel(ch, (uint16_t)(8000 + ch));
  sim_set_channel(PB_CANARY_CHANNEL, 1);
  TEST_ASSERT_TRUE(sensors_sweep());
}

static uint16_t build(void) { report_stamp(); return report_build(g_buf, sizeof g_buf); }

/* whole-token match: "ch1=8001" must not be found inside "ch11=8001" */
static bool has_tok(const char *tok) {
  size_t n = strlen(tok);
  for (const char *p = strstr(g_buf, tok); p; p = strstr(p + n, tok)) {
    bool left  = (p == g_buf) || p[-1] == ' ';
    bool right = (p[n] == ' ' || p[n] == '\n' || p[n] == '\0');
    if (left && right) return true;
  }
  return false;
}

static bool has_key(const char *key) {   /* key includes the '=' */
  size_t n = strlen(key);
  for (const char *p = strstr(g_buf, key); p; p = strstr(p + n, key))
    if (p == g_buf || p[-1] == ' ') return true;
  return false;
}

static void test_report_carries_c_t_and_the_valid_channels(void) {
  fresh_sweep();
  TEST_ASSERT_TRUE(build() > 0);
  char t[32];
  snprintf(t, sizeof t, "t=%lu", (unsigned long)report_t_wire());
  TEST_ASSERT_TRUE(has_tok("c=" PB_CONTROLLER));
  TEST_ASSERT_TRUE(has_tok(t));
  TEST_ASSERT_TRUE(has_tok("ch0=8000"));
  TEST_ASSERT_TRUE(has_tok("ch5=8005"));
  TEST_ASSERT_EQUAL_CHAR('\n', g_buf[strlen(g_buf) - 1]);
}

static void test_report_always_carries_at_least_one_diagnostic_channel(void) {
  sim_set_i2c_fail(true);                 /* a wedged bus empties the mux mask entirely */
  (void)sensors_sweep();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(has_key("ch0="));
  TEST_ASSERT_TRUE(has_key("ch203="));    /* butler 400s a report with no chN= at all */
}

static void test_report_omits_a_channel_whose_read_failed_rather_than_sending_zero(void) {
  fresh_sweep();
  sim_set_i2c_fail(true);
  (void)sensors_sweep();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(has_key("ch2="));
  TEST_ASSERT_FALSE(has_tok("ch2=0"));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_report_carries_c_t_and_the_valid_channels);
  RUN_TEST(test_report_always_carries_at_least_one_diagnostic_channel);
  RUN_TEST(test_report_omits_a_channel_whose_read_failed_rather_than_sending_zero);
  return UNITY_END();
}
```

2. [ ] **Run and see it fail.**

```
pio test -e native -f test_report
```

Expected: `test/test_report/test_report.cpp:12:10: fatal error: report.h: No such file or directory`.

3. [ ] **Create `include/report.h`** with the block printed under *Produces* above, wrapped in
   `#pragma once` and `#include <stdbool.h> <stdint.h>`, with this comment over `report_build`:

```c
/* Builds the whole body into buf. Returns the byte count, or 0 on truncation — which means
   err=txcap, the report is DROPPED, and `status` says so loudly (spec §4.2). Every key appears
   at most once: a repeated c, t, ack, flow_ml, float, pos or chN refuses the WHOLE report
   (butler.py:220-250), and the body is assembled from four independent sources. */
```

4. [ ] **Create `src/report.cpp`** — the skeleton, `c=`, `t=`, the wired channels and the ten
   diagnostics:

```c
/* src/report.cpp — the wire protocol, pure. Where the traps live (spec §4).
   NO signed integer conversion anywhere: t = hal_boot_salt() + hal_millis() is above 2^31 on
   ordinary boots, so a single percent-d against a uint32_t prints a leading '-', _int_in
   rejects it, and EVERY report 400s from the first one (spec §15.2, §9's grep). Every numeric
   site is %lu with an explicit cast. */
#include "report.h"
#include "config.h"
#include "hal.h"
#include "link.h"
#include "sensors.h"
#include "pulses.h"
#include "safety.h"
#include "cart.h"
#include "secrets.h"   /* PB_CONTROLLER: the two static_asserts below and the c= field.
                          [env:native] and [env:uno_r4_wifi_sim] pass it in build_flags;
                          the two device envs do not, and this header is its only other
                          definition. Its #ifndef guard makes both routes agree. */
#include <stdio.h>
#include <string.h>

static_assert(sizeof(PB_CONTROLLER) + 2 + PB_BODY_WORST_FIXED <= PB_BODY_CAP,
              "the body's own worst case does not fit PB_BODY_CAP (spec §7's term-by-term sum)");
static_assert(sizeof(PB_CONTROLLER) > 1,
              "an empty c= is 'no c= in the report': a permanent 400 (butler.py:252-253)");

static uint32_t g_t_wire, g_t_ms;
static report_ack_t g_ack;
static bool     g_ack_set;
static uint16_t g_last_len;
static uint32_t g_txcap_drops;

void     report_stamp(void)  { g_t_ms = hal_millis(); g_t_wire = hal_boot_salt() + g_t_ms; }
uint32_t report_t_wire(void) { return g_t_wire; }
uint32_t report_t_ms(void)   { return g_t_ms; }
uint16_t report_last_len(void)    { return g_last_len; }
uint32_t report_txcap_drops(void) { return g_txcap_drops; }

static bool put_s(char *b, uint16_t cap, uint16_t *n, const char *fmt, const char *v) {
  if (*n >= cap) return false;
  int w = snprintf(b + *n, (size_t)(cap - *n), fmt, v);
  if (w < 0 || (uint16_t)w >= (uint16_t)(cap - *n)) return false;
  *n = (uint16_t)(*n + w);
  return true;
}
static bool put_u(char *b, uint16_t cap, uint16_t *n, const char *fmt, uint32_t v) {
  if (*n >= cap) return false;
  int w = snprintf(b + *n, (size_t)(cap - *n), fmt, (unsigned long)v);
  if (w < 0 || (uint16_t)w >= (uint16_t)(cap - *n)) return false;
  *n = (uint16_t)(*n + w);
  return true;
}
static bool put_ch(char *b, uint16_t cap, uint16_t *n, uint32_t ch, uint32_t v) {
  if (*n >= cap) return false;
  int w = snprintf(b + *n, (size_t)(cap - *n), " ch%lu=%lu",
                   (unsigned long)ch, (unsigned long)v);
  if (w < 0 || (uint16_t)w >= (uint16_t)(cap - *n)) return false;
  *n = (uint16_t)(*n + w);
  return true;
}

/* Spec §12 item 0: "hal_begin() and EVERY REPORT check the break against the stack, because
   nothing else will." _sbrk is the unchecked libnosys version and __HeapLimit is referenced by
   nothing in the image, so this is the only heap bound that exists — and the 48-hour run is
   exactly when the largest allocator in the program, the network stack, is active. Returns
   false once the margin is crossed, and latches err=heap so the fact reaches the wire.

   It does NOT call net_disable() itself: that flag lives in netfsm.cpp, this file must not
   grow a dependency on a translation unit that does not exist until task 24, and report.cpp
   is compiled into every host suite. Task 24's NET_IDLE pass is the one caller that turns a
   false into `net_disable("heap")`; this function's job is the measurement and the token. */
bool report_heap_ok(void) {
  if (hal_heap_break() < hal_stack_limit() - (uint32_t)PB_STACK_MARGIN) return true;
  safety_set_err("heap");
  return false;
}

uint16_t report_build(char *buf, uint16_t cap) {
  uint16_t n = 0;
  bool ok = true;
  (void)report_heap_ok();      /* §12 item 0's per-report half. It does not abort the body:
                                  a report that says err=heap is worth more than no report. */
  const bool stuck = sensors_stuck();

  ok = ok && put_s(buf, cap, &n, "c=%s", PB_CONTROLLER);
  ok = ok && put_u(buf, cap, &n, " t=%lu", g_t_wire);

  if (!stuck) {                       /* §5: a stuck mux omits the WIRED channels, not the body */
    for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch)
      if (sensors_valid(ch))
        ok = ok && put_ch(buf, cap, &n, ch, sensors_value(ch));
  }

  /* ch200..ch209, every one clamped: chN must be < MAX_RAW = 2^31 (butler.py:88,251), and a
     storming D2 pushes ch205 past 2^31 in ~12.4 days. At least one is ALWAYS present, so a
     wedged bus produces an alarm instead of silence (§4.1). */
  const uint32_t diag[10] = {
    hal_heap_arena(), hal_heap_ordblks(), hal_stack_hwm(),
    sensors_i2c_errors(), sensors_float_change_age_s(), pulses_leak_count(),
    (uint32_t)link_desyncs(), safety_contra() ? 1u : 0u,
    cart_parked() ? 1u : 0u, hal_wdt_last_delta()
  };
  for (uint32_t i = 0; i < 10; ++i) {
    uint32_t v = diag[i] > (uint32_t)PB_DIAG_CLAMP ? (uint32_t)PB_DIAG_CLAMP : diag[i];
    ok = ok && put_ch(buf, cap, &n, 200u + i, v);
  }

  ok = ok && put_s(buf, cap, &n, "%s", "\n");
  if (!ok) { ++g_txcap_drops; safety_set_err("txcap"); return 0; }
  /* A truncation set err=txcap, and nothing else in the program ever clears it. One 384-byte
     body would otherwise put txcap on EVERY later report forever, long after the condition
     that caused it. Only this function's own token is cleared, and only by a body that fit —
     the boot tokens (wdt, adc, heap) and the dose tokens are none of this function's business. */
  if (strcmp(safety_last_err(), "txcap") == 0) safety_set_err("none");
  g_last_len = n;
  return n;
}
```

Add stubs so the file links: `void report_set_ack(uint32_t, uint16_t, const char *) {}`,
`void report_clear_ack(void) {}`, `bool report_ack_is_recv(void) { return false; }`,
`bool report_may_build(void) { return true; }` — steps 13-14 implement them.

5. [ ] **Run and see the three cases pass, then commit.**

```
pio test -e native -f test_report
```

Expected `3 Tests 0 Failures`. Then:

```
git add include/report.h src/report.cpp test/test_report/test_report.cpp
git commit -m "report_build(): c=, t=, the wired channels and ten clamped diagnostics

report_heap_ok() is an addition to the skeleton's list, and it is spec 12 item 0's
other half: 'hal_begin() and EVERY REPORT check the break against the stack,
because nothing else will'. _sbrk is unchecked, __HeapLimit is referenced by
nothing in the image, and the 48-hour run is exactly when the network stack is
active. It latches err=heap and returns false; netfsm's NET_IDLE pass is what
turns that into net_disable(\"heap\"), because the flag lives there and report.cpp
must not depend on a translation unit that does not exist yet.

Every diagnostic is min(v, PB_DIAG_CLAMP): chN must stay under MAX_RAW = 2^31 and
ch205 would cross it in ~12.4 days of a storming D2, after which every report is
400ed on one token. At least one diagnostic is always present, so a wedged bus
that empties the mux mask still sends a legal report carrying float=, pos= and
err= instead of sending nothing at all."
```

6. [ ] **Write the stuck-canary case and the float cases.** Add to the test file (and their
   `RUN_TEST` lines):

```c
static void test_report_omits_the_wired_channels_and_says_stuck_when_the_canary_matches(void) {
  for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) sim_set_channel(ch, 7777);
  sim_set_channel(PB_CANARY_CHANNEL, 7777);      /* unpowered mux / floating EN / broken S-line */
  sim_set_mux_stuck(true);
  /* FALSE, not TRUE: task 7's contract is that every failure returns false, and the canary
     matching every wired channel is one. The report must still be LEGAL on a false sweep -
     that is the whole point of the diagnostics - which is what the assertions below check. */
  TEST_ASSERT_FALSE(sensors_sweep());
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(has_key("ch0="));
  TEST_ASSERT_FALSE(has_key("ch4="));
  TEST_ASSERT_TRUE(has_tok("err=stuck"));
  TEST_ASSERT_TRUE(has_key("ch200="));
}

static void test_report_float_is_the_debounced_tank_verdict_anded_with_not_contra(void) {
  fresh_sweep();
  sim_set_float(true);
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("float=1"));
  sim_set_float(false);
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("float=0"));
}

static void test_report_float_is_only_ever_zero_or_one(void) {
  fresh_sweep();
  for (int i = 0; i < 6; ++i) {                  /* a float flapping at the waterline */
    sim_set_float(i % 2 == 0);
    TEST_ASSERT_TRUE(build() > 0);
    TEST_ASSERT_TRUE(has_tok("float=0") || has_tok("float=1"));
    TEST_ASSERT_FALSE(has_tok("float=2"));       /* _int_in(v,"float",0,2) is HALF-open */
    TEST_ASSERT_FALSE(has_tok("float=-1"));
  }
}

static void test_repeated_float_refusals_drive_float_to_zero_on_the_wire(void) {
  fresh_sweep();
  sim_set_float(true);
  for (int i = 0; i < PB_FLOAT_FLAP_LIMIT + 1; ++i) safety_float_refusal_count(true);
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("float=0"));          /* even though the tank samples OK */
}

static void test_a_granted_dose_clears_the_float_refusal_counter(void) {
  fresh_sweep();
  sim_set_float(true);
  for (int i = 0; i < PB_FLOAT_FLAP_LIMIT + 1; ++i) safety_float_refusal_count(true);
  safety_float_refusal_count(false);             /* any granted dose clears it */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("float=1"));
}
```

7. [ ] **Run and see them fail.**

```
pio test -e native -f test_report
```

Expected:
`test/test_report/test_report.cpp:NN:test_report_float_is_the_debounced_tank_verdict_anded_with_not_contra:FAIL: Expected TRUE Was FALSE`
— there is no `float=` in the body yet.

8. [ ] **Add `float=` and the stuck `err=` to `report_build()`**, after the diagnostics loop and
   before the trailing newline. Spec §4.1 gives `float=` exactly one definition, and this is it:

```c
  /* §2.10, §4.1: the DEBOUNCED tank verdict, ANDed with !contra, forced to 0 above
     PB_FLOAT_FLAP_LIMIT consecutive DOSE_REFUSED_FLOAT results. Never 2, never negative:
     _int_in(v,"float",0,2) is half-open and ASCII-digits-only. */
  const bool fl = safety_float_ok_debounced() && !safety_contra() && !safety_float_flap();
  ok = ok && put_u(buf, cap, &n, " float=%lu", fl ? 1u : 0u);
  ok = ok && put_s(buf, cap, &n, " err=%s", stuck ? "stuck" : "none");
```

(the `leak` and `safety_last_err()` sources arrive in step 12, together with the ack pair.)

(The `err=` source becomes real in step 14; this line only makes the stuck case honest now.)

Run `pio test -e native -f test_report`. Expected `8 Tests 0 Failures`. Commit:

```
git commit -am "float= is the debounced verdict AND not-contra, forced 0 on flap

Not a raw pin read: a float bouncing at the waterline satisfies one sample and
fails three, so the raw version reports float=1, water_rules queues, dose_run
refuses, the acked refusal pages HIGH and sets the pot's cooldown — forever.
Spec 2.10."
```

9. [ ] **Write the three `pos=` cases.**

```c
static void test_report_pos_is_unknown_while_the_going_live_flag_is_set(void) {
  fresh_sweep();
  TEST_ASSERT_EQUAL_INT(1, PB_REPORT_POS_UNKNOWN);   /* ships defined — §4.6 */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("pos=unknown"));
  TEST_ASSERT_FALSE(has_tok("pos=ok"));
}

static void test_report_pos_is_unknown_while_the_dry_latch_is_set(void) {
  fresh_sweep();
  safety_dry_set(true);
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("pos=unknown"));
  safety_dry_set(false);
}

static void test_report_pos_is_unknown_when_the_gate_pitch_is_uncalibrated(void) {
  fresh_sweep();
  TEST_ASSERT_EQUAL_INT(0, PB_PULSES_PER_GATE);      /* bring-up 6 has not run */
  TEST_ASSERT_FALSE(cart_pos_known());
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("pos=unknown"));
}
```

(add `#include "cart.h"` to the test file). Run: expected
`...test_report_pos_is_unknown_while_the_going_live_flag_is_set:FAIL: Expected TRUE Was FALSE`.

10. [ ] **Add `pos=`** to `report_build()`, immediately after the `float=` line:

```c
  /* §4.1, §2.10, §4.6: unknown UNCONDITIONALLY while the going-live flag is defined AND
     unconditionally while the dry latch is set (otherwise water_rules queues doses the board
     will refuse and ack, paging HIGH once per cooldown, forever). Otherwise ok only when the
     gate pitch is calibrated, a home has been seen since boot, and the last expander read
     succeeded. */
#if PB_REPORT_POS_UNKNOWN
  const bool pos_ok = false;
#else
  const bool pos_ok = !safety_dry() && cart_pos_known() && sensors_i2c_healthy();
#endif
  ok = ok && put_s(buf, cap, &n, " pos=%s", pos_ok ? "ok" : "unknown");
```

Because `PB_REPORT_POS_UNKNOWN` ships as 1, the `#else` arm is not compiled today.
`test_report_pos_is_unknown_while_the_dry_latch_is_set` and
`..._when_the_gate_pitch_is_uncalibrated` therefore pass through the `#if` arm; keep them, they
are what fails loudly on the day the flag is flipped. Say so in the commit message.

Run `pio test -e native -f test_report` — expected `11 Tests 0 Failures`. Commit:

```
git commit -am "pos= is unknown under the going-live flag and under the dry latch

While PB_REPORT_POS_UNKNOWN is 1 the #else arm is not compiled at all, so the two
dry/uncalibrated cases pass through the flag today; they are the cases that fail
the day it is flipped without cart_pos_known() being real. Spec 4.6, 2.10."
```

11. [ ] **Write the ack-pair cases.**

```c
static void test_report_omits_flow_ml_when_there_is_no_ack(void) {
  fresh_sweep();
  report_clear_ack();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(has_key("ack="));
  TEST_ASSERT_FALSE(has_key("flow_ml="));
}

static void test_report_never_emits_ack_without_flow_ml(void) {
  fresh_sweep();
  report_set_ack(17, 0, "float");        /* a refusal: flow_ml is 0, and MUST be present */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("ack=17"));
  TEST_ASSERT_TRUE(has_tok("flow_ml=0"));
  TEST_ASSERT_TRUE(has_tok("err=float"));
}

static void test_report_never_emits_ack_zero(void) {
  fresh_sweep();
  report_set_ack(0, 0, "none");          /* ack is _int_in(v,"ack",1,2**63): 0 400s the report */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(has_key("ack="));
}

static void test_report_ack_id_survives_above_sixty_five_thousand(void) {
  fresh_sweep();
  report_set_ack(4294967295u, 1000, "none");
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("ack=4294967295"));
  TEST_ASSERT_TRUE(has_tok("flow_ml=1000"));
}
```

Run: expected
`...test_report_never_emits_ack_without_flow_ml:FAIL: Expected TRUE Was FALSE`.

12. [ ] **Implement the ack slot and the pair.** Replace the four stubs in `src/report.cpp`:

```c
/* butler's ack UPDATE writes flow_ml = ? unconditionally (butler.py:830), so an ack= without a
   flow_ml= stores NULL, charges the pot the FULL ml against its daily cap (COALESCE(flow_ml, ml),
   :745-751) and skips the 2*flow_ml < ml branch entirely. Structurally one pair, both ways. */
void report_set_ack(uint32_t id, uint16_t flow_ml, const char *err) {
  g_ack.id = id; g_ack.flow_ml = flow_ml; g_ack.err = err;
  g_ack_set = (id != 0);                 /* ack=0 400s the whole report; never emit it */
}
void report_clear_ack(void) { g_ack_set = false; g_ack.id = 0; g_ack.flow_ml = 0; g_ack.err = 0; }
bool report_ack_is_recv(void) {
  return g_ack_set && g_ack.err && strcmp(g_ack.err, "recv") == 0;
}
/* §4.3: no report may be built while a command is pending and the ack slot still reads "recv".
   Without this the placeholder reaches the wire, butler marks the command acked with flow_ml=0,
   pages HIGH, sets the pot's cooldown and charges 0 ml — and THEN the board runs the dose. */
bool report_may_build(void) { return !report_ack_is_recv(); }
```

and, in `report_build()`, replace the provisional `err=` line with the ack pair followed by the
real `err=`:

```c
  const char *err;
  if (g_ack_set) {
    ok = ok && put_u(buf, cap, &n, " ack=%lu", g_ack.id);
    ok = ok && put_u(buf, cap, &n, " flow_ml=%lu", (uint32_t)g_ack.flow_ml);
    err = g_ack.err;
  } else {
    /* §1: pulses with the pump off raise ch205 and err=leak, and they never block a dose.
       This is that token's ONLY producer in the program: pulses_leak_poll() (driven from
       loop(), task 12 step 4) advances the count, and this line puts it on the wire. Below
       `stuck`, because a mux that is lying about every channel is the larger fact. */
    err = stuck ? "stuck" : (pulses_leak_seen() ? "leak" : safety_last_err());
  }
  if (!err || !*err) err = "none";
  ok = ok && put_s(buf, cap, &n, " err=%s", err);
```

**This is the fix for two defects a review found: `err=` had no source at all, because
`report_build()` never consumed `safety_last_err()`, and `err=leak` — a token in §4.1's fixed
enum — had no producer anywhere.** Both are closed here, and the precedence is: the ack's own
token when there is an ack, `stuck` when the canary matched, `leak` when the leak watch has
counted a pulse, otherwise the board's last error token, otherwise `none`. Note it in the commit
message.

Add at the top of `report_build()`:

```c
  if (!report_may_build()) return 0;    /* err=recv must never reach the wire (§4.3) */
```

Run `pio test -e native -f test_report` — expected `15 Tests 0 Failures`. Commit:

```
git commit -am "The ack pair, and err= finally has a source

ack= and flow_ml= are structurally one pair in both directions, and err= is the
ack's token when there is an ack, else `stuck`, else `leak`, else
safety_last_err(), else none. A previous draft declared safety_last_err() and
never consumed it, so every report would have carried err=none while status
printed the real reason - and err=leak, a token in 4.1's fixed enum, had no
producer anywhere in the program.

report_build() also clears its OWN txcap token on the next body that fits.
Nothing else ever cleared it, so one 384-byte report would have put err=txcap on
every later report for the rest of the run."
```

13. [ ] **Write the `t=` cases.** These are the ones that would otherwise 400 every report from
   the first one.

```c
static void test_report_t_is_unsigned_at_and_above_two_to_the_thirty_one(void) {
  fresh_sweep();
  const uint32_t targets[3] = { 0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFFu };
  for (int i = 0; i < 3; ++i) {
    /* JUMP the clock; do NOT sim_advance() 2^31 times. sim_set_clock_ms() is step 13's
       injector and it deliberately does not run the edge emitters. */
    sim_set_clock_ms((uint32_t)(targets[i] - hal_boot_salt()));
    TEST_ASSERT_TRUE(build() > 0);
    TEST_ASSERT_EQUAL_UINT32(targets[i], report_t_wire());
    char t[32];
    snprintf(t, sizeof t, "t=%lu", (unsigned long)targets[i]);
    TEST_ASSERT_TRUE(has_tok(t));
    TEST_ASSERT_NULL(strstr(g_buf, "t=-"));   /* a single %d here 400s EVERY report, forever */
  }
}

static void test_report_t_differs_across_two_boots_fifteen_seconds_apart(void) {
  sim_reset(true);                    /* warm: the .noinit boot counter advances */
  sensors_begin(); fresh_sweep();
  sim_advance(15000);
  TEST_ASSERT_TRUE(build() > 0);
  const uint32_t first = report_t_wire();
  sim_reset(true);
  sensors_begin(); fresh_sweep();
  sim_advance(15000);
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_NOT_EQUAL(first, report_t_wire());   /* else butler swallows the 2nd as a retry */
}
```

These two cases need a clock at 2^31 and beyond. **Do NOT make `sim_advance()` O(1) to get
there.** Task 3 step 5's `sim.h` states the fake's clock contract in full — one millisecond per
step, running every model — and task 6 step 6 hangs the flow and screw edge emitters inside
`advance_1ms_()` precisely so an edge lands at its own microsecond; an O(1) `sim_advance()`
silently stops emitting pulse edges and breaks task 6's rate case, task 14's screw model and
every dose-loop case in task 18. Add a separate injector instead, in `include/sim.h` and
`src/hal_sim.cpp`:

```c
/* `sim_set_clock_ms()` is NOT added here. Task 14 step 1 declares and defines it, for
   test_cart's rollover case; this task only consumes it. Its contract, restated so nobody
   re-derives it: it JUMPS the clock without pretending the models ran, settles the watchdog
   counter and the pump-on accumulator once, and runs NO edge emitter -- a test that wants
   edges advances a millisecond at a time like everything else. Two report cases below need a
   t= at and above 2^31, and sim_advance() is 1 ms per step by contract (task 3 step 5). */

/* Move the fake's break, so §12 item 0's per-report check has something to fail against.
   hal_heap_break() otherwise returns a fixed address comfortably inside the margin (task 3
   step 6), which exercises the happy path and nothing else. */
void sim_set_heap_break(uint32_t addr);
```

`sim_set_clock_ms()` is **task 14 step 1's**, already declared and defined; this task adds only
`sim_set_heap_break()`, which writes a file static
that `hal_heap_break()` now returns; `sim_reset()` restores it to task 3's `0x20001800u`.

14. [ ] **Run and see them fail, then confirm they pass.**

```
pio test -e native -f test_report
```

If `report_stamp()` was written correctly in step 4 both cases pass immediately; if either fails,
the bug is in `hal_boot_salt()` (task 3/4) — the salt must be `g_nv.boots` times a large odd
stride, so two warm boots fifteen seconds apart cannot produce the same `(controller, t)` inside
butler's 300 s dedup window. Fix there, not here. Commit:

```
git commit -am "t= is uint32 printed %lu, salted, and proved above 2^31"
```

15. [ ] **Write the remaining seven cases.**

```c
static void test_report_never_repeats_a_key(void) {
  fresh_sweep();
  report_set_ack(9, 5, "none");
  TEST_ASSERT_TRUE(build() > 0);
  char copy[PB_BODY_CAP]; memcpy(copy, g_buf, sizeof copy);
  char *keys[48]; int nk = 0;
  for (char *tok = strtok(copy, " \n"); tok && nk < 48; tok = strtok(NULL, " \n")) {
    char *eq = strchr(tok, '=');
    TEST_ASSERT_NOT_NULL(eq);            /* every token is k=v or the whole report 400s */
    *eq = '\0';
    for (int i = 0; i < nk; ++i) TEST_ASSERT_TRUE(strcmp(keys[i], tok) != 0);
    keys[nk++] = tok;
  }
  TEST_ASSERT_TRUE(nk >= 13);
}

/* ch205 is pulses_leak_count(), and NOTHING advances it except pulses_leak_poll() — which
   loop() calls once per pass (task 12 step 4) and no test harness calls for free. So the case
   has to drive the poller itself, exactly as loop() does, and it has to reach PB_DIAG_CLAMP:
   a 2 kHz storm for 60 s is ~120,000 pulses, an order of magnitude short. Storm the meter in
   ten-second bursts, polling as loop() would, until the count is past the clamp. */
static void test_a_saturated_diagnostic_counter_stays_inside_max_raw(void) {
  fresh_sweep();
  pulses_leak_poll(false);               /* arm the watch (the rearm window is long past) */
  sim_flow_storm(2000);
  for (int i = 0; i < 100 && pulses_leak_count() <= (uint32_t)PB_DIAG_CLAMP; ++i) {
    sim_advance(10000);
    pulses_leak_poll(false);             /* pump OFF: every one of these pulses is a leak */
  }
  sim_flow_storm(0);
  TEST_ASSERT_TRUE_MESSAGE(pulses_leak_count() > (uint32_t)PB_DIAG_CLAMP,
                           "the leak watch never reached the clamp: is pulses_leak_poll() "
                           "being called at all?");
  TEST_ASSERT_TRUE(build() > 0);
  char clamp[24];
  snprintf(clamp, sizeof clamp, "ch205=%lu", (unsigned long)PB_DIAG_CLAMP);
  TEST_ASSERT_TRUE(has_tok(clamp));
}

/* The same producer, at the other end of its range: one leaked pulse must reach the wire as
   BOTH ch205 and err=leak. §4.1 carries `leak` in its fixed enum and §1 says there is no
   latch, so this is the only surface the token has. */
static void test_ch205_counts_leak_pulses_and_err_leak_reaches_the_wire(void) {
  fresh_sweep();
  report_clear_ack();                    /* no ack, so err= falls through to the leak watch */
  pulses_leak_poll(false);
  sim_flow_storm(50);
  sim_advance(1000);
  sim_flow_storm(0);
  pulses_leak_poll(false);
  TEST_ASSERT_TRUE(pulses_leak_count() > 0u);
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_FALSE(has_tok("ch205=0"));
  TEST_ASSERT_TRUE_MESSAGE(has_tok("err=leak"), g_buf);
}

static void test_ch204_is_zero_before_d5_has_ever_changed_not_a_sentinel(void) {
  fresh_sweep();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("ch204=0"));  /* never -1, "unknown" or "never": _int_in 400s those */
}

static void test_report_err_token_never_contains_whitespace(void) {
  static const char *every_producer[] = {
    "none","float","pos","noflow","noise","cap","stop","wdt","dry","contra","boot","range",
    "cal","i2c","busy","cooldown","leak","adc","stuck","txcap","resetmid","heap","goto","recv"
  };
  for (unsigned i = 0; i < sizeof every_producer / sizeof every_producer[0]; ++i) {
    const char *t = every_producer[i];
    TEST_ASSERT_NULL(strpbrk(t, " \t\r\n"));    /* a space splits it into a non-k=v token */
    for (const char *p = t; *p; ++p) TEST_ASSERT_TRUE(*p >= 'a' && *p <= 'z');
  }
  fresh_sweep();
  safety_set_err("resetmid");
  report_clear_ack();
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(has_tok("err=resetmid"));
}

static void test_report_refuses_to_send_on_truncation_and_says_txcap(void) {
  fresh_sweep();
  const uint32_t before = report_txcap_drops();
  char small[40];
  report_stamp();
  TEST_ASSERT_EQUAL_UINT16(0, report_build(small, sizeof small));
  TEST_ASSERT_EQUAL_UINT32(before + 1, report_txcap_drops());
  TEST_ASSERT_EQUAL_STRING("txcap", safety_last_err());
  /* ...and the NEXT body that fits clears it. Nothing else in the program ever does, so one
     384-byte report would otherwise put err=txcap on every later report forever. */
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_EQUAL_STRING("none", safety_last_err());
}

/* Spec §12 item 0: "hal_begin() and EVERY REPORT check the break against the stack, because
   nothing else will." _sbrk is unchecked and __HeapLimit is referenced by nothing in the
   image, so this is the only bound that exists during the 48-hour run - and the run is
   exactly when the network stack, the largest allocator in the program, is active. */
static void test_a_break_inside_the_stack_margin_latches_err_heap(void) {
  fresh_sweep();
  TEST_ASSERT_TRUE(report_heap_ok());                  /* the fake starts well clear */
  sim_set_heap_break(hal_stack_limit() - (uint32_t)PB_STACK_MARGIN + 4u);
  TEST_ASSERT_FALSE(report_heap_ok());
  report_clear_ack();
  TEST_ASSERT_TRUE(build() > 0);                       /* a report saying heap beats no report */
  TEST_ASSERT_TRUE_MESSAGE(has_tok("err=heap"), g_buf);
}

static void test_report_fits_the_buffer_at_maximum_field_widths(void) {
  for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch) sim_set_channel(ch, 16383);  /* 14-bit maximum */
  sim_set_channel(PB_CANARY_CHANNEL, 1);
  TEST_ASSERT_TRUE(sensors_sweep());
  sim_set_clock_ms((uint32_t)(0xFFFFFFFFu - hal_boot_salt()));   /* jump, never 2^31 steps */
  report_set_ack(4294967295u, PB_DOSE_MAX_ML, "resetmid");
  TEST_ASSERT_TRUE(build() > 0);
  TEST_ASSERT_TRUE(strlen(g_buf) < PB_BODY_CAP);
  TEST_ASSERT_TRUE(strlen(g_buf) <= sizeof(PB_CONTROLLER) + 2 + PB_BODY_WORST_FIXED);
}

/* backend/fake_device.py's build_report() is the shape butler was written against:
   "c= t= chN=... float= pos= ack= flow_ml=", space-joined, one trailing newline. Ours adds
   ch200..ch209 and err=; strip those and the two must be byte-identical. */
static void test_report_matches_the_fake_device_shape(void) {
  fresh_sweep();
  sim_set_float(true);
  report_set_ack(17, 248, "none");
  TEST_ASSERT_TRUE(build() > 0);

  char spine[PB_BODY_CAP] = {0};
  char copy[PB_BODY_CAP]; memcpy(copy, g_buf, sizeof copy);
  for (char *tok = strtok(copy, " \n"); tok; tok = strtok(NULL, " \n")) {
    /* The diagnostic RANGE by name, never the prefix "ch2": `ch2=8002` is a WIRED channel and
       starts with the same three characters, so a prefix filter deletes a token the golden
       string keeps and this case can never pass. */
    if (strncmp(tok, "ch20", 4) == 0 || strncmp(tok, "err=", 4) == 0) continue;
    if (spine[0]) strncat(spine, " ", sizeof spine - strlen(spine) - 1);
    strncat(spine, tok, sizeof spine - strlen(spine) - 1);
  }
  char golden[PB_BODY_CAP];
  snprintf(golden, sizeof golden,
           "c=%s t=%lu ch0=8000 ch1=8001 ch2=8002 ch3=8003 ch4=8004 ch5=8005 "
           "float=1 pos=unknown ack=17 flow_ml=248",
           PB_CONTROLLER, (unsigned long)report_t_wire());
  TEST_ASSERT_EQUAL_STRING(golden, spine);
}
```

16. [ ] **Run them.**

```
pio test -e native -f test_report
```

Expected: `test_report_matches_the_fake_device_shape` fails first if the field ORDER differs from
spec §4.1's printed example — the order is `c t chN ch200..ch209 float pos ack flow_ml err`, and
`report_build()` as written above already emits exactly that. Fix any ordering drift in
`report_build()`, never in the golden string. Expected final: `25 Tests 0 Failures` — the
nineteen the earlier steps built plus this step's seven, less the count you actually see if
you split a case; what matters is `0 Failures`, and that
`test_ch205_counts_leak_pulses_and_err_leak_reaches_the_wire` and
`test_a_break_inside_the_stack_margin_latches_err_heap` are both among them, because each is
the only coverage its `err=` token has anywhere in the tree.

17. [ ] **Commit.**

```
git commit -am "The body's remaining traps: key uniqueness, the clamp, ch204, err, txcap, width

test_report_fits_the_buffer_at_maximum_field_widths executes spec 7's term-by-term
sum rather than restating it, and test_report_matches_the_fake_device_shape pins
our spine against backend/fake_device.py's build_report(), which is the shape
butler was written against."
```

18. [ ] **Add the three `status` lines.** In `src/cli.cpp`, inside `cli_print_status()`, after the
   `pos: FORCED unknown (PB_REPORT_POS_UNKNOWN=1)` line, add three lines and one include (via
   `cli_printf_u32`, no `%d`, no `%f`):

```c
  cli_printf_u32("report: last_body=%lu bytes\n", (uint32_t)report_last_len());
  cli_printf_u32("report: cap=%lu bytes\n", (uint32_t)PB_BODY_CAP);
  cli_printf_u32("report: DROPPED on truncation (err=txcap) x%lu\n", report_txcap_drops());
```

with `#include "report.h"` at the top. §4.2 requires the drop to be loud: a truncated body is a
silent reporting blackout otherwise, not a 400.

19. [ ] **Run both gates and commit.**

```
pio test -e native
pio run -e uno_r4_wifi
git commit -am "status prints the body length and the txcap drop count

A truncated body is a DROPPED report, not a 400 — the console is the only place
that failure is visible, so it says so loudly. Spec 4.2."
```

---

### Task 23: response_parse() and the replay guard

**Drop 3.**

**Files:**

- Modify: `include/report.h` (append the four declarations below, after `report_may_build()`),
  `src/report.cpp` (append `response_parse()` and its helpers at the end of the file),
  `src/cli.cpp` (one `#include "noinit.h"` if it is not already there, plus one line inside
  `cli_print_status()`),
  `test/test_report/test_report.cpp` (append ten cases, their `RUN_TEST` lines and
  `#include "noinit.h"`).
- Test: `test/test_report/test_report.cpp`.

**Interfaces:**

*Consumes.* From `include/noinit.h` (task 4): `pb_noinit_t` with its `uint32_t cmd_high_water;`
field, `extern pb_noinit_t g_nv;`, `void noinit_commit(void);`, and
`void sim_reset(bool warm);` / `void sim_noinit_clobber(void);` from `include/sim.h` (task 3).
From `include/config.h` (task 2): `PB_OUTLETS` 5.

*Produces.* `include/report.h` gains, verbatim from spec §4.5:

```c
typedef enum { CMD_NONE, CMD_WATER, CMD_STOP } cmd_kind_t;
typedef struct { uint32_t id; cmd_kind_t kind; uint8_t outlet; uint16_t ml; uint16_t cap_s; } cmd_t;
typedef struct { uint16_t next_s; cmd_t cmd; } response_t;
bool response_parse(const char *body, uint16_t len, response_t *out);   /* 200 bodies ONLY */
```

Two contract points the skeleton leaves to the implementer, fixed here so task 24 can rely on them:

- `*out` is **always** fully written, whatever the return value. The return value means "a command
  was accepted", i.e. `out->cmd.kind != CMD_NONE`.
- `out->next_s` is **0 when the interval must not change** — absent, malformed, or outside
  [5, 3600]. Task 24 assigns only when it is non-zero. A `next_s` of 0 is never a legal interval,
  so this needs no extra flag.

The body shape is butler's, exactly (`butler.py:1640-1647`): `next=<n>\n`, then optionally
`cmd=<id> water=<o> ml=<m> cap_s=<c>\n` or `cmd=<id> stop=1\n`.

An outlet outside 1..`PB_OUTLETS` — **including `water=0`, which butler accepts**
(`_int_in(value,"water",0,MAX_CHANNEL+1)` and the `outlet is None` guard does not catch 0) — **is
accepted here** and is refused and acked with `err=range` by task 26's range check, so the backend
learns the real reason instead of receiving whichever step happened to fail first.

---

1. [ ] **Write the two happy-path cases and the unknown-key case.** Append to
   `test/test_report/test_report.cpp` (and add the `RUN_TEST` lines):

```c
static void test_response_parses_next_only(void) {
  response_t r;
  const char *b = "next=60\n";
  TEST_ASSERT_FALSE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL_UINT16(60, r.next_s);
  TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
}

static void test_response_parses_a_water_command(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 water=3 ml=250 cap_s=30\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL_UINT16(60, r.next_s);
  TEST_ASSERT_EQUAL(CMD_WATER, r.cmd.kind);
  TEST_ASSERT_EQUAL_UINT32(17, r.cmd.id);
  TEST_ASSERT_EQUAL_UINT8(3, r.cmd.outlet);
  TEST_ASSERT_EQUAL_UINT16(250, r.cmd.ml);
  TEST_ASSERT_EQUAL_UINT16(30, r.cmd.cap_s);
}

static void test_response_parses_a_stop_command(void) {
  response_t r;
  const char *b = "next=60\ncmd=18 stop=1\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL(CMD_STOP, r.cmd.kind);
  TEST_ASSERT_EQUAL_UINT32(18, r.cmd.id);
}

static void test_response_ignores_unknown_keys(void) {
  response_t r;
  const char *b = "next=60 note=hello\ncmd=19 water=2 ml=100 cap_s=10 spare=7\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL(CMD_WATER, r.cmd.kind);
  TEST_ASSERT_EQUAL_UINT16(100, r.cmd.ml);
}
```

2. [ ] **Run and see them fail.**

```
pio test -e native -f test_report
```

Expected: `test/test_report/test_report.cpp:NN:NN: error: 'response_t' was not declared in this
scope`.

3. [ ] **Add the declarations to `include/report.h`** — the four lines printed under *Produces*,
   with this comment above `response_parse`:

```c
/* Parses a 200 body ONLY. Butler's 400 body echoes the board's own tokens
   (f"{key}= out of range: {value}"), so a 4xx body parsed here could water a plant.
   *out is always fully written; the return value means "a command was accepted".
   out->next_s == 0 means "keep the previous interval". Spec §4.5. */
```

4. [ ] **Implement `response_parse()`** at the end of `src/report.cpp`:

```c
/* ---- response parsing — where a fault becomes water (spec §4.5) ---- */

/* One unsigned field out of a k=v token. false == absent, non-numeric, or overlong.
   ASCII digits only, like butler's own _int_in (butler.py:192-201). */
static bool field_u32(const char *line, uint16_t len, const char *key, uint32_t *out) {
  const size_t kl = strlen(key);
  for (uint16_t i = 0; i < len; ++i) {
    if (i && line[i - 1] != ' ') continue;
    if (len - i < kl || memcmp(line + i, key, kl) != 0) continue;
    uint16_t j = (uint16_t)(i + kl);
    if (j >= len || line[j] < '0' || line[j] > '9') return false;
    uint32_t v = 0;
    for (; j < len && line[j] >= '0' && line[j] <= '9'; ++j) {
      if (v > 429496729u) return false;
      v = v * 10u + (uint32_t)(line[j] - '0');
    }
    if (j < len && line[j] != ' ') return false;     /* a trailing non-digit is not a number */
    *out = v;
    return true;
  }
  return false;
}

bool response_parse(const char *body, uint16_t len, response_t *out) {
  out->next_s = 0;                        /* 0 == keep the previous interval */
  out->cmd.id = 0; out->cmd.kind = CMD_NONE;
  out->cmd.outlet = 0; out->cmd.ml = 0; out->cmd.cap_s = 0;
  if (!body || len == 0) return false;

  /* Every line must be newline-TERMINATED inside len. A body truncated mid-token must never
     water: a half-read reply is not a command (§4.5). */
  uint16_t pos = 0;
  while (pos < len) {
    const char *nl = (const char *)memchr(body + pos, '\n', (size_t)(len - pos));
    if (!nl) break;                       /* trailing partial line: ignored, never parsed */
    const char *line = body + pos;
    const uint16_t llen = (uint16_t)(nl - line);
    pos = (uint16_t)(nl - body + 1);

    uint32_t v;
    if (field_u32(line, llen, "next=", &v) && v >= 5u && v <= 3600u)
      out->next_s = (uint16_t)v;          /* outside [5,3600]: keep the previous interval */

    if (!field_u32(line, llen, "cmd=", &v)) continue;
    if (v == 0) continue;                 /* ack must be >= 1 or the whole report 400s */
    /* Replay guard (§4.3): a response body left over from an earlier round trip — exactly what a
       poisoned AT session produces — would otherwise run cmd=17 a SECOND time; the second ack
       lands on a row no longer state='sent', so the UPDATE is a silent no-op, the cooldown and
       daily cap never see it, and the plant gets double the water with no alert. */
    if (v <= g_nv.cmd_high_water) continue;
    const uint32_t id = v;

    uint32_t stop = 0;
    if (field_u32(line, llen, "stop=", &stop) && stop != 0) {
      out->cmd.id = id; out->cmd.kind = CMD_STOP;
    } else {
      uint32_t outlet, ml, cap_s;
      if (!field_u32(line, llen, "water=", &outlet)) continue;
      if (!field_u32(line, llen, "ml=", &ml))       continue;   /* no ml= is no command */
      if (!field_u32(line, llen, "cap_s=", &cap_s)) continue;   /* an absent cap is unbounded */
      if (ml == 0) continue;
      if (outlet > 255u || ml > 65535u || cap_s > 65535u) continue;
      /* An outlet outside 1..PB_OUTLETS — water=0 included — IS accepted here and refused with
         err=range by exec_pending(), above cart_goto(), so the backend learns the real reason. */
      out->cmd.id = id; out->cmd.kind = CMD_WATER;
      out->cmd.outlet = (uint8_t)outlet;
      out->cmd.ml = (uint16_t)ml;
      out->cmd.cap_s = (uint16_t)cap_s;
    }
    g_nv.cmd_high_water = id;             /* bumped the moment a command is ACCEPTED */
    noinit_commit();                      /* .noinit, so a warm reset does not reopen the window */
    return true;
  }
  return false;
}
```

Add `#include "noinit.h"` to `src/report.cpp`.

Note `field_u32("ml=")` would also match the tail of `cap_s=`-style keys if the leading-space test
were dropped — it is not: the `i && line[i-1] != ' '` guard makes every match start-of-line or
after a space, which is what stops `ml=` matching inside `flow_ml=` on any future body shape.

5. [ ] **Run and see the four cases pass, then commit.**

```
pio test -e native -f test_report
```

Expected `26 Tests 0 Failures`.

```
git add include/report.h src/report.cpp test/test_report/test_report.cpp
git commit -m "response_parse(): butler's exact 200 body, and the replay guard

The high-water mark lives in .noinit so a warm reset does not reopen the replay
window, and is bumped the moment a command is accepted. Its monotonicity rests on
a schema comment: commands.id is INTEGER PRIMARY KEY with no AUTOINCREMENT, so a
rebuilt database restarts at 1 and the board then refuses every command until a
COLD boot. Spec 4.3, 16.5.9."
```

6. [ ] **Write the six rejection cases.**

```c
static void test_response_rejects_command_id_zero(void) {
  response_t r;
  const char *b = "next=60\ncmd=0 water=3 ml=250 cap_s=30\n";
  TEST_ASSERT_FALSE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
  TEST_ASSERT_EQUAL_UINT32(0, g_nv.cmd_high_water);   /* and it never moves the mark */
}

static void test_response_rejects_a_repeated_or_lower_command_id(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 water=3 ml=250 cap_s=30\n";
  TEST_ASSERT_TRUE(response_parse(b, (uint16_t)strlen(b), &r));
  TEST_ASSERT_EQUAL_UINT32(17, g_nv.cmd_high_water);
  TEST_ASSERT_FALSE(response_parse(b, (uint16_t)strlen(b), &r));   /* the same body again */
  const char *lower = "next=60\ncmd=9 water=3 ml=250 cap_s=30\n";
  TEST_ASSERT_FALSE(response_parse(lower, (uint16_t)strlen(lower), &r));
  const char *higher = "next=60\ncmd=18 water=3 ml=250 cap_s=30\n";
  TEST_ASSERT_TRUE(response_parse(higher, (uint16_t)strlen(higher), &r));
}

static void test_response_rejects_water_without_ml_or_without_cap_s(void) {
  response_t r;
  const char *no_ml = "next=60\ncmd=17 water=3 cap_s=30\n";
  TEST_ASSERT_FALSE(response_parse(no_ml, (uint16_t)strlen(no_ml), &r));
  const char *no_cap = "next=60\ncmd=17 water=3 ml=250\n";     /* an absent cap is unbounded */
  TEST_ASSERT_FALSE(response_parse(no_cap, (uint16_t)strlen(no_cap), &r));
  TEST_ASSERT_EQUAL_UINT32(0, g_nv.cmd_high_water);
}

static void test_response_rejects_ml_zero(void) {
  response_t r;
  const char *b = "next=60\ncmd=17 water=3 ml=0 cap_s=30\n";
  TEST_ASSERT_FALSE(response_parse(b, (uint16_t)strlen(b), &r));
}

static void test_response_truncated_body_yields_no_command(void) {
  response_t r;
  const char *full = "next=60\ncmd=17 water=3 ml=250 cap_s=30\n";
  for (uint16_t cut = 9; cut < strlen(full); ++cut) {        /* every mid-token truncation */
    TEST_ASSERT_FALSE(response_parse(full, cut, &r));
    TEST_ASSERT_EQUAL(CMD_NONE, r.cmd.kind);
  }
  TEST_ASSERT_EQUAL_UINT32(0, g_nv.cmd_high_water);
}

static void test_response_next_out_of_range_keeps_the_previous_interval(void) {
  response_t r;
  const char *lo = "next=4\n";
  TEST_ASSERT_FALSE(response_parse(lo, (uint16_t)strlen(lo), &r));
  TEST_ASSERT_EQUAL_UINT16(0, r.next_s);                    /* 0 == keep what we had */
  const char *hi = "next=3601\n";
  TEST_ASSERT_FALSE(response_parse(hi, (uint16_t)strlen(hi), &r));
  TEST_ASSERT_EQUAL_UINT16(0, r.next_s);
  const char *edge_lo = "next=5\n";
  TEST_ASSERT_FALSE(response_parse(edge_lo, (uint16_t)strlen(edge_lo), &r));
  TEST_ASSERT_EQUAL_UINT16(5, r.next_s);
  const char *edge_hi = "next=3600\n";
  TEST_ASSERT_FALSE(response_parse(edge_hi, (uint16_t)strlen(edge_hi), &r));
  TEST_ASSERT_EQUAL_UINT16(3600, r.next_s);
}
```

These read `g_nv.cmd_high_water`, so the test file needs `#include "noinit.h"`, and
`pb_test_setup()` (task 3) must leave it at 0 — it calls `sim_reset(false)`, a cold boot, which
zeroes the `.noinit` struct. If `test_response_rejects_command_id_zero` sees a non-zero mark, the
cold-boot path of task 4 is broken; fix it there.

7. [ ] **Run them.**

```
pio test -e native -f test_report
```

Expected `32 Tests 0 Failures`. If `test_response_truncated_body_yields_no_command` fails, the
newline-termination rule of step 4 was dropped: a body cut mid-token must never water.

8. [ ] **Commit.**

```
git commit -am "Every response rejection is a named case

cmd=0 (ack must be >= 1 or the whole report 400s), a replayed or lower id, water=
without ml= or cap_s=, ml=0, a body truncated mid-token, and next= outside
[5,3600]. water=0 is deliberately NOT rejected here: butler accepts it, and
exec_pending() acks it err=range so the backend learns the real reason."
```

9. [ ] **Add `cmd_high_water=` to `status`.** In `src/cli.cpp`, inside `cli_print_status()`,
   after the `report:` lines:

```c
  /* A rebuilt or restored backend database restarts commands.id at 1, and the board then
     refuses EVERY command as a replay until a COLD boot (power cycle, not RESET). Without this
     line that is silent and unexplainable. Spec §4.3, §16.5.9. */
  cli_printf_u32("cmd_high_water=%lu (recovery: cold boot)\n", g_nv.cmd_high_water);
```

with `#include "noinit.h"` if `cli.cpp` does not have it already.

10. [ ] **Run both gates and commit.**

```
pio test -e native
pio run -e uno_r4_wifi
git commit -am "status prints cmd_high_water=, and how to recover from it"
```

---

### Task 24: netfsm.cpp — the report state machine, the HTTP framing and the two-AT budget

**Drop 3.**

**Files:**

- Create: `include/netfsm.h`, `src/netfsm.cpp`.
- Modify: `include/sim.h` and `src/hal_sim.cpp` (one injector, `sim_on_pump_on()` — see step 13),
  `src/cli.cpp` (`#include "netfsm.h"` and `#include "link.h"`, and **seven** lines inside `cli_print_status()`, step 17),
  `test/support/harness.h` (`pb_net_passes()`, step 1),
  `test/test_net/test_netfsm.cpp` (append ten cases and their `RUN_TEST` lines).
- Not modified: `src/ui.cpp`. Task 10 step 4's `ui_render_lcd()` already renders a non-200
  `http_status` as `HTTP <n>` on row 1 and `ui_state_t` already carries the field; what this
  task adds is the field's *source*, `net_last_status()`, and the place that fills `ui_state_t`
  is `main.cpp`, which task 26 wires. **`include/ui.h` IS included by `src/netfsm.cpp`, for
  `ui_modem_ran()` — see step 4.** §9's grep over `netfsm.cpp` is
  `safety\.h|dose_run|hal_pump_write`, and `ui.h` is none of those.
- Test: `test/test_net/test_netfsm.cpp`.

**Interfaces:**

*Consumes.* All ten primitives of `include/link.h` (task 21) plus, from `include/sim.h`,
`link_fake_reset/set_state/queue_response/fail_open/timeout_next/drop_link/pass_begin/at_count/
saw_available/saw_connected/reset_count/sent`. From `include/report.h` (tasks 22, 23):
`void report_stamp(void);`, `uint32_t report_t_ms(void);`, `uint16_t report_build(char *buf,
uint16_t cap);`, `bool report_may_build(void);`, `bool report_heap_ok(void);`, `void report_set_ack(uint32_t,
uint16_t, const char *);`, `void report_clear_ack(void);`, `bool response_parse(const char *,
uint16_t, response_t *);`, `cmd_t`, `response_t`. From `lib/Manifold/include/cart.h` (task 14):
`bool cart_busy(void);`. From `include/sensors.h` (task 7): `bool sensors_sweep(void);`.
From `include/ui.h` (task 10): `void ui_modem_ran(void);`.
From `include/config.h` (task 2): `PB_NET_STEP_MS` 1200, `PB_NET_DEADLINE_MS` 5000,
`PB_NET_BACKOFF_MS` `{2000,4000,8000,16000,30000}`, `PB_TX_CAP` 768, `PB_RX_CAP` 256,
`PB_HDR_FIXED` 128, `PB_BODY_CAP` 384. From `include/secrets.h` (task 1): `HOST_NAME`,
`BUTLER_TOKEN`, `HTTP_PORT`.

*Produces.* `include/netfsm.h`:

```c
typedef enum { NET_DOWN, NET_JOIN_ISSUE, NET_JOIN_WAIT, NET_IDLE,
               NET_SOCK_CLOSE, NET_CONNECT, NET_SEND, NET_RECV, NET_CLOSE } net_state_t;
void        net_begin(void);
void        net_poll(bool dosing);          /* the flag is passed IN — see below */
net_state_t net_state(void);
uint16_t    net_last_status(void);
uint16_t    net_next_s(void);
uint32_t    net_reports_ok(void);
uint32_t    net_reports_failed(void);
bool        net_modem_ran_this_pass(void);  /* printed by `status` as modem_ran= (step 17) */
bool        net_take_command(cmd_t *out);
void        net_disable(const char *why);   /* addition — see below */
const char *net_disabled(void);             /* addition */
```

**The last two are additions to the skeleton's list, and they close a defect a review found in the
previous plan.** Spec §3 says `setup()` asserts `hal_wdt_granted() >= 2 * PB_NET_STEP_MS +
PB_NET_SLACK_MS` and, if it fails, *"disables the network and says why in `status`"*. Task 12 sets
that condition; before this task nothing consumed it — `net_poll()` had no disabled branch and
`status` never reported it, so a failed boot assertion would have gone on reporting anyway. Record
it in the commit message.

**Three ownership decisions this task makes, because nothing else can.**

1. **`sensors_sweep()` gets its owner and its cadence here, and nowhere else in this plan calls it.**
   It is called once per report cycle, in the `NET_IDLE` pass where the interval has elapsed,
   immediately before `report_stamp()`. That pass issues **zero** AT commands, which is how spec
   §3's rule — *"`ui_poll()` and `sensors_sweep()` are both skipped in any pass where a modem
   command ran"* — is satisfied by construction rather than by a flag. `main.cpp`'s `loop()` does
   **not** call it and there is no `sensors_poll()` in this plan (task 7's Interfaces block and
   task 12's deviation note both say so). A previous draft produced `sensors_sweep()` and read its
   results and never scheduled it, so every report would have carried no wired channels at all.
2. **`report_clear_ack()` is called on the 200 path and nowhere else.** A previous draft declared
   it and never called it, so every report would have repeated the same ack forever.
3. **`ui_modem_ran()` is called from here, directly, in every pass that issues a modem command.**
   Task 10 produced it with that contract written into `ui.h` and rejected the alternative
   (`main.cpp` reading `net_modem_ran_this_pass()` and forwarding it) on the grounds that it puts
   a rule which exists to bound ONE pass into a different translation unit from the pass that
   broke it. If nothing here calls it, `ui.cpp`'s flag is always false and §3's and §5's rule —
   *"neither screen is painted … [when] a modem command ran this pass"* — is not implemented at
   all, which is up to 102 s of wedged-bus LCD painting stacked on a 2.4 s modem pass.

**`netfsm.cpp` may not include `safety.h`** — §9's grep is `0 hits for safety\.h|dose_run|
hal_pump_write` in this file, and it is what keeps a `dose_run()` call one edit further away from a
state whose socket is open. §3's runtime guard still needs the dosing flag, so **`net_poll()` takes
it as a parameter**: `void net_poll(bool dosing)`, filled by `loop()` from `safety_dosing()` (task
12 step 4's placeholder comment and task 26 step 13(b) both write it that way, and the Assembly
notes record it as a deliberate departure). There is no `extern bool safety_dosing(void);`
forward declaration in this file — a forward declaration is an include by another name, and the
parameter costs nothing. **Every call site in this task's own suite passes `false`.**
`include/ui.h` *is* included, for `ui_modem_ran()`; it is not one of the three tokens the grep
names.

---

1. [ ] **Write the framing case.** Append to `test/test_net/test_netfsm.cpp` (and its `RUN_TEST`
   line), adding `#include "netfsm.h"`, `#include "report.h"`, `#include "sensors.h"`,
   `#include "secrets.h"`, `#include <stdio.h>` and `#include <string.h>` at the top
   (`snprintf` builds the Host and X-Token needles, because those two are `const char[]`
   objects and not macros):

**First, write the shared pass driver into `test/support/harness.h`**, not into this file: tasks 25 and 26 call it from `test_net`, `test_cart` and `test_contra`, and three separate Unity binaries cannot share a file static. Guard it `#ifdef PB_SIM` — it names the fake link, which the device test env filters out — and task 28 folds that guard into the host/device arms it creates.

```c
/* test/support/harness.h, guarded #ifdef PB_SIM. Drive n whole network passes, advancing the
   fake clock ms between them. THE ONE SPELLING: task 24's own cases, task 25's retry cases
   and task 26's ack-cycle cases all use it, so a change to what "a pass" means lands once. */
#ifdef PB_SIM
#  include "netfsm.h"
static inline void pb_net_passes(uint16_t n, uint32_t ms) {
  for (uint16_t i = 0; i < n; ++i) {
    link_fake_pass_begin();
    net_poll(false);            /* not dosing: dose_run() blocks, so a pass cannot overlap one */
    if (ms) pb_advance(ms);
  }
}
#endif
```

`harness.h` already includes `"sim.h"` (task 3 step 7); this adds `"netfsm.h"` beside it.

```c
/* test/test_net/test_netfsm.cpp. `pump_passes(n)` is pb_net_passes(n, 0) and nothing else —
   one spelling, so the cases below and task 25's cannot drift apart. */
static void pump_passes(uint8_t n) { pb_net_passes(n, 0u); }
static const char *k200 =
  "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: 8\r\n\r\nnext=60\n";

static void test_http_post_carries_host_token_and_content_length(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  pump_passes(8);
  uint16_t n = 0;
  const char *tx = (const char *)link_fake_sent(&n);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_TRUE(strstr(tx, "POST /report HTTP/1.1\r\n") == tx);
  /* HOST_NAME and BUTLER_TOKEN are `const char[]` OBJECTS in secrets.h, not string-literal
     macros the way PB_CONTROLLER is, so `"Host: " HOST_NAME` would not compile. Build the
     needles instead; the c= assertion below juxtaposes because PB_CONTROLLER really is one. */
  char want[128];
  snprintf(want, sizeof want, "\r\nHost: %s\r\n", HOST_NAME);
  TEST_ASSERT_NOT_NULL(strstr(tx, want));
  snprintf(want, sizeof want, "\r\nX-Token: %s\r\n", BUTLER_TOKEN);
  TEST_ASSERT_NOT_NULL(strstr(tx, want));
  TEST_ASSERT_NOT_NULL(strstr(tx, "\r\nContent-Type: text/plain\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(tx, "\r\nConnection: close\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(tx, "\r\n\r\nc=" PB_CONTROLLER " t="));
}

static void test_report_content_length_matches_the_bytes_actually_written(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  pump_passes(8);
  uint16_t n = 0;
  const char *tx = (const char *)link_fake_sent(&n);
  const char *hdr = strstr(tx, "Content-Length: ");
  TEST_ASSERT_NOT_NULL(hdr);
  unsigned long claimed = strtoul(hdr + strlen("Content-Length: "), NULL, 10);
  const char *body = strstr(tx, "\r\n\r\n") + 4;
  TEST_ASSERT_EQUAL_UINT32((uint32_t)claimed, (uint32_t)(n - (uint16_t)(body - tx)));
}
```

(add `#include <stdlib.h>` for `strtoul`.)

2. [ ] **Run and see it fail.**

```
pio test -e native -f test_net
```

Expected: `test/test_net/test_netfsm.cpp:NN:10: fatal error: netfsm.h: No such file or directory`.

3. [ ] **Create `include/netfsm.h`** with the block printed under *Produces*, wrapped in
   `#pragma once`, `#include <stdbool.h> <stdint.h>` and `#include "report.h"` (for `cmd_t`), and
   these comments:

```c
/* ONE bounded link/socket step per net_poll(), AT MOST 2 AT commands, each with its own
   deadline. Worst pass is CONNECT = _BEGINCLIENT + _CLIENTCONNECT = 2 * PB_NET_STEP_MS =
   2400 ms against a granted 5592 ms — and that holds ONLY because sock_close() ran in a PRIOR
   pass and left _sock == -1, and because every error exit routes through NET_SOCK_CLOSE
   instead of closing inline. Spec §3's per-pass AT table. */
```

4. [ ] **Create `src/netfsm.cpp`.** This is the whole file; the retry decision inside `finish()`
   is a stub that task 25 replaces.

```c
/* src/netfsm.cpp — the report state machine and the HTTP framing, above seam 2.
   §9 greps this file for ZERO hits of the safety header, the dosing entry point and the pump
   write. That is the direction that matters: one include of the safety header added here
   during a later change puts a call that asserts D6 one edit away from a state whose socket
   is open, with the build and make check both green.
   NO signed integer conversion in any format string (§9): every numeric site is %lu with an
   explicit cast. */
#include "netfsm.h"
#include "link.h"
#include "report.h"
#include "sensors.h"
#include "cart.h"
#include "config.h"
#include "hal.h"
#include "ui.h"          /* ui_modem_ran() ONLY. §9's grep over this file names the safety
                            header, the dosing entry point and the pump write, and ui.h is
                            none of those. */
#include "secrets.h"
#include <stdio.h>
#include <string.h>

/* THE DOSING FLAG IS A PARAMETER, not a forward declaration. §9's grep keeps the safety header
   out of this file, and an `extern bool safety_dosing(void);` here would be that include by
   another name; loop() supplies the flag instead (task 12 step 4, task 26 step 13b). Spec §3,
   "include hygiene, in both directions". */

static_assert(sizeof(HOST_NAME) + sizeof(BUTLER_TOKEN) + PB_HDR_FIXED + PB_BODY_CAP <= PB_TX_CAP,
              "HOST_NAME + BUTLER_TOKEN + headers + body do not fit PB_TX_CAP: snprintf would "
              "truncate silently while Content-Length still claimed the full length (spec §4.2)");
static_assert(sizeof(PB_CONTROLLER) + 2 + PB_BODY_WORST_FIXED <= PB_BODY_CAP, "body cap (§7)");
static_assert(sizeof(PB_CONTROLLER) > 1, "an empty c= is a permanent 400 (butler.py:252-253)");

static const uint32_t k_backoff[] = PB_NET_BACKOFF_MS;

static net_state_t g_state;
static const char *g_disabled;
static uint16_t g_status;
static uint16_t g_next_s = 60;
static uint32_t g_ok, g_failed;
static bool     g_modem_ran;
static uint8_t  g_backoff_i;
static uint32_t g_wait_until, g_deadline, g_last_report_ms;
static bool     g_first_report_due;

static char     g_body[PB_BODY_CAP];
static uint16_t g_body_len;                 /* != 0 == a report is pending on the wire */
static char     g_tx[PB_TX_CAP];
static uint16_t g_tx_len;
static char     g_rx[PB_RX_CAP];
static uint16_t g_rx_len;

static cmd_t    g_cmd;
static bool     g_have_cmd;

net_state_t net_state(void)           { return g_state; }
uint16_t    net_last_status(void)     { return g_status; }
uint16_t    net_next_s(void)          { return g_next_s; }
uint32_t    net_reports_ok(void)      { return g_ok; }
uint32_t    net_reports_failed(void)  { return g_failed; }
bool        net_modem_ran_this_pass(void) { return g_modem_ran; }
const char *net_disabled(void)        { return g_disabled; }
void        net_disable(const char *why) { g_disabled = why; }

/* EVERY pass that issues an AT command goes through this, never through a bare
   `g_modem_ran = true;`. ui.cpp's own flag has to be raised in the SAME pass, or spec §3's and
   §5's rule that neither screen is painted after a modem pass is not implemented at all — and
   the cost of not implementing it is up to 102 s of wedged-bus LCD painting stacked on top of
   a 2.4 s modem pass, inside a 5592 ms grant. net_modem_ran_this_pass() stays as the readable
   fact for `status` and for tests; ui_modem_ran() is the consumer that matters. */
static void modem_ran_(void) { g_modem_ran = true; ui_modem_ran(); }

bool net_take_command(cmd_t *out) {
  if (!g_have_cmd) return false;
  *out = g_cmd;
  g_have_cmd = false;                       /* surfaced ONCE per round trip */
  return true;
}

void net_begin(void) {
  link_begin(PB_NET_STEP_MS);
  g_state = NET_DOWN; g_status = 0; g_next_s = 60; g_ok = 0; g_failed = 0;
  g_backoff_i = 0; g_wait_until = 0; g_deadline = 0;
  g_last_report_ms = hal_millis(); g_first_report_due = true;
  g_body_len = 0; g_tx_len = 0; g_rx_len = 0; g_have_cmd = false;
}

/* Every error exit routes THROUGH NET_SOCK_CLOSE via this one function, and it does NOT call
   sock_close() itself: a failed CONNECT that closed inline would be _BEGINCLIENT +
   _CLIENTCONNECT + _CLIENTCLOSE = 3 ATs = 3600 ms, and 3600 + PB_NET_SLACK_MS = 5600 > 5592.
   Load-bearing arithmetic, not tidiness (spec §3 change 2). */
static void finish(uint16_t status, bool ok) {
  if (status) g_status = status;
  if (ok) ++g_ok; else ++g_failed;
  g_body_len = 0;                 /* task 25 keeps it when a retry is armed */
  g_state = NET_SOCK_CLOSE;
}

static void link_down(void) {
  g_wait_until = hal_millis() + k_backoff[g_backoff_i];
  if (g_backoff_i + 1 < sizeof k_backoff / sizeof k_backoff[0]) ++g_backoff_i;
  g_state = NET_DOWN;
}

static bool assemble(void) {
  int w = snprintf(g_tx, sizeof g_tx,
                   "POST /report HTTP/1.1\r\nHost: %s\r\nX-Token: %s\r\n"
                   "Content-Type: text/plain\r\nContent-Length: %lu\r\n"
                   "Connection: close\r\n\r\n",
                   HOST_NAME, BUTLER_TOKEN, (unsigned long)g_body_len);
  if (w < 0 || (size_t)w + g_body_len >= sizeof g_tx) return false;
  memcpy(g_tx + w, g_body, g_body_len);
  g_tx_len = (uint16_t)((uint16_t)w + g_body_len);
  return true;
}

static uint16_t rx_status(void) {
  if (g_rx_len < 12 || memcmp(g_rx, "HTTP/1.", 7) != 0) return 0;
  for (int i = 9; i < 12; ++i) if (g_rx[i] < '0' || g_rx[i] > '9') return 0;
  return (uint16_t)((g_rx[9] - '0') * 100 + (g_rx[10] - '0') * 10 + (g_rx[11] - '0'));
}

static bool rx_complete(const char **body, uint16_t *blen) {
  const char *end = NULL;
  for (uint16_t i = 0; i + 4 <= g_rx_len; ++i)
    if (memcmp(g_rx + i, "\r\n\r\n", 4) == 0) { end = g_rx + i + 4; break; }
  if (!end) return false;
  uint32_t cl = 0;
  for (uint16_t i = 0; i + 15 <= (uint16_t)(end - g_rx); ++i) {
    if (strncasecmp(g_rx + i, "content-length:", 15) != 0) continue;
    uint16_t j = (uint16_t)(i + 15);
    while (j < g_rx_len && g_rx[j] == ' ') ++j;
    while (j < g_rx_len && g_rx[j] >= '0' && g_rx[j] <= '9') cl = cl * 10u + (uint32_t)(g_rx[j++] - '0');
    break;
  }
  const uint16_t have = (uint16_t)(g_rx_len - (uint16_t)(end - g_rx));
  if (have < cl) return false;
  *body = end;
  *blen = (uint16_t)cl;
  return true;
}

void net_poll(bool dosing) {
  g_modem_ran = false;
  if (g_disabled) return;              /* the boot assertion's consumer (§3) */
  if (dosing || cart_busy()) return;   /* §3's runtime guard. The flag arrives as a PARAMETER
                                          because §9 keeps the safety header out of this
                                          file. */

  switch (g_state) {
    case NET_DOWN:
      if (hal_millis() < g_wait_until) return;
      g_state = NET_JOIN_ISSUE;
      return;

    case NET_JOIN_ISSUE:
      modem_ran_();
      link_join();                              /* 2 ATs (WiFi.cpp:43-67) */
      g_deadline = hal_millis() + PB_NET_DEADLINE_MS;
      g_state = NET_JOIN_WAIT;
      return;

    case NET_JOIN_WAIT: {
      modem_ran_();
      link_state_t s = link_state();            /* 1 AT */
      if (s == LINK_UP) { g_backoff_i = 0; g_state = NET_IDLE; return; }
      if (hal_millis() >= g_deadline) link_down();
      return;
    }

    case NET_IDLE: {
      const uint32_t due = (uint32_t)g_next_s * 1000u;
      if (!g_first_report_due && hal_millis() - g_last_report_ms < due) return;
      if (!report_may_build()) return;   /* §4.3: the report WAITS while the ack reads recv */
      /* §12 item 0's per-report break check. report_heap_ok() latches err=heap; disabling the
         network is this file's half, because the flag lives here. A board that stops reporting
         is the right answer once the break is inside the stack margin: the network stack is
         the largest allocator in the program, and continuing is how the corruption reaches a
         water command. */
      if (!report_heap_ok()) { net_disable("heap"); return; }
      /* This pass issues no AT command at all, which is what makes the sweep legal here:
         §3 skips sensors_sweep() in any pass where a modem command ran. It is the sweep's
         ONLY caller in the whole program and its cadence is one per report cycle; loop()
         does not call it and there is no sensors_poll() anywhere. */
      (void)sensors_sweep();
      report_stamp();
      g_body_len = report_build(g_body, sizeof g_body);
      g_last_report_ms = hal_millis();
      g_first_report_due = false;
      if (g_body_len == 0) { ++g_failed; return; }   /* err=txcap: DROPPED, never sent (§4.2) */
      g_state = NET_SOCK_CLOSE;
      return;
    }

    case NET_SOCK_CLOSE:
      modem_ran_();
      memset(g_rx, 0, sizeof g_rx);
      g_rx_len = 0;                     /* no byte of an earlier round trip survives into this one */
      sock_close();                     /* 1 AT, or 0 when _sock == -1 */
      g_state = g_body_len ? NET_CONNECT : NET_IDLE;
      return;

    case NET_CONNECT:
      modem_ran_();
      if (!sock_open()) { finish(0, false); return; }   /* 2 ATs; a failed open leaves _sock >= 0 */
      g_state = NET_SEND;
      return;

    case NET_SEND:
      modem_ran_();
      if (!assemble()) { finish(0, false); return; }
      if (sock_write((const uint8_t *)g_tx, g_tx_len) != (int)g_tx_len) { finish(0, false); return; }
      g_deadline = hal_millis() + PB_NET_DEADLINE_MS;
      g_state = NET_RECV;
      return;

    case NET_RECV: {
      modem_ran_();
      /* client.read(buf, cap) and NOTHING else: available() would add an _AVAILABLE, and
         connected() costs TWO more. The PB_NET_DEADLINE_MS deadline is the closed-socket
         detector instead, for zero AT commands (spec §3 change 3). */
      int r = sock_read((uint8_t *)g_rx + g_rx_len, (size_t)(sizeof g_rx - g_rx_len));
      if (r > 0) g_rx_len = (uint16_t)(g_rx_len + r);
      const char *body; uint16_t blen;
      if (rx_complete(&body, &blen)) { g_state = NET_CLOSE; return; }
      if (r < 0 || hal_millis() >= g_deadline) { finish(0, false); return; }
      return;
    }

    case NET_CLOSE: {                   /* 0 ATs: interpretation only */
      const uint16_t st = rx_status();
      const char *body; uint16_t blen;
      if (st == 200 && rx_complete(&body, &blen)) {
        /* The previous report's ack was delivered. Clear it BEFORE the next command can set
           the receipt placeholder, or every report repeats the same ack forever (§4.3). */
        report_clear_ack();
        response_t rs;
        const bool got = response_parse(body, blen, &rs);
        if (rs.next_s) g_next_s = rs.next_s;
        if (got) {
          g_cmd = rs.cmd; g_have_cmd = true;
          report_set_ack(rs.cmd.id, 0, "recv");   /* the ack exists from RECEIPT, not from a dose */
        }
        finish(200, true);
      } else {
        /* Only a 200 body reaches response_parse: butler's 400 body echoes the board's own
           tokens (f"{key}= out of range: {value}"), so a 4xx body could otherwise be parsed
           for cmd=/ml= (§4.2). */
        finish(st, false);
      }
      return;
    }
  }
}
```

`strncasecmp` comes from `<strings.h>` on the host and from `<string.h>` on the Renesas newlib;
include both, guarded, or hand-roll a five-line case-insensitive compare — hand-rolling is the
safer choice for the device build and costs nothing.

5. [ ] **Run the two framing cases.**

```
pio test -e native -f test_net
```

Expected `5 Tests 0 Failures` (three from task 21 plus these two).

6. [ ] **Commit.**

```
git add include/netfsm.h src/netfsm.cpp test/test_net/test_netfsm.cpp
git commit -m "netfsm: one bounded step per poll, headers and body in one write

Content-Length is computed from the bytes actually placed in the buffer. A
64-character token overflowing g_tx would otherwise cut the body while
Content-Length still claimed the full length: uvicorn waits for bytes that never
come, sock_close() raises ClientDisconnect, butler 400s 'client went away', and a
4xx is never retried — so every report 400s from the first one, forever, with the
console looking healthy. Spec 4.2.

net_disable()/net_disabled() are additions to the skeleton's list: setup()'s
watchdog-grant assertion disables the network per spec 3, and nothing consumed
that before — net_poll() had no disabled branch and status never said so.

net_poll() takes the dosing flag as a PARAMETER rather than forward-declaring
safety_dosing(). 9's grep keeps safety.h out of this file, and a forward
declaration is that include by another name; loop() supplies the flag.

sensors_sweep() gets its owner here, and its only one: once per report cycle, in
the NET_IDLE pass that issues zero AT commands, which is how spec 3's 'skipped in
any pass where a modem command ran' holds by construction. main.cpp's loop() does
not call it.

netfsm includes ui.h and calls ui_modem_ran() in every pass that issues an AT
command. Without that call ui.cpp's flag is always false and the other half of the
same 3 rule - neither screen painted after a modem pass - is not implemented at
all, which is up to 102 s of wedged-bus LCD painting on top of a 2.4 s modem pass."
```

7. [ ] **Write the socket-lifecycle cases.**

```c
static void test_socket_is_closed_on_success_error_timeout_and_a_failed_open(void) {
  sensors_begin();
  /* success */
  net_begin(); link_fake_queue_response(k200, strlen(k200));
  pump_passes(8);
  TEST_ASSERT_EQUAL(NET_IDLE, net_state());
  TEST_ASSERT_TRUE(sock_open());          /* the precondition holds: _sock was left -1 */
  sock_close();
  /* a failed open */
  net_begin(); link_fake_fail_open(true);
  pump_passes(8);
  link_fake_fail_open(false);
  TEST_ASSERT_TRUE(sock_open());          /* would be false if the failed open had not closed */
  sock_close();
  /* a timeout in RECV: no response was ever queued */
  net_begin();
  pump_passes(20);
  TEST_ASSERT_TRUE(sock_open());
  sock_close();
}

static void test_connect_is_never_issued_without_a_close_in_a_prior_pass(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  net_state_t prev = net_state();
  for (int i = 0; i < 24; ++i) {
    link_fake_pass_begin();
    net_poll(false);
    if (net_state() == NET_CONNECT) TEST_ASSERT_EQUAL(NET_SOCK_CLOSE, prev);
    prev = net_state();
  }
}

static void test_no_pass_issues_more_than_two_at_commands(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  for (int i = 0; i < 40; ++i) {
    link_fake_pass_begin();
    net_poll(false);
    TEST_ASSERT_TRUE(link_fake_at_count() <= 2);
  }
}

static void test_every_error_exit_transitions_to_sock_close_rather_than_closing_inline(void) {
  sensors_begin();
  net_begin();
  link_fake_fail_open(true);
  for (int i = 0; i < 24; ++i) {
    link_fake_pass_begin();
    net_state_t before = net_state();
    net_poll(false);
    if (before == NET_CONNECT) {
      TEST_ASSERT_EQUAL(NET_SOCK_CLOSE, net_state());
      TEST_ASSERT_EQUAL_UINT16(2, link_fake_at_count());  /* NOT 3: no inline _CLIENTCLOSE */
      link_fake_fail_open(false);
      return;
    }
  }
  TEST_FAIL_MESSAGE("the FSM never reached NET_CONNECT");
}

static void test_sock_read_calls_neither_available_nor_connected(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  for (int i = 0; i < 40; ++i) {
    link_fake_pass_begin();
    net_state_t before = net_state();
    net_poll(false);
    if (before == NET_RECV) TEST_ASSERT_EQUAL_UINT16(1, link_fake_at_count());
  }
  TEST_ASSERT_FALSE(link_fake_saw_available());
  TEST_ASSERT_FALSE(link_fake_saw_connected());
}
```

8. [ ] **Run them.**

```
pio test -e native -f test_net
```

Expected: all pass against the step-4 implementation. If
`test_every_error_exit_transitions_to_sock_close_rather_than_closing_inline` reports
`FAIL: Expected 2 Was 3`, something closed the socket inline — that pass is 3600 ms and
`3600 + PB_NET_SLACK_MS = 5600 > 5592`, a watchdog reset triggered by exactly the stale socket a
router reboot produces. Route it through `finish()`.

9. [ ] **Commit.**

```
git commit -am "The socket is closed on every exit, and never in a pass with another AT

A failed connect() leaves _sock >= 0 because getSocket() allocates before the
connect runs, and the design's exit list did not include a failed open. With
_sock == -1 in a prior pass, getSocket()'s connected()/stop() branch is skipped
and the connect path is 2 round trips instead of 5. Spec 3, changes 1 and 2."
```

10. [ ] **Write the two body-hygiene cases.**

```c
static const char *k400 =
  "HTTP/1.1 400 Bad Request\r\nContent-Length: 38\r\n\r\n"
  "next=60\ncmd=1 water=3 ml=250 cap_s=30\n";

static void test_response_is_never_parsed_from_a_four_hundred_body(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k400, strlen(k400));
  pump_passes(10);
  cmd_t c;
  TEST_ASSERT_FALSE(net_take_command(&c));      /* a 400 body echoes OUR tokens back at us */
  TEST_ASSERT_EQUAL_UINT16(400, net_last_status());
  TEST_ASSERT_EQUAL_UINT32(0, net_reports_ok());
}

static void test_stale_bytes_in_the_rx_buffer_cannot_become_a_command(void) {
  sensors_begin();
  net_begin();
  /* round 1: a complete 200 carrying a command */
  const char *with_cmd =
    "HTTP/1.1 200 OK\r\nContent-Length: 38\r\n\r\nnext=60\ncmd=5 water=3 ml=250 cap_s=30\n";
  link_fake_queue_response(with_cmd, strlen(with_cmd));
  pump_passes(10);
  cmd_t c;
  TEST_ASSERT_TRUE(net_take_command(&c));
  TEST_ASSERT_EQUAL_UINT32(5, c.id);
  report_clear_ack();                            /* stand in for exec_pending()'s real ack */
  /* round 2: the server answers with nothing at all. The old bytes must not be re-parsed. */
  link_fake_queue_response("", 0);
  pump_passes(30);
  TEST_ASSERT_FALSE(net_take_command(&c));
}
```

11. [ ] **Run them, and expect the second to be the one that bites.**

```
pio test -e native -f test_net
```

If `test_stale_bytes_in_the_rx_buffer_cannot_become_a_command` fails, `g_rx`/`g_rx_len` are not
being zeroed on entering `NET_SOCK_CLOSE`. That is the exact shape a poisoned AT session produces,
and with the replay guard of task 23 the second execution of `cmd=5` would still be refused — but
the guard is the second line of defence, not the first. Both must hold.

12. [ ] **Commit.**

```
git commit -am "Only a 200 body is parsed, and the rx buffer is zeroed every cycle"
```

13. [ ] **Add the pump-on injector to the fake.** `test_poll_is_a_noop_while_the_pump_is_asserted`
   cannot be written from outside a dose: `dose_run()` blocks, and `safety_dosing()` has no setter
   by design. Give `hal_sim` the hook, which is exactly the kind of fault injector `sim.h` exists
   for. In `include/sim.h`:

```c
void sim_on_pump_on(void (*cb)(void));   /* called ONCE, from inside the next pump assertion */
```

In `src/hal_sim.cpp`, a file static `static void (*g_on_pump_on)(void);`, set by
`sim_on_pump_on()`, cleared by `sim_reset()`, and invoked at the end of `hal_pump_write(true)`:

```c
  if (on && g_on_pump_on) { void (*cb)(void) = g_on_pump_on; g_on_pump_on = 0; cb(); }
```

14. [ ] **Write the guard case.**

```c
static uint16_t g_at_in_dose;
static net_state_t g_state_in_dose;
static void poke_net_from_inside_the_dose(void) {
  link_fake_pass_begin();
  /* safety_dosing() is TRUE here — we are inside hal_pump_write(true) — and this is the one
     call site in the suite that must pass it, because it is the guard under test. */
  net_poll(safety_dosing());
  g_at_in_dose = link_fake_at_count();
  g_state_in_dose = net_state();
}

static void test_poll_is_a_noop_while_the_pump_is_asserted(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  pump_passes(4);                              /* park the FSM somewhere with work to do */
  const net_state_t before = net_state();
  sim_set_float(true);
  sim_set_flow_ml_s(30);
  sim_on_pump_on(poke_net_from_inside_the_dose);
  dose_req_t q = { 0, 0, true, 1500, false, false };   /* by_time, no position needed */
  (void)dose_run(&q);
  TEST_ASSERT_EQUAL_UINT16(0, g_at_in_dose);   /* not one AT command while D6 is hot */
  TEST_ASSERT_EQUAL(before, g_state_in_dose);  /* and not one state transition either */
}
```

This case includes `safety.h` in the **test** file, which is fine — §9's grep is over
`src/netfsm.cpp`, not over `test/`.

15. [ ] **Run it and see it fail if the guard is missing.**

```
pio test -e native -f test_net
```

Expected with the guard present: pass. Delete the
`if (dosing || cart_busy()) return;` line once and rerun to see
`FAIL: Expected 0 Was 2` — that is a 2400 ms modem pass stacked on top of a running pump, inside a
5592 ms grant. Restore the line.

16. [ ] **Commit.**

```
git commit -am "net_poll() is a no-op while D6 is asserted, proved from inside the dose

sim_on_pump_on() is a new injector: the guard cannot be tested from outside a
dose, because dose_run() blocks. The dosing flag reaches netfsm as a PARAMETER,
not as a forward declaration of safety_dosing(), so check.sh's
zero-hits-for-safety.h grep over this file stays honest without a loophole in it."
```

17. [ ] **Add the seven `status` lines and TWO includes — `#include "netfsm.h"` AND
   `#include "link.h"`, both at `src/cli.cpp`'s top level.** `link_state()`, `link_rssi()` and
   `link_ip()` are declared in `link.h` and nowhere else; `netfsm.h` includes only
   `<stdbool.h>`, `<stdint.h>` and `report.h`, so it does not reach them. Without the second
   include this step's first three lines are undeclared and `pio test -e native` and
   `pio run -e uno_r4_wifi_sim` both fail at step 18. (`link.h` in `cli.cpp` is not a hygiene
   violation: §9's network-hygiene grep is scoped to `src/safety.cpp` and `lib/Manifold`.) Every one is bare `key=value`, like every other `status` line and like task 27 step 7's expected output — **do not write `link: state=`**, which is not a `k=v` token and which task 27's pass criteria do not contain. In `src/cli.cpp`, inside `cli_print_status()`:

```c
  cli_printf_u32("link=%lu\n", (uint32_t)link_state());       /* 0 down, 1 joining, 2 up */
  cli_printf_i32("rssi=%ld dBm\n", (int32_t)link_rssi());     /* task 11's signed printer */
  hal_serial_write("ip="); hal_serial_write(link_ip()); hal_serial_write("\n");
  cli_printf_u32("http_last=%lu\n", (uint32_t)net_last_status());
  cli_printf_u32("reports_ok=%lu\n", net_reports_ok());
  cli_printf_u32("reports_failed=%lu\n", net_reports_failed());
  cli_printf_u32("modem_ran=%lu\n", (uint32_t)(net_modem_ran_this_pass() ? 1u : 0u));
  if (net_disabled()) { hal_serial_write("net=DISABLED ("); hal_serial_write(net_disabled());
                        hal_serial_write(")\n"); }
```

RSSI is signed and `cli_printf_u32` is not, so this uses `cli_printf_i32()` — declared in
`include/cli.h` and defined in `src/cli.cpp` by task 11, precisely so this task does not have to
invent a public symbol. What is not fine is a `%d` in `report.cpp` or `netfsm.cpp`; `cli.cpp` is
outside that grep.

18. [ ] **Run both gates and commit.**

```
pio test -e native
pio run -e uno_r4_wifi_sim
```

`pio run -e uno_r4_wifi` does **not** link from this task until task 27 lands: `netfsm.cpp`
references seam 2, `[env:uno_r4_wifi]` excludes `link_fake.cpp`, and `link_wifi.cpp` does not exist
yet. The device-side gate for tasks 24, 25 and 26 is `pio run -e uno_r4_wifi_sim`, which compiles
`link_fake.cpp` and ignores `lib/Network`. Say so in the commit message; task 27 restores the
`uno_r4_wifi` gate.

```
git commit -am "status prints link state, RSSI, the last HTTP status and reports ok/failed

The device env does not link until task 27 supplies link_wifi.cpp; the gate for
this task and the next two is pio run -e uno_r4_wifi_sim, which compiles the fake
link and ignores lib/Network. A 400/401 loop is invisible to anyone not on the
serial port, which is why the last HTTP status is on the LCD too."
```

---

### Task 25: The retry policy, the 4xx/5xx split, and link poisoning

**Drop 3.**

**Files:**

- Modify: `src/netfsm.cpp` (`finish()`, every one of its call sites, and a new `poison()` helper;
  no new state and no new public declaration), `src/cli.cpp` (**four** lines inside
  `cli_print_status()`, step 14), `test/test_net/test_netfsm.cpp` (append ten cases and their
  `RUN_TEST` lines).
- Test: `test/test_net/test_netfsm.cpp`.

**Interfaces:**

*Consumes.* From `include/report.h` (task 22): `uint32_t report_t_ms(void);` — the **raw,
unsalted** `hal_millis()` stamped alongside `g_t_wire`. From `include/link.h` (task 21):
`void link_reset(void);`, `uint16_t link_desyncs(void);`. From `include/sim.h` (task 21):
`void link_fake_timeout_next(void);`, `void link_fake_drop_link(void);`,
`const uint8_t *link_fake_sent(uint16_t *len);`. From `include/config.h` (task 2):
`PB_RETRY_DEADLINE_MS` 30000, `PB_NET_BACKOFF_MS` `{2000,4000,8000,16000,30000}`,
`PB_NET_STEP_MS` 1200, `PB_NET_DEADLINE_MS` 5000.

*Produces.* No new declarations. Three behaviours inside `netfsm.cpp`:

- **The retry-eligible set is exactly two cases**: (a) zero response bytes arrived, and (b) a
  **complete** 503. Everything else is discarded — a 4xx, a truncated reply, a parse failure, a
  500, any other non-200.
- **The retry deadline is `hal_millis() - report_t_ms() >= PB_RETRY_DEADLINE_MS`, on the RAW
  stamp.** Past it the report is **abandoned**, never sent.
- **Any modem timeout is treated as link poisoned**: do not issue the next command,
  `link_reset()`, back to `NET_DOWN`, re-join on the `PB_NET_BACKOFF_MS` ladder, counted as
  `ch206`. `WiFi.ping()` is never called — it resets `modem.timeout()` to 10000 ms and would
  silently undo the entire margin (`WiFi.cpp:585-593`).

**A modem timeout has no primitive.** `link.h` is ten functions and stays ten. A timeout is
detected by elapsed time on a call that **failed**: a timeout always costs a full
`PB_NET_STEP_MS` and always yields the failure value, while a successful two-AT pass never
produces one. That is why the check is inside the failure branch and not around the whole pass —
a legitimate two-AT `sock_open()` can take 2 × 1199 ms with nothing wrong.

---

1. [ ] **Write the single-stamp bug first, so the deadline case has something to catch.** In
   `src/netfsm.cpp`, add above `finish()`:

```c
static bool g_retried;

/* WRONG ON PURPOSE (step 5 fixes it): measured against the SALTED wire stamp. */
static bool retry_window_open(void) {
  return (hal_millis() - report_t_wire()) < PB_RETRY_DEADLINE_MS;
}
```

and replace `finish()` with:

```c
/* The retry-eligible set is exactly two cases and nothing else (spec §4.4):
   (a) zero response bytes arrived, and (b) a COMPLETE 503, which is raised only on
   sqlite3.OperationalError (butler.py:1638-1639) and rolls the whole BEGIN IMMEDIATE back.
   Everything else is discarded: a 4xx (the backend answered; the same body cannot get better),
   a truncated reply, a parse failure, a 500, any other non-200.

   If ANY response bytes arrived, do not retry. When the request lands and the RESPONSE is lost,
   the backend has already moved a command queued -> sent (:870-873); the retry then hits the
   unconditional expire (:837-841) and kills a command the board never saw — a HIGH "never
   acknowledged" page for a dose that never existed, and the pot charged the full ml because
   flow_ml is NULL. Retrying is what destroys it, which is why a TRUNCATION is on the
   never-retry side: a truncation is bytes that arrived. */
static void finish(uint16_t status, bool ok, bool retry_eligible) {
  if (status) g_status = status;
  if (ok) ++g_ok; else ++g_failed;
  const bool retry = retry_eligible && !g_retried && g_body_len != 0 && retry_window_open();
  if (retry) g_retried = true;      /* g_body_len KEPT: SOCK_CLOSE routes back to CONNECT */
  else       g_body_len = 0;        /* past the deadline the report is ABANDONED, never sent */
  g_state = NET_SOCK_CLOSE;
}
```

Update the call sites in `net_poll()`:

```c
    case NET_CONNECT: ...  if (!sock_open()) { finish(0, false, true); return; }
    case NET_SEND:    ...  if (!assemble())  { finish(0, false, false); return; }
                           if (sock_write(...) != (int)g_tx_len) { finish(0, false, true); return; }
    case NET_RECV:    ...  if (r < 0 || hal_millis() >= g_deadline)
                             { finish(0, false, g_rx_len == 0); return; }
    case NET_CLOSE:   ...  finish(200, true, false);          /* the 200 path */
                      ...  finish(st, false, st == 503);      /* everything else */
```

and, in `NET_IDLE`, immediately after a new body is built successfully:

```c
      g_retried = false;              /* each report gets its own single retry */
```

2. [ ] **Write the two retry cases.** Append to `test/test_net/test_netfsm.cpp`:

```c
/* Passes with wall clock, because the RECV deadline and the retry deadline are both in ms.
   This is pb_net_passes(1, ms_each) with a send counter wrapped round it — one pass at a
   time, so that the counter can see the state the pass STARTED in. Do not re-derive the pass
   itself here: harness.h's helper is the one spelling (task 24 step 1). */
static int run_passes(int n, uint32_t ms_each) {
  int sends = 0;
  for (int i = 0; i < n; ++i) {
    const net_state_t before = net_state();
    pb_net_passes(1u, ms_each);
    if (before == NET_SEND) ++sends;
  }
  return sends;
}

static void test_an_exchange_that_produced_no_bytes_is_retried_exactly_once(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response("", 0);        /* the server says nothing at all */
  const int sends = run_passes(120, 200); /* 24 s: two RECV deadlines, inside the 30 s window */
  TEST_ASSERT_EQUAL_INT(2, sends);        /* the original and ONE retry */
  TEST_ASSERT_EQUAL_UINT32(0, net_reports_ok());
}

static void test_a_retry_is_abandoned_rather_than_sent_outside_the_dedup_window(void) {
  sim_reset(true);                        /* WARM: the boot counter advances, so the salt is
                                             non-zero and t= is above 2^31 (spec §15.2) */
  sensors_begin();
  TEST_ASSERT_NOT_EQUAL(0, hal_boot_salt());
  net_begin();
  link_fake_queue_response("", 0);
  /* inside the window: the retry IS sent */
  TEST_ASSERT_EQUAL_INT(2, run_passes(120, 200));
  /* a fresh report, then let the retry deadline expire before the FSM can resend */
  net_begin();
  link_fake_queue_response("", 0);
  int sends = run_passes(40, 200);        /* the original goes out, RECV then times out */
  TEST_ASSERT_EQUAL_INT(1, sends);
  sim_advance(PB_RETRY_DEADLINE_MS + 1000);
  sends += run_passes(40, 200);
  TEST_ASSERT_EQUAL_INT(1, sends);        /* ABANDONED: never sent outside the dedup window */
}
```

3. [ ] **Run them and see the first one fail.**

```
pio test -e native -f test_net
```

Expected:
`test/test_net/test_netfsm.cpp:NN:test_an_exchange_that_produced_no_bytes_is_retried_exactly_once:FAIL: Expected 2 Was 1`
— `hal_millis() - report_t_wire()` evaluates to `elapsed - salt` mod 2^32, a huge number for any
non-trivial salt, so the window is never open and **every retry is abandoned before it is sent**.
Cycle 1's "retry once" requirement quietly does not exist.

4. [ ] **Fix it: measure on the raw stamp.** Replace the helper:

```c
/* PB_RETRY_DEADLINE_MS = 30000, well inside butler's RETRY_WINDOW_S = 300 (butler.py:86).
   g_t_ms is the UNSALTED hal_millis() stamped alongside g_t_wire (§4.1). Measuring against the
   wire value gives `elapsed - salt` mod 2^32. Two variables, one purpose each (§4.4). */
static bool retry_window_open(void) {
  return (hal_millis() - report_t_ms()) < PB_RETRY_DEADLINE_MS;
}
```

5. [ ] **Run and see both pass, then commit.**

```
pio test -e native -f test_net
```

Expected: both green.

```
git commit -am "The retry is measured on the raw stamp, not the salted one

Written single-stamp first: millis() - g_t_wire is elapsed - salt mod 2^32, so
every retry was abandoned before it was sent and the 'retry once' requirement
quietly did not exist. Past the deadline the report is ABANDONED and never sent —
a retry deferred to the next interval falls outside butler's 300 s dedup window,
inserts the same reading twice with the same t=, pollutes the 5-sample median with
duplicates of one sample and runs water_rules on it a second time. Spec 4.4."
```

6. [ ] **Write the never-retry cases.**

```c
static void test_a_response_that_produced_any_bytes_is_never_retried(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response("HTTP/1.1 2", 10);      /* bytes arrived; the answer never completed */
  TEST_ASSERT_EQUAL_INT(1, run_passes(120, 200));
}

static void test_a_truncated_reply_is_never_retried(void) {
  sensors_begin();
  net_begin();
  const char *cut = "HTTP/1.1 200 OK\r\nContent-Length: 38\r\n\r\nnext=60\ncmd=5 wat";
  link_fake_queue_response(cut, strlen(cut));
  TEST_ASSERT_EQUAL_INT(1, run_passes(120, 200));  /* a truncation is bytes that ARRIVED */
  cmd_t c;
  TEST_ASSERT_FALSE(net_take_command(&c));         /* and a half-read reply never waters */
}

static void test_a_four_hundred_is_never_retried(void) {
  sensors_begin();
  net_begin();
  const char *b = "HTTP/1.1 400 Bad Request\r\nContent-Length: 5\r\n\r\nnope\n";
  link_fake_queue_response(b, strlen(b));
  TEST_ASSERT_EQUAL_INT(1, run_passes(60, 200));
  TEST_ASSERT_EQUAL_UINT16(400, net_last_status());
}

static void test_a_five_hundred_is_not_retried(void) {
  sensors_begin();
  net_begin();
  const char *b = "HTTP/1.1 500 Internal Server Error\r\nContent-Length: 5\r\n\r\noops\n";
  link_fake_queue_response(b, strlen(b));
  TEST_ASSERT_EQUAL_INT(1, run_passes(60, 200));   /* no rollback guarantee: not a 503 */
  TEST_ASSERT_EQUAL_UINT16(500, net_last_status());
}

static void test_a_five_oh_three_is_retried_once(void) {
  sensors_begin();
  net_begin();
  const char *b = "HTTP/1.1 503 Service Unavailable\r\nContent-Length: 5\r\n\r\nbusy\n";
  link_fake_queue_response(b, strlen(b));
  TEST_ASSERT_EQUAL_INT(2, run_passes(60, 200));   /* sqlite3.OperationalError rolls it all back */
  TEST_ASSERT_EQUAL_UINT16(503, net_last_status());
}

static void test_report_body_is_byte_identical_on_the_retry(void) {
  sensors_begin();
  net_begin();
  const char *b = "HTTP/1.1 503 Service Unavailable\r\nContent-Length: 5\r\n\r\nbusy\n";
  link_fake_queue_response(b, strlen(b));
  static uint8_t first[PB_TX_CAP];
  uint16_t first_len = 0, len = 0;
  int sends = 0;
  for (int i = 0; i < 60; ++i) {
    link_fake_pass_begin();
    const net_state_t before = net_state();
    net_poll(false);
    if (before == NET_SEND) {
      const uint8_t *tx = link_fake_sent(&len);
      if (++sends == 1) { memcpy(first, tx, len); first_len = len; }
      else { TEST_ASSERT_EQUAL_UINT16(first_len, len);
             TEST_ASSERT_EQUAL_MEMORY(first, tx, len); }
      link_fake_queue_response(b, strlen(b));      /* the same 503 again */
    }
    sim_advance(200);
  }
  TEST_ASSERT_EQUAL_INT(2, sends);
}
```

Note the 503 cases requeue the canned response after each send, because `link_fake_queue_response`
holds one answer at a time.

7. [ ] **Run them.**

```
pio test -e native -f test_net
```

Expected: green. If `test_a_truncated_reply_is_never_retried` reports `Expected 1 Was 2`, the
`finish()` call in `NET_RECV` is passing `true` instead of `g_rx_len == 0`; that is the bug that
kills a command the board never saw.

8. [ ] **Commit.**

```
git commit -am "4xx, 500, truncation and parse failures are discarded; only a 503 and a silent exchange retry

A 503 is raised ONLY on sqlite3.OperationalError, which rolls the whole
BEGIN IMMEDIATE transaction back, so retrying it is provably safe. Anything else
raised in the threadpool becomes a FastAPI 500 with no rollback guarantee, so it
is not in the same class. This is a deliberate deviation from fake_device.py,
which treats every HTTPError as terminal."
```

9. [ ] **Write the poisoning and backoff cases.**

```c
static void test_a_modem_timeout_poisons_the_link_and_counts_a_desync(void) {
  sensors_begin();
  net_begin();
  link_fake_queue_response(k200, strlen(k200));
  /* walk to the first state that issues an AT, then time it out */
  for (int i = 0; i < 40; ++i) {
    link_fake_pass_begin();
    const net_state_t before = net_state();
    if (before == NET_CONNECT) {
      link_fake_timeout_next();
      const uint16_t desyncs_before = link_desyncs();
      const uint16_t resets_before = link_fake_reset_count();
      net_poll(false);
      TEST_ASSERT_EQUAL_UINT16(desyncs_before + 1, link_desyncs());   /* rides out as ch206 */
      TEST_ASSERT_EQUAL_UINT16(resets_before + 1, link_fake_reset_count());
      TEST_ASSERT_EQUAL(NET_DOWN, net_state());        /* the NEXT command is NOT issued */
      /* and the next pass issues nothing at all while the backoff runs */
      link_fake_pass_begin();
      net_poll(false);
      TEST_ASSERT_EQUAL_UINT16(0, link_fake_at_count());
      return;
    }
    net_poll(false);
    sim_advance(50);
  }
  TEST_FAIL_MESSAGE("the FSM never reached NET_CONNECT");
}

static void test_link_drop_returns_to_joining_with_exponential_backoff(void) {
  static const uint32_t ladder[] = PB_NET_BACKOFF_MS;
  sensors_begin();
  net_begin();
  /* join, then pull the AP out from under it */
  for (int i = 0; i < 8 && net_state() != NET_IDLE; ++i) { link_fake_pass_begin(); net_poll(false); }
  TEST_ASSERT_EQUAL(NET_IDLE, net_state());
  link_fake_drop_link();
  uint32_t seen[3] = {0, 0, 0};
  for (int rung = 0; rung < 3; ++rung) {
    /* drive until the FSM parks in NET_DOWN, then measure how long it waits */
    for (int i = 0; i < 60 && net_state() != NET_DOWN; ++i) {
      link_fake_pass_begin(); net_poll(false); sim_advance(50);
    }
    TEST_ASSERT_EQUAL(NET_DOWN, net_state());
    uint32_t waited = 0;
    while (net_state() == NET_DOWN && waited < 60000) {
      link_fake_pass_begin(); net_poll(false); sim_advance(100); waited += 100;
    }
    seen[rung] = waited;
    /* still down: the join fails again */
    link_fake_drop_link();
  }
  TEST_ASSERT_TRUE(seen[0] <= ladder[0] + 200);
  TEST_ASSERT_TRUE(seen[1] > seen[0]);
  TEST_ASSERT_TRUE(seen[2] > seen[1]);
}
```

10. [ ] **Run and see the poisoning case fail.**

```
pio test -e native -f test_net
```

Expected:
`test/test_net/test_netfsm.cpp:NN:test_a_modem_timeout_poisons_the_link_and_counts_a_desync:FAIL: Expected 1 Was 0`
— nothing calls `link_reset()` yet, so the session goes permanently one answer out of phase,
nothing hangs, the watchdog never bites, and `status` keeps printing `wifi UP` while the board
silently stops reporting for the rest of the 48 hours.

11. [ ] **Add the poison path to `src/netfsm.cpp`.** Above `net_poll()`:

```c
/* buf_read breaks out on Timeout (Modem.cpp:185-187) and leaves the late answer sitting in
   Serial2's RX FIFO; write() clears its result string (:100) but does not drain the UART, and
   the FSM's restart logic (:241,:267) resyncs some shapes and not others. So ANY modem timeout
   is treated as link poisoned: do not issue the next command. link_reset() is end();
   beginned = false; begin(); ++desyncs — and the middle line is the one the 48-hour run depends
   on. Never the ping helper: it resets modem.timeout() to 10000 ms (WiFi.cpp:585-593). */
static void poison(void) {
  link_reset();
  g_state = NET_DOWN;
  g_wait_until = hal_millis() + k_backoff[g_backoff_i];
  if (g_backoff_i + 1 < sizeof k_backoff / sizeof k_backoff[0]) ++g_backoff_i;
}

/* A timeout always costs a full step and always yields the failure value; a SUCCESSFUL two-AT
   pass never does. So the test lives inside the failure branch, never around the whole pass. */
static bool was_timeout(uint32_t t0) { return hal_millis() - t0 >= PB_NET_STEP_MS; }
```

and bracket every seam-2 call site in `net_poll()`:

```c
    case NET_JOIN_WAIT: {
      modem_ran_();
      const uint32_t t0 = hal_millis();
      link_state_t s = link_state();
      if (s == LINK_UP) { g_backoff_i = 0; g_state = NET_IDLE; return; }
      if (was_timeout(t0)) { poison(); return; }
      if (hal_millis() >= g_deadline) link_down();
      return;
    }

    case NET_SOCK_CLOSE: {
      modem_ran_();
      memset(g_rx, 0, sizeof g_rx);
      g_rx_len = 0;
      const uint32_t t0 = hal_millis();
      sock_close();
      if (was_timeout(t0)) { poison(); return; }
      g_state = g_body_len ? NET_CONNECT : NET_IDLE;
      return;
    }

    case NET_CONNECT: {
      modem_ran_();
      const uint32_t t0 = hal_millis();
      if (!sock_open()) {
        if (was_timeout(t0)) { poison(); return; }
        finish(0, false, true);
        return;
      }
      g_state = NET_SEND;
      return;
    }

    case NET_SEND: {
      modem_ran_();
      if (!assemble()) { finish(0, false, false); return; }
      const uint32_t t0 = hal_millis();
      if (sock_write((const uint8_t *)g_tx, g_tx_len) != (int)g_tx_len) {
        if (was_timeout(t0)) { poison(); return; }
        finish(0, false, true);
        return;
      }
      g_deadline = hal_millis() + PB_NET_DEADLINE_MS;
      g_state = NET_RECV;
      return;
    }

    case NET_RECV: {
      modem_ran_();
      const uint32_t t0 = hal_millis();
      int r = sock_read((uint8_t *)g_rx + g_rx_len, (size_t)(sizeof g_rx - g_rx_len));
      if (r > 0) g_rx_len = (uint16_t)(g_rx_len + r);
      const char *body; uint16_t blen;
      if (rx_complete(&body, &blen)) { g_state = NET_CLOSE; return; }
      if (r < 0 && was_timeout(t0)) { poison(); return; }
      if (r < 0 || hal_millis() >= g_deadline) { finish(0, false, g_rx_len == 0); return; }
      return;
    }
```

`NET_JOIN_ISSUE` needs the same bracket around `link_join()`, with the poison taken when the link
is still `LINK_DOWN` after the call and the call consumed a full step.

**A poisoned pass does not discard the report.** `poison()` leaves `g_body_len` alone, so once the
link comes back the pending body is sent from `NET_SOCK_CLOSE` → `NET_CONNECT` if the retry
deadline is still open, and abandoned by `finish()` if it is not.

12. [ ] **Run both new cases.**

```
pio test -e native -f test_net
```

Expected: green. If the backoff case fails on `seen[1] > seen[0]`, `g_backoff_i` is being reset
somewhere other than a successful `LINK_UP`.

13. [ ] **Commit.**

```
git commit -am "A modem timeout poisons the link: reset, back off, count it as ch206

Without the reset the session goes permanently one answer out of phase, nothing
hangs, the watchdog never bites, and status keeps printing wifi UP while the board
silently stops reporting for the rest of the 48 hours. The timeout is detected by
elapsed time on a FAILED call, not around the pass: a legitimate two-AT sock_open()
can take 2 x 1199 ms with nothing wrong. Spec 3, 4.4."
```

14. [ ] **Add the `status` lines.** In `src/cli.cpp`, inside `cli_print_status()`, next to the
   link lines added in task 24:

Bare `key=value`, like every other `status` line and like task 27 step 7's expected output:

```c
  cli_printf_u32("modem_timeout_ms=%lu\n", (uint32_t)PB_NET_STEP_MS);
  cli_printf_u32("conn_timeout_ms=%lu\n", (uint32_t)PB_NET_STEP_MS);
  hal_serial_write("connect_form=_CLIENTCONNECT(HOST_NAME as a name)"
                   " - UNVERIFIED until bring-up (spec 3 change 4)\n");
  cli_printf_u32("desyncs=%lu\n", (uint32_t)link_desyncs());   /* rides out as ch206 */
```

The `form=` string is what `link_begin()` will have asked for; task 27 replaces the literal with
what the driver actually sent, because `_CLIENTCONNECT` and `_CLIENTCONNECTNAME` are distinct
ESP32-side entry points whose firmware is not in this package — neither the unit of the timeout nor
whether `_CLIENTCONNECT` still resolves a hostname is checkable here, and both are bring-up
questions. Say so in the commit message.

15. [ ] **Run the gates and the greps, then commit.**

```
pio test -e native
pio run -e uno_r4_wifi_sim
grep -rn "WiFi\.ping" src include lib | wc -l      # must print 0
```

`pio run -e uno_r4_wifi` still does not link until task 27 supplies `link_wifi.cpp`; the sim env is
the device-side gate for this task, as it was for task 24.

```
git commit -am "status prints the modem timeout, the connect form and the desync count

The connect form is printed rather than asserted: the driver proves only the
command selection, and whether _CLIENTCONNECT still resolves a hostname is a
bring-up question. tools/check.sh reports 0 hits for WiFi.ping."
```

---

### Task 26: `exec.cpp` — the ack cycle, the boot self-home and the park

**Drop 3.**

**Files:**
- Create: `include/exec.h`
- Create: `src/exec.cpp`
- Create: `test/support/bodies.h` (the canned HTTP responses `test_net`, `test_cart` and `test_contra` share — **separate suite directories are separate binaries and separate translation units**, so a `static const char *` in one suite's `.cpp` cannot be named from another's)
- Modify: `src/main.cpp` (`loop()` reaches its final shape; `setup()` gains `cart_begin()`, the `net_disable()` forwarding, `net_begin()` and `exec_begin()`; **`ui_fill_()` is rewritten whole — this task owns every remaining field of `ui_state_t`**)
- Modify: `src/cli.cpp` (`cli_print_status()` gains the cart's parked state and the `stop=1` help text)
- Test: `test/test_net/test_netfsm.cpp`, `test/test_contra/test_contra.cpp`, `test/test_cart/test_cart.cpp`

**Interfaces:**

*Consumes:* `net_state_t net_state(void)`, `bool net_take_command(cmd_t *)`, `net_last_status()`, `net_next_s()`, `net_disable()` (task 24); `void report_set_ack(uint32_t, uint16_t, const char *)`, `bool report_ack_is_recv(void)` (task 22 — **`report_clear_ack()` is deliberately NOT consumed here: task 24's `NET_CLOSE` 200 path owns it, because §4.3 clears the slot after a 200 and not when the command is executed**); `cmd_t`, `cmd_kind_t` (task 23); `dose_result_t dose_run(const dose_req_t *)`, `uint16_t dose_flow_ml(void)`, `const char *err_of(dose_result_t)` (task 17); `void cli_print_dose_summary(void)` from `include/cli.h` (task 20); `bool cart_begin(void)`, `bool cart_goto(uint8_t)`, `bool cart_home(void)`, `cart_pos_known()`, `cart_pos()`, `cart_parked()`, `cart_busy()` (task 14); `link_state()`, `link_rssi()`, `link_ip()` (task 21); `safety_contra()`, `safety_dry()` (tasks 15, 19); `PB_OUTLETS`, `PB_BOOT_HOME_MS`, `PB_HANG_MS`, `PB_ML_PER_S_MEASURED` (task 2).

*Produces* — `include/exec.h`:

```c
void        exec_begin(void);
void        exec_pending(void);
uint32_t    exec_last_cmd_id(void);     /* 0 before any command; the OLED's `cmd N` row */
const char *exec_last_cmd_text(void);   /* "ok 248ml" | "REF float" | 0 -- §5's row 7 */
```

Four functions, not five: **there is no `exec_has_pending()`.** An earlier draft declared it and nothing anywhere called it — `exec.cpp` reads its own file static, `main.cpp` calls `exec_pending()` unconditionally, and `ui_fill_()` selects its LCD state from the cart and the pump rather than from a pending flag. A declared accessor with no caller is a reader's false lead; if a later change needs one, it can be added with its caller in the same commit.

`exec_last_cmd_id()` and `exec_last_cmd_text()` are set inside `ack()`, which is the one function every terminal path goes through, from the same `(id, flow_ml, err)` triple: `"ok <n>ml"` for a delivered dose and `"REF <err>"` otherwise, into a `static char[16]` of `exec.cpp`'s. Spec §5's OLED row 7 has no other source, and `ui_fill_()` (step 13c) reads both, so leaving them undefined is a link failure in the device build and an empty row 7 for the whole run.

**Spec conflict, resolved here and recorded in the commit message.** §1's module table and §11's tree put `exec_pending()` in `src/main.cpp`, but `[env:native]` filters `main.cpp` out (§10), which would make four of §9's `test_net` cases unrunnable — and moving it into `netfsm.cpp` would break §9's grep of `safety\.h|dose_run|hal_pump_write` over that file. It gets its own translation unit, which is in neither list and does compile on native.

---

1. - [ ] **Write `test/support/bodies.h` first** — the canned responses three suites share. `test_net`, `test_cart` and `test_contra` are three separate Unity binaries built from three separate directories, so a file static in one is unreachable from the others; every body used by more than one suite lives here.

```c
/* test/support/bodies.h -- canned HTTP responses. A HEADER, like harness.h: PlatformIO
   builds one binary per test/ subdirectory, so a `static const char[]` in test_netfsm.cpp
   cannot be named from test_cart.cpp. Every body used by more than one suite lives here.
   EVERY Content-Length below is counted, not estimated. netfsm's rx_complete() returns false
   while `have < cl`, so a length one byte too large makes the response a permanent truncation:
   the case hangs until the RECV deadline and then fails for a reason that has nothing to do
   with what it was written to test. The arithmetic, once, so it can be checked by eye:
     "next=60\n"                          =  8
     "cmd=NN water=N ml=NNN cap_s=NN\n"   = 31   (6 + 1 + 7 + 1 + 6 + 1 + 8 + 1)
     "cmd=NN stop=1\n"                    = 14   (6 + 1 + 6 + 1)
   so a water body is 39 and a stop body is 22. */
#pragma once

static const char k_cmd_200[] =
  "HTTP/1.1 200 OK\r\nContent-Length: 39\r\n\r\nnext=60\ncmd=17 water=3 ml=100 cap_s=10\n";
static const char k_stop_200[] =
  "HTTP/1.1 200 OK\r\nContent-Length: 22\r\n\r\nnext=60\ncmd=31 stop=1\n";
static const char k_out_of_range_200[] =
  "HTTP/1.1 200 OK\r\nContent-Length: 39\r\n\r\nnext=60\ncmd=62 water=0 ml=100 cap_s=10\n";
```

**Add one case to `test/test_report/test_report.cpp` that makes the arithmetic self-checking**, so that the next body added here cannot be wrong by one and take three suites down with it:

```c
static void test_every_canned_body_declares_its_own_true_content_length(void) {
  const char *const raw[] = { k_cmd_200, k_stop_200, k_out_of_range_200 };
  for (unsigned i = 0; i < 3u; ++i) {
    const char *hdr  = strstr(raw[i], "Content-Length: ");
    const char *body = strstr(raw[i], "\r\n\r\n") + 4;
    TEST_ASSERT_NOT_NULL(hdr);
    TEST_ASSERT_EQUAL_UINT32((uint32_t)strtoul(hdr + 16, NULL, 10), (uint32_t)strlen(body));
  }
}
```

(with `#include "../support/bodies.h"` and `<stdlib.h>` in that suite).

Then write `include/exec.h` and the first test — the receipt ack. Add to
`test/test_net/test_netfsm.cpp`:

```c
#include "../support/bodies.h"
#include "exec.h"

static void test_an_ack_is_set_the_moment_a_command_is_received(void) {
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, strlen(k_cmd_200));
  net_begin(); exec_begin();
  pb_net_passes(12, 100);
  TEST_ASSERT_TRUE(report_ack_is_recv());     /* (id, flow_ml = 0, err = "recv") on RECEIPT */
  TEST_ASSERT_FALSE(report_may_build());
}

static void test_command_is_not_executed_in_the_pass_that_received_it(void) {
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, strlen(k_cmd_200));
  net_begin(); exec_begin();
  for (int i = 0; i < 40; ++i) {
    link_fake_pass_begin();
    net_poll(false);
    if (report_ack_is_recv()) {               /* the pass that received it */
      TEST_ASSERT_EQUAL_UINT32(0, sim_pump_on_ms());
      break;
    }
    pb_advance(100);
  }
}

static void test_command_is_surfaced_only_once_per_round_trip(void) {
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, strlen(k_cmd_200));
  net_begin(); exec_begin();
  pb_net_passes(12, 100);
  cmd_t c;
  TEST_ASSERT_TRUE(net_take_command(&c));
  TEST_ASSERT_EQUAL_UINT32(17, c.id);
  TEST_ASSERT_FALSE(net_take_command(&c));
}

static void test_no_report_is_built_between_receiving_a_command_and_executing_it(void) {
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, strlen(k_cmd_200));
  net_begin(); exec_begin();
  pb_net_passes(12, 100);
  /* link_fake_write_count() and not link_fake_sent(): the question is whether ANYTHING was
     sent across two whole report intervals, and the last-buffer accessor cannot answer it. */
  uint16_t writes = link_fake_write_count();
  pb_advance(120000);                          /* two report intervals go by */
  pb_net_passes(20, 100);
  TEST_ASSERT_EQUAL_UINT16(writes, link_fake_write_count());   /* the report WAITS */
}
```

2. - [ ] Run and see it fail:

```
pio test -e native -f test_net
```

Expected: `fatal error: exec.h: No such file or directory`.

3. - [ ] Write `include/exec.h` and `src/exec.cpp`'s boot self-home and receipt half. The invariants above the code are quoted from §2.11 and §4.3:

```c
/* exec.cpp — at most one command per pass, executed only when the socket is closed.
   §1's module table puts this in main.cpp; it lives here because [env:native] filters main.cpp
   out and netfsm.cpp is grepped for dose_run. See the commit message. */
#include "exec.h"
#include "cart.h"
#include "cli.h"        /* cli_print_dose_summary() -- §6 wants the summary line from EVERY
                           dose path, and this file is the path that runs unattended */
#include "config.h"
#include "hal.h"
#include "netfsm.h"
#include "report.h"
#include "safety.h"
#include <stdio.h>
#include <string.h>

static bool     g_boot_home_due;
static bool     g_pending;
static cmd_t    g_cmd;
static uint32_t g_last_id;
static char     g_last_text[16];

void exec_begin(void) {
  g_boot_home_due = true; g_pending = false;
  g_last_id = 0u; g_last_text[0] = '\0';
}

uint32_t    exec_last_cmd_id(void)   { return g_last_id; }
const char *exec_last_cmd_text(void) { return g_last_text[0] ? g_last_text : 0; }

/* THE one function every terminal path goes through, which is why the OLED's row-7 pair is
   filled here and not at four call sites that would drift. "ok <n>ml" when water actually
   moved, "REF <err>" otherwise: §5's row 7 reads `cmd 17 ok 248ml` or `cmd 17 REF float`,
   and 16 characters is all there is. No float conversion anywhere (§12 item 1). */
static void ack(uint32_t id, uint16_t flow_ml, const char *err) {
  report_set_ack(id, flow_ml, err);
  g_last_id = id;
  if (err && strcmp(err, "none") == 0)
    snprintf(g_last_text, sizeof g_last_text, "ok %luml", (unsigned long)flow_ml);
  else
    snprintf(g_last_text, sizeof g_last_text, "REF %s", err ? err : "?");
}

void exec_pending(void) {
  /* §2.11: the boot self-home runs under BOTH latches. Gating it on !dry_latched would leave
     the cart wherever a mid-dose watchdog reset stopped it — and §2.3 latches dry on exactly
     that case — holding gate N open under the reservoir head until a human types `dry off`.
     It drives the servo, not D6; safety_tick() re-asserts pump-OFF on every pass of the move. */
  if (g_boot_home_due && hal_millis() >= PB_BOOT_HOME_MS) {
    g_boot_home_due = false;
    (void)cart_home();
  }

  if (!g_pending && !net_take_command(&g_cmd)) return;
  g_pending = true;
  /* Nothing has been consumed yet, so no park is owed if the socket is still open. */
  if (net_state() != NET_IDLE) return;

  /* From here the command is CONSUMED. The ack already exists — netfsm set (id, 0, "recv") on
     receipt — and every path below OVERWRITES it (§4.3). */

  /* PLACEHOLDER, replaced whole by step 8. Written out rather than elided so that the file
     as printed compiles and step 4's four cases can actually run. */
  g_pending = false;
}
```

4. - [ ] Run and see the four pass.

```
pio test -e native -f test_net
```

Expected: `0 Failures`. **Read what each case is actually proving, because one of the four passes vacuously against the placeholder.** `test_an_ack_is_set_the_moment_a_command_is_received` and `test_no_report_is_built_between_receiving_a_command_and_executing_it` are real: both are properties of `netfsm` setting `(id, 0, "recv")` on receipt and `report_may_build()` refusing while the slot reads `recv`. `test_command_is_surfaced_only_once_per_round_trip` is real too — it never calls `exec_pending()`. But `test_command_is_not_executed_in_the_pass_that_received_it` passes because the placeholder consumes the command and silently drops it, which is also a way of not executing it; it only becomes meaningful after step 8. Do not read this green as coverage of the terminal paths — step 6's cases are what cover those, and they fail until step 8.

5. - [ ] Commit the receipt half:

```
git add include/exec.h src/exec.cpp test/test_net/test_netfsm.cpp
git commit -m "exec_pending(): the receipt ack, and the boot self-home under both latches

The ack is a property of RECEIVING a command, not of running a dose: netfsm sets
(id, flow_ml = 0, err = 'recv') the moment response_parse yields one, and no report may be
built while it still reads recv. Without that rule the placeholder reaches the wire, butler
applies state='acked' flow_ml=0, pages HIGH, sets the pot's cooldown and charges 0 ml — and
THEN the board runs the full dose. It was previously an emergent property of loop()'s ordering.

The cart homes itself once at PB_BOOT_HOME_MS, under the dry latch and under the contradiction
latch alike. dose_run() refuses when position is unknown and water_rules returns early on
pos != 'ok', so without a self-home one overnight reset leaves the rig permanently dry: the
backend never queues, so nothing ever triggers a home.

Spec conflict, resolved: §1 and §11 put exec_pending() in main.cpp, but [env:native] filters
main.cpp out (§10), which would make four of §9's test_net cases unrunnable, and netfsm.cpp is
grepped for dose_run. Its own translation unit is in neither list and compiles on native."
```

6. - [ ] Write the terminal-path tests. Add to `test/test_net/test_netfsm.cpp`:

```c
static void test_a_stop_command_is_acked(void) {
  static const char k[] = "HTTP/1.1 200 OK\r\nContent-Length: 22\r\n\r\nnext=60\ncmd=31 stop=1\n";
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k, strlen(k));
  net_begin(); exec_begin();
  pb_net_passes(14, 100);
  exec_pending();
  TEST_ASSERT_FALSE(report_ack_is_recv());
  report_stamp();
  char b[PB_BODY_CAP]; (void)report_build(b, sizeof b);
  TEST_ASSERT_NOT_NULL(strstr(b, " ack=31 flow_ml=0 err=stop"));
}

static void test_a_failed_goto_still_acks(void) {
  /* PB_PULSES_PER_GATE == 0 compiles cart_goto() to `return false` (§2.15), so this is the
     shipped configuration's normal answer, not a contrived one. */
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, strlen(k_cmd_200));
  net_begin(); exec_begin();
  pb_net_passes(14, 100);
  exec_pending();
  report_stamp();
  char b[PB_BODY_CAP]; (void)report_build(b, sizeof b);
  TEST_ASSERT_NOT_NULL(strstr(b, " ack=17 flow_ml=0 err=goto"));
}

static void test_every_terminal_path_in_exec_pending_sets_an_ack(void) {
  static const char *bodies[] = {
    "HTTP/1.1 200 OK\r\nContent-Length: 39\r\n\r\nnext=60\ncmd=41 water=0 ml=100 cap_s=10\n",
    "HTTP/1.1 200 OK\r\nContent-Length: 39\r\n\r\nnext=60\ncmd=42 water=9 ml=100 cap_s=10\n",
    "HTTP/1.1 200 OK\r\nContent-Length: 39\r\n\r\nnext=60\ncmd=43 water=3 ml=100 cap_s=10\n",
    "HTTP/1.1 200 OK\r\nContent-Length: 22\r\n\r\nnext=60\ncmd=44 stop=1\n"
  };
  for (unsigned i = 0; i < 4; ++i) {
    pb_test_setup();
    link_fake_reset(); link_fake_set_state(LINK_UP);
    link_fake_queue_response(bodies[i], strlen(bodies[i]));
    net_begin(); exec_begin();
    pb_net_passes(14, 100);
    exec_pending();
    TEST_ASSERT_FALSE(report_ack_is_recv());
    report_stamp();
    char b[PB_BODY_CAP]; (void)report_build(b, sizeof b);
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(b, " ack="), "a terminal path left no ack");
    TEST_ASSERT_NOT_NULL(strstr(b, " flow_ml="));
  }
}

static void test_refused_dose_acks_with_flow_ml_zero_and_an_err_token(void) {
  static const char k[] =
    "HTTP/1.1 200 OK\r\nContent-Length: 39\r\n\r\nnext=60\ncmd=51 water=3 ml=999 cap_s=10\n";
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k, strlen(k));
  net_begin(); exec_begin();
  pb_net_passes(14, 100);
  exec_pending();
  report_stamp();
  char b[PB_BODY_CAP]; (void)report_build(b, sizeof b);
  TEST_ASSERT_NOT_NULL(strstr(b, "flow_ml=0"));   /* an ACKED refusal charges the pot 0 ml */
}

static void test_pending_ack_rides_the_next_report_after_every_discard_path(void) {
  static const char k_400[] = "HTTP/1.1 400 Bad Request\r\nContent-Length: 3\r\n\r\nno\n";
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, strlen(k_cmd_200));
  net_begin(); exec_begin();
  pb_net_passes(14, 100);
  exec_pending();                       /* ack=17 is now real */
  link_fake_queue_response(k_400, strlen(k_400));
  pb_net_passes(30, 2500);                 /* that report 400s and is discarded */
  uint16_t n = 0;
  const char *tx = (const char *)link_fake_sent(&n);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_NOT_NULL(strstr(tx, " ack=17 "));                 /* still on the next one */
}

static void test_err_recv_never_reaches_the_wire(void) {
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, strlen(k_cmd_200));
  net_begin(); exec_begin();
  pb_net_passes(60, 2500);                 /* many intervals; exec runs, then reports resume */
  uint16_t n = 0;
  const char *tx = (const char *)link_fake_sent(&n);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_NULL(strstr(tx, "err=recv"));
}
```

7. - [ ] Run and see them fail:

```
pio test -e native -f test_net
```

Expected: `test_a_stop_command_is_acked:FAIL: Expected Not NULL` — `exec_pending()` consumes the command and acks nothing.

8. - [ ] Write the terminal half of `exec_pending()`, replacing the `g_pending = false;` placeholder. One `park:` label, one call site, every path through it:

```c
  if (g_cmd.kind == CMD_STOP) { ack(g_cmd.id, 0, "stop"); goto park; }

  /* ABOVE cart_goto() on purpose: this is what makes §4.5's promise true — an out-of-range
     outlet is refused with err=range and acked, rather than the backend receiving whichever
     cart error happened first. It is also what handles water=0, which butler accepts. */
  if (g_cmd.outlet < 1 || g_cmd.outlet > PB_OUTLETS) { ack(g_cmd.id, 0, "range"); goto park; }

  if (!cart_goto(g_cmd.outlet)) { ack(g_cmd.id, 0, "goto"); goto park; }

  {
    /* `= {0}`, NOT a bare declaration. dose_req_t carries an unconditional `hang` member
       (task 20 step 7), and an uninitialised one plus `el >= PB_HANG_MS` puts a BACKEND
       water command into the loop that deliberately starves the watchdog for bring-up 7c.
       cli_run_dose_() zero-initialises for the same reason; this is the path that runs
       unattended for 48 hours. */
    dose_req_t q = {0};
    q.outlet = g_cmd.outlet;
    q.ml = g_cmd.ml;
    q.by_time = false;
    q.cap_ms = (uint32_t)g_cmd.cap_s * 1000u;
    q.need_pos = true;                   /* a backend water command: position must be known */
    q.long_prime = false;                /* never from the wire: `prime` is a console token */
#if defined(PB_DOSE_BY_TIME) && PB_DOSE_BY_TIME
    /* §6's stated 7b fallback, against the SAME constant the cap clamp uses; config.h #errors
       if PB_ML_PER_S_MEASURED is 0, because a by-time dose against an unmeasured rate is an
       unbounded run in a costume. */
    q.by_time = true;
    {
      uint32_t byt = (uint32_t)g_cmd.ml * 1000u / PB_ML_PER_S_MEASURED;
      if (byt < q.cap_ms) q.cap_ms = byt;
    }
#endif
    dose_result_t r = dose_run(&q);
    /* §6: the per-dose summary line is printed "at the end of every dose, from EVERY path".
       cli.cpp's own helper covers `pump` and `calib`; this is the other path, and it is the
       one that runs unattended. This is cli.cpp's SECOND exported entry point into this
       file, and it does not affect the single-`dose_run(`-call-site grep, which is scoped
       to cli.cpp on purpose. */
    cli_print_dose_summary();
    ack(g_cmd.id, dose_flow_ml(), err_of(r));   /* the HONEST millilitres, 0 for a refusal */
  }

park:
  /* §2.9: EVERY consumed command parks, goto failures included. The magnet cart lifts the gate
     it sits over and the reservoir sits above the pump inlet, so a cart left over outlet N
     holds that gate open under a head of water until the next command — six hours, or never. */
  g_pending = false;
  (void)cart_home();
}
```

9. - [ ] Run and see them pass, then add two more while you are here — one for the summary line, one for `hang`:

```c
static void test_a_backend_dose_prints_the_per_dose_summary_line(void) {
  pb_test_setup();
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, sizeof k_cmd_200 - 1u);
  net_begin(); exec_begin();
  pb_net_passes(14, 100);
  char out[512]; (void)sim_serial_tx(out, sizeof out);
  exec_pending();
  size_t n = sim_serial_tx(out, sizeof out); out[n] = 0;
  TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, "dose outlet="), out);
}

/* A garbage `hang` byte plus el >= PB_HANG_MS puts a BACKEND command into the loop that
   deliberately starves the watchdog. The field is unconditional (§6's object-hash rule), so
   zero-initialising the request is the only defence there is. */
static void test_a_backend_command_never_sets_hang(void) {
  pb_test_setup();
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, sizeof k_cmd_200 - 1u);
  net_begin(); exec_begin();
  pb_net_passes(14, 100);
  uint32_t f0 = sim_feeds();
  pb_advance(PB_HANG_MS * 3u);
  exec_pending();
  TEST_ASSERT_GREATER_THAN_UINT32(f0, sim_feeds());   /* the dog was fed throughout */
}
```

```
pio test -e native -f test_net
```

Expected: `0 Failures`.

10. - [ ] Commit:

```
git add src/exec.cpp test/test_net/test_netfsm.cpp
git commit -m "exec_pending(): every terminal path acks, and every consumed command parks

CMD_STOP has a handler; an outlet outside 1..PB_OUTLETS is refused with err=range ABOVE
cart_goto(); a failed goto acks err=goto. The design set the ack only inside dose_run(), so a
cart_goto that stalled, timed out or was compiled out never reached it and CMD_STOP had no
handler at all — the command then sat 'sent', was expired by the next report, charged the pot
the full ml and raised a HIGH 'handed to the board and never acknowledged'. An unacked stop is
worse still: the dose-judging query filters kind='water', so it produces no alert and the
operator believes it was delivered.

An acked refusal charges 0 ml; an expired one charges the full ml. That is the reason to
prefer acking a refusal, not the loudness — both alert branches are priority=high with the
same tags."
```

11. - [ ] Add the park tests to the cart and contra suites, where they belong. **Both suites need the network layer's headers and the shared bodies**, which their own Interfaces blocks never mentioned: add to the top of each file

```c
#include "../support/bodies.h"
#include "../support/harness.h"
#include "exec.h"
#include "netfsm.h"
#include "report.h"
#include "sim.h"        /* the link_fake_* control surface lives HERE (task 21). There is no
                           include/link_fake.h anywhere in this tree. */
```

In `test/test_cart/test_cart.cpp`:

```c
static void test_the_cart_is_parked_off_every_outlet_after_every_command_including_a_failed_goto(void) {
  pb_test_setup();
  link_fake_reset(); link_fake_set_state(LINK_UP);
  link_fake_queue_response(k_cmd_200, sizeof k_cmd_200 - 1u);   /* water=3; goto will fail */
  net_begin(); exec_begin();
  pb_net_passes(14, 100);
  exec_pending();
  TEST_ASSERT_TRUE(cart_parked());
}

static void test_the_cart_is_parked_after_a_stop_command_and_after_an_out_of_range_outlet(void) {
  const char *bodies[2]; size_t lens[2];
  bodies[0] = k_stop_200;         lens[0] = sizeof k_stop_200 - 1u;
  bodies[1] = k_out_of_range_200; lens[1] = sizeof k_out_of_range_200 - 1u;
  for (unsigned i = 0; i < 2; ++i) {
    pb_test_setup();
    link_fake_reset(); link_fake_set_state(LINK_UP);
    link_fake_queue_response(bodies[i], lens[i]);
    net_begin(); exec_begin();
    pb_net_passes(14, 100);
    exec_pending();
    TEST_ASSERT_TRUE(cart_parked());
  }
}
```

and in `test/test_contra/test_contra.cpp` (`pb_latch_contra()` is task 19's fixture helper in `harness.h`, which drives a real latching dose — there is no `safety_contra_set_()` and there must not be one):

```c
static void test_boot_self_home_runs_under_both_latches(void) {
  safety_dry_set(true);
  pb_latch_contra();
  TEST_ASSERT_TRUE(safety_contra());
  exec_begin();
  pb_advance(PB_BOOT_HOME_MS + 1);
  exec_pending();
  TEST_ASSERT_TRUE(cart_parked());        /* parking is MORE wanted after a latch, not less */
  TEST_ASSERT_TRUE(safety_contra());      /* and homing clears nothing */
}

/* §9's fourteenth contra case, owed by task 19 and written HERE because it asserts the latch's
   three WIRE surfaces and report_build() does not exist until task 22. status and the LCD
   reach a human standing at the bench; err=contra, ch207=1 and float=0 are the only channel
   by which the backend and the phone ever learn about the latch at all. */
static void test_latch_reports_err_contra_and_ch207_and_float_zero(void) {
  sensors_begin();
  sim_set_float(true);                    /* the tank SAMPLES fine; the latch outranks it */
  pb_latch_contra();
  report_clear_ack();                     /* no ack, so err= falls through to the latch */
  report_stamp();
  char b[PB_BODY_CAP];
  TEST_ASSERT_TRUE(report_build(b, sizeof b) > 0);
  TEST_ASSERT_NOT_NULL_MESSAGE(strstr(b, " err=contra"), b);
  TEST_ASSERT_NOT_NULL_MESSAGE(strstr(b, " ch207=1"),    b);
  TEST_ASSERT_NOT_NULL_MESSAGE(strstr(b, " float=0"),    b);
}
```

12. - [ ] Run all three suites:

```
pio test -e native -f test_cart -f test_contra -f test_net
```

Expected: three suites, `0 Failures` each.

13. - [ ] Wire it into `src/main.cpp`. Four edits, and the last is the largest thing this task does to that file.

    **(a0) The include block gains four headers.** Task 12 wrote it against a tree with no cart, no seam 2, no report FSM and no exec, and every one of those is named below. Add, in the same alphabetical block:

```c
   #include "cart.h"     /* cart_begin/pos_known/pos/parked/busy */
   #include "exec.h"     /* exec_begin/exec_pending/exec_last_cmd_id/exec_last_cmd_text */
   #include "link.h"     /* link_state/link_rssi/link_ip */
   #include "netfsm.h"   /* net_disable/net_begin/net_poll/net_last_status/net_next_s */
```

    **(a) `setup()` gains four lines.** `cart_begin()` goes immediately after `pulses_begin()`, in the first half — task 14 produced it with no caller, so without this the boot self-home at `PB_BOOT_HOME_MS` runs against whatever the BSS initialiser left. The other three go after the boot assertions, and the `net_disable()` line is what makes spec §2.5's "disables the network" real: task 12 set a flag that until now nothing read.

```c
  cart_begin();                                    /* beside pulses_begin(), in setup()'s first half */
```
```c
  /* spec §2.5: a failed watchdog, ADC or heap assertion disables the network and says why in
     status. main.cpp holds the verdict; netfsm.cpp holds the flag, because [env:native]
     filters main.cpp out and no host test could otherwise reach it. */
  if (main_net_disabled()) net_disable(main_boot_err());
  net_begin();
  exec_begin();
```

    **(b) `loop()` reaches its final shape.** Six responsibilities, plus `ui_fill_()` as this file's own helper:

```c
void loop(void) {
  safety_tick();               /* pump idle re-asserted (D6's direction repaired), then fed */
  cli_poll();                  /* one whole line; may block, but only through safety_wait_ms() */
  net_poll(safety_dosing());   /* ONE bounded link/socket step. The flag is passed IN: netfsm.cpp
                                  may not include safety.h (§9), so the caller supplies it. */
  exec_pending();              /* at most one command; runs only when the socket is closed */
  pulses_leak_poll(safety_dosing());          /* the leak watch, EVERY pass: ch205's only
                                                 driver, and report_build() turns a non-zero
                                                 count into err=leak (task 22 step 12).
                                                 sensors_sweep() is NOT here - task 24's
                                                 NET_IDLE pass owns it, once per report cycle,
                                                 in the one pass with no AT command. */
  ui_fill_(&g_ui);
  ui_poll(&g_ui);              /* no-ops while dosing, while the cart moves, or after a modem pass */
}
```

    **(c) `ui_fill_()` is rewritten whole.** Task 12 left eleven of `ui_state_t`'s fields at zero, and this is the task where every one of their sources finally exists. Without this, rows 1 and 4-7 of the OLED read `pos ?`, `wifi -- 0 dBm`, `no link`, `next 0s` and `cmd -` for the whole 48-hour run, and spec §5's LCD state machine never gets an input:

```c
   static void ui_fill_(ui_state_t *s) {
     memset(s, 0, sizeof *s);
     strncpy(s->build, PB_BUILD_NAME, sizeof s->build - 1);
     strncpy(s->controller, PB_CONTROLLER, sizeof s->controller - 1);
     s->uptime_min   = hal_millis() / 60000u;      /* MINUTES: spec §5's bus rule */
     s->pump_on      = safety_dosing();            /* ui.cpp may not include safety.h itself */
     s->float_ok     = (hal_pin_read(PIN_HALL_FLOAT) == PB_LOW);
     s->screw_pulses = pulses_screw();
     s->flow_hz      = pulses_flow_rate();
     s->flow_total   = pulses_flow();
     s->dry          = safety_dry();
     s->contra       = safety_contra();
     s->pos_known    = cart_pos_known();
     s->pos          = cart_pos();
     s->parked       = cart_parked();
     s->link         = (uint8_t)(link_state() == LINK_UP ? 2 :
                                (link_state() == LINK_JOINING ? 1 : 0));
     s->rssi         = link_rssi();
     strncpy(s->ip, link_ip(), sizeof s->ip - 1);
     s->http_status  = net_last_status();          /* a 400/401 loop is otherwise invisible
                                                      to anyone not on the serial port */
     s->next_s       = net_next_s();
     s->cmd_id       = exec_last_cmd_id();
     s->cmd_text     = exec_last_cmd_text();
   #ifdef PB_SIM
     s->sim = true;
   #endif

     /* spec §5's LCD state selection, most-urgent first. Row 1 is human prose and is tested
        (task 10) never to equal a wire err= token. NOTE what this does NOT decide: the
        renderer itself overrides row 1 with `HTTP <n>` whenever http_status is a non-200
        (task 10 step 4, §4.2), and overrides row 0 with the contra banner and then the sim
        banner (task 19 step 7). A 400/401 loop is therefore visible on the panel whichever
        branch below happened to run, which is the point - it must not depend on this
        function choosing the right prose. */
     static char detail[17];
     if (s->contra)        { s->lcd_state = "CONTRA LATCH"; s->lcd_detail = "float ok,no flow"; }
     else if (s->dry)      { s->lcd_state = "REFUSED";      s->lcd_detail = "dry latch set"; }
     else if (s->pump_on)  { snprintf(detail, sizeof detail, "PUMP o%u", (unsigned)s->pos);
                             s->lcd_state = detail;         s->lcd_detail = "dosing"; }
     else if (cart_busy()) { snprintf(detail, sizeof detail, "MOVE o%u", (unsigned)s->pos);
                             s->lcd_state = detail;         s->lcd_detail = "cart moving"; }
     else if (s->link != 2){ s->lcd_state = "WIFI?";        s->lcd_detail = "no link"; }
     else                  { s->lcd_state = "IDLE";
                             snprintf(detail, sizeof detail, "next %us", (unsigned)s->next_s);
                             s->lcd_detail = detail; }
   }
```

    `detail` is a file static rather than a local because `lcd_state` and `lcd_detail` are
    `const char *` into the caller's storage and `ui_poll()` reads them after `ui_fill_()`
    has returned. Only one of the two ever points at it in a given pass, which is why one
    buffer is enough; if a later change makes both point at it, give it a second.

14. - [ ] Add the two `status` lines in `src/cli.cpp`:

```c
  cli_printf_u32("parked=%lu\n", (uint32_t)(cart_parked() ? 1u : 0u));
  /* §2.12: the dosing loop blocks and net_poll() cannot run while it does, and enqueue() returns
     409 while the water command it would abort is still 'sent'. Say so, so nobody reaches for
     it in an emergency. The live aborts are the console `stop`, `dry on`, the float, the two
     flow rules, the plausibility ceiling, the cap and the watchdog. */
  hal_serial_write("note: a backend stop=1 CANNOT interrupt a running dose; type `stop`\n");
```

15. - [ ] Run everything and commit:

```
pio test -e native && pio run -e uno_r4_wifi && pio run -e uno_r4_wifi_sim && make check
git add include/exec.h src/exec.cpp src/main.cpp src/cli.cpp test/support/bodies.h test/test_cart/test_cart.cpp test/test_contra/test_contra.cpp
git commit -m "loop() reaches its final shape, and the panels finally have something to paint

net_poll() and exec_pending() join the loop. net_poll() takes the dosing flag as a PARAMETER
rather than calling safety_dosing(): netfsm.cpp is grepped for safety.h and the two cannot
both be true - the same shape task 10 used for ui.cpp, recorded the same way.

ui_fill_() is rewritten whole. Eleven fields of ui_state_t had no source until this task:
pos, pos_known, parked, link, rssi, ip, http_status, next_s, cmd_id, cmd_text, and spec 5's
LCD state selection. Without them rows 1 and 4-7 of the OLED read pos ?, wifi -- 0 dBm,
no link, next 0s and cmd - for the whole run, and the LCD's MOVE/PUMP/WIFI?/REFUSED/
CONTRA LATCH machine never gets an input.

cart_begin() is called from setup(); it was produced in task 14 with no caller, so the boot
self-home ran against uninitialised bookkeeping. main_net_disabled() is forwarded into
net_disable(), which is the half of spec 2.5's 'disables the network and says why in status'
that did not previously exist anywhere.

exec.cpp zero-initialises its dose_req_t. The hang member is unconditional - a #if on it
would break spec 6's safety.o hash equality - so a bare declaration plus el >= PB_HANG_MS
would put a BACKEND water command into the loop that deliberately starves the watchdog. It
also prints the per-dose summary line, which spec 6 requires from EVERY path and which until
now only the console's two commands produced.

The park runs on every consumed exit, goto failures included, and under both latches. status
says in as many words that a backend stop=1 cannot interrupt a running dose — it is written
there so nobody reaches for it in an emergency.

exec.cpp carries the tree's SECOND dose_run( call site and cli.cpp's second exported entry
point (cli_print_dose_summary). Both are expected: task 30's grep is scoped to cli.cpp on
purpose, and the scope comment is what moves, never the invariant.

exec_last_cmd_id() and exec_last_cmd_text() are set inside ack(), the one function every
terminal path goes through, so spec 5's OLED row 7 has a source. exec_has_pending() is
deliberately NOT declared: nothing would have called it."
```

---

---

### Task 27: `link_wifi.cpp` — the real WiFiS3 driver behind seam 2

**Drop 3.**

**Files:**
- Create: `lib/Network/src/link_wifi.cpp`
- Test: **none on the host.** `lib/Network` is in `[env:native]`'s `lib_ignore` and this file is on spec §9's *"NOT tested on the host"* list. It is verified by `pio run -e uno_r4_wifi`, by `tools/check.sh`, by task 28's two on-device wall-clock tests, and on the bench by §13 step 7e.

**Interfaces:**

*Consumes:* all ten declarations of `include/link.h` (task 21); `WIFI_SSID`, `WIFI_PASS`, `HOST_NAME`, `HTTP_PORT` from `include/secrets.h`; `PB_NET_STEP_MS` (task 2).

*Produces:* no new declarations — this implements seam 2 for the board, and is the only file in the tree that names `WiFiS3`.

---

1. - [ ] Write the file's fixed parts: exactly one `WiFiClient` for the life of the program, and `link_begin()`'s four calls in order.

```c
/* link_wifi.cpp — the WiFiS3 driver behind seam 2, and nothing else. The ONLY file that names
   WiFiS3. DEVICE ONLY: lib/Network is in [env:native]'s lib_ignore and this file has no host
   coverage at all (spec §9). Zero hits for the safety header, the dosing entry point or the
   pump write are a make check invariant over this directory. */
#include <WiFiS3.h>
#include "link.h"
#include "config.h"
#include "secrets.h"

/* Exactly ONE WiFiClient for the life of the program: its ctor does
   `new FifoBuffer<uint8_t,1024>` on every construction (WiFiClient.cpp:6-8), so a file static
   heap-allocates that 1 KB once instead of once per report. */
static WiFiClient g_client;
static uint16_t g_desyncs;
static char g_ip[16];

void link_begin(uint32_t step_ms) {
  modem.timeout((int)step_ms);          /* MODEM_TIMEOUT defaults to 10000 (Modem.h:12) */
  modem.begin();                        /* EXPLICITLY, ONCE: drains the one-time _SOFTRESETWIFI
                                           into setup(), which is what makes link_join() 2 ATs */
  WiFi.setTimeout(0);                   /* CWifi::begin()'s poll loop body never runs
                                           (WiFi.cpp:61,563) */
  g_client.setConnectionTimeout((int)PB_NET_STEP_MS);
                                        /* selects _CLIENTCONNECT and appends the value
                                           (WiFiClient.cpp:57-61); _connectionTimeout defaults
                                           to 0, so without this nothing bounds the ESP32's own
                                           connect inside our 1200 ms window */
}
```

2. - [ ] Write `link_reset()` — the one line the 48-hour run depends on — and the join/state accessors:

```c
void link_reset(void) {
  modem.end();
  modem.beginned = false;   /* Modem.cpp:45-48 never does this, and begin() is guarded by it
                               (:35). Without this line link_reset() closes Serial2 and declines
                               to reopen it: every subsequent AT command writes into a closed
                               UART, buf_read times out at PB_NET_STEP_MS forever, ch206 climbs,
                               and the board silently stops reporting for the rest of the run. */
  modem.begin();            /* re-issues _SOFTRESETWIFI, which is the point of the reset */
  g_desyncs++;
}
uint16_t link_desyncs(void) { return g_desyncs; }

void link_join(void) { (void)WiFi.begin(WIFI_SSID, WIFI_PASS); }   /* 2 ATs; does NOT spin */

link_state_t link_state(void) {
  int s = WiFi.status();                                            /* ONE bounded query */
  if (s == WL_CONNECTED) return LINK_UP;
  if (s == WL_IDLE_STATUS || s == WL_SCAN_COMPLETED) return LINK_JOINING;
  return LINK_DOWN;
}
int8_t link_rssi(void) { return (int8_t)WiFi.RSSI(); }

const char *link_ip(void) {
  IPAddress a = WiFi.localIP();
  snprintf(g_ip, sizeof g_ip, "%u.%u.%u.%u",
           (unsigned)a[0], (unsigned)a[1], (unsigned)a[2], (unsigned)a[3]);
  return g_ip;
}
```

3. - [ ] Write the four socket primitives. `sock_read()` is one call and nothing else:

```c
bool sock_open(void) { return g_client.connect(HOST_NAME, (uint16_t)HTTP_PORT) == 1; }

int sock_write(const uint8_t *b, size_t n) {
  size_t w = g_client.write(b, n);      /* write_nowait(_CLIENTSEND) + one passthrough */
  return (int)w;
}

/* client.read(buf, cap) and NOTHING else — one _CLIENTRECEIVE per RECV pass
   (WiFiClient.cpp:145-182). available() would add an _AVAILABLE; connected() costs TWO because
   it calls available() itself (:224-238), and the FSM never calls it: PB_NET_DEADLINE_MS is the
   closed-socket detector instead (§3 change 3). The ping helper is never called anywhere: it
   resets the modem timeout to 10000 ms (WiFi.cpp:585-593) and would undo the whole margin.
   make check greps its name to zero over lib/, comments included, which is why it is not
   spelled here. */
int sock_read(uint8_t *b, size_t cap) { return g_client.read(b, (size_t)cap); }

void sock_close(void) { g_client.stop(); }   /* sets _sock = -1 (:217): idempotent by design */
```

4. - [ ] Build the two board environments:

```
pio run -e uno_r4_wifi -e uno_r4_wifi_bringup
```

Expected: both link. This is the first task since 24 at which `-e uno_r4_wifi` links at all — before it, `netfsm.cpp`'s `sock_open` and friends had no implementation on the board and the sim env was the device-compile gate.

5. - [ ] Run the greps and the host suite (which is unaffected — `lib/Network` is ignored on native):

```
make check && pio test -e native
```

Expected: `make check` reports 0 hits for `WiFi\.ping` (that grep exists from task 13) and `Arduino\.h`/`String` still confined to their allowed paths. The `safety\.h|dose_run|hal_pump_write` grep over `lib/Network/` is task 30 step 1's and does not exist yet — check it by hand with `grep -rEc 'safety\.h|dose_run|hal_pump_write' lib/Network`, expecting 0. The host suite is unchanged.

6. - [ ] Add the connect-form line to `cli_print_status()` in `src/cli.cpp`, replacing task 25's hard-coded string with what the driver actually sent:

```c
  hal_serial_write("connect_form=_CLIENTCONNECT to HOST_NAME as a NAME"
                   " (setConnectionTimeout != 0; unit unverified off-bench)\n");
```

The driver proves only the command selection. `_CLIENTCONNECT` and `_CLIENTCONNECTNAME` are distinct ESP32-side entry points (`WiFiCommands.h:52-53`) whose firmware is not in this package, so **neither the unit of the timeout nor whether `_CLIENTCONNECT` still resolves a hostname is checkable here**.

7. - [ ] Flash the bringup binary and read `status` — this is the first half of §3 change 4's two bring-up questions, and it is the only thing that can answer them:

```
pio run -e uno_r4_wifi_bringup -t upload && pio device monitor -b 115200
```

Then type `status`. Confirm it prints, in these exact key spellings — they are the ones tasks 24 step 17, 25 step 14 and step 6 above actually emit, and every `status` line in this program is a bare `key=value`:

```
link=2
rssi=-52 dBm
ip=192.168.1.42
http_last=200
reports_ok=1
reports_failed=0
modem_ran=0
modem_timeout_ms=1200
conn_timeout_ms=1200
connect_form=_CLIENTCONNECT to HOST_NAME as a NAME (setConnectionTimeout != 0; unit unverified off-bench)
desyncs=0
```

(the numbers will differ; the keys must not). **Question (a):** the board reached the backend by name, with the timeout set. If it did not, the written-down fallback is `hostByName()` once at join time with the address cached and `connect(IPAddress)` thereafter — not a rediscovery on the bench. **Question (b)** — timing a connect to a blackholed address and checking it returns near `PB_NET_STEP_MS` — is task 28's `test_sock_open_from_a_stale_socket_completes_within_the_wdt_window`.

8. - [ ] Commit, recording what is and is not proven:

```
git add lib/Network/src/link_wifi.cpp src/cli.cpp
git commit -m "link_wifi.cpp: the WiFiS3 driver behind seam 2

link_begin() pays modem.begin() once in setup(), so link_join() is 2 AT commands and not 3;
sets WiFi.setTimeout(0) so CWifi::begin()'s poll loop body never runs; and calls
setConnectionTimeout(PB_NET_STEP_MS) so the ESP32 bounds its own connect inside our window.

link_reset() writes modem.beginned = false between end() and begin(). ModemClass::end() never
clears it and begin() is guarded by it, so without that line link_reset() closes Serial2 and
declines to reopen it, and the board silently stops reporting while status keeps printing
wifi UP. That is the single line the 48-hour run depends on.

One WiFiClient for the life of the program, so its 1 KB FifoBuffer is heap-allocated once.
sock_read() is client.read() and nothing else. The modem's ping helper is never called: it
resets the modem timeout to 10000 ms and would undo the whole margin. The comment in the
file says it in those words rather than by name, because make check scans comments too.

NOT PROVEN HERE, and stated so nobody reads a green build as coverage: this file has no host
test at all. Whether setConnectionTimeout's unit is milliseconds, and whether _CLIENTCONNECT
still resolves a hostname, are ESP32-side questions whose firmware is not in this package.
status prints which form and which value were sent; bring-up asks (a) connect to HOST_NAME as
a name, then (b) time a connect to a blackholed address. If (a) fails, the fallback is
hostByName() once at join time with the address cached. Task 28's two device tests and §13's
7e run are what actually prove this file."
```

---

---

### Task 28: The on-device test environment and the six suites that must run on real silicon

**Drop 3.**

**Files:**
- Create: `test/test_device/test_device.cpp`
- Modify: `platformio.ini` (`[env:uno_r4_wifi_test]`'s `build_src_filter`), `test/support/harness.h` (the device fixture arm)
- Test: `pio test -e uno_r4_wifi_test`

**Interfaces:**
- Consumes: everything. `[env:uno_r4_wifi_test]` compiles `hal_uno.cpp` **and** `hal_sim.cpp`, so every `hal_*` **body** in `hal_sim.cpp` must be `#if PB_SIM`-gated (that env does not define `PB_SIM`) or the linker sees two definitions of `hal_millis`. **`PB_SIM` gates the HAL bodies and nothing else**; the `sim ...` console family is gated on its own `PB_SIM_CLI` (task 1, task 29), precisely so that task 29's absence case can compile the console away without taking the HAL with it. `link.h`'s ten primitives come from the real `lib/Network/src/link_wifi.cpp` (task 27), not from `link_fake.cpp`, which this env filters out.
- Produces: a device-only suite carrying the four checks spec §9 says cannot be simulated, plus the device arm of `harness.h` under which `test_report` and `test_cart` also run on the board.

**Spec sections to read in full before starting:** §9's "On-device runs" and "NOT tested on the host"; §10's `[env:uno_r4_wifi_test]` notes.

**Two deviations from spec §10, both to record in the commit message.**

- **`[env:uno_r4_wifi_test]`'s filter must also drop `main.cpp`.** §10 prints `build_src_filter = +<*> -<link_fake.cpp>`, which keeps `src/main.cpp` — and with `test_build_src = yes` that puts a second `setup()`/`loop()` pair in the link alongside the test runner's. The device suites test `report`, `cart` and the link driver, not the sketch's setup order, so the filter becomes `+<*> -<link_fake.cpp> -<main.cpp>`.
- **If the two HALs will not co-link, fall back to two device test envs, one per HAL, and say which shipped in `status`** — §10 names that fallback explicitly. Do not paper over a duplicate-symbol error by weakening `hal_sim.cpp`'s gate in a way that changes what the sim binary compiles.

---

1. - [ ] Fix the test env's filter and confirm the env builds a test at all:

   ```ini
   [env:uno_r4_wifi_test]                             ; spec §9's on-device suites. Never left running.
   extends = env:uno_r4_wifi_bringup
   test_framework = unity
   test_build_src = yes                               ; or src/report.cpp never links into a test
   ; main.cpp is dropped as well as link_fake.cpp: with test_build_src it would put a second
   ; setup()/loop() pair in the link alongside the runner's. Spec §10 prints only the
   ; link_fake exclusion; this is the correction.
   ; sim_console.cpp too, from task 29 onward: build_src_filter REPLACES the parent's rather
   ; than extending it, so task 29's exclusion in [env:uno_r4_wifi] does NOT reach here, and
   ; this env would otherwise compile the device-only console shim with PB_SIM undefined.
   ; [env:uno_r4_wifi_sim] is the ONE env that must keep the file.
   build_src_filter = +<*> -<link_fake.cpp> -<main.cpp> -<sim_console.cpp>
   ```

2. - [ ] Add the device arm to `harness.h`. On the board there are no fault injectors and no settable clock: `pb_advance()` is a fed real-time wait, and any host case that needs an injector must be `#ifdef PB_SIM`-guarded so it compiles out here.

   **This is an ADDITION, not a rewrite. Every helper the file already carries stays, with its existing body**: `pb_count` and `pb_expect_no_feed_between` (task 3), `pb_latch_contra` (task 19) and `pb_net_passes` (task 24). **`pb_test_setup()` keeps task 3's body in the host arm — `hal_begin(); hal_boot_pump_off(); (void)hal_wdt_start();` and all** — because without `hal_wdt_start()` every `hal_wdt_granted()` returns 0, `hal_wdt_alive()` is false, and the dose ladder's `!hal_wdt_alive()` guard refuses every dose in `test_dose` and `test_contra`. Only the three helpers that genuinely differ between host and board — `pb_test_setup`, `pb_test_teardown`, `pb_advance` — get a device arm at all. Four of the others read `sim_events()` or the fake link, so they live inside the `#ifdef PB_SIM` block too rather than becoming declarations with no definition; the device suite does not use them.

   ```c
   /* test/support/harness.h -- the Unity fixture. A HEADER (spec §10).
      Host arm: hal_sim's injectors and a driven clock. Device arm ([env:uno_r4_wifi_test]):
      real hardware, real time, no injectors -- a case needing one is #ifdef PB_SIM'd out.
      config.h and safety.h are NOT optional: pb_latch_contra() names dose_req_t, dose_run(),
      safety_contra(), PB_BOOT_GAP_MS, PB_PRIME_MS_DEFAULT and PB_STALL_MS_DEFAULT, and
      nothing else this header includes reaches any of them. */
   #pragma once
   #include "config.h"
   #include "hal.h"
   #include "safety.h"
   #include <unity.h>

   #ifdef PB_SIM
   #  include "sim.h"
   #  include "netfsm.h"
   static inline void pb_test_setup(void) {
     sim_reset(false);          /* a cold boot: clock at 0, .noinit cleared */
     hal_begin();
     hal_boot_pump_off();
     (void)hal_wdt_start();     /* KEEP THIS: without it hal_wdt_alive() is false and the
                                   dose ladder refuses every dose in test_dose/test_contra */
     sim_events_clear();
   }
   static inline void pb_test_teardown(void)  { sim_events_clear(); }
   static inline void pb_advance(uint32_t ms) { sim_advance(ms); }

   /* task 3: counts call-trace events of one kind. Used five times by test_sensors. */
   static inline uint32_t pb_count(sim_ev_kind_t kind) {
     const sim_ev_t *ev; size_t n = sim_events(&ev); uint32_t hits = 0;
     for (size_t i = 0; i < n; ++i) if (ev[i].kind == kind) hits++;
     return hits;
   }

   /* task 3: the ONE deliberately unfed window in the program is hal_wdt_alive()'s probe.
      Strictly inside: the two feeds that BRACKET the probe are legal. A static inline
      DEFINITION, not a declaration - the definition reads sim_events() and there is nowhere
      else it could live without every suite that includes this header owning a copy. */
   static inline void pb_expect_no_feed_between(uint32_t from_ms, uint32_t to_ms) {
     const sim_ev_t *ev; size_t n = sim_events(&ev); uint32_t hits = 0;
     for (size_t i = 0; i < n; ++i)
       if (ev[i].kind == SIM_EV_WDT_FEED && ev[i].at_ms > from_ms && ev[i].at_ms < to_ms) hits++;
     TEST_ASSERT_EQUAL_UINT32(0u, hits);
   }

   /* task 19: there is no safety_contra_set_(). The latch is settable in exactly one place,
      so the fixture EARNS it by driving a real latching dose. */
   static inline void pb_latch_contra(void) {
     pb_advance(PB_BOOT_GAP_MS + 1u);
     sim_set_float(true);
     sim_set_flow_ml_s(0);                  /* float OK, no flow: the contradiction */
     dose_req_t q = {0};
     q.by_time    = true;
     q.cap_ms     = PB_PRIME_MS_DEFAULT + PB_STALL_MS_DEFAULT + 1000u;
     q.long_prime = false;                  /* a console prime is EXEMPT (§2.7) */
     (void)dose_run(&q);
     TEST_ASSERT_TRUE_MESSAGE(safety_contra(), "pb_latch_contra did not latch");
   }

   /* task 24: drive n whole network passes, advancing the fake clock ms between them.
      THE ONE SPELLING of "a pass" -- task 24's cases, task 25's retries and task 26's ack
      cycles all use it. */
   static inline void pb_net_passes(uint16_t n, uint32_t ms) {
     for (uint16_t i = 0; i < n; ++i) {
       link_fake_pass_begin();
       net_poll(false);         /* not dosing: the dosing loop blocks, so no pass overlaps one */
       if (ms) pb_advance(ms);
     }
   }

   #else   /* ---- the device arm: real hardware, real time, no injectors ---- */
   static inline void pb_test_setup(void)     { hal_begin(); hal_boot_pump_off(); }
   static inline void pb_test_teardown(void)  {}
   static inline void pb_advance(uint32_t ms) { safety_wait_ms(ms); }   /* fed, on real time */
   #endif
   ```

   **Both bodies above are printed in full on purpose.** An earlier draft of this step printed them as empty `{ /* task 19 step 1, verbatim */ }` stubs with a pointer to the task that wrote them, and this step is emphatic that nothing already in the file may be dropped — an implementer pasting the printed block gets a silently no-op `pb_net_passes()`, which every ack-cycle and retry case in tasks 24 to 26 depends on. If the two bodies here ever disagree with tasks 19 and 24, those tasks are right and this one is stale.

   **There is no `pb_begin_fake_dose` / `pb_end_fake_dose` pair** (and there never was one in the tree): task 7's two I2C-recovery cases call `safety_set_dosing()` — task 5's declared seam, spelled exactly that way — directly. A wrapper here would be a second spelling of the same thing, and an earlier draft of this step wrote it as `safety_dosing_set_()`, a symbol that exists nowhere.

3. - [ ] Write the device suite. Note what is *not* here: no Arduino header and no blocking wait — spec §9's greps scan `test/` too, and everything this file needs is behind seam 1 or seam 2.

   ```cpp
   /* test/test_device/test_device.cpp -- DEVICE ONLY. The four checks spec §9 says cannot
      be simulated. link_fake cannot see WiFiClient's internal modem writes, so the AT budget
      of spec §3 is only half-proved by the host suite; these two wall-clock cases are the
      other half.

      test_a_recv_pass_against_a_slow_responder_completes_within_the_wdt_window needs a
      deliberately slow responder on HOST_NAME:HTTP_PORT. The spec names no fixture; run one
      on the laptop before this suite -- e.g. a listener that accepts, waits three seconds and
      then answers -- and record which one you used in the commit message. */
   #include "../support/harness.h"
   #include "config.h"
   #include "hal.h"
   #include "link.h"
   #include <unity.h>

   void setUp(void)    { pb_test_setup(); }
   void tearDown(void) { pb_test_teardown(); }

   static void test_wifi_begin_returns_within_two_seconds(void) {
     /* WiFi.setTimeout(0) makes CWifi::begin()'s poll loop body never run. A platform bump
        that breaks that trick must be loud here rather than a mysterious ten-second stall. */
     uint32_t t0 = hal_millis();
     link_begin(PB_NET_STEP_MS);
     link_join();
     TEST_ASSERT_LESS_THAN_UINT32(2000u, hal_millis() - t0);
   }

   static void test_sock_open_from_a_stale_socket_completes_within_the_wdt_window(void) {
     while (link_state() != LINK_UP && hal_millis() < 30000u) { safety_tick(); }
     TEST_ASSERT_EQUAL_INT(LINK_UP, link_state());
     TEST_ASSERT_TRUE(sock_open());          /* leave it open and abandon it on purpose */
     uint32_t t0 = hal_millis();
     sock_close();
     bool again = sock_open();
     uint32_t took = hal_millis() - t0;
     sock_close();
     TEST_ASSERT_TRUE(again);
     TEST_ASSERT_LESS_THAN_UINT32(PB_WDT_GRANTED_MS, took);
   }

   static void test_a_recv_pass_against_a_slow_responder_completes_within_the_wdt_window(void) {
     TEST_ASSERT_TRUE(sock_open());
     static const char req[] =
       "GET / HTTP/1.1\r\nHost: slow\r\nConnection: close\r\n\r\n";
     TEST_ASSERT_TRUE(sock_write((const uint8_t *)req, sizeof req - 1) > 0);
     uint8_t rx[64];
     uint32_t t0 = hal_millis();
     int n = 0;
     while (n == 0 && hal_millis() - t0 < PB_NET_DEADLINE_MS) {
       safety_tick();
       n = sock_read(rx, sizeof rx);
       TEST_ASSERT_LESS_THAN_UINT32(PB_WDT_GRANTED_MS, hal_millis() - t0);
     }
     sock_close();
   }

   static void test_wdt_alive_returns_true_on_real_silicon(void) {
     /* The sim's counter is a fake by construction, so this is the only place the probe of
        spec §2.5 meets the actual down-counter. Once started the dog cannot be stopped:
        this case runs LAST, and the board resets a few seconds after the summary prints.
        That reset is expected. */
     TEST_ASSERT_TRUE(hal_wdt_start());
     TEST_ASSERT_EQUAL_UINT32(PB_WDT_GRANTED_MS, hal_wdt_granted());
     TEST_ASSERT_TRUE(hal_wdt_alive());
     TEST_ASSERT_GREATER_OR_EQUAL_UINT32(PB_WDT_PROBE_MIN_COUNTS, hal_wdt_last_delta());
   }

   void setup(void) {
     while (hal_millis() < 2000u) { }        /* let the USB CDC bridge come up */
     UNITY_BEGIN();
     RUN_TEST(test_wifi_begin_returns_within_two_seconds);
     RUN_TEST(test_sock_open_from_a_stale_socket_completes_within_the_wdt_window);
     RUN_TEST(test_a_recv_pass_against_a_slow_responder_completes_within_the_wdt_window);
     RUN_TEST(test_wdt_alive_returns_true_on_real_silicon);
     UNITY_END();
   }

   void loop(void) {}
   ```

4. - [ ] Run the device suite on a connected board and watch it fail first for the reason you expect — a duplicate `hal_millis` if `hal_sim.cpp`'s bodies are not `#if PB_SIM`-gated, or a link failure if the AP is not reachable:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e uno_r4_wifi_test -f test_device
   ```
   A `multiple definition of 'hal_millis'` here means task 3's gate is missing: fix it in `hal_sim.cpp` (bodies of `hal_*` under `#if PB_SIM`, injectors outside), or take §10's fallback of two device test envs and say which shipped in `status`.

5. - [ ] Run the two host suites that must also run on real silicon. **This is the run that catches `int` width in the `k=v` formatting, unaligned struct access and `millis()` wrap arithmetic** (spec §9), and it is owed once per protocol change:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e uno_r4_wifi_test -f test_report -f test_cart
   ```
   Any case in those two suites that needs a fault injector must be `#ifdef PB_SIM`-guarded so it compiles out here. List the cases that actually ran on the device in the commit message — spec §9's split (`test_dose`, `test_contra`, `test_net`, `test_sensors`, `test_cli` stay host-only) has to stay accurate as the suites grow.

6. - [ ] Run the one test no suite can express, by hand. **This is the test that catches a `link_reset()` which does not clear `beginned`** — the single line the 48-hour run depends on, because `ModemClass::end()` never clears it and `begin()` is guarded by it, so the obvious `end(); begin();` closes Serial2 and then declines to reopen it.

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && make bringup && make monitor
   ```
   With the board reporting, pull the AP mid-connect (power the router down, or move the board out of range). Assert two things: (a) the board does **not** reset, and (b) after the AP returns, the next three exchanges still parse — `status` shows `reports_ok` climbing and `desyncs` stable. Record the result, and the desync count you saw, in the commit message.

7. - [ ] Commit.

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && git add platformio.ini test/support/harness.h test/test_device/test_device.cpp && git commit -m "The on-device suite: the four checks that cannot be simulated

   [env:uno_r4_wifi_test]'s filter also drops main.cpp, which spec §10 does not: with
   test_build_src = yes it would put a second setup()/loop() pair in the link beside the
   runner's. It drops sim_console.cpp too, because build_src_filter replaces the parent's
   rather than extending it, so task 29's exclusion in [env:uno_r4_wifi] does not reach here.
   harness.h gains a device arm where pb_advance() is a fed real-time wait and there are no
   injectors, so any injector-driven case in test_report or test_cart is #ifdef PB_SIM'd out.
   Nothing the file already carried is dropped: pb_test_setup() keeps hal_wdt_start(), without
   which hal_wdt_alive() is false and the dose ladder refuses every dose.

   The two wall-clock cases are the half of spec §3's AT budget that link_fake structurally
   cannot see: a RECV pass that quietly grew an available() or a connected() would pass
   every host test and blow the 5592 ms grant on the bench. The slow-responder case needs a
   fixture the spec does not name; used: TODO_FILL_IN.

   test_wdt_alive_returns_true_on_real_silicon runs last because the dog cannot be stopped
   once started - the board resets a few seconds after the summary, and that is not a
   failure. Pulled the AP mid-connect by hand: no reset, next three exchanges parsed,
   desyncs TODO_FILL_IN."
   ```

---

---

### Task 29: The sim binary — its fault-injection console, and the loudness that keeps it off a live rig

**Drop 3.**

**Files:**
- Create: `src/sim_console.h` (the two-line internal seam; **step 4 prints it in full**) and `src/sim_console.cpp` (the device-only console shim; **step 4 prints it in full**). `sim_console.cpp` is compiled **only** by `[env:uno_r4_wifi_sim]`: add `-<sim_console.cpp>` to `build_src_filter` in `[env:uno_r4_wifi]` (which the bringup and test envs inherit) and in `[env:native]`, so that the one file carrying an Arduino header outside `hal_uno.cpp` reaches exactly the one binary that needs it.
- Modify: `src/cli.cpp` (the `sim ...` family under `#if PB_SIM_CLI`, `sim resp` included — it calls task 21's already-written `link_fake_queue_response()`, so **`src/link_fake.cpp` is NOT modified by this task and is not in its `git add`**), `src/hal_sim.cpp` (`hal_serial_*` reach the shim on the device), `src/ui.cpp` (confirm the banner rows), `src/main.cpp` (the LED double-blink and the `SIM ` line prefix), `platformio.ini` (`-<sim_console.cpp>` in `[env:native]`'s `build_src_filter`), `Makefile` (`firmware-SIM.bin`), `tools/check.sh` (the sim env's file-set check)
- Test: `test/test_cli/test_cli.cpp`

**Interfaces:**
- Consumes: the whole `sim.h` injector API (tasks 3, 6, 7, 14) — `sim_reset`, `sim_advance`, `sim_set_float`, `sim_set_flow_ml_s`, `sim_flow_storm`, `sim_set_i2c_fail`, `sim_set_mux_stuck`, `sim_set_stall`, `sim_set_leak`, `sim_wdt_stop`, `sim_wdt_rate_hz`, `sim_noinit_clobber`, `sim_set_channel`; `link_fake_queue_response()` from `include/sim.h` (task 21 appends the `link_fake_*` control surface there — there is no `include/link_fake.h`); `PIN_LED` from `include/pins.h` (task 2 step 5).
- Produces: the thirteen `sim` console tokens of spec §8, the four loudness mechanisms, and one more `tools/check.sh` invariant.

**Spec sections to read in full before starting:** §8 in full; §16.5.5.

**The structural question the spec leaves open, SETTLED HERE so nobody has to settle it at the keyboard.** In the sim env `hal_uno.cpp` is excluded, so `hal_sim.cpp` is the only HAL — and on a real board it must therefore drive the real console, which needs the Arduino header, which spec §9 allows only in `hal_uno.cpp`, `lib/Network` and `lib/Screen`. **The answer is the device-only shim, not an `#ifndef PB_NATIVE` include inside `hal_sim.cpp`.** Add `src/sim_console.cpp` — device-only, excluded from `[env:native]`, `[env:uno_r4_wifi]` (and so from bringup) and `[env:uno_r4_wifi_test]` by `build_src_filter`, and **added to task 13 step 4's `Arduino\.h` exclusion list by step 6 of this task, in this task's own commit** (task 30 has no `Arduino.h` step; that invariant has existed since task 13) — carrying exactly three functions (`sim_console_begin`, `sim_console_read`, `sim_console_write`) that `hal_sim.cpp` calls through a two-line internal header. It is the cheaper answer on both counts that matter: the `Arduino\.h` invariant stays a *file* rule rather than becoming a file-plus-guard rule that a grep cannot express, and `hal_sim.cpp` stays **byte-identical** between the sim binary and the host suites, which is the whole reason one fake serves both. Record the decision in this task's commit message.

**One naming correction to carry into step 4:** the LED pin is `PIN_LED` from `include/pins.h`, which **task 2 step 5 defines as 13**, the board's own `LED_BUILTIN`, with a comment saying it is not from the spec. Do not write `LED_BUILTIN_PIN`, which is in no header, and do not write `LED_BUILTIN`, which is an Arduino-header name that `main.cpp` — the file that does the blinking — may not have (spec §9).

---

1. - [ ] Write the failing tests. The absence case is compiled twice, once with `-UPB_SIM`, exactly as `test_cli`'s bench-vs-bringup case is.

   ```cpp
   static void test_every_sim_command_is_parsed_and_dispatched(void) {
     pb_test_setup();
     TEST_ASSERT_TRUE(cli_dispatch("sim float 0"));
     TEST_ASSERT_TRUE(cli_dispatch("sim float 1"));
     TEST_ASSERT_TRUE(cli_dispatch("sim flow 30"));
     TEST_ASSERT_TRUE(cli_dispatch("sim flow storm"));
     TEST_ASSERT_TRUE(cli_dispatch("sim i2c fail"));
     TEST_ASSERT_TRUE(cli_dispatch("sim i2c ok"));
     TEST_ASSERT_TRUE(cli_dispatch("sim mux stuck"));
     TEST_ASSERT_TRUE(cli_dispatch("sim stall on"));
     TEST_ASSERT_TRUE(cli_dispatch("sim stall off"));
     TEST_ASSERT_TRUE(cli_dispatch("sim leak on"));
     TEST_ASSERT_TRUE(cli_dispatch("sim wdt stop"));
     TEST_ASSERT_TRUE(cli_dispatch("sim wdt slow 100"));
     TEST_ASSERT_TRUE(cli_dispatch("sim noinit clobber"));
     TEST_ASSERT_TRUE(cli_dispatch("sim ch 2 8123"));
     TEST_ASSERT_TRUE(cli_dispatch("sim resp \"next=60\\ncmd=7 water=3 ml=120 cap_s=11\\n\""));
     TEST_ASSERT_TRUE(cli_dispatch("sim reset warm"));
     TEST_ASSERT_TRUE(cli_dispatch("sim reset cold"));
     TEST_ASSERT_FALSE(cli_dispatch("sim ch 9 1"));       /* channel out of 0..5 */
     TEST_ASSERT_FALSE(cli_dispatch("sim nonsense"));
   }

   static void test_sim_commands_are_absent_from_the_bench_and_bringup_builds(void) {
   #ifdef PB_SIM_CLI
     TEST_ASSERT_TRUE(cli_dispatch("sim float 0"));
   #else
     TEST_ASSERT_FALSE(cli_dispatch("sim float 0"));      /* not a command at all */
   #endif
   }

   static void test_the_sim_banner_holds_row_zero_on_both_screens(void) {
     ui_state_t s = base_state();
     s.sim = true;
     char oled[8][17], lcd[2][17];
     ui_render(&s, oled);
     ui_render_lcd(&s, lcd);
     /* EXACT, not a substring. One 16-character string in both renderers (task 10): an
        earlier draft had "*** SIM: D6 NOT" on the OLED and "*** SIM: NO D6 *" on the LCD,
        and a substring check for "SIM" would have shipped that truncation. */
     TEST_ASSERT_EQUAL_STRING("*** SIM NO D6 **", oled[0]);
     TEST_ASSERT_EQUAL_STRING("*** SIM NO D6 **", lcd[0]);
     TEST_ASSERT_NULL(strstr(oled[0], "PB "));            /* the banner WINS row 0 */
   }
   ```

2. - [ ] Run and watch the first fail:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cli
   ```
   expected: `test_every_sim_command_is_parsed_and_dispatched ... FAIL` with `Expected TRUE Was FALSE` — `sim` is not yet a command.

3. - [ ] Add the `sim` family to `cli.cpp` under **`#if PB_SIM_CLI`** — its own macro, not `PB_SIM`. `PB_SIM` gates every `hal_*` body in `hal_sim.cpp` (task 28), so compiling the console away with `-UPB_SIM` would leave the host suite linking against no HAL at all; `[env:native_nosimcli]` undefines `PB_SIM_CLI` alone. Each token maps to a bring-up step or to a finding; there is no extra state and no interpreter.

   ```cpp
   #if PB_SIM_CLI
   #include "sim.h"           /* the injectors AND link_fake_queue_response(): task 21 put the
                                 link_fake_* control surface at the end of this header, and
                                 there is no include/link_fake.h in this tree */

   /* spec §8. Each of these maps to a bring-up step or a finding:
      float 0|1        -> 5a, and dropping it mid-dose is 5b
      flow <ml_s>      -> the prime abort at 0; a mid-dose stop is the stall abort (7b)
      flow storm       -> DOSE_REFUSED_NOISE / DOSE_ABORT_NOISE
      i2c fail|ok      -> position unknown, pump refused, pos=unknown
      mux stuck        -> the canary, err=stuck
      stall on|off     -> goto aborts, position lost
      leak on          -> ch205 rises and err=leak
      wdt stop         -> hal_wdt_alive() false, every dose refused
      wdt slow <hz>    -> delta below PB_WDT_PROBE_MIN_COUNTS
      noinit clobber   -> the checksum fails and it reads as a cold boot
      ch <0-5> <raw>   -> plant a raw count
      resp "<body>"    -> the ack offbeat, on a desk
      reset warm|cold  -> re-enter setup() with .noinit kept or cleared */
   static bool cmd_sim_(const char *a) {
     uint32_t v = 0, w = 0;
     if (strcmp(a, "float 0") == 0)  { sim_set_float(false);   return true; }
     if (strcmp(a, "float 1") == 0)  { sim_set_float(true);    return true; }
     if (strcmp(a, "flow storm") == 0) { sim_flow_storm(10000u); return true; }
     if (strncmp(a, "flow ", 5) == 0 && parse_u32_(a + 5, &v) && v <= 65535u) {
       sim_set_flow_ml_s((uint16_t)v); return true;
     }
     if (strcmp(a, "i2c fail") == 0) { sim_set_i2c_fail(true);  return true; }
     if (strcmp(a, "i2c ok") == 0)   { sim_set_i2c_fail(false); return true; }
     if (strcmp(a, "mux stuck") == 0){ sim_set_mux_stuck(true); return true; }
     if (strcmp(a, "stall on") == 0) { sim_set_stall(true);     return true; }
     if (strcmp(a, "stall off") == 0){ sim_set_stall(false);    return true; }
     if (strcmp(a, "leak on") == 0)  { sim_set_leak(true);      return true; }
     if (strcmp(a, "wdt stop") == 0) { sim_wdt_stop();          return true; }
     if (strncmp(a, "wdt slow ", 9) == 0 && parse_u32_(a + 9, &v)) {
       sim_wdt_rate_hz(v); return true;
     }
     if (strcmp(a, "noinit clobber") == 0) { sim_noinit_clobber(); return true; }
     if (strncmp(a, "ch ", 3) == 0) {
       const char *sp = strchr(a + 3, ' ');
       char chbuf[4];
       if (!sp || (size_t)(sp - (a + 3)) >= sizeof chbuf) return false;
       memcpy(chbuf, a + 3, (size_t)(sp - (a + 3)));
       chbuf[sp - (a + 3)] = '\0';
       if (!parse_u32_(chbuf, &v) || v >= PB_CHANNELS) return false;
       if (!parse_u32_(sp + 1, &w) || w > 16383u) return false;
       sim_set_channel((uint8_t)v, (uint16_t)w);
       return true;
     }
     if (strncmp(a, "resp \"", 6) == 0) {
       const char *body = a + 6;
       size_t n = strlen(body);
       if (n == 0 || body[n - 1] != '"') return false;
       link_fake_queue_response(body, n - 1);
       return true;
     }
     if (strcmp(a, "reset warm") == 0) { sim_reset(true);  return true; }
     if (strcmp(a, "reset cold") == 0) { sim_reset(false); return true; }
     hal_serial_write("sim: unknown injector; see spec section 8\n");
     return false;
   }
   #endif /* PB_SIM_CLI */
   ```
   and one line in `cli_dispatch()`, above the unknown-command fallthrough:
   ```cpp
   #if PB_SIM_CLI
     if (strncmp(line, "sim ", 4) == 0) return cmd_sim_(line + 4);
   #endif
   ```

4. - [ ] Make the sim loud. **None of this is decoration: the guarantee that a sim binary cannot pump is structural — `hal_uno.cpp` is excluded by `build_src_filter`, so no translation unit can even name `PIN_PUMP_EN`, D6 is never made an output, stays an input from reset, and R1 holds the relay's OFF level in hardware even with 12 V on COM (spec §8). The loudness exists so that a human never has to rely on remembering which binary is flashed.**

   In `ui.cpp`, the two banner rows are already in `ui_render`/`ui_render_lcd` from task 10 (`if (s->sim) row_(rows[0], ...)`); confirm they still hold row 0 unconditionally, after every other assignment to that row.

   In `main.cpp`, in `setup()` after the banner:
   ```cpp
   #if PB_SIM
     hal_pin_mode(PIN_LED, PB_OUT);   /* hal_pin_mode()'s ONE caller in the tree: seam 1
                                         declares it and nothing else in the program
                                         configures an ordinary output pin */
     hal_serial_write("SIM *** D6 NOT DRIVEN. This binary has no pump driver and no network stack.\n");
   #endif
   ```

   and the double blink, above `loop()` and as `loop()`'s **last** line:

   ```cpp
   #if PB_SIM
   /* 200 on / 200 off / 200 on / 1400 off: a rhythm no bench binary produces, readable across
      a room. `PIN_LED` is include/pins.h's (task 2 step 5, the value 13) -- NOT
      `LED_BUILTIN_PIN`, which is in no header, and NOT `LED_BUILTIN`, which is an
      Arduino-header name main.cpp may not have (spec §9). */
   static void sim_blink_(void) {
     const uint32_t p = hal_millis() % 2000u;
     hal_pin_write(PIN_LED, (p < 200u || (p >= 400u && p < 600u)) ? PB_HIGH : PB_LOW);
   }
   #endif
   ```
   ```cpp
   void loop(void) {
     /* ... the six lines task 12 step 4 and task 26 step 13 wrote, unchanged ... */
   #if PB_SIM
     sim_blink_();
   #endif
   }
   ```
   **This is an eighth line in `loop()` and task 12 step 4 forbade a seventh — the two are
   reconciled here, deliberately.** That rule bounds the loop's *responsibilities in the binary
   that runs unattended*: every loop which can iterate over an I2C transfer, a modem call or a
   millisecond of wall clock must call `safety_tick()` on each iteration. `sim_blink_()` is
   compiled out of `[env:uno_r4_wifi]` and `[env:uno_r4_wifi_bringup]` entirely, it iterates
   over nothing, and it touches neither the bus, the modem nor the clock beyond one
   `hal_millis()` read. Say so in this task's commit message so the next reader does not have
   to re-derive it.

   **The `SIM ` prefix, and the shim that carries it.** Every line in the program passes
   through `hal_serial_write()`, so that is where the prefix is applied — but in the DEVICE arm
   only. The host suites assert on exact console strings (`test_cli` compares whole lines), and
   a prefix on the host would break them for a property that has nothing to do with the host.
   `hal_sim.cpp` stays one file with two preprocessor arms, which is what "byte-identical
   between the sim binary and the host suites" means here.

   Create `src/sim_console.h`, in full:

   ```c
   /* src/sim_console.h -- the internal seam between hal_sim.cpp and the device-only console
      shim. NOT a public header: exactly two files include it, both in src/. It names no
      framework header of its own, which is why §9's header-location grep needs no exclusion
      for this file -- only for the .cpp beside it. */
   #pragma once
   #include <stddef.h>

   void   sim_console_begin(void);
   size_t sim_console_read(char *buf, size_t cap);
   void   sim_console_write(const char *s);
   ```

   Create `src/sim_console.cpp`, in full:

   ```c
   /* src/sim_console.cpp -- DEVICE ONLY, and only in [env:uno_r4_wifi_sim]. Three functions,
      and nothing else will ever be added here: no pin number, no ISR, no D6. PIN_PUMP_EN is
      not even reachable from this file -- pins.h defines it only under PB_PUMP_OWNER, and
      hal_uno.cpp is the only file in the tree that defines that, and hal_uno.cpp is not in
      this env's file set at all.
      It exists so that hal_sim.cpp needs no Arduino header of its own: §9's Arduino.h rule
      stays a FILE rule (four named files) rather than becoming a file-plus-guard rule that no
      grep can express. Step 6 widens that grep's exclusion list to name this file, in this
      task's own commit. */
   #include <Arduino.h>
   #include "sim_console.h"

   void sim_console_begin(void) { Serial.begin(115200); }   /* §7's project-wide baud */

   size_t sim_console_read(char *buf, size_t cap) {
     size_t n = 0;
     while (n < cap && Serial.available() > 0) buf[n++] = (char)Serial.read();
     return n;                                   /* bounded by cap AND by available() */
   }

   /* The `SIM ` prefix lives HERE, at the one point every console line in the sim binary
      passes through, and not on the host, where the suites compare whole lines. */
   void sim_console_write(const char *s) {
     Serial.write("SIM ");
     Serial.write(s);
   }
   ```

   Then give `hal_sim.cpp`'s three console functions their device arm. **Replace task 3's
   three bodies** — `sim_serial_rx()` and `sim_serial_tx()` stay exactly where they are and
   outside both arms, because task 28's rule is that the injectors are never gated:

   ```c
   /* ---- the console. ONE file, two arms. On the host this is the fake UART, byte for byte
      as task 3 wrote it. In the sim binary on a real board it is src/sim_console.cpp, the
      device-only shim, and hal_sim.cpp itself is unchanged between the two. ---- */
   #ifdef PB_NATIVE
   size_t hal_serial_read(char *buf, size_t cap) {
     size_t n = 0;
     while (n < cap && g_rx_pos < g_rx_len) buf[n++] = g_rx[g_rx_pos++];
     return n;
   }
   void hal_serial_write(const char *s) {
     while (*s && g_tx_len < sizeof g_tx - 1) g_tx[g_tx_len++] = *s++;
     g_tx[g_tx_len] = '\0';
   }
   void hal_serial_drain(void) { g_rx_pos = g_rx_len; }
   #else
   size_t hal_serial_read(char *buf, size_t cap) { return sim_console_read(buf, cap); }
   void   hal_serial_write(const char *s)        { sim_console_write(s); }
   void   hal_serial_drain(void) {
     char b[32];
     for (uint8_t i = 0; i < 8u; ++i)            /* BOUNDED: 256 bytes, never "until empty" */
       if (sim_console_read(b, sizeof b) == 0) return;
   }
   #endif
   ```

   with `#include "sim_console.h"` added to `hal_sim.cpp` under the same `#ifndef PB_NATIVE`,
   and one line added to `hal_begin()`'s body, below `g_servo_us = 1500u;`:

   ```c
   #ifndef PB_NATIVE
     sim_console_begin();     /* the real UART, in the one binary that has one */
   #endif
   ```

   Without that line the sim binary comes up with `Serial` unopened and the console is silent
   on a board that is otherwise working — the failure that looks like a dead build.

   **The `#ifdef PB_NATIVE` / `#else` pair goes INSIDE task 28's `#if PB_SIM` gate**, not
   around it. `[env:uno_r4_wifi_test]` compiles `hal_sim.cpp` with `PB_SIM` undefined (so every
   `hal_*` body in the file is already compiled away) and filters `sim_console.cpp` out of its
   own file set — if the device arm sat outside the `PB_SIM` gate, that env would reference
   three functions it does not compile and the device suite would not link. The three envs
   that matter, stated once: `[env:native]` has `PB_NATIVE` and `PB_SIM` → the host arm;
   `[env:uno_r4_wifi_sim]` has `PB_SIM` and not `PB_NATIVE` → the device arm, with
   `sim_console.cpp` in the build; `[env:uno_r4_wifi_test]` has neither → nothing at all.
   `sim_serial_rx()` and `sim_serial_tx()` stay outside both arms (task 28's rule: injectors
   are never gated) and are simply unused in the sim binary, which has a real console.

5. - [ ] Give `make sim` its artefact so a flashed sim binary is identifiable after the fact:

   ```make
   sim:
   	@echo "SIM BUILD - the 12 V brick must be unplugged"
   	$(PIO) run -e uno_r4_wifi_sim -t upload
   	cp .pio/build/uno_r4_wifi_sim/firmware.bin firmware-SIM.bin
   ```
   Confirm the artefact's real name first — `ls .pio/build/uno_r4_wifi_sim/` after a build — and use whatever the renesas-ra builder actually wrote.

6. - [ ] Widen ONE existing invariant, then add the file-set one.

   **First, the `Arduino\.h` grep.** Task 13 step 4 wrote it as `--exclude=hal_uno.cpp`, and `src/sim_console.cpp` is the second — and last — file in `src/` that legitimately carries the Arduino header. Edit that line in `tools/check.sh`:

   ```bash
   # spec §9 names three homes for the Arduino header; src/sim_console.cpp is a FOURTH, added
   # by task 29 and recorded as a deviation in that task's commit message. It exists so that
   # hal_sim.cpp stays byte-identical between the sim binary and the host suites, which is the
   # whole reason one fake serves both. It is the only widening this line ever takes.
   expect 0 "$(grep -rEn 'Arduino\.h' include src test lib/Manifold \
                 --exclude=hal_uno.cpp --exclude=sim_console.cpp \
                 2>/dev/null | wc -l | tr -d ' ')" \
     "the Arduino header lives only in hal_uno.cpp, sim_console.cpp, lib/Network and lib/Screen"
   ```

   **Then the file-set invariant. It greps the file set the env compiles, not a linker map: `PIN_PUMP_EN` is a macro and leaves no symbol, so a map cannot prove this** (spec §8).

   ```bash
   # ---- sim: spec §8. The sim binary compiles no pump driver at all. ----
   if [ -d .pio/build/uno_r4_wifi_sim ]; then
     if [ -e .pio/build/uno_r4_wifi_sim/src/hal_uno.cpp.o ]; then
       fail "the sim env compiled hal_uno.cpp: D6 could be driven with 12 V on COM"
     else
       ok "the sim env compiles no pump driver"
     fi
     [ -e .pio/build/uno_r4_wifi_sim/src/hal_sim.cpp.o ] || \
       fail "the sim env compiled no HAL at all - check build_src_filter"
   else
     printf 'skip  sim file set (run: pio run -e uno_r4_wifi_sim)\n'
   fi
   ```
   Confirm the object path with `find .pio/build/uno_r4_wifi_sim -name 'hal_*'` after the build in the next step and adjust the two paths to whatever PlatformIO actually wrote.

7. - [ ] Build the sim binary and run the checks:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio run -e uno_r4_wifi_sim 2>&1 | tail -3 && find .pio/build/uno_r4_wifi_sim -name 'hal_*' && make check
   ```
   expected: `SUCCESS`, a `hal_sim.cpp.o` and **no** `hal_uno.cpp.o`, then `ok    the sim env compiles no pump driver` and `all invariants hold`.

8. - [ ] Run the host suite twice, once with the sim console compiled out, so the absence case is actually exercised:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio test -e native -f test_cli && pio test -e native_nosimcli -f test_cli
   ```
   expected: both runs green. The second is the one that proves `sim float 0` is not a command in a build without `PB_SIM_CLI`. **`[env:native_nosimcli]` undefines `PB_SIM_CLI` and not `PB_SIM`** — `-UPB_SIM` would compile away every `hal_*` body in `hal_sim.cpp` (task 28's gate) and the suite would link against no HAL at all.

9. - [ ] Commit, and state the cost plainly rather than burying it.

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && git add src/cli.cpp src/hal_sim.cpp src/sim_console.h src/sim_console.cpp src/ui.cpp src/main.cpp platformio.ini Makefile tools/check.sh test/test_cli/test_cli.cpp && git commit -m "sim: thirteen injectors, four kinds of loudness, and a file-set check

   The guarantee that a sim binary cannot pump is structural: build_src_filter excludes
   hal_uno.cpp, so no translation unit can name PIN_PUMP_EN, D6 is never made an output and
   R1 holds the relay off in hardware even with 12 V on COM. check.sh verifies that by
   grepping the file set the env compiles - a linker map cannot, because PIN_PUMP_EN is a
   macro and leaves no symbol. lib_ignore = Network means the sim binary has no network
   stack and cannot mint a phantom bench1sim row: butler's silence rule iterates every
   controller with last_seen > 0 and has no notion of retirement, so one afternoon of desk
   work would otherwise page the phone hourly, forever.

   The Arduino-header question is settled as the device-only shim: src/sim_console.cpp holds
   the three console functions and the Arduino header, and this commit widens task 13's
   Arduino.h grep to exclude it beside hal_uno.cpp - a recorded deviation from spec 9's
   table, which names three homes and now has four. hal_sim.cpp therefore stays
   byte-identical between the sim binary and the host suites, which is the reason one fake
   serves both, and the invariant stays a file rule rather than a file-plus-guard rule no
   grep can express. The file is excluded from [env:native], [env:uno_r4_wifi] (and so from
   bringup) and [env:uno_r4_wifi_test]; [env:uno_r4_wifi_sim] is the one env that keeps it.

   The sim console is gated on PB_SIM_CLI, not PB_SIM. PB_SIM gates the fake rig's hal_*
   bodies, so it cannot also be the flag that compiles the console away: -UPB_SIM would have
   left the host suite linking against no HAL at all.

   What this costs, plainly: the binary the sim exercises is not the binary that meets
   12 V. One file differs, and it is the ~230 lines no host test covers."
   ```

---

---

### Task 30: The remaining mechanical invariants, and the documentation the rest of the work owes

**Drop 3.**

**Files:**
- Modify: `tools/check.sh` (the invariants of spec §9's table that needed drop-2 and drop-3 code), `Makefile` (confirm `check`), `AGENTS.md`, `README.md`
- Test: `make check`

**Interfaces:**
- Consumes: the whole tree.
- Produces: `tools/check.sh` completed to spec §9's full table; `AGENTS.md` describing the five envs, the two seams, the greps, 115200 baud and the three "running the bench" sentences spec §2.7, §2.9 and §15.2 require it to carry; `README.md` with its `String -> char[]` TODO closed by construction.

**Spec sections to read in full before starting:** §6's scoping paragraph; §9's full invariants table; §15.2; §16.2; §16.3.

---

1. - [ ] Add the include-hygiene invariants, in both directions. **The mirror is the direction that matters: one `#include "safety.h"` added to `netfsm.cpp` during a later change puts a `dose_run()` call one edit away from a state whose socket is open, with the build and `make check` both green** (spec §3).

   ```bash
   # ---- include hygiene, both directions: spec §3 ----
   expect 0 "$(count 'WiFiS3|link\.h|Network\.h' src/safety.cpp lib/Manifold)" \
     "safety.cpp and the cart cannot make a network call"
   expect 0 "$(count 'safety\.h|dose_run|hal_pump_write' src/netfsm.cpp src/ui.cpp lib/Network)" \
     "the network layer and the painter cannot assert D6"
   expect 1 "$(count 'dose_run\(' src/cli.cpp)" \
     "cli.cpp has exactly one dose_run call site"
   ```
   Note in a comment above the last one that `src/exec.cpp` carries the tree's **second** `dose_run(` call site and is expected to: this invariant is scoped to `cli.cpp` on purpose. Adjust the scope comment, never widen the invariant.

2. - [ ] Add the `%d` invariant. **This one is load-bearing, not belt-and-braces: `t = hal_boot_salt() + hal_millis()` is above 2^31 on ordinary boots, so a single `%d` against a `uint32_t` prints a leading `-`, `_int_in` rejects it, and every report 400s from the first one — with the console looking healthy. `-Wall -Wextra` does not diagnose it** (spec §9, §15.2).

   ```bash
   # ---- the wire's integers: spec §9, §15.2. Every t=/ack=/chN= site is %lu with an
   #      explicit (unsigned long) cast. -Wformat-signedness in build_src_flags is the
   #      early warning; this grep is the enforcement. ----
   expect 0 "$(count '%[0-9.]*[di]([^[:alnum:]]|$)' src/report.cpp src/netfsm.cpp)" \
     "no signed integer conversion in the report or the framing"
   ```

3. - [ ] Add the preprocessed-console invariant. It is scoped to `src/cli.cpp` because those tokens are **console affordances** and `cli.cpp` is where a console affordance can exist. `long_prime` is deliberately absent: it is a member of `dose_req_t` read by `dose_run()` in `safety.cpp`, both compiled into the bench env, so a whole-tree grep for it could never return zero (spec §6).

   **Grep for the C string literals, never the bare words.** A pattern of `hang` matches the substring inside `sensors_float_change_age_s`, inside `netfsm`'s `exchange`, and inside any `unchanged` — and preprocessing expands every included header into the same file, so all three reach this text. The pattern below quotes all four tokens, exactly as the original already did for two of them.

   ```bash
   # ---- the two binaries, proved on the PREPROCESSED bench source: spec §6, §9 ----
   pp="$(mktemp)"
   idedata=.pio/build/uno_r4_wifi/idedata.json
   if pio run -e uno_r4_wifi -t idedata >/dev/null 2>&1 && [ -f "$idedata" ]; then
     # Take the compiler, the defines and the include paths from PlatformIO rather than
     # retyping them: idedata.json is what the IDE integration itself reads.
     cxx="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["cxx_path"])' "$idedata")"
     flags="$(python3 -c '
import json, sys
d = json.load(open(sys.argv[1]))
print(" ".join(["-D" + x for x in d["defines"]] + ["-I" + x for x in d["includes"]["build"]]))
' "$idedata")"
     # shellcheck disable=SC2086
     "$cxx" -E $flags src/cli.cpp > "$pp" 2>/dev/null
     expect 0 "$(grep -cE '"hang"|" hang"|"prime"|" prime"|"cal "|"noinit pattern"' "$pp" | tr -d ' ')" \
       "the bench binary carries no hang, prime, cal or noinit console token"
   else
     printf 'skip  preprocessed bench console (run: pio run -e uno_r4_wifi -t idedata)\n'
   fi
   rm -f "$pp"
   ```

   **Run the grep against a real preprocessed file before you commit this step**, and against the *bringup* env's preprocessed `cli.cpp` too — the bench one must report 0 and the bringup one must report more than 0, or the check is passing for the wrong reason. If `idedata.json`'s key names differ on the installed PlatformIO version, print the file and adjust; if extracting the flags proves fragile at all, the honest fallback is `strings .pio/build/uno_r4_wifi/src/cli.cpp.o` grepped for the same quoted tokens. Say in the commit message which form shipped.

4. - [ ] Add the object-hash invariant. **This is the stronger check that replaces a whole-tree grep for `long_prime`, and it is what lets bring-up 7c prove the watchdog on the bringup binary and have that mean something for the bench one** (spec §6).

   ```bash
   # ---- the safety layer and the pin layer compile identically in both envs: spec §6 ----
   a="$(find .pio/build/uno_r4_wifi         -name 'safety*.o' 2>/dev/null | head -1)"
   b="$(find .pio/build/uno_r4_wifi_bringup -name 'safety*.o' 2>/dev/null | head -1)"
   c="$(find .pio/build/uno_r4_wifi         -name 'hal_uno*.o' 2>/dev/null | head -1)"
   d="$(find .pio/build/uno_r4_wifi_bringup -name 'hal_uno*.o' 2>/dev/null | head -1)"
   if [ -n "$a" ] && [ -n "$b" ] && [ -n "$c" ] && [ -n "$d" ]; then
     [ "$(shasum "$a" | cut -d' ' -f1)" = "$(shasum "$b" | cut -d' ' -f1)" ] \
       && ok "safety.o is identical in bench and bringup" \
       || fail "safety.o differs between bench and bringup: the safety layer is not the same code"
     [ "$(shasum "$c" | cut -d' ' -f1)" = "$(shasum "$d" | cut -d' ' -f1)" ] \
       && ok "hal_uno.o is identical in bench and bringup" \
       || fail "hal_uno.o differs between bench and bringup: the pin layer is not the same code"
   else
     printf 'skip  object-hash equality (run: pio run -e uno_r4_wifi -e uno_r4_wifi_bringup)\n'
   fi
   ```

5. - [ ] Build both envs and run the completed check:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && pio run -e uno_r4_wifi -e uno_r4_wifi_bringup >/dev/null && make check
   ```
   expected: every invariant in spec §9's table prints `ok`, none skips, `all invariants hold`, exit 0.

6. - [ ] Prove the mirror grep can fail, because that is the one whose absence would be silent:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && printf '#include "safety.h"\n' >> src/netfsm.cpp && make check; echo "exit=$?"
   ```
   expected:
   ```
   FAIL  the network layer and the painter cannot assert D6: expected 0, found 1
   exit=1
   ```
   Then revert:
   ```
   cd /Users/jcanton/projects/plant-butler/firmware && git checkout -- src/netfsm.cpp && make check
   ```

7. - [ ] Rewrite `AGENTS.md`'s build and layout sections (spec §16.3). The six things it owes:

   ```markdown
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

   ## The two seams

   `include/hal.h` is seam 1: Arduino-free free functions, implemented by `src/hal_uno.cpp`
   on the board and `src/hal_sim.cpp` on the host. `include/link.h` is seam 2: ten network
   primitives, implemented by `lib/Network/src/link_wifi.cpp` and `src/link_fake.cpp`.
   Implementations are selected by `build_src_filter`, never by a runtime flag.

   **`src/hal_uno.cpp` is the only file in the tree with an Arduino header, a pin number, an
   ISR or a write to D6.** `lib/Network/src/link_wifi.cpp` is the only one that names WiFiS3;
   `lib/Screen` the only one that names the LCD or u8x8 libraries.

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
   ```
   And replace the cycle-1 paragraph with:
   ```markdown
   Cycle 1's line "A4 becomes channel 5" is **dead**. The bench wiring supersedes it: A4/A5
   are I2C, carrying the expander (which carries the mux select lines and the home hall) and
   both screens. The five channels arrive through the mux on A0.
   ```

8. - [ ] Close the README TODO — it closes **by construction**, because `hal.h` exposes only `const char *` and `make check` greps `String` to zero outside `lib/Network` and `lib/Screen`:

   ```markdown
   # plant butler

   Bench-rig firmware for one manifold, five outlets. See
   `docs/superpowers/specs/2026-09-03-bench-sketch-design.md`, and `AGENTS.md` for the build.

   The old `String -> char[]` TODO is closed by construction: `include/hal.h` exposes only
   `const char *`, and `make check` greps `String` to zero outside `lib/Network` and
   `lib/Screen`.
   ```

9. - [ ] Commit the checks and the docs:

   ```
   cd /Users/jcanton/projects/plant-butler/firmware && make check && git add tools/check.sh Makefile AGENTS.md README.md && git commit -m "make check completed, and the docs the rest of the work owes

   The invariants that needed drop-2 and drop-3 code: the two include-hygiene greps in
   both directions, the single dose_run call site in cli.cpp (exec.cpp's is the expected
   second in the tree - the scope comment moved, the invariant did not widen), the signed
   conversion grep over report.cpp and netfsm.cpp, the preprocessed bench console grep, and
   the safety.o/hal_uno.o hash equality that replaces a whole-tree long_prime grep.

   The signed-conversion grep is load-bearing: t = boot_salt + millis is above 2^31 on
   ordinary boots, so one %d 400s every report from the first one and -Wall -Wextra does not
   diagnose it. Verified failing by adding an include of safety.h to netfsm.cpp: exit 1, and
   it names the invariant.

   AGENTS.md gains the five envs and which one is left running, the two seams, what each
   grep protects, 115200 baud, and a line marking cycle 1's A4-becomes-channel-5 dead."
   ```

10. - [ ] Flash the **bench** binary and confirm bring-up 7e by hand (spec §13). This is where "the unattended binary is a different binary" is actually checked:

    ```
    cd /Users/jcanton/projects/plant-butler/firmware && make upload && make monitor
    ```
    Type `status`: it must say `build=bench`. Then type `pump 2000`, `cal 5880` and `pump 3000 hang`: each must come back `? unknown; type help`. `clear contra` must still work — it ships in both binaries, because the unattended binary can latch and a rig that cannot be released except by a reflash is worse.

    Then expect, about two minutes after first boot, exactly **one** HIGH `pos:` push: *"bench1 lost track of its manifold position: watering is on hold"*. That is `PB_REPORT_POS_UNKNOWN=1` doing its job, not a fault. It raises once and stands for the whole run, and **while it stands the `pos:` rule is deaf, so a genuine position loss during the 48 hours will not page.** Do not clear it by hand.

11. - [ ] Open the issues this work owes the other repos. None of it is firmware work; all of it is this work's output. Find the repository names first rather than guessing them — `gh repo list plantbutler` — then file:

    - **`cad/wiring`** (spec §16.1, twelve items): the corrected D6 boot recipe is item 1 and is *the single most important line this work sends back to another repo*; the 10 k pull-up on D2 is item 2; the two screens and the 0x27 collision are item 3; the rest are bring-up table changes. Every one of them is a `nets.py` change followed by `make`, because `README.md` is generated.
    - **`DECISIONS.md`** (spec §16.4, two dated amendments): #10 says "IWDT" and that word is false today — this work ships the WDT, register-start and PCLKB-clocked, and `status` prints "(WDT, not IWDT)" so nobody reads #10 and believes something the code does not do. #6 says the two screens are dropped; the bench keeps both, and the amendment should record what keeping them costs.
    - **the bring-up notes in `cad/wiring`**: the three "Running the bench" sentences above must appear there too, because that is what the person at the bench is reading. They are `nets.py`-adjacent prose, so file them as one issue rather than editing the generated README by hand.
    - **`backend/` and `app/`** (spec §16.5, ten items). **Flag §16.5.3 — the contradiction latch's durable half — as a going-live blocker if bring-up 7c′ fails**, because until it lands a power cycle after a latch silently rearms the rig. Flag §16.5.10 as landing first if the 48-hour run needs a live `pos:` alert.

    ```
    gh issue create --repo TODO_FILL_IN_owner/TODO_FILL_IN_repo \
      --title "cad/wiring: the D6 boot recipe is wrong for this silicon" \
      --body "TODO_FILL_IN — the correction, in full, from this plan's task 8 commit message"
    ```

    **Every `TODO_FILL_IN` in this plan is a placeholder you must replace before committing**, and there are **four**, on four lines: two in task 28's commit message (the slow-responder fixture, the desync count) and two in the `gh issue create` above (the repo and the body). Task 29's commit message contains none — its Arduino-header decision is written out in full, not left blank — and an earlier draft that counted it made five. The proof below counts *lines*, not tokens, and the two on the `gh` line are on separate lines, so the expected count is four. Before finishing this task, prove none of them was committed verbatim:

    ```bash
    cd /Users/jcanton/projects/plant-butler/firmware && git log --format=%B $(git merge-base HEAD main)..HEAD | grep -c TODO_FILL_IN
    ```
    expected: `0`.
    Every issue body ends with its own last line:
    ```
    🤖 Written by an agent on behalf of @jcanton
    ```

---

## Assembly notes

Three notes on how this plan is put together, recorded here rather than left to be rediscovered inside the tasks. The second is the map of what is **not** finished; it was re-derived mechanically on 2026-09-03 by diffing each task's `**Tests:**` list against the test-function definitions actually printed in that task's steps, and by checking every path this plan lists as `Create` for printed content. A named gap is acceptable; an unnamed one is not.

- **The compile-twice host environments are SETTLED: named environments, and nothing else.** Four suites in this plan are compiled a second time under one different flag — `-DPB_PULSES_PER_GATE=1450` (task 14's calibrated arm), `-DPB_ML_PER_S_MEASURED=30` (task 17's and task 18's clamp and plausibility cases), `-UPB_BRINGUP` (task 20's bench-vs-bringup case) and `-UPB_SIM_CLI` (task 29's absence case). **Task 1 creates `[env:native_cal]`, `[env:native_measured]`, `[env:native_bench]` and `[env:native_nosimcli]`, each three lines of `extends = env:native` plus one flag, and no task in this plan uses `PLATFORMIO_BUILD_FLAGS`.** Note the last one: it undefines `PB_SIM_CLI`, not `PB_SIM`, because `PB_SIM` also gates every `hal_*` body in `hal_sim.cpp` (task 28) and `-UPB_SIM` would leave the host suite linking against no HAL at all.

- **What is NOT finished in this file, named precisely.** Everything below is a gap you must close, in this plan's rhythm — the failing test first, run it and see it fail, the minimal code, run it and see it pass, commit — before or while implementing the task it belongs to. The contract above each gap (its Files, Interfaces, spec citations and Deliverable) is what the missing work must satisfy; do not weaken a test name, an interface spelling or a deliverable to make a step easier to write. Anything not listed here **is** printed: every path this plan lists as `Create` has its content given in full, including `src/sim_console.h` and `src/sim_console.cpp` (task 29 step 4) and `test/support/bodies.h` (task 26 step 1).

  **(a) Test cases named but not printed — 21 of them, in three tasks.**

  - **Task 14 step 9 — nine of `test_cart`'s twelve.** Printed: `test_goto_refuses_when_pulses_per_gate_is_zero`, `test_home_from_outlet_five_actually_reaches_home`, `test_servo_is_stopped_on_every_exit_path`. Given a setup-and-assertion sentence but **no code**: `test_position_is_unknown_after_boot_until_homed`, `test_pos_is_never_ok_before_calibration`, `test_goto_counts_pulses_not_milliseconds`, `test_home_zeroes_the_count_only_when_the_hall_asserts`, `test_home_that_times_out_leaves_position_unknown`, `test_stall_aborts_within_the_stall_window_and_loses_position`, `test_an_i2c_error_on_the_home_hall_is_unknown_not_not_home`, `test_goto_rejects_an_outlet_outside_one_to_five`, `test_move_deadline_holds_across_a_millis_rollover`.
  - **Task 17 — five of `test_dose`'s twenty-four, and one moved out.** Printed by this task: `test_pump_is_off_on_every_exit_path` (step 1, and rewritten since — its loop asserts the number of arms it drove against `PB_DRIVABLE_RESULTS`, because the first version `TEST_IGNORE`d inside the loop and Unity's ignore longjmps out of the whole case, leaving sixteen of nineteen results undriven under a green `0 Failures`); `test_refusal_reports_zero_millilitres_not_the_previous_dose` and `test_the_ladder_reports_the_more_specific_reason` (step 7, both now registered — they were declared non-static and never had a `RUN_TEST` line, so the suite silently reported twenty-two); and **the whole refusal ladder and both target rules — all fifteen, in full, in step 8**, which is where the safety spine of drop 2 used to be a list of bare names. Still a prose bullet with its setup and assertion but **no code**: `test_target_pulses_match_the_calibration_within_one_pulse`, `test_dose_cap_holds_across_a_millis_rollover` (its fixture is now stated — the cap must straddle the wrap or the case is vacuous), `test_console_pump_does_not_require_a_known_position`, `test_bytes_buffered_during_a_dose_are_discarded_not_executed` (its task-17 form asserts through `status`; task 20 owns the `pump 60000` form, since `pump` is not a command until then), and `test_cap_is_clamped_to_twice_the_requested_millilitres` (fixture stated: `long_prime`, metered, `ml` at or below 200). **`test_dose_cap_is_clamped_to_sixty_seconds` is no longer task 17's** — it cannot be written before task 18's burst injector exists, because a no-flow dose ends at ~3001 ms and then latches `contra`, which is `.noinit`-backed and poisons every later case in the file.
  - **Task 18 step 8 — six of `test_dose`'s fourteen abort cases.** Each has a prose bullet naming its injector, its setup and its assertion; none has code: `test_dose_aborts_when_the_pulse_rate_exceeds_the_meter_rating`, `test_a_dose_that_reaches_target_implausibly_fast_is_noise_not_ok`, `test_dose_stops_within_one_iteration_when_the_float_drops`, `test_dose_aborts_when_the_expander_read_fails_mid_dose`, `test_stop_typed_mid_dose_stops_it_within_one_iteration`, `test_dry_on_typed_mid_dose_stops_it`.
  - **Task 19 step 4's six "does not set" cases are NOT a gap.** An earlier version of this note listed them; they are printed in full, with bodies and assertions, as is every other case task 19 names. The one name in its list that task 19 does not print, `test_latch_reports_err_contra_and_ch207_and_float_zero`, is deliberately deferred to **task 26 step 11**, which does print it.

  **(b) Fake-rig code described in prose rather than printed.** All of these are `src/hal_sim.cpp`; every one has its declaration printed in `include/sim.h` and its behaviour stated exactly, and none has a body.

  - **Task 14 step 1 — the largest of them: the whole screw/home-region/servo model.** The per-millisecond position integrator, the direction taken from the microseconds last written through seam 1, the `SIM_EV_SCREW` emitter, the home hall answered as `pos >= lo && pos <= hi` through the expander, the `[0, 40]` default region in `sim_reset()`, and the deletion of task 6 step 6's `sim_screw_hz_()` placeholder. **Twelve `test_cart` cases rest on this fixture**, and so does task 19's `test_latch_does_not_refuse_homing`. The same step also declares `sim_set_clock_ms()` and gives its contract in prose, not code.
  - **Task 15 step 1 — `sim_set_float_pattern()`'s body**: the `static char g_float_pat[16]`, the index advanced by each `hal_pin_read(PIN_HALL_FLOAT)`, the stick-at-the-last-character rule, and the `sim_reset()` clear.
  - **Task 22 step 13 — `sim_set_heap_break()`'s body**: one file static that `hal_heap_break()` returns when set.
  - **Task 24 step 13 — `sim_on_pump_on()`'s body**: the `static void (*g_on_pump_on)(void);`, its setter and its `sim_reset()` clear. (The one line that invokes it inside `hal_pump_write(true)` *is* printed.)
  - Task 18 step 1's four pump-relative injectors were in this list and are **no longer**: their state, their bodies, the `hal_pump_write()` arming and the `advance_1ms_()` application are all printed.

  **(c) Things a reader could otherwise take as finished, and cannot.**

  - **`include/secrets.h` is created by task 1 step 8 and committed by nothing** — it is gitignored. It is a build input from task 11 onward, on the host and on the board alike, so a fresh clone does not compile until an operator copies the example and fills it in. There is no step that can do this for them.
  - **Four `TODO_FILL_IN` placeholders**, on four lines, must be replaced before their commits: two in task 28 step 7's commit message (the slow-responder fixture actually used, the desync count actually seen) and two in task 30 step 7's `gh issue create` (the umbrella repo, the body). Task 30 step 11 is the check that none was committed verbatim.
  - **Task 28 step 3 needs a fixture the spec does not name**: a deliberately slow responder on `HOST_NAME:HTTP_PORT` for `test_a_recv_pass_against_a_slow_responder_completes_within_the_wdt_window`. Stand one up before the suite runs and record which.
  - **`link_fake_saw_available()` and `link_fake_saw_connected()` return a hard-coded `false`**, so `test_sock_read_calls_neither_available_nor_connected` asserts a tautology. It documents the shape of seam 2; it is not coverage of the driver, and nothing in this plan covers that half except task 28's two wall-clock cases, indirectly.
  - **The running `N Tests` totals are advisory** (see the Global Constraint). The `test_dose` chain was recounted and corrected to 1, 4, 10, 16, 18, 19 across tasks 1-6. The `test_report` chain was **not** settled: task 22 step 16 prints `25 Tests` while its own sentence says nineteen plus seven, its earlier running numbers reach 15, and this file prints 26 test-function definitions for that suite; task 23's `26` and `32` are self-consistent with each other and with a base of 22. Count the `RUN_TEST` lines and fix the numbers in the commit message. Nothing about the code depends on which is right, but a reader chasing a number will lose an hour.
  - **Eight steps cannot be done at a desk.** Task 12 step 7 (bring-up 0, the banner before 12 V), task 20 step 15's close-out note and the eight bring-up steps it lists, task 27 step 7 (`status` on real silicon, and the two connect-form questions the driver cannot answer off-bench), task 28 steps 4-6 (the device suite, the two host suites on silicon, the AP pulled by hand), task 29 step 7 (the sim binary on a board) and task 30 step 10 (bring-up 7e). A green `pio test -e native` is not coverage of any of them.


- **Where this plan deliberately departs from the spec, and why.** Each is recorded in the task that makes the departure and in that task's commit message, so none of them is a silent drift.
  - `hal_micros()`, `hal_adc_bits()` and `hal_adc_width_ok()` are added to seam 1, making it 35 functions rather than 32 (task 3).
  - `hal_begin()` rather than `main.cpp` opens the console at 115200 (task 8 step 2, task 12).
  - `loop()` is six lines rather than spec §3's five; the sixth is `pulses_leak_poll(safety_dosing())`, and `sensors_sweep()` is **not** in the loop — task 24's `NET_IDLE` pass owns it (tasks 12, 24, 26).
  - `net_poll()` takes the dosing flag as a parameter rather than calling `safety_dosing()` (task 24).
  - `exec_pending()` gets its own translation unit rather than living in `main.cpp` (task 26).
  - The sim binary's console reaches the Arduino header through a device-only shim, `src/sim_console.cpp`, rather than through `hal_sim.cpp` — so spec §9's `Arduino\.h` invariant names four files rather than three (task 29 step 6).
  - `make check`'s `PB_BRINGUP` file count is scoped to `src` and `lib` rather than the whole tree, because `include/config.h`'s uncalibrated-build guard and `test_cli.cpp`'s two-arm cases both name the macro deliberately (task 13 step 5, task 20 step 13).
  - `make check` counts the **definition** of `PB_PUMP_OWNER`, not the bare token, because `pins.h` necessarily tests it with `#ifdef` (task 8 step 1, task 13 step 1).
  - `report_heap_ok()`, `report_last_len()`, `report_txcap_drops()`, `link_fake_sent()` and `link_fake_write_count()` are additions to the skeleton's declaration lists (tasks 21, 22).
  - `noinit_reset_mid()` is a fifth `.noinit` accessor, and the only producer of the `resetmid` token (task 4).
  - `exec_has_pending()` is deliberately **not** declared: nothing would call it (task 26).

  Constants the spec needs but does not print, each decided in the file it belongs to rather than all in one place: `PB_BOOT_SALT_STRIDE` in `include/noinit.h` (task 4 step 3), `PIN_LED` in `include/pins.h` (task 2 step 5), `PB_SERVO_FWD_US` / `PB_SERVO_REV_US` / `PB_SERVO_STOP_US` in `include/config.h` (task 14 step 3), and the expander's `EXP_INPUTS_HI` / `EXP_HOME_BIT` layout as file-locals of `src/sensors.cpp` (task 7 step 4). **There is no `PB_SWEEP_MS`**: the sweep's cadence is `net_next_s`, in task 24's `NET_IDLE` pass, and a constant would be a second, disagreeing answer to the same question.
