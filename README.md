# Plant Butler firmware

The program on the board: an Arduino UNO R4 WiFi that reads one soil-moisture sensor per pot,
posts the readings to the backend every report interval, and waters one pot on command through a
pump and a servo-driven manifold. It holds the safety limits and fails dry: when in doubt, no
water. How the board, the backend and the app fit together is in the
[umbrella README](https://github.com/plantbutler/plantbutler#readme); the words are in its
[glossary](https://github.com/plantbutler/plantbutler/blob/main/GLOSSARY.md).

## Build and flash

Needs [PlatformIO](https://platformio.org) (`pio` on the path) and, for flashing, the board on USB.

```bash
cp include/secrets.h.example include/secrets.h   # WiFi, backend host and port, token, board number
make               # build the bench binary, the one left running
make upload        # flash it
make monitor       # serial console at 115200; type help
make test          # host tests, no board needed (needs secrets.h too)
make test-all      # the host tests under every build flag combination
make build-all && make check   # build every board variant, then the 34 invariant checks
```

`secrets.h` is gitignored and never committed. `make check` exits 2 until `make build-all` has run,
because seven of its checks read the compiled objects.

Bring-up and bench work, each a different binary:

```bash
make bringup       # pump, cal, servo, home, goto, hang commands compiled in; never left running
make sim           # no pump driver, no network: drive the rig by hand. Unplug the 12 V supply first
make calib         # bring-up binary plus the monitor; type calib to measure the pulses per litre
make test-device   # the on-board test suite; needs the board and a slow HTTP listener
```

## Environments

Which files compile is decided per environment in `platformio.ini` by `build_src_filter`, which
replaces rather than extends the base list.

| environment | what it is | left out |
| --- | --- | --- |
| `uno_r4_wifi` | the bench binary, unattended | `hal_sim.cpp`, `link_fake.cpp`, `sim_console.cpp` |
| `uno_r4_wifi_bringup` | bench plus the bring-up commands | same |
| `uno_r4_wifi_test` | on-board tests | `link_fake.cpp`, `main.cpp`, `sim_console.cpp` |
| `uno_r4_wifi_sim` | no pump driver, no network | `hal_uno.cpp` |
| `native` | host tests | `main.cpp`, `hal_uno.cpp`, `sim_console.cpp` |
| `native_bench`, `_cal`, `_measured`, `_nosimcli`, `_live` | host tests under one extra flag each | same as native |

## Files

Two seams, each a header of plain functions with a board and a host implementation:
`include/hal.h` for the hardware and `include/link.h` for the network.

| file | what it holds |
| --- | --- |
| `src/main.cpp` | `setup()` in its load-bearing order, the boot banner, `loop()` |
| `src/safety.cpp` | the only writer of the pump pin and the only watchdog feeder; the float debounce, the latches, `dose_run()` |
| `src/netfsm.cpp` | the report and response state machine, one bounded network step per pass |
| `src/report.cpp`, `src/exec.cpp` | the `k=v` report body; the one command per round trip that comes back |
| `src/sensors.cpp`, `src/pulses.cpp` | the sensor multiplexer and I2C health; the flow and screw pulse counters |
| `src/cli.cpp`, `src/ui.cpp` | the serial console (commands in the `k_commands` table); the two screens |
| `src/noinit.cpp` | the bytes that survive a warm reset: the latches and a checksum |
| `src/hal_uno.cpp`, `src/hal_sim.cpp` | the hardware seam on the board and on the host |
| `src/link_fake.cpp`, `lib/Network` | the network seam on the host and on the board |
| `lib/Manifold` | the cart: a servo moving a magnet along a lead screw, counted in pulses |
| `lib/Screen` | the OLED and the 16x2 LCD (address 0x27 or 0x3F, probed) |
| `include/config.h` | every tunable, with the derivation of each limit |
| `include/pins.h` | the wiring contract; no pin number lives anywhere else |
| `test/` | one directory per suite; `test/support` holds the shared fixture |
| `tools/check.sh` | the invariants a compiler cannot check, as greps and object-file checks |

Where to go: a new console command is one row in `k_commands` in `src/cli.cpp`; a pin change is
`include/pins.h`; the dose limit is `PB_DOSE_RIG_MAX_ML` in `include/config.h`, and the backend keeps
the same number.

## On the bench

- A power cycle clears the latches: they live in memory that survives a reset, not a power cut. A
  latched rig stays latched until a person reads `status` and decides.
- A report interval under about 60 s stutters while a dose runs: a dose blocks for up to 60 s.
- After a power event, look for gaps in the readings: the boot counter restarts, and two boots
  can collide on the same timestamp inside the backend's duplicate window.
