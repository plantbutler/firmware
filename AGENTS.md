# Working on the firmware

[README.md](README.md) says what this is, how to build it and where each thing lives. The
umbrella's [AGENTS.md](https://github.com/plantbutler/plantbutler/blob/main/AGENTS.md) holds the
project-wide rules; its [DECISIONS.md](https://github.com/plantbutler/plantbutler/blob/main/DECISIONS.md)
entries 4 (the wire), 5 (what the firmware may decide) and 7 (safety) are what this code keeps.

## Rules the checks enforce

`make check` runs 34 invariants; the ones below are the ones you will meet.

- `src/safety.cpp` is the only writer of the pump pin and the only caller of the watchdog feeder.
  `dose_run()` has exactly one call site in `src/cli.cpp`.
- The board library header is included only by `src/hal_uno.cpp`, `src/sim_console.cpp` and
  `lib/Screen`. Everything else reaches hardware through `include/hal.h`.
- The WiFi library is named only in `lib/Network/src/link_wifi.cpp`. Everything else reaches the
  network through `include/link.h`; adding a primitive there means editing `tools/check.sh`.
- A pin number lives in `include/pins.h` (and the host double `src/hal_sim.cpp`), nowhere else.
- No `delay()` outside `hal_uno.cpp`, no `String`, no `new`, no `malloc`, no unbounded loop
  outside `safety.cpp`, no `%f`, and no `%d` in `report.cpp` or `netfsm.cpp` (unsigned on the wire).
- `safety.o` and `hal_uno.o` are byte-identical between the bench and bring-up builds.
- The checks grep code with comments and strings stripped, so a comment may say any of the
  above in words.

## Traps

- `make test` needs `include/secrets.h` even though the host never uses the values.
- `make check` exits 2 until `make build-all` has run. A build counts when it has objects.
- `build_src_filter` replaces the base list; an environment that overrides it must repeat every
  exclusion it still wants.
- The five `native_*` environments compile the other arm of an `#if`; `make test` alone leaves
  13 cases skipped. Run `make test-all` before calling a change done.
- The bring-up binary is never left running, and the sim binary is flashed only with the 12 V
  supply unplugged.
- `include/secrets.h` never enters a commit.

## Comments

- A file starts with one line saying what it holds.
- A comment says why, not what. Keep units, derivations, hardware facts, the single-writer
  invariants, race explanations.
- No dates, task numbers, PR numbers, spec section marks, reviewer names or history.
- In a test, a comment says what a case proves only when its name does not.
