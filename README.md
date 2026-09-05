# plant butler

Bench-rig firmware for one manifold, five outlets. See
`docs/superpowers/specs/2026-09-03-bench-sketch-design.md`, and `AGENTS.md` for the build.

The old `String -> char[]` TODO is closed by construction: `include/hal.h`'s API is
`char *`/`const char *` throughout, never `String` (`hal_serial_read(char *buf, size_t
cap)` fills a caller-owned buffer; `hal_boot_pump_off()` and friends take no strings at
all), and `make check` greps `String` to zero outside `lib/Network` and `lib/Screen`.
