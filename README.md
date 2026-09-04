# plant butler

Bench-rig firmware for one manifold, five outlets. See
`docs/superpowers/specs/2026-09-03-bench-sketch-design.md`, and `AGENTS.md` for the build.

The old `String -> char[]` TODO is closed by construction: `include/hal.h` exposes only
`const char *`, and `make check` greps `String` to zero outside `lib/Network` and
`lib/Screen`.
