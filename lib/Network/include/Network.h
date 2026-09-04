/* Network.h — DO NOT DELETE. Tasks 26 and 27's fix rounds, and this task's own first
   draft, all recommended deleting it: nothing in the tree writes `#include "Network.h"`
   (grep confirms), so it looks like dead weight. It is not, and deleting it once already
   cost five tasks.

   PlatformIO's dependency finder only pulls a lib/<Name> into the build when something
   under src/ includes ONE OF THAT LIBRARY'S OWN HEADERS. Seam 2's header is
   include/link.h — a PROJECT header, not this library's — so nothing ever gave the LDF a
   reason to build lib/Network at all, and src/link_wifi.cpp was never compiled into any
   binary while this file was gone: every seam-2 symbol came out undefined the first time
   a device build ran, and the host suites couldn't see it because [env:native] never
   builds lib/Network either. The real fix (8e7df87) is naming `Network` directly in
   [env:uno_r4_wifi]'s lib_deps, which does not depend on this file existing. This header
   survives anyway, as the last legible trace of *why* the library was ever expected to
   build on its own — delete it again and that story has nowhere left to be read from.

   The library's whole surface is seam 2; lib/Network/src/link_wifi.cpp (task 27) is its
   only implementation, and it is the only file in the tree that may name WiFiS3. See
   spec §1. */
#pragma once
#include "link.h"
