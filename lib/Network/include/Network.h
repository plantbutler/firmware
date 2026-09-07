/* Network.h: nothing includes this. lib/Network is built because platformio.ini names it in
   lib_deps; PlatformIO's dependency finder alone would never pull it in, since the seam header
   (include/link.h) is a project header, not the library's. The library's one file,
   src/link_wifi.cpp, is the only file in the tree that may name WiFiS3. */
#pragma once
#include "link.h"
