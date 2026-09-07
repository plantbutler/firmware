/* bodies.h: canned HTTP responses shared by more than one suite, a header because PlatformIO builds one binary per test/ subdirectory. */
#pragma once

/* Every Content-Length is counted, not estimated: rx_complete() returns false while
   `have < cl`, so a length one byte too large makes the response a permanent truncation and
   the case fails at the RECV deadline for a reason unrelated to what it tests.
     "next=60\n"                          =  8
     "cmd=NN water=N ml=NNN cap_s=NN\n"   = 31   (6 + 1 + 7 + 1 + 6 + 1 + 8 + 1)
     "cmd=NN stop=1\n"                    = 14   (6 + 1 + 6 + 1)
   so a water body is 39 and a stop body is 22. */
static const char k_cmd_200[] =
  "HTTP/1.1 200 OK\r\nContent-Length: 39\r\n\r\nnext=60\ncmd=17 water=3 ml=100 cap_s=10\n";
static const char k_stop_200[] =
  "HTTP/1.1 200 OK\r\nContent-Length: 22\r\n\r\nnext=60\ncmd=31 stop=1\n";
static const char k_out_of_range_200[] =
  "HTTP/1.1 200 OK\r\nContent-Length: 39\r\n\r\nnext=60\ncmd=62 water=0 ml=100 cap_s=10\n";
