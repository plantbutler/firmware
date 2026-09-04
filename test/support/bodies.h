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
