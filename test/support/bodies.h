/* bodies.h: canned HTTP responses and whole-token body matchers shared by more than one suite, a header because PlatformIO builds one binary per test/ subdirectory. */
#pragma once
#include <stddef.h>
#include <string.h>

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

/* Whole-token matches over a k=v body or a printed line, because strstr() finds "ch1=8001"
   inside "ch11=8001" and "ml=" inside "flow_ml=". A token starts the text or follows a space
   or a newline, and ends at a space, a newline or the end of the text; `tok` may span several
   tokens ("ack=17 flow_ml=0"). A key is a token's left half, '=' included. */
static inline bool pb_has_tok(const char *body, const char *tok) {
  const size_t n = strlen(tok);
  for (const char *p = strstr(body, tok); p; p = strstr(p + 1, tok)) {
    const bool left  = (p == body) || p[-1] == ' ' || p[-1] == '\n';
    const bool right = (p[n] == ' ' || p[n] == '\n' || p[n] == '\0');
    if (left && right) return true;
  }
  return false;
}

static inline bool pb_has_key(const char *body, const char *key) {
  for (const char *p = strstr(body, key); p; p = strstr(p + 1, key))
    if (p == body || p[-1] == ' ' || p[-1] == '\n') return true;
  return false;
}
