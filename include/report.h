/* include/report.h — the wire protocol: report_build() and the ack pair. Spec §4. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct { uint32_t id; uint16_t flow_ml; const char *err; } report_ack_t;

void     report_stamp(void);
uint32_t report_t_wire(void);
uint32_t report_t_ms(void);

/* Builds the whole body into buf. Returns the byte count, or 0 on truncation — which means
   err=txcap, the report is DROPPED, and `status` says so loudly (spec §4.2). Every key appears
   at most once: a repeated c, t, ack, flow_ml, float, pos or chN refuses the WHOLE report
   (butler.py:220-250), and the body is assembled from four independent sources. */
uint16_t report_build(char *buf, uint16_t cap);

/* The diagnostic block -- ch200..ch(200+PB_DIAG_CHANNELS-1), every value min(v, PB_DIAG_CLAMP)
   -- over ITS INPUTS, an array of PB_DIAG_CHANNELS values. report_build() hands it the twelve
   producers; test_report hands it twelve values above the clamp. Public for that second
   caller and no other reason: three of the producers are constants in hal_sim.cpp and four
   are booleans, so through report_build() alone neither the clamp on ch207/ch208/ch210/ch211
   nor the block's full width is observable on the host -- an emitter that skipped the clamp
   for the two latch channels passed every case in the tree before this seam existed. Appends
   at *n; false on truncation, exactly as the body's other put_ helpers. */
bool report_put_diags(char *buf, uint16_t cap, uint16_t *n, const uint32_t *diag);

void     report_set_ack(uint32_t id, uint16_t flow_ml, const char *err);
void     report_clear_ack(void);
bool     report_ack_is_recv(void);
bool     report_may_build(void);

uint16_t report_last_len(void);      /* addition: `status` prints it */
uint32_t report_txcap_drops(void);   /* addition: §4.2 asks status to say so LOUDLY */
bool     report_heap_ok(void);       /* addition: spec §12 item 0's per-report break check */

typedef enum { CMD_NONE, CMD_WATER, CMD_STOP } cmd_kind_t;
typedef struct { uint32_t id; cmd_kind_t kind; uint8_t outlet; uint16_t ml; uint16_t cap_s; } cmd_t;
typedef struct { uint16_t next_s; cmd_t cmd; } response_t;

/* Parses a 200 body ONLY. Butler's 400 body echoes the board's own tokens
   (f"{key}= out of range: {value}"), so a 4xx body parsed here could water a plant.
   *out is always fully written; the return value means "a command was accepted".
   out->next_s == 0 means "keep the previous interval". Spec §4.5. */
bool response_parse(const char *body, uint16_t len, response_t *out);
