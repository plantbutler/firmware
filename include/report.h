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

void     report_set_ack(uint32_t id, uint16_t flow_ml, const char *err);
void     report_clear_ack(void);
bool     report_ack_is_recv(void);
bool     report_may_build(void);

uint16_t report_last_len(void);      /* addition: `status` prints it */
uint32_t report_txcap_drops(void);   /* addition: §4.2 asks status to say so LOUDLY */
bool     report_heap_ok(void);       /* addition: spec §12 item 0's per-report break check */
