/* UART-free crash record (see crashlog.c). */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Report any stored crash over the log hook (-> BLE). Call once at boot. */
void crashlog_report(void);

/* Fetch the stored crash (for the diag command). Returns false if none. */
bool crashlog_get(uint32_t *pc, uint32_t *exc, uint32_t *seq);
