/*
 * Lightweight UART-free crash record (option B).
 *
 * The IDF panic handler is --wrap'd: __wrap_esp_panic_handler stashes the
 * faulting PC, exception class and core into RTC_NOINIT RAM (pure RAM writes,
 * safe with the flash cache off and interrupts disabled), then chains to the
 * real handler. The next boot reports it through the log hook -> BLE, and the
 * `diag` command reprints it on demand. Locate the PC off-board with:
 *     xtensa-esp32s3-elf-addr2line -pfiaC -e build/tv-doom.elf <PC>
 *
 * ~40 bytes of RTC RAM vs ~9 KB DRAM for the full coredump component: no
 * backtrace/task dump, but the faulting PC pins most crashes.
 */
#include "crashlog.h"

#include <string.h>
#include <inttypes.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_private/panic_internal.h"

#define CRASH_MAGIC 0xC5A5B10Bu

static RTC_NOINIT_ATTR struct {
    uint32_t magic;
    uint32_t pc;          /* faulting instruction address */
    uint32_t exception;   /* panic_exception_t            */
    uint32_t core;
    uint32_t seq;         /* increments each recorded crash */
} s_crash;

extern void __real_esp_panic_handler(panic_info_t *info);

void __wrap_esp_panic_handler(panic_info_t *info)
{
    uint32_t seq = (s_crash.magic == CRASH_MAGIC) ? s_crash.seq + 1 : 1;
    s_crash.magic = CRASH_MAGIC;
    s_crash.pc = (uint32_t)info->addr;
    s_crash.exception = (uint32_t)info->exception;
    s_crash.core = (uint32_t)info->core;
    s_crash.seq = seq;
    __real_esp_panic_handler(info);   /* never returns */
}

void crashlog_report(void)
{
    if (s_crash.magic != CRASH_MAGIC || !s_crash.seq) return;
    ESP_LOGE("crashlog", "PREVIOUS CRASH #%" PRIu32 ": PC=0x%08" PRIx32
             " exc=%" PRIu32 " core=%" PRIu32 " (addr2line on tv-doom.elf)",
             s_crash.seq, s_crash.pc, s_crash.exception, s_crash.core);
}

bool crashlog_get(uint32_t *pc, uint32_t *exc, uint32_t *seq)
{
    if (s_crash.magic != CRASH_MAGIC || !s_crash.seq) return false;
    if (pc)  *pc  = s_crash.pc;
    if (exc) *exc = s_crash.exception;
    if (seq) *seq = s_crash.seq;
    return true;
}
