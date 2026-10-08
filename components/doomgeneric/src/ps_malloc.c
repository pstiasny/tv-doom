// PSRAM allocator shim. doomgeneric (lilka fork) calls ps_malloc() for the
// Doom zone heap, the framebuffer, and the relocated renderer arrays. On the
// ESP32-S3 route those to external SPI RAM.
//
// d_alloc.h deliberately does NOT include <esp_heap_caps.h> (it clashes with
// some Doom macros/literals), so we provide the definition here where it is
// safe to include the real header.

#include "esp_heap_caps.h"

void *ps_malloc(size_t size)
{
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}
