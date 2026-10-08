//
// WAD I/O for ESP32: serve the IWAD directly out of a dedicated flash
// data partition (labelled "wad"), instead of a filesystem. Keeps the
// stdc_wad_file class name so w_file.c needs no change.
//
// The partition is MEMORY-MAPPED once at open: reads are then plain memcpy
// through the flash cache. This matters beyond speed — raw esp_partition_read
// does SPI1 transactions that briefly disable the cache, which is forbidden
// from a task whose stack lives in PSRAM (the Doom task's does, since the BT
// controller ate the internal RAM the stack used to occupy).
//
// The path argument from D_DoomMain (-iwad ...) is ignored; the WAD is
// whatever was flashed into the "wad" partition.
//

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_partition.h"
#include "esp_log.h"

#include "w_file.h"
#include "z_zone.h"

static const char *TAG = "wad";

// Filled by doom_wad_map_init(), which main calls BEFORE the Doom task
// starts: esp_partition_mmap reprograms the flash MMU under a frozen cache,
// which is only legal from a task with an internal-RAM stack (the Doom
// task's stack is in PSRAM).
static const void *s_wad_map;
static size_t s_wad_size;

void doom_wad_map_init(void)
{
    const esp_partition_t *part =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                 ESP_PARTITION_SUBTYPE_ANY, "wad");
    if (part == NULL)
    {
        ESP_LOGE(TAG, "no 'wad' partition found");
        return;
    }

    esp_partition_mmap_handle_t map_h;
    esp_err_t err = esp_partition_mmap(part, 0, part->size,
                                       ESP_PARTITION_MMAP_DATA, &s_wad_map, &map_h);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "mmap of 'wad' partition failed: %s", esp_err_to_name(err));
        return;
    }
    s_wad_size = part->size;

    ESP_LOGI(TAG, "IWAD partition '%s' @0x%08" PRIx32 " size %" PRIu32 " mapped at %p",
             part->label, (uint32_t)part->address, (uint32_t)part->size, s_wad_map);
}

typedef struct
{
    wad_file_t wad;
    const void *map;                    // partition contents, cache-mapped
    size_t size;
} part_wad_file_t;

extern wad_file_class_t stdc_wad_file;

static wad_file_t *W_Part_OpenFile(char *path)
{
    if (s_wad_map == NULL)
    {
        ESP_LOGE(TAG, "WAD not mapped (doom_wad_map_init missing?), path '%s'", path);
        return NULL;
    }

    part_wad_file_t *result = Z_Malloc(sizeof(part_wad_file_t), PU_STATIC, 0);
    result->wad.file_class = &stdc_wad_file;
    result->wad.mapped = NULL;
    result->wad.length = s_wad_size;
    result->map = s_wad_map;
    result->size = s_wad_size;

    return &result->wad;
}

static void W_Part_CloseFile(wad_file_t *wad)
{
    // mapping stays for the app's lifetime (Doom never really closes the IWAD)
    Z_Free(wad);
}

size_t W_Part_Read(wad_file_t *wad, unsigned int offset,
                   void *buffer, size_t buffer_len)
{
    part_wad_file_t *pw = (part_wad_file_t *)wad;

    if (offset >= pw->size)
        return 0;
    if (offset + buffer_len > pw->size)
        buffer_len = pw->size - offset;

    memcpy(buffer, (const uint8_t *)pw->map + offset, buffer_len);
    return buffer_len;
}

wad_file_class_t stdc_wad_file =
{
    W_Part_OpenFile,
    W_Part_CloseFile,
    W_Part_Read,
};
