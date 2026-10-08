//
// i_fmsound.c -- SFX sound module for tv-doom.
//
// A minimal digital sound-effects mixer (no SDL, no music). It mixes the active
// DMX sound lumps down to a single mono 16-bit stream and hands each slice to a
// PCM "sink" registered by the platform. In tv-doom the sink is the FM sound
// subcarrier generator (main/sound.c), so game audio rides the PAL intercarrier.
//
// The library never references the sink directly -- the platform registers it via
// DG_SetPcmSink() -- so the main<->doomgeneric dependency stays one-way (see the
// component CMakeLists note).
//
// Mixing is driven from I_UpdateSound() (once per frame). Each call emits exactly
// as many samples as real time has advanced (esp_timer), so the average output
// rate is MIXRATE regardless of frame jitter, matching the sink's drain rate.
//

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "i_sound.h"
#include "w_wad.h"
#include "z_zone.h"
#include "doomtype.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#define MIXRATE     11025      // must equal SOUND_INPUT_RATE in main/sound.c
#define NCHAN       8          // simultaneous sfx channels Doom mixes
#define MAXSLICE    768        // cap samples per Update (~70 ms) to bound the burst

typedef struct {
    const uint8_t *data;       // unsigned 8-bit PCM (into the mmap'd WAD lump)
    uint32_t len;              // sample count
    uint32_t pos;             // Q16 playback position
    uint32_t step;             // Q16 increment = sfx_rate / MIXRATE
    int      vol;              // 0..127
    boolean32 active;
} chan_t;

static chan_t   chan[NCHAN];
static boolean32 use_prefix;
static void    (*s_sink)(const int16_t *, size_t);
static uint64_t last_us;
static int16_t *s_mixbuf;      // MAXSLICE samples, in PSRAM (internal RAM is scarce)

// Registered by the platform (main) to receive mixed mono PCM at MIXRATE.
void DG_SetPcmSink(void (*sink)(const int16_t *, size_t))
{
    s_sink = sink;
}

static boolean32 I_FM_Init(boolean32 use_sfx_prefix)
{
    use_prefix = use_sfx_prefix;
    memset(chan, 0, sizeof(chan));
    if (!s_mixbuf)
        s_mixbuf = heap_caps_malloc(MAXSLICE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    return s_mixbuf != NULL;
}

static void I_FM_Shutdown(void) { }

static int I_FM_GetSfxLumpNum(sfxinfo_t *sfx)
{
    char buf[9];
    if (use_prefix) snprintf(buf, sizeof(buf), "ds%s", sfx->name);
    else            snprintf(buf, sizeof(buf), "%s",   sfx->name);
    return W_GetNumForName(buf);
}

static int I_FM_StartSound(sfxinfo_t *sfx, int channel, int vol, int sep)
{
    if (channel < 0 || channel >= NCHAN) return -1;

    int lump = sfx->lumpnum > 0 ? sfx->lumpnum : I_FM_GetSfxLumpNum(sfx);
    const uint8_t *l = W_CacheLumpNum(lump, PU_STATIC);   // mmap'd: stable pointer
    if (!l) return -1;

    // DMX header: u16 format(=3), u16 sample_rate, u32 length, then 8-bit PCM.
    uint32_t rate   = (uint32_t)l[2] | ((uint32_t)l[3] << 8);
    uint32_t length = (uint32_t)l[4] | ((uint32_t)l[5] << 8) |
                      ((uint32_t)l[6] << 16) | ((uint32_t)l[7] << 24);
    uint32_t avail  = (uint32_t)W_LumpLength(lump);
    if (rate == 0) rate = 11025;
    if (length + 8 > avail) length = (avail > 8) ? avail - 8 : 0;

    chan[channel].data   = l + 8;
    chan[channel].len    = length;
    chan[channel].pos    = 0;
    chan[channel].step   = (uint32_t)(((uint64_t)rate << 16) / MIXRATE);
    chan[channel].vol    = vol;
    chan[channel].active = true;
    return channel;
}

static void I_FM_StopSound(int channel)
{
    if (channel >= 0 && channel < NCHAN) chan[channel].active = false;
}

static boolean32 I_FM_SoundIsPlaying(int channel)
{
    return (channel >= 0 && channel < NCHAN) ? chan[channel].active : false;
}

static void I_FM_UpdateSoundParams(int channel, int vol, int sep)
{
    if (channel >= 0 && channel < NCHAN) chan[channel].vol = vol;   // sep ignored (mono)
}

// Mix `n` samples of all active channels into buf (clamped mono 16-bit).
static void mix(int16_t *buf, int n)
{
    for (int i = 0; i < n; i++) {
        int acc = 0;
        for (int c = 0; c < NCHAN; c++) {
            if (!chan[c].active) continue;
            uint32_t idx = chan[c].pos >> 16;
            if (idx >= chan[c].len) { chan[c].active = false; continue; }
            int s = ((int)chan[c].data[idx] - 128) << 8;   // u8 -> +/-32768
            acc += (s * chan[c].vol) >> 7;                 // scale by vol/128
            chan[c].pos += chan[c].step;
        }
        if (acc >  32767) acc =  32767;
        if (acc < -32768) acc = -32768;
        buf[i] = (int16_t)acc;
    }
}

static void I_FM_Update(void)
{
    if (!s_sink) return;

    uint64_t now = esp_timer_get_time();
    if (!last_us) { last_us = now; return; }

    int64_t due = (int64_t)((now - last_us) * MIXRATE / 1000000);
    if (due <= 0) return;
    if (due > MAXSLICE) { due = MAXSLICE; last_us = now; }   // stall: drop backlog
    else last_us += (uint64_t)due * 1000000 / MIXRATE;

    mix(s_mixbuf, (int)due);
    s_sink(s_mixbuf, (size_t)due);
}

static snddevice_t fm_devices[] = {
    SNDDEVICE_SB, SNDDEVICE_PAS, SNDDEVICE_GUS,
    SNDDEVICE_WAVEBLASTER, SNDDEVICE_SOUNDCANVAS, SNDDEVICE_AWE32,
};

sound_module_t sound_fm_module = {
    fm_devices,
    sizeof(fm_devices) / sizeof(fm_devices[0]),
    I_FM_Init,
    I_FM_Shutdown,
    I_FM_GetSfxLumpNum,
    I_FM_Update,
    I_FM_UpdateSoundParams,
    I_FM_StartSound,
    I_FM_StopSound,
    I_FM_SoundIsPlaying,
    NULL,                  // CacheSounds: not needed (WAD is mmap'd)
};
