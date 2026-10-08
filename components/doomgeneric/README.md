# doomgeneric, vendored

This is the Doom engine as an ESP-IDF component. The lineage is:

1. The Doom source released by id Software in 1999, GPL-2.0-or-later.
2. Chocolate Doom, which cleaned it up and is the base of
3. doomgeneric by ozkl, which reduces the platform layer to a handful of
   `DG_*` callbacks, as forked by
4. the lilka project (ESP32-S3 handheld), which added PSRAM allocation for the
   large renderer arrays (`d_alloc.c`, `ps_malloc.c`) and other embedded
   fixes.

`src/` is that fork with the ESP32 platform code kept outside it. The exact
upstream revision was not recorded when the tree was copied.

## What is local to this project

- `platform/doomgeneric_esp32.c`: the `DG_*` hooks, wiring the engine to the
  video encoder, gamepad and timer. Lives in this component so that it links
  into the same static archive as its callers.
- `CMakeLists.txt`: builds the tree SDL-free (`ORIGCODE` is never defined),
  silences the 1990s warnings, and uses `-O3 -funroll-loops` for frame-time
  consistency.
- ESP32-specific edits inside `src/`, mainly in `w_file_stdc.c` (the IWAD is
  memory-mapped from a flash partition rather than read from a file),
  `i_fmsound.c`, `i_video.c`, `d_loop.c`, `r_main.c`, `r_draw.c` and
  `m_config.c`. Edits are marked with comments mentioning ESP32 or tv-doom.

Everything in this directory is GPL-2.0-or-later, see the repository
`LICENSE`.
