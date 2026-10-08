/* B&W internal-SRAM PAL display for the OTA loader (see ota_display.c: the
 * framebuffer lives in internal SRAM, so scan-out survives flash-write cache
 * suspension untouched). v2: 4 MHz pclk, 208 x 280 active, 5x7 text. */
#pragma once

#include <stdbool.h>

/* Drawable SAFE area: the full 208 x 280 active region reaches well into a
 * CRT's overscan (verified on a real set 2026-09-14), so the drawing API is
 * clipped to a centred window with ~12%% / 10%% margins per edge. (0,0) is
 * the top-left of the SAFE area; the border stays black. */
#define OTA_DISP_W 160            /* safe-area pixels per line */
#define OTA_DISP_H 224            /* safe-area lines           */
#define OTA_FONT_W 6              /* 5 px glyph + 1 px space  */
#define OTA_FONT_H 8              /* 7 px glyph + 1 px lead   */

/* Bring up the monochrome PAL output (requires the 48 MHz carrier and mixer
 * rail to already be on). Non-fatal on failure: loader runs headless. */
void ota_display_init(void);

/* Primitives; coordinates in active-area pixels, (0,0) top-left. All safe
 * any time after init -- internal SRAM is DMA-coherent, no sync needed. */
void ota_display_clear(void);
void ota_display_rect(int x, int y, int w, int h, bool white);
void ota_display_text(int x, int y, const char *s, bool invert);

/* Progress bar at a fixed spot near the bottom, 0..100; -1 hides it. */
void ota_display_progress(int percent);
