#ifndef DOOM_GENERIC
#define DOOM_GENERIC

#include <stdlib.h>
#include <stdint.h>

#define DOOMGENERIC_RESX 320
#define DOOMGENERIC_RESY 200

// 8-bit INDEXED framebuffer: one byte per pixel, an index into Doom's 256-entry
// palette (exported post-gamma as DG_PaletteRGB by i_video.c). The platform
// layer resolves colours; nothing expands to RGB per-pixel inside the engine.
typedef uint8_t pixel_t;

extern pixel_t* DG_ScreenBuffer;

void doomgeneric_Create(int argc, char **argv);
void doomgeneric_Tick();


//Implement below functions for your platform
void DG_Init();
void DG_DrawFrame();
void DG_SleepMs(uint32_t ms);
uint32_t DG_GetTicksMs();
int DG_GetKey(int* pressed, unsigned char* key);
void DG_SetWindowTitle(const char * title);

#endif //DOOM_GENERIC
