#include <stdio.h>

#include "m_argv.h"

#include "doomgeneric.h"
#include "d_alloc.h"

pixel_t* DG_ScreenBuffer = 0;

void M_FindResponseFile(void);
void D_DoomMain (void);


void doomgeneric_Create(int argc, char **argv)
{
	// save arguments
    myargc = argc;
    myargv = argv;

	M_FindResponseFile();

	DG_ScreenBuffer = ps_malloc(DOOMGENERIC_RESX * DOOMGENERIC_RESY * sizeof(pixel_t));

	DG_Init();

	D_DoomMain ();
}

