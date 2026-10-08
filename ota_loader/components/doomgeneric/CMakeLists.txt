# Header-only shim standing in for the real doomgeneric component: the
# gamepad component needs doomkeys.h, and the loader must NOT link Doom.
# The doom_* symbols gamepad.c references are stubbed in ota_menu.c.
idf_component_register(INCLUDE_DIRS include)
