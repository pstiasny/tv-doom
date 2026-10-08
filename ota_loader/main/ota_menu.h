/* Gamepad-driven WiFi-join + pull-update menu on the B&W display. */
#pragma once

/* Bring up WiFi (STA), input queue and the menu task. Requires: display up,
 * gamepad_start() called, NVS initialized. */
void ota_menu_start(void);

/* Running STAGED from ota_0 (a loader update pulled by the old loader as the
 * "game image"): copy ourselves into the factory slot, erase otadata (= boot
 * factory) and restart. Needs the display up; never returns. */
void ota_menu_install_to_factory(void);
