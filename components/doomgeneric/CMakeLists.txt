# doomgeneric (lilka embedded fork) as an ESP-IDF component.
# All SDL/platform code in these sources is behind #ifdef ORIGCODE, so the
# stock tree compiles SDL-free. The ESP32 platform hooks (DG_*) live in
# platform/doomgeneric_esp32.c in this same component so they share one
# static archive with their callers (avoids a circular main<->lib link).

file(GLOB DG_SRCS "src/*.c")

idf_component_register(
    SRCS ${DG_SRCS} "platform/doomgeneric_esp32.c"
    INCLUDE_DIRS "src"
    REQUIRES esp_partition spi_flash esp_timer video
)

# Vendored 1990s C: silence its warnings rather than patch every file, and make
# sure IDF's -Werror=implicit-function-declaration can't turn a warning fatal.
# Applied to the whole component; the small platform file is warning-clean.
# -O3/-funroll-loops: no change in AVERAGE fps vs the global -O2 (+70KB code),
# but kept for frame-time consistency — user observed less stutter with it
# (plausible: unrolling trims worst-case frames even when the mean is flat).
target_compile_options(${COMPONENT_LIB} PRIVATE
    -w
    -Wno-error=implicit-function-declaration
    -Wno-implicit-function-declaration
    -Dstricmp=strcasecmp
    -Dstrnicmp=strncasecmp
    -O3
    -funroll-loops
)
