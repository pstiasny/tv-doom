# PAL composite video output engine (ported from tv-plug). Drives the RGB-LCD
# peripheral into a 4-bit R-2R DAC on GPIO 11-14. See video.h for the API.
idf_component_register(
    SRCS "video.c"
    INCLUDE_DIRS "."
    REQUIRES esp_lcd esp_mm soc esp_timer
)

# The encode inner loop is the hottest code in the system; give it the extra
# optimisation the IDF-wide -O2 doesn't (unrolling amortises loop overhead over
# the 230-cycle line loop).
target_compile_options(${COMPONENT_LIB} PRIVATE -O3 -funroll-loops)
