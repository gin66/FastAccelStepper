# ESP-IDF component registration for the Saleae test app.
#
# Provided explicitly so PlatformIO does not auto-generate a component file that
# globs src/*.* (which would also pick up the headers and the CMakeLists.txt).
# The three sources are symlinked into the build's src/ by build-pio-dirs.sh.

idf_component_register(SRCS
    "saleae_app.cpp"
    "saleae_test.cpp"
    "saleae_hal_espidf.cpp")
