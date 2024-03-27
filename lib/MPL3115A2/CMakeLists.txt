# Define library
add_library(MPL3115A2
        MPL3115A2.cpp
        MPL3115A2.h
)

target_link_libraries(MPL3115A2 pico_stdlib hardware_i2c)

# Include directories
target_include_directories(MPL3115A2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
