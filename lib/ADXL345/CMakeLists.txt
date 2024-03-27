# Define library
add_library(ADXL345
        ADXL345.cpp
        ADXL345.h
)

target_link_libraries(ADXL345 pico_stdlib hardware_i2c)

# Include directories
target_include_directories(ADXL345 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
