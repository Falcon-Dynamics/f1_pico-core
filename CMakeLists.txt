cmake_minimum_required(VERSION 3.12)

# Define project
project(f1_pico-core)

# Set Pico SDK path
set(PICO_SDK_PATH "~/pico/pico-sdk")

# Initialize the Raspberry Pi Pico SDK
include(pico_sdk_import.cmake)
pico_sdk_init()

# Set FreeRTOS Kernel path
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)

# Import FreeRTOS Kernel library
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Set any variables required for importing libraries

# Add ADXL345 library
add_library(pico_ADXL345 lib/ADXL345_RP2040/ADXL345.cpp lib/ADXL345_RP2040/ADXL345.h)

# Link libraries to pico_ADXL345
target_link_libraries(pico_ADXL345 pico_stdlib hardware_i2c)

# Add your main project's executable
add_executable(${PROJECT_NAME} src/main.cpp)

# Link pico_ADXL345 library to your project
target_link_libraries(${PROJECT_NAME} pico_ADXL345)

# Link FreeRTOS Kernel library
target_link_libraries(${PROJECT_NAME} FreeRTOS-Kernel)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Add subdirectories if needed
# add_subdirectory(src)
