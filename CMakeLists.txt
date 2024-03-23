cmake_minimum_required(VERSION 3.12)

# Define project
project(f1_pico-core)

SET(ProjectName f1_pico-core)

# Set Pico SDK path
set(PICO_SDK_PATH "~/pico/pico-sdk")

# Initialize the Raspberry Pi Pico SDK
include(pico_sdk_import.cmake)
pico_sdk_init()

add_subdirectory(lib/ADXL345_RP2040)

# Set FreeRTOS Kernel path
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)

# Import FreeRTOS Kernel library
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Set any variables required for importing libraries

# Add ADXL345 library
target_link_libraries(${ProjectName} ADXL345)

# Link FreeRTOS Kernel library
target_link_libraries(${PROJECT_NAME} FreeRTOS-Kernel)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Add subdirectories if needed
add_subdirectory(src)

