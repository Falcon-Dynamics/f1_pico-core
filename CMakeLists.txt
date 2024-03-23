cmake_minimum_required(VERSION 3.12)

# Define project
project(f1_pico-core)

SET(ProjectName f1_pico-core)

# Set Pico SDK path
set(PICO_SDK_PATH "~/pico/pico-sdk")

# Initialize the Raspberry Pi Pico SDK
include(pico_sdk_import.cmake)
pico_sdk_init()

# Set FreeRTOS Kernel path
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)
# Import FreeRTOS Kernel library
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Add libraries
add_subdirectory(lib/ADXL345_RP2040)

# Add subdirectory of main
add_subdirectory(src)

