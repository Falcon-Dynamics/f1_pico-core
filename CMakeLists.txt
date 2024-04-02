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
# add nanopb import
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/nanopb/extra)

# Add libraries
add_subdirectory(lib/ADXL345)
add_subdirectory(lib/MPL3115A2)
add_subdirectory(lib/no-OS-FatFs/src)
add_subdirectory(lib/RadioLib)


# Add subdirectory of main
add_subdirectory(src)

