add_executable(${ProjectName}
        main.cpp
        hw_config.cpp
)

target_include_directories(${ProjectName} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib)

target_link_libraries(f1_pico-core
        pico_stdlib
        RadioLib
        hardware_gpio
        hardware_spi
        hardware_timer
        FreeRTOS-Kernel-Heap4
        ADXL345
        MPL3115A2
        no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
)

pico_enable_stdio_usb(${PROJECT_NAME} 1) # enable usb serial
pico_enable_stdio_uart(${PROJECT_NAME} 0) # disable uart

pico_add_extra_outputs(${ProjectName})