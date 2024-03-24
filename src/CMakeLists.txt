add_executable(${ProjectName}
        main.cpp
)

target_include_directories(${ProjectName} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib)

target_link_libraries(f1_pico-core
        pico_stdlib
        FreeRTOS-Kernel-Heap4
        ADXL345
        no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
)

pico_add_extra_outputs(${ProjectName})