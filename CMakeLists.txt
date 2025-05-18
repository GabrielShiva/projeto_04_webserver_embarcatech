cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(projeto_webserver_04 C CXX ASM)
pico_sdk_init()

add_executable(projeto_webserver_04
    main.c
    lib/ssd1306.c
    )

target_compile_definitions(${PROJECT_NAME} PRIVATE
        PICO_PRINTF_SUPPORT_FLOAT=1
        PICO_STDIO_ENABLE_PRINTF=1
    )

target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        pico_cyw43_arch_lwip_threadsafe_background
        hardware_adc
        hardware_i2c
      )

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(${PROJECT_NAME})
