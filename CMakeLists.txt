cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(apple1_bus C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()


# Initialize the SDK
pico_sdk_init()


# Add executable
add_executable(apple1_bus
        main.c
        6821_emulator/e6821.c
        )

target_include_directories(apple1_bus PRIVATE 
        .
        ./6821_emulator
        )

# enable usb output, disable uart output
pico_enable_stdio_usb(apple1_bus 1)
pico_enable_stdio_uart(apple1_bus 0)

# pull in common dependencies
target_link_libraries(apple1_bus PUBLIC
        pico_stdlib 
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(apple1_bus)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )
