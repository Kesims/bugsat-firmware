cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cansat-ble LANGUAGES C)

# Fetch all component source files
file(GLOB_RECURSE COMPONENT_SOURCES src/components/*.c)

target_sources(app PRIVATE
        src/main.c
        ${COMPONENT_SOURCES} # Include all component source files
)

# Include all component header files
target_include_directories(app PRIVATE src/components)

