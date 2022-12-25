cmake_minimum_required(VERSION 3.12)

# convenience
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
set(PICO_FREERTOS_PATH $ENV{PICO_FREERTOS_PATH})

# Import pico SDK
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Import FreeRTOS
include(${PICO_FREERTOS_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(robocar_router C CXX ASM)
set(TARGET_NAME ${PROJECT_NAME})

# Initialize SDK
pico_sdk_init()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(src)
