include($ENV{PICO_SDK_PATH}/tools/CMakeLists.txt)

add_executable(${TARGET_NAME}
    main.c
    mpu6050.c
    velocity_handler.c
    vec3.c
)

pico_generate_pio_header(${TARGET_NAME}
    ${PROJECT_SOURCE_DIR}/pio/wheel_rev.pio
)

target_include_directories(${TARGET_NAME}
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}
)

# Import pico_stdlib
target_link_libraries(${TARGET_NAME}
    # freertos libs
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    # pico libs
    pico_stdlib
    pico_multicore
    hardware_i2c
    hardware_gpio
    hardware_pio
)

# enable usb
pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(${TARGET_NAME})
