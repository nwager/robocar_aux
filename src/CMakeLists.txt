add_executable(${TARGET_NAME}
    main.c
    mpu6050.c
    velocity_handler.c
    vec3.c
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
)

# enable usb
pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(${TARGET_NAME})
