add_executable(${TARGET_NAME}
    main.c
    mpu6050.c
    core1_main.c
    vec.c
)

target_include_directories(${TARGET_NAME}
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

# Import pico_stdlib
target_link_libraries(${TARGET_NAME}
    pico_stdlib
    pico_multicore
    hardware_i2c
    hardware_gpio
    hardware_sync
    hardware_pwm
)

# enable usb
pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(${TARGET_NAME})
