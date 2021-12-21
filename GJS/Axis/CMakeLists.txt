if (TARGET tinyusb_device)
    add_executable(Axis
    Axis.c
            )

    # pull in common dependencies
    target_link_libraries(Axis pico_stdlib pico_multicore)

    # enable usb output, disable uart output
    pico_enable_stdio_usb(Axis 1)
    pico_enable_stdio_uart(Axis 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(Axis)

    # add url via pico_set_program_url
    example_auto_set_url(Axis)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building Axis because TinyUSB submodule is not initialized in the SDK")
endif()
