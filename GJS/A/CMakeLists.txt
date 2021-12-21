if (TARGET tinyusb_device)
    add_executable(A
    A.c
            )

    # pull in common dependencies
    target_link_libraries(A pico_stdlib pico_multicore)

    # enable usb output, disable uart output
    pico_enable_stdio_usb(A 1)
    pico_enable_stdio_uart(A 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(A)

    # add url via pico_set_program_url
    example_auto_set_url(A)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building A because TinyUSB submodule is not initialized in the SDK")
endif()
