if (TARGET usb_device)
    PROJECT(pico_1bit_dac_v2)

    add_executable(pico_1bit_dac_v2
		main.c
		bsp.c
		dsp.c
		i2s_rx.c
		pdm_output.c
		simple_queue.c
		usb_audio.c
        )

		pico_generate_pio_header(pico_1bit_dac_v2 ${CMAKE_CURRENT_LIST_DIR}/pio_pwm_4bit.pio)
		pico_generate_pio_header(pico_1bit_dac_v2 ${CMAKE_CURRENT_LIST_DIR}/pio_pwm_5bit.pio)
		pico_generate_pio_header(pico_1bit_dac_v2 ${CMAKE_CURRENT_LIST_DIR}/pio_pwm_6bit.pio)

		pico_generate_pio_header(pico_1bit_dac_v2 ${CMAKE_CURRENT_LIST_DIR}/i2s_rx.pio)


		target_compile_definitions(pico_1bit_dac_v2 PRIVATE

            # ours are zero based, so say so
            PICO_USBDEV_USE_ZERO_BASED_INTERFACES=1

            # need large descriptor
            PICO_USBDEV_MAX_DESCRIPTOR_SIZE=256

            # Endpoint Buffer Size[Byte]; _TYPE=1:256,2:512,3:1024
            PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE=3
            PICO_USBDEV_ENABLE_DEBUG_TRACE

    )

    target_link_libraries(
            pico_1bit_dac_v2
            pico_stdlib
            usb_device
            pico_multicore
            hardware_interp
            hardware_pio)
            
    pico_add_extra_outputs(pico_1bit_dac_v2)
endif()
