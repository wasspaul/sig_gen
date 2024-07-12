cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(sig_gen C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
if (TARGET tinyusb_device)
	add_executable(sig_gen sig_gen.c)
	
	pico_generate_pio_header(sig_gen ${CMAKE_CURRENT_LIST_DIR}/manchester_encoding.pio)

	target_link_libraries(sig_gen pico_stdlib hardware_spi hardware_dma
	                      hardware_pwm pico_multicore pico_rand 
	                      hardware_adc hardware_uart 
						  pico_i2c_slave hardware_i2c hardware_pio)

	# enable usb output, disable uart output
	pico_enable_stdio_usb(sig_gen 1)
	pico_enable_stdio_uart(sig_gen 0)

	pico_add_extra_outputs(sig_gen)

elseif(PICO_ON_DEVICE)
message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()
