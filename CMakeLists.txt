cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(pimp C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(pimp
    main.c
    msc_disk.c
    usb_descriptors.c
    fatfs/diskio.c
    fatfs/ff.c
)

pico_generate_pio_header(pimp ${CMAKE_CURRENT_LIST_DIR}/mk90.pio)

target_include_directories(pimp PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(
    pimp 
    pico_stdlib 
    tinyusb_device 
    hardware_flash
	hardware_sleep
    hardware_pio
    hardware_adc
    hardware_xosc
)

pico_set_binary_type(pimp copy_to_ram)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pimp)

add_custom_command(TARGET pimp
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/pimp.uf2 E:/pimp.uf2
)