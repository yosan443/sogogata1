cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

# We also need PICO EXTRAS
include(../pico-playground/pico_extras_import.cmake)

#project(pico_examples C CXX ASM)
project(cq_raspi_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# Initialize the SDK
pico_sdk_init()

include(example_auto_set_url.cmake)

#add_subdirectory(pico_1bit_dac)	# First Version, fs=48kHz Only, 1-bit 2nd Order Delta-Sigma
#add_subdirectory(pico_1bit_dac_4844)	# fs=44.1k/48kHz Support
#add_subdirectory(pico_1bit_dac_NR)	# Noise Reduced, 4-bit 3rd Order Delta-Sigma
#add_subdirectory(pico_1bit_dac_HR)	# HiReso support
#add_subdirectory(pico_1bit_dac_HR2)	# HiReso support 2nd
add_subdirectory(pico_1bit_dac_v2)	# for pico_1bit_dac/pico_diydac_v2 board

target_compile_definitions(pico_1bit_dac_v2 PRIVATE
    PICO_AUDIO_L_PIN=37
    PICO_AUDIO_R_PIN=39
)
