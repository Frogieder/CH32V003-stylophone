# CMake file to compile and flash projects with ch32fun
#
# This is an early version, some refactoring is needed to simplify configuration and adding new C files.
# Testing is also needed for boards other than CH32V002 and CH32VOO3
# The

cmake_minimum_required(VERSION 3.16)

# Early toolchain setup - must be before project()
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR riscv)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Find RISC-V toolchain
find_program(RISCV_GCC
        NAMES
        riscv64-unknown-elf-gcc
        riscv-none-elf-gcc
        riscv32-unknown-elf-gcc
        riscv64-elf-gcc
        riscv32-elf-gcc
        REQUIRED
)

if(NOT RISCV_GCC)
    message(FATAL_ERROR "No RISC-V toolchain found!")
endif()

# Extract toolchain prefix
get_filename_component(TOOLCHAIN_BIN_DIR ${RISCV_GCC} DIRECTORY)
get_filename_component(TOOLCHAIN_PREFIX ${RISCV_GCC} NAME_WE)
string(REGEX REPLACE "-gcc$" "" TOOLCHAIN_PREFIX ${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX})

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}-ar)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}-size)

# Skip compiler tests
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

project(STYLOPHONE C ASM)

#########################
# Configuration options #
#########################
set(TARGET "main" CACHE STRING "Target name without extension")
set(TARGET_EXTRAS
#        my_file.c
        CACHE STRING "Extra C files in src")
set(TARGET_EXT "c" CACHE STRING "File extension of target source file")
set(TARGET_MCU "CH32V003" CACHE STRING "MCU model")
set(MINICHLINK_PATH "${CMAKE_SOURCE_DIR}/ch32fun/minichlink" CACHE PATH "Path to minichlink")
set(CH32FUN_PATH "${CMAKE_SOURCE_DIR}/ch32fun/ch32fun" CACHE PATH "Path to ch32fun")
set(WRITE_SECTION "flash" CACHE STRING "Flash section")
set(MCU_PACKAGE "1" CACHE STRING "MCU package")

# Determine architecture and flags based on MCU
if(TARGET_MCU MATCHES "CH32V003")
    set(ARCH_FLAGS -march=rv32ec -mabi=ilp32e)
    set(MCU_DEFINES "-DCH32V003")
    set(TARGET_MCU_LD "0")
elseif(TARGET_MCU MATCHES "CH32V002")
    # Check GCC version for zmmul support
    execute_process(
            COMMAND ${CMAKE_C_COMPILER} -dumpversion
            OUTPUT_VARIABLE GCC_VERSION
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(GCC_VERSION VERSION_GREATER_EQUAL "13.0")
        set(ARCH_FLAGS -march=rv32ec_zmmul -mabi=ilp32e)
    else()
        set(ARCH_FLAGS -march=rv32ec -mabi=ilp32e)
    endif()
    set(MCU_DEFINES -DCH32V00x=1 -DCH32V002)
    set(TARGET_MCU_LD "5")
elseif(TARGET_MCU MATCHES "CH32V004")
    set(ARCH_FLAGS -march=rv32ec -mabi=ilp32e)
    set(MCU_DEFINES -DCH32V00x=1 -DCH32V004)
    set(TARGET_MCU_LD "6")
elseif(TARGET_MCU MATCHES "CH32V005|CH32V006|CH32V007")
    set(ARCH_FLAGS -march=rv32ec -mabi=ilp32e)
    set(MCU_DEFINES -DCH32V00x=1 -D${TARGET_MCU})
    set(TARGET_MCU_LD "7")
elseif(TARGET_MCU MATCHES "CH32V103")
    set(ARCH_FLAGS -march=rv32imac -mabi=ilp32)
    set(MCU_DEFINES -DCH32V10x=1)
    set(TARGET_MCU_LD "1")
elseif(TARGET_MCU MATCHES "CH32V203")
    set(ARCH_FLAGS -march=rv32imac -mabi=ilp32)
    set(MCU_DEFINES -DCH32V20x=1)
    set(TARGET_MCU_LD "2")
elseif(TARGET_MCU MATCHES "CH32V307")
    set(ARCH_FLAGS -march=rv32imac -mabi=ilp32)
    set(MCU_DEFINES -DCH32V30x=1 -DTARGET_MCU_MEMORY_SPLIT=3)
    set(TARGET_MCU_LD "3")
    set(TARGET_MCU_MEMORY_SPLIT "3")
else()
    message(FATAL_ERROR "Unsupported MCU: ${TARGET_MCU}")
endif()

# Set default memory split if not set
if(NOT DEFINED TARGET_MCU_MEMORY_SPLIT)
    set(TARGET_MCU_MEMORY_SPLIT "0")
endif()

# Generate linker script
set(GENERATED_LD "${CMAKE_BINARY_DIR}/generated.ld")
add_custom_command(
        OUTPUT ${GENERATED_LD}
        COMMAND ${CMAKE_C_COMPILER} -E -P -x c
        -DTARGET_MCU=${TARGET_MCU}
        -DMCU_PACKAGE=${MCU_PACKAGE}
        -DTARGET_MCU_LD=${TARGET_MCU_LD}
        -DTARGET_MCU_MEMORY_SPLIT=${TARGET_MCU_MEMORY_SPLIT}
        ${CH32FUN_PATH}/ch32fun.ld > ${GENERATED_LD}
        DEPENDS ${CH32FUN_PATH}/ch32fun.ld
        COMMENT "Generating linker script"
        VERBATIM
)

# Include directories
set(INCLUDE_DIRS
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CH32FUN_PATH}
        ${CH32FUN_PATH}/../misc
        ${CH32FUN_PATH}/../extralibs
)

# newlib is placed in a different directory in Fedora systems
if(EXISTS "/etc/fedora-release")
    list(APPEND INCLUDE_DIRS "/usr/arm-none-eabi/include")
elseif(EXISTS "/usr/include/newlib")
    list(APPEND INCLUDE_DIRS "/usr/include/newlib")
endif()

# Compiler flags
set(COMMON_FLAGS ${ARCH_FLAGS} ${MCU_DEFINES} -Os -g -Wall -ffunction-sections -fdata-sections -fno-common -fstack-usage -fno-builtin-printf -fno-builtin-sprintf)

# Sources
set(SOURCES
        ${CH32FUN_PATH}/ch32fun.c
        ${CMAKE_SOURCE_DIR}/src/${TARGET}.${TARGET_EXT}
)

# Add extra sources
foreach(extra ${TARGET_EXTRAS})
    list(APPEND SOURCES ${CMAKE_SOURCE_DIR}/src/${extra})
endforeach()

# Create executable
add_executable(${TARGET}.elf ${SOURCES} ${GENERATED_LD})

# Set compile options
target_compile_options(${TARGET}.elf PRIVATE ${COMMON_FLAGS})
target_include_directories(${TARGET}.elf PRIVATE ${INCLUDE_DIRS})

# Find libgcc
execute_process(
        COMMAND ${CMAKE_C_COMPILER} ${ARCH_FLAGS} -print-libgcc-file-name
        OUTPUT_VARIABLE LIBGCC_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
)
set(LIBGCC_PATH ${CH32FUN_PATH}/../misc/libgcc.a)
get_filename_component(LIBGCC_DIR ${LIBGCC_PATH} DIRECTORY)

set(PROJECT_LIBGCC "${CH32FUN_PATH}/../misc/libgcc.a")

target_link_libraries(${TARGET}.elf PRIVATE ${LIBGCC_PATH})


# Linker flags
target_link_options(${TARGET}.elf PRIVATE
        ${ARCH_FLAGS}
        -nostartfiles
        -nostdlib
        -Wl,-T,${GENERATED_LD}
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        -Wl,-Map=${CMAKE_BINARY_DIR}/${TARGET}.map
        -Wl,--start-group
        -L${LIBGCC_DIR}
        -lgcc
        -Wl,--end-group
)

# Generate other binary file formats and print memory usage
add_custom_command(TARGET ${TARGET}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET}.elf ${TARGET}.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${TARGET}.elf ${TARGET}.hex
        COMMAND ${CMAKE_OBJDUMP} -S ${TARGET}.elf > ${TARGET}.lst
        COMMAND ${CMAKE_SIZE} ${TARGET}.elf
        COMMENT "Generating binary, hex, lst files and size info"
)

# Utility targets
add_custom_target(flash
        COMMAND ${MINICHLINK_PATH}/minichlink -w ${CMAKE_BINARY_DIR}/${TARGET}.bin ${WRITE_SECTION} -b
        DEPENDS ${TARGET}.elf
        COMMENT "Flashing ${TARGET}.bin to ${WRITE_SECTION}"
)

add_custom_target(minichlink
        COMMAND ${CMAKE_MAKE_PROGRAM} -C ${MINICHLINK_PATH} all
        COMMENT "Building minichlink"
)

add_custom_target(unbrick
        COMMAND ${MINICHLINK_PATH}/minichlink -u
        COMMENT "Unbricking device"
)

add_custom_target(monitor
        COMMAND ${MINICHLINK_PATH}/minichlink -T
        COMMENT "Starting monitor"
)

add_custom_target(gdbserver
        COMMAND ${MINICHLINK_PATH}/minichlink -baG
        COMMENT "Starting GDB server"
)

# Clean target
add_custom_target(clean_all
        COMMAND ${CMAKE_COMMAND} -E remove
        ${TARGET}.elf ${TARGET}.bin ${TARGET}.hex
        ${TARGET}.map ${TARGET}.lst ${GENERATED_LD}
        COMMENT "Cleaning all generated files"
)

# Print configuration
message(STATUS "=====================================")
message(STATUS "Target: ${TARGET}")
message(STATUS "MCU: ${TARGET_MCU}")
message(STATUS "Toolchain: ${TOOLCHAIN_PREFIX}")
message(STATUS "Architecture flags: ${ARCH_FLAGS}")
message(STATUS "libgcc location: ${LIBGCC_PATH}")
message(STATUS "=====================================")
