# set CMAKE_SYSTEM_NAME to define build as CMAKE_CROSSCOMPILING
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION Cortex-M4-STM32F407)
set(CMAKE_SYSTEM_PROCESSOR arm)

# setting either of the compilers builds the absolute paths for the other tools:
#   ar, nm, objcopy, objdump, ranlib, readelf -- but not as, ld, size
# if the compiler cannot be found the try_compile() function will fail the build
# set(CMAKE_C_COMPILER arm-none-eabi-gcc)
# set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

find_program(CROSS_GCC_PATH "arm-none-eabi-gcc")
get_filename_component(TOOLCHAIN ${CROSS_GCC_PATH} PATH)

set(CMAKE_ASM_COMPILER ${TOOLCHAIN}/arm-none-eabi-gcc)
set(CMAKE_C_COMPILER ${TOOLCHAIN}/arm-none-eabi-gcc)
set(CMAKE_Cxx_COMPILER ${TOOLCHAIN}/arm-none-eabi-g++)
set(TOOLCHAIN_AS ${TOOLCHAIN}/arm-none-eabi-as CACHE STRING "arm-none-eabi-as")
set(TOOLCHAIN_LD ${TOOLCHAIN}/arm-none-eabi-ld CACHE STRING "arm-none-eabi-ld")
set(TOOLCHAIN_SIZE ${TOOLCHAIN}/arm-none-eabi-size CACHE STRING "arm-none-eabi-size")

# set(ARM_FLAGS )
set(CMAKE_C_FLAGS "-mcpu=cortex-m4 -std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")
 
add_definitions (
    -DDEBUG 
    -DUSE_HAL_DRIVER 
    -DSTM32F407xx
)

SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-T ../ldscripts/STM32F407VGTX_FLASH.ld")

#-Wl,-map=${TARGET}.map,--cref #-u _printf_float -u _scanf_float 


set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)