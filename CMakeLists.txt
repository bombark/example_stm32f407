cmake_minimum_required(VERSION 3.14)
project(target-cortexm LANGUAGES C ASM)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

include(toolchain/stm32f407.cmake)
include_directories(./include)
include_directories(./stm32cube/STM32F4xx_HAL_Driver/Inc/)


include_directories(./stm32cube/Core/Inc/)
include_directories(./stm32cube/CMSIS/Include/)
include_directories(./stm32cube/CMSIS/Device/ST/STM32F4xx/Include/)
include_directories(./stm32cube/Drivers/CMSIS/Include/)
include_directories(./stm32cube/Drivers/CMSIS/Device/ST/STM32F4xx/Include/)
include_directories(./stm32cube/Drivers/STM32F4xx_HAL_Driver/Inc)
include_directories(./stm32cube/Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc/)
include_directories(./stm32cube/Middlewares/ST/STM32_USB_Host_Library/Core/Inc/)
include_directories(./stm32cube/USB_DEVICE/App/)
include_directories(./stm32cube/USB_DEVICE/Target/)
include_directories(./stm32cube/Middlewares/ST/STM32_USB_Device_Library/Core/Inc/)
include_directories(./stm32cube/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc/)


add_executable(firmware.elf
    ## copied from stm32cube
    ./stm32cube/USB_DEVICE/Target/usbd_conf.c
    ./stm32cube/USB_DEVICE/App/usbd_desc.c
    ./stm32cube/USB_DEVICE/App/usbd_cdc_if.c
    ./stm32cube/USB_DEVICE/App/usb_device.c
    ./stm32cube/Core/Src/sysmem.c
    ./stm32cube/Core/Src/syscalls.c
    ./stm32cube/Core/Src/stm32f4xx_hal_msp.c
    ./stm32cube/Core/Src/main.c
    ./stm32cube/Core/Src/system_stm32f4xx.c
    ./stm32cube/Core/Src/stm32f4xx_it.c
    ./stm32cube/Core/Startup/startup_stm32f407vgtx.s
    ./stm32cube/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    ./stm32cube/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ./stm32cube/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ./stm32cube/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac_ex.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ./stm32cube/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac.c

    ## project
    ./src/btn_blue.c
    ./src/led_orange.c
    ./src/speaker.c
    ./src/led_blue.c
    ./src/rtc.c
    ./src/uart.c
    ./src/led_green.c
    ./src/led_red.c
    ./src/main_system.c
)

add_custom_target (firmware.bin
  DEPENDS firmware.elf
  COMMAND arm-none-eabi-objcopy -O binary firmware.elf firmware.bin
)
add_custom_target(bin ALL DEPENDS firmware.bin)
