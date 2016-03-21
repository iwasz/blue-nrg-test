#!/bin/bash

# Run from build
CUBE_ROOT="/home/iwasz/workspace/STM32Cube_FW_F7_V1.3.0"

cmake -DCMAKE_BUILD_TYPE='Debug' \
-DDEVICE='STM32F746xx' \
-DCUBE_ROOT=$CUBE_ROOT \
-DCRYSTAL_HZ=16000000 \
-DSTARTUP_CODE="$CUBE_ROOT/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/gcc/startup_stm32f746xx.s" \
-DLINKER_SCRIPT="$CUBE_ROOT/Projects/STM32746G-Discovery/Templates/SW4STM32/STM32746G_Discovery(AXIM-FLASH)/STM32F746NGHx_FLASH.ld" \
-DUSB_LIB="$CUBE_ROOT/Middlewares/ST/STM32_USB_Device_Library/" \
..
