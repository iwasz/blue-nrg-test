#!/bin/bash

# Run from build

. ../conf.sh.inc

cmake -DCMAKE_BUILD_TYPE='Debug' \
-DDEVICE='STM32F407xx' \
-DCUBE_ROOT=$CUBE_ROOT \
-DCRYSTAL_HZ=16000000 \
-DSTARTUP_CODE="$CUBE_ROOT/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s" \
-DLINKER_SCRIPT="$CUBE_ROOT/Projects/STM32F4-Discovery/Templates/TrueSTUDIO/STM32F4-Discovery/STM32F407VG_FLASH.ld" \
-DUSB_LIB="$CUBE_ROOT/Middlewares/ST/STM32_USB_Device_Library/" \
..
