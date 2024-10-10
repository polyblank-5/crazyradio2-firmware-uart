#!/usr/bin/env bash
nrfutil="/home/polyblank/Programms/nrfutil"
BUILD='build_uart_dongle' 
PROJECT="crazyradio2"
$nrfutil pkg generate --hw-version 52 --sd-req=0x00 \
                     --application $BUILD/zephyr/$PROJECT.hex \
                     --application-version 1 $BUILD/zephyr/$PROJECT.zip

echo Using NRF DFU bootloader on /dev/ttyACM0
$nrfutil dfu usb-serial -pkg $BUILD/zephyr/$PROJECT.zip -p /dev/ttyACM0