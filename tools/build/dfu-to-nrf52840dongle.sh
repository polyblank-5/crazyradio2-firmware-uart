#!/usr/bin/env bash
nrfutil="/home/polyblank/Programms/nrfutil"
BUILD='build_blinky_dongle' 
$nrfutil pkg generate --hw-version 52 --sd-req=0x00 \
                     --application $BUILD/zephyr/crazyradio2.hex \
                     --application-version 1 $BUILD/zephyr/crazyradio2.zip

echo Using NRF DFU bootloader on /dev/ttyACM0
$nrfutil dfu usb-serial -pkg $BUILD/zephyr/crazyradio2.zip -p /dev/ttyACM0