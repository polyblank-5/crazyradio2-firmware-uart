#pragma once


#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>

struct usb_command {
    char payload[64];
    uint32_t length;
};

int legacy_send(const struct usb_command *command);
int legacy_receive(struct usb_command *command);