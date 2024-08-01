#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>


#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0)); 
//https://docs.zephyrproject.org/latest/kernel/drivers/index.html#c.DEVICE_DT_GET

K_MSGQ_DEFINE(command_queue, sizeof(struct usb_command), 10, 4);
#define MSG_SIZE 64
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);


void serial_cb(const struct device *dev, void *user_data);
