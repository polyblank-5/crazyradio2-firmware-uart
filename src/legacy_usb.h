#pragma once
#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>


#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0)); 
//static const struct device *const uart_sys = DEVICE_DT_GET(DT_NODELABEL(uart1)); 
//https://docs.zephyrproject.org/latest/kernel/drivers/index.html#c.DEVICE_DT_GET

struct usb_command {
    char payload[64];
    uint32_t length;
};

//K_MSGQ_DEFINE(command_queue, sizeof(struct usb_command), 10, 4);
#define MSG_SIZE 64
//static char rx_buf[MSG_SIZE];

//K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

void print_uart(char *buf);
void serial_cb(const struct device *dev, void *user_data);

int legacy_send(const struct usb_command *command);
int legacy_receive(struct usb_command *command);
