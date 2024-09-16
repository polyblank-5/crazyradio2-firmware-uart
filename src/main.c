/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright 2023 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

#include <zephyr/usb/usb_device.h>

#include "radio_mode.h"
#include "led.h"
#include "crusb.h"
#include "fem.h"
#include "button.h"
#include "esb.h"

#include "rpc.h"
#include "api.h"

#include <tinycbor/cbor.h>
#include <tinycbor/cbor_buf_reader.h>
#include <tinycbor/cbor_buf_writer.h>

#include <nrfx_clock.h>

#include <zephyr/logging/log.h>

#include "legacy_usb.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);


int startHFClock(void)
{
    nrfx_clock_hfclk_start();
	return 0;
}

#ifndef CONFIG_LEGACY_USB_PROTOCOL
K_MUTEX_DEFINE(usb_send_buffer_mutex);
void send_usb_message(char* data, size_t length) {
	static struct crusb_message message;
	if (length > USB_MTU) {
		return;
	}

	k_mutex_lock(&usb_send_buffer_mutex, K_FOREVER);
	memcpy(message.data, data, length);
	message.length = length;
	crusb_send(&message);
	k_mutex_unlock(&usb_send_buffer_mutex);
}
#endif

void main(void)
{
	

    print_uart("Hello World! %s\n");

    // HFCLK crystal is needed by the radio and USB
    startHFClock();

    led_init();
	led_pulse_green(K_MSEC(500));
	led_pulse_red(K_MSEC(500));
	led_pulse_blue(K_MSEC(500));

	button_init();

#ifndef CONFIG_LEGACY_USB_PROTOCOL
	print_uart("RADIO INIT STARTED")
    radio_mode_init();
#else
	esb_init();
#endif

	fem_init();

	// Test the FEM
	print_uart("Enaabling TX\n");
	fem_txen_set(true);
	k_sleep(K_MSEC(10));
	bool enabled = fem_is_pa_enabled();
	//printk("PA enabled: %d\n", enabled);
	print_uart("PA enabled: %d\n");
	fem_txen_set(false);

	print_uart("Enaabling RX\n");
	fem_rxen_set(true);
	k_sleep(K_MSEC(10));
	enabled = fem_is_lna_enabled();
	//printk("LNA enabled: %d\n", enabled);
	print_uart("LNA enabled: %d\n");
	fem_rxen_set(false);

    int ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		print_uart("Failed to enable USB");
		return;
	}

#ifndef CONFIG_LEGACY_USB_PROTOCOL

	rpc_transport_t usb_transport = {
		.mtu = USB_MTU,
		.send = send_usb_message,
	};

    // RPC loop
    while(1) {
        static struct crusb_message message;
		static char response_buffer[USB_MTU];
        crusb_receive(&message);
		LOG_INF("Received %d byte message from usb!", message.length);

		rpc_error_t error = rpc_dispatch(&crazyradio2_rpc_api, message.data, message.length, usb_transport, response_buffer);
		LOG_INF("Dispatching result: %d", error);
    }

#else
	print_uart("Initializing UART");
	static struct usb_command command;
	static char radio_data[32];

	if (!device_is_ready(uart_dev)) {
		print_uart("UART device not found!");
		return;
	}

	/* configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	uart_irq_rx_enable(uart_dev);

	print_uart("Hello! I'm your echo bot.\r\n");
	print_uart("Tell me something and press enter:\r\n");

	/* indefinitely wait for input from the user */
	while (legacy_receive(&command) == 0) {
		print_uart("Echo: ");
		print_uart(command.payload);
		print_uart("\r\n");
		read_radio_queue(&radio_data);
		print_uart("Radio_MSG:");
		print_uart(radio_data);
		print_uart("\r\n");
	}
#endif
}
