/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <string.h>
#include <sys/printk.h>
#include <sys/__assert.h>

#include "ble_app.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif

extern struct k_fifo ble_led_fifo;

#define BLINK_STACK_SIZE 1024
/* scheduling priority used by each thread */
#define PRIORITY 7

void blink0(void)
{
	const struct device *dev;
	bool led_is_on = true;
	int ret;

	struct ble_led_data_t *rx_data;
	uint32_t ledDelay = 100;

	dev = device_get_binding(LED0);
	if (dev == NULL) {
		printk("*** led0 is null ***\n");
		return;
	}

	ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}

	printk("\n*** Hello World from blink0 thread***\n");

	while (1) {
		gpio_pin_set(dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;

		rx_data = k_fifo_get(&ble_led_fifo, K_MSEC(ledDelay));

		if (NULL != rx_data)
		{
			ledDelay = rx_data->cnt * 100;
		}
	}
}

void console_out(void)
{
	printk("\n*** Hello World from console thread***\n");

	while (1) {
		struct ble_led_data_t *rx_data = k_fifo_get(&ble_led_fifo,
							   K_FOREVER);
		printk("Toggled led -> current state =%d\n", rx_data->cnt);
	}
}

void ble_task()
{
	uint16_t status = 0;

	printk("\n*** Hello World from ble thread***\n");

	status = ble_app_init();

	if (0 != status) {
		printk("*** ble app init failed with status: %d ***\n", status);
		return;
	}

	while(1)
	{
		/* Process BLE Event */
		ble_app_main();
	}
}

void main(void)
{
}

K_THREAD_DEFINE(ble_task_id, 2048, ble_task, NULL, NULL, NULL,
		7, 0, 0);

K_THREAD_DEFINE(blink0_id, BLINK_STACK_SIZE, blink0, NULL, NULL, NULL,
		1, 0, 0);
/*
K_THREAD_DEFINE(console_out_id, BLINK_STACK_SIZE, console_out, NULL, NULL, NULL,
		PRIORITY, 0, 0);
*/
