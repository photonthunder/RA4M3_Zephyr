/*
 * ble_app.h
 *
 *  Created on: Nov 29, 2020
 *      Author: zoltan.ianosi.bp
 */

#ifndef SAMPLES_BLUETOOTH_BLE_EK_RA4W1_SRC_BLE_APP_H_
#define SAMPLES_BLUETOOTH_BLE_EK_RA4W1_SRC_BLE_APP_H_

#include <zephyr.h>

struct ble_led_data_t {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	uint32_t cnt;
};

uint16_t ble_app_init();
void ble_app_main();

#endif /* SAMPLES_BLUETOOTH_BLE_EK_RA4W1_SRC_BLE_APP_H_ */
