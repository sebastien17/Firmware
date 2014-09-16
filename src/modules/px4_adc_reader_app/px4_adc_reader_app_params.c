/*
 * px4_adc_reader_app_param.c
 *
 *  Created on: 16 sept. 2014
 *      Author: NimH
 */

#include <systemlib/param/param.h>

/**
 * Brown Linear Expo Factor
 *
 * Factor used in Brown Linear Expo Low Pass Filter
 *
 * @min 0.0
 * @max 1.0
 * @group Sonar
 */
PARAM_DEFINE_FLOAT(SNR_BLE_FACTOR, 0.1f);
