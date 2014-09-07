/**
 * @file adc_raw_data.h
 * Definition of the adc data uORB topic.
 *
 * @author Sebastien Lautier (sebastien.lautier@gmail.com)
 */

#ifndef ADC_RAW_DATA_T_H_
#define ADC_RAW_DATA_T_H_

#include <stdint.h>
#include <nuttx/analog/adc.h>
#include "../uORB.h"


/*
 * Modified original structure adc_msg_s
 */
struct adc_msg_s_m
{
  uint8_t     	am_channel;             /* The 8-bit ADC Channel */
  int32_t     	am_data;                /* ADC convert result (4 bytes) */
  int32_t		am_mean_value;			/* Mean value */
} ;

/**
* Raw adc value + additionnal info up to 12 channels
*/
typedef struct adc_msg_s_m adc_raw_data_s[12];

/* register this as object request broker structure */
ORB_DECLARE(adc_raw_data);

#endif
