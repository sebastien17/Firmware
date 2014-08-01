/**
 * @file adc.h
 * Definition of the adc data uORB topic.
 *
 * @author Sebastien Lautier (sebastien.lautier@gmail.com)
 */

#ifndef ADC_RAW_DATA_T_H_
#define ADC_RAW_DATA_T_H_

#include <stdint.h>
#include <nuttx/analog/adc.h>
#include "../uORB.h"


 /**
 * Raw adc value up to 12 channels
 */
typedef struct adc_msg_s adc_raw_data_s[12];

/* register this as object request broker structure */
ORB_DECLARE(adc_raw_data);

#endif
