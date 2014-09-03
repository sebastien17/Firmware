/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Anton Babushkin <anton.babushkin@me.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_pos_control_params.c
 * Multicopter position controller parameters.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <systemlib/param/param.h>

/**
 * Minimum thrust
 *
 * Minimum vertical thrust. It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 1.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_MIN, 0.1f);

/**
 * Maximum thrust
 *
 * Limit max allowed thrust.
 *
 * @min 0.0
 * @max 1.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_MAX, 1.0f);

/**
 * Proportional gain for vertical position error
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_P, 1.0f);

/**
 * Proportional gain for vertical velocity error
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_P, 0.1f);

/**
 * Integral gain for vertical velocity error
 *
 * Non zero value allows hovering thrust estimation on stabilized or autonomous takeoff.
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_I, 0.02f);

/**
 * Differential gain for vertical velocity error
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_D, 0.0f);

/**
 * Maximum vertical velocity
 *
 * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL).
 *
 * @unit m/s
 * @min 0.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX, 5.0f);

/**
 * Vertical velocity feed forward
 *
 * Feed forward weight for altitude control in stabilized modes (ALTCTRL, POSCTRL). 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_FF, 0.5f);

/**
 * Proportional gain for horizontal position error
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_P, 1.0f);

/**
 * Proportional gain for horizontal velocity error
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_P, 0.1f);

/**
 * Integral gain for horizontal velocity error
 *
 * Non-zero value allows to resist wind.
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_I, 0.02f);

/**
 * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_D, 0.01f);

/**
 * Maximum horizontal velocity
 *
 * Maximum horizontal velocity in AUTO mode and endpoint for position stabilized mode (POSCTRL).
 *
 * @unit m/s
 * @min 0.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_MAX, 5.0f);

/**
 * Horizontal velocity feed forward
 *
 * Feed forward weight for position control in position control mode (POSCTRL). 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_FF, 0.5f);

/**
 * Maximum tilt angle in air
 *
 * Limits maximum tilt in AUTO and POSCTRL modes during flight.
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TILTMAX_AIR, 45.0f);

/**
 * Maximum tilt during landing
 *
 * Limits maximum tilt angle on landing.
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TILTMAX_LND, 15.0f);

/**
 * Landing descend rate
 *
 * @unit m/s
 * @min 0.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_SPEED, 1.0f);


/**********************************************************************************************************/
/** Parameters definition for sonar
 */

/**
 * Sonar Activation switch
 * (0: unactivated; 1: activated)
 * @unit none
 * @min 0
 * @max 1
 * @group Sonar
 */
PARAM_DEFINE_INT32(SNR_ACTIVATE_SW, 0);
/**
 * Lateral Meaningful Threshold
 * (meaningful th of the left sonar, 1ft 80 = 0 80 per ft)
 * @unit none
 * @min 0
 * @max 1500
 * @group Sonar
 */
PARAM_DEFINE_INT32(SNR_LAT_M_TH, 100);
/**
 * Lateral Short Threshold
 *(very close on the left sonar 3ft=380 4.5 570)
 * @unit none
 * @min 0
 * @max 1500
 * @group Sonar
 */
PARAM_DEFINE_INT32(SNR_LAT_S_TH, 570);
/**
 * Lateral Long Threshold
 *(close on the left sonar, 6ft=760)
 * @unit none
 * @min 0
 * @max 1500
 * @group Sonar
 */
PARAM_DEFINE_INT32(SNR_LAT_L_TH, 760);
/**
 * Forward Meaningful Threshold
 * (meaningful th of the forward sonar, 1ft 30 = 0 40 per ft)
 * @unit none
 * @min 0
 * @max 1500
 * @group Sonar
 */
PARAM_DEFINE_INT32(SNR_FWD_M_TH, 270);
/**
 * Forward Short Threshold
 *(very close on the forward sonar,5ft = 270 7 350)
 * @unit none
 * @min 0
 * @max 1500
 * @group Sonar
 */
PARAM_DEFINE_INT32(SNR_FWD_S_TH, 390);
/**
 * Forward Long Threshold
 *(close on the forward sonar, 8ft=390)
 * @unit none
 * @min 0
 * @max 1500
 * @group Sonar
 */
PARAM_DEFINE_INT32(SNR_FWD_L_TH, 760);
/**
 * ADC Left Sonar Index
 *(index to identify the left sonar signal in an adc_raw_data_s struct)
 * @unit none
 * @min 0
 * @max 11
 * @group Sonar
 */
PARAM_DEFINE_INT32(SNR_LEFT_ADC_IND, 6); // adc channel 13
/**
 * ADC Right Sonar Index
 *(index to identify the right sonar signal in an adc_raw_data_s struct)
 * @unit none
 * @min 0
 * @max 11
 * @group Sonar
 */
PARAM_DEFINE_INT32(SNR_RIGHT_ADC_IND, 7); // adc channel 14
/**
 * ADC Fwd Sonar Index
 *(index to identify the fwd sonar signal in an adc_raw_data_s struct)
 * @unit none
 * @min 0
 * @max 11
 * @group Sonar
 */
PARAM_DEFINE_INT32(SNR_FWD_ADC_IND, 8); // adc channel 15
/**
 * Sonar Correction
 *(TODO)
 * @unit rad
 * @min 0
 * @max TODO
 * @group Sonar
 */
PARAM_DEFINE_FLOAT(SNR_CORR_STEP_SIZE, 0.15f);
/**
 * Sonar Small Correction
 *(TODO)
 * @unit rad
 * @min 0
 * @max TODO
 * @group Sonar
 */
PARAM_DEFINE_FLOAT(SNR_CORR_SML_STEP_SIZE, 0.05f);
/**
 * Sonar Fwd Correction
 *(TODO)
 * @unit TODO
 * @min 0
 * @max TODO
 * @group Sonar
 */
PARAM_DEFINE_FLOAT(SNR_CORR_FWD_STEP_SIZE, 0.2f);

/**
 * Sonar Stuck Threshold
 *(TODO)
 * @unit TODO
 * @min 0
 * @max TODO
 * @group Sonar
 */
PARAM_DEFINE_INT32(SNR_STUCK_TH, 30);
