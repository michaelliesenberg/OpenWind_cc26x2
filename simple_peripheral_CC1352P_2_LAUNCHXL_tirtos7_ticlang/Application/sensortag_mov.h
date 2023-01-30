/******************************************************************************

 @file  sensortag_mov.h

 @brief This file contains the Sensor Tag sample application,
        Movement Processor, for use with the TI Bluetooth Low
        Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************

 Copyright (c) 2015-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_00_31
 Release Date: 2016-06-16 18:57:29
 *****************************************************************************/

#ifndef SENSORTAGMOV_H
#define SENSORTAGMOV_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "simple_peripheral.h"
#ifdef BNO055
#include "BNO055.h"
#endif
#ifdef BNO080
#include "BNO080.h"
#endif
/*********************************************************************
 * CONSTANTS
 */


extern struct bno055_mag_offset_t  mag_offset_flash;
extern struct bno055_accel_offset_t  accel_offset_flash;
extern struct bno055_gyro_offset_t   gyro_offset_flash;

extern float roll;
extern float pitch;
extern float yaw;
extern uint16_t u16roll;
extern uint16_t u16pitch;
extern uint16_t u16yaw;


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

#ifndef EXCLUDE_MOV
/*
 * Create the Movement sensor task
 */
extern void SensorTagMov_createTask(void);

/*
 * Initialize Movement sensor module
 */
extern void SensorTagMov_init(void);

/*
 * Task Event Processor for Movement sensor module
 */
extern void SensorTagMov_processSensorEvent(void);

/*
 * Task Event Processor for characteristic changes
 */
extern void SensorTagMov_processCharChangeEvt(uint8_t paramID);

/*
 * Reset Movement sensor module
 */
extern void SensorTagMov_reset(void);

extern uint8_t parameter_mov;

static void SensorTagMov_processInterrupt(uint_least8_t index);
void SensorTagMov_INT_Handle(void);
void init_INT_BNO(void);
void disable_INT_BNO(void);
void read_calibration_values(void);
void appStateSet_mov(uint8_t newState);
extern bool MAGhasbeenCalibrated;
extern bool ACChasbeenCalibrated;
extern bool GYROhasbeenCalibrated;

extern uint16_t yaw_value;
extern uint16_t pitch_value;
extern uint16_t roll_value;
extern uint8_t mov_calib_status;

#else

/* Movement module not included */

#define SensorTagMov_init()
#define SensorTagMov_processCharChangeEvt(paramID)
#define SensorTagMov_processSensorEvent()
#define SensorTagMov_reset()

#endif // EXCLUDE_MOV

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SENSORTAGMOV_H */
