/******************************************************************************

 @file  movementservice.h

 @brief Gyroscope service definitions and prototypes

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

#ifndef BNOSERVICE_H
#define BNOSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "st_util.h"

/*********************************************************************
 * CONSTANTS
 */

// Service UUID
#define MOVEMENT_SERV_UUID             0xAA80

#define MOVEMENT_DATA1_UUID            0xAA81
#define MOVEMENT_CONF1_UUID            0xAA82
#define MOVEMENT_PERI_UUID             0xAA83

#define MOVEMENT_DATA2_UUID            0xAA85
#define MOVEMENT_DATA3_UUID            0xAA87

#define MOVEMENT_CONF2_UUID            0xAA84
#define MOVEMENT_AXISMAP_UUID          0xAA86

#define MOVEMENT_CALIBRATE_UUID        0xAA88
#define MOVEMENT_CALIBRATE_PITCH_UUID  0xAA89

#define MOVEMENT_CALIBRATE_STATUS_UUID 	0xAA90
#define MOVEMENT_CALIBRATE_RESET_UUID 	0xAA91

#define MOVEMENT_CALIBRATE              14
#define MOVEMENT_CALIBRATE_PITCH        15
#define MOVEMENT_CALIBRATE_MAG          16
#define MOVEMENT_CALIBRATE_ALL          17
#define MOVEMENT_CALIBRATE_STOP         18
// Sensor Profile Services bit fields
#define MOVEMENT_SERVICE               0x00000020

// Length of sensor data in bytes
#define MOVEMENT_DATA1_LEN              18
#define MOVEMENT_DATA2_LEN              8
#define MOVEMENT_DATA3_LEN              12
#define MOVEMENT_CONF1_LEN				1
#define MOVEMENT_CONF2_LEN				2
#define MOVEMENT_AXISMAP_LEN			2
#define MOVEMENT_CALIBRATE_LEN          2

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * API FUNCTIONS
 */


/*
 * Movement_addService - Initializes the Sensor GATT Profile service by
 *          registering GATT attributes with the GATT server.
 */
extern bStatus_t Movement_addService_bno(void);

/*
 * Movement_registerAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Movement_registerAppCBs_bno(sensorCBs_t *appCallbacks);

/*
 * Movement_setParameter - Set a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t Movement_setParameter_bno(uint8_t param, uint8_t len, void *value);

/*
 * Movement_getParameter - Get a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to read.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t Movement_getParameter_bno(uint8_t param, void *value);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* MOVEMENTSERVICE_H */
