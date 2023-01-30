/******************************************************************************

 @file  Windservice.h

 @brief This file contains the Heart Rate service definitions and prototypes
        prototypes.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************

 Copyright (c) 2012-2016, Texas Instruments Incorporated
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

#ifndef WINDSERVICE_H
#define WINDSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <bcomdef.h>
#include <att.h>
/*********************************************************************
 * CONSTANTS
 */

#define WIND_SERV_UUID              0xCC90
#define WIND_DATA_UUID              0xCC91
#define WIND_OFFSET_UUID            0xCC92
#define WIND_COMMAND_UUID           0xCC93
#define WIND_KEEP_ON_UUID           0xCC94
#define WIND_MAST_ANGLE_UUID        0xCC95
// Heart Rate Service Parameters
#define Wind_MEAS                      0
#define Wind_MEAS_CHAR_CFG             1
#define Wind_OFFSET                    2
#define Wind_COMMAND                   3
#define Wind_Keep                      4
#define Wind_MAST_ANGLE                5

#define WIND_OFFSET_LEN                12
#define WIND_KEEP_LEN                  1
#define WIND_MAST_ANGLE_LEN            3
// Maximum length of heart rate measurement characteristic
#define Wind_MEAS_MAX                  (ATT_MTU_SIZE -5)

// Values for flags
#define Wind_FLAGS_FORMAT_UINT16       0x01
#define Wind_FLAGS_CONTACT_NOT_SUP     0x00
#define Wind_FLAGS_CONTACT_NOT_DET     0x04
#define Wind_FLAGS_CONTACT_DET         0x06
#define Wind_FLAGS_ENERGY_EXP          0x08
#define Wind_FLAGS_RR                  0x10

// Values for sensor location
#define Wind_SENS_LOC_OTHER            0x00
#define Wind_SENS_LOC_CHEST            0x01
#define Wind_SENS_LOC_WRIST            0x02
#define Wind_SENS_LOC_FINGER           0x03
#define Wind_SENS_LOC_HAND             0x04
#define Wind_SENS_LOC_EARLOBE          0x05
#define Wind_SENS_LOC_FOOT             0x06

// Value for command characteristic
#define Wind_COMMAND_ENERGY_EXP        0x01

// ATT Error code
// Control point value not supported
#define Wind_ERR_NOT_SUP               0x80

// Heart Rate Service bit fields
#define Wind_SERVICE                   0x00000001

// Callback events
#define Wind_MEAS_NOTI_ENABLED         1
#define Wind_MEAS_NOTI_DISABLED        2
#define Wind_COMMAND_SET               3
#define Wind_OFFSET_SET                4
#define Wind_KEEP_SET                  5
#define Wind_MAST_ANGLE_SET            6
/*********************************************************************
 * TYPEDEFS
 */

// Heart Rate Service callback function
typedef void (*WindServiceCB_t)(uint8_t event);

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


/*********************************************************************
 * API FUNCTIONS
 */

/*
 * Wind_AddService- Initializes the Heart Rate service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t Wind_AddService();

/*
 * Wind_Register - Register a callback function with the
 *          Heart Rate Service
 *
 * @param   pfnServiceCB - Callback function.
 */

extern void Wind_Register(WindServiceCB_t pfnServiceCB);

/*
 * Wind_SetParameter - Set a Heart Rate parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Wind_SetParameter(uint8_t param, uint8_t len, void *value);

/*
 * Wind_GetParameter - Get a Heart Rate parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Wind_GetParameter(uint8_t param, void *value);

/*********************************************************************
 * @fn          Wind_MeasNotify
 *
 * @brief       Send a notification containing a heart rate
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
extern bStatus_t Wind_MeasNotify(uint16_t connHandle,
                                      attHandleValueNoti_t *pNoti);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* WindSERVICE_H */
