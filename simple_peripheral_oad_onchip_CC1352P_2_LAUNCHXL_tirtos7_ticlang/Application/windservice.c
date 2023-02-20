/******************************************************************************

 @file  Windservice.c

 @brief This file contains the Heart Rate sample service for use with the
        Heart Rate sample application.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************

 Copyright (c) 2011-2016, Texas Instruments Incorporated
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

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "simple_peripheral_oad_onchip.h"
#include "windservice.h"
#include "st_util.h"
#include "sensortag_wind.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Position of heart rate measurement value in attribute array
#define Wind_MEAS_VALUE_POS            2

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Heart rate service
CONST uint8 WindServUUID[TI_UUID_SIZE] =
{
 TI_UUID(WIND_SERV_UUID)
};

// Heart rate measurement characteristic
CONST uint8 WindMeasUUID[TI_UUID_SIZE] =
{
 TI_UUID(WIND_DATA_UUID)
};

// Sensor location characteristic
CONST uint8 WindOffsetUUID[TI_UUID_SIZE] =
{
 TI_UUID(WIND_OFFSET_UUID)
};

// Command characteristic
CONST uint8 WindCommandUUID[TI_UUID_SIZE] =
{
 TI_UUID(WIND_COMMAND_UUID)
};

// Command characteristic
CONST uint8 WindKeepOnUUID[TI_UUID_SIZE] =
{
 TI_UUID(WIND_KEEP_ON_UUID)
};

// Mast Angle characteristic
CONST uint8 WindMastAngle[TI_UUID_SIZE] =
{
 TI_UUID(WIND_MAST_ANGLE_UUID)
};

static uint8 SensorOffsetUserDesp[13] = "SensorOffset";

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static WindServiceCB_t WindServiceCB;

/*********************************************************************
 * Profile Attributes - variables
 */

// Heart Rate Service attribute
static CONST gattAttrType_t WindService = {TI_UUID_SIZE, WindServUUID};

// Heart Rate Measurement Characteristic
// Note characteristic value is not stored here
static uint8_t WindMeasProps = GATT_PROP_NOTIFY;
static uint8_t WindMeas = 0;
static gattCharCfg_t *WindMeasClientCharCfg;

// Wind Offset Characteristic
static uint8 WindOffsetProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 WindOffset[WIND_OFFSET_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0};//AWA_Offset,AWS_Offset,Yaw_Offset,Pitch_Offset,Roll__Offset

// Command Characteristic
static uint8_t WindCommandProps = GATT_PROP_WRITE;
static uint8_t WindCommand = 0;

static uint8_t WindKeepProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8_t WindKeep = 0;

static uint8_t WindMastAngleProps = GATT_PROP_READ | GATT_PROP_WRITE;//GATT_PROP_WRITE_NO_RSP;
static uint8_t WindMastAngleArray[WIND_MAST_ANGLE_LEN] = {0,0,0};//0x01, MAST_ANGLE MSB, LSB
/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t WindAttrTbl[] =
{
  // Heart Rate Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&WindService                /* pValue */
  },

    // Heart Rate Measurement Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &WindMeasProps
    },

      // Heart Rate Measurement Value
      {
        { TI_UUID_SIZE, WindMeasUUID },
        0,
        0,
        &WindMeas
      },

      // Heart Rate Measurement Client Characteristic Configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) &WindMeasClientCharCfg
      },

    // Sensor Offset Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &WindOffsetProps
    },

      // Sensor Offset Value
      {
        { TI_UUID_SIZE, WindOffsetUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        WindOffset
      },


    // Sensor Offset User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        SensorOffsetUserDesp
    },



    // Command Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &WindCommandProps
    },

      // Command Value
      {
        { TI_UUID_SIZE, WindCommandUUID },
        GATT_PERMIT_WRITE,
        0,
        &WindCommand
      },
      // Keep Declaration
      {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &WindKeepProps
      },

    // Keep Value
    {
      { TI_UUID_SIZE, WindKeepOnUUID },
      GATT_PERMIT_READ | GATT_PERMIT_WRITE,
      0,
      &WindKeep
    },
    // Mast Angle Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &WindMastAngleProps
    },
    // Mast Angle Value
    {
      { TI_UUID_SIZE, WindMastAngle },
      GATT_PERMIT_READ | GATT_PERMIT_WRITE,
      0,
      WindMastAngleArray
    }
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t Wind_ReadAttrCB(uint16_t connHandle,
                                      gattAttribute_t *pAttr, uint8_t *pValue,
                                      uint16_t *pLen, uint16_t offset,
                                      uint16_t maxLen, uint8_t method);
static bStatus_t Wind_WriteAttrCB(uint16_t connHandle,
                                       gattAttribute_t *pAttr, uint8_t *pValue,
                                       uint16_t len, uint16_t offset,
                                       uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Heart Rate Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t WindCBs =
{
  Wind_ReadAttrCB,  // Read callback function pointer
  Wind_WriteAttrCB, // Write callback function pointer
  NULL                   // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Wind_AddService
 *
 * @brief   Initializes the Heart Rate service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t Wind_AddService()
{

    uint8 status;
    uint8_t numcon = linkDB_NumConns();
    // Allocate Client Characteristic Configuration table
    WindMeasClientCharCfg = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *numcon );
    if ( WindMeasClientCharCfg == NULL )
    {
        return ( bleMemAllocError );
    }

    // Initialize Client Characteristic Configuration attributes.
    GATTServApp_InitCharCfg(0xFFFF, WindMeasClientCharCfg);

    status = GATTServApp_RegisterService(WindAttrTbl,GATT_NUM_ATTRS(WindAttrTbl),GATT_MAX_ENCRYPT_KEY_SIZE,&WindCBs);

    return status;

/*
  return GATTServApp_RegisterService( WindAttrTbl,
                                      GATT_NUM_ATTRS( WindAttrTbl ),
                                      GATT_MAX_ENCRYPT_KEY_SIZE,
                                      &WindCBs );*/
}

/*********************************************************************
 * @fn      Wind_Register
 *
 * @brief   Register a callback function with the Heart Rate Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void Wind_Register(WindServiceCB_t pfnServiceCB)
{
  WindServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      Wind_SetParameter
 *
 * @brief   Set a Heart Rate parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Wind_SetParameter(uint8 param, uint8 len, void *value)
{
  bStatus_t ret = SUCCESS;
  switch (param)
  {
     case Wind_MEAS_CHAR_CFG:
      // Need connection handle
      //WindMeasClientCharCfg.value = *((uint16*)value);
      break;

    case Wind_OFFSET:
      VOID memcpy( WindOffset, value, WIND_OFFSET_LEN);
      break;

    case Wind_Keep:
      VOID memcpy( WindKeep, value, WIND_KEEP_LEN);
      break;

    case Wind_MAST_ANGLE:
      VOID memcpy(WindMastAngleArray, value, WIND_MAST_ANGLE_LEN);
      break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn      Wind_GetParameter
 *
 * @brief   Get a Heart Rate parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Wind_GetParameter(uint8 param, void *value)
{
  bStatus_t ret = SUCCESS;
  switch (param)
  {
    case Wind_MEAS_CHAR_CFG:
      // Need connection handle
      //*((uint16*)value) = WindMeasClientCharCfg.value;
      break;

    case Wind_OFFSET:
      VOID memcpy(value, WindOffset, WIND_OFFSET_LEN);
      break;

    case Wind_COMMAND:
      *((uint8*)value) = WindCommand;
      break;

    case Wind_Keep:
        *((uint8*)value) = WindKeep;
          break;
    case Wind_MAST_ANGLE:
          VOID memcpy(value, WindMastAngleArray, WIND_MAST_ANGLE_LEN);
          break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

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
bStatus_t Wind_MeasNotify(uint16 connHandle, attHandleValueNoti_t *pNoti)
{
  uint16 value = GATTServApp_ReadCharCfg(connHandle,
                                         WindMeasClientCharCfg);

  // If notifications enabled
  if (value & GATT_CLIENT_CFG_NOTIFY)
  {
    // Set the handle.
    pNoti->handle = WindAttrTbl[Wind_MEAS_VALUE_POS].handle;

    // Send the notification.
    return GATT_Notification(connHandle, pNoti, FALSE);
  }

  return bleIncorrectMode;
}


bStatus_t Wind_MeasIndication(uint16 connHandle, attHandleValueInd_t *pIndi)
{
  uint16 value = GATTServApp_ReadCharCfg(connHandle,
                                         WindMeasClientCharCfg);

  // If notifications enabled
  if (value & GATT_CLIENT_CFG_NOTIFY)
  {
    // Set the handle.
      pIndi->handle = WindAttrTbl[Wind_MEAS_VALUE_POS].handle;

    return GATT_Indication( connHandle, pIndi, FALSE, 1 );

  }

  return bleIncorrectMode;
}

/*********************************************************************
 * @fn          Wind_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t Wind_ReadAttrCB(uint16_t connHandle,
                                      gattAttribute_t *pAttr, uint8_t *pValue,
                                      uint16_t *pLen, uint16_t offset,
                                      uint16_t maxLen, uint8_t method)
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);


  switch (uuid)
    {
      case WIND_OFFSET_UUID:

                *pLen = WIND_OFFSET_LEN;
                VOID memcpy(pValue, pAttr->pValue, WIND_OFFSET_LEN );
//                (*WindServiceCB)(Wind_OFFSET_SET);
            break;
      case WIND_KEEP_ON_UUID:
                *pLen = 1;
                VOID memcpy(pValue, pAttr->pValue, WIND_KEEP_LEN );
            break;

      case WIND_MAST_ANGLE_UUID:
            *pLen = WIND_MAST_ANGLE_LEN;
            VOID memcpy(pValue, pAttr->pValue, WIND_MAST_ANGLE_LEN );
            break;

      default:
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;

    }

  return (status);
}

/*********************************************************************
 * @fn      Wind_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t Wind_WriteAttrCB(uint16_t connHandle,
                                       gattAttribute_t *pAttr, uint8_t *pValue,
                                       uint16_t len, uint16_t offset,
                                       uint8_t method)
{
  bStatus_t status = SUCCESS;

  //uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
  uint16_t uuid;
  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    return ATT_ERR_INVALID_HANDLE;
  }
  switch (uuid)
  {
    case WIND_COMMAND_UUID:

    	(*WindServiceCB)(Wind_COMMAND_SET);

    	*(pAttr->pValue) = pValue[0];

      break;
    case WIND_OFFSET_UUID:

          if (offset > 0)
          {
            status = ATT_ERR_ATTR_NOT_LONG;
          }
          else if (len != WIND_OFFSET_LEN)
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
          else
          {
              if(pValue[0]==0x01 && pValue[11]==0x47)
              {
                  VOID memcpy(pAttr->pValue, pValue, WIND_OFFSET_LEN);
                  (*WindServiceCB)(Wind_OFFSET_SET);
              }
          }
          break;

    case WIND_KEEP_ON_UUID:

              if (offset > 0)
                {
                  status = ATT_ERR_ATTR_NOT_LONG;
                }
                else if (len != WIND_KEEP_LEN)
                {
                  status = ATT_ERR_INVALID_VALUE_SIZE;
                }
                else
                {
                    VOID memcpy(pAttr->pValue, pValue, WIND_KEEP_LEN);
                    (*WindServiceCB)(Wind_KEEP_SET);
                }

          break;
    case WIND_MAST_ANGLE_UUID:

          if (offset > 0)
          {
            status = ATT_ERR_ATTR_NOT_LONG;
          }
          else if (len != WIND_MAST_ANGLE_LEN)
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
          else
          {
              if(pValue[0]==0x01)
              {
                  VOID memcpy(pAttr->pValue, pValue, WIND_MAST_ANGLE_LEN);
                  (*WindServiceCB)(Wind_MAST_ANGLE_SET);
              }
          }
          break;

    case GATT_CLIENT_CHAR_CFG_UUID:
      status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                               offset, GATT_CLIENT_CFG_NOTIFY);
      if (status == SUCCESS)
      {
        uint16 charCfg = BUILD_UINT16(pValue[0], pValue[1]);
			(*WindServiceCB)((charCfg == GATT_CFG_NO_OPERATION) ?
									Wind_MEAS_NOTI_DISABLED :
									Wind_MEAS_NOTI_ENABLED);
      }
      break;

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;

  }

  return (status);
}


/*********************************************************************
*********************************************************************/
