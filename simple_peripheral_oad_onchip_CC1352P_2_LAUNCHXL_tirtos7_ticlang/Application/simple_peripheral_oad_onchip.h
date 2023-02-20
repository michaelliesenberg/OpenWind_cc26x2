/******************************************************************************

 @file  simple_peripheral_oad_onchip.h

 @brief This file contains the OAD sample application based on
        simple_peripheral for use with the CC2650 Bluetooth Low Energy
        Protocol Stack.

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2017-2022, Texas Instruments Incorporated
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
 
 
 *****************************************************************************/

#ifndef SBP_OAD_ONCHIP_H
#define SBP_OAD_ONCHIP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <ti/sysbios/knl/Clock.h>
#include <icall.h>

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */
#define delay_ms(i) Task_sleep( ((i) * 1000) / ti_sysbios_knl_Clock_tickPeriod )

/*********************************************************************
 * MACROS
 */

// Application events
#define SP_STATE_CHANGE_EVT                  0
#define SP_CHAR_CHANGE_EVT                   1
#define SP_KEY_CHANGE_EVT                    2
#define SP_ADV_EVT                           3
#define SP_PAIR_STATE_EVT                    4
#define SP_PASSCODE_EVT                      5
#define SP_PERIODIC_EVT                      6
#define SP_READ_RPA_EVT                      7
#define SP_SEND_PARAM_UPDATE_EVT             8
#define SP_CONN_EVT                          9
#define SP_OAD_RESET_EVT                     10

#define OW_CHAR_BAT                         11
#define SERVICE_ID_MOV                      12
#define SERVICE_ID_Wind                     13
#define OW_CHAR_DIR                         14
#define OW_CHAR_MOV                         15
#define OW_CHAR_GPS                         16
//OpenWind Events
#define OW_WIND_DIR                         17



#define SP_OAD_QUEUE_EVT                     OAD_QUEUE_EVT       // Event_Id_01
#define SP_OAD_COMPLETE_EVT                  OAD_DL_COMPLETE_EVT // Event_Id_02
#define SP_OAD_NO_MEM_EVT                    OAD_OUT_OF_MEM_EVT  // Event_Id_03

#define OW_WIND_TASK                        Event_Id_09
#define OW_WIND_MOV                         Event_Id_10
#define OW_WIND_BAT                         Event_Id_11
#define OW_WIND_GPS                         Event_Id_12
#define OW_WIND_GPS_READ                    Event_Id_13
#define OW_AS5055_READ                      Event_Id_18
#define MOV_INT_EVENT                       Event_Id_21
#define OW_WIND_LED                         Event_Id_22

// Internal Events for RTOS application
#define SP_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define SP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30


#define APP_STATE_ERROR           0xFF
#define APP_STATE_OFF             0
#define APP_STATE_IDLE            1
#define APP_STATE_ACTIVE          2




#define SP_ALL_EVENTS                       (SP_ICALL_EVT                | \
                                             SP_QUEUE_EVT                 | \
                                             SP_OAD_QUEUE_EVT                 | \
                                             SP_OAD_NO_MEM_EVT                 | \
                                             SP_QUEUE_EVT                 | \
                                             OW_WIND_TASK           | \
                                             OW_WIND_MOV              | \
                                             OW_WIND_BAT            | \
                                             OW_WIND_GPS         | \
                                             OW_CHAR_DIR          | \
                                             OW_CHAR_MOV            | \
                                             OW_CHAR_BAT             | \
                                             OW_CHAR_GPS         | \
                                             OW_AS5055_READ            | \
                                             SERVICE_ID_Wind   | \
                                             SERVICE_ID_MOV  | \
                                             MOV_INT_EVENT    | \
                                             OW_WIND_LED               | \
                                             OW_WIND_GPS_READ    )


/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the SBP OAD on-chip app
 */
extern void SimplePeripheral_createTask(void);

/*
 * Functions for menu action
 */
/* Actions for Menu: Choose connection to work with */
bool SimplePeripheral_doSelectConn(uint8_t index);

/* Action for Menu: AutoConnect */
bool SimplePeripheral_doAutoConnect(uint8_t index);

/* Actions for Menu: Set PHY - Select */
bool SimplePeripheral_doSetConnPhy(uint8_t index);

void OPENWIND_Event(int event);

uint8 SimplePeripheral_enqueueMsg(uint8_t event, void *pData);

void OpenWind_enqueueMsg(uint8_t event, uint8_t value);

void OpenWindFlash_Write();
void OpenWindFlash_Read();
void clear_watchdog();
void SensorTag_updateAdvertisingData(uint8_t location,uint8_t value);
extern float Wind_Direction;
extern bool Wind_Direction_Scale_Flag;
extern uint8_t wert_von_handy;
extern double Wind_Speed;


extern float AWA_offset;
extern float AWS_offset;
extern signed short int roll_offset;
extern signed short int pitch_offset;
extern signed short int yaw_offset;

extern uint8_t keep_on;
extern uint8_t bno_low_power;
extern int16_t MastAngle;

extern uint8_t Flag_ADV;

extern bool OpenWind_START_DIRECTION;
extern bool OpenWind_START_MOV;
extern bool OpenWind_START_GPS;
extern bool OpenWind_START_BAT;
extern bool OpenWind_has_Connection;


extern uint16_t OpenWind_SOG;
extern uint16_t OpenWind_COG;

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SBP_OAD_ONCHIP_H */
