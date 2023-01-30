/******************************************************************************

 @file  simple_peripheral.c

 @brief This file contains the Simple Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2013-2022, Texas Instruments Incorporated
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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#if (!(defined __TI_COMPILER_VERSION__) && !(defined __GNUC__))
#include <intrinsics.h>
#endif

#include <ti/drivers/GPIO.h>
#include <ti/drivers/utils/List.h>

#include <icall.h>
#include "util.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include <devinfoservice.h>

#ifdef USE_RCOSC
#include <rcosc_calibration.h>
#endif //USE_RCOSC

#include <ti_drivers_config.h>
#include <ti/drivers/NVS.h>
#include <board_key.h>

#include "simple_peripheral.h"
#include "ti_ble_config.h"

#ifdef PTM_MODE
#include "npi_task.h"               // To allow RX event registration
#include "npi_ble.h"                // To enable transmission of messages to UART
#include "icall_hci_tl.h"   // To allow ICall HCI Transport Layer
#endif // PTM_MODE


#include "sensortag_batt.h"
#include "sensortag_wind.h"
#include "sensortag_mov.h"
#include "SensorI2C.h"
#include "BQ27441.h"
#include "DRV5013.h"
#include "AS5055.h"
#include "YIC71513PGMGG.h"
#include <icall_ble_api.h> // OSAL SNV operations are defined through ICALL
#include <inc/hw_fcfg1.h>
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// How often to perform periodic event (in ms)
#define SP_PERIODIC_EVT_PERIOD               5000

// Task configuration
#define SP_TASK_PRIORITY                     1

#ifndef SP_TASK_STACK_SIZE
#define SP_TASK_STACK_SIZE                   1024
#endif



// Internal Events for RTOS application
#define SP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define SP_ADDR_STR_SIZE     15

// Row numbers for two-button menu
#define SP_ROW_SEPARATOR_1   (TBM_ROW_APP + 0)
#define SP_ROW_STATUS_1      (TBM_ROW_APP + 1)
#define SP_ROW_STATUS_2      (TBM_ROW_APP + 2)
#define SP_ROW_CONNECTION    (TBM_ROW_APP + 3)
#define SP_ROW_ADVSTATE      (TBM_ROW_APP + 4)
#define SP_ROW_RSSI          (TBM_ROW_APP + 5)
#define SP_ROW_IDA           (TBM_ROW_APP + 6)
#define SP_ROW_RPA           (TBM_ROW_APP + 7)
#define SP_ROW_DEBUG         (TBM_ROW_APP + 8)
#define SP_ROW_AC            (TBM_ROW_APP + 9)

// For storing the active connections
#define SP_RSSI_TRACK_CHNLS        1            // Max possible channels can be GAP_BONDINGS_MAX
#define SP_MAX_RSSI_STORE_DEPTH    5
#define SP_INVALID_HANDLE          0xFFFF
#define RSSI_2M_THRSHLD           -30           
#define RSSI_1M_THRSHLD           -40           
#define RSSI_S2_THRSHLD           -50           
#define RSSI_S8_THRSHLD           -60           
#define SP_PHY_NONE                LL_PHY_NONE  // No PHY set
#define AUTO_PHY_UPDATE            0xFF

// Spin if the expression is not true
#define SIMPLEPERIPHERAL_ASSERT(expr) if (!(expr)) simple_peripheral_spin();

/*********************************************************************
 * TYPEDEFS
 */

// Auto connect availble groups
enum
{
  AUTOCONNECT_DISABLE = 0,              // Disable
  AUTOCONNECT_GROUP_A = 1,              // Group A
  AUTOCONNECT_GROUP_B = 2               // Group B
};


// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct
{
  uint8_t event;                // event type
  void    *pData;               // pointer to message
} spEvt_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} spPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t uiInputs;
  uint8_t uiOutputs;
  uint32_t numComparison;
} spPasscodeData_t;

typedef struct {
    uint8_t value;
} openWindData_t;


// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} spGapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
  uint8_t event;                //
  uint8_t data[];
} spClockEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
  List_Elem elem;
  uint16_t  connHandle;
} spConnHandleEntry_t;

// Connected device information
typedef struct
{
  uint16_t         	    connHandle;                        // Connection Handle
  spClockEventData_t*   pParamUpdateEventData;
  Clock_Struct*    	    pUpdateClock;                      // pointer to clock struct
  int8_t           	    rssiArr[SP_MAX_RSSI_STORE_DEPTH];
  uint8_t          	    rssiCntr;
  int8_t           	    rssiAvg;
  bool             	    phyCngRq;                          // Set to true if PHY change request is in progress
  uint8_t          	    currPhy;
  uint8_t          	    rqPhy;
  uint8_t          	    phyRqFailCnt;                      // PHY change request count
  bool             	    isAutoPHYEnable;                   // Flag to indicate auto phy change
} spConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */


// Task configuration
Task_Struct spTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(spTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t spTaskStack[SP_TASK_STACK_SIZE];



/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Clock instance for internal periodic events. Only one is needed since
// GattServApp will handle notifying all connected GATT clients

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Memory to pass periodic event ID to clock handler
spClockEventData_t argPeriodic =
{ .event = SP_PERIODIC_EVT };

// Memory to pass RPA read event ID to clock handler
spClockEventData_t argRpaRead =
{ .event = SP_READ_RPA_EVT };

// Per-handle connection info
static spConnRec_t connList[MAX_NUM_BLE_CONNS];

// Current connection handle as chosen by menu
static uint16_t menuConnHandle = LINKDB_CONNHANDLE_INVALID;

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Auto connect Disabled/Enabled {0 - Disabled, 1- Group A , 2-Group B, ...}
uint8_t autoConnect = AUTOCONNECT_DISABLE;

// Advertising handles
static uint8 advHandleLegacy;
static uint8 advHandleLongRange;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimplePeripheral_init( void );
static void SimplePeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg);
static void SimplePeripheral_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData);
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg);
static void SimplePeripheral_updateRPA(void);
static void SimplePeripheral_clockHandler(UArg arg);
static void SimplePeripheral_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs,
                                        uint32_t numComparison);
static void SimplePeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status);
static void SimplePeripheral_processPairState(spPairStateData_t *pPairState);
static void SimplePeripheral_processPasscode(spPasscodeData_t *pPasscodeData);
static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void SimplePeripheral_initPHYRSSIArray(void);
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg);
static uint8_t SimplePeripheral_addConn(uint16_t connHandle);
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle);
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle);
static void SimplePeripheral_processParamUpdate(uint16_t connHandle);
static status_t SimplePeripheral_startAutoPhyChange(uint16_t connHandle);
static status_t SimplePeripheral_stopAutoPhyChange(uint16_t connHandle);
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts);
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle);
static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport);
#ifdef PTM_MODE
void simple_peripheral_handleNPIRxInterceptEvent(uint8_t *pMsg);  // Declaration
static void simple_peripheral_sendToNPI(uint8_t *buf, uint16_t len);  // Declaration
#endif // PTM_MODE



/*********************************************************************/
// OpenWind Functions
/*********************************************************************/


// How often to perform periodic event (in ms)
#define OW_PERIODIC_EVT_DIR               100
#define OW_PERIODIC_EVT_MOV               1000
#define OW_PERIODIC_EVT_BAT               1000
#define OW_PERIODIC_EVT_GPS               250
#define OW_PERIODIC_EVT_LED               10000
#define OW_PERIODIC_EVT_LED_Connected     5000
#define OW_PERIODIC_EVT_LED_OFF           100

static void OpenWind_Clock_Handler(UArg arg);
static void OpenWind_power_up();
static void OpenWind_power_down();
static void OpenWind_DIRECTION_Task();
static void OpenWind_MOVEMENT_Task();
static void OpenWind_GPS_Task();
static void OpenWind_GPS_Task_READ();
static void OpenWind_BATTERY_Task();
static void OpenWind_LED_Task();
static void OpenWind_toggleAdvertising();
static void OpenWind_setDeviceInfo();

static Clock_Struct ow_clk_wind_dir;
static Clock_Struct ow_clk_mov;
static Clock_Struct ow_clk_gps;
static Clock_Struct ow_clk_bat;
static Clock_Struct ow_clk_LED;

extern bool OpenWind_START_DIRECTION = false;
extern bool OpenWind_START_MOV = false;
extern bool OpenWind_START_GPS = false;
extern bool OpenWind_START_BAT = false;
extern bool OpenWind_has_Connection = false;


extern float Wind_Direction=0;
extern bool Wind_Direction_Scale_Flag=false;
extern uint8_t wert_von_handy=0;
extern double Wind_Speed=0;


extern float AWA_offset=0;
extern float AWS_offset=0;

extern uint8_t keep_on=0;
extern uint8_t bno_low_power=0;
extern int16_t MastAngle=0;
extern uint8_t Flag_ADV=0;

extern uint16_t OpenWind_SOG=0;
extern uint16_t OpenWind_COG=0;

NVS_Handle nvsHandle;
NVS_Attrs regionAttrs;
NVS_Params nvsParams;
void OpenWindFlash_Init();
#define SNV_ID_APP 0x80
uint8_t bufSen[32];
uint8_t bufRec[32];

static const uint8_t devInfoModelNumber[] = "OpenWind";
static const uint8_t devInfoNA[] =          "";
static const uint8_t devInfoFirmwareRev[] = "1.37";
static const uint8_t devInfoMfrName[] =     "Liesenberg GmbH";
static const uint8_t *devInfoHardwareRev =  devInfoNA;

uint8_t serial_number[B_ADDR_LEN]={0,0,0,0,0,0};
char string_array[B_ADDR_LEN];

static uint8_t advertData[] =
{
 #ifdef PLUS_BROADCASTER_VALUES_NEW//PLUS_BROADCASTER_VALUES
     0x02,   // length of this data
     GAP_ADTYPE_FLAGS,
     GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,//GAP_ADTYPE_FLAGS_LIMITED: Broadcast - GAP_ADTYPE_FLAGS_GENERAL: Connection

     0x03,   // length of this data
     GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
     LO_UINT16(WIND_SERV_UUID),
     HI_UINT16(WIND_SERV_UUID),

    0x17,//0x19, //0x15 geht soll aber 0x17
    GAP_ADTYPE_MANUFACTURER_SPECIFIC,
    0xD0,//LO_UINT16(LIESENBERG_COMPANY_ID),
    0x00,//HI_UINT16(LIESENBERG_COMPANY_ID),
    0x00, //Wind Direction 7
    0x00,
    0x00, //Wind Speed 9
    0x00,
    0x00, //Yaw 11
    0x00,
    0x00, //Pitch 13
    0x00,
    0x00, //Row 15
    0x00,
    0x00, //Level 17
    0x00,
    0x00, //temperature 19
    0x00,
    0x00, //Voltage 21
    0x00,
    0x00,//Remain 23
    0x00,
    0x00,//Total   25
    0x00
//    0x00,//Current 37
//    0x00

 #endif
   // Manufacturer specific advertising data
 #ifdef PLUS_BROADCASTER_VALUES
    0x03,   // length of this data
    GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
    LO_UINT16(WIND_SERV_UUID),
    HI_UINT16(WIND_SERV_UUID),
    //LO_UINT16(OAD_SERVICE_UUID),
    //HI_UINT16(OAD_SERVICE_UUID),
   0x19,
   GAP_ADTYPE_MANUFACTURER_SPECIFIC,
   LO_UINT16(LIESENBERG_COMPANY_ID),
   HI_UINT16(LIESENBERG_COMPANY_ID),
   0x00, //Wind Direction 8
   0x00,
   0x00, //Wind Speed 10
   0x00,
   0x00, //Yaw 12
   0x00,
   0x00, //Pitch 14
   0x00,
   0x00, //Row 16
   0x00,
   0x00, //Level 18
   0x00,
   0x00, //temperature 20
   0x00,
   0x00, //Voltage 22
   0x00,
   0x00,//Remain 24
   0x00,
   0x00,//Total 26
   0x00,
   0x00,//Current 28
   0x00
 #endif
};

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Bond Manager Callbacks
static gapBondCBs_t SimplePeripheral_BondMgrCBs =
{
  SimplePeripheral_passcodeCb,       // Passcode callback
  SimplePeripheral_pairStateCb       // Pairing/Bonding state Callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      simple_peripheral_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void simple_peripheral_spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}

#ifdef PTM_MODE
/*********************************************************************
* @fn      simple_peripheral_handleNPIRxInterceptEvent
*
* @brief   Intercept an NPI RX serial message and queue for this application.
*
* @param   pMsg - a NPIMSG_msg_t containing the intercepted message.
*
* @return  none.
*/
void simple_peripheral_handleNPIRxInterceptEvent(uint8_t *pMsg)
{
 // Send Command via HCI TL
 HCI_TL_SendToStack(((NPIMSG_msg_t *)pMsg)->pBuf);

 // The data is stored as a message, free this first.
 ICall_freeMsg(((NPIMSG_msg_t *)pMsg)->pBuf);

 // Free container.
 ICall_free(pMsg);
}

/*********************************************************************
* @fn      simple_peripheral_sendToNPI
*
* @brief   Create an NPI packet and send to NPI to transmit.
*
* @param   buf - pointer HCI event or data.
*
* @param   len - length of buf in bytes.
*
* @return  none
*/
static void simple_peripheral_sendToNPI(uint8_t *buf, uint16_t len)
{
 npiPkt_t *pNpiPkt = (npiPkt_t *)ICall_allocMsg(sizeof(npiPkt_t) + len);

 if (pNpiPkt)
 {
   pNpiPkt->hdr.event = buf[0]; //Has the event status code in first byte of payload
   pNpiPkt->hdr.status = 0xFF;
   pNpiPkt->pktLen = len;
   pNpiPkt->pData  = (uint8 *)(pNpiPkt + 1);

   memcpy(pNpiPkt->pData, buf, len);

   // Send to NPI
   // Note: there is no need to free this packet.  NPI will do that itself.
   NPITask_sendToHost((uint8_t *)pNpiPkt);
 }
}
#endif // PTM_MODE

/*********************************************************************
 * @fn      SimplePeripheral_createTask
 *
 * @brief   Task creation function for the Simple Peripheral.
 */
void SimplePeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = spTaskStack;
  taskParams.stackSize = SP_TASK_STACK_SIZE;
  taskParams.priority = SP_TASK_PRIORITY;

  Task_construct(&spTask, SimplePeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimplePeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void SimplePeripheral_init(void)
{

    GPIO_setConfig(RED_LED, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(GREEN_LED, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(BLUE_LED, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(LOAD_CTRL, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);

    GPIO_write(RED_LED, 1);
    GPIO_write(GREEN_LED, 1);
    GPIO_write(BLUE_LED, 0);
    GPIO_write(LOAD_CTRL, 1);

    //bspSpiOpen();
   // AS5055_init();
   // Flash_Open_Pin();

    delay_ms(500);
    SensorI2C_open();


  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  // Set device's Sleep Clock Accuracy
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
  HCI_EXT_SetSCACmd(500);
#endif // (CENTRAL_CFG | PERIPHERAL_CFG)
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueueHandle = Util_constructQueue(&appMsgQueue);


  Util_constructClock(&ow_clk_wind_dir, OpenWind_Clock_Handler,
                      OW_PERIODIC_EVT_DIR, 0, false, OW_WIND_DIR);
  Util_constructClock(&ow_clk_mov, OpenWind_Clock_Handler,
                      OW_PERIODIC_EVT_MOV, 0, false, OW_WIND_MOV);
  Util_constructClock(&ow_clk_gps, OpenWind_Clock_Handler,
                      OW_PERIODIC_EVT_GPS, 0, false, OW_WIND_GPS);
  Util_constructClock(&ow_clk_bat, OpenWind_Clock_Handler,
                      OW_PERIODIC_EVT_BAT, 0, false, OW_WIND_BAT);
  Util_constructClock(&ow_clk_LED, OpenWind_Clock_Handler,
                      OW_PERIODIC_EVT_LED, 0, false, OW_WIND_LED);


  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Configure GAP
  {
    uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

    // Pass all parameter update requests to the app for it to decide
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
  }

  // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
  // section in the User's Guide
  setBondManagerParameters();

  // Initialize GATT attributes
  GGS_AddService(GAP_SERVICE);                 // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
  DevInfo_AddService();                        // Device Information Service
  SensorTagBatt_init();
  SensorTagWind_init();
  SensorTagMov_init();
  SensorI2C_close();

  OpenWindFlash_Init();
  OpenWind_setDeviceInfo();


  // Start Bond Manager and register callback
  VOID GAPBondMgr_Register(&SimplePeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the HCI section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/<
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  {
    // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    // Some brand smartphone is essentially needing 251/2120, so we set them here.
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    // This API is documented in hci.h
    // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Initialize GATT Client
  GATT_InitClient("");


  // Initialize Connection List
  SimplePeripheral_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

  //Initialize GAP layer for Peripheral role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, &pRandomAddress);

  // Initialize array to store connection handle and RSSI values
  SimplePeripheral_initPHYRSSIArray();


  SensorI2C_close();
  GPIO_write(BLUE_LED, 1);

  Util_startClock(&ow_clk_LED);


#ifdef PTM_MODE
  // Intercept NPI RX events.
  NPITask_registerIncomingRXEventAppCB(simple_peripheral_handleNPIRxInterceptEvent, INTERCEPT);

  // Register for Command Status information
  HCI_TL_Init(NULL, (HCI_TL_CommandStatusCB_t) simple_peripheral_sendToNPI, NULL, selfEntity);

  // Register for Events
  HCI_TL_getCmdResponderID(ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, selfEntity));

  // Inform Stack to Initialize PTM
  HCI_EXT_EnablePTMCmd();
#endif // PTM_MODE
}

/*********************************************************************
 * @fn      SimplePeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple Peripheral.
 *
 * @param   a0, a1 - not used.
 */
static void SimplePeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimplePeripheral_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, SP_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Fetch any available messages that might have been sent from the stack
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = SimplePeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & SP_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueueHandle))
        {
          spEvt_t *pMsg = (spEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
          if (pMsg)
          {
            // Process message.
            SimplePeripheral_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
      if (events & OW_WIND_DIR) {
        OpenWind_DIRECTION_Task();
      }
      if (events & OW_WIND_MOV) {
          OpenWind_MOVEMENT_Task();
      }
      if (events & OW_WIND_BAT) {
          OpenWind_BATTERY_Task();
      }
      if (events & OW_WIND_LED) {
          OpenWind_LED_Task();
      }
      if (events & OW_WIND_GPS) {
          OpenWind_GPS_Task();
      }
      if (events & OW_WIND_GPS_READ) {
          OpenWind_GPS_Task_READ();
      }
      if (events & OW_AS5055_READ) {
          AS5055_Handle_INT_Read_SPI();
      }
      if (events & MOV_INT_EVENT) {
          SensorTagMov_INT_Handle();
      }
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  // Always dealloc pMsg unless set otherwise
  uint8_t safeToDealloc = TRUE;

  BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : Stack msg status=%d, event=0x%x\n", pMsg->status, pMsg->event);

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimplePeripheral_processGapMessage((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimplePeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Events here
        {
          SimplePeripheral_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
          break;
        }

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;

        // HCI Commands Events
        case HCI_COMMAND_STATUS_EVENT_CODE:
        {
          hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;
          switch ( pMyMsg->cmdOpcode )
          {
            case HCI_LE_SET_PHY:
            {
              if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
              {
              }
              else
              {
              }

              SimplePeripheral_updatePHYStat(HCI_LE_SET_PHY, (uint8_t *)pMsg);
              break;
            }

            default:
              break;
          }
          break;
        }

        // LE Events
        case HCI_LE_EVENT_CODE:
        {
          hciEvt_BLEPhyUpdateComplete_t *pPUC =
            (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

          // A Phy Update Has Completed or Failed
          if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
          {
            if (pPUC->status != SUCCESS)
            {
            }
            else
            {
              // Only symmetrical PHY is supported.
              // rxPhy should be equal to txPhy.
            }

            SimplePeripheral_updatePHYStat(HCI_BLE_PHY_UPDATE_COMPLETE_EVENT, (uint8_t *)pMsg);
          }
          break;
        }

        default:
          break;
      }

      break;
    }

    default:
      // do nothing
      break;
  }

#ifdef PTM_MODE
  // Check for NPI Messages
  hciPacket_t *pBuf = (hciPacket_t *)pMsg;

  // Serialized HCI Event
  if (pBuf->hdr.event == HCI_CTRL_TO_HOST_EVENT)
  {
    uint16_t len = 0;

    // Determine the packet length
    switch(pBuf->pData[0])
    {
      case HCI_EVENT_PACKET:
        len = HCI_EVENT_MIN_LENGTH + pBuf->pData[2];
        break;

      case HCI_ACL_DATA_PACKET:
        len = HCI_DATA_MIN_LENGTH + BUILD_UINT16(pBuf->pData[3], pBuf->pData[4]);
        break;

      default:
        break;
    }

    // Send to Remote Host.
    simple_peripheral_sendToNPI(pBuf->pData, len);

    // Free buffers if needed.
    switch (pBuf->pData[0])
    {
      case HCI_ACL_DATA_PACKET:
      case HCI_SCO_DATA_PACKET:
        BM_free(pBuf->pData);
      default:
        break;
    }
  }
#endif // PTM_MODE

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimplePeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimplePeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg)
{
  bool dealloc = TRUE;



  switch (pMsg->event)
  {
    case SP_ADV_EVT:
      SimplePeripheral_processAdvEvent((spGapAdvEventData_t*)(pMsg->pData));
      break;

    case SP_PAIR_STATE_EVT:
      SimplePeripheral_processPairState((spPairStateData_t*)(pMsg->pData));
      break;

    case SP_PASSCODE_EVT:
      SimplePeripheral_processPasscode((spPasscodeData_t*)(pMsg->pData));
      break;

    case SP_READ_RPA_EVT:
      SimplePeripheral_updateRPA();
      break;

    case SP_SEND_PARAM_UPDATE_EVT:
    {
      // Extract connection handle from data
      uint16_t connHandle = *(uint16_t *)(((spClockEventData_t *)pMsg->pData)->data);

      SimplePeripheral_processParamUpdate(connHandle);

      // This data is not dynamically allocated
      dealloc = FALSE;
      break;
    }

    case SP_CONN_EVT:
      SimplePeripheral_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));
      break;

    case OW_CHAR_DIR: {
            OpenWind_START_DIRECTION = true;
          break;
      }
    case OW_CHAR_MOV: {
        OpenWind_START_MOV = true;
          break;
      }
    case OW_CHAR_GPS: {
        OpenWind_START_GPS = true;
          break;
      }
    case SERVICE_ID_Wind: {
          OpenWind_START_DIRECTION = true;
          parameter_wind = (uint8_t)(((openWindData_t *)pMsg->pData)->value);
          SensorTagWind_processCharChangeEvt(parameter_wind);
          if(!OpenWind_START_GPS) {
              OpenWind_START_GPS = true;
              YIC71513PGMGG_uart_open();
          }
          break;
      }

    case SERVICE_ID_MOV:  {

        if(!OpenWind_START_MOV) {

          parameter_mov = (uint8_t)(((openWindData_t *)pMsg->pData)->value);
          SensorTagMov_processCharChangeEvt(parameter_mov);
          OpenWind_START_MOV = true;
        }
        break;
      }
    case OW_CHAR_BAT: {
      if(!OpenWind_START_BAT) {
          OpenWind_START_BAT = true;
          BQ27441_Setup();
          SensorTagBatt_processSensorEvent();
      }
      break;
    }
    default:
      // Do nothing.
      break;
  }

  // Free message data if it exists and we are to dealloc
  if ((dealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg)
{
  switch(pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      bStatus_t status = FAILURE;

      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      if(pPkt->hdr.status == SUCCESS)
      {
        // Store the system ID
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = pPkt->devAddr[0];
        systemId[1] = pPkt->devAddr[1];
        systemId[2] = pPkt->devAddr[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = pPkt->devAddr[5];
        systemId[6] = pPkt->devAddr[4];
        systemId[5] = pPkt->devAddr[3];

        // Set Device Info Service Parameter
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);


        // Setup and start Advertising
        // For more information, see the GAP section in the User's Guide:
        // http://software-dl.ti.com/lprf/ble5stack-latest/

        // Create Advertisement set #1 and assign handle
        status = GapAdv_create(&SimplePeripheral_advCallback, &advParams1,
                               &advHandleLegacy);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load advertising data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV, sizeof(advertData), advertData);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load scan response data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP, sizeof(scanResData1), scanResData1);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Set event mask for set #1
        status = GapAdv_setEventMask(advHandleLegacy,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable legacy advertising for set #1
        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- call GapAdv_create set=%d,%d\n", 1, 0);
        // Create Advertisement set #2 and assign handle
        status = GapAdv_create(&SimplePeripheral_advCallback, &advParams2,
                               &advHandleLongRange);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load advertising data for set #2 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLongRange, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advData2), advData2);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Set event mask for set #2
        status = GapAdv_setEventMask(advHandleLongRange,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable long range advertising for set #2
        status = GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);


        if (addrMode > ADDRMODE_RANDOM)
        {
          SimplePeripheral_updateRPA();

          // Create one-shot clock for RPA check event.
          Util_constructClock(&clkRpaRead, SimplePeripheral_clockHandler,
                              READ_RPA_PERIOD, 0, true,
                              (UArg) &argRpaRead);
        }
      }

      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive("");

      if (pPkt->hdr.status == SUCCESS)
      {
        // Add connection to list and start RSSI
        SimplePeripheral_addConn(pPkt->connectionHandle);

        OpenWind_power_up();
      }


      if ((numActive < MAX_NUM_BLE_CONNS) && (autoConnect == AUTOCONNECT_DISABLE))
      {
        // Start advertising since there is room for more connections
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      }
      else
      {
        // Stop advertising since there is no room for more connections
        GapAdv_disable(advHandleLongRange);
        GapAdv_disable(advHandleLegacy);
      }
      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive("");

      // Remove the connection from the list and disable RSSI if needed
      SimplePeripheral_removeConn(pPkt->connectionHandle);

      // If no active connections
      if (numActive == 0)
      {
          OpenWind_power_down();

      }

      BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : GAP msg: status=%d, opcode=%s\n", 0, "GAP_LINK_TERMINATED_EVENT");
      // Start advertising since there is room for more connections
      GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);


      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
      gapUpdateLinkParamReqReply_t rsp;

      gapUpdateLinkParamReqEvent_t *pReq = (gapUpdateLinkParamReqEvent_t *)pMsg;

      rsp.connectionHandle = pReq->req.connectionHandle;
      rsp.signalIdentifier = pReq->req.signalIdentifier;

      // Only accept connection intervals with slave latency of 0
      // This is just an example of how the application can send a response
      if(pReq->req.connLatency == 0)
      {
        rsp.intervalMin = pReq->req.intervalMin;
        rsp.intervalMax = pReq->req.intervalMax;
        rsp.connLatency = pReq->req.connLatency;
        rsp.connTimeout = pReq->req.connTimeout;
        rsp.accepted = TRUE;
      }
      else
      {
        rsp.accepted = FALSE;
      }

      // Send Reply
      VOID GAP_UpdateLinkParamReqReply(&rsp);

      break;
    }

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

      // Get the address from the connection handle
      linkDBInfo_t linkInfo;
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

      if(pPkt->status == SUCCESS)
      {
        // Display the address of the connection update
      }
      else
      {
        // Display the address of the connection update failure
      }

      // Check if there are any queued parameter updates
      spConnHandleEntry_t *connHandleEntry = (spConnHandleEntry_t *)List_get(&paramUpdateList);
      if (connHandleEntry != NULL)
      {
        // Attempt to send queued update now
        SimplePeripheral_processParamUpdate(connHandleEntry->connHandle);

        // Free list element
        ICall_free(connHandleEntry);
      }

      break;
    }

#if defined ( NOTIFY_PARAM_UPDATE_RJCT )
    case GAP_LINK_PARAM_UPDATE_REJECT_EVENT:
    {
      linkDBInfo_t linkInfo;
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

      // Get the address from the connection handle
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

      // Display the address of the connection update failure

      break;
    }
#endif

    default:

      break;
  }
}



/*********************************************************************
 * @fn      SimplePeripheral_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_updateRPA(void)
{
  uint8_t* pRpaNew;

  // Read the current RPA.
  pRpaNew = GAP_GetDevAddress(FALSE);

  if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
  {
    // If the RPA has changed, update the display
    memcpy(rpa, pRpaNew, B_ADDR_LEN);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimplePeripheral_clockHandler(UArg arg)
{
  spClockEventData_t *pData = (spClockEventData_t *)arg;

 if (pData->event == SP_READ_RPA_EVT)
 {
   // Start the next period
   Util_startClock(&clkRpaRead);

   // Post event to read the current RPA
   SimplePeripheral_enqueueMsg(SP_READ_RPA_EVT, NULL);
 }
 else if (pData->event == SP_SEND_PARAM_UPDATE_EVT)
 {
    // Send message to app
    SimplePeripheral_enqueueMsg(SP_SEND_PARAM_UPDATE_EVT, pData);
 }
}


/*********************************************************************
 * @fn      SimplePeripheral_doSetConnPhy
 *
 * @brief   Set PHY preference.
 *
 * @param   index - 0: 1M PHY
 *                  1: 2M PHY
 *                  2: 1M + 2M PHY
 *                  3: CODED PHY (Long range)
 *                  4: 1M + 2M + CODED PHY
 *
 * @return  always true
 */
bool SimplePeripheral_doSetConnPhy(uint8 index)
{
  bool status = TRUE;

  static uint8_t phy[] = {
    HCI_PHY_1_MBPS, HCI_PHY_2_MBPS, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS,
    HCI_PHY_CODED, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS | HCI_PHY_CODED,
    AUTO_PHY_UPDATE
  };

  uint8_t connIndex = SimplePeripheral_getConnIndex(menuConnHandle);
  if (connIndex >= MAX_NUM_BLE_CONNS)
  {
    return FALSE;
  }

  // Set Phy Preference on the current connection. Apply the same value
  // for RX and TX.
  // If auto PHY update is not selected and if auto PHY update is enabled, then
  // stop auto PHY update
  // Note PHYs are already enabled by default in build_config.opt in stack project.
  if(phy[index] != AUTO_PHY_UPDATE)
  {
    // Cancel RSSI reading  and auto phy changing
    SimplePeripheral_stopAutoPhyChange(connList[connIndex].connHandle);

    SimplePeripheral_setPhy(menuConnHandle, 0, phy[index], phy[index], 0);

  }
  else
  {
    // Start RSSI read for auto PHY update (if it is disabled)
    SimplePeripheral_startAutoPhyChange(menuConnHandle);
  }

  return status;
}
/*********************************************************************
 * @fn      SimplePeripheral_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void SimplePeripheral_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
  spGapAdvEventData_t *pData = ICall_malloc(sizeof(spGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(SimplePeripheral_enqueueMsg(SP_ADV_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData)
{
  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:

      break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:

      break;

    case GAP_EVT_ADV_START:
      break;

    case GAP_EVT_ADV_END:
      break;

    case GAP_EVT_ADV_SET_TERMINATED:
    {
#ifndef Display_DISABLE_ALL
      GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);


#endif
    }
    break;

    case GAP_EVT_SCAN_REQ_RECEIVED:
      break;

    case GAP_EVT_INSUFFICIENT_MEMORY:
      break;

    default:
      break;
  }

  // All events have associated memory to free except the insufficient memory
  // event
  if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
  {
    ICall_free(pEventData->pBuf);
  }
}


/*********************************************************************
 * @fn      SimplePeripheral_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimplePeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  spPairStateData_t *pData = ICall_malloc(sizeof(spPairStateData_t));

  // Allocate space for the event data.
  if (pData)
  {
    pData->state = state;
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if(SimplePeripheral_enqueueMsg(SP_PAIR_STATE_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimplePeripheral_passcodeCb(uint8_t *pDeviceAddr,
                                        uint16_t connHandle,
                                        uint8_t uiInputs,
                                        uint8_t uiOutputs,
                                        uint32_t numComparison)
{
  spPasscodeData_t *pData = ICall_malloc(sizeof(spPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData )
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    if(SimplePeripheral_enqueueMsg(SP_PASSCODE_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimplePeripheral_processPairState(spPairStateData_t *pPairData)
{
  uint8_t state = pPairData->state;
  uint8_t status = pPairData->status;

  switch (state)
  {
    case GAPBOND_PAIRING_STATE_STARTED:

      break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
      if (status == SUCCESS)
      {

      }
      else
      {

      }
      break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
      if (status == SUCCESS)
      {

      }
      else
      {

      }
      break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
      if (status == SUCCESS)
      {

      }
      else
      {

      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimplePeripheral_processPasscode(spPasscodeData_t *pPasscodeData)
{
  // Display passcode to user
  if (pPasscodeData->uiOutputs != 0)
  {
    //Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Passcode: %d",B_APP_DEFAULT_PASSCODE);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle , SUCCESS,
                         B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      SimplePeripheral_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if(SimplePeripheral_enqueueMsg(SP_CONN_EVT, pReport) != SUCCESS)
  {
    ICall_free(pReport);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
  // Get index from handle
  uint8_t connIndex = SimplePeripheral_getConnIndex(pReport->handle);

  if (connIndex >= MAX_NUM_BLE_CONNS)
  {
    return;
  }

  // If auto phy change is enabled
  if (connList[connIndex].isAutoPHYEnable == TRUE)
  {
    // Read the RSSI
    HCI_ReadRssiCmd(pReport->handle);
  }
}


/*********************************************************************
 * @fn      SimplePeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
uint8 SimplePeripheral_enqueueMsg(uint8_t event, void *pData)
{
  uint8_t success;
  spEvt_t *pMsg = ICall_malloc(sizeof(spEvt_t));

  // Create dynamic pointer to message.
  if(pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }

  return(bleMemAllocError);
}


/*********************************************************************
 * @fn      SimplePeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
void OpenWind_enqueueMsg(uint8_t event, uint8_t value)
{
    openWindData_t *pData = ICall_malloc(sizeof(openWindData_t));

    // Allocate space for the passcode event.
    if (pData )
    {
        pData->value = value;

        // Enqueue the event.
        if(SimplePeripheral_enqueueMsg(event, pData) != SUCCESS)
        {
          ICall_free(pData);
        }
    }
}


/*********************************************************************
 * @fn      SimplePeripheral_doSelectConn
 *
 * @brief   Select a connection to communicate with
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SimplePeripheral_doSelectConn(uint8_t index)
{
  menuConnHandle = connList[index].connHandle;


  return (true);
}
/*********************************************************************
 * @fn      SimplePeripheral_doAutoConnect
 *
 * @brief   Enable/Disable peripheral as AutoConnect node.
 *
 * @param   index - 0 : Disable AutoConnect
 *                  1 : Enable Group A
 *                  2 : Enable Group B
 *
 * @return  always true
 */
bool SimplePeripheral_doAutoConnect(uint8_t index)
{
    if (index == 1)
    {
      if (autoConnect != AUTOCONNECT_GROUP_A)
      {
        GapAdv_disable(advHandleLongRange);
        GapAdv_disable(advHandleLegacy);
        advData1[2] = 'G';
        advData1[3] = 'A';
        advData2[2] = 'G';
        advData2[3] = 'A';
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        autoConnect = AUTOCONNECT_GROUP_A;
      }
    }
    else if (index == 2)
    {
      if (autoConnect != AUTOCONNECT_GROUP_B)
      {
        GapAdv_disable(advHandleLongRange);
        GapAdv_disable(advHandleLegacy);
        advData1[2] = 'G';
        advData1[3] = 'B';
        advData2[2] = 'G';
        advData2[3] = 'B';
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        autoConnect = AUTOCONNECT_GROUP_B;
      }
    }
    else
    {
      if (autoConnect)
      {
        GapAdv_disable(advHandleLongRange);
        GapAdv_disable(advHandleLegacy);
        advData1[2] = 'S';
        advData1[3] = 'P';
        advData2[2] = 'S';
        advData2[3] = 'P';
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        autoConnect = AUTOCONNECT_DISABLE;
      }
    }


    return (true);
}


/*********************************************************************
 * @fn      SimplePeripheral_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_addConn(uint16_t connHandle)
{
  uint8_t i;
  uint8_t status = bleNoResources;

  // Try to find an available entry
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;

#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
      // Allocate data to send through clock handler
      connList[i].pParamUpdateEventData = ICall_malloc(sizeof(spClockEventData_t) +
                                                       sizeof (uint16_t));
      if(connList[i].pParamUpdateEventData)
      {
        connList[i].pParamUpdateEventData->event = SP_SEND_PARAM_UPDATE_EVT;
        *((uint16_t *)connList[i].pParamUpdateEventData->data) = connHandle;

        // Create a clock object and start
        connList[i].pUpdateClock
          = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

        if (connList[i].pUpdateClock)
        {
          Util_constructClock(connList[i].pUpdateClock,
                              SimplePeripheral_clockHandler,
                              SEND_PARAM_UPDATE_DELAY, 0, true,
                              (UArg) (connList[i].pParamUpdateEventData));
        }
        else
        {
            ICall_free(connList[i].pParamUpdateEventData);
        }
      }
      else
      {
        status = bleMemAllocError;
      }
#endif

      // Set default PHY to 1M
      connList[i].currPhy = HCI_PHY_1_MBPS;

      break;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return i;
    }
  }

  return(MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. LINKDB_CONNHANDLE_ALL will always succeed.
 */
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle)
{
  uint8_t i;
  // Set to invalid connection index initially
  uint8_t connIndex = MAX_NUM_BLE_CONNS;

  if(connHandle != LINKDB_CONNHANDLE_ALL)
  {
    // Get connection index from handle
    connIndex = SimplePeripheral_getConnIndex(connHandle);
    if(connIndex >= MAX_NUM_BLE_CONNS)
	{
	  return(bleInvalidRange);
	}
  }

  // Clear specific handle or all handles
  for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL))
    {
      connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
      connList[i].currPhy = 0;
      connList[i].phyCngRq = 0;
      connList[i].phyRqFailCnt = 0;
      connList[i].rqPhy = 0;
      memset(connList[i].rssiArr, 0, SP_MAX_RSSI_STORE_DEPTH);
      connList[i].rssiAvg = 0;
      connList[i].rssiCntr = 0;
      connList[i].isAutoPHYEnable = FALSE;
    }
  }

  return(SUCCESS);
}

/*********************************************************************
 * @fn      SimplePeripheral_clearPendingParamUpdate
 *
 * @brief   clean pending param update request in the paramUpdateList list
 *
 * @param   connHandle - connection handle to clean
 *
 * @return  none
 */
void SimplePeripheral_clearPendingParamUpdate(uint16_t connHandle)
{
  List_Elem *curr;

  for (curr = List_head(&paramUpdateList); curr != NULL; curr = List_next(curr)) 
  {
    if (((spConnHandleEntry_t *)curr)->connHandle == connHandle)
    {
      List_remove(&paramUpdateList, curr);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle)
{
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);

  if(connIndex != MAX_NUM_BLE_CONNS)
  {
    Clock_Struct* pUpdateClock = connList[connIndex].pUpdateClock;

    if (pUpdateClock != NULL)
    {
      // Stop and destruct the RTOS clock if it's still alive
      if (Util_isActive(pUpdateClock))
      {
        Util_stopClock(pUpdateClock);
      }

      // Destruct the clock object
      Clock_destruct(pUpdateClock);
      // Free clock struct
      ICall_free(pUpdateClock);
      // Free ParamUpdateEventData
      ICall_free(connList[connIndex].pParamUpdateEventData);
    }
    // Clear pending update requests from paramUpdateList
    SimplePeripheral_clearPendingParamUpdate(connHandle);
    // Stop Auto PHY Change
    SimplePeripheral_stopAutoPhyChange(connHandle);
    // Clear Connection List Entry
    SimplePeripheral_clearConnListEntry(connHandle);
  }

  return connIndex;
}

/*********************************************************************
 * @fn      SimplePeripheral_processParamUpdate
 *
 * @brief   Process a parameters update request
 *
 * @return  None
 */
static void SimplePeripheral_processParamUpdate(uint16_t connHandle)
{
  gapUpdateLinkParamReq_t req;
  uint8_t connIndex;

  req.connectionHandle = connHandle;
#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
  req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
  req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
  req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
  req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
#endif

  connIndex = SimplePeripheral_getConnIndex(connHandle);
  if (connIndex >= MAX_NUM_BLE_CONNS)
  {
    return;
  }

  // Deconstruct the clock object
  Clock_destruct(connList[connIndex].pUpdateClock);
  // Free clock struct, only in case it is not NULL
  if (connList[connIndex].pUpdateClock != NULL)
  {
      ICall_free(connList[connIndex].pUpdateClock);
      connList[connIndex].pUpdateClock = NULL;
  }
  // Free ParamUpdateEventData, only in case it is not NULL
  if (connList[connIndex].pParamUpdateEventData != NULL)
      ICall_free(connList[connIndex].pParamUpdateEventData);

  // Send parameter update
  bStatus_t status = GAP_UpdateLinkParamReq(&req);

  // If there is an ongoing update, queue this for when the udpate completes
  if (status == bleAlreadyInRequestedMode)
  {
    spConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(spConnHandleEntry_t));
    if (connHandleEntry)
    {
      connHandleEntry->connHandle = connHandle;

      List_put(&paramUpdateList, (List_Elem *)connHandleEntry);
    }
  }
}

/*********************************************************************
 * @fn      SimpleCentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  uint8_t status = pMsg->pReturnParam[0];

  //Find which command this command complete is for
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
    {
      int8 rssi = (int8)pMsg->pReturnParam[3];

      // Display RSSI value, if RSSI is higher than threshold, change to faster PHY
      if (status == SUCCESS)
      {
        uint16_t handle = BUILD_UINT16(pMsg->pReturnParam[1], pMsg->pReturnParam[2]);

        uint8_t index = SimplePeripheral_getConnIndex(handle);
        if (index >= MAX_NUM_BLE_CONNS)
        {
          return;
        }

        if (rssi != LL_RSSI_NOT_AVAILABLE)
        {
          connList[index].rssiArr[connList[index].rssiCntr++] = rssi;
          connList[index].rssiCntr %= SP_MAX_RSSI_STORE_DEPTH;

          int16_t sum_rssi = 0;
          for(uint8_t cnt=0; cnt<SP_MAX_RSSI_STORE_DEPTH; cnt++)
          {
            sum_rssi += connList[index].rssiArr[cnt];
          }
          connList[index].rssiAvg = (uint32_t)(sum_rssi/SP_MAX_RSSI_STORE_DEPTH);

          uint8_t phyRq = SP_PHY_NONE;
          uint8_t phyRqS = SP_PHY_NONE;
          uint8_t phyOpt = LL_PHY_OPT_NONE;

          if(connList[index].phyCngRq == FALSE)
          {
            if((connList[index].rssiAvg >= RSSI_2M_THRSHLD) &&
            (connList[index].currPhy != HCI_PHY_2_MBPS) &&
                 (connList[index].currPhy != SP_PHY_NONE))
            {
              // try to go to higher data rate
              phyRqS = phyRq = HCI_PHY_2_MBPS;
            }
            else if((connList[index].rssiAvg < RSSI_2M_THRSHLD) &&
                    (connList[index].rssiAvg >= RSSI_1M_THRSHLD) &&
                    (connList[index].currPhy != HCI_PHY_1_MBPS) &&
                    (connList[index].currPhy != SP_PHY_NONE))
            {
              // try to go to legacy regular data rate
              phyRqS = phyRq = HCI_PHY_1_MBPS;
            }
            else if((connList[index].rssiAvg >= RSSI_S2_THRSHLD) &&
                    (connList[index].rssiAvg < RSSI_1M_THRSHLD) &&
                    (connList[index].currPhy != SP_PHY_NONE))
            {
              // try to go to lower data rate S=2(500kb/s)
              phyRqS = HCI_PHY_CODED;
              phyOpt = LL_PHY_OPT_S2;
              phyRq = BLE5_CODED_S2_PHY;
            }
            else if(connList[index].rssiAvg < RSSI_S2_THRSHLD )
            {
              // try to go to lowest data rate S=8(125kb/s)
              phyRqS = HCI_PHY_CODED;
              phyOpt = LL_PHY_OPT_S8;
              phyRq = BLE5_CODED_S8_PHY;
            }
            if((phyRq != SP_PHY_NONE) &&
               // First check if the request for this phy change is already not honored then don't request for change
               (((connList[index].rqPhy == phyRq) &&
                 (connList[index].phyRqFailCnt < 2)) ||
                 (connList[index].rqPhy != phyRq)))
            {
              //Initiate PHY change based on RSSI
              SimplePeripheral_setPhy(connList[index].connHandle, 0,
                                      phyRqS, phyRqS, phyOpt);
              connList[index].phyCngRq = TRUE;

              // If it a request for different phy than failed request, reset the count
              if(connList[index].rqPhy != phyRq)
              {
                // then reset the request phy counter and requested phy
                connList[index].phyRqFailCnt = 0;
              }

              if(phyOpt == LL_PHY_OPT_NONE)
              {
                connList[index].rqPhy = phyRq;
              }
              else if(phyOpt == LL_PHY_OPT_S2)
              {
                connList[index].rqPhy = BLE5_CODED_S2_PHY;
              }
              else
              {
                connList[index].rqPhy = BLE5_CODED_S8_PHY;
              }

            } // end of if ((phyRq != SP_PHY_NONE) && ...
          } // end of if (connList[index].phyCngRq == FALSE)
        } // end of if (rssi != LL_RSSI_NOT_AVAILABLE)


	  } // end of if (status == SUCCESS)
      break;
    }

    case HCI_LE_READ_PHY:
    {
      if (status == SUCCESS)
      {
      }
      break;
    }

    default:
      break;
  } // end of switch (pMsg->cmdOpcode)
}

/*********************************************************************
* @fn      SimplePeripheral_initPHYRSSIArray
*
* @brief   Initializes the array of structure/s to store data related
*          RSSI based auto PHy change
*
* @param   connHandle - the connection handle
*
* @param   addr - pointer to device address
*
* @return  index of connection handle
*/
static void SimplePeripheral_initPHYRSSIArray(void)
{
  //Initialize array to store connection handle and RSSI values
  memset(connList, 0, sizeof(connList));
  for (uint8_t index = 0; index < MAX_NUM_BLE_CONNS; index++)
  {
    connList[index].connHandle = SP_INVALID_HANDLE;
  }
}
/*********************************************************************
      // Set default PHY to 1M
 * @fn      SimplePeripheral_startAutoPhyChange
 *
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   devAddr - device address
 *
 * @return  SUCCESS: Terminate started
 *          bleIncorrectMode: No link
 *          bleNoResources: No resources
 */
static status_t SimplePeripheral_startAutoPhyChange(uint16_t connHandle)
{
  status_t status = FAILURE;

  // Get connection index from handle
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Start Connection Event notice for RSSI calculation
  status = Gap_RegisterConnEventCb(SimplePeripheral_connEvtCB, GAP_CB_REGISTER, GAP_CB_CONN_EVENT_ALL, connHandle);

  // Flag in connection info if successful
  if (status == SUCCESS)
  {
    connList[connIndex].isAutoPHYEnable = TRUE;
  }

  return status;
}

/*********************************************************************
 * @fn      SimplePeripheral_stopAutoPhyChange
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
static status_t SimplePeripheral_stopAutoPhyChange(uint16_t connHandle)
{
  // Get connection index from handle
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Stop connection event notice
  Gap_RegisterConnEventCb(NULL, GAP_CB_UNREGISTER, GAP_CB_CONN_EVENT_ALL, connHandle);

  // Also update the phychange request status for active RSSI tracking connection
  connList[connIndex].phyCngRq = FALSE;
  connList[connIndex].isAutoPHYEnable = FALSE;

  return SUCCESS;
}

/*********************************************************************
 * @fn      SimplePeripheral_setPhy
 *
 * @brief   Call the HCI set phy API and and add the handle to a
 *          list to match it to an incoming command status event
 */
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts)
{
  // Allocate list entry to store handle for command status
  spConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(spConnHandleEntry_t));

  if (connHandleEntry)
  {
    connHandleEntry->connHandle = connHandle;

    // Add entry to the phy command status list
    List_put(&setPhyCommStatList, (List_Elem *)connHandleEntry);

    // Send PHY Update
    HCI_LE_SetPhyCmd(connHandle, allPhys, txPhy, rxPhy, phyOpts);
  }

  return SUCCESS;
}

/*********************************************************************
* @fn      SimplePeripheral_updatePHYStat
*
* @brief   Update the auto phy update state machine
*
* @param   connHandle - the connection handle
*
* @return  None
*/
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg)
{
  uint8_t connIndex;

  switch (eventCode)
  {
    case HCI_LE_SET_PHY:
    {
      // Get connection handle from list
      spConnHandleEntry_t *connHandleEntry =
                           (spConnHandleEntry_t *)List_get(&setPhyCommStatList);

      if (connHandleEntry)
      {
        // Get index from connection handle
        connIndex = SimplePeripheral_getConnIndex(connHandleEntry->connHandle);

        ICall_free(connHandleEntry);

        // Is this connection still valid?
        if (connIndex < MAX_NUM_BLE_CONNS)
        {
          hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;

          if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
          {
            // Update the phychange request status for active RSSI tracking connection
            connList[connIndex].phyCngRq = FALSE;
            connList[connIndex].phyRqFailCnt++;
          }
        }
      }
      break;
    }

    // LE Event - a Phy update has completed or failed
    case HCI_BLE_PHY_UPDATE_COMPLETE_EVENT:
    {
      hciEvt_BLEPhyUpdateComplete_t *pPUC =
                                     (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

      if(pPUC)
      {
        // Get index from connection handle
        connIndex = SimplePeripheral_getConnIndex(pPUC->connHandle);

        // Is this connection still valid?
        if (connIndex < MAX_NUM_BLE_CONNS)
        {
          // Update the phychange request status for active RSSI tracking connection
          connList[connIndex].phyCngRq = FALSE;

          if (pPUC->status == SUCCESS)
          {
            connList[connIndex].currPhy = pPUC->rxPhy;
          }
          if(pPUC->rxPhy != connList[connIndex].rqPhy)
          {
            connList[connIndex].phyRqFailCnt++;
          }
          else
          {
            // Reset the request phy counter and requested phy
            connList[connIndex].phyRqFailCnt = 0;
            connList[connIndex].rqPhy = 0;
          }
        }
      }

      break;
    }

    default:
      break;
  } // end of switch (eventCode)
}


/*********************************************************************
*********************************************************************/

static void OpenWind_Clock_Handler(UArg arg)
{
    //spClockEventData_t *pData = (spClockEventData_t *)arg;

    Event_post(syncEvent, arg);
/*
    if (pData->event == OW_WIND_DIR)
    {
        SimplePeripheral_enqueueMsg(OW_WIND_DIR, NULL);
    }
    else if (pData->event == OW_WIND_MOV)
    {
        SimplePeripheral_enqueueMsg(OW_WIND_MOV, NULL);
    }
    else if (pData->event == OW_WIND_GPS)
    {
        SimplePeripheral_enqueueMsg(OW_WIND_GPS, NULL);
    }
    else if (pData->event == OW_WIND_BAT)
    {
        SimplePeripheral_enqueueMsg(OW_WIND_BAT, NULL);
    }
    else if(pData->event == OW_WIND_LED) {
        SimplePeripheral_enqueueMsg(OW_WIND_LED, NULL);
    }
    else if(pData->event == OW_WIND_LED_OFF) {
        SimplePeripheral_enqueueMsg(OW_WIND_LED_OFF, NULL);
    }*/
}

static void OpenWind_power_up() {

    uint8_t numActive = linkDB_NumActive("");

    if(numActive == 1) {
        GPIO_write(LOAD_CTRL, 1);
        SensorI2C_open();
        Util_stopClock(&ow_clk_LED);
        Util_rescheduleClock(&ow_clk_LED, OW_PERIODIC_EVT_LED_Connected);
        Util_startClock(&ow_clk_LED);
        OpenWind_has_Connection=true;
        //YIC71513PGMGG_uart_open();
    }
    Util_startClock(&ow_clk_wind_dir);
    Util_startClock(&ow_clk_mov);
    Util_startClock(&ow_clk_gps);
    Util_startClock(&ow_clk_bat);
}
static void OpenWind_power_down() {

    Util_stopClock(&ow_clk_wind_dir);
    Util_stopClock(&ow_clk_mov);
    Util_stopClock(&ow_clk_gps);
    Util_stopClock(&ow_clk_bat);

    OpenWind_START_DIRECTION = false;
    OpenWind_START_MOV = false;
    OpenWind_START_GPS = false;
    OpenWind_START_BAT = false;


    YIC71513PGMGG_uart_close();
    PowerDown();

    SensorI2C_close();
    GPIO_write(LOAD_CTRL, 0);
    OpenWind_has_Connection=false;
    Util_rescheduleClock(&ow_clk_LED, OW_PERIODIC_EVT_LED);
}

static void OpenWind_DIRECTION_Task() {
    if(OpenWind_START_DIRECTION) {
        SensorTagWind_readData();
    }
    Util_startClock(&ow_clk_wind_dir);
}
static void OpenWind_MOVEMENT_Task() {
    if(OpenWind_START_MOV) {
        SensorTagMov_processSensorEvent();
    }
    Util_startClock(&ow_clk_mov);
}
static void OpenWind_GPS_Task() {
    if(OpenWind_START_GPS)
        YIC71513PGMGG_uart_read();
    Util_startClock(&ow_clk_gps);
}
static void OpenWind_GPS_Task_READ() {
    YIC71513PGMGG_uart_parse();
}
static void OpenWind_BATTERY_Task() {

    if(OpenWind_START_BAT) {
        SensorTagBatt_processSensorEvent();
    }
    else {
        OpenWind_START_BAT=true;
        BQ27441_Setup();
        SensorTagBatt_processSensorEvent();
    }
    Util_startClock(&ow_clk_bat);

}
static void OpenWind_LED_Task() {

    static bool sleep = false;

    if(sleep && !OpenWind_has_Connection) {
        sleep=false;
        Task_sleep(5000000 / Clock_tickPeriod);//5sekunden
    }
    else
        sleep=true;

    OpenWind_toggleAdvertising();

    if(OpenWind_has_Connection) {
        GPIO_toggle(GREEN_LED);
    }
    else {

        if(state_charge <= 20) {
            GPIO_toggle(RED_LED);
        }
        else {
            GPIO_toggle(BLUE_LED);
        }
    }
    delay_ms(OW_PERIODIC_EVT_LED_OFF);

    GPIO_write(RED_LED,1);
    GPIO_write(BLUE_LED,1);
    GPIO_write(GREEN_LED,1);

    Util_startClock(&ow_clk_LED);
}


void OPENWIND_Event(int event) {
    Event_post(syncEvent, event);
}

void OpenWindFlash_Init() {

    //OpenWindFlash_Write();
    OpenWindFlash_Read();

}
void OpenWindFlash_Write() {


    uint16_t WindDirection = (AWA_offset * 10);
    bufSen[0]=  (uint8_t)(WindDirection& 0x00FF);
    bufSen[1]=  (uint8_t)((WindDirection& 0xFF00) >> 8);
    uint16_t WindSpeed = (AWS_offset * 10);
    bufSen[2]=  (uint8_t)(WindSpeed& 0x00FF);
    bufSen[3]=  (uint8_t)((WindSpeed& 0xFF00) >> 8);
    uint16_t YawOffset = (yaw_offset );
    bufSen[4]=  (uint8_t)(YawOffset& 0x00FF);
    bufSen[5]=  (uint8_t)((YawOffset& 0xFF00) >> 8);

    uint16_t PitchOffset = (pitch_offset );
    bufSen[6]=  (uint8_t)(PitchOffset& 0x00FF);
    bufSen[7]=  (uint8_t)((PitchOffset& 0xFF00) >> 8);

    uint16_t RollOffset = (roll_offset );
    bufSen[8]=  (uint8_t)(RollOffset& 0x00FF);
    bufSen[9]=  (uint8_t)((RollOffset& 0xFF00) >> 8);

    WindOffsetArray[0] = 0x01;
    WindOffsetArray[1] = bufSen[1];
    WindOffsetArray[2] = bufSen[0];
    WindOffsetArray[3] = bufSen[3];
    WindOffsetArray[4] = bufSen[2];
    WindOffsetArray[5] = bufSen[5];
    WindOffsetArray[6] = bufSen[4];
    WindOffsetArray[7] = bufSen[7];
    WindOffsetArray[8] = bufSen[6];
    WindOffsetArray[9] = bufSen[9];
    WindOffsetArray[10] = bufSen[8];
    WindOffsetArray[11] = 0x47;

    Wind_SetParameter(Wind_OFFSET,WIND_OFFSET_LEN,WindOffsetArray);

#ifdef BNO055
    uint16_t Mag = mag_offset_flash.r;
    bufSen[10]=  (uint8_t)(Mag& 0x00FF);
    bufSen[11]=  (uint8_t)((Mag& 0xFF00) >> 8);
    Mag = mag_offset_flash.x;
    bufSen[12]=  (uint8_t)(Mag& 0x00FF);
    bufSen[13]=  (uint8_t)((Mag& 0xFF00) >> 8);
    Mag = mag_offset_flash.y;
    bufSen[14]= (uint8_t)(Mag& 0x00FF);
    bufSen[15]= (uint8_t)((Mag& 0xFF00) >> 8);
    Mag = mag_offset_flash.z;
    bufSen[16]= (uint8_t)(Mag& 0x00FF);
    bufSen[17]= (uint8_t)((Mag& 0xFF00) >> 8);
    uint16_t Acc = accel_offset_flash.r;
    bufSen[18]= (uint8_t)(Acc& 0x00FF);
    bufSen[19]= (uint8_t)((Acc& 0xFF00) >> 8);
    Acc = accel_offset_flash.x;
    bufSen[20]= (uint8_t)(Acc& 0x00FF);
    bufSen[21]= (uint8_t)((Acc& 0xFF00) >> 8);
    Acc = accel_offset_flash.y;
    bufSen[22]= (uint8_t)(Acc& 0x00FF);
    bufSen[23]= (uint8_t)((Acc& 0xFF00) >> 8);
    Acc = accel_offset_flash.z;
    bufSen[24]= (uint8_t)(Acc& 0x00FF);
    bufSen[25]= (uint8_t)((Acc& 0xFF00) >> 8);
    uint16_t Gyro = gyro_offset_flash.x;
    bufSen[26]= (uint8_t)(Gyro& 0x00FF);
    bufSen[27]= (uint8_t)((Gyro & 0xFF00) >> 8);
    Gyro = gyro_offset_flash.y;
    bufSen[28]= (uint8_t)(Gyro& 0x00FF);
    bufSen[29]= (uint8_t)((Gyro & 0xFF00) >> 8);
    Acc = gyro_offset_flash.z;
    bufSen[30]= (uint8_t)(Gyro& 0x00FF);
    bufSen[31]= (uint8_t)((Gyro & 0xFF00) >> 8);
#endif

    uint8 status = osal_snv_write(SNV_ID_APP, sizeof(bufSen), (uint8 *)bufSen);
    if(status != SUCCESS) {
       int test=0;
    }
}

void OpenWindFlash_Read() {

    uint8 status = osal_snv_read(SNV_ID_APP, sizeof(bufRec), (uint8 *)bufRec);
    if(status != SUCCESS) {

        MAGhasbeenCalibrated=false;
        ACChasbeenCalibrated=false;
        GYROhasbeenCalibrated=false;

        AWA_offset = (float)((int8_t)bufRec[1]<<8 | (uint8_t)bufRec[0])/10;
        AWS_offset = (float)((int8_t)bufRec[3]<<8 | (uint8_t)bufRec[2])/10;
        yaw_offset = (signed short int)((int8_t)bufRec[5]<<8 | (uint8_t)bufRec[4]);
        pitch_offset = (signed short int)((int8_t)bufRec[7]<<8 | (uint8_t)bufRec[6]);
        roll_offset = (signed short int)((int8_t)bufRec[9]<<8 | (uint8_t)bufRec[8]);

        WindOffsetArray[0] = 0x01;
        WindOffsetArray[1] = bufRec[1];
        WindOffsetArray[2] = bufRec[0];
        WindOffsetArray[3] = bufRec[3];
        WindOffsetArray[4] = bufRec[2];
        WindOffsetArray[5] = bufRec[5];
        WindOffsetArray[6] = bufRec[4];
        WindOffsetArray[7] = bufRec[7];
        WindOffsetArray[8] = bufRec[6];
        WindOffsetArray[9] = bufRec[9];
        WindOffsetArray[10] = bufRec[8];
        WindOffsetArray[11]=0x47;

        Wind_SetParameter(Wind_OFFSET,WIND_OFFSET_LEN,WindOffsetArray);

#ifdef BNO055
        mag_offset_flash.r=(signed short int)((int8_t)bufRec[11]<<8 | (uint8_t)bufRec[10]);
        mag_offset_flash.x=(signed short int)((int8_t)bufRec[13]<<8 | (uint8_t)bufRec[12]);
        mag_offset_flash.y=(signed short int)((int8_t)bufRec[15]<<8 | (uint8_t)bufRec[14]);
        mag_offset_flash.z=(signed short int)((int8_t)bufRec[17]<<8 | (uint8_t)bufRec[16]);
        accel_offset_flash.r=(signed short int)((int8_t)bufRec[19]<<8 | (uint8_t)bufRec[18]);
        accel_offset_flash.x=(signed short int)((int8_t)bufRec[21]<<8 | (uint8_t)bufRec[20]);
        accel_offset_flash.y=(signed short int)((int8_t)bufRec[23]<<8 |  (uint8_t)bufRec[22]);
        accel_offset_flash.z=(signed short int)((int8_t)bufRec[25]<<8 | (uint8_t)bufRec[24]);
        gyro_offset_flash.x=(signed short int)((int8_t)bufRec[27]<<8 | (uint8_t)bufRec[26]);
        gyro_offset_flash.y=(signed short int)((int8_t)bufRec[29]<<8 | (uint8_t)bufRec[28]);
        gyro_offset_flash.z=(signed short int)((int8_t)bufRec[31]<<8 | (uint8_t)bufRec[30]);

        if(mag_offset_flash.r!=0 || mag_offset_flash.x!=0 || mag_offset_flash.y!=0 || mag_offset_flash.z!=0)
          MAGhasbeenCalibrated=true;
        if(accel_offset_flash.r!=0 || accel_offset_flash.x!=0 || accel_offset_flash.y!=0 || accel_offset_flash.z!=0)
          ACChasbeenCalibrated=true;
        if(gyro_offset_flash.x!=0 || gyro_offset_flash.y!=0 || gyro_offset_flash.z!=0)
          GYROhasbeenCalibrated=true;
#endif

    }

}
void clear_watchdog() {

}

void SensorTag_updateAdvertisingData(uint8_t location,uint8_t value)
{
  // Record key state in advertising data
  advertData[location] = value;
  //GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
  GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV, sizeof(advertData), advertData);
}

static void OpenWind_toggleAdvertising() {
    static bool is_adv=false;
    static bool advertEnabledNonConnectable=false;

    if(is_adv && !OpenWind_has_Connection) {
        is_adv=false;
        GapAdv_disable(advHandleLegacy);
    }
    else {
        is_adv=true;
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV, sizeof(advertData), advertData);

        if(OpenWind_has_Connection)
            advertEnabledNonConnectable=true;
        else
            advertEnabledNonConnectable=false;

        //GapAdv_setParam(GAP_ADV_PROP_SCANNABLE, sizeof(uint8_t),&advertEnabledNonConnectable);
    }
}

static void OpenWind_setDeviceInfo(void)
{
    uint64_t bleAddress = *((uint64_t *)(FCFG1_BASE + FCFG1_O_MAC_BLE_0)) & 0xFFFFFFFFFFFF;

    //GAPRole_GetParameter(GAPROLE_BD_ADDR, serial_number);

    DevInfo_SetParameter(DEVINFO_MODEL_NUMBER, sizeof(devInfoModelNumber),
                       (void*)devInfoModelNumber);
    serial_number[0] = 0x0000000000FF & bleAddress;
    serial_number[1] = (0x00000000FF00 & bleAddress) >> 8;
    serial_number[2] = (0x000000FF0000 & bleAddress) >> 16;
    serial_number[3] = (0x0000FF000000 & bleAddress) >> 24;
    serial_number[4] = (0x00FF00000000 & bleAddress) >> 32;
    serial_number[5] = (0xFF0000000000 & bleAddress) >> 40;

    if(serial_number[0]==0x00)
        serial_number[0]=0x01;

    //DevInfo_SetParameter(DEVINFO_SERIAL_NUMBER, sizeof(devInfoNA), (void*)devInfoNA);
    DevInfo_SetParameter(DEVINFO_SERIAL_NUMBER, sizeof(serial_number), (void*)serial_number);

    DevInfo_SetParameter(DEVINFO_SOFTWARE_REV, sizeof(devInfoNA),
                       (void*)devInfoNA);
    DevInfo_SetParameter(DEVINFO_FIRMWARE_REV, sizeof(devInfoFirmwareRev),
                       (void*)devInfoFirmwareRev);
    DevInfo_SetParameter(DEVINFO_HARDWARE_REV, sizeof(devInfoHardwareRev),
                       (void*)devInfoHardwareRev);
    DevInfo_SetParameter(DEVINFO_MANUFACTURER_NAME, sizeof(devInfoMfrName),
                       (void*)devInfoMfrName);
}
