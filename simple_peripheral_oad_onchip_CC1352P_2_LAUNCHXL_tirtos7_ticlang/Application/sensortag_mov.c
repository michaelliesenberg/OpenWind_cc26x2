/******************************************************************************

 @file  sensortag_mov.c

 @brief This file contains the Movement Processor sub-application. It uses the
        MPU-9250 Wake-on-movement feature to allow the
        MPU to turn off the gyroscope and magnetometer when no activity is
        detected.

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

/*********************************************************************
 * INCLUDES
 */
#ifdef BNO080
#include "BNO080.h"
#endif


#ifdef BNO055
#include "BNO055.h"
#endif

#define BNO_OW_MODE  BNO055_OPERATION_MODE_COMPASS //BNO055_OPERATION_MODE_NDOF BNO055_OPERATION_MODE_COMPASS


#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Task.h>

#include "gatt.h"
#include "gattservapp.h"

#include "ti_drivers_config.h"
#include <ti/drivers/GPIO.h>
#include "bnoservice.h"
#include "sensortag_mov.h"
#include "SensorUtil.h"
#include "util.h"
#include "string.h"
#include "BQ27441.h"
#include "SensorI2C.h"
#include "osal_snv.h"
#include "hal_board.h"
#include <stdio.h>
#include "DRV5013.h"
#include <math.h>
#define SENSOR_CALIBRATE 	9



/*********************************************************************
 * MACROS
 */
#define MOVEMENT_INACT_CYCLES   (MOVEMENT_INACT_TIMEOUT * (1000/sensorPeriod))

/*********************************************************************
 * CONSTANTS and MACROS
 */
// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD     1000 //100

// Time start measurement and data ready
#define MOV_DELAY_PERIOD        15

// Length of the data for this sensor
#define SENSOR_DATA1_LEN           MOVEMENT_DATA1_LEN
#define SENSOR_DATA2_LEN           MOVEMENT_DATA2_LEN
#define SENSOR_DATA3_LEN           MOVEMENT_DATA3_LEN
#define SENSOR_CONF1_LEN		   MOVEMENT_CONF1_LEN
#define SENSOR_CONF2_LEN		   MOVEMENT_CONF2_LEN
#define SENSOR_AXISMAP_LEN		   MOVEMENT_AXISMAP_LEN

// Event flag for this sensor
#define SENSOR_EVT                ST_GYROSCOPE_SENSOR_EVT



// Task configuration
#define SENSOR_TASK_PRIORITY    1
#define SENSOR_TASK_STACK_SIZE  600

#define SNV_ID_MOV 0x82
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
#ifdef BNO055



struct bno055_quaternion_t values_read_quartenion={0,0,0,0};
 int8_t get_return_value=0;
/*----------------------------------------------------------------------------*
 *  struct bno055_t parameters can be accessed by using BNO055
 *	BNO055_t having the following parameters
 *	Bus write function pointer: BNO055_WR_FUNC_PTR
 *	Bus read function pointer: BNO055_RD_FUNC_PTR
 *	Burst read function pointer: BNO055_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
*---------------------------------------------------------------------------*/
struct bno055_t bno055;

uint8_t interrupt_clock_active=1;
uint8_t notify_Client=0;
uint8_t calibrationStatus[MOVEMENT_CALIBRATE_LEN];

bool resetBNO=false;
bool calibrate_roll=0;
bool calibrate_pitch=0;
bool calibrateMAGBNO=false;

extern uint16_t yaw_value=0;
extern uint16_t pitch_value=0;
extern uint16_t roll_value=0;
extern uint8_t mov_calib_status=0;

bool is_active = true;

struct bno055_mag_offset_t  mag_offset_flash;
struct bno055_accel_offset_t  accel_offset_flash;
struct bno055_gyro_offset_t   gyro_offset_flash;

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern bool MAGhasbeenCalibrated=false;
extern bool ACChasbeenCalibrated=false;
extern bool GYROhasbeenCalibrated=false;
extern signed short int roll_offset=0;
extern signed short int pitch_offset=0;
extern signed short int yaw_offset=0;
extern uint8_t parameter_mov=0;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
 // Entity ID globally used to check for source and/or destination of messages
 //static ICall_EntityID sensorSelfEntity;

 // Semaphore globally used to post events to the application thread
// static ICall_Semaphore sensorSem;

 // Task setup


static uint16_t sensorPeriod;
static volatile bool sensorReadScheduled;
static uint8_t sensorData1[SENSOR_DATA1_LEN];//3x int16_t accel x, y, z; 3x int16_t mag x, y, z; 3x int16_t gyro x, y, z
 uint8_t sensorData2[SENSOR_DATA2_LEN];
static uint8_t sensorData3[SENSOR_DATA3_LEN];

uint8_t sensorData2_Copy[SENSOR_DATA2_LEN];


//Bit 6 : Data3 Enable
//Bit 5 : Data2 Enable
//Bit 4 : Data1 Enable
//Bits 3:0 : Operating mode
//BNO Operating Mode
//BNO055_OPERATION_MODE_CONFIG
//BNO055_OPERATION_MODE_ACCONLY
//BNO055_OPERATION_MODE_MAGONLY
//BNO055_OPERATION_MODE_GYRONLY
//BNO055_OPERATION_MODE_ACCMAG
//BNO055_OPERATION_MODE_ACCGYRO
//BNO055_OPERATION_MODE_MAGGYRO
//BNO055_OPERATION_MODE_AMG
//BNO055_OPERATION_MODE_IMUPLUS
//BNO055_OPERATION_MODE_COMPASS
//BNO055_OPERATION_MODE_M4G
//BNO055_OPERATION_MODE_NDOF_FMC_OFF
//BNO055_OPERATION_MODE_NDOF
#define BNO055_OPERATION_MODE_M 0x0F
//#define BNO055_DATA1_ENABLE     0x10
//#define BNO055_DATA2_ENABLE     0x20
//#define BNO055_DATA3_ENABLE     0x40
static uint8_t sensorConfig1;

static uint8_t sensorConfig2;

static uint16_t sensorAxisMap;

static uint8_t appState;
static volatile bool bnoDataRdy;
//static uint32_t nActivity;
//static uint8_t movThreshold;
//static uint8_t mpuIntStatus;
//static bool shakeDetected;
//static uint8_t nMotions;
u8 value1=0, value2=0, value3=0;
uint8_t error_BNO=0, opmode=0;
/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void sensorChangeCB(uint8_t paramID);
static void initCharacteristicValue(uint8_t paramID, uint8_t value, uint8_t paramLen);
void readStatusCalibration(void);


/*********************************************************************
 * PROFILE CALLBACKS
 */
static sensorCBs_t sensorCallbacks =
{
  sensorChangeCB,  // Characteristic value change callback
};


/*********************************************************************
 * @fn      SensorTagMov_init
 *
 * @brief   Initialization function for the SensorTag movement sub-application
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagMov_init(void)
{

    Movement_addService_bno();
    // Register callbacks with profile
	Movement_registerAppCBs_bno(&sensorCallbacks);

	// Initialize the module state variables
    sensorConfig1 = ST_CFG_SENSOR_DISABLE;
    sensorConfig2 = 0;
    sensorAxisMap = 0x24;
    sensorPeriod = SENSOR_DEFAULT_PERIOD;
    sensorReadScheduled = false;

    appState = APP_STATE_OFF;
    // nMotions = 0;

    if(bno055_init(&bno055) == BNO055_SUCCESS)
    {
      SensorTagMov_reset();
    }


    // Initialize characteristics
    initCharacteristicValue(SENSOR_PERI,SENSOR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION,sizeof(uint8_t));
}
void init_INT_BNO(void)
{

    GPIO_setConfig(BNO_INT, GPIO_CFG_INPUT | GPIO_CFG_IN_INT_RISING);
        /* Install Button callback */
    GPIO_setCallback(BNO_INT, SensorTagMov_processInterrupt);

    GPIO_enableInt(BNO_INT);
}
void disable_INT_BNO(void)
{
    GPIO_disableInt(BNO_INT);
    GPIO_write(BNO_INT,0);
}


/*********************************************************************
 * @fn      SensorTagMov_processSensorEvent
 *
 * @brief   SensorTag Movement sensor event processor.
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagMov_processSensorEvent(void)
{
  //Are we scheduled to read data?

    if(sensorConfig1 & BNO055_DATA2_ENABLE)
    {
        //bno055_read_quaternion_wxyz((struct bno055_quaternion_t *)&sensorData2[0]);

        if(calibrate_roll==1)
        {
            calibrate_roll=0;
            interrupt_clock_active=1;
            roll_offset= (s16)((((s32)((s8)sensorData2[BNO055_SENSOR_DATA_EULER_HRP_R_MSB])) <<BNO055_SHIFT_EIGHT_BITS)| (sensorData2[BNO055_SENSOR_DATA_EULER_HRP_R_LSB]))+roll_offset;
            OpenWindFlash_Write();
        }
        if(calibrate_pitch==1)
        {
            calibrate_pitch=0;
            interrupt_clock_active=1;
            pitch_offset=(s16)((((s32)((s8)sensorData2[BNO055_SENSOR_DATA_EULER_HRP_P_MSB])) <<BNO055_SHIFT_EIGHT_BITS)| (sensorData2[BNO055_SENSOR_DATA_EULER_HRP_P_LSB]))+pitch_offset;
            OpenWindFlash_Write();
        }


        if(calibrateMAGBNO)
        {
            if(resetBNO)
            {
                resetBNO=false;

                MAGhasbeenCalibrated=false;
                ACChasbeenCalibrated=false;
                GYROhasbeenCalibrated=false;


                bno055_set_sys_rst(BNO055_BIT_ENABLE);//Reset System
                DELAY_MS(650);
                bno055_set_sys_rst(BNO055_BIT_DISABLE);//Reset System
                DELAY_MS(10);

                bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);

                //Set axis mapping
                if(bno055_set_operation_mode(BNO_OW_MODE) == BNO055_SUCCESS)
                {
                    bno055_read_quaternion_wxyz((struct bno055_quaternion_t *)&sensorData2[0]);
                }
                bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);

                read_calibration_values();
            }
            else
            {
                read_calibration_values();
            }
        }
        else
        {
            if(interrupt_clock_active==1)// || (calibration_valius & 0x03 != 3))
            {
                //interrupt_clock_active=0;
                notify_Client=1;

                bno055_read_euler_hrp((struct bno055_euler_t *)&sensorData2[0]);

                yaw_value=   (s16)((((s32)((s8)sensorData2[BNO055_SENSOR_DATA_EULER_HRP_H_MSB])) <<BNO055_SHIFT_EIGHT_BITS)| (sensorData2[BNO055_SENSOR_DATA_EULER_HRP_H_LSB]));
                roll_value=  (s16)((((s32)((s8)sensorData2[BNO055_SENSOR_DATA_EULER_HRP_R_MSB])) <<BNO055_SHIFT_EIGHT_BITS)| (sensorData2[BNO055_SENSOR_DATA_EULER_HRP_R_LSB]));
                pitch_value= (s16)((((s32)((s8)sensorData2[BNO055_SENSOR_DATA_EULER_HRP_P_MSB])) <<BNO055_SHIFT_EIGHT_BITS)| (sensorData2[BNO055_SENSOR_DATA_EULER_HRP_P_LSB]));

                memcpy(sensorData2_Copy,sensorData2,SENSOR_DATA2_LEN);
    #ifdef PLUS_BROADCASTER_VALUES
                SensorTag_updateAdvertisingData(12,sensorData2[BNO055_SENSOR_DATA_EULER_HRP_H_MSB]);
                SensorTag_updateAdvertisingData(13,sensorData2[BNO055_SENSOR_DATA_EULER_HRP_H_LSB]);
                SensorTag_updateAdvertisingData(14,sensorData2[BNO055_SENSOR_DATA_EULER_HRP_P_MSB]);
                SensorTag_updateAdvertisingData(15,sensorData2[BNO055_SENSOR_DATA_EULER_HRP_P_LSB]);
                SensorTag_updateAdvertisingData(16,sensorData2[BNO055_SENSOR_DATA_EULER_HRP_R_MSB]);
                SensorTag_updateAdvertisingData(17,sensorData2[BNO055_SENSOR_DATA_EULER_HRP_R_LSB]);
    #endif
    #ifdef PLUS_BROADCASTER_VALUES_NEW
                SensorTag_updateAdvertisingData(15,sensorData2[BNO055_SENSOR_DATA_EULER_HRP_H_MSB]);
                SensorTag_updateAdvertisingData(16,sensorData2[BNO055_SENSOR_DATA_EULER_HRP_H_LSB]);
                SensorTag_updateAdvertisingData(17,sensorData2[BNO055_SENSOR_DATA_EULER_HRP_P_MSB]);
                SensorTag_updateAdvertisingData(18,sensorData2[BNO055_SENSOR_DATA_EULER_HRP_P_LSB]);
                SensorTag_updateAdvertisingData(19,sensorData2[BNO055_SENSOR_DATA_EULER_HRP_R_MSB]);
                SensorTag_updateAdvertisingData(20,sensorData2[BNO055_SENSOR_DATA_EULER_HRP_R_LSB]);
    #endif

                static uint8_t counter=4;

                uint8_t id=0;
                bno055_read_chip_id(&id);
                bno055_get_operation_mode(&opmode);

                if(id != BNO055_ID || opmode != BNO_OW_MODE )
                {
                    error_BNO = 1;

                    if(opmode != BNO_OW_MODE)
                    {
                        OpenWindFlash_Read();

                        bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);

                        if(MAGhasbeenCalibrated)
                            bno055_write_mag_offset(&mag_offset_flash);
                        clear_watchdog();
                        if(ACChasbeenCalibrated)
                            bno055_write_accel_offset(&accel_offset_flash);
                        clear_watchdog();
                        if(GYROhasbeenCalibrated && BNO_OW_MODE != BNO055_OPERATION_MODE_COMPASS)
                            bno055_write_gyro_offset(&gyro_offset_flash);

                        bno055_set_operation_mode(BNO_OW_MODE);
                    }

                }
                else
                    error_BNO = 0;

                if(counter>=4)
                {
                    counter=0;
                    readStatusCalibration();
                }
                else
                    counter++;
            }
            else
            {
                memcpy(sensorData2,sensorData2_Copy,SENSOR_DATA2_LEN);
            }
        }
    }
    if(sensorConfig1 & BNO055_DATA3_ENABLE)
    {
        bno055_read_linear_accel_xyz((struct bno055_linear_accel_t *)&sensorData3[0]);
        bno055_read_gravity_xyz((struct bno055_gravity_t *)&sensorData3[6]);
    }

    //Send the data
    if(sensorConfig1 & BNO055_DATA1_ENABLE)
    {
        Movement_setParameter_bno(SENSOR_DATA1, SENSOR_DATA1_LEN, sensorData1);
    }
    if(sensorConfig1 & BNO055_DATA2_ENABLE)
    {
        if(notify_Client==1)
        {
            notify_Client=0;
            Movement_setParameter_bno(SENSOR_DATA2, SENSOR_DATA2_LEN, sensorData2);
        }
    }
    if(sensorConfig1 & BNO055_DATA3_ENABLE)
    {
        Movement_setParameter_bno(SENSOR_DATA3, SENSOR_DATA3_LEN, sensorData3);
    }

}


void read_calibration_values(void)
{
    bno055_get_mag_calib_stat(&value1);

    if(value1==3)//Magnetometer Fully Calibrated
    {
        bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
        DELAY_MS(30);
        bno055_read_mag_offset(&mag_offset_flash);
        bno055_set_operation_mode(BNO_OW_MODE);
        DELAY_MS(30);
        MAGhasbeenCalibrated=true;
    }


    bno055_get_accel_calib_stat(&value2);

    if(value2==3)// Fully Calibrated
    {
        bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
        DELAY_MS(30);
        bno055_read_accel_offset(&accel_offset_flash);
        bno055_set_operation_mode(BNO_OW_MODE);
        DELAY_MS(30);
        ACChasbeenCalibrated=true;
    }

    if(BNO_OW_MODE != BNO055_OPERATION_MODE_COMPASS)
    {
        bno055_get_gyro_calib_stat(&value3);

        if(value3==3)// Fully Calibrated
        {
            bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            DELAY_MS(30);
            bno055_read_gyro_offset(&gyro_offset_flash);
            bno055_set_operation_mode(BNO_OW_MODE);
            DELAY_MS(30);
            GYROhasbeenCalibrated=true;
        }
    }
    else
    {
        GYROhasbeenCalibrated=true;
        value3=3;
    }



    calibrationStatus[0]= (value3)<<4 | (value2)<<2 | (value1);

    mov_calib_status=calibrationStatus[0];

    Movement_setParameter_bno(SENSOR_CALIBRATE, MOVEMENT_CALIBRATE_LEN, &calibrationStatus);

    //if(GYROhasbeenCalibrated && ACChasbeenCalibrated && MAGhasbeenCalibrated)
    {
        calibrateMAGBNO=false;
        OpenWindFlash_Write();
    }
}
/*********************************************************************
 * @fn      SensorTagMov_processCharChangeEvt
 *
 * @brief   SensorTag Movement event handling
 *
 * @param   paramID - identifies which characteristic has changed
 *
 * @return  none
 */
void SensorTagMov_processCharChangeEvt(uint8_t paramID)
{
  uint16_t newCfg;
  uint8_t newValue8;
  volatile uint8_t test;

  switch (paramID)
  {
  case SENSOR_CONF1:

	  sensorConfig1=0;

    if (sensorConfig1 != ST_CFG_ERROR)
    {
    	Movement_getParameter_bno(SENSOR_CONF1, &newCfg);

      if ((newCfg ) == ST_CFG_SENSOR_DISABLE)
      {
        // All axes off, turn off device power
        sensorConfig1 = newCfg;
        Movement_setParameter_bno(SENSOR_CONF1, sizeof(sensorConfig1), (uint8_t*)&sensorConfig1);
        appStateSet_mov(APP_STATE_OFF);

      }
      else
      {
        // Some axes on; power up and activate BNO
        sensorConfig1 = newCfg;

        if(sensorConfig1 == 0x0C){
        	asm(" NOP");
        	test++;
        }

        if(sensorConfig1 == 0x20){
        	asm(" NOP");
        	test++;
        }

        Movement_setParameter_bno(SENSOR_CONF1, sizeof(sensorConfig1), (uint8_t*)&sensorConfig1);

        appStateSet_mov(APP_STATE_ACTIVE);

      }
    }
    else
    {
      // Make sure the previous characteristics value is restored
      initCharacteristicValue(SENSOR_CONF1, sensorConfig1, sizeof(sensorConfig1));
    }

    break;

  case SENSOR_CONF2:


    if (sensorConfig2 != ST_CFG_ERROR)
    {
    	Movement_getParameter_bno(SENSOR_CONF2, &newCfg);

      if ((newCfg ) == ST_CFG_SENSOR_DISABLE)
      {
        // All axes off, turn off device power
        sensorConfig2 = newCfg;
        appStateSet_mov(APP_STATE_OFF);
      }
      else
      {
        // Some axes on; power up and activate MPU
        sensorConfig2 = newCfg;
        appStateSet_mov(APP_STATE_ACTIVE);
      }

      Movement_setParameter_bno(SENSOR_CONF2, sizeof(sensorConfig2), (uint8_t*)&sensorConfig2);
    }
    else
    {
      // Make sure the previous characteristics value is restored
      initCharacteristicValue(SENSOR_CONF2, sensorConfig2, sizeof(sensorConfig2));
    }

    // Data initially zero
    initCharacteristicValue(SENSOR_DATA1, 0, SENSOR_DATA1_LEN);
    initCharacteristicValue(SENSOR_DATA2, 0, SENSOR_DATA2_LEN);
    initCharacteristicValue(SENSOR_DATA3, 0, SENSOR_DATA3_LEN);
    break;

  case SENSOR_AXISMAP:
	  Movement_getParameter_bno(SENSOR_AXISMAP, &newCfg);
	  if(newCfg != sensorAxisMap)
	  {
		  bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
		  bno055_set_axis_remap_value(newCfg & 0x3F);
		  bno055_set_remap_x_sign((newCfg & 0x0040) >> 6);
		  bno055_set_remap_y_sign((newCfg & 0x0080) >> 7);
		  bno055_set_remap_z_sign((newCfg & 0x0010) >> 8);
		  bno055_set_operation_mode(BNO_OW_MODE);
		  sensorAxisMap = newCfg;
	  }
	  break;

  case SENSOR_PERI:
    Movement_getParameter_bno(SENSOR_PERI, &newValue8);
    sensorPeriod = newValue8 * SENSOR_PERIOD_RESOLUTION;
    break;
  case MOVEMENT_CALIBRATE:
	  calibrate_roll=1;
	  break;
  case MOVEMENT_CALIBRATE_PITCH:
	  calibrate_pitch=1;
	  break;
  case MOVEMENT_CALIBRATE_MAG:
      calibrateMAGBNO=true;
	  resetBNO=true;
	  GYROhasbeenCalibrated=true;
	  ACChasbeenCalibrated=true;
	  MAGhasbeenCalibrated=false;
     calibrationStatus[0]=0;
	  Movement_setParameter_bno(SENSOR_CALIBRATE, MOVEMENT_CALIBRATE_LEN, &calibrationStatus);
	  break;
  case MOVEMENT_CALIBRATE_ALL:
    calibrateMAGBNO=true;
    resetBNO=true;
    GYROhasbeenCalibrated=false;
    ACChasbeenCalibrated=false;
    MAGhasbeenCalibrated=false;
    calibrationStatus[0]=0;
    Movement_setParameter_bno(SENSOR_CALIBRATE, MOVEMENT_CALIBRATE_LEN, &calibrationStatus);
    break;

    case MOVEMENT_CALIBRATE_STOP:
        resetBNO=false;
        calibrateMAGBNO=false;

        if(!MAGhasbeenCalibrated)
        {
            bno055_write_mag_offset(&mag_offset_flash);
        }
        if(!GYROhasbeenCalibrated && BNO_OW_MODE != BNO055_OPERATION_MODE_COMPASS)
        {
           bno055_write_gyro_offset(&gyro_offset_flash);
        }
        if(!ACChasbeenCalibrated)
        {
            bno055_write_accel_offset(&accel_offset_flash);
        }

        OpenWindFlash_Write();
    break;

  default:
    // Should not get here
    break;
  }
}

/*********************************************************************
 * @fn      SensorTagMov_reset
 *
 * @brief   Reset characteristics and disable sensor
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagMov_reset(void)
{
  initCharacteristicValue(SENSOR_DATA1, 0, SENSOR_DATA1_LEN);
  initCharacteristicValue(SENSOR_DATA2, 0, SENSOR_DATA2_LEN);
  initCharacteristicValue(SENSOR_DATA3, 0, SENSOR_DATA3_LEN);
  initCharacteristicValue(SENSOR_CONF1, 0, SENSOR_CONF1_LEN);
  initCharacteristicValue(SENSOR_CONF2, 0, SENSOR_CONF2_LEN);
  initCharacteristicValue(SENSOR_AXISMAP, 0, SENSOR_AXISMAP_LEN);


  // Remove power from the BNO
  appStateSet_mov(APP_STATE_OFF);
}


/*********************************************************************
* Private functions
*/

/*********************************************************************
 * @fn      SensorTagMov_processInterrupt
 *
 * @brief   Interrupt handler for BNO
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTagMov_processInterrupt(uint_least8_t index)
{
    if (index == BNO_INT)
    {
        GPIO_clearInt(BNO_INT);
        //OPENWIND_charValueChangeCB(MOV_INT_EVENT, 0);
        OPENWIND_Event(MOV_INT_EVENT);

        interrupt_clock_active=1;
    }
}
void SensorTagMov_INT_Handle(void)
{
	// Wake up the application thread
	bnoDataRdy = true;
	sensorReadScheduled = true;
	bno055_set_intr_rst(0x01);
	interrupt_clock_active=1;
	SensorTagMov_processSensorEvent();
}
/*********************************************************************
 * @fn      sensorChangeCB
 *
 * @brief   Callback from Movement Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void sensorChangeCB(uint8_t paramID)
{
    parameter_mov=paramID;
  // Wake up the application thread
    OpenWind_enqueueMsg(SERVICE_ID_MOV, paramID);
}


/*********************************************************************
 * @fn      initCharacteristicValue
 *
 * @brief   Initialize a characteristic value
 *
 * @param   paramID - parameter ID of the value is to be cleared
 *
 * @param   value - value to initialize with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 */
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen)
{
	switch(paramID){
	case SENSOR_CONF1:
		  memset(&sensorConfig1,value,paramLen);
		  Movement_setParameter_bno(paramID, paramLen, &sensorConfig1);
		break;
	case SENSOR_CONF2:
		  memset(&sensorConfig2,value,paramLen);
		  Movement_setParameter_bno(paramID, paramLen, &sensorConfig2);
		break;
	case SENSOR_AXISMAP:
		  memset(&sensorAxisMap,value,paramLen);
		  Movement_setParameter_bno(paramID, paramLen, &sensorAxisMap);
		break;
	case SENSOR_DATA1:
		  memset(sensorData1,value,paramLen);
		  Movement_setParameter_bno(paramID, paramLen, sensorData1);
		break;
	case SENSOR_DATA2:
		  memset(sensorData2,value,paramLen);
		  Movement_setParameter_bno(paramID, paramLen, sensorData2);
		break;
	case SENSOR_DATA3:
		  memset(sensorData3,value,paramLen);
		  Movement_setParameter_bno(paramID, paramLen, sensorData3);
		break;


	default:
		break;
	}
}

void readStatusCalibration(void)
{


    if(bno055_get_mag_calib_stat(&value1) != BNO055_SUCCESS)
    {
        return;
    }
    if(bno055_get_accel_calib_stat(&value2) != BNO055_SUCCESS)
    {
        return;
    }
    if(BNO_OW_MODE != BNO055_OPERATION_MODE_COMPASS)
    {
        if(bno055_get_gyro_calib_stat(&value3) != BNO055_SUCCESS)
        {
            return;
        }
    }
    else
        value3=3;

    calibrationStatus[0]= (value3)<<4 | (value2)<<2 | (value1);
    mov_calib_status=calibrationStatus[0];

    Movement_setParameter_bno(SENSOR_CALIBRATE, MOVEMENT_CALIBRATE_LEN, &calibrationStatus);
}
/*******************************************************************************
 * @fn      appStateSet
 *
 * @brief   Set the application state
 *
 */
void appStateSet_mov(uint8_t newState)
{

//    SensorTagTrueWindFlash_Read();
	volatile uint8_t test;

	if (newState == APP_STATE_OFF)
	{
		appState = APP_STATE_OFF;
		// Stop scheduled data measurements
	}

      if (newState == APP_STATE_ACTIVE || newState == APP_STATE_IDLE)
      {

        appState = APP_STATE_ACTIVE;
        bnoDataRdy = false;
        //PIN_setInterrupt(hBnoPin, PIN_ID(Board_BNO_INT)|PIN_IRQ_POSEDGE);


        bno055_set_sys_rst(BNO055_BIT_ENABLE);//Reset System
        clear_watchdog();
        DELAY_MS(50);

        bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
        DELAY_MS(650);

        uint8_t id=0, counter=0;

        while (id != BNO055_ID)
        {
           bno055_read_chip_id(&id);
		   clear_watchdog();
           DELAY_MS(10);
		   counter++;
		   
		   if(counter>10)

		       break;
        }

        bno055_write_page_id(BNO055_PAGE_ZERO);
        bno055_set_sys_rst(BNO055_BIT_DISABLE);//Reset System
        DELAY_MS(10);
        clear_watchdog();

        bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
        DELAY_MS(30);

        clear_watchdog();

        if(MAGhasbeenCalibrated)
            bno055_write_mag_offset(&mag_offset_flash);
        clear_watchdog();
        if(ACChasbeenCalibrated)
            bno055_write_accel_offset(&accel_offset_flash);
        clear_watchdog();
        if(GYROhasbeenCalibrated && BNO_OW_MODE != BNO055_OPERATION_MODE_COMPASS)
            bno055_write_gyro_offset(&gyro_offset_flash);
        clear_watchdog();


        if(bno055_set_operation_mode(BNO_OW_MODE) == BNO055_SUCCESS)
        {
            //bno055_read_quaternion_wxyz((struct bno055_quaternion_t *)&sensorData2[0]);
            //bno055_read_euler_hrp((struct bno055_euler_t *)&sensorData2[0]);
        }

        DELAY_MS(30);

        //Set axis mapping


//        bno055_set_intr_mask_accel_any_motion(BNO055_BIT_ENABLE);
//        bno055_set_intr_accel_any_motion(BNO055_BIT_ENABLE);
//        bno055_set_intr_mask_gyro_any_motion(BNO055_BIT_ENABLE);
//        bno055_set_intr_gyro_any_motion(BNO055_BIT_ENABLE);
//
//        bno055_set_mag_data_output_rate(0x07);//0x00 MAG_DATA_OUTPUT_RATE_2HZ
//        bno055_set_mag_operation_mode(0x03);//0x03 MAG_OPR_MODE_HIGH_ACCURACY
//        bno055_set_mag_operation_mode(0x03); //0x03   BNO055_MAG_POWER_MODE_FORCE_MODE

        interrupt_clock_active=1;
        init_INT_BNO();

        if (newState == APP_STATE_ACTIVE)
        {

        }
        else
        {
          // Stop scheduled data measurements
        }
      }

}
#endif
/*********************************************************************
*********************************************************************/
#ifdef BNO080


uint8_t notify_Client=0;
uint8_t calibrationStatus=0;


bool resetBNO=false;
bool calibrate_roll=0;
bool calibrate_pitch=0;
bool calibrateMAGBNO=true;
extern float roll=0;
extern float pitch=0;
extern float yaw=0;
float Magnet=0;
extern uint16_t u16roll=0;
extern uint16_t u16pitch=0;
extern uint16_t u16yaw=0;
static uint8_t counter=4;
/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern bool MAGhasbeenCalibrated=false;
extern bool ACChasbeenCalibrated=false;
extern bool GYROhasbeenCalibrated=false;
extern signed short int roll_offset=0;
extern signed short int pitch_offset=0;
signed short int roll_value=0;
signed short int pitch_value=0;


uint8_t BNO080_valueAccelAccuracy=0;
uint8_t BNO080_valueGyroAccuracy=0;
uint8_t BNO080_valueMagAccuracy=0;
float BNO080_valueQuatRadianAccuracy=0;
uint8_t value1=0, value2=0, value3=0;
static Clock_Struct periodicClock;
static volatile bool sensorReadScheduled;

uint8_t sensorData1[SENSOR_DATA2_LEN];
uint8_t sensorData2[SENSOR_DATA2_LEN];



PIN_State pinState_i2c;
PIN_Handle i2c_pinhandle;
PIN_State pinGpioStateMov;
PIN_Handle hBnoPin = NULL;

static PIN_Config BoardI2C_PIN_Table[] =
{
    CC2640R2_LAUNCHXL_I2C0_SDA0 | PIN_INPUT_EN | PIN_PULLDOWN | PIN_OPENDRAIN,
    CC2640R2_LAUNCHXL_I2C0_SCL0 | PIN_INPUT_EN | PIN_PULLDOWN | PIN_OPENDRAIN,
    PIN_TERMINATE
};
// Pins that are used by the BNO055
static PIN_Config BnoPinTable_LOW[] =
{
    Board_BNO_INT    | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_NEGEDGE | PIN_HYSTERESIS,

    PIN_TERMINATE
};
static PIN_Config BnoPinTable_HIGH[] =
{
    Board_BNO_INT    | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE | PIN_HYSTERESIS,

    PIN_TERMINATE
};

static uint8_t sensorConfig1;
static uint8_t sensorConfig2;
static uint16_t sensorAxisMap;


static void sensorChangeCB(uint8_t paramID);
static void initCharacteristicValue(uint8_t paramID, uint8_t value, uint8_t paramLen);
static void SensorTagMov_clockHandler(UArg arg);



/*********************************************************************
 * PROFILE CALLBACKS
 */
static sensorCBs_t sensorCallbacks =
{
  sensorChangeCB,  // Characteristic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */


/*********************************************************************
 * @fn      SensorTagMov_init
 *
 * @brief   Initialization function for the SensorTag movement sub-application
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagMov_init(void)
{
  Movement_addService_bno();
  // Register callbacks with profile
  Movement_registerAppCBs_bno(&sensorCallbacks);

  // Initialize the module state variables
  sensorConfig1 = ST_CFG_SENSOR_DISABLE;
  sensorConfig2 = 0;
  sensorAxisMap = 0x24;
  sensorReadScheduled = false;


  // Initialize characteristics
  initCharacteristicValue(SENSOR_PERI,SENSOR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION,sizeof(uint8_t));
  // Create continuous clock for internal periodic events.
  Util_constructClock(&periodicClock, SensorTagMov_clockHandler,100, 0, false, MOV_INT_EVENT);//sensorPeriod
}
void init_INT_BNO(void)
{
    if(hBnoPin!=NULL)
        PIN_close(hBnoPin);
    if(hBnoPin==NULL)
    {
        hBnoPin = PIN_open(&pinGpioStateMov, BnoPinTable_HIGH);
    }
    if(hBnoPin!=NULL)
    {
        PIN_setInterrupt(hBnoPin, PIN_ID(Board_BNO_INT)|PIN_IRQ_DIS);
        if (PIN_registerIntCb(hBnoPin, SensorTagMov_processInterrupt) != NULL)
        {

        }
        else
        {
            PIN_setInterrupt(hBnoPin, PIN_ID(Board_BNO_INT)|PIN_IRQ_NEGEDGE);
        }
    }
}
void disable_INT_BNO(void)
{
    if(hBnoPin!=NULL)
    {
        PIN_setInterrupt(hBnoPin, PIN_ID(Board_BNO_INT) | PIN_IRQ_DIS);
        PIN_close(hBnoPin);
        hBnoPin = PIN_open(&pinGpioStateMov, BnoPinTable_LOW);
        PIN_setInterrupt(hBnoPin, PIN_ID(Board_BNO_INT) | PIN_IRQ_DIS);
    }
}
void set_i2c_pins_low_power(void)
{
    if(i2cHandle==NULL)
    {
        i2c_pinhandle = PIN_open(&pinState_i2c, BoardI2C_PIN_Table);

    }
}
void clear_i2c_pins_low_power(void)
{
    if(i2c_pinhandle!=NULL)
    {
        PIN_close(i2c_pinhandle);
        i2c_pinhandle=NULL;
    }
}

/*********************************************************************
 * @fn      SensorTagMov_processSensorEvent
 *
 * @brief   SensorTag Movement sensor event processor.
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagMov_processSensorEvent(void)
{
    if(in_connection)
    {
        if(i2cHandle==NULL && i2c_pinhandle==NULL)
        {
            SensorI2C_open();
        }


        if(sensorConfig1 & 0x20)
        {

            if(calibrate_roll==1)
            {
                calibrate_roll=0;
                roll_offset= (int16_t)((((int32_t)((int8_t)sensorData2[2])) <<8)| (sensorData2[3]))+roll_offset;
                SensorTagTrueWindFlash_Write();
            }
            if(calibrate_pitch==1)
            {
                calibrate_pitch=0;
                pitch_offset=(int16_t)((((int32_t)((int8_t)sensorData2[4])) <<8)| (sensorData2[5]))+pitch_offset;
                SensorTagTrueWindFlash_Write();
            }
            if(BNO080_dataAvailable())// || (calibration_valius & 0x03 != 3))
            {
                notify_Client=1;

                roll = ((BNO080_getRoll()) * 180.0 / 3.14) ; // Convert roll to degrees
                pitch = ((BNO080_getPitch()) * 180.0 / 3.14) * -1; // Convert pitch to degrees
                yaw = 360 - ((BNO080_getYaw()) * 180.0 / 3.14 +180); // Convert yaw / heading to degrees



                BNO080_valueAccelAccuracy=BNO080_getAccelAccuracy();
                BNO080_valueGyroAccuracy=BNO080_getGyroAccuracy();
                BNO080_valueMagAccuracy=BNO080_getMagAccuracy();
                BNO080_valueQuatRadianAccuracy=BNO080_getQuatRadianAccuracy();

//                Magnet=(atan2(BNO080_getMagY(), BNO080_getMagX()) * 180.0 / 3.14);
//
//                if(Magnet<0)
//                    Magnet+=360;

                yaw = yaw +180;

                if(yaw > 360)
                    yaw = yaw -360;

                if(yaw > 360)
                    yaw = yaw -360;


                u16roll = roll *16 - roll_offset;
                u16pitch = pitch *16 - pitch_offset;
                u16yaw = yaw *16; //360-0

                sensorData2[1] = (uint8_t) (u16yaw >>8);
                sensorData2[0] = (uint8_t) u16yaw;
                sensorData2[3] = (uint8_t) (u16roll >>8);
                sensorData2[2] = (uint8_t) u16roll;
                sensorData2[5] = (uint8_t) (u16pitch >>8);
                sensorData2[4] = (uint8_t) u16pitch;
                sensorData2[6] = (uint8_t) 0x07;
                sensorData2[7] = (uint8_t) 0x08;


    #ifdef PLUS_BROADCASTER_VALUES
                SensorTag_updateAdvertisingData(12,sensorData2[1]);
                SensorTag_updateAdvertisingData(13,sensorData2[0]);
                SensorTag_updateAdvertisingData(14,sensorData2[3]);
                SensorTag_updateAdvertisingData(15,sensorData2[2]);
                SensorTag_updateAdvertisingData(16,sensorData2[5]);
                SensorTag_updateAdvertisingData(17,sensorData2[4]);
    #endif
    #ifdef PLUS_BROADCASTER_VALUES_NEW
                SensorTag_updateAdvertisingData(15,sensorData2[1]);
                SensorTag_updateAdvertisingData(16,sensorData2[0]);
                SensorTag_updateAdvertisingData(17,sensorData2[3]);
                SensorTag_updateAdvertisingData(18,sensorData2[2]);
                SensorTag_updateAdvertisingData(19,sensorData2[5]);
                SensorTag_updateAdvertisingData(20,sensorData2[4]);
    #endif
            }

            if(counter>=4)
            {
                counter=0;
                Event_post(syncEvent, Battery_I2C_EVENT);
            }
            else
                counter++;

            if(calibrateMAGBNO)
            {
                if(resetBNO)
                {
                    resetBNO=false;
                    BNO080_enableMagnetometer(50);
                    BNO080_enableGyro(50);
                    BNO080_enableAccelerometer(50);
                    BNO080_enableRotationVector(50);
                    BNO080_calibrateAll();

                    MAGhasbeenCalibrated=false;
                    ACChasbeenCalibrated=false;
                    GYROhasbeenCalibrated=false;

                    DELAY_MS(650);
                }
                else
                {
                    read_calibration_values();
                }
            }
        }

        if(sensorConfig1 & 0x20)
        {
            if(notify_Client==1)
            {
                notify_Client=0;
                Movement_setParameter_bno(SENSOR_DATA2, SENSOR_DATA2_LEN, sensorData2);
            }
        }
    }

}


void read_calibration_values(void)
{


    BNO080_requestCalibrationStatus();

    value1 = BNO080_getMagAccuracy();
    if(value1==3)//Magnetometer Fully Calibrated
    {
        MAGhasbeenCalibrated=true;
    }

    value2 = BNO080_getAccelAccuracy();

    if(value2==3)//Magnetometer Fully Calibrated
    {
        ACChasbeenCalibrated=true;
    }

    value3 = BNO080_getGyroAccuracy();

    if(value3==3)//Magnetometer Fully Calibrated
    {
        GYROhasbeenCalibrated=true;
    }

    calibrationStatus=(value1*10+value2*10+value3*10)*100/90;//get procent on calibration status

    Movement_setParameter_bno(SENSOR_CALIBRATE, MOVEMENT_CALIBRATE_LEN, &calibrationStatus);

    if(GYROhasbeenCalibrated && ACChasbeenCalibrated && MAGhasbeenCalibrated)
    //if(BNO080_calibrationComplete())
    {
        calibrateMAGBNO=false;
        calibrationStatus=100;
        BNO080_endCalibration();
        BNO080_saveCalibration();
        BNO080_enableRotationVector(1000);
    }
}
/*********************************************************************
 * @fn      SensorTagMov_processCharChangeEvt
 *
 * @brief   SensorTag Movement event handling
 *
 * @param   paramID - identifies which characteristic has changed
 *
 * @return  none
 */
void SensorTagMov_processCharChangeEvt(uint8_t paramID)
{
  uint16_t newCfg;
  uint8_t newValue8;
  volatile uint8_t test;

  switch (paramID)
  {
      case SENSOR_CONF1:

          sensorConfig1=0;

        if (sensorConfig1 != ST_CFG_ERROR)
        {
            Movement_getParameter_bno(SENSOR_CONF1, &newCfg);

          if ((newCfg ) == ST_CFG_SENSOR_DISABLE)
          {
            // All axes off, turn off device power
            sensorConfig1 = newCfg;
            Movement_setParameter_bno(SENSOR_CONF1, sizeof(sensorConfig1), (uint8_t*)&sensorConfig1);
            appStateSet_mov(APP_STATE_OFF);

          }
          else
          {
            // Some axes on; power up and activate BNO
            sensorConfig1 = newCfg;

            if(sensorConfig1 == 0x0C){
                asm(" NOP");
                test++;
            }

            if(sensorConfig1 == 0x20){
                asm(" NOP");
                test++;
            }

            Movement_setParameter_bno(SENSOR_CONF1, sizeof(sensorConfig1), (uint8_t*)&sensorConfig1);

            appStateSet_mov(APP_STATE_ACTIVE);

          }
        }
        else
        {
          // Make sure the previous characteristics value is restored
          initCharacteristicValue(SENSOR_CONF1, sensorConfig1, sizeof(sensorConfig1));
        }
    break;

  case SENSOR_CONF2:

    if (sensorConfig2 != ST_CFG_ERROR)
    {
        Movement_getParameter_bno(SENSOR_CONF2, &newCfg);

      if ((newCfg ) == ST_CFG_SENSOR_DISABLE)
      {
        // All axes off, turn off device power
        sensorConfig2 = newCfg;
        appStateSet_mov(APP_STATE_OFF);
      }
      else
      {
        // Some axes on; power up and activate MPU
        sensorConfig2 = newCfg;
        appStateSet_mov(APP_STATE_ACTIVE);
      }

      Movement_setParameter_bno(SENSOR_CONF2, sizeof(sensorConfig2), (uint8_t*)&sensorConfig2);
    }
    else
    {
      // Make sure the previous characteristics value is restored
      initCharacteristicValue(SENSOR_CONF2, sensorConfig2, sizeof(sensorConfig2));
    }

    // Data initially zero
    initCharacteristicValue(SENSOR_DATA1, 0, SENSOR_DATA1_LEN);
    initCharacteristicValue(SENSOR_DATA2, 0, SENSOR_DATA2_LEN);
    initCharacteristicValue(SENSOR_DATA3, 0, SENSOR_DATA3_LEN);
    break;

  case SENSOR_AXISMAP:
      Movement_getParameter_bno(SENSOR_AXISMAP, &newCfg);

      break;

  case SENSOR_PERI:
    Movement_getParameter_bno(SENSOR_PERI, &newValue8);
    Util_rescheduleClock(&periodicClock,newValue8);
    break;
  case MOVEMENT_CALIBRATE:
      calibrate_roll=1;
      break;
  case MOVEMENT_CALIBRATE_PITCH:
      calibrate_pitch=1;
      break;
  case MOVEMENT_CALIBRATE_RESET:
      calibrateMAGBNO=1;
      resetBNO=true;
      GYROhasbeenCalibrated=false;
      ACChasbeenCalibrated=false;
      MAGhasbeenCalibrated=false;
      calibrationStatus=0;
      Movement_setParameter_bno(SENSOR_CALIBRATE, MOVEMENT_CALIBRATE_LEN, &calibrationStatus);
      break;


  default:
    // Should not get here
    break;
  }
}

/*********************************************************************
 * @fn      SensorTagMov_reset
 *
 * @brief   Reset characteristics and disable sensor
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagMov_reset(void)
{
  initCharacteristicValue(SENSOR_DATA1, 0, SENSOR_DATA1_LEN);
  initCharacteristicValue(SENSOR_DATA2, 0, SENSOR_DATA2_LEN);
  initCharacteristicValue(SENSOR_DATA3, 0, SENSOR_DATA3_LEN);
  initCharacteristicValue(SENSOR_CONF1, 0, SENSOR_CONF1_LEN);
  initCharacteristicValue(SENSOR_CONF2, 0, SENSOR_CONF2_LEN);
  initCharacteristicValue(SENSOR_AXISMAP, 0, SENSOR_AXISMAP_LEN);


  // Remove power from the BNO
  appStateSet_mov(APP_STATE_OFF);
}


/*********************************************************************
* Private functions
*/

/*********************************************************************
 * @fn      SensorTagMov_processInterrupt
 *
 * @brief   Interrupt handler for BNO
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTagMov_processInterrupt(PIN_Handle handle, PIN_Id pinId)
{
    if (pinId == Board_BNO_INT)
    {
        Event_post(syncEvent, MOV_INT_EVENT);
    }
}
void SensorTagMov_INT_Handle(void)
{
    Util_stopClock(&periodicClock);
    Util_rescheduleClock(&periodicClock,1000);
    Util_startClock(&periodicClock);

    SensorTagMov_processSensorEvent();
}
/*********************************************************************
 * @fn      SensorTagMov_clockHandler
 *
 * @brief   Handler function for clock time-outs.
 *
 * @param   arg - not used
 *
 * @return  none
 */
static void SensorTagMov_clockHandler(UArg arg)
{
    if(in_connection)
    {
        Event_post(syncEvent, arg);
        sensorReadScheduled = true;
        Util_startClock(&periodicClock);
    }
    else
    {

        disable_INT_BNO();
        BNO080_enableRotationVector(0);
        SensorI2C_close();
        set_i2c_pins_low_power();
        Util_stopClock(&periodicClock);
    }
}

/*********************************************************************
 * @fn      sensorChangeCB
 *
 * @brief   Callback from Movement Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void sensorChangeCB(uint8_t paramID)
{
  // Wake up the application thread
    SensorTag_charValueChangeCB(SERVICE_ID_MOV, paramID);
}


/*********************************************************************
 * @fn      initCharacteristicValue
 *
 * @brief   Initialize a characteristic value
 *
 * @param   paramID - parameter ID of the value is to be cleared
 *
 * @param   value - value to initialize with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 */
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen)
{
    switch(paramID){
    case SENSOR_CONF1:
          memset(&sensorConfig1,value,paramLen);
          Movement_setParameter_bno(paramID, paramLen, &sensorConfig1);
        break;
    case SENSOR_CONF2:
          memset(&sensorConfig2,value,paramLen);
          Movement_setParameter_bno(paramID, paramLen, &sensorConfig2);
        break;
    case SENSOR_AXISMAP:
          memset(&sensorAxisMap,value,paramLen);
          Movement_setParameter_bno(paramID, paramLen, &sensorAxisMap);
        break;
    case SENSOR_DATA1:
          memset(sensorData1,value,paramLen);
          Movement_setParameter_bno(paramID, paramLen, sensorData1);
        break;
    case SENSOR_DATA2:
          memset(sensorData2,value,paramLen);
          Movement_setParameter_bno(paramID, paramLen, sensorData2);
        break;
    case SENSOR_DATA3:
          //memset(sensorData3,value,paramLen);
          //Movement_setParameter_bno(paramID, paramLen, sensorData3);
        break;


    default:
        break;
    }
}
void stopClockMov(void)
{
    Util_stopClock(&periodicClock);
}
/*******************************************************************************
 * @fn      appStateSet_mov
 *
 * @brief   Set the application state
 *
 */
void appStateSet_mov(uint8_t newState)
{
    volatile uint8_t test;

    clear_i2c_pins_low_power();
    SensorI2C_close();

    if(i2cHandle==NULL && i2c_pinhandle ==NULL)
    {
        SensorI2C_open();
    }
    if (newState == APP_STATE_OFF)
    {
        // Stop scheduled data measurements
        Util_stopClock(&periodicClock);
        disable_INT_BNO();
        SensorI2C_close();
        set_i2c_pins_low_power();
    }

    if (newState == APP_STATE_ACTIVE || newState == APP_STATE_IDLE)
    {
        sensorReadScheduled = false;
        clear_watchdog();
        DELAY_MS(200);
        SensorI2C_deselect();
        BNO080_Init();
        clear_watchdog();
        init_INT_BNO();

        BNO080_enableRotationVector(1000);
//        BNO080_enableARVRStabilizedRotationVector(1000);
//        BNO080_enableMagnetometer(1000);

        if (newState == APP_STATE_ACTIVE)
        {
            Util_rescheduleClock(&periodicClock,1000);
            Util_startClock(&periodicClock);
            Event_post(syncEvent, Battery_I2C_EVENT);
        }
        else
        {
          // Stop scheduled data measurements
          Util_stopClock(&periodicClock);
        }
    }
}
#endif
