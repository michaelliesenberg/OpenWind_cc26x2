
/*********************************************************************
 * INCLUDES
 */
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <driverlib/aon_rtc.h>
#include <DRV5013.h>
#include "gatt.h"
#include "gattservapp.h"

#include <ti/drivers/GPIO.h>
#include "sensortag_wind.h"
#include "AS5055.h"
#include "SensorUtil.h"
#include "util.h"
#include "string.h"
#include "sensortag_mov.h"
#include "SensorI2C.h"
#include "SensorUtil.h"
//#include "ExtFlash.h"
#include "st_util.h"
#include "simple_peripheral.h"
#include "BQ27441.h"
/*********************************************************************
 * CONSTANTS and MACROS
 */
// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD     100//100//200

// Length of the data for this sensor
#define SENSOR_DATA_LEN           9

// Event flag for this sensor
#define SENSOR_EVT                ST_GYROSCOPE_SENSOR_EVT

// Windement task states


#define WIND_MEAS_LEN			  22//22 12

static uint16_t gapConnHandle;
extern int8_t value_finetunning = 0;
extern uint8_t WindOffsetArray[WIND_OFFSET_LEN_ARRAY] = {0,0,0,0,0,0,0,0,0,0,0,0};//AWA_Offset,AWS_Offset,Yaw_Offset,Pitch_Offset,Roll__Offset
uint8 WindOffset_local[WIND_OFFSET_LEN_ARRAY] = {0,0,0,0,0,0,0,0,0,0,0,0};//AWA_Offset,AWS_Offset,Yaw_Offset,Pitch_Offset,Roll__Offset
uint8 WindMastAngleArray_local[WIND_MAST_ANGLE_LEN] = {0,0,0};
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
double rememberTime=0;
UInt64 actualTime;
uint32_t seconds=0;
uint32_t mINTseconds=0;
double mseconds=0;
double elapsedTime=0;
int Speed_Counter_Save=0;
uint8_t counter_up=0;
uint8_t speed_counter=0;
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */


/*********************************************************************
 * LOCAL VARIABLES
 */

static uint16_t sensorPeriod;
static volatile bool sensorReadScheduled;
static uint8_t sensorData[SENSOR_DATA_LEN];

// Application state variables

// MPU config:
// bit 0-2:   accelerometer enable(z,y,x)
// bit 3-5:   gyroscope enable (z,y,x)
// bit 6:     magnetometer enable
// bit 7:     WOM enable
// bit 8-9:   accelerometer range (2,4,8,16)
static uint16_t mpuConfig;

uint8_t count_speed_time=0;
static volatile bool mpuDataRdy;


extern uint8_t parameter_wind=0;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorChangeCB(uint8_t paramID);
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen);
static void appStateSet(uint8_t newState);

/*********************************************************************
 * PROFILE CALLBACKS
 */
static WindServiceCB_t sensorCallbacks =
{
  sensorChangeCB,  // Characteristic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SensorTagWind_init
 *
 * @brief   Initialization function for the SensorTag Windement sub-application
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagWind_init(void)
{
  // Add service
  Wind_AddService();

  // Register callbacks with profile
  Wind_Register(sensorCallbacks);

  sensorPeriod = SENSOR_DEFAULT_PERIOD;
  sensorReadScheduled = false;

  uint8_t senloc =Wind_SENS_LOC_WRIST;
  Wind_SetParameter(Wind_OFFSET, sizeof (uint8_t), &senloc);

  AS5055_init();
  DRV5013_init();

}

/*********************************************************************
 * @fn      SensorTagWind_processSensorEvent
 *
 * @brief   SensorTag Windement sensor event processor.
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagWind_processSensorEvent(void)
{

}


void Start_Speed_Clock(void)
{
}
/*******************************************************************************
 * @fn      PowerDown
 *
 * @brief
 * @return  none
 */
void PowerDown(void)
{
	AS5055_Power_Down();
	DRV5013_Power_Down();
}
/*******************************************************************************
 * @fn      PowerUp
 *
 * @brief
 * @return  none
 */
void PowerUp(void)
{
    DRV5013_Power_Up();
	AS5055_Power_Up();
	rememberTime=0;
	speed_counter=0;
}
/*********************************************************************
 * @fn      SensorTagWind_processCharChangeEvt
 *
 * @brief   SensorTag Windement event handling
 *
 * @param   paramID - identifies which characteristic has changed
 *
 * @return  none
 */
void SensorTagWind_processCharChangeEvt(uint8_t paramID)
{
  switch (paramID)
  {
  case Wind_MEAS_NOTI_ENABLED:
	  	  appStateSet(paramID);
    break;

  case Wind_MEAS_NOTI_DISABLED:
	  appStateSet(paramID);
    break;
  case Wind_COMMAND_SET: {
      int8_t value=0;
      Wind_GetParameter(Wind_COMMAND,&value);
      appStateSet(paramID);
    break;
  }
  case Wind_KEEP_SET: {
      uint8_t getvalue=0;
      Wind_GetParameter(Wind_Keep,&getvalue);

      keep_on = getvalue & 0x01;
      bno_low_power = (getvalue & 0x02) >> 1;
    break;
  }
  case Wind_MAST_ANGLE_SET: {
      Wind_GetParameter(Wind_MAST_ANGLE, WindMastAngleArray_local);

      if(WindMastAngleArray_local[0]==0x01)
      {
          MastAngle = (int16_t)(WindMastAngleArray_local[1] <<8 | WindMastAngleArray_local[2]);
      }
    break;
  }
  case Wind_OFFSET_SET: {
      Wind_GetParameter(Wind_OFFSET,WindOffset_local);

      if(WindOffset_local[0]==0x01 && WindOffset_local[11]==0x47)
      {
          memcpy(WindOffsetArray, WindOffset_local, WIND_OFFSET_LEN_ARRAY);
          AWA_offset    = (float)((int8_t)WindOffsetArray[1]<<8         | (uint8_t)WindOffsetArray[2])/10;
          AWS_offset    = (float)((int8_t)WindOffsetArray[3]<<8         | (uint8_t)WindOffsetArray[4])/10;
          yaw_offset    = (int16_t)(WindOffsetArray[5]<<8       | (uint8_t)WindOffsetArray[6]);
          pitch_offset  = (int16_t)(WindOffsetArray[7]<<8       | (uint8_t)WindOffsetArray[8]);
          roll_offset   = (int16_t)(WindOffsetArray[9]<<8       | (uint8_t)WindOffsetArray[10]);

          counter_up++;
          OpenWindFlash_Write();
      }
      else
      {
          Wind_SetParameter(Wind_OFFSET,WIND_OFFSET_LEN_ARRAY,WindOffsetArray);
      }

      //appStateSet(paramID);
    break;
  }
  default:
    // Should not get here
    break;
  }
}

/*********************************************************************
 * @fn      SensorTagWind_reset
 *
 * @brief   Reset characteristics and disable sensor
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagWind_reset(void)
{
  initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);

  Wind_SetParameter(SENSOR_CONF, sizeof(mpuConfig), (uint8_t*)&mpuConfig);

  // ReWinde power from the MPU
  appStateSet(APP_STATE_OFF);
}


/*********************************************************************
* Private functions
*/

void Set_semaphore_spi(void)
{
	if(OpenWind_has_Connection)
	{
	    //OPENWIND_charValueChangeCB(OW_AS5055_READ, 0);
	    OPENWIND_Event(OW_AS5055_READ);
	}
}

void SensorTagWind_readData()
{
    double rps;

    if(speed_counter>=10)
    {
        speed_counter=0;
        Speed_Counter_Save = Speed_Counter;

            /* read current RTC count */
        actualTime = AONRTCCurrent64BitValueGet();
        mINTseconds = (UInt32) (actualTime >> 16);
        mseconds = ((double)mINTseconds)/65536;
        elapsedTime = (mseconds-rememberTime)*1000;

         if(Speed_Counter_Save!=0)
         {
             //if(rememberTime==0)
             //    rps=Speed_Counter_Save/22 * 10;//Wind speed in rotation per seconds
             //else
             rps=(((float)Speed_Counter_Save)/22)* ((float)sensorPeriod)/elapsedTime * 1000/sensorPeriod;

             Wind_Speed = (rps*0.4606651786+0.1876025073)*1.94384;//m/s in Knots;
             Speed_Counter=0;

             if(Wind_Speed - AWS_offset >=0)
                 Wind_Speed -= AWS_offset;

         }
         else
             Wind_Speed=0;


         Speed_Counter=0;


         rememberTime = mseconds;
    }
    else
        speed_counter++;


    TrueWind_measPerTask();
}

/*********************************************************************
 * @fn      sensorChangeCB
 *
 * @brief   Callback from Windement Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void sensorChangeCB(uint8_t paramID)
{
  // Wake up the application thread
    parameter_wind=paramID;
    OpenWind_enqueueMsg(SERVICE_ID_Wind, paramID);//SERVICE_ID_Wind
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
static void initCharacteristicValue(uint8_t paramID, uint8_t value,uint8_t paramLen)
{
  memset(sensorData,value,paramLen);
  Wind_SetParameter(paramID, paramLen, sensorData);
}

/*******************************************************************************
 * @fn      appStateSet
 *
 * @brief   Set the application state
 *
 */
static void appStateSet(uint8_t newState)
{
  if (newState == Wind_MEAS_NOTI_ENABLED)
   {
	 // If connected start periodic measurement.
	  PowerUp();

	  OpenWind_START_DIRECTION=true;

	  GPIO_write(AS5055_CS, 1);

   }

   else if (newState == Wind_MEAS_NOTI_DISABLED)
   {
	 // Stop periodic measurement.
       OpenWind_START_DIRECTION=false;

   }
   else if (newState == Wind_COMMAND_SET)
   {
	   Wind_Direction_Scale_Flag=true;//set direction
   }
}

/*******************************************************************************
*******************************************************************************/
void TrueWind_measPerTask(void)
{
// Send TrueWind measurement notification.

	 if(OpenWind_has_Connection)
	 {
		 spiReadData();
		 TrueWind_measNotify(Wind_Direction);
	 }
	 else
	 {
		PowerDown();
		SPI_CS_POWEROFF();
	 }
}

void SPI_CS_POWEROFF(void)
{
    GPIO_write(AS5055_CS, 0);
}
/********************************************************************
 * @fn      Wind_measNotify_ECG_ECG
 *
 * @brief   Prepare and send a heart rate measurement notification.
 *
 * @return  none
 */
void TrueWind_measNotify(float Sample)
{
    long  long int number;
	attHandleValueNoti_t WindMeas;

  WindMeas.pValue = GATT_bm_alloc(gapConnHandle, ATT_HANDLE_VALUE_NOTI,WIND_MEAS_LEN, NULL);

  if (WindMeas.pValue != NULL)
  {
    uint8_t *p = WindMeas.pValue;
    // Build hert rate measurement structure from simulated values.
    *p++ = 0x01;//flags;
    Sample = Sample - MastAngle;

    if(Sample >360)
        Sample = Sample - 360;
    if(Sample <0)
        Sample = 360 + Sample;

	number = (Sample)*10;
	*p++ = (uint8_t)(number);
	*p++ = (uint8_t)((number)>>8);

#ifdef PLUS_BROADCASTER_VALUES
    SensorTag_updateAdvertisingData(8,HI_UINT16(number));
    SensorTag_updateAdvertisingData(9,LO_UINT16(number));
#endif
#ifdef PLUS_BROADCASTER_VALUES_NEW
    SensorTag_updateAdvertisingData(11,HI_UINT16(number));
    SensorTag_updateAdvertisingData(12,LO_UINT16(number));
#endif

	number = Wind_Speed*100;
	*p++ = (uint8_t)(number);
	*p++ = (uint8_t)((number)>>8);
	*p++ =  (uint8_t)(yaw_value);
	*p++ =  (uint8_t)(yaw_value >>8);
    *p++ =  (uint8_t)(roll_value);
    *p++ =  (uint8_t)(roll_value >>8);
    *p++ =  (uint8_t)(pitch_value);
    *p++ =  (uint8_t)(pitch_value >>8);
    *p++ =  (uint8_t)mov_calib_status;

 /*   *p++ =  (uint8_t)state_charge;
    *p++ =  (uint8_t)((state_charge)>>8);
    *p++ =  (uint8_t)average_current;
    *p++ =  (uint8_t)((average_current)>>8);
    *p++ =  (uint8_t)remaining_capacity;
    *p++ =  (uint8_t)((remaining_capacity)>>8);
    *p++ =  (uint8_t)voltage;
    *p++ =  (uint8_t)((voltage)>>8);
    *p++ =  (uint8_t)temperature;
    *p++ =  (uint8_t)((temperature)>>8);
    *p++ =  (uint8_t)system_capacity;
    *p++ =  (uint8_t)((system_capacity)>>8);
*/




#ifdef PLUS_BROADCASTER_VALUES
	SensorTag_updateAdvertisingData(10,HI_UINT16(number));
    SensorTag_updateAdvertisingData(11,LO_UINT16(number));
#endif
#ifdef PLUS_BROADCASTER_VALUES_NEW
    SensorTag_updateAdvertisingData(13,HI_UINT16(number));
    SensorTag_updateAdvertisingData(14,LO_UINT16(number));
#endif

    WindMeas.len = (uint8)(p - WindMeas.pValue);
    // Send notification.
    if (Wind_MeasNotify(gapConnHandle, &WindMeas) != SUCCESS)
    {
      GATT_bm_free((gattMsg_t *)&WindMeas, ATT_HANDLE_VALUE_NOTI);
    }
  }
}
