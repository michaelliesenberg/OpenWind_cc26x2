

#ifndef SENSORTAGWIND_H
#define SENSORTAGWIND_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "simple_peripheral_oad_onchip.h"
#include "windservice.h"
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */
#define WIND_OFFSET_LEN_ARRAY                12
/*********************************************************************
 * FUNCTIONS
 */

extern uint8_t parameter_wind;

/*
 * Initialize Windement sensor module
 */
extern void SensorTagWind_init(void);

/*
 * Task Event Processor for Windement sensor module
 */
extern void SensorTagWind_processSensorEvent(void);

/*
 * Task Event Processor for characteristic changes
 */
 void SensorTagWind_processCharChangeEvt(uint8_t paramID);

extern void SensorTagWind_readData();

/*
 * Reset Windement sensor module
 */
extern void SensorTagWind_reset(void);
void SPI_CS_POWEROFF(void);
void TrueWind_measNotify(float Sample);
void TrueWind_measPerTask(void);
void Set_semaphore_speed(void);
void PowerDown(void);
void PowerUp(void);
void Set_semaphore_spi(void);
void Start_Speed_Clock(void);
extern int8_t value_finetunning;
extern uint8_t WindOffsetArray[WIND_OFFSET_LEN_ARRAY];
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SENSORTAGWind_H */
