/*
 * AS5055.h
 *
 *  Created on: 23.04.2016
 *      Author: michael
 */

#ifndef APPLICATION_AS5055_H_
#define APPLICATION_AS5055_H_

#include "bsp_spi.h"

#define SPI_CMD_READ 				0x8000 /*!< flag indicating read attempt when using SPI interface */
#define SPI_REG_DATA 				0x7ffe /*!< data register when using SPI */
#define SPI_REG_AGC 				0x7ff0 /*!< agc register when using SPI */
#define SPI_REG_CLRERR 				0x6700 /*!< clear error register when using SPI */

#define POR_OFF						0x3F22
#define Software_Reset				0x3C00
#define Master_Reset				0x33A5
#define Clear_EF					0x3380
#define NOP							0x0000
#define AGC							0x3FF8
#define Angular_Data				0x3FFF
#define Error_Status				0x335A
#define System_Config				0x3F20


#define POR_OFF_Value				0x005A
#define Software_Rese_Value			0xB800
#define Software_Rese_Package		0x0004
#define Master_Reset_Value			0xAB4A
#define NOP_Value					0x0000
#define AGC_Value					0xFFFF
#define System_Config_Value			0x0000

void AS5055_Handle_INT_Read_SPI(void);
void AS5055_init();
void AS5055_Power_Up(void);
void AS5055_Power_Down(void);
static uint8_t spiCalcEvenParity(uint16_t value);
void spiReadData();
void openCSpin(void);
void Set_AS5055_Chip_Enable(void);
void Clear_AS5055_Chip_Enable(void);
void AS5055_Read_Error(void);
void AS5055_Send_Error(void);
void set_low_on_pin_INT_AS5055(void);
void spiWriteData(uint8_t value);
void spiWriteRegister(uint16_t value);
void PullDown_AS5055_Chip_Enable(void);
void AS5055_INT_Callback(uint_least8_t index);

extern int angle;
extern uint8_t alarmHi;
extern uint8_t alarmLo;
extern 	uint16_t value;
extern 	uint16_t agc;
extern float Angle_Value;
extern 	uint16_t error_flag;
extern uint8_t error_flag_set;



#endif /* APPLICATION_AS5055_H_ */
