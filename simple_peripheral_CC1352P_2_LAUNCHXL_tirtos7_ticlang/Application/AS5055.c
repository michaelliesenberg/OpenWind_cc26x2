

/*!

*****************************************************************************

* Reads out chip data via SPI interface

*

* This function is used to read out cordic value from chips supporting SPI

* interface.

*****************************************************************************

*/

#include "AS5055.h"
//#include "ExtFlash.h"
#include "simple_peripheral.h"
#include <ti_drivers_config.h>
#include "sensortag_wind.h"
#include "osal_snv.h"

#define BUF_LEN 1
#define SNV_ID_APP 0x80
uint8 bufas[2] = {0,0};
uint8 status = 0;
uint8_t count_average=0;
float average=0;


extern int angle = 0;
extern uint8_t alarmHi=0;
extern uint8_t alarmLo=0;
extern 	uint16_t value=0;
extern 	uint16_t agc=0;
extern 	uint16_t error_flag=0;
extern float Angle_Value=0;
extern uint8_t error_flag_set=0;
int angle_average[4]={0,0,0,0};
int count_samples=0;
uint16_t agcreg; // 16-bit data buffer for SPI communication
uint16_t dat;
uint8_t data[2]={0,0};


uint8_t check_open=0;
uint8_t flagint=0;

void AS5055_init() {
    GPIO_setConfig(AS5055_CS, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(AS5055_INT, GPIO_CFG_INPUT | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(AS5055_INT, AS5055_INT_Callback);

    GPIO_disableInt(AS5055_INT);
    GPIO_write(AS5055_INT,0);
    GPIO_write(AS5055_CS, 0);
}

void AS5055_Power_Up(void)
{
	if(check_open==0)
	{
		bspSpiOpen();
	    //Flash_Open_Pin();
		//extFlashDeselect();
		GPIO_write(AS5055_CS, 1);

		spiWriteRegister(POR_OFF);
		spiWriteRegister(0x0000);
		//spiWriteRegister(Angular_Data);

		GPIO_enableInt(AS5055_INT);

		check_open=1;

	}

}
void AS5055_Power_Down(void)
{
	if(check_open==1)
	{
		check_open=0;

		GPIO_disableInt(AS5055_INT);
		bspSpiClose();
		GPIO_write(AS5055_CS, 0);

		//extFlashSelect();
	}
}
void openCSpin(void)
{
    GPIO_write(AS5055_CS, 1);
}


void set_low_on_pin_INT_AS5055(void)
{
    GPIO_disableInt(AS5055_INT);
}

void AS5055_INT_Callback(uint_least8_t index)
{
  if (index == AS5055_INT)
  {
      GPIO_clearInt(AS5055_INT);
      Set_semaphore_spi();
  }
}

void Set_AS5055_Chip_Enable(void)
{
	//extFlashDeselect();
    GPIO_write(AS5055_CS, 0);
}

void Clear_AS5055_Chip_Enable(void)
{
    GPIO_write(AS5055_CS, 1);
}
void PullDown_AS5055_Chip_Enable(void)
{
    GPIO_write(AS5055_CS, 0);
}

void AS5055_Handle_INT_Read_SPI(void)
{
    if(flagint==1)
       {
           flagint=0;

        /* Send NOP command. Received data is the ANGLE value, from the precedent command */
            dat = 0x0000; // NOP command.
            data[0] = dat>>8;
            data[1] = dat &0x00FF;
            //extFlashDeselect();
            Set_AS5055_Chip_Enable();
            //__delay_cycles(10);
            bspSpiRead(data, sizeof(uint16_t));
            //__delay_cycles(10);
            Clear_AS5055_Chip_Enable();
            //__delay_cycles(10);
            angle = data[0];
            angle = angle << 8 | data[1];

            uint16_t check_error = angle;


			error_flag_set=0;

			angle=angle/2;

			angle_average[0]= angle;//AS5055_modul_DSP(angle);

			count_samples=0;
			Angle_Value=angle_average[0];
			value = (uint16_t)Angle_Value  & 0x0fff; // Angle value (0..4095 for AS5055) //(angle >> 2) & 0x3fff

			Angle_Value = (float)(value * 360) / 4095; // Angle value in degree (0..359.9�)
			Angle_Value = 360 -Angle_Value;

			if(Wind_Direction_Scale_Flag)
			{
				Wind_Direction_Scale_Flag=false;
				AWA_offset= Angle_Value;

				OpenWindFlash_Write();
				OpenWindFlash_Read();
			}

			Angle_Value = Angle_Value -AWA_offset;

			if(Angle_Value<0)
			{
				Angle_Value=(360+Angle_Value);
			}

			int wert = Angle_Value*10;

			Angle_Value = (float)wert/10;

			Wind_Direction= Angle_Value;
       }
}
void AS5055_Send_Error(void)
{

    dat = SPI_CMD_READ | Error_Status;//(Error_Status<<1);
    dat |= spiCalcEvenParity(dat);
    data[0] = dat>>8;
    data[1] = dat &0x00FF;
    Set_AS5055_Chip_Enable();
    //__delay_cycles(10);
    bspSpiWrite(data, sizeof(uint16_t));
    //__delay_cycles(10);
    Clear_AS5055_Chip_Enable();

}
void AS5055_Read_Error(void)
{
	error_flag_set=0;
	// Send NOP command.
	dat = 0x0000; // NOP command.
	data[0] = dat>>8;
	data[1] = dat &0x00FF;
	Set_AS5055_Chip_Enable();
	bspSpiRead(data, sizeof(uint16_t));
	Clear_AS5055_Chip_Enable();

	error_flag = data[0];
	error_flag = error_flag << 8 | data[1];


	dat = Clear_EF | SPI_REG_DATA;;
	dat |= spiCalcEvenParity(dat);
	data[0] = dat>>8;
	data[1] = dat &0x00FF;
	Set_AS5055_Chip_Enable();
	bspSpiWrite(data, sizeof(uint16_t));
	Clear_AS5055_Chip_Enable();
}
void spiReadData()
{

	/* Send READ AGC command. Received data is thrown away: this data comes from the precedent
	command (unknown)*/
	/*
	dat = SPI_CMD_READ | SPI_REG_AGC;
	//dat = SPI_CMD_READ | AGC<<1;
	dat |= spiCalcEvenParity(dat);
	data[0] = dat>>8;
	data[1] = dat &0x00FF;
	Set_AS5055_Chip_Enable();
	__delay_cycles(100);
	bspSpiWrite(data, sizeof(uint16_t));
	__delay_cycles(100);
	Clear_AS5055_Chip_Enable();
	__delay_cycles(100);

*/
	/* Send READ ANGLE command. Received data is the AGC value, from the precedent command */
	dat = SPI_CMD_READ | SPI_REG_DATA;
	dat |= spiCalcEvenParity(dat);
	data[0] = dat>>8;
	data[1] = dat &0x00FF;
	Set_AS5055_Chip_Enable();
	//__delay_cycles(10);
	bspSpiWrite(data, sizeof(uint16_t));
	//__delay_cycles(10);
	Clear_AS5055_Chip_Enable();
	//__delay_cycles(10);
	//agcreg = dat;

	flagint=1;

}


void spiWriteRegister(uint16_t value)
{
	dat = value;
	dat |= spiCalcEvenParity(dat);
	data[0] = dat>>8;
	data[1] = dat & 0x00FF;
	Set_AS5055_Chip_Enable();
	//__delay_cycles(10);
	bspSpiWrite(data, sizeof(uint16_t));
	//__delay_cycles(10);
	Clear_AS5055_Chip_Enable();
}



/*!
*****************************************************************************
* Calculate even parity of a 16 bit unsigned integer
*
* This function is used by the SPI interface to calculate the even parity
* of the data which will be sent via SPI to the encoder.
*
* \param[in] value : 16 bit unsigned integer whose parity shall be calculated
*
* \return : Even parity
*
*****************************************************************************
*/
static uint8_t spiCalcEvenParity(uint16_t value)
{
	uint8_t cnt = 0;
	uint8_t i;

	for (i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}

	return cnt & 0x1;

}
