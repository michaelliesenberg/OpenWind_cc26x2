/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//****************************************************************************
//
// HAL_BQ27441.c - Hardware abstraction layer for interfacing BQ27441
//
//****************************************************************************

#include <stdio.h>
#include <stdint.h>
#include "BQ27441.h"
#include "SensorI2C.h"
#include "simple_peripheral.h"

/* Configures BQ27441 device properties */
uint8_t error=0;

extern uint16_t system_capacity=0;
extern uint16_t voltage=0;
extern uint16_t remaining_capacity=0;
extern uint16_t temperature=0;
extern uint16_t state_charge=0;
extern int16_t average_current=0;

/* BQ27441 Configuration */
#define BAT_CAPACITY  battery_capacity//1260;
#define CONF_TAPER_CURRENT  58    //TaperCurrent = 58mA Charger stop charging once consider batterry full

int CONF_DESIGN_CAPACITY =BAT_CAPACITY;    // Design Capacity = 1320mAh
int CONF_DESIGN_ENERGY= (int)(BAT_CAPACITY * 3.7) ;//4884;//(int)(CONF_DESIGN_CAPACITY * 3.7)  //DesignEnergy = DesignCapacity(mAh) * 3.7V
int CONF_TERMINATE_VOLTAGE=3000;    //TerminateVoltage = 3200mV

int CONF_TAPER_RATE =(BAT_CAPACITY / (0.1 * CONF_TAPER_CURRENT));



/* Control Subcommands */
int CONTROL_STATUS = 0x0000;
int DEVICE_TYPE =0x0001;
int FW_VERSION =0x0002;
int DM_CODE=0x0004;
int PREV_MACWRITE=0x0007;
int CHEM_ID=0x0008;
int BAT_INSERT=0x000C;
int BAT_REMOVE=0x000D;
int SET_HIBERNATE=0x0011;
int CLEAR_HIBERNATE=0x0012;

int SHUTDOWN_ENABLE=0x001B;
int SHUTDOWN=0x001C;
int SEALED =0x0020;
int TOGGLE_GPOUT=0x0023;
int RESET=0x0041;
int SOFT_RESET=0x0042;
int EXIT_CFGUPDATE =0x0043;
int EXIT_RESIM=0x0044;
int SET_CFGUPDATE  =0x0013;


uint8_t delat_time = 1;
uint16_t timeOut = 100;

void BQ27441_Setup(void)
{

	if (!BQ27441_initConfig())
	{
		error=1;//Error initializing BQ27441 Config
	}


	/*while (!BQ27441_initOpConfig())
	{
		delay_ms(100);//20
		//Clearing BIE in Operation Configuration\r\n

		    break;
	}*/

}

void BQ27441_Read_Values(void)
{
	int8_t result16[2] = {};
	//SensorI2C_deselect();
	SensorI2C_select(0, BQ27441_SLAVE_ADDRESS);
	/* Read Design Capacity */
	if(!BQ27441_read16(DESIGN_CAPACITY, result16, timeOut))
	{

	}
	else
	{
		system_capacity = result16[1]<<8 | (uint8_t)result16[0];//Design Capacity: result16
#ifdef PLUS_BROADCASTER_VALUES
	    SensorTag_updateAdvertisingData(26,result16[1]);
	    SensorTag_updateAdvertisingData(27,result16[0]);
#endif
	}

	/* Read Remaining Capacity */
	if(!BQ27441_read16(REMAINING_CAPACITY, result16, timeOut))
	{
		//Error Reading Remaining Capacity
	}
	else
	{
		remaining_capacity = result16[1]<<8 | (uint8_t)result16[0];//Remaining Capacity: result16
#ifdef PLUS_BROADCASTER_VALUES
	    SensorTag_updateAdvertisingData(24,result16[1]);
	    SensorTag_updateAdvertisingData(25,result16[0]);
#endif
#ifdef PLUS_BROADCASTER_VALUES_NEW
        SensorTag_updateAdvertisingData(27,result16[1]);
        SensorTag_updateAdvertisingData(28,result16[0]);
#endif
	}

	/* Read State Of Charge */
	if(!BQ27441_read16(STATE_OF_CHARGE, result16, timeOut))
	{
		//Error Reading State Of Charge
	}
	else
	{
		state_charge = result16[1]<<8 | (uint8_t)result16[0];//State of Charge: result16)
#ifdef PLUS_BROADCASTER_VALUES
		 SensorTag_updateAdvertisingData(18,result16[1]);
		 SensorTag_updateAdvertisingData(19,result16[0]);
#endif
#ifdef PLUS_BROADCASTER_VALUES_NEW
        SensorTag_updateAdvertisingData(21,result16[1]);
        SensorTag_updateAdvertisingData(22,result16[0]);
#endif
	}

	/* Read Temperature */
	if(!BQ27441_read16(TEMPERATURE, result16, timeOut))
	{
		//Error Reading Temperature
	}
	else
	{
		temperature = result16[1]<<8 | (uint8_t)result16[0];//Temperature:  result16/10 - 273;
		temperature = temperature/10 -273;
#ifdef PLUS_BROADCASTER_VALUES
		SensorTag_updateAdvertisingData(20,result16[1]);
		SensorTag_updateAdvertisingData(21,result16[0]);
#endif
#ifdef PLUS_BROADCASTER_VALUES_NEW
        SensorTag_updateAdvertisingData(23,result16[1]);
        SensorTag_updateAdvertisingData(24,result16[0]);
#endif
	}

	/* Read Voltage */
	if(!BQ27441_read16(VOLTAGE, result16, timeOut))
	{
		//Error Reading Voltage
	}
	else
	{
		voltage = result16[1]<<8 | (uint8_t)result16[0];//Voltage: result16
#ifdef PLUS_BROADCASTER_VALUES
	    SensorTag_updateAdvertisingData(22,result16[1]);
	    SensorTag_updateAdvertisingData(23,result16[0]);
#endif
#ifdef PLUS_BROADCASTER_VALUES_NEW
        SensorTag_updateAdvertisingData(25,result16[1]);
        SensorTag_updateAdvertisingData(26,result16[0]);
#endif
	}

	/* Read Average Current */
	if(!BQ27441_read16(AVERAGE_CURRENT, result16, timeOut))
	{
		//Error Reading Average Current
	}
	else
	{
		average_current = result16[1]<<8 | (uint8_t)result16[0];
#ifdef PLUS_BROADCASTER_VALUES
        SensorTag_updateAdvertisingData(28,result16[1]);
        SensorTag_updateAdvertisingData(29,result16[0]);
#endif
#ifdef PLUS_BROADCASTER_VALUES_NEW
        SensorTag_updateAdvertisingData(29,result16[1]);
        SensorTag_updateAdvertisingData(30,result16[0]);
#endif
		//Average Current:  result16
		if (result16[0] > 0)
		{
			//Status : charging
		}
		else
		{
			//Status : discharging
		}
	}
	SensorI2C_deselect();
	//SensorI2C_select(0, 0x28);//BNO Address

}

void read_state_battery(void)
{
	/* Read State Of Charge */
	SensorI2C_select(0, BQ27441_SLAVE_ADDRESS);
	int8_t result16[2] = {};
	if(!BQ27441_read16(STATE_OF_CHARGE, result16, timeOut))
	{
		//Error Reading State Of Charge
	}
	else
	{
		state_charge = result16[1]<<8 | (uint8_t)result16[0];//State of Charge: result16)
	}
}
bool BQ27441_initConfig()
{
	int16_t result = 0;
	int8_t char_result[2]={};
	int8_t send_help=0;
	//Default Config, DesignCapacity = 1200mAh, DesignEnergy = 1200mAh*3.7V, Terminate Voltage = 3200mV, Taper Current = 120mA


	if (!BQ27441_read16(FLAGS, char_result, timeOut))
		return 0;

	result = char_result[1]<<8 | char_result[0];//168 Decimal
	/* Check if ITPOR bit is set in FLAGS */
	if (result & 0x0020)
	{
		/* Instructs fuel gauge to enter CONFIG UPDATE mode. */
		send_help=SET_CFGUPDATE;
		int8_t send_char[2]={};
		send_char[0]=SET_CFGUPDATE & 0x00FF;
		send_char[1]=(SET_CFGUPDATE & 0xFF00) >>8;

		if (!BQ27441_control(send_char, timeOut))// 0x1300
			return 0;

		delay_ms(delat_time);

		result = 0;

		/* Check if CFGUPMODE bit is set in FLAGS */
		while(!(result & 0x0010))
		{
			if (!BQ27441_read16(FLAGS, char_result, timeOut))
				return 0;
			result = char_result[1]<<8 | (uint8_t)char_result[0];//184 Decimal

			    break;

		}

		/* Enable Block data memory control */
		uint8_t send = 0x00;
		if (!BQ27441_command(BLOCK_DATA_CONTROL, &send, timeOut))// 61 00
			return 0;

		/* Set the data class to be accessed */
		send=0x52;
		if (!BQ27441_command(DATA_CLASS, &send, timeOut))// 3E 52
			return 0;

		/* Write the block offset loaction */
		send=0x00;
		if (!BQ27441_command(DATA_BLOCK, &send, timeOut))// 3F 00
			return 0;

		delay_ms(delat_time);

		uint8_t old_chksum = 0;
		uint8_t new_chksum = 0;
		uint8_t tmp_chksum = 0;
		uint8_t chksum = 0;
		do
		{
			// Read Block Data Checksum
			if (!BQ27441_readChecksum(&old_chksum, timeOut))// 55W 60 55R AC 00 -> 172
				return 0;

			delay_ms(delat_time);

			// Checksum calculation
			tmp_chksum = (uint8_t)old_chksum; //172 Decimal

			uint16_t old_designCapacity = 0;
			uint16_t old_designEnergy = 0;
			uint16_t old_terminateVoltage = 0;
			uint16_t old_taperRate = 0;
			//uint8_t char_help[2]={};

			// Read old design capacity
			if (!BQ27441_read16(0x4A, char_result, timeOut))// 55R 05 3C -> 15365
				return 0;
			old_designCapacity = char_result[1]<<8 | char_result[0];//15365

			tmp_chksum = computeCheckSum(tmp_chksum, old_designCapacity, CONF_DESIGN_CAPACITY);

			// Read old design energy
			if (!BQ27441_read16(0x4C, char_result, timeOut))
				return 0;
			old_designEnergy = char_result[1]<<8 | char_result[0];//55W 4C 55R 13 00 ->24595

			tmp_chksum = computeCheckSum(tmp_chksum, old_designEnergy, CONF_DESIGN_ENERGY);

			// Read old terminate voltage
			if (!BQ27441_read16(0x50, char_result, timeOut))// 55w 50 55R 0C 80->-32756
				return 0;
			old_terminateVoltage = char_result[1]<<8 | char_result[0];//-32756

			tmp_chksum = computeCheckSum(tmp_chksum, old_terminateVoltage, CONF_TERMINATE_VOLTAGE);//67

			// Read old taper rate
			if (!BQ27441_read16(0x5B, char_result, timeOut))//55w 5B 55R 00 64->25600
				return 0;
			old_taperRate = char_result[1]<<8 | char_result[0];//25600
			// Checksum calculation
			tmp_chksum = computeCheckSum(tmp_chksum, old_taperRate, CONF_TAPER_RATE);

			// Write new design capacity
			send_help = 0x4A;


			uint8_t send_command[2]={};
			send_command[1]=(CONF_DESIGN_CAPACITY & 0x00FF);
			send_command[0]=(CONF_DESIGN_CAPACITY & 0xFF00)>>8;

			if (!BQ27441_write16(send_help, send_command, timeOut))//04 B0
				return 0;

			send_command[1]=CONF_DESIGN_ENERGY & 0x00FF; ;
			send_command[0]=(CONF_DESIGN_ENERGY & 0xFF00)>>8; ;

			if (!BQ27441_write16(0x4C, send_command, timeOut))//11 58
				return 0;

			// Write new terminate voltage

			send_command[1]=(CONF_TERMINATE_VOLTAGE & 0x00FF);
			send_command[0]=(CONF_TERMINATE_VOLTAGE & 0xFF00)>>8;

			if (!BQ27441_write16(0x50, send_command, timeOut))//0C 80
				return 0;

			// Write new taper rate

			send_command[1]=(CONF_TAPER_RATE & 0x00FF);
			send_command[0]=(CONF_TAPER_RATE & 0xFF00)>>8;
			if (!BQ27441_write16(0x5B, send_command, timeOut))// 00 68
				return 0;

			// Checksum calculation
			new_chksum = tmp_chksum;// 63

			// Write new checksum
			//send_help=BLOCK_DATA_CHECKSUM;
			uint8_t test = BLOCK_DATA_CHECKSUM;


			if (!BQ27441_command(test, &new_chksum, timeOut))// 55W 60 3F
				return 0;

			delay_ms(delat_time);

			// Read Block Data Checksum
			if (!BQ27441_readChecksum(&chksum, timeOut))//172
				return 0;

			delay_ms(delat_time);


			break;

		}
		while(new_chksum != chksum);

		/* Send SOFT_RESET control command */
		send_char[1]=0x42;//SOFT_RESET & 0x00FF;
		send_char[0]=0x00;//(SOFT_RESET & 0xFF00) >>8;
		if (!BQ27441_control(send_char, timeOut))//
			return 0;

		delay_ms(delat_time);

		result = 0;
		int8_t get_result[2]={};
		/* Check if CFGUPMODE bit is cleared in FLAGS */
		while(result & 0x0010)
		{
			if (!BQ27441_read16(FLAGS, get_result, timeOut))
				return 0;
			result = get_result[1]<<8 | (uint8_t)get_result[0];


                break;

		}

		return 1;
	}
	else
	{
		return 1;
	}
}


/* Configures BQ27441 opconfig */
bool BQ27441_initOpConfig()
{
	int16_t result = 0;
	int8_t char_result[2]={};

	/* Instructs fuel gauge to enter CONFIG UPDATE mode. */
	int8_t send_reg[2]={};
	send_reg[0] =0x13;//SET_CFGUPDATE & 0xFF00>>8;
	send_reg[1] =0x00;//SET_CFGUPDATE & 0x00FF;
	if (!BQ27441_control(send_reg, timeOut))// 55W 00 13 00
		return 0;

	delay_ms(delat_time);

	result = 0;
	// Check if CFGUPMODE bit is set in FLAGS
	while(!(result & 0x0010))//24
	{
		if (!BQ27441_read16(FLAGS, char_result, timeOut))
			return 0;
		result=(uint8_t)char_result[1]<<8 | (uint8_t)char_result[0];//152
	}

	//delay_ms(500);

	uint8_t reg = 0x00;

	// Enable Block data memory control
	if (!BQ27441_command(BLOCK_DATA_CONTROL, &reg, timeOut))// 55W 61 00
		return 0;

	// Set the data class to be accessed
	reg = 0x40;
	if (!BQ27441_command(DATA_CLASS, &reg, timeOut))// 55W 3E 40
		return 0;
	reg = 0x00;
	// Write the block offset loaction
	if (!BQ27441_command(DATA_BLOCK, &reg, timeOut))// 55W 3F 00
		return 0;

	delay_ms(delat_time);

	uint8_t old_chksum = 0;
	uint8_t new_chksum = 0;
	uint8_t tmp_chksum = 0;
	uint8_t chksum = 0;
	int8_t send = 0;

	return 1;
}


/* Send control subcommand */
bool BQ27441_control(int8_t *subcommand, unsigned int timeout)
{
/*	if(SensorI2C_open() == false)//I2C_init();
		return false;
	else
	{*/
		/* Specify slave address for BQ27441 */
		//SensorI2C_select(0, BQ27441_SLAVE_ADDRESS);//I2C_setslave(BQ27441_SLAVE_ADDRESS);


		if(!SensorI2C_writeReg(CONTROL, (uint8_t *)subcommand, 2))//if (!I2C_write16(CONTROL, subcommand, timeout))
			return 0;

		return 1;
	//}
}


/* Send control subcommand then read from control command */
bool BQ27441_controlRead(int8_t * subcommand, int8_t *result, unsigned int timeout)
{
	if (!BQ27441_control(subcommand, timeout))
		return 0;

	delay_ms(delat_time);

	if(!SensorI2C_readReg(CONTROL, (uint8_t *)result, 2))//if (!BQ27441_read16(CONTROL, result, timeout))
		return 0;

	return 1;
}

/* Send command */
bool BQ27441_command(uint8_t command, uint8_t* data, unsigned int timeout)
{
	/* Specify slave address for BQ27441 */
	//SensorI2C_select(0, BQ27441_SLAVE_ADDRESS);//I2C_setslave(BQ27441_SLAVE_ADDRESS);

	if(!SensorI2C_writeReg(command, data, 1))//if (!I2C_write8(command, data, timeout))
		return 0;

	return 1;
}

/* Write word to address */
bool BQ27441_write16(int8_t addr, uint8_t* data, unsigned int timeout)
{
	/* Specify slave address for BQ27441 */
	//SensorI2C_select(0, BQ27441_SLAVE_ADDRESS);

	if(!SensorI2C_writeReg(addr, (uint8_t *)data, 2))//if (!I2C_write16(addr, data, timeout))
		return 0;

	return 1;
}


/* Read from standard command*/
bool BQ27441_read16(int8_t stdcommand, int8_t *result, unsigned int timeout)
{
	//I2C_init();

	/* Specify slave address for BQ27441 */
	SensorI2C_select(0, BQ27441_SLAVE_ADDRESS);

	if(!SensorI2C_readReg(stdcommand, (uint8_t *)result, 2))//if (!I2C_read16(stdcommand, result, timeout))
		return 0;

	return 1;
}


/* Read block data checksum */
bool BQ27441_readChecksum(uint8_t *result, unsigned int timeout)
{
	//I2C_init();

	/* Specify slave address for BQ27441 */
	//SensorI2C_select(0, BQ27441_SLAVE_ADDRESS);

	if(!SensorI2C_readReg(BLOCK_DATA_CHECKSUM, (uint8_t *)result, 1))//if (!I2C_read8(BLOCK_DATA_CHECKSUM, result, timeout))
		return 0;

	return 1;
}


/* Computes checksum for fuel gauge */
static unsigned char computeCheckSum(unsigned char oldCheckSum, int oldData, int newData)
{
	unsigned char tmpCheckSum = 0xFF - oldCheckSum - ( unsigned char )oldData - ( unsigned char )( oldData >> 8 );
	unsigned char newCheckSum = 0xFF - (  tmpCheckSum + ( unsigned char )newData + ( unsigned char )( newData >> 8 ) );
	return newCheckSum;
}




