/*
 * A1171.h
 *
 *  Created on: 23.04.2016
 *      Author: michael
 */

#ifndef APPLICATION_DRV5013_H_
#define APPLICATION_DRV5013_H_

#include "ti_drivers_config.h"

#include "simple_peripheral_oad_onchip.h"

void DRV5013_init(void);
void DRV5013_Power_Up(void);
void DRV5013_Power_Down(void);


extern int Speed_Counter;

#endif /* APPLICATION_DRV5013_H_ */
