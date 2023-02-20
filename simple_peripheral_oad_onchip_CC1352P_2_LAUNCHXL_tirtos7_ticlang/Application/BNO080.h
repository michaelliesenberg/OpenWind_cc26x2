/*
 * BNO080.h
 *
 *  Created on: 12 Aug 2020
 *      Author: michaelliesenberg
 */

#ifndef BNO080_H_
#define BNO080_H_

#include "simple_peripheral_oad_onchip.h"
#include "SensorUtil.h"
#include "SensorI2C.h"

#define I2C_BUFFER_LENGTH 32
#define BNO080_DEFAULT_ADDRESS 0x4B
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


bool BNO080_Init(void);
void BNO080_enableRotationVector(uint16_t timeBetweenReports);
void BNO080_enableGameRotationVector(uint16_t timeBetweenReports);
void BNO080_enableARVRStabilizedRotationVector(uint16_t timeBetweenReports);
void BNO080_enableMagnetometer(uint16_t timeBetweenReports);
void BNO080_enableGyro(uint16_t timeBetweenReports);
void BNO080_enableAccelerometer(uint16_t timeBetweenReports);
bool BNO080_dataAvailable(void);

float BNO080_getRoll();
float BNO080_getPitch();
float BNO080_getYaw();
void BNO080_calibrateAll();
void BNO080_endCalibration();
void BNO080_saveCalibration();
void BNO080_requestCalibrationStatus(); //Sends command to get status
bool BNO080_calibrationComplete();   //Checks ME Cal response for byte 5, R0 - Status
uint8_t BNO080_getAccelAccuracy();
uint8_t BNO080_getGyroAccuracy();
uint8_t BNO080_getMagAccuracy();
float BNO080_getQuatRadianAccuracy();


float BNO080_getMagX();
float BNO080_getMagY();
float BNO080_getMagZ();

#endif /* BNO080_H_ */
