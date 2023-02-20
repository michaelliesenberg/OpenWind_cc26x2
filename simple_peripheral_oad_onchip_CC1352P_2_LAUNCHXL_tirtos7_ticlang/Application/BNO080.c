#include "BNO080.h"
#include "math.h"
#include "stdio.h"
#include <string.h>

//Registers
enum
{
    CHANNEL_COMMAND = 0,
    CHANNEL_EXECUTABLE,
    CHANNEL_CONTROL,
    CHANNEL_REPORTS,
    CHANNEL_WAKE_REPORTS,
    CHANNEL_GYRO
};

uint8_t arrayHelper[200];
uint8_t shtpDataHelper[200];
//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)


void BNO080_softReset();     //Try to reset the IMU via software
uint8_t BNO080_resetReason(); //Query the IMU for the reason it last reset

float qToFloat(int16_t fixedPointValue, uint8_t qPoint); //Given a Q value, converts fixed point floating to regular floating point number

bool BNO080_receivePacket(void);
bool BNO080_getData(uint16_t bytesRemaining); //Given a number of bytes, send the requests in I2C_BUFFER_LENGTH chunks
bool BNO080_sendPacket(uint8_t channelNumber, uint8_t dataLength);

void BNO080_parseInputReport(void);   //Parse sensor readings out of report
void BNO080_parseCommandReport(void); //Parse command responses out of report

float BNO080_getQuatI();
float BNO080_getQuatJ();
float BNO080_getQuatK();
float BNO080_getQuatReal();
float BNO080_getQuatRadianAccuracy();
uint8_t BNO080_getQuatAccuracy();

float BNO080_getAccelX();
float BNO080_getAccelY();
float BNO080_getAccelZ();

float BNO080_getLinAccelX();
float BNO080_getLinAccelY();
float BNO080_getLinAccelZ();
uint8_t BNO080_getLinAccelAccuracy();

float BNO080_getGyroX();
float BNO080_getGyroY();
float BNO080_getGyroZ();

float BNO080_getFastGyroX();
float BNO080_getFastGyroY();
float BNO080_getFastGyroZ();

void BNO080_calibrateAccelerometer();
void BNO080_calibrateGyro();
void BNO080_calibrateMagnetometer();
void BNO080_calibratePlanarAccelerometer();


uint32_t BNO080_getTimeStamp();
uint16_t BNO080_getStepCount();
uint8_t BNO080_getStabilityClassifier();
uint8_t BNO080_getActivityClassifier();

int16_t BNO080_getRawAccelX();
int16_t BNO080_getRawAccelY();
int16_t BNO080_getRawAccelZ();

int16_t BNO080_getRawGyroX();
int16_t BNO080_getRawGyroY();
int16_t BNO080_getRawGyroZ();

int16_t BNO080_getRawMagX();
int16_t BNO080_getRawMagY();
int16_t BNO080_getRawMagZ();



void BNO080_setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports);
void BNO080_setFeature_Command(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig);
void BNO080_sendCommand(uint8_t command);
void BNO080_sendCalibrateCommand(uint8_t thingToCalibrate);

//Metadata functions
int16_t BNO080_getQ1(uint16_t recordID);
int16_t BNO080_getQ2(uint16_t recordID);
int16_t BNO080_getQ3(uint16_t recordID);
float BNO080_getResolution(uint16_t recordID);
float BNO080_getRange(uint16_t recordID);
uint32_t BNO080_readFRSword(uint16_t recordID, uint8_t wordNumber);
void BNO080_frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize);
bool BNO080_readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);

//Global Variables
uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
uint8_t shtpData[MAX_PACKET_SIZE];
uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
uint8_t commandSequenceNumber = 0;              //Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
uint32_t metaData[MAX_METADATA_SIZE];           //There is more than 10 words in a metadata record but we'll stop at Q point 3



//These are the raw sensor values (without Q applied) pulled from the user requested Input Report
uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
uint16_t stepCount;
uint32_t timeStamp;
uint8_t stabilityClassifier;
uint8_t activityClassifier;
uint8_t *_activityConfidences;                        //Array that store the confidences of the 9 possible activities
uint8_t calibrationStatus_BNO080;                            //Byte R0 of ME Calibration Response
uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ; //Raw readings from MEMS sensor
uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;  //Raw readings from MEMS sensor
uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;       //Raw readings from MEMS sensor

//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
//See the read metadata example for more info
int16_t rotationVector_Q1 = 14;
int16_t rotationVectorAccuracy_Q1 = 12; //Heading accuracy estimate in radians. The Q point is 12.
int16_t accelerometer_Q1 = 8;
int16_t linear_accelerometer_Q1 = 8;
int16_t gyro_Q1 = 9;
int16_t magnetometer_Q1 = 4;
int16_t angular_velocity_Q1 = 10;




bool BNO080_Init(void)
{
    //Begin by resetting the IMU
    BNO080_softReset();

    //Check communication with device
    shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
    shtpData[1] = 0;                              //Reserved

    //Transmit packet on channel 2, 2 bytes
    BNO080_sendPacket(CHANNEL_CONTROL, 2);

    //Now we wait for response
    if (BNO080_receivePacket() == true)
    {
        if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
        {
            return (true);
        }
    }

    return (false); //Something went wrong
}

//Updates the latest variables if possible
//Returns false if new readings are not available
bool BNO080_dataAvailable(void)
{
    if (BNO080_receivePacket() == true)
    {
        //Check to see if this packet is a sensor reporting its data to us
        if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
        {
            BNO080_parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
            return (true);
        }
        else if (shtpHeader[2] == CHANNEL_CONTROL)
        {
            BNO080_parseCommandReport(); //This will update responses to commands, calibrationStatus_BNO080, etc.
            return (true);
        }
        else if(shtpHeader[2] == CHANNEL_GYRO)
        {
            BNO080_parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
          return (true);
        }
    }
    return (false);
}

//This function pulls the data from the command response report

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0]: The Report ID
//shtpData[1]: Sequence number (See 6.5.18.2)
//shtpData[2]: Command
//shtpData[3]: Command Sequence Number
//shtpData[4]: Response Sequence Number
//shtpData[5 + 0]: R0
//shtpData[5 + 1]: R1
//shtpData[5 + 2]: R2
//shtpData[5 + 3]: R3
//shtpData[5 + 4]: R4
//shtpData[5 + 5]: R5
//shtpData[5 + 6]: R6
//shtpData[5 + 7]: R7
//shtpData[5 + 8]: R8
void BNO080_parseCommandReport(void)
{
    if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
    {
        //The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
        uint8_t command = shtpData[2]; //This is the Command byte of the response

        if (command == COMMAND_ME_CALIBRATE)
        {
            calibrationStatus_BNO080 = shtpData[5 + 0]; //R0 - Status (0 = success, non-zero = fail)
        }
    }
    else
    {
        //This sensor report ID is unhandled.
        //See reference manual to add additional feature reports as needed
    }
}

//This function pulls the data from the input report
//The input reports vary in length so this function stores the various 16-bit values as globals

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
//shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
//shtpData[5 + 1]: Sequence number (See 6.5.18.2)
//shtpData[5 + 2]: Status
//shtpData[3]: Delay
//shtpData[4:5]: i/accel x/gyro x/etc
//shtpData[6:7]: j/accel y/gyro y/etc
//shtpData[8:9]: k/accel z/gyro z/etc
//shtpData[10:11]: real/gyro temp/etc
//shtpData[12:13]: Accuracy estimate
void BNO080_parseInputReport(void)
{
    //Calculate the number of data bytes in this packet
    int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
    dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
    //Ignore it for now. TODO catch this as an error and exit

    dataLength -= 4; //Remove the header bytes from the data count

    timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | ((uint32_t)shtpData[3] << (8 * 2)) | ((uint32_t)shtpData[2] << (8 * 1)) | ((uint32_t)shtpData[1] << (8 * 0));

    // The gyro-integrated input reports are sent via the special gyro channel and do no include the usual ID, sequence, and status fields
    if(shtpHeader[2] == CHANNEL_GYRO) {
        rawQuatI = (uint16_t)shtpData[1] << 8 | shtpData[0];
        rawQuatJ = (uint16_t)shtpData[3] << 8 | shtpData[2];
        rawQuatK = (uint16_t)shtpData[5] << 8 | shtpData[4];
        rawQuatReal = (uint16_t)shtpData[7] << 8 | shtpData[6];
        rawFastGyroX = (uint16_t)shtpData[9] << 8 | shtpData[8];
        rawFastGyroY = (uint16_t)shtpData[11] << 8 | shtpData[10];
        rawFastGyroZ = (uint16_t)shtpData[13] << 8 | shtpData[12];

        return;
    }

    uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits
    uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
    uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
    uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
    uint16_t data4 = 0;
    uint16_t data5 = 0; //We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports

    if (dataLength - 5 > 9)
    {
        data4 = (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
    }
    if (dataLength - 5 > 11)
    {
        data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
    }

    //Store these generic values to their proper global variable
    if (shtpData[5] == SENSOR_REPORTID_ACCELEROMETER)
    {
        accelAccuracy = status;
        rawAccelX = data1;
        rawAccelY = data2;
        rawAccelZ = data3;
    }
    else if (shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION)
    {
        accelLinAccuracy = status;
        rawLinAccelX = data1;
        rawLinAccelY = data2;
        rawLinAccelZ = data3;
    }
    else if (shtpData[5] == SENSOR_REPORTID_GYROSCOPE)
    {
        gyroAccuracy = status;
        rawGyroX = data1;
        rawGyroY = data2;
        rawGyroZ = data3;
    }
    else if (shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD)
    {
        magAccuracy = status;
        rawMagX = data1;
        rawMagY = data2;
        rawMagZ = data3;
    }
    else if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR ||
        shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
        shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR ||
        shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR)
    {
        quatAccuracy = status;
        rawQuatI = data1;
        rawQuatJ = data2;
        rawQuatK = data3;
        rawQuatReal = data4;

        //Only available on rotation vector and ar/vr stabilized rotation vector,
        // not game rot vector and not ar/vr stabilized rotation vector
        rawQuatRadianAccuracy = data5;
    }
    else if (shtpData[5] == SENSOR_REPORTID_STEP_COUNTER)
    {
        stepCount = data3; //Bytes 8/9
    }
    else if (shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER)
    {
        stabilityClassifier = shtpData[5 + 4]; //Byte 4 only
    }
    else if (shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER)
    {
        activityClassifier = shtpData[5 + 5]; //Most likely state

        //Load activity classification confidences into the array
        for (uint8_t x = 0; x < 9; x++)                    //Hardcoded to max of 9. TODO - bring in array size
            _activityConfidences[x] = shtpData[5 + 6 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
    }
    else if (shtpData[5] == SENSOR_REPORTID_RAW_ACCELEROMETER)
    {
        memsRawAccelX = data1;
        memsRawAccelY = data2;
        memsRawAccelZ = data3;
    }
    else if (shtpData[5] == SENSOR_REPORTID_RAW_GYROSCOPE)
    {
        memsRawGyroX = data1;
        memsRawGyroY = data2;
        memsRawGyroZ = data3;
    }
    else if (shtpData[5] == SENSOR_REPORTID_RAW_MAGNETOMETER)
    {
        memsRawMagX = data1;
        memsRawMagY = data2;
        memsRawMagZ = data3;
    }
    else if (shtpData[5] == SHTP_REPORT_COMMAND_RESPONSE)
    {
        //The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
        uint8_t command = shtpData[5 + 2]; //This is the Command byte of the response

        if (command == COMMAND_ME_CALIBRATE)
        {
            calibrationStatus_BNO080 = shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
        }
    }
    else
    {
        //This sensor report ID is unhandled.
        //See reference manual to add additional feature reports as needed
    }

    //TODO additional feature reports may be strung together. Parse them all.
}

// Quaternion to Euler conversion
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/issues/5#issuecomment-306509440
// Return the roll (rotation around the x-axis) in Radians
float BNO080_getRoll()
{
    float dqw = BNO080_getQuatReal();
    float dqx = BNO080_getQuatI();
    float dqy = BNO080_getQuatJ();
    float dqz = BNO080_getQuatK();

    float norm = sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
    dqw = dqw/norm;
    dqx = dqx/norm;
    dqy = dqy/norm;
    dqz = dqz/norm;

    float ysqr = dqy * dqy;

    // roll (x-axis rotation)
    float t0 = +2.0 * (dqw * dqx + dqy * dqz);
    float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
    float roll = atan2(t0, t1);

    return (roll);
}

// Return the pitch (rotation around the y-axis) in Radians
float BNO080_getPitch()
{
    float dqw = BNO080_getQuatReal();
    float dqx = BNO080_getQuatI();
    float dqy = BNO080_getQuatJ();
    float dqz = BNO080_getQuatK();

    float norm = sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
    dqw = dqw/norm;
    dqx = dqx/norm;
    dqy = dqy/norm;
    dqz = dqz/norm;

    float ysqr = dqy * dqy;

    // pitch (y-axis rotation)
    float t2 = +2.0 * (dqw * dqy - dqz * dqx);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    float pitch = asin(t2);

    return (pitch);
}

// Return the yaw / heading (rotation around the z-axis) in Radians
float BNO080_getYaw()
{
    float dqw = BNO080_getQuatReal();
    float dqx = BNO080_getQuatI();
    float dqy = BNO080_getQuatJ();
    float dqz = BNO080_getQuatK();

    float norm = sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
    dqw = dqw/norm;
    dqx = dqx/norm;
    dqy = dqy/norm;
    dqz = dqz/norm;

    float ysqr = dqy * dqy;

    // yaw (z-axis rotation)
    float t3 = +2.0 * (dqw * dqz + dqx * dqy);
    float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
    float yaw = atan2(t3, t4);

    return (yaw);
}

//Return the rotation vector quaternion I
float BNO080_getQuatI()
{
    float quat = qToFloat(rawQuatI, rotationVector_Q1);
    return (quat);
}

//Return the rotation vector quaternion J
float BNO080_getQuatJ()
{
    float quat = qToFloat(rawQuatJ, rotationVector_Q1);
    return (quat);
}

//Return the rotation vector quaternion K
float BNO080_getQuatK()
{
    float quat = qToFloat(rawQuatK, rotationVector_Q1);
    return (quat);
}

//Return the rotation vector quaternion Real
float BNO080_getQuatReal()
{
    float quat = qToFloat(rawQuatReal, rotationVector_Q1);
    return (quat);
}

//Return the rotation vector accuracy
float BNO080_getQuatRadianAccuracy()
{
    float quat = qToFloat(rawQuatRadianAccuracy, rotationVectorAccuracy_Q1);
    return (quat);
}

//Return the acceleration component
uint8_t BNO080_getQuatAccuracy()
{
    return (quatAccuracy);
}

//Return the acceleration component
float BNO080_getAccelX()
{
    float accel = qToFloat(rawAccelX, accelerometer_Q1);
    return (accel);
}

//Return the acceleration component
float BNO080_getAccelY()
{
    float accel = qToFloat(rawAccelY, accelerometer_Q1);
    return (accel);
}

//Return the acceleration component
float BNO080_getAccelZ()
{
    float accel = qToFloat(rawAccelZ, accelerometer_Q1);
    return (accel);
}

//Return the acceleration component
uint8_t BNO080_getAccelAccuracy()
{
    return (accelAccuracy);
}

// linear acceleration, i.e. minus gravity

//Return the acceleration component
float BNO080_getLinAccelX()
{
    float accel = qToFloat(rawLinAccelX, linear_accelerometer_Q1);
    return (accel);
}

//Return the acceleration component
float BNO080_getLinAccelY()
{
    float accel = qToFloat(rawLinAccelY, linear_accelerometer_Q1);
    return (accel);
}

//Return the acceleration component
float BNO080_getLinAccelZ()
{
    float accel = qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
    return (accel);
}

//Return the acceleration component
uint8_t BNO080_getLinAccelAccuracy()
{
    return (accelLinAccuracy);
}

//Return the gyro component
float BNO080_getGyroX()
{
    float gyro = qToFloat(rawGyroX, gyro_Q1);
    return (gyro);
}

//Return the gyro component
float BNO080_getGyroY()
{
    float gyro = qToFloat(rawGyroY, gyro_Q1);
    return (gyro);
}

//Return the gyro component
float BNO080_getGyroZ()
{
    float gyro = qToFloat(rawGyroZ, gyro_Q1);
    return (gyro);
}

//Return the gyro component
uint8_t BNO080_getGyroAccuracy()
{
    return (gyroAccuracy);
}

//Return the magnetometer component
float BNO080_getMagX()
{
    float mag = qToFloat(rawMagX, magnetometer_Q1);
    return (mag);
}

//Return the magnetometer component
float BNO080_getMagY()
{
    float mag = qToFloat(rawMagY, magnetometer_Q1);
    return (mag);
}

//Return the magnetometer component
float BNO080_getMagZ()
{
    float mag = qToFloat(rawMagZ, magnetometer_Q1);
    return (mag);
}

//Return the mag component
uint8_t BNO080_getMagAccuracy()
{
    return (magAccuracy);
}

// Return the high refresh rate gyro component
float BNO080_getFastGyroX()
{
    float gyro = qToFloat(rawFastGyroX, angular_velocity_Q1);
    return (gyro);
}

// Return the high refresh rate gyro component
float BNO080_getFastGyroY()
{
    float gyro = qToFloat(rawFastGyroY, angular_velocity_Q1);
    return (gyro);
}

// Return the high refresh rate gyro component
float BNO080_getFastGyroZ()
{
    float gyro = qToFloat(rawFastGyroZ, angular_velocity_Q1);
    return (gyro);
}

//Return the step count
uint16_t BNO080_getStepCount()
{
    return (stepCount);
}

//Return the stability classifier
uint8_t BNO080_getStabilityClassifier()
{
    return (stabilityClassifier);
}

//Return the activity classifier
uint8_t BNO080_getActivityClassifier()
{
    return (activityClassifier);
}

//Return the time stamp
uint32_t BNO080_getTimeStamp()
{
    return (timeStamp);
}

//Return raw mems value for the accel
int16_t BNO080_getRawAccelX()
{
    return (memsRawAccelX);
}
//Return raw mems value for the accel
int16_t BNO080_getRawAccelY()
{
    return (memsRawAccelY);
}
//Return raw mems value for the accel
int16_t BNO080_getRawAccelZ()
{
    return (memsRawAccelZ);
}

//Return raw mems value for the gyro
int16_t BNO080_getRawGyroX()
{
    return (memsRawGyroX);
}
int16_t BNO080_getRawGyroY()
{
    return (memsRawGyroY);
}
int16_t BNO080_getRawGyroZ()
{
    return (memsRawGyroZ);
}

//Return raw mems value for the mag
int16_t BNO080_getRawMagX()
{
    return (memsRawMagX);
}
int16_t BNO080_getRawMagY()
{
    return (memsRawMagY);
}
int16_t BNO080_getRawMagZ()
{
    return (memsRawMagZ);
}

//Given a record ID, read the Q1 value from the metaData record in the FRS (ya, it's complicated)
//Q1 is used for all sensor data calculations
int16_t BNO080_getQ1(uint16_t recordID)
{
    //Q1 is always the lower 16 bits of word 7
    uint16_t q = BNO080_readFRSword(recordID, 7) & 0xFFFF; //Get word 7, lower 16 bits
    return (q);
}

//Given a record ID, read the Q2 value from the metaData record in the FRS
//Q2 is used in sensor bias
int16_t BNO080_getQ2(uint16_t recordID)
{
    //Q2 is always the upper 16 bits of word 7
    uint16_t q = BNO080_readFRSword(recordID, 7) >> 16; //Get word 7, upper 16 bits
    return (q);
}

//Given a record ID, read the Q3 value from the metaData record in the FRS
//Q3 is used in sensor change sensitivity
int16_t BNO080_getQ3(uint16_t recordID)
{
    //Q3 is always the upper 16 bits of word 8
    uint16_t q = BNO080_readFRSword(recordID, 8) >> 16; //Get word 8, upper 16 bits
    return (q);
}

//Given a record ID, read the resolution value from the metaData record in the FRS for a given sensor
float BNO080_getResolution(uint16_t recordID)
{
    //The resolution Q value are 'the same as those used in the sensor's input report'
    //This should be Q1.
    int16_t Q = BNO080_getQ1(recordID);

    //Resolution is always word 2
    uint32_t value = BNO080_readFRSword(recordID, 2); //Get word 2

    float resolution = qToFloat(value, Q);

    return (resolution);
}

//Given a record ID, read the range value from the metaData record in the FRS for a given sensor
float BNO080_getRange(uint16_t recordID)
{
    //The resolution Q value are 'the same as those used in the sensor's input report'
    //This should be Q1.
    int16_t Q = BNO080_getQ1(recordID);

    //Range is always word 1
    uint32_t value = BNO080_readFRSword(recordID, 1); //Get word 1

    float range = qToFloat(value, Q);

    return (range);
}

//Given a record ID and a word number, look up the word data
//Helpful for pulling out a Q value, range, etc.
//Use readFRSdata for pulling out multi-word objects for a sensor (Vendor data for example)
uint32_t BNO080_readFRSword(uint16_t recordID, uint8_t wordNumber)
{
    if (BNO080_readFRSdata(recordID, wordNumber, 1) == true) //Get word number, just one word in length from FRS
        return (metaData[0]);                         //Return this one word

    return (0); //Error
}

//Ask the sensor for data from the Flash Record System
//See 6.3.6 page 40, FRS Read Request
void BNO080_frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize)
{
    shtpData[0] = SHTP_REPORT_FRS_READ_REQUEST; //FRS Read Request
    shtpData[1] = 0;                            //Reserved
    shtpData[2] = (readOffset >> 0) & 0xFF;     //Read Offset LSB
    shtpData[3] = (readOffset >> 8) & 0xFF;     //Read Offset MSB
    shtpData[4] = (recordID >> 0) & 0xFF;       //FRS Type LSB
    shtpData[5] = (recordID >> 8) & 0xFF;       //FRS Type MSB
    shtpData[6] = (blockSize >> 0) & 0xFF;      //Block size LSB
    shtpData[7] = (blockSize >> 8) & 0xFF;      //Block size MSB

    //Transmit packet on channel 2, 8 bytes
    BNO080_sendPacket(CHANNEL_CONTROL, 8);
}

//Given a sensor or record ID, and a given start/stop bytes, read the data from the Flash Record System (FRS) for this sensor
//Returns true if metaData array is loaded successfully
//Returns false if failure
bool BNO080_readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead)
{
    uint8_t spot = 0;

    //First we send a Flash Record System (FRS) request
    BNO080_frsReadRequest(recordID, startLocation, wordsToRead); //From startLocation of record, read a # of words

    //Read bytes until FRS reports that the read is complete
    while (1)
    {
        //Now we wait for response
        while (1)
        {
            uint8_t counter = 0;
            while (BNO080_receivePacket() == false)
            {
                if (counter++ > 100)
                    return (false); //Give up
                DELAY_MS(1);
            }

            //We have the packet, inspect it for the right contents
            //See page 40. Report ID should be 0xF3 and the FRS types should match the thing we requested
            if (shtpData[0] == SHTP_REPORT_FRS_READ_RESPONSE)
                if (((uint16_t)shtpData[13] << 8 | shtpData[12]) == recordID)
                    break; //This packet is one we are looking for
        }

        uint8_t dataLength = shtpData[1] >> 4;
        uint8_t frsStatus = shtpData[1] & 0x0F;

        uint32_t data0 = (uint32_t)shtpData[7] << 24 | (uint32_t)shtpData[6] << 16 | (uint32_t)shtpData[5] << 8 | (uint32_t)shtpData[4];
        uint32_t data1 = (uint32_t)shtpData[11] << 24 | (uint32_t)shtpData[10] << 16 | (uint32_t)shtpData[9] << 8 | (uint32_t)shtpData[8];

        //Record these words to the metaData array
        if (dataLength > 0)
        {
            metaData[spot++] = data0;
        }
        if (dataLength > 1)
        {
            metaData[spot++] = data1;
        }

        if (spot >= MAX_METADATA_SIZE)
        {
            return (true); //We have run out of space in our array. Bail.
        }

        if (frsStatus == 3 || frsStatus == 6 || frsStatus == 7)
        {
            return (true); //FRS status is read completed! We're done!
        }
    }
}

//Send command to reset IC
//Read all advertisement packets from sensor
//The sensor has been seen to reset twice if we attempt too much too quickly.
//This seems to work reliably.
void BNO080_softReset(void)
{
    shtpData[0] = 1; //Reset

    //Attempt to start communication with sensor
    BNO080_sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte

    //Read all incoming data and flush it
    DELAY_MS(50);
    while (BNO080_receivePacket() == true)
        ;
    DELAY_MS(50);
    while (BNO080_receivePacket() == true)
        ;
}

//Get the reason for the last reset
//1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
uint8_t BNO080_resetReason()
{
    shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
    shtpData[1] = 0;                              //Reserved

    //Transmit packet on channel 2, 2 bytes
    BNO080_sendPacket(CHANNEL_CONTROL, 2);

    //Now we wait for response
    if (BNO080_receivePacket() == true)
    {
        if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
        {
            return (shtpData[1]);
        }
    }

    return (0);
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
    float qFloat = fixedPointValue;
    qFloat *= pow(2, qPoint * -1);
    return (qFloat);
}

//Sends the packet to enable the rotation vector
void BNO080_enableRotationVector(uint16_t timeBetweenReports)
{
    BNO080_setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
void BNO080_enableARVRStabilizedRotationVector(uint16_t timeBetweenReports)
{
    BNO080_setFeatureCommand(SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the rotation vector
void BNO080_enableGameRotationVector(uint16_t timeBetweenReports)
{
    BNO080_setFeatureCommand(SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
void BNO080_enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports)
{
    BNO080_setFeatureCommand(SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
void BNO080_enableAccelerometer(uint16_t timeBetweenReports)
{
    BNO080_setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
void BNO080_enableLinearAccelerometer(uint16_t timeBetweenReports)
{
    BNO080_setFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports);
}

//Sends the packet to enable the gyro
void BNO080_enableGyro(uint16_t timeBetweenReports)
{
    BNO080_setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports);
}

//Sends the packet to enable the magnetometer
void BNO080_enableMagnetometer(uint16_t timeBetweenReports)
{
    BNO080_setFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports);
}

//Sends the packet to enable the high refresh-rate gyro-integrated rotation vector
void BNO080_enableGyroIntegratedRotationVector(uint16_t timeBetweenReports)
{
    BNO080_setFeatureCommand(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the step counter
void BNO080_enableStepCounter(uint16_t timeBetweenReports)
{
    BNO080_setFeatureCommand(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports);
}

//Sends the packet to enable the Stability Classifier
void BNO080_enableStabilityClassifier(uint16_t timeBetweenReports)
{
    BNO080_setFeatureCommand(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void BNO080_enableRawAccelerometer(uint16_t timeBetweenReports)
{
    BNO080_setFeatureCommand(SENSOR_REPORTID_RAW_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void BNO080_enableRawGyro(uint16_t timeBetweenReports)
{
    BNO080_setFeatureCommand(SENSOR_REPORTID_RAW_GYROSCOPE, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void BNO080_enableRawMagnetometer(uint16_t timeBetweenReports)
{
    BNO080_setFeatureCommand(SENSOR_REPORTID_RAW_MAGNETOMETER, timeBetweenReports);
}


//Sends the commands to begin calibration of the accelerometer
void BNO080_calibrateAccelerometer()
{
    BNO080_sendCalibrateCommand(CALIBRATE_ACCEL);
}

//Sends the commands to begin calibration of the gyro
void BNO080_calibrateGyro()
{
    BNO080_sendCalibrateCommand(CALIBRATE_GYRO);
}

//Sends the commands to begin calibration of the magnetometer
void BNO080_calibrateMagnetometer()
{
    BNO080_sendCalibrateCommand(CALIBRATE_MAG);
}

//Sends the commands to begin calibration of the planar accelerometer
void BNO080_calibratePlanarAccelerometer()
{
    BNO080_sendCalibrateCommand(CALIBRATE_PLANAR_ACCEL);
}

//See 2.2 of the Calibration Procedure document 1000-4044
void BNO080_calibrateAll()
{
    BNO080_sendCalibrateCommand(CALIBRATE_ACCEL_GYRO_MAG);
}

void BNO080_endCalibration()
{
    BNO080_sendCalibrateCommand(CALIBRATE_STOP); //Disables all calibrations
}

//See page 51 of reference manual - ME Calibration Response
//Byte 5 is parsed during the readPacket and stored in calibrationStatus_BNO080
bool BNO080_calibrationComplete()
{
    if (calibrationStatus_BNO080 == 0)
        return (true);
    return (false);
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
void BNO080_setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports)
{
    BNO080_setFeature_Command(reportID, timeBetweenReports, 0); //No specific config
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void BNO080_setFeature_Command(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
    long microsBetweenReports = (long)timeBetweenReports * 1000L;

    shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;   //Set feature command. Reference page 55
    shtpData[1] = reportID;                            //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
    shtpData[2] = 0;                                   //Feature flags
    shtpData[3] = 0;                                   //Change sensitivity (LSB)
    shtpData[4] = 0;                                   //Change sensitivity (MSB)
    shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
    shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  //Report interval
    shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
    shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
    shtpData[9] = 0;                                   //Batch Interval (LSB)
    shtpData[10] = 0;                                  //Batch Interval
    shtpData[11] = 0;                                  //Batch Interval
    shtpData[12] = 0;                                  //Batch Interval (MSB)
    shtpData[13] = (specificConfig >> 0) & 0xFF;       //Sensor-specific config (LSB)
    shtpData[14] = (specificConfig >> 8) & 0xFF;       //Sensor-specific config
    shtpData[15] = (specificConfig >> 16) & 0xFF;     //Sensor-specific config
    shtpData[16] = (specificConfig >> 24) & 0xFF;     //Sensor-specific config (MSB)

    //Transmit packet on channel 2, 17 bytes
    BNO080_sendPacket(CHANNEL_CONTROL, 17);
}

//Tell the sensor to do a command
//See 6.3.8 page 41, Command request
//The caller is expected to set P0 through P8 prior to calling
void BNO080_sendCommand(uint8_t command)
{
    shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
    shtpData[1] = commandSequenceNumber++;   //Increments automatically each function call
    shtpData[2] = command;                     //Command

    //Caller must set these
    /*shtpData[3] = 0; //P0
    shtpData[4] = 0; //P1
    shtpData[5] = 0; //P2
    shtpData[6] = 0;
    shtpData[7] = 0;
    shtpData[8] = 0;
    shtpData[9] = 0;
    shtpData[10] = 0;
    shtpData[11] = 0;*/

    //Transmit packet on channel 2, 12 bytes
    BNO080_sendPacket(CHANNEL_CONTROL, 12);
}

//This tells the BNO080 to begin calibrating
//See page 50 of reference manual and the 1000-4044 calibration doc
void BNO080_sendCalibrateCommand(uint8_t thingToCalibrate)
{
    /*shtpData[3] = 0; //P0 - Accel Cal Enable
    shtpData[4] = 0; //P1 - Gyro Cal Enable
    shtpData[5] = 0; //P2 - Mag Cal Enable
    shtpData[6] = 0; //P3 - Subcommand 0x00
    shtpData[7] = 0; //P4 - Planar Accel Cal Enable
    shtpData[8] = 0; //P5 - Reserved
    shtpData[9] = 0; //P6 - Reserved
    shtpData[10] = 0; //P7 - Reserved
    shtpData[11] = 0; //P8 - Reserved*/

    for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
        shtpData[x] = 0;

    if (thingToCalibrate == CALIBRATE_ACCEL)
        shtpData[3] = 1;
    else if (thingToCalibrate == CALIBRATE_GYRO)
        shtpData[4] = 1;
    else if (thingToCalibrate == CALIBRATE_MAG)
        shtpData[5] = 1;
    else if (thingToCalibrate == CALIBRATE_PLANAR_ACCEL)
        shtpData[7] = 1;
    else if (thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG)
    {
        shtpData[3] = 1;
        shtpData[4] = 1;
        shtpData[5] = 1;
    }
    else if (thingToCalibrate == CALIBRATE_STOP)
        ; //Do nothing, bytes are set to zero

    //Make the internal calStatus variable non-zero (operation failed) so that user can test while we wait
    calibrationStatus_BNO080 = 1;

    //Using this shtpData packet, send a command
    BNO080_sendCommand(COMMAND_ME_CALIBRATE);
}

//Request ME Calibration Status from BNO080
//See page 51 of reference manual
void BNO080_requestCalibrationStatus()
{
    /*shtpData[3] = 0; //P0 - Reserved
    shtpData[4] = 0; //P1 - Reserved
    shtpData[5] = 0; //P2 - Reserved
    shtpData[6] = 0; //P3 - 0x01 - Subcommand: Get ME Calibration
    shtpData[7] = 0; //P4 - Reserved
    shtpData[8] = 0; //P5 - Reserved
    shtpData[9] = 0; //P6 - Reserved
    shtpData[10] = 0; //P7 - Reserved
    shtpData[11] = 0; //P8 - Reserved*/

    for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
        shtpData[x] = 0;

    shtpData[6] = 0x01; //P3 - 0x01 - Subcommand: Get ME Calibration

    //Using this shtpData packet, send a command
    BNO080_sendCommand(COMMAND_ME_CALIBRATE);
}

//This tells the BNO080 to save the Dynamic Calibration Data (DCD) to flash
//See page 49 of reference manual and the 1000-4044 calibration doc
void BNO080_saveCalibration()
{
    /*shtpData[3] = 0; //P0 - Reserved
    shtpData[4] = 0; //P1 - Reserved
    shtpData[5] = 0; //P2 - Reserved
    shtpData[6] = 0; //P3 - Reserved
    shtpData[7] = 0; //P4 - Reserved
    shtpData[8] = 0; //P5 - Reserved
    shtpData[9] = 0; //P6 - Reserved
    shtpData[10] = 0; //P7 - Reserved
    shtpData[11] = 0; //P8 - Reserved*/

    for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
        shtpData[x] = 0;

    //Using this shtpData packet, send a command
    BNO080_sendCommand(COMMAND_DCD); //Save DCD command
}




//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
bool BNO080_receivePacket(void)
{
    if(!SensorI2C_select(SENSOR_I2C_0,BNO080_DEFAULT_ADDRESS))
    {
        SensorI2C_deselect();
        return false;
    }


    //Get the first four bytes, aka the packet header
//    uint8_t packetLSB =0;
//    uint8_t packetMSB =0;
//    uint8_t channelNumber =0;
//    uint8_t sequenceNumber =0;
    uint8_t dummy =0;
    SensorI2C_writeRead(&dummy,0,&shtpHeader[0], 4);


    //Calculate the number of data bytes in this packet
    int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
    dataLength &= ~(1 << 15); //Clear the MSbit.
    //This bit indicates if this package is a continuation of the last. Ignore it for now.
    //TODO catch this as an error and exit
    if (dataLength == 0)
    {
        //Packet is empty
        return (false); //All done
    }
    dataLength -= 4; //Remove the header bytes from the data count

    BNO080_getData(dataLength);

    SensorI2C_deselect();

    return (true); //We're done!
}

//Sends multiple requests to sensor until all data bytes are received from sensor
//The shtpData buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
//Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes
bool BNO080_getData(uint16_t bytesRemaining)
{
    uint16_t dataSpot = 0; //Start at the beginning of shtpData array

    //Setup a series of chunked 32 byte reads
    while (bytesRemaining > 0)
    {
        uint16_t numberOfBytesToRead = bytesRemaining;

        if (numberOfBytesToRead > (I2C_BUFFER_LENGTH - 4))
            numberOfBytesToRead = (I2C_BUFFER_LENGTH - 4);

        //The first four bytes are header bytes and are throw away
        uint8_t dummy =0;
        SensorI2C_writeRead(&dummy,0,  &shtpDataHelper[dataSpot], numberOfBytesToRead+4);
        memcpy(shtpData,&shtpDataHelper[4],numberOfBytesToRead);

        bytesRemaining -= numberOfBytesToRead;
    }
    return (true); //Done!
}

//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
//TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.
bool BNO080_sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
    uint8_t packetLength = dataLength + 4; //Add four bytes for the header

    if(!SensorI2C_select(SENSOR_I2C_0,BNO080_DEFAULT_ADDRESS))
    {
        SensorI2C_deselect();
        return false;
    }

    //Send the 4 byte packet header
    arrayHelper[0]=packetLength & 0xFF;
    arrayHelper[1]=packetLength >> 8;
    arrayHelper[2]=channelNumber;
    arrayHelper[3]=sequenceNumber[channelNumber]++;

    //Send the user's data packet
    for (uint8_t i = 4; i < dataLength +4; i++)
    {
        arrayHelper[i]= shtpData[i-4];
    }

    SensorI2C_write(&arrayHelper[0], dataLength+4);

    SensorI2C_deselect();


    return (true);
}

