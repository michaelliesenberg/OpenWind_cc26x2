/*
 * YIC71513PGMGG.c
 *
 *  Created on: 28 Jan 2023
 *      Author: michaelliesenberg
 */

#include "YIC71513PGMGG.h"

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#include "simple_peripheral_oad_onchip.h"

#include "nmea0183.h"

#include <string.h>
#include <xdc/std.h>
#include <stdbool.h>

#include <ti/sysbios/knl/Event.h>



UART2_Handle uart;
UART2_Params uartParams;

size_t bytesRead;
size_t byteAvailable=0;
char line[MINMEA_MAX_SENTENCE_LENGTH * 2];
char input;
bool isReading = false;
int16_t readReturn =0;
/*
 *  ======== callbackFxn ========
 */
void callbackFxn(UART2_Handle handle, void *buffer, size_t count, void *userArg, int_fast16_t status)
{
    if (status != UART2_STATUS_SUCCESS)
    {
        /* RX error occured in UART2_read() */
    }
    else {
        memcpy(line, buffer, count);
        bytesRead = count;
        OPENWIND_Event(OW_WIND_GPS_READ);
    }
    isReading=false;
}


void YIC71513PGMGG_uart_open() {


    GPIO_resetConfig(CONFIG_GPIO_UART2_0_TX);
    GPIO_resetConfig(CONFIG_GPIO_UART2_0_RX);

    UART2_Params_init(&uartParams);
    uartParams.baudRate     = 9600;
    uartParams.readReturnMode = UART2_ReadReturnMode_PARTIAL;//UART2_ReadReturnMode_FULL; UART2_ReadReturnMode_PARTIAL
    //uartParams.readMode = UART2_Mode_CALLBACK;
    //uartParams.readCallback = callbackFxn;


    uart = UART2_open(CONFIG_UART2_0, &uartParams);



    if (uart == NULL)
    {
       /* UART2_open() failed */
    }
    else
        UART2_rxEnable(uart);

}

void YIC71513PGMGG_uart_close() {

    //UART2_readCancel(uart);
    UART2_rxDisable(uart);
    UART2_close(uart);

    GPIO_resetConfig(CONFIG_GPIO_UART2_0_TX);
    GPIO_resetConfig(CONFIG_GPIO_UART2_0_RX);

    GPIO_setConfig(CONFIG_GPIO_UART2_0_TX, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_UART2_0_RX, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);

    GPIO_write(CONFIG_GPIO_UART2_0_TX, 0);
    GPIO_write(CONFIG_GPIO_UART2_0_RX, 0);

}

void YIC71513PGMGG_uart_read() {

    if(isReading)
        return;
    byteAvailable = UART2_getRxCount(uart);
    if(byteAvailable >= MINMEA_MAX_SENTENCE_LENGTH) {
        YIC71513PGMGG_read(MINMEA_MAX_SENTENCE_LENGTH);
        /*if(readReturn == UART2_STATUS_SUCCESS)
            OPENWIND_Event(OW_WIND_GPS_READ);
        else
            isReading=false;*/
    }

}

void YIC71513PGMGG_read(int lenght){
    isReading=true;
    //readReturn = UART2_read(uart, &input, lenght, &bytesRead);
    readReturn = UART2_readTimeout(uart, &input, lenght, &bytesRead, 100);
}

void YIC71513PGMGG_uart_parse() {
    isReading=false;
    switch (minmea_sentence_id(line, false)) {
        case MINMEA_SENTENCE_RMC: {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, line)) {
                /*printf(INDENT_SPACES "$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
                        frame.latitude.value, frame.latitude.scale,
                        frame.longitude.value, frame.longitude.scale,
                        frame.speed.value, frame.speed.scale);
                printf(INDENT_SPACES "$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
                        minmea_rescale(&frame.latitude, 1000),
                        minmea_rescale(&frame.longitude, 1000),
                        minmea_rescale(&frame.speed, 1000));
                printf(INDENT_SPACES "$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
                        minmea_tocoord(&frame.latitude),
                        minmea_tocoord(&frame.longitude),
                        minmea_tofloat(&frame.speed));*/
            }
            else {
                //printf(INDENT_SPACES "$xxRMC sentence is not parsed\n");
            }
        } break;

        case MINMEA_SENTENCE_GGA: {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, line)) {
                //printf(INDENT_SPACES "$xxGGA: fix quality: %d\n", frame.fix_quality);
            }
            else {
               // printf(INDENT_SPACES "$xxGGA sentence is not parsed\n");
            }
        } break;

        case MINMEA_SENTENCE_GST: {
            struct minmea_sentence_gst frame;
            if (minmea_parse_gst(&frame, line)) {
                /*printf(INDENT_SPACES "$xxGST: raw latitude,longitude and altitude error deviation: (%d/%d,%d/%d,%d/%d)\n",
                        frame.latitude_error_deviation.value, frame.latitude_error_deviation.scale,
                        frame.longitude_error_deviation.value, frame.longitude_error_deviation.scale,
                        frame.altitude_error_deviation.value, frame.altitude_error_deviation.scale);
                printf(INDENT_SPACES "$xxGST fixed point latitude,longitude and altitude error deviation"
                       " scaled to one decimal place: (%d,%d,%d)\n",
                        minmea_rescale(&frame.latitude_error_deviation, 10),
                        minmea_rescale(&frame.longitude_error_deviation, 10),
                        minmea_rescale(&frame.altitude_error_deviation, 10));
                printf(INDENT_SPACES "$xxGST floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)",
                        minmea_tofloat(&frame.latitude_error_deviation),
                        minmea_tofloat(&frame.longitude_error_deviation),
                        minmea_tofloat(&frame.altitude_error_deviation));*/
            }
            else {
               // printf(INDENT_SPACES "$xxGST sentence is not parsed\n");
            }
        } break;

        case MINMEA_SENTENCE_GSV: {
            struct minmea_sentence_gsv frame;
            if (minmea_parse_gsv(&frame, line)) {
             /*   printf(INDENT_SPACES "$xxGSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
                printf(INDENT_SPACES "$xxGSV: satellites in view: %d\n", frame.total_sats);
                for (int i = 0; i < 4; i++)
                    printf(INDENT_SPACES "$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
                        frame.sats[i].nr,
                        frame.sats[i].elevation,
                        frame.sats[i].azimuth,
                        frame.sats[i].snr);*/
            }
            else {
             //   printf(INDENT_SPACES "$xxGSV sentence is not parsed\n");
            }
        } break;

        case MINMEA_SENTENCE_VTG: {
           struct minmea_sentence_vtg frame;
           if (minmea_parse_vtg(&frame, line)) {
               /* printf(INDENT_SPACES "$xxVTG: true track degrees = %f\n",
                       minmea_tofloat(&frame.true_track_degrees));
                printf(INDENT_SPACES "        magnetic track degrees = %f\n",
                       minmea_tofloat(&frame.magnetic_track_degrees));
                printf(INDENT_SPACES "        speed knots = %f\n",
                        minmea_tofloat(&frame.speed_knots));
                printf(INDENT_SPACES "        speed kph = %f\n",
                        minmea_tofloat(&frame.speed_kph));*/
               OpenWind_SOG = (uint16_t)(minmea_tofloat(&frame.speed_knots) * 10);
               OpenWind_COG = (uint16_t)(minmea_tofloat(&frame.true_track_degrees));
           }
           else {
                //printf(INDENT_SPACES "$xxVTG sentence is not parsed\n");
           }
        } break;

        case MINMEA_SENTENCE_ZDA: {
            struct minmea_sentence_zda frame;
            if (minmea_parse_zda(&frame, line)) {
               /* printf(INDENT_SPACES "$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n",
                       frame.time.hours,
                       frame.time.minutes,
                       frame.time.seconds,
                       frame.date.day,
                       frame.date.month,
                       frame.date.year,
                       frame.hour_offset,
                       frame.minute_offset);*/
            }
            else {
               // printf(INDENT_SPACES "$xxZDA sentence is not parsed\n");
            }
        } break;

        case MINMEA_INVALID: {
            if(!isReading)
                UART2_flushRx(uart);
        } break;

        default: {

        } break;
    }
}
