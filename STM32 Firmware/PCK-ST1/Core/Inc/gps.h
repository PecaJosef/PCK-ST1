/*
 * gps.h
 *
 *  Created on: Aug 15, 2025
 *      Author: pecka
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"

typedef struct {
    double latitude;       // decimal degrees
    double longitude;      // decimal degrees
    float altitude;        // meters
    uint8_t satellites;    // number of satellites in use
    float hdop;            // horizontal dilution of precision
    uint8_t fix;           // 0 = no fix, 1 = GPS fix, 2 = DGPS fix
    char time[16];         // UTC time hhmmss.sss
} GPS_Data_t;

#define GPS_UART &huart4 //GPS UART Handler
#define GPS_DMA_RX_BUF_SIZE 512

void GPS_Init(void);
GPS_Data_t Get_GPS_Data(void);

#endif /* INC_GPS_H_ */
