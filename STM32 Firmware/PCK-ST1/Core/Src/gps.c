/*
 * gps.c
 *
 *  Created on: Aug 15, 2025
 *      Author: pecka
 */

#include "gps.h"

extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;  // Your DMA handle

static uint8_t dma_rx_buf[GPS_DMA_RX_BUF_SIZE];
static GPS_Data_t gps_data;

// Utility: convert NMEA lat/lon to decimal degrees
static double nmea_to_decimal(char *nmea_coord)
{
    if (!nmea_coord || strlen(nmea_coord) < 6) return 0;
    double raw = atof(nmea_coord);
    int deg = (int)(raw / 100);
    double min = raw - deg * 100;
    return deg + (min / 60.0);
}

// Parse a GGA sentence into gps_data
static void GPS_ParseLine(char *line)
{
    if (line[0] != '$') return;

    char *type = strtok(line, ",");
    if (!type) return;

    if (strcasecmp(type, "$GNGGA") != 0 && strcasecmp(type, "$GPGGA") != 0) return;

    char *time     = strtok(NULL, ",");
    char *lat      = strtok(NULL, ",");
    char *lat_dir  = strtok(NULL, ",");
    char *lon      = strtok(NULL, ",");
    char *lon_dir  = strtok(NULL, ",");
    char *fix      = strtok(NULL, ",");
    char *sats     = strtok(NULL, ",");
    char *hdop     = strtok(NULL, ",");
    char *alt      = strtok(NULL, ",");


   //Always get number of satellites and HDOP
    gps_data.satellites = sats ? (uint8_t)atoi(sats) : 0;
	gps_data.hdop       = hdop ? atof(hdop) : 0;

	//Get UTC time if available
	if (time)
	{
		strncpy(gps_data.time, time, sizeof(gps_data.time) - 1);
		gps_data.time[sizeof(gps_data.time)-1] = '\0';
	}
	else
	{
		gps_data.time[0] = '\0';
	}

	if (!fix || strcmp(fix, "0") == 0) {
		gps_data.fix = 0;
		gps_data.latitude  = 0;
		gps_data.longitude = 0;
		gps_data.altitude  = 0;
		return;
	}

	gps_data.fix = atoi(fix);
	gps_data.latitude  = nmea_to_decimal(lat)  * ((lat_dir && lat_dir[0]=='S') ? -1 : 1);
	gps_data.longitude = nmea_to_decimal(lon)  * ((lon_dir && lon_dir[0]=='W') ? -1 : 1);
	gps_data.altitude  = alt ? atof(alt) : 0;
}

// Initialize UART + DMA reception
void GPS_Init(void)
{
    HAL_UART_Receive_DMA(GPS_UART, dma_rx_buf, GPS_DMA_RX_BUF_SIZE);
}

// Read latest complete NMEA sentence from DMA buffer safely
GPS_Data_t Get_GPS_Data(void)
{
    static uint16_t last_read_ptr = 0;
    uint16_t dma_write_ptr = GPS_DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);

    int start_idx = -1, end_idx = -1;
    uint16_t i = last_read_ptr;
    uint16_t count = 0;

    //printf("DMA write ptr: %d\n", dma_write_ptr);

    // Safety: limit max iterations to buffer size
    while (i != dma_write_ptr && count < GPS_DMA_RX_BUF_SIZE)
    {
    	//printf("loop running\n");
    	if (dma_rx_buf[i] == '$') start_idx = i;
        if ((dma_rx_buf[i] == '\n' || dma_rx_buf[i] == '\r') && start_idx >= 0)
        {
            end_idx = i;
        }
        i = (i + 1) % GPS_DMA_RX_BUF_SIZE;
        count++;
    }

    if (start_idx >= 0 && end_idx >= 0)
    {
        static char sentence[512];
        int len;

        if (end_idx >= start_idx)
            len = end_idx - start_idx + 1;
        else
            len = GPS_DMA_RX_BUF_SIZE - start_idx + end_idx + 1;

        // Copy safely from circular buffer
        for (int j = 0; j < len; j++)
            sentence[j] = dma_rx_buf[(start_idx + j) % GPS_DMA_RX_BUF_SIZE];
        sentence[len] = '\0';

        last_read_ptr = (end_idx + 1) % GPS_DMA_RX_BUF_SIZE;

        printf("Sentence: %s\n", sentence);

        GPS_ParseLine(sentence);
    }

    return gps_data;
}
