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

static const uint8_t GPS_CONFIG[] = {
	    0xB5,0x62,0x06,0x8A,0x28,0x00,
	    0x00,0x01,0x00,0x00,
	    0x01,0x00,0x21,0x30,0xE8,0x03,
	    0xCA,0x00,0x91,0x20,0x00,
	    0xC0,0x00,0x91,0x20,0x00,
	    0xC5,0x00,0x91,0x20,0x00,
	    0xB1,0x00,0x91,0x20,0x00,
	    0x07,0x00,0x91,0x20,0x00,
		0xBB,0x00,0x91,0x20,0x00,
	    0xDE,0xA9
	};

//Hex message - B5 62 06 8A 28 00 00 01 00 00 01 00 21 30 E8 03 CA 00 91 20 00 C0 00 91 20 00 C5 00 91 20 00 B1 00 91 20 00 07 00 91 20 00 BB 00 91 20 00 DE A9


// Utility: convert NMEA lat/lon to decimal degrees
static double nmea_to_decimal(char *nmea_coord)
{
    if (!nmea_coord || strlen(nmea_coord) < 6) return 0;
    double raw = atof(nmea_coord);
    int deg = (int)(raw / 100);
    double min = raw - deg * 100;
    return deg + (min / 60.0);
}

static void GPS_ParseLine(char *line)
{
    if (line[0] != '$') {
        return;
    }

    char *stringp = line;
    // The strsep function will advance the stringp pointer past the delimiter
    char *type = strsep(&stringp, ",");
    if (!type) {
        return;
    }

    // --- GGA sentence handling ---
    if (strcasecmp(type, "$GNGGA") == 0 || strcasecmp(type, "$GPGGA") == 0) {
        char *time     = strsep(&stringp, ",");
        char *lat      = strsep(&stringp, ",");
        char *lat_dir  = strsep(&stringp, ",");
        char *lon      = strsep(&stringp, ",");
        char *lon_dir  = strsep(&stringp, ",");
        char *fix      = strsep(&stringp, ",");
        char *sats     = strsep(&stringp, ",");
        char *hdop     = strsep(&stringp, ",");
        char *alt      = strsep(&stringp, ",");

        // atoi and atof return 0 for an empty string, which is the desired behavior
        gps_data.satellites = sats ? (uint8_t)atoi(sats) : 0;
        gps_data.hdop       = hdop ? atof(hdop) : 0;

        if (time && time[0] != '\0') {
            strncpy(gps_data.time, time, sizeof(gps_data.time) - 1);
            gps_data.time[sizeof(gps_data.time) - 1] = '\0';
        } else {
            gps_data.time[0] = '\0';
        }

        // If fix quality is empty or "0", there is no valid fix
        if (!fix || fix[0] == '\0' || fix[0] == '0') {
            gps_data.fix = 0;
            gps_data.latitude  = 0;
            gps_data.longitude = 0;
            gps_data.altitude  = 0;
            return;
        }

        gps_data.fix = atoi(fix);
        gps_data.latitude  = nmea_to_decimal(lat) * ((lat_dir && lat_dir[0] == 'S') ? -1 : 1);
        gps_data.longitude = nmea_to_decimal(lon) * ((lon_dir && lon_dir[0] == 'W') ? -1 : 1);
        gps_data.altitude  = alt ? atof(alt) : 0;
    }
    // --- RMC sentence handling ---
    else if (strcasecmp(type, "$GNRMC") == 0 || strcasecmp(type, "$GPRMC") == 0) {
        char *time     = strsep(&stringp, ",");   // hhmmss.sss
        char *status   = strsep(&stringp, ",");   // A = valid, V = invalid
        char *lat      = strsep(&stringp, ",");
        char *lat_dir  = strsep(&stringp, ",");
        char *lon      = strsep(&stringp, ",");
        char *lon_dir  = strsep(&stringp, ",");
        (void)strsep(&stringp, ",");   			  // speed over ground not used
        (void)strsep(&stringp, ",");   			  // course over ground not used
        char *date     = strsep(&stringp, ",");   // ddmmyy

        // update time
        if (time && time[0] != '\0') {
            strncpy(gps_data.time, time, sizeof(gps_data.time) - 1);
            gps_data.time[sizeof(gps_data.time) - 1] = '\0';
        }
        // parse date (ddmmyy)
                if (date && strlen(date) == 6)
                {
                    gps_data.day   = (uint8_t)((date[0] - '0') * 10 + (date[1] - '0'));
                    gps_data.month = (uint8_t)((date[2] - '0') * 10 + (date[3] - '0'));
                    gps_data.year  = (uint8_t)((date[4] - '0') * 10 + (date[5] - '0'));
                }

        // If status is not 'A' (Active/valid), there is no fix
        if (!status || status[0] != 'A') {
            gps_data.fix = 0;
            gps_data.latitude  = 0;
            gps_data.longitude = 0;
            return;
        }

        gps_data.fix = 1; // RMC only tells us valid/invalid, not DGPS/RTK etc.

        gps_data.latitude  = nmea_to_decimal(lat) * ((lat_dir && lat_dir[0] == 'S') ? -1 : 1);
        gps_data.longitude = nmea_to_decimal(lon) * ((lon_dir && lon_dir[0] == 'W') ? -1 : 1);
    }
}

bool GPS_IsAlive()
{
	if(__HAL_DMA_GET_COUNTER(&hdma_uart4_rx) < GPS_DMA_RX_BUF_SIZE)
	{
		return true;
	}
	else
	{
		return false;
	}

}

// Initialize UART + DMA reception
void GPS_Init_DMA(void)
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
        static char sentence[256];
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

#ifdef GPS_DEBUG
        strcpy(sentence, "$GNRMC,154745.00,A,4912.46246,N,01635.99785,E,0.050,,230825,,,A,V*1E");
        printf("%03u   %s\r\n",last_read_ptr, sentence);

#endif
        GPS_ParseLine(sentence);
    }

    return gps_data;
}

// --- GPS configuration function ---
void GPS_Config(void)
{
    // Set update rate = 1 Hz
	HAL_UART_Transmit(GPS_UART, GPS_CONFIG, sizeof(GPS_CONFIG), HAL_MAX_DELAY);

}
