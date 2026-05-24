/*
 * gps.c
 *
 *  Created on: Aug 15, 2025
 *      Author: pecka
 */

#include "gps.h"
#include "cmd_handler.h"

extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;

static uint8_t dma_rx_buf[GPS_DMA_RX_BUF_SIZE];
static GPS_Data_t gpsData;
static uint16_t last_read_ptr = 0;

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


//convert NMEA lat/lon to decimal degrees
static double nmeaToDecimal(char *nmea_coord)
{
    if (!nmea_coord || strlen(nmea_coord) < 6) return 0;
    double raw = atof(nmea_coord);
    int deg = (int)(raw / 100);
    double min = raw - deg * 100;
    return deg + (min / 60.0);
}

static void gpsParseLine(char *line)
{
    if (line[0] != '$')
    {
        return;
    }

    char *stringp = line;
    char *type = strsep(&stringp, ",");
    if (!type) {
        return;
    }

    //RMC sentence parsing
    else if (strcasecmp(type, "$GNRMC") == 0 || strcasecmp(type, "$GPRMC") == 0) {
        char *time     = strsep(&stringp, ",");   //hhmmss.sss
        char *status   = strsep(&stringp, ",");   //A = valid, V = invalid
        char *lat      = strsep(&stringp, ",");
        char *lat_dir  = strsep(&stringp, ",");
        char *lon      = strsep(&stringp, ",");
        char *lon_dir  = strsep(&stringp, ",");
        (void)strsep(&stringp, ",");   			  //speed over ground not used
        (void)strsep(&stringp, ",");   			  //course over ground not used
        char *date     = strsep(&stringp, ",");   //ddmmyy

        //update time
        if (time && time[0] != '\0') {
            strncpy(gpsData.time, time, sizeof(gpsData.time) - 1);
            gpsData.time[sizeof(gpsData.time) - 1] = '\0';
        }
        //parse date (ddmmyy)
		if (date && strlen(date) == 6)
		{
			gpsData.day   = (uint8_t)((date[0] - '0') * 10 + (date[1] - '0'));
			gpsData.month = (uint8_t)((date[2] - '0') * 10 + (date[3] - '0'));
			gpsData.year  = (uint8_t)((date[4] - '0') * 10 + (date[5] - '0'));
		}

        //If status is not 'A' (Active/valid), there is no fix and the coordinates are left as 0, 0
        if (!status || status[0] != 'A') {
            gpsData.fix = 0;
            gpsData.latitude  = 0;
            gpsData.longitude = 0;
            return;
        }

        gpsData.fix = 1;

        gpsData.latitude  = nmeaToDecimal(lat) * ((lat_dir && lat_dir[0] == 'S') ? -1 : 1);
        gpsData.longitude = nmeaToDecimal(lon) * ((lon_dir && lon_dir[0] == 'W') ? -1 : 1);
    }
}

bool gpsIsAlive()
{
	if(__HAL_DMA_GET_COUNTER(&hdma_uart4_rx) < GPS_DMA_RX_BUF_SIZE)
	{
		uartSend(PC_UART_SRC, "GPS_OK\r\n");
		return true;
	}
	else
	{
		uartSend(PC_UART_SRC, "GPS_nOK\r\n");
		return false;
	}
}

//Initialize UART with DMA
void GPS_Init_DMA(void)
{
    HAL_UART_Receive_DMA(GPS_UART, dma_rx_buf, GPS_DMA_RX_BUF_SIZE);
}

//Read latest complete NMEA sentence from the DMA buffer
GPS_Data_t getGPSData(void)
{
    uint16_t dma_write_ptr = GPS_DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);

    int start_idx = -1, end_idx = -1;
    uint16_t i = last_read_ptr;
    uint16_t count = 0;

    while (i != dma_write_ptr && count < GPS_DMA_RX_BUF_SIZE)
    {
    	if (dma_rx_buf[i] == '$')
		{
			start_idx = i;
		}

        if ((dma_rx_buf[i] == '\n' || dma_rx_buf[i] == '\r') && start_idx >= 0)
        {
            end_idx = i;
            break;
        }
        i = (i + 1) % GPS_DMA_RX_BUF_SIZE;
        count++;
    }

    if (start_idx != -1 && end_idx != -1) //Org - start_idx >= 0 && end_idx >= 0
    {
        static char sentence[128];
        int len;

        if (end_idx >= start_idx)
            len = end_idx - start_idx + 1;
        else
            len = GPS_DMA_RX_BUF_SIZE - start_idx + end_idx + 1;
        //Safety len cap
        if (len >= sizeof(sentence))
        {
        	len = sizeof(sentence)-1;
        }
        //Copy the sentence from the buffer
        for (int j = 0; j < len; j++)
        {
        	sentence[j] = dma_rx_buf[(start_idx + j) % GPS_DMA_RX_BUF_SIZE];
        }
        sentence[len] = '\0';

        last_read_ptr = (end_idx + 1) % GPS_DMA_RX_BUF_SIZE;

        //For debug
        /*
        uartSend(PC_UART_SRC, sentence);
        uartSend(PC_UART_SRC, "\r\n");
        */

        //Fake gps data for testing
        #ifdef GPS_DEBUG
        uartSend(PC_UART_SRC, "GPS_HERE\r\n");
        strcpy(sentence, "$GNRMC,154745.00,A,4912.46246,N,01635.99785,E,0.050,,180226,,,A,V*1E");
        printf("%03u   %s\r\n",last_read_ptr, sentence);
        uartSend(PC_UART_SRC, sentence);
        uartSend(PC_UART_SRC, "\r\n");
		#endif

        gpsParseLine(sentence);
    }

    return gpsData;
}

//GPS config
void gpsConfig(void)
{
    //set update rate to 1Hz and sentence output to RMC
	HAL_UART_Transmit(GPS_UART, GPS_CONFIG, sizeof(GPS_CONFIG), HAL_MAX_DELAY);
	gpsFlushBuffer();
}

void gpsFlushBuffer()
{
	last_read_ptr = GPS_DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
}

GPS_Ack_t gpsCheckAck(void)
{
    uint16_t dma_write_ptr = GPS_DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
    uint16_t i = last_read_ptr;
    uint16_t count = 0;

    while (i != dma_write_ptr && count < GPS_DMA_RX_BUF_SIZE)
    {
        //look for UBX header 0xB5 0x62
        if (dma_rx_buf[i] == 0xB5 && dma_rx_buf[(i + 1) % GPS_DMA_RX_BUF_SIZE] == 0x62)
        {
            uint8_t cls = dma_rx_buf[(i + 2) % GPS_DMA_RX_BUF_SIZE];
            uint8_t id  = dma_rx_buf[(i + 3) % GPS_DMA_RX_BUF_SIZE];

            if (cls == 0x05 && id == 0x01)
            {
            	gpsFlushBuffer();
                return GPS_ACK_OK;
            }
            if (cls == 0x05 && id == 0x00)
            {
            	gpsFlushBuffer();
                return GPS_ACK_NAK;
            }
        }
        i = (i + 1) % GPS_DMA_RX_BUF_SIZE;
        count++;
    }
    return GPS_ACK_PENDING;
}




