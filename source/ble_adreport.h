/*
 * Tracker Firmware
 *
 * BLE driver header file
 *
 */
 
#ifndef BLE_ADREPORT_H
#define BLE_ADREPORT_H

#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"
/*-----------------------------------------------------------*/

/* Size of the command queue from the timer task to the advertising report task. */
#define		bleADRPT_QUEUE_SIZE				10

/* RSSI filter algorithm configuration values. */
#define RSSI_NONE						0
#define RSSI_AVG						1
#define RSSI_MAXAVG						2

/* Time interval at which the advertiser RSSI filter is called. */
#define FILTER_POLL_TIME				1 * portTICKS_PER_SEC

/* Time interval at which the advertiser RSSI filter is called. */
#define FILTER_INTERVAL					10 * portTICKS_PER_SEC

/* Commands from the periodic timer task to the advertiser handler task. */
enum xADREPORT_CMD
{
	AD_REPORT_CHK							/* Check if advertiser are due for being reported. */
};
/*-----------------------------------------------------------*/

/* Public function prototypes. */
extern void vBleAdReportInit( void );

/* Send a reported advertiser to the host via UART. */
extern void vSendAdvertiserData( uint8_t ucAddrType, uint8_t *pucPeerAddr, uint16_t usDataLen, 
								 uint8_t *pucData, unsigned char ucPrimaryPhy, int8_t cRssiValue );
/*-----------------------------------------------------------*/

/* Global variables. */
/* Timer handle for the advertisement RSSI filter timer. */
extern TimerHandle_t 				xBleFilterTimer;

/* Configuration for the RSSI filter. */
extern unsigned portBASE_TYPE		uxRssiFilterMethod;
extern unsigned portBASE_TYPE		uxRssiFilterWindow;
/*-----------------------------------------------------------*/


#endif