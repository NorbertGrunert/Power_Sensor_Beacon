/*
 * Tracker Firmware
 *
 * BLE driver header file
 *
 */
 
#ifndef BLE_H
#define BLE_H

#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "drv_uart.h"

/* Size of the GSM UART Rx ring buffer. */
#define	bleUART_RX_BUFFER_SIZE			uartRX_BUFFER_SIZE

/* Tag that refers to the BLE stack configuration. */
#define APP_BLE_CONN_CFG_TAG            1 		

/* Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_OBSERVER_PRIO           3       

/* Output power definitions. */
#define OUTPUT_PWR_0_dBm 				0 
#define OUTPUT_PWR_4_dBm 				4
#define OUTPUT_PWR_6_dBm 				6
#define OUTPUT_PWR_8_dBm 				8

/* Advertising state. */
#define ADV_NONE						0
#define ADV_STD1MBPS					1
#define ADV_LR125KBPS					4

/* Scan state. */
#define SCAN_NONE						0
#define SCAN_STD1MBPS					1
#define SCAN_LR125KBPS					4

/* Advertising interval (in units of 0.625 ms) */
#define ADV_INTERVAL 					160

/* Interval at which the advertising mode is toggled between long range and standard, if configured. */
#define ADV_INTERLEAVING_TIME			10 * portTICKS_PER_100MSEC

/* SCAN_INTERVAL - Scanning interval, determines scan interval in units of 0.625 millisecond. */
#define SCAN_INTERVAL 					320

/* SCAN_WINDOW - Scanning window, determines scan window in units of 0.625 millisecond. */
#define SCAN_WINDOW 					160

/* Temperature poll interval. */
#define TEMP_POLL_INTERVAL				20 * portTICKS_PER_SEC


/* Public function prototypes. */
extern void vBleInit( void );

/* Start advertising. */
extern void vStartAdvertising( portBASE_TYPE xNewAdvState );

/* Stop advertising. */
extern void vStopAdvertising( void );

/* Start scanning. */
extern void vStartScan( portBASE_TYPE xNewScanState );

/* Stop advertising. */
extern void vStopScan( void );

/* Return advertising state. */
extern portBASE_TYPE xGetAdvState( void );

/* Return scan state. */
extern portBASE_TYPE xGetScanState( void );
/*-----------------------------------------------------------*/

/* Global variables. */
/* BLE command queue handle. The queue is read in the BLE task and filled in the CTRL task. */
extern QueueHandle_t 		xBleCmdQueue; 

/* Advertising data. */
extern signed char			cEncodedStd1MbpsAdvData[ 100 ];
extern portBASE_TYPE		xEncodedStd1MbpsAdvDataLen;
extern signed char			cEncodedLR125kbpsAdvData[ 257 ];
extern portBASE_TYPE		xEncodedLR125kbpsAdvDataLen;

/* Configured output power in different modes. */
extern uint8_t				cTxPower1Mbps;
extern uint8_t				cTxPower125kbps;
extern uint32_t				ul1MbpsAdvInterval;
extern uint32_t				ul125kbpsAdvInterval;

#endif