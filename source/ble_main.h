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

/* Size of the GSM UART Rx ring buffer. */
#define	bleUART_RX_BUFFER_SIZE	( 150 )

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
#define ADV_LR125KBPS					2

/* Advertising interval (in units of 0.625 ms) */
#define ADV_INTERVAL 					300

/* SCAN_INTERVAL - Scanning interval, determines scan interval in units of 0.625 millisecond. */
#define SCAN_INTERVAL 					160

/* SCAN_WINDOW - Scanning window, determines scan window in units of 0.625 millisecond. */
#define SCAN_WINDOW 					160

#define PHY_1M_PHY						0
#define PHY_CODED						1


/* Public function prototypes. */
extern void vBleInit( void );

/* Start advertising. */
extern void vStartAdvertising( portBASE_TYPE xNewAdvState );

/* Stop advertising. */
extern void vStopAdvertising( void );

/* Return advertising state. */
extern portBASE_TYPE xGetAdvState( void );
/*-----------------------------------------------------------*/

/* Global variables. */
/* BLE command queue handle. The queue is read in the BLE task and filled in the CTRL task. */
extern QueueHandle_t 		xBleCmdQueue; 

/* Advertising data. */
extern signed char			cEncodedAdvData[];
extern portBASE_TYPE		xEncodedAdvDataLen;

/* Configured output power in different modes. */
extern uint8_t				cTxPower1Mbps;
extern uint8_t				cTxPower125kbps;

#endif