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

#include "ble_gap.h"
#include "ble_advdata.h"

/* Macros */

#define APP_ERROR_CHECK_ATIF( ERR_STRG, ERR_CODE )								\
    do																			\
    {																			\
        const uint32_t LOCAL_ERR_CODE = ( ERR_CODE );							\
        if ( LOCAL_ERR_CODE != NRF_SUCCESS )									\
        {																		\
			sprintf( cErrorStrg, ERR_STRG, xErrCode );							\
			xComSendStringRAM( COM0, cErrorStrg );								\
			vTaskDelay( 3 * portTICKS_PER_SEC );								\
            APP_ERROR_HANDLER( LOCAL_ERR_CODE );								\
        }																		\
    } while (0)
/*-----------------------------------------------------------*/

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
#define SCAN_INTERVAL 					280

/* SCAN_WINDOW - Scanning window, determines scan window in units of 0.625 millisecond. */
#define SCAN_WINDOW_DUAL				140
#define SCAN_WINDOW_MONO				280

/* Temperature poll interval. */
#define TEMP_POLL_INTERVAL				20 * portTICKS_PER_SEC

/* Length of the list of known devices. */
#define KNOWN_DEVICE_LIST_LEN			50

/* Length of the list of RSSI per known device. The list has to be sufficiently large to
   store all RX event RSSIs during once FILTER_INTERVAL (e.g.: FILTER_INTERVAL=10s, devices set
   to 1 report/s --> RSSI_LIST_LEN = 10). */
#define RSSI_LIST_LEN					12

#define	BLE_OS_TIMEOUT					( 10 * configTICK_RATE_HZ )

/* Structure to store advertising reports for known devices. */
struct KNOWN_DEVICE
{
	bool						bUsed;								/* The entry in the known device list is used. */
	uint8_t						ucAddrType;							/* Type of the advertiser address:
																			BLE_GAP_ADDR_TYPE_PUBLIC   						0x00
																			BLE_GAP_ADDR_TYPE_RANDOM_STATIC   				0x01 
																			BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE 	0x02
																			BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE 0x03
																			BLE_GAP_ADDR_TYPE_ANONYMOUS 					0x7F		*/
	uint8_t						pucPeerAddr[ BLE_GAP_ADDR_LEN ];
	uint16_t					usDataLen;
	uint8_t						pucData[ 256 ];
	unsigned char				ucPrimaryPhy;	
	TickType_t					xRxTimeStamp;
	int8_t						xRSSI[ RSSI_LIST_LEN ];
};
/*-----------------------------------------------------------*/

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

/* Mutex handle to protect access to BLE configuration. */
extern SemaphoreHandle_t	xMutexBleConfig;

/* Mutex to protect access to the BLE advertiser data in xKnownDeviceList[]. */
extern SemaphoreHandle_t	xMutexBleAdvData;

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

/* List of known devices. */
extern struct KNOWN_DEVICE	xKnownDeviceList[ KNOWN_DEVICE_LIST_LEN ];

#endif