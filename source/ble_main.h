/*
 * Tracker Firmware
 *
 * BLE driver header file
 *
 */
 
#ifndef BLE_MAIN_H
#define BLE_MAIN_H

#include <stdbool.h>

#include "tracker.h"
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
			xComSendString( COM_TRC, cErrorStrg );								\
			vTaskDelay( 3 * portTICKS_PER_SEC );								\
            APP_ERROR_HANDLER( LOCAL_ERR_CODE );								\
        }																		\
    } while (0)
/*-----------------------------------------------------------*/

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

/* SR / LR definition. */
enum xIF_TYPE
{
	SR,
	LR
};

enum xBLE_BCN_FORMAT
{
	BLE_BEACON_NONE,
	BLE_IBEACON,
	BLE_ALTBEACON,
	BLE_SWISSPHONE_BEACON,
	BLE_DISTRESS_BEACON
};

/* Structure to store advertising reports for known devices. */
struct KNOWN_DEVICE
{
	bool						bUsed;								/* The entry in the known device list is used. */
	enum xBLE_BCN_FORMAT		xBcnFormat;							/* Type of the beacon. */
	unsigned char 				ucUuidFilterMatchIdx;				/* UUID match index (valid only for SwissPhone beacons). */
	uint8_t						pucPeerAddr[ BLE_GAP_ADDR_LEN ];
	uint16_t					usDataLen;
	uint8_t						pucAdvData[ 256 ];
	unsigned char				ucPrimaryPhy;	
	TickType_t					xRxTimeStamp;
	int8_t						xRSSI[ RSSI_LIST_LEN ];
};
/*-----------------------------------------------------------*/

/* Public function prototypes. */
extern void vBleMainInit( void );

/* Check if the UUID filter is enabled and fill in the bBleUuidFilterEnable variable. */
extern void bCheckAndSetUUIDFilterEnabled( void );

/* Start advertising. */
extern void vStartAdvertising( portBASE_TYPE xNewAdvState );

/* Stop advertising. */
extern void vStopAdvertising( void );

/* Start scanning. */
extern void vStartScan( portBASE_TYPE xNewScanState );

/* Stop scanning. */
extern void vStopScan( void );

/* Set RSSI filter algorithm configuration. */
extern void vSetRssiCfg( unsigned portBASE_TYPE uxFilterMethod, unsigned portBASE_TYPE uxFilterWindow );

/* Set TX power. */
extern void vSetTxPower( enum xIF_TYPE xRfInterface, unsigned portBASE_TYPE uxTxPower );

/* Set the advertising interval. */
extern void vSetAdvInterval( enum xIF_TYPE xRfInterface, unsigned long ulAdvInterval );

/* Set the advertising payload: Initialise. 
   Fill the first three bytes of the payload with the AD flag. */
extern void vSetAdvPayloadInit( enum xIF_TYPE xRfInterface );

/* Continue the advertising payload. */
extern void vSetAdvPayload( enum xIF_TYPE xRfInterface, unsigned char *pucAdvPayload, unsigned portBASE_TYPE uxAdvPayloadLen );

/* Return the BLE cumulative on-duration. */
extern TickType_t xGetBleOnDuration( void );

/* Return the BLE functional state. */
extern void vResetBleOnDuration( TickType_t xResetTargetValue );
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

/* List of known devices. */
extern struct KNOWN_DEVICE	xKnownDeviceList[ KNOWN_DEVICE_LIST_LEN ];


#endif