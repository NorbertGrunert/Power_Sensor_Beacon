/*
 * Tracker Firmware
 *
 * BLE task
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "tracker.h"

/* nRF SDK include files */
#include "nordic_common.h"
#include "nrf.h"
#include "ble.h"
#include "ble_gatt.h"
#include "ble_hci.h"
#include "ble_gap.h"
#include "ble_advdata.h"
#include "ble_srv_common.h"
#include "nrf_drv_rng.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "nrf_gpio.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_temp.h"

#include "bsp_btn_ble.h"

#include "app_timer.h"
#include "app_error.h"

#include "custom_board.h"

#define NRF_LOG_MODULE_NAME 		BLE_AD
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_log_backend_flash.h"
#include "nrf_fstorage_nvmc.h"


/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"


/* Device specific include files. */
#include "drv_accelerometer.h"
#include "drv_aes.h"
#include "drv_nvm.h"
#include "drv_uart.h"
#include "drv_vibrator.h"

#include "ble_ctrl.h"
#include "ble_cmd.h"
#include "ble_adreport.h"
#include "ble_main.h"
#include "config.h"
#include "ctrl.h"
#include "evacuation.h"
#include "gsm.h"
#include "main.h"
#include "parser.h"
#include "rtc.h"
#include "trace.h"
#include "tracemsg.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Initialise the BLE task. */
void vBleAdReportInit( UBaseType_t uxPriority );

/* Dummy timer callback - does nothing but satisfy the OS requirements. */
static void prvDummyTimerCallback( TimerHandle_t xTimer );

/* In danger zone timer callback. */
static void prvInDangerZoneTimerCallback( TimerHandle_t xTimer );

/* Filter the list of RSSI values. */
int8_t xFilterRSSI( int8_t *rssi_list );

/* Timer callback for advertisement RSSI filter timer. */
void prvBleFilterTimerCallback( TimerHandle_t xTimer );

/* Look at the advertiser address of received packets to decide if the payload needs to be parsed. */
void vParseBTD( struct KNOWN_DEVICE *pxBcnData, int8_t cRssiValue );
						
/* Build the table of found BLE beacons from the +BTD responses. */
static void vParseLocBeacon( struct KNOWN_DEVICE *pxBcnData, int8_t cRssiValue );

/* Find the next free entry in the beacon table. If there is non, locate the oldest entry and replace it. */
static signed portBASE_TYPE xGetBeaconTableIndex( volatile struct xBLE_LOC_BEACON *xBeaconTable, unsigned char *pucBeaconAddr, 
     												 int8_t cRssiValue, signed char cRefRSSI, bool bIsSwissPhoneBeacon );

/* Enter the beacon data at the next free entry in the beacon table. If there is none, locate the oldest entry and replace it. */
static void vEnterBeaconInBeaconTable( volatile struct xBLE_LOC_BEACON *pxBeaconTable, struct KNOWN_DEVICE *pxBcnData, unsigned char *pucBeaconAddr, 
									   int8_t cRssiValue );

/* Get the next decyrpted payload byte. */
static unsigned char ucGetNextDecryptedByte( unsigned char **ppusEncrAdvData, unsigned portBASE_TYPE  *puxPayloadBytesAvailable, 
											 unsigned char *pucPlainTextBlock );

/* Calculate the length of the variable TL beacon field */
unsigned portBASE_TYPE uxCalculateVarBcnFieldLen( unsigned long ulDFMap, unsigned portBASE_TYPE uxDFMapFieldIdx,
										          unsigned portBASE_TYPE *puxNumFields );

/* Extract the FDFMAP and parse the LR beacon payload. Decrypt the payload on-the-fly. */
static bool bCopyLRBeaconData( unsigned char *pucAdvData, unsigned char ucPlainTextBlock[], unsigned portBASE_TYPE uxTblIdx, 
							   unsigned char *pucChkSum, bool bStoreData );
							   
/* Extract alert type and foreign tracker ID from received distress beacons and copy the information to the GSM data fields. */
static void vParseDistressBeacon( struct KNOWN_DEVICE *pxBcnData, int8_t cRssiValue );

/* Store the contents of the BLE location beacon dictionary in the location beacon record array. */
void vStoreBLEAdvertisers( void );

/* The BLE advertising report handling task. */
static portTASK_FUNCTION_PROTO( vBleAdHandlerTask, pvParameters );
/*-----------------------------------------------------------*/

/* Local variables. */

/* Broadcast address. */
static const char 					pcRlyBroadcastAddr[] = "CAFECAFECAFE000";

/* Configuration for the RSSI filter. */
unsigned portBASE_TYPE				uxRssiFilterMethod = RSSI_MAXAVG;
unsigned portBASE_TYPE				uxRssiFilterWindow = 10;

/* Advertiser handler command queue handle. */
QueueHandle_t 						xBleAdHandlerQueue; 
/*-----------------------------------------------------------*/

/* Public variables. */

/* Timer handle for the advertisement RSSI filter timer. */
TimerHandle_t 						xBleFilterTimer;

/* Timer handle for the timer indicating when running that the module is in a danger zone. */
TimerHandle_t 						xInDangerZoneTimer;

/* Timer handle for the timer indicating when running that the module had received a foreign TL beacon with any alert indication set. */
TimerHandle_t 						xForeignAlertTimer;

/* Foreign alert type. */	
volatile unsigned portBASE_TYPE		uxForeignAlertType;
	
/* Table of location beacons. */	
volatile struct xBLE_LOC_BEACON		xBleLocBcnGSM;
volatile struct xBLE_LOC_BEACON		xBleLocBcnBLE;
	
/* Table of TL beacons. */	
volatile struct xBLE_TL_BEACON		xBleTLBcn[ MAX_TL_BCN_ENTRIES ];
volatile unsigned portBASE_TYPE		uxBleTLBcnCnt;

/* Flag, set when XSWITCH has been received. */
volatile bool 						bBleXSwitchObserved;

/* Time stamp of the last time reaction to a special beacon. */
unsigned short						usBcnNoAbnormalTimeStamp;
unsigned short						usBcnImmobilityTimeStamp;
unsigned short						usBcnPrivateZoneTimeStamp;

/* Flags indicating the received special purpose beacons. */
volatile struct xSPECIAL_BCN_RCVD	xSpecialBcnRcvd;

/* ROM-table with TL beacon data fields lengths. */
const unsigned portBASE_TYPE 		uxTLBcnDataFieldsLen[] = 
{
	BLE_BCN_TXTIM_LEN,	
	BLE_BCN_PWR_LEN,		
	BLE_BCN_EVAC_ID_LEN,
	BLE_BCN_BAT_LEN,		
	BLE_BCN_BCN_LEN,		
	BLE_BCN_BCN_LEN,		
	BLE_BCN_BCN_LEN,		
	BLE_BCN_LAT_LEN,		
	BLE_BCN_LON_LEN,		
	BLE_BCN_FIX_LEN,		
	BLE_BCN_LOCM_LEN,	
	BLE_BCN_HDOP_LEN,
	BLE_BCN_ACCH_LEN,	
	BLE_BCN_SV_LEN,		
	BLE_BCN_TEMP_LEN,	
	BLE_BCN_STATE_LEN,	
	BLE_BCN_DGSM_LEN,	
	BLE_BCN_GSMRSSI_LEN,
	BLE_BCN_GSMQUAL_LEN
};

/* Storage for BLE location beacons if recording is required. */ 
struct xBLE_LOC_BEACON 				xBleLocBeaconStore[ POS_STORE_LEN ];
/*-----------------------------------------------------------*/

/* 
 * Initialise the BLE AD report module. 
 */
void vBleAdReportInit( UBaseType_t uxPriority )
{
 	/* Initialise reception time stamps for special beacons. */
 	usBcnNoAbnormalTimeStamp = 0;
	usBcnImmobilityTimeStamp = 0;
 	usBcnPrivateZoneTimeStamp = 0;	
	
	/* Create advertisement timer. */
	/* Create advertisement RSSI filter timer. */
	xBleFilterTimer = xTimerCreate
							( "BLE_AD",			 			/* Timer name for debug. */
							  FILTER_POLL_TIME,				/* Interval at which the BLE RSSI filter is called. */
							  pdTRUE,						/* Auto-reload. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvBleFilterTimerCallback		/* Callback for the advertisement RSSI filter timer. */
							);
							
	/* Create timer which, when running, indicates that the module is a danger zone. */
	xInDangerZoneTimer =  xTimerCreate
							( "",		 							/* Timer name for debug. */
							  IN_DANGER_ZONE_DURATION, 				/* Timer period in ticks - dummy, will be overwritten later. */
							  pdFALSE,								/* Auto-reload. */
							  ( void * )0,							/* Init for the ID / expiry count. */
							  prvInDangerZoneTimerCallback			/* Timer callback. */
							);
							
	/* Timer handle for the timer indicating when running that the module had received a foreign TL beacon with any indication set. */
	xForeignAlertTimer =  xTimerCreate
							( "",		 							/* Timer name for debug. */
							  IN_DANGER_ZONE_DURATION, 				/* Timer period in ticks - dummy, will be overwritten later. */
							  pdFALSE,								/* Auto-reload. */
							  ( void * )0,							/* Init for the ID / expiry count. */
							  prvDummyTimerCallback					/* Dummy callback. */
							);
	
	/* Initialise flag for observed XSWITCHes*/
	bBleXSwitchObserved = false;

	/* Create a queue from the timer task to the BLE_AD task for to trigger checks for advertising reports. */
	xBleAdHandlerQueue = xQueueCreate( bleADRPT_QUEUE_SIZE, ( unsigned portBASE_TYPE ) sizeof( enum xADREPORT_CMD ) );	

	/* Create the task to filter beacon RSSIs and send the result to the host via UART. */
	xTaskCreate( vBleAdHandlerTask, "BLE_AD", bleAdHandlerSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
	
	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* Dummy timer callback - does nothing but satisfy the OS requirements. */
static void prvDummyTimerCallback( TimerHandle_t xTimer )
{
	/* Calm down the compiler. */
	( void )xTimer;
}
/*-----------------------------------------------------------*/

/* In danger zone timer callback. */
static void prvInDangerZoneTimerCallback( TimerHandle_t xTimer )
{
	/* Calm down the compiler. */
	( void )xTimer;
	
	/* Remove the 'in danger zone' flag. */
	xBleModuleState.bBleInDangerZoneRequest = false;
}
/*-----------------------------------------------------------*/

/* Filter the list of RSSI values. */
int8_t xFilterRSSI( int8_t *pxRSSIList )
{
	unsigned portBASE_TYPE		uxListIdx;
    int8_t                     	xRSSI;
	double						dAvgRSSI;
	signed int					iMaxRSSI;
	unsigned int				uiRSSICnt;
    
	switch ( uxRssiFilterMethod )
	{
		case RSSI_AVG:		/* Build a simple average over all captured RSSI values. */
							uiRSSICnt = 0;
                            dAvgRSSI = 0;
							for ( uxListIdx = 0; uxListIdx < RSSI_LIST_LEN; uxListIdx++ )
							{
								if ( *( pxRSSIList + uxListIdx ) != 0 )
								{
									uiRSSICnt++;
									dAvgRSSI += pow( 10.0, ( double )*( pxRSSIList + uxListIdx ) / 10.0 );
								}
							}

							/* Last entry. Return the average RSSI. */
							dAvgRSSI /= ( double )uiRSSICnt;
							xRSSI = ( int8_t )( 10.0 * log10( dAvgRSSI )  - 0.5 );
							return xRSSI;

							break;
		
		case RSSI_MAXAVG:	/* Build the average of all RSSI values which are not smaller than 5dB below the maximum value. */
							iMaxRSSI = -128;
							for ( uxListIdx = 0; uxListIdx < RSSI_LIST_LEN; uxListIdx++ )
							{
								if ( *( pxRSSIList + uxListIdx ) != 0 )
								{
									if ( iMaxRSSI < *( pxRSSIList + uxListIdx ) ) 
									{
										iMaxRSSI = *( pxRSSIList + uxListIdx );
									}
								}
							}
							
							/* Now average all values which are higher that the maximum RSSI - 5dB. */
							iMaxRSSI -= 5;
							uiRSSICnt = 0;
							dAvgRSSI = 0;
							for ( uxListIdx = 0; uxListIdx < RSSI_LIST_LEN; uxListIdx++ )
							{
								if ( *( pxRSSIList + uxListIdx ) != 0 )
								{
									if ( *( pxRSSIList + uxListIdx ) > iMaxRSSI )
									{
										uiRSSICnt++;
										dAvgRSSI += pow( 10.0, ( double )*( pxRSSIList + uxListIdx ) / 10.0 );
									}
								}
							}
							
							/* Last entry. Return the average RSSI. The -0.5 converts truncating to rounding. */
							dAvgRSSI /= ( double )uiRSSICnt;
							xRSSI = ( int8_t )( 10.0 * log10( dAvgRSSI )  - 0.5 );
							return xRSSI;
							
							break;
		
		
		default:			/* No filtering. return the newest RSSI value. */
							xRSSI = *pxRSSIList;
							for ( uxListIdx = 0; uxListIdx < RSSI_LIST_LEN; uxListIdx++ )
							{
								if ( *( pxRSSIList + uxListIdx ) != 0 )
								{
									xRSSI = *( pxRSSIList + uxListIdx );
								}
							}
							
							/* Last entry. Return the RSSI found so far. */
							return xRSSI;

							break;
	}

	/* Dummy return. */
	return 0;			
}
/*-----------------------------------------------------------*/

/* Timer callback for advertisement RSSI filter timer. 
   
   The timer is called periodically (e.g. once per second) and sends a tick to the 
   advertiser report task.

   There, it is checked if any advertisers have been received which are now due to 
   be sent to the host processor. Once a new advertiser has been received, all reception 
   RSSIs are stored over a certain period (ex. 10s). 
   
   Then, the RSSI values are filtered and a final RSSI is calculated. The advertiser 
   message is sent to the host with this filtered RSSI.

   CAUTION: This function is running in the timer task!
*/
void prvBleFilterTimerCallback( TimerHandle_t xTimer )
{
	enum xADREPORT_CMD			xBleAdHandlerCmd;

	xBleAdHandlerCmd = AD_REPORT_CHK;
	xQueueSend( xBleAdHandlerQueue, &xBleAdHandlerCmd, 0 ); 	
}
/*-----------------------------------------------------------*/

/* Delete the BLE localiser beacons store contents. Access to the data field is mutex-protected. */
void vBleDeleteLocBcnStore( void )
{
	/* Modification of the GSM data is protected by a mutex which avoids that both Parser and GSM 
	   task try to modify the data at the same time. */
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );
	xBleLocBcnGSM.uxBleStdLocBcnCnt 		= 0;	/* Delete all stored BLE positions. */
	xBleLocBcnGSM.uxBleSwissPhoneLocBcnCnt 	= 0;
	xBleLocBcnBLE.uxBleStdLocBcnCnt 		= 0;	/* Delete all stored BLE positions. */
	xBleLocBcnBLE.uxBleSwissPhoneLocBcnCnt 	= 0;
	xSemaphoreGive( xMutexGsmData );	
}
/*-----------------------------------------------------------*/

/* Distinguish between location and distress beacons and dispatch accordingly. */
void vParseBTD( struct KNOWN_DEVICE *pxBcnData, int8_t cRssiValue  )
{
	/* Check if the beacon is a recognised localis3er beacon. */
	if (    ( pxBcnData->xBcnFormat == BLE_ALTBEACON )
		 || ( pxBcnData->xBcnFormat == BLE_IBEACON )
		 || ( pxBcnData->xBcnFormat == BLE_SWISSPHONE_BEACON ) )
	{
		/* Parse the locapliser beacon. */
		vParseLocBeacon( pxBcnData, cRssiValue );
	}
	else 
	{
		if ( pxBcnData->xBcnFormat == BLE_DISTRESS_BEACON )
		{
			/* Parspe the TL beacon. */
			vParseDistressBeacon( pxBcnData, cRssiValue );
		}
	}

	/* In any other case, ignore the packet. */
}
/*-----------------------------------------------------------*/

/* Find the next free entry in the beacon table. If there is none, locate the oldest entry and replace it. 
   Returns the index to the table. 
*/
static signed portBASE_TYPE xGetBeaconTableIndex( volatile struct xBLE_LOC_BEACON *pxBeaconTable, unsigned char *pucBeaconAddr, 
													 int8_t cRssiValue, signed char cRefRSSI, bool bIsSwissPhoneBeacon )
{
	unsigned portBASE_TYPE						xIdx;
	signed portBASE_TYPE						xTblIdx;
	unsigned portBASE_TYPE						uxBleLocBcnCnt;
	bool										bLocationFound;
	bool										bSkipBeacon;
	volatile struct xBLE_LOC_BEACON_DATA_ENTRY	*pxBleLocBcnDataEntry;
	unsigned portBASE_TYPE						uxBcnFilterOption;
	
	uxBcnFilterOption = usConfigReadShort( &xNvdsConfig.usBleBcnFilter ) & ( unsigned short )0x000f;
	bLocationFound = false;
	bSkipBeacon = false;
	xTblIdx = 0;
	
	/* Step 1: Test, if the beacon is already in the table. If yes, overwrite it if the conditions set in usBleBcnFilter are met. If not, append it to the 
	   table unless there is no space left. */
	uxBleLocBcnCnt = pxBeaconTable->uxBleStdLocBcnCnt + pxBeaconTable->uxBleSwissPhoneLocBcnCnt;
	while ( ( xTblIdx < uxBleLocBcnCnt ) && !bLocationFound && !bSkipBeacon )
	{
		pxBleLocBcnDataEntry = &pxBeaconTable->xBleLocBcn[ xTblIdx ].uBleLocBeaconData.xBleLocBeaconData;

		/* Test if it contains the same beacon. */
		if ( memcmp( ( char * )&( pxBleLocBcnDataEntry->ucBeaconAddr ), ( char * )pucBeaconAddr, 6 ) == 0 )
		{
			int8_t		iRefRssiOffset;

			/* Yes: decide what to do in function of usBleBcnFilter[3:0]. */
			switch ( uxBcnFilterOption )
			{
				case 0:
				case 1:		/* Option 0 or 1: Overwrite the entry. */
							bLocationFound = true;
							break;
				case 2:		/* Option 2: Overwrite the present entry if the RSSI of the new reception is higher. */
							if ( cRssiValue >= pxBleLocBcnDataEntry->cRSSI )
							{
								bLocationFound = true;
							}
							else
							{
								/* Avoid that the beacon is added to the table in the following steps as the beacon is too weak. */
								bSkipBeacon = true;
							}
							break;
				case 3:		/* Option 3: A newer one only replaces the present entry if its RSSI is higher
										 and higher than the beacon's reference RSSI (RSSI_REF) + BCN_REF_RSSI_OFFSET. */
							iRefRssiOffset = ( int8_t )( ( usConfigReadShort( &xNvdsConfig.usBleBcnFilter ) & ( unsigned short )0xff00 ) >> 8 );

							if (    ( cRssiValue >= pxBleLocBcnDataEntry->cRSSI )
							     && ( cRssiValue >= cRefRSSI + iRefRssiOffset ) )
							{
								bLocationFound = true;
							}
							else
							{
								/* Avoid that the beacon is added to the table in the following steps as the beacon is too weak. */
								bSkipBeacon = true;
							}
							break;
			}
		}
		else
		{
			xTblIdx++;
		}
	}

	/* If the beacon must be skipped, return immediately. Set xTblIdx to a spcial value telling the caller that the beacon
	   must be discarded. */
	if ( bSkipBeacon )
	{
		return BLE_BCN_DO_NOT_STORE;
	}
	
	if ( !bLocationFound && ( uxBleLocBcnCnt < MAX_LOC_BCN_ENTRIES ) )
	{
		/* Beacon is not yet in table and there is an empty field. Save the beacon. */
		bLocationFound = true;		
		if ( bIsSwissPhoneBeacon )
		{
			pxBeaconTable->uxBleSwissPhoneLocBcnCnt++;
		}
		else
		{
			pxBeaconTable->uxBleStdLocBcnCnt++;
		}
	}		
	
	/* Step 2: The table is full. Select and entry to replace. */
	if ( !bLocationFound )
	{
		if ( uxBcnFilterOption == 0 )
		{
			/* Replace the oldest entry. */
			unsigned short			usOldestTimeStamp;
			
			xTblIdx = 0;
			usOldestTimeStamp = usReadRTC();
			
			for ( xIdx = 0; xIdx < MAX_LOC_BCN_ENTRIES; xIdx++ )
			{
				/* Test if the current table entry is older than the oldest one found until now. */
				if ( bIsANewerThanB( usOldestTimeStamp, pxBleLocBcnDataEntry->usTimeStamp ) )
				{
					/* Yes: Remember this entry as oldest. */
					usOldestTimeStamp = pxBleLocBcnDataEntry->usTimeStamp;
					xTblIdx = xIdx;
				}
			}
		}
		else
		{
			/* Replace the entry with the lowest RSSI. */
			int8_t					iLowestRssi;

			xTblIdx = 0;
			iLowestRssi = 0;

			for ( xIdx = 0; xIdx < MAX_LOC_BCN_ENTRIES; xIdx++ )
			{
				/* Test if the current table entry has a lower RSSI than the one with the lowest RSSI found until now. */
				if ( pxBleLocBcnDataEntry->cRSSI < iLowestRssi )
				{
					/* Yes: Remember this entry as the one with the lowest RSSI. */
					iLowestRssi = pxBleLocBcnDataEntry->cRSSI;
					xTblIdx = xIdx;
				}
			}
		}
	}
	
	return xTblIdx;
}
/*-----------------------------------------------------------*/

/* Enter the beacon data at the next free entry in the beacon table. If there is none, locate the oldest entry and replace it. */
static void vEnterBeaconInBeaconTable( volatile struct xBLE_LOC_BEACON *pxBeaconTable, struct KNOWN_DEVICE *pxBcnData, unsigned char *pucBeaconAddr, 
									   int8_t cRssiValue )
{
	signed portBASE_TYPE						xTblIdx;
	volatile struct xBLE_LOC_BEACON_ENTRY		*pxBleLocBcnEntry;
	volatile struct xBLE_LOC_BEACON_DATA_ENTRY	*pxBleLocBcnDataEntry;
	
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );
	
	/* Enter the beacon in the beacon table destined to be sent over GSM. */
	xTblIdx = xGetBeaconTableIndex( pxBeaconTable, pucBeaconAddr, cRssiValue, *( pxBcnData->pucAdvData + 29 ), pxBcnData->xBcnFormat == BLE_SWISSPHONE_BEACON );

	/* If xGetBeaconTableIndex() decides that the beacon should not be stored, e.g. because it is weaker than an already stored one, return immediately. */
	if ( xTblIdx == BLE_BCN_DO_NOT_STORE )
	{
		xSemaphoreGive( xMutexGsmData );
		return;
	}

	/* Save the beacon data to the identified location. */
	pxBleLocBcnEntry = &pxBeaconTable->xBleLocBcn[ xTblIdx ];
	pxBleLocBcnEntry->bIsSwissPhoneBeacon = ( pxBcnData->xBcnFormat == BLE_SWISSPHONE_BEACON );
	pxBleLocBcnDataEntry = &pxBleLocBcnEntry->uBleLocBeaconData.xBleLocBeaconData;
	memcpy( ( void * )pxBleLocBcnDataEntry->ucBeaconAddr, pucBeaconAddr, 6 );				/* Beacon device ID */
	pxBleLocBcnDataEntry->usTimeStamp = usReadRTC();										/* Time-stamp */
	pxBleLocBcnDataEntry->cRSSI = cRssiValue;												/* RSSI */
	pxBleLocBcnDataEntry->cRefRSSI = *( pxBcnData->pucAdvData + 29 );						/* Beacon reference RSSI */
	if (    ( pxBcnData->xBcnFormat != BLE_IBEACON )
	     && ( pxBcnData->usDataLen == 31 ) )
	{
		/* AltBeacons or SwissPhone beacon. */
		pxBleLocBcnDataEntry->ucBatteryLevel = *( pxBcnData->pucAdvData + 30 );				/* Beacon battery level */
	}
	else
	{
		/* iBeacons do not transmit their battery level. */
		pxBleLocBcnDataEntry->ucBatteryLevel = 0;
	}

	xSemaphoreGive( xMutexGsmData );
}
/*-----------------------------------------------------------*/
	
	
/* Build the table of found BLE beacons from the +BTD responses.
	Advertising data format:
		iBeacons:              02 01 06 1A FF 4C00 0215 80A9059AAD4C488BA267 DD7F2A389675 0000 0000 C3
		Swissphone Beacons:    02 01 06 1B FF 4C00 0215 05A106683FC743B0B6830B1E6A06F73D  02AC 1F52 B9 64
		Radius beacons:        02 01 06 1B FF 1801 BEAC 80A9059AAD4C488BA267 CAF3E52DEEAB 0002 0000 C3 26
		                       |        |     |    |    |                    |            |    |    |  |
		                       |        |     |    |    |                    |            |    |    |  30: MFG reserved (battery, 0x26 = 38%)
		                       |        |     |    |    |                    |            |    |    29: reference RSSI (0xC5 = -59dBm)
		                       |        |     |    |    |                    |            |    27: Minor
		                       |        |     |    |    |                    |            25: Major (BcnUse, e.g. 0x0002 = BCNUSE_NO_ABNORMAL)
						       |	    |     |	   |    |				    19: copied version of beacon advertiser address
		                       |        |     |    |    9: beacon ID                                        
		                       |        |     |    7: altBeacon code (0xBEAC)                                   
		                       |        |     5: MfgID (Radius Networks = 0x1801, Apple = 0x4C00)
		                       |        3: AD field length and identifier for manufacturer specific data (here: 1B = altBeacon, 1A = iBeacon)
						       0: advertiser payload flags

		iBeacons (SwissPhone): 02 01 06 1A FF 4C00 0215 A0397630B0CE12A7845200A56FF11062 0002 0000 C3
		                  	   |        |     |    |    |                                |    |    |  
		                  	   |        |     |    |    |                                |    |    |  
		                  	   |        |     |    |    |                                |    |    29: reference RSSI (0xC5 = -59dBm)
		                  	   |        |     |    |    |                                |    27: Minor
		                  	   |        |     |    |    |                                25: Major (e.g. 0x0002 = BCNUSE_NO_ABNORMAL)
		                  	   |        |     |    |    9: beacon ID                                        
		                  	   |        |     |    7: iBeacon type + data length (0x0215)
		                  	   |        |     5: MfgID (Radius Networks = 0x1801, Apple = 0x4C00)
		                  	   |        3: AD field length and identifier for manufacturer specific data (here: 1B = altBeacon, 1A = iBeacon)
						  	   0: advertiser payload flags
*/
static void vParseLocBeacon( struct KNOWN_DEVICE *pxBcnData, int8_t cRssiValue )
{
	unsigned short								usBcnUse;
	unsigned char								ucBeaconAddr[ 6 ];
	unsigned portBASE_TYPE						xTblIdx;
	enum xBLE_CMD								xBleCmd;	
	volatile struct xBLE_LOC_BEACON_ENTRY		*pxBleLocBcnEntry;
	volatile struct xBLE_LOC_BEACON_DATA_ENTRY	*pxBleLocBcnDataEntry;
	
	/* Extract the BcnUse field and check it against defined bit masks, if there is at least one bit set. 
	   If that is the case, check the BcnIdent if we are really looking at a beacon deployed by TRAXxs.
	   Only then the speacial beacon fields are interpreted.
	   However, all beacons are reported, whether they contain the right BcnIdent field or not. */
	usBcnUse = ( *( pxBcnData->pucAdvData + 25 ) << 8 ) + *( pxBcnData->pucAdvData + 26 );
	if ( ( usBcnUse != 0 ) && ( pxBcnData->xBcnFormat != BLE_SWISSPHONE_BEACON ) )
	{
		/* The beacon is a TRAXxs-installed beacon. Interpret the BcnUse field. */

		/* Test for no abnormal position detection. */
		if ( usBcnUse & BCNUSE_NO_ABNORMAL )
		{
			/* The device is in an area where no abnormal position detection is performed.
			Send a message to the CTRL task but no more often than once every BCNUSE_BLOCKTIME seconds.
			Exclude the init value of usBcnNoAbnormalTimeStamp indicating that the device has never seen
			such a zone before. */
			if (   (    ( usBcnNoAbnormalTimeStamp == 0 ) 
					|| ( usReadRTC() - usBcnNoAbnormalTimeStamp >= BCNUSE_BLOCKTIME ) )
				&& ( cRssiValue >= ( signed char )ucConfigReadByte( ( uint8_t * )&xNvdsConfig.cNoAbnBcnThres ) ) )
			{
				enum xCTRL_EVENT		xCtrlEvent;

				xCtrlEvent = CTRL_NO_ABNORMAL;
				xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );

				usBcnNoAbnormalTimeStamp = usReadRTC();

				/* Set the corresponding bit in the xSpecialBcnRcvd variable so that the cGSM_IND field for the next messages sent to the server
				can be set too. */
				xSpecialBcnRcvd.bNoAbn = true;
			}
		}

		/* Test for immobility zone beacon. */
		if ( usBcnUse & BCNUSE_IMMOBILITY )
		{
			/* The device is in an area where immobility detection is performed.
			Send a message to the CTRL task but no more often than once every BCNUSE_BLOCKTIME seconds.
			Exclude the init value of usBcnImmobilityTimeStamp indicating that the device has never seen
			such a zone before. */
			if (   (   ( usBcnImmobilityTimeStamp == 0 ) 
					|| ( usReadRTC() - usBcnImmobilityTimeStamp >= BCNUSE_BLOCKTIME ) )
				&& ( cRssiValue >= ( signed char )ucConfigReadByte( ( uint8_t * )&xNvdsConfig.cImmobilityBcnThres ) ) )
			{
				enum xCTRL_EVENT		xCtrlEvent;

				xCtrlEvent = CTRL_IMMOBILITY;
				xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );

				usBcnImmobilityTimeStamp = usReadRTC();

				/* Set the corresponding bit in the xSpecialBcnRcvd variable so that the cGSM_IND field for the next messages sent to the server
				can be set too. */
				xSpecialBcnRcvd.bImmobility = true;
			}
		}	
		/* Test for danger zone. */
		if ( usBcnUse & BCNUSE_DANGER )
		{
			/* The device is in a danger area.
			Test, if the danger zone timer is already running indicating that the module has already responded to the situation. */
			if (   !xBleModuleState.bBleInDangerZoneRequest
				&& ( cRssiValue >= ( signed char )ucConfigReadByte( ( uint8_t * )&xNvdsConfig.cDangerBcnThres ) ) )
			{
				/* Note yet responded: Launch the danger zone timer. */
				V_TRACE_PRINT( TRACE_VIBR_DANGER_ZONE, TRACE_UART );

				/* Set the corresponding bit in the xSpecialBcnRcvd variable so that the cGSM_IND field for the next messages sent to the server
				can be set too. */
				xSpecialBcnRcvd.bDanger = true;

				/* Start the timer during which the module is assumed to be in a danger zone after reception of a danger beacon. 
				During this period, reception of further danger beacon messages is blocked and the DANGER bit on the BLE beacon is sent. */
				if ( usConfigReadShort( &xNvdsConfig.usBleInDangerDur ) > 0 )
				{
					( void )xTimerChangePeriod( xInDangerZoneTimer, usConfigReadShort( &xNvdsConfig.usBleInDangerDur ) * portTICKS_PER_SEC, portMAX_DELAY );
				}
				else
				{
					( void )xTimerChangePeriod( xInDangerZoneTimer, IN_DANGER_ZONE_DURATION * portTICKS_PER_SEC, portMAX_DELAY );
				}
				( void )xTimerStart( xInDangerZoneTimer, portMAX_DELAY );
				xBleModuleState.bBleInDangerZoneRequest = true;

				/* Let the BLE process immediately update the BLE beacon contents. */
				xBleCmd = BLE_UPDATE_BCN;
				xQueueSend( xBleCmdQueue, &xBleCmd, 0 );

				/* If there wasn't a foreign alert event just before, vibrate. */
				if ( !xTimerIsTimerActive( xForeignAlertTimer ) )
				{
					/* Vibrate with the given parameters (on/off/repetitions). */
					vCustomVibrate( 7, 3, 10 );			
				}
			}
		}

		/* Test for private zone. */
		if ( usBcnUse & BCNUSE_PRIVATE )
		{
			/* The device is in a private area and stops all reporting.
			Send a message to the GSM task but no more often than once every BCNUSE_BLOCKTIME seconds.
			Exclude the init value of usBcnPrivateZoneTimeStamp indicating that the device has never seen
			such a zone before. */
			if (   (   ( usBcnPrivateZoneTimeStamp == 0 )
					|| ( usReadRTC() - usBcnPrivateZoneTimeStamp >= BCNUSE_BLOCKTIME ) )
				&& ( cRssiValue >= ( signed char )ucConfigReadByte( ( uint8_t * )&xNvdsConfig.cPrivateBcnThres ) ) )
			{
				/* Inform GSM that we are in a private zone but no more often than once every BCNUSE_BLOCKTIME seconds. */
				enum xGSM_CMD		xGsmCmd;

				xGsmCmd = GSM_IN_PRIVATE_ZONE;
				xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmCmd, 0 );

				usBcnPrivateZoneTimeStamp = usReadRTC();

				/* Set the corresponding bit in the xSpecialBcnRcvd variable so that the cGSM_IND field for the next messages sent to the server
				can be set too. */
				xSpecialBcnRcvd.bPrivate = true;
			}

			/* Always immediately clear the received beacon data field to avoid that in the transitional period to SLEEP mode
			received beacons get sent out. */
			vBleDeleteLocBcnStore();
		}

		/* Test if no location reporting is required. */
		if ( !( usBcnUse & BCNUSE_LOC ) )
		{
			/* The location reporting bit is not set but another bit is.
			So the beacon explicitly asks not to be reported. */
			return;
		}	
	}

	/* Copy beacon address byte-wise and put it in the right order. */
	if ( pxBcnData->xBcnFormat != BLE_SWISSPHONE_BEACON )
	{
		for ( xTblIdx = 0; xTblIdx < 6; xTblIdx++ )
		{
			ucBeaconAddr[ xTblIdx ] = *( pxBcnData->pucPeerAddr + 5 - xTblIdx );
		}
	}
	else
	{
		/* For UUID-filtered SwissPhone beacons, replace the Advertiser Address with a combination
		   of UUID filter match index and major and minor fields. */
		ucBeaconAddr[ 0 ] = pxBcnData->ucUuidFilterMatchIdx;
		memcpy( ucBeaconAddr + 1, pxBcnData->pucAdvData + 25, 4 );
		ucBeaconAddr[ 5 ] = 0;
	}
	
	/* Enter beacon data into a dictionary of max. MAX_LOC_BCN_ENTRIES beacon entries.
	   Algorithm:
			- Add new entry if not already in and less than 5 entries in dictionary.
			- Overwrite if entry already exists.
			- Overwrite oldest entry if dictionary is full.

	   Walk through all beacon table entries and identify the location the beacon data needs to go. */
	
	/* Send a message to the BLE main task which takes care of flashing the blue LED. */
	xBleCmd = BLE_BEACON_CAPTURED;
	xSendUniqueMsgToQueue( xBleCmdQueue, &xBleCmd, 0 ); 
	
	/* Enter the beacon in the beacon table to be sent over GSM. */
	vEnterBeaconInBeaconTable( &xBleLocBcnGSM, pxBcnData, ucBeaconAddr, cRssiValue );
	
	/* Enter the beacon in the beacon table to be sent over BLE. */
	vEnterBeaconInBeaconTable( &xBleLocBcnBLE, pxBcnData, ucBeaconAddr, cRssiValue );
}
/*-----------------------------------------------------------*/


/* Get the next decyrpted payload byte.
   The function first checks the number of bytes requested and read. If all read bytes have been used, it reads the next 
   payload block from the UART buffer, decrypts it and updates its uxPayloadBytesAvailable counter.
   Then, it delivers the next decrypted byte. 
*/
static unsigned char ucGetNextDecryptedByte( unsigned char **ppusEncrAdvData, unsigned portBASE_TYPE  *puxPayloadBytesAvailable, unsigned char *pucPlainTextBlock )
{
	/* Check if a new block needs to be read and decrypted. */
	if ( *puxPayloadBytesAvailable == 0 )
	{
		/* Decrypt the next AES block. */
		bAES_CBC_DecryptBlock( ( uint8_t * )*ppusEncrAdvData, ( uint8_t * )pucPlainTextBlock );
		
		*ppusEncrAdvData += AES_BLOCK_LENGTH;
		*puxPayloadBytesAvailable += AES_BLOCK_LENGTH;
	}
	
	return *( pucPlainTextBlock + ( AES_BLOCK_LENGTH - ( *puxPayloadBytesAvailable )-- ) );
}
/*-----------------------------------------------------------*/


/* Calculate the length of the variable TL beacon field, the contents of which are specified in the corresponding FDFMAP.
   A ROM table gives the individual length of each field.
   Special care needs to be taken when determining the length of the FBCN beacon field. The number of beacons is given with
   FDFMAP(19,6:4).
   
   The function also returns the number of fields.
*/
unsigned portBASE_TYPE uxCalculateVarBcnFieldLen( unsigned long ulDFMap, unsigned portBASE_TYPE uxDFMapFieldIdx,
												  unsigned portBASE_TYPE *puxNumFields )
{
	unsigned portBASE_TYPE	uxTlBcnFieldLen;
	unsigned portBASE_TYPE	uxNumFields;
	unsigned portBASE_TYPE	uxBcnNum;
	unsigned portBASE_TYPE	uxDFMapIdx;
	
	uxNumFields = 0;
	uxTlBcnFieldLen = 0;
	
	/* Walk through the FDFMAP and add the data fields present there up to the field specified by uxDFMapFieldIdx. */
	for ( uxDFMapIdx = 0; uxDFMapIdx < uxDFMapFieldIdx; uxDFMapIdx++ )
	{
		/* Skip over all indices belonging to the beacon field. 
		   */
		if (    ( uxDFMapIdx != BLE_BCN_BCN_ONE_IDX ) 
		     && ( uxDFMapIdx != BLE_BCN_BCN_TWO_IDX ) 
		     && ( uxDFMapIdx != BLE_BCN_BCN_THREE_IDX ) 
		     && ( uxDFMapIdx != BLE_BCN_BCN_FOUR_IDX ) 
		     && ( uxDFMapIdx != BLE_BCN_UUID_BCN_ONE_IDX ) 
		     && ( uxDFMapIdx != BLE_BCN_UUID_BCN_TWO_IDX ) 
		     && ( uxDFMapIdx != BLE_BCN_UUID_BCN_THREE_IDX ) 
		     && ( uxDFMapIdx != BLE_BCN_UUID_BCN_FOUR_IDX ) )
		{
			/* Ordinary data field, not a BCN. */
			if ( ( ulDFMap >> uxDFMapIdx ) & 1l )
			{
				/* Add the length of the data expressed as HEX-ASCII. */
				uxTlBcnFieldLen += uxTLBcnDataFieldsLen[ uxDFMapIdx ];
				
				/* Count the field. */
				uxNumFields++;
			}

			continue;
		}

		if ( uxDFMapIdx == BLE_BCN_BCN_ONE_IDX )
		{
			/* BCN data. Calculate the length data once for all four beacon length bits. */
			/* Number of locator beacons. Take BCN2 in the extension DFMAP into account. */
			uxBcnNum = ( ( ulDFMap & BLE_BCN_BCN_ALL_MSK ) >> BLE_BCN_BCN_ONE_IDX );
			uxBcnNum += ( ( ulDFMap & BLE_BCN_BCN2_MSK ) >> BLE_BCN_BCN_FOUR_IDX ) << 3;
			
			/* Total length of all beacons. */
			uxTlBcnFieldLen += uxBcnNum * BLE_LOC_BCN_LEN;
			/* Count all fields within all BCN fields. */
			uxNumFields += uxBcnNum * 5;
			
			/* Skip over remaining bits belonging to the BCN field. */
			uxDFMapIdx += 2;
		}

		if ( uxDFMapIdx == BLE_BCN_UUID_BCN_ONE_IDX )
		{
			/* BCN data. Calculate the length data once for all four beacon length bits. */
			/* Number of locator beacons. Take BCN2 in the extension DFMAP into account. */
			uxBcnNum = ( ( ulDFMap & BLE_BCN_UUID_BCN_ALL_MSK ) >> BLE_BCN_UUID_BCN_ONE_IDX );
			
			/* Total length of all beacons. */
			uxTlBcnFieldLen += uxBcnNum * BLE_LOC_BCN_LEN;
			/* Count all fields within all BCN fields. */
			uxNumFields += uxBcnNum * 5;
			
			/* Skip over remaining bits belonging to the BCN field. */
			uxDFMapIdx += 3;
		}
	}
	
	if ( puxNumFields != NULL )
	{
		*puxNumFields = uxNumFields;
	}
	
	return uxTlBcnFieldLen;
}
/*-----------------------------------------------------------*/


/* Extract the FDFMAP and parse the LR beacon payload. Decrypt the payload on-the-fly. 
   The LR beacon payload typically spans several AES blocks. When entering this function, we have already decrypted the very first one.
   
   All data fields up to index 28 are copied to xBleTLBcn w/o looking at the contents. Data fields 28, 29 and 30 belong to relayed server 
   commands which are identified here. Index 28 is the destination address which is compared against the own IMEI and a global broadcast address.
   If there is a match, the command index and command argument are stored.
*/
static bool bCopyLRBeaconData( unsigned char *pucAdvData, unsigned char ucPlainTextBlock[], unsigned portBASE_TYPE uxTblIdx, unsigned char *pucChkSum,
						       bool bStoreData )
{
	unsigned portBASE_TYPE	uxIdx;
	unsigned portBASE_TYPE	uxPayloadBytesAvailable;
	unsigned portBASE_TYPE	uxBcnVarDatLen;
	unsigned long			ulDFMap;
	unsigned char			ucDecryptedByte;
	unsigned char			*pucEncrAdvData;
	
	/* Get the DFMAP which is transmitted right next to the ID. We need to reformat it slightly for the FDFMAP sent
	   to the server.
	   First, copy the bits corresponding to FLAT and above. */
	ulDFMap =       ( unsigned long )ucPlainTextBlock[ 12 ] 
				+ ( ( unsigned long )ucPlainTextBlock[ 13 ] <<  8 )
				+ ( ( unsigned long )ucPlainTextBlock[ 14 ] << 16 )
				+ ( ( unsigned long )ucPlainTextBlock[ 15 ] << 24 );
	if ( bStoreData )
	{
		xBleTLBcn[ uxTblIdx ].ulDFMap = ulDFMap;
	}
									
	/* Update the checksum. */
	*pucChkSum += ucPlainTextBlock[ 12 ] + ucPlainTextBlock[ 13 ] + ucPlainTextBlock[ 14 ] + ucPlainTextBlock[ 15 ];
	
	/* Walk through the FDFMAP and add the data fields present there up to the relay command fields. */
	uxBcnVarDatLen = uxCalculateVarBcnFieldLen( ulDFMap, BLE_BCN_RLYADDR_IDX, NULL );
	
	/* The entire decoded AES block data has been used up. Start using a new one. */
	uxPayloadBytesAvailable = 0;
	
	/* Now that we have got the data length, we can simply copy the payload data byte-by-byte. */
	pucEncrAdvData = pucAdvData;
	for ( uxIdx = 0; uxIdx < uxBcnVarDatLen; uxIdx++ )
	{
		/* Copy the foreign locator beacon byte to the target structure. */
		ucDecryptedByte = ucGetNextDecryptedByte( &pucEncrAdvData, &uxPayloadBytesAvailable, ucPlainTextBlock );
		*pucChkSum += ucDecryptedByte;
		if ( bStoreData )
		{
			xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxIdx ] = ucDecryptedByte;
		}		
	}
	
	/* Check, if a relayed command is present in the LR payload. */
	if ( ulDFMap & BLE_BCN_RLYADDR_MSK )
	{
		bool					bRelayedCmdAddressMatch;
		bool					bRelayedCmdBcAddressMatch;
		unsigned portBASE_TYPE	uxRelayedCmdIdx;	
		unsigned short			usRelayedCmdParam;	
				
		/* Get the destination address IMEI and compare with the device's own IMEI. */
		ucDecryptedByte = ucGetNextDecryptedByte( &pucEncrAdvData, &uxPayloadBytesAvailable, ucPlainTextBlock );
		*pucChkSum += ucDecryptedByte;
		bRelayedCmdAddressMatch   = ( cNibbleToChar( ucDecryptedByte & 0x0f ) == ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID ) );
		bRelayedCmdBcAddressMatch = ( cNibbleToChar( ucDecryptedByte & 0x0f ) == *( pcRlyBroadcastAddr + 0 ) );
		for ( uxIdx = 1; uxIdx < LEN_GSM_ID; uxIdx += 2 )
		{
			ucDecryptedByte = ucGetNextDecryptedByte( &pucEncrAdvData, &uxPayloadBytesAvailable, ucPlainTextBlock );
			*pucChkSum += ucDecryptedByte;
			bRelayedCmdAddressMatch   &= ( cNibbleToChar( ( ucDecryptedByte & 0xf0 ) >> 4 ) == ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID + uxIdx     ) );
			bRelayedCmdAddressMatch   &= ( cNibbleToChar(   ucDecryptedByte & 0x0f )        == ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID + uxIdx + 1 ) );
			bRelayedCmdBcAddressMatch &= ( cNibbleToChar( ( ucDecryptedByte & 0xf0 ) >> 4 ) == *( pcRlyBroadcastAddr + uxIdx     ) );
			bRelayedCmdBcAddressMatch &= ( cNibbleToChar(   ucDecryptedByte & 0x0f )        == *( pcRlyBroadcastAddr + uxIdx + 1 ) );
		}
		
		/* Get the relayed command index. */
		uxRelayedCmdIdx = ucGetNextDecryptedByte( &pucEncrAdvData, &uxPayloadBytesAvailable, ucPlainTextBlock );
		*pucChkSum += uxRelayedCmdIdx;
		
		/* If present, get the command parameter. */
		if ( ulDFMap & BLE_BCN_RLYPARAM_MSK )
		{
			ucDecryptedByte = ucGetNextDecryptedByte( &pucEncrAdvData, &uxPayloadBytesAvailable, ucPlainTextBlock );
			*pucChkSum += ucDecryptedByte;
			usRelayedCmdParam = ucDecryptedByte;
			ucDecryptedByte = ucGetNextDecryptedByte( &pucEncrAdvData, &uxPayloadBytesAvailable, ucPlainTextBlock );
			*pucChkSum += ucDecryptedByte;
			usRelayedCmdParam += ucDecryptedByte << 8;
		}
		else
		{
			usRelayedCmdParam = 0;
		}
			
		/* Handle the command if it was for this device. */
		if ( bRelayedCmdAddressMatch || bRelayedCmdBcAddressMatch )
		{
			/* Send the command to the GSM task which handles it. */ 
			vTreatBleCmd( uxRelayedCmdIdx, usRelayedCmdParam );
		}
	}
	
	/* The last byte is the checksum. Extract it and check it against the locally built one and return the result. */
	if ( *pucChkSum == ucGetNextDecryptedByte( &pucEncrAdvData, &uxPayloadBytesAvailable, ucPlainTextBlock ) )
	{
		return true;
	}
	else
	{
		return false;
	}
}
/*-----------------------------------------------------------*/

/* Extract alert type and foreign tracker ID from received distress beacons and copy the information 
   to the GSM data fields.
			
	Advertising data format for TL beacons:
	                          02 01 06 12 FF 1700 01 01 96A975 0358578080077244 DBFB0300...
			                  |        |     |    |  |  |      ||               | 
			                  |        |     |    |  |  |      ||               20: DFMAP
			                  |        |     |    |  |  |      |foreign tracker ID (IMEI) 
			                  |        |     |    |  |  |      12: padding                                        
			                  |        |     |    |  |  9: TRAXxs manufacturer code (0x75A996)                                   
			                  |        |     |    |  8: descriptor bitfield: bit 0 = evacuation, bit 1 = alert, bit 2 = SOS                           
			                  |        |     |    7: meshing indicator (0x01)
			                  |        |     5: MfgID (Newlogic = 0x0017)                                   
			                  |        3: AD field length and identifier for manufacturer specific data (here: distress beacon)
							  0: advertiser payload flags
*/
static void vParseDistressBeacon( struct KNOWN_DEVICE *pxBcnData, int8_t cRssiValue )
{
	unsigned portBASE_TYPE	uxDesc;
	unsigned portBASE_TYPE	uxIdx;
	unsigned portBASE_TYPE	uxSkipMesh;
	unsigned char			ucPlainTextBlock[ AES_BLOCK_LENGTH ];
	enum xGSM_CMD			xGsmCmd;
	enum xBLE_CMD			xBleCmd;
	bool					bLocationFound;
	unsigned portBASE_TYPE	uxTblIdx;
	unsigned char			ucLongRange;
	unsigned char			ucChkSum;
	
	/* Get character index of PHY type. */
	if ( pxBcnData->ucPrimaryPhy == 4 )
	{
		ucLongRange = BLE_RF_IF_LR;
		uxSkipMesh = 1;
	}
	else
	{
		ucLongRange = BLE_RF_IF_SR;
		uxSkipMesh = 0;
	}
	
	/* Check if MESH has the right value for distress beacons (as opposed to XSWITCH internal communication). */
	if ( ( ucLongRange == BLE_RF_IF_LR ) && ( *( pxBcnData->pucAdvData + 7 ) != 0x01 ) )
	{
		return;
	}

	/* Decrypt the first payload AES block. */
	/* Test if the payload was encrypted with AES key 0. For this, decrypt and check the value of the TRAXxs manufacturer code. */
	vAES_CBC_DecryptStart( ( uint8_t * )&xNvdsConfig.pcAESKey[ 0 ] );		
	bAES_CBC_DecryptBlock( pxBcnData->pucAdvData + uxSkipMesh + 7, ( uint8_t * )ucPlainTextBlock );
	if ( ( ucPlainTextBlock[ 1 ] != 0x96 ) || ( ucPlainTextBlock[ 2 ] != 0xA9 ) || ( ucPlainTextBlock[ 3 ] != 0x75 ) )
	{
		/* Key 0 did not work. Repeat the test with AES key 1. */
		vAES_CBC_EncryptStop();
		vAES_CBC_DecryptStart( ( uint8_t * )&xNvdsConfig.pcAESKey[ 1 ] );
		bAES_CBC_DecryptBlock( pxBcnData->pucAdvData + uxSkipMesh + 7, ( uint8_t * )ucPlainTextBlock );
		if ( ( ucPlainTextBlock[ 1 ] != 0x96 ) || ( ucPlainTextBlock[ 2 ] != 0xA9 ) || ( ucPlainTextBlock[ 3 ] != 0x75 ) )
		{
			/* AES key 1 did not work either. Abandon attempt. */
			vAES_CBC_EncryptStop();
			NRF_LOG_WARNING( "%i Unable to decrypt TL beacon.", ulReadRTC() );
			return;
		}
	}
	
	/* Initialise the checksum with the data used/verified so far. */
	ucChkSum = ( unsigned char )( 0x01 + 0x96 + 0xA9 + 0x75 );
	
	/* At this point, the payload is available as octet array in ucPlainTextBlock.
	   Format: '01 96A975 0358578080077244 DBFB0300'
			    |  |      |                |  
			    |  |      |                12: DFMAP
			    |  |      4: foreign tracker ID (IMEI) 
			    |  1: TRAXxs manufacturer code (0x75A996)                                   
			    0: descriptor bitfield: bit 0 = evacuation, bit 1 = alert, bit 2 = SOS   */                        
				
	/* Register the foreign alert type. The definition of bits in uxForeignAlertType matches that of the distress beacon (see ble.c, prvAddAdvData()). */
	uxDesc = ucPlainTextBlock[ 0 ];
	ucChkSum += uxDesc;
	
	/* Set flag if the TL beacon report is received from an XSWITCH. */
	if ( uxDesc & BLE_XSWITCH_ORIGIN )
	{
		bBleXSwitchObserved = true;
	}
	
	/* TL devices only report received TL distress beacons, XSWITCH devices all of them. 
       Beacons containing relay commands are decoded but not stored. */
	if ( ( uxDesc & BLE_ALERT ) || ( uxDesc & BLE_SOS ) || ( uxDesc & BLE_EVAC ) || ( uxDesc & BLE_DANGER ) )
	{
		/* Also send a message to the BLE main task which takes care of flashing the blue LED. */
		xBleCmd = BLE_RX_FOREIGN_ALERT_SOS;
		xQueueSend( xBleCmdQueue, &xBleCmd, 0 ); 
		
		/* For ALERT and SOS (as well as later heartbeat beacons), enter beacon data into a dictionary of max. MAX_TL_BCN_ENTRIES beacon entries.
		   Algorithm:
				- Add new entry if not already in and less than 5 entries in dictionary.
				- Overwrite if entry already exists.
				- Overwrite oldest entry if dictionary is full.

		   Walk through all beacon table entries and identify the location the beacon data needs to go. */
		bLocationFound = false;
		uxTblIdx = 0;
		
		/* Step 1: Test, if the device (identified by its FID) is already in the table. If yes, overwrite it. If not, append it to the table 
		   unless there is no space left. */
		while ( ( uxTblIdx < uxBleTLBcnCnt ) && !bLocationFound )
		{
			/* Test if it contains the same beacon. */
			if ( memcmp( ( char * )&( xBleTLBcn[ uxTblIdx ].ucFID ), ( char * )( ucPlainTextBlock + 4 ), 8 ) == 0 )
			{
				/* Yes. */
				/* Test if both entry and the new beacon are of the same RF type. 
				   Catches the cases:
								stored one = SR			new one = SR
								stored one = LR			new one = LR */
				if  ( xBleTLBcn[ uxTblIdx ].ucLongRange == ucLongRange )
				{
					/* Yes: Overwrite the entry. */
					bLocationFound = true;
				}
				/* Test if the entry is a combined one (SR + LR) and the new beacon is long range.
				   Catches the case:
								stored one = LR + SR	new one = LR */
				else if ( ucLongRange == BLE_RF_IF_LR )
				{
					/* We have received a long-range beacon but we already have a combined version stored. Overwrite all entries 
					   except for RSSI. */
					cRssiValue = xBleTLBcn[ uxTblIdx ].cRSSI;
					bLocationFound = true;
				}
				/* Test if the entry is a combined one (SR + LR) and the new beacon is short range.
				   Catches the case:
								stored one = LR + SR	new one = SR 
				   As this is the only case left, a simple 'else' statement is sufficient. */
				else 
				{
					/* Overwrite only the RSSI entry. The RF type is now a combined one. */
					configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );
					xBleTLBcn[ uxTblIdx ].cRSSI = cRssiValue;
					vAES_CBC_EncryptStop();
					xSemaphoreGive( xMutexGsmData );
					return;
				}
			}
			else
			{
				uxTblIdx++;
			}
		}
		
		/* Record the data in a new table entry if there is still an empty one. */
		if ( ( uxTblIdx < MAX_TL_BCN_ENTRIES ) && !bLocationFound )
		{
			uxBleTLBcnCnt++;
			bLocationFound = true;
		}

		/* Step 2: Identify the oldest entry. */
		if ( !bLocationFound )
		{
			unsigned short			usOldestTimeStamp;
			
			uxTblIdx = 0;
			usOldestTimeStamp = usReadRTC();
			
			for ( uxIdx = 0; uxIdx < MAX_LOC_BCN_ENTRIES ; uxIdx++ )
			{
				/* Test if the current table entry is older than the oldest one found until now. */
				if ( bIsANewerThanB( usOldestTimeStamp, xBleTLBcn[ uxIdx ].usRxTimeStamp ) )
				{
					/* Yes: Remember this entry as oldest. */
					usOldestTimeStamp = xBleTLBcn[ uxIdx ].usRxTimeStamp;
					uxTblIdx = uxIdx;
				}
			}
		}
	
		/* Save the beacon data to the identified location. */
		configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );
			
		/* First, parts common to both SR and LR beacon types. Note that the beacon definitions and data are different between these two. */
		xBleTLBcn[ uxTblIdx ].usRxTimeStamp = usReadRTC();										/* RX Time-stamp */
		xBleTLBcn[ uxTblIdx ].ucFDesc = uxDesc;													/* Descriptor */
		memcpy( ( void *) xBleTLBcn[ uxTblIdx ].ucFID, ucPlainTextBlock + 4, BLE_BCN_FID_LEN );	/* FID */
		xBleTLBcn[ uxTblIdx ].cRSSI = cRssiValue;												/* RSSI */
		xBleTLBcn[ uxTblIdx ].ucLongRange = ucLongRange;										/* Long-range */
		
		/* Second, add the DFMAP and type-specific data fields. */
		if ( ucLongRange == BLE_RF_IF_SR )
		{
			unsigned portBASE_TYPE		uxDFPos;
			
			/* SR beacons. */			
			uxDFPos = 0;
			
			/* TX power. */
			xBleTLBcn[ uxTblIdx ].ulDFMap = 0x00000002;							/* The DFMAP for SR beacons. */
			xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxDFPos++ ] = ucPlainTextBlock[ 12 ];	
			if ( uxDesc & BLE_EVAC ) 
			{
				/* Evacuation ID. */
				/* In case of an evacuation alert, add the evacuation ID. In Short Range beacons, 
				   the evacuation ID is given in a fixed position. */
				xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxDFPos++ ] = ucPlainTextBlock[ 13 ];
				xBleTLBcn[ uxTblIdx ].ulDFMap |= 0x00000004;
			}
			if ( ucPlainTextBlock[ 14 ] != 0 )
			{
				/* Battery level. */
				xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxDFPos++ ] = ucPlainTextBlock[ 14 ];		
				xBleTLBcn[ uxTblIdx ].ulDFMap |= 0x00000008;
			}
		}
		else
		{
			/* LR beacons. */
			/* Update the checksum with the FID. */
			for ( uxIdx = 0; uxIdx < BLE_BCN_FID_LEN ; uxIdx++ )
			{
				ucChkSum += xBleTLBcn[ uxTblIdx ].ucFID[ uxIdx ];
			}
			
			/* Copy the advertiser data and check the data against the checksum. */
			if ( !bCopyLRBeaconData( pxBcnData->pucAdvData + 8 + AES_BLOCK_LENGTH, ucPlainTextBlock, uxTblIdx, &ucChkSum, BLE_STOREDATA ) )
			{
				/* Check failed: removed the data from the table. */
				V_TRACE_PRINT( TRACE_BLE_LR_BEACON_CHKSUM_FAIL, TRACE_UART );
				
				/* Remove the data from the table. */
				/* First, check if there are not any other entries after the current one. */
				if ( uxTblIdx  == uxBleTLBcnCnt - 1 ) 
				{
					/* No more data is following: just reduce the counter for total number of entries. */
					uxBleTLBcnCnt--;
				}
				else
				{
					/* Copy the last entry in the table to the current position and reduce the counter. */
					for ( uxIdx = 0; uxIdx < BLE_BCN_FID_LEN ; uxIdx++ )
					{
						xBleTLBcn[ uxTblIdx ].ucFID[ uxIdx ] = xBleTLBcn[ uxBleTLBcnCnt - 1 ].ucFID[ uxIdx ];
					}
					xBleTLBcn[ uxTblIdx ].usRxTimeStamp = xBleTLBcn[ uxBleTLBcnCnt - 1 ].usRxTimeStamp;
					xBleTLBcn[ uxTblIdx ].ucFDesc = xBleTLBcn[ uxBleTLBcnCnt - 1 ].ucFDesc;
					xBleTLBcn[ uxTblIdx ].cRSSI = xBleTLBcn[ uxBleTLBcnCnt - 1 ].cRSSI;
					xBleTLBcn[ uxTblIdx ].ucLongRange = xBleTLBcn[ uxBleTLBcnCnt - 1 ].ucLongRange;
					xBleTLBcn[ uxTblIdx ].ulDFMap = xBleTLBcn[ uxBleTLBcnCnt - 1 ].ulDFMap;
					for ( uxIdx = 0; uxIdx < BLE_BCN_VAR_FLD_LEN ; uxIdx++ )
					{
						xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxIdx ] = xBleTLBcn[ uxBleTLBcnCnt - 1 ].ucVarBcnData[ uxIdx ];
					}
					
					uxBleTLBcnCnt--;
				}
			}
		}
			
		vAES_CBC_EncryptStop();
		xSemaphoreGive( xMutexGsmData );

		/* In case of ALERT or SOS, inform the GSM task to send a foreign alert. */
		if ( ( uxDesc & BLE_ALERT ) || ( uxDesc & BLE_SOS ) )
		{
			/* Indicate received foreign SOS or ALERT to GSM to set the flags accordingly. */
			uxForeignAlertType = uxDesc & ( BLE_ALERT | BLE_SOS );
				
			/* Tell the GSM task to immediately send a message containing the foreign ALERT or SOS. */
			xGsmCmd = GSM_SEND_FOREIGN_ALERT_SOS;
			xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmCmd, 0 ); 

			/* Check using the foreign alert timer if the module has already responded to the event. */
			if ( !xTimerIsTimerActive( xForeignAlertTimer ) )
			{
				if (   (    ( uxDesc & BLE_ALERT )
					     && (    ( ( ucLongRange == BLE_RF_IF_SR ) && ( usConfigReadShort( &xNvdsConfig.usBleForeignAlert ) & REACT_FOREIGN_ALERT_SR ) )
				              || ( ( ucLongRange == BLE_RF_IF_LR ) && ( usConfigReadShort( &xNvdsConfig.usBleForeignAlert ) & REACT_FOREIGN_ALERT_LR ) )
					   	    )
						)
					 || (   ( uxDesc & BLE_SOS )
					     && (    ( ( ucLongRange == BLE_RF_IF_SR ) && ( usConfigReadShort( &xNvdsConfig.usBleForeignAlert ) & REACT_FOREIGN_SOS_SR ) )
				              || ( ( ucLongRange == BLE_RF_IF_LR ) && ( usConfigReadShort( &xNvdsConfig.usBleForeignAlert ) & REACT_FOREIGN_SOS_LR ) )
					   	    )
						)
				   )
				{
					/* Start the timer during which the module responds to a foreign alert after reception of a TL beacon. */
					if ( usConfigReadShort( &xNvdsConfig.usBleInDangerDur ) > 0 )
					{
						( void )xTimerChangePeriod( xForeignAlertTimer, usConfigReadShort( &xNvdsConfig.usBleInDangerDur ) * portTICKS_PER_SEC, portMAX_DELAY );
					}
					else
					{
						( void )xTimerChangePeriod( xForeignAlertTimer, IN_DANGER_ZONE_DURATION * portTICKS_PER_SEC, portMAX_DELAY );						
					}
					( void )xTimerStart( xForeignAlertTimer, portMAX_DELAY );
					
					/* Only vibrate if neither own nor the foreign danger zone timer is already running indicating that the danger 
					   zone alert has already been given. */
					if ( !xBleModuleState.bBleInDangerZoneRequest )
					{
						/* Vibrate with the given parameters (on/off/repetitions). */
						vCustomVibrate( 7, 3, 10 );			
					}
				}
			}
		}
		
		/* In case of an evacuation, start the evacuation vibration. */
		if ( uxDesc & BLE_EVAC )
		{
			if ( ucLongRange == BLE_RF_IF_SR ) 
			{
				/* In Short Range beacons, the evacuation ID is given in a fixed position. */
				vIndicateEvacuation( ucPlainTextBlock[ 13 ] );
			}
			else
			{
				unsigned portBASE_TYPE		uxBcnVarDatEvacIDIdx;
				
				/* In Long Range, the position of the EVAC_ID is determined by the FDFMAP. */
				uxBcnVarDatEvacIDIdx = uxCalculateVarBcnFieldLen( xBleTLBcn[ uxTblIdx ].ulDFMap, BLE_BCN_EVAC_ID_IDX, NULL );
				vIndicateEvacuation( xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxBcnVarDatEvacIDIdx ] );
			}
		}
		
		/* In case of a foreign danger, start the danger vibration if enabled for this type of RF interface. */
		if ( uxDesc & BLE_DANGER )
		{
			/* Check using the foreign alert timer if the module has already responded to the event. 
			   The softweare uses the same timer (xForeignAlertTimer) for both foreign alerts and foreign dagner zones. */
			if ( !xTimerIsTimerActive( xForeignAlertTimer ) )
			{
				/* No: Test if the alert for the given interface is enabled. */
				if (   ( ( ucLongRange == BLE_RF_IF_SR ) && ( usConfigReadShort( &xNvdsConfig.usBleForeignAlert ) & REACT_FOREIGN_DANGER_SR ) )
				    || ( ( ucLongRange == BLE_RF_IF_LR ) && ( usConfigReadShort( &xNvdsConfig.usBleForeignAlert ) & REACT_FOREIGN_DANGER_LR ) )
				   )
				{
					V_TRACE_PRINT( TRACE_VIBR_FOREIGN_DANGER_ZONE, TRACE_UART );

					/* Start the timer during which the module responds to a foreign alert after reception of a TL beacon. */
					if ( usConfigReadShort( &xNvdsConfig.usBleInDangerDur ) > 0 )
					{
						( void )xTimerChangePeriod( xForeignAlertTimer, usConfigReadShort( &xNvdsConfig.usBleInDangerDur ) * portTICKS_PER_SEC, portMAX_DELAY );
					}					
					else
					{
						( void )xTimerChangePeriod( xForeignAlertTimer, IN_DANGER_ZONE_DURATION * portTICKS_PER_SEC, portMAX_DELAY );
					}
					( void )xTimerStart( xForeignAlertTimer, portMAX_DELAY );
					
					/* Only vibrate if neither own nor the foreign danger zone timer is already running indicating that the danger 
					   zone alert has already been given. */
					if ( !xBleModuleState.bBleInDangerZoneRequest )
					{
						/* Vibrate with the given parameters (on/off/repetitions). */
						vCustomVibrate( 7, 3, 10 );			
					}
				}				
			}
		}
	}
	else
	{
		/* From all other beacons only the relayed commands are needed. */
		if ( uxDesc & BLE_RELAYCMD ) 		
		{
			/* Update the checksum with the FID. */
			for ( uxIdx = 0; uxIdx < BLE_BCN_FID_LEN ; uxIdx++ )
			{
				ucChkSum += *( pxBcnData->pucAdvData + 4 + uxIdx );
			}
			
			/* Extract the relayed command and check the data against the checksum. */
			if ( !bCopyLRBeaconData( pxBcnData->pucAdvData + 8 + AES_BLOCK_LENGTH, ucPlainTextBlock, 0, &ucChkSum, BLE_TRASHDATA ) )
			{
				/* Checksum error: do nothing. */
				V_TRACE_PRINT( TRACE_BLE_LR_BEACON_CHKSUM_FAIL, TRACE_UART );
			}
		}

		vAES_CBC_EncryptStop();
	}
}
/*-----------------------------------------------------------*/

/* Store the contents of the BLE location beacon dictionary in the location beacon record array. */
void vStoreBLEAdvertisers( void )
{
	/* Only store the current dictionary if it contains at least one entry. */ 
	if ( ( xBleLocBcnGSM.uxBleStdLocBcnCnt + xBleLocBcnGSM.uxBleSwissPhoneLocBcnCnt ) > 0 )
	{
		/* Write the current BLE dictionary information to the BLE advertiser store. */
		xBleLocBeaconStore[ uxPosStoreWrIdx ] = xBleLocBcnGSM;
	}
	else
	{
		xBleLocBeaconStore[ uxPosStoreWrIdx ].uxBleStdLocBcnCnt = 0;
	}
}
/*-----------------------------------------------------------*/

/* 
 * Advertiser report task.
 */
static void vBleAdHandlerTask( void * pvParameter )
{
	enum xADREPORT_CMD			xBleAdHandlerCmd;
    uint8_t                   	xRSSI;
	TickType_t				  	xCurrentTime;

    ( void )pvParameter;

	vTaskSetApplicationTaskTag( NULL, ( void * ) BLE_AD_TASK_TAG );
	
	/* Wait untile the configuration handler is initialised. */
	while ( !bCheckConfigInitialised() || !bCheckTraceInitialised() )
	{
		vTaskDelay( 1 );
	}

	NRF_LOG_INFO( "Task started." );
	NRF_LOG_FLUSH();
	
	/* Task main loop. */
    while ( true )
    {
		if ( xQueueReceive( xBleAdHandlerQueue, &xBleAdHandlerCmd, portMAX_DELAY ) != errQUEUE_EMPTY )
		{
			/* Received a trigger from the timer task. */
			switch ( xBleAdHandlerCmd )
			{
				case AD_REPORT_CHK:			/* Walk through the list of stored advertisers and see if there is one who has already been waiting for 
											   FILTER_INTERVAL seconds. */
											xCurrentTime = xTaskGetTickCount();
											
											/* Take mutex as xKnownDeviceList is also accessed by BLE main vStoreAdvertiser(). */
											xSemaphoreTake( xMutexBleAdvData, BLE_OS_TIMEOUT );

											for ( unsigned portBASE_TYPE uxListIdx = 0; uxListIdx < KNOWN_DEVICE_LIST_LEN; uxListIdx++ )
											{
												if (   xKnownDeviceList[ uxListIdx ].bUsed
												    && ( xCurrentTime - xKnownDeviceList[ uxListIdx ].xRxTimeStamp > uxRssiFilterWindow * portTICKS_PER_SEC ) )
												{
													/* Filter RSSIs. */
													xRSSI = xFilterRSSI( xKnownDeviceList[ uxListIdx ].xRSSI );
													
													/* Parse the advertiser data. */
													vParseBTD( &xKnownDeviceList[ uxListIdx ], xRSSI );
													
													/* Clear the list entry. */
													xKnownDeviceList[ uxListIdx ].bUsed = false;
												}
											}
											
											/* Give mutex for xKnownDeviceList. */
											xSemaphoreGive( xMutexBleAdvData );

											break;
											
				default:					break;
			}
		}
    }

    /* Tasks must be implemented to never return... */
}
/*-----------------------------------------------------------*/
