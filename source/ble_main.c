/*
 * Tracker Firmware
 *
 * BLE Main
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>

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

#define NRF_LOG_MODULE_NAME 		BLE_MAIN
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
#include "drv_adc.h"
#include "drv_uart.h"

#include "ble_adreport.h"
#include "ble_ctrl.h"
#include "ble_main.h"
#include "config.h"
#include "main.h"
#include "rtc.h"
#include "trace.h"
#include "tracemsg.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Initialise the BLE task. */
void vBleMainInit( void );

/* Startup function. */
static void vStartUp( void * pvParameter );

/* Function for handling BLE_GAP_ADV_REPORT events. */
static void vOnAdvReport( ble_gap_evt_adv_report_t const *pxAdvReport );

/* Check if the beacon's UUID is in the list of configured UUID filters. */
static bool bFilterUUID( unsigned char *pucBeaconUuid, unsigned char *pucUuidFilterMatchIdx );

/* Function for handling BLE Stack events. */
static void vBleEvtHandler( ble_evt_t const *pxBleEvt, void *pvContext );

/* Initialise the SoftDevice and the BLE event interrupt. */
static void vBleStackInit( void );

/* This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 * device including the device name and the preferred connection parameters. */
static void vGapParamsInit( void );

/* Start advertising. */
void vStartAdvertising( portBASE_TYPE xNewAdvState );

/* Stop advertising. */
void vStopAdvertising( void );

/* Start advertising. */
void vStartScan( portBASE_TYPE xNewScanState );

/* Stop advertising. */
void vStopScan( void );

/* Set RSSI filter algorithm configuration. */
void vSetRssiCfg( unsigned portBASE_TYPE uxFilterMethod, unsigned portBASE_TYPE uxFilterWindow );

/* Set TX power. */
void vSetTxPower( enum xIF_TYPE xRfInterface, unsigned portBASE_TYPE uxTxPower );

/* Set the advertising interval. */
void vSetAdvInterval( enum xIF_TYPE xRfInterface, unsigned long ulAdvInterval );

/* Set the advertising payload: Initialise. 
   Fill the first three bytes of the pyload with the AD flag. */
void vSetAdvPayloadInit( enum xIF_TYPE xRfInterface );

/* Set the advertising payload. */
void vSetAdvPayload( enum xIF_TYPE xRfInterface, unsigned char *pucAdvPayload, unsigned portBASE_TYPE uxAdvPayloadLen );

/* Timer callback for periodic BLE beacon scans. */
static void vBleAdvTimerCallback( TimerHandle_t xBleAdvTimer );
/*-----------------------------------------------------------*/

/* Local variables. */
/* Timer handle for the BLE advertising timer. */
static TimerHandle_t 		xBleAdvTimer;	

/* Buffer where advertising reports will be stored by the SoftDevice. */
static uint8_t				cScanBufferData[ BLE_GAP_SCAN_BUFFER_EXTENDED_MIN ]; 

/* Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t xScanParam =
{
	.extended      = 1,
	.active        = 0x00,							/* Passive scan only. */
	.interval      = SCAN_INTERVAL,
    .window        = SCAN_WINDOW_MONO,				/* Prepare for mono (i.e. 1Mbps only as opposed to simultaneous) scan. */
    .timeout       = 0x0000,						/* No timeout. */
    .scan_phys     = BLE_GAP_PHY_1MBPS,				/* 1Mbps PHY only. */
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
};

/* Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
static ble_data_t			xScanBuffer =
{
    cScanBufferData,
    BLE_GAP_SCAN_BUFFER_EXTENDED_MIN
};

/* Advertising state:
		bit 0				1Mbps advertising configured
		bit 2				125kbps advertising configured */
static portBASE_TYPE		xAdvState;

/* Current advertising mode (exclusive):
		bit 0				1Mbps advertising ongoing
		bit 2				125kbps advertising ongoing */
static portBASE_TYPE		xAdvMode;

/* Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t	xAdvData =
{
    .adv_data =
    {
        .p_data = cEncodedStd1MbpsAdvData,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};

/* Advertisement parameter collection. */
ble_gap_adv_params_t		xAdvParams =
{
	.properties				=
	{
	  .type					= BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED,
	  .anonymous			= 0
	},
	.p_peer_addr			= NULL,
	.filter_policy			= BLE_GAP_ADV_FP_ANY,
	.interval				= BLE_ADV_INTVL_STD,
	.duration				= 0,

	.primary_phy			= BLE_GAP_PHY_1MBPS,		
	.secondary_phy			= BLE_GAP_PHY_1MBPS,
	.scan_req_notification	= 0,
};

/* Handle for advertisement. */
static uint8_t				xAdvHandle;

/* Scan state:
		bit 0				1Mbps scanning
		bit 2				125kbps scanning */
static portBASE_TYPE		xScanState;

/* List of known devices. */
struct KNOWN_DEVICE			xKnownDeviceList[ KNOWN_DEVICE_LIST_LEN ];

/* Error string. */
signed char					cErrorStrg[ 60 ];		

/* On-duration counters for the BLE RF module. The counter is reset every time a packet has been 
   sent to the server using the interface function. */
TickType_t					xBleOnTimeStamp;
TickType_t					xBleOnDuration;
/*-----------------------------------------------------------*/

/* Public variables. */
/* Mutex handle to protect access to BLE configuration. */
SemaphoreHandle_t			xMutexBleConfig;

/* Mutex to protect access to the BLE advertiser data in xKnownDeviceList[]. */
SemaphoreHandle_t			xMutexBleAdvData;
	
	/* Advertising data. */
signed char					cEncodedStd1MbpsAdvData[ 100 ];
portBASE_TYPE				xEncodedStd1MbpsAdvDataLen;
signed char					cEncodedLR125kbpsAdvData[ 257 ];
portBASE_TYPE				xEncodedLR125kbpsAdvDataLen;

/* Configured output power in different modes. */
uint8_t						cTxPower1Mbps;
uint8_t						cTxPower125kbps;

/* Advertising intervals for long and short range. */
uint32_t					ul1MbpsAdvInterval;
uint32_t					ul125kbpsAdvInterval;

/* Enable for UUID filtering of received iBeacons. */
bool						bBleUuidFilterEnable;
/*-----------------------------------------------------------*/

/* 
 * Initialise the BLE task. 
 */
void vBleMainInit( void )
{
    unsigned portBASE_TYPE    uxListIdx;

	/* Initialise the BLE stack. The task for handling SoftDevice events is spawned in here. */
	#if !defined ( NO_SOFTDEVICE )
		vBleStackInit();
	#endif
	
	/* Create advertisement timer. */
	xBleAdvTimer = xTimerCreate
							( "ADV",		 				/* Timer name for debug. */
							  ADV_INTERLEAVING_TIME	,		/* BLE switch rate between advertising modes. */
							  pdTRUE,						/* Auto-reload. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  vBleAdvTimerCallback			/* Callback for the BLE timer. */
							);

	/* Create a mutex to protect access to the BLE configuration. */
	xMutexBleConfig = xSemaphoreCreateMutex();
	
	/* Create a mutex to protect access to the BLE advertiser data in xKnownDeviceList[]. */
	xMutexBleAdvData = xSemaphoreCreateMutex();
	
	/* No advertisement data set yet. */
	xEncodedStd1MbpsAdvDataLen = 0;
	xEncodedLR125kbpsAdvDataLen = 0;
	
	/* Reset the list of known BLE devices. */
	for ( uxListIdx = 0; uxListIdx < RSSI_LIST_LEN; uxListIdx++ )
	{
		xKnownDeviceList[ uxListIdx ].bUsed = false;
	}
	
	/* Initialise BLE on-duration counter. */
	xBleOnTimeStamp = 0;
	xBleOnDuration = 0;

	/* Preliminary initialisation of the UUID filter enable variable. 
	   The variable will be finally initialised once the NVDS module has loaded
	   the configuration record. */
	bBleUuidFilterEnable = true;

	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* 
 * Startup function.
 */
static void vStartUp( void * pvParameter )
{
    ( void )pvParameter;
	
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/*
 * Store the advertiser in a table. 
 * The table is read by the periodically called prvBleFilterTimerCallback() function
 * which determines if an advertiser needs to be sent to the host and its entry 
 * deleted.
 */
void vStoreAdvertiser( ble_gap_evt_adv_report_t xAdvReport, enum xBLE_BCN_FORMAT xBcnFormat, unsigned char ucUuidFilterMatchIdx )
{
	unsigned portBASE_TYPE	uxListIdx;
	unsigned portBASE_TYPE	uxListIdx2;
	bool					bFound;
	bool					bDeviceToReport;
	
	bDeviceToReport = false;
	uxListIdx = 0;
	bFound = false;
	
	/* Take mutex as xKnownDeviceList is also accessed by AD handler. */
	if ( !xSemaphoreTake( xMutexBleAdvData, BLE_OS_TIMEOUT ) )
	{
		/* Could not obtain the mutex. Now just ignore the report. */			
		return;
	}
	
	/* Check if the device is already in the list of known devices. */
	do
	{
		if ( xKnownDeviceList[ uxListIdx ].bUsed )
		{
			if (   ( memcmp( xKnownDeviceList[ uxListIdx ].pucPeerAddr, xAdvReport.peer_addr.addr, 6 ) == 0 )
				&& ( xKnownDeviceList[ uxListIdx ].ucPrimaryPhy == xAdvReport.primary_phy ) )
			{
				bFound = true;
			}
			else
			{
				uxListIdx++;
			}
		}
		else
		{
			uxListIdx++;
		}
	}
	while ( !bFound && ( uxListIdx < KNOWN_DEVICE_LIST_LEN ) ) ;
	
	if ( bFound )
	{
		/* The device in already in the list. Find the next free entry in the list of already received RSSIs and add the new one. */
		/* Wipe RSSI list and add RSSI. */
		uxListIdx2 = 0;
		do
		{
			bFound = ( xKnownDeviceList[ uxListIdx ].xRSSI[ uxListIdx2++ ] == 0 );
		}
		while ( !bFound && ( uxListIdx2 < RSSI_LIST_LEN ) );
		
		if ( bFound ) 
		{
			xKnownDeviceList[ uxListIdx ].xRSSI[ --uxListIdx2 ] = xAdvReport.rssi;
		}
	}
	else
	{
		/* The device has not been found in the list. Check, if there is a free slot in the list. */
		uxListIdx = 0;
		
		do 
		{
			bFound = !xKnownDeviceList[ uxListIdx++ ].bUsed;
		}
		while ( !bFound && ( uxListIdx < KNOWN_DEVICE_LIST_LEN ) ) ;
		
		if ( bFound )
		{
			uxListIdx--;
			
			/* Not in list and still room in the list. */
			memcpy( xKnownDeviceList[ uxListIdx ].pucPeerAddr, xAdvReport.peer_addr.addr, BLE_GAP_ADDR_LEN );
			xKnownDeviceList[ uxListIdx ].xRxTimeStamp 			= xTaskGetTickCount();
			xKnownDeviceList[ uxListIdx ].xBcnFormat    		= xBcnFormat;
			xKnownDeviceList[ uxListIdx ].ucUuidFilterMatchIdx  = ucUuidFilterMatchIdx;
			xKnownDeviceList[ uxListIdx ].usDataLen 			= xAdvReport.data.len;
			xKnownDeviceList[ uxListIdx ].ucPrimaryPhy			= xAdvReport.primary_phy;
			/* Copy the advertiser data. */
			memcpy( xKnownDeviceList[ uxListIdx ].pucAdvData, xAdvReport.data.p_data, xAdvReport.data.len );
		
			/* Wipe RSSI list and add RSSI. */
			for ( uxListIdx2 = 1; uxListIdx2 < RSSI_LIST_LEN; uxListIdx2++ )
			{
				xKnownDeviceList[ uxListIdx ].xRSSI[ uxListIdx2 ] = 0;
			}
			xKnownDeviceList[ uxListIdx ].xRSSI[ 0 ] = xAdvReport.rssi ;

            xKnownDeviceList[ uxListIdx ].bUsed = true;
		}
		else
		{
			/* Not in list and list is full. Nothing we can do here but to trash the data. */
		}
	}
	
	/* Give mutex for xKnownDeviceList. */
	xSemaphoreGive( xMutexBleAdvData );
}
/*-----------------------------------------------------------*/

/* Check if the UUID filter is enabled and fill in the bBleUuidFilterEnable variable. */
void bCheckAndSetUUIDFilterEnabled( void )
{
	unsigned char				ucFilterIdx;

	bBleUuidFilterEnable = false;

	for ( ucFilterIdx = 0; ( ucFilterIdx < BLE_NUM_UUID_FILTER ) && !bBleUuidFilterEnable; ucFilterIdx++ )
	{
		/* Check if there is at least one filter which is not all 0's. */
		if ( !( bMemVCmp( xNvdsConfig.ucBleBeaconUuidFilter[ ucFilterIdx ], 0, BLE_LEN_UUID_FILTER ) ) )
		{
			bBleUuidFilterEnable = true;
			return;
		}
	}
}
/*-----------------------------------------------------------*/

/* Check if the beacon's UUID is in the list of configured UUID filters. */
static bool bFilterUUID( unsigned char *pucBeaconUuid, unsigned char *pucUuidFilterMatchIdx )
{
	unsigned char				ucFilterIdx;

	if ( bBleUuidFilterEnable )
	{
		for ( ucFilterIdx = 0; ucFilterIdx < BLE_NUM_UUID_FILTER; ucFilterIdx++ )
		{
			/* Only check against the configured UUID if the configured UUID is not all 0. */
			if ( !( bMemVCmp( xNvdsConfig.ucBleBeaconUuidFilter[ ucFilterIdx ], 0, BLE_LEN_UUID_FILTER ) ) )
			{
				/* The filter has been configured. Check if it is all 0xFF, meaning a pass-through. */
				if ( bMemVCmp( xNvdsConfig.ucBleBeaconUuidFilter[ ucFilterIdx ], 0xff, BLE_LEN_UUID_FILTER ) )
				{
					/* The filter is a pass-though, so return 'true' immediately. */
					*pucUuidFilterMatchIdx = ucFilterIdx;
					return true;
				}

				/* The filter is configure with a specific value. Check the received UUID against it. */
				if ( memcmp( pucBeaconUuid,  xNvdsConfig.ucBleBeaconUuidFilter[ ucFilterIdx ], BLE_LEN_UUID_FILTER ) == 0 )
				{
					*pucUuidFilterMatchIdx = ucFilterIdx;
					return true;
				}
			}
		}
	}

	*pucUuidFilterMatchIdx = 0;
	return false;
}
/*-----------------------------------------------------------*/

/* Function for handling BLE Stack events.
   Parameters:	pxBleEvt  Bluetooth stack event.

   BLE advertising reports are filtered for supported beacons.

   Advertising data format for localiser beacons:
		iBeacons:         	  02 01 06 1A FF 4C00 0215 80A9059AAD4C488BA267 DD7F2A389675 0000 0000 C3
		Swissphone Beacons:   02 01 06 1B FF 4C00 0215 05A106683FC743B0B6830B1E6A06F73D  02AC 1F52 B9 64
		Radius beacons:   	  02 01 06 1B FF 1801 BEAC 80A9059AAD4C488BA267 CAF3E52DEEAB 0002 0000 C3 26
			                  |        |     |    |    |                    |            |    |    |  |
			                  |        |     |    |    |                    |            |    |    |  30: MFG reserved (battery, 0x26 = 38%)
			                  |        |     |    |    |                    |            |    |    29: reference RSSI (0xC5 = -59dBm)
			                  |        |     |    |    |                    |            |    27: Minor
			                  |        |     |    |    |                    |            25: Major (BcnUse, 0x0002 = BCNUSE_NO_ABNORMAL)
							  |	  	   |     |	  |    |				    19: copied version of beacon advertiser address
			                  |        |     |    |    9: beacon ID                                        
			                  |        |     |    7: altBeacon code (0xBEAC)                                   
			                  |        |     5: MfgID (Radius Networks = 0x1801, Apple = 0x4C00)
			                  |        3: AD field length and identifier for manufacturer specific data (here: 1B = altBeacon, 1A = iBeacon)
							  0: advertiser payload flags
		
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
static void vBleEvtHandler( ble_evt_t const *pxBleEvt, void *pvContext )
{
	ret_code_t					xErrCode;
	ble_gap_evt_t const			*pxGapEvt = &pxBleEvt->evt.gap_evt;
	ble_gap_evt_adv_report_t	xAdvReport = pxGapEvt->params.adv_report;
	const char					uTLSignature[]               =       { 0xFF, 0x17, 0x00 };
	const char					cRadiusSignature[]           = { 0x1B, 0xFF, 0x18, 0x01, 0xBE, 0xAC, 0x80, 0xA9, 0x05, 0x9A, 0xAD, 0x4C, 0x48, 0x8B, 0xA2, 0x67 };
	const char					cIBeaconSignature[]          = { 0x1A, 0xFF, 0x4C, 0x00, 0x02, 0x15, 0x80, 0xA9, 0x05, 0x9A, 0xAD, 0x4C, 0x48, 0x8B, 0xA2, 0x67 };
	const char					cSwissPhoneBeaconSignature[] =       { 0xFF, 0x4C, 0x00, 0x02, 0x15 };
	enum xBLE_BCN_FORMAT 		xBcnFormat;
	bool						bAdvReportTL; 		
	bool						bAdvReportRadius;		
	bool						bAdvReportIBeacon;		
	bool						bAdvReportSwissPhoneBeacon;		
	unsigned char 				ucUuidFilterMatchIdx;

    switch ( pxBleEvt->header.evt_id )
    {
        case BLE_GAP_EVT_ADV_REPORT:

			/* Initialise the beacon report match results. Distress beacons, AltBecons and TRAXxs-issued iBeacons are always supported. */
			ucUuidFilterMatchIdx	   = 0;
		
			/* Check if the adv report is sent from one of the recognised devices. 
			   The filtering is in the order of increased filtering complexity. */
			if (    ( memcmp( uTLSignature, xAdvReport.data.p_data + 4, sizeof( uTLSignature ) ) == 0 )
			     && ( xAdvReport.data.len >= 24 ) )
			{
				xBcnFormat = BLE_DISTRESS_BEACON;
			}
			else
			{
				if (    ( memcmp( cRadiusSignature, xAdvReport.data.p_data + 3, sizeof( cRadiusSignature ) ) == 0 )
 			         && ( xAdvReport.data.len == 31 ) )
				{
					xBcnFormat = BLE_ALTBEACON;
				}
				else
				{
					if (    ( memcmp( cIBeaconSignature, xAdvReport.data.p_data + 3, sizeof( cIBeaconSignature ) ) == 0 )
 			             && ( xAdvReport.data.len == 30 ) )
					{
						xBcnFormat = BLE_IBEACON;
					}
					else
					{
						if (    ( memcmp( cSwissPhoneBeaconSignature, xAdvReport.data.p_data + 4, sizeof( cSwissPhoneBeaconSignature ) ) == 0 )
 			             	 && ( ( xAdvReport.data.len == 30 ) || ( xAdvReport.data.len == 31 ) ) )
						{
							if ( bFilterUUID( xAdvReport.data.p_data + 9, &ucUuidFilterMatchIdx ) )
							{
								xBcnFormat = BLE_SWISSPHONE_BEACON;

							}
							else
							{
								xBcnFormat = BLE_BEACON_NONE;
							}
						}						
					}
				}
			}

			if ( xBcnFormat != BLE_BEACON_NONE )
			{   
				/* Store the device in the list of known devices. */
				vStoreAdvertiser( xAdvReport, xBcnFormat, ucUuidFilterMatchIdx );
			}
						
			/* Continue scanning. */
			xErrCode = sd_ble_gap_scan_start( NULL, &xScanBuffer );
			if ( xErrCode == NRF_ERROR_INVALID_STATE )
			{
				NRF_LOG_WARNING( "%i vBleEvtHandler: sd_ble_gap_scan_start returned NRF_ERROR_INVALID_STATE.", ulReadRTC() );
			}
			else
			{
				APP_ERROR_CHECK_ATIF( "ERROR scan continue: 0x%x", xErrCode );
			}
			
            break;
        
        default:
        
			NRF_LOG_INFO( "%i Received an unimplemented BLE event.", ulReadRTC() );
			NRF_LOG_FLUSH();
            /* No implementation needed. */
			break;
    }
}
/*-----------------------------------------------------------*/

/*
 * Initialise the SoftDevice and the BLE event interrupt.
 */
static void vBleStackInit( void )
{
    ret_code_t		xErrCode;
    uint32_t		ulRamStart;
	ble_gap_addr_t	xBleGapAddr = 	
					{
						.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC,
						.addr = { 0x00, 0x00, 0x00, 0x6E, 0xCA, 0xD4 }
					};
	uint8_t			cRngBytesAvailable;
	signed char		cDevAddrStrg[ 18 ];
	uint32_t		uiIdx;
	
	/* Initialise the output power to default values. */
	cTxPower1Mbps = OUTPUT_PWR_6_dBm;
	cTxPower125kbps = OUTPUT_PWR_6_dBm;
 	
	/* Set advertisement interval to default. */
	ul1MbpsAdvInterval = BLE_ADV_INTVL_STD;
	ul125kbpsAdvInterval = BLE_ADV_INTVL_STD;

    xErrCode = nrf_sdh_enable_request();
    APP_ERROR_CHECK_ATIF( "ERROR enabling SDH: 0x%x", xErrCode );

    /* Configure the BLE stack using the default settings.
       Fetch the start address of the application RAM. */
    ulRamStart = 0;
    xErrCode = nrf_sdh_ble_default_cfg_set( APP_BLE_CONN_CFG_TAG, &ulRamStart );
    APP_ERROR_CHECK_ATIF( "ERROR configuring BLE stack: 0x%x", xErrCode );

    /* Enable BLE stack. */
    xErrCode = nrf_sdh_ble_enable( &ulRamStart );
    APP_ERROR_CHECK_ATIF( "ERROR enabling BLE: 0x%x", xErrCode );

    /* Register a handler for BLE events. */
    NRF_SDH_BLE_OBSERVER( m_ble_observer, APP_BLE_OBSERVER_PRIO, vBleEvtHandler, NULL );
	
	/* Set the device's advertiser address. To remain compatible with ublox devices containing connectivity software,
	   we will set the upper 24bit to ublox manufacturer ID and take the lower 24bit from the the least significant 24-bit 
	   of the (unique) device address. */	
	xBleGapAddr.addr[ 0 ] = ( char )( ( NRF_FICR->DEVICEADDR[ 0 ] & 0x000000ff ) >>  0 );
	xBleGapAddr.addr[ 1 ] = ( char )( ( NRF_FICR->DEVICEADDR[ 0 ] & 0x0000ff00 ) >>  8 );
	xBleGapAddr.addr[ 2 ] = ( char )( ( NRF_FICR->DEVICEADDR[ 0 ] & 0x00ff0000 ) >> 16 );
	
	xErrCode = sd_ble_gap_addr_set( &xBleGapAddr );
    APP_ERROR_CHECK_ATIF( "ERROR while setting GAP advertiser address: 0x%x", xErrCode );
	
	for ( uiIdx = 0; uiIdx < 6; uiIdx++ )
	{
		vByteToHexStrg( cDevAddrStrg + 3 * uiIdx, xBleGapAddr.addr[ 5 - uiIdx ] );
		cDevAddrStrg[ 3 * uiIdx + 2 ] = ':';
	}
	cDevAddrStrg[ 3 * 5 + 2 ] = 0;
	NRF_LOG_INFO( "BLE device address: %s", cDevAddrStrg );
	NRF_LOG_FLUSH();
	
    /* Create a FreeRTOS task to retrieve SoftDevice events. The first parameter is an optional pointer to a function
  	   to be executed */
    nrf_sdh_freertos_init( vStartUp, NULL, SD_TASK_PRIORITY );	
}
/*-----------------------------------------------------------*/

/* Start advertising. */
void vStartAdvertising( portBASE_TYPE xNewAdvState )
{
	ret_code_t	xErrCode;
	int8_t		cTxPower;

	#if defined ( NO_SOFTDEVICE )
		( void )xErrCode;
		( void )cTxPower;
		return;
	#endif
	
	/* Request access to BLE configuration. */
	xSemaphoreTake( xMutexBleConfig, BLE_OS_TIMEOUT );
	
	/* Configure the advertisement:
	   Advertisement data is in xAdvData.
	   Advertiser physical configuration is in xAdvParams. */
	NRF_LOG_DEBUG( "%i Advertising mode set to %i.", ulReadRTC(), xNewAdvState );	   
	NRF_LOG_FLUSH();
	
	/* Set the advertising PHY and TX power as required. */
	if ( xNewAdvState == ADV_LR125KBPS )
	{
		/* Long Range only: Set the PHY to 125kbps. */
		xAdvParams.primary_phy = BLE_GAP_PHY_CODED;
		xAdvParams.secondary_phy = BLE_GAP_PHY_CODED;
		xAdvParams.interval = ul125kbpsAdvInterval;
		cTxPower = cTxPower125kbps;		
		xAdvMode = ADV_LR125KBPS;
		xAdvData.adv_data.p_data = cEncodedLR125kbpsAdvData;
		xAdvData.adv_data.len = xEncodedLR125kbpsAdvDataLen; 
		
		/* For long range, always Use extended advertising. */
		xAdvParams.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
	}
	else
	{
		if ( xNewAdvState & ADV_STD1MBPS )
		{
			/* In case standard advertising is set alone or combined with long range, start with the advertising 1Mbps PHY. */
			xAdvParams.primary_phy = BLE_GAP_PHY_1MBPS;
			xAdvParams.secondary_phy = BLE_GAP_PHY_1MBPS;
			xAdvParams.interval = ul1MbpsAdvInterval;
			cTxPower = cTxPower1Mbps;	
			xAdvMode = ADV_STD1MBPS;
			xAdvData.adv_data.p_data = cEncodedStd1MbpsAdvData;
			xAdvData.adv_data.len = xEncodedStd1MbpsAdvDataLen; 
		
			if ( xEncodedStd1MbpsAdvDataLen > 31 )
			{
				/* The payload length exceeds the capacity of a standard packet or long range is required. 
				   Use extended advertising. */
				xAdvParams.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
			}
			else
			{
				/* Use standard advertising. */
				xAdvParams.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
			}		
		}
	}

	/* Stop any ongoing advertising before starting another with different parameters. */
	( void )sd_ble_gap_adv_stop( xAdvHandle );
	
	xAdvState = xNewAdvState;

	if ( xNewAdvState != ADV_NONE )
	{
		/* Start the BLE advertising timer. On expiry, the timer switch between LR and sort range advertising. */
		( void )xTimerStart( xBleAdvTimer, BLE_OS_TIMEOUT );	

		/* Configure GAP advertising. */
		xErrCode = sd_ble_gap_adv_set_configure( &xAdvHandle, &xAdvData, &xAdvParams );
		APP_ERROR_CHECK_ATIF( "ERROR while configuring advertising: 0x%x", xErrCode );
		
		/* Set the output power. */
		xErrCode = sd_ble_gap_tx_power_set( BLE_GAP_TX_POWER_ROLE_ADV, xAdvHandle, cTxPower );
		APP_ERROR_CHECK_ATIF( "ERROR while setting TX power: 0x%x", xErrCode );

		/* Start the advertising. BLE_CONN_CFG_TAG_DEFAULT is ignored as this is non-connectable advertising. */
		xErrCode = sd_ble_gap_adv_start( xAdvHandle, BLE_CONN_CFG_TAG_DEFAULT );
		APP_ERROR_CHECK_ATIF( "ERROR while starting advertising: 0x%x", xErrCode );

		NRF_LOG_DEBUG( "%i Advertising started.", ulReadRTC() );
		NRF_LOG_FLUSH();
	}
	else
	{
		( void )xTimerStop( xBleAdvTimer, BLE_OS_TIMEOUT );	
	}
		
	/* Release access to BLE configuration. */
	xSemaphoreGive( xMutexBleConfig );
}
/*-----------------------------------------------------------*/

/* Stop advertising. */
void vStopAdvertising( void )
{
	ret_code_t	xErrCode;

	#if defined ( NO_SOFTDEVICE )
		return;
	#endif
	
	NRF_LOG_DEBUG( "%i Advertising stopped.", ulReadRTC() );	   
	NRF_LOG_FLUSH();

	( void )xTimerStop( xBleAdvTimer, BLE_OS_TIMEOUT );		

	if ( xAdvState != ADV_NONE )
	{
		/* Request access to BLE configuration. */
		xSemaphoreTake( xMutexBleConfig, BLE_OS_TIMEOUT );

		xErrCode = sd_ble_gap_adv_stop( xAdvHandle );
		APP_ERROR_CHECK_ATIF( "ERROR while stopping advertising: 0x%x", xErrCode );

		/* Release access to BLE configuration. */
		xSemaphoreGive( xMutexBleConfig );
		
		xAdvState = ADV_NONE;
	}
}
/*-----------------------------------------------------------*/

/* Start scanning with the given parameter. */
void vStartScan( portBASE_TYPE xNewScanState )
{
	ret_code_t				xErrCode;
    unsigned portBASE_TYPE	uxListIdx;
	
	#if defined ( NO_SOFTDEVICE )
		( void )xErrCode;
	    ( void )uxListIdx;
		return;
	#endif

	/* Request access to BLE configuration. */
	xSemaphoreTake( xMutexBleConfig, BLE_OS_TIMEOUT );
	
	NRF_LOG_DEBUG( "%i Scanning mode set to %i.", ulReadRTC(), xNewScanState );	   
	NRF_LOG_FLUSH();
	
	xScanParam.scan_phys = 0;
	
	if ( xNewScanState & SCAN_STD1MBPS )
	{
		xScanParam.scan_phys |= BLE_GAP_PHY_1MBPS;
	}
	if ( xNewScanState & SCAN_LR125KBPS )
	{
		xScanParam.scan_phys |= BLE_GAP_PHY_CODED;
	}
	
	/* Adapt the scan window to the scan type (mono scan or simultaneous scan). */
	if ( xScanParam.scan_phys == ( BLE_GAP_PHY_CODED | BLE_GAP_PHY_1MBPS ) )
	{
		xScanParam.window = SCAN_WINDOW_DUAL;
	}
	else
	{
		xScanParam.window = SCAN_WINDOW_MONO;
	}
	
	/* If already scanning, stop scanning. */
	( void )sd_ble_gap_scan_stop();
   
	/* Start scanning. */
	xErrCode = sd_ble_gap_scan_start( &xScanParam, &xScanBuffer );
	APP_ERROR_CHECK_ATIF( "ERROR while starting scan: 0x%x", xErrCode );
		
	/* Release access to BLE configuration. */
	xSemaphoreGive( xMutexBleConfig );
	
	/* Remember the time stamp when the BLE scanning was started. */
	if ( xScanState == SCAN_NONE )
	{
		xBleOnTimeStamp = xTaskGetTickCount();	
	}
	
	xScanState = xNewScanState;
	
	/* Reset the list of known BLE devices. */
	for ( uxListIdx = 0; uxListIdx < RSSI_LIST_LEN; uxListIdx++ )
	{
		xKnownDeviceList[ uxListIdx ].bUsed = false;
	}
	
	( void )xTimerStart( xBleFilterTimer, BLE_OS_TIMEOUT );
}
/*-----------------------------------------------------------*/

/* Stop scanning. */
void vStopScan( void )
{
	ret_code_t				xErrCode;

	#if defined ( NO_SOFTDEVICE )
		return;
	#endif

	( void )xTimerStop( xBleFilterTimer, BLE_OS_TIMEOUT );

	if ( xScanState != SCAN_NONE )
	{
		/* Request access to BLE configuration. */
		xSemaphoreTake( xMutexBleConfig, BLE_OS_TIMEOUT );

		/* Stop scanning. */
		NRF_LOG_DEBUG( "%i Scanning stopped.", ulReadRTC() );	   
		NRF_LOG_FLUSH();
		xErrCode = sd_ble_gap_scan_stop();
		APP_ERROR_CHECK_ATIF( "ERROR while stoping scan: 0x%x", xErrCode );

		/* Accumulated the time duration during which the module was running. */
		xBleOnDuration += xTaskGetTickCount() - xBleOnTimeStamp;

		/* Release access to BLE configuration. */
		xSemaphoreGive( xMutexBleConfig );

		xScanState = SCAN_NONE;
	}	
}
/*-----------------------------------------------------------*/

/* Set RSSI filter algorithm configuration. */
void vSetRssiCfg( unsigned portBASE_TYPE uxFilterMethod, unsigned portBASE_TYPE uxFilterWindow )
{
	uxRssiFilterMethod = uxFilterMethod;
	uxRssiFilterWindow = uxFilterWindow;
}
/*-----------------------------------------------------------*/

/* Set TX power. */
void vSetTxPower( enum xIF_TYPE xRfInterface, unsigned portBASE_TYPE uxTxPower )
{
	if ( xRfInterface == SR )
	{
		cTxPower1Mbps = uxTxPower;
	}
	else
	{
		if ( xRfInterface == LR )
		{
			cTxPower125kbps = uxTxPower;
		}
	}
}
/*-----------------------------------------------------------*/

/* Set the advertising interval. */
void vSetAdvInterval( enum xIF_TYPE xRfInterface, unsigned long ulAdvInterval )
{
	if ( xRfInterface == SR )
	{
		ul1MbpsAdvInterval = ulAdvInterval;
	}
	else
	{
		if ( xRfInterface == LR )
		{
			ul125kbpsAdvInterval = ulAdvInterval;
		}
	}
}
/*-----------------------------------------------------------*/

/* Set the advertising payload: Initialise. 
   Fill the first three bytes of the payload with the AD flag. */
void vSetAdvPayloadInit( enum xIF_TYPE xRfInterface )
{
	if ( xRfInterface == SR )
	{
		cEncodedStd1MbpsAdvData[ 0 ] = 0x02;			/* AD length: 2 bytes. */
		cEncodedStd1MbpsAdvData[ 1 ] = 0x01;			/* AD Type: 1 = advertiser flags. */
		cEncodedStd1MbpsAdvData[ 2 ] = 0x04;			/* AD Flag value: not discoverable, BR/EDR not supported. */
		xEncodedStd1MbpsAdvDataLen = 3;
	}
	else
	{
		if ( xRfInterface == LR )
		{
			cEncodedLR125kbpsAdvData[ 0 ] = 0x02;		/* AD length: 2 bytes. */
			cEncodedLR125kbpsAdvData[ 1 ] = 0x01;		/* AD Type: 1 = advertiser flags. */
			cEncodedLR125kbpsAdvData[ 2 ] = 0x04;		/* AD Flag value: not discoverable, BR/EDR not supported. */
			xEncodedLR125kbpsAdvDataLen = 3;
		}
	}
}
/*-----------------------------------------------------------*/

/* Set the advertising payload: Continue writing the payload. */
void vSetAdvPayload( enum xIF_TYPE xRfInterface, unsigned char *pucAdvPayload, unsigned portBASE_TYPE uxAdvPayloadLen )
{
	if ( xRfInterface == SR )
	{
		memcpy( cEncodedStd1MbpsAdvData + xEncodedStd1MbpsAdvDataLen, pucAdvPayload, uxAdvPayloadLen );
		xEncodedStd1MbpsAdvDataLen += uxAdvPayloadLen;
	}
	else
	{
		if ( xRfInterface == LR )
		{
			memcpy( cEncodedLR125kbpsAdvData + xEncodedLR125kbpsAdvDataLen, pucAdvPayload, uxAdvPayloadLen );
			xEncodedLR125kbpsAdvDataLen += uxAdvPayloadLen;
		}
	}
}
/*------------------------------------------------

/* Return the BLE cumulative on-duration. */
TickType_t xGetBleOnDuration( void )
{
	if ( xScanState == SCAN_NONE )
	{
		return xBleOnDuration;
	}
	else
	{
		return ( xBleOnDuration + xTaskGetTickCount() - xBleOnTimeStamp );
	}
}
/*-----------------------------------------------------------*/

/* Return the BLE functional state. */
void vResetBleOnDuration( TickType_t xResetTargetValue )
{
	if ( xScanState != SCAN_NONE )
	{
		xBleOnDuration = xResetTargetValue;
		xBleOnTimeStamp = xTaskGetTickCount();
	}
	else
	{
		/* Else, just set it to 0. */
		xBleOnDuration = 0;
	}
}
/*-----------------------------------------------------------*/

/* Timer callback for periodic BLE beacon scans. 
   CAUTION: This function is running in the timer task!
*/
static void vBleAdvTimerCallback( TimerHandle_t xTimer )
{
	ret_code_t	xErrCode;
	int8_t		cTxPower;

	( void )xTimer;
	
	/* Test for concurrent standard / long range advertising requirement. */
	if ( xAdvState == ( ADV_STD1MBPS | ADV_LR125KBPS ) )
	{
		/* Request access to BLE configuration. */
		xSemaphoreTake( xMutexBleConfig, BLE_OS_TIMEOUT );

		/* Switch between modes. */
		if ( xAdvMode == ADV_STD1MBPS )
		{
			/* switch to LR. */
			NRF_LOG_DEBUG( "%i Advertising switched to long range.", ulReadRTC() );	   
			NRF_LOG_FLUSH();
			xAdvParams.primary_phy = BLE_GAP_PHY_CODED;
			xAdvParams.secondary_phy = BLE_GAP_PHY_CODED;
			xAdvParams.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
			xAdvData.adv_data.p_data = cEncodedLR125kbpsAdvData;
			xAdvData.adv_data.len = xEncodedLR125kbpsAdvDataLen; 
		
			/* For long range, always Use extended advertising. */
			cTxPower = cTxPower125kbps;		
			xAdvMode = ADV_LR125KBPS;
		}
		else
		{
			/* Switch to standard. */
			NRF_LOG_DEBUG( "%i Advertising switched to 1Mbps.", ulReadRTC() );	   
			NRF_LOG_FLUSH();
			xAdvParams.primary_phy = BLE_GAP_PHY_1MBPS;
			xAdvParams.secondary_phy = BLE_GAP_PHY_1MBPS;
			xAdvData.adv_data.p_data = cEncodedStd1MbpsAdvData;
			xAdvData.adv_data.len = xEncodedStd1MbpsAdvDataLen; 
			
			if ( xEncodedStd1MbpsAdvDataLen > 31 )
			{
				/* The payload length exceeds the capacity of a standard packet or long range is required. 
				   Use extended advertising. */
				xAdvParams.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
			}
			else
			{
				/* Use standard advertising. */
				xAdvParams.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
			}		
			cTxPower = cTxPower1Mbps;	
			xAdvMode = ADV_STD1MBPS;
		}
		
		/* Stop any ongoing advertising before starting another with different parameters. */
		xErrCode = sd_ble_gap_adv_stop( xAdvHandle );
		APP_ERROR_CHECK_ATIF( "ERROR while stopping advertising: 0x%x", xErrCode );
	
		/* Configure advertsising. Catch error and return error as AT event. An error could mean that the 
		   advertising data is bad. An error here reboots the module.
		   In case NRF_ERROR_INVALID_LENGTH, the new data is not accepted but the module not rebooted
		   to avoid service disruption. The next time the data is updated, the error should have disappeared. */
		xErrCode = sd_ble_gap_adv_set_configure( &xAdvHandle, &xAdvData, &xAdvParams );
		if ( xErrCode != NRF_ERROR_INVALID_LENGTH )
		{
			APP_ERROR_CHECK_ATIF( "ERROR while configuring advertising: 0x%x", xErrCode );
		}
		else
		{				
			/* Trace the error. */
			V_TRACE_PRINT_LONG( TRACE_BLE_ADVERTISING_ERROR, xAdvMode, TRACE_UART_AND_FILE );
			V_TRACE_PRINT_LONG( TRACE_BLE_ADVERTISING_ERROR, xAdvData.adv_data.len, TRACE_UART_AND_FILE );
		}
		
		/* Set the output power. */
		xErrCode = sd_ble_gap_tx_power_set( BLE_GAP_TX_POWER_ROLE_ADV, xAdvHandle, cTxPower );
		APP_ERROR_CHECK_ATIF( "ERROR while setting TX power: 0x%x", xErrCode );

		/* Start the advertising. BLE_CONN_CFG_TAG_DEFAULT is ignored as this is non-connectable advertising. */
		xErrCode = sd_ble_gap_adv_start( xAdvHandle, BLE_CONN_CFG_TAG_DEFAULT );
		APP_ERROR_CHECK_ATIF( "ERROR while starting advertising: 0x%x", xErrCode );
		
		/* Release access to BLE configuration. */
		xSemaphoreGive( xMutexBleConfig );
		
		NRF_LOG_FLUSH();
	}	
}
/*-----------------------------------------------------------*/