/*
 * Tracker Firmware
 *
 * BLE task
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>


/* nRF include files */
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

#include "boards.h"

#include "nrf_log.h"
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
#include "main.h"
#include "drv_uart.h"
#include "power_sensor.h"
#include "ble_main.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Initialise the BLE task. */
void vBleInit( void );

/* Startup function. */
static void vStartUp( void * pvParameter );

/* Function for handling BLE_GAP_ADV_REPORT events. */
static void vOnAdvReport( ble_gap_evt_adv_report_t const *pxAdvReport );

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

/* Return advertising state. */
portBASE_TYPE xGetAdvState( void );

/* Timer callback for periodic BLE beacon scans. */
void prvBleAdvTimerCallback( TimerHandle_t xBleAdvTimer );
/*-----------------------------------------------------------*/

/* Local variables. */
/* Trace UART Rx character ring buffer */
signed char					cTraceUartRxBuffer[ trcUART_RX_BUFFER_SIZE ];

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
	.interval				= ADV_INTERVAL,
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
/*-----------------------------------------------------------*/

/* 
 * Initialise the BLE task. 
 */
void vBleInit( void )
{
    unsigned portBASE_TYPE    uxListIdx;

	/* Initialise the BLE stack. The task for handling SoftDevice events is spawned in here. */
	vBleStackInit();
	
	/* Create advertisement timer. */
	xBleAdvTimer = xTimerCreate
							( "",			 				/* Timer name for debug. */
							  ADV_INTERLEAVING_TIME	,		/* BLE switch rate between advertising modes. */
							  pdTRUE,						/* Auto-reload. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvBleAdvTimerCallback		/* Callback for the BLE timer. */
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
}
/*-----------------------------------------------------------*/

/* 
 * Startup function.
 */
static void vStartUp( void * pvParameter )
{
	ret_code_t xErrCode;

    ( void )pvParameter;
	
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* Function for handling BLE Stack events.
   Parameters:	pxBleEvt  Bluetooth stack event.
*/
static void vBleEvtHandler( ble_evt_t const *pxBleEvt, void *pvContext )
{
	ret_code_t					xErrCode;

    switch ( pxBleEvt->header.evt_id )
    {
        case BLE_GAP_EVT_ADV_REPORT:
		
			/* Continue scanning. */
			xErrCode = sd_ble_gap_scan_start( NULL, &xScanBuffer );
			APP_ERROR_CHECK_ATIF( "ERROR scan continue: 0x%x", xErrCode );
			
            break;
        
        default:
        
			NRF_LOG_DEBUG( "Received an unimplemented BLE event." );
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
	
	/* Initialise the output power to default values. */
	cTxPower1Mbps = OUTPUT_PWR_8_dBm;
	cTxPower125kbps = OUTPUT_PWR_8_dBm;
 	
	/* Set advertisement interval to default. */
	ul1MbpsAdvInterval = ADV_INTERVAL;
	ul125kbpsAdvInterval = ADV_INTERVAL;

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
	xBleGapAddr.addr[0] = ( char )( ( NRF_FICR->DEVICEADDR[0] & 0x000000ff ) >>  0 );
	xBleGapAddr.addr[1] = ( char )( ( NRF_FICR->DEVICEADDR[0] & 0x0000ff00 ) >>  8 );
	xBleGapAddr.addr[2] = ( char )( ( NRF_FICR->DEVICEADDR[0] & 0x00ff0000 ) >> 16 );
	
	xErrCode = sd_ble_gap_addr_set( &xBleGapAddr );
    APP_ERROR_CHECK_ATIF( "ERROR while setting GAP advertiser address: 0x%x", xErrCode );
	
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
	
	/* Request access to BLE configuration. */
	xSemaphoreTake( xMutexBleConfig, BLE_OS_TIMEOUT );
	
	/* Configure the advertisement:
	   Advertisement data is in xAdvData.
	   Advertiser physical configuration is in xAdvParams. */
	NRF_LOG_DEBUG( "Advertising mode set to %i.", xNewAdvState );	   
	
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
		/* Stop any ongoing advertising before starting another with different parameters. */
		( void ) sd_ble_gap_adv_stop( xAdvHandle );
	
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
	/* Request access to BLE configuration. */
	xSemaphoreTake( xMutexBleConfig, BLE_OS_TIMEOUT );

	NRF_LOG_DEBUG( "Advertising stopped." );	   
	( void )xTimerStop( xBleAdvTimer, BLE_OS_TIMEOUT );		
	( void )sd_ble_gap_adv_stop( xAdvHandle );
  
	xAdvState = ADV_NONE;
		
	/* Release access to BLE configuration. */
	xSemaphoreGive( xMutexBleConfig );
}
/*-----------------------------------------------------------*/

portBASE_TYPE xGetAdvState( void )
{
	return xAdvState;
}
/*-----------------------------------------------------------*/

/* Timer callback for periodic BLE beacon scans. 
   CAUTION: This function is running in the timer task!
*/
void prvBleAdvTimerCallback( TimerHandle_t xTimer )
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
			NRF_LOG_DEBUG( "Advertising switched to long range." );	   
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
			NRF_LOG_DEBUG( "Advertising switched to 1Mbps." );	   
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
		( void ) sd_ble_gap_adv_stop( xAdvHandle );
	
		/* Configure advertsising. Catch error and return error as AT event. An error could mean that the 
		   advertising data is bad. */
		xErrCode = sd_ble_gap_adv_set_configure( &xAdvHandle, &xAdvData, &xAdvParams );
		APP_ERROR_CHECK_ATIF( "ERROR while configuring advertising: 0x%x", xErrCode );
		
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
