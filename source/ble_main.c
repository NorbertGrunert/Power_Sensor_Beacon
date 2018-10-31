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
#include "ble_parser.h"
#include "ble_main.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Initialise the BLE task. */
void vBleInit( void );

/* Function to start scanning. */
static void vScanStart( void * pvParameter );

/* Function for handling BLE_GAP_ADV_REPORT events. */
static void vOnAdvReport( ble_gap_evt_adv_report_t const *pxAdvReport );

 /* Function for handling BLE Stack events. */
static void vBleEvtHandler( ble_evt_t const *pxBleEvt, void *pvContext );

/* Initialise the SoftDevice and the BLE event interrupt. */
static void vBleStackInit( void );

/* This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 * device including the device name and the preferred connection parameters. */
static void vGapParamsInit( void );

/* Timer callback for periodic BLE beacon scans. */
void prvBleTimerCallback( TimerHandle_t xTimer );

/* Start advertising. */
void vStartAdvertising( portBASE_TYPE xNewAdvState );

/* Stop advertising. */
void StopAdvertising( void );

/* Return advertising state. */
portBASE_TYPE xGetAdvState( void );

/* The BLE advertising report handling task. */
static portTASK_FUNCTION_PROTO( vBleAdHandlerTask, pvParameters );
/*-----------------------------------------------------------*/

/* Local variables. */
/* UART Rx character ring buffer */
signed char					cBleUartRxBuffer[ bleUART_RX_BUFFER_SIZE ];

/* Timer handle for the BLE timer. */
static TimerHandle_t 		xBleTimer;	

/* Buffer where advertising reports will be stored by the SoftDevice. */
static uint8_t				cScanBufferData[ BLE_GAP_SCAN_BUFFER_EXTENDED_MIN ]; 

/* Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const xScanParam =
{
    .active        = 0x00,							/* Passive scan only. */
	.interval      = SCAN_INTERVAL,
    .window        = SCAN_WINDOW,
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
		bit 0				1Mbps advertising
		bit 1				125kbps advertising ongoing */
static portBASE_TYPE		xAdvState;

/* Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t	xAdvData =
{
    .adv_data =
    {
        .p_data = cEncodedAdvData,
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
/*-----------------------------------------------------------*/

/* Public variables. */

/* Advertising data. */
signed char					cEncodedAdvData[ 100 ];
portBASE_TYPE				xEncodedAdvDataLen;

/* Configured output power in different modes. */
uint8_t						cTxPower1Mbps;
uint8_t						cTxPower125kbps;
/*-----------------------------------------------------------*/

/* 
 * Initialise the BLE task. 
 */
void vBleInit( void )
{
	/* Initialise the BLE stack. The task for handling SoftDevice events is spawned in here. */
	vBleStackInit();
	
	/* Open the COM port with the physical UART RX buffer. */
	vCOMOpen( COM0, cBleUartRxBuffer );

	/* Create advertisement timer. */
	xBleTimer = xTimerCreate
							( "",			 				/* Timer name for debug. */
							  1,							/* BLE interval dummy; will be programmed when starting the timer. */
							  pdTRUE,						/* Auto-reload. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvBleTimerCallback			/* Callback for the BLE timer. */
							);
	
	/* No advertisement data set yet. */
	xEncodedAdvDataLen = 0;
}
/*-----------------------------------------------------------*/

/* 
 * Function to start scanning.
 * Scanning is started based on the internal parameters (global variables) set.
 */
static void vScanStart( void * pvParameter )
{
	ret_code_t xErrCode;

    ( void )pvParameter;
	
	/* If already scanning, stop scanning. */
	( void )sd_ble_gap_scan_stop();
   
	/* Set the correct TX power. */
	xErrCode = sd_ble_gap_tx_power_set( BLE_GAP_TX_POWER_ROLE_SCAN_INIT, NULL, OUTPUT_PWR_4_dBm );
	APP_ERROR_CHECK( xErrCode );
 
	xErrCode = sd_ble_gap_scan_start( &xScanParam, &xScanBuffer );
	APP_ERROR_CHECK( xErrCode );
	
	/* At this point the scheduler is running and the BLE stack initialised. */
	xComSendStringRAM( COM0, "+STARTUP\r\n" );
}
/*-----------------------------------------------------------*/

/*
 * Function for handling BLE Stack events.
 * Parameters:	pxBleEvt  Bluetooth stack event.
 */
static void vBleEvtHandler( ble_evt_t const *pxBleEvt, void *pvContext )
{
	ret_code_t					xErrCode;
	ble_gap_evt_t const			*pxGapEvt = &pxBleEvt->evt.gap_evt;
	ble_gap_evt_adv_report_t	xAdvReport = pxGapEvt->params.adv_report;
	static int8_t				cRssiValue = 0;
	const char					uUbloxAddr[]  = { 0x6E, 0xCA, 0xD4 };
	const char					cRadiusAddr[] = { 0xEE, 0xF3, 0x0C };   
	bool						bAdvReportUblox; 		
	bool						bAdvReportRadius;		
	char						cAdvReportStrg[ 100 ];
	char						*pcAdvReportStrg;

    switch ( pxBleEvt->header.evt_id )
    {
        case BLE_GAP_EVT_ADV_REPORT:
		
			/* Check if the adv report is sent from one of the recognised devices. */
			bAdvReportUblox  = ( strncmp(  uUbloxAddr, xAdvReport.peer_addr.addr + 3, 3 ) == 0 );
			bAdvReportRadius = ( strncmp( cRadiusAddr, xAdvReport.peer_addr.addr + 3, 3 ) == 0 );

			 if ( ( bAdvReportUblox  == true ) ||
				  ( bAdvReportRadius == true ) )
			 {   
				cRssiValue = xAdvReport.rssi;
		
				/* Create a fake ublox scan report. */
				pcAdvReportStrg = cAdvReportStrg;
				pcAdvReportStrg += sprintf( pcAdvReportStrg, "+UBTD:%02X%02X%02X%02X%02X%02Xp,%2.2d,\"\",2,",
											xAdvReport.peer_addr.addr[ 5 ],
											xAdvReport.peer_addr.addr[ 4 ],
											xAdvReport.peer_addr.addr[ 3 ],
											xAdvReport.peer_addr.addr[ 2 ],
											xAdvReport.peer_addr.addr[ 1 ],
											xAdvReport.peer_addr.addr[ 0 ],
											cRssiValue );

				/* Append the advertising data unless ble_gap_adv_report_type_t::status is set to BLE_GAP_ADV_DATA_STATUS_INCOMPLETE_MORE_DATA */
				if ( xAdvReport.type.status != BLE_GAP_ADV_DATA_STATUS_INCOMPLETE_MORE_DATA )
				{
					for ( int idx = 0; idx < xAdvReport.data.len; idx++ )
					{
						pcAdvReportStrg += sprintf( pcAdvReportStrg, "%02X", *(xAdvReport.data.p_data + idx ) );
					}
				}		  

				/* Add receiving PHY. Coding:
					1		BLE_GAP_PHY_1MBPS
					4		BLE_GAP_PHY_CODED   */
				pcAdvReportStrg += sprintf( pcAdvReportStrg, ",%i\r\n", xAdvReport.primary_phy ); 		 
		
				xComSendStringRAM( COM0, cAdvReportStrg );
				
				/* Flush the log buffer from time to time. */
				NRF_LOG_FLUSH();
			}
						
			/* Continue scanning. */
			xErrCode = sd_ble_gap_scan_start( NULL, &xScanBuffer );
			APP_ERROR_CHECK( xErrCode );
			
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
	cTxPower1Mbps = OUTPUT_PWR_4_dBm;
	cTxPower125kbps = OUTPUT_PWR_4_dBm;
 
    xErrCode = nrf_sdh_enable_request();
    APP_ERROR_CHECK( xErrCode );

    /* Configure the BLE stack using the default settings.
       Fetch the start address of the application RAM. */
    ulRamStart = 0;
    xErrCode = nrf_sdh_ble_default_cfg_set( APP_BLE_CONN_CFG_TAG, &ulRamStart );
    APP_ERROR_CHECK( xErrCode );

    /* Enable BLE stack. */
    xErrCode = nrf_sdh_ble_enable( &ulRamStart );
    APP_ERROR_CHECK( xErrCode );

    /* Register a handler for BLE events. */
    NRF_SDH_BLE_OBSERVER( m_ble_observer, APP_BLE_OBSERVER_PRIO, vBleEvtHandler, NULL );
	
	/* Set the device's advertiser address. To remain compatible with ublox devices containing connectivity software,
	   we will set the upper 24bit to ublox manufacturer ID and generate the lower 24bit randomly. The address will
	   be marked as random static as opposed to public. */
	/* Initialise the random number generator. */
    xErrCode = nrf_drv_rng_init( NULL );
    APP_ERROR_CHECK( xErrCode );	   

	do
	{
		nrf_drv_rng_bytes_available( &cRngBytesAvailable );
	} while ( cRngBytesAvailable < 3 );
    xErrCode = nrf_drv_rng_rand( xBleGapAddr.addr, 3 );
    APP_ERROR_CHECK( xErrCode );
	
	xErrCode = sd_ble_gap_addr_set( &xBleGapAddr );
    APP_ERROR_CHECK( xErrCode );
	
    /* Create a FreeRTOS task to retrieve SoftDevice events. The first parameter is an optional pointer to a function
  	   to be executed */
    nrf_sdh_freertos_init( vScanStart, NULL, SD_TASK_PRIORITY );	
}
/*-----------------------------------------------------------*/

/* Start advertising. */
void vStartAdvertising( portBASE_TYPE xNewAdvState )
{
	ret_code_t	xErrCode;
	int8_t		cTxPower;
	
	/* Configure the advertisement:
	   Advertisement data is in xAdvData.
	   Advertiser physical configuration is in xAdvParams.
	   Returns the handle for the advertisement. */  
	/* Set the advertisement payload length. */
	xAdvData.adv_data.len = xEncodedAdvDataLen; 
	if ( ( xEncodedAdvDataLen > 31 ) || ( xNewAdvState & ADV_LR125KBPS ) )
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
	/* Set the advertising PHY and TX power as required. */
	if ( xNewAdvState & ADV_LR125KBPS )
	{
		/* Long Range: Set the PHY to 125kbps. */
		xAdvParams.primary_phy = BLE_GAP_PHY_CODED;
		xAdvParams.secondary_phy = BLE_GAP_PHY_CODED;
		cTxPower = cTxPower125kbps;		
	}
	else
	{
		/* Standard advertising: the the advertising PHY to 1Mbps. */
		xAdvParams.primary_phy = BLE_GAP_PHY_1MBPS;
		xAdvParams.secondary_phy = BLE_GAP_PHY_1MBPS;
		cTxPower = cTxPower1Mbps;		
	}
	xErrCode = sd_ble_gap_adv_set_configure( &xAdvHandle, &xAdvData, &xAdvParams );
	APP_ERROR_CHECK( xErrCode );
	
		
	/* Set the output power. */
	xErrCode = sd_ble_gap_tx_power_set( BLE_GAP_TX_POWER_ROLE_ADV, xAdvHandle, OUTPUT_PWR_4_dBm );
	APP_ERROR_CHECK( xErrCode );

	/* Start the advertising. BLE_CONN_CFG_TAG_DEFAULT is ignored as this is non-connectable advertising. */
	xErrCode = sd_ble_gap_adv_start( xAdvHandle, BLE_CONN_CFG_TAG_DEFAULT );
	APP_ERROR_CHECK( xErrCode );
	
	xAdvState = ADV_STD1MBPS;
}
/*-----------------------------------------------------------*/

/* Stop advertising. */
void vStopAdvertising( void )
{
  ( void )sd_ble_gap_adv_stop( xAdvHandle );
  
  xAdvState = 0;
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
void prvBleTimerCallback( TimerHandle_t xTimer )
{
	( void )xTimer;
}
/*-----------------------------------------------------------*/