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
#include "ble_parser.h"
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

/* Start advertising. */
void vStartScan( portBASE_TYPE xNewScanState );

/* Stop advertising. */
void StopScan( void );

/* Return advertising state. */
portBASE_TYPE xGetAdvState( void );

/* Return scan state. */
portBASE_TYPE xGetScanState( void );

/* Timer callback for periodic BLE beacon scans. */
void prvBleAdvTimerCallback( TimerHandle_t xBleAdvTimer );

/* Timer callback for periodic temperature sensor polling. */
void prvTempTimerCallback( TimerHandle_t xTimer );

/* The BLE advertising report handling task. */
static portTASK_FUNCTION_PROTO( vBleAdHandlerTask, pvParameters );
/*-----------------------------------------------------------*/

/* Local variables. */
/* UART Rx character ring buffer */
signed char					cBleUartRxBuffer[ bleUART_RX_BUFFER_SIZE ];

/* Timer handle for the BLE advertising timer. */
static TimerHandle_t 		xBleAdvTimer;	

/* Timer handle for the temperature capture timer. */
static TimerHandle_t 		xTemperatureTimer;	

/* Buffer where advertising reports will be stored by the SoftDevice. */
static uint8_t				cScanBufferData[ BLE_GAP_SCAN_BUFFER_EXTENDED_MIN ]; 

/* Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t xScanParam =
{
	.extended      = 1,
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
/*-----------------------------------------------------------*/

/* Public variables. */

/* Advertising data. */
signed char					cEncodedStd1MbpsAdvData[ 100 ];
portBASE_TYPE				xEncodedStd1MbpsAdvDataLen;
signed char					cEncodedLR125kbpsAdvData[ 256 ];
portBASE_TYPE				xEncodedLR125kbpsAdvDataLen;

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
	xBleAdvTimer = xTimerCreate
							( "",			 				/* Timer name for debug. */
							  ADV_INTERLEAVING_TIME	,		/* BLE switch rate between advertising modes. */
							  pdTRUE,						/* Auto-reload. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvBleAdvTimerCallback		/* Callback for the BLE timer. */
							);
	
	/* Create temperature sensor timer. */
	xTemperatureTimer = xTimerCreate
							( "",			 				/* Timer name for debug. */
							  TEMP_POLL_INTERVAL	,		/* BLE switch rate between advertising modes. */
							  pdTRUE,						/* Auto-reload. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvTempTimerCallback		/* Callback for the BLE timer. */
							);
	
	( void )xTimerStart( xTemperatureTimer, portMAX_DELAY );
	
	/* No advertisement data set yet. */
	xEncodedStd1MbpsAdvDataLen = 0;
	xEncodedLR125kbpsAdvDataLen = 0;
}
/*-----------------------------------------------------------*/

/* 
 * Startup function.
 */
static void vStartUp( void * pvParameter )
{
	ret_code_t xErrCode;

    ( void )pvParameter;
	
	xComSendStringRAM( COM0, "\r\n+STARTUP\r\n" );
	NRF_LOG_FLUSH();
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
	   we will set the upper 24bit to ublox manufacturer ID and take the lower 24bit from the the least significant 24-bit 
	   of the (unique) device address. */	
	xBleGapAddr.addr[0] = ( char )( ( NRF_FICR->DEVICEADDR[0] & 0x000000ff ) >>  0 );
	xBleGapAddr.addr[1] = ( char )( ( NRF_FICR->DEVICEADDR[0] & 0x0000ff00 ) >>  8 );
	xBleGapAddr.addr[2] = ( char )( ( NRF_FICR->DEVICEADDR[0] & 0x00ff0000 ) >> 16 );
	
	xErrCode = sd_ble_gap_addr_set( &xBleGapAddr );
    APP_ERROR_CHECK( xErrCode );
	
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

	/* Start the BLE advertising timer. On expiry, the timer switch between LR and sort range advertising. */
	( void )xTimerStart( xBleAdvTimer, portMAX_DELAY );	

	/* Stop any ongoing advertising before starting another with different parameters. */
	( void ) sd_ble_gap_adv_stop( xAdvHandle );
	
	xAdvState = xNewAdvState;

	if ( xNewAdvState != ADV_NONE )
	{
		// TODO: Catch error and return error as AT event. An error could mean tha tthe advertising data is bad.
		xErrCode = sd_ble_gap_adv_set_configure( &xAdvHandle, &xAdvData, &xAdvParams );
		APP_ERROR_CHECK( xErrCode );	
		
		/* Set the output power. */
		xErrCode = sd_ble_gap_tx_power_set( BLE_GAP_TX_POWER_ROLE_ADV, xAdvHandle, cTxPower );
		APP_ERROR_CHECK( xErrCode );

		/* Start the advertising. BLE_CONN_CFG_TAG_DEFAULT is ignored as this is non-connectable advertising. */
		xErrCode = sd_ble_gap_adv_start( xAdvHandle, BLE_CONN_CFG_TAG_DEFAULT );
		APP_ERROR_CHECK( xErrCode );	
	}
}
/*-----------------------------------------------------------*/

/* Stop advertising. */
void vStopAdvertising( void )
{
	NRF_LOG_DEBUG( "Advertising stopped." );	   
	( void )xTimerStop( xBleAdvTimer, portMAX_DELAY );		
	( void )sd_ble_gap_adv_stop( xAdvHandle );
  
  xAdvState = ADV_NONE;
}
/*-----------------------------------------------------------*/

portBASE_TYPE xGetAdvState( void )
{
	return xAdvState;
}
/*-----------------------------------------------------------*/

/* Start scanning with the given parameter. */
void vStartScan( portBASE_TYPE xNewScanState )
{
	ret_code_t xErrCode;
	
	NRF_LOG_DEBUG( "Scanning mode set to %i.", xNewScanState );	   
	
	xScanParam.scan_phys = 0;
	
	if ( xNewScanState & SCAN_STD1MBPS )
	{
		xScanParam.scan_phys |= BLE_GAP_PHY_1MBPS;
	}
	if ( xNewScanState & SCAN_LR125KBPS )
	{
		xScanParam.scan_phys |= BLE_GAP_PHY_CODED;
	}
	
	/* If already scanning, stop scanning. */
	( void )sd_ble_gap_scan_stop();
   
	/* Start scanning. */
	xErrCode = sd_ble_gap_scan_start( &xScanParam, &xScanBuffer );
	APP_ERROR_CHECK( xErrCode );
	
	xScanState = xNewScanState;
}
/*-----------------------------------------------------------*/

/* Stop scanning. */
void vStopScan( void )
{
	/* Stop scanning. */
	NRF_LOG_DEBUG( "Scanning stopped." );	   
	( void )sd_ble_gap_scan_stop();
	
	xScanState = SCAN_NONE;
}
/*-----------------------------------------------------------*/

portBASE_TYPE xGetScanState( void )
{
	return xScanState;
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
	
		// TODO: Catch error and return error as AT event. An error could mean that the advertising data is bad.
		xErrCode = sd_ble_gap_adv_set_configure( &xAdvHandle, &xAdvData, &xAdvParams );
		APP_ERROR_CHECK( xErrCode );	
		
		/* Set the output power. */
		xErrCode = sd_ble_gap_tx_power_set( BLE_GAP_TX_POWER_ROLE_ADV, xAdvHandle, cTxPower );
		APP_ERROR_CHECK( xErrCode );

		/* Start the advertising. BLE_CONN_CFG_TAG_DEFAULT is ignored as this is non-connectable advertising. */
		xErrCode = sd_ble_gap_adv_start( xAdvHandle, BLE_CONN_CFG_TAG_DEFAULT );
		APP_ERROR_CHECK( xErrCode );	
		NRF_LOG_FLUSH();
	}	
}
/*-----------------------------------------------------------*/

/* Timer callback for periodic temperature sensor polling. 
   CAUTION: This function is running in the timer task!
*/
void prvTempTimerCallback( TimerHandle_t xTimer )
{
    // This function contains workaround for PAN_028 rev2.0A anomalies 28, 29,30 and 31.
    int32_t				temp;
	int32_t				temperature;
	ret_code_t			xErrCode;
	char				cResStrg[ 20 ];
	
	( void )xTimer;
	
	/* Read temperature sensor. */
	sd_temp_get( &temp );
	temperature = ( 10 * temp ) / 4.0;

	sprintf( cResStrg, "\r\n+UTEMP:%.i.%i\r\n", ( int )( temperature / 10 ), temperature - ( 10 * ( int )( temperature / 10 ) ) );
	xComSendStringRAM( COM0, cResStrg );
}
/*-----------------------------------------------------------*/
		