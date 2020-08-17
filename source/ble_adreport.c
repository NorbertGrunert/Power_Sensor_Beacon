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
#include "ble_adreport.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Initialise the BLE task. */
void vBleAdReportInit( void );

/* Send a reported advertiser to the host via UART. */
void vSendAdvertiserData( uint8_t ucAddrType, uint8_t *pucPeerAddr, uint16_t usDataLen, 
						  uint8_t *pucData, unsigned char ucPrimaryPhy, int8_t cRssiValue );

/* Filter the list of RSSI values. */
int8_t xFilterRSSI( int8_t *rssi_list );

/* Timer callback for advertisement RSSI filter timer. */
void prvBleFilterTimerCallback( TimerHandle_t xTimer );

/* The BLE advertising report handling task. */
static portTASK_FUNCTION_PROTO( vBleAdHandlerTask, pvParameters );
/*-----------------------------------------------------------*/

/* Local variables. */
/* Configuration for the RSSI filter. */
unsigned portBASE_TYPE		uxRssiFilterMethod = RSSI_MAXAVG;
unsigned portBASE_TYPE		uxRssiFilterWindow = 10;
/*-----------------------------------------------------------*/

/* Public variables. */

/* Timer handle for the advertisement RSSI filter timer. */
TimerHandle_t 				xBleFilterTimer;

/* Advertiser handler command queue handle. */
QueueHandle_t 				xBleAdHandlerQueue; 
/*-----------------------------------------------------------*/

/* 
 * Initialise the BLE task. 
 */
void vBleAdReportInit( void )
{
	/* Create advertisement timer. */
	/* Create advertisement RSSI filter timer. */
	xBleFilterTimer = xTimerCreate
							( "",			 				/* Timer name for debug. */
							  FILTER_POLL_TIME,				/* Interal at which the BLE RSSI filter is called. */
							  pdTRUE,						/* Auto-reload. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvBleFilterTimerCallback		/* Callback for the advertisement RSSI filter timer. */
							);
							
	/* Create a queue from the CTRL task to the GSM task for commands. */
	xBleAdHandlerQueue = xQueueCreate( bleADRPT_QUEUE_SIZE, ( unsigned portBASE_TYPE ) sizeof( enum xADREPORT_CMD ) );	

	/* Create the task to filter beacon RSSIs and send the result to the host via UART. */
	xTaskCreate( vBleAdHandlerTask, "ADHDL", bleAdHandlerSTACK_SIZE, NULL, BLE_ADHANDLER_PRIORITY, ( TaskHandle_t * ) NULL );
}
/*-----------------------------------------------------------*/

/* 
 * Send a reported advertiser to the host via UART.
 */
void vSendAdvertiserData( uint8_t ucAddrType, uint8_t *pucPeerAddr, uint16_t usDataLen, 
						  uint8_t *pucData, unsigned char ucPrimaryPhy, int8_t cRssiValue )
{
	char				*pcAdvReportStrg;
	char				cAddrType;
	char				cAdvReportStrg[ 600 ];


	switch ( ucAddrType )
	{
		case BLE_GAP_ADDR_TYPE_PUBLIC:							cAddrType = 'p'; break;
		case BLE_GAP_ADDR_TYPE_RANDOM_STATIC:					cAddrType = 'r'; break;
		case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE :		cAddrType = 's'; break;
		case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE:	cAddrType = 'n'; break;
		case BLE_GAP_ADDR_TYPE_ANONYMOUS:						cAddrType = 'a'; break;
		default:												cAddrType = 'u'; break;
	}

	/* Create a fake ublox scan report. */
	pcAdvReportStrg = cAdvReportStrg;
	pcAdvReportStrg += sprintf( pcAdvReportStrg, "+UBTD:%02X%02X%02X%02X%02X%02X%c,%2.2d,\"\",2,",
								pucPeerAddr[ 5 ],
								pucPeerAddr[ 4 ],
								pucPeerAddr[ 3 ],
								pucPeerAddr[ 2 ],
								pucPeerAddr[ 1 ],
								pucPeerAddr[ 0 ],
								cAddrType,
								cRssiValue );

	/* Append the advertising data. */
	for ( int idx = 0; idx < usDataLen; idx++ )
	{
		pcAdvReportStrg += sprintf( pcAdvReportStrg, "%02X", *( pucData + idx ) );
	}

	/* Add receiving PHY. Coding:
		1		BLE_GAP_PHY_1MBPS
		4		BLE_GAP_PHY_CODED   */
	pcAdvReportStrg += sprintf( pcAdvReportStrg, ",%i\r\n", ucPrimaryPhy ); 		 

	xComSendStringRAM( COM0, cAdvReportStrg );
	
	/* Flush the log buffer from time to time. */
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/*
 * Filter the list of RSSI values.
 */
int8_t xFilterRSSI( int8_t *pxRSSIList )
{
	unsigned portBASE_TYPE		uxListIdx;
    int8_t                     	xRSSI;
	signed int					iAvgRSSI;
	signed int					iMaxRSSI;
	unsigned int				uiRSSICnt;
    
	switch ( uxRssiFilterMethod )
	{
		case RSSI_AVG:		/* Build a simple average over all captured RSSI values. */
							uiRSSICnt = 0;
                            iAvgRSSI = 0;
							for ( uxListIdx = 0; uxListIdx < RSSI_LIST_LEN; uxListIdx++ )
							{
								if ( *( pxRSSIList + uxListIdx ) != 0 )
								{
									uiRSSICnt++;
									iAvgRSSI += ( signed int )*( pxRSSIList + uxListIdx );
								}
								else
								{
									/* Last entry. Return the average RSSI. */
									return ( int8_t )( ( ( float )iAvgRSSI / ( float )uiRSSICnt ) - 0.5 );
								}
							}
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
							iAvgRSSI = 0;
							for ( uxListIdx = 0; uxListIdx < RSSI_LIST_LEN; uxListIdx++ )
							{
								if ( *( pxRSSIList + uxListIdx ) != 0 )
								{
									if ( *( pxRSSIList + uxListIdx ) > iMaxRSSI )
									{
										iAvgRSSI += ( signed int )*( pxRSSIList + uxListIdx );
										uiRSSICnt++;
									}
								}
							}
							
							/* Last entry. Return the average RSSI. The -0.5 converts truncating to rounding. */
							return ( int8_t )( ( ( float )iAvgRSSI / ( float )uiRSSICnt ) - 0.5 );
							
							break;
		
		
		default:			/* No filtering. return the newest RSSI value. */
							xRSSI = *pxRSSIList;
							for ( uxListIdx = 0; uxListIdx < RSSI_LIST_LEN; uxListIdx++ )
							{
								if ( *( pxRSSIList + uxListIdx ) != 0 )
								{
									xRSSI = *( pxRSSIList + uxListIdx );
								}
								else
								{
									/* Last entry. Return the RSSI found so far. */
									return xRSSI;
								}
							}
							break;
	}

	/* Dummy return. */
	return 0;			
}
/*-----------------------------------------------------------*/

/* Timer callback for advertisement RSSI filter timer. 
   
   The timer is called periodically (e.g. once per second) and sends a tick to the 
   advertiser report task.

   There, it is checked, if any advertisers have been received which are now due to 
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

/* 
 * Advertiser report task.
 */
static void vBleAdHandlerTask( void * pvParameter )
{
	enum xADREPORT_CMD			xBleAdHandlerCmd;
    uint8_t                   	xRSSI;
	TickType_t				  	xCurrentTime;

    ( void )pvParameter;
	
	/* Task main loop. */
    while ( true )
    {
		if ( xQueueReceive( xBleAdHandlerQueue, &xBleAdHandlerCmd, portMAX_DELAY ) != errQUEUE_EMPTY )
		{
			/* Received a command from the CTRL task. */
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
													
													/* Send the device to the host via UART. */
													vSendAdvertiserData( xKnownDeviceList[ uxListIdx ].ucAddrType, 
																		 xKnownDeviceList[ uxListIdx ].pucPeerAddr, 
																		 xKnownDeviceList[ uxListIdx ].usDataLen, 
																		 xKnownDeviceList[ uxListIdx ].pucData, 
																		 xKnownDeviceList[ uxListIdx ].ucPrimaryPhy, 
																		 xRSSI );
													
//													// DEBUG DEBUG DEBUG
//													// Also send the RSSI buffer.
//													{
//														char				*pcAdvReportStrg;
//														char				cAddrType;
//														char				cAdvReportStrg[ 600 ];
//														
//														switch ( xKnownDeviceList[ uxListIdx ].ucAddrType )
//														{
//															case BLE_GAP_ADDR_TYPE_PUBLIC:							cAddrType = 'p'; break;
//															case BLE_GAP_ADDR_TYPE_RANDOM_STATIC:					cAddrType = 'r'; break;
//															case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE :		cAddrType = 's'; break;
//															case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE:	cAddrType = 'n'; break;
//															case BLE_GAP_ADDR_TYPE_ANONYMOUS:						cAddrType = 'a'; break;
//															default:												cAddrType = 'u'; break;
//														}
//
//														pcAdvReportStrg = cAdvReportStrg;
//														pcAdvReportStrg += sprintf( pcAdvReportStrg, "+DBG: %02X%02X%02X%02X%02X%02X%c,",
//																					xKnownDeviceList[ uxListIdx ].pucPeerAddr[ 5 ],
//																					xKnownDeviceList[ uxListIdx ].pucPeerAddr[ 4 ],
//																					xKnownDeviceList[ uxListIdx ].pucPeerAddr[ 3 ],
//																					xKnownDeviceList[ uxListIdx ].pucPeerAddr[ 2 ],
//																					xKnownDeviceList[ uxListIdx ].pucPeerAddr[ 1 ],
//																					xKnownDeviceList[ uxListIdx ].pucPeerAddr[ 0 ],
//																					cAddrType );
//														for ( int idx = 0; idx < RSSI_LIST_LEN; idx++ )
//														{
//															if ( xKnownDeviceList[ uxListIdx ].xRSSI[ idx ] != 0 )
//															{
//																pcAdvReportStrg += sprintf( pcAdvReportStrg, "%2.2d,", xKnownDeviceList[ uxListIdx ].xRSSI[ idx ] );
//															}
//														}	
//														
//														pcAdvReportStrg += sprintf( pcAdvReportStrg, "\r\n" ); 		 
//
//														xComSendStringRAM( COM0, cAdvReportStrg );
//														
//														/* Flush the log buffer from time to time. */
//														NRF_LOG_FLUSH();
//													}

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
