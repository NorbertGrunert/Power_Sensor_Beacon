/*
 * Tracker Firmware
 *
 * Power sensor message parser task.
 *
 * Messages from the power sensor are in fixed format. They arrive as a train of 24 octets every 800ms.
 * This task triggers on the 800ms pause between messsages and then starts cuumulating an entire message.
 * Once complete, the message is decoded, power values are calculated and global values for the BLE
 * advertising packets are prepared.
 *
 * The parser task is stateless.
 *
 */

/* nRF SDK files. */
#define NRF_LOG_MODULE_NAME 				PWR_SENSOR
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

/* Scheduler include files. */
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "string.h"
#include "timers.h"
#include "ble_gap.h"

/* Device specific include files. */
#include "ble_main.h"
#include "custom_board.h"
#include "drv_uart.h"
#include "main.h"
#include "version.h"
#include "power_sensor.h"
#include "rtc.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Local Function prototypes. */
/* Timer callback for periodic BLE beacon advertising payload update. */
void vBleAdvUpdateCallback( TimerHandle_t xTimer );

/* Store a long value in 4 individual, consecutive octets in little endian. */
void vStoreLongInDatField( uint8_t *puiTarget, uint32_t ui32Data );

/* Create the BLE long-range advertising payload. */
void vFormatBeaconPayload( uint32_t ui16Status, double dVoltage, double dCurrent, double dPower );

/* Convert 3 individual octets to a 24-bit value. The octets are stored in the order of high/middle/low otets. */
uint32_t ui32ConvertSensorData24Bit( uint8_t *pui8Data );

/* Convert 3 individual octets to a 32-bit long value. The octets are stored in little-endian the order. */
uint32_t ui32ConvertSensorData32BitLE( uint8_t *pui8Data );

/* Parse Sensor data. */
void vParseSensorData( uint8_t *pui8SensorData );
/*-----------------------------------------------------------*/

/* Global Function prototypes. */
void vPowerSensorInit( UBaseType_t uxPriority );

/* The parser task as described at the top of the file. */
static portTASK_FUNCTION_PROTO( vPowerSensorTask, pvParameters );
/*-----------------------------------------------------------*/

/* Local variables. */
/* Sensor UART Rx character ring buffer */
signed char					cPowerSensorUartRxBuffer[ powerSensorUART_RX_BUFFER_SIZE ];
/*-----------------------------------------------------------*/

/* Global variables. */
uint8_t						ui8SensorStatus;
uint16_t					ui8SensorVoltage;
uint16_t					ui8SensorVoltageCycle;
uint8_t						ui8SensorCurrent;
uint8_t						ui8SensorCurrentCycle;
uint16_t					ui8SensorPower;
uint8_t						ui8SensorPowerCycle;

/* Timer handle for the BLE advertising timer. */
static TimerHandle_t 		xBleAdvUpdateTimer;
/*-----------------------------------------------------------*/

void vPowerSensorInit( UBaseType_t uxPriority )
{
	/* Create advertisement update timer. */
	xBleAdvUpdateTimer = xTimerCreate (
										"",			 				/* Timer name for debug. */
										ADV_UPDATE_INTERVAL,			/* BLE advertising payload update inetrval. */
										pdTRUE,						/* Auto-reload. */
										( void * )0,					/* Init for the ID / expiry count. */
										vBleAdvUpdateCallback			/* Callback. */
									  );

	/* Open the sensor COM port with the physical UART RX buffer. */
	vCOMOpen( COM0, cPowerSensorUartRxBuffer, powerSensorUART_RX_BUFFER_SIZE  - 1, COM_BINARY );

	/* The Power Sensor Parser task is spawned here. */
	xTaskCreate( vPowerSensorTask, "PS", powerSensorSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
}
/*-----------------------------------------------------------*/

/* Convert 3 individual octets to a 24-bit value. The octets are stored in the order of high/middle/low otets
   sequencially at the given pointer. 
*/
uint32_t ui32ConvertSensorData24Bit( uint8_t *pui8Data )
{
	uint32_t	ui32Result;

	ui32Result =   ( *pui8Data         << 16 )
				 + ( *( pui8Data + 1 ) <<  8 )
				 + ( *( pui8Data + 2 ) <<  0 );
	return ui32Result;
}
/*-----------------------------------------------------------*/

/* Convert 3 individual octets to a 32-bit long value. The octets are stored in little-endian the order (low to high)
   sequencially at the given pointer. 
*/
uint32_t ui32ConvertSensorData32BitLE( uint8_t *pui8Data )
{
	uint32_t	ui32Result;

	ui32Result =   ( *pui8Data         <<  0 )
				 + ( *( pui8Data + 1 ) <<  8 )
				 + ( *( pui8Data + 2 ) << 16 )
				 + ( *( pui8Data + 3 ) << 24 );
	return ui32Result;
}
/*-----------------------------------------------------------*/

/* Store a long value in 4 individual, consecutive octets in little endian. */
void vStoreLongInDatField( uint8_t *puiTarget, uint32_t ui32Data )
{
	*( puiTarget + 0 ) = ( uint8_t )( ui32Data & 0x000000ff ) >> 0;
	*( puiTarget + 1 ) = ( uint8_t )( ui32Data & 0x0000ff00 ) >> 8;
	*( puiTarget + 2 ) = ( uint8_t )( ui32Data & 0x00ff0000 ) >> 16;
	*( puiTarget + 3 ) = ( uint8_t )( ui32Data & 0xff000000 ) >> 24;
}
/*-----------------------------------------------------------*/

/* Timer callback for periodic BLE beacon advertising payload update. 
   CAUTION: This function is running in the timer task!
*/
void vBleAdvUpdateCallback( TimerHandle_t xTimer )
{
	ret_code_t	xErrCode;
	int8_t		cTxPower;

	( void )xTimer;
	
	vStartAdvertising( ADV_LR125KBPS );

	NRF_LOG_DEBUG( "current: %imA", ui32ConvertSensorData32BitLE( &cEncodedLR125kbpsAdvData[ SENSOR_TX_CURRENT_OFFS + DATA_OFFS ] ) );
	NRF_LOG_DEBUG( "voltage: %imV", ui32ConvertSensorData32BitLE( &cEncodedLR125kbpsAdvData[ SENSOR_TX_VOLTAGE_OFFS + DATA_OFFS ] ) );
	NRF_LOG_DEBUG( "power:   %imW", ui32ConvertSensorData32BitLE( &cEncodedLR125kbpsAdvData[ SENSOR_TX_POWER_OFFS   + DATA_OFFS ] ) );
		
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* Create the BLE long-range advertising payload. 
   
	Advertising data format:
	    02 01 06 12 FF 1700 CAFECAFE 358578080077244 DBFB0300...
		|        |     |    |        |  
		|        |     |    |        Variable number of datafields
		|        |     |    9: Manufacturer code
		|        |     5: MfgID (Newlogic = 0x0017)                                   
		|        3: AD field length and identifier for manufacturer specific data
		0: advertiser payload flags

   Datafield format:
        01 05 00F32713A0
		|  |  |
		|  |  data octets
		|  length in octets
		field ID

	Numbers consisting for multiple octets are stored in little endian.

	field ID  |  length  |  format  |  description
	----------+----------+--------------------------------------------
	0		  |    1	 |  uint8   | system ID
	1         |    1     |  uint8	| sensor ID
			  |			 |	  		|	0:  consumption
			  |			 |	  		|	1:  production
	2		  |	   2	 |  uint16	| status
	3		  |	   4	 |  uint32	| voltage in mV
	4		  |	   4	 |  uint32	| current in mA
	5		  |	   4	 |  uint32  | power in mW
*/
void vFormatBeaconPayload( uint32_t ui16Status, double dVoltage, double dCurrent, double dPower )
{
	uint32_t				ui32Voltage;
	uint32_t				ui32Current;
	uint32_t				ui32Power;

	ui32Voltage = ( uint32_t )( dVoltage * 1000.0 );
	ui32Current = ( uint32_t )( dCurrent * 1000.0 );
	ui32Power   = ( uint32_t )( dPower   * 1000.0 );

	cEncodedLR125kbpsAdvData[ SENSOR_TX_SYSTEM_ID_OFFS +   ID_OFFS     ] = SENSOR_TX_SYSTEM_ID_ID;
	cEncodedLR125kbpsAdvData[ SENSOR_TX_SYSTEM_ID_OFFS +  LEN_OFFS     ] = 1;
	cEncodedLR125kbpsAdvData[ SENSOR_TX_SYSTEM_ID_OFFS + DATA_OFFS     ] = SYSTEM_ID;

	cEncodedLR125kbpsAdvData[ SENSOR_TX_SENSOR_ID_OFFS +   ID_OFFS     ] = SENSOR_TX_SENSOR_ID_ID;
	cEncodedLR125kbpsAdvData[ SENSOR_TX_SENSOR_ID_OFFS +  LEN_OFFS     ] = 1;
	cEncodedLR125kbpsAdvData[ SENSOR_TX_SENSOR_ID_OFFS + DATA_OFFS     ] = SENSOR_ID;

	cEncodedLR125kbpsAdvData[ SENSOR_TX_STATUS_OFFS    +   ID_OFFS     ] = SENSOR_TX_STATUS_ID;
	cEncodedLR125kbpsAdvData[ SENSOR_TX_STATUS_OFFS    +  LEN_OFFS     ] = 2;
	cEncodedLR125kbpsAdvData[ SENSOR_TX_STATUS_OFFS    + DATA_OFFS + 0 ] = ( ui16Status & 0x00ff ) >> 0;
	cEncodedLR125kbpsAdvData[ SENSOR_TX_STATUS_OFFS    + DATA_OFFS + 1 ] = ( ui16Status & 0xff00 ) >> 8;

	cEncodedLR125kbpsAdvData[ SENSOR_TX_VOLTAGE_OFFS   +   ID_OFFS     ] = SENSOR_TX_VOLTAGE_ID;
	cEncodedLR125kbpsAdvData[ SENSOR_TX_VOLTAGE_OFFS   +  LEN_OFFS     ] = 4;
	vStoreLongInDatField( &cEncodedLR125kbpsAdvData[ SENSOR_TX_VOLTAGE_OFFS + DATA_OFFS ], ui32Voltage );

	cEncodedLR125kbpsAdvData[ SENSOR_TX_CURRENT_OFFS   +   ID_OFFS     ] = SENSOR_TX_CURRENT_ID;
	cEncodedLR125kbpsAdvData[ SENSOR_TX_CURRENT_OFFS   +  LEN_OFFS     ] = 4;
	vStoreLongInDatField( &cEncodedLR125kbpsAdvData[ SENSOR_TX_CURRENT_OFFS + DATA_OFFS ], ui32Current );

	cEncodedLR125kbpsAdvData[ SENSOR_TX_POWER_OFFS     +   ID_OFFS     ] = SENSOR_TX_POWER_ID;
	cEncodedLR125kbpsAdvData[ SENSOR_TX_POWER_OFFS     +  LEN_OFFS     ] = 4;
	vStoreLongInDatField( &cEncodedLR125kbpsAdvData[ SENSOR_TX_POWER_OFFS   + DATA_OFFS ], ui32Power );

	xEncodedLR125kbpsAdvDataLen = 28;
}
/*-----------------------------------------------------------*/

/* Parse Sensor data. */
void vParseSensorData( uint8_t *pui8SensorData )
{
	unsigned portBASE_TYPE			uxIdx;
	uint8_t							ui8RxChkSum;
	uint32_t						ui32SensorVCoeff;
	uint32_t						ui32SensorVCycles; 
	uint32_t						ui32SensorICoef;
	uint32_t						ui32SensorICcycles;
	uint32_t						ui32SensorPCoeff;
	uint32_t						ui32SensorPCycles;
	uint16_t						ui16SensorStatus;
	double							dCurrent;
	double							dVoltage;
	double							dPower;

	nrf_gpio_pin_set( LED_3 );					// DEBUG DEBUG DEBUG
	nrf_gpio_pin_clear( LED_3 );				// DEBUG DEBUG DEBUG

	/* Clear old sensor status. */
	ui16SensorStatus = 0;

	/* Check checksum. The checksum excludes the two header octets and the checksum octet itself. */
	ui8RxChkSum = 0;
	for ( uxIdx = 2; uxIdx < SENSOR_RX_LENGTH - 1; uxIdx++ )
	{
		ui8RxChkSum += *( pui8SensorData + uxIdx );
	}

	if ( ui8RxChkSum != *( pui8SensorData + SENSOR_CHKSUM_OFFS ) )
	{
		ui16SensorStatus |= SENSOR_CHKSUM_ERROR;
	}


	/* Check packet header and, if abnormal, extract the status. */
	if ( ( *( pui8SensorData + SENSOR_HEADER1_OFFS ) & SENSOR_HEADER1_ABNORMAL_MSK ) == SENSOR_HEADER1_ABNORMAL )
	{
		/* Abnormal header. Extract error code. */
		ui16SensorStatus |= *( pui8SensorData + SENSOR_HEADER1_OFFS ) & SENSOR_HEADER1_ERROR_MSK;
	}
	else
	{
		if ( *( pui8SensorData + SENSOR_HEADER1_OFFS ) == SENSOR_HEADER1_NOT_CALIBRATED )
		{
			/* Device not calibrated. */
			ui16SensorStatus |= SENSOR_UNCALIBRATED_ERROR;
		}
		else
		{
			if ( *( pui8SensorData + SENSOR_HEADER1_OFFS ) != SENSOR_HEADER1_VALID )
			{
				/* Invalid header1. */
				ui16SensorStatus |= SENSOR_HEADER1_ERROR;
			}
		}
	}

	if ( *( pui8SensorData + SENSOR_HEADER2_OFFS ) != SENSOR_HEADER2_VALID )
	{
		/* Invalid header1. */
		ui16SensorStatus |= SENSOR_HEADER2_ERROR;
	}

	/* Do not touch any other fields if there was an error so far. */
	if ( ui16SensorStatus == 0 )
	{
		/* Convert electrical data. 
				Measured current Ix(A)= current coefficient / ( current cycle ∗ V1R )
				Measured voltage Ux(V)= voltage coefficient ∗ V2R / voltage cycle
				Measured power Px(W)= Power coefficient ∗ V2R / ( Power cycle ∗ V1R )
		with:
				V1R = 1000 as 1Ohm = 1000 * 1Ohm
				V2R = 1.88 as (0.91 + 0.91 + 0.0604)MOHm = 1.8804 * 1MOhm = 1.88 * 1MOhm
		*/
		ui32SensorVCoeff = ui32ConvertSensorData24Bit( pui8SensorData + SENSOR_UKH_OFFS );
		ui32SensorVCycles = ui32ConvertSensorData24Bit( pui8SensorData + SENSOR_UTH_OFFS );
		ui32SensorICoef = ui32ConvertSensorData24Bit( pui8SensorData + SENSOR_IKH_OFFS );
		ui32SensorICcycles = ui32ConvertSensorData24Bit( pui8SensorData + SENSOR_ITH_OFFS );
		ui32SensorPCoeff = ui32ConvertSensorData24Bit( pui8SensorData + SENSOR_PKH_OFFS );
		ui32SensorPCycles = ui32ConvertSensorData24Bit( pui8SensorData + SENSOR_PTH_OFFS );

		dCurrent = ( double )ui32SensorICoef          / ( double )ui32SensorICcycles;
		dVoltage = ( double )ui32SensorVCoeff * 1.88  / ( double )ui32SensorVCycles;
		dPower   = ( double )ui32SensorPCoeff * 1.88  / ( double )ui32SensorPCycles;

		NRF_LOG_DEBUG( "current: %i", ( int )dCurrent );
		NRF_LOG_DEBUG( "voltage: %i", ( int )dVoltage );
		NRF_LOG_DEBUG( "power:   %i", ( int )dPower );
	}
	else
	{
		dCurrent = 0.0;
		dVoltage = 0.0;
		dPower = 0.0;
	}

	/* Format the beacon data fied. */
	vFormatBeaconPayload( ui16SensorStatus, dVoltage, dCurrent, dPower );
}
/*-----------------------------------------------------------*/

static portTASK_FUNCTION( vPowerSensorTask, pvParameters )
{
	uint8_t					ui8SensorData[ SENSOR_RX_LENGTH ];
	unsigned portBASE_TYPE	uxSensorDataIdx;
	TickType_t				xRxTimeStamp;
	
	/* Just to stop compiler warnings. */
	( void ) pvParameters;

	uxSensorDataIdx = 0;
	xRxTimeStamp = xTaskGetTickCount();

	nrf_gpio_pin_clear( LED_1 );				// DEBUG DEBUG DEBUG
	nrf_gpio_pin_clear( LED_2 );				// DEBUG DEBUG DEBUG
	nrf_gpio_pin_clear( LED_3 );				// DEBUG DEBUG DEBUG

	vTaskDelay ( 1 );							// DEBUG DEBUG DEBUG

	nrf_gpio_pin_set( LED_1 );					// DEBUG DEBUG DEBUG
	nrf_gpio_pin_clear( LED_1 );				// DEBUG DEBUG DEBUG
	nrf_gpio_pin_set( LED_2 );					// DEBUG DEBUG DEBUG
	nrf_gpio_pin_clear( LED_2 );				// DEBUG DEBUG DEBUG
	nrf_gpio_pin_set( LED_3 );					// DEBUG DEBUG DEBUG
	nrf_gpio_pin_clear( LED_3 );				// DEBUG DEBUG DEBUG


	while ( 1 )
	{
		/* Wait indefinitely for a string to arrive on the BLE UART. */
		if ( uiGetNumberOfCharsInBuffer( COM0 ) > 0 )
		{
			/* New data has been received. */
			while ( bGetRxCharFromBuffer( COM0, ( signed char * )&ui8SensorData[ uxSensorDataIdx ] ) )
			{
				if ( uxSensorDataIdx < SENSOR_RX_LENGTH )
				{
					uxSensorDataIdx++;
				}
			}

			/* Update the receive timer. */
			xRxTimeStamp = xTaskGetTickCount();
		}
		else
		{
			/* No new data has been received. Check if the no receive data timeout has expired 
			   and, if this is the case, parse the data and configure the beacon payload with it. */
			if ( ( xTaskGetTickCount() - xRxTimeStamp ) > SENSOR_RX_TIMEOUT )
			{
				if ( uxSensorDataIdx >= SENSOR_RX_LENGTH )
				{
					vParseSensorData( ui8SensorData );
				}
				uxSensorDataIdx = 0;
			}
		}

		vTaskDelay ( 1 * portTICKS_PER_10MSEC );
	}
}		
/*-----------------------------------------------------------*/