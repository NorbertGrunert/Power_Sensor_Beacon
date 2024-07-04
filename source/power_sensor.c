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
uint8_t						ui8SensorStatus;

uint8_t						ui8SensorTxData[ SENSOR_TX_LENGTH ];
/*-----------------------------------------------------------*/

void vPowerSensorInit( UBaseType_t uxPriority )
{
	/* Open the sensor COM port with the physical UART RX buffer. */
	vCOMOpen( COM0, cPowerSensorUartRxBuffer, powerSensorUART_RX_BUFFER_SIZE  - 1, COM_BINARY );

	/* The Power Sensor Parser task is spawned here. */
	xTaskCreate( vPowerSensorTask, "PS", powerSensorSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
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
        01 05 00F3
		|  |  |
		|  |  data
		|  length
		field ID

	field ID  |  length  |  description
	----------+----------+--------------------------------------------
	0         |    1     |  identification
			  |			 |	  0:  consumption
			  |			 |	  1:  production
	1		  |	

*/

/*-----------------------------------------------------------*/

/* Convert 3 individual octets to a long value. The octets are stored in the order of high/middle/low otets
   sequencially at the given pointer. 
*/
uint32_t ui32ConvertSensorDataLong( uint8_t *pui8Data )
{
	uint32_t	ui32Result;

	ui32Result =   ( *pui8Data         << 16 )
				 + ( *( pui8Data + 1 ) <<  8 )
				 + ( *( pui8Data + 2 ) <<  0 );
	return ui32Result;
}
/*-----------------------------------------------------------*/

/* Parse Sensor data. */
void vParseSensorData( uint8_t *pui8SensorData )
{
	unsigned portBASE_TYPE			uxIdx;
	uint8_t							ui8RxChkSum;

	uint32_t						ui32SensorVoltage;
	uint32_t						ui32SensorCurrent;
	uint32_t						ui32SensorPower;

	nrf_gpio_pin_set( LED_3 );					// DEBUG DEBUG DEBUG
	nrf_gpio_pin_clear( LED_3 );				// DEBUG DEBUG DEBUG

	/* Clear old sensor Tx data. */
	for ( uxIdx = 0; uxIdx < SENSOR_TX_LENGTH; uxIdx++ )
	{
		ui8SensorTxData[ uxIdx ] = 0;
	}

	/* Check checksum. The checksum excludes the two header octets and the checksum octet itself. */
	ui8RxChkSum = 0;
	for ( uxIdx = 2; uxIdx < SENSOR_RX_LENGTH - 1; uxIdx++ )
	{
		ui8RxChkSum += *( pui8SensorData + uxIdx );
	}

	if ( ui8RxChkSum != *( pui8SensorData + SENSOR_CHKSUM_OFFS ) )
	{
		ui8SensorTxData[ SENSOR_TX_STATUS_OFFS ] |= SENSOR_CHKSUM_ERROR;
	}


	/* Check packet header and, if abnormal, extract the status. */
	if ( ( *( pui8SensorData + SENSOR_HEADER1_OFFS ) & SENSOR_HEADER1_ABNORMAL_MSK ) == SENSOR_HEADER1_ABNORMAL )
	{
		/* Abnormal header. Extract error code. */
		ui8SensorTxData[ SENSOR_TX_STATUS_OFFS ] |= *( pui8SensorData + SENSOR_HEADER1_OFFS ) & SENSOR_HEADER1_ERROR_MSK;
	}
	else
	{
		if ( *( pui8SensorData + SENSOR_HEADER1_OFFS ) == SENSOR_HEADER1_NOT_CALIBRATED )
		{
			/* Device not calibrated. */
			ui8SensorTxData[ SENSOR_TX_STATUS_OFFS ] |= SENSOR_UNCALIBRATED_ERROR;
		}
		else
		{
			if ( *( pui8SensorData + SENSOR_HEADER1_OFFS ) != SENSOR_HEADER1_VALID )
			{
				/* Invalid header1. */
				ui8SensorTxData[ SENSOR_TX_STATUS_OFFS ] |= SENSOR_HEADER1_ERROR;
			}
		}
	}

	if ( *( pui8SensorData + SENSOR_HEADER2_OFFS ) != SENSOR_HEADER2_VALID )
	{
		/* Invalid header1. */
		ui8SensorTxData[ SENSOR_TX_STATUS_OFFS ] |= SENSOR_HEADER2_ERROR;
	}

	/* Do not touch any other fields if there was an error so far. */
	if ( ui8SensorTxData[ SENSOR_TX_STATUS_OFFS ] != 0 )
	{
		return;
	}

	/* Convert voltage data. */
	ui32SensorVoltage = ui32ConvertSensorDataLong( pui8SensorData + SENSOR_UKH_OFFS );
	ui32SensorCurrent = ui32ConvertSensorDataLong( pui8SensorData + SENSOR_IKH_OFFS );
	ui32SensorPower = ui32ConvertSensorDataLong( pui8SensorData + SENSOR_PKH_OFFS );


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