/*
 * Tracker Firmware
 *
 * UART driver header file
 *
 */
 
#ifndef DRV_GPS_H
#define DRV_GPS_H

/* nRF include files */
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_twi_mngr.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "timers.h"
#include "timers.h"

/* Device specific include files. */
#include "tracker.h"
/*-----------------------------------------------------------*/

/* Definitions for the TWI instance. */
#define TWI_INSTANCE_ID             0
#define MAX_PENDING_TRANSACTIONS    1
#define TWI_FREQUENCY_FREQUENCY_K100 (0x01980000UL)		/* 100 kbps */
#define NRF_DRV_TWI_FREQ_50K		(0x00cc0000UL)		/* 50 kbps */


/* Time interval at which the advertiser RSSI filter is called. */
#define GPS_START_TO				( 1 * portTICKS_PER_SEC )
#define GPS_REGISTER_READ_TO		( 5 * portTICKS_PER_SEC )
#define GPS_REGISTER_WRITE_TO		( 5 * portTICKS_PER_SEC )

#define MSG_BUFFER_LEN				256
#define AN_BUFFER_LEN				10000


/* ZOE I2C definitions. */
#define GPS_SLAVE_ADDR              0x42

/* u-blox ubx protocol definitions. /*/
#define SYNC_CHAR_1                 0xb5
#define SYNC_CHAR_2                 0x62
#define ASCII_LF					0x0a

/* Custom error codes. */
#define NRF_I2C_PROTOCOL_ERROR		0x200
#define NRF_UBX_CHKERROR			0x201
#define NRF_NMEA_CHKERROR			0x202
/*-----------------------------------------------------------*/

/* Variable type definitions. */

/* State of the uiDispatchRdData() FSM. */
enum xDISPATCH_FSM
{
	I2C_IDLE,				/* All read transactions are termined. */
	I2C_UBX,				/* The I2C interface receives a UBX message. */
	I2C_ASCII				/* The I2C interface receives an ASCII string. */
};

/* Functional state of the GPS module. */
enum xGPS_STATE
{
	GPS_ON,
	GPS_OFF
};
/*-----------------------------------------------------------*/

/* External function declarations. 
   These functions constitue the API of the GPS driver. */

/* Initialise the I2C interface to the GPS module. */
extern void vGpsDrvInit( void );

/* Boot the GPS module. */
extern void vGpsStart( void );

/* Shut the GPS module down. */
extern void vGpsStop( void );

/* Return the GPS functional state. */
extern enum xGPS_STATE xGetGpsState( void );

/* Calculate the checksum over an UBX message. */
extern uint16_t uiCalcChkSum( uint8_t *puiUbxMessage, uint32_t uiMessageLength );

/* Read any available bytes from the GPS module and dispatch them to the UBX message buffer *pcUbxMsgBuffer or send them directly to 
   the UART in case of NMEA sentences. */
extern ret_code_t xGpsGetAvailBytesAndRead( uint8_t *pcUbxMsgBuffer, uint32_t *puiUbxMsgLen );

/* Flush all data out of the GPS receiver. */
extern ret_code_t xGpsFlushAvailBytes( void );

/* Read a GPS register. */
extern ret_code_t xGpsReadRegister( uint16_t uiUbxMsgClass, uint16_t uiUbxMsgID, uint8_t *pcUbxMsgBuffer, uint32_t *puiUbxMsgLen );

/* Write a GPS register. */
extern ret_code_t xGpsWriteRegister( const char *pcUbxMsg, uint32_t uiUbxMsgLen );

/* Raw write of a UBX message to the GPS module. */
extern ret_code_t xGpsWriteRaw( uint8_t **ppuiUbxMessage );

/* Return the GPS cumulative on-duration. */
extern TickType_t xGetGpsOnDuration( void );

/* Return the GPS functional state. */
extern void vResetGpsOnDuration( TickType_t xResetTargetValue );
/*-----------------------------------------------------------*/

#endif
