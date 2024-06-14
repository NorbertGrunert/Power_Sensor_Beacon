/*
 * Tracker Firmware
 *
 * GPS driver
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>

#include "tracker.h"

/* nRF SDK include files */
#define NRF_LOG_MODULE_NAME 		DRV_GPS
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nordic_common.h"
#include "nrf.h"
#include "custom_board.h"
#include "nrf_twi_mngr.h"

 /* Scheduler include files. */
#include "FreeRTOS.h"
#include "timers.h"

/* Device specific include files. */
#include "drv_gps.h"
#include "drv_uart.h"

#include "gps.h"
#include "rtc.h"
#include "trace.h"
#include "tracemsg.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Local variables. */
static enum xGPS_STATE		xGpsState;

/* Buffer for octets to send to the I2B bus or for octets read from the bus. From the I2C side, the buffer 
   is read/written by DMA. */
static uint8_t				cTransferBuffer[ MSG_BUFFER_LEN ];

/* Buffer for received UBX messages from the I2C. The messages are extracted from the cTransferBuffer[] buffer. */
static uint8_t				cRdUbxBuffer[ MSG_BUFFER_LEN ];

/* Write index to cRdUbxBuffer[]. */
static uint32_t				uiUbxBufferWrIdx;

/* Expected number of UBX message bytes. */
static uint32_t				uiUbxExpectedNumberOfBytes;

/* Buffer for received ASCII messages from the I2C. The messages are extracted from the cTransferBuffer[] buffer. */
static uint8_t				cRdAsciiBuffer[ MSG_BUFFER_LEN ];

/* Write index to cRdAsciiBuffer[]. */
static uint32_t				uiAsciiBufferWrIdx;

/* Variable containing the state of the FSM which dispatches the data from the cTransferBuffer to either UBX or ASCII
   buffers. */
static enum xDISPATCH_FSM	xI2cRdDataState;

/* On-duration counters for the GPS RF module. The counter is reset every time a packet has been 
   sent to the server using the interface function. */
TickType_t					xGpsOnTimeStamp;
TickType_t					xGpsModuleOnDuration;
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* TWI (with transaction manager) initialization. */
void vTwiConfig( void );

/* Initialise the I2C interface to the GPS module. */
void vGpsDrvInit( void );

/* Verify the received NMEA sentence using the checksum. */
static bool bCheckNmeaSentence( uint8_t *puiAsciiMessage, uint32_t uiMessageLength );

/* Calculate the checksum over an UBX message. */
uint16_t uiCalcChkSum( uint8_t *puiUbxMessage, uint32_t uiMessageLength );

/* Go through the rdUbxBuffer and see if the message in there is complete. */
static bool bUbxIsComplete( uint8_t *pcRdUbxBuffer, uint32_t uiUbxBufferWrIdx );

/* Check if the received UBX message was an acknowledgement with the class and message ID given in the parameters. */
static bool bUbxIsACK( uint8_t *pcUbxRxMsgBuffer, uint32_t uiUbxRxMsgLen, uint8_t uiClass, uint8_t uiMsgID );

/* Dispatch the received I2C data into either the ASCII or the UBX buffer. */
static ret_code_t xDispatchRdData( uint8_t *pcTransferBuffer, uint32_t uiBytesRead, uint8_t *pcUbxMsgBuffer, uint32_t *puiUbxMsgLen );

/* Check if there is a message from the GPS module in the UBX buffer. */
static bool bCheckForUbxMessage( uint32_t uiGpsBytesAvail );

/* Get the number of available bytes from the GPS module. -*/
ret_code_t xGpsGetAvailBytes( uint32_t *puiGpsBytesAvail );

/* Read the available bytes from the GPS module. -*/
ret_code_t xGpsGetAvailBytesAndRead( uint8_t *pcUbxMsgBuffer, uint32_t *puiUbxMsgLen );

/* Flush all data out of the GPS receiver. */
ret_code_t xGpsFlushAvailBytes( void );

/* Boot the GPS module. */
void vGpsStart( void );

/* Shut the GPS module down. */
void vGpsStop( void );

/* Return the GPS functional state. */
enum xGPS_STATE	 xGetGpsState( void );

/* Read a GPS register. */
ret_code_t xGpsReadRegister( uint16_t uiUbxMsgClass, uint16_t uiUbxMsgID, uint8_t *pcUbxMsgBuffer, uint32_t *puiUbxMsgLen );

/* Write a GPS register. */
ret_code_t xGpsWriteRegister( const char *pcUbxMsg, uint32_t uiUbxMsgLen );

/* Raw write of a UBX message to the GPS module. */
ret_code_t xGpsWriteRaw( uint8_t **ppuiUbxMessage );
/*-----------------------------------------------------------*/

/* Create the TWI instance. */
NRF_TWI_MNGR_DEF( m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID );

/* TWI (with transaction manager) initialization. */
void vTwiConfig( void )
{
    uint32_t xErrCode;

    nrf_drv_twi_config_t const config = {
       .scl                = GPS_SCL_PIN,                   /* TP57 - BleGreen - IO7 */
       .sda                = GPS_SDA_PIN,                   /* TP56 - BleRed - IO1 */
       .frequency          = NRF_DRV_TWI_FREQ_100K,       
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    xErrCode = nrf_twi_mngr_init( &m_nrf_twi_mngr, &config );
    APP_ERROR_CHECK( xErrCode );
}
/*-----------------------------------------------------------*/


/* Initialise the I2C interface to the GPS module. */
void vGpsDrvInit( void )
{
    /* Power-enable is 0. */
    nrf_gpio_cfg_output( GPS_CTRL_PWR );
    nrf_gpio_pin_clear( GPS_CTRL_PWR );

	/* Pull the bus pins low. */
    nrf_gpio_cfg_output( GPS_SCL_PIN );
    nrf_gpio_pin_clear( GPS_SCL_PIN );
    nrf_gpio_cfg_output( GPS_SDA_PIN );
    nrf_gpio_pin_clear( GPS_SDA_PIN );

    xGpsState = GPS_OFF;

	/* Initialise GPS on-duration counter. */
	xGpsOnTimeStamp = 0;
	xGpsModuleOnDuration = 0;
	
	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/


/* Verify the received NMEA sentence using the checksum.

   The checksum is an EXOR over all characters between the '$' and the '*'.
   The initial value is 0.
*/
static bool bCheckNmeaSentence( uint8_t *puiAsciiMessage, uint32_t uiMessageLength )
{	
	uint32_t		uiIdx;
	uint8_t			uiChkSum;
	uint8_t			uiRxChkSum;
	signed char		cChar;
	
	uiChkSum = 0x0;
	
	if ( uiMessageLength < 5 )
	{
		NRF_LOG_ERROR( "%i NMEA: NMEA sentence too short, length: 0x%08x", ulReadRTC(), uiMessageLength );	
		NRF_LOG_FLUSH();
	}

	/* Walk through the received string containing the NMEA sentence in between the '$' and the '*'. */
	for ( uiIdx = 1; uiIdx < uiMessageLength - 5; uiIdx++ )
	{
		/* EXOR all characters. */
		uiChkSum ^= *( puiAsciiMessage + uiIdx );
	}
	
	/* The received checkum must be preceded by a '*'. */
	if ( *( puiAsciiMessage + uiMessageLength - 5 ) != '*' )
	{
		return false;
	}
	
	/* Now verify the received checksum against the calculated one. */
	uiRxChkSum  = ( cCharToNibble( *( puiAsciiMessage + uiMessageLength - 4 ) ) << 4 )
				+   cCharToNibble( *( puiAsciiMessage + uiMessageLength - 3 ) );

	/* Compare received and calculated checksums and return result. */
	if ( uiRxChkSum == uiChkSum )
	{
		return true;
	}
	else
	{
		NRF_LOG_ERROR( "%i UBX: Received checksum mismatch!", ulReadRTC() );	
		NRF_LOG_FLUSH();
		return false;
	}
}
/*-----------------------------------------------------------*/


/* Calculate the checksum over an UBX message.
   Input:
		uiMessageLength		length of the entire message including SYNC, CLASS, ID, LENGTH fields and checksum.
		puiUbxMessage		pointer to the message (to the first SYNC character). 
*/
uint16_t uiCalcChkSum( uint8_t *puiUbxMessage, uint32_t uiMessageLength )
{
    uint16_t    	uiCk_a;  
    uint16_t    	uiCk_b;
    uint32_t    	uiIdx;

    /* Skip sync characters. */
    puiUbxMessage += 2;

    uiCk_a = 0;
    uiCk_b = 0;

    /* Loop over payload while skipping the sync characters and the checksum itself. */
    for ( uiIdx = 0; uiIdx < uiMessageLength - 4; uiIdx++ )
    {
        uiCk_a = ( uiCk_a + *( puiUbxMessage + uiIdx ) ) & 0xff;
        uiCk_b = ( uiCk_b + uiCk_a ) & 0xff;
    }

    return ( uiCk_b << 8 ) + uiCk_a;
}
/*-----------------------------------------------------------*/


/* Go through the rdUbxBuffer and see if the message in there is complete.
   If this is the case, return true.
*/
static bool bUbxIsComplete( uint8_t *pcRdUbxBuffer, uint32_t uiUbxBufferWrIdx ) 
{
	/* Check if the UBX message length which is contained in the UBX header has already been received. */
	if ( uiUbxBufferWrIdx == 6 )
	{
		uiUbxExpectedNumberOfBytes = ( *( pcRdUbxBuffer + uiUbxBufferWrIdx - 1 ) << 8 ) + *( pcRdUbxBuffer + uiUbxBufferWrIdx - 2 );
		return false;
	}

	/* Now check if all bytes of the UBX message have been reeived. */
	if ( uiUbxBufferWrIdx == uiUbxExpectedNumberOfBytes + 8 )
	{
		return true;
	}
	
	return false;
}
/*-----------------------------------------------------------*/

/* Check if the received UBX message was an acknowledgement with the class and message ID given in the parameters. */
static bool bUbxIsACK( uint8_t *pcUbxRxMsgBuffer, uint32_t uiUbxRxMsgLen, uint8_t uiClass, uint8_t uiMsgID ) 
{
	return (    ( uiUbxRxMsgLen == 10 )
	         && ( *( pcUbxRxMsgBuffer + 02 ) == UBX_ACK )
		     && ( *( pcUbxRxMsgBuffer + 03 ) == ( UBX_ACK_ACK & 0x00ff ) )
		     && ( *( pcUbxRxMsgBuffer + 06 ) == uiClass )
             && ( *( pcUbxRxMsgBuffer + 07 ) == uiMsgID ) );
}
/*-----------------------------------------------------------*/


/* Dispatch the received I2C data into either the ASCII or the UBX buffer.
  
   For write operations, the dataBuffer contains only UBX messages.
   For read, however, the buffer (may) contain a mix of UBX messages and ASCII data. Usually (it appears), UBX messages are never
   interrupted by ASCII strings. 
   ASCII strings are separated into lines which are terminated by CRLF (0x0d, 0x0a). Lines themselves appear to be never split
   by UBX messages.

   Returns false if there was a problem with the received data, else true.
*/
static ret_code_t xDispatchRdData( uint8_t *pcTransferBuffer, uint32_t uiBytesRead, uint8_t *pcUbxMsgBuffer, uint32_t *puiUbxMsgLen )
{
	uint32_t		uiIdx;
	char			cRxByte;	
	bool			xErrCode;
	
	xErrCode = NRF_SUCCESS;

	/* Walk through all received octets. */
	for ( uiIdx = 0; uiIdx < uiBytesRead; uiIdx++ )
	{
		cRxByte = *( pcTransferBuffer + uiIdx );
		
		/* If the interface is in idle state, decide the next state based on the value of the next octet. */
		if ( xI2cRdDataState == I2C_IDLE )
		{
			/* Test for a UBX message. */
			if ( cRxByte == SYNC_CHAR_1 )
			{
				xI2cRdDataState = I2C_UBX;
			}
			/* Test for an ASCII character. */
			else 
			{
				if ( cRxByte < 0x80 )
				{
					xI2cRdDataState = I2C_ASCII;
				}
				else
				{
					/* We received something which is neither a UBX message nor an ASCII string. So something is wrong here. */
					NRF_LOG_ERROR( "%i I2C: Received unexpected character!", ulReadRTC() );	
					NRF_LOG_FLUSH();

                    xErrCode = NRF_I2C_PROTOCOL_ERROR;
				}
			}
		}
		
		/* Treat the UBX messages. */
		if ( xI2cRdDataState == I2C_UBX )
		{
			cRdUbxBuffer[ uiUbxBufferWrIdx++ ] = cRxByte;

			if ( uiUbxBufferWrIdx > MSG_BUFFER_LEN - 1 ) 
			{
				NRF_LOG_ERROR( "%i I2C: Message buffer overrun!", ulReadRTC() );	
				NRF_LOG_FLUSH();
				
				xErrCode = NRF_I2C_PROTOCOL_ERROR;
			}
			
			/* Check if the message is complete. */
			if ( bUbxIsComplete( cRdUbxBuffer, uiUbxBufferWrIdx ) )
			{
				uint16_t	uiUbxChkSum;
				
				/* Verify the checksum of the received message. */
				uiUbxChkSum = uiCalcChkSum( cRdUbxBuffer, uiUbxBufferWrIdx );
				
				if (    ( cRdUbxBuffer[ uiUbxBufferWrIdx - 2 ] == ( uiUbxChkSum & 0x00ff ) )
				     && ( cRdUbxBuffer[ uiUbxBufferWrIdx - 1 ] == ( uiUbxChkSum & 0xff00 ) >> 8 ) )	
				{
					uint32_t	uiBufIdx;
					
					/* The message passed the check. Now process it. */
					
					/* Copy the received message to the final destination so that it can be treated by the higher protocol layers. 
					   If there is already a message in the buffer (*puiUbxMsgLen > 0), the new message is simply appended. 
					   The higher layer is then responsible to separate the messages. */
					if ( ( pcUbxMsgBuffer != NULL ) && ( puiUbxMsgLen != NULL ) )
					{
						for ( uiBufIdx = 0; uiBufIdx < uiUbxBufferWrIdx; uiBufIdx++ )
						{
							if ( *puiUbxMsgLen + uiBufIdx < MAX_UBX_MESSAGE_LEN )
							{
								*( pcUbxMsgBuffer + *puiUbxMsgLen + uiBufIdx ) = cRdUbxBuffer[ uiBufIdx ];
							}
						}		
									
                        *puiUbxMsgLen += uiUbxBufferWrIdx;
					}
					
					/*  Extract the useful information from the UBX messages.
					    Some messages are copied as ASCII data to the UART interface. */
					xErrCode = xProcessUbxMessage( cRdUbxBuffer, uiUbxBufferWrIdx );
				}
				else
				{
					/* The message produces a checksum error. */
					NRF_LOG_ERROR( "%i UBX: Received checksum mismatch!", ulReadRTC() );	
					NRF_LOG_FLUSH();

                    xErrCode = NRF_UBX_CHKERROR;
				}
				
				/* Set the FSM to idle. */
				xI2cRdDataState = I2C_IDLE;
				
				/* Reset the write pointer. */
				uiUbxBufferWrIdx = 0;
			}
		}

		/* Treat the ASCII data. */
		if ( xI2cRdDataState == I2C_ASCII )
		{
			cRdAsciiBuffer[ uiAsciiBufferWrIdx++ ] = cRxByte;
			
			/* Check if the ASCII string is complete. */
			if ( cRxByte == ASCII_LF )
			{
				/* Verify the NMEA checksum. */
				if ( bCheckNmeaSentence( cRdAsciiBuffer, uiAsciiBufferWrIdx ) )
				{
					/* Extract the useful information from the NMEA sentences.
					   Some sentences are copied as ASCII data to the UART interface. */
					/* Convert the last 'LF' character to an end-of-string 0x00. */
					cRdAsciiBuffer[ uiAsciiBufferWrIdx ] = 0;
					xErrCode = xProcessNmeaSentence( cRdAsciiBuffer );
				}
				else
				{
					/* The checksum did not match. Discard the string. */
					NRF_LOG_ERROR( "NMEA: Received checksum mismatch!" );	
					NRF_LOG_FLUSH();

                    xErrCode = NRF_NMEA_CHKERROR;
				}
				
				/* Set the FSM to idle. */
				xI2cRdDataState = I2C_IDLE;
				
				/* Reset the write pointer. */
				uiAsciiBufferWrIdx = 0;
			}
		}
	}

	return xErrCode;
}
/*-----------------------------------------------------------*/


/* Check if there is a message from the GPS module in the UBX buffer. */
static bool bCheckForUbxMessage(uint32_t uiGpsBytesAvail )
{
	return ( uiGpsBytesAvail != 0 );
}
/*-----------------------------------------------------------*/


/* Read only the number of available bytes from the GPS module. 

   The function returns the number of bytes read.
*/
static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND xGpsAccessBytesAvail[] = { 0xfd };

nrf_twi_mngr_transfer_t const xGpsReadAvailBytes [ 2 ] =
{
    NRF_TWI_MNGR_WRITE( GPS_SLAVE_ADDR, xGpsAccessBytesAvail, sizeof( xGpsAccessBytesAvail ), NRF_TWI_MNGR_NO_STOP ),
    NRF_TWI_MNGR_READ( GPS_SLAVE_ADDR, cTransferBuffer, 2, 0 )
};

ret_code_t xGpsGetAvailBytes( uint32_t *puiGpsBytesAvail )
{
    ret_code_t  xErrCode;

    /* Get the number of available bytes for reading from the GPS module. */
    xErrCode = nrf_twi_mngr_perform( &m_nrf_twi_mngr, NULL, xGpsReadAvailBytes, 2, NULL );
	if ( xErrCode != NRF_SUCCESS )
	{
		return xErrCode;
	}

    *puiGpsBytesAvail = ( ( uint32_t )cTransferBuffer[ 0 ] << 8 ) + ( uint32_t )cTransferBuffer[ 1 ];

	return NRF_SUCCESS;
}
/*-----------------------------------------------------------*/


/* Read the number of available bytes from the GPS module. 

   The function returns the number of bytes read.
   The response is available in cTransferBuffer. 
*/

nrf_twi_mngr_transfer_t xGpsReadResponse [ 1 ] =
{
    NRF_TWI_MNGR_READ(GPS_SLAVE_ADDR, cTransferBuffer, 0, 0)
};

ret_code_t xGpsGetAvailBytesAndRead( uint8_t *pcUbxMsgBuffer, uint32_t *puiUbxMsgLen )
{
    ret_code_t  xErrCode;
	uint32_t 	uiGpsBytesAvail;

	/* Return immediately if the GPS is not running. */
	if ( xGetGpsState() != GPS_ON )
	{
		return NRF_SUCCESS;
	}

    /* Get the number of available bytes for reading from the GPS module. */
    xErrCode = xGpsGetAvailBytes( &uiGpsBytesAvail );
	if ( xErrCode != NRF_SUCCESS )
	{
		return xErrCode;
	}

	/* If there are more that 256 bytes to transfer, break the transfer into several chunks. */
	while ( ( uiGpsBytesAvail > 0 ) && ( xErrCode == NRF_SUCCESS ) )
	{
		if ( uiGpsBytesAvail > MSG_BUFFER_LEN - 1 )
		{
			xGpsReadResponse[ 0 ].length = MSG_BUFFER_LEN - 1;
            uiGpsBytesAvail -= MSG_BUFFER_LEN - 1;
		}
		else
		{
			xGpsReadResponse[ 0 ].length = uiGpsBytesAvail;
            uiGpsBytesAvail = 0;
		}

        xErrCode = nrf_twi_mngr_perform( &m_nrf_twi_mngr, NULL, xGpsReadResponse, 1, NULL );
		
		if ( xErrCode != NRF_SUCCESS )
		{
			return xErrCode;
		}
		
		/* Dispatch the data chunk into either UBX message or ASCII data buffer. Once a complete
		   UBX message or ASCII string has been received, trigger the subsequenct treatment of
		   the message. */
		xErrCode = xDispatchRdData( cTransferBuffer, xGpsReadResponse[ 0 ].length, pcUbxMsgBuffer, puiUbxMsgLen );
	}

	/* Call the function which pushes a a synopsis of the data to the trace interface for debug purposes. */
	vPushGpsFixSynopsis();

    return xErrCode;
}
/*-----------------------------------------------------------*/


/* Flush all data out of the GPS receiver.

   For this, get the available bytes an read as long as there are any available. Do not forward these data.
*/
ret_code_t xGpsFlushAvailBytes( void )
{
    ret_code_t  xErrCode;
	uint32_t 	uiGpsBytesAvail;

	/* Return immediately if the GPS is not running. */
	if ( xGetGpsState() != GPS_ON )
	{
		return NRF_SUCCESS;
	}

    /* Get the number of available bytes for reading from the GPS module. */
    xErrCode = xGpsGetAvailBytes( &uiGpsBytesAvail );
	if ( xErrCode != NRF_SUCCESS )
	{
		return xErrCode;
	}

	/* If there are more that 256 bytes to transfer, break the transfer into several chunks. */
	while ( ( uiGpsBytesAvail > 0 ) && ( xErrCode == NRF_SUCCESS ) )
	{
		if ( uiGpsBytesAvail > MSG_BUFFER_LEN - 1 )
		{
			xGpsReadResponse[ 0 ].length = MSG_BUFFER_LEN - 1;
            uiGpsBytesAvail -= MSG_BUFFER_LEN - 1;
		}
		else
		{
			xGpsReadResponse[ 0 ].length = uiGpsBytesAvail;
            uiGpsBytesAvail = 0;
		}

        xErrCode = nrf_twi_mngr_perform( &m_nrf_twi_mngr, NULL, xGpsReadResponse, 1, NULL );
		
		if ( xErrCode != NRF_SUCCESS )
		{
			return xErrCode;
		}
	}

    return xErrCode;
}
/*-----------------------------------------------------------*/


/* Read a GPS register.

   The iUbxAddress is composed of 256 * <MESSAGE CLASS> + <MESSAGE ID>.
   MESSAGE LENGTH is 0. 
   
   The UBX message is integrity-checked and copied to the address pointed to by pcUbxMsgBuffer. 
   *puiUbxMsgLen contains its length.
*/
static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND xGpsReadRegisterReq[] = { SYNC_CHAR_1, SYNC_CHAR_2, 0, 0, 0, 0, 0, 0 };

nrf_twi_mngr_transfer_t const xGpsRequestRegister[ 1 ] =
{
    NRF_TWI_MNGR_WRITE(GPS_SLAVE_ADDR, xGpsReadRegisterReq, sizeof( xGpsReadRegisterReq ), 0)
};

ret_code_t xGpsReadRegister( uint16_t uiUbxMsgClass, uint16_t uiUbxMsgID, uint8_t *pcUbxMsgBuffer, uint32_t *puiUbxMsgLen )
{
    uint16_t        uiChkSum;
    ret_code_t      xErrCode;
	TickType_t		xStartTime;
	bool			bUbxComplete;

	/* Empty all buffers. */
	( void )xGpsFlushAvailBytes();

    /* Create the request. */
    xGpsReadRegisterReq[ 2 ] = uiUbxMsgClass;						/* Message CLASS. */
    xGpsReadRegisterReq[ 3 ] = uiUbxMsgID;							/* Message ID. */
    uiChkSum = uiCalcChkSum( xGpsReadRegisterReq, sizeof( xGpsReadRegisterReq ) );
    xGpsReadRegisterReq[ 6 ] =   uiChkSum & 0x00ff;                 /* Message CHK_A. */
    xGpsReadRegisterReq[ 7 ] = ( uiChkSum & 0xff00 ) >> 8;          /* Message CHK_B. */
    
    xErrCode = nrf_twi_mngr_perform( &m_nrf_twi_mngr, NULL, xGpsRequestRegister, 1, NULL );
	if ( xErrCode != NRF_SUCCESS )
	{
		NRF_LOG_ERROR( "%i UBX: Error writing to GPS: 0x%08x.", ulReadRTC(), xErrCode );	
		NRF_LOG_FLUSH();
		return xErrCode;
	}

	/* Poll the GPS module until there is a valid, complete response. */
	xStartTime = xTaskGetTickCount();
	*puiUbxMsgLen = 0;
	bUbxComplete = false;
	while ( !bUbxComplete && ( xTaskGetTickCount() < xStartTime + GPS_REGISTER_READ_TO ) )
	{
		uint32_t		uiUbxMsgBufferIdx;

		xErrCode = xGpsGetAvailBytesAndRead( pcUbxMsgBuffer, puiUbxMsgLen );
		if ( xErrCode != NRF_SUCCESS )
		{
			NRF_LOG_ERROR( "%i UBX: Error while receiving response: 0x%08x.", ulReadRTC(), xErrCode );	
			NRF_LOG_FLUSH();
			return xErrCode;
		}

		/* Check if the received message is complete. */
		for ( uiUbxMsgBufferIdx = 0; uiUbxMsgBufferIdx < *puiUbxMsgLen; uiUbxMsgBufferIdx++ )
		{
			if ( bUbxIsComplete( pcUbxMsgBuffer, uiUbxMsgBufferIdx ) )
			{
				bUbxComplete = true;
				break;
			}
		}
	}

	/* If there was no time-out, return the corresponding status. */
	if ( xTaskGetTickCount() > xStartTime + GPS_REGISTER_READ_TO )
	{
		NRF_LOG_ERROR( "%i UBX: Error time-out in read register command response.", ulReadRTC() );	
		NRF_LOG_FLUSH();
		return NRF_ERROR_TIMEOUT;
	}

	return NRF_SUCCESS;
}
/*-----------------------------------------------------------*/


/* Write a GPS register.

   pcUbxMsgBuffer points to a UBX message in ROM. The first two characters are message class and ID 
   followed by the payload bytes. The checksum is not part of the data field.
   uiUbxMsgLen contains the length of this record. It is 2 bytes larger than the LENGTH field defined by 
   ublox. 
   Message length and checksum are calculated locally and copied into a local buffer.
   
   The function waits for a write confirmation (ACK-ACK) from the GPS module.
*/
nrf_twi_mngr_transfer_t xGpsWriteRegisterReq[ 1 ] =
{
    NRF_TWI_MNGR_WRITE(GPS_SLAVE_ADDR, NULL, 0, 0)
};

ret_code_t xGpsWriteRegister( const char *pcUbxMsg, uint32_t uiUbxMsgLen )
{
    uint16_t        uiChkSum;
    ret_code_t      xErrCode;
	TickType_t		xStartTime;
	uint32_t		uiUbxRxMsgLen;
	char			cUbxMsgBuffer[ MSG_BUFFER_LEN ];

    /* Create the request. */
    cUbxMsgBuffer[ 0 ] = SYNC_CHAR_1;
    cUbxMsgBuffer[ 1 ] = SYNC_CHAR_2;
    cUbxMsgBuffer[ 2 ] = *( pcUbxMsg + 0 );										/* Message CLASS */
    cUbxMsgBuffer[ 3 ] = *( pcUbxMsg + 1 );										/* Message ID */
    cUbxMsgBuffer[ 4 ] =   ( uiUbxMsgLen - 2 ) & 0x00ff;						/* Message LENGTH (LSB) */
    cUbxMsgBuffer[ 5 ] = ( ( uiUbxMsgLen - 2 ) & 0xff00 ) >> 8;					/* Message LENGTH (MSB) */
	memcpy( cUbxMsgBuffer + 6, pcUbxMsg + 2, uiUbxMsgLen - 2 );					/* Message body. */
    uiChkSum = uiCalcChkSum( cUbxMsgBuffer, uiUbxMsgLen + 6 );
    cUbxMsgBuffer[ uiUbxMsgLen + 4 ] =   uiChkSum & 0x00ff;						/* Message CHK_A. */
    cUbxMsgBuffer[ uiUbxMsgLen + 5 ] = ( uiChkSum & 0xff00 ) >> 8;				/* Message CHK_B. */
	xGpsWriteRegisterReq[ 0 ].p_data = cUbxMsgBuffer;
	xGpsWriteRegisterReq[ 0 ].length = uiUbxMsgLen + 6;

	/* Perform the write access. */
    xErrCode = nrf_twi_mngr_perform( &m_nrf_twi_mngr, NULL, xGpsWriteRegisterReq, 1, NULL );
	if ( xErrCode != NRF_SUCCESS )
	{
		NRF_LOG_ERROR( "%i UBX: Error writing to GPS: 0x%08x.", ulReadRTC(), xErrCode );	
		NRF_LOG_FLUSH();
		return xErrCode;
	}

	/* Poll the GPS module until there is a response. Check the response if it is the required ACK. */
	xStartTime = xTaskGetTickCount();
    uiUbxRxMsgLen = 0;
	while (   !bUbxIsACK( cUbxMsgBuffer, uiUbxRxMsgLen, *( pcUbxMsg + 0 ), *( pcUbxMsg + 1 ) ) 
	       && ( xTaskGetTickCount() < xStartTime + GPS_REGISTER_WRITE_TO ) )
	{
		xErrCode = xGpsGetAvailBytesAndRead( cUbxMsgBuffer, &uiUbxRxMsgLen );
		if ( xErrCode != NRF_SUCCESS )
		{
			NRF_LOG_ERROR( "%i UBX: Error while receiving ACK: 0x%08x.", ulReadRTC(), xErrCode );	
			NRF_LOG_FLUSH();
			return xErrCode;
		}
	}
	
	/* If there was no time-out, return the corresponding status. */
	if ( xTaskGetTickCount() >= xStartTime + GPS_REGISTER_WRITE_TO )
	{
		NRF_LOG_ERROR( "%i UBX: Error time-out in write register command response (class %x, ID %x).", ulReadRTC(), *( pcUbxMsg + 0 ), *( pcUbxMsg + 1 ) );	
		NRF_LOG_FLUSH();
		return NRF_ERROR_TIMEOUT;
	}

	return NRF_SUCCESS;
}
/*-----------------------------------------------------------*/


/* Raw write of a UBX message to the GPS module.

   *ppuiUbxMessage points to a UBX message in RAM. All fields are contained within the message.
   On exit, *ppuiUbxMessage points to the next byte following the UBX message.
   
   The function does not expect a write confirmation (ACK-ACK) from the GPS module.
*/
ret_code_t xGpsWriteRaw( uint8_t **ppuiUbxMessage )
{
    ret_code_t      xErrCode;
	uint32_t		uiUbxMsgLen;

    /* Create the request. */
	uiUbxMsgLen = *( *ppuiUbxMessage + 4 ) + ( *( *ppuiUbxMessage + 5 ) << 8 ) + 6 + 2;
	xGpsWriteRegisterReq[ 0 ].p_data = *ppuiUbxMessage;
	xGpsWriteRegisterReq[ 0 ].length = uiUbxMsgLen;

	/* Perform the write access. */
    xErrCode = nrf_twi_mngr_perform( &m_nrf_twi_mngr, NULL, xGpsWriteRegisterReq, 1, NULL );
	if ( xErrCode != NRF_SUCCESS )
	{
		NRF_LOG_ERROR( "%i UBX: Error writing to GPS: 0x%08x.", ulReadRTC(), xErrCode );	
		NRF_LOG_FLUSH();
	}
	
	/* Update pointer to point to the start of the next message. */
	*ppuiUbxMessage = *ppuiUbxMessage + uiUbxMsgLen;

	return xErrCode;
}
/*-----------------------------------------------------------*/


/* Boot the GPS module. */
void vGpsStart( void )
{
	/* Initialise the I2C interface state variables. */
	uiUbxBufferWrIdx = 0;
	uiUbxExpectedNumberOfBytes = 0;
	uiAsciiBufferWrIdx = 0;
	xI2cRdDataState = I2C_IDLE;

	if ( xGpsState == GPS_OFF )
	{
		/* Boot the GPS module. */
		nrf_gpio_cfg_output( GPS_CTRL_PWR );
		nrf_gpio_pin_set( GPS_CTRL_PWR );

		/* Wait for 400ms assuming that the module is ready then. That is what SARA is doing. */
		vTaskDelay( 426 );

		/* Reconnect the I2C transceiver block to the SDA/SCL pins. */
		vTwiConfig();
	}
	
	if ( xGpsState == GPS_OFF )
	{
		/* Remember the time stamp when the module was started. */
		xGpsOnTimeStamp = xTaskGetTickCount();
	}

    xGpsState = GPS_ON;
}
/*-----------------------------------------------------------*/

/* Shut the GPS module down. */
void vGpsStop( void )
{
	/* Disconnect the I2C transceiver block from the SDA/SCL pins. */
    nrf_twi_mngr_uninit( &m_nrf_twi_mngr );

	/* Set the SDA/SCL pins as output and low. */
    nrf_gpio_cfg_output( GPS_SDA_PIN );
    nrf_gpio_cfg_output( GPS_SCL_PIN );
	nrf_gpio_pin_clear( GPS_SDA_PIN );
	nrf_gpio_pin_clear( GPS_SCL_PIN );

    /* Shut down the GPS module. */
    nrf_gpio_pin_clear( GPS_CTRL_PWR );

	if ( xGpsState == GPS_ON )
	{
		/* Accumulated the time duration during which the module was running. */
		xGpsModuleOnDuration += xTaskGetTickCount() - xGpsOnTimeStamp;
	}

    xGpsState = GPS_OFF;
}
/*-----------------------------------------------------------*/

/* Return the GPS functional state. */
enum xGPS_STATE xGetGpsState( void )
{
    return xGpsState;
}
/*-----------------------------------------------------------*/

/* Return the GPS cumulative on-duration. */
TickType_t xGetGpsOnDuration( void )
{
	if ( xGpsState == GPS_OFF )
	{
		return xGpsModuleOnDuration;
	}
	else
	{
		return ( xGpsModuleOnDuration + xTaskGetTickCount() - xGpsOnTimeStamp );
	}
}
/*-----------------------------------------------------------*/

/* Reset the GPS cumulative on-duration, either to 0 (if the GPS is off) or to the parameter value (if GPS is on). */
void vResetGpsOnDuration( TickType_t xResetTargetValue )
{
	/* If the module is currently on, set the on-duration to the requested value. 
	   This allows for compensating for the time between capturing the time duration for sending it to the server and
       this moment where the duration is reset. */
	if ( xGpsState == GPS_ON )
	{
		xGpsModuleOnDuration = xResetTargetValue;
		xGpsOnTimeStamp = xTaskGetTickCount();
	}
	else
	{
		/* Else, just set it to 0. */
		xGpsModuleOnDuration = 0;
	}
}
/*-----------------------------------------------------------*/
