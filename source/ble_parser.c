/*
 * Tracker Firmware
 *
 * BLE AT message parser task
 *
 * This task runs with the highest priority. It is triggered by the BLE UART RX interrupt
 * which detects a carriage-return line-feed indicating end of an AT response.
 * The RX interrut routine sends a semaphore triggering the parser. 
 * The parser compares the received string simultaneously against all reference
 * strings. If a match is found, it stores the remainder of the received string as
 * parameter into the location pointed to in the table containing all references
 * and sends an indication to the BLE task.
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
#include "main.h"
#include "version.h"
#include "drv_uart.h"
#include "ble_parser.h"
#include "ble_main.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
void vBleParserInit( UBaseType_t uxPriority );
/* The parser task as described at the top of the file. */
static portTASK_FUNCTION_PROTO( vBleParserTask, pvParameters );
/* Locate a data field by number in a comma-separated string in the UART buffer and return its index in the buffer. */
unsigned portBASE_TYPE  uxLocateInUARTBuffer( unsigned portBASE_TYPE uxStrgIdx, unsigned portBASE_TYPE uxFieldNum );
/* Copy a number of characters from the UART buffer to a specified destination. */
static void prvCopyNFromUartBuffer( unsigned portBASE_TYPE uxStrgIdx, signed char *pcDest, unsigned portBASE_TYPE uxCnt );
/* Send an 'OK' response. */
static void vOk( unsigned portBASE_TYPE uxStrgIdx );
/* Send an 'OK' response with partial match only. */
static void vOkNoEOLCheck( unsigned portBASE_TYPE uxStrgIdx );
/* Return version string. */
static void vGetVersion( unsigned portBASE_TYPE uxStrgIdx );
/* Return device address string. */
static void vGetBTAddr( unsigned portBASE_TYPE uxStrgIdx );
/* Set short range advertising data. */
static void vSetStd1MbpsAdvData( unsigned portBASE_TYPE uxStrgIdx );
/* Get short range advertising data. */
static void vGetStd1MbpsAdvData( unsigned portBASE_TYPE uxStrgIdx );
/* Set long range advertising data. */
static void vSetLR125kbpsAdvData( unsigned portBASE_TYPE uxStrgIdx );
/* Get long range advertising data. */
static void vGetLR125kbpsAdvData( unsigned portBASE_TYPE uxStrgIdx );
/* Set advertising state. */
static void vSetAdvState( unsigned portBASE_TYPE uxStrgIdx );
/* Get advertising state. */
static void vGetAdvState( unsigned portBASE_TYPE uxStrgIdx );
/* Set scan state. */
static void vSetScanState( unsigned portBASE_TYPE uxStrgIdx );
/* Get scan state. */
static void vGetScanState( unsigned portBASE_TYPE uxStrgIdx );
/* Set output power. */
static void vSetPower( unsigned portBASE_TYPE uxStrgIdx );
/* Get output power. */
static void vGetPower( unsigned portBASE_TYPE uxStrgIdx );
/*-----------------------------------------------------------*/

/* Global variables. */
QueueHandle_t 			xBleParserAtCmdQueue; 	/* BLE AT response indicator queue handle. */
/*-----------------------------------------------------------*/

/* Module scope variables. */
/* Definition of the AT command strings used to compare the actual
   responses received over the UART. The strings are stored in program
   memory. */
char pcAt_At[] 			= "AT";	
char pcAt_I9[] 			= "ATI9";	
char pcAt_UMLa[]		= "AT+UMLA?";	
char pcAt_UBtAdS[]		= "AT+UBTADS=";	
char pcAt_UBtAdSQ[]		= "AT+UBTADS?";	
char pcAt_UBtAdL[]		= "AT+UBTADL=";	
char pcAt_UBtAdLQ[]		= "AT+UBTADL?";	
char pcAt_UBtA[]		= "AT+UBTA=";
char pcAt_UBtAQ[]		= "AT+UBTA?";
char pcAt_UBtS[]		= "AT+UBTS=";
char pcAt_UBtSQ[]		= "AT+UBTS?";
char pcAt_UBtPwr[]		= "AT+UBTPWR=";
char pcAt_UBtPwrQ[]		= "AT+UBTPWR?";

/* Table containing pointers to the AT response strings, parameters to be stored
   (if any) and the AT response ID to be sent to the BLE task via queue. */
static const struct xAT_CMD xAtCmd[] = 
{
	 { pcAt_I9, 	 				vGetVersion,				AT_NOMSG						},
	 { pcAt_UMLa, 	 				vGetBTAddr,					AT_NOMSG						},
	 { pcAt_UBtAdS,  				vSetStd1MbpsAdvData,		AT_NOMSG						},
	 { pcAt_UBtAdSQ, 				vGetStd1MbpsAdvData,		AT_NOMSG						},
	 { pcAt_UBtAdL,  				vSetLR125kbpsAdvData,		AT_NOMSG						},
	 { pcAt_UBtAdLQ, 				vGetLR125kbpsAdvData,		AT_NOMSG						},
	 { pcAt_UBtA,	 				vSetAdvState,				AT_NOMSG						},
	 { pcAt_UBtAQ,	 				vGetAdvState,				AT_NOMSG						},
	 { pcAt_UBtS,	 				vSetScanState,				AT_NOMSG						},
	 { pcAt_UBtSQ,	 				vGetScanState,				AT_NOMSG						},
	 { pcAt_UBtPwr,	 				vSetPower,					AT_NOMSG						},
	 { pcAt_UBtPwrQ, 				vGetPower,					AT_NOMSG						},
	 { pcAt_At, 	 				vOk,						AT_NOMSG						}
};
/*-----------------------------------------------------------*/

void vBleParserInit( UBaseType_t uxPriority )
{
	/* Create a queue to the GSM task for AT response indications. */
	xBleParserAtCmdQueue = xQueueCreate( bleParserAT_CMD_QUEUE_SIZE, ( unsigned portBASE_TYPE ) sizeof( enum xAT_MSG_ID ) );
	
	#if defined ( DEBUG )	
		/* Register queue for inspection with a kernel-aware debugger. */
		vQueueAddToRegistry( xBleParserAtCmdQueue, "BLE Parser AT resp");
	#endif

	/* The BLE Parser task is spawned here. */
	xTaskCreate( vBleParserTask, "BPR", bleParserSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
}
/*-----------------------------------------------------------*/

/* Locate a data field by number in a comma-separated string in the UART buffer and return its index in the buffer.
   The first field has field number uxFieldNum = 0.
*/
unsigned portBASE_TYPE  uxLocateInUARTBuffer( unsigned portBASE_TYPE uxStrgIdx, unsigned portBASE_TYPE uxFieldNum )
{
	unsigned portBASE_TYPE	uxCnt;
	signed char				cChar;
	
	/* Locate the uxFieldNum-th comma-separated data field, i.e. find and skip uxFieldNum commas. */
	for ( uxCnt = 0; uxCnt < uxFieldNum; uxCnt++ )
	{
		cChar = 0;
		
		while ( ( cChar = cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx++ ) ) != ',' )
		{
			;
		}
	}
	
	return uxStrgIdx;	
}
/*-----------------------------------------------------------*/

/* Copy a number of characters from the UART buffer to a specified destination. 
*/
static void prvCopyNFromUartBuffer( unsigned portBASE_TYPE uxStrgIdx, signed char *pcDest, unsigned portBASE_TYPE uxCnt )
{
	signed char				cChar;
	
	/* Copy the data field. */
	while ( ( uxCnt-- > 0 ) && ( ( cChar = cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx++ ) ) != 0 ) )
	{
		*pcDest++ = cChar;
	}
	
	*pcDest = 0;
}
/*-----------------------------------------------------------*/
	
/* Return an 'OK' response. */
static void vOk( unsigned portBASE_TYPE uxStrgIdx )
{
	/* Check, if the received string is terminated here. In this case, send the 'OK' response. */
	if ( cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx ) == 0 )
	{
		xComSendStringRAM( COM0, "\n\rOK\n\r" );
	}
	else
	{
		/* We got here because the parser detected an "AT". However, there were characters after the command
		   which did not match any other command. So we return a syntax error. */
		xComSendStringRAM( COM0, "\r\nERROR\r\n" );
	}
}
/*-----------------------------------------------------------*/
	
/* Return an 'OK' response without checking for complete match. */
static void vOkNoEOLCheck( unsigned portBASE_TYPE uxStrgIdx )
{
	/* Send the 'OK' response without checking if received string is terminated. */
	xComSendStringRAM( COM0, "\n\rOK\n\r" );
}
/*-----------------------------------------------------------*/
	
/* Return version string. */
static void vGetVersion( unsigned portBASE_TYPE uxStrgIdx )
{
	#ifdef DEBUG
		static char 		cFwVersion[] = "\r\n" VERSION "-DBG FW=";
	#else
		static char 		cFwVersion[] = "\r\n" VERSION "-REL FW=";
	#endif
	static char 		cFwVersion2[] = "\r\nOK\r\n";

	/* Check, if the received string is terminated here. In this case, send the 'OK' response. */
	if ( cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx ) == 0 )
	{
		xComSendStringRAM( COM0, cFwVersion );
		xComSendStringRAM( COM0, cGitVersion );
		xComSendStringRAM( COM0, cFwVersion2 );
	}
}
/*-----------------------------------------------------------*/
	
/* Return BT device address string. */
static void vGetBTAddr( unsigned portBASE_TYPE uxStrgIdx )
{
	ble_gap_addr_t	xBleGapAddr;
	char			cBtAddr[ 3 ];
	portBASE_TYPE	xIdx;
	
	/* Retrieve BLE GAP address. */
	sd_ble_gap_addr_get( &xBleGapAddr );

	xComSendStringRAM( COM0, "\r\n+UMLA:" );
	for ( xIdx = 0; xIdx < 6; xIdx++)
	{
		vByteToHexStrg( cBtAddr, xBleGapAddr.addr[ 5 - xIdx ] );
		xComSendStringRAM( COM0, cBtAddr );
	}
	xComSendStringRAM( COM0, "\r\nOK\r\n" );	
}
/*-----------------------------------------------------------*/
	
/* Set short range advertising data.
   Setting the advertisement data will also start advertising. 
   Setting the advertisement data to a string of length 0 will stop advertising. */
static void vSetStd1MbpsAdvData( unsigned portBASE_TYPE uxStrgIdx )
{
	signed char 		cLocalAdvData[ 129 ];
	portBASE_TYPE		xChrIdx;
	
	/* Copy the advertisement data to local buffer (HEX format, max. 64 bytes). */
	prvCopyNFromUartBuffer( uxStrgIdx, cLocalAdvData, 128 );
	
	if ( strlen( cLocalAdvData ) > 0 )
	{
		/* Convert to binary and store in global advertisement data buffer. 
		   Leave space for the advertisement flag field (requires 3 bytes). */
		for ( xChrIdx = 0; xChrIdx < strlen( cLocalAdvData ); xChrIdx += 2 )
		{
			cEncodedStd1MbpsAdvData[ ( xChrIdx >> 1 ) + 3 ] = ucHexStrgToByte( cLocalAdvData + xChrIdx );
		}
	
		/* Add the default advertising flags in the payload. */
		cEncodedStd1MbpsAdvData[ 0 ] = 0x02;			/* AD length: 2 bytes. */
		cEncodedStd1MbpsAdvData[ 1 ] = 0x01;			/* AD Type: 1 = advertiser flags. */
		cEncodedStd1MbpsAdvData[ 2 ] = 0x04;			/* AD Flag value: not discoverable, BR/EDR not supported. */

		/* Set the total payload length. */
		xEncodedStd1MbpsAdvDataLen = ( strlen( cLocalAdvData ) >> 1 ) + 3;
	}
	
	/* Restart advertising in the same state so that the new data gets correctly taken into account. */
	vStartAdvertising( xGetAdvState() );

	xComSendStringRAM( COM0, "\r\nOK\r\n");
}
/*-----------------------------------------------------------*/
	
/* Get short range advertising data. */
static void vGetStd1MbpsAdvData( unsigned portBASE_TYPE uxStrgIdx )
{
	portBASE_TYPE		xChrIdx;
	signed char			cOctetStrg[ 3 ];
	
	/* Check, if the received string is terminated here. In this case, treat the command. */
	if ( cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx++ ) == 0 )
	{
		xComSendStringRAM( COM0, "\r\n+UBTAD:" );
		
		for ( xChrIdx = 0; xChrIdx < xEncodedStd1MbpsAdvDataLen; xChrIdx++ )
		{
			vByteToHexStrg( cOctetStrg, cEncodedStd1MbpsAdvData[ xChrIdx ] );
			xComSendStringRAM( COM0, cOctetStrg );
		}
		
		xComSendStringRAM( COM0, "\r\nOK\r\n" );
	}
}
/*-----------------------------------------------------------*/
	
/* Set long range advertising data.
   Setting the advertisement data will also start advertising. 
   Setting the advertisement data to a string of length 0 will stop advertising. */
static void vSetLR125kbpsAdvData( unsigned portBASE_TYPE uxStrgIdx )
{
	signed char 		cLocalAdvData[ 2 * 256 + 1 ];
	portBASE_TYPE		xChrIdx;
	
	/* Copy the advertisement data to local buffer (HEX format, max. 256 bytes). */
	prvCopyNFromUartBuffer( uxStrgIdx, cLocalAdvData, 256 );
	
	if ( strlen( cLocalAdvData ) > 0 )
	{
		/* Convert to binary and store in global advertisement data buffer. 
		   Leave space for the advertisement flag field (requires 3 bytes). */
		for ( xChrIdx = 0; xChrIdx < strlen( cLocalAdvData ); xChrIdx += 2 )
		{
			cEncodedLR125kbpsAdvData[ ( xChrIdx >> 1 ) + 3 ] = ucHexStrgToByte( cLocalAdvData + xChrIdx );
		}
	
		/* Add the default advertising flags in the payload. */
		cEncodedLR125kbpsAdvData[ 0 ] = 0x02;			/* AD length: 2 bytes. */
		cEncodedLR125kbpsAdvData[ 1 ] = 0x01;			/* AD Type: 1 = advertiser flags. */
		cEncodedLR125kbpsAdvData[ 2 ] = 0x04;			/* AD Flag value: not discoverable, BR/EDR not supported. */

		/* Set the total payload length. */
		xEncodedLR125kbpsAdvDataLen = ( strlen( cLocalAdvData ) >> 1 ) + 3;
	}
	
	/* Restart advertising in the same state so that the new data gets correctly taken into account. */
	vStartAdvertising( xGetAdvState() );

	xComSendStringRAM( COM0, "\r\nOK\r\n");
}
/*-----------------------------------------------------------*/
	
/* Get long range advertising data. */
static void vGetLR125kbpsAdvData( unsigned portBASE_TYPE uxStrgIdx )
{
	portBASE_TYPE		xChrIdx;
	signed char			cOctetStrg[ 3 ];
	
	/* Check, if the received string is terminated here. In this case, treat the command. */
	if ( cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx++ ) == 0 )
	{
		xComSendStringRAM( COM0, "\r\n+UBTAD:" );
		
		for ( xChrIdx = 0; xChrIdx < xEncodedLR125kbpsAdvDataLen; xChrIdx++ )
		{
			vByteToHexStrg( cOctetStrg, cEncodedLR125kbpsAdvData[ xChrIdx ] );
			xComSendStringRAM( COM0, cOctetStrg );
		}
		
		xComSendStringRAM( COM0, "\r\nOK\r\n" );
	}
}
/*-----------------------------------------------------------*/

/* Set advertising state. */
static void vSetAdvState( unsigned portBASE_TYPE uxStrgIdx )
{
	signed char			cSelection;
	
	cSelection = cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx );
	
	if ( cSelection == '0' )
	{
		vStopAdvertising();
	}
	else
	{
		vStartAdvertising( cCharToNibble( cSelection ) );
	}
	xComSendStringRAM( COM0, "\r\nOK\r\n" );
}
/*-----------------------------------------------------------*/

/* Get advertising state. */
static void vGetAdvState( unsigned portBASE_TYPE uxStrgIdx )
{
	signed char		cRespStrg[ 24 ];
	
	/* Check, if the received string is terminated here. In this case, treat the command. */
	if ( cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx++ ) == 0 )
	{
		sprintf( cRespStrg, "\r\n+UBTA:%i\r\nOK\r\n", xGetAdvState() );
		xComSendStringRAM( COM0, cRespStrg );
	}	
}
/*-----------------------------------------------------------*/

/* Set scan mode. */
static void vSetScanState( unsigned portBASE_TYPE uxStrgIdx )
{
	signed char			cSelection;
	
	cSelection = cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx );
	
	vStopScan();
	
	if ( cSelection != '0' )
	{
		vStartScan( cCharToNibble( cSelection ) );
	}
	xComSendStringRAM( COM0, "\r\nOK\r\n" );
}
/*-----------------------------------------------------------*/

/* Get scan mode. */
static void vGetScanState( unsigned portBASE_TYPE uxStrgIdx )
{
	signed char		cRespStrg[ 24 ];
	
	/* Check, if the received string is terminated here. In this case, treat the command. */
	if ( cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx++ ) == 0 )
	{
		sprintf( cRespStrg, "\r\n+UBTS:%i\r\nOK\r\n", xGetScanState() );
		xComSendStringRAM( COM0, cRespStrg );
	}	
}
/*-----------------------------------------------------------*/

/* Set output power. */
static void vSetPower( unsigned portBASE_TYPE uxStrgIdx )
{
	signed char			cMode;
	signed char			cPower;
	bool				bError;
	
	bError = false;
	
	cMode = cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx++ );
	if ( cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx++ ) != ',' )
	{
		bError = true;
	}
	cPower = cCharToNibble( cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx ) );	

	/* Check and translate the values. */
	switch ( cPower )
	{
		case 0:		cPower = OUTPUT_PWR_0_dBm;	break;
		case 4:		cPower = OUTPUT_PWR_4_dBm;	break;
		case 6:		cPower = OUTPUT_PWR_6_dBm;	break;
		case 8:		cPower = OUTPUT_PWR_8_dBm;	break;
		default:	bError = true;				break;
	}
	
	if ( ( cMode == '1' ) && !bError )
	{
		cTxPower1Mbps = cPower;
	}
	else if ( ( cMode == '4' ) && !bError )
	{
		cTxPower125kbps = cPower;
	}
	else
	{
		bError = true;
	}
	
	if ( !bError )
	{
		/* Set the values. */
		xComSendStringRAM( COM0, "\r\nOK\r\n" );
	}
	else
	{
		xComSendStringRAM( COM0, "\r\nERROR\r\n" );
	}
}
/*-----------------------------------------------------------*/

/* Get output power. */
static void vGetPower( unsigned portBASE_TYPE uxStrgIdx )
{
	signed char		cRespStrg[ 24 ];
	
	/* Check, if the received string is terminated here. In this case, treat the command. */
	if ( cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx++ ) == 0 )
	{
		sprintf( cRespStrg, "\r\n+UBTPWR:1,%i;4,%i\r\nOK\r\n", cTxPower1Mbps, cTxPower125kbps );
		xComSendStringRAM( COM0, cRespStrg );
	}	
}
/*-----------------------------------------------------------*/

static portTASK_FUNCTION( vBleParserTask, pvParameters )
{
	/* Just to stop compiler warnings. */
	( void ) pvParameters;

	while ( 1 )
	{
		unsigned portBASE_TYPE	xCmdIdx = 0;
		bool					atCmdFound = false;
		unsigned portBASE_TYPE	uxStrgIdx;
		signed char				cAtCmd;
		signed char				*pcAtRef;
		signed char				cAtRef;
		
		/* Wait indefinitely for a string to arrive on the BLE UART. */
		( void )xComReceiveString( COM0, portMAX_DELAY );
		
			/* Scan all strings in the AT command table to find a match. */
			while ( ( !atCmdFound ) && ( xCmdIdx < ( portBASE_TYPE )( sizeof( xAtCmd ) / sizeof( xAtCmd[ 0 ] ) ) ) )
			{
				/* Check if the response pointed to by xCmdIdx is a match. */
				uxStrgIdx = 0;
				do
				{
					cAtCmd = cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx );
					pcAtRef = xAtCmd[ xCmdIdx ].pcAtCommand;
					cAtRef = *( pcAtRef + uxStrgIdx );
					/* A '*' means: match any character. */
					if ( ( cAtCmd != 0 ) && ( cAtRef == '*' ) )
					{
						cAtCmd = cAtRef;
					}
					uxStrgIdx++;
				} 
				while (    ( cAtCmd != 0 ) 
						&& ( cAtRef != 0 )
						&& ( cAtRef == cAtCmd ) );

				/* A match has been found if the end of the reference string has been reached and 
				   no mismatch has been encountered before. */
				if ( cAtRef == 0 )
				{
					atCmdFound = true;
					uxStrgIdx--;
				}
				else
				{
					/* No match: select next entry in the command table. */
					xCmdIdx++;
				}
			}
			
			/* Treat the command, if a match was found in the table. The resulting index is xCmdIdx. */
			if ( xCmdIdx < sizeof( xAtCmd ) / sizeof( xAtCmd[0] ) )
			{
				enum xAT_MSG_ID xMsgId;
				void			( *pcParameterParser )( unsigned portBASE_TYPE	uxStrgIdx );
			
				/* If there is a parameter to be taken, store it in the indicated location. */
				pcParameterParser = xAtCmd[ xCmdIdx ].prvParamParser;
				if ( pcParameterParser != NULL )
				{
					/* Updating the parameters need to be protected by a mutex as they might be read anytime by the GSM task.
					   The mutex avoids that the GSM task reads inconsistent data. Care must be taken that the GSM task
					   takes the mutex only for a very short, determined time as the mutex may result in priority inversion. 
					   Also, it must be ensured that there is only one mutex used between GSM and Parser task, else the system
					   might enter a deadlock. */
					pcParameterParser( uxStrgIdx );
				}
				
				/* If required, send an indication to the BLE task. */
				xMsgId = xAtCmd[ xCmdIdx ].xAtMsgId;
				if ( xMsgId != AT_NOMSG )
				{
					if ( xQueueSend( xBleParserAtCmdQueue, &xMsgId, bleParserAT_CMD_BLOCKTIME ) != pdPASS )
					{
						/* The indication could not be sent for a certain time. 
						   TODO: log event. */
					}
				}
			}
			else
			{
				/* No match was found in the table. Return a syntax error. */
				xComSendStringRAM( COM0, "\r\nERROR\r\n" );
			}
			
			/* Remove the received string from the BLE UART RX ring buffer. */
			vUartRemoveString( COM0 );
	}
}		
/*-----------------------------------------------------------*/