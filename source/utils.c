/*
 * Tracker Firmware
 *
 * Utility functions
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "drv_uart.h"

/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Convert a 4-bit value into a HEX-character. */
signed char cNibbleToChar( signed char cNibble );

/* Convert a HEX-character ('0'- '9', 'A' - 'F') into a 4-bit value. */
signed char cCharToNibble( signed char cChar );

/* Convert a 8-bit value into a string of HEX-characters including end-of-string 0x0. */
void vByteToHexStrg( signed char *pcStrg, unsigned char ucValue );

/* Convert a string of HEX-characters into an 8-bit value. */
unsigned char ucHexStrgToByte( signed char *pcStrg );

/* Convert a 16-bit value into a string of HEX-characters including end-of-string 0x0. */
void vShortToHexStrg( signed char *pcStrg, unsigned short usValue );

/* Convert a 16-bit value into a string of unsigned integer characters including end-of-string 0x0. */
void vShortToIntStrg( signed char *pcDest, unsigned short usValue, bool bSuppressLeadingZ );

/* Convert a string of HEX-characters in little endian in the UART Rx buffer into an 32-bit value. */
unsigned long ulHexStrgToLongLE( unsigned portBASE_TYPE uxStrgIdx );

/* Convert a string of HEX-characters into a 16-bit value. */
unsigned short usHexStrgToShort( signed char *pcStrg );

/* Convert a 32-bit value into a string of HEX-characters including end-of-string 0x0. */
void vLongToHexStrg( signed char *pcStrg, unsigned long ulValue );

/* Convert a string of HEX-characters into a 32-bit value. */
unsigned long ulHexStrgToLong( signed char *pcStrg );

/* Convert a a string of unsigned integer characters including end-of-string 0x0 into a 16-bit value. 
   The maximum size of the string is 5 characters (xLen). */
unsigned short usIntStrgToShort( signed char *pcStrg, portBASE_TYPE	xLen );

/* Return RAM string length. */
unsigned char ucStrgRAMLen( signed char *pcChar );

// /* Return ROM string length. */
// unsigned char ucStrgROMLen( PGM_P pcChar );

// /* Return EEPROM string length. */
// unsigned char ucStrgEEPROMLen( signed char *pcChar );

// /* Copy a string from ROM to RAM. */
// void vStrcpyROM2RAM( signed char *pcRAMChar, PGM_P pcROMChar, bool bTranslateApostr );

/* Return pointer to the character after the next comma ','  in a string. */
signed char *pcFindComma( signed char *pcChar, unsigned portBASE_TYPE uxMaxChar );

/* Convert double quotes in RAM string to single quotes. */
void vTranslateDoubleQuote( signed char *pcChar );

/* Convert single quotes in RAM string to double quotes. */
void vTranslateSingleQuote( signed char *pcChar );

/* Remove all messages of a specific type from a queue. All other messages remain in the 
   queue in the same order as before.
*/
void vRemoveTypeFromQueue( QueueHandle_t xQueue, unsigned portBASE_TYPE xMessageType );

/* Remove all messages which are not server commands from a queue. Server command messages remain in the 
   queue in the same order as before.
*/
void vRemoveAllMsgsFromQueue( QueueHandle_t xQueue );

/* Check, if a message of a specific type is in a queue. */
bool bMsgTypeInQueue( QueueHandle_t xQueue, unsigned portBASE_TYPE xMessageType );

/* Send a message to the specified queue if it is nbot already in there. */
BaseType_t xSendUniqueMsgToQueue( QueueHandle_t xQueue, unsigned portBASE_TYPE *xMessage, TickType_t xTicksToWait );

/* Short delay outside the scheduler to generate a delay. */
void vDelay( unsigned short ucMaxCnt );

// /* Hard-reset the system. */
// void vSystemReset( void );
/*-----------------------------------------------------------*/

/* Convert a 4-bit value into a HEX-character. */
signed char cNibbleToChar( signed char cNibble )
{
	if ( cNibble <= 9 )
	{
		return cNibble + '0';
	}
	else
	{
		return cNibble + 'A' - 10;
	}
}
/*-----------------------------------------------------------*/

/* Convert a HEX-character ('0'- '9', 'A' - 'F') into a 4-bit value. 
   Lowercase characters are converted to uppercase. Any non-HEX
   character returns 0.
*/
signed char cCharToNibble( signed char cChar )
{
	if ( ( cChar >= '0' ) && ( cChar <= '9' ) )
	{
		return cChar - '0';
	}
	else 
	{
		/* Make sure the character is uppercase. 'A' = 0x41, 'a' = 0x61. */
		cChar &= ( unsigned char )~0x20;
		
		if ( ( cChar >= 'A' ) && ( cChar <= 'F' ) )
		{
			return cChar - 'A' + 10;
		}
	}
	
	return 0;
}
/*-----------------------------------------------------------*/

/* Convert a 8-bit value into a string of HEX-characters including end-of-string 0x0. */
void vByteToHexStrg( signed char *pcStrg, unsigned char ucValue )
{
	*( pcStrg++ ) = cNibbleToChar( ( ucValue >>  4 ) & 0xf );
	*( pcStrg++ ) = cNibbleToChar( ( ucValue >>  0 ) & 0xf );
	*( pcStrg   ) = 0;
}
/*-----------------------------------------------------------*/

/* Convert a string of HEX-characters into an 8-bit value. */
unsigned char ucHexStrgToByte( signed char *pcStrg )
{
	unsigned short ucValue;
	
	ucValue  = ( cCharToNibble( *( pcStrg++ ) ) <<  4 );
	ucValue += ( cCharToNibble( *( pcStrg++ ) ) <<  0 );
	
	return ucValue;
}
/*-----------------------------------------------------------*/

/* Convert a 16-bit value into a string of HEX-characters including end-of-string 0x0. */
void vShortToHexStrg( signed char *pcStrg, unsigned short usValue )
{
	*( pcStrg++ ) = cNibbleToChar( ( usValue >> 12 ) & 0xf );
	*( pcStrg++ ) = cNibbleToChar( ( usValue >>  8 ) & 0xf );
	*( pcStrg++ ) = cNibbleToChar( ( usValue >>  4 ) & 0xf );
	*( pcStrg++ ) = cNibbleToChar( ( usValue >>  0 ) & 0xf );
	*( pcStrg   ) = 0;
}
/*-----------------------------------------------------------*/

/* Convert a 16-bit value into a string of unsigned integer characters including end-of-string 0x0. */
void vShortToIntStrg( signed char *pcStrg, unsigned short usValue, bool bSuppressLeadingZ )
{
	portBASE_TYPE	xIdx;
	unsigned short 	usTempValue;
	unsigned short 	usDivider;
	
	usDivider = 10000;
	for ( xIdx = 4; xIdx >= 0; xIdx -- )
	{
		usTempValue = usValue / usDivider;
		/* Only output the digit if it is not 0 (suppressing leading zeros) or the last digit (to have
		   at least one digit always present, even if the value is 0). */
		if ( !bSuppressLeadingZ || ( ( usTempValue > 0 )  || ( xIdx == 0 ) ) )
		{
			*( pcStrg++ ) = cNibbleToChar( usTempValue );
			usValue -= usTempValue * usDivider;
		}
		usDivider /= 10; 
	}
	
	/* Always add the end-of-string. */
	*pcStrg = 0;
}
/*-----------------------------------------------------------*/

/* Convert a string of HEX-characters into an 32-bit value. The characters are
   located inside the UART Rx buffer. 
   The value is stored in little endian (LSB first).
   TODO: move COM port index to caller.
*/
unsigned long ulHexStrgToLongLE( unsigned portBASE_TYPE uxStrgIdx )
{
	unsigned long 			ulValue;
	unsigned portBASE_TYPE	uxIdx;
	
	ulValue = 0l;
	for ( uxIdx = 0; uxIdx < 8; uxIdx += 2 )
	{
		ulValue += ( unsigned long )cCharToNibble( cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx++ ) ) << ( 4 * ( uxIdx + 1 ) );
		ulValue += ( unsigned long )cCharToNibble( cGetRxCharFromBufferWithIndex( COM0, uxStrgIdx++ ) ) << ( 4 * ( uxIdx     ) );
	}
	
	return ulValue;	
}

/*-----------------------------------------------------------*/

/* Convert a 32-bit value into a string of HEX-characters including end-of-string 0x0. */
void vLongToHexStrg( signed char *pcStrg, unsigned long ulValue )
{
	vShortToHexStrg( pcStrg, ( unsigned short )( ulValue >> 16 ) );
	pcStrg += 4;
	vShortToHexStrg( pcStrg, ( unsigned short )( ulValue & 0xffff ) );
}
/*-----------------------------------------------------------*/

/* Convert a string of HEX-characters into a 16-bit value. */
unsigned short usHexStrgToShort( signed char *pcStrg )
{
	unsigned short usValue;
	
	usValue  = ( cCharToNibble( *( pcStrg++ ) ) << 12 );
	usValue += ( cCharToNibble( *( pcStrg++ ) ) <<  8 );
	usValue += ( cCharToNibble( *( pcStrg++ ) ) <<  4 );
	usValue += ( cCharToNibble( *( pcStrg   ) ) <<  0 );
	
	return usValue;
}
/*-----------------------------------------------------------*/

/* Convert a string of HEX-characters into a 32-bit value. */
unsigned long ulHexStrgToLong( signed char *pcStrg )
{
	unsigned long ulValue;
	
	ulValue = usHexStrgToShort( pcStrg );
	ulValue <<= 16;
	ulValue += usHexStrgToShort( pcStrg + 4 );
	
	return ulValue;
}
/*-----------------------------------------------------------*/

/* Convert a a string of unsigned integer characters including end-of-string 0x0 into a 16-bit value. 
   The maximum size of the string is 5 characters (xLen). 
*/
unsigned short usIntStrgToShort( signed char *pcStrg, portBASE_TYPE	xLen )
{
	signed char		cChar;
	unsigned short 	usValue;
	
	/* Maximum length of an unsigned short expressed as HEX string. */
	usValue = 0;
	while ( ( ( cChar = *( pcStrg++ ) ) != 0 ) && ( xLen-- > 0 ) )
	{
		usValue = 10 * usValue + cCharToNibble( cChar );
	}
	
	return usValue;
}
/*-----------------------------------------------------------*/

/* Return RAM string length. */
unsigned char ucStrgRAMLen( signed char *pcChar )
{
	unsigned char ucLen;
	
	ucLen = 0;
	
	if ( pcChar != NULL )
	{
		while ( ( *( pcChar++ ) != 0 ) && ( ucLen < 255 ) )
		{
			ucLen++;
		}
	}
	
	return ucLen;
}
/*-----------------------------------------------------------*/

// /* Return ROM string length. */
// unsigned char ucStrgROMLen( PGM_P pcChar )
// {
	// unsigned char ucLen;
	
	// ucLen = 0;
	
	// if ( pcChar != NULL )
	// {
		// while ( ( pgm_read_byte( pcChar++ ) != 0 ) && ( ucLen < 255 ) )
		// {
			// ucLen++;
		// }
	// }
	
	// return ucLen;
// }
// /*-----------------------------------------------------------*/

// /* Return EEPROM string length. */
// unsigned char ucStrgEEPROMLen( signed char *pcChar )
// {
	// unsigned char ucLen;
	
	// ucLen = 0;
	
	// if ( pcChar != NULL )
	// {
		// while ( ( ucEEPROMReadByte( ( unsigned char * )pcChar++ ) != 0 ) && ( ucLen < 255 ) )
		// {
			// ucLen++;
		// }
	// }
	
	// return ucLen;
// }
// /*-----------------------------------------------------------*/

// /* Copy a string from ROM to RAM. */
// void vStrcpyROM2RAM( signed char *pcRAMChar, PGM_P pcROMChar, bool bTranslateApostr )
// {
	// signed char pcChar;
	
	// do
	// {
		// pcChar = pgm_read_byte( pcROMChar++ );
		// if ( bTranslateApostr && ( pcChar == '\'' ) )
		// {
			// pcChar = '\"';
		// }
		// *( pcRAMChar++ ) = pcChar;
	// } while ( pcChar != 0 );
// }
// /*-----------------------------------------------------------*/

/* Convert double quotes in RAM string to single quotes. */
void vTranslateDoubleQuote( signed char *pcChar )
{
	while( *pcChar != 0 )
	{
		if ( *pcChar == '\"' )
		{
			 *pcChar = '\'' ;
		}
		
		pcChar++;
	}
}
/*-----------------------------------------------------------*/

/* Convert single quotes in RAM string to double quotes. */
void vTranslateSingleQuote( signed char *pcChar )
{
	while( *pcChar != 0 )
	{
		if ( *pcChar == '\'' )
		{
			 *pcChar = '\"' ;
		}
		
		pcChar++;
	}
}
/*-----------------------------------------------------------*/

/* Return pointer to the character after the next comma ','  in a string. 
   uxMaxChar is the maximum number of characters in the field excluding the comma.
   If no comma is found, the position pcChar+uxMaxChar is returned (i.e. one position behind
   the last allowed character).
   If an en-of string is encountered before, the position after the '0' of the string is returned.
   pcChar points to the first field character.
*/
signed char *pcFindComma( signed char *pcChar, unsigned portBASE_TYPE uxMaxChar )
{
	signed char cChar;
	
	uxMaxChar++;
	
	do
	{
		cChar = *( pcChar++ );
		uxMaxChar--;
		if ( cChar == ',' )
		{
			return pcChar;
		}
	} while ( ( cChar != 0 ) && ( uxMaxChar > 0 ) );
	
	return pcChar;
}
/*-----------------------------------------------------------*/

/* Remove all messages of a specific type from a queue. All other messages remain in the 
   queue in the same order as before.
*/
void vRemoveTypeFromQueue( QueueHandle_t xQueue, unsigned portBASE_TYPE xMessageType )
{
	unsigned portBASE_TYPE	uxQueueIdx;
	unsigned portBASE_TYPE	xMessage;
	
	portENTER_CRITICAL();
	
	/* Walk through the entire queue. */
	for ( uxQueueIdx = 0; uxQueueIdx < uxQueueMessagesWaiting( xQueue ); uxQueueIdx++ )
	{
		/* Get the next queue entry. */
		xQueueReceive( xQueue, &xMessage, 0 );
		
		/* If it is not of the message type we want to remove, store it back to the queue. 
		   Else, do nothing so that the entry disappears. */
		if ( xMessage != xMessageType )
		{
			/* Send it back to the queue. */
			xQueueSend( xQueue, &xMessage, 0 );
		}
	}
	portEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

///* Remove all messages which are not server commands from a queue. Server command messages remain in the 
//   queue in the same order as before.
//*/
//void vRemoveAllMsgsFromQueue( QueueHandle_t xQueue )
//{
//	unsigned portBASE_TYPE	uxQueueIdx;
//	unsigned portBASE_TYPE	xMessage;
//	
//	portENTER_CRITICAL();
//	
//	/* Walk through the entire queue. */
//	for ( uxQueueIdx = 0; uxQueueIdx < uxQueueMessagesWaiting( xQueue ); uxQueueIdx++ )
//	{
//		/* Get the next queue entry. */
//		xQueueReceive( xQueue, &xMessage, 0 );
//		
//		/* If it is not of the message type we want to remove, store it back to the queue. 
//		   Else, do nothing so that the entry disappears. */
//		if (   ( ( xMessage >= SRVCMD_RD_VERSION ) && ( xMessage <= SRVCMD_OTHER ) ) 
//			|| ( xMessage == AT_TCP_RX_DATA_IND )
//		   )
//		{
//			/* Send it back to the queue. */
//			xQueueSend( xQueue, &xMessage, 0 );
//		}
//	}
//	portEXIT_CRITICAL();
//}
///*-----------------------------------------------------------*/

/* Check, if a message of a specific type is in a queue. */
bool bMsgTypeInQueue( QueueHandle_t xQueue, unsigned portBASE_TYPE xMessageType )
{
	portBASE_TYPE			uxQueueIdx;
	unsigned portBASE_TYPE	xMessage;
	bool					msgInQueue;
	
	portENTER_CRITICAL();
	
	msgInQueue = false;
	
	/* Walk through the entire queue. */
	for ( uxQueueIdx = 0; uxQueueIdx < uxQueueMessagesWaiting( xQueue ); uxQueueIdx++ )
	{
		/* Get the next queue entry. */
		xQueueReceive( xQueue, &xMessage, 0 );
		
		/* Set a flag if it is of the message type we want to find. */
		if ( xMessage == xMessageType )
		{
			msgInQueue = true;
		}
		/* Send it back to the queue. */
		xQueueSend( xQueue, &xMessage, 0 );		
	}
	portEXIT_CRITICAL();
	
	return msgInQueue;
}
/*-----------------------------------------------------------*/

/* Send a message to the specified queue if it is nbot already in there. */
BaseType_t xSendUniqueMsgToQueue( QueueHandle_t xQueue, unsigned portBASE_TYPE *xMessage, TickType_t xTicksToWait )
{

	if ( !bMsgTypeInQueue( xQueue, *xMessage ) )
	{
		return xQueueSend( xQueue, xMessage, xTicksToWait );
	}
	
	return pdTRUE;
}
/*-----------------------------------------------------------*/
 
// /* Short delay outside the scheduler to generate a delay. 
   // ucMaxCnt:  ~53 per ms delay @ 1MHz.
// */
// void vDelay( unsigned short ucMaxCnt )
// {
	// volatile unsigned short	ucCnt;
	
	// for ( ucCnt = 0; ucCnt < ucMaxCnt; ucCnt++ )
	// {
		// asm volatile ( "wdr" );			/* Stroke the watchdog. */
	// }
// }
// /*-----------------------------------------------------------*/

// /* Hard-reset the system. */
// void vSystemReset( void )
// {
	// /* A software reset will also release all GPIOs so that all switched power supplies will be off. */
	// reset_do_soft_reset();
// }
// /*-----------------------------------------------------------*/
