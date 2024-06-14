/*
 * Tracker Firmware
 *
 * Utility functions
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#include "tracker.h"

/* nRF SDK include files */
#define NRF_LOG_MODULE_NAME 		UTILS
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_log_backend_flash.h"
#include "nrf_delay.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* Device specific include files. */
#include "custom_board.h"
#include "tracemsg.h"

#include "drv_adc.h"
#include "drv_uart.h"
#include "drv_nvm.h"
#include "main.h"
#include "parser.h"
#include "rtc.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */

/* Implementation  of the Linux-type stpncpy() function. */
char *stpncpy ( char *pcDst, const char *pcSrc, size_t xLen );

/* Check if a memory block is completely filled with a certain value. */
bool bMemVCmp( char *pcSrc, const char cVal, size_t xLen );

/* Convert a 4-bit value into a HEX-character. */
signed char cNibbleToChar( signed char cNibble );

/* Convert a HEX-character ('0'- '9', 'A' - 'F') into a 4-bit value. */
signed char cCharToNibble( signed char cChar );

/* Convert a 8-bit value into a string of HEX-characters including end-of-string 0x0. */
void vByteToHexStrg( signed char *pcStrg, unsigned char ucValue );

/* Convert a 8-bit value into a string of unsigned integer characters including end-of-string 0x0. */
void vByteToIntStrg( signed char *pcStrg, unsigned char ucValue, bool bSuppressLeadingZ );

/* Convert a a string of unsigned integer characters including end-of-string 0x0 into a 8-bit value. */
unsigned char ucIntStrgToByte( signed char *pcStrg, portBASE_TYPE xLen );

/* Convert a string of HEX-characters into an 8-bit value. */
unsigned char ucHexStrgToByte( signed char *pcStrg );

/* Convert a 16-bit value into a string of HEX-characters including end-of-string 0x0. */
void vShortToHexStrg( signed char *pcStrg, unsigned short usValue );

/* Convert a 16-bit value into a string of unsigned integer characters including end-of-string 0x0. */
void vShortToIntStrg( signed char *pcDest, unsigned short usValue, bool bSuppressLeadingZ );

/* Convert a a string of unsigned integer characters including end-of-string 0x0 into a 16-bit value. */
unsigned short usIntStrgToShort( signed char *pcStrg, portBASE_TYPE	xLen );

/* Convert a a string of unsigned long integer characters including end-of-string 0x0 into a 32-bit value. */
unsigned long ulIntStrgToLong( signed char *pcStrg, portBASE_TYPE xLen );

/* Convert a string of HEX-characters in little endian in the UART Rx buffer into an 32-bit value. */
unsigned long ulHexComStrgToLongLE( unsigned portBASE_TYPE xComID, unsigned portBASE_TYPE uxStrgIdx );

/* Convert a string of HEX-characters into a 16-bit value. */
unsigned short usHexStrgToShort( signed char *pcStrg );

/* Convert a 32-bit value into a string of HEX-characters including end-of-string 0x0. */
void vLongToHexStrg( signed char *pcStrg, unsigned long ulValue );

/* Convert a string of HEX-characters into a 32-bit value. */
unsigned long ulHexStrgToLong( signed char *pcStrg );

/* Return pointer to the character after the next comma ','  in a string. */
signed char *pcFindNComma( signed char *pcChar, unsigned portBASE_TYPE uxN, unsigned portBASE_TYPE uxMaxChar );

/* Compare two 16-bit time stamps and determine which is later. */
bool bIsANewerThanB( uint16_t a, uint16_t b );

/* Convert double quotes in RAM string to single quotes. */
void vTranslateDoubleQuote( signed char *pcChar );

/* Convert single quotes in RAM string to double quotes. */
void vTranslateSingleQuote( signed char *pcChar );

/* Calculate the logarithm to the base of 2 (log2() or ld()) using fixed-point arithmetic. */
unsigned long nlog2_16 ( unsigned short x );

/* Remove all messages of a specific type from a queue. All other messages remain in the 
   queue in the same order as before.
*/
void vRemoveTypeFromQueue( QueueHandle_t xQueue, unsigned portBASE_TYPE xMessageType );

/* Remove all messages which are not server commands from a queue. Server command messages remain in the 
   queue in the same order as before.
*/
void vRemoveAllMsgsFromQueue( QueueHandle_t xQueue );

/* Check, if a message of a specific type is in a queue. */
bool bMsgTypeInQueue( QueueHandle_t xQueue, void *pxMessageType );

/* Send a message to the specified queue if it is nbot already in there. */
BaseType_t xSendUniqueMsgToQueue( QueueHandle_t xQueue, const void * const xMessage, TickType_t xTicksToWait );

/* Dump data in hex format to the logger. */
void vDebugHexDump( char *pcMemory, uint32_t	uiSize );

/* Hard-reset the system. */
void vSystemReset( uint32_t uiErrorCode, uint32_t uiLineNum, const char * pcFileName );
/*-----------------------------------------------------------*/

/* Copy a string from pcSrc to pcDst. Limit the string size to xLen. Return a pointer to the end marker 0x00 of the destination string. */
char *stpncpy( char *pcDst, const char *pcSrc, size_t xLen )
{
	size_t xN = strlen( pcSrc );
	
	if ( xN > xLen )
	{
		xN = xLen;
	}
	return strncpy( pcDst, pcSrc, xLen ) + xN;
}

/*-----------------------------------------------------------*/

/* Check if a memory block is completely filled with a certain value. */
bool bMemVCmp( char *pcSrc, const char cVal, size_t xLen )
{
	bool						bResult = true;
	unsigned portBASE_TYPE		uxIdx;
	
	for ( uxIdx = 0; ( uxIdx < xLen ) && bResult; ++uxIdx )
	{
		bResult &= ( *( pcSrc + uxIdx ) ) == cVal;
	}

	return bResult;
}
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

/* Convert a 8-bit value into a string of unsigned integer characters including end-of-string 0x0. */
void vByteToIntStrg( signed char *pcStrg, unsigned char ucValue, bool bSuppressLeadingZ )
{
	portBASE_TYPE	xIdx;
	unsigned char 	ucTempValue;
	unsigned char 	ucDivider;
	bool			bNonZeroDigitPrinted;
	
	ucDivider = 100;
	bNonZeroDigitPrinted = false;
	for ( xIdx = 2; xIdx >= 0; xIdx -- )
	{
		ucTempValue = ucValue / ucDivider;
		/* Only output the digit if it is not 0 (suppressing leading zeros) or the last digit (to have
		   at least one digit always present, even if the value is 0). */
		if ( !bSuppressLeadingZ || bNonZeroDigitPrinted || ( ucTempValue > 0 )  || ( xIdx == 0 ) )
		{
			*( pcStrg++ ) = cNibbleToChar( ucTempValue );
			ucValue -= ucTempValue * ucDivider;
			bNonZeroDigitPrinted = true;
		}
		ucDivider /= 10; 
	}
	
	/* Always add the end-of-string. */
	*pcStrg = 0;
}
/*-----------------------------------------------------------*/

/* Convert a a string of unsigned integer characters including end-of-string 0x0 into a 8-bit value.
   The maximum size of the string is 3 characters (xLen). 
   Abort, if a ',' is read instead of a digit.
*/
unsigned char ucIntStrgToByte( signed char *pcStrg, portBASE_TYPE xLen )
{
	signed char		cChar;
	unsigned char 	ucValue;
	
	/* Maximum length of an unsigned short expressed as HEX string. */
	ucValue = 0;
	while ( ( ( cChar = *( pcStrg++ ) ) != 0 ) && ( xLen-- > 0 ) )
	{
		if (   ( cChar != ',' )
			&& ( cChar != '\r' )
		    && ( cChar != '\n' ) )
		{
			ucValue = 10 * ucValue + cCharToNibble( cChar );
		}
		else
		{
			xLen = 0;
		}
	}
	
	return ucValue;
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
	bool			bNonZeroDigitPrinted;
	
	usDivider = 10000;
	bNonZeroDigitPrinted = false;
	for ( xIdx = 4; xIdx >= 0; xIdx -- )
	{
		usTempValue = usValue / usDivider;
		/* Only output the digit if it is not 0 (suppressing leading zeros) or the last digit (to have
		   at least one digit always present, even if the value is 0). */
		if ( !bSuppressLeadingZ || bNonZeroDigitPrinted || ( ( usTempValue > 0 )  || ( xIdx == 0 ) ) )
		{
			*( pcStrg++ ) = cNibbleToChar( usTempValue );
			usValue -= usTempValue * usDivider;
			bNonZeroDigitPrinted = true;
		}
		usDivider /= 10; 
	}
	
	/* Always add the end-of-string. */
	*pcStrg = 0;
}
/*-----------------------------------------------------------*/

/* Convert a 32-bit value into a string of unsigned integer characters including end-of-string 0x0. */
void vLongToIntStrg( signed char *pcStrg, unsigned long ulValue, bool bSuppressLeadingZ )
{
	portBASE_TYPE	xIdx;
	unsigned long 	ulTempValue;
	unsigned long 	ulDivider;
	bool			bNonZeroDigitPrinted;
	
	ulDivider = 1000000000;
	bNonZeroDigitPrinted = false;
	for ( xIdx = 9; xIdx >= 0; xIdx -- )
	{
		ulTempValue = ulValue / ulDivider;
		/* Only output the digit if it is not 0 (suppressing leading zeros) or the last digit (to have
		   at least one digit always present, even if the value is 0). */
		if ( !bSuppressLeadingZ || bNonZeroDigitPrinted || ( ( ulTempValue > 0 )  || ( xIdx == 0 ) ) )
		{
			*( pcStrg++ ) = cNibbleToChar( ulTempValue );
			ulValue -= ulTempValue * ulDivider;
			bNonZeroDigitPrinted = true;
		}
		ulDivider /= 10; 
	}
	
	/* Always add the end-of-string. */
	*pcStrg = 0;
}
/*-----------------------------------------------------------*/

/* Convert a a string of unsigned integer characters including end-of-string 0x0 into a 16-bit value. 
   The maximum size of the string is 5 characters (xLen). 
   '"', \r or \n terminates the conversion.
*/
unsigned short usIntStrgToShort( signed char *pcStrg, portBASE_TYPE xLen )
{
	signed char		cChar;
	unsigned short 	usValue;
	
	/* Maximum length of an unsigned short expressed as HEX string. */
	usValue = 0;
	while (    ( ( cChar = *pcStrg ) != 0 ) 
			&& ( ( cChar = *pcStrg ) != '"' ) 
			&& ( ( cChar = *pcStrg ) != '\r' ) 
			&& ( ( cChar = *pcStrg ) != '\n' ) 
			&& ( xLen-- > 0 ) )
	{
		usValue = 10 * usValue + cCharToNibble( cChar );
		pcStrg++;
	}
	
	return usValue;
}
/*-----------------------------------------------------------*/

/* Convert a a string of unsigned long integer characters including end-of-string 0x0 into a 32-bit value. 
   The maximum size of the string is 10 characters (xLen). 
   '"', \r or \n terminates the conversion.
*/
unsigned long ulIntStrgToLong( signed char *pcStrg, portBASE_TYPE xLen )
{
	signed char		cChar;
	unsigned long 	ulValue;
	
	/* Maximum length of an unsigned short expressed as HEX string. */
	ulValue = 0;
	while (    ( ( cChar = *pcStrg ) != 0 ) 
			&& ( ( cChar = *pcStrg ) != '"' ) 
			&& ( ( cChar = *pcStrg ) != '\r' ) 
			&& ( ( cChar = *pcStrg ) != '\n' ) 
		    && ( xLen-- > 0 ) )
	{
		ulValue = 10 * ulValue + cCharToNibble( cChar );
		pcStrg++;
	}
	
	return ulValue;
}
/*-----------------------------------------------------------*/

/* Convert a string of HEX-characters into an 32-bit value. The characters are
   located inside the UART Rx buffer. 
   The value is stored in little endian (LSB first).
*/
unsigned long ulHexComStrgToLongLE( unsigned portBASE_TYPE xComID, unsigned portBASE_TYPE uxStrgIdx )
{
	unsigned long 			ulValue;
	unsigned portBASE_TYPE	uxIdx;
	
	ulValue = 0l;
	for ( uxIdx = 0; uxIdx < 8; uxIdx += 2 )
	{
		ulValue += ( unsigned long )cCharToNibble( cGetRxCharFromBufferWithIndex( xComID, uxStrgIdx++ ) ) << ( 4 * ( uxIdx + 1 ) );
		ulValue += ( unsigned long )cCharToNibble( cGetRxCharFromBufferWithIndex( xComID, uxStrgIdx++ ) ) << ( 4 * ( uxIdx     ) );
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
	unsigned short 				usValue;
	unsigned portBASE_TYPE		uxIdx;
	signed char					cDigit;
	
	uxIdx = 0;
	usValue = 0;
	cDigit = *( pcStrg++ );
	
	while ( ( uxIdx++ < 4 ) && ( cDigit != ' ' ) && ( cDigit != 0 ) )
	{
		usValue <<= 4;
		usValue += cCharToNibble( cDigit );
		cDigit = *( pcStrg++ );
	}
	
	return usValue;
}
/*-----------------------------------------------------------*/

/* Convert a string of HEX-characters into a 32-bit value. */
unsigned long ulHexStrgToLong( signed char *pcStrg )
{
	unsigned long 				ulValue;
	unsigned portBASE_TYPE		uxIdx;
	signed char					cDigit;
	
	uxIdx = 0;
	ulValue = 0;
	cDigit = *( pcStrg++ );
	
	while ( ( uxIdx++ < 8 ) && ( cDigit != ' ' ) && ( cDigit != 0 ) )
	{
		ulValue <<= 4;
		ulValue += cCharToNibble( cDigit );
		cDigit = *( pcStrg++ );
	}
	
	return ulValue;
}
/*-----------------------------------------------------------*/

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

/* Return pointer to the character after the Nth comma ','  in a string. 
   uxMaxChar is the maximum number of characters in the field excluding the comma.
   If no comma is found, the position pcChar+uxMaxChar is returned (i.e. one position behind
   the last allowed character).
   If an end-of-string is encountered before, the position after the '0' of the string is returned.
   pcChar points to the first field character.
*/
signed char *pcFindNComma( signed char *pcChar, unsigned portBASE_TYPE uxN, unsigned portBASE_TYPE uxMaxChar )
{
	signed char cChar;
	
	uxMaxChar++;
	
	do
	{
		cChar = *( pcChar++ );
		uxMaxChar--;
		if ( cChar == ',' )
		{
			uxN--;
		}
	} 
	while ( ( cChar != 0 ) && ( uxMaxChar > 0 ) && ( uxN != 0 ) );
	
	return pcChar;
}
/*-----------------------------------------------------------*/

/* Compare two 16-bit time stamps and determine which is later. It is assumed
   that the largest difference between the time stamps is 2^15 (0x8000).
   
   https://freertos.org/FreeRTOS_Support_Forum_Archive/February_2012/freertos_Tick_count_overflow_5005076.html
*/
bool bIsANewerThanB( uint16_t a, uint16_t b ) 
{
	uint16_t 		u;

	u = ( b - a );
	
	return ( u & 0x8000u );
}
/*-----------------------------------------------------------*/

/* Calculate the logarithm to the base of 2 (log2() or ld()) using fixed-point arithmetic.

We do so by multiplying the original input by a sequence of factors of decreasing magnitude that
approach 1. Considering each of the factor in sequence, we multiply the input only by those factors
that result in a product closer to 1, but without exceeding it. While doing so, we sum the log2() of
the factors that "fit". At the end of this procedure we wind up with a number very close to 1 as our
final product, and a sum that represents the binary logarithm.

This process is known in the literature as multiplicative normalization or pseudo division, and some
early publications describing it are the works by De Lugish and Meggitt. The latter indicates that
the origin is basically Henry Briggs's method for computing common logarithms.

	B. de Lugish. "A Class of Algorithms for Automatic Evaluation of Functions and Computations in a
Digital Computer". PhD thesis, Dept. of Computer Science, University of Illinois, Urbana, 1970.

	J. E. Meggitt. "Pseudo division and pseudo multiplication processes". IBM Journal of Research
and Development, Vol. 6, No. 2, April 1962, pp. 210-226

As the chosen set of factors comprises 2^i and (1+2^(-i)) the necessary multiplications can be
performed without the need for a multiplication instruction: the products can be computed by either
shift or shift plus add.

Since the input x is a purely fractional numbers with 16 bits, we chose a 5.16 fixed-point result
for the logarithm.

Note that the fixed-point computation of the logarithm results in large relative error for inputs
near 1.

Source:
https://stackoverflow.com/questions/32159300/compute-logarithmic-expression-without-floating-point-arithmetics-or-log
*/
unsigned long nlog2_16 ( unsigned short usX )
{
    unsigned long ulR;
    unsigned long ulT;
	unsigned long ulA;
	
	ulR = 0;
	ulA = usX;

    /* Try factors 2**i with i = 8, 4, 2, 1 */
    if ( ( ulT = ulA << 8            ) < 0x10000 ) { ulA = ulT; ulR += 0x80000; }
    if ( ( ulT = ulA << 4            ) < 0x10000 ) { ulA = ulT; ulR += 0x40000; }
    if ( ( ulT = ulA << 2            ) < 0x10000 ) { ulA = ulT; ulR += 0x20000; }
    if ( ( ulT = ulA << 1            ) < 0x10000 ) { ulA = ulT; ulR += 0x10000; }
    /* Try factors (1+2**(-i)) with i = 1, ..., 16 */
    if ( ( ulT = ulA + ( ulA >>  1 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x095c0; }
    if ( ( ulT = ulA + ( ulA >>  2 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x0526a; }
    if ( ( ulT = ulA + ( ulA >>  3 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x02b80; }
    if ( ( ulT = ulA + ( ulA >>  4 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x01664; }
    if ( ( ulT = ulA + ( ulA >>  5 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x00b5d; }
    if ( ( ulT = ulA + ( ulA >>  6 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x005ba; }
    if ( ( ulT = ulA + ( ulA >>  7 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x002e0; }
    if ( ( ulT = ulA + ( ulA >>  8 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x00171; }
    if ( ( ulT = ulA + ( ulA >>  9 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x000b8; }
    if ( ( ulT = ulA + ( ulA >> 10 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x0005c; }
    if ( ( ulT = ulA + ( ulA >> 11 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x0002e; }
    if ( ( ulT = ulA + ( ulA >> 12 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x00017; }
    if ( ( ulT = ulA + ( ulA >> 13 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x0000c; }
    if ( ( ulT = ulA + ( ulA >> 14 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x00006; }
    if ( ( ulT = ulA + ( ulA >> 15 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x00003; }
    if ( ( ulT = ulA + ( ulA >> 16 ) ) < 0x10000 ) { ulA = ulT; ulR += 0x00001; }

    return ulR;
}
/*-----------------------------------------------------------*/

/* Remove all messages of a specific type from a queue. All other messages remain in the 
   queue in the same order as before.
*/
void vRemoveTypeFromQueue( QueueHandle_t xQueue, unsigned portBASE_TYPE uxMessageType )
{
	unsigned portBASE_TYPE	uxQueueIdx;
	unsigned portBASE_TYPE	uxMessage = 0;
	unsigned portBASE_TYPE	xMessagesWaiting;
	
	portENTER_CRITICAL();
	
	/* Walk through the entire queue. */
	xMessagesWaiting = uxQueueMessagesWaiting( xQueue );
	for ( uxQueueIdx = 0; uxQueueIdx < xMessagesWaiting; uxQueueIdx++ )
	{
		/* Get the next queue entry. */
		xQueueReceive( xQueue, &uxMessage, 0 );
		
		/* If it is not of the message type we want to remove, store it back to the queue. 
		   Else, do nothing so that the entry disappears. */
		if ( uxMessage != uxMessageType )
		{
			/* Send it back to the queue. */
			ASSERT( xQueueSend( xQueue, &uxMessage, 0 ) );
		}
	}
	portEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

/* Remove all messages which are not server commands from a queue. Server command messages remain in the 
   queue in the same order as before.
*/
void vRemoveAllMsgsFromQueue( QueueHandle_t xQueue )
{
	unsigned portBASE_TYPE	uxQueueIdx;
	unsigned portBASE_TYPE	xMessage = 0;
	unsigned portBASE_TYPE	xMessagesWaiting;
	
	portENTER_CRITICAL();
	
	/* Walk through the entire queue. */
	xMessagesWaiting = uxQueueMessagesWaiting( xQueue );
	for ( uxQueueIdx = 0; uxQueueIdx < xMessagesWaiting; uxQueueIdx++ )
	{
		/* Get the next queue entry. */
		xQueueReceive( xQueue, &xMessage, 0 );
		
		/* Stuff messages concerning server commands or RX data indications back into the AT response queue. */
		if (   ( xQueue == xParserAtRespQueue )
		    && (   ( ( xMessage >= SRVCMD_RD_VERSION ) && ( xMessage <= SRVCMD_OTHER ) ) 
			    || ( xMessage == AT_TCP_RX_DATA_IND )
		       )
		   )
		{
			/* Send it back to the queue. */
			ASSERT( xQueueSend( xQueue, &xMessage, 0 ) );
		}
	}
	portEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

/* Check, if a message of a specific type is in a queue. 
   ATTENTION: Only works for queues with an item size of 1! 
*/
bool bMsgTypeInQueue( QueueHandle_t xQueue, void *pxMessageType )
{
	portBASE_TYPE			uxQueueIdx;
	unsigned char			xMessage = 0;
	bool					msgInQueue;
	unsigned portBASE_TYPE	xMessagesWaiting;
	
	portENTER_CRITICAL();
	
	msgInQueue = false;
	
	/* Walk through the entire queue. */
	xMessagesWaiting = uxQueueMessagesWaiting( xQueue );
	for ( uxQueueIdx = 0; uxQueueIdx < xMessagesWaiting; uxQueueIdx++ )
	{
		/* Get the next queue entry. */
		xQueueReceive( xQueue, &xMessage, 0 );
		
		/* Set a flag if it is of the message type we want to find. 
		   Note that we assume here that the message is only 8-bit long! */
		if ( xMessage == *( unsigned char * )pxMessageType )
		{
			msgInQueue = true;
		}
		/* Send it back to the queue. */
		ASSERT( xQueueSend( xQueue, &xMessage, 0 ) );		
	}
	portEXIT_CRITICAL();
	
	return msgInQueue;
}
/*-----------------------------------------------------------*/

/* Send a message to the specified queue if it is not already in there. */
BaseType_t xSendUniqueMsgToQueue( QueueHandle_t xQueue, const void * const pxMessage, TickType_t xTicksToWait )
{

	if ( !bMsgTypeInQueue( xQueue, ( void * )pxMessage ) )
	{
		return xQueueSend( xQueue, pxMessage, xTicksToWait );
	}
	
	return pdTRUE;
}
/*-----------------------------------------------------------*/

/* Safe variant of FreeRTOS' task delay. Uses the nRF SDK delay function if the scheduler is not yet running. */
void vTaskDelaySafe( const TickType_t xTicksToDelay )
{
	if ( xTaskGetSchedulerState() == taskSCHEDULER_RUNNING ) 
	{
		vTaskDelay( xTicksToDelay );
	}
	else
	{
		nrf_delay_ms( xTicksToDelay );
	}
}
/*-----------------------------------------------------------*/

/* Search the bootloader flash for the bootloader version string. 
   Returns true if the bootloader version string was found. 
   *ppcBootloaderVerstionStrg then contains a pointer to the string.
   
   Note that the bootloader flash region runs from UICR.NRFFW[0] to UICR.NRFFW[1]. 
*/
bool bLocateBootloaderVersion( signed char **ppcBootloaderVerstionStrg )
{
	signed char 	*pcBlFlashEnd  ;
	signed char		cMagicString[]	= BL_VERSION_MAGIC;
	uint32_t		uiStrgIdx;	
	bool			bFound;
	
	/* Retrieve pointers to start and end of the bootloader flash memory. */
	*ppcBootloaderVerstionStrg = ( signed char * )NRF_UICR->NRFFW[ 0 ];	
	pcBlFlashEnd  			   = ( signed char * )NRF_UICR->NRFFW[ 1 ];	
	
	while ( ( *ppcBootloaderVerstionStrg < pcBlFlashEnd ) && !bFound )
	{
		for ( uiStrgIdx = 0; ( cMagicString[ uiStrgIdx ] != '\0' ) && ( *ppcBootloaderVerstionStrg + uiStrgIdx < pcBlFlashEnd ) && !bFound; )
		{
			if( *( *ppcBootloaderVerstionStrg + uiStrgIdx ) == cMagicString[ uiStrgIdx ] )
			{
				/* Matched the first character of magic string, so advance. */
				uiStrgIdx++; 
				bFound = ( uiStrgIdx >= strlen( cMagicString ) );
			}
			else
			{   
				/* No match yet, reset the magic string index and move along the flash memory. */
				(*ppcBootloaderVerstionStrg)++;
				uiStrgIdx = 0;
			}
		}
		(*ppcBootloaderVerstionStrg)++;
	}	

	*ppcBootloaderVerstionStrg += strlen( cMagicString ) - 1;
	
	return bFound;
}
/*-----------------------------------------------------------*/


/* Dump data in hex format to the logger. The parameter uiSize is the number of 16-byte chunks to dump.
   pcMemory points to the memory where the data is located.
   Note that the dump is deliberately very slow to avoid that the system crashes. Sometimes it does never-
   theless, particularly with large data sizes. Sizes <=6 appear to be fine. 
*/
void vDebugHexDump( char *pcMemory, uint32_t	uiSize )
{
	char		cHexStrg[ 50 ];
	uint32_t	uiDataIdx;

	for ( uiDataIdx = 0; uiDataIdx < uiSize * 16; uiDataIdx++ )
	{
		vByteToHexStrg( cHexStrg + 3*( uiDataIdx % 16 ), *( pcMemory + uiDataIdx ) );
		cHexStrg[ 3*( uiDataIdx % 16 ) + 2 ] = ' ';

		if ( ( uiDataIdx % 16 ) == 15 ) 
		{
			cHexStrg[ 3*( uiDataIdx % 16 ) + 2 ] = 0;
			NRF_LOG_DEBUG( "%s", cHexStrg );
			NRF_LOG_FLUSH();
			vTaskDelay( 50 );
		}
	}
}
/*-----------------------------------------------------------*/

/* Hard-reset the system. */
void vSystemReset( uint32_t uiErrorCode, uint32_t uiLineNum, const char * pcFileName )
{
	uint32_t		uiIdx;
	uint32_t		*puiSP;
	
	/* Disable IRQs so that the access to the Flash memory can work without being disturbed by the 
	   SoftDevice (which hardfaults if it enters an interrupt routine). */
    __disable_irq();

	/* Save relevant information to the system's persistent RAM region. */
	ulErrorId = SYSTEM_RESET_ID;
	ulErrorPc = __get_LR();
	xErrorInfo.line_num = uiLineNum;
	xErrorInfo.p_file_name = pcFileName;
	xErrorInfo.err_code = uiErrorCode;
	
	/* Save the first entries of the current stack. 
	   The first 4 entries are not */
	if ( ( __get_xPSR() && xPSR_ISR_Msk ) == 0 )
	{
		/* In thread context. */
		puiSP = ( uint32_t * )__get_PSP();
	}
	else
	{
		/* In ISR context. */
		puiSP = ( uint32_t * )__get_MSP();
	}
	for ( uiIdx = 0; uiIdx < ERROR_STACK_DUMP_LEN; uiIdx++ )
	{
		uiErrorStack[ uiIdx ] = *( ( uint32_t * )__get_PSP() + uiIdx + 4 );
	}
	
    NRF_LOG_WARNING( "%i System reset", ulReadRTC() );

    NVIC_SystemReset();
}
/*-----------------------------------------------------------*/
