/* TRACE UTILITIES ON TARGET */

/* Standard include files. */
#include <stdlib.h>
#include <string.h>

#include "tracker.h"

/* nRF SDK files. */
#define NRF_LOG_MODULE_NAME 		TRC
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

#if defined( SOFTDEVICE_PRESENT ) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#endif
#include "nordic_common.h"
#include "app_error.h"
#include "sdk_errors.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* Device specific include files. */
#include "drv_uart.h"
#include "drv_nvm.h"

#include "config.h"
#include "ctrl.h"
#include "gsm.h"
#include "rtc.h"
#include "trace.h"
#include "tracemsg.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/*
	Tracing is done to the trace UART and - optionally - into a log file on the SARA/QTEL file system via a temporary buffer
	in the NVDS. The buffer allows for writing trace messages while the SARA/QTEL is unable to accept any writes to the
	file system.
	
	Trace messages are identified by an 8-bit number. The trace logger on the PC has to translate them into readable 
	strings.
	
	Is configured, the trace messages are also written to a temporary buffer in NVDS. Every once in a while, the
	buffer is written out to the SARA/QTEL log file (trace.log).
	
	The format here is:
			tttt			time stamp
			' '
			ii				message identifier
			[ ' '
			  arguments]	any optional arguments: byte, short or string.
			'\r'
*/

/* Function prototypes. */
/* Initialise the COM port for trace. */
void vTraceInit( void );

/* Check if the trace handler is fully initialised. */
bool bCheckTraceInitialised( void );
	
/* Check if the trace COM port is closed. If so, open it. */
void vCheckAndOpenTraceComPort( void );

/* Store the common trace message header to UART. */
signed char *pcTraceWriteUARTHeader( signed char *pcComStrg, unsigned portBASE_TYPE uxTraceMsgId );
	
/* Return the number of characters in the trace buffer. */
unsigned short usGetTraceBufferNumOfChars( void );

/* Store a single character in the circular trace buffer. 
   If the buffer is full, move the read pointer one position ahead. */
void vStoreCharInTraceBuffer( signed char cTrcChar );

/* Store a string in the circular trace buffer. 
   If the buffer is full, move the read pointer ahead. 
   Do not copy the trailing end-of-string marker 0x00. */
void vStoreStrgInTraceBuffer( signed char *pcTrcStrg, unsigned portBASE_TYPE uxMaxLen );

/* Geta single character from the circular trace buffer. 
   If the buffer is return false. */
bool bGetCharFromTraceBuffer( signed char *pcTrcChar );

/* Store a long value in the trace buffer.
   If bLeadingSpace is set, also store a leading space. 
*/
void vStoreLongInTraceBuffer( unsigned long ulVal, bool bLeadingSpace );

/* Store the common trace message header in NVDS. */
void vTraceStoreHeader( unsigned portBASE_TYPE uxTraceMsgId );

/* Basic trace print to UART and optionally to the cellular module trace file. 
   This print takes no arguments.
*/
void vTracePrint( unsigned portBASE_TYPE uxTraceMsgId, unsigned portBASE_TYPE uxFileLog );

/* Trace message with string as argument. */
void vTracePrintStrg(  unsigned portBASE_TYPE uxTraceMsgId, signed char *pcStrg, unsigned portBASE_TYPE uxFileLog );

/* Trace message with byte as argument. */
void vTracePrintByte(  unsigned portBASE_TYPE uxTraceMsgId, unsigned char ucArg, unsigned portBASE_TYPE uxFileLog );

/* Trace message with a 3-tuple as argument. */
void vTracePrint3Tuple(  unsigned portBASE_TYPE uxTraceMsgId, short sArg0, short sArg1, short sArg2, unsigned portBASE_TYPE uxFileLog );

/* Print raw string. */
void vTracePrintSp( const signed char *pcStrg1 );

/* Write fault information to trace buffer. */
void vTraceFault( uint32_t uiId, uint32_t uiPc, struct xERROR_INFO *pxErrorInfo, uint32_t *puiErrorStack, unsigned portBASE_TYPE uxFileLog );

/* Trace message with short as argument. */
void vTracePrintShort(  unsigned portBASE_TYPE uxTraceMsgId, short sArg, unsigned portBASE_TYPE uxFileLog );

/* Add the file and the number of characters to store to the file write command. Include the time stamp in the count. */
void prvAddTrcCount( void );

/* Add the file handle. */
void prvAddHandle( void );

/* Write NVDS trace contents to the trace file on cellular module's file system. */
void vTraceFlushLog( void );

/* Collect raw statistics about the system. */
void vSystemStateStats( enum xACTION xAction, enum xCTRL_STATE xNextCtrlState );

/* Collect raw statistics about the NoAbn system mode. */
void vSystemNoAbnStats( enum xNOABNACTION xAction );

/* Increment reset counters. */
void vIncrementResetCount( void );

/* Function called in case of stack overflows. */
void vApplicationStackOverflowHook( void * xTask, char *pcTaskName );

/* Function called in case of heap allocation failure. */
void vApplicationMallocFailedHook( void );
/*-----------------------------------------------------------*/

/* Statistics. */
union uSTATS				uStats;
unsigned short 				usStateEnterTimeStamp;
unsigned short				usNoAbnEnterTimeStamp;

/* Persistent memory section containing variables pertaining to the Trace buffer. */
struct xSYSTEM_ERROR_RECORD	__attribute__( ( section( ".non_init" ) ) ) uxErrorRecord;

/* Flag indicating a recovery from a watchdog reset. */
bool						bWatchdogRecovery;

/* Flag indicating a recovery from a software reset. */
bool						bSoftResetRecovery;

/* UART Rx character ring buffer */
signed char					cTraceUartRxBuffer[ TRC_UART_RX_BUFFER_SIZE ];

/* Mutex handle to protect configuration record access. */
SemaphoreHandle_t			xMutexTrace;

/* AT command string to append string to logfile. */
static const char 			pcAt_OpenTrcFile[]		= "+QFOPEN=\"TRACE.LOG\",3";			/* Open the trace file on the filesystem for appending (3). */		
static const char 			pcAt_QFWrTrcFile[]	 	= "+QFWRITE=";
static const char 		  	pcAt_CloseFile[]		= "+QFCLOSE=";							/* Close file. */	

/* Open the trace file for appending and write to it. */
static const struct xAT_CMD xAtQWrTrcFile[]			=
{
	{ pcAt_OpenTrcFile,		vReqStorage,			{ AT_OK, AT_NOMSG },			AT_NOMSG,		1,		TO_FILE_WRITE		},
	{ pcAt_QFWrTrcFile,		prvAddTrcCount,			{ AT_CONNECT, AT_NOMSG },		AT_NOMSG,		1,		TO_FILE_WRITE		},
};

/* Close to trace file. */
static const struct xAT_CMD xAtQFClTrcFile			=
	{ pcAt_CloseFile,		prvAddHandle,			{ AT_OK, AT_NOMSG },			AT_NOMSG,		3,		TO_FILE_WRITE		};
	
/* Trace file handle. */
unsigned portBASE_TYPE		uxTrcFileHandle;

/* Flag indicating that the configuration handler is initialised. No access to the configuration may be done before this is the case. */
bool						bTraceInitialised = false;

/* Copy of the NRF_POWER->RESETREAS register for later storage into the trace record. */
unsigned long				ulResetReasonCopy;
/*-----------------------------------------------------------*/

/* Trace functions for HW environment.
   
   Here, any trace strings are sent to the UART allocated for this purpose
   using COM1.
*/
/* Initialise the COM port for trace and retrieve previous, unprocessed trace records. */
void vTraceInit()
{
	#ifdef OS_TRACE
		/* Open the TRC COM port. */	
		vCOMOpen( COM_TRC, cTraceUartRxBuffer, TRC_UART_RX_BUFFER_SIZE - 1, COM_ASCII );			
		
		/* Create a mutex to protect access to the NVDS so that it	can be used from different tasks - notably GSM and CTRL. */
		xMutexTrace = xSemaphoreCreateMutex();
		
		/* Initialise the index pointers to the trace buffer, if needed. */
		if (    ( uxErrorRecord.usTraceRdIdx > LEN_TRACE_BUFFER - 1)
		     || ( uxErrorRecord.usTraceWrIdx > LEN_TRACE_BUFFER - 1)
		   )
		{
			vEraseTraceBuffer();
		}
		
		NRF_LOG_INFO( "Module initialised." );
		NRF_LOG_FLUSH();
	#endif
		
	bTraceInitialised = true;
}
/*-----------------------------------------------------------*/
	
/* Check if the trace handler is fully initialised. */
bool bCheckTraceInitialised( void )
{
	return bTraceInitialised;
}
/*-----------------------------------------------------------*/
	
/* Erase the trace buffer after a FW update. */
void vEraseTraceBuffer( void )
{
	uxErrorRecord.usTraceRdIdx = 0;
	uxErrorRecord.usTraceWrIdx = 0;
	memset( uxErrorRecord.cTraceBuffer, 0, LEN_TRACE_BUFFER );
}
/*-----------------------------------------------------------*/
	
/* Check if the trace COM port is closed. If so, open it. */
void vCheckAndOpenTraceComPort( void )
{
	if ( !bCOMIsOpen( COM_TRC ) )
	{
		vCOMOpen( COM_TRC, cTraceUartRxBuffer, TRC_UART_RX_BUFFER_SIZE - 1, COM_ASCII );
	}
}
/*-----------------------------------------------------------*/

/* Store the common trace message header to UART. */
signed char *pcTraceWriteUARTHeader( signed char *pcComStrg, unsigned portBASE_TYPE uxTraceMsgId )
{
	strcpy( pcComStrg, "TRC: " );
	*( pcComStrg + 5 ) = cNibbleToChar( ( uxTraceMsgId >> 4 ) & 0xf );
	*( pcComStrg + 6 ) = cNibbleToChar( ( uxTraceMsgId      ) & 0xf );
	*( pcComStrg + 7 ) = ' ';
	
	return pcComStrg + 8;
}
/*-----------------------------------------------------------*/

/* Calculate the number of free positions in the trace buffer. */
unsigned short usGetTraceBufferNumOfChars( void )
{
	/* Check if the trace buffer wrapped around. */
	if ( uxErrorRecord.usTraceWrIdx >= uxErrorRecord.usTraceRdIdx )
	{
		return uxErrorRecord.usTraceWrIdx - uxErrorRecord.usTraceRdIdx;
	}
	else
	{
		return uxErrorRecord.usTraceWrIdx + LEN_TRACE_BUFFER - uxErrorRecord.usTraceRdIdx;
	}
}
/*-----------------------------------------------------------*/

/* Store a single character in the circular trace buffer. 
   If the buffer is full, move the read pointer one position ahead. */
void vStoreCharInTraceBuffer( signed char cTrcChar )
{
	uxErrorRecord.cTraceBuffer[ uxErrorRecord.usTraceWrIdx ] = cTrcChar;
	uxErrorRecord.usTraceWrIdx = ( uxErrorRecord.usTraceWrIdx + 1 ) % LEN_TRACE_BUFFER;

	/* Check for buffer overflow. */
	if ( uxErrorRecord.usTraceWrIdx == uxErrorRecord.usTraceRdIdx )
	{
		/* Move the read pointer one position ahead. */
		uxErrorRecord.usTraceRdIdx = ( uxErrorRecord.usTraceRdIdx + 1 ) % LEN_TRACE_BUFFER;
	}
}
/*-----------------------------------------------------------*/

/* Store a string in the circular trace buffer. 
   If the buffer is full, move the read pointer ahead. 
   Do not copy the trailing end-of-string marker 0x00. */
void vStoreStrgInTraceBuffer( signed char *pcTrcStrg, unsigned portBASE_TYPE uxMaxLen )
{
	unsigned portBASE_TYPE	uxIdx;

	if ( strlen( pcTrcStrg ) < uxMaxLen )
	{
		uxMaxLen = strlen( pcTrcStrg );
	}

	for ( uxIdx = 0; uxIdx < uxMaxLen; uxIdx++ )
	{
		vStoreCharInTraceBuffer( *( pcTrcStrg++ ) );
	}
}
/*-----------------------------------------------------------*/

/* Geta single character from the circular trace buffer. 
   If the buffer is return false. */
bool bGetCharFromTraceBuffer( signed char *pcTrcChar )
{
	/* Check for buffer not empty. */
	if ( uxErrorRecord.usTraceWrIdx != uxErrorRecord.usTraceRdIdx )
	{
		*pcTrcChar = uxErrorRecord.cTraceBuffer[ uxErrorRecord.usTraceRdIdx ];
		uxErrorRecord.usTraceRdIdx = ( uxErrorRecord.usTraceRdIdx + 1 ) % LEN_TRACE_BUFFER;

		return true;		
	}
	else
	{
		return false;
	}
}
/*-----------------------------------------------------------*/

/* Store a long value in the trace buffer.
   If bLeadingSpace is set, also store a leading space. 
*/
void vStoreLongInTraceBuffer( unsigned long ulVal, bool bLeadingSpace )
{
	signed char		sTmpStrg[ 9 ];

	if ( bLeadingSpace )
	{
		vStoreCharInTraceBuffer( ' ' );
	}
	vLongToHexStrg( sTmpStrg, ulVal );
	vStoreStrgInTraceBuffer( sTmpStrg, 8 );
}
/*-----------------------------------------------------------*/

/* Store the common trace message header in NVDS. */
void vTraceStoreHeader( unsigned portBASE_TYPE uxTraceMsgId )
{
	signed char		sTmpStrg[ 5 ];

	/* Start writing at either the start position of the NVDS buffer or at the last end of string character (0x00). */
	vShortToHexStrg( sTmpStrg, usReadRTC() );
	vStoreStrgInTraceBuffer( sTmpStrg, 4 );
	vStoreCharInTraceBuffer( ' ' );
	vStoreCharInTraceBuffer( cNibbleToChar( ( uxTraceMsgId >> 4 ) & 0xf ) );
	vStoreCharInTraceBuffer( cNibbleToChar( ( uxTraceMsgId      ) & 0xf ) );
	vStoreCharInTraceBuffer( ' ' );
}
/*-----------------------------------------------------------*/
	
/* Basic trace print to UART and optionally to the cellular module's trace file. 
   This print takes no arguments.
*/
void vTracePrint( unsigned portBASE_TYPE uxTraceMsgId, unsigned portBASE_TYPE uxFileLog )
{
	if ( xMutexTrace != NULL )
	{
		configASSERT( xSemaphoreTake( xMutexTrace, TO_MUTEX_TRACE ) );
	}
	
	if ( uxFileLog & TRACE_UART )
	{
		signed char 		cComStrg[ 11 ];
		signed char 		*pcComStrg;

		pcComStrg = pcTraceWriteUARTHeader( cComStrg, uxTraceMsgId );
		*( pcComStrg++ ) = '\r';
		*( pcComStrg++ ) = '\n';
		*( pcComStrg++ ) = 0;
		vCheckAndOpenTraceComPort();
		xComSendString( COM_TRC, cComStrg );
		#if !defined ( TL500 ) && !defined ( TL510 )  && !defined ( TL501 ) 
			/* Only for TL502 and newer. */
			vCOMClose( COM_TRC );
		#endif
	}

	#if defined ( FILE_LOG )
		/* Log the message to the NVDS trace buffer, if required. */
		if ( xNvdsConfig.usLogEnable && ( uxFileLog & TRACE_FILE ) )
		{
			vTraceStoreHeader( uxTraceMsgId );
			vStoreCharInTraceBuffer( '\r' );			
		}		
	#endif

	if ( xMutexTrace != NULL )
	{
		xSemaphoreGive( xMutexTrace );
	}
}
/*-----------------------------------------------------------*/

/* Trace message with string in RAM as argument. */
void vTracePrintStrg(  unsigned portBASE_TYPE uxTraceMsgId, signed char *pcStrg, unsigned portBASE_TYPE uxFileLog )
{
	unsigned portBASE_TYPE			uxStrgLen;
	
	if ( xMutexTrace != NULL )
	{
		configASSERT( xSemaphoreTake( xMutexTrace, TO_MUTEX_TRACE ) );
	}

	if ( uxFileLog & TRACE_UART )
	{
		signed char 		cComStrg[ 9 ];
		signed char 		*pcComStrg;

		pcComStrg = pcTraceWriteUARTHeader( cComStrg, uxTraceMsgId );
		*( pcComStrg++ ) = 0;
		vCheckAndOpenTraceComPort();
		xComSendString( COM_TRC, cComStrg );
		xComSendString( COM_TRC, ( signed char * )pcStrg );
		xComSendChar( COM_TRC, '\r' );
		xComSendChar( COM_TRC, '\n' );
		#if !defined ( TL500 ) && !defined ( TL510 )  && !defined ( TL501 ) 
			/* Only for TL502 and newer. */
			vCOMClose( COM_TRC );
		#endif
	}
	
	#if defined ( FILE_LOG )
		/* Log the message to the EEPROM trace buffer, if required. */
		if ( usConfigReadShort( &xNvdsConfig.usLogEnable ) && ( uxFileLog & TRACE_FILE ) )
		{
			vTraceStoreHeader( uxTraceMsgId );
			vStoreStrgInTraceBuffer( pcStrg, 130 );
			vStoreCharInTraceBuffer( '\r' );			
		}		
	#endif

	if ( xMutexTrace != NULL )
	{
		xSemaphoreGive( xMutexTrace );
	}
}
/*-----------------------------------------------------------*/

/* Trace message with byte as argument. */
void vTracePrintByte(  unsigned portBASE_TYPE uxTraceMsgId, unsigned char ucArg, unsigned portBASE_TYPE uxFileLog )
{
	configASSERT( xSemaphoreTake( xMutexTrace, TO_MUTEX_TRACE ) );

	if ( uxFileLog & TRACE_UART )
	{
		signed char 		cComStrg[ 13 ];
		signed char 		*pcComStrg;

		pcComStrg = pcTraceWriteUARTHeader( cComStrg, uxTraceMsgId );
		*( pcComStrg++ ) = cNibbleToChar( ( ucArg >>  4 ) & 0xf );
		*( pcComStrg++ ) = cNibbleToChar(   ucArg         & 0xf );
		*( pcComStrg++ ) = '\r';
		*( pcComStrg++ ) = '\n';
		*( pcComStrg++ ) = 0;
		vCheckAndOpenTraceComPort();
		xComSendString( COM_TRC, cComStrg );
		#if !defined ( TL500 ) && !defined ( TL510 )  && !defined ( TL501 ) 
			/* Only for TL502 and newer. */
			vCOMClose( COM_TRC );
		#endif
	}
	
	#if defined ( FILE_LOG )
		/* Log the message to the NVDS trace buffer, if required. */
		if ( xNvdsConfig.usLogEnable && ( uxFileLog & TRACE_FILE ) )
		{
			vTraceStoreHeader( uxTraceMsgId );
			vStoreCharInTraceBuffer( cNibbleToChar( ( ucArg >>  4 ) & 0xf ) );
			vStoreCharInTraceBuffer( cNibbleToChar(   ucArg         & 0xf ) );
			vStoreCharInTraceBuffer( '\r' );			
		}		
	#endif

	xSemaphoreGive( xMutexTrace );
}
/*-----------------------------------------------------------*/

/* Trace message with short as argument. */
void vTracePrintShort(  unsigned portBASE_TYPE uxTraceMsgId, short sArg, unsigned portBASE_TYPE uxFileLog )
{
	configASSERT( xSemaphoreTake( xMutexTrace, TO_MUTEX_TRACE ) );

	if ( uxFileLog & TRACE_UART )
	{
		signed char 		cComStrg[ 15 ];
		signed char 		*pcComStrg;

		pcComStrg = pcTraceWriteUARTHeader( cComStrg, uxTraceMsgId );
		*( pcComStrg++ ) = cNibbleToChar( ( sArg >> 12 ) & 0xf );
		*( pcComStrg++ ) = cNibbleToChar( ( sArg >>  8 ) & 0xf );
		*( pcComStrg++ ) = cNibbleToChar( ( sArg >>  4 ) & 0xf );
		*( pcComStrg++ ) = cNibbleToChar(   sArg         & 0xf );
		*( pcComStrg++ ) = '\r';
		*( pcComStrg++ ) = '\n';
		*( pcComStrg++ ) = 0;
		vCheckAndOpenTraceComPort();
		xComSendString( COM_TRC, cComStrg );
		#if !defined ( TL500 ) && !defined ( TL510 )  && !defined ( TL501 ) 
			/* Only for TL502 and newer. */
			vCOMClose( COM_TRC );
		#endif
	}
	
	#if defined ( FILE_LOG )
		/* Log the message to the NVDS trace buffer, if required. */
		if ( xNvdsConfig.usLogEnable && ( uxFileLog & TRACE_FILE ) )
		{
			vTraceStoreHeader( uxTraceMsgId );
			vStoreCharInTraceBuffer( cNibbleToChar( ( sArg >> 12 ) & 0xf ) );
			vStoreCharInTraceBuffer( cNibbleToChar( ( sArg >>  8 ) & 0xf ) );
			vStoreCharInTraceBuffer( cNibbleToChar( ( sArg >>  4 ) & 0xf ) );
			vStoreCharInTraceBuffer( cNibbleToChar(   sArg         & 0xf ) );
			vStoreCharInTraceBuffer( '\r' );	
		}		
	#endif

	xSemaphoreGive( xMutexTrace );
}
/*-----------------------------------------------------------*/

/* Trace message with long as argument. */
void vTracePrintLong(  unsigned portBASE_TYPE uxTraceMsgId, long lArg, unsigned portBASE_TYPE uxFileLog )
{
	configASSERT( xSemaphoreTake( xMutexTrace, TO_MUTEX_TRACE ) );

	if ( uxFileLog & TRACE_UART )
	{
		signed char 		cComStrg[ 19 ];
		signed char 		*pcComStrg;

		pcComStrg = pcTraceWriteUARTHeader( cComStrg, uxTraceMsgId );
		vLongToHexStrg( pcComStrg, lArg );
		pcComStrg += 8;
		*( pcComStrg++ ) = '\r';
		*( pcComStrg++ ) = '\n';
		*( pcComStrg++ ) = 0;
		vCheckAndOpenTraceComPort();
		xComSendString( COM_TRC, cComStrg );
		#if !defined ( TL500 ) && !defined ( TL510 )  && !defined ( TL501 ) 
			/* Only for TL502 and newer. */
			vCOMClose( COM_TRC );
		#endif
	}
	
	#if defined ( FILE_LOG )
		/* Log the message to the NVDS trace buffer, if required. */
		if ( xNvdsConfig.usLogEnable && ( uxFileLog & TRACE_FILE ) )
		{
			vTraceStoreHeader( uxTraceMsgId );
			vStoreLongInTraceBuffer( lArg, false );
			vStoreCharInTraceBuffer( '\r' );			
		}		
	#endif

	xSemaphoreGive( xMutexTrace );
}
/*-----------------------------------------------------------*/

/* Trace message with a 3-tuple as argument. */
void vTracePrint3Tuple(  unsigned portBASE_TYPE uxTraceMsgId, short sArg0, short sArg1, short sArg2, unsigned portBASE_TYPE uxFileLog )
{
	configASSERT( xSemaphoreTake( xMutexTrace, TO_MUTEX_TRACE ) );

	if ( uxFileLog & TRACE_UART )
	{
		signed char 		cComStrg[ 25 ];
		signed char 		*pcComStrg;

		pcComStrg = pcTraceWriteUARTHeader( cComStrg, uxTraceMsgId );
		*( pcComStrg++ ) = cNibbleToChar( ( sArg0 >> 12 ) & 0xf );
		*( pcComStrg++ ) = cNibbleToChar( ( sArg0 >>  8 ) & 0xf );
		*( pcComStrg++ ) = cNibbleToChar( ( sArg0 >>  4 ) & 0xf );
		*( pcComStrg++ ) = cNibbleToChar(   sArg0         & 0xf );
		*( pcComStrg++ ) = ' ';
		*( pcComStrg++ ) = cNibbleToChar( ( sArg1 >> 12 ) & 0xf );
		*( pcComStrg++ ) = cNibbleToChar( ( sArg1 >>  8 ) & 0xf );
		*( pcComStrg++ ) = cNibbleToChar( ( sArg1 >>  4 ) & 0xf );
		*( pcComStrg++ ) = cNibbleToChar(   sArg1         & 0xf );
		*( pcComStrg++ ) = ' ';
		*( pcComStrg++ ) = cNibbleToChar( ( sArg2 >> 12 ) & 0xf );
		*( pcComStrg++ ) = cNibbleToChar( ( sArg2 >>  8 ) & 0xf );
		*( pcComStrg++ ) = cNibbleToChar( ( sArg2 >>  4 ) & 0xf );
		*( pcComStrg++ ) = cNibbleToChar(   sArg2         & 0xf );
		*( pcComStrg++ ) = '\r';
		*( pcComStrg++ ) = '\n';
		*( pcComStrg++ ) = 0;
		vCheckAndOpenTraceComPort();
		xComSendString( COM_TRC, cComStrg );
		#if !defined ( TL500 ) && !defined ( TL510 )  && !defined ( TL501 ) 
			/* Only for TL502 and newer. */
			vCOMClose( COM_TRC );
		#endif
	}
	
	#if defined ( FILE_LOG )
		/* Log the message to the NVDS trace buffer, if required. */
		if ( xNvdsConfig.usLogEnable && ( uxFileLog & TRACE_FILE ) )
		{
			vTraceStoreHeader( uxTraceMsgId );
			vStoreCharInTraceBuffer( cNibbleToChar( ( sArg0 >> 12 ) & 0xf ) );
			vStoreCharInTraceBuffer( cNibbleToChar( ( sArg0 >>  8 ) & 0xf ) );
			vStoreCharInTraceBuffer( cNibbleToChar( ( sArg0 >>  4 ) & 0xf ) );
			vStoreCharInTraceBuffer( cNibbleToChar(   sArg0         & 0xf ) );
			vStoreCharInTraceBuffer( ' ' );
			vStoreCharInTraceBuffer( cNibbleToChar( ( sArg1 >> 12 ) & 0xf ) );
			vStoreCharInTraceBuffer( cNibbleToChar( ( sArg1 >>  8 ) & 0xf ) );
			vStoreCharInTraceBuffer( cNibbleToChar( ( sArg1 >>  4 ) & 0xf ) );
			vStoreCharInTraceBuffer( cNibbleToChar(   sArg1         & 0xf ) );
			vStoreCharInTraceBuffer( ' ' );
			vStoreCharInTraceBuffer( cNibbleToChar( ( sArg2 >> 12 ) & 0xf ) );
			vStoreCharInTraceBuffer( cNibbleToChar( ( sArg2 >>  8 ) & 0xf ) );
			vStoreCharInTraceBuffer( cNibbleToChar( ( sArg2 >>  4 ) & 0xf ) );
			vStoreCharInTraceBuffer( cNibbleToChar(   sArg2         & 0xf ) );
			vStoreCharInTraceBuffer( '\r' );	
		}		
	#endif

	xSemaphoreGive( xMutexTrace );
}
/*-----------------------------------------------------------*/

/* Print raw string. */
void vTracePrintSp( const signed char *pcStrg1 )
{
	configASSERT( xSemaphoreTake( xMutexTrace, TO_MUTEX_TRACE ) );

	vCheckAndOpenTraceComPort();
 	xComSendString( COM_TRC, pcStrg1 );
 	xComSendString( COM_TRC, "\r\n" );
	#if !defined ( TL500 ) && !defined ( TL510 )  && !defined ( TL501 ) 
		/* Only for TL502 and newer. */
		vCOMClose( COM_TRC );
	#endif

	xSemaphoreGive( xMutexTrace );
}
/*-----------------------------------------------------------*/

/* Write fault information to trace buffer. 

   When calling this function, a critical error has occurred. The interfaces are accessed without using the mutex protection.
*/
void vTraceFault( uint32_t uiId, uint32_t uiPc, struct xERROR_INFO *pxErrorInfo, uint32_t *puiErrorStack, unsigned portBASE_TYPE uxFileLog )
{
	uint32_t			uiStrLen;
	uint32_t			uiIdx;

	#if defined ( FILE_LOG )
		/* Log the message to the NVDS trace buffer, if required. */
		if ( xNvdsConfig.usLogEnable && ( uxFileLog & TRACE_FILE ) )
		{
			vTraceStoreHeader( TRACE_RESET_EXT_INFO );

			/* ID */
			vStoreLongInTraceBuffer( uiId, false );

			/* PC */
			vStoreLongInTraceBuffer( uiPc, true );
			
			/* INFO */
			if ( uiId == NRF_FAULT_ID_APP_MEMACC )
			{
				vStoreLongInTraceBuffer( pxErrorInfo->err_code, true );
			}
			
			if (    ( uiId == NRF_FAULT_ID_SDK_ASSERT )
				 || ( uiId == NRF_FAULT_ID_SDK_ERROR )
				 || ( uiId == SYSTEM_RESET_ID )
			   )
			{
				/* Line number. */
				vStoreLongInTraceBuffer( pxErrorInfo->line_num, true );

				/* Filename, max. 20 characters. */
				if ( *pxErrorInfo->p_file_name != 0 )
				{
					vStoreLongInTraceBuffer( pxErrorInfo->line_num, true );
					vStoreCharInTraceBuffer( ' ' );
					/* Limit the string length. */
					vStoreStrgInTraceBuffer( ( signed char * )pxErrorInfo->p_file_name, 130 );
				}
			
				if (    ( uiId == NRF_FAULT_ID_SDK_ERROR )
  				     || ( uiId == SYSTEM_RESET_ID )
				   )
				{
					/* Error code. */
					vStoreLongInTraceBuffer( pxErrorInfo->line_num, true );
				}
			}

			if ( uiId == HARDFAULT_ID )
			{
				vStoreLongInTraceBuffer( pxErrorInfo->cfsr,  true );
				vStoreLongInTraceBuffer( pxErrorInfo->hfsr,  true );
				vStoreLongInTraceBuffer( pxErrorInfo->dfsr,  true );
				vStoreLongInTraceBuffer( pxErrorInfo->mmfar, true );
				vStoreLongInTraceBuffer( pxErrorInfo->bfar,  true );
				vStoreLongInTraceBuffer( pxErrorInfo->afsr,  true );
				vStoreLongInTraceBuffer( pxErrorInfo->r0,    true );
				vStoreLongInTraceBuffer( pxErrorInfo->r1,    true );
				vStoreLongInTraceBuffer( pxErrorInfo->r2,    true );
				vStoreLongInTraceBuffer( pxErrorInfo->r3,    true );
				vStoreLongInTraceBuffer( pxErrorInfo->r12,   true );
				vStoreLongInTraceBuffer( pxErrorInfo->lr,    true );
				vStoreLongInTraceBuffer( pxErrorInfo->psr,   true );
			}

			/* Dump the stack contents when the error occurred. */
			for ( uiIdx = 0; uiIdx < ERROR_STACK_DUMP_LEN; uiIdx++ )
			{
				vStoreLongInTraceBuffer( *( puiErrorStack + uiIdx ),  true );
			}		

			vStoreCharInTraceBuffer( '\r' );
		}		
	#endif
	
	if ( uxFileLog & TRACE_UART )
	{
		signed char 		cComStrg[ 500 ];
		signed char 		*pcComStrg;

		pcComStrg = pcTraceWriteUARTHeader( cComStrg, TRACE_RESET_EXT_INFO );
		
		/* ID */
		vLongToHexStrg( pcComStrg, uiId );
		pcComStrg += 8;

		/* PC */
		*( pcComStrg++ ) = ' ';		
		vLongToHexStrg( pcComStrg, uiPc );
		pcComStrg += 8;
			
		/* INFO */
		if ( uiId == NRF_FAULT_ID_APP_MEMACC )
		{
			*( pcComStrg++ ) = ' ';		
			vLongToHexStrg( pcComStrg, pxErrorInfo->err_code );
			pcComStrg += 8;
		}

		if (    ( uiId == NRF_FAULT_ID_SDK_ASSERT )
			 || ( uiId == NRF_FAULT_ID_SDK_ERROR )
			 || ( uiId == SYSTEM_RESET_ID )
		   )
		{
			/* Line number. */
			*( pcComStrg++ ) = ' ';		
			vLongToHexStrg( pcComStrg, pxErrorInfo->line_num );
			pcComStrg += 8;

			/* Filename, max. 20 characters. */
			if ( *pxErrorInfo->p_file_name != 0 )
			{
				uiStrLen = strlen( ( char * )( pxErrorInfo->p_file_name ) );
				if ( uiStrLen > 130 )
				{
					uiStrLen = 130;
				}
				*( pcComStrg++ ) = ' ';		
				strncpy( pcComStrg, pxErrorInfo->p_file_name, uiStrLen  );
				pcComStrg += uiStrLen ;
			}
		
			if (    ( uiId == NRF_FAULT_ID_SDK_ERROR )
				 || ( uiId == SYSTEM_RESET_ID )
			   )
			{
				/* Error code. */
				*( pcComStrg++ ) = ' ';		
				vLongToHexStrg( pcComStrg, pxErrorInfo->err_code );
				pcComStrg += 8;
			}
		}
		

		if ( uiId == HARDFAULT_ID )
		{
			*( pcComStrg++ ) = ' ';		
			vLongToHexStrg( pcComStrg, pxErrorInfo->cfsr );
			pcComStrg += 8;
			
			*( pcComStrg++ ) = ' ';		
			vLongToHexStrg( pcComStrg, pxErrorInfo->hfsr );
			pcComStrg += 8;
			
			*( pcComStrg++ ) = ' ';		
			vLongToHexStrg( pcComStrg, pxErrorInfo->dfsr );
			pcComStrg += 8;
			
			*( pcComStrg++ ) = ' ';		
			vLongToHexStrg( pcComStrg, pxErrorInfo->mmfar );
			pcComStrg += 8;
			
			*( pcComStrg++ ) = ' ';		
			vLongToHexStrg( pcComStrg, pxErrorInfo->bfar );
			pcComStrg += 8;
			
			*( pcComStrg++ ) = ' ';		
			vLongToHexStrg( pcComStrg, pxErrorInfo->afsr );
			pcComStrg += 8;
			
			*( pcComStrg++ ) = ' ';		
			vLongToHexStrg( pcComStrg, pxErrorInfo->r0 );
			pcComStrg += 8;                                                      
																				  
			*( pcComStrg++ ) = ' ';		
			vLongToHexStrg( pcComStrg, pxErrorInfo->r1 );
			pcComStrg += 8;                                                      
																				  
			*( pcComStrg++ ) = ' ';		
			vLongToHexStrg( pcComStrg, pxErrorInfo->r2 );
			pcComStrg += 8;                                                      
																				  
			*( pcComStrg++ ) = ' ';		
			vLongToHexStrg( pcComStrg, pxErrorInfo->r3 );
			pcComStrg += 8;                                                      
																				  
			*( pcComStrg++ ) = ' ';		
			vLongToHexStrg( pcComStrg, pxErrorInfo->r12 );
			pcComStrg += 8;
																				  
			*( pcComStrg++ ) = ' ';		
			vLongToHexStrg( pcComStrg, pxErrorInfo->lr );
			pcComStrg += 8;
																				  
			*( pcComStrg++ ) = ' ';		
			vLongToHexStrg( pcComStrg, pxErrorInfo->psr );
			pcComStrg += 8;
		}

		/* Dump the stack contents when the error occurred. */
		for ( uiIdx = 0; uiIdx < ERROR_STACK_DUMP_LEN; uiIdx++ )
		{
			*( pcComStrg++ ) = ' ';
			vLongToHexStrg( pcComStrg, *( puiErrorStack + uiIdx ) );
			pcComStrg += 8;
		}		
			
		*( pcComStrg++ ) = '\r';
		*( pcComStrg++ ) = '\n';
		*( pcComStrg++ ) = 0;
		vCheckAndOpenTraceComPort();
		xComSendString( COM_TRC, cComStrg );
		#if !defined ( TL500 ) && !defined ( TL510 )  && !defined ( TL501 ) 
			/* Only for TL502 and newer. */
			vCOMClose( COM_TRC );
		#endif
	}
}
/*-----------------------------------------------------------*/

/* Function called once a complete update of the GPS data has been received from the GPS module.

   This function pushes a synopsis of the data to the trace interface for debug purposes. The synopsis is in the form:
   
	TRC: <ID> <cFIX>,<cHDOP>,<usGps_ACCHCM>,<cGsm_SAT>,<uxTotalSatVisible>
   
   Only send this report if all parts of its message have been updated.
*/
void vTracePushGpsFixSynopsis( unsigned char *pcFIX, unsigned char *pcHDOP, unsigned short usGps_ACCHCM, unsigned char *pcGsm_SAT, unsigned portBASE_TYPE uxTotalSatVisible )
{
	signed char 		cComStrg[ 25 ];
	const signed char	cUndefStrg[] = "FFFF";
	signed char 		*pcComStrg;

	configASSERT( xSemaphoreTake( xMutexTrace, TO_MUTEX_TRACE ) );

	pcComStrg = pcTraceWriteUARTHeader( cComStrg, TRACE_GPS_FIX_SYNOPSIS );

	if ( *pcFIX == 0 )
	{
		*( pcComStrg++ ) = '0';
	}
	else
	{
		strncpy( pcComStrg, pcFIX, LEN_GSM_FIX );
		pcComStrg += strlen( pcFIX );
	}
	*( pcComStrg++ ) = ',';
	
	if ( *pcHDOP == 0 )
	{
		strcpy( pcComStrg, cUndefStrg );
		pcComStrg += 4;
	}
	else
	{
		unsigned portBASE_TYPE		uxGPSFixQuality;
		
		/* Convert the HDOP to hex(10*HDOP). 
		   The decimal point of HDOP is either in the second (cGsm_HDOP[1]) or third position (cGsm_HDOP[2]). */
		if ( *( pcHDOP + 2 ) == '.' )
		{
			uxGPSFixQuality = 100 * cCharToNibble( *( pcHDOP + 0 ) )
							 + 10 * cCharToNibble( *( pcHDOP + 1 ) );
		}
		else
		{
			uxGPSFixQuality = 10 * cCharToNibble( *( pcHDOP + 0 ) )
								 + cCharToNibble( *( pcHDOP + 2 ) );
		}
		
		vShortToHexStrg( pcComStrg, uxGPSFixQuality );
		pcComStrg += 4;
	}
	*( pcComStrg++ ) = ',';		
	
	if ( usGps_ACCHCM == 0 )
	{
		strcpy( pcComStrg, cUndefStrg );
		pcComStrg += 4;
	}
	else
	{
		vShortToHexStrg( pcComStrg, usGps_ACCHCM );
		pcComStrg += 4;
	}
	*( pcComStrg++ ) = ',';		
	
	strncpy( pcComStrg, pcGsm_SAT, LEN_GSM_SAT );
	pcComStrg += strlen( pcGsm_SAT );
	*( pcComStrg++ ) = ',';		
	
	if ( uxTotalSatVisible != 0 )
	{
		vShortToHexStrg( pcComStrg, ( unsigned short )uxTotalSatVisible );
		pcComStrg += 4;
	}

	*( pcComStrg++ ) = '\r';		
	*( pcComStrg++ ) = '\n';		
	*( pcComStrg++ ) = 0;		

	vCheckAndOpenTraceComPort();
	xComSendString( COM_TRC, cComStrg );
	#if !defined ( TL500 ) && !defined ( TL510 )  && !defined ( TL501 ) 
		/* Only for TL502 and newer. */
		vCOMClose( COM_TRC );
	#endif

	xSemaphoreGive( xMutexTrace );
}
/*-----------------------------------------------------------*/

/* Add the file and the number of characters to store to the file write command. The handle is in cServerCmdParams. 
   Include the time stamp in the count. */
void prvAddTrcCount( void )
{
	signed char			cLenCharBuf[ 6 ];
	
	/* Get the file handle from the OPEN command. */
	uxTrcFileHandle = ucIntStrgToByte( cServerCmdParams, 3 );
	
	/* and append it right away to current command. */
	vShortToIntStrg( cLenCharBuf, uxTrcFileHandle, true );
	xComSendString( COM_GSM, cLenCharBuf );
	xComSendChar( COM_GSM, ',' );
	
	/* Add the number of characters to write. */
	vShortToIntStrg( cLenCharBuf, usGetTraceBufferNumOfChars(), true );
	xComSendString( COM_GSM, cLenCharBuf );
}
/*-----------------------------------------------------------*/

/* Add the file handle. */
void prvAddHandle( void )
{
	signed char			cLenCharBuf[ 6 ];
	
	vShortToIntStrg( cLenCharBuf, uxTrcFileHandle, true );
	xComSendString( COM_GSM, cLenCharBuf );
}
/*-----------------------------------------------------------*/

/* Write NVDS trace contents to the trace file on Quectel's file system. */
void vTraceFlushLog( void )
{
	#if defined ( OS_TRACE ) && defined ( FILE_LOG )

		bool			bSendSuccess;
		signed char		cChar;

		if ( usGetTraceBufferNumOfChars() > 0 ) 
		{
			/* Get the remaining free space in the UFS Flash memory. */
			if ( xSendAtCommand( &xAtLstFreeU, true, true ) != AT_SUCCESS )
			{
				return;
			}
			
			/* Compare the amount of free space against the required minimum. Abort, if there is less left. */
			if ( ulIntStrgToLong( cServerCmdParams, 6 ) < TRC_MIN_FREE_FLASH_MEMORY + usGetTraceBufferNumOfChars() )
			{
				return;
			}			
			
			/* Take the Trace mutex to make sure no-one else writes the NVDS which we are flushing the log buffer. */
			configASSERT( xSemaphoreTake( xMutexTrace, TO_MUTEX_TRACE ) );
			
			/* Send command initialisation. */
			uxTrcFileHandle = 0xff;
			if ( xSendBatchAtCommand( xAtQWrTrcFile, sizeof( xAtQWrTrcFile ) / sizeof( struct xAT_CMD ), false ) != AT_SUCCESS )
			{
				/* Close the file in case the open had worked. */
				if ( uxTrcFileHandle != 0xff )
				{
					( void )xSendAtCommand( &xAtQFClTrcFile, true, false );
				}

				xSemaphoreGive( xMutexTrace );
				
				return;
			}
			
			/* Write the NVDS trace buffer contents while avoiding to take the Mutex as we already have it. 
			   Make sure the string to send is terminated correctly */
			bSendSuccess = true;
			while ( bGetCharFromTraceBuffer( &cChar ) && bSendSuccess )
			{
				bSendSuccess = xComSendChar( COM_GSM, cChar ) == NRF_SUCCESS;
			}

			if ( !bSendSuccess )
			{
				/* Close the file. */
				( void )xSendAtCommand( &xAtQFClTrcFile, true, false );

				xSemaphoreGive( xMutexTrace );
				
				/* Remove all pending GSM messages, especially those containing errors from the trace flushing. */
				vRemoveAllMsgsFromQueue( xParserAtRespQueue );
				
				return;
			}
			
			/* Wait for the response. */
			if ( xReceiveResponse( AT_OK, AT_NOMSG, AT_NOMSG, TO_GSM_FS ) != AT_SUCCESS )
			{
				/* In case there was no write confirm, send a few characters and wait for the GSM module to process them in the hope that 
				   this will terminate the write operation. Usually when this occurs, only a single characters has been omitted. */
				xComSendChar( COM_GSM, ' ' );
				xComSendChar( COM_GSM, ' ' );
				xComSendChar( COM_GSM, ' ' );
				xComSendChar( COM_GSM, ' ' );
				xComSendChar( COM_GSM, '\r' );
				vTaskDelay( TO_GSM_FS );
			}			
			/* Close the file. */
			( void )xSendAtCommand( &xAtQFClTrcFile, true, false );
			
			/* Remove all pending GSM messages, especially those containing errors from the trace flushing. */
			vRemoveAllMsgsFromQueue( xParserAtRespQueue );
			
			xSemaphoreGive( xMutexTrace );

			/* Clear the trace buffer. */
			vEraseTraceBuffer();
		}
	#endif
}
/*-----------------------------------------------------------*/

/* Collect raw statistics about the system, To start with, just count the number of times a 
   certain system state is entered. We focus here on states which are not entered on server
   command.
*/
void vSystemStateStats( enum xACTION xAction, enum xCTRL_STATE xNextCtrlState )
{
	unsigned short		usStateDwellTime;
	
	if ( xAction == SYSSTATS_INIT )
	{
		/* Initialise GSM/GPS module statistics. */
		uStats.xStats.usGsmModuleCrash			= 0;
		uStats.xStats.usGsmModuleReboot			= 0;
		uStats.xStats.usGsmAttachFail			= 0;	
		uStats.xStats.usGprsConnectFail			= 0;		
		
		/* Initialise statistics variables. */
		uStats.xStats.usCntEnterSystemActive	= 0;		
		uStats.xStats.usCntEnterSystemInactive	= 0;        
		uStats.xStats.usCntEnterSystemStill		= 0;        
		uStats.xStats.usCntEnterSystemSleep		= 0;
		uStats.xStats.usCntEnterSystemPrealert	= 0;        
		uStats.xStats.usCntEnterSystemAlert		= 0;        
		uStats.xStats.usCntEnterSystemPreSos	= 0;
		uStats.xStats.usCntEnterSystemSos		= 0;
		uStats.xStats.usCntEnterSystemCharging	= 0;        
		                                                    
		uStats.xStats.usCntDwellSystemActive	= 0;	    
		uStats.xStats.usCntDwellSystemInactive 	= 0;
		uStats.xStats.usCntDwellSystemStill		= 0;
		uStats.xStats.usCntDwellSystemSleep		= 0;
		uStats.xStats.usCntDwellSystemPrealert 	= 0;
		uStats.xStats.usCntDwellSystemAlert		= 0;
		uStats.xStats.usCntDwellSystemPreSos	= 0;
		uStats.xStats.usCntDwellSystemSos		= 0;
		uStats.xStats.usCntDwellSystemCharging 	= 0;	
		
		usStateEnterTimeStamp = usReadRTC();
	}
	else
	{
		/* Track the accumulated state dwell time for any other action. */
		usStateDwellTime = usReadRTC() - usStateEnterTimeStamp;
		usStateEnterTimeStamp = usReadRTC();
		
		switch ( xGetCtrlState() )
		{
			case CTRL_ACTIVE:			uStats.xStats.usCntDwellSystemActive 	+= usStateDwellTime;	break;
			
			case CTRL_INACTIVE:			uStats.xStats.usCntDwellSystemInactive 	+= usStateDwellTime;	break;
										
			case CTRL_STILL:			uStats.xStats.usCntDwellSystemStill 	+= usStateDwellTime;	break;
										
			case CTRL_SLEEP:			uStats.xStats.usCntDwellSystemSleep 	+= usStateDwellTime;	break;
										
			case CTRL_PREALERT:			uStats.xStats.usCntDwellSystemPrealert 	+= usStateDwellTime;	break;
			
			case CTRL_ALERT:			uStats.xStats.usCntDwellSystemAlert 	+= usStateDwellTime;	break;
			
			case CTRL_PRESOS:			uStats.xStats.usCntDwellSystemPreSos 	+= usStateDwellTime;	break;
			
			case CTRL_SOS:				uStats.xStats.usCntDwellSystemSos	 	+= usStateDwellTime;	break;
			
			case CTRL_CHRG_NRML:	
			case CTRL_CHRG_OOO:			uStats.xStats.usCntDwellSystemCharging 	+= usStateDwellTime; 	break;
			
			case CTRL_STANDBY:
			case CTRL_OOO:
			default: 					/* Do not track these states. */								break;
		}
		
		/* In standard cases where not only the dwell time is updated, also update the counters for entering system states. */
		if ( xAction != SYSSTATS_UPDATEDWELL )
		{
			/* Increment the corresponding state enter counter. Explicit variable names here rather than an array 
			   to make identification of the variable address in the .map file easy.*/
			switch ( xNextCtrlState )
			{
				case CTRL_ACTIVE:			uStats.xStats.usCntEnterSystemActive++;			break;
				
				case CTRL_INACTIVE:			uStats.xStats.usCntEnterSystemInactive++;		break;
											
				case CTRL_STILL:			uStats.xStats.usCntEnterSystemStill++;			break;
											
				case CTRL_SLEEP:			uStats.xStats.usCntEnterSystemSleep++;			break;
				
				case CTRL_PREALERT:			uStats.xStats.usCntEnterSystemPrealert++;		break;
				
				case CTRL_ALERT:			uStats.xStats.usCntEnterSystemAlert++;			break;
				
				case CTRL_PRESOS:			uStats.xStats.usCntEnterSystemPreSos++;			break;
				
				case CTRL_SOS:				uStats.xStats.usCntEnterSystemSos++;			break;
				
				case CTRL_CHRG_NRML:	
				case CTRL_CHRG_OOO:			uStats.xStats.usCntEnterSystemCharging++; 		break;
				
				case CTRL_STANDBY:			
				case CTRL_OOO:
				default: 					/* Do not track these states. */				break;
			}
		}
	}	
}
/*-----------------------------------------------------------*/

/* Collect raw statistics about the NoAbn system mode. */
void vSystemNoAbnStats( enum xNOABNACTION xAction )
{
	switch ( xAction )
	{
		case SYSSTATS_NOABN_INIT:	
								uStats.xStats.usCntEnterSystemNoAbn			= 0;   
								uStats.xStats.usCntDwellSystemNoAbn 		= 0;			
								break;

		case SYSSTATS_NOABN_UPDATEDWELL:
								/* Track the accumulated state dwell time. */
								uStats.xStats.usCntDwellSystemNoAbn += usReadRTC() - usNoAbnEnterTimeStamp;
								usNoAbnEnterTimeStamp = usReadRTC();
								break;
								
		case SYSSTATS_NOABN_MODESTART:
								usNoAbnEnterTimeStamp = usReadRTC();
								uStats.xStats.usCntEnterSystemNoAbn++;
								break;
								
		case SYSSTATS_NOABN_MODESTOP:
								uStats.xStats.usCntDwellSystemNoAbn += usReadRTC() - usNoAbnEnterTimeStamp;
								break;
								
		default:				break;
	}	
}
/*-----------------------------------------------------------*/

/* Increment reset counters. 

   Several reset reasons might be flagged. Therefore, first only the reset counters held in the xNvdsConfig structure are
   incremented. 
   
   The trace buffer copy is in RAM.
*/
void vIncrementResetCount( void )
{
	bWatchdogRecovery = false;
	bSoftResetRecovery = false;
	
	/* Increase the count of unsuccessful boots. */
	uxErrorRecord.ulUnsuccessfulBootCount++;
	
	/* Backup the reset reason register contents so that it can be stored in the trace later. */
	ulResetReasonCopy = NRF_POWER->RESETREAS;
	
	/* Check for power-on reset. Those resets are not flagged by any dedicated bit in the reset reason register (for some strange reason). */
	if ( ulResetReasonCopy == 0 )
	{
		/* Increment the reset error counter in the NVDS. */
		uxErrorRecord.ulPOResetCount++;
	}	
	/* Check for software reset. */
	else if ( ( ulResetReasonCopy & POWER_RESETREAS_SREQ_Msk ) != 0 )
	{
		/* Increment the software reset error counter in the NVDS. */
		uxErrorRecord.ulSWResetCount++;
		
		bSoftResetRecovery = true;
	}
	/* Check for watchdog reset. */
	else if ( ( ulResetReasonCopy & POWER_RESETREAS_DOG_Msk ) != 0 )
	{
		/* Increment the watchdog reset error counter in the NVDS. */
		uxErrorRecord.ulWDResetCount++;
		
		bWatchdogRecovery = true;
	}	
	/* Check for lockup reset. */
	else if ( ( ulResetReasonCopy & POWER_RESETREAS_LOCKUP_Msk ) != 0 )
	{
		/* Increment the power-on reset error counter in the NVDS. */
		uxErrorRecord.ulLockupResetCount++;
	}
	/* Anything else is not normal but should be recorded as well. 
	   Flagged by the device are:
			- Reset from pin-reset detected.
			- Reset due to wake up from System OFF mode when wakeup is triggered from DETECT signal from GPIO.
			- Reset due to wake up from System OFF mode when wakeup is triggered from ANADETECT signal from LPCOMP.
			- Reset due to wake up from System OFF mode when wakeup is triggered from entering into debug interface mode.
			- Reset due to wake up from System OFF mode by NFC field detect.
			- Reset due to wake up from System OFF mode by VBUS rising into valid range.			*/
	else
	{
		uxErrorRecord.ulOtherResetCount++;
	}
	
	/* Write the system error record back to Flash memory so that the latest
	   reset counters are recorded. */
	vWriteSystemErrorRecord( &uxErrorRecord, FSTORAGE_NVMC );
	
	/* Clear reset status. */
	NRF_POWER->RESETREAS = 0xffffffff;
}
/*-----------------------------------------------------------*/

/* Function called in case of stack overflows. As this is critical,
   store all relevant information in the NVDS and execute a hard reset of 
   the entire system.
*/
void vApplicationStackOverflowHook( void * xTask, char *pcTaskName )
{
	struct xSYSTEM_ERROR_RECORD		uxErrorRecord;
	
	( void )xTask;
	
	/* Get the current system error record from NVDS. 	
	   Increment the stack overflow counter and write the record back. */
	uxErrorRecord.ulStackOverflowCount++;

	/* Store the Task name. */
	strncpy( uxErrorRecord.cStackOverflowTaskName, pcTaskName, LEN_TASKNAME );
	
	/* Raise system error. 
	   The system error will get transferred to NVDS as part of the error routine. */
	configASSERT( false );
}
/*-----------------------------------------------------------*/

/* Function called in case of heap allocation failure. As this is critical,
   execute a hard reset of the entire system.
*/
void vApplicationMallocFailedHook( void )
{
	/* Raise system error. */
	configASSERT( false );
}
/*-----------------------------------------------------------*/
