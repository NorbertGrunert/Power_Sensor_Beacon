/*
 * Tracker Firmware
 *
 * UART driver
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>

 /* Scheduler include files. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

/* nRF specific include files. */
#include "bsp.h"
#include "nrf_serial.h"

/* Device specific include files. */
#include "drv_uart.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
static void 			vSerialEventHandlerCom0( struct nrf_serial_s const * p_serial, nrf_serial_event_t event );
static void 			vSleepHandler( void );
void			 		vUartPortInit( void );
void					vCOMOpen( unsigned portBASE_TYPE xComID, signed char *pcUartRxBuffer );
void 					vCOMClose( unsigned portBASE_TYPE xComID );
signed portBASE_TYPE 	xStoreRxCharInBuffer( struct COM_PORT *pxCom, signed char cChar );
signed portBASE_TYPE 	xGetRxCharFromBuffer( unsigned portBASE_TYPE xComID, signed char *pcChar );
signed char				cGetRxCharFromBufferWithIndex( unsigned portBASE_TYPE xComID, unsigned char ucIdx );
signed char 			*cGetPointerToBufferWithIndex( unsigned portBASE_TYPE xComID, unsigned char ucIdx );
signed portBASE_TYPE 	xComReceiveString( unsigned portBASE_TYPE xComID, TickType_t xBlockTime );
void 					vUartRemoveString( unsigned portBASE_TYPE xComID );
unsigned portBASE_TYPE 	uxGetNumberOfCharsInBuffer( unsigned portBASE_TYPE xComID );
unsigned portBASE_TYPE 	uxGetNumberOfStringsInBuffer( unsigned portBASE_TYPE xComID );
unsigned portBASE_TYPE 	uxGetNumberOfMsgWaiting( unsigned portBASE_TYPE xComID );
// signed portBASE_TYPE 	xComSendChar( unsigned portBASE_TYPE xComID, signed char cChar  );
// signed portBASE_TYPE 	xComSendStringROM( unsigned portBASE_TYPE xComID, PGM_P pcTxString );
signed portBASE_TYPE 	xComSendStringRAM( unsigned portBASE_TYPE xComID, signed char *pcTxString );
// signed portBASE_TYPE 	xComSendStringEEPROM( unsigned portBASE_TYPE xComID, signed char *pcTxString, bool bTranslateApostr );
signed portBASE_TYPE	vUartRxISR( struct nrf_serial_s const * p_serial, struct COM_PORT *pxCom );
// signed portBASE_TYPE 	vUartTxISR( struct COM_PORT *pxCom );
// /*-----------------------------------------------------------*/

/* Module scope variables */
struct COM_PORT	xCom[NUM_COMS];						/* Array of structures containing all variables for all supported UARTs. */

NRF_SERIAL_DRV_UART_CONFIG_DEF( mDrvConfigCom0,
							    RX_PIN_NUMBER, TX_PIN_NUMBER,
							    RTS_PIN_NUMBER, CTS_PIN_NUMBER,
							    NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
							    NRF_UART_BAUDRATE_38400,
							    UART_DEFAULT_CONFIG_IRQ_PRIORITY );

#define SERIAL_FIFO_TX_SIZE 300					/* Allow for enough space in the TX buffer to accomodate 3 scan reports. */
#define SERIAL_FIFO_RX_SIZE 100					/* Allow for enough space in the RX buffer to accomodate 1 advertisement packet. */

NRF_SERIAL_QUEUES_DEF( serialQueuesCom0, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE );


#define SERIAL_BUFF_TX_SIZE 1
#define SERIAL_BUFF_RX_SIZE 1

NRF_SERIAL_BUFFERS_DEF( serialBuffersCom0, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE );

NRF_SERIAL_CONFIG_DEF( serialConfigCom0, NRF_SERIAL_MODE_IRQ,
                       &serialQueuesCom0, &serialBuffersCom0, vSerialEventHandlerCom0, vSleepHandler );


NRF_SERIAL_UART_DEF( serialCom0, 0 );
/*-----------------------------------------------------------*/

/* Initialise the UART driver: create TX queue and RX semaphore. Done only once.
 *
 * The UART HW itself will only be initialised when opening the UART port.
*/
 void vUartPortInit( void )
 {
    ret_code_t				xErrCode;
	unsigned portBASE_TYPE	uxID;
	
	/* Initialise all UART HW instances. */
    xErrCode = nrf_serial_init(&serialCom0, &mDrvConfigCom0, &serialConfigCom0);
    APP_ERROR_CHECK(xErrCode);

	/* Initialise COM structures for all UARTs. */
	for ( uxID = 0; uxID < NUM_COMS; uxID++ )
	{
		/* Enter identifier. */
		xCom[ uxID ].uxUartPortID = uxID;
		
		// /* Create the transmit queues. */
		// xCom[ uxID ].xUartTxQueue = xQueueCreate( uartTX_BUFFER_SIZE, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
		
		// #if defined ( DEBUG )	
			// /* Register queue for inspection with a kernel-aware debugger. */
			// vQueueAddToRegistry( xCom[ uxID ].xUartTxQueue, "UART Tx");
		// #endif
		
		xCom[ uxID ].xUartRxCountingSemaphore = xSemaphoreCreateCounting( 7, 0 );	/* max. count to 7, 0 initially. */
		
		#if defined ( DEBUG )	
			/* Register queue for inspection with a kernel-aware debugger. */
			vQueueAddToRegistry( xCom[ uxID ].xUartRxCountingSemaphore, "UART Rx strg cnt");
		#endif
	}
	
	nrf_gpio_cfg_input( RX_PIN_NUMBER, NRF_GPIO_PIN_PULLUP );		// DEBUG DEBUG DEBUG
 }
 /*-----------------------------------------------------------*/

/* Sleep handler. 
   Not sure what is does...
*/
static void vSleepHandler( void )
{
    __WFE();
    __SEV();
    __WFE();
}
/*-----------------------------------------------------------*/


/* Open COM port. 

   The association of a physical UART with a logical COM port is done here.

   Input parameters:
		xComID					ID of the COM port to be opened.
		pcUartRxBuffer			Pointer to RX character ring buffer.
*/
void vCOMOpen( unsigned portBASE_TYPE xComID, signed char *pcUartRxBuffer )
{
	/* Initialise the COM port structure. */
	portENTER_CRITICAL();
	{		
		/* Initialise pointer to Rx ring buffer. */
		xCom[ xComID ].pcUartRxBuffer = pcUartRxBuffer;
		
		/* Initialise Rx ring buffer read/write pointers. */
		xCom[ xComID ].uxUartRxWrIdx = 0;
		xCom[ xComID ].uxUartRxRdIdx = 0;
		xCom[ xComID ].uxRxStrgLen = 0;
	}
	portEXIT_CRITICAL();
	
	/* Register HW UART for COM0. */
	if ( xComID == COM0 )
	{
		xCom[ COM0 ].pxUartHWInst = &serialCom0;
	}
}
/*-----------------------------------------------------------*/

/* Close UART port. */
void vCOMClose( unsigned portBASE_TYPE xComID )
{
    ret_code_t				xErrCode;
	
	/* Turn off the interrupts.  We may also want to delete the queues and/or
	   re-install the original ISR. */
    xErrCode = nrf_serial_uninit ( xCom[ xComID ].pxUartHWInst );
    APP_ERROR_CHECK( xErrCode );
	
	/* Flush TX buffer. */
	// ( void )xQueueReset( xCom[ xComID ].xUartTxQueue );	
}
/*-----------------------------------------------------------*/

/* Store one character in the RX ring buffer. The buffer must be able to
   contain at least the character and one end-of-string marker (0x00) to
   make sure that it does not contain empty strings.
   Return FALSE when buffer is full, else TRUE.
*/
signed portBASE_TYPE xStoreRxCharInBuffer( struct COM_PORT *pxCom, signed char cChar )
{
	if ( ( ( pxCom->uxUartRxWrIdx + 1 ) % uartRX_BUFFER_SIZE ) == pxCom->uxUartRxRdIdx )
	{
		/* Buffer is full */
		return pdFALSE;
	}
	else
	{
		/* Store character in buffer */
		*( pxCom->pcUartRxBuffer + pxCom->uxUartRxWrIdx ) = cChar;
		pxCom->uxUartRxWrIdx = ( pxCom->uxUartRxWrIdx + 1 ) % uartRX_BUFFER_SIZE;
		
		/* If buffer is full now, add the end-of-string character 0x0 to truncate the string. */
		if ( ( ( pxCom->uxUartRxWrIdx + 1 ) % uartRX_BUFFER_SIZE ) == pxCom->uxUartRxRdIdx )
		{
			*( pxCom->pcUartRxBuffer + pxCom->uxUartRxWrIdx ) = 0;
		}
	}
	
	return pdTRUE;
}
/*-----------------------------------------------------------*/

/* Get one character from the RX ring buffer. 
   Return FALSE if the buffer is empty, else TRUE.
*/
signed portBASE_TYPE xGetRxCharFromBuffer( unsigned portBASE_TYPE xComID, signed char *pcChar )
{
	if ( xCom[ xComID ].uxUartRxRdIdx == xCom[ xComID ].uxUartRxWrIdx )
	{
		/* Buffer is empty. */
		return pdFALSE;
	}
	else
	{
		*pcChar = *( xCom[ xComID ].pcUartRxBuffer + xCom[ xComID ].uxUartRxRdIdx );
		xCom[ xComID ].uxUartRxRdIdx = ( xCom[ xComID ].uxUartRxRdIdx + 1 ) % uartRX_BUFFER_SIZE;
	}
		
	return pdTRUE;
}
/*-----------------------------------------------------------*/

/* Get one character from the RX ring buffer with index. 
   The read pointer is not updated.
   Does not check buffer status!
*/
signed char cGetRxCharFromBufferWithIndex( unsigned portBASE_TYPE xComID, unsigned char ucIdx )
{
	return *( xCom[ xComID ].pcUartRxBuffer + ( ( xCom[ xComID ].uxUartRxRdIdx + ucIdx ) % uartRX_BUFFER_SIZE ) );
}
/*-----------------------------------------------------------*/

/* Get pointer to character in the RX ring buffer from index. */
signed char *cGetPointerToBufferWithIndex( unsigned portBASE_TYPE xComID, unsigned char ucIdx )
{
	return xCom[ xComID ].pcUartRxBuffer + ( ( xCom[ xComID ].uxUartRxRdIdx + ucIdx ) % uartRX_BUFFER_SIZE );
}
/*-----------------------------------------------------------*/

/* Wait for the counting semaphore to be given by the RX ISR which indicated that a complete 
   string has been received.
   Return false if no complete string is available or arrives before xBlockTime expires. 
*/
signed portBASE_TYPE xComReceiveString( unsigned portBASE_TYPE xComID, TickType_t xBlockTime )
{
	if( xSemaphoreTake( xCom[ xComID ].xUartRxCountingSemaphore, xBlockTime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}
/*-----------------------------------------------------------*/

/* Remove the next string from the RX ring buffer. */
void vUartRemoveString( unsigned portBASE_TYPE xComID )
{
	signed char	cRxChar;
	signed portBASE_TYPE xBufferFilled = 0;
	
	/* Remove the next string from the RX ring buffer. */
	do
	{
		xBufferFilled = xGetRxCharFromBuffer( xComID, &cRxChar );
	} while ( ( xBufferFilled ) && ( cRxChar != 0 ) );
}
/*-----------------------------------------------------------*/

/* Get the number of characters (including end-of-string, 0x00) currently present in the 
   RX ring buffer.
*/
unsigned portBASE_TYPE uxGetNumberOfCharsInBuffer( unsigned portBASE_TYPE xComID )
{
	if ( xCom[ xComID ].uxUartRxRdIdx <= xCom[ xComID ].uxUartRxWrIdx )
	{
		/* Write index is ahead of read index. */
		return xCom[ xComID ].uxUartRxWrIdx - xCom[ xComID ].uxUartRxRdIdx;
	}
	else
	{
		/* Read index is ahead of write index: buffer has wrapped around. */
		return uartRX_BUFFER_SIZE + xCom[ xComID ].uxUartRxWrIdx - xCom[ xComID ].uxUartRxRdIdx;
	}
}
/*-----------------------------------------------------------*/

/* Get the number of strings currently present in the RX ring buffer. */
unsigned portBASE_TYPE uxGetNumberOfStringsInBuffer( unsigned portBASE_TYPE xComID )
{
	unsigned char ucRdIdx;
	unsigned char ucNumStrgs;
	
	ucNumStrgs = 0;
	
	for ( ucRdIdx = 0; ucRdIdx < uxGetNumberOfCharsInBuffer( xComID ); ucRdIdx++ )
	{
		/* Check, if the character pointed to by the current index is end-of-string. */
		if ( cGetRxCharFromBufferWithIndex( xComID, ucRdIdx ) == 0 )
		{
			ucNumStrgs++;
		}
	}
	
	return ucNumStrgs;
}
/*-----------------------------------------------------------*/

/* Get the number of semaphores stored in xUartRxCountingSemaphore. */
unsigned portBASE_TYPE uxGetNumberOfMsgWaiting( unsigned portBASE_TYPE xComID )
{
	return uxQueueMessagesWaiting( xCom[ xComID ].xUartRxCountingSemaphore );
}
/*-----------------------------------------------------------*/

// /* Set the baud rate on an open COM port . 

   // Input parameters:
		// xComID					ID of the COM port.
		// usBaudRate 				Baud rate generator setting: high 4 bits are BSCALE, low 12 bits are BSEL.
// */
// void vComSetBaudrate( unsigned portBASE_TYPE xComID, unsigned short usBaudRate )
// {
	// vUSARTBaudrateSet( xCom[ xComID ].pxUartRegs , usBaudRate );
// }
// /*-----------------------------------------------------------*/

// /* Send one character to the UART. */
// signed portBASE_TYPE xComSendChar( unsigned portBASE_TYPE xComID, signed char cChar )
// {
	// /* Return false if after the block time there is no room on the Tx queue. */
	// if( xQueueSend( xCom[ xComID ].xUartTxQueue, &cChar, uartTX_BLOCKTIME ) != pdPASS )
	// {
		// signed char		cNextChar;
		
		// /* Getting here is an error condition. Usually, this means that the 
		   // UART TX is blocked for some reason, i.e. the queue is full, interrupts
		   // are enabled but the TX interrupt got lost. 
		   // Fetch the next character from the TX queue and feed it manually to the 
		   // UART and re-enable the TX interrupt. That should unblock the situation. */
		// xQueueReceive( xCom[ xComID ].xUartTxQueue, &cNextChar, 0);
		// vUSARTDreInterruptLevelSet( xCom[ xComID ].pxUartRegs, USART_DREINTLVL_LO_gc );		
		// vUSARTPutChar( xCom[ xComID ].pxUartRegs, cNextChar );
		
		// /* Now stuff the next character to send in the queue. */
		// xQueueSend( xCom[ xComID ].xUartTxQueue, &cChar, uartTX_BLOCKTIME );
		
		// return pdFAIL;
	// }
	
	// /* Enable TXD interrupt. */
	// vUSARTDreInterruptLevelSet( xCom[ xComID ].pxUartRegs, USART_DREINTLVL_LO_gc );

	// return pdPASS;
// }
// /*-----------------------------------------------------------*/

// /* Send the string pointed to by pcTxString character by character.     
// */
// signed portBASE_TYPE xComSendStringROM( unsigned portBASE_TYPE xComID, PGM_P pcTxString )
// {
	// if ( pcTxString != NULL )
	// {
		// while ( pgm_read_byte( pcTxString ) != 0 )
		// {
			// if ( !xComSendChar( xComID, pgm_read_byte( pcTxString++ ) ) )
			// {
				// return pdFALSE;
			// }
		// }
	// }
	// return pdTRUE;
// }
// /*-----------------------------------------------------------*/

/* Send the string pointed to by pcTxString character by character. The string is located in RAM. 
   If there is not enough space left in the TX queue written by nrf_serial_write, do not attempt to 
   send the string but return pdFALSE. */
signed portBASE_TYPE xComSendStringRAM( unsigned portBASE_TYPE xComID, signed char *pcTxString )
{
    ret_code_t 				xErrCode;
	size_t					xTxQueueSpaceLeft;
	size_t					xTxStrgLen;
	signed portBASE_TYPE	xSuccessCode;
	
	xSuccessCode = pdTRUE;
	
	if ( pcTxString != NULL )
	{
		/* Verify that the string to be written has enough space in the TX queue. */
		xTxQueueSpaceLeft = nrf_queue_available_get( serialQueuesCom0.p_txq );
		xTxStrgLen = strlen( pcTxString );
		if ( xTxQueueSpaceLeft < xTxStrgLen )
		{
			char		cBuf[ 10 ];
			
			sprintf( cBuf, "_%i\r\n", xTxQueueSpaceLeft );
			xErrCode = nrf_serial_write( xCom[ xComID ].pxUartHWInst,
										 cBuf,
										 strlen( cBuf ),
										 NULL,
										 0 );					/* nrf_serial operates in non-blocking mode. */
			/* Trash string to be sent and return an error code. */
			xSuccessCode = pdFALSE;
		}
		else
		{
			xErrCode = nrf_serial_write( xCom[ xComID ].pxUartHWInst,
										 pcTxString,
										 xTxStrgLen,
										 NULL,
										 0 );					/* nrf_serial operates in non-blocking mode. */
			APP_ERROR_CHECK( xErrCode );
		}
	}
	return xSuccessCode;
}
/*-----------------------------------------------------------*/

// /* Send the string pointed to by pcTxString character by character. The string is located in EEPROM. 
   // If requested, translate any quotation marks into single quotes. */
// signed portBASE_TYPE xComSendStringEEPROM( unsigned portBASE_TYPE xComID, signed char *pcTxString, bool bTranslateApostr )
// {
	// signed char cTxChar;
	
	// if ( pcTxString != NULL )
	// {
		// while ( ( cTxChar = ucEEPROMReadByte( ( unsigned char * )pcTxString++ ) ) != 0 )
		// {
			// if ( bTranslateApostr && ( cTxChar == '\'' ) )
			// {
				// cTxChar = '\"';
			// }

			// if ( !xComSendChar( xComID, cTxChar ) )
			// {
				// return pdFALSE;
			// }
			// vDelay( uartTX_DELAY );
		// }
	// }
	// return pdTRUE;
// }
// /*-----------------------------------------------------------*/

	
/* Generic RX interrupt routine. 
   Returns true if a yield from ISR needs to be done. 
*/
signed portBASE_TYPE vUartRxISR( struct nrf_serial_s const * p_serial, struct COM_PORT *pxCom )
{
	signed char cChar;
	signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* Get the character, store it in the RX ring buffer and give the RX semaphore for the Parser task.
	If the semaphore causes the Parser task to wake, force a context switch as the woken task
	may have a higher priority than the task we have interrupted. */
	APP_ERROR_CHECK( nrf_queue_read( p_serial->p_ctx->p_config->p_queues->p_rxq, &cChar, sizeof( cChar ) ) );
		
	#if defined ( OS_TRACE )
		/* Output a '#' each time a character from the upper 128-character portion of the ASCII table is 
		   received. Normally this should not occur in the standard AT-ptotocol. */
		if ( cChar < 0 )
		{
			/* TODO: Maybe log something later??? */
		}
	#endif
	
	/* Check if a CR (0x0d, '\r') has been received. In this case, add a '0' to the buffer and indicate that 
	   a string has been received. 
	   Suppress LF characters (0x0a, '\n'), 0x1a and 0x00. Store all other characters in the RX ring buffer. */
	if ( cChar == '\r' )
	{
		/* Only pass a string to the Parser, if it contains any characters at all. So sequences of CRLF only will never
		   trigger AT parsing. */
		if ( pxCom->uxRxStrgLen != 0 )
		{
			if ( !xStoreRxCharInBuffer( pxCom, 0 ) )
			{
				/* TODO: handle buffer full condition */
			}
			xSemaphoreGiveFromISR( pxCom->xUartRxCountingSemaphore, &xHigherPriorityTaskWoken );
			pxCom->uxRxStrgLen = 0;
		}
	}
	else if ( ( ( cChar == '[' ) || ( cChar == '{' ) ) && ( pxCom->uxRxStrgLen != 0 ) )
	{
		/* The string contains two concatenated messages from the server. Server messages are
		   delimited by '[' and ']' or by '{' and '}'.
		   In this case, terminate the first message before storing the new message. */
		if ( !xStoreRxCharInBuffer( pxCom, 0 ) )
		{
			/* TODO: handle buffer full condition */
		}
		pxCom->uxRxStrgLen = 0;
		xSemaphoreGiveFromISR( pxCom->xUartRxCountingSemaphore, &xHigherPriorityTaskWoken );

		/* Now store the start of the message. */
		if ( !xStoreRxCharInBuffer( pxCom, cChar ) )
		{
			/* TODO: handle buffer full condition */
		}
		else
		{
			pxCom->uxRxStrgLen++;				/* should never exeed 255; but if it does, it simply wraps (might lose a string, though) */
		}
	}
	else if ( ( cChar != '\n' ) && ( cChar != 0 ) && ( cChar != 0x1a ) )
	{
		if ( !xStoreRxCharInBuffer( pxCom, cChar ) )
		{
			/* TODO: handle buffer full condition */
		}
		else
		{
			pxCom->uxRxStrgLen++;				/* should never exeed 255; but if it does, it simply wraps (might lose a string, though) */
		}
	}
	
	return xHigherPriorityTaskWoken;
}
/*-----------------------------------------------------------*/

char	cUartRxChar;

/* UART event handler, one per UART instance. */
static void vSerialEventHandlerCom0( struct nrf_serial_s const *p_serial, nrf_serial_event_t event )
{
	switch ( event )
	{
		case NRF_SERIAL_EVENT_TX_DONE:
			break;
		
		case NRF_SERIAL_EVENT_RX_DATA:
			( void )vUartRxISR( p_serial, &xCom[ COM0 ] );
			break;
		
		case NRF_SERIAL_EVENT_DRV_ERR:
			( void )nrf_serial_write( xCom[ 0 ].pxUartHWInst,
										 "_2\r\n",
										 4,
										 NULL,
										 0 );					/* nrf_serial operates in non-blocking mode. */
			/* TODO: Maybe log something later??? */
			break;
		
		case NRF_SERIAL_EVENT_FIFO_ERR:
			( void )nrf_serial_write( xCom[ 0 ].pxUartHWInst,
										 "_3\r\n",
										 4,
										 NULL,
										 0 );					/* nrf_serial operates in non-blocking mode. */
			/* TODO: Maybe log something later??? */
			break;
		
		default:
			break;
	}
}