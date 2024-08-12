/*
 * Tracker Bootloader
 *
 * UART driver
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>

/* nRF SDK files. */
#define NRF_LOG_MODULE_NAME 				DRV_UART
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

/* Device specific include files. */
#include "custom_board.h"
#include "drv_uart.h"
#include "main.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* UART event handler, one per UART instance. */
static void 			vUartEventHandler( void * context, nrf_libuarte_async_evt_t * p_evt );
void			 		vUartPortInit( void );
void					vCOMOpen( unsigned portBASE_TYPE xComID, signed char *pcUartRxBuffer, unsigned portBASE_TYPE uxUartRxBufLenM1, enum xCOM_MODE xMode );
void 					vCOMClose( unsigned portBASE_TYPE xComID );
bool 					bCOMIsOpen( unsigned portBASE_TYPE xComID );
enum COM_RXBUF_STORE 	xStoreRxCharInBuffer( struct COM_PORT *pxCom, signed char cChar );
bool				 	bGetRxCharFromBuffer( unsigned portBASE_TYPE xComID, signed char *pcChar );
signed char				cGetRxCharFromBufferWithIndex( unsigned portBASE_TYPE xComID, uint32_t uiIdx );
signed char 			*cGetPointerToBufferWithIndex( unsigned portBASE_TYPE xComID, uint32_t uiIdx );
bool					bComReceiveString( unsigned portBASE_TYPE xComID, TickType_t xBlockTime );
void 					vUartRemoveString( unsigned portBASE_TYPE xComID );
uint32_t				uiGetNumberOfCharsInBuffer( unsigned portBASE_TYPE xComID );
uint32_t				uiGetNumberOfStringsInBuffer( unsigned portBASE_TYPE xComID );
ret_code_t				xComSendChar( unsigned portBASE_TYPE xComID, signed char cChar );
ret_code_t				xComSendString( unsigned portBASE_TYPE xComID, const signed char *pcTxString );
ret_code_t				xComSendStringIRQ( unsigned portBASE_TYPE xComID, const signed char *pcTxString );
void					vUartRxISR( nrf_libuarte_async_evt_t * p_evt, struct COM_PORT *pxCom );
/*-----------------------------------------------------------*/

/* Module scope variables */
struct COM_PORT		xCom[ NUM_COMS ];				/* Array of structures containing all variables for all supported UARTs. */

/* Define the UART structure for COM0. 
		UARTE instance:																		0
		TIMER instance used by libuarte for bytes counting (timer0 is used for the SD):		1
		RTC instance used for timeout:														NRF_LIBUARTE_PERIPHERAL_NOT_USED
		TIMER instance used for timeout:													NRF_LIBUARTE_PERIPHERAL_NOT_USED
		Size of single RX buffer:															127
		Number of buffers in the RX buffer pool:											3								 
   As there is no HW ressource defined for timeout, an app timer is used. */
NRF_LIBUARTE_ASYNC_DEFINE( xLibUarteCOM0, 0, 1, NRF_LIBUARTE_PERIPHERAL_NOT_USED, NRF_LIBUARTE_PERIPHERAL_NOT_USED, 127, 3 );
NRF_QUEUE_DEF( COM_BUFFER, xTxBufferQueueCOM0, 128, NRF_QUEUE_MODE_NO_OVERFLOW );

/* Define the UART structure for COM1. 
		UARTE instance:																		1
		TIMER instance used by libuarte for bytes counting (timer0 is used for the SD):		2
		RTC instance used for timeout:														NRF_LIBUARTE_PERIPHERAL_NOT_USED
		TIMER instance used for timeout:													NRF_LIBUARTE_PERIPHERAL_NOT_USED
		Size of single RX buffer:															15
		Number of buffers in the RX buffer pool:											3								 
   As there is no HW ressource defined for timeout, an app timer is used. */
NRF_LIBUARTE_ASYNC_DEFINE( xLibUarteCOM1, 1, 2, NRF_LIBUARTE_PERIPHERAL_NOT_USED, NRF_LIBUARTE_PERIPHERAL_NOT_USED, 15, 3 );
NRF_QUEUE_DEF( COM_BUFFER, xTxBufferQueueCOM1, 63, NRF_QUEUE_MODE_NO_OVERFLOW );
/*-----------------------------------------------------------*/


/* Initialise the UART driver: create TX queue and RX semaphore. Done only once.
 
   The UART HW itself will only be initialised when opening the UART port.
*/
void vUartPortInit( void )
{
	unsigned portBASE_TYPE	uxID;
	
	/* Initialise all UART HW instances. */
	/* Initialise COM structures for all UARTs. */
	for ( uxID = 0; uxID < NUM_COMS; uxID++ )
	{
		/* Enter identifier. */
		xCom[ uxID ].uxUartPortID = uxID;

		/* Create a mutex to protect access to the ADC so that it can be used from different tasks. */
		xCom[ uxID ].xMutexUart = xSemaphoreCreateMutex();	

		/* Create the receive queue. */
		xCom[ uxID ].xUartRxCountingSemaphore = xSemaphoreCreateCounting( 7, 0 );	/* max. count to 7, 0 initially. */
		
		xCom[ uxID ].bOpen = false;
		xCom[ uxID ].bRequestClose = false;
	}

	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* Structure duplicated from app_timer_freertos.c to allow for a clean work-around for the p_xLibUarteCOM0_app_timer_data->active bug. */
/**@brief This structure keeps information about osTimer.*/
typedef struct
{
    void                      * argument;
    TimerHandle_t               osHandle;
    app_timer_timeout_handler_t func;
    /**
     * This member is to make sure that timer function is only called if timer is running.
     * FreeRTOS may have timer running even after stop function is called,
     * because it processes commands in Timer task and stopping function only puts command into the queue. */
    bool                        active;
    bool                        single_shot;
} app_timer_info_t;

/* Open COM port. 

   The association of a physical UART with a logical COM port is done here.

   Input parameters:
		xComID					ID of the COM port to be opened.
		pcUartRxBuffer			Pointer to RX character ring buffer.
		uiUartRxBufLenM1		Length of the RX buffer - 1.
		xMode					Operating mode (ASCII/binary).
*/
void vCOMOpen( unsigned portBASE_TYPE xComID, signed char *pcUartRxBuffer, unsigned portBASE_TYPE uxUartRxBufLenM1, enum xCOM_MODE xMode )
{
	void						( *pvUartEvtHandler )();
    ret_code_t					xErrCode;
    nrf_libuarte_async_t		*pxLibUarte;
	nrf_queue_t					*pxTxBufferQueue;
	nrf_libuarte_async_config_t nrf_libuarte_async_config;
	
	/* Initialise the COM port structure. */
	/* Initialise pointer to Rx ring buffer. */
	xCom[ xComID ].pcUartRxBuffer = pcUartRxBuffer;
	
	/* Initialise Rx ring buffer read/write pointers. */
	xCom[ xComID ].uiUartRxWrIdx = 0;
	xCom[ xComID ].uiUartRxRdIdx = 0;
	xCom[ xComID ].uiUartRxBufLenM1 = uxUartRxBufLenM1;
	xCom[ xComID ].uiRxStrgLen = 0;

	/* Set the COM port mode. */
	xCom[ xComID ].xMode = xMode;

	/* Avoid initialising a LibUARTe driver of an already open port. This results in a crash. */
	if ( !xCom[ xComID ].bOpen )
	{
		switch( xComID )
		{
			case COM0:		nrf_libuarte_async_config.tx_pin     = SENSOR_TXD;			/* COM port to sensor chip. */	
							nrf_libuarte_async_config.rx_pin     = SENSOR_RXD;
							nrf_libuarte_async_config.baudrate   = NRF_UARTE_BAUDRATE_4800;
							nrf_libuarte_async_config.parity     = NRF_UARTE_PARITY_INCLUDED;
							nrf_libuarte_async_config.hwfc       = NRF_UARTE_HWFC_DISABLED;
							nrf_libuarte_async_config.timeout_us = NRF_UARTE_RX_TIMEOUT;
							nrf_libuarte_async_config.int_prio   = APP_IRQ_PRIORITY_LOW_MID;

							pxLibUarte = ( nrf_libuarte_async_t * )&xLibUarteCOM0;
							pxTxBufferQueue = ( nrf_queue_t * )&xTxBufferQueueCOM0;
							break;

			case COM1:		nrf_libuarte_async_config.tx_pin     = TRC_TXD;				/* COM port for trace output. */
							nrf_libuarte_async_config.rx_pin     = TRC_RXD;
							nrf_libuarte_async_config.baudrate   = NRF_UARTE_BAUDRATE_115200;
							nrf_libuarte_async_config.parity     = NRF_UARTE_PARITY_EXCLUDED;
							nrf_libuarte_async_config.hwfc       = NRF_UARTE_HWFC_DISABLED;
							nrf_libuarte_async_config.timeout_us = NRF_UARTE_RX_TIMEOUT;
							nrf_libuarte_async_config.int_prio   = APP_IRQ_PRIORITY_LOW_MID;

							pxLibUarte = ( nrf_libuarte_async_t * )&xLibUarteCOM1;
							pxTxBufferQueue = ( nrf_queue_t * )&xTxBufferQueueCOM1;
							break;

			default:		break;
		}
		xCom[ xComID ].pxLibUarte = pxLibUarte;
		xCom[ xComID ].pxTxBufferQueue = pxTxBufferQueue;
		xCom[ xComID ].bTxBusy = false;

		/* Work-around for incompatibility between app_timer and FreeRTOS app_timer_freertos. */
		app_timer_info_t *p_xLibUarteCOM_app_timer_data = ( app_timer_info_t * )*( xCom[ xComID ].pxLibUarte->p_app_timer );
		p_xLibUarteCOM_app_timer_data->active = 0;
		
		xErrCode = nrf_libuarte_async_init( xCom[ xComID ].pxLibUarte, &nrf_libuarte_async_config, vUartEventHandler, ( void * )pxLibUarte );
		APP_ERROR_CHECK( xErrCode );
		
		/* Enable the interface driver. */
		nrf_libuarte_async_enable( xCom[ xComID ].pxLibUarte );	
	}
	
	xCom[ xComID ].bOpen = true;
}
/*-----------------------------------------------------------*/

/* Close UART port. */
void vCOMClose( unsigned portBASE_TYPE xComID )
{
	/* Check if the UART must be shutdown from the interrupt handler. 
	   The check has to be done in a critical section. */
	portENTER_CRITICAL();
	if ( xCom[ xComID ].bTxBusy )
	{
		/* There is an ongoing transmission. Request the TX IRQ gandler to shut down the UART as soon as the transmission has been terminated. */
		xCom[ xComID ].bRequestClose = true;
	}
	portEXIT_CRITICAL();
	   
	if (    !xCom[ xComID ].bTxBusy
		 && xCom[ xComID ].bOpen 
	   )
	{
		/* There is not any TX going on. 
		   Disable LibUARTE immediately. The TXD signal is freed and can be pulled low externally, if required. */
		nrf_libuarte_async_uninit( xCom[ xComID ].pxLibUarte );
		
		/* HW fix to power-cycle the UARTE. This may reduce the power consumption, ref. to:
		   https://devzone.nordicsemi.com/f/nordic-q-a/26030/how-to-reach-nrf52840-uarte-current-supply-specification/102605#102605 */
		if ( xComID == COM0 )
		{
			*( volatile uint32_t * )UARTE0_SECRET_REG = 0;
			*( volatile uint32_t * )UARTE0_SECRET_REG = 1;
		}
		else
		{
			*( volatile uint32_t * )UARTE1_SECRET_REG = 0;
			*( volatile uint32_t * )UARTE1_SECRET_REG = 1;
		}
	
		xCom[ xComID ].bOpen = false;
	}
}
/*-----------------------------------------------------------*/

/* Return the 'open' state of a COM port. */
bool bCOMIsOpen( unsigned portBASE_TYPE xComID )
{
	return xCom[ xComID ].bOpen;
}
/*-----------------------------------------------------------*/

/* Store one character in the RX ring buffer. The buffer must be able to
   contain at least the character and one end-of-string marker (0x00) to
   make sure that it does not contain empty strings.
   Return FALSE when buffer is full, else TRUE.
   

   Non-zero (i.e. end-of-string) characters are stored until the second to last position.
   The last position contains then a 0x00 character.
   
   The buffer must be able to contain at least the character and one end-of-string marker (0x00) to
   make sure that it does not contain empty strings.
   
   The modulo operation with the buffer length is very critical. First of all, the buffer length must
   be a power of two. Next, the compiler needs to be convinced that it actually is a power of two
   - which requires to use a constant and not a variable / function parameter.
   
   Return COM_RXBUF_TRUNCATED when buffer when the current string is terminated because of buffer full, 
   COM_RXBUF_FULL if the buffer is full, else COM_RXBUF_SUCCESS.
*/
enum COM_RXBUF_STORE xStoreRxCharInBuffer( struct COM_PORT *pxCom, signed char cChar )
{
	unsigned portBASE_TYPE			uxUartRxWrIdx_plus1;
	
	uxUartRxWrIdx_plus1 = ( pxCom->uiUartRxWrIdx + 1 ) & pxCom->uiUartRxBufLenM1;

	if ( ( ( pxCom->uiUartRxWrIdx + 2 ) & pxCom->uiUartRxBufLenM1 ) == pxCom->uiUartRxRdIdx )
	{
		/* We reached the second to last position. Store an end-of-string character instead of the 
		   received one. The user of the interface must make sure it has got all the necessary 
		   characters. */
		*( pxCom->pcUartRxBuffer + pxCom->uiUartRxWrIdx ) = 0;
		pxCom->uiUartRxWrIdx = uxUartRxWrIdx_plus1;
		
		return COM_RXBUF_TRUNCATED;
	}
	
	if ( uxUartRxWrIdx_plus1 == pxCom->uiUartRxRdIdx )
	{
		/* Buffer is full. Do not store any new characters. */
		return COM_RXBUF_FULL;
	}	
	
	/* Store character in buffer and increment write pointer. */
	*( pxCom->pcUartRxBuffer + pxCom->uiUartRxWrIdx ) = cChar;
	pxCom->uiUartRxWrIdx = uxUartRxWrIdx_plus1;
	
	return COM_RXBUF_SUCCESS;	
}
/*-----------------------------------------------------------*/

/* Get one character from the RX ring buffer. 
   Return FALSE if the buffer is empty, else TRUE.
*/
bool bGetRxCharFromBuffer( unsigned portBASE_TYPE xComID, signed char *pcChar )
{
	unsigned portBASE_TYPE		uxRxBufIdx;
	
	uxRxBufIdx = xCom[ xComID ].uiUartRxRdIdx;
	
	if ( uxRxBufIdx == xCom[ xComID ].uiUartRxWrIdx )
	{
		/* Buffer is empty. */
		return false;
	}
	else
	{
		*pcChar = *( xCom[ xComID ].pcUartRxBuffer + uxRxBufIdx );
		xCom[ xComID ].uiUartRxRdIdx = ( uxRxBufIdx + 1 ) & xCom[ xComID ].uiUartRxBufLenM1;
	}
		
	return true;
}
/*-----------------------------------------------------------*/

/* Get one character from the RX ring buffer with index. The index may be positive or negative.
   The read pointer is not updated.
   Does not check buffer status!
*/
signed char cGetRxCharFromBufferWithIndex( unsigned portBASE_TYPE xComID, uint32_t uiIdx )
{
	unsigned portBASE_TYPE			uxRxBufIdx;
	
	uxRxBufIdx = xCom[ xComID ].uiUartRxRdIdx;
	if ( uxRxBufIdx + uiIdx < 0 )
	{
		uxRxBufIdx = uxRxBufIdx + uiIdx + xCom[ xComID ].uiUartRxBufLenM1 + 1;
	}
	else
	{
		uxRxBufIdx = ( uxRxBufIdx + uiIdx ) & xCom[ xComID ].uiUartRxBufLenM1;
	}

	return *( xCom[ xComID ].pcUartRxBuffer + uxRxBufIdx );
}
/*-----------------------------------------------------------*/

/* Get pointer to character in the RX ring buffer from index. */
signed char *cGetPointerToBufferWithIndex( unsigned portBASE_TYPE xComID, uint32_t uiIdx )
{
	unsigned portBASE_TYPE			uxRxBufIdx;
	
	uxRxBufIdx = xCom[ xComID ].uiUartRxRdIdx;	
	uxRxBufIdx = ( uxRxBufIdx + uiIdx ) & xCom[ xComID ].uiUartRxBufLenM1;
	
	return xCom[ xComID ].pcUartRxBuffer + uxRxBufIdx;	
}
/*-----------------------------------------------------------*/

/* Wait for the counting semaphore to be given by the RX ISR which indicated that a complete 
   string has been received.
   Return false if no complete string is available or arrives before xBlockTime expires. 
*/
bool bComReceiveString( unsigned portBASE_TYPE xComID, TickType_t xBlockTime )
{
	return xSemaphoreTake( xCom[ xComID ].xUartRxCountingSemaphore, xBlockTime );
}
/*-----------------------------------------------------------*/

/* Remove the next string from the RX ring buffer. */
void vUartRemoveString( unsigned portBASE_TYPE xComID )
{
	signed char				cRxChar;
	bool				 	bBufferFilled ;
	
	/* Remove the next string from the RX ring buffer. */
	do
	{
		bBufferFilled = bGetRxCharFromBuffer( xComID, &cRxChar );
	} while ( ( bBufferFilled ) && ( cRxChar != 0 ) );
}
/*-----------------------------------------------------------*/

/* Get the number of characters (including end-of-string, 0x00) currently present in the 
   RX ring buffer.
*/
uint32_t uiGetNumberOfCharsInBuffer( unsigned portBASE_TYPE xComID )
{
	if ( xCom[ xComID ].uiUartRxRdIdx <= xCom[ xComID ].uiUartRxWrIdx )
	{
		/* Write index is ahead of read index. */
		return xCom[ xComID ].uiUartRxWrIdx - xCom[ xComID ].uiUartRxRdIdx;
	}
	else
	{
		/* Read index is ahead of write index: buffer has wrapped around. */
		return xCom[ xComID ].uiUartRxBufLenM1 + 1 + xCom[ xComID ].uiUartRxWrIdx - xCom[ xComID ].uiUartRxRdIdx;
	}
}
/*-----------------------------------------------------------*/

/* Get the number of strings currently present in the RX ring buffer. */
uint32_t uiGetNumberOfStringsInBuffer( unsigned portBASE_TYPE xComID )
{
	uint32_t 	uiRdIdx;
	uint32_t 	uiNumStrgs;
	
	uiNumStrgs = 0;
	
	for ( uiRdIdx = 0; uiRdIdx < uiGetNumberOfCharsInBuffer( xComID ); uiRdIdx++ )
	{
		/* Check, if the character pointed to by the current index is end-of-string. */
		if ( cGetRxCharFromBufferWithIndex( xComID, uiRdIdx ) == 0 )
		{
			uiNumStrgs++;
		}
	}
	
	return uiNumStrgs;
}
/*-----------------------------------------------------------*/

/* Flush the RX ring buffer. */
void vFlushRxBuffer( unsigned portBASE_TYPE xComID )
{
	xCom[ xComID ].uiUartRxRdIdx = 0;
	xCom[ xComID ].uiUartRxWrIdx = 0;
	xCom[ xComID ].uiRxStrgLen = 0;
}
/*-----------------------------------------------------------*/

/* Send the string pointed to by pcTxString. 
   If the interface is busy (i.e. currently transmitting), push a pointer to the buffer in the TX SDK queue.
   The queue is managed in the TX_DONE ISR. 
   The data to transmit is copied to a dynamically allocated memory block. 
*/
ret_code_t xComSendString( unsigned portBASE_TYPE xComID, const signed char *pcTxString )
{
    ret_code_t 				xErrCode;
	signed char 			*pcTxBuffer;
	unsigned portBASE_TYPE	uxTxStringLen;
	bool					bNotDone;
	TickType_t				xStartTime;
	
	if ( ( pcTxString == NULL ) || ( strlen( pcTxString ) == 0 ) )
	{
		return NRF_SUCCESS;
	}
	
	/* Copy the string to send to a newly allocated memory buffer. 
	   There are several options for the required dynamic memory allocation:
			1. libc malloc(): Not thread-safe so not suitable here. Also requires dedicated heap other than FreeRTOS' heap which is not used 
			   by any other function (arm_linker_heap_size="1024"). Not an option.
			2. FreeRTOS memory management: requires heap_4.c because of variable block sizes. Nordic's adaptation of FreeRTOS leads to 
			   an assert during task creation as xCreateTask() calls pvPortMalloc which in turn calls vPortEnterCritical. That function checks 
			   uxCriticalNesting vs. interrupt state as reported by SCB, decides that the call is in interrupt context and asserts.
				--> CHOSEN SOLUTION
			3. Use the memory manager (nrf_mem_init(), nrf_malloc() ) provided in the SDK which allocates memory from the common pool.
			   Not ideal either as the memory manager only works with classes of fixed-sized blocks.
			4. Do not use any dynamic memory allocation at all and let the sending task wait for the TX buffer to become free. There is one 
			   TX buffer per COM structure. Also quite suboptimal.
			   
	   The SDK queue management does not take care of the actual data buffer, it only takes pointers to the buffer and the buffer size.
	   The actual buffer needs to be allocated and freed using the dynammic memory management.
	   
	   The difficulty here is to know whether to send the buffer directly or to push it in the queue. Between checking that state in the
	   foreground and reacting the state might change. The state change happens in the UART interrupt service routine which can interrupt 
	   the foreground process at any time. So there is a risk of a race condition. This function needs to make sure that a state change
	   cannot happen by cleverly using a critical section. 
	   
	   So the overall sending process works as follows:
		- Allocate a data buffer using pvMalloc().
		- Copy the data to transmit there. This allows the caller to reuse its buffer immediately.
		- Check the status of the UART. If it is idle (not transmitting anything, i.e. !bTxBusy):
			- Send the buffer directly using nrf_libuarte_async_tx().
		  else:
			- Push a transmission order (pointer to the data buffer and its length) the transmission queue using nrf_queue_push().
		- The UART interrupts and calls the vUartEventHandler() with NRF_LIBUARTE_ASYNC_EVT_TX_DONE.
		- There, the pointer to the data buffer is retrieved and the allocated buffer is freed using vPortFree().	   */

	uxTxStringLen = strlen( pcTxString );
	
	/* Monitor the time it takes to finish pushing the string. If it takes too long because of unavailable buffers,
	   assert. */
	xStartTime = xTaskGetTickCount();
	
	/* In any case, we need to allocate a memory block for the string to transmit. The copy the string there. */
	bNotDone = true;
	while ( bNotDone )
	{
		/* The call to pvPortMalloc() must be in a critical section as the corresponding call to vPortFree() is in an
		   interrupt routine.
		   Caution: The total string length is that of all the characters and the end '0'. The end '0', however, is not transmitted 
		   but stored in the TX buffer. */
		portENTER_CRITICAL();
		pcTxBuffer = pvPortMalloc( uxTxStringLen + 1 );
		portEXIT_CRITICAL();
		if ( pcTxBuffer == NULL )
		{
			/* Busy-wait until a buffer becomes free. */
			configASSERT( xTaskGetTickCount() - xStartTime < UART_TX_TIMEOUT );
		
			#if !defined ( NO_SOFTDEVICE )
				( void )sd_app_evt_wait();
			#endif
			vTaskDelay( 1 );		
		}
		else
		{
			bNotDone = false;
		}
	}
	
	NRF_LOG_DEBUG( "alloc %8.8x, %i", pcTxBuffer, uxTxStringLen );
	NRF_LOG_FLUSH();
	
	/* Allocation was successful, now copy the data. */
	if ( ( uint32_t )pcTxString > RAM_START )
	{
		/* Copy from RAM. */
		strcpy( pcTxBuffer, ( signed char * )pcTxString );
	}
	else
	{
		/* Copy from Flash. */
		strcpy( pcTxBuffer, ( const signed char * )pcTxString );
	}

	/* Lock access to the UART only if the scheduler is already running. Else, simply skip this step.
	   The scheduler might not be running in the app_error_fault_handler(), if an early error during the
	   initialisation occurred. */
	if ( xCom[ xComID ].xMutexUart != 0 )
	{
		xSemaphoreTake( xCom[ xComID ].xMutexUart, portMAX_DELAY );
	}
	
	/* If there is a transmission ongoing, push the string to be transmitted into the transmission queue. 
	   Else, transmit directly.
	   CAUTION: There is the risk of race conditions between the process in which xComSendString runs the and UART interrupt service 
	            routine. Hence the critical section. */
	bNotDone = true;
	
	while ( bNotDone )
	{
		portENTER_CRITICAL();
		if ( !xCom[ xComID ].bTxBusy )
		{
			/* Send cTxBuffer directly. nrf_libuarte_async_tx() is not even allowed to return NRF_ERROR_BUSY since we checked bTxBusy 
			   before. */
			xErrCode = nrf_libuarte_async_tx( xCom[ xComID ].pxLibUarte, pcTxBuffer, uxTxStringLen );
			xCom[ xComID ].bTxBusy = true;
			portEXIT_CRITICAL();
			
			APP_ERROR_CHECK( xErrCode );

			bNotDone = false;
		}
		else
		{
			COM_BUFFER 		xTxBuffer = {
											.pcData   = pcTxBuffer,
											.uxLength = uxTxStringLen
										};
			
			/* The transmitter is currently busy. Push the transmission order into the transmission queue. */
			xErrCode = nrf_queue_push( xCom[ xComID ].pxTxBufferQueue, &xTxBuffer );
			if ( xErrCode == NRF_ERROR_NO_MEM )
			{
				/* Busy-wait until the queue becomes available. In the meantime, allow for scheduling the
				   SD and other tasks. 
				   Since we have to exit from the critical section here to do the busy-wait, we need to re-check
				   bTxBusy again after the delay since not only the transmission queue might have become available 
				   but also the last bit of data might have finished transmitting and the transmitter is now idle.
				   Only doing the buffering in that case would mean that the data would be queued but not transmitted.
				   It would then be transmitted *after* the next directly triggered transmission, i.e. out-of-sequence. */
				portEXIT_CRITICAL();
				configASSERT( xTaskGetTickCount() - xStartTime < UART_TX_TIMEOUT );
				
				#if !defined ( NO_SOFTDEVICE )
					( void )sd_app_evt_wait();
				#endif
				vTaskDelay( 1 );
			}
			else
			{
				portEXIT_CRITICAL();
				
				APP_ERROR_CHECK( xErrCode );	
				
				bNotDone = false;
			}
		} 
	}
	
	if ( xCom[ xComID ].xMutexUart != 0 )
	{
		xSemaphoreGive( xCom[ xComID ].xMutexUart );
	}

	return xErrCode;
}
/*-----------------------------------------------------------*/

/* Send the string pointed to by pcTxString character by character, interrupt-safe. 
   Caution: The function waits for the UART mutex. This might lead to deadlocks! */
ret_code_t xComSendStringIRQ( unsigned portBASE_TYPE xComID, const signed char *pcTxString )
{
	return xComSendString( xComID, pcTxString );
}
/*-----------------------------------------------------------*/

/* Send one character to the UART. */
ret_code_t xComSendChar( unsigned portBASE_TYPE xComID, signed char cChar )
{
	signed char				cTxString[ 2 ];
    ret_code_t 				xErrCode;
	
	cTxString[ 0 ] = cChar;
	cTxString[ 1 ] = 0;
	
	xErrCode = xComSendString( xComID, cTxString );

	return xErrCode;
}
/*-----------------------------------------------------------*/


/* Generic RX interrupt routine. */
void vUartRxISR( nrf_libuarte_async_evt_t * p_evt, struct COM_PORT *pxCom )
{
	signed char				cChar;
	uint32_t				uiRxCharCnt;
	signed portBASE_TYPE 	xHigherPriorityTaskWoken = pdFALSE;

	/* Get the character, store it in the RX ring buffer and give the RX semaphore for the Parser task.
	If the semaphore causes the Parser task to wake, force a context switch as the woken task
	may have a higher priority than the task we have interrupted. */
	for ( uiRxCharCnt = 0; uiRxCharCnt < p_evt->data.rxtx.length; uiRxCharCnt++ )
	{
		cChar = *(p_evt->data.rxtx.p_data + uiRxCharCnt ); 
	
		if ( pxCom->xMode == COM_ASCII )
		{
			/* In ASCII mode, check if a LF (0x0a, '\n') has been received. In this case, add a '0' to the buffer and indicate that 
			   a string has been received by setting the global flag. 
			   Suppress CR characters (0x0d, '\r'), 0x1a and 0x00. Store all other characters in the RX ring buffer. */
			if ( ( cChar == '\n' ) || ( cChar == '\r' ) )
			{
				/* Only pass a string to the Parser, if it contains any characters at all. So sequences of CRLF only will never
				   trigger AT parsing. */
				if ( pxCom->uiRxStrgLen != 0 )
				{
					/* Try to store the character in the RX buffer and evaluate the result. The three return values and the consequent use of a'case' construct here 
					   and in the code below is not the most elegant solution but it seems to do the job. */
					switch ( xStoreRxCharInBuffer( pxCom, 0 ) )
					{
						case COM_RXBUF_SUCCESS:					/* Nothing special: store was OK. Inform the Parser. */
						case COM_RXBUF_TRUNCATED:				/* Nothing special either: We wanted to store a 0x00 and did it, even if it was the last location in the buffer. */
													xSemaphoreGiveFromISR( pxCom->xUartRxCountingSemaphore, &xHigherPriorityTaskWoken );
													pxCom->uiRxStrgLen = 0;
													break;
						case COM_RXBUF_FULL:		break;		/* Ignore storing the end-of-string. The string has already been truncated before. */
					}
				}
			}
			else if ( ( cChar != 0 ) && ( cChar != 0x1a ) )
			{
				/* Any other non-special character: store it in the buffer. */
				switch ( xStoreRxCharInBuffer( pxCom, cChar ) )
				{
					case COM_RXBUF_SUCCESS:					/* Nothing special: store was OK. */
												pxCom->uiRxStrgLen++;				
												break;
					case COM_RXBUF_TRUNCATED:				/* String has been truncated. Nevertheless, inform the Parser about the new string. */
												xSemaphoreGiveFromISR( pxCom->xUartRxCountingSemaphore, &xHigherPriorityTaskWoken );
												pxCom->uiRxStrgLen = 0;
												break;
					case COM_RXBUF_FULL:		break;		/* Ignore the character. */
				}
			}
		}
		else
		{
			/* Binary mode: Store any character in the buffer. */
			if ( xStoreRxCharInBuffer( pxCom, cChar ) == COM_RXBUF_SUCCESS )
			{
				pxCom->uiRxStrgLen++;				
			}
		}
	}
}
/*-----------------------------------------------------------*/

/* UART event handler, one per UART instance. */
static void vUartEventHandler( void * pvContext, nrf_libuarte_async_evt_t * pxEvt )
{
    nrf_libuarte_async_t 	*pxLibUarte = ( nrf_libuarte_async_t * )pvContext;
	struct COM_PORT			*pxCom;
	unsigned portBASE_TYPE	uxID;
	size_t					xBlockAllocatedBit;
    ret_code_t 				xErrCode;

	/* First, identfy which COM-port structure is associated with this interrupt. The nRF SDK only provides the context,
	   so walk through all possible COM port structures to see which one matches. */
	for ( uxID = 0; ( uxID < NUM_COMS ) && ( pxLibUarte != xCom[ uxID ].pxLibUarte ); uxID++ )
	{
		;
	}

	if ( uxID == NUM_COMS )
	{
		/* Not found: return without doing anything. */
		return;
	}
	pxCom = &xCom[ uxID ];

    switch (pxEvt->type)
    {
        case NRF_LIBUARTE_ASYNC_EVT_ERROR:
            break;

        case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:
			/* A data buffer has been received. Treat the data buffer. */
			( void )vUartRxISR( pxEvt, pxCom );
			
			/* Now release the data buffer. */
			nrf_libuarte_async_rx_free( pxLibUarte, pxEvt->data.rxtx.p_data, pxEvt->data.rxtx.length );
            break;

        case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:
			/* The transmission of a buffer is finished.
			   Free the currently used TX data buffer. 
			   It seems that sometimes a buffer to free is not allocated. In this case only 
			   flag an error but continue execution. */
			/* xBlockAllocatedBit set to the top bit of an size_t type. When this bit in the xBlockSize
			   member of an BlockLink_t structure is set then the block belongs to the application.  
			   When the bit is free the block is still part of the free heap space. */
			xBlockAllocatedBit = ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * 8 ) - 1 );			/* 8 bits per byte */
			if ( ( *( size_t * )( pxEvt->data.rxtx.p_data - sizeof( size_t ) ) & xBlockAllocatedBit  ) != 0 )
			{
				vPortFree( pxEvt->data.rxtx.p_data );
			}
			else
			{
				NRF_LOG_ERROR( "Error: Memory block to free is not marked as allocated @0x%8.8x", pxEvt->data.rxtx.p_data );
			}
			
			NRF_LOG_DEBUG( "free %8.8x, %i", pxEvt->data.rxtx.p_data, pxEvt->data.rxtx.length );
			NRF_LOG_FLUSH();

			/* Check, if there is another buffer queued for transmission. */
			if ( !nrf_queue_is_empty( pxCom->pxTxBufferQueue ) )
			{
				COM_BUFFER		xTxBuffer;
				
				/* Get the pointers to the next buffer to be transmitted from the queue and feed it to libuarte. */
				xErrCode = nrf_queue_pop( pxCom->pxTxBufferQueue, &xTxBuffer );
				APP_ERROR_CHECK( xErrCode );
				
				/* Start transmitting this buffer. */
				( void )nrf_libuarte_async_tx( pxLibUarte, xTxBuffer.pcData, xTxBuffer.uxLength );
			}
			else
			{
				/* Nothing to do anymore. */
				pxCom->bTxBusy = false;				

				/* Check, if a UART close has been requested. If yes, disable LibUARTE. */				
				if ( pxCom->bRequestClose )
				{
					nrf_libuarte_async_uninit( pxCom->pxLibUarte );			
					pxCom->bRequestClose = false;
					pxCom->bOpen = false;
				}
			}
            break;

        default:
            break;
    }
}
/*-----------------------------------------------------------*/
