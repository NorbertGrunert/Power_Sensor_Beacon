/*
 * Tracker Bootloader
 *
 * UART driver header file
 *
 */
 
#ifndef DRV_UART_H
#define DRV_UART_H

#include "tracker.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

/* nRF SDK files. */
#include "nrf_error.h"
#include "nrf_libuarte_async.h"

/* Standard include files. */
#include <stdbool.h>
#include "tracker.h"
/*-----------------------------------------------------------*/

/* Definitions. */
#define NUM_COMS				2							/* Number of simultaneously supported COM ports. */
#define NUM_UARTS				2							/* Number of UARTs in the HW. */

#define OP_QUEUES_SIZE          3
#define APP_TIMER_PRESCALER     NRF_SERIAL_APP_TIMER_PRESCALER

#define NRF_UARTE_RX_TIMEOUT	(  500 )					/* Timeout (us) for the UARTE without receiving data before sending an RX IRQ. */
#define UART_TX_TIMEOUT			( 1 * portTICKS_PER_SEC )	/* Timeout (s) for the UARTE freeing any buffers for transmission. */

#define UARTE0_SECRET_REG		0x40002FFC					/* UARTE0 secret register to power-cycle the UART. */
#define UARTE1_SECRET_REG		0x40028FFC					/* UARTE1 secret register to power-cycle the UART. */
/*-----------------------------------------------------------*/

/* IDs for the supported UARTs. The nRF52840 only supports two UARTs. */
enum COM_ID 
{
	COM0,
	COM1
};

/* Operating mode of the COM port. */
enum xCOM_MODE
{
	COM_ASCII,
	COM_BINARY
};

/* Result of storing a character in the RX buffer. */
enum COM_RXBUF_STORE
{
	COM_RXBUF_SUCCESS,
	COM_RXBUF_TRUNCATED,
	COM_RXBUF_FULL
};

/* COM port structure. */
struct COM_PORT
{
	unsigned portBASE_TYPE		uxUartPortID;				/* COM port ID. */ 
	nrf_libuarte_async_t		*pxLibUarte;				/* Pointer to nRF libUARTE instance. */
	SemaphoreHandle_t			*xMutexUart;				/* Mutex handle. */
	nrf_queue_t					*pxTxBufferQueue;			/* Pointer to the nRF queue used as TX buffer. */
	bool						bTxBusy;					/* Flag indicating that a transmission is ongoing. */
	SemaphoreHandle_t 			xUartRxCountingSemaphore;	/* RX counting semaphore - received complete AT string terminated by 0x0a. */
	
	signed char					*pcUartRxBuffer;			/* Pointer to RX ring buffer, size of the buffer is a power of 2. 
																		Empty: RdPtr == WrPtr
																		Full: WrPtr == RdPtr - 1 */
	uint32_t					uiUartRxBufLenM1;			/* Size of the RX ring buffer - 1. */
	uint32_t					uiUartRxWrIdx;				/* Index into the RX ring buffer used by the RX ISR to store received characters. 
															   points to the next free location, except if the buffer is full. In that case, the 
															   last character contains a string end 0x0. */
	uint32_t					uiUartRxRdIdx;				/* Index into the RX ring buffer used by the get string function. */
	uint32_t					uiRxStrgLen;				/* Length of the currently received string; only used to identify empty strings. */
	enum xCOM_MODE				xMode;						/* ASCII or binary mode. */
	bool						bOpen;						/* Boolean telling if the COMport is open (i.e. the LibUARTe driver is initialised). */
	bool						bRequestClose;				/* Boolean requesting the IRQ handler to close the COM port as soon as all data have been transmitted. */
};

typedef struct
{
    signed char					*pcData;	
    unsigned portBASE_TYPE		uxLength;
} COM_BUFFER;
/*-----------------------------------------------------------*/

/* Public function prototypes. */
extern void			 			vUartPortInit( void );
extern void						vCOMOpen( unsigned portBASE_TYPE xComID, signed char *pcUartRxBuffer, unsigned portBASE_TYPE uxUartRxBufLenM1, enum xCOM_MODE xMode );
extern void 					vCOMClose( unsigned portBASE_TYPE xComID );
extern bool 					bCOMIsOpen( unsigned portBASE_TYPE xComID );
extern bool					 	bStoreRxCharInBuffer( struct COM_PORT *pxCom, signed char cChar );
extern bool						bGetRxCharFromBuffer( unsigned portBASE_TYPE xComID, signed char *pcChar );
extern signed char				cGetRxCharFromBufferWithIndex( unsigned portBASE_TYPE xComID, uint32_t uiIdx );
extern signed char 				*cGetPointerToBufferWithIndex( unsigned portBASE_TYPE xComID, uint32_t uiIdx );
extern bool						bComReceiveString( unsigned portBASE_TYPE xComID, TickType_t xBlockTime );
extern void 					vUartRemoveString( unsigned portBASE_TYPE xComID );
extern uint32_t				 	uiGetNumberOfCharsInBuffer( unsigned portBASE_TYPE xComID );
extern uint32_t				 	uiGetNumberOfStringsInBuffer( unsigned portBASE_TYPE xComID );
extern void 					vFlushRxBuffer( unsigned portBASE_TYPE xComID );
extern ret_code_t				xComSendChar( unsigned portBASE_TYPE xComID, signed char cChar  );
extern ret_code_t				xComSendString( unsigned portBASE_TYPE xComID, const signed char *pcTxString );
extern ret_code_t				xComSendStringIRQ( unsigned portBASE_TYPE xComID, const signed char *pcTxString );

extern void UartInit( void );
extern void UartEventHandler( void * context, nrf_libuarte_async_evt_t * p_evt );
/*-----------------------------------------------------------*/
#endif