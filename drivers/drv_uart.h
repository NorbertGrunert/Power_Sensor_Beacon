/*
 * Tracker Firmware
 *
 * UART driver header file
 *
 */
 
#ifndef DRV_UART_H
#define DRV_UART_H

/* Standard include files. */
#include <stdbool.h>

 /* Scheduler include files. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "nrf_serial.h"


/* Definitions. */
#define NUM_COMS				3							/* Number of simultaneously supported COM ports. */
#define NUM_UARTS				2							/* Number of UARTs in the HW. */

#define OP_QUEUES_SIZE          3
#define APP_TIMER_PRESCALER     NRF_SERIAL_APP_TIMER_PRESCALER

#define uartRX_BUFFER_SIZE		350							/* size in items: max. length of one AT frame + max. number of rx-chars. during one tick */
#define uartTX_BLOCKTIME		( ( TickType_t ) ( 50 / portTICK_RATE_MS ) )	/* wait max. 50ms for TX queue to become available. One character takes about 2ms to send @4800bps. */


/* IDs for the supported UARTs. */
enum COM_ID 
{
	COM0,
	COM1,
	COM2
};


/* COM port structure. */
struct COM_PORT
{
	unsigned portBASE_TYPE		uxUartPortID;				/* COM port ID. */ 
	nrf_serial_t const			*pxUartHWInst;				/* Pointer to UART HW instance. */
//	QueueHandle_t 				xUartTxQueue; 				/* TX queue handle */
	SemaphoreHandle_t 			xUartRxCountingSemaphore;	/* RX counting semaphore - received complete AT string terminated by 0x0a. */
	
	signed char					*pcUartRxBuffer;			/* Pointer to RX ring buffer, size of the buffer is a power of 2. 
																		Empty: RdPtr == WrPtr
																		Full: WrPtr == RdPtr - 1 */
	unsigned portBASE_TYPE		uxUartRxWrIdx;				/* Index into the RX ring buffer used by the RX ISR to store received characters. 
															   points to the next free location, except if the buffer is full. In that case, the 
															   last character contains a string end 0x0. */
	unsigned portBASE_TYPE		uxUartRxRdIdx;				/* Index into the RX ring buffer used by the get string function. */
	unsigned portBASE_TYPE		uxRxStrgLen;				/* Length of the currently received string; only used to identify empty strings. */
};


/* Public function prototypes. */
extern void			 			vUartPortInit( void );
extern void						vCOMOpen( unsigned portBASE_TYPE xComID, signed char *pcUartRxBuffer );
extern void 					vCOMClose( unsigned portBASE_TYPE xComID );
extern signed portBASE_TYPE 	xStoreRxCharInBuffer( struct COM_PORT *pxCom, signed char cChar );
extern signed portBASE_TYPE 	xGetRxCharFromBuffer( unsigned portBASE_TYPE xComID, signed char *pcChar );
extern signed char				cGetRxCharFromBufferWithIndex( unsigned portBASE_TYPE xComID, unsigned char ucIdx );
extern signed char 				*cGetPointerToBufferWithIndex( unsigned portBASE_TYPE xComID, unsigned char ucIdx );
extern signed portBASE_TYPE 	xComReceiveString( unsigned portBASE_TYPE xComID, TickType_t xBlockTime );
extern void 					vUartRemoveString( unsigned portBASE_TYPE xComID );
extern unsigned portBASE_TYPE 	uxGetNumberOfCharsInBuffer( unsigned portBASE_TYPE xComID );
extern unsigned portBASE_TYPE 	uxGetNumberOfStringsInBuffer( unsigned portBASE_TYPE xComID );
extern unsigned portBASE_TYPE 	uxGetNumberOfMsgWaiting( unsigned portBASE_TYPE xComID );
// extern signed portBASE_TYPE 	xComSendChar( unsigned portBASE_TYPE xComID, signed char cChar  );
// extern signed portBASE_TYPE 	xComSendStringROM( unsigned portBASE_TYPE xComID, PGM_P pcTxString );
extern signed portBASE_TYPE 	xComSendStringRAM( unsigned portBASE_TYPE xComID, signed char *pcTxString );
// extern signed portBASE_TYPE 	xComSendStringEEPROM( unsigned portBASE_TYPE xComID, signed char *pcTxString, bool bTranslateApostr );

#endif

