/*
 * Tracker Firmware
 *
 * Utility functions
 *
 */
#ifndef UTILS_H
#define UTILS_H

#include "FreeRTOS.h"
#include "queue.h"
/*-----------------------------------------------------------*/

#define __ASM __asm /*!< asm keyword for GNU Compiler */ 
#define __INLINE inline /*!< inline keyword for GNU Compiler */ 
#define __STATIC_INLINE static inline 
/*-----------------------------------------------------------*/

/* Return the value of the Link Register. */ 
__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __get_LR( void ) 
{ 
  register uint32_t result; 

  __ASM volatile ( "MOV %0, LR\n" : "=r" ( result ) ); 

  return( result ); 
} 
/*-----------------------------------------------------------*/

/* Return the value on the stack at position 24. */ 
__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __getReturnAddr24( void ) 
{ 
  register uint32_t result; 

  __ASM volatile ( "LDR %0, [SP, #0x24]\n" : "=r" ( result) ); 

  return( result ); 
} 
/*-----------------------------------------------------------*/

/* Macro for instatiating a call to vSystemReset which provides debug information. */
#define V_SYSTEM_RESET( ERR_CODE )                  				\
do                                                              	\
{                                                               	\
	vSystemReset( ( ERR_CODE ), __LINE__, ( uint8_t* ) __FILE__ );  \
} while ( 0 )
/*-----------------------------------------------------------*/

/* Implementation  of the Linux-type stpncpy() function. */
char *stpncpy( char *pcDst, const char *pcSrc, size_t xLen );

/* Check if a memory block is completely filled with a certain value. */
extern bool bMemVCmp( char *pcSrc, const char cVal, size_t xLen );

/* Convert a 4-bit value into a HEX-character. */
extern signed char cNibbleToChar( signed char cNibble );

/* Convert a HEX-character ('0'- '9', 'A' - 'F') into a 4-bit value. */
extern signed char cCharToNibble( signed char cChar );

/* Convert a 8-bit value into a string of HEX-characters including end-of-string 0x0. */
extern void vByteToHexStrg( signed char *pcStrg, unsigned char ucValue );

/* Convert a string of HEX-characters into an 8-bit value. */
extern unsigned char ucHexStrgToByte( signed char *pcStrg );

/* Convert a 8-bit value into a string of unsigned integer characters including end-of-string 0x0. */
extern void vByteToIntStrg( signed char *pcStrg, unsigned char ucValue, bool bSuppressLeadingZ );

/* Convert a a string of unsigned integer characters including end-of-string 0x0 into a 8-bit value. */
unsigned char ucIntStrgToByte( signed char *pcStrg, portBASE_TYPE xLen );

/* Convert a 16-bit value into a string of HEX-characters including end-of-string 0x0. */
extern void vShortToHexStrg( signed char *pcStrg, unsigned short usValue );

/* Convert a 16-bit value into a string of unsigned integer characters including end-of-string 0x0. */
extern void vShortToIntStrg( signed char *pcDest, unsigned short usValue, bool bSuppressLeadingZ );

/* Convert a 32-bit value into a string of unsigned integer characters including end-of-string 0x0. */
extern void vLongToIntStrg( signed char *pcStrg, unsigned long ulValue, bool bSuppressLeadingZ );

/* Convert a a string of unsigned integer characters including end-of-string 0x0 into a 16-bit value. */
extern unsigned short usIntStrgToShort( signed char *pcStrg, portBASE_TYPE xLen );

/* Convert a a string of unsigned long integer characters including end-of-string 0x0 into a 32-bit value. */
extern unsigned long ulIntStrgToLong( signed char *pcStrg, portBASE_TYPE xLen );

/* Convert a string of HEX-characters in little endian in the UART Rx buffer into an 32-bit value. */
extern unsigned long ulHexComStrgToLongLE( unsigned portBASE_TYPE xComID, unsigned portBASE_TYPE uxStrgIdx );

/* Convert a string of HEX-characters into a 16-bit value. */
extern unsigned short usHexStrgToShort( signed char *pcStrg );

/* Convert a 32-bit value into a string of HEX-characters including end-of-string 0x0. */
extern void vLongToHexStrg( signed char *pcStrg, unsigned long ulValue );

/* Convert a string of HEX-characters into a 32-bit value. */
extern unsigned long ulHexStrgToLong( signed char *pcStrg );

/* Convert a a string of unsigned integer characters including end-of-string 0x0 into a 16-bit value. 
   The maximum size of the string is 5 characters (xLen). */
extern unsigned short usIntStrgToShort( signed char *pcStrg, portBASE_TYPE	xLen );

/* Return pointer to the character after the Nth comma ','  in a string. */
extern signed char *pcFindNComma( signed char *pcChar, unsigned portBASE_TYPE uxN, unsigned portBASE_TYPE uxMaxChar );

/* Compare two 16-bit time stamps and determine which is later. */
extern bool bIsANewerThanB( uint16_t a, uint16_t b );

/* Convert double quotes in RAM string to single quotes. */
extern void vTranslateDoubleQuote( signed char *pcChar );

/* Convert single quotes in RAM string to double quotes. */
extern void vTranslateSingleQuote( signed char *pcChar );

/* Calculate the logarithm to the base of 2 (log2() or ld()) using fixed-point arithmetic. */
extern unsigned long nlog2_16 ( unsigned short x );

/* Remove all messages of a specific type from a queue. All other messages remain in the 
   queue in the same order as before.
*/
extern void vRemoveTypeFromQueue( QueueHandle_t xQueue, unsigned portBASE_TYPE xMessageType );

/* Remove all messages which are not server commands from a queue. Server command messages remain in the 
   queue in the same order as before.
*/
extern void vRemoveAllMsgsFromQueue( QueueHandle_t xQueue );

/* Check, if a message of a specific type is in a queue. */
extern bool bMsgTypeInQueue( QueueHandle_t xQueue, void *pxMessageType );

/* Send a message to the specified queue if it is not already in there. */
extern BaseType_t xSendUniqueMsgToQueue( QueueHandle_t xQueue, const void * const pxMessage, TickType_t xTicksToWait );

/* Return the bootloader flash for the bootloader version string. */
extern bool bLocateBootloaderVersion( signed char **ppcBootloaderVerstionStrg );


/* Dump data in hex format to the logger. */
extern void vDebugHexDump( char *pcMemory, uint32_t	uiSize );

/* Safe vaeriob of FreeRTOS' task delay. Does nothing if the scheduler is not yet running. */
extern void vTaskDelaySafe( const TickType_t xTicksToDelay );

/* Hard-reset the system. */
extern void vSystemReset( uint32_t uiErrorCode, uint32_t uiLineNum, const char * pcFileName );
/*-----------------------------------------------------------*/

#endif