/*
 * Tracker Firmware
 *
 * NVDS layout
 *
 */
#ifndef DRV_NVM_H
#define DRV_NVM_H

/* Standard include files. */
#include "FreeRTOS.h"
#include "semphr.h"

/* Device specific include files. */
#include "tracker.h"
#include "main.h"
#include "trace.h"

/*-----------------------------------------------------------*/

#define CONFIG_FILE_KEY				( 0x0001 )							/* ID of the configuation file. */
#define CONFIG_REC_KEY  			CONFIG_FILE_KEY						/* ID of the configuration record. */

/* NOTE: A trace file entry for the FDS is defined here and supported by the NVDS driver but not used. The FDS is instable with high BLE load.
         A RAM based approach is used for the trace storage instead. */
#define TRACE_FILE_KEY 				( 0x0002 )							/* ID of the trace file. */
#define TRACE_REC_KEY				TRACE_FILE_KEY						/* ID of the trace record. */

#define FDS_MAX_NUM_DIRTY_REC		( 10 )								/* Limit size of the FDS to (50 + 2) * FDS_VIRTUAL_PAGE_SIZE  = 52KB */

#define FDS_MAX_RETRY				( 1 )

enum xFSTORAGE_BACKEND
{
	FSTORAGE_SD,		/*  SoftDevice. */
	FSTORAGE_NVMC		/*  Non-volatile Memory Controller. */
};	
/*-----------------------------------------------------------*/

/* Public variables. */
/*-----------------------------------------------------------*/

/* Public function prototypes. */
/* Initialise FDS access. */
extern void vFDSInit( void );

/* Check if the specified record exists. */
extern bool bCheckIfRecordExists( uint16_t uiFileId );

/* Update a record with new data. If the record does not exist, create it. */
extern void vUpdateRecord( uint16_t uiFileId );

/* Get a record from NVDS. The existence of the record has already been established in the init function.
   Therefore, it is assumed here that it exists and that non-existence is an error. */
extern void vReadRecord( uint16_t uiFileId );

/* Read one byte from record. */
extern uint8_t ucNVDSReadByte( uint8_t *pucAddress );

/* Read one word from record. */
extern uint16_t usNVDSReadWord( uint16_t *pusAddress );

/* Read one dword from record. */
extern uint32_t ulNVDSReadLong( uint32_t *pulAddress );

/* Write one byte to record. */
extern void vNVDSWriteByteUnprot( uint8_t *pucAddress, uint8_t ucValue );

/* Write one word to record. */
extern void vNVDSWriteWordUnprot( uint16_t *pusAddress, uint16_t usValue );

/* Write one dword to record. */
extern void vNVDSWriteLongUnprot( uint32_t *pulAddress, uint32_t ulValue );

/* Write a string from ROM or RAM to the NVDS into the location pointed to by pcNVDSDest. */
extern void vNVDSWriteStringUnprot( signed char *pcNVDSDest, const signed char *pcSrc, unsigned portBASE_TYPE uxLen );

/* Initialise NVDS access. */
extern void vNVDSInit( void );

/* Get the reset count from the reserved Flash memory page. */
extern void vReadSystemErrorRecord( struct xSYSTEM_ERROR_RECORD *pxResetCount, enum xFSTORAGE_BACKEND xBackend );

/* 	Write the system error record. */
extern void vWriteSystemErrorRecord( struct xSYSTEM_ERROR_RECORD *pxResetCount, enum xFSTORAGE_BACKEND xBackend );

/* Write the system error record in case of a system assert. */
extern void vEmergencyWriteSystemErrorRecord( struct xSYSTEM_ERROR_RECORD *puxErrorRecord );

/* This part of the driver allows erasing the configuration record held in Flash memory without SoftDevice and interrupts. */
extern void vEmergencyEraseConfigRecord( void );
/*-----------------------------------------------------------*/

#endif