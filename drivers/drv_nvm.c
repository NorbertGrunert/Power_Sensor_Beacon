/*
 * Tracker Firmware
 *
 * NVDS handling routines
 *
 * The NVM is handled by the SDK's FDS.
 *
 * Physically, the reserved Flash memory area is located just below the bootloader:
 *		m_fs.start_addr = m_fs.end_addr - FDS_VIRTUAL_PAGES * FDS_VIRTUAL_PAGE_SIZE * 4
 *		m_fs.end_addr = bootloader_addr (0xe4000)
 *
 * FDS_VIRTUAL_PAGES and FDS_VIRTUAL_PAGE_SIZE are set in sdk_config.h. With FDS_VIRTUAL_PAGES = 6 and FDS_VIRTUAL_PAGE_SIZE = 4096,
 * the reserved  flash range is:
 *			0xdd000... 0xe3000
 *
 * One flash page is reserved to hold the reset counters. This page is only written while the SoftDevice is off. Access is done using 
 * the nrf_fstorage library with the NVMC as backend.
 * The page is reserved using the FDS_VIRTUAL_PAGES_RESERVED mechanism. Therefore the page is physically located just after the FDS 
 * memory space.
 *			0xe3000... 0xe4000
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>

#include "tracker.h"

/* nRF SDK files. */
#include "nrf_sdh.h"
#include "fds.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_nvmc.h"
#include "nrf_fstorage_sd.h"
#include "nrf_delay.h"

#define NRF_LOG_MODULE_NAME 		DRV_NVM
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_sdh.h"
#include "nrf_nvmc.h"

/* Device specific include files. */
#include "drv_nvm.h"
#include "ble_ctrl.h"
#include "config.h"
#include "trace.h"
#include "tracemsg.h"
/*-----------------------------------------------------------*/
/* For information: The following FDS error codes are defined:

   NRF_ERROR_FDS_ERR_BASE = 0x8600
	 +0	FDS_ERR_OPERATION_TIMEOUT,							 // Error. The operation timed out.
	 +1	FDS_ERR_NOT_INITIALIZED,                             // Error. The module has not been initialized.
	 +2	FDS_ERR_UNALIGNED_ADDR,                              // Error. The input data is not aligned to a word boundary.
	 +3	FDS_ERR_INVALID_ARG,                                 // Error. The parameter contains invalid data.
	 +4	FDS_ERR_NULL_ARG,                                    // Error. The parameter is NULL.
	 +5	FDS_ERR_NO_OPEN_RECORDS,                             // Error. The record is not open, so it cannot be closed.
	 +6	FDS_ERR_NO_SPACE_IN_FLASH,                           // Error. There is no space in flash memory.
	 +7	FDS_ERR_NO_SPACE_IN_QUEUES,                          // Error. There is no space in the internal queues.
	 +8	FDS_ERR_RECORD_TOO_LARGE,                            // Error. The record exceeds the maximum allowed size.
	 +9	FDS_ERR_NOT_FOUND,                                   // Error. The record was not found.
	+10	FDS_ERR_NO_PAGES,                                    // Error. No flash pages are available.
	+11	FDS_ERR_USER_LIMIT_REACHED,                          // Error. The maximum number of users has been reached.
	+12	FDS_ERR_CRC_CHECK_FAILED,                            // Error. The CRC check failed.
	+13	FDS_ERR_BUSY,                                        // Error. The underlying flash subsystem was busy.
	+14	FDS_ERR_INTERNAL,                                    // Error. An internal error occurred.
*/
/* Function prototypes. */
/*  Wait for FDS to initialize. */
static bool bWaitForFdsReady( void );

/* FDS event handler. */
static void vFdsEvtHandler( fds_evt_t const * p_evt );

/* Initialise FDS access. */
void vFDSInit( void );

/* Check if the specified record exists. */
bool bCheckIfRecordExists( uint16_t uiFileId );

/* Update a record with new data. If the record does not exist, create it. */
void vUpdateRecord( uint16_t uiFileId );

/* Get a record from NVDS. The existence of the record has already been established in the init function.
   Therefore, it is assumed here that it exists and that non-existence is an error. */
void vReadRecord( uint16_t uiFileId );

/* Update a record with new data. If the record does not exist, create it. */
void vUpdateData( void *pvAddress );

/* Read one byte from record. */
uint8_t ucNVDSReadByte( uint8_t *pucAddress );

/* Read one word from record. */
uint16_t usNVDSReadWord( uint16_t *pusAddress );

/* Read one dword from record. */
uint32_t ulNVDSReadLong( uint32_t *pulAddress );

/* Write one byte to record. */
void vNVDSWriteByteUnprot( uint8_t *pucAddress, uint8_t ucValue );

/* Write one word to record. */
void vNVDSWriteWordUnprot( uint16_t *pusAddress, uint16_t usValue );

/* Write one dword to record. */
void vNVDSWriteLongUnprot( uint32_t *pulAddress, uint32_t ulValue );

/* Write a string from ROM or RAM to the NVDS into the location pointed to by pcNVDSDest. */
void vNVDSWriteStringUnprot( signed char *pcNVDSDest, const signed char *pcSrc, unsigned portBASE_TYPE uxLen );

/* Callback function for the fstorage library. */
static void vFStorageCallback( nrf_fstorage_evt_t *pxFStorageEvt );

/* Initialise FDS access. */
void vNVDSInit( void );
/*-----------------------------------------------------------*/

/* Global variables. */
/* Flag to check FDS ready state. */
static bool volatile 	bFdsReady;
/* Flag to check FDS operation timeout. Provides the possiility to repeat. */
static bool volatile 	bFdsTimeOut;

/* A record containing the configuration record used for *writing* records. */
static fds_record_t 	xConfigFdsWrRecord =
{
    .file_id           = CONFIG_FILE_KEY,
    .key               = CONFIG_REC_KEY,
    .data.p_data       = &xNvdsConfig,
    .data.length_words = sizeof( xNvdsConfig ) / sizeof( uint32_t )			/* The length of a record is always expressed in 4-byte units (words). */
};

/* fstorage structure for defining the flash page containing the reset counters.
   This instance is used for accesses with NVMC backend.
   Start and end address are calculated in real-time. */
NRF_FSTORAGE_DEF( nrf_fstorage_t xFStorageNVMC ) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = vFStorageCallback,
};

/* fstorage structure for defining the flash page containing the reset counters.
   This instance is used for accesses with SD backend.
   Start and end address are calculated in real-time. */
NRF_FSTORAGE_DEF( nrf_fstorage_t xFStorageSD ) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = vFStorageCallback,
};

/* Flags indicating the state of fstorage actions. */
bool	bWriteFinished;
bool	bEraseFinished;
/*-----------------------------------------------------------*/

/*-------------------------------------------------------------
 *
 * FDS managed part
 *
 *------------------------------------------------------------*/

/*  Wait for FDS to finish operation. 

    Returns false if the operation timed out. */
static bool bWaitForFdsReady( void )
{
	TickType_t			xStartTime;

	xStartTime = xTaskGetTickCount();

    while ( !bFdsReady && !bFdsTimeOut && ( xTaskGetTickCount() - xStartTime < 10 * portTICKS_PER_SEC ) )
    {
		( void )sd_app_evt_wait();
		vTaskDelay ( 1 );
    }

	if ( !bFdsReady )
	{
		bFdsTimeOut = true;
	}

	return !bFdsTimeOut;
}
/*-----------------------------------------------------------*/

/* FDS event handler. */
static void vFdsEvtHandler( fds_evt_t const * p_evt )
{
    if ( p_evt->result == NRF_SUCCESS )
    {
		bFdsReady = true;
		bFdsTimeOut = false;
    }
    else
    {
		if ( p_evt->result == FDS_ERR_OPERATION_TIMEOUT )
		{
			bFdsReady = false;
			bFdsTimeOut = true;
		}
		else
		{
	        NRF_LOG_ERROR( "FDS event: ID %i, file %i ( result 0x%x )",
							p_evt->id,
							p_evt->write.file_id,
	                        p_evt->result );
		}
    }
}
/*-----------------------------------------------------------*/

/* Initialise FDS access. 

   Requires the SD to be initialised and the scheduler to be running.
*/
void vFDSInit( void )
{
	ret_code_t			xErrCode;
	fds_stat_t 			xFdsStat = { 0 };
		
    /* Register first to receive an event when initialization is complete. */
    ( void )fds_register( vFdsEvtHandler );	

	bFdsReady = false;
	bFdsTimeOut = false;
    xErrCode = fds_init();
    APP_ERROR_CHECK( xErrCode );

    /* Wait for FDS to initialize. */
    ( void )bWaitForFdsReady();

	/* Check the number of dirty records and perform garbage collection if necessary. */
    xErrCode = fds_stat( &xFdsStat );
    APP_ERROR_CHECK( xErrCode ) ;

	NRF_LOG_INFO( "FDS: %d pages available.", xFdsStat.pages_available );
	NRF_LOG_INFO( "FDS: %d open records.", xFdsStat.open_records );
	NRF_LOG_INFO( "FDS: %d valid records.", xFdsStat.valid_records );
    NRF_LOG_INFO( "FDS: %d dirty records (ready to be garbage collected).", xFdsStat.dirty_records );
    NRF_LOG_INFO( "FDS: %d words used.", xFdsStat.words_used );
    NRF_LOG_INFO( "FDS: %d freeable words.", xFdsStat.freeable_words );
    NRF_LOG_INFO( "FDS: %d (corruption).", xFdsStat.corruption );
	NRF_LOG_FLUSH();

    if ( xFdsStat.dirty_records > FDS_MAX_NUM_DIRTY_REC )
	{
		NRF_LOG_INFO( "FDS: Collecting gargabe." );

		xErrCode = fds_gc();
		APP_ERROR_CHECK( xErrCode ) ;

	    xErrCode = fds_stat( &xFdsStat );
		APP_ERROR_CHECK( xErrCode ) ;

		NRF_LOG_INFO( "%d pages available.", xFdsStat.pages_available );
		NRF_LOG_INFO( "%d open records.", xFdsStat.open_records );
		NRF_LOG_INFO( "%d valid records.", xFdsStat.valid_records );
	    NRF_LOG_INFO( "%d dirty records (ready to be garbage collected).", xFdsStat.dirty_records );
	    NRF_LOG_INFO( "%d words used.", xFdsStat.words_used );
	    NRF_LOG_INFO( "%d freeable words.", xFdsStat.freeable_words );
	    NRF_LOG_INFO( "%d (corruption).", xFdsStat.corruption );
		NRF_LOG_FLUSH();
	}	

	NRF_LOG_INFO( "FDS Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* Get the highest record ID for a given file/record key pair and set the file descriptor to the newest one 
   (the one with the highest record ID).
   
   Returns (-1) if none exists.
*/
portBASE_TYPE xGetHighestRecordID( uint16_t uiFileId, fds_record_desc_t *pxRecDesc )
{
	ret_code_t						xErrCode;
	portBASE_TYPE					xNewestRecordID;
	unsigned portBASE_TYPE			uxRecordsFound;
	fds_find_token_t 				xFindToken;	
	static const fds_find_token_t	xZeroFindToken = { 0 };
	
	xNewestRecordID = -1;
	uxRecordsFound = 0;
	xFindToken = xZeroFindToken;
	do
	{
		xErrCode = fds_record_find_in_file( uiFileId, pxRecDesc, &xFindToken );
		if ( xErrCode == NRF_SUCCESS )
		{
			uxRecordsFound++;
			if ( xNewestRecordID < ( portBASE_TYPE )pxRecDesc->record_id )
			{
				xNewestRecordID = ( portBASE_TYPE )pxRecDesc->record_id;
			}
		}
		else
		{
			if ( xErrCode != FDS_ERR_NOT_FOUND )
			{
				/* Any other case is an error. */
				NRF_LOG_ERROR( "Error accessing the config/trace file! %8.8x", xErrCode );
				NRF_LOG_FLUSH();		
				APP_ERROR_CHECK( xErrCode );
			}
		}
	}
	while ( xErrCode == NRF_SUCCESS );

	/* If at least one record has been found, position the record descriptor onto this one. */
	if ( uxRecordsFound > 0 )
	{
		/* Found at least one valid record. Position on the latest entry (the one with the highest record ID). */
		xFindToken = xZeroFindToken;
		do
		{
			xErrCode = fds_record_find_in_file( uiFileId, pxRecDesc, &xFindToken );
		} 
		while ( ( xErrCode == NRF_SUCCESS ) && ( ( portBASE_TYPE )pxRecDesc->record_id != xNewestRecordID ) );
	}
	else
	{
		NRF_LOG_INFO( "No existing record found." );		
	}

	return xNewestRecordID;
}
/*-----------------------------------------------------------*/	

/* Check if the specified record exists. */
bool bCheckIfRecordExists( uint16_t uiFileId )
{
	fds_record_desc_t				xRecDesc;				/* Record descriptor. */

    return ( xGetHighestRecordID( uiFileId, &xRecDesc ) != -1 );
}
/*-----------------------------------------------------------*/	

/* Update a record with new data. If the record does not exist, create it. 
 
   Even though the SDK tries to avoid it, itmight happen that several valid records with the same
   file/record key exist. In that case, iterate through all entries and identify the one with the newest
   record ID.
*/
void vUpdateRecord( uint16_t uiFileId )
{
	ret_code_t						xErrCode;
	fds_record_desc_t				xRecDesc;				/* Record descriptor. */
	fds_record_t					*pxWrRecord;			/* Pointer to a record to be written. */
	portBASE_TYPE					xNewestRecordID;
	fds_stat_t 						xFdsStat = { 0 };
	unsigned portBASE_TYPE			uxTrialCount;
	
	( void )uiFileId;

	/* Depending on the required file, point to the correct data structures. */
	pxWrRecord 	= &xConfigFdsWrRecord;
	NRF_LOG_INFO( "Updating CONFIG file." );
	
	/* Clean up all records belonging to the file. 
	   Two conditions need attention:
			1. There are too many dirty records filling up the Flash space. Run garbage collection.
			2. More than one valid record exists. Delete all records belonging to that file and perform garbage collection.
 			   A new record will be created thereafter. */
	/* Check the number of dirty records and perform garbage collection if necessary. */
    xErrCode = fds_stat( &xFdsStat );
    APP_ERROR_CHECK( xErrCode ) ;

    NRF_LOG_INFO( "Found %d valid records.", xFdsStat.valid_records );
    NRF_LOG_INFO( "Found %d dirty records (ready to be garbage collected).", xFdsStat.dirty_records );

	/* First, check if there are more than one valid record and recreate the file system from scratch if this is the case. */
    if ( xFdsStat.valid_records > 2 )
	{
		NRF_LOG_WARNING( "Several valid records found file %i. Deleting all records and creating a new structure.", uiFileId );
		
		uxTrialCount = 0;
		do
		{
			bFdsReady = false;
			bFdsTimeOut = false;
			xErrCode = fds_file_delete( uiFileId );
			APP_ERROR_CHECK( xErrCode );
			bWaitForFdsReady();
			uxTrialCount++;
		}
		/* Wait for FDS to process the request. Repeat, if the operation timed-out. */
		while ( !bFdsReady && ( uxTrialCount < FDS_MAX_RETRY ) );

		if ( !bFdsReady )
		{
			/* In case the update failed, just return. Do not reset the system. */
			V_TRACE_PRINT( TRACE_FDS_FAILURE, TRACE_UART );
			return;
		}
	}	

	/* Now perform the garbage collection. */
    if ( ( xFdsStat.valid_records > 2 ) || ( xFdsStat.dirty_records > FDS_MAX_NUM_DIRTY_REC ) )
	{
		NRF_LOG_INFO( "Garbage collection." );

		uxTrialCount = 0;
		do
		{
			bFdsReady = false;
			bFdsTimeOut = false;
			xErrCode = fds_gc();
			APP_ERROR_CHECK( xErrCode );
			bWaitForFdsReady();
			uxTrialCount++;
		}
		/* Wait for FDS to process the request. Repeat, if the operation timed-out. */
		while ( !bFdsReady && ( uxTrialCount < FDS_MAX_RETRY ) );

		if ( !bFdsReady )
		{
			/* In case the update failed, just return. Do not reset the system. */
			V_TRACE_PRINT( TRACE_FDS_FAILURE, TRACE_UART );
			return;
		}
	}	

	/* Position onto the newest record. */
	xNewestRecordID = xGetHighestRecordID( uiFileId, &xRecDesc );

	uxTrialCount = 0;
	do
	{
		bFdsReady = false;
		bFdsTimeOut = false;
		if ( xNewestRecordID == -1 )
		{
			/* There is no previous record, create a new one. */
			NRF_LOG_INFO( "Creating new record." );
			NRF_LOG_FLUSH();
			xErrCode = fds_record_write( NULL, pxWrRecord );		
		}
		else
		{
			/* Found at least one valid record. Update it. */
			NRF_LOG_INFO( "Updating existing record %i.", xNewestRecordID );
			NRF_LOG_FLUSH();
			xErrCode = fds_record_update( &xRecDesc, pxWrRecord );
		}

		/* Check the error code. */
		APP_ERROR_CHECK( xErrCode ) ;
		bWaitForFdsReady();
		uxTrialCount++;
	}
	/* Wait for FDS to process the request. Repeat, if the operation timed-out. */
	while ( !bFdsReady && ( uxTrialCount < FDS_MAX_RETRY ) );
	
	if ( !bFdsReady )
	{
		/* In case the update failed, just return. Do not reset the system. */
		V_TRACE_PRINT( TRACE_FDS_FAILURE, TRACE_UART );
	}
}
/*-----------------------------------------------------------*/
	
/* Get a record from NVDS. The existence of the record has already been established in the init function.
   Therefore, it is assumed here that it exists and that non-existence is an error. */
void vReadRecord( uint16_t uiFileId )
{
	ret_code_t			xErrCode;
	fds_record_desc_t	xRecDesc;				/* Record descriptor. */
	unsigned char 		*pucDest;				/* Pointer to where to store the data. */
	size_t				xLen;					/* Length of the data record to copy. */
	fds_flash_record_t 	xFdsRdRecord = { 0 };	/* A structure used for *reading* records. */

	( void )uiFileId;

	/* Depending on the required file, point to the correct data structures. */
	pucDest		= ( unsigned char * )&xNvdsConfig;
	xLen		= sizeof( xNvdsConfig );

	/* Position onto the newest record. */
	if ( xGetHighestRecordID( uiFileId, &xRecDesc ) == -1 )
	{
		/* No existing record found. */
	    NRF_LOG_INFO( "Reading from file %i: No existing record found.", uiFileId );
		NRF_LOG_FLUSH();
		ASSERT( false );
	}

	/* Open and read the record. */
    xErrCode = fds_record_open( &xRecDesc, &xFdsRdRecord );
	APP_ERROR_CHECK( xErrCode );
	
	/* Copy the configuration from flash to the destination. */
	memcpy( pucDest, xFdsRdRecord.p_data, xLen );	
	
	/* Close the record. */
	xErrCode = fds_record_close( &xRecDesc );
	APP_ERROR_CHECK( xErrCode );
}
/*-----------------------------------------------------------*/


/*-------------------------------------------------------------
 *
 * FSTORAGE managed part
 *
 *------------------------------------------------------------*/
/* Access to the FSTORAGE managed part is done in two different ways depending on the state of the system.
   While the SoftDevice is not yet initialised, the FSTORAGE library uses the NVMC. Once it is initialised,
   the SoftDevice is used.
   
   To be able to access the same chunk of memory using the two different backends, two different FSTORAGE
   instances are used. */
   
/* Callback function for the fstorage library. */
static void vFStorageCallback( nrf_fstorage_evt_t *pxFStorageEvt )
{
    if ( pxFStorageEvt->id == NRF_FSTORAGE_EVT_WRITE_RESULT ) 
	{
		bWriteFinished = true;
    }
    if ( pxFStorageEvt->id == NRF_FSTORAGE_EVT_ERASE_RESULT ) 
	{
        bEraseFinished = true;
	}
}
/*-----------------------------------------------------------*/

/* Initialise NVDS access. */
void vNVDSInit( void )
{
	ret_code_t			xErrCode;
		
	/* Initialise the fstorage instance using the nrf_fstorage_nvmc backend (implementation where the 
	   SoftDevice is not running). Using this implementation when the SoftDevice is enabled results in a hardfault. */
	#if defined ( NO_BOOTLOADER )
		xFStorageNVMC.start_addr = FSTORAGE_PAGE_START - FDS_VIRTUAL_PAGES_RESERVED * NRF_FICR->CODEPAGESIZE;
		xFStorageNVMC.end_addr = FSTORAGE_PAGE_START - 1;
	#else
		xFStorageNVMC.start_addr = NRF_UICR->NRFFW[ 0 ] - FDS_VIRTUAL_PAGES_RESERVED * NRF_FICR->CODEPAGESIZE;
		xFStorageNVMC.end_addr = NRF_UICR->NRFFW[ 0 ] - 1;
	#endif
	xErrCode = nrf_fstorage_init( &xFStorageNVMC, &nrf_fstorage_nvmc, NULL );
	APP_ERROR_CHECK( xErrCode );		
	
	#if !defined ( NO_SOFTDEVICE )
		/* Initialise the fstorage instance using the nrf_fstorage_sd backend (implementation where the 
		   SoftDevice is not running). Using this implementation when the SoftDevice is enabled results in a hardfault. */
		#if defined ( NO_BOOTLOADER )
			xFStorageSD.start_addr = FSTORAGE_PAGE_START - FDS_VIRTUAL_PAGES_RESERVED * NRF_FICR->CODEPAGESIZE;
			xFStorageSD.end_addr = FSTORAGE_PAGE_START - 1;
		#else
			xFStorageSD.start_addr = NRF_UICR->NRFFW[ 0 ] - FDS_VIRTUAL_PAGES_RESERVED * NRF_FICR->CODEPAGESIZE;
			xFStorageSD.end_addr = NRF_UICR->NRFFW[ 0 ] - 1;
		#endif
		xErrCode = nrf_fstorage_init( &xFStorageSD, &nrf_fstorage_sd, NULL );
		APP_ERROR_CHECK( xErrCode );
	#endif

	NRF_LOG_INFO( "NVDS Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* Get the system error record from the reserved Flash memory page. */
void vReadSystemErrorRecord( struct xSYSTEM_ERROR_RECORD *pxSystemErrorRecord, enum xFSTORAGE_BACKEND xBackend )
{
    ret_code_t 		xErrCode; 
	nrf_fstorage_t 	*pxFStorage;
	
	if (xBackend == FSTORAGE_SD )
	{
		#if !defined ( NO_SOFTDEVICE )	
			pxFStorage = &xFStorageSD;
		#else
			pxFStorage = &xFStorageNVMC;
		#endif
	}
	else
	{
		pxFStorage = &xFStorageNVMC;
	}
	
	xErrCode = nrf_fstorage_read( pxFStorage,     							/* The instance to use. */
                                  pxFStorage->start_addr,	 				/* The address in flash where to read data from. */
                                  pxSystemErrorRecord,						/* A buffer to copy the data into. */
                                  sizeof( struct xSYSTEM_ERROR_RECORD ) );	/* Lenght of the data, in bytes. */
    
    if ( xErrCode != NRF_SUCCESS) 
	{
        NRF_LOG_ERROR( "Error reading reserved Flash page!" );
    }

	/* Any values 0xffffffff mean that the device has been freshly initialised after factory. In that case, 
	   reset all value to 0 first. */
	if (    ( pxSystemErrorRecord->ulSWResetCount     == 0xffffffff )
	     || ( pxSystemErrorRecord->ulWDResetCount     == 0xffffffff )		
	     || ( pxSystemErrorRecord->ulPOResetCount     == 0xffffffff )		
	     || ( pxSystemErrorRecord->ulLockupResetCount == 0xffffffff )
	     || ( pxSystemErrorRecord->ulOtherResetCount  == 0xffffffff ) )
	{
		/* New device: reset all error fields to 0 (including the trace buffer). */
		memset( pxSystemErrorRecord, 0, sizeof( struct xSYSTEM_ERROR_RECORD ) );
	}
}
/*-----------------------------------------------------------*/

/* 	Write the system error record. */
void vWriteSystemErrorRecord( struct xSYSTEM_ERROR_RECORD *pxSystemErrorRecord, enum xFSTORAGE_BACKEND xBackend )
{
    ret_code_t 		xErrCode;
	nrf_fstorage_t 	*pxFStorage;
	
	if (xBackend == FSTORAGE_SD )
	{
		#if !defined ( NO_SOFTDEVICE )	
			/* Stop all BLE activity. The SoftDevice takes up so many cycles that it stalls the FDS accesses on a collision.
			   It appears that the SoftDevice / SDK library routines do not recover well from a stalling so it is better to prevent 
			   collisions in the first place. */
			vBlockingSuspendAllBleActivity();

			pxFStorage = &xFStorageSD;
		#else
			pxFStorage = &xFStorageNVMC;
		#endif
	}
	else
	{
		pxFStorage = &xFStorageNVMC;
	}
	
	bEraseFinished = false;
	bWriteFinished = false;
	
    xErrCode = nrf_fstorage_erase( pxFStorage,     							/* The instance to use. */
                                   pxFStorage->start_addr,	 				/* The address of the flash pages to erase. */
                                   1,              							/* The number of pages to erase. */
                                   NULL );             						/* Optional parameter, backend-dependent. */

    if ( xErrCode != NRF_SUCCESS) 
	{
        NRF_LOG_ERROR( "Error erasing reserved Flash page!" );
    }

	while ( nrf_fstorage_is_busy( pxFStorage ) && !bEraseFinished )
	{
		;
	}
	
	xErrCode = nrf_fstorage_write( pxFStorage,     							/* The instance to use. */
                                   pxFStorage->start_addr,					/* The address in flash where to read data from. */
                                   pxSystemErrorRecord,						/* A buffer to copy the data into. */
                                   sizeof( struct xSYSTEM_ERROR_RECORD ),	/* Lenght of the data, in bytes. */
								   NULL );
    
    if ( xErrCode != NRF_SUCCESS) 
	{
        NRF_LOG_ERROR( "Error reading reserved Flash page!" );
    }

	while ( nrf_fstorage_is_busy( pxFStorage ) && !bWriteFinished )
	{
		;
	}	

	if (xBackend == FSTORAGE_SD )
	{
		#if !defined ( NO_SOFTDEVICE )	
			/* Restore the activity to the same state it was in when vBlockingSuspendAllBleActivity() was called. */
			vResumeAllBleActivity();	
		#endif
	}
}
/*-----------------------------------------------------------*/


/*-------------------------------------------------------------
 *
 * Bare metal Flash access
 *
 *------------------------------------------------------------*/
/* This part of the driver allows writing the Flash memory without SoftDevice and interrupts.
   The purpose is to write the system error record in case of a system assert. */
void vEmergencyWriteSystemErrorRecord( struct xSYSTEM_ERROR_RECORD *puxErrorRecord )
{
	uint32_t			uiStartaddress;
	uint32_t			uiNumLong;
	
	/* Configure MWU to disable sandboxing of the NVMC. */
	NRF_MWU->PREGION[ 0 ].SUBS &= ~( MWU_PREGION_SUBS_SR30_Include << MWU_PREGION_SUBS_SR30_Pos );
	
	/* Barrier to ensure register is set before accessing NVMC.  */
	__DSB(); 
	
	#if defined ( NO_BOOTLOADER )
		uiStartaddress = FSTORAGE_PAGE_START - FDS_VIRTUAL_PAGES_RESERVED * NRF_FICR->CODEPAGESIZE;
	#else
		uiStartaddress = NRF_UICR->NRFFW[ 0 ] - FDS_VIRTUAL_PAGES_RESERVED * NRF_FICR->CODEPAGESIZE;
	#endif
	uiNumLong = sizeof( struct xSYSTEM_ERROR_RECORD ) / 4;
	if ( ( sizeof( struct xSYSTEM_ERROR_RECORD ) & 0x3 ) != 0 )
	{
		uiNumLong++;
	}
	
	/* Erase the Flash page containing the error record. */
	nrf_nvmc_page_erase( uiStartaddress );

	/* Barrier to ensure register is set before writing the NVMC.  */
	__DSB(); 
	
	/* Write the system error record to Flash. */
	nrf_nvmc_write_words( uiStartaddress,
						  ( uint32_t * )puxErrorRecord,
						  uiNumLong );
}
/*-----------------------------------------------------------*/

/* This part of the driver allows erasing the configuration record held in Flash memory without SoftDevice and interrupts. */
void vEmergencyEraseConfigRecord( void )
{
	uint32_t				uiStartaddress;
	uint32_t				uiNumLong;
	unsigned portBASE_TYPE	uxPageIdx;
	
	/* Configure MWU to disable sandboxing of the NVMC. */
	NRF_MWU->PREGION[ 0 ].SUBS &= ~( MWU_PREGION_SUBS_SR30_Include << MWU_PREGION_SUBS_SR30_Pos );
	
	/* Barrier to ensure register is set before accessing NVMC.  */
	__DSB(); 
	
	#if defined ( NO_BOOTLOADER )
		uiStartaddress = FSTORAGE_PAGE_START - FDS_VIRTUAL_PAGES_RESERVED * NRF_FICR->CODEPAGESIZE - FDS_VIRTUAL_PAGES * NRF_FICR->CODEPAGESIZE;
	#else
		uiStartaddress = NRF_UICR->NRFFW[ 0 ] - FDS_VIRTUAL_PAGES_RESERVED * NRF_FICR->CODEPAGESIZE - FDS_VIRTUAL_PAGES * NRF_FICR->CODEPAGESIZE;
	#endif
	
	/* Erase the Flash page containing the error record. */
	for ( uxPageIdx = 0; uxPageIdx < FDS_VIRTUAL_PAGES; uxPageIdx++ )
	{
		nrf_nvmc_page_erase( uiStartaddress + ( uxPageIdx * NRF_FICR->CODEPAGESIZE ) );
	}

	/* Barrier to ensure register is set before writing the NVMC.  */
	__DSB(); 
}
/*-----------------------------------------------------------*/

