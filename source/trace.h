/*
 * Tracker Firmware
 *
 * trace header file
 *
 */ 
#ifndef TRACE_H
#define TRACE_H

#include "ctrl.h"
/*-----------------------------------------------------------*/

/* Trace macros. */
#define TO_MUTEX_TRACE			( 10 * portTICKS_PER_SEC )			/* Time-out (s) for gaining access to the config record. */
#define	LEN_TASKNAME			( 10 )								/* Maximum length of the task name to store in case of stack overflow. */


#if defined ( OS_TRACE )
	#define	V_TRACE_PRINT(          uxTraceMsgId, uxFileLog )						vTracePrint(        uxTraceMsgId, uxFileLog )
	#define	V_TRACE_PRINT_STRG(     uxTraceMsgId, pcStrg, uxFileLog )				vTracePrintStrg(    uxTraceMsgId, pcStrg, uxFileLog )
	#define	V_TRACE_PRINT_BYTE(     uxTraceMsgId, ucArg, uxFileLog )				vTracePrintByte(    uxTraceMsgId, ucArg, uxFileLog )
	#define	V_TRACE_PRINT_3TUPLE(   uxTraceMsgId, sArg0, sArg1, sArg2, uxFileLog )	vTracePrint3Tuple(  uxTraceMsgId, sArg0, sArg1, sArg2, uxFileLog )
	#define	V_TRACE_PRINT_SHORT(    uxTraceMsgId, sArg, uxFileLog )					vTracePrintShort(   uxTraceMsgId, sArg, uxFileLog )
	#define	V_TRACE_PRINT_LONG(     uxTraceMsgId, lArg, uxFileLog )					vTracePrintLong(    uxTraceMsgId, lArg, uxFileLog )
#else
	#define	V_TRACE_PRINT(          uxTraceMsgId, uxFileLog )
    #define	V_TRACE_PRINT_STRG( 	uxTraceMsgId, pcStrg, uxFileLog )				
    #define	V_TRACE_PRINT_BYTE(     uxTraceMsgId, ucArg, uxFileLog )					
    #define	V_TRACE_PRINT_3TUPLE(   uxTraceMsgId, sArg0, sArg1, sArg2, uxFileLog )	
    #define	V_TRACE_PRINT_SHORT(    uxTraceMsgId, sArg, uxFileLog )
	#define	V_TRACE_PRINT_LONG(     uxTraceMsgId, lArg, uxFileLog )	
#endif


/* Size of the trace UART Rx ring buffer. Must be a power of 2. */
#define	TRC_UART_RX_BUFFER_SIZE		( 16 )

/* Default value for logging to the GSM module flash. */
//#define LOG_ENABLE					( false )								
#define LOG_ENABLE					( true )								// DEBUG DEBUG DEBUG

/* Minimum space left of the GSM module's file system to allow storage of trace information. */
#define	TRC_MIN_FREE_FLASH_MEMORY	( 2000 )

/* Time-out for writing to the file system. */
#define TO_FILE_WRITE 				(  5 * portTICKS_PER_100MSEC )		/* ms */

/* Size of the trace buffer in NVDS. */
#define LEN_TRACE_BUFFER			( 2048 )		

/* Structure to keep all statistics variables physically together. */
#define XSTATS_ELEMENTS				( 4 + 2 * 10 )					/* Number of short elements in the structure. */
union uSTATS
{
	struct xSTATS
	{
		/* GSM/GPS module statistics. */
		unsigned short	usGsmModuleCrash;				/* Counter for GSM module crashes. */
		unsigned short	usGsmModuleReboot;				/* Counter for forced GSM module reboots because of connection / AT protocol problems. */
		unsigned short	usGsmAttachFail;				/* Counter for GSM attach failures. */
		unsigned short	usGprsConnectFail;				/* Counter for GPRS connection failures, i.e. GPRS/TCP connection time-outs. */
		
		/* State change statistics counters. */
		unsigned short	usCntEnterSystemActive;			/* Counter for state transitions entering ACTIVE state. */
		unsigned short	usCntEnterSystemInactive;		/* Counter for state transitions entering INACTIVE state. */
		unsigned short	usCntEnterSystemStill;			/* Counter for state transitions entering STILL state. */
		unsigned short	usCntEnterSystemSleep;			/* Counter for state transitions entering SLEEP state. */
		unsigned short	usCntEnterSystemPrealert;		/* Counter for state transitions entering PREALERT state. */
		unsigned short	usCntEnterSystemAlert;			/* Counter for state transitions entering ALERT state. */
		unsigned short	usCntEnterSystemPreSos;			/* Counter for state transitions entering PRESOS state. */
		unsigned short	usCntEnterSystemSos;			/* Counter for state transitions entering SOS state. */
		unsigned short	usCntEnterSystemCharging;		/* Counter for state transitions entering any charging state. */
		
		/* State dwell time statistics counters. */
		unsigned short	usCntDwellSystemActive;			/* Counter for state dwell times in ACTIVE state. */
		unsigned short	usCntDwellSystemInactive;		/* Counter for state dwell times in INACTIVE state. */
		unsigned short	usCntDwellSystemStill;			/* Counter for state dwell times in STILL state. */
		unsigned short	usCntDwellSystemSleep;			/* Counter for state dwell times in SLEEP state. */
		unsigned short	usCntDwellSystemPrealert;		/* Counter for state dwell times in PREALERT state. */
		unsigned short	usCntDwellSystemAlert;			/* Counter for state dwell times in ALERT state. */
		unsigned short	usCntDwellSystemPreSos;			/* Counter for state dwell times in PRESOS state. */
		unsigned short	usCntDwellSystemSos;			/* Counter for state dwell times in SOS state. */
		unsigned short	usCntDwellSystemCharging;		/* Counter for state dwell times in any charging state. */	
		
		/* NOABN mode statistics */
		unsigned short	usCntEnterSystemNoAbn;			/* Counter for entering no abnormal position detection mode. */
		unsigned short	usCntDwellSystemNoAbn;			/* Counter for dwell times in no abnormal position detection mode. */
	} xStats;
	
	unsigned short		usStats[ XSTATS_ELEMENTS ];		/* Memory overlay to address the variable as an array. */
};


/* Define actions for system stats counters. */
enum xACTION 
{
	SYSSTATS_INIT,			/* Initialise. */
	SYSSTATS_UPDATEDWELL,	/* Update current dwell time. */
	SYSSTATS_UPDATE			/* Update all counters. */
};

enum xNOABNACTION
{
	SYSSTATS_NOABN_INIT,		/* Initialise. */
	SYSSTATS_NOABN_UPDATEDWELL,	/* Update current dwell time. */
	SYSSTATS_NOABN_MODESTART,	/* NOABN mode starts. */
	SYSSTATS_NOABN_MODESTOP		/* NOABN mode stops. */
};

/* Memory structure holding a copy of the system error record. */
struct xSYSTEM_ERROR_RECORD 
{
	unsigned long 	ulSWResetCount;										/* Software reset. */
	unsigned long 	ulWDResetCount;										/* Watchdog reset. */
	unsigned long 	ulPOResetCount;										/* Power-on reset. */
	unsigned long 	ulLockupResetCount;									/* Lockup reset. */
	unsigned long 	ulOtherResetCount;									/* Other reset. */
	unsigned long 	ulHardfaultCount;									/* Counter for hardfaults. */
	unsigned long 	ulStackOverflowCount;								/* Counter for stack overflows. */
	unsigned long 	ulSystemErrorCount;									/* RTOS system error count. */
	unsigned long 	ulSDAssertCount;									/* SoftDevice assertion failed. */
	unsigned long 	ulSDMemAccCount;									/* SoftDevice invalid memory access. */	
	unsigned long 	ulnRFSDKAssertCount;								/* nRF SDK assertion failed. */
	unsigned long 	ulnRFSDKErrorCount;									/* nRF SDK error. */
	unsigned long 	ulnRFUnknownErrorCount;								/* nRF unknown error count. */	
	unsigned long 	ulUnsuccessfulBootCount;							/* Count of successive unsuccessful boot attempts. */	
	signed char 	cStackOverflowTaskName[ LEN_TASKNAME + 1 ];			/* Name of the task which has caused the last stack overflow. */

	/* Trace ring buffer read/write indices, stored in persistent memory.
	Conditions:
			usTraceRdIdx == usTraceWrIdx: 			buffer empty
			usTraceRdIdx == usTraceWrIdx + 1:		buffer full					
	usTraceWrIdx		 always points to the next free position. 
	usTraceRdIdx		 points to the next unread position, unless the buffer is empty. */
	unsigned short 	usTraceRdIdx;										/* Trace ring buffer read/write index. */
	unsigned short 	usTraceWrIdx;										/* Trace ring buffer read/write index. */
	unsigned char 	cTraceBuffer[ LEN_TRACE_BUFFER ];					/* Trace buffer storage which survives a system reset. */
};

/* Structure containing info about an error of the type @ref NRF_FAULT_ID_SDK_ERROR. */
struct xERROR_INFO
{
    uint32_t        line_num;    	/* The line number where the error occurred. */
    uint8_t const * p_file_name; 	/* The file in which the error occurred. */
    uint32_t        err_code;    	/* The error code representing the error that occurred. */
	uint32_t        cfsr;  			/* Configurable Fault Status Registers. */	
	uint32_t        hfsr;  			/* HardFault Status Register. */
	uint32_t        dfsr;  			/* Debug Fault Status Register. */
	uint32_t        mmfar;  		/* MemManage Fault Address Register. */
	uint32_t        bfar;  			/* BusFault Address Register. */
	uint32_t        afsr;  			/* Auxiliary Fault Status Register. */
	uint32_t        r0;    			/* Register R0 (hardfault only). */
	uint32_t        r1;    			/* Register R1 (hardfault only). */
	uint32_t        r2;    			/* Register R2 (hardfault only). */
	uint32_t        r3;    			/* Register R3 (hardfault only). */
	uint32_t        r12;   			/* Register R12 (hardfault only). */
	uint32_t        lr;    			/* Register LR (hardfault only). */
	uint32_t        psr;   			/* Register PSR (hardfault only). */
};

/*-----------------------------------------------------------*/

/* Public global variables. */

/* Trace buffer to be written to NVDS. */
extern struct xSYSTEM_ERROR_RECORD	uxErrorRecord;

/* Flag indicating a recovery from a watchdog reset. */
extern bool							bWatchdogRecovery;

/* Flag indicating a recovery from a software reset. */
extern bool							bSoftResetRecovery;

/* Statistics. */
extern union uSTATS					uStats;

/* Copy of the NRF_POWER->RESETREAS register for later storage into the trace record. */
extern unsigned long				ulResetReasonCopy;
/*-----------------------------------------------------------*/

/* Public function prototypes. */
extern void vTraceInit( void );
extern bool bCheckTraceInitialised( void );
extern void vEraseTraceBuffer( void );
extern void vTracePrint( unsigned portBASE_TYPE uxTraceMsgId, unsigned portBASE_TYPE uxFileLog );
extern void vTracePrintStrg( unsigned portBASE_TYPE uxTraceMsgId, signed char *pcStrg, unsigned portBASE_TYPE uxFileLog );
extern void vTracePrintByte(  unsigned portBASE_TYPE uxTraceMsgId, unsigned char ucArg, unsigned portBASE_TYPE uxFileLog );
extern void vTracePrint3Tuple(  unsigned portBASE_TYPE uxTraceMsgId, short sArg0, short sArg1, short sArg2, unsigned portBASE_TYPE uxFileLog );
extern void vTracePrintShort( unsigned portBASE_TYPE uxTraceMsgId, short sArg, unsigned portBASE_TYPE uxFileLog );
extern void vTracePrintLong( unsigned portBASE_TYPE uxTraceMsgId, long lArg, unsigned portBASE_TYPE uxFileLog );
extern void vTracePrintSp( const signed char *pcStrg1 );
extern void vTraceFault( uint32_t uiId, uint32_t uiPc, struct xERROR_INFO *pxErrorInfo, uint32_t *puiErrorStack, unsigned portBASE_TYPE uxFileLog );
extern void vTracePushGpsFixSynopsis( unsigned char *pcFIX, unsigned char *pcHDOP, unsigned short usGps_ACCHCM, unsigned char *pcGsm_SAT, unsigned portBASE_TYPE uxTotalSatVisible );
extern void vTraceFlushLog( void );
extern void vSystemStateStats( enum xACTION xAction, enum xCTRL_STATE xNextCtrlState );
extern void vSystemNoAbnStats( enum xNOABNACTION xAction );
extern void vIncrementResetCount( void );
extern void vApplicationStackOverflowHook( void * xTask, char *pcTaskName );
extern void vApplicationMallocFailedHook( void );
/*-----------------------------------------------------------*/

#endif