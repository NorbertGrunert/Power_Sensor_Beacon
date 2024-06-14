/*
 * Tracker Firmware
 *
 * General header file for misc. definitions
 *
 */
 
/* nRF52840 Flash memory and RAM layout s
   (Put here for the lack of a better place)
 
Flash memory layout																	RAM layout
===================																	==========
	
0x0010 0000 	+-------------------------------+									0x2040 0000	    +-------------------------------+
				|  MBR Parameter Storage		|                               					|  								|
0x000f e000		+-------------------------------+ MBRPARAMADDR: UICR.NRFFW[1]   	   				|  								|
				|  								|                               					|  								|
				|  Bootloader                   |                               					|  								|
				|  								|                               					|  								|
				+- - - - - - - - - - - - - - - -+                               					|  								|
				|  Bootloader Vector Table		|                               					+-------------------------------+
0x000e 4000		+-------------------------------+ BOOTLOADERADDR: UICR.NRFFW[0] 	   				|  								|
				|  fstorage: Reset counters		| 1 PHY page: 1 * FICR.CODEPAGESIZE					|  								|
0x000e 3000		+-------------------------------+                               					|  								|
				|  FDS	(6 pages * 0x1000)		| FDS_VIRTUAL_PAGES * 								|  								|
				|  								| * FDS_VIRTUAL_PAGE_SIZE       					|  								|
0x000d d000		+-------------------------------+                               					|  FreeRTOS Heap				| configTOTAL_HEAP_SIZE
				|  								|                               					|  								|
				|  								|                               					|  								|
				|  								|                               					|  								|
				|  								|                               					|  								|
				|  								|                               					|  								|
				|  								|                               					+-------------------------------+
				|  								|                               					|  								|
				+-------------------------------+                               					|  								|
				|  								|                               					|  Static variables				|
				|  								|                               					|  								|
				|     							|                               					|  								|
				|  								|                               	0x2000 5978		+-------------------------------+ APP_RAM_BASE
				|  								|                               					|  								|
				|  Application					|                               					|  								|
				|  								|                               					|  Used by Softdevice			|
				|  								|                               					|  								|
				|  								|                               					|  								|
				|  								|                               	0x2000 0000		+-------------------------------+
				|  								|                               
				+- - - - - - - - - - - - - - - -+                               
				|  Application Vector Table		|
0x0002 7000		+-------------------------------+ APP_CODE_BASE
				|  								|
				|  								|
				|  								|
				|  								|
				|  								|
				|  SoftDevice					|
				|  								|
				|  								|
				|  								|
				|  								|
				|  								|
				+- - - - - - - - - - - - - - - -+
				|  SD Vector Table				|
0x0000 1000		+-------------------------------+
				|  Master boot record (MBR)		|
				+- - - - - - - - - - - - - - - -+
				|  MBR Vector Table				|
0x0000 0000		+-------------------------------+

	length of the fstorgae reset counter record: FDS_VIRTUAL_PAGES_RESERVED * FICR.CODEPAGESIZE = 1 * FICR.CODEPAGESIZE
	
	
Interrupt Usage
===============

In normal run state, the following hardware interrupts are enabled:

											priority (high to low)		BLE usage		APP usage					Note
		----------------------------------------------------------------------------------------------------------------------------------
        RADIO                               	0						SD							
        RTC0                                	0 						SD					
		MWU                                 	1						SD	 										memory watch unit
		WDT                                 	2										watchdog debug
		POWER_CLOCK                         	4						SD											
        RNG                                  	4           			SD											random number generator
        ECB                                  	4           			SD
        CCM_AAR                              	4           			SD											AES CCM
        SWI5_EGU5                           	4						SD					
        UARTE0_UART0                         	5           							UART (COM0)
        UARTE1                              	5										UART (COM1)
        SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1   	6										SPI/I2C				
        SAADC                               	6										ADC					
        TIMER1                               	6           							UART (COM0)				
        TIMER2                              	6										UART (COM1)				
        SWI2_EGU2                           	6						SD					
        SWI2_EGU2                           	6						SD					
        GPIOTE                              	7										accelerometer IRQ
        RTC1                                	7										OS tick
*/

#ifndef TRACKER_H
#define TRACKER_H

#define VERSION						"nRF52840"

#define RAM_START					( 0x20000000 )
#define RAM_END						( 0x20040000 )

/* Trace ID for a system reset. */
#define	SYSTEM_RESET_ID				( 0xF0000000 )

/* Trace ID for a hardfault. */
#define	HARDFAULT_ID				( 0xE0000000 )
#define	HARDFAULT_STACK_OVR			( 0xE0000000 )

#define FSTORAGE_PAGE_START			( 0x000E4000 )

/* Magic string to locate the bootloader version. */
#define	BL_VERSION_MAGIC	"\x73\xa5\x9e\x01\x22\x8f\xc5\xcc"

/* System reset error codes. */
#define RESET_BAD_FW_UPDATE			( 0x1001 )
#define RESET_BAD_GPS_MODULE		( 0x1002 )
#define RESET_FROM_COMMAND			( 0x2001 )
#define NEW_FW_NOT_FUNCTIONAL		( 0x2002 )
#define RESET_FOR_FW_UPDATE			( 0x2003 )
#define RESET_FROM_BLE_COMMAND		( 0x3001 )
/*-----------------------------------------------------------*/

#if defined ( DEBUG )
	#define 	DEBUG_GPIO
	// #define		DEBUG_GPIO_TASKS
#endif

/* Set the base confguration for NRF logging. 
   Lower individual module severity level:
		https://devzone.nordicsemi.com/f/nordic-q-a/18307/logger-module-config-question-sdk12 */
#ifndef TL_CONFIG_LOG_ENABLED
	#define TL_CONFIG_LOG_ENABLED 1
	
			/* <0=> Off 		*/
			/* <1=> Error       */
			/* <2=> Warning     */
			/* <3=> Info        */
			/* <4=> Debug       */
	#define TL_CONFIG_LOG_LEVEL		4
	
			/* <0=> Default 	*/
			/* <1=> Black       */
			/* <2=> Red         */
			/* <3=> Green       */
			/* <4=> Yellow      */
			/* <5=> Blue        */
			/* <6=> Magenta     */
			/* <7=> Cyan        */
			/* <8=> White 	    */
	#define TL_CONFIG_INFO_COLOR	5
	#define TL_CONFIG_DEBUG_COLOR	5	
#endif

#define NRF_LOG_DEFAULT_OFF			0
#define NRF_LOG_DEFAULT_ERROR		1
#define NRF_LOG_DEFAULT_WARNING		2
#define NRF_LOG_DEFAULT_INFO		3
#define NRF_LOG_DEFAULT_DEBUG		4

#undef  NRF_LOG_DEFAULT_LEVEL
#define NRF_LOG_DEFAULT_LEVEL		NRF_LOG_DEFAULT_WARNING
/*-----------------------------------------------------------*/

#define	portTICKS_PER_10MSEC		( configTICK_RATE_HZ / 102 )
#define	portTICKS_PER_100MSEC		( configTICK_RATE_HZ / 10 )
#define	portTICKS_PER_SEC			( configTICK_RATE_HZ )
#define	portTICKS_PER_MIN			( 60 * portTICKS_PER_SEC )

/* Task stack sizes in words (32-bit). The idle task and timer stack sizes are defined in FreeRTOSConfig.h */
#define bleAdHandlerSTACK_SIZE		( ( unsigned short ) 512 )
#define bleSTACK_SIZE				( ( unsigned short ) 256 )
#define chrgSTACK_SIZE				( ( unsigned short ) 256 )
#define ctrlSTACK_SIZE				( ( unsigned short ) 256 )
#define gsmSTACK_SIZE				( ( unsigned short ) 256 )
#define gpsSTACK_SIZE				( ( unsigned short ) 512 )
#define parserSTACK_SIZE			( ( unsigned short ) 256 )

/* Task priority definitions. */
#define SD_TASK_PRIORITY			( tskIDLE_PRIORITY + 5 )
#define PARSER_TASK_PRIORITY        ( tskIDLE_PRIORITY + 4 )
#define BLE_ADHANDLER_PRIORITY		( tskIDLE_PRIORITY + 3 )
#define CHRG_TASK_PRIORITY          ( tskIDLE_PRIORITY + 1 )
#define CTRL_TASK_PRIORITY          ( tskIDLE_PRIORITY + 2 )
#define BLE_CTRL_PRIORITY			( tskIDLE_PRIORITY + 1 )
#define GPS_TASK_PRIORITY           ( tskIDLE_PRIORITY + 1 )
#define GSM_TASK_PRIORITY           ( tskIDLE_PRIORITY + 1 )

/* Task tags for debugging. */
#define GSM_TASK_TAG				( 1 )
#define GPS_TASK_TAG				( 2 )
#define BLE_CTRL_TASK_TAG			( 3 )
#define CTRL_TASK_TAG				( 4 )
#define BLE_AD_TASK_TAG				( 5 )
#define PAR_TASK_TAG				( 6 )
#define SD_TASK_TAG					( 7 )
#define TMR_TASK_TAG				( 8 )						/* ATTENTION: Set hard in timers.c! */
#define CHRG_TASK_TAG				( 9 )

/* Highest application-related HW-interrupt priority. All higher interrupts (lower number)
   belong to the Softdevice. */
#define	HIGHEST_APP_IRQ_PRIORITY	( 5 )

/* Timer priority is tskIDLE_PRIORITY + 6 ! Defined in FreeRTOSConfig.h.
   configMAX_PRIORITIES  defined in FreeRTOSConfig.h. */

/* Enable trace via UART. */
#define OS_TRACE

/* Enable sending the trace output to file. */
#define FILE_LOG
/*-----------------------------------------------------------*/

enum xMEM_TYPE
{
	ROM,				/* Flash memory. */
	RAM,				/* RAM */
	NVDS				/* NVDS */
};

/* Flags for some data copy routines in different modules to translate apostrophes to quotation marks. */
#define TRANSLATE_APOSTR			( true )
#define NO_TRANSLATE_CHR			( false )

/* Access to peripheral does not use mutexes as the OS has not yet booted. */
#define	 UNPROTECTED				( false )				
#define	 PROTECTED					( true )

/* Helper macros to print defines during compile time. */
#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "="  VALUE(var)
/*-----------------------------------------------------------*/

#endif