/*
 * Tracker firmware.
 */

/* Standard include files. */
#include <stdbool.h>
#include <stdint.h>

/* Project include files. */
#include "tracker.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

/* nRF specific include files. */
#include "nrf52840_bitfields.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "hardfault.h"
#include "sdk_errors.h"
#include "app_error.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_rng.h"
#include "nrf_drv_wdt.h"
#include "nrf_power.h"
#include "nrf_delay.h"

#if defined( SOFTDEVICE_PRESENT ) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#endif


#define NRF_LOG_MODULE_NAME 		MAIN
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* Device specific include files. */
#include "drv_accelerometer.h"
#include "drv_adc.h"
#include "drv_gps.h"
#include "drv_led.h"
#include "drv_nvm.h"
#include "drv_uart.h"
#include "drv_pwm.h"
#include "drv_vibrator.h"

#include "ble_adreport.h"
#include "ble_cmd.h"
#include "ble_ctrl.h"
#include "ble_main.h"
#include "charger.h"
#include "config.h"
#include "ctrl.h"
#include "evacuation.h"
#include "gps.h"
#include "gsm.h"
#include "main.h"
#include "motiondet.h"
#include "rtc.h"
#include "trace.h"
#include "tracemsg.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */

/* Check for a bad firmware update. */
static void vCheckBadFWUpdate( void );

/* Set the bad boot counter in the system error record to 0. */
void vResetBadBootCounter( void );

/* Watchdog event handler. */
static void vWdtEventHandler( void );

/* Application error handler. */
void app_error_fault_handler( uint32_t uiId, uint32_t uiPc, uint32_t uiInfo );
/*-----------------------------------------------------------*/

/* Global variables. */

/* Watchdog channel ID. Needs to be used when feeding the watchdog. */ 
nrf_drv_wdt_channel_id 		xWdtChannelId;

/* Persistent memory section containing variables pertaining to the watchdog and software resets.
   On a WD reset, the HW reset is delayed by 2 32kHz clock cycles and an interrupt is executed. The IRQ handler 
   pushes the return address in the persistent memory which allows to see which code section caused the watchdog 
   reset. */
long __attribute__( ( section( ".non_init" ) ) ) 				ulErrorId;
long __attribute__( ( section( ".non_init" ) ) ) 				ulErrorPc;
struct xERROR_INFO __attribute__( ( section( ".non_init" ) ) )	xErrorInfo;
uint32_t __attribute__( ( section( ".non_init" ) ) ) 			uiErrorStack[ ERROR_STACK_DUMP_LEN ];

/* Flag telling that the system is inside the fatal error handler. */
bool						bInFatalErrorHandler = false;
/*-----------------------------------------------------------*/

int main( void )
{
    ret_code_t 				xErrCode;
    nrf_drv_wdt_config_t 	xWdtConfig = NRF_DRV_WDT_DEAFULT_CONFIG;
	
    xErrCode = NRF_LOG_INIT( NULL );
    APP_ERROR_CHECK( xErrCode );

    NRF_LOG_DEFAULT_BACKENDS_INIT();
	
    /* Initialize clock driver for better time accuracy in FREERTOS */
    xErrCode = nrf_drv_clock_init();
    APP_ERROR_CHECK( xErrCode );
	
	/* Apply work-around for nRF52840 erratum 248:
	   A high-speed peripheral (CPU for example) accesses a RAM block which is being accessed by a low-speed peripheral through the DMA bus, with 
	   a specific timing, and the high-speed peripheral has higher priority than the low-speed peripheral. 
	   This causes an extra current consumption in the range of 350 µA when in System On Idle.
	   Workaround consequences: Up to 40 µA current increase when the 16 MHz clock is used. */
	*( volatile uint32_t * )0x4007AC84ul = 0x00000002ul;
	
	/* Add a 2s delay before enabling the watchdog. This makes sure that the battery's PCM gets enabled before consuming more than the trickle charge
	   current. Else the device would boot on the charge current only w/o battery support and crash as soon as significantly more than the charge 
	   current is required, e.g. during cellular module start-up. */
	nrf_delay_ms( 2000 );
	
    /* Configure watchdog. 8 watchdog channels are available. For the time being only
	   one is used and fed in the RTOS scheduler.
	   TODO: Use one watchdog channel per task so that the watchdog barks even if only
	   one task crashes. */
    xErrCode = nrf_drv_wdt_init( &xWdtConfig, vWdtEventHandler );
    APP_ERROR_CHECK( xErrCode );
    xErrCode = nrf_drv_wdt_channel_alloc( &xWdtChannelId );
    APP_ERROR_CHECK( xErrCode );
    nrf_drv_wdt_enable();	
	
	/* See, if the NFC pin have not yet been configured as GPIO. If not, configure them and reset. */
	if (   ( NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk )
		== ( UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos ) ) 
	{
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
		while ( NRF_NVMC->READY == NVMC_READY_READY_Busy ) 
		{
			;
		}

		NRF_UICR->NFCPINS &= ~UICR_NFCPINS_PROTECT_Msk;
		while ( NRF_NVMC->READY == NVMC_READY_READY_Busy ) 
		{
			;
		}

		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
		while ( NRF_NVMC->READY == NVMC_READY_READY_Busy ) 
		{
			;
		}

		NVIC_SystemReset();
	}
	
	#if defined ( DEBUG_GPIO )
		nrf_gpio_cfg_output( DEBUG0 );
		nrf_gpio_cfg_output( DEBUG1 );
		nrf_gpio_cfg_output( DEBUG2 );
		nrf_gpio_cfg_output( DEBUG3 );
		nrf_gpio_cfg_output( DEBUG4 );
		nrf_gpio_cfg_output( DEBUG5 );
		nrf_gpio_cfg_output( DEBUG6 );
		nrf_gpio_cfg_output( DEBUG7 );
		nrf_gpio_pin_clear( DEBUG0 );
		nrf_gpio_pin_clear( DEBUG1 );
		nrf_gpio_pin_clear( DEBUG2 );
		nrf_gpio_pin_clear( DEBUG3 );
		nrf_gpio_pin_clear( DEBUG4 );
		nrf_gpio_pin_clear( DEBUG5 );
		nrf_gpio_pin_clear( DEBUG6 );
		nrf_gpio_pin_clear( DEBUG7 );
	#endif
	
	/* Initialise NVDS access. */
	vNVDSInit();
	
	/* Read current state of the reset counters and trace information from the Flash storage. */
	vReadSystemErrorRecord( &uxErrorRecord, FSTORAGE_NVMC );

	/* Increment reset counter in NVDS. Requires DRV_NVM to be initialised. */
	vIncrementResetCount();

	/* Check if the firmware has been updated but the update did not work. The most likely result is in a repetitive HW reset. 
	   This function must run BEFORE the SD is initialised. */
	vCheckBadFWUpdate();	
	
	/* Initialise BLE_MAIN and create the softdevice task SD. */
	vBleMainInit();
	
	/* Initialise ADC peripheral. */
	vADCInit();

	/* Initialise accelerometer HW and driver. 
	   Requires vADCInit(). */
	vAccInit();
	
	/* Initialise the crypto engine. */
	vAESInit();

	/* Initialise the TWI interface for GPS access. */
	vGpsDrvInit();
	
	/* Initialise serial interface. */
    vUartPortInit();
	
	/* Initialise the PWM driver for the charge current control. */
	vPWMInit();

	/* Initialise the charger task. The ADC needs to be initialised before. 
	   The charger init also initialises the state of the battery voltage and temperature as well as the on-charger state. */
	vChargerInit( CHRG_TASK_PRIORITY);

	/* Initialise the BLE AD report task. */
	vBleAdReportInit( BLE_ADHANDLER_PRIORITY );
	
    /* Initialise the CTRL task. 
	   Also initialises CONFIG. */
    vCtrlInit( CTRL_TASK_PRIORITY );

	/* Initialise the BLE control task. */
	vBleCtrlInit( BLE_CTRL_PRIORITY );

	/* Initialize the BLE command handler. */
	vInitBleCmdHandler();	

	/* Initialise the GPS task. */
	vGpsInit( GPS_TASK_PRIORITY );
	
	/* Initialise the real-time clock. */
	vRTCInit();

	/* Initialise the evacuation module. */
	vEvacuationInit();

	/* Initialise the GSM task. Requires the RTC to be initialised. */
	vGsmInit( GSM_TASK_PRIORITY );
	
	/* Initialise the LEDs. */
	vLedInit();

	/* Initialise motion detection. */
	vMDInit();
	
	/* Initialise the AT interface parser task. */
	vParserInit( PARSER_TASK_PRIORITY );

	/* Initialise the vibration motor driver. */
	vVibratorInit();
	
	/* Configure power system (enable VDDH DC/DC converter). Note that this function requires the softdevice. */
	#if !defined ( NO_SOFTDEVICE )	
		xErrCode  = sd_power_dcdc0_mode_set( NRF_POWER_DCDC_ENABLE  );
		APP_ERROR_CHECK( xErrCode );
	#endif

    /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Start FreeRTOS scheduler. 
	
	   Once the CTRL task is scheduled for the first time, the CONFIG module is initialised and the configuration record is
	   created, unless it existst already. 
	   All other tasks wait until CONFIG is available. */
	NRF_LOG_INFO( "Starting scheduler." );	
	NRF_LOG_FLUSH();
    vTaskStartScheduler();

    while (true)
    {
        /* FreeRTOS should not be here... FreeRTOS goes back to the start of stack
         * in vTaskStartScheduler function. */
        configASSERT( false );
    }
}
/*-----------------------------------------------------------*/

/* Check for a bad firmware update. This is characterised (by this definition) by:
	- The xEEPROM.bFWUpdated is set
	- The total reset count is high. 
	
   In this case, set the FW update flags to signal the bootloader to roll back the FW and reset.
   
   Note that this function is only called before enabling the softdevice. Consequently, the nrf_ SDK calls have to be used.
*/
static void vCheckBadFWUpdate( void )
{
	/* Check if the FW has been updated. If this is the case, erase the configuration record as its information may be invalid. */
	if ( nrf_power_gpregret2_get() & ( BL_FW_UPDATED | BL_FW_ROLLED_BACK ) )
	{
		/* Invalidate the NVDS to make sure the original configuration record is loaded from the
		   image as the current configuration may be bad. */
		vEmergencyEraseConfigRecord();
		/* Also fully erase the trace buffer. */
		vEraseTraceBuffer();
	}
	
	if ( uxErrorRecord.ulUnsuccessfulBootCount > BAD_FW_RESET_CNT )
	{
		/* We have a new FW but the FW has not been declared good afer several resets.
		   Roll-back the FW to a previous state. */
		/* Set the flags. */
		nrf_power_gpregret_set( BOOTLOADER_DFU_START );
		nrf_power_gpregret2_set( BL_NEW_FW_NOT_FUNCTIONAL );
		
		/* Update the trace. Set usLogEnable to true as the config record has not yet been read. */
		xNvdsConfig.usLogEnable = true;
		vTracePrint( TRACE_FWNOTFUNCTIONAL, TRACE_FILE );
		
		/* Invalidate the NVDS to make sure the original configuration record is loaded from the
		   image as the current configuration may be bad. */
		vEmergencyEraseConfigRecord();
		
		/* Set the failed boot counter to 0 as the FW is being rolled back. */
		uxErrorRecord.ulUnsuccessfulBootCount = 0;
		vEmergencyWriteSystemErrorRecord( &uxErrorRecord );

		/* Reset to get the bootloader running. */
		V_SYSTEM_RESET( RESET_BAD_FW_UPDATE );
	}
}
/*-----------------------------------------------------------*/

/* Set the bad boot counter to 0 once it is fairly sure that the current FW is working correctly. */
void vResetBadBootCounter( void ) 
{
	if ( uxErrorRecord.ulUnsuccessfulBootCount != 0 )
	{
		uxErrorRecord.ulUnsuccessfulBootCount = 0;
		
		/* Write the system error record back to Flash memory. */
		vWriteSystemErrorRecord( &uxErrorRecord, FSTORAGE_SD );
	}
}
/*-----------------------------------------------------------*/

/* Watchdog event handler.

   This handler is called in case of a watchdog time-out.
*/
static void vWdtEventHandler( void )
{
	unsigned long		*pulProcessStackPointer;
	
	/* Look for the originating address on the process stack. 
	   Note that this fuction is called in interrupt context so that the
	   current stack pointer the is main stack pointer MSP. */
	pulProcessStackPointer =  ( unsigned long * )__get_PSP();
	
	ulErrorPc = *( pulProcessStackPointer + 6 );
	
	/* Force a system reset. */
	configASSERT( false );
}
/*-----------------------------------------------------------*/

/* Application error handler. 

   Parameters:
		uint32_t uiId			NRF_FAULT_ID_SD_ASSERT    	(NRF_FAULT_ID_SD_RANGE_START + 1) = 0x00000001
										SoftDevice assertion.
										uiInfo reserved for future use.
								NRF_FAULT_ID_APP_MEMACC   	(NRF_FAULT_ID_APP_RANGE_START + 1) = 0x00001001
										Application invalid memory access.
										uiInfo = 0:			SoftDevice RAM access violation
										else:				SoftDevice peripheral register violation 
															The info parameter contains the sub-region number of PREGION[0], on whose address 
															range the disallowed write access caused the memory access fault.
								NRF_FAULT_ID_SDK_ERROR    	(NRF_FAULT_ID_SDK_RANGE_START + 1) = 0x00004001
										call to ASSERT. The info parameter is a pointer to an assert_info_t variable.
								NRF_FAULT_ID_SDK_ASSERT   	(NRF_FAULT_ID_SDK_RANGE_START + 2) = 0x00004002
										call to APP_ERROR_CHECK or APP_ERROR_CHECK_BOOL. The info parameter is a pointer to an error_info_t variable. 
								SYSTEM_RESET_ID				= 0xF0000000
															FreeRTOS system reset due to an ASSERT.
		uint32_t uiPc			The program counter of the instruction that triggered the fault, or 0 if unavailable. 
		uint32_t uiInfo			see uiId
		
		assert_info_t:			uint32_t 		line_num		The line number where the error occurred.
								uint8_t const 	*p_file_name	The file in which the error occurred.
								
		error_info_t:			uint32_t 		line_num 		The line number where the error occurred.
								uint8_t const 	*p_file_name    The file in which the error occurred.
								uint32_t 		err_code		The error code representing the error that occurred. 		
*/
void app_error_fault_handler( uint32_t uiId, uint32_t uiPc, uint32_t uiInfo )
{
	unsigned portBASE_TYPE		uxIdx;
	uint32_t					*puiSP;
	
    __disable_irq();
	
	if ( bInFatalErrorHandler )
	{
		/* The system is was already inside the fatal error handler. 
		   Now it is the error handler itself which caused another error. 
		   In that case, there is not much what can be done more.
		   Just gracefully reset the system. */
	    NRF_BREAKPOINT_COND;
	
	    /* On assert, the system can only recover with a reset. */
		#ifndef DEBUG
			NRF_LOG_WARNING( "%i System reset", ulReadRTC() );
			NVIC_SystemReset();
		#else
			app_error_save_and_stop( uiId, uiPc, uiInfo );
		#endif
	}

	bInFatalErrorHandler = true;
	
	/* Increment the error counters. */
	switch ( uiId )
	{
		#if defined( SOFTDEVICE_PRESENT ) && SOFTDEVICE_PRESENT
		
			case NRF_FAULT_ID_SD_ASSERT:
				uxErrorRecord.ulSDAssertCount++;
				break;
				
			case NRF_FAULT_ID_APP_MEMACC:
				uxErrorRecord.ulSDMemAccCount++;
				break;
		#endif
		
		case NRF_FAULT_ID_SDK_ASSERT:
		{
			uxErrorRecord.ulnRFSDKAssertCount++;
			break;
		}
		
		case NRF_FAULT_ID_SDK_ERROR:
		{
			uxErrorRecord.ulnRFSDKErrorCount++;
			break;
		}
		
		case SYSTEM_RESET_ID:
		{
			uxErrorRecord.ulSystemErrorCount++;
			break;
		}		
		
		default:
			uxErrorRecord.ulnRFUnknownErrorCount++;
			break;
	}
	
	/* Save some part of the fault information to non-initialised RAM. */
	ulErrorId = uiId;
	ulErrorPc = uiPc;
	if ( uiId == NRF_FAULT_ID_APP_MEMACC ) 
	{
		xErrorInfo.err_code = uiInfo;
	}
	else
	{
		if ( uiId == NRF_FAULT_ID_SDK_ASSERT ) 
		{
			assert_info_t		*pxAssertInfo = ( assert_info_t * )uiInfo;
			
			xErrorInfo.line_num = pxAssertInfo->line_num;
			xErrorInfo.p_file_name = pxAssertInfo->p_file_name;
			xErrorInfo.err_code = 0;
		}
		else
		{
			if ( uiId == NRF_FAULT_ID_SDK_ERROR ) 
			{
				error_info_t		*pxErrorInfo = ( error_info_t * )uiInfo;
				
				xErrorInfo.line_num = pxErrorInfo->line_num;
				xErrorInfo.p_file_name = pxErrorInfo->p_file_name;
				xErrorInfo.err_code = pxErrorInfo->err_code;
			}
			else
			{
				xErrorInfo.line_num = 0;
				xErrorInfo.p_file_name = 0;
				xErrorInfo.err_code = 0;
			}
		}
	}
	
	/* Save the first entries of the current stack. 
	   The first N entries are locally used variables which do not help to understand the error cause. */
	if ( ( __get_xPSR() & xPSR_ISR_Msk ) == 0 )
	{
		/* In thread context. */
		puiSP = ( uint32_t * )__get_PSP();
	}
	else
	{
		/* In ISR context. */
		puiSP = ( uint32_t * )__get_MSP();
	}
	
	/* Copy the stack. Make sure that the read pointer never goes beyond the physical RAM as this would result in a hard fault. */
	for ( uxIdx = 0; uxIdx < ERROR_STACK_DUMP_LEN; uxIdx++ )
	{
		if (    ( ( puiSP + uxIdx + 5 ) >= ( uint32_t * )RAM_START ) 
			 && ( ( puiSP + uxIdx + 5 ) < ( uint32_t * )RAM_END ) 
		   )
		{
			uiErrorStack[ uxIdx ] = *( puiSP + uxIdx + 5 );
		}
		else
		{
			uiErrorStack[ uxIdx ] = 0xaaaaaaaa;
		}
	}

    NRF_LOG_FINAL_FLUSH();

	#ifndef DEBUG
		NRF_LOG_ERROR( "%i Fatal error", ulReadRTC() );
	#else
		switch ( uiId )
		{
			#if defined( SOFTDEVICE_PRESENT ) && SOFTDEVICE_PRESENT
			
				case NRF_FAULT_ID_SD_ASSERT:
					NRF_LOG_ERROR( "%i SOFTDEVICE: ASSERTION FAILED", ulReadRTC() );
					break;
					
				case NRF_FAULT_ID_APP_MEMACC:
					NRF_LOG_ERROR( "%i SOFTDEVICE: INVALID MEMORY ACCESS", ulReadRTC() );
					break;
			#endif
			
			case NRF_FAULT_ID_SDK_ASSERT:
			{
				assert_info_t * p_info = ( assert_info_t * )uiInfo;
							  
				NRF_LOG_ERROR( "%i ASSERTION FAILED at %s:%u, PC at 0x%08x",
							   ulReadRTC(),
							   p_info->p_file_name,
							   p_info->line_num,
							   uiPc );
				break;
			}
			
			case NRF_FAULT_ID_SDK_ERROR:
			{
				error_info_t * p_info = (error_info_t *)uiInfo;

				NRF_LOG_ERROR( "%i ERROR %u [%s] at %s:%u\r\nPC at 0x%08x",
							   ulReadRTC(),
							   p_info->err_code,
							   nrf_strerror_get( p_info->err_code ),
							   p_info->p_file_name,
							   p_info->line_num,
							   uiPc );
				break;
			}
			
			default:
				NRF_LOG_ERROR( "%i UNKNOWN FAULT at 0x%08X", ulReadRTC(), uiPc );
				break;
		}
		NRF_LOG_ERROR( "%i End of error report", ulReadRTC() );
	#endif

	nrf_delay_ms( 1000 );

    NRF_BREAKPOINT_COND;
	
    /* On assert, the system can only recover with a reset. */
	#ifndef DEBUG
		NVIC_SystemReset();
	#else
		app_error_save_and_stop( uiId, uiPc, uiInfo );
	#endif
}
/*-----------------------------------------------------------*/

/* HardFault exception handler.
  
   Called from the HardFault handler exception. 
  
   The parameter p_stack is a pointer to the stack bottom.
   This pointer might be NULL if the HardFault was called when the main stack was the active stack and a stack overrun is detected.
   In such a situation, the stack pointer is reinitialized to the default position, and the stack content is lost.
*/
 void HardFault_process( HardFault_stack_t * pxStack )
{
	unsigned portBASE_TYPE		uxIdx;
	
	/* Disable all IRQs. This is probably not really necessary as the hardfault exception is already the highest exception. */
    __disable_irq();
	
	if ( bInFatalErrorHandler )
	{
		/* The system is was already inside the fatal error handler. 
		   Now it is the error handler itself which caused another error. 
		   In that case, there is not much what can be done more.
		   Just gracefully reset the system. */
	    NRF_BREAKPOINT_COND;
	
	    /* On assert, the system can only recover with a reset. */
		NVIC_SystemReset();
	}

	bInFatalErrorHandler = true;
	
	uxErrorRecord.ulHardfaultCount++;

	/* Save some part of the fault information to non-initialised RAM. */
	ulErrorId = HARDFAULT_ID;

	/* Read the CPU fault registers (ref. to the Cortex-M4 generic user guide). */
	xErrorInfo.cfsr  = SCB->CFSR;			/* Configurable Fault Status Registers. */
	xErrorInfo.hfsr  = SCB->HFSR;        	/* HardFault Status Register. */
	xErrorInfo.dfsr  = SCB->DFSR;        	/* Debug Fault Status Register. */
	xErrorInfo.mmfar = SCB->MMFAR;       	/* MemManage Fault Address Register. */
	xErrorInfo.bfar  = SCB->BFAR;        	/* BusFault Address Register. */
	xErrorInfo.afsr  = SCB->AFSR;	    	/* Auxiliary Fault Status Register. */
	
	/* Save the first entries of the current stack. 
	   The first N entries are locally used variables which do not help to understand the error cause. */
	if ( pxStack != NULL )
	{
		ulErrorPc      = pxStack->pc;
		xErrorInfo.r0  = pxStack->r0;
		xErrorInfo.r1  = pxStack->r1;
		xErrorInfo.r2  = pxStack->r2;
		xErrorInfo.r3  = pxStack->r3;
		xErrorInfo.r12 = pxStack->r12;
		xErrorInfo.lr  = pxStack->lr;
		xErrorInfo.psr = pxStack->psr;
		
		for ( uxIdx = 0; uxIdx < ERROR_STACK_DUMP_LEN; uxIdx++ )
		{
			if (    ( ( ( uint32_t * )pxStack->psr + uxIdx ) >= ( uint32_t * )RAM_START ) 
				 && ( ( ( uint32_t * )pxStack->psr + uxIdx ) < ( uint32_t * )RAM_END ) 
			   )
			{
				uiErrorStack[ uxIdx ] = *( ( uint32_t * )pxStack->psr + uxIdx );
			}
		}
	}
	else
	{
		/* Stack violation: stack pointer outside stack area. */
		ulErrorPc = 0xcccc3333;
	}

	NRF_LOG_ERROR( "%i Hardfault", ulReadRTC() );
    NRF_LOG_FINAL_FLUSH();
	
    /* On a hardfault, the system can only recover with a reset. */
    NVIC_SystemReset();
}
/*-----------------------------------------------------------*/
