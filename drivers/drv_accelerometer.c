/*
 * Tracker Firmware
 *
 * Accelerometer Driver
 *
 * Based on LIS3DH 3-axis accelerometer
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>

#include "tracker.h"

/* nRF SDK include files. */
#define NRF_LOG_MODULE_NAME 		DRV_ACC
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_sdh.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "nrf_log_ctrl.h"
#include "nrfx_gpiote.h"
#include "nrf_delay.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/* Device specific include files. */
#include "custom_board.h"
#include "drv_adc.h"
#include "drv_accelerometer.h"
#include "ctrl.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Start or restart the accelerometer from scratch. */
void vAccPhyStart( bool bProtectedAccess );

/* Stop the accelerometer. This function also disabled and de-configures the SPI access. */
void vAccPhyStop( bool bProtectedAccess );

/* Accelerometer HW and driver init. */
void vAccInit( void );
void vAccDeviceConfig( void );

/* Disable the accelerometer's I2C. */
void vAccDisableI2C( void );

/* Version of this function not protected by mutexes. This function is only to be called while the OS has not yet booted. */
void vAccDisableI2CUnprot( void );

/* Power-up the accelerometer and set it to Inactive state. */
void vAccPowerUp( void );

/* Shut down the accelerometer. */
void vAccPowerDown( void );

/* Shut-down the accelerometer unprotected, i.e. before OS and any interrupts are running. */
void vAccPowerDownUnprot( void );

/* Restart the accelerometer's FIFO. */
void vAccFifoRestart( void );

/* Disable x and y accelerometer axes. */
void vAccDisXYAxes( void );

/* Suspend servicing the delayed timer 1 interrupt. */
static void vAccSuspendDelayedTimer2Svc( void );

/* Resume servicing the delayed timer 1 interrupt. If the timer expired while servicing was suspended, execute the
   service routine immediately. */
static void vAccResumeDelayedTimer2Svc( void );

/* Enable accelerometer interrupt 1. */
void vAccEnableInt1( unsigned portBASE_TYPE uxIntMask );

/* Set Accelerator activity characteristics on interrupt 1. */
void vAccDisableInt1( void );

/* Set Accelerator activity characteristics on interrupt 1. Version to use from IRQ. */
void vAccDisableInt1FromISR( void );

void vAccSetWUCharacInt1( unsigned short usAccThres, unsigned portBASE_TYPE xDuration );

/* Enable accelerometer interrupt 2. */
void vAccEnableInt2( unsigned portBASE_TYPE uxIntMask );

/* Set Accelerator activity characteristics on interrupt 2. */
void vAccDisableInt2( void );

/* Set Accelerator activity characteristics on interrupt 2. Version to use from IRQ. */
void vAccDisableInt2FromISR( void );

/* Set Accelerator activity characteristics on interrupt 2. */
void vAccSetWUCharacInt2( unsigned short usAccThres, unsigned portBASE_TYPE xDuration );

/* Set Accelerator inactivity characteristics. */
void vAccSetInactCharac( unsigned short usAccThres, unsigned short usDuration );

/* Re-enable accelerometer HW interrupts. */
void vAccEnableHwIrq( void );

/* Lock HW access to the accelerometer.  */
void vAccLockAccess( void );

/* Release HW access to accelerometer. */
void vAccReleaseAccess( void );

/* Read a single 8-bit register. */
unsigned portBASE_TYPE ucAccRead8( unsigned portBASE_TYPE ucRegAddr );

/* Read a single 8-bit register. Unprotected version. */
unsigned portBASE_TYPE ucAccRead8Unprot( unsigned portBASE_TYPE ucAddress );

/* Read a single 8-bit register. Version to use from IRQ. */
unsigned portBASE_TYPE ucAccRead8FromISR( unsigned portBASE_TYPE ucAddress );

/* Read a 16-bit register (two consecutive 8-bit registers). */
short sAccRead16( unsigned portBASE_TYPE ucRegAddr );

/* Read accelerometer X/Y/Z data - unprotected. */
void vReadAccDataUnprot( union xXYZ_DATA *xAccData );

/* Read accelerometer X/Y/Z data. */
void vReadAccData( union xXYZ_DATA *xAccData );

/* Write a single 8-bit register. */
void vAccWrite8( unsigned portBASE_TYPE ucRegAddr, unsigned char ucRegValue );

/* Write a single 8-bit register. Unprotected version. */
void vAccWrite8Unprot( unsigned portBASE_TYPE ucRegAddr, unsigned char ucRegValue );

/* Write a single 8-bit register. Version to use from IRQ. */
void vAccWrite8FromISR( unsigned portBASE_TYPE ucRegAddr, unsigned char ucRegValue );

/* Clear any pending delayed accelerometer interrupts and messages already sent to the CRTL queue. */
void vAccClearInterrupts( void );

/* Accelerometer interrupt routine for accelerometer interrupts INT1 and INT2. */
static void vAccIrqHandler( nrf_drv_gpiote_pin_t xIrqPin, nrf_gpiote_polarity_t xAction );

/* Timers serving as a delayed accelerometer interrupts. */
static void vAccTmrHandler1( TimerHandle_t xTimer );
static void vAccTmrHandler2( TimerHandle_t xTimer );

/* Accelerometer HW access test. */
bool bAccTest( unsigned short *usErrorCode );

/* Accelerometer register dump to memory for debug. */
void vAccDump( void );

/* Dump the ADXL362 registers to a memory array for debug. */
void vAccStatus( struct xACC_STATUS *xAccStatus );
void vAccDumpStatus( void );
/*-----------------------------------------------------------*/

/* Global variables. */

/* Local copy of the accelerometer's interrupt enable bits. Inside an IRQ,
   when reading the INT status, all bits are set, even those which are disabled.
   So the IRQ needs to AND the status register contents with the stored enabled
   bits. */
unsigned char			ucAccIntSrc1;
unsigned char			ucAccIntSrc2;

/* SPI instance. */
static const 			nrf_drv_spi_t xSpi = NRF_DRV_SPI_INSTANCE( SPI_INSTANCE );  		

/* Flag used to indicate that SPI instance completed the transfer. */
static volatile bool 	bSpiXferDone;  

/* Mutex handle to protect accelerometer access. */
SemaphoreHandle_t		xMutexAcc = NULL;

/* Timer handle for the one-shot timer for delayed interrupts from the accelerometer. */
static TimerHandle_t 	xAccDelayedIrq1Timer;
static TimerHandle_t 	xAccDelayedIrq2Timer;

/* Flag indicating that the delayed IRQ timer has expired while the accelerometer access was locked. */
bool 					bDelayedIrqTimer2SvcPending;

/* Flag indicating that servicing the delayed IRQ timer is suspended. */
bool 					bDelayedIrqTimer2SvcSuspended;
/*-----------------------------------------------------------*/

/* Module scope variables. */
/*-----------------------------------------------------------*/



/*-----------------------------------------------------------*
 *															 *
 *      Basic operation										 *
 *															 *
 *-----------------------------------------------------------*/
/*
	The accelerometer driver encapsulates all LIS3DH accesses via SPI.
	
	The accesses are protected with a mutex to make them thread-safe.
	
	The accelerometer can drive one physical interrupt onto which two logical interrupts are 
	multiplexed. The interrupt service routine figures out which interrupt is the source and 
	disables the corresponding interrupt on the LIS3DH.
	
	INT1 interrupts are delayed by 15ms before being indicated as message to the CTRL task. 	
	INT2 interrupts are delayed by 10ms before notifying the CTRL task. 
	Both delays are achieved by using operating system timers in one-shot mode.
	The INT1 delay has to be larger than ACC_DELAYED_IRQ2 to make sure that IRQ2 is served first 
	in case both interrupts have triggered. INT1 will reprogram the accelerometer and destroy the FIFO 
	contents. 
	It does not matter if the timer expires while the INT2 handler is still running. The handler runs 
	in the CTRL task, the same which also handles INT1. So the INT1 handler only runs when the INT2 
	handler has finished and the CTRL task checks again for new messages (and will find CTRL_ACC_WU_DET1 
	and then execute the INT1 handler).
	
	The access to the accelerometer can be locked and released. In locked mode, neither
	accelerometer interrupts nor timer expiries are served so that a foreground process
	can safely manupulate the accelerometer. It had to be made sure that no interrupts or timer
	expiries are lost, though. Any unserved accelerometer interrupts are served as soon as the
	accelerometer access is released. The delayed interrupt timer continues to run in locked mode
	but calling the service routine is reported until the the device is released.
	
	Reconfiguring the accelerometer is somewhat tricky. To avoid that the LIS3DH HW hangs,
	it must be temporarily put in power-down mode. (Note: I have read that somewhere but
	I cannot find the reference anymore.) If the delayed accelerometer interrupt fires during 
	this time, the timer handler is not executed (suspended) but a flag is set to indicate that it
	is pending. When the timer resumes, the flag is checked and the handler is called manually if
	necessary.
	
	A special function is provided which does a full re-initialisation of the accelerometer.
	This includes removing any pending delayed INT2 and messages already sent to the CTRL
	task.
*/


/* SPI user event handler. */
void vSpiEventHandler( nrf_drv_spi_evt_t const * pxEvent,
                       void *                    pvContext)
{
    bSpiXferDone = true;
}

/*-----------------------------------------------------------*
 *															 *
 *      API													 *
 *															 *
 *-----------------------------------------------------------*/
/* Initialise the accelerometer driver. 
   Base initialisation:
		- Sensitivity (range) +/-8g
		- High-pass filter for wake-up detectors 1 and 2
		- Activity and inactivity detection routed to INT1
*/
void vAccInit( void )
{
	ret_code_t 						xErrCode;
	nrfx_gpiote_in_config_t 		xGpioInConfig = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI( true );
	
	/* Create a mutex to protect access to the accelerometer so that it can be used from different tasks. */
	xMutexAcc = xSemaphoreCreateMutex();	
	
	/* Create the one-shot timer for delayed interrupts from the accelerometer. */
	xAccDelayedIrq1Timer = xTimerCreate
							( "ACCDLY",			 					/* Timer name for debug. */
							  ACC_DELAYED_IRQ1,						/* Timer value: delay long enough to make sure the accelerometer resets the interrupt (takes 1/400s). */
							  false,								/* One-shot. */
							  ( void * )0,							/* Init for the ID / expiry count. */
							  vAccTmrHandler1						/* Callback for the delayed accelerometer interrupt. */
							);
							
	/* Create the one-shot timer for delayed interrupts from the accelerometer. */
	xAccDelayedIrq2Timer = xTimerCreate
							( "ACCDLY",			 					/* Timer name for debug. */
							  ACC_DELAYED_IRQ2,						/* Timer value. */
							  false,								/* One-shot. */
							  ( void * )0,							/* Init for the ID / expiry count. */
							  vAccTmrHandler2						/* Callback for the delayed accelerometer interrupt. */
							);
							
	bDelayedIrqTimer2SvcPending	= false;
	bDelayedIrqTimer2SvcSuspended = false;					
							
	/* Initialise the GPIOTE module. */
    xErrCode = nrfx_gpiote_init();
    APP_ERROR_CHECK( xErrCode );

    xGpioInConfig.pull = NRF_GPIO_PIN_PULLDOWN;
    xErrCode = nrfx_gpiote_in_init( ACC_INT1, &xGpioInConfig, vAccIrqHandler );
    APP_ERROR_CHECK( xErrCode );							

	vAccPhyStart( UNPROTECTED );

	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/
	
/* Start or restart the accelerometer from scratch. */
void vAccPhyStart( bool bProtectedAccess )
{
	nrf_drv_spi_config_t 		xSpiConfig = NRF_DRV_SPI_DEFAULT_CONFIG;

	if ( bProtectedAccess )
	{
		/* Block accelerometer HW. */
		vAccLockAccess();
	}
	
	/* Remove accelerometer power supply and set all SPI pins to output and low. */
	nrf_gpio_cfg_output( SPI_CS );
	nrf_gpio_cfg_output( SPI_MOSI );
	nrf_gpio_cfg_output( SPI_SCK );

	nrf_gpio_pin_clear( SPI_CS );
	nrf_gpio_pin_clear( SPI_MOSI );
	nrf_gpio_pin_clear( SPI_SCK );
	
	/* Set INT1 and MISO to input. */
	nrf_gpio_cfg_input( ACC_INT1, NRF_GPIO_PIN_PULLDOWN );
	nrf_gpio_cfg_input( SPI_MISO, NRF_GPIO_PIN_PULLDOWN );
	
	nrf_delay_ms( 30 );					/* Make sure the VCC gets down to 0: ~30ms delay. */
	
	/* Check VCC voltage. Make sure that VCC is at least 83% of nominal VCC. Else, the accelerometer might not boot
	   correctly. */
	while ( uiReadADC( VCC_ADC, bProtectedAccess ) < LIS3DH_MIN_VCC )
	{
		;
	}
									   
	/* Start the accelerometer. The accelerometer's VCC is connected to a GPIO to control its internal reset. */
	nrf_gpio_pin_clear( SPI_CS );
	nrf_gpio_pin_clear( SPI_MOSI );
	nrf_gpio_pin_clear( SPI_SCK );
	nrf_delay_ms( LIS3DH_BOOT_DELAY );	/* Give the accelerometer some time to boot from its Flash memory: 100ms. */
	
	/* Enable the SPI interface. COnfiguration:
			SPI uses double clock
	        SPI enabled
	        MSB first, Master
	        Mode3
	        Clk/2 (2MHz / 4 = 500kHz). */
	/* Default:
			.irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY
			.orc          = 0xFF
			.frequency    = NRF_DRV_SPI_FREQ_4M
			.mode         = NRF_DRV_SPI_MODE_0
			.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST */
    xSpiConfig.ss_pin   	= SPI_CS;
    xSpiConfig.miso_pin 	= SPI_MISO;
    xSpiConfig.mosi_pin 	= SPI_MOSI;
    xSpiConfig.sck_pin  	= SPI_SCK;
	xSpiConfig.frequency    = NRF_DRV_SPI_FREQ_500K;
	xSpiConfig.mode 		= NRF_DRV_SPI_MODE_3;
    APP_ERROR_CHECK( nrf_drv_spi_init( &xSpi, &xSpiConfig, vSpiEventHandler, NULL ) );	
	
	/* Add pull-up to INT1 and configure interrupts to rising edge. */
	nrf_gpio_cfg_input( ACC_INT1, NRF_GPIO_PIN_PULLUP );
	nrfx_gpiote_in_event_enable( ACC_INT1, true );
		
	if ( bProtectedAccess )
	{
		/* Release accelerometer HW. */
		vAccReleaseAccess();
		/* The accesses necessary for disabling I2C will protect themselves. */
		vAccDisableI2C();
	}
	else
	{
		vAccDisableI2CUnprot();
	}
}
/*-----------------------------------------------------------*/
 
/* Stop the accelerometer. This function also disabled and de-configures the SPI access. */
void vAccPhyStop( bool bProtectedAccess )
{
	nrf_drv_spi_config_t 		xSpiConfig = NRF_DRV_SPI_DEFAULT_CONFIG;

	if ( bProtectedAccess )
	{
		/* Block accelerometer HW. */
		vAccLockAccess();
	}
	
    nrf_drv_spi_uninit( &xSpi );	

	/* Set all SPI pins to output and low. */
	nrf_gpio_cfg_output( SPI_CS );
	nrf_gpio_cfg_output( SPI_MOSI );
	nrf_gpio_cfg_output( SPI_SCK );

	nrf_gpio_pin_set( SPI_CS );
	nrf_gpio_pin_clear( SPI_MOSI );
	nrf_gpio_pin_clear( SPI_SCK );
	
	/* Set INT1 and MISO to input. */
	nrf_gpio_cfg_input( ACC_INT1, NRF_GPIO_PIN_PULLDOWN );
	nrf_gpio_cfg_input( SPI_MISO, NRF_GPIO_PIN_PULLDOWN );
	
	if ( bProtectedAccess )
	{
		/* Release accelerometer HW. */
		xSemaphoreGive( xMutexAcc );													
	}
}
/*-----------------------------------------------------------*/

/* Configuration accelerometer device to base config. */
void vAccDeviceConfig( void )
{
	/* Disable I2C - just to be sure. */
	vAccDisableI2C();
	
	/* Disable interrupts for the time being. */
	vAccWrite8( LIS3DH_INT1_CFG, 0b00000000 );		
	vAccWrite8( LIS3DH_INT2_CFG, 0b00000000 );		
	
	/* Configure HPF. */
	vAccWrite8( LIS3DH_CTRL_REG2, LIS3DH_HPM_NORM | LIS3DH_HPCF_1HZ | LIS3DH_FDS_BYP | LIS3DH_HPIA1 | LIS3DH_HPIA2 );		
						
	/* Map AOI1 and AOI2 to INT1 pin. */
	vAccWrite8( LIS3DH_CTRL_REG3, LIS3DH_I1_AOI1 | LIS3DH_I1_AOI2 );		
						
	/* 8g full-scale. */
	vAccWrite8( LIS3DH_CTRL_REG4, LIS3DH_BDU | LIS3DH_FS_8G | LIS3DH_HR );	

	/* Latch interrupt requests on INT1 and INT2. */
	vAccWrite8( LIS3DH_CTRL_REG5, LIS3DH_LIR_INT1 | LIS3DH_LIR_INT2 );	

	/* Configure the stop event in FIFO bypass mode to INT2. */
	vAccWrite8( LIS3DH_FIFO_CTRL_REG, LIS3DH_BYPASS | LIS3DH_TRG_ON_INT2 );	
}
/*-----------------------------------------------------------*/

/* Disable the accelerometer's I2C. */
void vAccDisableI2C( void )
{
	unsigned portBASE_TYPE			accReg;
	
	/* Make sure the LIS3DH accelerometer's I2C interface is disabled as soon as possible. Else,
	   programming the processor via SPI will corrupt the LIS3DH's calibration data.
	   The LIS3DH CS input is normally used to select either I2C or SPI interface. Tying the input
	   high by a pull-up sets the interface to I2C. Then, the programming data sent to the processor 
	   might be interpreted as I2C accesses and do anything with the LIS3DH registers. 
	   In future revisions of the hardware, this needs to be addressed differently. 
	   
	   The procedure to disable the I2C can be found in the ST forum in a post dated  7/8/2016 5:03 PM,
	   topic Accelerometers:Use LIS3DH on the same SPI bus as an NVDS, is it possible?:
	   https://my.st.com/public/STe2ecommunities/mems_sensors/Lists/Accelerometers/Flat.aspx?RootFolder=/public/STe2ecommunities/mems_sensors/Lists/Accelerometers/Use%20LIS3DH%20on%20the%20same%20SPI%20bus%20as%20an%20NVDS,%20is%20it%20possible&currentviews=321
	   */
	accReg = ucAccRead8( 0x17 );	
	vAccWrite8( 0x17, accReg | 0x80 );	
}
/*-----------------------------------------------------------*/

/* Version of this function not protected by mutexes. This function is only to be called while the OS has not yet booted. */
void vAccDisableI2CUnprot( void )
{
	unsigned portBASE_TYPE			accReg;
	
	accReg = ucAccRead8Unprot( 0x17 );	
	vAccWrite8Unprot( 0x17, accReg | 0x80 );	
}
/*-----------------------------------------------------------*/

/* Power-up the accelerometer. 
   Base initialisation:
		- Sampling rate 400Hz.
		- All axes enabled.
		- Timer TCC1 interrupt enabled.
*/
void vAccPowerUp( void )
{
	/* Reset LPEN bit. */
	vAccWrite8( LIS3DH_CTRL_REG1,  LIS3DH_ODR400 
								 | LIS3DH_Z_EN
								 | LIS3DH_Y_EN
								 | LIS3DH_X_EN );
								 
	vTaskDelaySafe( 10 );

	/* Re-enable the delayed interrupt timer. if the timer was running when the accelerometer was stopped. The assumption is that 
	   a pending interrupt is executed immediately after having re-enabled it. */
	portENTER_CRITICAL();
	vAccResumeDelayedTimer2Svc();
	portEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

/* Shut-down the accelerometer. */
void vAccPowerDown( void )
{
	/* Power-down mode. */
	vAccWrite8( LIS3DH_CTRL_REG1, 0b00000000 );
	
	/* If the timer for the delayed interrupt is running, disable it. */
	vAccSuspendDelayedTimer2Svc();
}
/*-----------------------------------------------------------*/

/* Shut-down the accelerometer unprotected, i.e. before OS and any interrupts are running. */
void vAccPowerDownUnprot( void )
{
	/* Power-down mode. */
	vAccWrite8Unprot( LIS3DH_CTRL_REG1, 0b00000000 );
	
	/* If the timer for the delayed interrupt is running, disable it. */
	vAccSuspendDelayedTimer2Svc();
}
/*-----------------------------------------------------------*/

/* Restart the accelerometer's FIFO. */
void vAccFifoRestart( void )
{
	/* Set FIFO to bypass mode. */
	vAccWrite8( LIS3DH_FIFO_CTRL_REG, LIS3DH_BYPASS | LIS3DH_TRG_ON_INT2 );
	
	/* Short delay (> 1 accelerometer sample time) to make sure the FIFO had time to reset. */
	vTaskDelaySafe( 1 );
								 
	/* Set FIFO to Stream mode. */
	vAccWrite8( LIS3DH_FIFO_CTRL_REG, LIS3DH_STREAM | LIS3DH_TRG_ON_INT2 );
}
/*-----------------------------------------------------------*/

/* Disable x and y accelerometer axes. */
void vAccDisXYAxes( void )
{
	/* Reset LPEN bit. */
	vAccWrite8( LIS3DH_CTRL_REG1,  LIS3DH_ODR400 
								 | LIS3DH_Z_EN );
								 
	vTaskDelaySafe( 10 );
}
/*-----------------------------------------------------------*/

/* Suspend servicing the delayed timer 1 interrupt. */
static void vAccSuspendDelayedTimer2Svc( void )
{
	/* Set the flag that the timer servicing is suspended. The timer itself continues running, though. */
	bDelayedIrqTimer2SvcSuspended = true;						
}
/*-----------------------------------------------------------*/


/* Resume servicing the delayed timer 1 interrupt. If the timer expired while servicing was suspended, execute the
   service routine immediately. */
static void vAccResumeDelayedTimer2Svc( void )
{
	/* Resume normal timer opation. */
	bDelayedIrqTimer2SvcSuspended = false;					

	/* Serve any pending interrupts. */
	if ( bDelayedIrqTimer2SvcPending )
	{
		vAccTmrHandler2( xAccDelayedIrq2Timer );
	}	
}
/*-----------------------------------------------------------*/

/* Enable accelerometer interrupt 1. Also start the FIFO at the same time as its stop event is INT1. */
void vAccEnableInt1( unsigned portBASE_TYPE uxIntMask )
{
	/* Configure INT1 as wake-up, i.e. OR-combination of the configured events: |X| > thres., |Y| > thres., |Z| > thres. */
	vAccWrite8( LIS3DH_INT1_CFG, uxIntMask );		
	
 	/* Read the INT1 SOURCE register to clear any pending interrupts. The normal lock/release mechanism cannot be used
	   as it checks for pending interrupts and serves them. Here, we only want the interrupt to be cleared but not 
	   served. */	
	configASSERT( xSemaphoreTake( xMutexAcc, TO_MUTEX_ACC ) );
	( void )ucAccRead8Unprot( LIS3DH_INT1_SOURCE );
	xSemaphoreGive( xMutexAcc );													

	/* Wait >2.5ms until the interrupt reset has been taken into account. */
	vTaskDelay( 3 );
	
	/* INT1 interrupt enable. */
	vAccEnableHwIrq();
}
/*-----------------------------------------------------------*/

/* Disable accelerometer interrupt 1. */
void vAccDisableInt1( void )
{
	/* Disable INT1. */
	vAccWrite8( LIS3DH_INT1_CFG, 0 );		

 	/* Read the INT1 SOURCE register to clear any pending interrupts. */	
	( void )ucAccRead8( LIS3DH_INT1_SOURCE );
}
/*-----------------------------------------------------------*/

/* Disable accelerometer interrupt 1. */
void vAccDisableInt1FromISR( void )
{
	/* Disable INT1. */
	vAccWrite8FromISR( LIS3DH_INT1_CFG, 0 );		

 	/* Read the INT1 SOURCE register to clear any pending interrupts. */	
	( void )ucAccRead8FromISR( LIS3DH_INT1_SOURCE );
}
/*-----------------------------------------------------------*/

/* Set Accelerator wake-up characteristics on detector 1. */
void vAccSetWUCharacInt1( unsigned short usAccThres, unsigned portBASE_TYPE xDuration )
{
	vAccWrite8( LIS3DH_INT1_THS, usAccThres & 0x00ff );
	vAccWrite8( LIS3DH_INT1_DURATION, xDuration );
}
/*-----------------------------------------------------------*/

/* Enable accelerometer interrupt 2. 

   The normal lock/release mechanism cannot be used as it checks for pending interrupts and serves them. 
   Here, interrupts are not yet enabled so we do not want the interrupt to be served.
*/
void vAccEnableInt2( unsigned portBASE_TYPE uxIntMask )
{
	configASSERT( xSemaphoreTake( xMutexAcc, TO_MUTEX_ACC ) );

	/* Configure INT2 as wake-up, i.e. OR-combination of the follwing events: |X| > thres., |Y| > thres., |Z| > thres. */
	vAccWrite8Unprot( LIS3DH_INT2_CFG, uxIntMask );		
	
	/* Latch interrupt requests on INT1 and INT2 and enable FIFO. */
	vAccWrite8Unprot( LIS3DH_CTRL_REG5, LIS3DH_FIFO_EN | LIS3DH_LIR_INT1 | LIS3DH_LIR_INT2 );	

 	/* Read the INT2 SOURCE register to clear any pending interrupts. */	
	( void )ucAccRead8Unprot( LIS3DH_INT2_SOURCE );

	/* Wait >2.5ms until the interrupt reset has been taken into account. */
	vTaskDelay( 3 );
	
	/* Set FIFO to Stream mode. */
	vAccWrite8Unprot( LIS3DH_FIFO_CTRL_REG, LIS3DH_STREAM | LIS3DH_TRG_ON_INT2 );	

	xSemaphoreGive( xMutexAcc );													

	/* INT1 interrupt enable on the processor side (INT2 is mapped to the same uC pin). */
	vAccEnableHwIrq();
}
/*-----------------------------------------------------------*/

/* Disable accelerometer interrupt 2. */
void vAccDisableInt2( void )
{
	/* Disable INT2. */
	vAccWrite8FromISR( LIS3DH_INT2_CFG, 0 );		

 	/* Read the INT2 SOURCE register to clear any pending interrupts. */	
	( void )ucAccRead8( LIS3DH_INT2_SOURCE );
}
/*-----------------------------------------------------------*/

/* Disable accelerometer interrupt 2. */
void vAccDisableInt2FromISR( void )
{
	/* Disable INT2. */
	vAccWrite8FromISR( LIS3DH_INT2_CFG, 0 );		

 	/* Read the INT2 SOURCE register to clear any pending interrupts. */	
	( void )ucAccRead8FromISR( LIS3DH_INT2_SOURCE );
}
/*-----------------------------------------------------------*/

/* Set Accelerator wake-up characteristics on detector 2. */
void vAccSetWUCharacInt2( unsigned short usAccThres, unsigned portBASE_TYPE xDuration )
{
	vAccWrite8( LIS3DH_INT2_THS, usAccThres & 0x00ff );
	vAccWrite8( LIS3DH_INT2_DURATION, xDuration );
}
/*-----------------------------------------------------------*/

/* Re-enable accelerometer HW interrupts.
   If any accelerometer interrupts have been missed due to disabled interrupts, call the interrupt routine manually. 
   This will both service the interrupt and clear the source. */
void vAccEnableHwIrq( void )
{														
	portENTER_CRITICAL();								
	nrfx_gpiote_in_event_enable( ACC_INT1, true );	
	portEXIT_CRITICAL(); 								
	if ( nrf_gpio_pin_read( ACC_INT1 ) )				
	{													
		vAccIrqHandler( ACC_INT1, 0 );					
	}													
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*
 *															 *
 *      Accelerometer driver								 *
 *															 *
 *-----------------------------------------------------------*/ 
 
/* Lock HW access to the accelerometer. HW interrupts from the accelerometer are disabled. Pending interrupts are served once
   the normal operation resumes.
   The delayed interrupt timer continues to run. If it expires while the access is locked, servicing it is deferred until
   normal operation is resumed. */
void vAccLockAccess( void )
{															
	configASSERT( xSemaphoreTake( xMutexAcc, TO_MUTEX_ACC ) );
	portENTER_CRITICAL();									
	nrfx_gpiote_in_event_disable( ACC_INT1 );			
	vAccSuspendDelayedTimer2Svc();							
	portEXIT_CRITICAL();									
}
/*-----------------------------------------------------------*/

/* Release HW access to accelerometer. The mutex protecting from access by concurrent tasks is given. The interrupt from the
   GPIO pin receiving the accelerometer INT is re-enabled. The macro checks if there was a pending interrupt. If yes, it calls 
   the accelerometer interrupt handler which will clear the interrupt source. 
   The delayed interrupt timer executes pending interrupts while servicing the interrupt is resumed. 
   
   Note that there can only be one pending interrupt. If ACC_INT1 is high at this point (i.e. there was a new interrupt
   while the access was locked), it will remain high even if vAccIrqHandler() ran. Even if there is a new interrupt source,
   it will be detected from the protected accelerometer accesses once the delayed timers 1 and/or 2 launched inside the handler 
   expire. */
void vAccReleaseAccess( void )
{																					
	xSemaphoreGive( xMutexAcc );													
	vAccResumeDelayedTimer2Svc();													
	
	if ( nrf_gpio_pin_read( ACC_INT1 ) )								
	{								
		/* The accelerometer sent a new interrupt while the device access was locked.
		   Now assume control of the device and serve the interrupt.
		   Repeat this as long as there are new interrupts pending. */
		configASSERT( xSemaphoreTake( xMutexAcc, TO_MUTEX_ACC ) );
		vAccIrqHandler( ACC_INT1, 0 );												
		xSemaphoreGive( xMutexAcc );
	}																				
	
	portENTER_CRITICAL();															
	nrfx_gpiote_in_event_enable( ACC_INT1, true );								
	portEXIT_CRITICAL(); 															
}
/*-----------------------------------------------------------*/

/* Read an LIS3DH accelerometer register over SPI.
   A read consist of three phases:
		1. send read command with address
		2. read the value (by sending a dummy value)
		The sequencing of the three phases can be done by interrupts/scheduling
		or purely by polling. When using interrupts and scheduling, each phase takes
		one OS tick in the best case (10ms), i.e. 20ms for the entire read. 
		When using polling, the same can be achieved in ~2*8 SPI clock cycles,
		i.e. ~64us for an SPI clock cycle of 1MHz / 4 = 250kHz
		
   Protected version.
*/    
unsigned portBASE_TYPE ucAccRead8( unsigned portBASE_TYPE ucAddress )
{
	unsigned portBASE_TYPE 	ucReadVal;
	
	vAccLockAccess();									/* Lock accelerometer HW. */
	
	ucReadVal = ucAccRead8Unprot( ucAddress );
	
	vAccReleaseAccess();								/* Release accelerometer HW. */
	
	return ucReadVal;
}
/*-----------------------------------------------------------*/

/* Unprotected version. */	
unsigned portBASE_TYPE ucAccRead8Unprot( unsigned portBASE_TYPE ucAddress )
{
	unsigned char			ucSpiTxBuffer[ 2 ];
	unsigned char			ucSpiRxBuffer[ 2 ];
	unsigned portBASE_TYPE	uxLoopCnt;
	ret_code_t				xErrCode;
	
	/* Configure LIS3DH read command in the TX buffer and reset the transfer done flag. */
	ucSpiTxBuffer[ 0 ] = LIS3DH_RWN | ucAddress; 	
	ucSpiTxBuffer[ 1 ] = 0;
	bSpiXferDone = false;

	xErrCode = NRF_ERROR_BUSY;
	uxLoopCnt = 0;
	
	while ( ( xErrCode == NRF_ERROR_BUSY ) && ( uxLoopCnt < 10 ) )
	{
		xErrCode = nrf_drv_spi_transfer( &xSpi, ucSpiTxBuffer, 2, ucSpiRxBuffer, 2 );
		nrf_delay_us( 50 );
		uxLoopCnt++;
	}
	APP_ERROR_CHECK( xErrCode );

	uxLoopCnt = 0;
	while ( !bSpiXferDone && ( uxLoopCnt < 10 ) )
	{
		#if !defined ( NO_SOFTDEVICE )
			sd_app_evt_wait();
		#endif
		vTaskDelaySafe( 1 );
		uxLoopCnt++;
	}

	configASSERT( uxLoopCnt < 10 );

	return ucSpiRxBuffer[ 1 ];
}
/*-----------------------------------------------------------*/

/* ISR version. */	
unsigned portBASE_TYPE ucAccRead8FromISR( unsigned portBASE_TYPE ucAddress )
{
	unsigned char			ucSpiTxBuffer[ 2 ];
	unsigned char			ucSpiRxBuffer[ 2 ];
	unsigned portBASE_TYPE	uxLoopCnt;
	ret_code_t				xErrCode;
	
	/* Configure LIS3DH read command in the TX buffer and reset the transfer done flag. */
	ucSpiTxBuffer[ 0 ] = LIS3DH_RWN | ucAddress; 	
	ucSpiTxBuffer[ 1 ] = 0;
	bSpiXferDone = false;
	
	xErrCode = NRF_ERROR_BUSY;
	uxLoopCnt = 0;

	while ( ( xErrCode == NRF_ERROR_BUSY ) && ( uxLoopCnt < 10 ) )
	{
		xErrCode = nrf_drv_spi_transfer( &xSpi, ucSpiTxBuffer, 2, ucSpiRxBuffer, 2 );
		nrf_delay_us( 50 );
		uxLoopCnt++;
	}
	APP_ERROR_CHECK( xErrCode );

	uxLoopCnt = 0;
	while ( !bSpiXferDone && ( uxLoopCnt < 10 ) )
	{
		nrf_delay_us( 50 );
		uxLoopCnt++;
	}

	configASSERT( uxLoopCnt < 10 );

	return ucSpiRxBuffer[ 1 ];
}
/*-----------------------------------------------------------*/

/* Read two consecutive LIS3DH accelerometer registers over SPI.
   The first register read is returned as LSB, the next as MSB.
   Note: A double read consist of four phases:
		1. send read command with address
		2. read the first value (by sending a dummy value)
		3. read the second value (by sending a dummy value)
*/
short sAccRead16( unsigned portBASE_TYPE ucAddress )
{
	unsigned char			ucSpiTxBuffer[ 3 ];
	unsigned char			ucSpiRxBuffer[ 3 ];
	unsigned portBASE_TYPE	uxLoopCnt;
	ret_code_t				xErrCode;
	
	vAccLockAccess();									/* Lock accelerometer HW. */

	/* Configure LIS3DH read command in the TX buffer and reset the transfer done flag. */
	memset( ucSpiTxBuffer, 0, 3 );
	ucSpiTxBuffer[ 0 ] =   LIS3DH_RWN
						 | LIS3DH_MULT 
						 | ucAddress; 	
	memset( ucSpiRxBuffer, 0, 3 );
	bSpiXferDone = false;

	xErrCode = NRF_ERROR_BUSY;
	uxLoopCnt = 0;

	while ( ( xErrCode == NRF_ERROR_BUSY ) && ( uxLoopCnt < 10 ) )
	{
		xErrCode = nrf_drv_spi_transfer( &xSpi, ucSpiTxBuffer, 3, ucSpiRxBuffer, 3 );
		nrf_delay_us( 50 );
		uxLoopCnt++;
	}
	APP_ERROR_CHECK( xErrCode );

	while ( !bSpiXferDone )
	{
		#if !defined ( NO_SOFTDEVICE )
			sd_app_evt_wait();
		#endif
		vTaskDelaySafe( 1 );
	}

	vAccReleaseAccess();								/* Release accelerometer HW. */
	
	return ucSpiRxBuffer[ 1 ] + ( ucSpiRxBuffer[ 2 ] << 8 );
}
/*-----------------------------------------------------------*/
	
/* Read X/Y/Z acceleration data registers from Accelerometer via SPI.
   Unprotected version.
*/
void vReadAccDataUnprot( union xXYZ_DATA *xAccData )
{
	unsigned char			ucSpiTxBuffer[ 7 ];
	unsigned char			ucSpiRxBuffer[ 7 ];
	portBASE_TYPE			xRegIdx;
	unsigned portBASE_TYPE	uxLoopCnt;
	ret_code_t				xErrCode;
	
	memset( ucSpiTxBuffer, 0, 7 );
	ucSpiTxBuffer[ 0 ] =   LIS3DH_RWN
						 | LIS3DH_MULT 
						 | LIS3DH_OUT_X_L; 	
	memset( ucSpiRxBuffer, 0, 7 );
	bSpiXferDone = false;

	xErrCode = NRF_ERROR_BUSY;
	uxLoopCnt = 0;

	while ( ( xErrCode == NRF_ERROR_BUSY ) && ( uxLoopCnt < 10 ) )
	{
		xErrCode = nrf_drv_spi_transfer( &xSpi, ucSpiTxBuffer, 7, ucSpiRxBuffer, 7 );
		nrf_delay_us( 50 );
		uxLoopCnt++;
	}
	APP_ERROR_CHECK( xErrCode );

	while ( !bSpiXferDone )
	{
		#if !defined ( NO_SOFTDEVICE )
			sd_app_evt_wait();
		#endif
		vTaskDelaySafe( 1 );
	}

	for ( xRegIdx = 0; xRegIdx < 6; xRegIdx++ )
	{
		*( xAccData->ucAccData + xRegIdx ) = ucSpiRxBuffer[ 1 + xRegIdx ];
	}
}
/*-----------------------------------------------------------*/

/* Read X/Y/Z acceleration data registers from Accelerometer via SPI
*/
void vReadAccData( union xXYZ_DATA *xAccData )
{
	portBASE_TYPE			xRegIdx;
	
	vAccLockAccess();									/* Lock accelerometer HW. */
	
	vReadAccDataUnprot( xAccData );
	
	vAccReleaseAccess();								/* Release accelerometer HW. */
}
/*-----------------------------------------------------------*/

/* Write an LIS3DH accelerometer register over SPI
   Note: A write consist of three phases:
		1. send write command with address
		2. write the value
*/
void vAccWrite8( unsigned portBASE_TYPE ucAddress, unsigned char ucValue )
{
	vAccLockAccess();									/* Lock accelerometer HW. */
	
	vAccWrite8Unprot( ucAddress, ucValue );
	                                        		
	vAccReleaseAccess();								/* Release accelerometer HW. */
}
/*-----------------------------------------------------------*/

/* Unprotected version. */	
void vAccWrite8Unprot( unsigned portBASE_TYPE ucAddress, unsigned char ucValue )
{
	unsigned char			ucSpiTxBuffer[ 2 ];
	unsigned char			ucSpiRxBuffer[ 2 ];
	unsigned portBASE_TYPE	uxLoopCnt;
	ret_code_t				xErrCode;
	
	/* Configure LIS3DH read command in the TX buffer and reset the transfer done flag. */
	ucSpiTxBuffer[ 0 ] = ucAddress; 	
	ucSpiTxBuffer[ 1 ] = ucValue;
	bSpiXferDone = false;
	
	xErrCode = NRF_ERROR_BUSY;
	uxLoopCnt = 0;

	while ( ( xErrCode == NRF_ERROR_BUSY ) && ( uxLoopCnt < 10 ) )
	{
		xErrCode = nrf_drv_spi_transfer( &xSpi, ucSpiTxBuffer, 2, ucSpiRxBuffer, 2 );
		nrf_delay_us( 50 );
		uxLoopCnt++;
	}
	APP_ERROR_CHECK( xErrCode );

	while ( !bSpiXferDone )
	{
		#if !defined ( NO_SOFTDEVICE )
			sd_app_evt_wait();
		#endif
		vTaskDelaySafe( 1 );
	}
}
/*-----------------------------------------------------------*/

/* ISR version. */	
void vAccWrite8FromISR( unsigned portBASE_TYPE ucAddress, unsigned char ucValue )
{
	unsigned char			ucSpiTxBuffer[ 2 ];
	unsigned char			ucSpiRxBuffer[ 2 ];
	unsigned portBASE_TYPE	uxLoopCnt;
	ret_code_t				xErrCode;
	
	/* Configure LIS3DH read command in the TX buffer and reset the transfer done flag. */
	ucSpiTxBuffer[ 0 ] = ucAddress; 	
	ucSpiTxBuffer[ 1 ] = ucValue;
	bSpiXferDone = false;

	xErrCode = NRF_ERROR_BUSY;
	uxLoopCnt = 0;

	while ( ( xErrCode == NRF_ERROR_BUSY ) && ( uxLoopCnt < 10 ) )
	{
		xErrCode = nrf_drv_spi_transfer( &xSpi, ucSpiTxBuffer, 2, ucSpiRxBuffer, 2 );
		nrf_delay_us( 50 );
		uxLoopCnt++;
	}
	APP_ERROR_CHECK( xErrCode );

	while ( !bSpiXferDone )
	{
		nrf_delay_us( 50 );
	}
}
/*-----------------------------------------------------------*/

/* Clear any pending delayed accelerometer interrupts and messages already sent to the CRTL queue. */
void vAccClearInterrupts( void )
{
	/* Stop accelerometer delayed interrupt. */
	( void )xTimerStop( xAccDelayedIrq2Timer, ACC_TMR_BLOCKTIME );
	bDelayedIrqTimer2SvcPending = false;
	
	/* Remove any messages already sent from the interrupt routines. */
	vRemoveTypeFromQueue( xCtrlEventQueue, CTRL_ACC_WU_DET1 );
	vRemoveTypeFromQueue( xCtrlEventQueue, CTRL_ACC_WU_DET2 );
}
/*-----------------------------------------------------------*/

/* Accelerometer interrupt routine for accelerometer interrupts INT1 and INT2. */
static void vAccIrqHandler( nrf_drv_gpiote_pin_t xIrqPin, nrf_gpiote_polarity_t xAction )
{
	signed portBASE_TYPE 	xHigherPriorityTaskWoken = false;

	traceINTERRUPT_IN();

	/* We got an event from the accelerometer. */
	/* Read the status registers to clear the interrupt. Store the value to allow the handling routine to figure out the 
	   interrupt source and interrupt priorities. */
	ucAccIntSrc1 = ucAccRead8FromISR( LIS3DH_INT1_SOURCE );
	ucAccIntSrc2 = ucAccRead8FromISR( LIS3DH_INT2_SOURCE );

	/* Test for interrupt on wake-up 1 detector. */
	if ( ucAccIntSrc1 & LIS3DH_IA )
	{
		/* Disable further interrupts of the same type until this interrupt has been treated in the upper layers. */
		vAccDisableInt1FromISR();	
		
		/* Program a delayed interrupt.This allows capturing a few more acceleration samples in the FIFO before stopping it. */
		( void )xTimerStartFromISR( xAccDelayedIrq1Timer, &xHigherPriorityTaskWoken );
	}
	
	/* Test for interrupt on wake-up 2 detector. */
	if ( ucAccIntSrc2 & LIS3DH_IA )
	{
		/* Disable further interrupts of the same type until this interrupt has been treated in the upper layers. */
		vAccDisableInt2FromISR();	
		
		/* Program a delayed interrupt.This allows capturing a few more acceleration samples in the FIFO before stopping it. */
		( void )xTimerStartFromISR( xAccDelayedIrq2Timer, &xHigherPriorityTaskWoken );
	}
	
	traceINTERRUPT_OUT();
	
	if( xHigherPriorityTaskWoken != false )
    {
        /* Call the interrupt safe yield function here. */
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }	
}
/*-----------------------------------------------------------*/

/* Timer handler serving as a delayed accelerometer interrupt 1. */
static void vAccTmrHandler1( TimerHandle_t xTimer )
{
	enum xCTRL_EVENT		xCtrlEvent;

	( void )xTimer;
	
	/* Send a corresponding event to the control task which then might reprogram the accelerometer. */
	xCtrlEvent = CTRL_ACC_WU_DET1;

	if( xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 ) != true )
	{
		/* Could not send event to control queue. */
	}
}
/*-----------------------------------------------------------*/

/* Timer handler serving as a delayed accelerometer interrupt 2. 

   NOTE: This interrupt service routine is responsible for freezing the FIFO contents. It has to be precise down to
         one accelerometer clock cycle (= 1/400Hz = 2.5ms). Seen that the RTOS systick is 1/1024Hz = 976us, this should
		 be fulfilled. If, however, there are nevertheless latency issues, consider using a HW timer interrupt.
*/
static void vAccTmrHandler2( TimerHandle_t xTimer )
{
	enum xCTRL_EVENT		xCtrlEvent;

	( void )xTimer;

	if ( !bDelayedIrqTimer2SvcSuspended )
	{
		/* Reset the IRQ2 pending flag at first. Not doing so would retrigger this function with the next accelerometer access. */
		bDelayedIrqTimer2SvcPending = false;

		/* Set the accelerometer to FIFO mode to freeze the FIFO contents (has to be done right in the interrupt routine, else the latency is 
		   too high and the FIFO has advanced). */
		vAccWrite8( LIS3DH_FIFO_CTRL_REG, LIS3DH_FIFO | LIS3DH_TRG_ON_INT2 );
		
		/* Send a corresponding event to the control task which then might reprogram the accelerometer. */
		xCtrlEvent = CTRL_ACC_WU_DET2;

		if( xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 ) != true )
		{
			/* Could not send event to control queue. */
		}
	}
	else
	{
		bDelayedIrqTimer2SvcPending = true;
	}
}
/*-----------------------------------------------------------*/

/* Perform an LIS3DH self test. The test comprises:
	- Register access test.
	- Verification of current X/Y/Z accelerations for reasonable values.
	- Positive and negative acceleration self-test.
*/
bool bAccTest( unsigned short *usErrorCode )
{
	bool					bPassed;
	short					sXAccRef, sYAccRef, sZAccRef;

	/* Prepare test: Remove accelerometer power and restart it. */
	vAccDeviceConfig();
	
	/* Set accelerometer into defined state, i.e. basically everything is off. */
	vAccPowerDown();	
	
	bPassed = true;
	*usErrorCode = ACC_ERR_NONE;
	
	/* 8-bit read test: read device ID AD. */
	if ( ucAccRead8( LIS3DH_WHO_AM_I ) != LIS3DH_WHO_AM_I_VAL )
	{
		bPassed = false;
		*usErrorCode |= ACC_ERR_WHOAMI;
	}
	
	/* 8-bit write tests. */
	vAccWrite8( LIS3DH_REFERENCE, 	0x00 );	
	if ( ucAccRead8( LIS3DH_REFERENCE ) != 0x00 )
	{
		bPassed = false;
		*usErrorCode |= ACC_ERR_WR00;
	}
	vAccWrite8( LIS3DH_REFERENCE, 	0x55 );	
	if ( ucAccRead8( LIS3DH_REFERENCE ) != 0x55 )
	{
		bPassed = false;
		*usErrorCode |= ACC_ERR_WR55;
	}
	vAccWrite8( LIS3DH_REFERENCE, 	0xAA );	
	if ( ucAccRead8( LIS3DH_REFERENCE ) != 0xAA )
	{
		bPassed = false;
		*usErrorCode |= ACC_ERR_WRAA;
	}
	vAccWrite8( LIS3DH_REFERENCE, 	0xFF );	
	if ( ucAccRead8( LIS3DH_REFERENCE ) != 0xFF )
	{
		bPassed = false;
		*usErrorCode |= ACC_ERR_WRFF;
	}

	/* Return register to initial reset value. */
	vAccWrite8( LIS3DH_REFERENCE, 	0x00 );	

	if ( bPassed )
	{
		/* Test the accelerometer static values for abnormalities. */
		/* 8g full-scale. */
		vAccWrite8( LIS3DH_CTRL_REG4, LIS3DH_BDU | LIS3DH_FS_8G | LIS3DH_HR );	
		vAccPowerUp();	
		sXAccRef = sAccRead16( LIS3DH_OUT_X_L );
		sYAccRef = sAccRead16( LIS3DH_OUT_Y_L );
		sZAccRef = sAccRead16( LIS3DH_OUT_Z_L );	
		
		/* 30 degree is the maximum deviation of the horizontal position allowed for this test. */
		if ( abs( sXAccRef ) > ( short )( 32768 / 8 * 0.5 ) )
		{
			bPassed = false;
			*usErrorCode |= ACC_ERR_XABOVE;
		}
		if ( abs( sYAccRef ) > ( short )( 32768 / 8 * 0.5 ) )
		{
			bPassed = false;
			*usErrorCode |= ACC_ERR_YABOVE;
		}
		if ( sZAccRef > ( short )( 32768 / 8 * 1.1 ) )
		{
			bPassed = false;
			*usErrorCode |= ACC_ERR_ZABOVE;
		}
		if ( sZAccRef < ( short )( 32768 / 8 * 0.866 ) )
		{
			bPassed = false;
			*usErrorCode |= ACC_ERR_ZBELOW;
		}
		
		{
			char		cStrg[ 100 ];

			float fAcc = ( float )sXAccRef / 32768 * 8;
			sprintf( cStrg, "self-test reference X acceleration: %d.%04dg", ( signed int )fAcc, ( int )abs( 10000 * ( fAcc - ( int )fAcc ) ) );
			NRF_LOG_INFO( "%s", cStrg );	
			NRF_LOG_FLUSH();
			fAcc = ( float )sYAccRef / 32768 * 8;
			sprintf( cStrg, "self-test reference Y acceleration: %d.%04dg", ( signed int )fAcc, ( int )abs( 10000 * ( fAcc - ( int )fAcc ) ) );
			NRF_LOG_INFO( "%s", cStrg );	
			NRF_LOG_FLUSH();
			fAcc = ( float )sZAccRef / 32768 * 8;
			sprintf( cStrg, "self-test reference Z acceleration: %d.%04dg", ( signed int )fAcc, ( int )abs( 10000 * ( fAcc - ( int )fAcc ) ) );
			NRF_LOG_INFO( "%s", cStrg );	
			NRF_LOG_FLUSH();
		}		

		/* Initialise the accelerometer for self-testing. */
		/* 2g full-scale. */
		vAccWrite8( LIS3DH_CTRL_REG4, LIS3DH_BDU | LIS3DH_FS_2G | LIS3DH_HR );	
		vAccPowerUp();
		/* Get acceleration reference values. */
		sXAccRef = sAccRead16( LIS3DH_OUT_X_L );
		sYAccRef = sAccRead16( LIS3DH_OUT_Y_L );
		sZAccRef = sAccRead16( LIS3DH_OUT_Z_L );
		
		/* Self-test 1 @ 2g max swing. */
		vAccWrite8( LIS3DH_CTRL_REG4, 0b00001010 );				
		/* 100ms delay to let the values stabilise. */
		vTaskDelay( 10 );		
		/* Compare the X/Y/Z deviations to the datasheet limits. */
		if ( sAccRead16( LIS3DH_OUT_X_L ) - sXAccRef < LIS3DH_SELF_TEST_X_DEV )
		{
			bPassed = false;
			*usErrorCode |= ACC_ERR_XTEST1;
		}
		if ( sAccRead16( LIS3DH_OUT_Y_L ) - sYAccRef < LIS3DH_SELF_TEST_Y_DEV )
		{
			bPassed = false;
			*usErrorCode |= ACC_ERR_YTEST1;
		}
		if ( sAccRead16( LIS3DH_OUT_Z_L ) - sZAccRef < LIS3DH_SELF_TEST_Z_DEV )
		{
			bPassed = false;
			*usErrorCode |= ACC_ERR_ZTEST1;
		}
	
		/* Self-test 2 @ 2g max swing. */
		vAccWrite8( LIS3DH_CTRL_REG4, 0b00001100 );				
		/* 100ms delay to let the values stabilise. */
		vTaskDelay( 10 );		
		/* Compare the X/Y/Z deviations to the datasheet limits. */
		if ( sXAccRef - sAccRead16( LIS3DH_OUT_X_L ) < LIS3DH_SELF_TEST_X_DEV )
		{
			bPassed = false;
			*usErrorCode |= ACC_ERR_XTEST2;
		}
		if ( sYAccRef - sAccRead16( LIS3DH_OUT_Y_L ) < LIS3DH_SELF_TEST_Y_DEV )
		{
			bPassed = false;
			*usErrorCode |= ACC_ERR_YTEST2;
		}
		if ( sZAccRef - sAccRead16( LIS3DH_OUT_Z_L ) < LIS3DH_SELF_TEST_Z_DEV )
		{
			bPassed = false;
			*usErrorCode |= ACC_ERR_ZTEST2;
		}
		
		vAccWrite8( LIS3DH_INT2_THS, 	0x00 );	
		if ( ucAccRead8( LIS3DH_INT2_THS ) != 0x00 )
		{
			bPassed = false;
			*usErrorCode |= ACC_ERR_WRINT2;
		}
		vAccWrite8( LIS3DH_INT2_THS, 	0x55 );	
		if ( ucAccRead8( LIS3DH_INT2_THS ) != 0x55 )
		{
			bPassed = false;
			*usErrorCode |= ACC_ERR_WRINT2;
		}
		vAccWrite8( LIS3DH_INT2_THS, 	0x2A );	
		if ( ucAccRead8( LIS3DH_INT2_THS ) != 0x2A )
		{
			bPassed = false;
			*usErrorCode |= ACC_ERR_WRINT2;
		}
		vAccWrite8( LIS3DH_INT2_THS, 	0x7F );	
		if ( ucAccRead8( LIS3DH_INT2_THS ) != 0x7F )
		{
			bPassed = false;
			*usErrorCode |= ACC_ERR_WRINT2;
		}
	}
	
	/* Test done: reboot the accelerometer. */
	/* 20ms delay (required: 5ms). */
	vAccWrite8( LIS3DH_CTRL_REG5, LIS3DH_BOOT );	
	vTaskDelaySafe( 2 );		
	
	/* This call is used ONLY to make sure the linker does not drop the vAccDumpStatus() function from the 
	   program code. */
	vAccDumpStatus();

	vAccDeviceConfig();
	vAccDisableI2C();
	vAccPowerDown();

	if ( bPassed )
	{
		NRF_LOG_INFO( "self-test passed." );	
	}
	else
	{
		NRF_LOG_WARNING( "self-test failed." );	
	}

	NRF_LOG_FLUSH();
	
	return bPassed;
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*
 *															 *
 *      Debug functions										 *
 *															 *
 *-----------------------------------------------------------*/
/* Dump the LIS3DH registers to a memory array. */
void vAccStatus( struct xACC_STATUS *xAccStatus )
{
	xAccStatus->x 				= sAccRead16( LIS3DH_OUT_X_L 		 );
	xAccStatus->y 				= sAccRead16( LIS3DH_OUT_Y_L 		 );
	xAccStatus->z 				= sAccRead16( LIS3DH_OUT_Z_L 		 );
	
	xAccStatus->statusAux 		= ucAccRead8( LIS3DH_STATUS_REG_AUX  );
	xAccStatus->intCounter 		= ucAccRead8( LIS3DH_INT_COUNTER_REG );
	xAccStatus->ctrlReg0 		= ucAccRead8( LIS3DH_CTRL_REG0		 );
	xAccStatus->ctrlReg1 		= ucAccRead8( LIS3DH_CTRL_REG1		 );
	xAccStatus->ctrlReg2 		= ucAccRead8( LIS3DH_CTRL_REG2		 );
	xAccStatus->ctrlReg3 		= ucAccRead8( LIS3DH_CTRL_REG3		 );
	xAccStatus->ctrlReg4 		= ucAccRead8( LIS3DH_CTRL_REG4		 );
	xAccStatus->ctrlReg5 		= ucAccRead8( LIS3DH_CTRL_REG5		 );
	xAccStatus->ctrlReg6 		= ucAccRead8( LIS3DH_CTRL_REG6		 );
	xAccStatus->reference 		= ucAccRead8( LIS3DH_REFERENCE		 );
	xAccStatus->status 			= ucAccRead8( LIS3DH_STATUS_REG 	 );
	xAccStatus->fifoCtrl 		= ucAccRead8( LIS3DH_FIFO_CTRL_REG	 );
	xAccStatus->fifoSrc 		= ucAccRead8( LIS3DH_FIFO_SRC_REG  	 );
	xAccStatus->int1Cfg 		= ucAccRead8( LIS3DH_INT1_CFG 		 );
	xAccStatus->int1Src 		= ucAccRead8( LIS3DH_INT1_SOURCE 	 );
	xAccStatus->int1Ths 		= ucAccRead8( LIS3DH_INT1_THS 		 );
	xAccStatus->int1Duration	= ucAccRead8( LIS3DH_INT1_DURATION 	 );
	xAccStatus->int2Cfg 		= ucAccRead8( LIS3DH_INT2_CFG 		 );
	xAccStatus->int2Src 		= ucAccRead8( LIS3DH_INT2_SOURCE 	 );
	xAccStatus->int2Ths 		= ucAccRead8( LIS3DH_INT2_THS 		 );
	xAccStatus->int2Duration	= ucAccRead8( LIS3DH_INT2_DURATION 	 );
	xAccStatus->clickCfg 		= ucAccRead8( LIS3DH_CLICK_CFG 		 );
	xAccStatus->clickSrc 		= ucAccRead8( LIS3DH_CLICK_SRC 		 );
	xAccStatus->clickThs 		= ucAccRead8( LIS3DH_CLICK_THS 		 );
	xAccStatus->timeLimit 		= ucAccRead8( LIS3DH_TIME_LIMIT 	 );
	xAccStatus->timeLatency		= ucAccRead8( LIS3DH_TIME_LATENCY 	 );
	xAccStatus->timeWindow 		= ucAccRead8( LIS3DH_TIME_WINDOW 	 );
}   
/*-----------------------------------------------------------*/

/* Dump the LIS3DH registers to a memory array. */
void vAccDumpStatus( void )
{
	struct xACC_STATUS 	xAccStatus;
	
	vAccStatus( &xAccStatus );
}
/*-----------------------------------------------------------*/