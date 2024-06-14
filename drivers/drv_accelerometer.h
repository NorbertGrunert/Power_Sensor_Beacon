/*
 * Tracker Firmware
 *
 * GSM/GPS driver header file
 *
 */
 
#ifndef DRV_ACC_H
#define DRV_ACC_H

#include "utils.h"
#include "nrf_drv_gpiote.h"


/* LIS3DH registers. */
#define LIS3DH_STATUS_REG_AUX 			( 0x07 )
#define LIS3DH_OUT_ADC1_L				( 0x08 )
#define LIS3DH_OUT_ADC1_H				( 0x09 )
#define LIS3DH_OUT_ADC2_L				( 0x0A )
#define LIS3DH_OUT_ADC2_H				( 0x0B )
#define LIS3DH_OUT_ADC3_L				( 0x0C )
#define LIS3DH_OUT_ADC3_H				( 0x0D )
#define LIS3DH_INT_COUNTER_REG 			( 0x0E )
#define LIS3DH_WHO_AM_I 				( 0x0F )
#define LIS3DH_CTRL_REG0				( 0x1E )
#define LIS3DH_TEMP_CFG_REG 			( 0x1F )
#define LIS3DH_CTRL_REG1				( 0x20 )
#define LIS3DH_CTRL_REG2				( 0x21 )
#define LIS3DH_CTRL_REG3				( 0x22 )
#define LIS3DH_CTRL_REG4				( 0x23 )
#define LIS3DH_CTRL_REG5				( 0x24 )
#define LIS3DH_CTRL_REG6				( 0x25 )
#define LIS3DH_REFERENCE				( 0x26 )
#define LIS3DH_STATUS_REG 				( 0x27 )
#define LIS3DH_OUT_X_L					( 0x28 )
#define LIS3DH_OUT_X_H					( 0x29 )
#define LIS3DH_OUT_Y_L					( 0x2A )
#define LIS3DH_OUT_Y_H					( 0x2B )
#define LIS3DH_OUT_Z_L					( 0x2C )
#define LIS3DH_OUT_Z_H					( 0x2D )
#define LIS3DH_FIFO_CTRL_REG 			( 0x2E )
#define LIS3DH_FIFO_SRC_REG 			( 0x2F )
#define LIS3DH_INT1_CFG 				( 0x30 )
#define LIS3DH_INT1_SOURCE 				( 0x31 )
#define LIS3DH_INT1_THS 				( 0x32 )
#define LIS3DH_INT1_DURATION 			( 0x33 )
#define LIS3DH_INT2_CFG 				( 0x34 )
#define LIS3DH_INT2_SOURCE 				( 0x35 )
#define LIS3DH_INT2_THS 				( 0x36 )
#define LIS3DH_INT2_DURATION 			( 0x37 )
#define LIS3DH_CLICK_CFG 				( 0x38 )
#define LIS3DH_CLICK_SRC 				( 0x39 )
#define LIS3DH_CLICK_THS 				( 0x3A )
#define LIS3DH_TIME_LIMIT 				( 0x3B )
#define LIS3DH_TIME_LATENCY 			( 0x3C )
#define LIS3DH_TIME_WINDOW 				( 0x3D )
#define LIS3DH_ACT_THS 					( 0x3E )
#define LIS3DH_ACT_DUR	 				( 0x3F )

/* LIS3DH default values. */            
#define LIS3DH_WHO_AM_I_VAL				( 0x33 )	

/* LIS3DH Status register bit masks. */
#define	ACC_STATUS_ACT					( 0x10 )
#define	ACC_STATUS_INACT				( 0x20 )
                                         
/* Misc definitions. */                  
#define  LIS3DH_RWN						( 0b10000000 )
#define  LIS3DH_MULT					( 0b01000000 )
/* TEMP_CFG_REG */
#define  LIS3DH_ADC_PD					( 0b10000000 )
#define  LIS3DH_TEMP_EN					( 0b01000000 )
/* CTRL_REG1 */
#define  LIS3DH_ODR010					( 0b00100000 )
#define  LIS3DH_ODR025					( 0b00110000 )
#define  LIS3DH_ODR050					( 0b01000000 )
#define  LIS3DH_ODR100					( 0b01010000 )
#define  LIS3DH_ODR200					( 0b01100000 )
#define  LIS3DH_ODR400					( 0b01110000 )
#define  LIS3DH_LPEN					( 0b00001000 )
#define  LIS3DH_Z_EN					( 0b00000100 )
#define  LIS3DH_Y_EN					( 0b00000010 )
#define  LIS3DH_X_EN					( 0b00000001 )
/* CTRL_REG2 */
#define  LIS3DH_HPM_REF					( 0b01000000 )			/* High-pass filter: reference signal for filtering. */
#define  LIS3DH_HPM_NORM				( 0b10000000 )			/* High-pass filter: normal mode. */
#define  LIS3DH_HPM_AUTORES				( 0b11000000 )			/* High-pass filter: autoreset on interrupt event. */
#define  LIS3DH_HPCF_8HZ				( 0b00000000 )			/* Cut-off frequency 8Hz at 400Hz ODR, 0.2Hz at 10Hz ODR. */
#define  LIS3DH_HPCF_4HZ				( 0b00010000 )			/* Cut-off frequency 4Hz at 400Hz ODR, 0.08Hz at 10Hz ODR. */
#define  LIS3DH_HPCF_2HZ				( 0b00100000 )			/* Cut-off frequency 2Hz at 400Hz ODR. */
#define  LIS3DH_HPCF_1HZ				( 0b00110000 )			/* Cut-off frequency 1Hz at 400Hz ODR. */
#define  LIS3DH_FDS_BYP					( 0b00000000 )			/* Bypass internal filter for data output / FIFO. */
#define  LIS3DH_FDS_USE					( 0b00001000 )			/* Use internal filter for data output / FIFO. */
#define  LIS3DH_HPIA2					( 0b00000010 )			/* High-pass filter enabled for AOI function, int. 2. */
#define  LIS3DH_HPIA1					( 0b00000001 )			/* High-pass filter enabled for AOI function, int. 1. */
/* CTRL_REG3 */
#define  LIS3DH_I1_AOI1					( 0b01000000 )			/* AOI1 interrupt on INT1. */
#define  LIS3DH_I1_AOI2					( 0b00100000 )			/* AOI2 interrupt on INT1. */
/* CTRL_REG4 */
#define  LIS3DH_BDU						( 0b10000000 )			/* Block data update. */
#define  LIS3DH_FS_2G					( 0b00000000 )			/* 2g full scale. */
#define  LIS3DH_FS_8G					( 0b00100000 )			/* 8g full scale. */
#define  LIS3DH_HR						( 0b00001000 )			/* High-resolution enable. */
/* CTRL_REG5 */
#define  LIS3DH_BOOT					( 0b10000000 )			/* Force memory contents reboot. */
#define  LIS3DH_FIFO_EN					( 0b01000000 )			/* FIFO enable. */
#define  LIS3DH_LIR_INT1				( 0b00001000 )			/* Latch interrupt requests on INT1. */
#define  LIS3DH_LIR_INT2				( 0b00000010 )			/* Latch interrupt requests on INT2. */
/* CTRL_REG6 */
#define  LIS3DH_I2_AOI1					( 0b01000000 )			/* AOI1 interrupt on INT2. */
#define  LIS3DH_I2_AOI2					( 0b00100000 )			/* AOI2 interrupt on INT2. */
/* FIFO_CTRL_REG */
#define  LIS3DH_BYPASS					( 0b00000000 )			/* Bypass mode. */
#define  LIS3DH_FIFO					( 0b01000000 )			/* FIFO mode. */
#define  LIS3DH_STREAM					( 0b10000000 )			/* Stream mode. */
#define  LIS3DH_STREAM2FIFO				( 0b11000000 )			/* Stream-to-FIFO mode. */
#define  LIS3DH_TRG_ON_INT1				( 0b00000000 )			/* Trigger event for Stream-to-FIFO mode from detector 1. */
#define  LIS3DH_TRG_ON_INT2				( 0b00100000 )			/* Trigger event for Stream-to-FIFO mode from detector 2. */
/* FIFO_SRC_REG */
#define  LIS3DH_EMPTY_MSK				( 0b00100000 )			/* Mask for FIFO empty flag. */
/* INTx_CFG */
#define  LIS3DH_OREV					( 0b00000000 )			/* OR events. */
#define  LIS3DH_ANDEV					( 0b10000000 )			/* AND events. */
#define  LIS3DH_ZHIE					( 0b00100000 )			/* Enable interrupt generation on Z high event. */
#define  LIS3DH_ZLIE					( 0b00010000 )			/* Enable interrupt generation on Z low event. */
#define  LIS3DH_YHIE					( 0b00001000 )			/* Enable interrupt generation on Y high event. */
#define  LIS3DH_YLIE					( 0b00000100 )			/* Enable interrupt generation on Y low event. */
#define  LIS3DH_XHIE					( 0b00000010 )			/* Enable interrupt generation on X high event. */
#define  LIS3DH_XLIE					( 0b00000001 )			/* Enable interrupt generation on X low event. */
/* INTx_SRC */
#define  LIS3DH_IA						( 0b01000000 )			/* Interrupt active. */
#define  LIS3DH_ZH						( 0b00100000 )			/* Z high event. */
#define  LIS3DH_ZL						( 0b00010000 )			/* Z low event. */
#define  LIS3DH_YH						( 0b00001000 )			/* Y high event. */
#define  LIS3DH_YL						( 0b00000100 )			/* Y low event. */
#define  LIS3DH_XH						( 0b00000010 )			/* X high event. */
#define  LIS3DH_XL						( 0b00000001 )			/* X low event. */

#define  LIS3DH_SELF_TEST_X_DEV			( 250 )					/* Typical value is 276. */
#define  LIS3DH_SELF_TEST_Y_DEV			( 250 )					/* Typical value is 276. */
#define  LIS3DH_SELF_TEST_Z_DEV			( 900 )					/* Typical value is 984. */

#define  SPI_INSTANCE  					1	 					/* SPI HW peripheral instance index. Note that SPI instance 0 overlaps with TWI instance 0, see
																   https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fmemory.html&anchor=topic */

#define	 LIS3DH_MIN_VCC					( 2549 )				/* Minimum VCC to boot the accelerometer: 83% * 1.8V / ADC_GAIN_VCC / ADC_REF * ADC_MAX_VALUE = 2549 */
#define  LIS3DH_BOOT_DELAY				( 150 )					/* Time required by the accelerometer to boot from its Flash (ms). */
                                         
#define	 ACC_ABOVE						( 0 )
#define	 ACC_BELOW						( 1 )

#define	 ACC_DELAYED_IRQ1				( 15 )					/* Add a ~15ms delay to the accelerometer INT1 interrupt which allows the accelerometer HW to take the IRQ 
																   acknowledgement into account.
																   This delay has to be larger than ACC_DELAYED_IRQ2 to make sure that IRQ2 is served first in case both interrupts
																   have triggered. IRQ1 will reprogram the accelerometer and destroy the FIFO contents. 
																   It does not matter if the timer expires while the IRQ2 handler is still running. The handler runs in the CTRL task,
																   the same which also handles IRQ1. So the IRQ1 handler only runs when the IRQ2 handler has finished and the CTRL task
																   checks again for new messages (and will find CTRL_ACC_WU_DET1 and then execute the IRQ1 handler). */
#define	 ACC_DELAYED_IRQ2				( 10 )					/* Add a ~10ms delay to the accelerometer INT2 interrupt which allows to stop
																   the accelerometer FIFO with some delay and thus get a few more acceleration
																   samples (~10ms / 2.5ms = 4) after the actual trigger event. 
																   The timer is based on configTICK_RATE_HZ = 1/1024. */
																   
#define	ACC_TMR_BLOCKTIME  				( 1 * portTICKS_PER_SEC )

/* Macro to protect access to the accelerometer hardware. 
   
   The protection is achieved for foreground processes by a mutex. Accesses to the accelerometer inside interrupt routines are not protected 
   by the mutex since the processor cannot wait for a mutex to be released while in interrupt context. To achive protection also for interrupt
   routines, all interrupts which access the accelerometer are blocked by the macro and released on exit (the accelerometer interrupt itself
   and the TCC1 interrupt which read the accelerometer). */    
#define	 TO_MUTEX_ACC   				( 10 * portTICKS_PER_SEC )

				
union xXYZ_DATA                          
{
	unsigned char	ucAccData[6];
	struct
	{
		signed short	sXData;
		signed short	sYData;
		signed short	sZData;
	} xXYZ;
};

/* Structure to hold the most important accelerometer register values for debug. */
struct xACC_STATUS
{
	short					x;
	short					y;
	short					z;
	
	unsigned portBASE_TYPE	statusAux;
	unsigned portBASE_TYPE	intCounter;
	unsigned portBASE_TYPE	ctrlReg0;
	unsigned portBASE_TYPE	ctrlReg1;
	unsigned portBASE_TYPE	ctrlReg2;
	unsigned portBASE_TYPE	ctrlReg3;
	unsigned portBASE_TYPE	ctrlReg4;
	unsigned portBASE_TYPE	ctrlReg5;
	unsigned portBASE_TYPE	ctrlReg6;
	unsigned portBASE_TYPE	reference;
	unsigned portBASE_TYPE	status;
	unsigned portBASE_TYPE	fifoCtrl;
	unsigned portBASE_TYPE	fifoSrc;
	unsigned portBASE_TYPE	int1Cfg;
	unsigned portBASE_TYPE	int1Src;
	unsigned portBASE_TYPE	int1Ths;
	unsigned portBASE_TYPE	int1Duration;
	unsigned portBASE_TYPE	int2Cfg;
	unsigned portBASE_TYPE	int2Src;
	unsigned portBASE_TYPE	int2Ths;
	unsigned portBASE_TYPE	int2Duration;
	unsigned portBASE_TYPE	clickCfg;
	unsigned portBASE_TYPE	clickSrc;
	unsigned portBASE_TYPE	clickThs;
	unsigned portBASE_TYPE	timeLimit;
	unsigned portBASE_TYPE	timeLatency;
	unsigned portBASE_TYPE	timeWindow;
};

/* Accelerometer self-test error codes. */
#define ACC_ERR_NONE		0x0000
#define ACC_ERR_WHOAMI		0x0001
#define ACC_ERR_WR00		0x0002	
#define ACC_ERR_WR55		0x0004		
#define ACC_ERR_WRAA		0x0008			
#define ACC_ERR_WRFF		0x0010		
#define ACC_ERR_XABOVE		0x0020		
#define ACC_ERR_YABOVE		0x0040	
#define ACC_ERR_ZABOVE		0x0080
#define ACC_ERR_ZBELOW		0x0100	
#define ACC_ERR_XTEST1		0x0200	
#define ACC_ERR_YTEST1		0x0400	
#define ACC_ERR_ZTEST1		0x0800	
#define ACC_ERR_XTEST2		0x1000	
#define ACC_ERR_YTEST2		0x2000	
#define ACC_ERR_ZTEST2		0x4000	
#define ACC_ERR_WRINT2		0x8000	
/*-----------------------------------------------------------*/

/* Public function prototypes. */
/* Start or restart the accelerometer from scratch. */
extern void vAccPhyStart( bool bProtectedAccess );

/* Stop the accelerometer. This function also disabled and de-configures the SPI access. */
extern void vAccPhyStop( bool bProtectedAccess );

/* Accelerometer HW and driver init. */
extern void vAccInit( void );
extern void vAccDeviceConfig( void );

/* Lock HW access to the accelerometer. HW interrupts from the accelerometer are disabled. Pending interrupts are served once
   the normal operation resumes.
   The delayed interrupt timer continues to run. If it expires while the access is locked, servicing it is deferred until
   normal operation is resumed. */
extern void vAccLockAccess( void );

/* Release HW access to accelerometer. The mutex protecting from access by concurrent tasks is given. The interrupt from the
   GPIO pin receiving the accelerometer INT is re-enabled. The macro checks if there was a pending interrupt. If yes, it calls 
   the accelerometer interrupt handler which will clear the interrupt source. 
   The delayed interrupt timer executes pending interrupts while servicing the interrupt is resumed. */
extern void vAccReleaseAccess( void );

/* Disable the accelerometer's I2C. */
extern void vAccDisableI2C( void );

/* Read a single 8-bit register. */
extern unsigned portBASE_TYPE ucAccRead8( unsigned portBASE_TYPE ucRegAddr );

/* Power-up the accelerometer and set it to Inactive state. */
extern void vAccPowerUp( void );

/* Bring the accelerometer into stand-by mode. */
extern void vAccPowerDown( void );

/* Shut-down the accelerometer unprotected, i.e. before OS and any interrupts are running. */
extern void vAccPowerDownUnprot( void );

/* Restart the accelerometer's FIFO. */
extern void vAccFifoRestart( void );

/* Disable x and y accelerometer axes. */
extern void vAccDisXYAxes( void );

/* Enable accelerometer interrupt 1. */
extern void vAccEnableInt1( unsigned portBASE_TYPE uxIntMask );

/* Disable accelerometer interrupt 1. */
extern void vAccDisableInt1( void );

/* Set Accelerator activity characteristics on interrupt 1. */
extern void vAccSetWUCharacInt1( unsigned short usAccThres, unsigned portBASE_TYPE xDuration );

/* Enable accelerometer interrupt 2. */
extern void vAccEnableInt2( unsigned portBASE_TYPE uxIntMask );

/* Disable accelerometer interrupt 2. */
extern void vAccDisableInt2( void );

/* Set Accelerator activity characteristics on interrupt 2. */
extern void vAccSetWUCharacInt2( unsigned short usAccThres, unsigned portBASE_TYPE xDuration );

/* Set Accelerator inactivity characteristics. */
extern void vAccSetInactCharac( unsigned short usAccThres, unsigned short usDuration );

/* Read accelerometer X/Y/Z data. */
extern void vReadAccData( union xXYZ_DATA *xAccData );

/* Read accelerometer X/Y/Z data - unprotected. */
extern void vReadAccDataUnprot( union xXYZ_DATA *xAccData );

/* Clear any pending delayed accelerometer interrupts and messages already sent to the CRTL queue. */
extern void vAccClearInterrupts( void );

/* Accelerometer HW test. */
extern bool bAccTest( unsigned short *usErrorCode );

/* Dump the LIS3DH registers to a memory array. */
extern void vAccDumpStatus( void );

/* Dump the LIS3DH registers to a memory array for debug. */
extern void vAccStatus( struct xACC_STATUS *xAccStatus );

/* Public variables. */
extern unsigned char			ucAccIntSrc1;
extern unsigned char			ucAccIntSrc2;
extern union xXYZ_DATA          xXYZAcc;
extern SemaphoreHandle_t		xMutexAcc;
/*-----------------------------------------------------------*/

#endif