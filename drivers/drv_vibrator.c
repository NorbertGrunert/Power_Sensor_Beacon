/*
 * Tracker Firmware
 *
 * Vibrator driver
 *
 * The vibration driver takes care of all types of vibrations.
 * The vibrations are started/stopped under timer control, i.e. no function is blocking.
 *
 * The current state of the vibration motor can be queried. The query result takes into account that the motor
 * will continue turning for a while even though its power is shut down.
 *
 * Concurrency is not properly addressed. Vibrator functions which are called from different tasks concurrently, are:
 * 
 *	function						task			condition								actions
 *	--------------------------------------------------------------------------------------------------------------------
 *	vStartPrealertVibrate()			CTRL			on state change							set global vibration timing parameters, 
 *																							xTimerChangePeriod, xTimerStart
 *	vStopPrealertVibrate()			CTRL			on state change							xTimerStop
 *	vIndicateEvacuation()  			BLE_PARSER		reception BLE_EVAC distress beacon		set global vibration timing parameters, 
 *									GSM				reception SRVCMD_EVACUATE				xTimerChangePeriod, xTimerStart
 *	vCustomVibrate()				BLE_PARSER	 	reception danger beacon					set global vibration timing parameters, 
 *									CTRL			ACK entering ALERT/SOS					xTimerChangePeriod, xTimerStart
 *									GSM				reception SRVCMD_VIBRATE
 *	prvVibratorTimerCallback()		TMR				timer expiry							xTimerChangePeriod
 *
 * All xTimer... functions are based on FreeRTOS Queue operations and can be considered thread-safe. Set global 
 * vibration timing parameters are always put into critical section (except for handling them in the TMR task as 
 * this one has the highest priority and cannot be interrupted). This avoids that the global parameters are left 
 * in an inconsistent state.
 * 
 * If two vibrator requests arrive at the same moment, the last one wins.
 *
 * The vibrator can be disabled by writing to RAM. This is only a debug last resort.
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>

#include "tracker.h"

/* nRF SDK include files */
#define NRF_LOG_MODULE_NAME 		VIBRATOR
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/* Device specific include files. */
#include "drv_accelerometer.h"
#include "drv_nvm.h"
#include "drv_uart.h"
#include "drv_vibrator.h"

#include "config.h"
#include "ctrl.h"
#include "custom_board.h"
#include "gsm.h"
#include "parser.h"
#include "trace.h"
#include "tracemsg.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Reset motor turning indication as the motor should have stopped spinning by now. */
void prvVibratorMotorOffCallback( TimerHandle_t xTimer );

/* Turn on vibration motor. */
void vStartMotor( TickType_t xBlockTime );

/* Turn off vibration motor. Reset the status variable with some time lag since when having been powered down, 
   the motor continues turning a bit. */
void vStopMotor( TickType_t xBlockTime );

/* Get status of the vibration motor. */
bool bVibrMotorIsTurning( void );

/* Start vibrating in Pre-Alert state to notify the user of an imminent alert message. */
void vStartPrealertVibrate( void );

/* Let the vibrator stop emitting pre-alert. */
void vStopPrealertVibrate( void );

/* Perform a commanded, custom vibration sequence. */
void vCustomVibrate( unsigned portBASE_TYPE uxVibrationParamOn, unsigned portBASE_TYPE uxVibrationParamOff, unsigned portBASE_TYPE uxVibrationParamRep );

/* Vibrator timer callback. */
void prvVibratorTimerCallback( TimerHandle_t xTimer);

/* Vibrator driver init. */
void vVibratorInit( void );

/* Vibrator self-test. */
bool bVibrTest( void );
/*-----------------------------------------------------------*/

/* Global variables. */
/* Indicator set if the vibration motor had been activated at least once. The indication is sent in the
   next IND field and reset afterwards. */
bool bVibrationMotorOn;
/*-----------------------------------------------------------*/

/* Module scope variables. */
/* Timer handle for the vibrator task. */
static 	TimerHandle_t 	xVibratorTimer;

/* Timer handle for the motor spin-down. */
static	TimerHandle_t	xMotorLagTimer;

/* Variable allowing to disable the vibrator during the pilot project - if it really works badly. */
bool 					bVibratorEnabled;

/* Describes the actual state of the vibrator. */
bool 					bVibratorOn;

/* Parameters of the current vibration sequence. */
unsigned portBASE_TYPE	uxVibrationOn;
unsigned portBASE_TYPE	uxVibrationOff;
unsigned portBASE_TYPE	uxVibrationRep;
unsigned portBASE_TYPE	uxVibrationCycleCount;

/* Motor turning status. */
bool					bMotorTurning;
/*-----------------------------------------------------------*/

/* Create timers. */
void vVibratorInit( void )
{
	/* Enable the vibrator. */
	bVibratorEnabled = true;
		
	/* Initial vibrator state if off. */
	bVibratorOn = false;
	
	xVibratorTimer = xTimerCreate
							( "VIBR",	 					/* Timer name for debug. */
							  PREALERT_VIBRATOR_ON_TIME, 	/* Timer period in ticks. */
							  pdFALSE,						/* Auto-reload. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvVibratorTimerCallback		/* Callback for the vibrator timer. */
							);
							
	xMotorLagTimer = xTimerCreate
							( "MOTLAG",	 					/* Timer name for debug. */
							  VIBR_MOTOR_OFF_LAG, 			/* Timer period in ticks. */
							  pdFALSE,						/* Auto-reload. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvVibratorMotorOffCallback	/* Callback for the motor lag timer. */
							);

	/* Set vibrator driver pin to output and 0. */
	nrf_gpio_cfg_output( VIBR );
	nrf_gpio_pin_clear( VIBR );

	/* Initialise motor turning variable. */
	bMotorTurning = false;
	
	/* Initialise the vibrator counters. */
	uxVibrationCycleCount = 1;
	uxVibrationOn = 1;
	uxVibrationOff = 1;
	uxVibrationRep = 0;
	
	/* Initialise the vibration motor indication. */
	bVibrationMotorOn = false;

	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* Reset motor turning indication as the motor should have stopped spinning by now. 
   But only do this if the motor has not been restarted in the meantime (safety-net only). */
void prvVibratorMotorOffCallback( TimerHandle_t xTimer )
{
	/* Calm down the compiler. */
	( void )xTimer;
	
	if ( !nrf_gpio_pin_out_read( VIBR ) )
	{
		bMotorTurning = false;
	}
}
/*-----------------------------------------------------------*/

/* Turn on vibration motor. */
void vStartMotor( TickType_t xBlockTime )
{
	( void )xTimerStop( xMotorLagTimer, xBlockTime );
	nrf_gpio_pin_set( VIBR );
	bMotorTurning = true;
	
	/* Set the vibration motor indication. */
	bVibrationMotorOn = true;	
}
/*-----------------------------------------------------------*/

/* Turn off vibration motor. Start the timer for the motor spin-down as
   the motor continues turning a bit. */
void vStopMotor( TickType_t xBlockTime )
{
	nrf_gpio_pin_clear( VIBR );
	( void )xTimerStart( xMotorLagTimer, xBlockTime );
}
/*-----------------------------------------------------------*/

/* Get status of the vibration motor. */
bool bVibrMotorIsTurning( void )
{
	return bMotorTurning;
}
/*-----------------------------------------------------------*/

/* Start vibrating in Pre-Alert state to notify the user of an imminent alert message. */
void vStartPrealertVibrate( void )
{
	/* Switch vibrator on. */
	if ( bVibratorEnabled )
	{
		vStartMotor( portMAX_DELAY );
	}
	
	portENTER_CRITICAL();
	
	/* Update the vibrator state. */
	bVibratorOn = true;
   	uxVibrationCycleCount = 1;
	
	/* Set the parameters. */
	uxVibrationOn  = PREALERT_VIBRATOR_ON_TIME;
	uxVibrationOff = PREALERT_VIBRATOR_OFF_TIME;
	uxVibrationRep = 255;							/* Dummy, vibrate as long as vStopPrealertVibrate() is not called. */
	
	portEXIT_CRITICAL();
	
	/* Set vibrator timer to ON duration. */
	( void )xTimerChangePeriod( xVibratorTimer, uxVibrationOn * portTICKS_PER_100MSEC, portMAX_DELAY );

	/* Start the vibrator timer. */
	( void )xTimerStart( xVibratorTimer, portMAX_DELAY );
}
/*-----------------------------------------------------------*/

/* Let the vibrator stop emitting pre-alert. */
void vStopPrealertVibrate( void )
{
	/* Stop the vibrator timer. */
	( void )xTimerStop( xVibratorTimer, portMAX_DELAY );

	/* Update the vibrator state. */
	bVibratorOn = false;
	
	/* Switch vibrator off. */
	vStopMotor( portMAX_DELAY );
}
/*-----------------------------------------------------------*/

/* Perform a commanded, custom vibration sequence.
   The parameters uxVibrationParamOn and uxVibrationParamOff express the on and off time in 100milliseconds. */
void vCustomVibrate( unsigned portBASE_TYPE uxVibrationParamOn, unsigned portBASE_TYPE uxVibrationParamOff, unsigned portBASE_TYPE uxVibrationParamRep )
{
	/* Switch vibrator on. */
	if ( bVibratorEnabled )
	{
		vStartMotor( portMAX_DELAY );
	}

	portENTER_CRITICAL();
	
	/* Set the parameters. */
	uxVibrationOn  = uxVibrationParamOn;
	uxVibrationOff = uxVibrationParamOff;
	uxVibrationRep = uxVibrationParamRep;
	
	/* Update the vibrator state. */
	bVibratorOn = true;
	uxVibrationCycleCount = 1;
	
	/* Safety net: xTimerChangePeriod crashes with new period set to 0. */
	uxVibrationOn = ( uxVibrationOn == 0 ) ? 1 : uxVibrationOn;
	
	portEXIT_CRITICAL();
	
	/* Set vibrator timer to ON duration. */
	( void )xTimerChangePeriod( xVibratorTimer, uxVibrationOn * portTICKS_PER_100MSEC, portMAX_DELAY );

	/* Start the vibrator timer. */
	( void )xTimerStart( xVibratorTimer, portMAX_DELAY );
}
/*-----------------------------------------------------------*/

/* Vibrator callback. */
void prvVibratorTimerCallback( TimerHandle_t xTimer )
{
	/* Calm down the compiler. */
	( void )xTimer;

	if ( nrf_gpio_pin_out_read( VIBR ) )
	{
		/* Vibrator is on: */
		/* Check the vibration cycle count. If the number of cycles has been reached,
		   switch the vibration off. */
		if ( uxVibrationCycleCount < uxVibrationRep )
		{
			/* Safety net: xTimerChangePeriod crashes with new period set to 0. */
			uxVibrationOff = ( uxVibrationOff == 0 ) ? 1 : uxVibrationOff;
	
			/* Set vibrator timer to OFF duration. */
			( void )xTimerChangePeriod( xVibratorTimer, uxVibrationOff * portTICKS_PER_100MSEC, 0 );
			
			uxVibrationCycleCount++;
		}
		else
		{
			/* Stop the vibration: Update the vibrator state. */
			bVibratorOn = false;
		}

		/* Switch off the vibrator. This call needs to come at the end as it contains a 50ms inherent 
		   delay. */
		vStopMotor( 0 );
	}
	else
	{
		/* Update the vibrator state. */
		if ( bVibratorOn )
		{
			/* Vibrator is off: */
			/* Switch on the vibrator. */
			vStartMotor( 0 );
			
			/* Safety net: xTimerChangePeriod crashes with new period set to 0. */
			uxVibrationOn = ( uxVibrationOn == 0 ) ? 1 : uxVibrationOn;
			
			/* Set the vibrator timer to ON duration. */
			( void )xTimerChangePeriod( xVibratorTimer, uxVibrationOn * portTICKS_PER_100MSEC, 0 );
		}
	}
}
/*-----------------------------------------------------------*/

/* Vibrator self-test. 

   The self-test is based on the mechanical coupling between vibrator and accelerometer. When the vibrator is
   active, the vibration can be captured in the accelerometer.
   
   The measurement is based on taking a number of acceleration snapshots while the vibrator is off and determining
   the maximum acceleration of asll three axes. Then, the same measurement is repeated with the vibrator on.
   The difference between the two sets is the acceleration inflicted by the vibrator per axis.
   
   Worst case we expect about 0.1g on at least one axis. Typically, the difference can be observed on both x and y
   axes as the vibrator has the tendency to vibrate horizontally.
*/
bool bVibrTest( void )
{
	/* The autotest tests first samples the accelerometer x/y/z values and takes their respective maximum.
	   Next, the vibrator is switched on. Again, the accelerations are sampled. The the difference is evalued
	   and returned as test result.
	*/
	unsigned short		usLoopCount;
	union xXYZ_DATA		xXYZData;
	signed short		maxOnX, maxOnY, maxOnZ;
	signed short		maxOffX, maxOffY, maxOffZ;
	
	/* Power-up the accelerometer. */
	vAccPowerUp();
	
	/* Capture the maximum acceleration on all three axes for 1000 samples. */
	maxOffX = 0x8000;
	maxOffY = 0x8000;
	maxOffZ = 0x8000;
	
	for ( usLoopCount = 0; usLoopCount < 1000; usLoopCount++ )
	{
		vReadAccData( &xXYZData );
		if ( maxOffX < xXYZData.xXYZ.sXData ) { maxOffX = xXYZData.xXYZ.sXData; }
		if ( maxOffY < xXYZData.xXYZ.sYData ) { maxOffY = xXYZData.xXYZ.sYData; }
		if ( maxOffZ < xXYZData.xXYZ.sZData ) { maxOffZ = xXYZData.xXYZ.sZData; }
	}
	
	/* Switch vibrator on. */
	vStartMotor( portMAX_DELAY );
	
	/* Capture the maximum acceleration on all three axes for 1000 samples. */
	maxOnX = 0x8000;
	maxOnY = 0x8000;
	maxOnZ = 0x8000;		

	for ( usLoopCount = 0; usLoopCount < 1000; usLoopCount++ )
	{
		vReadAccData( &xXYZData );
		if ( maxOnX < xXYZData.xXYZ.sXData ) { maxOnX = xXYZData.xXYZ.sXData; }
		if ( maxOnY < xXYZData.xXYZ.sYData ) { maxOnY = xXYZData.xXYZ.sYData; }
		if ( maxOnZ < xXYZData.xXYZ.sZData ) { maxOnZ = xXYZData.xXYZ.sZData; }
	}
	
	/* Switch off the vibrator. */
	vStopMotor( portMAX_DELAY );
	
	/* Restore the initial default state of the accelerometer. */
	vAccDeviceConfig();	

	/* Return true as test result if the acceleration difference on any axis is larger than 
	   the defined minimum. */
	return(    ( maxOnX - maxOffX > VIBR_SELF_TEST_THR )
			|| ( maxOnY - maxOffY > VIBR_SELF_TEST_THR )
			|| ( maxOnZ - maxOffZ > VIBR_SELF_TEST_THR ) );
}
/*-----------------------------------------------------------*/