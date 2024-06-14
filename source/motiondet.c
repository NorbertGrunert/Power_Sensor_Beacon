/*
 * Tracker Firmware
 *
 * Motion detection
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>

#include "tracker.h"

/* nRF SDK include files */
#define NRF_LOG_MODULE_NAME 		MD
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
#include "custom_board.h"

#include "drv_accelerometer.h"
#include "drv_nvm.h"
#include "drv_vibrator.h"

#include "ble_ctrl.h"
#include "charger.h"
#include "config.h"
#include "ctrl.h"
#include "evacuation.h"
#include "motiondet.h"
#include "trace.h"
#include "tracemsg.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* 
 * API 
 */
/* Motion detection init. */
void vMDInit( void );

/* Start motion detection.
   Only step, SOS and abnormal detection are running for now as those function are active in all device states. 
   For everything else, the enter function to the corresponding device state needs to be called.
 */
void vMDStartDetection( void );

/* Disable motion detection logic. */
void vMDStopDetection( void );


/* 
 * Step and SOS detection.
 */
/* Function to stop the NOABN command mode. */
void prvStopNoAbnMode( void );

/* Set the accelerometer to step and SOS detection. */
void vMDConfigureStepSOS( void );

/* Set the accelerometer to abnormal position detection. */
void vMDConfigureAbnormal( void );


/* SLEEP */
/* Set accelerometer to movement detection. */
void vMDEnterSleep( void );


/* STILL */
/* Configure the device for STILL state. */
void vMDEnterStill( void );


/* INACTIVE */
/* Configure the device for INACTIVE state. */
void vMDEnterInactive( void );

/* ACTIVE */
/* Configure the device for ACTIVE state. */
void vMDEnterActive( void );


/* PREALERT */
/* Set the accelerometer to ALERT_CANCEL detection. */
void vMDEnterPreAlert( enum xCTRL_STATE xNewCtrlState );


/* ALERT */
/* Set the accelerometer to STEP detection. */
void vMDEnterAlert( void );


/* 
 * Private functions. 
 */
/* Timer callback for motion detection. */
void prvMDAccDetTimerCallback1( TimerHandle_t xTimer );

/* Timer callback for motion detection. */
void prvMDAccDetTimerCallback2( TimerHandle_t xTimer );

/* Timer callback for motion detection. */
void prvMDAccDetTimerCallback3( TimerHandle_t xTimer );


/* Set the accelerometer to motion detection. */
void vMDConfigureMotion( void );

/* Stop the accelerometer to motion detection. */
void prvMDStopMotion( void );

/* Motion detection handler. */
void prvMDMotion_WUHandler( void );

/* Motion time handler. */
void prvMDMotion_TimerHandler( void );

/* Accelerometer wake-up handler for steps and SOS movements. */
void prvMDStepSOS_WUHandler( void );



/* SLEEP */
/* Any movement detection will bring the accelerometer out of SLEEP state. */
void prvMDSleep_WUHandler( void );


/* STILL */
/* Timer handler for abnormal positon detection. */
void prvMDAbnormalDet_TimerHandler( void );

/* Sleep detection time handler in STILL state. */
void prvMDStill_TimerHandler( void );

/* Any movement detection will bring the accelerometer out of STILL state. */
void prvMDStill_WUHandler( void );

/* Action for any detected steps in STILL state. */
void prvMDStill_StepEvent( enum xSTEP_EVENT xStepEventType );


/* INACTIVE */
/* Motion detection handler. */
void prvMDInactive_WUHandler( void );

/* Still detection time handler in INACTIVE state. */
void prvMDInactive_TimerHandler( void );

/* Action for any detected steps in INACTIVE state. */
void prvMDInactive_StepEvent( enum xSTEP_EVENT xStepEventType );


/* ACTIVE */
/* Inactivity handler for the inactivity interrupt. */
void prvMDActive_TimerHandler( void );

/* Action for any detected steps. */
void prvMDActive_StepEvent( enum xSTEP_EVENT xStepEventType  );


/* PREALERT */
/* Activity handler for the accelerometer MOTION/STILL detection. */
void prvMDAlertCancelDet_TimerHandler( void );

/* Activity handler for the accelerometer ALERT_CANCEL detection. */
void prvMDAlertCancelTapDet_WUHandler( void );

/* Activity handler for the accelerometer ALERT_CANCEL detection by movement. */
void prvMDAlertCancelMvmtDet_WUHandler( void );

/* Start the long xNoAbnStopTimer. */
void vStartNoAbnStopTimer( void );

/* Timer callback for long time-out for reactivate the abnormal position detection. */
void xNoAbnStopTimerCB( TimerHandle_t xTimer );

/* Timer callback to reactivate the abnormal position detection. */
static void prvActivateAbnPosDetection( TimerHandle_t xTimer );
/*-----------------------------------------------------------*/

/* Global variables. */
/*-----------------------------------------------------------*/

/* Module scope variables. */
/* Timer handle for the various accelerator tasks. */
static TimerHandle_t 		xAccTimer1;		
static TimerHandle_t 		xAccTimer2;		
static TimerHandle_t 		xAccTimer3;		

/* Callback for motion detection timer handler. */
void( *prvMotionDetTimerHandler1 )( void );
void( *prvMotionDetTimerHandler2 )( void );
void( *prvMotionDetTimerHandler3 )( void );

/* Relative step count for transitioning from INACTIVE to ACTIVE. */
unsigned portBASE_TYPE		uxStepCount;			

/* SOS tap count for transitioning to PRESOS. */	
unsigned portBASE_TYPE		uxSosCount;					

/* Command tap count for "No Abnormal Detection" command. */	
unsigned portBASE_TYPE		uxCmdCount;			

/* Callback for action upon step detection. */
void( *prvStepAction )( enum xSTEP_EVENT xStepEventType );

/* Shared variable (write in this module, read from LED module) indicating tilted device attitude. */
bool						bDeviceTilted;

/* Initialize the counter for orientation changes. */
unsigned portBASE_TYPE		uxOrientationChangeCount;

/* Variables controlling the abnormal position detection diabled feature. */
bool						bNoAbnActive;
static TimerHandle_t		xNoAbnStopTimer;
static TimerHandle_t		xAbnPosDetectedTimer;
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*
 *															 *
 *      API													 *
 *															 *
 *-----------------------------------------------------------*/

/* Initialise motion detection. */
void vMDInit( void )
{
	/* Create timer to monitor the next acceleration peaks. */
	xAccTimer1 = xTimerCreate
							( "", 							/* Timer name for debug. */
							  STEP_MAX_INTERVAL,			/* Maximum interval for motion detection. */
							  pdFALSE,						/* Single-shot. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvMDAccDetTimerCallback1		/* Callback for the acceleration timer for motion detection. */
							);

	xAccTimer2 = xTimerCreate
							( "", 							/* Timer name for debug. */
							  ABNORMAL_POS_TIMER_NRML,		/* Interval for sampling abnormal positions in all states except SLEEP. */
							  pdTRUE,						/* Automatic reload. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvMDAccDetTimerCallback2		/* Callback for the acceleration timer for motion detection. */
							);

	xAccTimer3 = xTimerCreate
							( "", 							/* Timer name for debug. */
							  MOTION_HOLDOFF,				/* Holdoff after detecting a motion interrupt. */
							  pdFALSE,						/* Single-shot. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvMDAccDetTimerCallback3		/* Callback for the acceleration timer for motion detection. */
							);

	/* Invalidate the handlers. They will be initialised correctly when a function is started. */
	prvMotionDetTimerHandler1 = NULL;
	prvMotionDetTimerHandler2 = NULL;
	prvMotionDetTimerHandler3 = NULL;
	vAccWUHandlerInt1 = NULL;
	vAccWUHandlerInt2 = NULL;
	prvStepAction = NULL;
	
	/* Initialise absolute step count. */
	usAbsStepCount = 0;
	uxStepCount = 0;
	
	/* At init, the device is supposed not to be tilted. */
	bDeviceTilted = false;
	
	/* Initialize the counter for orientation changes. */
	uxOrientationChangeCount = 0;
	/* Initialise the 'No Abnormal Position Detection' feature variable to feature inactive. */
	bNoAbnActive = false;
	xNoAbnStopTimer = xTimerCreate
							( "NOABSTP",					/* Timer name for debug. */
							  10,							/* Dummy value. The actual value is filled in when the timer is started. */
															/* Maximum time for 'No Abnormal Position Detection' to be off. */
							  pdTRUE,						/* Single-shot. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  xNoAbnStopTimerCB				/* Timer callback. */
							);

	xAbnPosDetectedTimer = xTimerCreate
							( "POSDET",						/* Timer name for debug. */
							  ABN_POS_DET_WINDOW * portTICKS_PER_SEC,	/* Maximum time between no abnormal position detections to maintain NOABN mode.
																		   Default timer version. The actual value is set once the MD is started. */
							  pdTRUE,						/* Single-shot. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvActivateAbnPosDetection	/* Timer callback. */
							);

	vAccPowerDownUnprot();	
	
	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/
 
/* Start motion detection.
   Only step, SOS and abnormal detection are running for now as those function are active in all device states. 
   For everything else, the enter function to the corresponding device state needs to be called.
 */
void vMDStartDetection( void )
{
	/* Safety net: xTimerChangePeriod crashes with new period set to 0. */
	if ( usConfigReadShort( &xNvdsConfig.usAbnPosDetectedWindow ) > 0 )
	{
		( void )xTimerChangePeriod( xAbnPosDetectedTimer, usConfigReadShort( &xNvdsConfig.usAbnPosDetectedWindow ) * portTICKS_PER_SEC, portMAX_DELAY );
	}
	else
	{
		( void )xTimerChangePeriod( xAbnPosDetectedTimer, ABN_POS_DET_WINDOW * portTICKS_PER_SEC, portMAX_DELAY );
	}
	
	/* Set the accelerometer to standby mode to be able to safely configure it. */
	vAccPowerDown();

	/* Stop all pending interrupts. Full restart! */
	vAccClearInterrupts();

	/* Initialize accelerometer. */
	vAccDeviceConfig();
	
	/* Configure step and SOS detection. */
	vMDConfigureStepSOS();
	
	/* Configure abnormal position detection. */
	vMDConfigureAbnormal();
	
	/* Configure motion detection (i.e. any type of movement). */
	vMDEnterInactive();
	
	/* Set the accelerometer to motion detection. */
	vMDConfigureMotion();
	
	/* Reset the step count used for transitions from INACTIVE to ACTIVE. */
	uxStepCount = 0;	
	
	/* Power-up the accelerometer. */
	vAccPowerUp();	
}
/*-----------------------------------------------------------*/
 
/* Stop motion detection entirely.
   This function is used whenever the device exits a state which uses Motion Detection (e.g. STILL)
   and enters a state which does not (e.g. CHARGING).
*/
void vMDStopDetection( void )
{
	/* Shut down the accelerometer. */
	vAccPowerDown();	
	
	/* Stop all pending interrupts, including delayed INT2. */
	vAccClearInterrupts();	

	/* Stop timers, if running. */
	if ( xTimerIsTimerActive( xAccTimer1 ) )
	{
		( void )xTimerStop( xAccTimer1, portMAX_DELAY );
	}

	if ( xTimerIsTimerActive( xAccTimer2 ) )
	{
		( void )xTimerStop( xAccTimer2, portMAX_DELAY );
	}

	prvMotionDetTimerHandler1 = NULL;
	prvMotionDetTimerHandler2 = NULL;
	vAccWUHandlerInt1 = NULL;
	vAccWUHandlerInt2  = NULL;
	prvStepAction = NULL;
	
	bDeviceTilted = false;

	prvStopNoAbnMode();
}
/*-----------------------------------------------------------*/

/* Timer callback for motion detection. */
void prvMDAccDetTimerCallback1( TimerHandle_t xTimer )
{
	/* Calm down the compiler. */
	( void )xTimer;

	/* If a call back exists, execute it. */
	if ( prvMotionDetTimerHandler1 != NULL )
	{
		prvMotionDetTimerHandler1();
	}
}
/*-----------------------------------------------------------*/

/* Timer callback for motion detection. */
void prvMDAccDetTimerCallback2( TimerHandle_t xTimer )
{
	/* Calm down the compiler. */
	( void )xTimer;

	/* If a call back exists, execute it. */
	if ( prvMotionDetTimerHandler2 != NULL )
	{
		prvMotionDetTimerHandler2();
	}
}
/*-----------------------------------------------------------*/

/* Timer callback for motion detection. */
void prvMDAccDetTimerCallback3( TimerHandle_t xTimer )
{
	/* Calm down the compiler. */
	( void )xTimer;
	
	/* If a call back exists, execute it. */
	if ( prvMotionDetTimerHandler3 != NULL )
	{
		prvMotionDetTimerHandler3();
	}
}
/*-----------------------------------------------------------*/

/* Start the long xNoAbnStopTimer. */
void vStartNoAbnStopTimer( void )
{
	unsigned long	ulAbnPosDetectionStopTO;
	
	/* Convert position send interval from seconds to ticks. */
	ulAbnPosDetectionStopTO = ( unsigned long )usConfigReadShort( &xNvdsConfig.usNoAbnMaxPeriod ) * ( unsigned long )portTICKS_PER_SEC;
	
	/* Program the first timer period. */
	( void )xTimerChangePeriod( xNoAbnStopTimer, ulAbnPosDetectionStopTO, 0 );
	
	xTimerStart( xNoAbnStopTimer, 0 );
}
/*-----------------------------------------------------------*/


/* Timer callback for long time-out for reactivate the abnormal position detection. */
void xNoAbnStopTimerCB( TimerHandle_t xTimer )
{
	( void )xTimer;
	
	if ( bNoAbnActive )
	{
		/* Timer expired: Reactivate the abnormal position detection. */		   
		bNoAbnActive = false;
		
		uxCmdCount = 0;
		
		/* Also stop the timer which monitors time spent in normal position without abnormal position detection. */
		xTimerStop( xAbnPosDetectedTimer, 0 );

		V_TRACE_PRINT( TRACE_MD_NOABN_EXIT, TRACE_UART_AND_FILE );				

		/* Update the stats counter. */
		vSystemNoAbnStats( SYSSTATS_NOABN_MODESTOP );
	}
}
/*-----------------------------------------------------------*/		
		
/* Timer callback to reactivate the abnormal position detection. 
   The timer is a one-shot, i.e. it will stop here.
*/
void prvActivateAbnPosDetection( TimerHandle_t xTimer )
{
	/* Calm down the compiler. */
	( void )xTimer;
	
	if ( bNoAbnActive )
	{
		bNoAbnActive = false;
		
		uxCmdCount = 0;
		
		/* Also stop the timer which monitors the max. dwell time without abnormal position detection. */
		xTimerStop( xNoAbnStopTimer, 0 );
		
		V_TRACE_PRINT( TRACE_MD_NOABN_EXIT, TRACE_UART_AND_FILE );				

		/* Update the stats counter. */
		vSystemNoAbnStats( SYSSTATS_NOABN_MODESTOP );
	}
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*
 *															 *
 *      STEP AND SOS AND CMD DETECTION		 				 *
 *															 *
 *-----------------------------------------------------------*/

 /* Global variables. */
unsigned short 			usAbsStepCount;				/* Absolute step count to be sent to server. */
TickType_t		 		xLastStepTS;				/* Timestamp of last step. */
TickType_t		 		xLastSosTS;					/* Timestamp of last SOS tap. */
TickType_t		 		xLastCmdTS;					/* Timestamp of last command tap. */
/*-----------------------------------------------------------*/

/* Function to stop the NOABN command mode. */
void prvStopNoAbnMode( void )
{
	if ( bNoAbnActive )
	{
		bNoAbnActive = false;		
		xTimerStop( xAbnPosDetectedTimer, 0 );
		xTimerStop( xNoAbnStopTimer, 0 );

		V_TRACE_PRINT( TRACE_MD_NOABN_EXIT, TRACE_UART_AND_FILE );				

		/* Update the stats counter. */
		vSystemNoAbnStats( SYSSTATS_NOABN_MODESTOP );
	}
	uxCmdCount = 0;
	xLastCmdTS = xTaskGetTickCount();
}
/*-----------------------------------------------------------*/

/* Set the accelerometer to step and SOS detection. */
void vMDConfigureStepSOS( void )
{
	/* Set activity threshold 1 to detect steps. 
	   The threshold here is used as common basis for step and SOS wake-ups.
	   The duration is one sample. */
	vAccSetWUCharacInt2( STEP_SOS_WAKEUP_ACC, 1 );
	vAccWUHandlerInt2 = prvMDStepSOS_WUHandler;
	
	/* Enable accelerator wake-up interrupt. The interrupts will be disabled at every occurrence in the HW handler and need to  
	   be re-enabled in the prvMDStepSOS_WUHandler(). */
	vAccEnableInt2( LIS3DH_XHIE | LIS3DH_YHIE | LIS3DH_ZHIE );	
	
	uxSosCount = 0;
	uxCmdCount = 0;
	xLastStepTS = xTaskGetTickCount();
	xLastSosTS = xTaskGetTickCount();
	xLastCmdTS = xTaskGetTickCount();
	
	/* Start accelerometer FIFO. */
	vAccFifoRestart();
}
/*-----------------------------------------------------------*/
	
/* Accelerometer wake-up handler for steps and SOS movements. The handler runs when the accelerometer detects
   the event associated with INT2. 
   The accelerometer's interrupt status registers are copied to RAM in the actual interrupt routine to
   help further classifying the event.
   Note, that these registers not only reflect the actual interrupt but all status bits even 
   if the associated interrupt source is disabled (e.g. when ZH is enabled and triggered the interrupt,
   XL/XH and YL/YH are also updated).
	
   The mechanism for SOS detection is basically the same as for step detection.
*/
void prvMDStepSOS_WUHandler( void )
{
	portBASE_TYPE 				xIdx;
	union xXYZ_DATA 			xAccData;
	unsigned portBASE_TYPE 		uxStepPeakIdx;
	unsigned portBASE_TYPE 		uxSosPeakIdx;
	unsigned portBASE_TYPE 		uxCmdPeakIdx;	
	short						sStepAccelerationThreshold;
	short						sSosAccelerationThreshold;
	short						sSosMinXAccelerationThreshold;
	short						sSosMinZAccelerationThreshold;
	short						sSosMaxZAccelerationThreshold;
	unsigned long 				ulStepAboveThres;
	unsigned long 				ulXSosAboveThres;
	unsigned long 				ulZSosBelowHiThres;
	unsigned long 				ulZSosAboveLoThres;
	unsigned long 				ulXCmdAboveThres;
	unsigned long 				ulZCmdBelowHiThres;
	unsigned long 				ulZCmdAboveLoThres;
	bool						bIsStep;
	bool						bIsSos;
	bool						bIsCmd;	

	uxStepPeakIdx = 0x7f;
	uxSosPeakIdx = 0x7f;
	uxCmdPeakIdx = 0x7f;	
	ulStepAboveThres = 0;
	ulXSosAboveThres = 0;
	ulZSosBelowHiThres = 0;
	ulZSosAboveLoThres = 0;
	ulXCmdAboveThres = 0;
	ulZCmdBelowHiThres = 0;
	ulZCmdAboveLoThres = 0;
	
	/* Copy the alert cancel acceleration threshold for speed reasons. */
	sStepAccelerationThreshold = ( signed short )usConfigReadShort( &xNvdsConfig.usStepAccelerationThreshold );
	sSosAccelerationThreshold = ( signed short )usConfigReadShort( &xNvdsConfig.usSosAccelerationThreshold );
	sSosMinXAccelerationThreshold = ( signed short )usConfigReadShort( &xNvdsConfig.usSosMinXAccelerationThreshold );
	sSosMinZAccelerationThreshold = ( signed short )usConfigReadShort( &xNvdsConfig.usSosMinZAccelerationThreshold );
	sSosMaxZAccelerationThreshold = ( signed short )usConfigReadShort( &xNvdsConfig.usSosMaxZAccelerationThreshold );

	/* Reset the abnormal position timer so that its reading does not interfere with the FIFO access. */
	if ( xTimerIsTimerActive( xAccTimer2 ) )
	{
		( void )xTimerReset( xAccTimer2, portMAX_DELAY );
	}
	
	/* Look at the history of each detected wake-up event and compare against the parameters for STEP and SOS. 
	   Lock the accelerometer HW while reading the FIFO so that no one else can steal samples.
	   Release only after having read all 32 samples. Locking/releasing only once also helps a lot with 
	   execution speed. */
	vAccLockAccess();
	for ( xIdx = 31; xIdx >= 0; xIdx-- )
	{
		signed short			sTmpData;
		
		/* Read the acceleration data from the FIFO. Unprotected version, as the access was locked before. */
		vReadAccDataUnprot( &xAccData );
   
		/* The accelerometer is in a different orientation from what the firmware expects. Swap x/y data around to take care of this. 
		   The FW expects x in direction east and y in direction north (if the GPS antenna is in the north). 
		   Note that the accelerometer orientation is not hte same between TL500/501/502 and TL510/512. */
		#if defined ( TL510 ) || defined ( TL512 )
			sTmpData = xAccData.xXYZ.sXData;
			xAccData.xXYZ.sXData =  xAccData.xXYZ.sYData;
			xAccData.xXYZ.sYData = -sTmpData;
		#else
			xAccData.xXYZ.sXData = -xAccData.xXYZ.sXData;
			xAccData.xXYZ.sYData = -xAccData.xXYZ.sYData;
		#endif
		
		/* If the module is used on the left foot, inverse the sign of the x-acceleration. */
		if ( usConfigReadShort( &xNvdsConfig.usModuleSide ) == LEFT_FOOT )
		{
			xAccData.xXYZ.sXData = -xAccData.xXYZ.sXData;
		}
	
		/*
		 * Step detection. 
		 */
		/* Check y axis. If its acceleration value is above the threshold, set the corresponding bit in the check result bit map. */
		if (  xAccData.xXYZ.sYData >= STEP_Y_ACC_THRES )
		{
			ulStepAboveThres |= ( unsigned long )1 << xIdx;
		}

		/* Check we the Step trigger event was positive and sufficiently strong. Record its FIFO trigger index. */
		if ( ( xAccData.xXYZ.sYData >= sStepAccelerationThreshold ) || ( xAccData.xXYZ.sZData >= sStepAccelerationThreshold ) )
		{
			uxStepPeakIdx = xIdx;
		}

		/*
		 * SOS detection. 
		*/
		/* Check x axis. If its acceleration value is below the threshold, set the corresponding bit in the check result bit map. */
		if (  xAccData.xXYZ.sXData >= sSosMinXAccelerationThreshold ) 
		{
			ulXSosAboveThres |= ( unsigned long )1 << xIdx;
		}
	   
		/* Check z axis. If its acceleration value is below the high threshold, set the corresponding bit in the check result bit map. */
		if (  xAccData.xXYZ.sZData <= sSosMaxZAccelerationThreshold ) 
		{
			ulZSosBelowHiThres |= ( unsigned long )1 << xIdx;
		}
	   
		/* Check z axis. If its acceleration value is above the low threshold, set the corresponding bit in the check result bit map. */
		if (  xAccData.xXYZ.sZData >= sSosMinZAccelerationThreshold ) 
		{
			ulZSosAboveLoThres |= ( unsigned long )1 << xIdx;
		}
	   
		/* Check if the SOS trigger event was negative and sufficiently strong. Record its FIFO trigger index. */
		if ( xAccData.xXYZ.sXData <= -sSosAccelerationThreshold )
		{
			uxSosPeakIdx = xIdx;
		}

		/*
		 * Command detection (tap to the opposite side of SOS)
		 */
		/* Check x axis. If its acceleration value is below the threshold, set the corresponding bit in the check result bit map. */
		if (  -xAccData.xXYZ.sXData >= sSosMinXAccelerationThreshold ) 
		{
			ulXCmdAboveThres |= ( unsigned long )1 << xIdx;
		}
	   
		/* Check z axis. If its acceleration value is below the threshold, set the corresponding bit in the check result bit map. */
		if (  xAccData.xXYZ.sZData <= sSosMaxZAccelerationThreshold ) 
		{
			ulZCmdBelowHiThres |= ( unsigned long )1 << xIdx;
		}
	   
		/* Check z axis. If its acceleration value is above the low threshold, set the corresponding bit in the check result bit map. */
		if (  xAccData.xXYZ.sZData >= sSosMinZAccelerationThreshold ) 
		{
			ulZCmdAboveLoThres |= ( unsigned long )1 << xIdx;
		}
	   
		/* Check if the SOS trigger event was negative and sufficiently strong. Record its FIFO trigger index. */
		if ( -xAccData.xXYZ.sXData <= -sSosAccelerationThreshold )
		{
			uxCmdPeakIdx = xIdx;
		}
	}
	vAccReleaseAccess();
	
	/* Shift the acceleration bit map right so that the trigger event is in the rightmost position. This compensates for any latencies between the
	   accelerometer hardware trigger and the FIFO being frozen. 
	   If the peak index has not been touched at all since initialisation because no trigger event has been found (i.e. its value is still 0xff), 
	   the bit map will be shifted until it is all 0. */
	ulStepAboveThres   >>= uxStepPeakIdx;
	
	ulXSosAboveThres   >>= uxSosPeakIdx;
	ulZSosBelowHiThres >>= uxSosPeakIdx;
	ulZSosAboveLoThres >>= uxSosPeakIdx;

	ulXCmdAboveThres   >>= uxCmdPeakIdx;
	ulZCmdBelowHiThres >>= uxCmdPeakIdx;
	ulZCmdAboveLoThres >>= uxCmdPeakIdx;	
	
	bIsStep = ( ulStepAboveThres & STEP_Y_ACC_HIST_WIN ) == STEP_Y_ACC_HIST_WIN;
	bIsSos =    ( ( ulXSosAboveThres & SOS_X_ACC_HIST_WIN ) == SOS_X_ACC_HIST_WIN ) 
			 && ( ( ulZSosBelowHiThres & SOS_Z_ACC_HI_HIST_WIN ) == SOS_Z_ACC_HI_HIST_WIN )
			 && ( ( ulZSosAboveLoThres & SOS_Z_ACC_LO_HIST_WIN ) == SOS_Z_ACC_LO_HIST_WIN );
	bIsCmd =    ( ( ulXCmdAboveThres & SOS_X_ACC_HIST_WIN ) == SOS_X_ACC_HIST_WIN ) 
			 && ( ( ulZCmdBelowHiThres & SOS_Z_ACC_HI_HIST_WIN ) == SOS_Z_ACC_HI_HIST_WIN )
			 && ( ( ulZCmdAboveLoThres & SOS_Z_ACC_LO_HIST_WIN ) == SOS_Z_ACC_LO_HIST_WIN );

	/*
	 * Step handling. 
	 */
	if ( bIsStep )
	{
		/* An acceleration classified as step has been detected. */
		
		/* SOS and command detection is disabled while walking. */
		uxSosCount = 0;
		uxCmdCount = 0;
		
		/* Now check, if it was sufficiently far away from the previous step. */
		if ( xTaskGetTickCount() > xLastStepTS + usConfigReadShort( &xNvdsConfig.usStepMinInterval ) )
		{
			/* Count the step. */
			usAbsStepCount++;	
		}
	
		/* Check, if it the distance from the last step was in the defined interval. */
		if (   ( xTaskGetTickCount() > xLastStepTS + usConfigReadShort( &xNvdsConfig.usStepMinInterval ) )
			&& ( xTaskGetTickCount() < xLastStepTS + usConfigReadShort( &xNvdsConfig.usStepMaxInterval ) ) )
		{
			/* If defined, call the action to perform upon step detection. */
			if ( prvStepAction != NULL )
			{
				prvStepAction( STEP_REGULAR );
			}
		}
		else
		{
			/* If defined, call the action to perform upon step detection. */
			if ( prvStepAction != NULL )
			{
				prvStepAction( STEP_TOO_FAR );
			}
		}
		xLastStepTS = xTaskGetTickCount();
	}
	
	/*
	 * SOS handling. 
	 */
	if ( bIsSos )
	{
		unsigned portBASE_TYPE		ucSosNumberPeaks;				
		
		ucSosNumberPeaks = usConfigReadShort( &xNvdsConfig.usSosNumberPeaks );
		
		/* At this point we are reasonably sure that we have an SOS tap. */
		/* Next check: make sure the minimal and maximal distance between SOS taps is maintained. */
		if (   ( xTaskGetTickCount() > xLastSosTS + usConfigReadShort( &xNvdsConfig.usSosMinInterval ) )
			&& ( xTaskGetTickCount() < xLastSosTS + usConfigReadShort( &xNvdsConfig.usSosMaxInterval ) ) )
		{
			/* Check if enough SOS taps have been detected. Only send the event once. To re-trigger, uxSosCount has to be reset. */
			if ( uxSosCount >= ucSosNumberPeaks - 1 )
			{
				if ( uxSosCount == ucSosNumberPeaks - 1 )
				{
					enum xCTRL_EVENT	xCtrlEvent;

					/* Send a detection event to the CTRL state machine - non-blocking as the queue is emptied in the same task. If
					   blocking, the task could enter a dead loop. */
					xCtrlEvent = CTRL_SOS_DET;
					xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );
					
					/* Increment uxSosCount once more to block it. */
					uxSosCount++;
				}
				/* else: Do nothing until uxSosCount has been reset. */
			}
			else
			{
				uxSosCount++;
				xLastSosTS = xTaskGetTickCount();
			}
			
			V_TRACE_PRINT_BYTE( TRACE_MD_SOS, uxSosCount, TRACE_UART );			
		}
		else
		{
			/* If we were counting SOS taps to determine a transition, reset the SOS count. */
			if ( uxSosCount < ucSosNumberPeaks )
			{
				uxSosCount = 1;
				
				V_TRACE_PRINT_BYTE( TRACE_MD_SOS, uxSosCount, TRACE_UART );			
			}
			
			/* Update the time stamp of the last SOS tap. */
			xLastSosTS = xTaskGetTickCount();
		}
	}
	
	/*
	 * Detection of deactivation command for abnormal position detection (NOABN). 
	 * The command is only supported in ACTIVE, INACTIVE and STILL states.
	 */
	if ( bIsCmd && usConfigReadShort( &xNvdsConfig.usNoAbnEnabled ) )
	{
		if ( 	( xGetCtrlState() == CTRL_ACTIVE )
			 || ( xGetCtrlState() == CTRL_INACTIVE )
			 || ( xGetCtrlState() == CTRL_STILL ) )
		{
			unsigned portBASE_TYPE		ucNoAbnCmdPeaks;				
			
			/* Even though the parameter is 16-bit long, we use only 8 bits here. */
			ucNoAbnCmdPeaks = ( unsigned char )usConfigReadShort( &xNvdsConfig.usNoAbnCmdPeaks );
			
			/* At this point we are reasonably sure that we have a command tap. */
			/* Next check: make sure the minimal and maximal distance between taps is maintained. For simplicity, the same parameters as for SOS are used. */
			if (   ( xTaskGetTickCount() > xLastCmdTS + usConfigReadShort( &xNvdsConfig.usSosMinInterval ) )
				&& ( xTaskGetTickCount() < xLastCmdTS + usConfigReadShort( &xNvdsConfig.usSosMaxInterval ) ) )
			{
				/* Check if enough SOS taps have been detected. Only send the event once. To re-trigger, uxCmdCount has to be reset. */
				if ( uxCmdCount >= ucNoAbnCmdPeaks - 1 )
				{
					if ( uxCmdCount == ucNoAbnCmdPeaks - 1 )
					{
						/* At this point, we have detected a NOABN command. Now we need to do the following things:
							- Set the status variable to indicate that the "No Abnormal Detection" mode is on.
							- Start the xNoAbnStopTimer (long) which - when elapsing - stops the mode and goes back to normal operation.
							- Start the xAbnPosDetectedTimer (short) which will be restarted each time an abnormal position is detected.
							- Do a confirmation vibration. */
						bNoAbnActive = true;

						vStartNoAbnStopTimer();
						xTimerStart( xAbnPosDetectedTimer, 0 );
						vCustomVibrate( ucConfigReadByte( &xNvdsConfig.ucCmdVibrationParamOn ),
										ucConfigReadByte( &xNvdsConfig.ucCmdVibrationParamOff ),
										ucConfigReadByte( &xNvdsConfig.ucCmdVibrationParamRep ) );
										
						/* Increment uxCmdCount once more to block it. */
						uxCmdCount++;
						
						V_TRACE_PRINT( TRACE_MD_NOABN_ENTER, TRACE_UART_AND_FILE );		

						/* Increment the corresponding stats count. */
						vSystemNoAbnStats( SYSSTATS_NOABN_MODESTART );
					}
					/* else: Do nothing until uxCmdCount has been reset. */
				}
				else
				{
					uxCmdCount++;
					xLastCmdTS = xTaskGetTickCount();
				}
				
				V_TRACE_PRINT_BYTE( TRACE_MD_CMD, uxCmdCount, TRACE_UART );			
			}
			else
			{
				/* If we were counting command taps to determine a transition, reset the command tap count. */
				if ( uxCmdCount < ucNoAbnCmdPeaks )
				{
					uxCmdCount = 1;
					
					V_TRACE_PRINT_BYTE( TRACE_MD_CMD, uxCmdCount, TRACE_UART );			
				}
				
				/* Update the time stamp of the last SOS tap. */
				xLastCmdTS = xTaskGetTickCount();
			}
		}
		else
		{
			/* The device is in a state where NOABN command detection is not supported. If the Reset all respective state 
			   variables to start anew once the device enters a state where this is again allowed. */
			prvStopNoAbnMode();
			uxCmdCount = 0;
		}
	}
	
	/* Restart accelerometer FIFO. */
	vAccFifoRestart();
	
	/* Re-enable interrupts. */
	vAccEnableInt2( LIS3DH_XHIE | LIS3DH_YHIE | LIS3DH_ZHIE );	
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*
 *															 *
 *      ABNORMAL POSITION DETECTION						 	 *
 *															 *
 *-----------------------------------------------------------*/
 
/* Global variables. */
/* Abnormal position count. */
unsigned portBASE_TYPE		uxAbnormalPosCnt;
union xXYZ_DATA 			xPreviousOrientation;
/*-----------------------------------------------------------*/

/* Set the accelerometer to abnormal position detection. 
   Configure a regular timer to regularly poll the accelerometer z-axis. */
void vMDConfigureAbnormal( void )
{
	/* Reset the number of detected abnormal positions. */
	uxAbnormalPosCnt = 0;
	
	/* Preset the previous orientation. */
	vReadAccData( &xPreviousOrientation );
	uxOrientationChangeCount = 0;

	/* Register the callback for timer. */ 
	prvMotionDetTimerHandler2 = prvMDAbnormalDet_TimerHandler;
	
	/* Change timer period to poll the acceleration values for abnormal position dection for all states except SLEEP. */
	( void )xTimerChangePeriod( xAccTimer2, ABNORMAL_POS_TIMER_NRML, portMAX_DELAY );

	/* Start the timer. */
	( void )xTimerStart( xAccTimer2, portMAX_DELAY );
}
/*-----------------------------------------------------------*/

/* Accelerator callback for abnormal position detection. */
void prvMDAbnormalDet_TimerHandler( void )
{
	union xXYZ_DATA xAccData;
	short			sAbnormalMaxOrientationChg;
	
	vReadAccData( &xAccData );
	
	/* Read the z-acceleration value to see if it is below the threshold. */
	if ( xAccData.xXYZ.sZData < ( short )usConfigReadShort( &xNvdsConfig.usAbnormalPositionZAccelerationThreshold ) )
	{
		/* Yes: Abnormal position. */
		
		/* Set the indicator for tilted device. This indicator is used by the LED module to show the battery charge status. */
		bDeviceTilted = true;
		
		/* While the abnormal position detection is stopped by command, renew the xAbnPosDetectedTimer timer each time the device is tilted. */
		if ( bNoAbnActive )
		{
			xTimerStart( xAbnPosDetectedTimer, 0 );
		}
		
		/* Count the number of times the device has been detected as tilted.
		   The detection is blocked, if:
				- the xNoAbnPosTimer is active (i.e. a no abnormal position beacon has been seen lately),
				- or the abnormal position detection is stopped by command. */
		if (    xTimerIsTimerActive( xNoAbnPosTimer ) 
			 || bNoAbnActive )
		{
			uxAbnormalPosCnt = 0;
		}
		else
		{
			/* Device did not change orientation: Increment the abnormal position sample counter. */
			uxAbnormalPosCnt++;
			V_TRACE_PRINT_BYTE( TRACE_MD_ABNORMAL_POSITION, uxAbnormalPosCnt, TRACE_UART );
		}		
	}
	else
	{
		if ( uxAbnormalPosCnt > 0 )
		{
			V_TRACE_PRINT_BYTE( TRACE_MD_ABN_RESET_TILT, uxAbnormalPosCnt, TRACE_UART );
		}

		/* No: Reset the abnormal position sample counter. */
		uxAbnormalPosCnt = 0;

		/* Reset the indicator for tilted device. */
		bDeviceTilted = false;
	}
		
	/* Test, if the absolute device orientation has changed and cancel the abnormal detection, if this is the 
	   case. */
	if ( !bVibrMotorIsTurning() )
	{
		/* We had a previous device orientation sample we can use to assess the absolute orientation change. */
		sAbnormalMaxOrientationChg = ( short )usConfigReadShort( &xNvdsConfig.usAbnormalMaxOrientationChg );
		
		if (    ( abs( xPreviousOrientation.xXYZ.sXData - xAccData.xXYZ.sXData ) > sAbnormalMaxOrientationChg )  
			 || ( abs( xPreviousOrientation.xXYZ.sYData - xAccData.xXYZ.sYData ) > sAbnormalMaxOrientationChg )
			 || ( abs( xPreviousOrientation.xXYZ.sZData - xAccData.xXYZ.sZData ) > sAbnormalMaxOrientationChg ) )
		{
			/* Increment counter for orientation changes. */
			uxOrientationChangeCount++;
			
			/* The accelerometer readings might glitch so we need a certain number of consecutive values exceeding the
			   threshold to act on them. */
			if ( uxOrientationChangeCount >= MIN_ORIENTATION_CHG_CNT )
			{
				if ( uxAbnormalPosCnt > 0 )
				{
					V_TRACE_PRINT_BYTE( TRACE_MD_ABN_RESET_ACC_DELTA, uxAbnormalPosCnt, TRACE_UART );
				}

				/* The acceleration delta on at least one axis exceeded the limit, i.e. the device changed
				   its spatial orientation too much: Reset the abnormal position sample counter. */
				uxAbnormalPosCnt = 0;
				
				/* If the device is in SLEEP state, wake it up. */
				if ( xGetCtrlState() == CTRL_SLEEP )
				{
					prvMDSleep_WUHandler();
				}

				/* Cancel pending ALERT if alert cancel is programmed to movement. SOS cancel always uses TAP method. */
				if ( xGetCtrlState() == CTRL_PREALERT )
				{
					if ( ucConfigReadByte( &xNvdsConfig.ucAlertCancelMethod ) == ALERT_CNCL_BY_MVMT )
					{
						prvMDAlertCancelMvmtDet_WUHandler();
					}
				}

				/* Save the current orientation for the next sample time. */
				xPreviousOrientation = xAccData;
				
				uxOrientationChangeCount = 0;
			}
		}
		else
		{
			/* Reset the orientation change counter. If it was non-0 before, it means that we did not
			   get sufficient consecutive values exceeding the threshold for orientation change, i.e.
			   we received a glitch and suppress it. */
			uxOrientationChangeCount = 0;
		}
	}
	
	/* Test the abnormal position sample counter. If above the defined threshold,
	   send a pre-alert message to CTRL, unless the battery is low. */
	if (   ( uxAbnormalPosCnt > usConfigReadShort( &xNvdsConfig.usAbnormalPositionDuration ) )
		 && !bBatteryEmpty()
	   )
	{
		enum xCTRL_EVENT	xCtrlEvent;
			
		/* Condition for pre-alert detected: send CTRL_ABNORMAL_DET message to control task. */
		xCtrlEvent = CTRL_ABNORMAL_DET;
		if ( !xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 ) )
		{
			V_TRACE_PRINT( TRACE_MD_CTRL_QUEUE_FULL, TRACE_UART );
		}
		
		V_TRACE_PRINT( TRACE_MD_ABN_DET, TRACE_UART );
			
		uxAbnormalPosCnt = 0;
	}
	else
	{
		/* Restart the timer. */
		( void )xTimerStart( xAccTimer2, 0 );
	}
}	
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*
 *															 *
 *      MOTION DETECTION								 	 *
 *															 *
 *-----------------------------------------------------------*/
 
/* Set the accelerometer to motion detection. */
void vMDConfigureMotion( void )
{
	/* Register the callback for timer. */ 
	prvMotionDetTimerHandler3 = prvMDMotion_TimerHandler;
	
	/* Set activity threshold to detect first peak of motion. */
	vAccSetWUCharacInt1( MOTION_ACC_THRES, 1 );
	vAccWUHandlerInt1 = prvMDMotion_WUHandler;
	
	/* Enable accelerator wake-up interrupts. */
	vAccEnableInt1( LIS3DH_XHIE | LIS3DH_YHIE | LIS3DH_ZHIE );	
}
/*-----------------------------------------------------------*/

/* Stop the accelerometer to motion detection. */
void prvMDStopMotion( void )
{
	/* Disable accelerator wake-up interrupts. */
	vAccDisableInt1();	
	vAccWUHandlerInt1 = NULL;
	
	/* Stop the motion hold-off timer. */
	( void )xTimerStop( xAccTimer3, portMAX_DELAY );
}
/*-----------------------------------------------------------*/

/* Motion detection handler.
   The handler is called every time the accelerometer detects any kind of motion. 
*/
void prvMDMotion_WUHandler( void )
{
	/* 
	 * Transition from STILL to INACTIVE state or remain in INACTIVE.
	 */
	/* If the handler is called, it means that the time-out period for transitions to STILL
	   did not expire and that we have to restart the timer. */
	if ( xTimerIsTimerActive( xAccTimer1 ) )
	{
		( void )xTimerReset( xAccTimer1, portMAX_DELAY );
	}
		
	if ( uxAbnormalPosCnt > 0 )
	{
		V_TRACE_PRINT_BYTE( TRACE_MD_ABN_RESET_MOTION, uxAbnormalPosCnt, TRACE_UART );
	}

	/*
	 * Reset Abnormal Position detection
	 */
	uxAbnormalPosCnt = 0;
	
	/* Start hold-off interval for new motion interrupts. */
	( void )xTimerStart( xAccTimer3, portMAX_DELAY );
}
/*-----------------------------------------------------------*/
	
/* Motion time handler.
   The timer 3 is started in the motion WU handler. On its expiry, the accelerometer motion
   detection interrupt is re-enabled. This implements thus the hold-off.
*/
void prvMDMotion_TimerHandler( void )
{
	/* Re-enable interrupts. */
	vAccEnableInt1( LIS3DH_XHIE | LIS3DH_YHIE | LIS3DH_ZHIE );	
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*
 *															 *
 *		SLEEP STATE											 *
 *      MOVEMENT DETECTION		 							 *
 *															 *
 *-----------------------------------------------------------*/

/* Set accelerometer to movement detection. */
void vMDEnterSleep( void )
{
	/* No particular action is required when a step occurs. */
	prvStepAction = NULL;	
	
	/* Set the accelerometer to standby mode to be able to safely configure it. */
	vAccPowerDown();
				
	/* Register the callback for accelerator activity event. */ 
	vAccWUHandlerInt1 = prvMDSleep_WUHandler;
	
	/* Set the action for any detected steps. */
	prvStepAction = NULL;

	/* Change timer period to poll the acceleration values for abnormal position dection for SLEEP state. */
	( void )xTimerChangePeriod( xAccTimer2, ABNORMAL_POS_TIMER_SLEEP, portMAX_DELAY );
	
	/* Power-up the accelerometer. */
	vAccPowerUp();		
}
/*-----------------------------------------------------------*/
	
/* Any movement detection will bring the accelerometer out of SLEEP state. 
   CAUTION:
		Called from both TMR and CTRL tasks! 
*/
void prvMDSleep_WUHandler( void )
{
	enum xCTRL_EVENT	xCtrlEvent;
	
	/* Change timer period to poll the acceleration values for abnormal position dection for all states except SLEEP. */
	( void )xTimerChangePeriod( xAccTimer2, ABNORMAL_POS_TIMER_NRML, portMAX_DELAY );

	/* Send CTRL_MOVEMENT_DET message to control task. */
	xCtrlEvent = CTRL_MOVEMENT_DET;
	xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 ); 
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*
 *															 *
 *		STILL STATE											 *
 *      ABNORMAL POSITION, MOVEMENT AND SOS DETECTION		 *
 *															 *
 *-----------------------------------------------------------*/

/* ABNORMAL POSITION and SOS DETECTION are already configured. So only configure detection of 
   STILL-to-SLEEP. The accelerometer itself does not need to be reconfigured so that we do not 
   need to power it down/up here. */
void vMDEnterStill( void )
{
	/* No particular action is required when a step occurs. */
	prvStepAction = NULL;	
	
	/* Register the callback for accelerator activity event. */ 
	vAccWUHandlerInt1 = prvMDStill_WUHandler;
	
	/* Enable motion interrupt. */
	vAccEnableInt1( LIS3DH_XHIE | LIS3DH_YHIE | LIS3DH_ZHIE );

	/* Set the action for any detected steps. */
	prvStepAction = prvMDStill_StepEvent;
	   
	if ( xTimerIsTimerActive( xImmobilityTimer ) )
	{
		/* Change timer 1 period to monitor IMMOBILITY ALERT transitions. */
		if ( usConfigReadShort( &xNvdsConfig.usImmobilityDuration ) > 0 )
		{
			( void )xTimerChangePeriod( xAccTimer1, usConfigReadShort( &xNvdsConfig.usImmobilityDuration ) * portTICKS_PER_SEC, portMAX_DELAY );
		}
		else
		{
			( void )xTimerChangePeriod( xAccTimer1, IMMOBILITY_DURATION * portTICKS_PER_SEC, portMAX_DELAY );
		}
	}
	else
	{
		/* Change timer 1 period to monitor STILL-to-SLEEP transitions. */
		if ( usConfigReadShort( &xNvdsConfig.usSleepDuration ) > 0 )
		{
			( void )xTimerChangePeriod( xAccTimer1, usConfigReadShort( &xNvdsConfig.usSleepDuration ) * portTICKS_PER_SEC, portMAX_DELAY );
		}
		else
		{
			( void )xTimerChangePeriod( xAccTimer1, SLEEP_DURATION * portTICKS_PER_SEC, portMAX_DELAY );
		}
	}
	
	/* Register the callback for timer. */ 
	prvMotionDetTimerHandler1 = prvMDStill_TimerHandler;
}
/*-----------------------------------------------------------*/

/* Sleep detection time handler in STILL state.
   If this handler is called in STILL state, the device will change to SLEEP state.
*/
void prvMDStill_TimerHandler( void )
{
	enum xCTRL_EVENT	xCtrlEvent;

	if ( xTimerIsTimerActive( xImmobilityTimer ) )
	{
		/* The immobility timer expired: let the system go to pre-alert state. */
		xCtrlEvent = CTRL_ABNORMAL_DET;
		if ( !xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 ) )
		{
			V_TRACE_PRINT( TRACE_MD_CTRL_QUEUE_FULL, TRACE_UART );
		}
		
		V_TRACE_PRINT( TRACE_MD_ABN_DET, TRACE_UART );
	}
	else
 	{
		/* Condition for SLEEP detected: send CTRL_STILL_DET message to control task unless the device is in EVAC state (in which
		   case it is not allowed to go to sleep). */
		if ( !bGetEvacOngoing() )
		{
			xCtrlEvent = CTRL_SLEEP_DET;
			xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );
		}	
	}
}
/*-----------------------------------------------------------*/

/* Any movement detection will bring the accelerometer out of STILL state. */
void prvMDStill_WUHandler( void )
{
	enum xCTRL_EVENT	xCtrlEvent;
		
	prvMotionDetTimerHandler1 = NULL;
	
	/* Send CTRL_MOVEMENT_DET message to control task. */
	xCtrlEvent = CTRL_MOVEMENT_DET;
	xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );
}
/*-----------------------------------------------------------*/

/* Action for any detected steps in STILL state. 
	
   uxStepCount contains a temporary step count used to identify transitions from INACTIVE to ACTIVE state.
   By definition, when being in STILL state there have been no previous steps. Thus, the step detected is the
   first one and is used to transition to INACTIVE state. 
   So the step counter uxStepCount is set to 1.
*/
void prvMDStill_StepEvent( enum xSTEP_EVENT xStepEventType )
{
	( void )xStepEventType;
	
	uxStepCount = 1;
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*
 *															 *
 *		INACTIVE STATE										 *
 *		MOTION / STILL DETECTION							 *
 *															 *
 *-----------------------------------------------------------*/

/* Set the accelerometer to MOTION detection. MOTION is a generalised term to indicate any type
   of movement. MOTION is used to:
		- Transition from STILL to INACTIVE state.
		- Reset Abnormal Position detection (i.e. set the abnormal position count to 0).
		- Transition from INACTIVE to STILL state if there was now motion detected for a certain 
		  period of time.

   Motion detection is based on the accelerometer's wake-up 2 and on timer 1.
*/
void vMDEnterInactive( void )
{
	/* Set the action for any detected steps. */
	prvStepAction = prvMDInactive_StepEvent;
	
	/* Change timer 1 period to monitor INACTIVE-to-STILL transitions. */
	if ( usConfigReadShort( &xNvdsConfig.usStillDuration ) > 0 )
	{
		( void )xTimerChangePeriod( xAccTimer1, usConfigReadShort( &xNvdsConfig.usStillDuration ) * portTICKS_PER_SEC, portMAX_DELAY );
	}
	else
	{
		( void )xTimerChangePeriod( xAccTimer1, STILL_DURATION * portTICKS_PER_SEC, portMAX_DELAY );
	}

	/* Register the callback for timer. */ 
	prvMotionDetTimerHandler1 = prvMDInactive_TimerHandler;
	
	/* Reset the step count which is used to assess transition to ACTIVE state. */
	uxStepCount = 0;
		
	/* Start the timer. */
	( void )xTimerStart( xAccTimer1, portMAX_DELAY );
}
/*-----------------------------------------------------------*/

/* Still detection time handler in INACTIVE state.
   If this handler is called in INACTIVE state, the device will change to STILL state.
*/
void prvMDInactive_TimerHandler( void )
{
	enum xCTRL_EVENT	xCtrlEvent;
	
	/* Send CTRL_STILL_DET message to control task. */
	xCtrlEvent = CTRL_STILL_DET;
	xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );
}
/*-----------------------------------------------------------*/

/* Action for any detected steps in INACTIVE state. 

   uxStepCount contains a temporary step count used to identify transitions from INACTIVE to ACTIVE state.
   When uxStepCount reaches xNvdsConfig.usActiveNumberSteps, the CTRL_WALK_DET message is sent. If initialised
   to 255, steps are just counted but no message is sent.   
*/
void prvMDInactive_StepEvent( enum xSTEP_EVENT xStepEventType  )
{
	unsigned portBASE_TYPE		uxMotionNumberSteps;				

	uxMotionNumberSteps = usConfigReadShort( &xNvdsConfig.usActiveNumberSteps );
	
	/* A regular step has been detected. */
	if ( xStepEventType == STEP_REGULAR )
	{
		/* Check if enough steps have been detected. Only send the event once. To re-trigger, uxStepCount has to be reset. */
		if ( uxStepCount >= uxMotionNumberSteps - 1 )
		{
			if ( uxStepCount == uxMotionNumberSteps - 1 )
			{
				enum xCTRL_EVENT	xCtrlEvent;

				/* Send a detection event to the CTRL state machine - non-blocking as the queue is emptied in the same task. If
				   blocking, the task could enter a dead loop. */
				xCtrlEvent = CTRL_WALK_DET;
				xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );
				
				/* Increment uxStepCount once more to block it. */
				uxStepCount++;
			}
			/* else: Do nothing until uxStepCount has been reset. */
		}
		else
		{
			uxStepCount++;
		}		
		
		V_TRACE_PRINT_BYTE( TRACE_MD_STEP_INACT, uxStepCount, TRACE_UART );
	}
	
	/* A step either too close to the last one or too far from the last one has been detected. */
	if ( xStepEventType == STEP_TOO_FAR )
	{
		/* If we were counting steps to determine a transition, reset the step count. */
		if ( uxStepCount < uxMotionNumberSteps )
		{
			uxStepCount = 1;
		}
	}
}
/*-----------------------------------------------------------*/
		
		
/*-----------------------------------------------------------*
 *															 *
 *		ACTIVE STATE										 *
 *      INACTIVITY AND STEP DETECTION						 *
 *															 *
 *-----------------------------------------------------------*/
/* Set timer 1 to detect the absence of steps for a certain time. */
void vMDEnterActive()
{
	/* Set the action for any detected steps. */
	prvStepAction = prvMDActive_StepEvent;

	/* Change timer 1 period to monitor ACTIVE-to-INACTIVE transitions. */
	if ( usConfigReadShort( &xNvdsConfig.usStillDuration ) > 0 )
	{
		( void )xTimerChangePeriod( xAccTimer1, usConfigReadShort( &xNvdsConfig.usStillDuration ) * portTICKS_PER_SEC, portMAX_DELAY );
	}
	else
	{
		( void )xTimerChangePeriod( xAccTimer1, STILL_DURATION * portTICKS_PER_SEC, portMAX_DELAY );
	}
	
	/* Register the callback for timer. */ 
	prvMotionDetTimerHandler1 = prvMDActive_TimerHandler;
	
	/* Start timer. */
	( void )xTimerStart( xAccTimer1, portMAX_DELAY );		
}
/*-----------------------------------------------------------*/
 
/* Inactive detection time handler.
   If this handler is called in ACTIVE state, the device will change to Inactive state.
   Condition for entering this handler is that there are no steps detected during the expiry
   time.
*/
void prvMDActive_TimerHandler( void )
{
	enum xCTRL_EVENT	xCtrlEvent;
	
	/* Send CTRL_INACT_DET message to control task. */
	xCtrlEvent = CTRL_INACT_DET;
	xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );
}
/*-----------------------------------------------------------*/

/* Action for any detected steps. */
void prvMDActive_StepEvent( enum xSTEP_EVENT xStepEventType  )
{
	( void )xStepEventType;
	
	/* Any step event leads to the timer being reset. In other words, the timer expoires only
	   when no steps as detected during a certain time. */
	if ( xTimerIsTimerActive( xAccTimer1 ) )
	{
		( void )xTimerReset( xAccTimer1, portMAX_DELAY );
	}
	
	V_TRACE_PRINT_SHORT( TRACE_MD_STEP, usAbsStepCount, TRACE_UART );
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*
 *															 *
 *		PREALERT/PRESOS STATES								 *
 *      ALERT CANCEL DETECTION								 *
 *															 *
 *-----------------------------------------------------------*/

/* Global variables. */
TickType_t		 		xAlertCancelTS;
unsigned portBASE_TYPE	uxAlertCancelCnt;
/*-----------------------------------------------------------*/
 
/* Set the accelerometer to ALERT_CANCEL detection. */
void vMDEnterPreAlert( enum xCTRL_STATE xNewCtrlState )
{
	V_TRACE_PRINT( TRACE_MD_PREALERT, TRACE_UART_AND_FILE );				
	
	/* No particular action is required when a step occurs. */
	prvStepAction = NULL;	
	
	/* Set the accelerometer to standby mode to be able to safely configure it. */
	vAccPowerDown();

	/* Stop all pending interrupts, including delayed INT2. */
	vAccClearInterrupts();

	/* Stop motion detection. */
	prvMDStopMotion();
	/* Small delay of about 50ms to make sure the interrupt 1 really stopped. */
	vTaskDelay( 5 );
	
	/* Configure alert cancel dependent on method. */
	if (   ( ucConfigReadByte( &xNvdsConfig.ucAlertCancelMethod ) == ALERT_CNCL_BY_MVMT )
		&& ( xNewCtrlState == CTRL_PREALERT ) )
	{	
		/* Alert cancel by movement. */
		/* Register the callback for accelerator activity event. */ 
		vAccWUHandlerInt1 = prvMDAlertCancelMvmtDet_WUHandler;
		
		/* Enable motion interrupt. */
		vAccEnableInt1( LIS3DH_XHIE | LIS3DH_YHIE | LIS3DH_ZHIE );
		
		/* Disable INT2 which was used for SOS/step detection. */
		vAccDisableInt2();		
		vAccWUHandlerInt2 = NULL;		
	}	
	else
	{
		/* Alert or SOS cancel by tap. */
		/* Set WU2 activity threshold to detect an alert cancel tap. */
		vAccSetWUCharacInt2( usConfigReadShort( &xNvdsConfig.usAlertCancelThreshold ), 1 );
		
		/* Register the callbacks for accelerator activity events. */ 
		vAccWUHandlerInt2 = prvMDAlertCancelTapDet_WUHandler;
		
		/* Enable accelerometer interrupts on z axis. */
		vAccEnableInt2( LIS3DH_ZHIE );		
 	}	
	
	/* Use timer1 for alert cancel timeout. */
	prvMotionDetTimerHandler1 = prvMDAlertCancelDet_TimerHandler;
	if ( xNewCtrlState == CTRL_PREALERT )
	{
		/* PREALERT */
		if ( usConfigReadShort( &xNvdsConfig.usAlertCancelTimeout ) > 0 )
		{
			( void )xTimerChangePeriod( xAccTimer1, usConfigReadShort( &xNvdsConfig.usAlertCancelTimeout ) * portTICKS_PER_SEC, portMAX_DELAY );
		}
		else
		{
			( void )xTimerChangePeriod( xAccTimer1, ALERT_CNCL_TIMEOUT * portTICKS_PER_SEC, portMAX_DELAY );
		}
	}
	else
	{
		/* PRESOS */
		if ( usConfigReadShort( &xNvdsConfig.usSosCancelTimeout ) > 0 )
		{
			( void )xTimerChangePeriod( xAccTimer1, usConfigReadShort( &xNvdsConfig.usSosCancelTimeout ) * portTICKS_PER_SEC, portMAX_DELAY );
		}
		else
		{
			( void )xTimerChangePeriod( xAccTimer1, SOS_CNCL_TIMEOUT * portTICKS_PER_SEC, portMAX_DELAY );
		}
	}
		
	/* Start timer. */
	( void )xTimerStart( xAccTimer1, portMAX_DELAY );
	
	/* Initialise alert cancel time stamp and counter. */
	xAlertCancelTS = 0;
	uxAlertCancelCnt = 0;
	
	/* Power-up the accelerometer. */
	vAccPowerUp();
}
/*-----------------------------------------------------------*/

/* Timer handler for the ALERT_CANCEL detection. */
void prvMDAlertCancelDet_TimerHandler( void )
{
	enum xCTRL_EVENT	xCtrlEvent;

	/* If we ever get here, the pre-alert has not been canceled. Send the alert. */
	/* Send a detection event to the CTRL state machine. */
	xCtrlEvent = CTRL_ALERT_CNCL_TO;
	if ( !xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 ) )
	{
		/* Fatal error: Could not send the ALERT event to the queue. The ALERT is lost(!) and
		   the device stuck in the PREALERT. */
		V_TRACE_PRINT( TRACE_MD_ALERT_MSG_FAIL, TRACE_UART_AND_FILE );
		configASSERT( false );
	}
	V_TRACE_PRINT( TRACE_MD_PREALERT_CNCL, TRACE_UART_AND_FILE );				
}
/*-----------------------------------------------------------*/

/* Activity handler for the accelerometer ALERT_CANCEL tap detection. */
void prvMDAlertCancelTapDet_WUHandler( void )
{
	portBASE_TYPE 				xIdx;
	union xXYZ_DATA 			xAccData;
	unsigned portBASE_TYPE 		uxAlertCnclPeakIdx;
	short						sAlertCancelThreshold;
	unsigned long 				ulBelowAlertCnclThres;
	bool						bIsAlertCncl;
	
	uxAlertCnclPeakIdx = 0x7f;
	
	ulBelowAlertCnclThres = 0;
	
	/* Copy the alert cancel acceleration threshold for speed reasons. Note that xNvdsConfig.usAlertCancelThreshold is defined as RANGE_8BIT 
	   whereas the values read from the accelerometer are RANGE (16-bit) wide. As both ranges are left-aligned, it is sufficient to 
	   shift the value by 8 to get to the same range. */
	sAlertCancelThreshold = usConfigReadShort( &xNvdsConfig.usAlertCancelThreshold ) << 8;

	/* Check, if the wake-up was due to an alert-cancel tap. */
	/* Look at the history of the detected wake-up event and compare against the parameters for ALERT CANCEL. */
	vAccLockAccess();
	for ( xIdx = 31; xIdx >= 0; xIdx-- )
	{
		/* Read the acceleration data from the FIFO. */
		vReadAccDataUnprot( &xAccData );
		
		/*
		 * Alert cancel tap detection. 
		 */
		/* Check if z acceleration values are below the threshold. If true, set the corresponding bit in the check result bit map. */
		if ( xAccData.xXYZ.sZData <= ALERT_CNCL_Z_ACC_THRES )
		{
			ulBelowAlertCnclThres |= ( unsigned long )1 << xIdx;
		}
		
		/* Check we the Step trigger event was positive and sufficiently strong. Record its FIFO trigger index. */
		if ( xAccData.xXYZ.sZData >= sAlertCancelThreshold )
		{
			uxAlertCnclPeakIdx = xIdx;
		}
	}
	vAccReleaseAccess();
	
	/* Shift the acceleration bit map right so that the trigger event is in the rightmost position. This compensates for any latencies between the
	   accelerometer hardware trigger and the FIFO being frozen. 
	   If the peak index has not been touched at all since initialisation because no trigger event has been found (i.e. its value is still 0xff), 
	   the bit map will be shifted until it is all 0. */
	ulBelowAlertCnclThres >>= uxAlertCnclPeakIdx;
	
	bIsAlertCncl = ( ulBelowAlertCnclThres & ALERT_CNCL_Z_ACC_HIST_WIN ) == ALERT_CNCL_Z_ACC_HIST_WIN;
	
	if ( bIsAlertCncl )
	{
		/* Check, if there was a previous event, If not, register the time and set the cancel counter. */
		if ( uxAlertCancelCnt == 0 )
		{
			xAlertCancelTS = xTaskGetTickCount();
			uxAlertCancelCnt++;
		}
		else
		{
			/* There was a previous event. Check the minimum time distance. ATTENTION: Minimum distance re-used from motion detection! */
			if ( ( xTaskGetTickCount() - xAlertCancelTS ) > usConfigReadShort( &xNvdsConfig.usAlertCancelMinInterval ) )
			{
				/* The minimum distance is okay. Now check the maximum time distance. */
				if ( ( xTaskGetTickCount() - xAlertCancelTS ) <= usConfigReadShort( &xNvdsConfig.usAlertCancelMaxInterval ) )
				{
					/* New event within max. distance from previous event. */
					xAlertCancelTS = xTaskGetTickCount();
					uxAlertCancelCnt++;	

					/* Check if we have enough events now for an alert cancel. */
					if ( uxAlertCancelCnt >= usConfigReadShort( &xNvdsConfig.usAlertCancelNumberPeaks ) )
					{
						enum xCTRL_EVENT	xCtrlEvent;
					
						/* Accelerometer triggered on condition for ALERT cancel: send ALERT_CNCL message to control task. */
						xCtrlEvent = CTRL_ALERT_CNCL;
						xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );				
					}
				}
				else
				{
					/* Interval since last event was too long. 
					   Start from scratch. */
					xAlertCancelTS = xTaskGetTickCount();
					uxAlertCancelCnt = 1;
				}
			}
		}
		
		V_TRACE_PRINT_BYTE( TRACE_MD_CANCEL, uxAlertCancelCnt, TRACE_UART );		
	}	
	
	/* Restart accelerometer FIFO. */
	vAccFifoRestart();
	
	/* Re-enable accelerometer interrupts on z axis. */
	vAccEnableInt2( LIS3DH_ZHIE );
}
/*-----------------------------------------------------------*/

/* Activity handler for the accelerometer ALERT_CANCEL detection by movement. */
void prvMDAlertCancelMvmtDet_WUHandler( void )
{
	/* Check, if the movement was detected while the vibration motor was running. */
	if ( !bVibrMotorIsTurning() )
	{
		enum xCTRL_EVENT	xCtrlEvent;
	
		V_TRACE_PRINT( TRACE_MD_CANCEL, TRACE_UART );

		/* The motor was not running and the accelerometer triggered on condition for ALERT cancel: 
		   send ALERT_CNCL message to control task. */
		xCtrlEvent = CTRL_ALERT_CNCL;
		if ( xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 ) != pdTRUE )		
		{
			V_TRACE_PRINT( TRACE_MD_CTRL_QUEUE_FULL, TRACE_UART );
		}
	}	
	
	/* Delay until the accelerometer interrupt is physically reset. Due to the
	   latency of the accelerometer HW, this might take a while (one clock cycle or
	   1/400Hz = 2.5ms). If we access the accelerometer via SPI before the IRQ
	   is reset, the vAccReleaseAccess() macro will relaunch the interrupt routine
	   manually. */
	vTaskDelay( 1 );	
	
	/* Restart accelerometer FIFO. */
	vAccFifoRestart();
	
	/* Re-enable motion interrupts. */
	vAccEnableInt1( LIS3DH_XHIE | LIS3DH_YHIE | LIS3DH_ZHIE );
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*
 *															 *
 *		ALERT/SOS STATES									 *
 *      STEP DETECTION										 *
 *															 *
 *-----------------------------------------------------------*/

/* Global variables. */
/*-----------------------------------------------------------*/
 
/* Set the accelerometer to STEP detection. */
void vMDEnterAlert( void )
{
	/* No particular action is required when a step occurs. */
	prvStepAction = NULL;	
	
	/* Set the accelerometer to standby mode to be able to safely configure it. */
	vAccPowerDown();
	
	/* Set activity threshold 1 to detect steps. 
	   The threshold here is used as common basis for step and SOS wake-ups.
	   The duration is one sample. */
	vAccSetWUCharacInt2( STEP_SOS_WAKEUP_ACC, 1 );
	vAccWUHandlerInt2 = prvMDStepSOS_WUHandler;
	
	/* Enable accelerator wake-up interrupt. The interrupts will be disabled at every occurrence in the HW handler and need to  
	   be re-enabled in the prvMDStepSOS_WUHandler(). */
	vAccEnableInt2( LIS3DH_XHIE | LIS3DH_YHIE | LIS3DH_ZHIE );	
	
	/* No special action on step detection. */
	prvStepAction = NULL;
	
	/* Block SOS detection. */
	uxSosCount = usConfigReadShort( &xNvdsConfig.usSosNumberPeaks );	
	
	/* Start accelerometer FIFO. */
	vAccFifoRestart();
	
	/* Power-up the accelerometer. */
	vAccPowerUp();
}
/*-----------------------------------------------------------*/
