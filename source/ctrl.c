/*
 * Tracker Firmware
 *
 * Device control task
 *
 */

/* Standard include files. */
#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"

#include "tracker.h"

/* nRF SDK include files */
#define NRF_LOG_MODULE_NAME 		CTRL
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* Device specific include files. */
#include "drv_nvm.h"
#include "drv_uart.h"
#include "drv_accelerometer.h"
#include "drv_led.h"
#include "drv_vibrator.h"

#include "ble_adreport.h"
#include "ble_ctrl.h"
#include "charger.h"
#include "config.h"
#include "ctrl.h"
#include "custom_board.h"
#include "evacuation.h"
#include "gsm.h"
#include "gps.h"
#include "motiondet.h"
#include "rtc.h"
#include "trace.h"
#include "tracemsg.h"
#include "utils.h"
#include "version.h"
/*-----------------------------------------------------------*/


/* Function prototypes. */
/* Main control state machine of the tracker device. Please see FS_Tracker_<date>.docx for details. */
void vCtrlStateMachine( enum xCTRL_EVENT xCtrlEvent );

/* Get the position send interval from NVDS parameters in function of the state of the Ctrl state machine. */
TickType_t xGetCtrlPositionSendInterval( void );

/* Obtain the state of the Control state machine.. */
enum xCTRL_STATE xGetCtrlState( void );


/* Returns true if the current device state requires sending any position information in the packt to the server. */
bool bCtrlPositionRequired( void );

/* Returns true if the current device state requires sending GPS position information in the packet to the server. 
   This takes positioning using BLE beacons into account. */
bool bCtrlGpsPositionRequired( void );

/* Returns true if the current device state requires sending indoor (BLE) position information in the packet to the server. */
bool bCtrlBlePositionRequired( void );

/* Request starting the GPS. */
void prvCtrlStartGPS( void );

/* Request stopping the GPS. */
void prvCtrlStopGPS( void );

/* Request starting BLE, if BLE is enabled. */
void prvCtrlStartBleLoc( void );

/* Request stopping BLE. */
void prvCtrlStopBleLoc( void );

/* Request starting BLE, if BLE is enabled. */
void prvCtrlStartBleAlertLoc( void );

/* Request stopping BLE. */
void prvCtrlStopBleAlertLoc( void );

/* Charging to OOO state. */
void prvChrg_2_Ooo( void );

/* State machine action when transitioning from any unused state (e.g. charging, OOO) to INACTIVE state. */
void prvUnused_2_Inactive( void );

/* State machine action when transitioning from SLEEP to INACTIVE state. */
void prvSleep_2_Inactive( void );

/* State machine action when transitioning from SLEEP to PREALERT state. */
void prvSleep_2_PreAlert( void );

/* State machine action when transitioning from SLEEP to any unused state (e.g. charging, OOO). */
void prvSleep_2_Unused( void );

/* State machine action when transitioning from any normal state (STILL, INACTIVE, ACTIVE) to any unused state 
   (e.g. charging, OOO) to INACTIVE state. */
void prvNormal_2_Unused( void );

/* Actions when transitioning from any normal state (e.g. STILL, INACTIVE and ACTIVE) to PREALERT state. */
void prvNormal_2_PreAlert( void );

/* Actions when transitioning from any normal state (e.g. STILL, INACTIVE and ACTIVE) to PRESOS state. */
void prvNormal_2_PreSos( void );

/* Actions when transitioning from INACTIVE to STILL state. */
void prvInactive_2_Still( void );

/* Actions when transitioning from STILL to INACTIVE state. */
void prvStill_2_Inactive( void );

/* Actions when transitioning from STILL to SLEEP state. */
void prvStill_2_Sleep( void );

/* Actions when transitioning from ACTIVE to INACTIVE state. */
void prvActive_2_Inactive( void );

/* Actions when transitioning from PREALERT to INACTIVE state. */
void prvPreAlert_2_Inactive( void );

/* Actions when transitioning from PREALERT to ALERT state. */
void prvPreAlert_2_Alert( void );

/* Actions when transitioning from PREALERT to SOS state. */
void prvPreAlert_2_Sos( void );

/* Common actions when transitioning from PREALERT to ALERT or SOS state. */
void prvPreAlert_2_Alert_Common( void );

/* Actions when transitioning from ALERT to INACTIVE state. */
void prvAlert_2_Inactive( void );

/* Actions when transitioning from ALERT to PREALERT state. */
void prvAlert_2_PreAlert( void );

/* State machine action when transitioning from any normal state (STILL, INACTIVE, ACTIVE) to the STANDBY state. */
void prvNormal_2_Standby( void );


/* The control task. */
static portTASK_FUNCTION_PROTO( vCtrlTask, pvParameters );

/* CTRL task initialisation. */
void vCtrlInit( UBaseType_t uxPriority );

/* Timer callback for the timer supervising the periods of no abnormal position detection. */
void prvNoAbnPosTimerCallBack( TimerHandle_t xTimer );

/* Timer callback for the immobility timer. */
void prvImmobilityTimerCallBack( TimerHandle_t xTimer );
/*-----------------------------------------------------------*/

/* Global variables. */
/* Control state machine event queue. */
QueueHandle_t 			xCtrlEventQueue; 	

/* Accelerometer callback for wake-up detection handler on interrupt 1. */
void( *vAccWUHandlerInt1 )( void );

/* Accelerometer callback for wake-up detection handler on interrupt 2. */
void( *vAccWUHandlerInt2 )( void );

/* Use accelerometer only if it passed self test. */
bool 					bUseAccelerometer;

/* Accelerometer / vibrator self-test result. */
unsigned short 			usAccErrorCode;
unsigned portBASE_TYPE 	xCtrlAccVibrTestResult;

/* Timer handle for no abnormal position timer. */
TimerHandle_t 			xNoAbnPosTimer;

/* Timer handle for the immobility timer. */
TimerHandle_t 			xImmobilityTimer;
/*-----------------------------------------------------------*/

/* Local variables. */
/* State of the Ctrl task. */
enum xCTRL_STATE	xCtrlState;	

/* Control state machine definition. */
static const struct xSTATE_MACHINE xCtrlStateMachine[] = 
{
	/* Current State		Event				Action								Next State			*/
    { CTRL_STANDBY,			CTRL_ON_CHRG,		NULL,								CTRL_CHRG_NRML		},
	
    { CTRL_CHRG_OOO,		CTRL_GOTO_NORMAL,	NULL,								CTRL_CHRG_NRML		},
    { CTRL_CHRG_OOO,		CTRL_OFF_CHRG,		prvChrg_2_Ooo,						CTRL_OOO			},
	
    { CTRL_CHRG_NRML,		CTRL_GOTO_OOO,		NULL,								CTRL_CHRG_OOO		},
    { CTRL_CHRG_NRML,		CTRL_OFF_CHRG,		prvUnused_2_Inactive,				CTRL_INACTIVE		},
	
    { CTRL_OOO,				CTRL_ON_CHRG,		NULL,								CTRL_CHRG_OOO		},
	{ CTRL_OOO,				CTRL_GOTO_NORMAL,	prvUnused_2_Inactive,				CTRL_INACTIVE		},
	{ CTRL_OOO,				CTRL_GOTO_STBY,		NULL,								CTRL_STANDBY		},
	
	{ CTRL_INACTIVE,		CTRL_GOTO_OOO,		prvNormal_2_Unused,					CTRL_OOO			},
	{ CTRL_INACTIVE,		CTRL_ON_CHRG,		prvNormal_2_Unused,					CTRL_CHRG_NRML		},
	{ CTRL_INACTIVE,		CTRL_WALK_DET,		vMDEnterActive,						CTRL_ACTIVE			},
	{ CTRL_INACTIVE,		CTRL_STILL_DET,		prvInactive_2_Still,				CTRL_STILL			},
	{ CTRL_INACTIVE,		CTRL_ABNORMAL_DET,	prvNormal_2_PreAlert,				CTRL_PREALERT		},
	{ CTRL_INACTIVE,		CTRL_GOTO_PREALERT,	prvNormal_2_PreAlert,				CTRL_PREALERT		},
	{ CTRL_INACTIVE,		CTRL_SOS_DET,		prvNormal_2_PreSos,					CTRL_PRESOS			},
	{ CTRL_INACTIVE,		CTRL_GOTO_STBY,		prvNormal_2_Standby,				CTRL_STANDBY		},
	
	{ CTRL_ACTIVE,			CTRL_GOTO_OOO,		prvNormal_2_Unused,					CTRL_OOO			},
	{ CTRL_ACTIVE,			CTRL_ON_CHRG,		prvNormal_2_Unused,					CTRL_CHRG_NRML		},
	{ CTRL_ACTIVE,			CTRL_INACT_DET,		prvActive_2_Inactive,				CTRL_INACTIVE		},
	{ CTRL_ACTIVE,			CTRL_ABNORMAL_DET,	prvNormal_2_PreAlert,				CTRL_PREALERT		},
	{ CTRL_ACTIVE,			CTRL_GOTO_PREALERT,	prvNormal_2_PreAlert,				CTRL_PREALERT		},
	{ CTRL_ACTIVE,			CTRL_SOS_DET,		prvNormal_2_PreSos,					CTRL_PRESOS			},
	{ CTRL_ACTIVE,			CTRL_GOTO_STBY,		prvNormal_2_Standby,				CTRL_STANDBY		},

    { CTRL_STILL,			CTRL_GOTO_OOO,		prvNormal_2_Unused,					CTRL_OOO			},
	{ CTRL_STILL,			CTRL_ON_CHRG,		prvNormal_2_Unused,					CTRL_CHRG_NRML		},
    { CTRL_STILL,			CTRL_MOVEMENT_DET,	prvStill_2_Inactive,				CTRL_INACTIVE		},
    { CTRL_STILL,			CTRL_SLEEP_DET,		prvStill_2_Sleep,					CTRL_SLEEP			},
	{ CTRL_STILL,			CTRL_ABNORMAL_DET,	prvNormal_2_PreAlert,				CTRL_PREALERT		},
	{ CTRL_STILL,			CTRL_GOTO_PREALERT,	prvNormal_2_PreAlert,				CTRL_PREALERT		},
	{ CTRL_STILL,			CTRL_SOS_DET,		prvNormal_2_PreSos,					CTRL_PRESOS			},
	{ CTRL_STILL,			CTRL_GOTO_STBY,		prvNormal_2_Standby,				CTRL_STANDBY		},
	                                                                    		
    { CTRL_SLEEP,			CTRL_GOTO_OOO,		prvSleep_2_Unused,					CTRL_OOO			},
	{ CTRL_SLEEP,			CTRL_ON_CHRG,		prvSleep_2_Unused,					CTRL_CHRG_NRML		},
    { CTRL_SLEEP,			CTRL_MOVEMENT_DET,	prvSleep_2_Inactive,				CTRL_INACTIVE		},
	{ CTRL_SLEEP,			CTRL_GOTO_PREALERT,	prvSleep_2_PreAlert,				CTRL_PREALERT		},
	{ CTRL_SLEEP,			CTRL_GOTO_STBY,		prvNormal_2_Standby,				CTRL_STANDBY		},
	                                                                    		
    { CTRL_PREALERT,		CTRL_ALERT_CNCL,	prvPreAlert_2_Inactive,				CTRL_INACTIVE		},
    { CTRL_PREALERT,		CTRL_ALERT_CNCL_TO,	prvPreAlert_2_Alert,				CTRL_ALERT			},
	                                                                    		
	{ CTRL_ALERT,			CTRL_ALERT_ACK,		prvAlert_2_Inactive,				CTRL_INACTIVE		},
	{ CTRL_ALERT,			CTRL_GOTO_PREALERT,	prvAlert_2_PreAlert,				CTRL_PREALERT		},
	
    { CTRL_PRESOS,			CTRL_ALERT_CNCL,	prvPreAlert_2_Inactive,				CTRL_INACTIVE		},
    { CTRL_PRESOS,			CTRL_ALERT_CNCL_TO,	prvPreAlert_2_Sos,					CTRL_SOS			},
	                                                                    		
	{ CTRL_SOS,				CTRL_ALERT_ACK,		prvAlert_2_Inactive,				CTRL_INACTIVE		},
	{ CTRL_SOS,				CTRL_GOTO_PREALERT,	prvAlert_2_PreAlert,				CTRL_PREALERT		}
};	

/* Defines which states require GPS position information. 
   Caution: Has to be in the same order as enum xCTRL_STATE! */
static const bool bTransmitGpsPositionPerState[] = 
{
	false,			/* CTRL_STANDBY			*/
	false,			/* CTRL_CHRG_OOO		*/
	false,			/* CTRL_OOO				*/
	false,			/* CTRL_CHRG_NRML		*/
	true,			/* CTRL_ACTIVE			*/
	true,			/* CTRL_INACTIVE		*/
	true,			/* CTRL_STILL			*/
	true,			/* CTRL_PREALERT		*/
	true,			/* CTRL_ALERT			*/
	true,			/* CTRL_PRESOS			*/
	true,			/* CTRL_SOS				*/
	false           /* CTRL_SLEEP			*/
};

/* Defines which states require BLE position information. 
   Caution: Has to be in the same order as enum xCTRL_STATE! */
static const bool bTransmitBlePositionPerState[] = 
{
	false,			/* CTRL_STANDBY			*/
	false,			/* CTRL_CHRG_OOO		*/
	false,			/* CTRL_OOO				*/
	false,			/* CTRL_CHRG_NRML		*/
	true,			/* CTRL_ACTIVE			*/
	true,			/* CTRL_INACTIVE		*/
	true,			/* CTRL_STILL			*/
	true,			/* CTRL_PREALERT		*/
	true,			/* CTRL_ALERT			*/
	true,			/* CTRL_PRESOS			*/
	true,			/* CTRL_SOS				*/
	false           /* CTRL_SLEEP			*/
};
/*-----------------------------------------------------------*/

void vCtrlInit( UBaseType_t uxPriority )
{
	/* Create a queue to the control task for any events potentially chaing system states. */
	xCtrlEventQueue = xQueueCreate( ctrlEVENT_QUEUE_SIZE, ( unsigned portBASE_TYPE ) sizeof( enum xCTRL_EVENT ) );
	
	xNoAbnPosTimer = xTimerCreate
							( "NOABN", 						/* Timer name for debug. */
							  NO_ABN_POS_DET_INTERVAL,		/* Time-span during which no abnormal position detection is running. */
							  pdFALSE,						/* Single-shot. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvNoAbnPosTimerCallBack		/* Callback for the acceleration timer for motion detection. */
							);	
	
	xImmobilityTimer = xTimerCreate
							( "", 							/* Timer name for debug. */
							  IMMOBILITY_INTERVAL,			/* Time-span during which the immobility window is active. */
							  pdFALSE,						/* Single-shot. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvImmobilityTimerCallBack	/* Callback for the acceleration timer for the immobility timer. */
							);		
	
	/* The Control task is spawned here. */
	xTaskCreate( vCtrlTask, "CTRL", ctrlSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );

	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* Timer callback for the timer supervising the periods of no abnormal position detection. */
void prvNoAbnPosTimerCallBack( TimerHandle_t xTimer )
{
	( void )xTimer;
}
/*-----------------------------------------------------------*/

/* Timer callback for the immobility timer. */
void prvImmobilityTimerCallBack( TimerHandle_t xTimer )
{
	( void )xTimer;
}
/*-----------------------------------------------------------*/

/* Update the position send interval from NVDS parameters in function of the state of the Ctrl state machine. */
TickType_t xGetCtrlPositionSendInterval( void )
{
	/* In case of an ongoing evacuation alert, set the transmission interval to the same as in ALERT state. */
	if ( bGetEvacOngoing() )
	{
		return ( TickType_t )usConfigReadShort( &xNvdsConfig.usAlertPositionSendInterval );
	}

	/* Else, use the normal scheduling. */
	switch ( xCtrlState )
	{
		case CTRL_STANDBY:		return ( CTRL_STANDBY_POS_INT );
								break;
		case CTRL_ACTIVE:		return ( TickType_t )usConfigReadShort( &xNvdsConfig.usActivePositionSendInterval );
								break;
		case CTRL_INACTIVE:		return ( TickType_t )usConfigReadShort( &xNvdsConfig.usInactivePositionSendInterval );
								break;
		case CTRL_STILL:		return ( TickType_t )usConfigReadShort( &xNvdsConfig.usStillPositionSendInterval );
								break;
		case CTRL_SLEEP:		return ( TickType_t )usConfigReadShort( &xNvdsConfig.usSleepPositionSendInterval );
								break;
		case CTRL_OOO:			return ( TickType_t )usConfigReadShort( &xNvdsConfig.usOooPositionSendInterval );
								break;
		case CTRL_CHRG_OOO:	
		case CTRL_CHRG_NRML:	return ( TickType_t )usConfigReadShort( &xNvdsConfig.usChargingPositionSendInterval );
								break;
		case CTRL_PREALERT:		return ( TickType_t )usConfigReadShort( &xNvdsConfig.usActivePositionSendInterval );
								break;
		case CTRL_ALERT:		return ( TickType_t )usConfigReadShort( &xNvdsConfig.usAlertPositionSendInterval );
								break;
		case CTRL_SOS:			return ( TickType_t )usConfigReadShort( &xNvdsConfig.usAlertPositionSendInterval );
								break;
		default:				return ( TickType_t )usConfigReadShort( &xNvdsConfig.usActivePositionSendInterval );
	}
	
	return ( TickType_t )usConfigReadShort( &xNvdsConfig.usActivePositionSendInterval );
}
/*-----------------------------------------------------------*/

/* Obtain the state of the Control state machine. */
enum xCTRL_STATE xGetCtrlState( void )
{
	return xCtrlState;
}
/*-----------------------------------------------------------*/

/* Obtain the name string for the state of the Control state machine. */
void vGetCtrlStateName( signed char **const ppcStateName, unsigned portBASE_TYPE *uxPacketLength )
{
	/* Select the string to send in function of the device state. */
	switch ( xGetCtrlState() )
	{
		case CTRL_STANDBY:		*ppcStateName = ( signed char * )"STANDBY";
								*uxPacketLength = 7;
								break;
		case CTRL_ACTIVE:		*ppcStateName = ( signed char * )"ACTIVE";
								*uxPacketLength = 6;		
								break;
		case CTRL_INACTIVE:		*ppcStateName = ( signed char * )"INACTIVE";
								*uxPacketLength = 8;		
								break;
		case CTRL_STILL:   		*ppcStateName = ( signed char * )"STILL";
								*uxPacketLength = 5;		
								break;
		case CTRL_SLEEP:   		*ppcStateName = ( signed char * )"SLEEP";
								*uxPacketLength = 5;		
								break;
		case CTRL_OOO:			*ppcStateName = ( signed char * )"OUT-OF-OFFICE";
								*uxPacketLength = 13;		
								break;
		case CTRL_CHRG_OOO:		*ppcStateName = ( signed char * )"CHARGING OOO";
								*uxPacketLength = 12;		
								break;
		case CTRL_CHRG_NRML:	*ppcStateName = ( signed char * )"CHARGING NORMAL";
								*uxPacketLength = 15;		
								break;
		case CTRL_PREALERT:		*ppcStateName = ( signed char * )"PREALERT";
								*uxPacketLength = 8;		
								break;
		case CTRL_ALERT:		*ppcStateName = ( signed char * )"ALERT";
								*uxPacketLength = 5;		
								break;
		case CTRL_PRESOS:		*ppcStateName = ( signed char * )"PRESOS";
								*uxPacketLength = 6;		
								break;
		case CTRL_SOS:			*ppcStateName = ( signed char * )"SOS";
								*uxPacketLength = 3;		
								break;
		default:				*ppcStateName = ( signed char * )"?";
								*uxPacketLength = 1;		
								break;
	}
}
/*-----------------------------------------------------------*/

/* Returns true if the current device state requires sending any position information in the packet to the server. 
   This does not take positioning using BLE beacons into account. */
bool bCtrlPositionRequired( void )
{
	unsigned short		usGpsEnable;
	
	usGpsEnable = usConfigReadShort( &xNvdsConfig.usGpsEnable );
	return (   (   bTransmitGpsPositionPerState[ xCtrlState ]
				&& ( bool )usGpsEnable
			   ) 
			|| bTestMode  			
		   );
}
/*-----------------------------------------------------------*/

/* Returns true if the current device state requires sending GPS position information in the packet to the server. 
   This takes positioning using BLE beacons into account. */
bool bCtrlGpsPositionRequired( void )
{
	return (    (    bCtrlPositionRequired()
			      && !bEnoughBleBcnsForFix()
				)
			 || bTestMode 
		   );
}
/*-----------------------------------------------------------*/
/* Returns true if the current device state requires sending indoor (BLE) position information in the packet to the server. */
bool bCtrlBlePositionRequired( void )
{
	unsigned short		usBleEnable;
	
	usBleEnable = usConfigReadShort( &xNvdsConfig.usBleEnable );
	return (   (    bTransmitBlePositionPerState[ xCtrlState ]
				&& ( bool )usBleEnable
			   ) 
			|| bTestMode  			
		   );
}
/*-----------------------------------------------------------*/

/* Request starting the GPS. Note that when the device is requested to activate it is in communication with the server,
   i.e. the GSM/GPS module is running. 
*/
void prvCtrlStartGPS( void )
{
	enum xGPS_CMD			xGpsCmd;

	xGpsCmd = GPS_START;
	xQueueSend( xGpsCmdQueue, &xGpsCmd, 0 ); 
}
/*-----------------------------------------------------------*/
	
/* Request stopping the GPS. 
*/
void prvCtrlStopGPS( void )
{
	enum xGPS_CMD			xGpsCmd;
	
	xGpsCmd = GPS_STOP;
	xQueueSend( xGpsCmdQueue, &xGpsCmd, 0 ); 
}
/*-----------------------------------------------------------*/

/* Request starting BLE, if BLE is enabled. */
void prvCtrlStartBleLoc( void )
{
	enum xBLE_CMD			xBleCmd;
	
	/* Request starting the BLE, if BLE is enabled. */
	if (   ( bool )( usConfigReadShort( &xNvdsConfig.usBleEnable ) )
		&& !( bool )( usConfigReadShort(&xNvdsConfig.usBleScanDuringFixOnly ) )
		&& !bTemperatureOutsideOp()
	   )
	{
		/* Activate BLE. */
		xBleCmd = BLE_LOC_START;
		xQueueSend( xBleCmdQueue, &xBleCmd, bleCMD_BLOCKTIME ); 
		
		/* If the battery voltage is below the depletion level, suspend the BLE immediately. 
		   It wil be resumed as soon as the condition is cleared by the RTC. */
		if ( bBatteryDepleted() )
		{
			vNonBlockingSuspendAllBleActivity();
		}
	}
}
/*-----------------------------------------------------------*/

/* Request stopping BLE. */
void prvCtrlStopBleLoc( void )
{
	enum xBLE_CMD			xBleCmd;

	/* Request stopping the BLE. */
	xBleCmd = BLE_LOC_STOP;
	xQueueSend( xBleCmdQueue, &xBleCmd, bleCMD_BLOCKTIME ); 

	/* Delete contents of the BLE localiser store. */
	vBleDeleteLocBcnStore();
}
/*-----------------------------------------------------------*/

/* Request starting BLE, if BLE is enabled. */
void prvCtrlStartBleAlertLoc( void )
{
	enum xBLE_CMD			xBleCmd;
	
	/* Request starting the BLE, if BLE is enabled. */
	if ( ( bool )( usConfigReadShort( &xNvdsConfig.usBleEnable ) ) )
	{
		/* Activate BLE. */
		xBleCmd = BLE_SR_ALERT_SOS_START;
		xQueueSend( xBleCmdQueue, &xBleCmd, bleCMD_BLOCKTIME ); 
	}
}
/*-----------------------------------------------------------*/

/* Request stopping BLE. */
void prvCtrlStopBleAlertLoc( void )
{
	enum xBLE_CMD			xBleCmd;

	/* Request stopping the BLE. */
	xBleCmd = BLE_SR_ALERT_SOS_STOP;
	xQueueSend( xBleCmdQueue, &xBleCmd, bleCMD_BLOCKTIME ); 
}
/*-----------------------------------------------------------*/

/*
 *
 * State machine actions
 *
 */
	
/* Charging to OOO state. */
void prvChrg_2_Ooo( void )
{
	/* Leave test mode. */
	bTestMode = false;
}
/*-----------------------------------------------------------*/

/* State machine action when transitioning from any unused state (e.g. charging, OOO) to INACTIVE state. */
void prvUnused_2_Inactive( void )
{
	/* Configure the accelerometer to start MOTION/STILL detection as we enter Inactive state. */
	vMDStartDetection();
	
	/* Request starting BLE localisation (scan). */
	prvCtrlStartBleLoc();
	
	/* Leave test mode. */
	bTestMode = false;
}
/*-----------------------------------------------------------*/

/* State machine action when transitioning from SLEEP to INACTIVE state. */
void prvSleep_2_Inactive( void )
{
	/* Request starting BLE localisation (scan). */
	prvCtrlStartBleLoc();
	
	/* Set the accelerometer to standby mode to be able to safely configure it. */
	vAccPowerDown();
	
	/* Set the accelerometer to motion detection. */
	vMDConfigureMotion();

	/* Set the accelerometer to abnormal position detection. */
	vMDConfigureAbnormal();	

	/* Let the motion detection perform the necessary settings for INACTIVE state. */
	vMDEnterInactive();

	/* Power-up the accelerometer. */
	vAccPowerUp();	
}
/*-----------------------------------------------------------*/

/* State machine action when transitioning from SLEEP to PREALERT state. 
   Some of the actions to bring MD first into normal mode and then into PERALERT might
   be superfluous.
*/
void prvSleep_2_PreAlert( void )
{
	/* Request starting BLE localisation (scan). */
	prvCtrlStartBleLoc();
	
	/* Set the accelerometer to standby mode to be able to safely configure it. */
	vAccPowerDown();
	
	/* Set the accelerometer to motion detection. */
	vMDConfigureMotion();

	/* Set the accelerometer to abnormal position detection. */
	vMDConfigureAbnormal();	

	/* Let the motion detection perform the necessary settings for INACTIVE state. */
	vMDEnterInactive();

	/* Power-up the accelerometer. */
	vAccPowerUp();	
	
	/* Start vibration motor sequence. */
	vStartPrealertVibrate();

	/* Let the motion detection perform the necessary settings for PREALERT state. */
	vMDEnterPreAlert( CTRL_PREALERT );	
}
/*-----------------------------------------------------------*/

/* State machine action when transitioning from SLEEP to any unused state (e.g. charging, OOO). */
void prvSleep_2_Unused( void )
{
	/* Stop MOTION/STILL detection as we enter an unused  state. */
	vMDStopDetection();

	/* Reinitialise evacuation ID. */
	vResetEvacuation();
}
/*-----------------------------------------------------------*/

/* State machine action when transitioning from any normal state (STILL, INACTIVE, ACTIVE) to any unused state
   (e.g. charging, OOO). 
*/
void prvNormal_2_Unused( void )
{
	/* Stop MOTION/STILL detection as we enter an unused  state. */
	vMDStopDetection();
	
	/* Request stopping GPS. */
	prvCtrlStopGPS();
	
	/* Request stopping BLE. */
	prvCtrlStopBleLoc();	
	
	/* Reinitialise evacuation ID. */
	vResetEvacuation();
}
/*-----------------------------------------------------------*/

/* Actions when transitioning from any normal state (e.g. STILL, INACTIVE and ACTIVE) to PREALERT state. */
void prvNormal_2_PreAlert( void )
{
	/* Request starting BLE localisation (scan). */
	prvCtrlStartBleLoc();
	
	/* Start vibration motor sequence. */
	vStartPrealertVibrate();

	/* Let the motion detection perform the necessary settings for PREALERT state. */
	vMDEnterPreAlert( CTRL_PREALERT );
}
/*-----------------------------------------------------------*/

/* Actions when transitioning from any normal state (e.g. STILL, INACTIVE and ACTIVE) to PRESOS state. */
void prvNormal_2_PreSos( void )
{
	/* Request starting BLE localisation (scan). */
	prvCtrlStartBleLoc();
	
	/* Start vibration motor sequence. */
	vStartPrealertVibrate();

	/* Let the motion detection perform the necessary settings for PREALERT state. */
	vMDEnterPreAlert( CTRL_PRESOS );
}
/*-----------------------------------------------------------*/
	
/* Actions when transitioning from INACTIVE to STILL state. */
void prvInactive_2_Still( void )
{
	/* Set the accelerometer to standby mode to be able to safely configure it. */
	vAccPowerDown();

	/* Let the motion detection perform the necessary settings for STILL state. */
	vMDEnterStill();
	
	/* Power-up the accelerometer. */
	vAccPowerUp();	
}
/*-----------------------------------------------------------*/

/* Actions when transitioning from STILL to INACTIVE state. */
void prvStill_2_Inactive( void )
{
	/* Set the accelerometer to standby mode to be able to safely configure it. */
	vAccPowerDown();

	/* Set the accelerometer to motion detection. */
	vMDConfigureMotion();

	/* Let the motion detection perform the necessary settings for INACTIVE state. */
	vMDEnterInactive();
	
	/* Power-up the accelerometer. */
	vAccPowerUp();	
}
/*-----------------------------------------------------------*/

/* Actions when transitioning from STILL to SLEEP state. */
void prvStill_2_Sleep( void )
{
	/* Request stopping BLE localisation (scan). */
	prvCtrlStopBleLoc();
	
	/* Request stopping GPS. */
	prvCtrlStopGPS();
	
	/* Let the motion detection perform the necessary settings for SLEEP state. */
	vMDEnterSleep();
}
/*-----------------------------------------------------------*/

/* Actions when transitioning from ACTIVE to INACTIVE state. */
void prvActive_2_Inactive( void )
{
	/* Set the accelerometer to standby mode to be able to safely configure it. */
	vAccPowerDown();

	/* Let the motion detection perform the necessary settings for INACTIVE state. */
	vMDEnterInactive();
	
	/* Set the accelerometer to motion detection. */
	vMDConfigureMotion();
	
	/* Power-up the accelerometer. */
	vAccPowerUp();		
}
/*-----------------------------------------------------------*/

/* Actions when transitioning from PREALERT to INACTIVE state. */
void prvPreAlert_2_Inactive( void )
{
	/* Stop vibration motor. */
	vStopPrealertVibrate();	

	/* Set the accelerometer to standby mode to be able to safely configure it. */
	vAccPowerDown();
	
	/* Clear all interrupts. */
	vAccClearInterrupts();

	/* Set the accelerometer to step and SOS detection. */
	vMDConfigureStepSOS();

	/* Set the accelerometer to abnormal position detection. */
	vMDConfigureAbnormal();
	
	/* Let the motion detection perform the necessary settings for INACTIVE state. */
	vMDEnterInactive();
	
	/* Set the accelerometer to motion detection. */
	vMDConfigureMotion();

	/* Power-up the accelerometer. */
	vAccPowerUp();		
}
/*-----------------------------------------------------------*/

/* Actions when transitioning from PREALERT to ALERT state. */
void prvPreAlert_2_Alert( void )
{
	/* Remember that the new state is ALERT. That state will be restored if the device resets before the state is
	   acknowledged. */
	enum xCTRL_STATE	xCtrlStateTmp;
	
	xCtrlStateTmp = CTRL_ALERT;
	vConfigWriteArray( ( unsigned char * )&xNvdsConfig.xCtrlStateBackup, ( unsigned char * )&xCtrlStateTmp, sizeof( enum xCTRL_STATE ) );
	
	prvPreAlert_2_Alert_Common();
}
/*-----------------------------------------------------------*/

/* Actions when transitioning from PREALERT to SOS state. */
void prvPreAlert_2_Sos( void )
{
	/* Remember that the new state is ALERT. That state will be restored if the device resets before the state is
	   acknowledged. */
	enum xCTRL_STATE	xCtrlStateTmp;
	
	xCtrlStateTmp = CTRL_SOS;
	vConfigWriteArray( ( unsigned char * )&xNvdsConfig.xCtrlStateBackup, ( unsigned char * )&xCtrlStateTmp, sizeof( enum xCTRL_STATE ) );
	
	prvPreAlert_2_Alert_Common();
}
/*-----------------------------------------------------------*/

/* Common actions when transitioning from PREALERT to ALERT or SOS state. */
void prvPreAlert_2_Alert_Common( void )
{
	enum xGSM_CMD			xGsmCmd;

	/* Stop vibration motor. */
	vStopPrealertVibrate();
	
	/* Let the motion detection perform the necessary settings for ALERT state. */
	vMDEnterAlert();	
	
	/* Set the 'push immediate' flag to let the GSM task know that we do not require a fix for the first packet. */
	bPushPacketImmediately = true;	

	/* Vibrate to acknowledge the ALERT/SOS. */
	/* Start with a 2s pause. */
	vTaskDelay( ALERT_ACK_VIBR_PAUSE );
	/* Vibrate once during 3 seconds. */
	vCustomVibrate( ALERT_ACK_VIBR_ON, 0, 1 );

	/* Tell the GSM task to send an alert or SOS message. */
	xGsmCmd = GSM_SEND_ALERT_SOS;
	xQueueSend( xGsmCmdQueue, &xGsmCmd, 0 ); 
	
	/* Request entering BLE combined advertising/scan mode to emit a distress beacon. */
	prvCtrlStartBleAlertLoc();
}
/*-----------------------------------------------------------*/

/* Actions when transitioning from ALERT to INACTIVE state. */
void prvAlert_2_Inactive( void )
{
	enum xCTRL_STATE	xCtrlStateTmp;

	/* Remember that the ALERT has been acknowledged. */
	xCtrlStateTmp = CTRL_INACTIVE;
	vConfigWriteArray( ( unsigned char * )&xNvdsConfig.xCtrlStateBackup, ( unsigned char * )&xCtrlStateTmp, sizeof( enum xCTRL_STATE ) );

	/* Set the accelerometer to standby mode to be able to safely configure it. */
	vAccPowerDown();

	/* Clear all interrupts. */
	vAccClearInterrupts();

	/* Set the accelerometer to step and SOS detection. */
	vMDConfigureStepSOS();

	/* Set the accelerometer to abnormal position detection. */
	vMDConfigureAbnormal();

	/* Let the motion detection perform the necessary settings for INACTIVE state. */
	vMDEnterInactive();
	
	/* Set the accelerometer to motion detection. */
	vMDConfigureMotion();
	
	/* Power-up the accelerometer. */
	vAccPowerUp();		
	
	/* Request entering BLE combined advertising/scan mode to emit a distress beacon. */
	prvCtrlStopBleAlertLoc();
	
	/* Request starting the normal BLE scan. */
	prvCtrlStartBleLoc();
}
/*-----------------------------------------------------------*/

/* Actions when transitioning from ALERT to PREALERT state. */
void prvAlert_2_PreAlert( void )
{
	enum xCTRL_STATE	xCtrlStateTmp;
	
	/* Remember that the ALERT has been acknowledged. */
	xCtrlStateTmp = CTRL_INACTIVE;
	vConfigWriteArray( ( unsigned char * )&xNvdsConfig.xCtrlStateBackup, ( unsigned char * )&xCtrlStateTmp, sizeof( enum xCTRL_STATE ) );

	/* First, enter normal state. */
	prvAlert_2_Inactive();
	
	/* Then PREALERT. */
	prvNormal_2_PreAlert();
}
/*-----------------------------------------------------------*/

/* State machine action when transitioning from any normal state (STILL, INACTIVE, ACTIVE) to the STANDBY state. */
void prvNormal_2_Standby( void )
{
	enum xGSM_CMD			xGsmCmd;

	/* Stop all activity as in an unused state. */
	prvNormal_2_Unused();
	
	/* Additionally, shut down the cellular module. */
	xGsmCmd = GSM_IN_STANDBY;
	xQueueSend( xGsmCmdQueue, &xGsmCmd, 0 ); 
	
}
/*-----------------------------------------------------------*/

/* Main control state machine of the tracker device. Please see FS_Tracker_<date>.docx for details.
   The implementation evaluates the command received in the current state, decides upon an action and moves to a new state. */
void vCtrlStateMachine( enum xCTRL_EVENT xCtrlEvent )
{
	unsigned portBASE_TYPE	xIdx;
	void					( *pvAction )( void );
	enum xGSM_CMD			xGsmCmd;
	
	/* Walk through the table describing the state machine and find an entry corresponding to the current state and the event. */
	xIdx = 0;
	
	while ( xIdx < ( unsigned portBASE_TYPE )( sizeof( xCtrlStateMachine ) / sizeof( struct xSTATE_MACHINE ) ) )
	{
		if (   ( xCtrlState == xCtrlStateMachine[ xIdx ].xCurrentState )
			&& ( xCtrlEvent == xCtrlStateMachine[ xIdx ].xEvent ) )
		{
			enum xCTRL_STATE	xNextCtrlState;
			
			/* Found a corresponding entry. */
			/* Execute the action - if any. */
			pvAction = ( void ( * )( void ) )xCtrlStateMachine[ xIdx ].prvAction;	
			if ( pvAction != NULL )
			{
				pvAction();			
			}
			
			/* Read the next state. */
			xNextCtrlState = xCtrlStateMachine[ xIdx ].xNextState;
			
			/* Update the system statistics counters. */
			vSystemStateStats( SYSSTATS_UPDATE, xNextCtrlState );				
			
			/* Perform the state transition. */
			xCtrlState = xNextCtrlState;
			
			/* Inform the GSM task that the position interval timer has been updated. */
			xGsmCmd = GSM_POS_INT_UPDATE;
			xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmCmd, 0 ); 
			
			switch ( xCtrlState )
			{
				case CTRL_STANDBY:		V_TRACE_PRINT_BYTE( TRACE_CTRL_STANDBY, 		xCtrlEvent, TRACE_UART );			break;
				case CTRL_ACTIVE:		V_TRACE_PRINT_BYTE( TRACE_CTRL_ACTIVE, 		 	xCtrlEvent, TRACE_UART );			break;
				case CTRL_INACTIVE:		V_TRACE_PRINT_BYTE( TRACE_CTRL_INACTIVE, 		xCtrlEvent, TRACE_UART );			break;
				case CTRL_SLEEP:		V_TRACE_PRINT_BYTE( TRACE_CTRL_SLEEP, 			xCtrlEvent, TRACE_UART );			break;
				case CTRL_OOO:			V_TRACE_PRINT_BYTE( TRACE_CTRL_OUT_OF_OFFICE,  	xCtrlEvent, TRACE_UART );			break;
				case CTRL_CHRG_OOO:		V_TRACE_PRINT_BYTE( TRACE_CTRL_CHARGING_OOO, 	xCtrlEvent, TRACE_UART );			break;
				case CTRL_CHRG_NRML:	V_TRACE_PRINT_BYTE( TRACE_CTRL_CHARGING_NORMAL, xCtrlEvent, TRACE_UART );			break;
				case CTRL_STILL:		V_TRACE_PRINT_BYTE( TRACE_CTRL_STILL, 			xCtrlEvent, TRACE_UART );			break;
				case CTRL_PREALERT:		V_TRACE_PRINT_BYTE( TRACE_CTRL_PREALERT, 		xCtrlEvent, TRACE_UART );			break;
				case CTRL_ALERT:		V_TRACE_PRINT_BYTE( TRACE_CTRL_ALERT, 			xCtrlEvent, TRACE_UART_AND_FILE );	break;
				case CTRL_PRESOS:		V_TRACE_PRINT_BYTE( TRACE_CTRL_PRESOS, 		 	xCtrlEvent, TRACE_UART );			break;
				case CTRL_SOS:			V_TRACE_PRINT_BYTE( TRACE_CTRL_SOS, 			xCtrlEvent, TRACE_UART_AND_FILE );	break;
			}
			
			/* Exit function. */
			return;
		}
		
		/* Try next entry. */
		xIdx++;
	}	
}
/*-----------------------------------------------------------*/

static portTASK_FUNCTION( vCtrlTask, pvParameters )
{
	enum xCTRL_EVENT				xCtrlEvent;
	enum xGSM_CMD					xGsmCmd;
	
	/* Just to stop compiler warnings. */
	( void ) pvParameters;
	
	vTaskSetApplicationTaskTag( NULL, ( void * ) CTRL_TASK_TAG );
	
	/* NVDS driver initialisation. Requires the SD to be initialised and the scheduler to be running. */
	vFDSInit();
	
	/* Initialise the Trace handler. */
	vTraceInit();

	/* Initialise the configuration handling. */
	vConfigInit();

	/* Control state init. */
	{
		enum xCTRL_STATE				xCtrlStateBackup;
		
		vConfigReadArray( ( unsigned char * )&xCtrlStateBackup, ( unsigned char * )&xNvdsConfig.xCtrlStateBackup, sizeof( enum xCTRL_STATE ) );
		if ( xCtrlStateBackup == CTRL_INACTIVE )
		{
			if ( bGetOnCharger() )
			{
				/* Charging state detected. */
				xCtrlState = CTRL_CHRG_NRML;
				prvCtrlStopBleLoc();
			}
			else
			{
				xCtrlState = CTRL_INACTIVE;
				
				/* Activate the device, consistent with the state INACTIVE. 
				   Note that for this the timers need to be initialised. */
				prvUnused_2_Inactive();
			}
		}
		else
		{
			/* Juste before the system's reset the control state was either ALERT or SOS.
			   Restore these states. */
			xCtrlState = CTRL_INACTIVE;
			prvUnused_2_Inactive();
			
			xCtrlState = xCtrlStateBackup;
			
			/* Let the motion detection perform the necessary settings for ALERT state. */
			vMDEnterAlert();	
			
			/* Set the 'push immediate' flag to let the GSM task know that we do not require a fix for the first packet. */
			bPushPacketImmediately = true;	

			/* Tell the GSM task to send an alert or SOS message. */
			xGsmCmd = GSM_SEND_ALERT_SOS;
			xQueueSend( xGsmCmdQueue, &xGsmCmd, 0 ); 
			
			/* Request entering BLE combined advertising/scan mode to emit a distress beacon. */
			prvCtrlStartBleAlertLoc();
		}
	}

	/* Let the LED start blinking. */
	vStartLedBlinking();	

	/* Once FreeRTOS and the application is up and running, report the free heap space. */
	{
		char	cHwVersion[ LEN_HW_VERSION + 1 ] = HW_VERSION;

		V_TRACE_PRINT_STRG( TRACE_VERSION, cGitVersion, TRACE_UART );
		cHwVersion[ LEN_HW_VERSION ] = 0;
		vTracePrintSp( cHwVersion );
	}

	/* Search for the bootloader version and display it. */
	{
		signed char	*pcBootloaderVersionStrg;
		
		if ( bLocateBootloaderVersion( &pcBootloaderVersionStrg ) )
		{
			V_TRACE_PRINT_STRG( TRACE_BL_VERSION, pcBootloaderVersionStrg, TRACE_UART );			
		}
	}
	
	/* The heap should now be fully allocated. We do not dynamically allocate any further memory. 
	   Anything left over now is wasted memory. */
	V_TRACE_PRINT_SHORT( TRACE_HEAP, ( short )xPortGetFreeHeapSize(), TRACE_UART );
	
	/* Log trace message. */
	V_TRACE_PRINT_LONG( TRACE_RESET, 				ulResetReasonCopy, 				 	 	TRACE_UART_AND_FILE );
	V_TRACE_PRINT_LONG( TRACE_WD_RESETS, 			uxErrorRecord.ulWDResetCount, 		 	TRACE_UART );
	V_TRACE_PRINT_LONG( TRACE_SW_RESETS, 			uxErrorRecord.ulSWResetCount, 		 	TRACE_UART );
	V_TRACE_PRINT_LONG( TRACE_POR_RESETS, 			uxErrorRecord.ulPOResetCount, 		 	TRACE_UART );
	V_TRACE_PRINT_LONG( TRACE_LOCKUP_RESETS, 		uxErrorRecord.ulLockupResetCount, 	 	TRACE_UART );
	V_TRACE_PRINT_LONG( TRACE_OTHER_RESETS, 		uxErrorRecord.ulOtherResetCount, 	 	TRACE_UART );
	V_TRACE_PRINT_LONG( TRACE_RTOS_SYSTEM_ERRORS, 	uxErrorRecord.ulSystemErrorCount, 	 	TRACE_UART );
	V_TRACE_PRINT_LONG( TRACE_HARDFAULTS,	 		uxErrorRecord.ulHardfaultCount, 	 	TRACE_UART );
	V_TRACE_PRINT_LONG( TRACE_STACK_OVERFLOWS,		uxErrorRecord.ulStackOverflowCount,  	TRACE_UART );
	V_TRACE_PRINT_LONG( TRACE_SD_ASSERT, 			uxErrorRecord.ulSDAssertCount, 		 	TRACE_UART );
	V_TRACE_PRINT_LONG( TRACE_SD_MEM_ERROR, 		uxErrorRecord.ulSDMemAccCount, 		 	TRACE_UART );
	V_TRACE_PRINT_LONG( TRACE_NRF_SDK_ASSERT, 		uxErrorRecord.ulnRFSDKAssertCount,   	TRACE_UART );	
	V_TRACE_PRINT_LONG( TRACE_NRF_SDK_ERROR,		uxErrorRecord.ulnRFSDKErrorCount,  	 	TRACE_UART );		
	V_TRACE_PRINT_LONG( TRACE_NRF_UNKNOWN_ERROR,	uxErrorRecord.ulnRFUnknownErrorCount,	TRACE_UART );	
	V_TRACE_PRINT_LONG( TRACE_BOOT_ERROR,			uxErrorRecord.ulUnsuccessfulBootCount,	TRACE_UART );	

	if ( bWatchdogRecovery )
	{
		/* Log the watchdog's origin address. */
		V_TRACE_PRINT_LONG( TRACE_ORIGINATING_ADDRESS, ulErrorPc, TRACE_UART_AND_FILE );
	}
	
	if ( bSoftResetRecovery )
	{
		vTraceFault( ulErrorId, ulErrorPc, &xErrorInfo, uiErrorStack, TRACE_UART_AND_FILE );
	}
	
	/* Set the non-initialised section entirely to '0'. */
	ulErrorId = 0;
	ulErrorPc = 0;
	memset( &xErrorInfo, 0, sizeof( struct xERROR_INFO ) );
	{
		unsigned portBASE_TYPE	uxIdx;
		
		for ( uxIdx = 0; uxIdx < ERROR_STACK_DUMP_LEN; uxIdx++ )
		{
			uiErrorStack[ uxIdx ] = 0;
		}
	}
	
	NRF_LOG_INFO( "Task started." );
	NRF_LOG_FLUSH();

	while ( true )
	{
		/* Wait for an event. */
		if ( xQueueReceive( xCtrlEventQueue, &xCtrlEvent, portMAX_DELAY ) != errQUEUE_EMPTY )
		{
			switch ( xCtrlEvent )
			{
				case CTRL_ACC_WU_DET1:
										/* Event coming from the accelerometer interrupt. */
										if ( vAccWUHandlerInt1 != NULL )
										{
											vAccWUHandlerInt1(); /* Execute the handler callback for interrupt 1. */
										}	
										break;
				
				case CTRL_ACC_WU_DET2:
										/* Event coming from the timer interrupt. */
										if ( vAccWUHandlerInt2 != NULL )
										{
											vAccWUHandlerInt2(); /* Execute the handler callback. */
										}	
										break;
										
				case CTRL_NO_ABNORMAL:
										/* The device is in a zone where no abnormal position detection is necessary. 
										   (Re-)launch the timer which blocks abnormal position detections in the motiondetection module. */
										xTimerChangePeriod( xNoAbnPosTimer, NO_ABN_POS_DET_INTERVAL, portMAX_DELAY );
										V_TRACE_PRINT( TRACE_CTRL_ABNORMAL_POS_DET_BLOCKED, TRACE_UART );
										break;

				case CTRL_IMMOBILITY:
+										/* The device is in a zone where immobility detection is necessary. 
+										   (Re-)launch the timer which allows immobility detection in the motiondetection module. */
										xTimerChangePeriod( xImmobilityTimer, IMMOBILITY_INTERVAL, portMAX_DELAY );
										V_TRACE_PRINT( TRACE_CTRL_IMMOBILITY, TRACE_UART );
										break;

				case CTRL_ACC_VIBR_TEST:
										/* Run the accelerometer / vibrator self-test.
										   As this test changes the accelerometer settings, it can only be done in 
										   a CHARGING state. Furthermore, it must be executed in the CTRL task 
										   to avoid collisions between settings coming from motion-detection and
										   the test.
										   While the test is running, CTRL remains in the same state. */
										if ( ( xCtrlState == CTRL_CHRG_NRML ) || ( xCtrlState == CTRL_CHRG_OOO ) )
										{
											V_TRACE_PRINT( TRACE_CTRL_ACCELEROMETER_SELF_TEST, TRACE_UART );
											
											xCtrlAccVibrTestResult = 0;
											
											if ( bAccTest( &usAccErrorCode ) )
											{
												xCtrlAccVibrTestResult |= ACC_PASSED;
											}
											
											/* Vibrator self-test. */
											V_TRACE_PRINT( TRACE_CTRL_VIBRATION_MOTOR_SELF_TEST, TRACE_UART );

											if ( bVibrTest() )
											{
												xCtrlAccVibrTestResult |= VIBR_PASSED;
											}
										}	
										else
										{
											xCtrlAccVibrTestResult = COULD_NOT_RUN;
										}
										
										/* Inform the GSM task that the test has run. */
										xGsmCmd = GSM_ACC_VIBR_TEST_DONE;
										xQueueSend( xGsmCmdQueue, &xGsmCmd, 0 ); 
										break;
										
				default:				/* All other events are routed to the control state machine. */
										/* Execute the Ctrl state machine. */
										vCtrlStateMachine( xCtrlEvent );				
			}	
		}
	}
	
	/* Unreachable code. Place all function calls here for unreferenced functions which should be kept in the binary. */
	vAccDumpStatus();	
}
/*-----------------------------------------------------------*/