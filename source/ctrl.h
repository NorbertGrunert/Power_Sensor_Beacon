/*
 * Tracker Firmware
 *
 * Control task header file
 *
 */
#ifndef CTRL_H
#define CTRL_H

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"

#define ctrlEVENT_QUEUE_SIZE	( 16 )
#define ctrlEVENT_BLOCKTIME		( ( TickType_t ) ( 5 * portTICKS_PER_SEC) )		/* Wait max. 5 seconds for control event queue to become available. */

/* Define timers for sending position messages in different states. */
/* Default interval (seconds) between two position messages in different states. Set to 0xffff to inibit entirely. */
#define CTRL_ACTIVE_POS_INT		(   1 * 60 )									/* Active state. */
#define CTRL_INACTIVE_POS_INT	(   2 * 60 )									/* Inactive state. */
#define CTRL_STILL_POS_INT		(   3 * 60 )									/* Still state. */
#define CTRL_SLEEP_POS_INT		(  30 * 60 )									/* Sleep state. */
#define CTRL_OOO_POS_INT		(  10 * 60 )									/* Out-Of-Office state. */
#define CTRL_CHARGING_POS_INT	(   5 * 60 )									/* Charging state. */
#define CTRL_ALERT_POS_INT		(       30 )									/* Alert state. */
#define CTRL_STANDBY_POS_INT	( portMAX_DELAY / portTICKS_PER_SEC )			/* Standby state. */

#define NO_ABN_POS_DET_INTERVAL	( ( TickType_t ) ( 60 * portTICKS_PER_SEC) )	/* Time-span during which no abnormal position detection is running in sec. */

#define IMMOBILITY_INTERVAL		( ( TickType_t ) ( 60 * portTICKS_PER_SEC) )	/* Time-span during which the immobility window is active in sec. */

#define	ALERT_ACK_VIBR_PAUSE 	( ( TickType_t ) ( 2 * portTICKS_PER_SEC ) )	/* Pause before starting ALERT acknowledgement vibration. */
#define	ALERT_ACK_VIBR_ON		( 30 ) 											/* Duration of ALERT acknowledgement vibration in 100ms. */

/* Coding of the Control state machine. Please see FS_Packet_<date>.docx for description of the states. 
   CAUTION: The coding of the states is reflected in the XSOLE server software! Also, the order of states
   needs to be consistent with bTransmitGpsPositionPerState and bTransmitBlePositionPerState in ctrl.c. */
enum xCTRL_STATE
{
	CTRL_STANDBY	=  0,		
	CTRL_CHRG_OOO	=  1,		
	CTRL_OOO		=  2,		
	CTRL_CHRG_NRML	=  3,		
	CTRL_ACTIVE		=  4,			
	CTRL_INACTIVE	=  5,		
	CTRL_STILL		=  6,		
	CTRL_PREALERT	=  7,		
	CTRL_ALERT		=  8,		
	CTRL_PRESOS		=  9,		
	CTRL_SOS		= 10,		
	CTRL_SLEEP		= 11
};

/* Events impacting the state of the control state machine. These might be commands from the server to the control state 
   machine to bring the tracker device into a specific state. */
enum xCTRL_EVENT
{
	CTRL_GOTO_STBY		=  0,	/* Go to standby state. */
	CTRL_GOTO_NORMAL	=  1,	/* From Standby state, resume normal operation. */
	CTRL_GOTO_OOO		=  2,	/* Go to Out-of-office state. */
	CTRL_ON_CHRG		=  3,	/* On charger detected. */
	CTRL_OFF_CHRG		=  4,	/* Off charger detected. */
	CTRL_WALK_DET		=  5,	/* Walking detected. */
	CTRL_INACT_DET		=  6,	/* Inactivity detected. */
	CTRL_STILL_DET		=  7,	/* Still detected (no movement at all). */
	CTRL_SLEEP_DET	    = 17,	/* Sleep detected. */
	CTRL_MOVEMENT_DET	=  8,	/* Movement (any) detected. */
	CTRL_ABNORMAL_DET	=  9,	/* Abnormal position detected. */
	CTRL_GOTO_PREALERT	= 20,	/* Server command to enter PREALERT. */
	CTRL_SOS_DET		= 10,	/* Abnormal position detected. */
		                
	CTRL_ALERT_CNCL		= 11,	/* Alert cancel. */
	CTRL_ALERT_CNCL_TO	= 12,	/* Alert cancel timeout reached. */
	CTRL_ALERT_ACK		= 13,	/* Server acknowledge of alert. */	

	CTRL_ACC_WU_DET1	= 14,	/* Accelerometer activity on detector 1. */
	CTRL_ACC_WU_DET2	= 18,	/* Accelerometer activity on detector 2. */
	
	CTRL_NO_ABNORMAL	= 19,	/* No abnormal position detection required. */
	CTRL_IMMOBILITY		= 21,	/* Alert on immobility. */
	
	CTRL_ACC_VIBR_TEST	= 16	/* Accelerometer / vibrator self-test. */
};  

/* Structure of a state machine definition. */
struct xSTATE_MACHINE 
{   
	enum xCTRL_STATE	xCurrentState;				/* Current state. */
	enum xCTRL_EVENT	xEvent;						/* Event. */
	void				( *prvAction )( void );		/* Action upon transition. */
	enum xCTRL_STATE	xNextState;					/* New state. */
};

/* Definitions for self-test results. */
#define COULD_NOT_RUN	( 0x01 )
#define ACC_PASSED		( 0x02 )
#define VIBR_PASSED		( 0x04 )
/*-----------------------------------------------------------*/

/* Public function prototypes. */
extern void vCtrlInit( UBaseType_t uxPriority );

/* Obtain the state of the Control state machine. */
extern enum xCTRL_STATE xGetCtrlState( void );

/* Obtain the name string for the state of the Control state machine. */
extern void vGetCtrlStateName( signed char **const pcStateName, unsigned portBASE_TYPE *uxPacketLength );

/* Returns true if the current device state requires sending any position information in the packt to the server. */
extern bool bCtrlPositionRequired( void );

/* Returns true if the current device state requires sending GPS position information in the packet to the server. 
   This takes positioning using BLE beacons into account. */
extern bool bCtrlGpsPositionRequired( void );

/* Returns true if the current device state requires sending indoor (BLE) position information in the packet to the server. */
extern bool bCtrlBlePositionRequired( void );

/* Get the position send interval from NVDS parameters in function of the state of the Ctrl state machine. */
extern TickType_t xGetCtrlPositionSendInterval( void );
/*-----------------------------------------------------------*/

/* Global variables. */
/* Control state machine event queue. */
extern QueueHandle_t 			xCtrlEventQueue; 	

/* Accelerometer callback for wake-up detection handler on interrupt 1. */
extern void						( *vAccWUHandlerInt1 )( void );

/* Accelerometer callback for wake-up detection handler on interrupt 2. */
extern void						( *vAccWUHandlerInt2 )( void );

/* Accelerometer / vibrator self-test result. */
extern unsigned short 			usAccErrorCode;
extern unsigned portBASE_TYPE 	xCtrlAccVibrTestResult;

/* Timer handle for no abnormal position timer. */
extern TimerHandle_t 			xNoAbnPosTimer;

/* Timer handle for the immobility timer. */
extern TimerHandle_t 			xImmobilityTimer;

#endif