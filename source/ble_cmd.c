/*
 * Tracker Firmware
 *
 * BLE Relayed Command Handler
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "tracker.h"

/* nRF SDK include files */
#define NRF_LOG_MODULE_NAME 		BLE_CMD
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
#include "drv_adc.h"
#include "drv_aes.h"
#include "drv_nvm.h"
#include "drv_uart.h"
#include "drv_vibrator.h"

#include "ble_cmd.h"
#include "ble_ctrl.h"
#include "charger.h"
#include "config.h"
#include "ctrl.h"
#include "evacuation.h"
#include "gsm.h"
#include "drv_led.h"
#include "parser.h"
#include "rtc.h"
#include "trace.h"
#include "tracemsg.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Initialize the BLE command handler. */
void vInitBleCmdHandler( void );

/* Timer callback for the relayed command timer. */
void prvBleRecvdCmdCallback( TimerHandle_t xTimer );

/* Parse the BLE command given as part of the BLE relay command. */
enum xAT_MSG_ID xParseBleCmd( signed char **ppcCmdStrg );

/* Handle a command relayed from the server and received via BLE. 
   The same command can only be treated once every 60 seconds. 
*/
void vTreatBleCmd( enum xAT_MSG_ID uxRelayedCmdIdx, unsigned short usRelayedCmdParam );
/*-----------------------------------------------------------*/

/* Local variables. */
/* Timer handle for the timer supervising sending of the relayed command. */
TimerHandle_t 		xBcnReceivedCmdTimer;

/* BLE relayed command index and parameter. */
enum xAT_MSG_ID		xBleRlyCmd;
unsigned short 		usBleRlyCmdParam;
/*-----------------------------------------------------------*/

/* Initialize the BLE command handler. */
void vInitBleCmdHandler( void )
{
	/* Create the timer for relayed commands. */
	xBcnReceivedCmdTimer = xTimerCreate
							( "",			 				/* Timer name for debug. */
							  T_BLE_RLYCMD,					/* Timer period in ticks. */
							  pdFALSE,						/* One-shot. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvBleRecvdCmdCallback		/* Callback for the relayed command timer. */
							);
							
	/* Initalize the ID for relayed server commands to NONE so that the next command can be treated. */
	xBleRlyCmd = AT_NOMSG;
	usBleRlyCmdParam = 0;

	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* Timer callback for the relayed command timer. Upon expiry of the timer, a new BLE command of the same type may be sent.
   CAUTION: This function is running in the timer task!
*/
void prvBleRecvdCmdCallback( TimerHandle_t xTimer )
{
	( void )xTimer;
	
	/* Allow treatment od a new server command via BLE. */
	xBleRlyCmd = AT_NOMSG;
	usBleRlyCmdParam = 0;
}
/*-----------------------------------------------------------*/

/* Parse the BLE command given as part of the BLE relay command.
   Format:	   
		RELAY_CMD=<imei>,<cmd>,<cmd param> 

   Examples:
		RELAY_CMD=358578080179290,ALERT_ACK
		RELAY_CMD=358578080179290,LED,0x0107
*/
enum xAT_MSG_ID xParseBleCmd( signed char **ppcCmdStrg )
{
	unsigned portBASE_TYPE	xRespIdx;
	bool					atRespFound;
	unsigned portBASE_TYPE	uxStrgIdx;
	signed char				cAtResp;
	signed char				*pcAtRef;
	signed char				cAtRef;
		
	xRespIdx = 0;
	atRespFound = false;
	
	/* Scan all strings in the AT response table to find a match. */
	while ( ( !atRespFound ) && ( xRespIdx < ( unsigned portBASE_TYPE )( xNumAtResp ) ) )
	{
		/* Check if the response pointed to by xRespIdx is a match. Skip the bracket '{'. */
		uxStrgIdx = 0;
		do
		{
			cAtResp = *( *ppcCmdStrg + uxStrgIdx );
			pcAtRef = ( signed char * )xAtResp[ xRespIdx ].pcAtResponse;
			cAtRef = *( pcAtRef + uxStrgIdx + 1 );
			/* A '*' means: match any character. */
			if ( ( cAtResp != 0 ) && ( cAtRef == '*' ) )
			{
				cAtResp = cAtRef;
			}
			uxStrgIdx++;
		} 
		while (    ( cAtResp != 0 ) 
				&& ( cAtRef != 0 )
				&& ( cAtRef != '=' )
				&& ( cAtRef == cAtResp ) );

		/* A match has been found if the end of the reference string has been reached and 
		   no mismatch has been encountered before. */
		if ( ( cAtRef == 0 ) || ( cAtRef == '=' ) )
		{
			atRespFound = true;
			uxStrgIdx--;
		}
		else
		{
			/* No match: select next entry in the response table. */
			xRespIdx++;
		}
	}
	
	/* Treat the response, if a match was found in the table. The resulting index is xRespIdx. */
	if ( atRespFound )
	{
		/* Update the pointer to the string containing the command to let it point atfer the command string. */
		*ppcCmdStrg += uxStrgIdx;
		
		return xAtResp[ xRespIdx ].xAtMsgId;
	}
	else
	{
		return AT_NOMSG;
	}
}
/*-----------------------------------------------------------*/


/* Handle a command relayed from the server and received via BLE. 
   The same command can only be treated once every 60 seconds. 
*/
void vTreatBleCmd( enum xAT_MSG_ID uxRelayedCmdIdx, unsigned short usRelayedCmdParam )
{
	enum xBLE_CMD			xBleCmd;
	unsigned portBASE_TYPE	xGsmPosIntUpdateCmd;
	enum xCTRL_EVENT		xCtrlEvent;
	
	if ( ( xBleRlyCmd == uxRelayedCmdIdx ) && ( usBleRlyCmdParam == usRelayedCmdParam ) )
	{
		/* Refuse accepting commands which have already been treated. */
		return;
	}
	
	/* Remember the command index so that subsequent reception of the same type can be refused. */
	xBleRlyCmd = uxRelayedCmdIdx;
	usBleRlyCmdParam = usRelayedCmdParam;
	( void )xTimerStart( xBcnReceivedCmdTimer, portMAX_DELAY );
	
	/* GSM command to update postion interval, in case it is needed. */
	xGsmPosIntUpdateCmd = GSM_POS_INT_UPDATE;
	
	switch ( uxRelayedCmdIdx )
	{
		case SRVCMD_RESET:					/* Reset device. */
											/* Shut down the GSM/GSP module. */
											vGSMModulePowerDown( SOFT_POWER_OFF );
											/* Execute reset. */
											V_SYSTEM_RESET( RESET_FROM_BLE_COMMAND );
											break;
											
		case SRVCMD_WR_RAT:					vConfigWriteShort( &xNvdsConfig.usCfgRat, usRelayedCmdParam );				/* Radio Access Technology. */
											break;
		case SRVCMD_WR_ATT_TO:				/* Network attachment time-out. The value is in 10ms steps whereas the configuration stored in NVDS is in 1ms steps (based on the FreeRTOS tick rate). */
											vConfigWriteLong( &xNvdsConfig.ulRegTimeOut, 10UL * ( unsigned long )usRelayedCmdParam );		
											break;
		case SRVCMD_WR_ACT_INT:				vConfigWriteShort( &xNvdsConfig.usActivePositionSendInterval, usRelayedCmdParam );
											xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmPosIntUpdateCmd, 0 );				/* Notify position interval update. */
											break;
		case SRVCMD_WR_INACT_INT:			vConfigWriteShort( &xNvdsConfig.usInactivePositionSendInterval, usRelayedCmdParam );
											xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmPosIntUpdateCmd, 0 );				/* Notify position interval update. */
											break;
		case SRVCMD_WR_STILL_INT:			vConfigWriteShort( &xNvdsConfig.usStillPositionSendInterval, usRelayedCmdParam );
											xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmPosIntUpdateCmd, 0 );				/* Notify position interval update. */
											break;
		case SRVCMD_WR_SLEEP_INT:			vConfigWriteShort( &xNvdsConfig.usSleepPositionSendInterval, usRelayedCmdParam );
											xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmPosIntUpdateCmd, 0 );				/* Notify position interval update. */
											break;
		case SRVCMD_WR_OOO_INT:				vConfigWriteShort( &xNvdsConfig.usOooPositionSendInterval, usRelayedCmdParam );
											xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmPosIntUpdateCmd, 0 );				/* Notify position interval update. */
											break;
		case SRVCMD_WR_CHRG_INT:			vConfigWriteShort( &xNvdsConfig.usChargingPositionSendInterval, usRelayedCmdParam );
											xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmPosIntUpdateCmd, 0 );				/* Notify position interval update. */
											break;
		case SRVCMD_WR_ALERT_INT:			vConfigWriteShort( &xNvdsConfig.usAlertPositionSendInterval, usRelayedCmdParam );
											xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmPosIntUpdateCmd, 0 );				/* Notify position interval update. */
											break;
		case SRVCMD_WR_PD_INT:				vConfigWriteShort( &xNvdsConfig.usModulePwrDownInt, usRelayedCmdParam );
											break;
											
		case SRVCMD_WR_STEP_ACC_THR:		vConfigWriteShort( &xNvdsConfig.usStepAccelerationThreshold, usRelayedCmdParam );
											break;
		case SRVCMD_WR_STEP_MIN_INT:		vConfigWriteShort( &xNvdsConfig.usStepMinInterval, usRelayedCmdParam );
											break;
		case SRVCMD_WR_STEP_MAX_INT:		vConfigWriteShort( &xNvdsConfig.usStepMaxInterval, usRelayedCmdParam );
											break;
		case SRVCMD_WR_ACTIVE_NUM_STEPS:	vConfigWriteShort( &xNvdsConfig.usActiveNumberSteps, usRelayedCmdParam );
											break;
											
		case SRVCMD_WR_SOS_ACC_THR:			vConfigWriteShort( &xNvdsConfig.usSosAccelerationThreshold, usRelayedCmdParam );
											break;
		case SRVCMD_WR_SOS_MIN_X_ACC_THR:	vConfigWriteShort( &xNvdsConfig.usSosMinXAccelerationThreshold, usRelayedCmdParam );
											break;
		case SRVCMD_WR_SOS_MIN_Z_ACC_THR:	vConfigWriteShort( &xNvdsConfig.usSosMinZAccelerationThreshold, usRelayedCmdParam );
											break;
		case SRVCMD_WR_SOS_MAX_Z_ACC_THR:	vConfigWriteShort( &xNvdsConfig.usSosMaxZAccelerationThreshold, usRelayedCmdParam );
											break;
		case SRVCMD_WR_SOS_MIN_INT:			vConfigWriteShort( &xNvdsConfig.usSosMinInterval, usRelayedCmdParam );
											break;
		case SRVCMD_WR_SOS_MAX_INT:			vConfigWriteShort( &xNvdsConfig.usSosMaxInterval, usRelayedCmdParam );
											break;
		case SRVCMD_WR_SOS_NUM_PEAKS:		vConfigWriteShort( &xNvdsConfig.usSosNumberPeaks, usRelayedCmdParam );
											break;
											
		case SRVCMD_WR_STILL_DUR:			vConfigWriteShort( &xNvdsConfig.usStillDuration, usRelayedCmdParam );
											break;						
		case SRVCMD_WR_IMMOBILITY_DUR:		vConfigWriteShort( &xNvdsConfig.usImmobilityDuration, usRelayedCmdParam );
											break;	
		case SRVCMD_WR_SLEEP_DUR:			vConfigWriteShort( &xNvdsConfig.usSleepDuration, usRelayedCmdParam );
											break;
											
		case SRVCMD_WR_ABNPOS_Z_ACC_THR:	vConfigWriteShort( &xNvdsConfig.usAbnormalPositionZAccelerationThreshold, usRelayedCmdParam );
											break;
		case SRVCMD_WR_ABNPOS_DUR:			vConfigWriteShort( &xNvdsConfig.usAbnormalPositionDuration, usRelayedCmdParam );
											break;
		case SRVCMD_WR_ABNPOS_MAX_ORIENT:	vConfigWriteShort( &xNvdsConfig.usAbnormalMaxOrientationChg, usRelayedCmdParam );
											break;
											
		case SRVCMD_WR_ALERT_CNCL_METHOD:	vConfigWriteByte( &xNvdsConfig.ucAlertCancelMethod, ( unsigned char )( usRelayedCmdParam & 0xff ) );
											break;
		case SRVCMD_WR_ALERT_CNCL_THR:		vConfigWriteShort( &xNvdsConfig.usAlertCancelThreshold, usRelayedCmdParam );
											break;
		case SRVCMD_WR_ALERT_CNCL_MIN_INT:	vConfigWriteShort( &xNvdsConfig.usAlertCancelMinInterval, usRelayedCmdParam );
											break;
		case SRVCMD_WR_ALERT_CNCL_MAX_INT:	vConfigWriteShort( &xNvdsConfig.usAlertCancelMaxInterval, usRelayedCmdParam );
											break;
		case SRVCMD_WR_ALERT_CNCL_NUM_PEAKS:vConfigWriteShort( &xNvdsConfig.usAlertCancelNumberPeaks, usRelayedCmdParam );
											break;
		case SRVCMD_WR_ALERT_CNCL_TIMEOUT:	vConfigWriteShort( &xNvdsConfig.usAlertCancelTimeout, usRelayedCmdParam );
											break;
		case SRVCMD_WR_SOS_CNCL_TIMEOUT:	vConfigWriteShort( &xNvdsConfig.usSosCancelTimeout, usRelayedCmdParam );
											break;
											
		case SRVCMD_WR_NOABN_ENABLE:		vConfigWriteShort( &xNvdsConfig.usNoAbnEnabled, usRelayedCmdParam );
											break;
		case SRVCMD_WR_NOABN_NUM_PEAKS:		vConfigWriteShort( &xNvdsConfig.usNoAbnCmdPeaks, usRelayedCmdParam );
											break;
		case SRVCMD_WR_NOABN_MAX_PERIOD:	vConfigWriteShort( &xNvdsConfig.usNoAbnMaxPeriod, usRelayedCmdParam );
											break;
		case SRVCMD_WR_NOABN_WINDOW:		vConfigWriteShort( &xNvdsConfig.usAbnPosDetectedWindow, usRelayedCmdParam );
											break;
		case SRVCMD_WR_NOABN_ACK_ON:		vConfigWriteByte( &xNvdsConfig.ucCmdVibrationParamOn, ( unsigned char )( usRelayedCmdParam & 0xff ) );
											break;
		case SRVCMD_WR_NOABN_ACK_OFF:		vConfigWriteByte( &xNvdsConfig.ucCmdVibrationParamOff, ( unsigned char )( usRelayedCmdParam & 0xff ) );
											break;
		case SRVCMD_WR_NOABN_ACK_REP:		vConfigWriteByte( &xNvdsConfig.ucCmdVibrationParamRep, ( unsigned char )( usRelayedCmdParam & 0xff ) );
											break;											
											
		case SRVCMD_WR_BATT_EMPTY_THRES:	vConfigWriteShort( &xNvdsConfig.usBattEmptyThres, usRelayedCmdParam );
											break;
		case SRVCMD_WR_BATT_LOW_THRES:		vConfigWriteShort( &xNvdsConfig.usBattLowThres, usRelayedCmdParam );
											break;
		case SRVCMD_WR_BATT_FULL_THRES:		vConfigWriteShort( &xNvdsConfig.usBattFullThres, usRelayedCmdParam );
											break;
		case SRVCMD_WR_BATT_LOOP_INT:		vConfigWriteShort( &xNvdsConfig.usBattLoopInt, usRelayedCmdParam );
											break;
											
		case SRVCMD_WR_GPS_ENABLE:			vConfigWriteShort( &xNvdsConfig.usGpsEnable, usRelayedCmdParam );
											break;
		case SRVCMD_WR_GPS_FIX_QUALITY:		vConfigWriteShort( &xNvdsConfig.usGPSMinQuality, usRelayedCmdParam );
											break;
		case SRVCMD_WR_GPS_MAX_HACC:		vConfigWriteShort( &xNvdsConfig.usGPSMaxHAcc, usRelayedCmdParam );
											break;
		case SRVCMD_WR_GPS_WAIT_FOR_FIX:	/* The value is in 10ms steps whereas the configuration stored in NVDS is in 1ms steps (based on the FreeRTOS tick rate). */
											vConfigWriteLong( &xNvdsConfig.ulGPSMaxWaitForFix, 10UL * ( unsigned long )usRelayedCmdParam );
											break;
		case SRVCMD_WR_GPS_WAIT_FOR_SAT:	vConfigWriteLong( &xNvdsConfig.ulGPSMaxWaitForSat, 10UL * ( unsigned long )usRelayedCmdParam );
											break;
		case SRVCMD_WR_GPS_PD_INT:			vConfigWriteShort( &xNvdsConfig.usGpsPwrDownInt, usRelayedCmdParam );
											break;
		case SRVCMD_WR_GPS_PSM:				vConfigWriteShort( &xNvdsConfig.usGpsPsm, usRelayedCmdParam );
											break;
		case SRVCMD_WR_GPS_POSREC:			vConfigWriteShort( &xNvdsConfig.usGpsRecording, usRelayedCmdParam );
											break;											
											
		case SRVCMD_WR_BLE_UPD_INT:			if ( ( unsigned char )( ( usRelayedCmdParam >> 8 ) & 0xff ) > 0 )
											{
												vConfigWriteByte( &xNvdsConfig.ucBleBcnUpdInterval, ( unsigned char )( ( usRelayedCmdParam >> 8 ) & 0xff ) );
											}
											break;
		case SRVCMD_WR_BLE_TXPWR:			vConfigWriteByte( &xNvdsConfig.ucBleTxPwr, ( unsigned char )( ( usRelayedCmdParam >> 8 ) & 0xff ) );
											break;
		case SRVCMD_WR_BLE_RSSI_CFG:		vConfigWriteShort( &xNvdsConfig.usBleRssiCfg, usRelayedCmdParam );
											break;
		case SRVCMD_WR_BLE_MIN_BCN:			vConfigWriteShort( &xNvdsConfig.usBleMinBcn, usRelayedCmdParam );
											break;
		case SRVCMD_WR_BLE_IN_DANGER_DUR:	vConfigWriteShort( &xNvdsConfig.usBleInDangerDur, usRelayedCmdParam );
											break;
		case SRVCMD_WR_BLE_FOREIGN_ALERT: 	vConfigWriteShort( &xNvdsConfig.usBleForeignAlert, usRelayedCmdParam );
											break;
		case SRVCMD_WR_BLE_SCAN_FIX_ONLY: 	vConfigWriteShort( &xNvdsConfig.usBleScanDuringFixOnly, usRelayedCmdParam );
											break;
		case SRVCMD_WR_BLE_MAX_WAIT_BCN: 	vConfigWriteShort( &xNvdsConfig.usBLEMaxWaitForBcn, usRelayedCmdParam );
											break;
		case SRVCMD_WR_BLE_STD_ADV_INT: 	vConfigWriteShort( &xNvdsConfig.usBleStdAdvInterval, usRelayedCmdParam );
											break;
		case SRVCMD_WR_BLE_BCN_FILTER:		vConfigWriteShort( &xNvdsConfig.usBleBcnFilter, usRelayedCmdParam );
											break;
											
		case SRVCMD_VIBRATE:				/* Vibrate for the requested duration. */
											xBleCmd = BLE_RELAYED_VIBR_CMD;
											xQueueSend( xBleCmdQueue, &xBleCmd, 0 ); 
											break;
		case SRVCMD_LED:					/* Switch on the LED for the requested duration. */
											xBleCmd = BLE_RELAYED_LED_CMD;
											xQueueSend( xBleCmdQueue, &xBleCmd, 0 ); 
											break;
		case SRVCMD_EVACUATION_STOP:		/* Stop evacuation alert. This will mainly stop the distress beacon. */
											vStopTxEvacuation();
											break;
		case SRVCMD_STANDBY_MODE:			/* Go to standby mode. */
											xCtrlEvent = CTRL_GOTO_STBY;
											xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );
											break;
		case SRVCMD_NORMAL_MODE:			/* Go to normal (operating) mode. */
											xCtrlEvent = CTRL_GOTO_NORMAL;
											xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );
											break;
		case SRVCMD_OOO_MODE:				/* Go to Out-of-office mode. */
											xCtrlEvent = CTRL_GOTO_OOO;
											xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );
											break;
		case SRVCMD_ALERT_ACK:				/* Server acknowledged Alert. */
											xCtrlEvent = CTRL_ALERT_ACK;
											xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );
											break;
		case SRVCMD_GOTO_PREALERT:			/* Server acknowledged Alert. */
											xCtrlEvent = CTRL_GOTO_PREALERT;
											xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );
											break;
		case SRVCMD_WR_SEC_ENABLE:			vConfigWriteShort( &xNvdsConfig.usSecEnable, usRelayedCmdParam );
											break;
		case SRVCMD_WR_LOG_ENABLE:			vConfigWriteShort( &xNvdsConfig.usLogEnable, usRelayedCmdParam );
											break;
		case SRVCMD_WR_MOD_SIDE:			vConfigWriteShort( &xNvdsConfig.usModuleSide, usRelayedCmdParam );
											break;	

		case SRVCMD_WR_DANGER_BCN_THRES:	vConfigWriteByte( &xNvdsConfig.cDangerBcnThres, ( unsigned char )( ( usRelayedCmdParam >> 8 ) & 0xff ) );
											break;				
		case SRVCMD_WR_NOABN_BCN_THRES:		vConfigWriteByte( &xNvdsConfig.cNoAbnBcnThres, ( unsigned char )( ( usRelayedCmdParam >> 8 ) & 0xff ) );
											break;				
		case SRVCMD_WR_PRIVATE_BCN_THRES:	vConfigWriteByte( &xNvdsConfig.cPrivateBcnThres, ( unsigned char )( ( usRelayedCmdParam >> 8 ) & 0xff ) );
											break;	
		case SRVCMD_WR_IMMOBILITY_BCN_THRES:vConfigWriteByte( &xNvdsConfig.cImmobilityBcnThres, ( unsigned char )( ( usRelayedCmdParam >> 8 ) & 0xff ) );
											
		default:							/* Command not supported via BLE. */
											break;
	}
}	