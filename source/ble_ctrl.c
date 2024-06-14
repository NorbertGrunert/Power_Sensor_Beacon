/*
 * Tracker Firmware
 *
 * BLE Control task
 *
 * The control task receives xBLE_CMD commands from other tasks leading to a change in behaviour.
 * Many of these are registered as module state xBLE_MODULE_STATE on reception, other xBLE_MODULE_STATE flags are
 * set directly in other modules:
 *
 *		Module state flag			BLE activity					Description
 *
 *		bBleLocRequest  			RX scan							Request for a BLE localisation.
 *      bBleAlertRequest  			TX SR/LR, short interval		Request to send ALERT or SOS beacons.
 *      bBleInDangerZoneRequest		flag in heartbeat beacon		Module is in danger zone.
 *      bBleHeartBeatRequest		TX LR, long interval			Module  sends heartbeat beacon.
 *      bBleTxSuspended				TX suspended					Suspend all transmissions allowing to e.g. receive GPS without perturbation.
 *		bBleTxRelayCmd				TX SR/LR, short interval		Transmit a relay command.
 *      bBleSoftDeviceOn											Softdevice state. 'On' is required for any BLE activity.
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "tracker.h"

/* nRF SDK include files */
#define NRF_LOG_MODULE_NAME 		BLE_CTRL
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_sdh.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/* Device specific include files. */
#include "custom_board.h"
#include "drv_uart.h"
#include "drv_nvm.h"
#include "drv_aes.h"
#include "drv_adc.h"
#include "drv_vibrator.h"

#include "ble_adreport.h"
#include "ble_cmd.h"
#include "ble_ctrl.h"
#include "ble_main.h"
#include "charger.h"
#include "config.h"
#include "ctrl.h"
#include "evacuation.h"
#include "gsm.h"
#include "gps.h"
#include "drv_led.h"
#include "rtc.h"
#include "trace.h"
#include "tracemsg.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Initialise the BLE task. */
void vBleCtrlInit( UBaseType_t uxPriority );

/* Read the battery voltage in the binary format required for beacon reporting. */
unsigned char ucReadBleBatteryVoltage( void );

/* Timer callback for periodic beacon data updates. */
static void vBcnUpdateCallback( TimerHandle_t xTimer );

/* Timer callback for the relayed command timer. */
static void vRelayedCmdCallback( TimerHandle_t xTimer );

/* Lock the BLE module for control by the GSM process. In this case, the BLE process is not allowed
   to touch the BLE module in any way. This mechanism is used for BLE module FW update which has to run in the GSM process.
   When set to 1, the BLE module *MUST* be in state BLE_LOC_STOP. */
void vSetBLEModuleLock( void );

/* Start scanning for BLE beacons. The results of the scan are parsed and recorded in the BLE parser task. */
static void vBleScanStart( void );

/* Start BLE advertising. */
static void vBleSetAdvertising( void );

/* Add a byte to the BLE beacon payload. Collect a full block of 16 bytes, encrypt it and send it. */
void vEncrAndSendAdv( unsigned portBASE_TYPE *puxByteIdx, unsigned char *pucChkSum, unsigned char ucPayloadByte );

/* Add a byte to the BLE beacon payload. Collect a full block of 16 bytes, encrypt it and
   send it. */
void vFinishEncrAndSendAdv( unsigned portBASE_TYPE *puxByteIdx, unsigned char *pucChkSum );

/* Build the encryptedBLE short range advertising data payload. */
void vBuildAdvDataS( void );

/* Build the encryptedBLE long range advertising data payload. */
void vBuildAdvDataL( bool bNoPositionInfo );

/* Obtain the BLE module state. */
struct xBLE_MODULE_STATE xGetBleModuleState( void );

/* Suspend all BLE activity: receptions as well as transmissions (blocking). */
void vBlockingSuspendAllBleActivity( void );

/* Suspend all BLE activity: receptions as well as transmissions (non-blocking). */
void vNonBlockingSuspendAllBleActivity( void );

/* Restore the activity to the same state it was in when vBlockingSuspendAllBleActivity() was called. */
void vResumeAllBleActivity( void );

/* Suspend BLE TX activity. */
void vNonBlockingSuspendBleTxActivity( void );

/* Resume BLE TX activity. */
void vResumeBleTxActivity( void );

/* The transmit task as described at the top of the file. */
static portTASK_FUNCTION_PROTO( vBLETask, pvParameters );
/*-----------------------------------------------------------*/

/* Global variables. */
/* GSM command queue handle. The queue is read in the GSM task and filled in the CTRL task. */
QueueHandle_t 				xBleCmdQueue; 

/* Mutex handle to protect UART access to the BLE module. */
SemaphoreHandle_t			xMutexBle = NULL;

/* Status variable reflecting the BLE module state. */
struct xBLE_MODULE_STATE	xBleModuleState;
struct xBLE_MODULE_STATE	xBleModuleStateBackup;
/*-----------------------------------------------------------*/

/* Local variables. */
/* Counting semaphores for  BLE blocking requests. */
SemaphoreHandle_t			xBleAllBlockedCountingSemaphore;
SemaphoreHandle_t			xBleTxBlockedCountingSemaphore;

/* Temporary buffer for advertising payloads to be encrypted. If the data size increases, the buffer size 
   needs to be adjusted to a integer multiple of AES_BLOCK_LENGTH octets. */
unsigned char				ucAdvData[ AES_BLOCK_LENGTH ];

/* Timer handle for the beacon update timer. */
static TimerHandle_t 		xBcnUpdateTimer;

/* Timer handle for the timer supervising sending of the relayed command. */
static TimerHandle_t 		xBcnRelayedCmdTimer;

/* Parameters for relaying commands to remote devices via BLE:
		destination address (IMEI)
		command index
		command parameter				*/
char						cRelayCmdDestination[ 8 ];
unsigned portBASE_TYPE		uxRelayCmdIdx;
bool						bRelayCmdParamPresent;
unsigned short				usRelayCmdParam;
/*-----------------------------------------------------------*/

/* Initialise the BLE task. 
   
   When calling this task, the schduler is not yet running. So all accesses to NVDS configuration items need to be 
   done directly. */
void vBleCtrlInit( UBaseType_t uxPriority )
{
	unsigned portBASE_TYPE		uxIdx;
	
	/* Create a queue from the CTRL task to the GSM task for commands. */
	xBleCmdQueue = xQueueCreate( bleCMD_QUEUE_SIZE, ( unsigned portBASE_TYPE ) sizeof( enum xBLE_CMD ) );	
	
	/* Create BLE beacon update timer. */
	xBcnUpdateTimer = xTimerCreate
							( "BCNUPD",			 			/* Timer name for debug. */
							  T_BLE_BCNUPDATE * portTICKS_PER_SEC,	/* Timer default value. The actual configured value is set in the task. */
							  pdTRUE,						/* Auto-reload. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  vBcnUpdateCallback			/* Callback for the beacon update timer. */
							);
							
	/* Create the timer for relayed commands. */
	xBcnRelayedCmdTimer = xTimerCreate
							( "BCNRLYD",			 		/* Timer name for debug. */
							  T_BLE_RLYCMD,					/* Timer period in ticks. */
							  pdFALSE,						/* One-shot. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  vRelayedCmdCallback			/* Callback for the relay command timer. */
							);
							
	/* The BLE task is spawned here. */
	xTaskCreate( vBLETask, "BLE_CTRL", bleSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
	
	/* Create a mutex to protect access to the BLE module via the UART so that it can be used from different tasks - notably BLE and GPS. */
	xMutexBle = xSemaphoreCreateMutex();
	
	/* Create counting semaphores for BLE blocking requests. The blocking is released only of the corresponding semaphore has been
	   completely given. */
	xBleAllBlockedCountingSemaphore = xSemaphoreCreateCounting( BLE_SUSP_SEMAPH_MAX, BLE_SUSP_SEMAPH_MAX );	
	xBleTxBlockedCountingSemaphore  = xSemaphoreCreateCounting( BLE_SUSP_SEMAPH_MAX, BLE_SUSP_SEMAPH_MAX );	
	
	/* No alert/scan pending. */
	xBleModuleState.bBleLocRequest 			= false;
	xBleModuleState.bBleAlertRequest 		= false;
	xBleModuleState.bBleInDangerZoneRequest = false;
	xBleModuleState.bBleHeartBeatRequest 	= false;
	xBleModuleState.bBleTxSuspended 		= false;
	xBleModuleState.bBleTxRelayCmd	 		= false;
	xBleModuleState.bBleSoftDeviceOn		= true;
	
	/* Invalidate the data. */
	cGsm_BLE_LAT   [ 0 ] = 0;
	cGsm_BLE_LON   [ 0 ] = 0;
	cGsm_BLE_HDOP  [ 0 ] = 0;
	cGsm_BLE_FIX   [ 0 ] = 0;
	cGsm_BLE_LOCM  [ 0 ] = 0;
	cGsm_BLE_SAT   [ 0 ] = 0;
	cGsm_BLE_TEMP  [ 0 ] = 0;
	cGsm_BLE_STATE [ 0 ] = 0;
	cGsm_BLE_ACCHM [ 0 ] = 0;
	cGsm_BLE_RSSI  [ 0 ] = 0;
	cGsm_BLE_QUAL  [ 0 ] = 0;
	cGsm_BLE_MCC   [ 0 ] = 0;
	cGsm_BLE_MNC   [ 0 ] = 0;
	cGsm_BLE_TAC   [ 0 ] = 0;
	cGsm_BLE_CI    [ 0 ] = 0;
	cGsm_BLE_RAT   [ 0 ] = 0;
	cGsm_BLE_EARFCN[ 0 ] = 0;
	
	/* Initialize relay command fields. */
	for ( uxIdx = 0; uxIdx < 8; uxIdx++ )
	{
		cRelayCmdDestination[ uxIdx ] = 0;
	}
	uxRelayCmdIdx = 0;
	bRelayCmdParamPresent = false;
	usRelayCmdParam = 0;
	
	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}

/*-----------------------------------------------------------*/
/* Read the battery voltage in the binary format required for beacon reporting. */
unsigned char ucReadBleBatteryVoltage( void )
{
	signed long	lLocalBattVoltage;
	
	lLocalBattVoltage = ( signed long )sGetBattVoltage();
	
	/* Take just offset to 3.0V:  3.0V / ADC_GAIN_VBAT / ADC_REF * ADC_MAX_VALUE = 2047
	   The value reported is then 6.205398 * 1.465mV = 9.091mV / LSB so that the value needs to be divided by 6.205398 which
	   is nearly equivalent to multiplying it with 0.16. */

	lLocalBattVoltage -= 2047;					
	lLocalBattVoltage *= 16;
	lLocalBattVoltage /= 100;				
	if ( lLocalBattVoltage < 0 )					/* Clip at 3.0V. */
	{
		lLocalBattVoltage = 0;
	}
	if ( lLocalBattVoltage > 0xff )					/* Clip at max. value. */
	{
		lLocalBattVoltage = 0xff;
	}
	
	return ( unsigned char )lLocalBattVoltage;
}
/*-----------------------------------------------------------*/

/* Timer callback for periodic transmissions to the server and checking operating conditions. 
   CAUTION: This function is running in the timer task!
*/
static void vBcnUpdateCallback( TimerHandle_t xTimer )
{
	enum xBLE_CMD			xBleCmd;
	
	( void )xTimer;
	
	if ( usConfigReadShort( &xNvdsConfig.usBleEnable ) )
	{
		/* Send an update request to the BLE task, but only if the BLE module is ready and not stuck in
		   the power-on sequence. */
		xBleCmd = BLE_UPDATE_BCN;
		xQueueSend( xBleCmdQueue, &xBleCmd, 0 );		
	}
}
/*-----------------------------------------------------------*/

/* Timer callback for the relayed command timer. Upon expiry of the timer, the sending of the relayed command in the 
   beacon is stopped. 
   CAUTION: This function is running in the timer task!
*/
static void vRelayedCmdCallback( TimerHandle_t xTimer )
{
	enum xBLE_CMD			xBleCmd;
	
	( void )xTimer;
	
	/* Cancel sending the relayed command. */
	xBleModuleState.bBleTxRelayCmd = false;
	
	if ( usConfigReadShort( &xNvdsConfig.usBleEnable ) )
	{
		/* Update the beacon data and sending interval. */
		xBleCmd = BLE_UPDATE_BCN;
		xQueueSend( xBleCmdQueue, &xBleCmd, 0 );		
	}
}
/*-----------------------------------------------------------*/

/* Add a byte to the BLE beacon payload. Collect a full block of 16 bytes, encrypt it and
   send it.
*/
void vEncrAndSendAdv( unsigned portBASE_TYPE *puxByteIdx, unsigned char *pucChkSum, unsigned char ucPayloadByte )
{
	unsigned portBASE_TYPE	uxIdx;
	unsigned char			ucEncrAdvData[ AES_BLOCK_LENGTH ];
	
	uxIdx = *puxByteIdx;
	
	/* Store byte in segment buffer. */
	ucAdvData[ uxIdx ] = ucPayloadByte;
	/* Increment segment buffer counter. */
	uxIdx++;
	
	if ( uxIdx == AES_BLOCK_LENGTH )
	{
		/* A full AES block has been collected. Build the checksum and encrypt it. */
		/* First, the checksum on the unencrypted data. */
		for ( uxIdx = 0; uxIdx < AES_BLOCK_LENGTH; uxIdx++ )
		{
			*pucChkSum += ucAdvData[ uxIdx ];
		}

		/* Encrypt the payload. The length parameter must be adapted when changing the beacon payload. */
		if ( !bAES_CBC_EncryptBlock( ucAdvData, ucEncrAdvData, AES_BLOCK_LENGTH ) )
		{
			V_TRACE_PRINT( TRACE_BLE_ENCRYPTION_FAIL, TRACE_UART_AND_FILE );
		}
		
		/* Copy the encrypted payload to the BLE module's advertising data structures. */
		vSetAdvPayload( LR, ucEncrAdvData, AES_BLOCK_LENGTH );
		
		uxIdx = 0;
	}
	
	/* Store the byte index back to the original location. */
	*puxByteIdx = uxIdx;
}
/*-----------------------------------------------------------*/

/* Add a byte to the BLE beacon payload. Collect a full block of 16 bytes, encrypt it and
   send it.
*/
void vFinishEncrAndSendAdv( unsigned portBASE_TYPE *puxByteIdx, unsigned char *pucChkSum )
{
	unsigned portBASE_TYPE	uxIdx;
	unsigned portBASE_TYPE	uxSndIdx;
	unsigned char			ucEncrAdvData[ AES_BLOCK_LENGTH ];
	
	uxIdx = *puxByteIdx;
	
	/* Build the checksum on the remaining unencrypted data. */
	for ( uxSndIdx = 0; uxSndIdx < uxIdx; uxSndIdx++ )
	{
		*pucChkSum += ucAdvData[ uxSndIdx ];
	}
	
	/* Add the checksum to the segment buffer. Whenever vFinishEncrAndSendAdv() is called, we are sure that there is 
	   at least one byte space left in the buffer, else bAES_CBC_Encrypt_Continue_Block() would have already pushed out 
	   the block. */
	ucAdvData[ uxIdx ] = *pucChkSum;
	/* Increment segment buffer counter. */
	uxIdx++;	
	
	/* Finish encrypting the AES block. */
	if ( !bAES_CBC_EncryptBlock( ucAdvData, ucEncrAdvData, uxIdx ) )
	{
		V_TRACE_PRINT( TRACE_BLE_ENCRYPTION_FAIL, TRACE_UART_AND_FILE );
	}
	
	/* Copy the encrypted payload to the BLE module's advertising data structures. */
	vSetAdvPayload( LR, ucEncrAdvData, AES_BLOCK_LENGTH );
	
	/* Stop AES CBC encryption. */
	vAES_CBC_EncryptStop();	
}
/*-----------------------------------------------------------*/

/* Build the encryptedBLE short range advertising data payload. The payload format is:
		octet	length		meaning
		0		1			Length of data block, N, excluding length byte (19).
		1		1			GAP advertising data type, 0xFF for Manufacturer Specific Data
		2		2			The Company Identifier Code (0x0017 in little endian)
		4		1			Beacon descriptor bitfield (1 octet):
								bit 0: EVAC
								bit 1: ALERT
								bit 2: SOS
		5		3			The TRAXxs company ID field (3 octets, 0x75A996). 
		8		8			IMEI	8 octets, 0-padded, e.g. (0)358578080077244  
		16		1			Output power in dBm (1 octet, signed)
		17		1			Evacuation ID
		18		1			Battery level
		19		1			Padding (0)
		
	Starting from octet 4, the advertiser data is temporarily stored in an octet field and encypted in-place.
*/
void vBuildAdvDataS( void )
{
	unsigned char 			ucPlainAdvData[ AES_BLOCK_LENGTH ];
	unsigned char 			ucEncrAdvData[ AES_BLOCK_LENGTH + 4 ];
	signed char				cDesc;
	unsigned portBASE_TYPE	uxIdx;

	
	/* Send the field length depending on the evacuation status followed by the company identifier. GAP advertising 
	   data type is already sent. */
	ucEncrAdvData[ 0 ] = 0x13;
	ucEncrAdvData[ 1 ] = 0xFF;
	ucEncrAdvData[ 2 ] = 0x17;
	ucEncrAdvData[ 3 ] = 0x00;
	
	/* Add the payload descriptor bitfield. No encryption yet. */
	cDesc = 0;
	switch ( xGetCtrlState() )
	{
		case CTRL_ALERT:		cDesc += BLE_ALERT;		break;
		case CTRL_SOS:			cDesc += BLE_SOS;		break;
		default:										break;
	}
	
	if ( bGetEvacOngoing() )
	{
		cDesc += BLE_EVAC;
	}
	
	/* Set the flag for DANGER if the module is believed to be in a danger zone. */
	if ( xBleModuleState.bBleInDangerZoneRequest )
	{
		cDesc += BLE_DANGER;
	}
	
	ucPlainAdvData[ 0 ] = cDesc;
	
	/* Add TRAXXS_ID "96A975" in little endian. */
	ucPlainAdvData[ 1 ] = 0x96;
	ucPlainAdvData[ 2 ] = 0xA9;
	ucPlainAdvData[ 3 ] = 0x75;
	
	/* Modification of the GSM data is protected by a mutex. Taking the mutex here avoids that the data is modified while the 
	   BLE task assembles the beacon payload. */
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );

	/* Add the IMEI. The IMEI contains 15 digits, so the MSB is 0-padded. */
	if ( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID ) != 0 )
	{
		ucPlainAdvData[  4 ] = cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID ) );
		ucPlainAdvData[  5 ] = 16 * cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID +  1 ) ) + cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID +  2 ) );
		ucPlainAdvData[  6 ] = 16 * cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID +  3 ) ) + cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID +  4 ) );
		ucPlainAdvData[  7 ] = 16 * cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID +  5 ) ) + cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID +  6 ) );
		ucPlainAdvData[  8 ] = 16 * cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID +  7 ) ) + cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID +  8 ) );
		ucPlainAdvData[  9 ] = 16 * cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID +  9 ) ) + cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID + 10 ) );
		ucPlainAdvData[ 10 ] = 16 * cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID + 11 ) ) + cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID + 12 ) );
		ucPlainAdvData[ 11 ] = 16 * cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID + 13 ) ) + cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID + 14 ) );
	}
	else
	{
		ucPlainAdvData[  4 ] = 0;
		ucPlainAdvData[  5 ] = 0;
		ucPlainAdvData[  6 ] = 0;
		ucPlainAdvData[  7 ] = 0;
		ucPlainAdvData[  8 ] = 0;
		ucPlainAdvData[  9 ] = 0;
		ucPlainAdvData[ 10 ] = 0;
		ucPlainAdvData[ 11 ] = 0;
	}
	
	/* Add the TX power. */
	ucPlainAdvData[ 12 ] = ucConfigReadByte( &xNvdsConfig.ucBleTxPwr );
	
	/* In EVAC mode, add the EVAC_ID. */
	if ( bGetEvacOngoing() )
	{
		ucPlainAdvData[ 13 ] = ( unsigned char )uxGetEvacId();
	}
	else
	{
		/* 0-pad, if it is not an EVAC-beacon. */
		ucPlainAdvData[ 13 ] = 0;
	}
	
	/* Add the battery voltage. */
	ucPlainAdvData[ 14 ] = ucReadBleBatteryVoltage();
	
	/* Restore write access to the GSM data. */
	xSemaphoreGive( xMutexGsmData );
	
	/* Encrypt the payload. The length parameter must be adapted when changing the beacon payload. 
	   The encypted data is directly stored in the destination buffer. */
	vAES_CBC_EncryptStart( ( uint8_t * )&xNvdsConfig.pcAESKey[ ucConfigReadByte( &xNvdsConfig.ucActiveAESKey ) ] );
	
	if ( !bAES_CBC_EncryptBlock( ucPlainAdvData, &ucEncrAdvData[ 4 ], 15 ) )
	{
		V_TRACE_PRINT( TRACE_BLE_ENCRYPTION_FAIL, TRACE_UART_AND_FILE );
	}
	
	vAES_CBC_EncryptStop();
	
	/* Let the BLE handler copy the data into the softdevice control structures. 
	   Note that the payload length is fixed to 20 bytes as this is the length of a standard BLE payload. */
	vSetAdvPayloadInit( SR );
	vSetAdvPayload( SR, ucEncrAdvData, 20 );
}
/*-----------------------------------------------------------*/

/* Build the BLE long range advertising data payload. The payload format is:
		octet	length		meaning
		0		1			Length of data block, N, excluding length byte (19).
		1		1			GAP advertising data type, 0xFF for Manufacturer Specific Data
		2		2			The Company Identifier Code (0x0017 in little endian)
		4		1			Meshing indication (1 octet, 0x01)
		5		1			Beacon descriptor bitfield (1 octet):
								bit 0: EVAC
								bit 1: ALERT
								bit 2: SOS
								bit 3: RELAYCMD
								bit 4: XSWITCH report
		6		3			The TRAXxs company ID field (3 octets, 0x75A996). 
		9		8			IMEI	8 octets, 0-padded, e.g. (0)358578080077244  
		17		4			DFMAP
		All other fields depend on the contents of DFMAP.
*/
void vBuildAdvDataL( bool bNoPositionInfo )
{
	signed char				cDesc;
	unsigned portBASE_TYPE	uxIdx;
	unsigned portBASE_TYPE	uxIdx2;
	unsigned char			ucPayload[ 5 ];
	unsigned long 			ulConfiguredDFMap;		
	unsigned long 			ulActualDFMap;		
	unsigned portBASE_TYPE	uxPayloadLength;	
	unsigned portBASE_TYPE	uxByteIdx;
	unsigned char			ucChkSum;

	/* Read the configured DFMap from EEPROM. */
	ulConfiguredDFMap = ulConfigReadLong( &xNvdsConfig.ulBleDFMap );
	if ( bNoPositionInfo )
	{
		ulConfiguredDFMap &= BLE_HELLO_FIELDS_MASK;
	}
	ulActualDFMap = 0ll;
	uxPayloadLength = BLE_BCN_PL_ENCR_OVR_LEN;
	
	/* Modification of the GSM data is protected by a mutex. Taking the mutex here avoids that the data is modified while the 
	   BLE task assembles the beacon payload. */
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );
	
	/* Identify which payload fields are filled with data and ready to be sent. Only a few fields are really not always available. */
	/* Bit 0: Tracker TX time-stamp. */
	if ( ulConfiguredDFMap & BLE_BCN_TXTIM_MSK )
	{
		uxPayloadLength += BLE_BCN_TXTIM_LEN;
		ulActualDFMap |= BLE_BCN_TXTIM_MSK;
	}
	/* Bit 1: Conducted BLE output power. */
	if ( ulConfiguredDFMap & BLE_BCN_PWR_MSK )
	{
		uxPayloadLength += BLE_BCN_PWR_LEN;
		ulActualDFMap |= BLE_BCN_PWR_MSK;
	}
	/* Bit 2: Evacuation field, always present in evac state. */
	if ( bGetEvacOngoing() )
	{
		uxPayloadLength += BLE_BCN_EVAC_ID_LEN;
		ulActualDFMap |= BLE_BCN_EVAC_ID_MSK;
	}
	/* Bit 3: Battery field. */
	if ( ulConfiguredDFMap & BLE_BCN_BAT_MSK )
	{
		uxPayloadLength += BLE_BCN_BAT_LEN;
		ulActualDFMap |= BLE_BCN_BAT_MSK;
	}
	/* Bit 19, 6:4 : Standard (AltBeacon or iBeacon) locator beacon field. */
	if ( ulConfiguredDFMap & BLE_BCN_BCN_ALL_MSK )
	{
		uxPayloadLength += BLE_BCN_BCN_LEN * xBleLocBcnBLE.uxBleStdLocBcnCnt;
		ulActualDFMap += BLE_BCN_BCN_ONE_MSK * ( xBleLocBcnBLE.uxBleStdLocBcnCnt & 0x7 );
		ulActualDFMap += BLE_BCN_BCN2_MSK * ( ( xBleLocBcnBLE.uxBleStdLocBcnCnt & 0x8 ) >> 3 );
	}
	/* Bit 7: GPS latitude. */
	if ( ( cGsm_BLE_FIX[ 0 ] != 0 ) && ( cGsm_BLE_LAT[ 0 ] != 0 ) && ( ulConfiguredDFMap & BLE_BCN_LAT_MSK ) )
	{
		uxPayloadLength += BLE_BCN_LAT_LEN;
		ulActualDFMap |= BLE_BCN_LAT_MSK;
	}
	/* Bit 8: GPS longitude. */
	if ( ( cGsm_BLE_FIX[ 0 ] != 0 ) && ( cGsm_BLE_LON[ 0 ] != 0 ) && ( ulConfiguredDFMap & BLE_BCN_LON_MSK ) )
	{
		uxPayloadLength += BLE_BCN_LON_LEN;
		ulActualDFMap |= BLE_BCN_LON_MSK;
	}
	/* Bit 9: GPS fix. */
	if ( ( cGsm_BLE_FIX[ 0 ] != 0 ) && ( ulConfiguredDFMap & BLE_BCN_FIX_MSK ) )
	{
		uxPayloadLength += BLE_BCN_FIX_LEN;
		ulActualDFMap |= BLE_BCN_FIX_MSK;
	}
	/* Bit 10: Localisation method. */
	if ( ( cGsm_BLE_LOCM[ 0 ] != 0 ) && ( ulConfiguredDFMap & BLE_BCN_LOCM_MSK ) )
	{
		uxPayloadLength += BLE_BCN_LOCM_LEN;
		ulActualDFMap |= BLE_BCN_LOCM_MSK;
	}
	/* Bit 11: GPS Horizontal Dilution of Precision. */
	if ( ( cGsm_BLE_FIX[ 0 ] != 0 ) && ( cGsm_BLE_HDOP[ 0 ] != 0 ) && ( ulConfiguredDFMap & BLE_BCN_HDOP_MSK ) )
	{
		uxPayloadLength += BLE_BCN_HDOP_LEN;
		ulActualDFMap |= BLE_BCN_HDOP_MSK;
	}
	/* Bit 12: GPS horizontal accuracy. */
	if (   ( ( cGsm_BLE_FIX[ 0 ] != 0 ) || ( cGsm_BLE_LOCM[ 0 ] != '1' ) )
		&& ( cGsm_BLE_ACCHM[ 0 ] != 0 ) && ( ulConfiguredDFMap & BLE_BCN_ACCH_MSK ) )
	{
		uxPayloadLength += BLE_BCN_ACCH_LEN;
		ulActualDFMap |= BLE_BCN_ACCH_MSK;
	}
	/* Bit 13: GPS satellites in view. */
	if ( ( cGsm_BLE_FIX[ 0 ] != 0 ) && ( cGsm_BLE_SAT[ 0 ] != 0 ) && ( ulConfiguredDFMap & BLE_BCN_SV_MSK ) )
	{
		uxPayloadLength += BLE_BCN_SV_LEN;
		ulActualDFMap |= BLE_BCN_SV_MSK;
	}
	/* Bit 14: Device temperature. */
	/* TL230 and newer read the temperature before filling in the beacon payload. */
	if ( ulConfiguredDFMap & BLE_BCN_TEMP_MSK )
	{
		uxPayloadLength += BLE_BCN_TEMP_LEN;
		ulActualDFMap |= BLE_BCN_TEMP_MSK;
	}
	/* Bit 15: Device state. */
	if ( ulConfiguredDFMap & BLE_BCN_STATE_MSK )
	{
		uxPayloadLength += BLE_BCN_STATE_LEN;
		ulActualDFMap |= BLE_BCN_STATE_MSK;
	}
	/* Bit 16: GSM debug information. */
	if ( ( cGsm_BLE_MCC[ 0 ] != 0 ) && ( ulConfiguredDFMap & BLE_BCN_DGSM_MSK ) )
	{
		uxPayloadLength += BLE_BCN_DGSM_LEN;
		ulActualDFMap |= BLE_BCN_DGSM_MSK;
	}
	/* Bit 17: GSM receive signal strength indicator. */
	if ( ( cGsm_BLE_RSSI[ 0 ] != 0 ) && ( ulConfiguredDFMap & BLE_BCN_GSMRSSI_MSK ) )
	{
		uxPayloadLength += BLE_BCN_GSMRSSI_LEN;
		ulActualDFMap |= BLE_BCN_GSMRSSI_MSK;
	}
	/* Bit 18: GSM receive signal quality. */
	if ( ( cGsm_BLE_QUAL[ 0 ] != 0 ) && ( ulConfiguredDFMap & BLE_BCN_GSMQUAL_MSK ) )
	{
		uxPayloadLength += BLE_BCN_GSMQUAL_LEN;
		ulActualDFMap |= BLE_BCN_GSMQUAL_MSK;
	}	
	
	/* Bit 20...23 : UUID-filtered (SwissPhone) locator beacon field. */
	if ( ulConfiguredDFMap & BLE_BCN_UUID_BCN_ALL_MSK )
	{
		uxPayloadLength += BLE_BCN_BCN_LEN * xBleLocBcnBLE.uxBleSwissPhoneLocBcnCnt;
		ulActualDFMap += BLE_BCN_UUID_BCN_ONE_MSK * xBleLocBcnBLE.uxBleSwissPhoneLocBcnCnt;
	}

	/* Bit 28, 29 and 30: Relayed commands. */
	if ( xBleModuleState.bBleTxRelayCmd )
	{
		uxPayloadLength += BLE_BCN_RLYADDR_LEN + BLE_BCN_RLYCMD_LEN;
		ulActualDFMap |= BLE_BCN_RLYADDR_MSK | BLE_BCN_RLYCMD_MSK;
		
		if ( bRelayCmdParamPresent )
		{
			uxPayloadLength += BLE_BCN_RLYPARAM_LEN;
			ulActualDFMap |= BLE_BCN_RLYPARAM_MSK;			
		}
	}

	/* Take the checksum byte into account. */
	uxPayloadLength++;
	
	/* uxPayloadLength contains now the length plaintext part to be encrypted. This length must be rounded up the next 
	   multiple of AES_BLOCK_LENGTH as the AES engine pads with '0'. */
	if ( uxPayloadLength % AES_BLOCK_LENGTH )
	{
		/* Make the length of the to-be encrypted part a multiple of AES_BLOCK_LENGTH. */
		uxPayloadLength = ( uxPayloadLength + AES_BLOCK_LENGTH ) & ~( AES_BLOCK_LENGTH - 1 );
	}
	
	/* Add the non-encrypted portion of the payload to the length. */
	uxPayloadLength += BLE_BCN_PL_NON_ENCR_OVR_LEN;
	
	/* Let the BLE handler copy the data into the softdevice control structures. */
	vSetAdvPayloadInit( LR );

	ucPayload[ 0 ] = uxPayloadLength;
	ucPayload[ 1 ] = 0xFF;
	ucPayload[ 2 ] = 0x17;
	ucPayload[ 3 ] = 0x00;
	ucPayload[ 4 ] = 0x01;
	vSetAdvPayload( LR, ucPayload, 5 );
	   
	vAES_CBC_EncryptStart( ( uint8_t * )&xNvdsConfig.pcAESKey[ ucConfigReadByte( &xNvdsConfig.ucActiveAESKey ) ] );

	/* Construct, encrypt and send the payload. */
	uxByteIdx = 0;
	
	/* Build the payload descriptor bitfield and encrypt it. */
	cDesc = 0;
	/* Initialise the checksum. */
	ucChkSum = 1;

	switch ( xGetCtrlState() )
	{
		case CTRL_ALERT:		cDesc += BLE_ALERT;		break;
		case CTRL_SOS:			cDesc += BLE_SOS;		break;
		default:										break;
	}
	
	#if defined ( XSWITCH )
		cDesc += BLE_XSWITCH_ORIGIN;
	#endif
	
	if ( bGetEvacOngoing() )
	{
		cDesc += BLE_EVAC;
	}	
	
	/* Set the flag for having received an XSWITCH in the IND field. */
	if ( bBleXSwitchObserved )
	{
		cDesc += BLE_XSWITCH_OBSERVED;
		bBleXSwitchObserved = false;
	}		
	
	/* Set the flag for RELAYCMD if such a command needs to be sent. */
	if ( xBleModuleState.bBleTxRelayCmd )
	{
		cDesc += BLE_RELAYCMD;
	}		
	
	/* Set the flag for DANGER if the module is believed to be in a danger zone. */
	if ( xBleModuleState.bBleInDangerZoneRequest )
	{
		cDesc += BLE_DANGER;
	}
	
	vEncrAndSendAdv( &uxByteIdx, &ucChkSum, cDesc );
	
	/* Add TRAXXS_ID "96A975" in little endian. */
	vEncrAndSendAdv( &uxByteIdx, &ucChkSum, 0x96 );
	vEncrAndSendAdv( &uxByteIdx, &ucChkSum, 0xA9 );
	vEncrAndSendAdv( &uxByteIdx, &ucChkSum, 0x75 );
	
	/* Add the IMEI. The IMEI contains 15 digits, so the MSB is 0-padded. */
	if ( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID ) != 0 )
	{
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID ) ) );
		for ( uxIdx = 1; uxIdx < 8; uxIdx++ )
		{
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum,   16 * cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID + 2 * uxIdx - 1 ) )
													+      cCharToNibble( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID + 2 * uxIdx     ) ) );
		}
	}
	else
	{
		for ( uxIdx = 0; uxIdx < BLE_BCN_FID_LEN; uxIdx++ )
		{
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, 0 );
		}
	}
	
	/* Add DFMAP in little endian. */
	vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ulActualDFMap >>  0 ) & 0xff );
	vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ulActualDFMap >>  8 ) & 0xff );
	vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ulActualDFMap >> 16 ) & 0xff );
	vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ulActualDFMap >> 24 ) & 0xff );
	
	/* Bit 0: TX timestamp. */
	if ( ulActualDFMap & BLE_BCN_TXTIM_MSK )
	{
		unsigned short		usTxTimeStamp;
		
		/* Add the current time stamp. */
		usTxTimeStamp = usReadRTC();
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( usTxTimeStamp >>  0 ) & 0xff );
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( usTxTimeStamp >>  8 ) & 0xff );
	}
	
	/* Bit 1: Conducted BLE output power. */
	if ( ulActualDFMap & BLE_BCN_PWR_MSK )
	{
		/* Add the TX power. */
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucConfigReadByte( &xNvdsConfig.ucBleTxPwr ) );
	}
	
	/* Bit 2: In EVAC mode, add the EVAC_ID. */
	if ( bGetEvacOngoing() )
	{
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )uxGetEvacId() );
	}
	
	/* Bit 3: Battery field. */
	if ( ulActualDFMap & BLE_BCN_BAT_MSK )
	{
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucReadBleBatteryVoltage() );	
	}
	
	/* Bit 19, 6:4: Locator beacon fields. */
	if ( ( ulActualDFMap & BLE_BCN_BCN_ALL_MSK ) || ( ulActualDFMap & BLE_BCN_BCN2_MSK ) )
	{
		/* Note: xBleLocBcnBLE.uxBleLocStdBcnCnt as well as the other beacon data is protected by a mutex from
		   getting updared during runtime of this function. */
		
		/* Walk throught all stored beacons. */
		for ( uxIdx = 0; uxIdx < ( xBleLocBcnBLE.uxBleStdLocBcnCnt + xBleLocBcnBLE.uxBleSwissPhoneLocBcnCnt ); uxIdx++ )
		{
			/* Find all standard entries. */
			if ( !xBleLocBcnBLE.xBleLocBcn[ uxIdx ].bIsSwissPhoneBeacon )
			{
				/* Now send this beacon entry. */
				for ( uxIdx2 = 0; uxIdx2 < BLE_LOC_BCN_LEN; uxIdx2++ )
				{
					vEncrAndSendAdv( &uxByteIdx, &ucChkSum, xBleLocBcnBLE.xBleLocBcn[ uxIdx ].uBleLocBeaconData.ucBleLocBeaconData[ uxIdx2 ] );
				}
			}
		}
	}
	
	/* Bit 7: LAT GPS latitude. */
	if ( ulActualDFMap & BLE_BCN_LAT_MSK )
	{
		signed char		cLat;

		if ( cGsm_BLE_LOCM[ 0 ] == '1' )
		{
			/* CellLocate: The data field is coded in format 2, for example: 43.6259722 (no leading '0's!).
			   Possible formats are:
						 43.6259722
						-43.6259722
						  2.6737650
						 -2.6737650			*/
			unsigned portBASE_TYPE	uxNegative;
			unsigned portBASE_TYPE	uxDecPos;

			uxNegative = 0;
			uxDecPos = 0;

			/* Identify position of decimal point. */
			if ( cGsm_BLE_LAT[ 0 ] == '-' )
			{
				uxNegative = 1;
			}
			if ( cGsm_BLE_LAT[ 1 ] == '.' )
			{
				uxDecPos = 1;
			}
			if ( cGsm_BLE_LAT[ 2 ] == '.' )
			{
				uxDecPos = 2;
			}
			if ( cGsm_BLE_LAT[ 3 ] == '.' )
			{
				uxDecPos = 3;
			}

			cLat = ( signed char )ucIntStrgToByte( cGsm_BLE_LAT + uxNegative, uxDecPos - uxNegative );
			if ( uxNegative ==  1 )
			{
				cLat |= 0x80;
			}

			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, cLat );
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucIntStrgToByte( cGsm_BLE_LAT + uxDecPos + 1, 2 ) );
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucIntStrgToByte( cGsm_BLE_LAT + uxDecPos + 3, 2 ) );
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucIntStrgToByte( cGsm_BLE_LAT + uxDecPos + 5, 2 ) );
		}
		else
		{
			/* GPS: The data field is coded in format 1, for example: N 4337.421095 */
			cLat = ( signed char )ucIntStrgToByte( cGsm_BLE_LAT + 2, 2 );
			
			if ( cGsm_BLE_LAT[ 0 ] == 'S' )
			{
				cLat |= 0x80;
			}
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, cLat );
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucIntStrgToByte( cGsm_BLE_LAT + 4, 2 ) );	
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucIntStrgToByte( cGsm_BLE_LAT + 7, 2 ) );	
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucIntStrgToByte( cGsm_BLE_LAT + 9, 2 ) );	
		}
	}
	
	/* Bit 8: LON GPS longitude. */
	if ( ulActualDFMap & BLE_BCN_LON_MSK )
	{
		signed short		sLon;

		if ( cGsm_BLE_LOCM[ 0 ] == '1' )
		{
			/* CellLocate: The data field is coded in format 2, for example: -7.0444309 (no leading '0's!).
			   Possible formats are:
						  7.0444309
						 -7.0444309
						 43.6259722
						-43.6259722
						152.3878725
					   -152.3878725		*/
			unsigned portBASE_TYPE	uxNegative;
			unsigned portBASE_TYPE	uxDecPos;

			uxNegative = 0;
			uxDecPos = 0;

			if ( cGsm_BLE_LON[ 0 ] == '-' )
			{
				uxNegative = 1;
			}
			if ( cGsm_BLE_LON[ 1 ] == '.' )
			{
				uxDecPos = 1;
			}
			if ( cGsm_BLE_LON[ 2 ] == '.' )
			{
				uxDecPos = 2;
			}
			if ( cGsm_BLE_LON[ 3 ] == '.' )
			{
				uxDecPos = 3;
			}
			if ( cGsm_BLE_LON[ 4 ] == '.' )
			{
				uxDecPos = 4;
			}

			sLon = ( signed char )usIntStrgToShort( cGsm_BLE_LON + uxNegative, uxDecPos - uxNegative );
			if ( uxNegative == 1 )
			{
				sLon |= 0x8000;
			}
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( sLon & 0xff00 ) >> 8 );
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum,   sLon & 0x00ff );
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucIntStrgToByte( cGsm_BLE_LON + uxDecPos + 1, 2 ) );
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucIntStrgToByte( cGsm_BLE_LON + uxDecPos + 3, 2 ) );
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucIntStrgToByte( cGsm_BLE_LON + uxDecPos + 5, 2 ) );
		}
		else
		{
			/* GPS: The data field is coded in format 1, for example: E 00702.338457 */
			sLon = ( signed short )usIntStrgToShort( cGsm_BLE_LON + 2, 3 );
			
			if ( cGsm_BLE_LON[ 0 ] == 'W' )
			{
				sLon |= 0x8000;
			}
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( sLon & 0xff00 ) >> 8 );
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum,   sLon & 0x00ff );
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucIntStrgToByte( cGsm_BLE_LON +  5, 2 ) );	
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucIntStrgToByte( cGsm_BLE_LON +  8, 2 ) );	
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucIntStrgToByte( cGsm_BLE_LON + 10, 2 ) );	
		}
	}
	
	/* Bit 9: GPS fix. */
	if ( ulActualDFMap & BLE_BCN_FIX_MSK )
	{
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, cCharToNibble( cGsm_BLE_FIX[ 0 ] ) );	
	}
	
	/* Bit 10: LOCM Localisation method. */
	if ( ulActualDFMap & BLE_BCN_LOCM_MSK )
	{
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, cCharToNibble( cGsm_BLE_LOCM[ 0 ] ) );	
	}
	
	/* Bit 11: GPS Horizontal Dilution of Precision. */
	if ( ulActualDFMap & BLE_BCN_HDOP_MSK )
	{
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, cCharToNibble( cGsm_BLE_HDOP[ 0 ] ) );	
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucHexStrgToByte( cGsm_BLE_HDOP + 2 ) );	
	}
	
	/* Bit 12: ACCH GPS horizontal accuracy. */
	if (ulActualDFMap & BLE_BCN_ACCH_MSK )
	{
		unsigned short			usAccH;
		
		usAccH =  usIntStrgToShort( cGsm_BLE_ACCHM, 5 );
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum,   usAccH & 0x00ff );
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( usAccH & 0xff00 ) >> 8 );
	}
	
	/* Bit 13: SV GPS satellites in view. */
	if ( ulActualDFMap & BLE_BCN_SV_MSK )
	{
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucHexStrgToByte( cGsm_BLE_SAT ) );	
	}
	
	/* Bit 14: TEMP Device temperature. */
	if ( ulActualDFMap & BLE_BCN_TEMP_MSK )
	{
		/* Send only the first two characters as temperature. Omit the decimal places. */
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, sGetBatteryTemperature() / 10 );
	}
	
	/* Bit 15: STATE Device state. */
	if ( ulActualDFMap & BLE_BCN_STATE_MSK )
	{
		enum xCTRL_STATE 		xCtrlState;
		
		xCtrlState = xGetCtrlState();
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )xCtrlState );		
	}
	
	/* Bit 16: DGSM debug information. 
	   Format is:    208;1;7200;25cf;39;0004 */
	if ( ( cGsm_BLE_MCC[ 0 ] != 0 ) && ( ulActualDFMap & BLE_BCN_DGSM_MSK ) )
	{
		unsigned short	usTempValue;
		unsigned long	ulTempValue;
		
		/* MCC */
		usTempValue = usIntStrgToShort( cGsm_BLE_MCC, LEN_GSM_MCC );
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ( usTempValue & 0x00ff ) >> 0 ) );		
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ( usTempValue & 0xff00 ) >> 8 ) );		

		/* MNC */
		usTempValue = usIntStrgToShort( cGsm_BLE_MNC, LEN_GSM_MNC );
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ( usTempValue & 0x00ff ) >> 0 ) );		
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ( usTempValue & 0xff00 ) >> 8 ) );		
		
		/* TAC */
		usTempValue = usHexStrgToShort( cGsm_BLE_TAC );
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ( usTempValue & 0x00ff ) >> 0 ) );		
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ( usTempValue & 0xff00 ) >> 8 ) );		
		
		/* CI */
		ulTempValue = ulHexStrgToLong( cGsm_BLE_CI );
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ( ulTempValue & 0x000000ff ) >>  0 ) );		
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ( ulTempValue & 0x0000ff00 ) >>  8 ) );		
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ( ulTempValue & 0x00ff0000 ) >> 16 ) );		
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ( ulTempValue & 0xff000000 ) >> 24 ) );		
		
		/* RAT */
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, cCharToNibble( cGsm_BLE_RAT[ 0 ] ) );		

		/* EARFCN */
		usTempValue = usIntStrgToShort( cGsm_BLE_EARFCN, LEN_GSM_EARFCN );
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ( usTempValue & 0x00ff ) >> 0 ) );		
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ( usTempValue & 0xff00 ) >> 8 ) );		
	}
	
	/* Bit 17: RSSI GSM receive signal strength indicator. */
	if ( ulActualDFMap & BLE_BCN_GSMRSSI_MSK )
	{
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ucHexStrgToByte( cGsm_BLE_RSSI ) );		
	}
	
	/* Bit 18: QUAL GSM receive signal quality. */
	if ( ulActualDFMap & BLE_BCN_GSMQUAL_MSK )
	{
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, cCharToNibble( cGsm_BLE_QUAL[ 0 ] ) );	
	}	
	
	/* Bit 20...23 : UUID-filtered (SwissPhone) locator beacon field. */
	/* Walk throught all stored beacons. */
	if ( ulActualDFMap & BLE_BCN_UUID_BCN_ALL_MSK )
	{
		for ( uxIdx = 0; uxIdx < ( xBleLocBcnBLE.uxBleStdLocBcnCnt + xBleLocBcnBLE.uxBleSwissPhoneLocBcnCnt ); uxIdx++ )
		{
			/* Find all SwissPhone beacon entries. */
			if ( xBleLocBcnBLE.xBleLocBcn[ uxIdx ].bIsSwissPhoneBeacon )
			{
				/* Now send this beacon entry. */
				for ( uxIdx2 = 0; uxIdx2 < BLE_LOC_BCN_LEN; uxIdx2++ )
				{
					vEncrAndSendAdv( &uxByteIdx, &ucChkSum, xBleLocBcnBLE.xBleLocBcn[ uxIdx ].uBleLocBeaconData.ucBleLocBeaconData[ uxIdx2 ] );
				}
			}
		}
	}
	
	/* Bit 28, 29 and 30: Relayed commands. */
	if ( xBleModuleState.bBleTxRelayCmd )
	{
		/* Relayed command destination address. */
		for ( uxIdx = 0; uxIdx < 8; uxIdx++ )
		{
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, cRelayCmdDestination[ uxIdx ] );
		}
		/* Relayed command index. */
		vEncrAndSendAdv( &uxByteIdx, &ucChkSum, uxRelayCmdIdx );
		/* Relayed command parameter. */
		if ( bRelayCmdParamPresent )
		{
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ( usRelayCmdParam & 0x00ff ) >> 0 ) );		
			vEncrAndSendAdv( &uxByteIdx, &ucChkSum, ( unsigned char )( ( usRelayCmdParam & 0xff00 ) >> 8 ) );		
		}
	}	

	/* Terminate the last AES block. */
	vFinishEncrAndSendAdv( &uxByteIdx, &ucChkSum );

	/* Invalidate the data. */
	cGsm_BLE_LAT   [ 0 ] = 0;
	cGsm_BLE_LON   [ 0 ] = 0;
	cGsm_BLE_HDOP  [ 0 ] = 0;
	cGsm_BLE_FIX   [ 0 ] = 0;
	cGsm_BLE_LOCM  [ 0 ] = 0;
	cGsm_BLE_SAT   [ 0 ] = 0;
	cGsm_BLE_TEMP  [ 0 ] = 0;
	cGsm_BLE_STATE [ 0 ] = 0;
	cGsm_BLE_ACCHM [ 0 ] = 0;
	cGsm_BLE_RSSI  [ 0 ] = 0;
	cGsm_BLE_QUAL  [ 0 ] = 0;
	cGsm_BLE_MCC   [ 0 ] = 0;
	cGsm_BLE_MNC   [ 0 ] = 0;
	cGsm_BLE_TAC   [ 0 ] = 0;
	cGsm_BLE_CI    [ 0 ] = 0;
	cGsm_BLE_RAT   [ 0 ] = 0;
	cGsm_BLE_EARFCN[ 0 ] = 0;

	/* Empty the locator beacon storage. */
	xBleLocBcnBLE.uxBleStdLocBcnCnt 		= 0;
	xBleLocBcnBLE.uxBleSwissPhoneLocBcnCnt 	= 0;
	
	/* Restore write access to the GSM data. */
	xSemaphoreGive( xMutexGsmData );
}
/*-----------------------------------------------------------*/

/* Obtain the BLE module state. */
struct xBLE_MODULE_STATE xGetBleModuleState( void )
{
	return xBleModuleState;
}
/*-----------------------------------------------------------*/

/* Start or stop the softdevice. */
void vStartStopSoftDevice( bool bStartDevice )
{
	#if !defined( NO_SOFTDEVICE_SHUTDOWN )
		if ( bStartDevice )
		{
			/* If the SoftDevice is disabled, re-enable it. */
			if (    !xBleModuleState.bBleSoftDeviceOn 
			     && !xBleModuleState.bBleTxSuspended
				 && (    xBleModuleState.bBleLocRequest
					  || xBleModuleState.bBleAlertRequest
					  || bGetEvacOngoing()  		
					  || xBleModuleState.bBleInDangerZoneRequest	
					  || xBleModuleState.bBleHeartBeatRequest 
					  || xBleModuleState.bBleTxRelayCmd
				    ) 
			   ) 
			{
				nrf_sdh_enable_request();
				xBleModuleState.bBleSoftDeviceOn = true;

				vTaskDelay( 2 );
				/* Check if the SoftDevice is enabled by now. */
				ASSERT( nrf_sdh_is_enabled() );
			}
		}
		else
		{
			/* If no Bluetooth function is needed anymore, stop the SoftDevice entirely. */
			if (   !xBleModuleState.bBleAlertRequest
				&& !bGetEvacOngoing()  		
				&& !xBleModuleState.bBleInDangerZoneRequest	
				&& !xBleModuleState.bBleHeartBeatRequest )
			{
				nrf_sdh_disable_request();
				xBleModuleState.bBleSoftDeviceOn = false;

				vTaskDelay( 2 );
				/* Check if the SoftDevice disabled by now. */
				ASSERT( !nrf_sdh_is_enabled() );
			}			
		}
	#else
		( void )bStartDevice;
	#endif	
}
/*-----------------------------------------------------------*/

/* Start BLE scan for beacons. The results of the scan are parsed and recorded in the BLE parser task. 
   The function loops until it succeeds.
*/
static void vBleScanStart( void )
{
	unsigned short			usRssiParameter;
	unsigned portBASE_TYPE	uxFilterMethod;
	unsigned portBASE_TYPE  uxFilterWindow;
	uint8_t					uiSoftDeviceEnabled;
	
	/* Start the softdevice. */
	vStartStopSoftDevice( BLE_START_SD );
	
	/* Configure the RSSI filtering algorithm. */
	usRssiParameter = usConfigReadShort( &xNvdsConfig.usBleRssiCfg );
	uxFilterMethod = ( unsigned char )( ( usRssiParameter >> 13 ) & 0x07 );
	uxFilterWindow = ( unsigned char )( ( usRssiParameter >>  8 ) & 0x1F );				
	vSetRssiCfg( uxFilterMethod, uxFilterWindow );

	/* Configure the module for normal scan only. */
	vStartScan( SCAN_STD1MBPS | SCAN_LR125KBPS );
}
/*-----------------------------------------------------------*/

/* Start BLE TL beacon advertising or update advertising payload.

   Depending on xBleModuleState, the function sends either a distress beacon (ALERT/EVAC) or a 
   heartbeat beacon. 
   
   The function may be called anytime even without advertising being requested. 
*/
static void vBleSetAdvertising( void )
{
	/* Configure the module for transmitting advertising packets. 
	   In ALERT or evac state or in a danger zone, both long range and short range beacons are sent.
	   In contrast, heartbeats are only sent on long range. */
	/* Note that the nRF takes care of alternatingly advertising in SR and LR modes. */
	if (    !xBleModuleState.bBleTxSuspended 
	     && !xBleModuleState.bBleSuspended )
	{
		if ( 	xBleModuleState.bBleAlertRequest 
		 	 ||	xBleModuleState.bBleInDangerZoneRequest
			 || bGetEvacOngoing() 
			 || xBleModuleState.bBleTxRelayCmd
		   )
		{
			/* ALERT / DANGER / EVAC / TX relay command:
			   Restart advertising rapidly in both long and short range. The new data gets taken into account at the same time. */
			/* Start the softdevice. */
			vStartStopSoftDevice( BLE_START_SD );
			
			vBuildAdvDataS();
			vBuildAdvDataL( false );

			vSetTxPower( SR, ucConfigReadByte( &xNvdsConfig.ucBleTxPwr ) );
			vSetTxPower( LR, ucConfigReadByte( &xNvdsConfig.ucBleTxPwr ) );

			vSetAdvInterval( SR, BLE_ADV_INTVL_ALERT );
			vSetAdvInterval( LR, BLE_ADV_INTVL_ALERT );
			vStartAdvertising( ADV_STD1MBPS | ADV_LR125KBPS );
		}
		else
		{
			if ( xBleModuleState.bBleHeartBeatRequest )
			{
				/* Heartbeat:
				   Restart advertising slowly in long range only. The new data gets taken into account at the same time. 
				   Position informatyion is included only if the module is not in GPS recording mode and nothing usual is 
				   going on, i.e. the module only wants to send HELLO packets. */
				/* Start the softdevice. */
				vStartStopSoftDevice( BLE_START_SD );

				vBuildAdvDataL( bIsHelloPacketInGpsRecording() );

				vSetTxPower( LR, ucConfigReadByte( &xNvdsConfig.ucBleTxPwr ) );

				vSetAdvInterval( LR, ( uint32_t )usConfigReadShort( &xNvdsConfig.usBleStdAdvInterval ) );
				vStartAdvertising( ADV_LR125KBPS );
			}
		}
	}
}
/*-----------------------------------------------------------*/

/* Suspend all BLE activity: receptions as well as transmissions. 

   Non-blocking function which tries to suspend the BLE.
   The previous module state is backed-up so that it can be restored using the vResumeAllBleActivity() function. 
   
   Thread-safe function. 
		Thread-safe means that while BLE is suspended (or being suspended), the same function may be called again. 
		If that happenes, the second call does not overwrite the saved BLE state but only increments the counting
		semaphore.   
   
   May be called from any task.
*/
void vNonBlockingSuspendAllBleActivity( void )
{
	#if !defined ( NO_SOFTDEVICE )
		enum xBLE_CMD					xBleCmd;
		
		if ( ( bool )( usConfigReadShort( &xNvdsConfig.usBleEnable ) ) )
		{
			/* Take the BLE suspended all semphore. */
			xSemaphoreTake( xBleAllBlockedCountingSemaphore, 0 );

			V_TRACE_PRINT_BYTE( TRACE_BLE_SUSPEND_ALL, uxSemaphoreGetCount( xBleAllBlockedCountingSemaphore ), TRACE_UART );

			/* Perform the requested action if the semaphore was not yet taken. */
			if ( uxSemaphoreGetCount( xBleAllBlockedCountingSemaphore ) == BLE_SUSP_SEMAPH_MAX - 1 )
			{
				/* Save the current BLE state. */
				xBleModuleStateBackup = xGetBleModuleState();

				xBleModuleState.bBleSuspended = true;

				/* Suspend BLE scanning. */
				xBleCmd = BLE_LOC_STOP;
				xQueueSend( xBleCmdQueue, &xBleCmd, 0 );

				if (    xBleModuleState.bBleAlertRequest  		
					 || xBleModuleState.bBleInDangerZoneRequest	
				     || xBleModuleState.bBleHeartBeatRequest	)
				{
					/* Next, suspend the BLE TX. */
					vNonBlockingSuspendBleTxActivity();
				}
			}			
		}
	#endif
}
/*-----------------------------------------------------------*/

/* Suspend all BLE activity: receptions as well as transmissions. 

   Wait until the BLE module state has changed (blocking).
   The previous module state is backed-up so that it can be restored using the vResumeAllBleActivity() function. 
   
   Thread-safe function. 
		Thread-safe means that while BLE is suspended (or being suspended), the same function may be called again. 
		If that happenes, the second call does not overwrite the saved BLE state but only increments the counting
		semaphore.   
   
   May be called from any task with a priority lower than that of the TIMER task.
*/
void vBlockingSuspendAllBleActivity( void )
{
	#if !defined ( NO_SOFTDEVICE )
		enum xBLE_CMD					xBleCmd;
		unsigned portBASE_TYPE			uxTrialCount;
		
		if ( ( bool )( usConfigReadShort( &xNvdsConfig.usBleEnable ) ) )
		{
			/* Request suspension - non blocking. */
			vNonBlockingSuspendAllBleActivity();
			
			/* Wait for the suspend to be executed. */
			if ( uxSemaphoreGetCount( xBleAllBlockedCountingSemaphore ) == BLE_SUSP_SEMAPH_MAX - 1 )
			{
				uxTrialCount = 0;								
				do												
				{
					vTaskDelay( 1 );							
					uxTrialCount++;								
				}
				while( ( uxTrialCount < TO_BLE_ACTIVITY_OFF ) && ( !xGetBleModuleState().bBleTxSuspended || xGetBleModuleState().bBleLocRequest ) );
				
				if ( !xGetBleModuleState().bBleTxSuspended || xGetBleModuleState().bBleLocRequest )
				{
					NRF_LOG_ERROR( "%i BLE did not react to activity suspend.", ulReadRTC() );
				}				
			}			
		}
	#endif
}
/*-----------------------------------------------------------*/

/* Resume all BLE activity. 

   Restore the activity to the same state it was in when vBlockingSuspendAllBleActivity() was called. 

   Does not wait for execution. Returns immediately.
   Thread-safe.
   May be called from TIMER.
*/
void vResumeAllBleActivity( void )
{
	#if !defined ( NO_SOFTDEVICE )
		enum xBLE_CMD					xBleCmd;

		if ( ( bool )( usConfigReadShort( &xNvdsConfig.usBleEnable ) ) )
		{
			/* Give the BLE suspended all semaphore. */
			xSemaphoreGive( xBleAllBlockedCountingSemaphore );
			
			V_TRACE_PRINT_BYTE( TRACE_BLE_RESUME_ALL, uxSemaphoreGetCount( xBleAllBlockedCountingSemaphore ), TRACE_UART );			
			
			/* Resume activity if the counting semaphore has been completely given. */
			if ( uxSemaphoreGetCount( xBleAllBlockedCountingSemaphore ) == BLE_SUSP_SEMAPH_MAX )
			{
				xBleModuleState = xBleModuleStateBackup;
				
				if ( xBleModuleState.bBleLocRequest )
				{
					/* Resume localisation. */
					xBleCmd = BLE_LOC_START;
					xQueueSend( xBleCmdQueue, &xBleCmd, 0 );	   
				}

				if (    xBleModuleState.bBleAlertRequest  		
					 || xBleModuleState.bBleInDangerZoneRequest	
				     || xBleModuleState.bBleHeartBeatRequest	)
				{
					/* Resume BLE TX. */
					vResumeBleTxActivity();
				}
			}				
		}	
	#endif
}
/*-----------------------------------------------------------*/

/* Non-blocking function which tries to align the BLE suspended/resumed state with the actual state.

   If it is different, the function sends commands to the BLE task.
   Does not wait for execution. Returns immediately.
   Thread-safe.
   May be called from TIMER.
*/
void vNonBlockingAlignBleActivitySuspension( void )
{
	#if !defined ( NO_SOFTDEVICE )
		enum xBLE_CMD					xBleCmd;

		if ( ( bool )( usConfigReadShort( &xNvdsConfig.usBleEnable ) ) )
		{
			/* Check if the requested and actual state of the BLE module match. If not. align them. */
			/* First, check suspend all. */
			if (    xGetBleModuleState().bBleSuspended
			     && xGetBleModuleState().bBleLocRequest )
			{
				/* Suspend localisation. */
				xBleCmd = BLE_LOC_STOP;
				xQueueSend( xBleCmdQueue, &xBleCmd, 0 );	   
			}

			/* Request suspending TX activity if the counting semaphore has been completely given but the status is not set. */
			if (    ( uxSemaphoreGetCount( xBleTxBlockedCountingSemaphore ) < BLE_SUSP_SEMAPH_MAX )
				 && ( !xBleModuleState.bBleTxSuspended ) )
			{
				xBleCmd = BLE_TX_SUSPEND;
				xQueueSend( xBleCmdQueue, &xBleCmd, 0 );
			}
			
			/* Request resuming TX activity if the counting semaphore has been completely given but the status is not set. */
			if (    ( uxSemaphoreGetCount( xBleTxBlockedCountingSemaphore ) == BLE_SUSP_SEMAPH_MAX )
				 && ( xBleModuleState.bBleTxSuspended ) )
			{
				xBleCmd = BLE_TX_RESUME;
				xQueueSend( xBleCmdQueue, &xBleCmd, 0 );
			}			
		}	
	#endif
}
/*-----------------------------------------------------------*/

/* Suspend BLE TX activity. 

   Non-blocking function which tries to suspend the BLE transmissions.
   
   Thread-safe function. 
		Thread-safe means that while BLE is suspended (or being suspended), the same function may be called again. 
		If that happenes, the second call does not overwrite the saved BLE state but only increments the counting
		semaphore.   
   
   May be called from any task.
*/
void vNonBlockingSuspendBleTxActivity( void )
{
	#if !defined ( NO_SOFTDEVICE )
		enum xBLE_CMD					xBleCmd;

		if ( ( bool )( usConfigReadShort( &xNvdsConfig.usBleEnable ) ) )
		{	
			/* Next, take the BLE TX suspended all semphore. */
			xSemaphoreTake( xBleTxBlockedCountingSemaphore, 0 );

			V_TRACE_PRINT_BYTE( TRACE_BLE_SUSPEND_TX, uxSemaphoreGetCount( xBleTxBlockedCountingSemaphore ), TRACE_UART );

			/* Perform the requested action if the semaphore was not yet taken. */
			if ( uxSemaphoreGetCount( xBleTxBlockedCountingSemaphore ) == BLE_SUSP_SEMAPH_MAX - 1 )
			{
				/* Request suspending the BLE TX. */
				xBleCmd = BLE_TX_SUSPEND;
				xQueueSend( xBleCmdQueue, &xBleCmd, 0 );
			}
		}
	#endif
}
/*-----------------------------------------------------------*/

/* Resume BLE TX activity. 

   Does not wait for execution. Returns immediately.
   Thread-safe.
   May be called from TIMER.
*/
void vResumeBleTxActivity( void )
{
	#if !defined ( NO_SOFTDEVICE )
		enum xBLE_CMD					xBleCmd;

		if ( ( bool )( usConfigReadShort( &xNvdsConfig.usBleEnable ) ) )
		{
			/* Give the BLE TX suspended semaphore. */
			xSemaphoreGive( xBleTxBlockedCountingSemaphore );
			
			V_TRACE_PRINT_BYTE( TRACE_BLE_RESUME_TX, uxSemaphoreGetCount( xBleTxBlockedCountingSemaphore ), TRACE_UART );

			/* Resume TX activity if the counting semaphore has been completely given. */
			if ( uxSemaphoreGetCount( xBleTxBlockedCountingSemaphore ) == BLE_SUSP_SEMAPH_MAX )
			{
				xBleCmd = BLE_TX_RESUME;
				xQueueSend( xBleCmdQueue, &xBleCmd, 0 );
			}
		}	
	#endif
}
/*-----------------------------------------------------------*/

/* Main BLE task function. */
static portTASK_FUNCTION( vBLETask, pvParameters )
{
	enum xBLE_CMD			xBleCmd;
	unsigned short			usFlashTS;
	
	/* Just to stop compiler warnings. */
	( void )pvParameters;

	vTaskSetApplicationTaskTag( NULL, ( void * ) BLE_CTRL_TASK_TAG );
	
	/* Wait untile the configuration handler is initialised. */
	while ( !bCheckConfigInitialised() || !bCheckTraceInitialised() )
	{
		vTaskDelay( 1 );
	}

	usFlashTS = usReadRTC();

	/* Update the UUID-filtering enabled variable. */
	bCheckAndSetUUIDFilterEnabled();

	/* Change beacon update timer to actual configuration value. */
	if ( ucConfigReadByte( &xNvdsConfig.ucBleBcnUpdInterval ) > 0 )
	{
		( void )xTimerChangePeriod( xBcnUpdateTimer, ucConfigReadByte( &xNvdsConfig.ucBleBcnUpdInterval ) * portTICKS_PER_SEC, portMAX_DELAY );
	}
		
	NRF_LOG_INFO( "Task started." );
	NRF_LOG_FLUSH();

	while ( 1 )
	{
		if ( xQueueReceive( xBleCmdQueue, &xBleCmd, portMAX_DELAY ) != errQUEUE_EMPTY )
		{
			/* Received a command from the CTRL task. */
			switch ( xBleCmd )
			{
				case BLE_LOC_START:			/* Start localisation via BLE beacons. */
											V_TRACE_PRINT( TRACE_BLE_LOCALISATION_START, TRACE_UART );

											/* Set status variable to reflect request state. */
											xBleModuleState.bBleLocRequest 		 	= true;
											xBleModuleState.bBleHeartBeatRequest 	= true;

											if ( !xBleModuleState.bBleSuspended )
											{
												vBleScanStart();
											}

											/* Update the beacon data and start advertising. */
											if (    !xBleModuleState.bBleTxSuspended 
											     && !xBleModuleState.bBleSuspended )
											{
												vBleSetAdvertising();
											}

											break;
											

				case BLE_LOC_STOP:			/* Stop localisation via BLE beacons. */
											V_TRACE_PRINT( TRACE_BLE_LOCALISATION_STOP, TRACE_UART );

											if ( xBleModuleState.bBleLocRequest )
											{											
												/* Set status variable to reflect request state. */
												xBleModuleState.bBleLocRequest 		 	= false;
												xBleModuleState.bBleHeartBeatRequest 	= false;

												vStopScan();
												vStopAdvertising();
											}
											
											/* Stop the softdevice. */
											vStartStopSoftDevice( BLE_STOP_SD );
											
											break;
											
											
				case BLE_SR_ALERT_SOS_START:/* Start short range advertising the ALERT or SOS state. */
											V_TRACE_PRINT( TRACE_BLE_START_SR_SOS, TRACE_UART );

											/* Set status variable to reflect request state. */
											xBleModuleState.bBleLocRequest 		 	= true;
											xBleModuleState.bBleHeartBeatRequest 	= true;
											xBleModuleState.bBleAlertRequest 	 	= true;
											
											/* Add a short pause to allow the CTRL task to finish its state transition. */
											vTaskDelay( 2 );
											
											vBleSetAdvertising();
											
											break;

											
				case BLE_SR_ALERT_SOS_STOP:	/* Stop short range advertising the ALERT or SOS state. */
											V_TRACE_PRINT( TRACE_BLE_STOP_SR_SOS, TRACE_UART );

											/* Set status variable to reflect stopped state. */
											xBleModuleState.bBleLocRequest 		 	= true;
											xBleModuleState.bBleHeartBeatRequest 	= true;
											xBleModuleState.bBleAlertRequest 	 	= false;

											vBleSetAdvertising();
											
											break;

											
				case BLE_SR_EVAC_START:		/* Start short range advertising EVAC. */
											V_TRACE_PRINT( TRACE_BLE_START_SR_EVAC, TRACE_UART );

											/* Set status variable to reflect request state. */
											xBleModuleState.bBleLocRequest 		 	= true;
											xBleModuleState.bBleHeartBeatRequest 	= true;
											
											vBleSetAdvertising();
											
											break;

											
				case BLE_SR_EVAC_STOP:		/* Stop short range advertising EVAC. */
											V_TRACE_PRINT( TRACE_BLE_STOP_SR_EVAC, TRACE_UART );

											/* Set status variable to reflect stopped state. */
											xBleModuleState.bBleLocRequest 		 	= true;
											xBleModuleState.bBleHeartBeatRequest 	= true;

											vBleSetAdvertising();
											
											break;
											

				case BLE_UPDATE_BCN:		/* Update the beacon data. */
											if ( !xBleModuleState.bBleTxSuspended )
											{
												vBleSetAdvertising();
											}
											/* Reprogram the timer with the beacon update interval in the EEPROM. */
											if ( ucConfigReadByte( &xNvdsConfig.ucBleBcnUpdInterval ) > 0 )
											{
												( void )xTimerChangePeriod( xBcnUpdateTimer, ucConfigReadByte( &xNvdsConfig.ucBleBcnUpdInterval ) * portTICKS_PER_SEC, 
																			portMAX_DELAY );
											}
											break;
											
											
				case BLE_TX_SUSPEND:		/* Suspend sending any beacons. This reduces the noise impact on the GPS. */
											xBleModuleState.bBleTxSuspended 		= true;													
											vStopAdvertising();

											break;
											
											
				case BLE_TX_RESUME:			/* Resume sending beacons. */
											if ( xBleModuleState.bBleTxSuspended )
											{
												xBleModuleState.bBleTxSuspended 	= false;													
												vBleSetAdvertising();
											}

											break;
											
				case BLE_RELAY_CMD:			/* Relay a command to a remote device. */
											xBleModuleState.bBleTxRelayCmd = true;
											/* Launch timer supervising the relayed command. */
											( void )xTimerStart( xBcnRelayedCmdTimer, portMAX_DELAY );
											/* Update the beacon data. */
											if ( !xBleModuleState.bBleTxSuspended )
											{
												vBleSetAdvertising();
											}
											
											break;
											/* TODO: Check what happens if there is a command to relay but the module is not ready to send:
												- it is on the charger
												- is in suspend mode
											*/
											
											
				case BLE_RELAYED_LED_CMD:	/* Execute an LED-on command received via BLE, relayed from the server. 
											   The command cannot be handled in the BLE parser task as it blocks for the duration of the LED-on. */
										    /* Request switching on the LEDs. The short command parameter is interprested as follows:
													bit 15 - 8		duration in seconds
													bit  2 - 0		LED bitmask							*/
											if ( usBleRlyCmdParam & 0x01 )
											{
												vSwitchOnLedRed ( ledRED_BLE_REQ );
											}
											if ( usBleRlyCmdParam & 0x02 )
											{
												vSwitchOnLedGreen ( ledGREEN_BLE_REQ );
											}
											if ( usBleRlyCmdParam & 0x04 )
											{
												vSwitchOnLedBlue ( ledBLUE_BLE_REQ );
											}
											
											/* Keep it on for the requested duration. */
											vTaskDelay( ( ( usBleRlyCmdParam >> 8 ) & 0xff ) * portTICKS_PER_SEC );
											
											/* Switch off the LED. */
											vSwitchOffLedRed( ledRED_BLE_REQ );
											vSwitchOffLedGreen( ledGREEN_BLE_REQ );
											vSwitchOffLedBlue( ledBLUE_BLE_REQ );
											
											break;
											
											
				case BLE_RELAYED_VIBR_CMD:	/* Execute a vibrate command received via BLE, relayed from the server. 
											   The command cannot be handled in the BLE parser task as it blocks for the duration of the vibrate-on. 
											   The short command parameter is interprested as follows:
													bit 15 - 8		on-duration in 100 milliseconds
													bit  7 - 4		off-duration in 100 milliseconds
													bit  3 - 0		repetitions							*/
											vCustomVibrate( ( ( usBleRlyCmdParam >> 8 ) & 0xff ), 
															( ( usBleRlyCmdParam >> 4 ) & 0x0f ), 
															usBleRlyCmdParam & 0x0f );
											break;
											
											
				case BLE_RX_FOREIGN_ALERT_SOS:	/* Received a foreign alert or SOS message. */
											/* Flash the blue LED once long. */
											vSwitchOnLedBlue( ledBLUE_BLE_REQ );
											vTaskDelay( T_BLE_LED_RX_TL );
											vSwitchOffLedBlue( ledBLUE_BLE_REQ );
											
											break;
											
											
				case BLE_BEACON_CAPTURED:	/* Received a beacon. */
											/* Flash the blue LED once short. */
											if ( usReadRTC() - usFlashTS > 3 )
											{
												vSwitchOnLedBlue( ledBLUE_BLE_REQ );
												vTaskDelay( T_BLE_LED_RX_BCN );
												vSwitchOffLedBlue( ledBLUE_BLE_REQ );
												usFlashTS = usReadRTC();

											}

											break;
																						
				default:					break;
			}
		}
	}
}
/*-----------------------------------------------------------*/
