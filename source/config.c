/*
 * Tracker Firmware
 *
 * Configuration record handling routines.
 * The configuration record is instantiated in the FDS system. A copy is kept in RAM and written to Flash
 * whenever the activity permits.
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>

#include "tracker.h"

/* nRF SDK include files */
#define NRF_LOG_MODULE_NAME 		CONFIG
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* Device specific include files. */
#include "drv_nvm.h"
#include "drv_vibrator.h"

#include "ble_adreport.h"
#include "ble_ctrl.h"
#include "ble_main.h"
#include "charger.h"
#include "config.h"
#include "ctrl.h"
#include "evacuation.h"
#include "gps.h"
#include "gsm.h"
#include "motiondet.h"
#include "trace.h"
#include "tracemsg.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Initialise the configuration data handler. */
void vConfigInit( void );

/* Initialise the configuration record with the default values. */
void vInitialiseConfig( void );

/* Invalidate the configuration record. */
void vInvalidateConfig( void );

/* Check if the configuration record is properly initialised. If not, initialise it with the default values. */
bool bCheckNVDSIsGood( void );

/* Generate a 16-bit checksum on the NVDS contents in between and including the two magic keys.
   Is NOT a re-entrant function! Needs to be protected by the caller. */
unsigned short usGenerateCheckSum( void );
/*-----------------------------------------------------------*/

/* Global variables. */
/* RAM copy  of the configuration record. */
struct xNVDS_CONFIG		xNvdsConfig;

/* Mutex handle to protect configuration record access. */
SemaphoreHandle_t		xMutexConfig;

/* Flag indicating that the configuration handler is initialised. No access to the configuration may be done before this is the case. */
bool					bConfigInitialised = false;

/* Flag indicating that the configuration record has been reinitialised because of corruption. */
bool					bNVDSReInitialised;
/*-----------------------------------------------------------*/

/* Initialise the configuration data handler. */
void vConfigInit( void )
{
	bool		bNvdsDirty;
	
	/* Create a mutex to protect access to the NVDS so that it can be used from different tasks - notably GSM and CTRL. */
	xMutexConfig = xSemaphoreCreateMutex();

	bNvdsDirty = false;
	
	#if !defined ( NO_SOFTDEVICE )	
		/* Check,if the configuration record exists in NVDS. 
		If yes, copy it to RAM. Else, create a new copy. */
		if ( !bCheckIfRecordExists( CONFIG_FILE_KEY ) )
		{
			/* The configuration record does not exist. Create it from scratch. */
			vInitialiseConfig();
			bNvdsDirty = true;
		}
		else
		{
			/* The configuration record exists in NVDS. Copy it to RAM. */
			vReadRecord( CONFIG_FILE_KEY );
		}
		
		/* Check the configuration record for data consistency. If the data is not good, re-initialise it from scratch. */
		if ( !bCheckNVDSIsGood() )
		{
			/* Create it from scratch. */
			vInitialiseConfig();
			bNvdsDirty = true;
		}
		
		/* If the configuration data has been modified, write it back to NVDS. */
		if ( bNvdsDirty )
		{
			vUpdateRecord( CONFIG_FILE_KEY );
		}
	#else
		vInitialiseConfig();
	#endif
		
	bConfigInitialised = true;

	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* Check if the configuration handler is fully initialised. */
bool bCheckConfigInitialised( void )
{
	return bConfigInitialised;
}
/*-----------------------------------------------------------*/

/* Check if the configuration record is properly initialised. If not, initialise it with the default values. */
bool bCheckNVDSIsGood( void )
{
	bNVDSReInitialised = false;
	
	/* Check that the configuration page is initialised:
	   - The magic key needs to match.
	   - The checksum needs to be valid. 
	   The configuration page is always updated in case there was a previous FW update. */     
	if (   ( xNvdsConfig.usMagicKey1 == MAGIC_KEY_1 )
		&& ( xNvdsConfig.usMagicKey2 == MAGIC_KEY_2 )
	    && ( xNvdsConfig.usChkSum  	 == usGenerateCheckSum() ) )
	{
		/* Configuration record is okay. */
		return true;
	}
	
	return false;
}
/*-----------------------------------------------------------*/

/* Initialise the configuration record with the default values. */
void vInitialiseConfig( void )
{
	bNVDSReInitialised = false;
	
	/* If getting here, the NVDS is either uninitialised or corrupt. Re-initialise it from scratch but leave out
	   the flags returned by the boot loader. */
	xNvdsConfig.usMagicKey1 								= MAGIC_KEY_1;
	xNvdsConfig.usMagicKey2 								= MAGIC_KEY_2;
	
	xNvdsConfig.xCtrlStateBackup							= CTRL_INACTIVE;
	
	strncpy( xNvdsConfig.pcFTPServerURL, 			cFTPServerDefaultURL, 		FWUPD_MAX_SRV_URL );
	strncpy( xNvdsConfig.pcFTPCfg,		 			cFTPDefaultCfg,		 		FTP_MAX_CFG );
		
	strncpy( xNvdsConfig.pcGsmGPRSConnCfg, 			cGsmGPRSDefaultConnParam,	LEN_GSM_GPRS_CONN_CFG );
	strncpy( xNvdsConfig.pcGsmTCPConnCfg, 			cGsmTCPDefaultConnParam,  	LEN_GSM_TCP_CONN_CFG );
	strncpy( xNvdsConfig.pcGsmSecTCPConnCfg, 		cGsmSecTCPConnParam, 	  	LEN_GSM_TCP_CONN_CFG );

	xNvdsConfig.ulDFMap 									= GSM_DEFAULT_DFMAP;	
	xNvdsConfig.usCfgRat									= DEFAULT_RATS;
	xNvdsConfig.ulRegTimeOut								= TO_GSM_REG;
	
	xNvdsConfig.usActivePositionSendInterval				= CTRL_ACTIVE_POS_INT;	
	xNvdsConfig.usInactivePositionSendInterval				= CTRL_INACTIVE_POS_INT;	
	xNvdsConfig.usStillPositionSendInterval					= CTRL_STILL_POS_INT;	
	xNvdsConfig.usSleepPositionSendInterval					= CTRL_SLEEP_POS_INT;	
	xNvdsConfig.usOooPositionSendInterval					= CTRL_OOO_POS_INT;	
	xNvdsConfig.usChargingPositionSendInterval				= CTRL_CHARGING_POS_INT;	
	xNvdsConfig.usAlertPositionSendInterval					= CTRL_ALERT_POS_INT;	
	xNvdsConfig.usGpsRecordingPacketInterval				= GSM_GPS_RECORDING_PACKET_INT;
	xNvdsConfig.usModulePwrDownInt	 						= GSM_DEFAULT_MOD_PWR_DOWN_INT;	

	xNvdsConfig.usStepAccelerationThreshold           		= STEP_ACC_THRES;
	xNvdsConfig.usStepMinInterval							= STEP_MIN_INTERVAL;
	xNvdsConfig.usStepMaxInterval							= STEP_MAX_INTERVAL;
	xNvdsConfig.usActiveNumberSteps                     	= ACTIVE_NUM_STEPS;

	xNvdsConfig.usStillDuration                         	= STILL_DURATION;
	xNvdsConfig.usImmobilityDuration                        = IMMOBILITY_DURATION;
	xNvdsConfig.usSleepDuration                         	= SLEEP_DURATION;

	xNvdsConfig.usAbnormalPositionZAccelerationThreshold	= ABNORMAL_POS_Z_ACC_THRES;
	xNvdsConfig.usAbnormalPositionDuration					= ABNORMAL_DURATION;
	xNvdsConfig.usAbnormalMaxOrientationChg					= ABNORMAL_MAX_ORIENTATION_CHG;

	xNvdsConfig.ucAlertCancelMethod							= ALERT_CNCL_BY_TAP;
	xNvdsConfig.usAlertCancelThreshold                  	= ALERT_CNCL_ACC_THRES;
	xNvdsConfig.usAlertCancelMinInterval					= ALERT_CNCL_MIN_INTERVAL;
	xNvdsConfig.usAlertCancelMaxInterval					= ALERT_CNCL_MAX_INTERVAL;
	xNvdsConfig.usAlertCancelNumberPeaks                	= ALERT_CNCL_NUM_PEAKS;
	xNvdsConfig.usAlertCancelTimeout                       	= ALERT_CNCL_TIMEOUT;
	xNvdsConfig.usSosCancelTimeout                       	= SOS_CNCL_TIMEOUT;

	xNvdsConfig.usSosAccelerationThreshold	       			= SOS_ACC_THRES;
	xNvdsConfig.usSosMinXAccelerationThreshold     			= SOS_X_ACC_THRES;
	xNvdsConfig.usSosMinZAccelerationThreshold     			= SOS_Z_ACC_LO_THRES;
	xNvdsConfig.usSosMaxZAccelerationThreshold     			= SOS_Z_ACC_HI_THRES;
	xNvdsConfig.usSosMinInterval							= SOS_MIN_INTERVAL;
	xNvdsConfig.usSosMaxInterval							= SOS_MAX_INTERVAL;
	xNvdsConfig.usSosNumberPeaks          		      		= SOS_NUM_PEAKS;
	
	xNvdsConfig.usNoAbnEnabled								= NOABN_EN;
	xNvdsConfig.usNoAbnCmdPeaks								= NOABN_NUM_PEAKS;
	xNvdsConfig.usNoAbnMaxPeriod							= NOABN_MAX_PERIOD;
	xNvdsConfig.usAbnPosDetectedWindow						= ABN_POS_DET_WINDOW;	
	xNvdsConfig.ucCmdVibrationParamOn						= CMD_ACK_VIBR_ON;
	xNvdsConfig.ucCmdVibrationParamOff						= CMD_ACK_VIBR_OFF;
	xNvdsConfig.ucCmdVibrationParamRep						= CMD_ACK_VIBR_REP;
	
	xNvdsConfig.usGPSMinQuality		        				= GPS_MIN_QUALITY;
	xNvdsConfig.usGPSMaxHAcc	        					= GPS_HPREC_IMPROVMENT_LIMIT;
	xNvdsConfig.ulGPSMaxWaitForFix							= TO_GPS_FIX;
	xNvdsConfig.ulGPSMaxWaitForSat							= TO_GPS_SV_RECVD;
	xNvdsConfig.usGpsEnable									= GPS_ENABLE;
	xNvdsConfig.usGpsPwrDownInt								= GPS_DEFAULT_MOD_PWR_DOWN_INT;
	xNvdsConfig.usGpsPsm									= GPS_DEFAULT_PSM;	
	xNvdsConfig.usGpsRecording								= GPS_RECORDING_DISABLE;			

	strncpy( xNvdsConfig.pcGPSLocEstSrvAddr,		cGPSLocEstSrvAddr, 	  		LEN_GPS_LOCSRV_ADDR );

	xNvdsConfig.usBattEmptyThres							= BATT_EMPTY_THRES;
	xNvdsConfig.usBattLowThres								= BATT_LOW_THRES;
	xNvdsConfig.usBattFullThres								= BATT_FULL_THRES;

	xNvdsConfig.usBattLoopInt								= BATT_MEAS_INTERVAL;
	
	xNvdsConfig.usBleEnable									= BLE_ENABLE;
	xNvdsConfig.ucBleBcnUpdInterval							= T_BLE_BCNUPDATE;	
	xNvdsConfig.ulBleDFMap	 								= BLE_DEFAULT_DFMAP;
	xNvdsConfig.ucBleTxPwr	 								= BLE_DEFAULT_TXPWR;
	xNvdsConfig.usBleRssiCfg								= BLE_RSSI_CFG;
	xNvdsConfig.usBleMinBcn									= BLE_MIN_BCN;
	xNvdsConfig.usBleInDangerDur							= IN_DANGER_ZONE_DURATION;
	xNvdsConfig.usBleForeignAlert							= REACT_FOREIGN_ALERT_DEF;
	xNvdsConfig.usBleScanDuringFixOnly						= BLE_SCAN_ALWAYS;					
	xNvdsConfig.usBLEMaxWaitForBcn							= BLE_MAX_WAIT_FOR_BCN;
	xNvdsConfig.usBleStdAdvInterval							= BLE_ADV_INTVL_STD;
	
	xNvdsConfig.usSecEnable									= SEC_ENABLE;
	
	xNvdsConfig.ucEvacVibrOn								= EVAC_VIBR_ON;
	xNvdsConfig.ucEvacVibrOff								= EVAC_VIBR_OFF;
	xNvdsConfig.ucEvacVibrRep								= EVAC_VIBR_REP;
	
	
	xNvdsConfig.usLogEnable									= LOG_ENABLE;
	
	strncpy( xNvdsConfig.pcAESKey[ 0 ],	 			cAesDefaultKey0, 			AES_BLOCK_LENGTH );
	strncpy( xNvdsConfig.pcAESKey[ 1 ],	 			cAesDefaultKey1, 			AES_BLOCK_LENGTH );
	xNvdsConfig.ucActiveAESKey								= 0;
	
	xNvdsConfig.usModuleSide								= RIGHT_FOOT;

	strncpy( xNvdsConfig.pcGpsTCPStreamingServer,	cGpsTCPStreamingServer, 	LEN_GPS_TCP_STREAM_CFG );
	
	xNvdsConfig.cDangerBcnThres								= DANGER_BCN_THRES;
	xNvdsConfig.cNoAbnBcnThres								= NO_ABN_BCN_THRES;						
	xNvdsConfig.cPrivateBcnThres							= PRIVATE_BCN_THRES;
	xNvdsConfig.cImmobilityBcnThres 						= IMMOBILITY_BCN_THRES;

	memset( ( void * )xNvdsConfig.ucBleBeaconUuidFilter, 0, BLE_LEN_UUID_FILTER * BLE_NUM_UUID_FILTER );

	xNvdsConfig.usBleBcnFilter								= BLE_BCN_FILTER;
	
	xNvdsConfig.usChkSum									= usGenerateCheckSum();
	
	V_TRACE_PRINT( TRACE_NVDS_INIT, TRACE_UART_AND_FILE );
 
	bNVDSReInitialised = true;
}
/*-----------------------------------------------------------*/

/* Invalidate the configuration record. */
void vInvalidateConfig( void )
{
	xNvdsConfig.usMagicKey1 								= 0;
	xNvdsConfig.usMagicKey2 								= 0;
	
	xNvdsConfig.usChkSum									= 0;
	
	vUpdateRecord( CONFIG_FILE_KEY );	
}
/*-----------------------------------------------------------*/

/* Generate a 16-bit checksum on the NVDS contents in between and including the two magic keys.
   Is NOT a re-entrant function! Needs to be protected by the caller.
 */
unsigned short usGenerateCheckSum( void )
{
	unsigned short		usChkSum;
	unsigned short		*pusNVDSdata;
	
	usChkSum = 0;
	
	/* Loop over the used data in the NVDS in between the first magic key and the end of the NVDS excluding the checksum itself. */
	for ( pusNVDSdata = ( unsigned short * )&xNvdsConfig.usMagicKey1; pusNVDSdata < ( unsigned short * )&xNvdsConfig.usChkSum; pusNVDSdata++ )
	{
		usChkSum += *pusNVDSdata;
	}
	
	return usChkSum;
}
/*-----------------------------------------------------------*/

/* Write a string to the configuration record pointed to by pcNVDSDest. 
   Update the checksum at the end of the NVDS. 

   Re-entrant function using a Mutex to protect the NVDS access so that it can be safely called from 
   different tasks.
*/
void vConfigWriteString( signed char *pcNVDSDest, const signed char *pcSrc )
{
	/* Stop all BLE activity. The SoftDevice takes up so many cycles that it stalls the FDS accesses on a collision.
	   It appears that the SoftDevice / SDK library routines do not recover well from a stalling so it is better to prevent 
	   collisions in the first place. */
	vBlockingSuspendAllBleActivity();
	
	configASSERT( xSemaphoreTake( xMutexConfig, TO_MUTEX_CONFIG ) );
	
	strcpy( pcNVDSDest, pcSrc );
	
	#if !defined ( NO_SOFTDEVICE )	
		/* Generate checksum and update NVDS. */
		xNvdsConfig.usChkSum = usGenerateCheckSum();
		vUpdateRecord( CONFIG_FILE_KEY );
	#endif
	
	configASSERT( xSemaphoreGive( xMutexConfig ) );
	
	/* Restore the activity to the same state it was in when vBlockingSuspendAllBleActivity() was called. */
	vResumeAllBleActivity();	
}
/*-----------------------------------------------------------*/

/* Write a byte to the NVDS into the location pointed to by pcNVDSDest. 
   Update the checksum at the end of the NVDS. 
  
   Re-entrant function using a Mutex to protect the NVDS access so that it can be safely called from 
   different tasks.
*/
void vConfigWriteByte( unsigned char *pucNVDSDest, unsigned char ucValue )
{
	/* Stop all BLE activity. The SoftDevice takes up so many cycles that it stalls the FDS accesses on a collision.
	   It appears that the SoftDevice / SDK library routines do not recover well from a stalling so it is better to prevent 
	   collisions in the first place. */
	vBlockingSuspendAllBleActivity();

	configASSERT( xSemaphoreTake( xMutexConfig, TO_MUTEX_CONFIG ) );
	
	*pucNVDSDest = ucValue;
		
	#if !defined ( NO_SOFTDEVICE )	
		/* Generate checksum and update NVDS. */
		xNvdsConfig.usChkSum = usGenerateCheckSum();
		vUpdateRecord( CONFIG_FILE_KEY );
	#endif

	configASSERT( xSemaphoreGive( xMutexConfig ) );

	/* Restore the activity to the same state it was in when vBlockingSuspendAllBleActivity() was called. */
	vResumeAllBleActivity();	
}
/*-----------------------------------------------------------*/

/* Write a short to the NVDS into the location pointed to by pcNVDSDest. 
   Update the checksum at the end of the NVDS. 
  
   Re-entrant function using a Mutex to protect the NVDS access so that it can be safely called from 
   different tasks.
   
   ATTENTION: This function may take several milliseconds to complete! Check NVDS
   status before using in a time-critical section. 
*/
void vConfigWriteShort( unsigned short *pusNVDSDest, unsigned short usValue )
{
	/* Stop all BLE activity. The SoftDevice takes up so many cycles that it stalls the FDS accesses on a collision.
	   It appears that the SoftDevice / SDK library routines do not recover well from a stalling so it is better to prevent 
	   collisions in the first place. */
	vBlockingSuspendAllBleActivity();

	configASSERT( xSemaphoreTake( xMutexConfig, TO_MUTEX_CONFIG ) );
	
	*pusNVDSDest = usValue;
		
	#if !defined ( NO_SOFTDEVICE )	
		/* Generate checksum and update NVDS. */
		xNvdsConfig.usChkSum = usGenerateCheckSum();
		vUpdateRecord( CONFIG_FILE_KEY );
	#endif

	configASSERT( xSemaphoreGive( xMutexConfig ) );

	/* Restore the activity to the same state it was in when vBlockingSuspendAllBleActivity() was called. */
	vResumeAllBleActivity();	
}
/*-----------------------------------------------------------*/

/* Write a long value to the NVDS into the location pointed to by pulNVDSDest. 
   Update the checksum at the end of the NVDS. 
  
   Re-entrant function using a Mutex to protect the NVDS access so that it can be safely called from 
   different tasks.
   
   ATTENTION: This function may take several milliseconds to complete! Check NVDS
   status before using in a time-critical section. 
*/
void vConfigWriteLong( unsigned long *pulNVDSDest, unsigned long ulValue )
{
	/* Stop all BLE activity. The SoftDevice takes up so many cycles that it stalls the FDS accesses on a collision.
	   It appears that the SoftDevice / SDK library routines do not recover well from a stalling so it is better to prevent 
	   collisions in the first place. */
	vBlockingSuspendAllBleActivity();

	configASSERT( xSemaphoreTake( xMutexConfig, TO_MUTEX_CONFIG ) );
	
	*pulNVDSDest = ulValue;
		
	#if !defined ( NO_SOFTDEVICE )	
		/* Generate checksum and update NVDS. */
		xNvdsConfig.usChkSum = usGenerateCheckSum();
		vUpdateRecord( CONFIG_FILE_KEY );
	#endif

	configASSERT( xSemaphoreGive( xMutexConfig ) );
	
	/* Restore the activity to the same state it was in when vBlockingSuspendAllBleActivity() was called. */
	vResumeAllBleActivity();	
}
/*-----------------------------------------------------------*/

/* Write an array to the configuration record pointed to by pcNVDSDest. 
   Update the checksum at the end of the NVDS. 

   Re-entrant function using a Mutex to protect the NVDS access so that it can be safely called from 
   different tasks.
*/
void vConfigWriteArray( unsigned char *pucNVDSDest, const unsigned char *pucSrc, unsigned portBASE_TYPE uxLength )
{
	unsigned char		ucData;
	
	/* Stop all BLE activity. The SoftDevice takes up so many cycles that it stalls the FDS accesses on a collision.
	   It appears that the SoftDevice / SDK library routines do not recover well from a stalling so it is better to prevent 
	   collisions in the first place. */
	vBlockingSuspendAllBleActivity();
	
	configASSERT( xSemaphoreTake( xMutexConfig, TO_MUTEX_CONFIG ) );
	
	memcpy( pucNVDSDest, pucSrc, uxLength );
	
	#if !defined ( NO_SOFTDEVICE )	
		/* Generate checksum and update NVDS. */
		xNvdsConfig.usChkSum = usGenerateCheckSum();
		vUpdateRecord( CONFIG_FILE_KEY );
	#endif
	
	configASSERT( xSemaphoreGive( xMutexConfig ) );
	
	/* Restore the activity to the same state it was in when vBlockingSuspendAllBleActivity() was called. */
	vResumeAllBleActivity();	
}
/*-----------------------------------------------------------*/

/* Read a string from the configuration record into the location pointed to by pcDest. 
   Returns the a pointer to the last character of the string in the destination.
*/
signed char *pcConfigReadString( signed char *pcSrc, signed char *pcDest, portBASE_TYPE xNumChars, bool bTranslateApostr )
{
	signed char		cChar;
	
	do
	{
		cChar = *( pcSrc++ );
		if ( bTranslateApostr && ( cChar == '\'' ) )
		{
			cChar = '\"';
		}
		*( pcDest++ ) = cChar;
		xNumChars--;
	} while ( ( cChar != 0 ) && ( xNumChars > 0 ) );
	
	return --pcDest;
}
/*-----------------------------------------------------------*/

/* Read a byte value from the config record. The actual read is protected by a mutex. */
unsigned char ucConfigReadByte( unsigned char *pucConfigSrc )
{
	unsigned short	ucResult;
	
	configASSERT( xSemaphoreTake( xMutexConfig, TO_MUTEX_CONFIG ) );
	ucResult = *pucConfigSrc;
	configASSERT( xSemaphoreGive( xMutexConfig ) );
	
	return ucResult;
}
/*-----------------------------------------------------------*/

/* Read a short value from the config record. The actual read is protected by a mutex. */
unsigned short usConfigReadShort( unsigned short *pusConfigSrc )
{
	unsigned short	usResult;
	
	configASSERT( xSemaphoreTake( xMutexConfig, TO_MUTEX_CONFIG ) );
	usResult = *pusConfigSrc;
	configASSERT( xSemaphoreGive( xMutexConfig ) );
	
	return usResult;
}
/*-----------------------------------------------------------*/

/* Read a long value from the config record. The actual read is protected by a mutex. */
unsigned long ulConfigReadLong( unsigned long *pulConfigSrc )
{
	unsigned long	ulResult;
	
	configASSERT( xSemaphoreTake( xMutexConfig, TO_MUTEX_CONFIG ) );
	ulResult = *pulConfigSrc;
	configASSERT( xSemaphoreGive( xMutexConfig ) );
	
	return ulResult;
}
/*-----------------------------------------------------------*/

/* Read an array from the configuration record pointed to by pucConfigSrc. */
void vConfigReadArray( unsigned char *pucDest, const unsigned char *pucConfigSrc, unsigned portBASE_TYPE uxLength )
{
	unsigned char		ucData;
	
	configASSERT( xSemaphoreTake( xMutexConfig, TO_MUTEX_CONFIG ) );
	memcpy( pucDest, pucConfigSrc, uxLength );
	configASSERT( xSemaphoreGive( xMutexConfig ) );
}
/*-----------------------------------------------------------*/
