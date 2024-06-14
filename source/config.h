/*
 * Tracker Firmware
 *
 * NVDS layout
 *
 */
#ifndef NVDS_H
#define NVDS_H

/* Standard include files. */
#include "FreeRTOS.h"
#include "task.h"

#include "tracker.h"

/* Device specific include files. */
#include "drv_nvm.h"
#include "drv_aes.h"
#include "ble_adreport.h"

#include "gsm.h"
/*-----------------------------------------------------------*/

#define	MAGIC_KEY_1					( 0x1234 )		/* Value of the NVDS magic key. */
#define	MAGIC_KEY_2					( 0x5678 )		/* Value of the NVDS magic key. */

#define	LEN_GSM_GPRS_CONN_CFG		( 0x20 )		/* Size of the GPRS configuration parameter string. */
#define	LEN_GSM_TCP_CONN_CFG		( 0x20 )		/* Size of the TCP configuration parameter string. */
#define	LEN_GPS_TCP_STREAM_CFG		( 0x20 )		/* Size of the GPS TCP streaming server configuration parameter string. */
#define	LEN_GPS_LOCSRV_ADDR			( 0x20 )		/* Size of the location server address parameter string. */

#define TO_MUTEX_CONFIG				( 20 * portTICKS_PER_SEC ) 		/* Time-out (s) for gaining access to the config record. */
/*-----------------------------------------------------------*/

/* Public function prototypes. */
/* Initialise the configuration data handler. */
extern void				vConfigInit( void );

/* Initialise the configuration record with the default values. */
extern void				vInitialiseConfig( void );

/* Invalidate the configuration record. */
extern void 			vInvalidateConfig( void );
/*-----------------------------------------------------------*/

/* Global structure declaring the variables storewd in the NVDS.
   A structure is used to ensure that variablkes are stored in a specific sequence, i.e. make sure that
   the variables are inside the pair of MAGIC_KEY1 and MAGIC_KEY2. */
struct xNVDS_CONFIG
{
	/* Magic key 1. 
	   This key delimits the area checked by checksum and intialised by the FW. */
	unsigned short		usMagicKey1;				

	/* Backup of the CTRL state to be restored in case of an uncommanded reboot. 
	   Only ALERT or SOS state values are restored. All other states are considered to be
	   not important to be restored. */
	enum xCTRL_STATE	xCtrlStateBackup;
	

	/* GPRS Connection configuration. */
	signed char			pcGsmGPRSConnCfg[ LEN_GSM_GPRS_CONN_CFG + 1 ];			

	/* TCP Connection configuration. */
	signed char			pcGsmTCPConnCfg[ LEN_GSM_TCP_CONN_CFG + 1 ];				
	signed char			pcGsmSecTCPConnCfg[ LEN_GSM_TCP_CONN_CFG + 1 ];				
	
	/* URL of the FTP server for FW download. Content might be a URL or an IP address. */
	signed char			pcFTPServerURL[ FWUPD_MAX_SRV_URL + 1 ];				
	
	/* DFMap configuration. */
	unsigned long		ulDFMap;											
	
	unsigned short		usCfgRat;											
	unsigned long		ulRegTimeOut;										

	/* Minimum intervals between transmissions in different states. */
	unsigned short		usActivePositionSendInterval;						
	unsigned short		usInactivePositionSendInterval;						
	unsigned short		usStillPositionSendInterval;						
	unsigned short		usSleepPositionSendInterval;						
	unsigned short		usOooPositionSendInterval;							
	unsigned short		usChargingPositionSendInterval;						
	unsigned short		usAlertPositionSendInterval;						
	unsigned short		usGpsRecordingPacketInterval;

	/* Minimum interval between transmissions to power-down the GSM/GPS module between transmissions. */
	unsigned short		usModulePwrDownInt;								

	/* Accelerometer parameters for motion detection. */  
	unsigned short		usStepAccelerationThreshold;					
	unsigned short		usStepMinInterval;								
	unsigned short		usStepMaxInterval;								
	unsigned short		usActiveNumberSteps;							

	/* Accelerometer parameters for still detection. */   
	unsigned short		usStillDuration;								

	/* Accelerometer parameters for immobility alert detection. */   
	unsigned short		usImmobilityDuration;

	/* Accelerometer parameters for sleep detection. */
	unsigned short		usSleepDuration;								

	/* Accelerometer parameters for abnormal position detection. */
	unsigned short		usAbnormalPositionZAccelerationThreshold;		
	unsigned short		usAbnormalPositionDuration;						
	unsigned short		usAbnormalMaxOrientationChg;					

	/* Accelerometer parameters for SOS movement detection. */    
	unsigned short		usSosAccelerationThreshold;						
	unsigned short		usSosMinXAccelerationThreshold;
	unsigned short		usSosMinZAccelerationThreshold;
	unsigned short		usSosMaxZAccelerationThreshold;
	unsigned short		usSosMinInterval;								
	unsigned short		usSosMaxInterval;								
	unsigned short		usSosNumberPeaks;								
	
	/* Accelerometer parameters for no abnormal position detection command and confirmation vibration. */   
	unsigned short		usNoAbnEnabled;								
	unsigned short		usNoAbnCmdPeaks;								
	unsigned short		usNoAbnMaxPeriod;							
	unsigned short		usAbnPosDetectedWindow;							
	unsigned char		ucCmdVibrationParamOn;						
	unsigned char		ucCmdVibrationParamOff;						
	unsigned char		ucCmdVibrationParamRep;						

	/* Accelerometer parameters for alert cancel detection. */  
	unsigned char		ucAlertCancelMethod;						
	unsigned short		usAlertCancelThreshold;						
	unsigned short		usAlertCancelMinInterval;					
	unsigned short		usAlertCancelMaxInterval;					
	unsigned short		usAlertCancelNumberPeaks;					
	unsigned short		usAlertCancelTimeout;						
	unsigned short		usSosCancelTimeout;							
                                                                           
	/* Minimum HDOP (times 10) for an acceptable GPS fix. */               
	unsigned short		usGPSMinQuality;							
	
	/* Maximum horizontal accuracy improvement within 2 seconds for an acceptable GPS fix. */
	unsigned short		usGPSMaxHAcc;								

	/* Maximum time waiting for a good fix before sending a position packet without position. */
	unsigned long		ulGPSMaxWaitForFix;							
	
	/* Maximum time waiting for a good fix before sending a position packet without position. */
	unsigned long		ulGPSMaxWaitForSat;

	/* GPS enable. */
	unsigned short		usGpsEnable;								
	
	/* Minimum interval between transmissions to power-down the GPS module only between transmissions. */
	unsigned short		usGpsPwrDownInt;									

	/* GPS Power Save Mode. */
	unsigned short		usGpsPsm;											

	/* GPS recording mode. */
	unsigned short		usGpsRecording;											

	/* URL of the XSole location estimation server. */
	signed char			pcGPSLocEstSrvAddr[ LEN_GPS_LOCSRV_ADDR + 1 ];			

	/* Voltage threshold for power-safe mode. */
	unsigned short		usBattEmptyThres;									
	unsigned short		usBattLowThres;										
	unsigned short		usBattFullThres;									

	/* Loop interval for reading battery voltage and temperature. The voltage IIR filter runs at the same time base. */
	unsigned short		usBattLoopInt;

	/* Bluetooth Low Energy parameters. */
	unsigned short		usBleEnable;										
	unsigned char		ucBleBcnUpdInterval;									
	unsigned long		ulBleDFMap;											
	unsigned char		ucBleTxPwr;											
	unsigned short		usBleRssiCfg;												
	unsigned short		usBleMinBcn;											
	unsigned short		usBleInDangerDur;									
	unsigned short		usBleForeignAlert;									
	unsigned short		usBleScanDuringFixOnly;		
	unsigned short		usBLEMaxWaitForBcn;
	unsigned short		usBleStdAdvInterval;
	
	/* Enable flag for TLS. */
	unsigned short		usSecEnable;										
	
	/* FTP mode. */
	signed char			pcFTPCfg[ FTP_MAX_CFG + 1 ];						

	/* Evacuation alert virbration parameters. */
	unsigned char		ucEvacVibrOn;										
	unsigned char		ucEvacVibrOff;										
	unsigned char		ucEvacVibrRep;										
	                                                                    
	/* Enable flag for log. */                                          
	unsigned short		usLogEnable;										
	                                                                    
	/* AES encryption keys for BLE beacons. */                          
	signed char			pcAESKey[ 2 ][ AES_BLOCK_LENGTH ];					
	unsigned char		ucActiveAESKey;										
	                                                                    
	/* GSM module identity (IMEI). */                                   
	signed char 		pcGsm_ID[ LEN_GSM_ID + 1 ];							
	                                                                    
	/* SIM ICCID */                                                     
	signed char 		pcGsm_ICCID[ LEN_GSM_ICCID + 1 ];					
	                                                                    
	/* Module side. Left or right (default) foot. */                    
	unsigned short		usModuleSide;										
	                                                                    
	/* GPS stream IP port. 0 for disabled. */                           
	signed char 		pcGpsTCPStreamingServer[ LEN_GPS_TCP_STREAM_CFG + 1 ];	
	                                                                    
	/* RSSI threshold to accept 'danger zone' beacons. */               
	signed char			cDangerBcnThres;									
	                                                                    
	/* RSSI threshold to accept 'no abnormal position zone' beacons. */ 
	signed char			cNoAbnBcnThres;										
	                                                                    
	/* RSSI threshold to accept 'private zone' beacons. */              
	signed char			cPrivateBcnThres;									

	/* RSSI threshold to accept 'immobility' beacons. */                 
	signed char			cImmobilityBcnThres;

	/* iBeacon UUID filter pattern. */
	unsigned char		ucBleBeaconUuidFilter[ BLE_NUM_UUID_FILTER ][ BLE_LEN_UUID_FILTER ];

	/* BLE localiser beacon filter configuration. */
	unsigned short		usBleBcnFilter;
	                                                                        
	/* Configuration record check sum. */                       
	unsigned short		usChkSum;											

	/* Magic key 2. */                                                  
	unsigned short		usMagicKey2;										
};

/* Flag indicating that the NVDS has been reinitialised because of corruption. */
extern bool						bNVDSReInitialised;

/* RAM copy  of the configuration record. */
extern struct xNVDS_CONFIG		xNvdsConfig;
/*-----------------------------------------------------------*/

/* Public function prototypes. */
/* Check if the configuration handler is fully initialised. */
extern bool bCheckConfigInitialised( void );

/* Generate a 16-bit checksum on the NVDS contents in between and including the two magic keys.
   Is NOT a re-entrant function! Needs to be protected by the caller.
 */
extern unsigned short usGenerateCheckSum( void );

/* Write a string to the configuration record pointed to by pcNVDSDest. */
extern void vConfigWriteString( signed char *pcNVDSDest, const signed char *pcSrc );

/* Write a byte to the NVDS into the location pointed to by pcNVDSDest. */
extern void vConfigWriteByte( unsigned char *pucNVDSDest, unsigned char ucValue );

/* Write a short to the NVDS into the location pointed to by pcNVDSDest. */
extern void vConfigWriteShort( unsigned short *pusNVDSDest, unsigned short usValue );

/* Write a long value to the NVDS into the location pointed to by pulNVDSDest. */
extern void vConfigWriteLong( unsigned long *pulNVDSDest, unsigned long ulValue );

/* Write an array to the configuration record pointed to by pcNVDSDest. */
extern void vConfigWriteArray( unsigned char *pucNVDSDest, const unsigned char *pucSrc, unsigned portBASE_TYPE uxLength );

/* Read a string from the configuration record into the location pointed to by pcDest. */
extern signed char *pcConfigReadString( signed char *pcSrc, signed char *pcDest, portBASE_TYPE xNumChars, bool bTranslateApostr );

/* Read a byte value from the config record. The actual read is protyected by a mutex. */
extern unsigned char ucConfigReadByte( unsigned char *pucConfigSrc );

/* Read a short value from the config record. The actual read is protyected by a mutex. */
unsigned short usConfigReadShort( unsigned short *pusConfigSrc );

/* Read a long value from the config record. The actual read is protyected by a mutex. */
extern unsigned long ulConfigReadLong( unsigned long *pulConfigSrc );

/* Read an array from the configuration record pointed to by pucConfigSrc. */
extern void vConfigReadArray( unsigned char *pucDest, const unsigned char *pucConfigSrc, unsigned portBASE_TYPE uxLength );
/*-----------------------------------------------------------*/

#endif