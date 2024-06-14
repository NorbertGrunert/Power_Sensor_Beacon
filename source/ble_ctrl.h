/*
 * Tracker Firmware
 *
 * BLE driver header file
 *
 */
 
#ifndef BLE_CTRL_H
#define BLE_CTRL_H

#include <stdbool.h>
#include "parser.h"
#include "semphr.h"

#include "gsm.h"
/*-----------------------------------------------------------*/

#define NO_SOFTDEVICE_SHUTDOWN								/* With this define, do not shut the softdevice down while Bluetooth is not in use.
															   Shutting it down does not give any power benefit but results in a NRF_ERROR_SVC_HANDLER_MISSING 
															   error after the SoftDevice is restarted. */

/* BLE default TX power. */
#define BLE_DEFAULT_TXPWR			( 8 )					/* 8 dBm */

/* Define Long Range beacon payload fields. */
#define BLE_DEFAULT_DFMAP			( 0x00ffffbf )			/* Default DFMap value for enabled fields. */
#define BLE_HELLO_FIELDS_MASK		( 0x0007c00f )			/* Bit mask which describes all fields in the DFMAP relevant for GPS recording HELLO packets. */

#define BLE_BCN_TXTIM_MSK			( 1l << 0  )			/* Tracker time-stamp (report transmission). */
#define BLE_BCN_PWR_MSK				( 1l << 1  )			/* Conducted BLE output power, signed, in dBm. */
#define BLE_BCN_EVAC_ID_MSK			( 1l << 2  )			/* Evacuation ID in case the EVAC bit is set. */
#define BLE_BCN_BAT_MSK				( 1l << 3  )			/* Battery level. */
#define BLE_BCN_BCN_ONE_MSK			( 1l << 4  )			/* Locator beacon reports. */
#define BLE_BCN_BCN_ALL_MSK			( 7l << 4  )			/* Locator beacon reports. */
#define BLE_BCN_LAT_MSK				( 1l << 7  )			/* GPS latitude. */
#define BLE_BCN_LON_MSK				( 1l << 8  )			/* GPS longitude .*/
#define BLE_BCN_FIX_MSK				( 1l << 9  )			/* GPS fix. */
#define BLE_BCN_LOCM_MSK			( 1l << 10 )			/* Localisation method. */
#define BLE_BCN_HDOP_MSK			( 1l << 11 )			/* GPS Horizontal Dilution of Precision. */
#define BLE_BCN_ACCH_MSK			( 1l << 12 )			/* GPS horizontal accuracy, expressed in meters. */
#define BLE_BCN_SV_MSK				( 1l << 13 )			/* GPS satellites in view. */
#define BLE_BCN_TEMP_MSK			( 1l << 14 )			/* Device temperature, expressed in degrees Celsius. */
#define BLE_BCN_STATE_MSK			( 1l << 15 )			/* Device state. */
#define BLE_BCN_DGSM_MSK			( 1l << 16 )			/* GSM debug information. */
#define BLE_BCN_GSMRSSI_MSK			( 1l << 17 )			/* GSM receive signal strength indicator. */
#define BLE_BCN_GSMQUAL_MSK			( 1l << 18 )			/* GSM receive signal quality. */ 
#define BLE_BCN_BCN2_MSK			( 1l << 19 )			/* Locator beacon reports, 4th bit. */
#define BLE_BCN_UUID_BCN_ONE_MSK    ( 1l << 20 )			/* UUID-filtered locator beacon reports. */
#define BLE_BCN_UUID_BCN_ALL_MSK  ( 0xfl << 20 )			/* UUID-filtered locator beacon reports. */
#define BLE_BCN_RLYADDR_MSK 		( 1l << 28 )			/* Relay command destination address. */
#define BLE_BCN_RLYCMD_MSK			( 1l << 29 )			/* Relay command index. */
#define BLE_BCN_RLYPARAM_MSK		( 1l << 30 )			/* Relay command parameter. */
#define BLE_BCN_EXT					( 1l << 31 )			/* Indicator for extended DFMAPs. */

#define BLE_BCN_TXTIM_IDX			( 0  )					/* Tracker time-stamp (report transmission). */
#define BLE_BCN_PWR_IDX				( 1  )					/* Conducted BLE output power, signed, in dBm. */
#define BLE_BCN_EVAC_ID_IDX			( 2  )					/* Evacuation ID in case the EVAC bit is set. */
#define BLE_BCN_BAT_IDX				( 3  )					/* Battery level. */
#define BLE_BCN_BCN_ONE_IDX			( 4  )					/* Locator beacon reports, bit 0. */
#define BLE_BCN_BCN_TWO_IDX			( 5  )					/* Locator beacon reports, bit 1. */
#define BLE_BCN_BCN_THREE_IDX		( 6  )					/* Locator beacon reports, bit 2. */
#define BLE_BCN_LAT_IDX				( 7  )					/* GPS latitude. */
#define BLE_BCN_LON_IDX				( 8  )					/* GPS longitude .*/
#define BLE_BCN_FIX_IDX				( 9  )					/* GPS fix. */
#define BLE_BCN_LOCM_IDX			( 10 )					/* Localisation method. */
#define BLE_BCN_HDOP_IDX			( 11 )					/* GPS Horizontal Dilution of Precision. */
#define BLE_BCN_ACCH_IDX			( 12 )					/* GPS horizontal accuracy, expressed in meters. */
#define BLE_BCN_SV_IDX				( 13 )					/* GPS satellites in view. */
#define BLE_BCN_TEMP_IDX			( 14 )					/* Device temperature, expressed in degrees Celsius. */
#define BLE_BCN_STATE_IDX			( 15 )					/* Device state. */
#define BLE_BCN_DGSM_IDX			( 16 )					/* GSM debug information. */
#define BLE_BCN_GSMRSSI_IDX			( 17 )					/* GSM receive signal strength indicator. */
#define BLE_BCN_GSMQUAL_IDX			( 18 )					/* GSM receive signal quality. */ 
#define BLE_BCN_BCN_FOUR_IDX		( 19 )					/* Locator beacon reports, bit 3. */
#define BLE_BCN_UUID_BCN_ONE_IDX	( 20 )					/* Locator beacon reports, bit 0. */
#define BLE_BCN_UUID_BCN_TWO_IDX	( 21 )					/* Locator beacon reports, bit 1. */
#define BLE_BCN_UUID_BCN_THREE_IDX	( 22 )					/* Locator beacon reports, bit 2. */
#define BLE_BCN_UUID_BCN_FOUR_IDX	( 23 )					/* Locator beacon reports, bit 3. */
#define BLE_BCN_RLYADDR_IDX 		( 28 )					/* Relay command destination address. */
#define BLE_BCN_RLYCMD_IDX			( 29 )					/* Relay command index. */
#define BLE_BCN_RLYPARAM_IDX		( 30 )					/* Relay command parameter. */


#define BLE_BCN_PL_NON_ENCR_OVR_LEN	(  4 )					/* BLE beacon payload overhead length (not encrypted part only): AD_TYPE + CIC + MESH. */
#define BLE_BCN_PL_ENCR_OVR_LEN		( 16 )					/* BLE beacon payload overhead length (encrypted part only): DESC + TRAXXS_ID + ID + DFMAP. */

#define BLE_BCN_FID_LEN				(  8 )					/* Tracker ID. */
#define BLE_BCN_RXTIM_LEN			(  2 )					/* Tracker time-stamp (reception). */
#define BLE_BCN_FDESC_LEN			(  1 )					/* Foreign TL device report descriptor. */
#define BLE_BCN_FRSSI_LEN			(  1 )					/* Foreign TL device received signal strength. */
#define BLE_BCN_FLR_LEN				(  1 )					/* RF interface used for reception. */
#define BLE_BCN_FDFMAP_LEN			(  1 )					/* DFMAP. */
		                                            
#define BLE_BCN_TXTIM_LEN			(  2 )					/* Tracker time-stamp (reported transmission). */
#define BLE_BCN_PWR_LEN				(  1 )					/* Conducted BLE output power, signed, in dBm. */
#define BLE_BCN_EVAC_ID_LEN			(  1 )					/* Evacuation ID in case the EVAC bit is set. */
#define BLE_BCN_BAT_LEN				(  1 )					/* Battery level. */
#define BLE_BCN_BCN_LEN				( 11 )					/* Locator beacon reports. */
#define BLE_BCN_LAT_LEN				(  4 )					/* GPS latitude. */
#define BLE_BCN_LON_LEN				(  5 )					/* GPS longitude .*/
#define BLE_BCN_FIX_LEN				(  1 )					/* GPS fix. */
#define BLE_BCN_LOCM_LEN			(  1 )					/* Localisation method. */
#define BLE_BCN_HDOP_LEN			(  2 )					/* GPS Horizontal Dilution of Precision. */
#define BLE_BCN_ACCH_LEN			(  2 )					/* GPS horizontal accuracy, expressed in meters. */
#define BLE_BCN_SV_LEN				(  1 )					/* GPS satellites in view. */
#define BLE_BCN_TEMP_LEN			(  1 )					/* Device temperature, expressed in degrees Celsius. */
#define BLE_BCN_STATE_LEN			(  1 )					/* Device state. */
#define BLE_BCN_DGSM_LEN			( 13 )					/* GSM debug information. */
#define BLE_BCN_GSMRSSI_LEN			(  1 )					/* GSM receive signal strength indicator. */
#define BLE_BCN_GSMQUAL_LEN			(  1 )					/* GSM receive signal quality. */ 
#define BLE_BCN_RLYADDR_LEN 		(  8 )					/* Relay command destination address. */
#define BLE_BCN_RLYCMD_LEN			(  1 )                  /* Relay command index. */
#define BLE_BCN_RLYPARAM_LEN		(  2 )                  /* Relay command parameter. */
		                                            
/* Define FTL server data field bits in FDFMAP. */
#define GSM_FTL_FDFMAP_FID_MSK		( 1l <<  0 )			/* Foreign TL device identifier (IMEI).	*/
#define GSM_FTL_FDFMAP_FRXTIM_MSK	( 1l <<  1 )			/* Time-stamp of the report reception based on the time base of the local device. */ 
#define GSM_FTL_FDFMAP_FTXTIM_MSK	( 1l <<  2 )			/* Time-stamp of the report transmission based on the time base of the remote (foreign) device. */ 
#define GSM_FTL_FDFMAP_FDESC_MSK	( 1l <<  3 )			/* Foreign TL device report descriptor. */ 
#define GSM_FTL_FDFMAP_FRSSI_MSK	( 1l <<  4 )			/* Foreign TL device received signal strength. */ 
#define GSM_FTL_FDFMAP_FLR_MSK		( 1l <<  5 )			/* RF interface used for receiving the foreign TL device  */ 
#define GSM_FTL_FDFMAP_FPWR_MSK		( 1l <<  6 )			/* Foreign TL device transmit power in dBm. */ 
#define GSM_FTL_FDFMAP_FBAT_MSK		( 1l <<  7 )			/* Foreign TL device battery level. */ 
#define GSM_FTL_FDFMAP_FBCN_MSK		( 1l <<  8 )			/* One or more locator beacon ID received by the foreign TL. */ 
#define GSM_FTL_FDFMAP_FLAT_MSK		( 1l <<  9 )			/* Foreign TL device latitude (LAT). */ 
#define GSM_FTL_FDFMAP_FLON_MSK		( 1l << 10 )			/* Foreign TL device longitude (LON). */ 
#define GSM_FTL_FDFMAP_FFIX_MSK		( 1l << 11 )			/* Foreign GPS fix type. */ 
#define GSM_FTL_FDFMAP_FLOCM_MSK	( 1l << 12 )			/* Foreign TL device localisation method. */ 
#define GSM_FTL_FDFMAP_FHDOP_MSK	( 1l << 13 )			/* Foreign TL GPS Horizontal Dilution of Precision (HDOP). */ 
#define GSM_FTL_FDFMAP_FACCH_MSK	( 1l << 14 )			/* Foreign TL GPS horizontal accuracy (ACCH). */ 
#define GSM_FTL_FDFMAP_FSV_MSK		( 1l << 15 )			/* Foreign TL GPS satellites in view (SV). */ 
#define GSM_FTL_FDFMAP_FTEMP_MSK	( 1l << 16 )			/* Foreign TL temperature, expressed in degrees Celsius. */ 
#define GSM_FTL_FDFMAP_FSTATE_MSK	( 1l << 17 )			/* Foreign TL Device state. */ 
#define GSM_FTL_FDFMAP_FDGSM_MSK	( 1l << 18 )			/* Foreign TL GSM debug information (DGSM). */ 
#define GSM_FTL_FDFMAP_FGSMRSSI_MSK	( 1l << 19 )			/* Foreign TL GSM receive signal strength indicator (GSMRSSI). */ 
#define GSM_FTL_FDFMAP_FGSMQUAL_MSK	( 1l << 20 )			/* Foreign TL GSM receive signal quality (GSMQUAL). */ 

#define GSM_FTL_FDFMAP_FID_IDX		(  0 )					/* Foreign TL device identifier (IMEI).	*/
#define GSM_FTL_FDFMAP_FRXTIM_IDX	(  1 )					/* Time-stamp of the report reception based on the time base of the local device. */ 
#define GSM_FTL_FDFMAP_FTXTIM_IDX	(  2 )					/* Time-stamp of the report transmission based on the time base of the remote (foreign) device. */ 
#define GSM_FTL_FDFMAP_FDESC_IDX	(  3 )					/* Foreign TL device report descriptor. */ 
#define GSM_FTL_FDFMAP_FRSSI_IDX	(  4 )					/* Foreign TL device received signal strength. */ 
#define GSM_FTL_FDFMAP_FLR_IDX		(  5 )					/* RF interface used for receiving the foreign TL device  */ 
#define GSM_FTL_FDFMAP_FPWR_IDX		(  6 )					/* Foreign TL device transmit power in dBm. */ 
#define GSM_FTL_FDFMAP_FBAT_IDX		(  7 )					/* Foreign TL device battery level. */ 
#define GSM_FTL_FDFMAP_FBCN_IDX		(  8 )					/* One or more locator beacon ID received by the foreign TL. */ 
#define GSM_FTL_FDFMAP_FLAT_IDX		(  9 )					/* Foreign TL device latitude (LAT). */ 
#define GSM_FTL_FDFMAP_FLON_IDX		( 10 )					/* Foreign TL device longitude (LON). */ 
#define GSM_FTL_FDFMAP_FFIX_IDX		( 11 )					/* Foreign GPS fix type. */ 
#define GSM_FTL_FDFMAP_FLOCM_IDX	( 12 )					/* Foreign TL device localisation method. */ 
#define GSM_FTL_FDFMAP_FHDOP_IDX	( 13 )					/* Foreign TL GPS Horizontal Dilution of Precision (HDOP). */ 
#define GSM_FTL_FDFMAP_FACCH_IDX	( 14 )					/* Foreign TL GPS horizontal accuracy (ACCH). */ 
#define GSM_FTL_FDFMAP_FSV_IDX		( 15 )					/* Foreign TL GPS satellites in view (SV). */ 
#define GSM_FTL_FDFMAP_FTEMP_IDX	( 16 )					/* Foreign TL temperature, expressed in degrees Celsius. */ 
#define GSM_FTL_FDFMAP_FSTATE_IDX	( 17 )					/* Foreign TL Device state. */ 
#define GSM_FTL_FDFMAP_FDGSM_IDX	( 18 )					/* Foreign TL GSM debug information (DGSM). */ 
#define GSM_FTL_FDFMAP_FGSMRSSI_IDX	( 19 )					/* Foreign TL GSM receive signal strength indicator (GSMRSSI). */ 
#define GSM_FTL_FDFMAP_FGSMQUAL_IDX	( 20 )					/* Foreign TL GSM receive signal quality (GSMQUAL). */ 

/* Define timers */
#define	TO_BLE_AT			(    1  * portTICKS_PER_SEC ) 	/* Time-out (ms) for getting a response from the BLE module. */
#define	TO_BLE_FAST			( 0.03  * portTICKS_PER_SEC ) 	/* Time-out (ms) for getting a response from the BLE module to the At and AT+BTADDR query. */
#define T_BLE_STARTUP		(    5  * portTICKS_PER_SEC ) 	/* Time-out (s) for the BLE module to become ready after power-on. */
#define TO_BLE_UMRS			(    2  * portTICKS_PER_SEC ) 	/* Time-out (s) for the BLE module to acknowledge the baudrate switch. */
#define TO_BLE_SCAN			(   60  * portTICKS_PER_SEC ) 	/* Time-out (s) for terminating a BLE scan. */
#define TO_BLE_FW_RESP		(  0.5  * portTICKS_PER_SEC ) 	/* Time-out (ms) for receiving an expected response from the BLE module bootloader. */
#define TO_BLE_VERSION_READ (   10 )						/* Time-out (s) for reqesting the BLE module version. */

#define TO_BLE_ACTIVITY_OFF ( 500 )							/* Time-out (ticks) for stopping all BLE activity. */

#define T_BLE_RST			(  0.2  * portTICKS_PER_SEC ) 	/* Time (ms) for the BLE module's reset line to stay active for module reset. */
#define T_BLE_RTOSBOOT		(    5  * portTICKS_PER_SEC ) 	/* Time (s) to allow the BLE module to boot its operating system. */

#define T_BLE_BCNUPDATE		(   30 ) 						/* Interval (s) for updating the BLE module with current beacon information. */

#define T_BLE_RLYCMD		(   60  * portTICKS_PER_SEC ) 	/* Send time (s) for relay commands. */

#define T_BLE_LED_RX_TL		(  0.3  * portTICKS_PER_SEC )		/* LED blue flash duration when receiving a beacon from a TL device. */
#define T_BLE_LED_RX_BCN	( 0.01  * portTICKS_PER_SEC )		/* LED blue flash duration when receiving a location beacon. */

#define BLE_ENABLE			( true )						/* Default value for BLE enable. */

#define BLE_RSSI_CFG		( ( 0x2 << 13 ) + ( 10 << 8 ) ) /* Default RSSI filter configuration.
															   BLE_RSSI_CFG[ 15:13 ]	Algorithm, (0 = none, 1 = avg. 2 = max. avg.)
															   BLE_RSSI_CFG[ 12: 0 ]	Window size in sec.	*/
#define BLE_MIN_BCN			(  5 )							/* Minimum number of BLE localiser beacons to allow for shutting down the GPS. */
	                                                
#define bleCMD_QUEUE_SIZE	( 10 )							/* Length of the GSM command queue. */
#define bleCMD_BLOCKTIME	( 10 * portTICKS_PER_SEC  )		/* Max. time (s) to wait for the BLE command queue to become available. */

#define BLE_ADV_INTVL_STD	( 8192 )						/* Standard advertising interval (5.12s). */
#define BLE_ADV_INTVL_ALERT	( 160 )							/* Advertising interval in case of ALERT (100ms). */

#define BLE_SCAN_ALWAYS				( 0x0000 )				/* Always scan for BLE beacons (default). */
#define BLE_SCAN_DURING_FIX_ONLY	( 0x0001 )				/* Perform BLE scans only while looking for BLE beacons. */
#define BLE_MAX_WAIT_FOR_BCN		( 0x000A )				/* If BLE scans are performed only while looking for BLE beacons, maximum time to wait for a beacon. */

#define BLE_START_SD		( true )
#define BLE_STOP_SD			( false )

#define	BLE_SUSP_SEMAPH_MAX	( 7 )							/* Max. count value for queued BLE suspend requests. */


/* Advertising payload descriptor definitions. */
#define	BLE_EVAC					( 0x01 )
#define	BLE_ALERT					( 0x02 )
#define	BLE_SOS						( 0x04 )
#define	BLE_XSWITCH_ORIGIN			( 0x08 )
#define	BLE_XSWITCH_OBSERVED		( 0x10 )
#define	BLE_RELAYCMD				( 0x20 )
#define	BLE_DANGER					( 0x40 )
/*-----------------------------------------------------------*/


struct xBLE_MODULE_STATE
{
	bool	bBleLocRequest  		: 1;
	bool	bBleAlertRequest  		: 1;
	bool	bBleInDangerZoneRequest	: 1;
	bool	bBleHeartBeatRequest	: 1;
	bool	bBleTxSuspended			: 1;
	bool	bBleSuspended			: 1;
	bool	bBleTxRelayCmd			: 1;
	bool	bBleSoftDeviceOn		: 1;
};
/*-----------------------------------------------------------*/

/* Commands from the CTRL task or the remote server to the BLE task. */
enum xBLE_CMD
{
	BLE_LOC_START,							/* Start localisation via BLE beacons. */
	BLE_LOC_STOP,							/* Stop localisation via BLE beacons. */ 
	BLE_SCAN_START,							/* Perform a single BLE scan for beacons. */
	BLE_SCAN_INT_UPDATE,					/* The configured scan interval has been updated by the server. */
	BLE_RSSI_CFG_UPDATE,					/* The configured RSSI algorithm parameters have been updated by the server. */
	BLE_SR_ALERT_SOS_START,					/* Start advertising ALERT or SOS state on short-range. */
	BLE_SR_ALERT_SOS_STOP,					/* Stop advertising ALERT or SOS state on short-range. */
	BLE_SR_EVAC_START,						/* Start advertising EVACUATION state on short-range. */
	BLE_SR_EVAC_STOP,						/* Stop advertising EVACUATION state on short-range. */
	BLE_RX_FOREIGN_ALERT_SOS,				/* Received foreign Alert or SOS. */
	BLE_BEACON_CAPTURED,					/* Received a beacon. */
	BLE_UPDATE_BCN,							/* Update the beacon data. */
	BLE_TX_SUSPEND,							/* Suspend sending beacons. */
	BLE_TX_RESUME,							/* Resume sending beacons. */
	BLE_RELAY_CMD,							/* Relay a command to a remote device. */
	BLE_RELAYED_LED_CMD,					/* Execute a relayed LED command. */
	BLE_RELAYED_VIBR_CMD,					/* Execute a relayed vibration command. */
	BLE_SUSPEND,							/* Suspend all BLE activities. */
	BLE_RESUME								/* Resume BLE activities. */ 
};
/*-----------------------------------------------------------*/

/* Public function prototypes. */
extern void vBleCtrlInit( UBaseType_t uxPriority );

/* Obtain the BLE module state. */
extern struct xBLE_MODULE_STATE xGetBleModuleState( void );

/* Suspend all BLE activity: receptions as well as transmissions (blocking). */
extern void vBlockingSuspendAllBleActivity( void );

/* Suspend all BLE activity: receptions as well as transmissions (non-blocking). */
extern void vNonBlockingSuspendAllBleActivity( void );

/* Restore the activity to the same state it was in when vBlockingSuspendAllBleActivity() was called. */
extern void vResumeAllBleActivity( void );

/* Non-blocking function which tries to align the BLE suspended/resumed state with the actual state. */
extern void vNonBlockingAlignBleActivitySuspension( void );

/* Suspend BLE TX activity. */
extern void vNonBlockingSuspendBleTxActivity( void );

/* Resume BLE TX activity. */
extern void vResumeBleTxActivity( void );
/*-----------------------------------------------------------*/

/* Global variables. */
/* BLE command queue handle. The queue is read in the BLE task and filled in the CTRL task. */
extern QueueHandle_t 				xBleCmdQueue; 

/* Parameters for reaying commands to remote devices via BLE:
		destination address (IMEI)
		command index
		command parameter				*/
extern char							cRelayCmdDestination[ 8 ];
extern unsigned portBASE_TYPE		uxRelayCmdIdx;
extern bool							bRelayCmdParamPresent;
extern unsigned short				usRelayCmdParam;

/* Status variable reflecting the BLE module state. */
extern struct xBLE_MODULE_STATE		xBleModuleState;
/*-----------------------------------------------------------*/

#endif