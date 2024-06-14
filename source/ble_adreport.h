/*
 * Tracker Firmware
 *
 * BLE driver header file
 *
 */
 
#ifndef BLE_ADREPORT_H
#define BLE_ADREPORT_H

#include <stdbool.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

/* Device specific include files. */
#include "main.h"
#include "gsm.h"
/*-----------------------------------------------------------*/

/* Size of the command queue from the timer task to the advertising report task. */
#define	bleADRPT_QUEUE_SIZE				10

/* RSSI filter algorithm configuration values. */
#define RSSI_NONE					( 0 )
#define RSSI_AVG					( 1 )
#define RSSI_MAXAVG					( 2 )

#define FILTER_POLL_TIME			( 1 * portTICKS_PER_SEC )			/* Time interval at which the advertiser RSSI filter is called. */
#define FILTER_INTERVAL				( 10 * portTICKS_PER_SEC )			

#define MAX_LOC_BCN_ENTRIES			( 12 )								/* Maximum numbers of location beacons to report to the server. */
#define MAX_TL_BCN_ENTRIES			( 16 )								/* Maximum numbers of TL beacons to report to the server. */

#define BLE_RF_IF_SR				( 1 )
#define BLE_RF_IF_LR				( 2 )

#define	BLE_STOREDATA				( true )
#define BLE_TRASHDATA				( false )

#define BCNUSE_LOC					( 0x0001 )							/* BcnUse field bitmask for localiser beacon. */
#define BCNUSE_NO_ABNORMAL			( 0x0002 )							/* BcnUse field bitmask for zone without abnormal position detection. */
#define BCNUSE_DANGER				( 0x0004 )							/* BcnUse field bitmask for danger zone. */
#define BCNUSE_PRIVATE				( 0x0008 )							/* BcnUse field bitmask for private zone. */
#define BCNUSE_IMMOBILITY			( 0x0010 )							/* BcnUse field bitmask for zone with alert on immobility. */
				
#define BCNUSE_BLOCKTIME			( 10 )								/* No abnormal position beacons are acted upon / reported once every 10s. */
#define IN_DANGER_ZONE_DURATION		( 30 )								/* Time interval after having received a danger zone beacon during wich the module is considered to be within a danger zone. */
#define REACT_FOREIGN_DANGER_SR		( 0x0001 )							/* React on remote danger signalled in short-range beacons. */
#define REACT_FOREIGN_DANGER_LR		( 0x0002 )							/* React on remote danger signalled in long-range beacons. */
#define REACT_FOREIGN_ALERT_SR		( 0x0004 )							/* React on remote alert signalled in short-range beacons. */
#define REACT_FOREIGN_ALERT_LR		( 0x0008 )							/* React on remote alert signalled in long-range beacons. */
#define REACT_FOREIGN_SOS_SR		( 0x0010 )							/* React on remote SOS signalled in short-range beacons. */
#define REACT_FOREIGN_SOS_LR		( 0x0020 )							/* React on remote SOS signalled in long-range beacons. */
#define REACT_FOREIGN_ALERT_DEF		( 0x0000 )							/* By default, do not react on long-range beacons. */

#define DANGER_BCN_THRES 			( -128 )							/* Default RSSI threshold for accepting danger zone beacons [dBm]. */
#define NO_ABN_BCN_THRES    		( -128 )							/* Default RSSI threshold for accepting beacons for zones without abnormal position detection [dBm]. */
#define PRIVATE_BCN_THRES   		( -128 )							/* Default RSSI threshold for accepting private zone beacons [dBm]. */
#define IMMOBILITY_BCN_THRES 		( -128 )							/* Default RSSI threshold for accepting immobility zone beacons [dBm]. */

#define STRING_TRUNCATED			( 0xffff )

#define BLE_BCN_VAR_FLD_LEN			( 38 + MAX_LOC_BCN_ENTRIES * 11 )	/* Total maximum length of all variable fields, starting from FTXTIM to FGSMQUAL
																		   including MAX_LOC_BCN_ENTRIES locator beacons (see ble_ctrl.h for the ordered list of fields). */

#define BLE_LEN_UUID_FILTER			( 16 )								/* Length of a beacon UUID filter, determined by the field length (iBeacon spec). */
#define BLE_NUM_UUID_FILTER			( 2 )								/* Number of configurable BLE beacon UUID filters. */

#define BLE_BCN_FILTER				( 0x0000 )							/* Localiser beacon filter configuration. This filter determines how localiser beacons
																		   are stored in the xBleLocBcnGSM and xBleLocBcnBLE tables. 
																		   	bit [ 2:0]		BCN_FILTER_OPTION		Beacon replacement option.
																							For received beacons for which is there is already an entry in the table:
																								0	A newer one always replaces the present entry. Only the RSSI and RX timestamp
																								    are updated.
																								1	As option 0.
																								2	A newer one only replaces the present entry if its RSSI is higher.
																								3	A newer one only replaces the present entry if its RSSI is higher
																									and higher than the beacon's reference RSSI (RSSI_REF) + BCN_REF_RSSI_OFFSET.
																							To decide which entry to replace if the table is full:
																								0	Replace the oldest entry.
																								1	Replace the entry with the lowest RSSI.
																								2	As option 1.
																								3	As option 1.
																		   	bit [15:8]		BCN_REF_RSSI_OFFSET		Offset to the beacon's reference RSSI. 
																							signed 8-bit value (range -128dB...+127dB). */
#define BLE_BCN_DO_NOT_STORE		( -1 )								/* Flag indicating that a localiser beacon should not be stored in the beacon table. */
/*-----------------------------------------------------------*/

/* Structure of one BLE beacon entry in the location beacon dictionary. */
#define BLE_LOC_BCN_LEN				( 11 )								/* Length of a location beacon entry in bytes. */

/* Structure of a single location beacon entry data. */
union uBLE_LOC_BEACON_DATA
{
	struct xBLE_LOC_BEACON_DATA_ENTRY
	{
		unsigned char				ucBeaconAddr[ 6 ];						/* Device-specific part of the AdvA (lower 24 bit). 
																				If bIsSwissPhoneBeacon is true, replaced by a combination of UUID filter match 
																				index and major and minor fields. */
		unsigned short				usTimeStamp;							/* Time stamp of the beacon reception (in RTC seconds). */
		signed char					cRSSI;									/* RSSI of received signal. */
		signed char					cRefRSSI;								/* Reference RSSI of this beacon at 1m distance (as reported by the beacon). */
		unsigned char				ucBatteryLevel;							/* Battery level of the beacon (battery voltage; 100% is 3.2V). */
	}								xBleLocBeaconData;
	unsigned char					ucBleLocBeaconData[ BLE_LOC_BCN_LEN ];
};

/* Structure oif a single location beacon entry. */
struct xBLE_LOC_BEACON_ENTRY
{
	bool						bIsSwissPhoneBeacon;					/* True, if the beacon is a special SwissPhone beacon. */
	union uBLE_LOC_BEACON_DATA	uBleLocBeaconData;
};

/* Storage for BLE location beacons if recording is required. */ 
struct xBLE_LOC_BEACON
{
	unsigned portBASE_TYPE			uxBleStdLocBcnCnt;					/* Number of beacons in the table which are standard beacons (either AltBeacon or iBeacon). */
	unsigned portBASE_TYPE			uxBleSwissPhoneLocBcnCnt;			/* Number of beacons in the table which are UUID-filtered SwissPhone beacons. */
	struct xBLE_LOC_BEACON_ENTRY	xBleLocBcn[ MAX_LOC_BCN_ENTRIES ];
};

/* Structure of one BLE beacon entry in the TL beacon dictionary. */
struct xBLE_TL_BEACON
{
	unsigned char					ucFID[ 8 ] ;						/* Foreign ID (IMEI). */
	unsigned short					usRxTimeStamp;						/* Time stamp of the beacon reception (in RTC seconds). */
	unsigned char					ucFDesc;							/* Beacon descriptor (ALERT / SOS / EVAC). */
	signed char						cRSSI;								/* RSSI of received signal. */
	unsigned char					ucLongRange;						/* RF interface over which the beacon has been received (bit 0: SR, bit 1: LR; TL230 only). */
	unsigned long					ulDFMap;							/* DFMap parameter describing the following datafields. 0 if the entry is not used. */
	unsigned char					ucVarBcnData[ BLE_BCN_VAR_FLD_LEN ];/* Field containing all beacon data specified by ulDFMap. */
};

/* Structure for the flags indicating the received special purpose beacons. */
struct xSPECIAL_BCN_RCVD
{
	bool							bPrivate 	: 1;					/* The device has seen a no abnormal position detection beacon. */
	bool							bDanger  	: 1;                    /* The device has seen a danger zone beacon. */
	bool							bNoAbn   	: 1;                    /* The device has seen a private zone beacon. */
	bool							bImmobility : 1;                    /* The device has seen a immobility zone beacon. */
};

/* Commands from the periodic timer task to the advertiser handler task. */
enum xADREPORT_CMD
{
	AD_REPORT_CHK														/* Check if advertiser are due for being reported. */
};
/*-----------------------------------------------------------*/

/* Public function prototypes. */
extern void vBleAdReportInit( UBaseType_t uxPriority );

/* Delete the BLE localiser beacons store contents. */
extern void vBleDeleteLocBcnStore( void );

/* Calculate the length of the variable TL beacon field, the contents of which are specified in the corresponding FDFMAP. */
extern unsigned portBASE_TYPE uxCalculateVarBcnFieldLen( unsigned long ulDFMap, unsigned portBASE_TYPE uxDFMapFieldIdx,
												         unsigned portBASE_TYPE *puxNumFields );

/* Store the contents of the BLE location beacon dictionary in the location beacon record array. */
extern void vStoreBLEAdvertisers( void );
/*-----------------------------------------------------------*/

/* Global variables. */
/* Timer handle for the advertisement RSSI filter timer. */
extern TimerHandle_t 						xBleFilterTimer;

/* Configuration for the RSSI filter. */
extern unsigned portBASE_TYPE				uxRssiFilterMethod;
extern unsigned portBASE_TYPE				uxRssiFilterWindow;

/* Foreign alert type. */	
extern volatile unsigned portBASE_TYPE		uxForeignAlertType;

/* Table of location beacons. */	
extern volatile struct xBLE_LOC_BEACON		xBleLocBcnGSM;
extern volatile struct xBLE_LOC_BEACON		xBleLocBcnBLE;

/* Table of TL beacons. */	
extern volatile struct xBLE_TL_BEACON		xBleTLBcn[ MAX_TL_BCN_ENTRIES ];
extern volatile unsigned portBASE_TYPE		uxBleTLBcnCnt;

/* Flags indicating the received special purpose beacons. */
extern volatile struct xSPECIAL_BCN_RCVD	xSpecialBcnRcvd;

/* ROM-table with TL beacon data fields lengths. */
extern const unsigned portBASE_TYPE 		uxTLBcnDataFieldsLen[];

/* Flag, set when XSWITCH has been received. */
extern volatile bool 						bBleXSwitchObserved;

/* Timer handle for the timer indicating when running that the module is in a danger zone. */
extern TimerHandle_t 						xInDangerZoneTimer;

/* Storage for BLE location beacons if recording is required. */ 
extern struct xBLE_LOC_BEACON 				xBleLocBeaconStore[ POS_STORE_LEN ];
/*-----------------------------------------------------------*/


#endif