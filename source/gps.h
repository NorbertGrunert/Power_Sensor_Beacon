/*
 * Tracker Firmware
 *
 * GPS header file
 *
 */ 
#ifndef GPS_H
#define GPS_H
/*-----------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "queue.h"

#include "main.h"
#include "gsm.h"
/*-----------------------------------------------------------*/


/* u-blox ubx protocol definitions. */
/* UBX message classes */
#define UBX_NAV					0x01
#define UBX_ACK                 0x05
#define UBX_CFG					0x06
#define UBX_MON                 0x0a
#define UBX_NMEA				0xf0		/* NMEA Messages. */
#define UBX_NMEA_PROP       	0xf1		/* NMEA PUBX Messages Proprietary Messages. */
                            
/* UBX message IDs */
#define UBX_ACK_ACK             0x01		/* Message Acknowledged. */
#define UBX_ACK_NAK             0x02		/* Message Not-Acknowledged. */
                    
#define UBX_CFG_RST				0x04		/* Reset. */
#define UBX_CFG_CFG				0x09		/* Clear, save and load configurations. */
#define UBX_CFG_GNSS            0x3e		/* GNSS system configuration. */
#define UBX_CFG_ITFM			0x39		/* Get/Set Jamming/Interference Monitor... */
#define UBX_CFG_MSG             0x01		/* Set message rate. */
#define UBX_CFG_NMEA			0x17		/* Extended NMEA protocol configuration V1. */
#define UBX_CFG_PMS				0x86		/* Power Mode Setup. */
                            
#define UBX_MON_VER             0x04		/* Poll Receiver/Software Version. */
#define UBX_MON_GNSS            0x28		/* Information message major GNSS... */
#define UBX_MON_HW2		       	0x0b		/* Extended Hardware Status. */
#define UBX_MON_HW			    0x09		/* Hardware Status. */
                            
#define UBX_NAV_POSLLH			0x02		/* Geodetic Position Solution. */

#define UBX_NMEA_DTM	       	0x0A		/* Datum Reference. */
#define UBX_NMEA_GBQ	       	0x44		/* Poll a standard message (if the current Talker ID is GB). */
#define UBX_NMEA_GBS	       	0x09		/* GNSS Satellite Fault Detection. */
#define UBX_NMEA_GGA	       	0x00		/* Global positioning system fix data. */
#define UBX_NMEA_GLL	       	0x01		/* Latitude and longitude, with time of position fix and status. */
#define UBX_NMEA_GLQ	       	0x43		/* Poll a standard message (if the current Talker ID is GL). */
#define UBX_NMEA_GNQ	       	0x42		/* Poll a standard message (if the current Talker ID is GN). */
#define UBX_NMEA_GNS	       	0x0D		/* GNSS fix data. */
#define UBX_NMEA_GPQ	       	0x40		/* Poll a standard message (if the current Talker ID is GP). */
#define UBX_NMEA_GRS	       	0x06		/* GNSS Range Residuals. */
#define UBX_NMEA_GSA	       	0x02		/* GNSS DOP and Active Satellites. */
#define UBX_NMEA_GST	       	0x07		/* GNSS Pseudo Range Error Statistics. */
#define UBX_NMEA_GSV	       	0x03		/* GNSS Satellites in View. */
#define UBX_NMEA_RMC	       	0x04		/* Recommended Minimum data. */
#define UBX_NMEA_TXT	       	0x41		/* Text Transmission. */
#define UBX_NMEA_VLW	       	0x0F		/* Dual ground/water distance. */
#define UBX_NMEA_VTG	       	0x05		/* Course over ground and Ground speed. */
#define UBX_NMEA_ZDA	       	0x08		/* Time and Date. */

#define UBX_NMEA_PROP_CONFIG 	0x41		/* Set Protocols and Baudrate. */
#define UBX_NMEA_PROP_POSITION 	0x00		/* Lat/Long Position Data. */
#define UBX_NMEA_PROP_RATE 		0x40		/* Set NMEA message output rate. */
#define UBX_NMEA_PROP_SVSTATUS 	0x03		/* Satellite Status. */
#define UBX_NMEA_PROP_TIME 		0x04		/* Time of Day and Clock Information. */
/*-----------------------------------------------------------*/

/* GPS report types. */
#define RPT_NMEA_GNGSA					( 0x01 )
#define RPT_NMEA_GPGSV          		( 0x02 )
#define RPT_NMEA_GAGSV          		( 0x04 )
#define RPT_NMEA_GNGGA          		( 0x08 )
#define RPT_NMEA_GNVTG          		( 0x10 )
#define RPT_UBX_NAV_POSLLH      		( 0x20 )
#define RPT_UBX_MON_HW          		( 0x40 )
#define RPT_UBX_MON_HW2         		( 0x80 )
		
#define gpsCMD_QUEUE_SIZE				( 8 )
#define gpsCMD_BLOCKTIME				( ( TickType_t ) ( 5 * portTICKS_PER_SEC ) )	/* wait max. 5 second for the response indication queue to become available */
		
#define GPS_POLL_INTERVAL				( ( TickType_t )( 2 * portTICKS_PER_100MSEC ) )
#define GPS_ANFILE_TO					( ( TickType_t )( 2 * portTICKS_PER_SEC ) )

/* Delay to add in between two MGA messages sent to the GPS receiver as per ublox specification. Delay estimated (~120 messages). */
#define GPS_UBX_MSG_DLY					( ( TickType_t )( 1 * portTICKS_PER_10MSEC ) )

/* Delay to add after GNSS configuration message as per ublox specification. */
#define GPS_CFG_GNSS_DLY				( ( TickType_t )( 5 * portTICKS_PER_100MSEC ) )

/* Maximum age of a GPS fix for the fix to be valid. */
#define GPS_MAX_FIX_AGE					( ( TickType_t )( 10 * portTICKS_PER_MIN ) )

#define MAX_UBX_MESSAGE_LEN				( 256 )

/* Default GPS fix acceptance criteria. */
#define GPS_MIN_QUALITY					( 150 )		/* Minimum GPS quality for a position fix. Value is 10 times the HDOP limit. */
#define GPS_HPREC_IMPROVMENT_LIMIT		(  50 )		/* GPS horizontal precision improvement between two consecutive reports for accepting the fix (in cm). */

/* Typedef for the GPS debug SVN data entry. */
#define	GPSDEB_SVN						( 40 )		/* Maximum number of GPS and GLONASS satellites to be recorded. */
struct xGSV_ENTRY	
{	
	unsigned portBASE_TYPE		uxSvn;		/* Satellite identifier (SVN). */
	unsigned portBASE_TYPE		uxCn0;		/* Satellite signal strength CN0. */
};

/* ID for a GNSS constellation. */
enum xGNSS_CONSTELLATION_ID {
	GPS					= 0,				
	GLONASS				= 1,				
	GALILEO				= 2						
};

enum xFIX_RPT {
	FIXRPT_DIGEST		= 0,				/* GPS fix report digest only. */
	FIXRPT_NMEA			= 1,				/* NMEA messages and fix report digest. */
	FIXRPT_NMEA_UBX		= 2,				/* NMEA and UBX raw messages and fix report digest. */	
	FIXRPT_ALL			= 3					/* All messages received from the GPS receiver as well and fix report digest. */	
};

/* Definition of a structure field used for parsing and classifying NMEA sentences. */
struct xNMEA_ID 
{
	char				*pcNmeaStrg;
	void				( *pvNmeaParser )( signed char *pcNmeaStrg );
};

/* Typedef for a GPS position entry in the position store. */
struct xGPS_POS_ENTRY	
{	
	signed char 			cPosLAT  [ LEN_GSM_LAT  	+ 1	];	/* latitude */
	signed char 			cPosLON  [ LEN_GSM_LON  	+ 1 ];	/* longitude */
	signed char 			cPosALT  [ LEN_GSM_ALT  	+ 1	];	/* altitude (4 digits + sign) */
	signed char 			cPosHDG  [ LEN_GSM_HDG  	+ 1	];	/* heading */
	signed char 			cPosFIX  [ LEN_GSM_FIX  	+ 1	];	/* GPS fix type */
	signed char 			cPosHDOP [ LEN_GSM_HDOP 	+ 1	];	/* Horizontal Dilution of Precision */
	signed char 			cPosLOCM [ LEN_GSM_LOCM 	+ 1	];	/* Localization method */
	signed char 			cPosSV   [ LEN_GSM_SAT  	+ 1	];	/* Satellites in view and used */
	signed char 			cPosACCH [ LEN_GSM_ACCHM	+ 1	];	/* GPS fix horizontal accuracy */
	signed char 			cPosACCV [ LEN_GSM_ACCVM	+ 1	];	/* GPS fix vertical accuracy */
	bool					bPosValidEntry;
};

/* Commands from the CTRL or GSM task to the GPS task. */
enum xGPS_CMD
{
	GPS_START,							/* Start localisation via GPS. */
	GPS_STOP,							/* Stop localisation via GPS. */ 
	GPS_ANDATA_UPDATED					/* The GPS assitance data has been updated. */
};
/*-----------------------------------------------------------*/

/* Global variables. */

/* GPS command queue handle. */
extern QueueHandle_t				xGpsCmdQueue; 	

/* Length of the AssistNow binary data file received from the main controller. */
extern uint32_t						uiANDataLength;

/* GPS and GLONASS debug data. */
extern struct xGSV_ENTRY			xGpsSvnData[ GPSDEB_SVN ];				

/* Array to store GPS positions if recording is required. */
extern struct xGPS_POS_ENTRY		xGpsPosStore[ POS_STORE_LEN ];
/*-----------------------------------------------------------*/

/* GPS init. */
extern void 						vGpsInit( UBaseType_t uxPriority );
			
/* Store the UBX assistance data in the data buffer. */
extern bool 						bStoreANData( uint8_t uiDataByte, bool bInit );
			
/* Process a received NMEA sentence. */
extern ret_code_t					xProcessNmeaSentence( signed char *cNmeaSentenceBuffer );
			
/* Process a received UBX message. */
extern ret_code_t 					xProcessUbxMessage( uint8_t *puiUbxMessage, uint32_t uiUbxMessageLen );
			
/* Function called once a complete update of the GPS data has been received from the GPS module. */
extern void 						vPushGpsFixSynopsis( void );
			
/* Read the GPS position data from the GPS module and return true if the GPS receiver has a good fix. */
extern bool 						bHasGPSGoodFix( void );
			
/* Invalidate all GPS data fields to be sent to the server. */
extern void 						vInvalidateGpsData( void );
			
/* Trace the failure reason for a GPS fix fail. */
extern void 						vTraceGpsFixFail( void );

/* Store the current GPS position in the GPS position record array. */
extern void 						vStoreGPSPosition( void );

/* Array of member offsets into the xGPS_POS_ENTRY structure. */
extern const unsigned portBASE_TYPE	cPosFieldOffset[];

/*-----------------------------------------------------------*/
#endif