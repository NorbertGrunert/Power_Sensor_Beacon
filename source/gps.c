/*
 * Tracker Firmware
 *
 * GPS handler
 *
 * This task interfaces with the u-blox ZOE GPS module. The physical interface is an
 * I2C bus (here called TWI).
 * The communication is based on the ned SDK' TWI manager (nrf_twi_mngr).
 *
 * Settings in sdk_config.h:
 * NRFX_TWIM_ENABLED - nrfx_twim - TWIM peripheral driver
 * 		#define NRFX_TWIM_ENABLED 1
 * NRFX_TWI_ENABLED - nrfx_twi - TWI peripheral driver
 *		#define NRFX_TWI_ENABLED 1
 * TWI_ENABLED - nrf_drv_twi - TWI/TWIM peripheral driver - legacy layer
 *		#define TWI_ENABLED 1
 * TWI0_ENABLED - Enable TWI0 instance
 *		#define TWI0_ENABLED 1
 * TWI0_USE_EASY_DMA  - Use EasyDMA (if present)
 *		#define TWI0_USE_EASY_DMA 0				(1 ?)
 * NRF_TWI_MNGR_ENABLED  - nrf_twi_mngr - TWI transaction manager
 *		#define NRF_TWI_MNGR_ENABLED 1
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "tracker.h"

/* nRF SDK include files */
#define NRF_LOG_MODULE_NAME 		GPS
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_log_backend_flash.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "string.h"

/* Device specific include files. */
#include "custom_board.h"
#include "drv_gps.h"
#include "drv_uart.h"

#include "ble_adreport.h"
#include "ble_main.h"
#include "config.h"
#include "gps.h"
#include "main.h"
#include "trace.h"
#include "tracemsg.h"
#include "rtc.h"
#include "utils.h"
#include "version.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */

/* GPS task init. */
void vGpsInit( UBaseType_t uxPriority );

/* Store the UBX assistance data in the data buffer. */
bool bStoreANData( uint8_t uiDataByte, bool bInit );

/* GPS module configuration. */
static bool bConfigureGPS( void );

/* Receive the binary AssistNow data file and store it. */
static bool bReceiveAssistNowData();

/* Verify a UBX message checksum. */
static bool bVerifyUbxMessage( uint8_t **ppuiUbxMessage );

/* Locate a data field by number in a comma-separated string and copy the field to a specified destination. */
static unsigned portBASE_TYPE xCopyFieldFromStrg( signed char *pcSrc, unsigned portBASE_TYPE uxFieldNum, unsigned portBASE_TYPE uxOffset, signed char *pcDest, unsigned portBASE_TYPE uxMaxCnt );

/* Parse NMEA sentences. Extract the relevant information. */
static void vParseGNGGA( signed char *pcNmeaStrg );
static void vParseGPGSV( signed char *pcNmeaStrg );
static void vParseGLGSV( signed char *pcNmeaStrg );
static void vParseGAGSV( signed char *pcNmeaStrg );
static void vParseG_GSV( signed char *pcNmeaStrg, enum xGNSS_CONSTELLATION_ID xConstellationId );
static void vParseGNGSA( signed char *pcNmeaStrg );
static void vParseGNVTG( signed char *pcNmeaStrg );

/* Parse UBX messages. Extract the relevant information. */
static void vGetPrecision( uint8_t *uiDataPtr, signed char *pcDestM, unsigned short *usDestCM );
static void vParseUbxNavPos( uint8_t *puiUbxMessage );
static void vParseUbxMonHw( uint8_t *puiUbxMessage );
static void vParseUbxMonHw2( uint8_t *puiUbxMessage );

/* Process a received NMEA sentence. */
ret_code_t	xProcessNmeaSentence( signed char *cNmeaSentenceBuffer );

/* Process a received UBX message. */
ret_code_t 	xProcessUbxMessage( uint8_t *puiUbxMessage, uint32_t uiUbxMessageLen );

/* Function called once a complete update of the GPS data has been received from the GPS module. */
void vPushGpsFixSynopsis( void );

/* Trace the failure reason for a GPS fix fail. */
void vTraceGpsFixFail( void );

/* Store the current GPS position in the GPS position record array. */
void vStoreGPSPosition( void );

/* GPS task function. */
void vGpsTask( void * pvParameter );
/*-----------------------------------------------------------*/

/* Local static variables. */

/* Horizontal and vertical precision in centimeters. */
unsigned short 				usGps_ACCHCM;		/* GPS fix horizontal accuracy (cm) */
unsigned short 				usGps_ACCVCM;		/* GPS fix vertical accuracy (cm) */

/* Received UBX message. */
static uint8_t				cUbxMessage[ MAX_UBX_MESSAGE_LEN ];

/* UBX message length. */
static uint32_t				uiUbxMessageLen;

/* Structures required for identifying and parsintg NMEA sentences. */
char pcAt_GpsGNGga[] 		= "$GNGGA,";		
char pcAt_GpsGNVtg[] 		= "$GNVTG,";		
char pcAt_GpsGNGsa[] 		= "$GNGSA,";
char pcAt_GpsGPGsv[]		= "$GPGSV,";	
char pcAt_GpsGLGsv[]		= "$GLGSV,";	
char pcAt_GpsGAGsv[]		= "$GAGSV,";	


static struct xNMEA_ID xNmeaId[] = 
{
	 { pcAt_GpsGNGga, 	 vParseGNGGA		},
	 { pcAt_GpsGNVtg, 	 vParseGNVTG		},
	 { pcAt_GpsGNGsa, 	 vParseGNGSA		},
	 { pcAt_GpsGPGsv, 	 vParseGPGSV		},
	 { pcAt_GpsGLGsv, 	 vParseGLGSV		},
	 { pcAt_GpsGAGsv, 	 vParseGAGSV		}
};

/* GPS and GLONASS debug data. */
struct xGSV_ENTRY	xGpsSvnData[GPSDEB_SVN ];

uint32_t					uiGPSReportStatus;

/* Timestamp of the last GPS fix. */
TickType_t						xGpsFixTimeStamp;
/*-----------------------------------------------------------*/

/* Global variables. */

/* GPS command queue handle. */
QueueHandle_t					xGpsCmdQueue; 	

/* Length of the AssistNow binary data file received from the ublox server. */
uint32_t						uiANDataLength;

/* AssistNow data. */
uint8_t							uiAssistNowData[ 10000 ];

/* Length of the UBX message in ucUbxMessage[]. */
uint32_t						uiUbxMsgLength;

/* Write pointer into uiAssistNowData[]. */
uint8_t							*puiUbxMsgWritePtr;

/* Read pointer into uiAssistNowData[]. */
uint8_t							*puiUbxMsgReadPtr;

/* Checksum of the message received from the ublox server. */
uint16_t						uiMsgChkSum;

/* GPS fix reporting level while the GPS receiver is attempting a fix. */
enum xFIX_RPT					xGpsFixReportLevel;

/* Previous horizontal accuracy to calculate imrpovement. */
unsigned short					usPrevHorizontalAccuracy;

/* Total number of visible satellites. */
unsigned portBASE_TYPE			uxGnssVisible[ 3 ];

/* Array to store GPS positions if recording is required. */
struct xGPS_POS_ENTRY			xGpsPosStore[ POS_STORE_LEN ];

/* Array of member offsets into the xGPS_POS_ENTRY structure. */
const unsigned portBASE_TYPE	cPosFieldOffset[] = 
{
	offsetof( struct xGPS_POS_ENTRY, cPosLAT  ),
	offsetof( struct xGPS_POS_ENTRY, cPosLON  ),
	offsetof( struct xGPS_POS_ENTRY, cPosALT  ),
	offsetof( struct xGPS_POS_ENTRY, cPosHDG  ),
	offsetof( struct xGPS_POS_ENTRY, cPosFIX  ),
	offsetof( struct xGPS_POS_ENTRY, cPosHDOP ),
	offsetof( struct xGPS_POS_ENTRY, cPosLOCM ),
	offsetof( struct xGPS_POS_ENTRY, cPosSV   ),
	offsetof( struct xGPS_POS_ENTRY, cPosACCH ),
	offsetof( struct xGPS_POS_ENTRY, cPosACCV )
};
/*-----------------------------------------------------------*/


/*
 * GPS task init
 */
void vGpsInit( UBaseType_t uxPriority )
{
	/* Preset GPS fix report level. */
	xGpsFixReportLevel = FIXRPT_DIGEST;
	
	/* Create a queue to the GPS task for AT commands. */
	xGpsCmdQueue = xQueueCreate( gpsCMD_QUEUE_SIZE, ( unsigned portBASE_TYPE ) sizeof( enum xGPS_CMD ) );

    /* Create task for LED0 blinking with priority set to 2 */
    ( void )xTaskCreate( vGpsTask, "GPS", gpsSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );

	xGpsFixTimeStamp = 0;

	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* GPS module configuration. */
static const char cCfgRmc[]  		= { UBX_CFG, UBX_CFG_MSG, UBX_NMEA, UBX_NMEA_RMC,   0 };
static const char cCfgGll[]  		= { UBX_CFG, UBX_CFG_MSG, UBX_NMEA, UBX_NMEA_GLL,   0 };
static const char cCfgGrs[]  		= { UBX_CFG, UBX_CFG_MSG, UBX_NMEA, UBX_NMEA_GRS,   0 };
static const char cCfgGst[]  		= { UBX_CFG, UBX_CFG_MSG, UBX_NMEA, UBX_NMEA_GST,   0 };
static const char cCfgZda[]  		= { UBX_CFG, UBX_CFG_MSG, UBX_NMEA, UBX_NMEA_ZDA,   0 };
static const char cCfgGbs[]  		= { UBX_CFG, UBX_CFG_MSG, UBX_NMEA, UBX_NMEA_GBS,   0 };
static const char cCfgDtm[]  		= { UBX_CFG, UBX_CFG_MSG, UBX_NMEA, UBX_NMEA_DTM,   0 };

static const char cCfgGsv[]  		= { UBX_CFG, UBX_CFG_MSG, UBX_NMEA, UBX_NMEA_GSV,   2 };
static const char cCfgGga[]  		= { UBX_CFG, UBX_CFG_MSG, UBX_NMEA, UBX_NMEA_GGA,   2 };
static const char cCfgVtg[]  		= { UBX_CFG, UBX_CFG_MSG, UBX_NMEA, UBX_NMEA_VTG,   2 };
static const char cCfgGsa[]  		= { UBX_CFG, UBX_CFG_MSG, UBX_NMEA, UBX_NMEA_GSA,   2 };
static const char cCfgMonHw2[]  	= { UBX_CFG, UBX_CFG_MSG, UBX_MON,  UBX_MON_HW2,    2 };
static const char cCfgMonHw[]  		= { UBX_CFG, UBX_CFG_MSG, UBX_MON,  UBX_MON_HW,     2 };
static const char cCfgNavPosLLH[]	= { UBX_CFG, UBX_CFG_MSG, UBX_NAV,  UBX_NAV_POSLLH, 2 };

static const char cCfgCfg[]			= { UBX_CFG, UBX_CFG_CFG,  0x00, 0x00, 0x00, 0x00,	/* clearMask */
															   0x1c, 0x1f, 0x00, 0x00,	/* saveMask (infMsg, rxmConf, senConf, rinvConf, antConf, logConf, ftsConf) */
															   0x00, 0x00, 0x00, 0x00,  /* loadMask. */
															   0x01 };            		/* devBBR: save to battery-backed RAM */
static const char cCfgRst[]			= { UBX_CFG, UBX_CFG_RST,  0x00, 0x00,				/* hotstart */
															   0x00,					/* hardware reset. */
															   0x00 };  				/* reserved. */
static const char cCfgPms[]			= { UBX_CFG, UBX_CFG_PMS,  0x00, 					/* Message version (0x00 for this version) */
															   0x00, 					/* Power setup value, 0x00 -> Full power */
															   0x00, 0x00,              /* Position update period and search period. Only valid when powerSetupValueset to Interval, otherwise must be set to '0'. */
															   0x00, 0x00,              /* Duration of the ON phase, must be smaller than the period. Only valid when powerSetupValue set to Interval, otherwise must be set to '0'. */
															   0x00, 0x00 };            /* Reserved */
															   
static const char cCfgGnss[] 		= { UBX_CFG, UBX_CFG_GNSS, 0x00, 					/* 0 - msgVer - Message version (=0 for this version). */
 															   0x20,                    /* 1 - numTrkChHw - Number of tracking channels available in hardware (read only). */
 															   0x20,                    /* 2 - numTrkChUse - (Read only in protocol versions greater than 23) Number of tracking channels to use. Must be > 0, <= numTrkChHw. */
															   0x07,                    /* 3 - numConfigBlocks - Number of configuration blocks following. */
															   
															   0x00, 					/* 4 + 8*0 - gnssId - GPS System identifier (see Satellite Numbering). */
															   0x08,                    /* 5 + 8*0 - resTrkCh - (Read only in protocol versions greater than 23) Number of reserved (minimum) tracking channels for this system. */
															   0x10,                    /* 6 + 8*0 - maxTrkCh - (Read only in protocol versions greater than 23) Maximum number of tracking channels used for this system. */
															   0x00,                    /* 7 + 8*0 - reserved1 - Reserved */
															   0x01, 0x00, 0x01, 0x01,  /* 8 + 8*0 - flags - bitfield of flags. Enabled. */
 															   
															   0x01, 					/* 4 + 8*0 - gnssId - SBAS System identifier (see Satellite Numbering). */
 															   0x01,                    /* 5 + 8*0 - resTrkCh - (Read only in protocol versions greater than 23) Number of reserved (minimum) tracking channels for this system. */
 															   0x03,                    /* 6 + 8*0 - maxTrkCh - (Read only in protocol versions greater than 23) Maximum number of tracking channels used for this system. */
 															   0x00,                    /* 7 + 8*0 - reserved1 - Reserved */
															   0x00, 0x00, 0x01, 0x01,  /* 8 + 8*0 - flags - bitfield of flags. Disabled. */
 															   
															   0x02, 					/* 4 + 8*1 - gnssId - Galileo System identifier (see Satellite Numbering). */
 															   0x04,                    /* 5 + 8*1 - resTrkCh - (Read only in protocol versions greater than 23) Number of reserved (minimum) tracking channels for this system. */
 															   0x08,                    /* 6 + 8*1 - maxTrkCh - (Read only in protocol versions greater than 23) Maximum number of tracking channels used for this system. */
 															   0x00,                    /* 7 + 8*1 - reserved1 - Reserved */
															   0x01, 0x00, 0x01, 0x01,  /* 8 + 8*1 - flags - bitfield of flags. Enabled. */

															   0x03, 					/* 4 + 8*1 - gnssId - Beidou System identifier (see Satellite Numbering). */
															   0x08,                    /* 5 + 8*1 - resTrkCh - (Read only in protocol versions greater than 23) Number of reserved (minimum) tracking channels for this system. */
															   0x10,                    /* 6 + 8*1 - maxTrkCh - (Read only in protocol versions greater than 23) Maximum number of tracking channels used for this system. */
															   0x00,                    /* 7 + 8*1 - reserved1 - Reserved */
															   0x00, 0x00, 0x01, 0x01,  /* 8 + 8*1 - flags - bitfield of flags. Disabled. */

															   0x04, 					/* 4 + 8*1 - gnssId - IMES System identifier (see Satellite Numbering). */
															   0x00,                    /* 5 + 8*1 - resTrkCh - (Read only in protocol versions greater than 23) Number of reserved (minimum) tracking channels for this system. */
															   0x08,                    /* 6 + 8*1 - maxTrkCh - (Read only in protocol versions greater than 23) Maximum number of tracking channels used for this system. */
															   0x00,                    /* 7 + 8*1 - reserved1 - Reserved */
															   0x00, 0x00, 0x01, 0x03,  /* 8 + 8*1 - flags - bitfield of flags. Disabled. */

															   0x05, 					/* 4 + 8*1 - gnssId - QZSS System identifier (see Satellite Numbering). */
															   0x00,                    /* 5 + 8*1 - resTrkCh - (Read only in protocol versions greater than 23) Number of reserved (minimum) tracking channels for this system. */
															   0x03,                    /* 6 + 8*1 - maxTrkCh - (Read only in protocol versions greater than 23) Maximum number of tracking channels used for this system. */
															   0x00,                    /* 7 + 8*1 - reserved1 - Reserved */
															   0x00, 0x00, 0x00, 0x05,  /* 8 + 8*1 - flags - bitfield of flags. Disabled. */
 															   
															   0x06, 					/* 4 + 8*2 - gnssId - GLONASS System identifier (see Satellite Numbering). */
															   0x08,                    /* 5 + 8*2 - resTrkCh - (Read only in protocol versions greater than 23) Number of reserved (minimum) tracking channels for this system. */
															   0x0e,                    /* 6 + 8*2 - maxTrkCh - (Read only in protocol versions greater than 23) Maximum number of tracking channels used for this system. */
 															   0x00,                    /* 7 + 8*2 - reserved1 - Reserved */
															   0x01, 0x00, 0x01, 0x01 };/* 8 + 8*2 - flags - bitfield of flags. Enabled. */
static const char cCfgNmea[]		= { UBX_CFG, UBX_CFG_NMEA, 0x00, 								/* 0 - filter - filter 																														*/
															   0x41,                                /* 1 - nmeaVersion - 0x41: NMEA version 4.10                                                                                                */
															   0x00,                                /* 2 - numSV - Maximum Number of SVs to report per TalkerId (0: unlimited)                                                                  */
															   0x02,                                /* 3 - flags - flags (0x02 = enable considering mode)                                                                                       */
															   0x00, 0x00, 0x00, 0x00,              /* 4 - gnssToFilter - Filters out satellites based on their GNSS.                                                                           */
															   0x01,                                /* 8 - svNumbering - Configures the display of satellites that do not have an NMEA-defined value (1: Extended - Use proprietary numbering)	*/
															   0x00,                                /* 9 - mainTalkerId (0: Main Talker ID is not overridden)                                                                                   */
															   0x00,                                /* 10 - gsvTalkerId - By default the Talker ID for GSV messages is GNSS specific (as defined by NMEA).                                      */
															   0x01,                                /* 11 - version - Message version (set to 1 for this version)                                                                               */
															   0x00, 0x00,                          /* 12 - bdsTalkerId - Sets the two characters that should be used for the BeiDou Talker ID                                                	*/
															   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };/* 14 - reserved	 																														*/	
static const char cCfgItfm[]		= { UBX_CFG, UBX_CFG_ITFM, 0xf3, 0xac, 0x62, 0xad, 	/* 0 - config - interference config word (enabled, bbThreshold 3dB, cwThreshold 15dB)						*/
															   0x1e, 0x53, 0x00, 0x00 };/* 4 - config2 - extra settings for jamming/interference (generalBits 0x31e, antSetting passive, enabled2). */


static bool bConfigureGPS( void )
{
	uint32_t		uiUbxMsgLen;
	uint8_t			uiUbxMsg[ MAX_UBX_MESSAGE_LEN ];
	
	/* Query GNSS configuration and configure if incorrect. */
	if ( xGpsReadRegister( UBX_CFG, UBX_CFG_GNSS, uiUbxMsg, &uiUbxMsgLen ) != NRF_SUCCESS )
	{
		if ( xGpsReadRegister( UBX_CFG, UBX_CFG_GNSS, uiUbxMsg, &uiUbxMsgLen ) != NRF_SUCCESS )
		{
			/* No response from GPS after two attempts. Something is wrong here, get back with an error flag. */
			return false;
		}
	}

	/* Check if the response was really for the GNSS query. */
	if (   ( uiUbxMsgLen >= ( sizeof( cCfgGnss ) + 2 ) )
		&& ( uiUbxMsg[ 03 ] == UBX_CFG_GNSS ) )
	{
		uint32_t	uiGnssDataIdx;
		bool		bIdentical;

		/* Simply compare the response data with the configuration data. */
		bIdentical = true;
		for ( uiGnssDataIdx = 0; uiGnssDataIdx < ( sizeof( cCfgGnss ) - 2 ); uiGnssDataIdx++ )
		{
			if ( uiUbxMsg[ uiGnssDataIdx + 6 ] != cCfgGnss[ uiGnssDataIdx + 2 ] )
			{
				bIdentical = false;
			}
		}

		if ( !bIdentical )
		{
			/* The current GNSS configuration differs from the target one. Set GNSS configuration. */
			( void )xGpsWriteRegister( cCfgGnss, sizeof ( cCfgGnss ) );
			vTaskDelay( GPS_CFG_GNSS_DLY );
			( void )xGpsWriteRegister( cCfgCfg, sizeof ( cCfgCfg ) );
			( void )xGpsWriteRegister( cCfgRst, sizeof ( cCfgRst ) );
			vTaskDelay( GPS_CFG_GNSS_DLY );
		}
	}

	/* Set reporting OFF. */
	( void )xGpsWriteRegister( cCfgRmc, sizeof ( cCfgRmc ) );
	( void )xGpsWriteRegister( cCfgGll, sizeof ( cCfgGll ) );
	( void )xGpsWriteRegister( cCfgGrs, sizeof ( cCfgGrs ) );
	( void )xGpsWriteRegister( cCfgGst, sizeof ( cCfgGst ) );
	( void )xGpsWriteRegister( cCfgZda, sizeof ( cCfgZda ) );
	( void )xGpsWriteRegister( cCfgGbs, sizeof ( cCfgGbs ) );
	( void )xGpsWriteRegister( cCfgDtm, sizeof ( cCfgDtm ) );

	/* Set reporting ON. */
	( void )xGpsWriteRegister( cCfgGsv, sizeof ( cCfgGsv ) );
	( void )xGpsWriteRegister( cCfgGga, sizeof ( cCfgGga ) );
	( void )xGpsWriteRegister( cCfgVtg, sizeof ( cCfgVtg ) );
	( void )xGpsWriteRegister( cCfgGsa, sizeof ( cCfgGsa ) );
	( void )xGpsWriteRegister( cCfgMonHw2, sizeof ( cCfgMonHw2 ) );
	( void )xGpsWriteRegister( cCfgMonHw, sizeof ( cCfgMonHw ) );
	( void )xGpsWriteRegister( cCfgNavPosLLH, sizeof ( cCfgNavPosLLH ) );
	
	/* Configure NMEA output. */
	( void )xGpsWriteRegister( cCfgNmea, sizeof ( cCfgNmea ) );

	/* Configure interference monitor. */
	( void )xGpsWriteRegister( cCfgItfm, sizeof ( cCfgItfm ) );
	
	/* Configure power mode. */
	( void )xGpsWriteRegister( cCfgPms, sizeof ( cCfgPms ) );	
	
	/* Save configuration to battery-backed RAM. */
	( void )xGpsWriteRegister( cCfgCfg, sizeof ( cCfgCfg ) );	

	return true;
}
/*-----------------------------------------------------------*/


/* Store the UBX assistance data in the data buffer. At the same time, parse it and verify the checksum of each individual
   UBX message.
*/
bool bStoreANData( uint8_t uiDataByte, bool bInit )
{
    ret_code_t				xErrCode;
	bool					bSuccess;
	uint16_t				uiLocalChkSum;
	
	if ( bInit )
	{
		/* Initialise read and write pointers to the message buffer start. */
		puiUbxMsgWritePtr = uiAssistNowData;
		puiUbxMsgReadPtr = uiAssistNowData;
		
		/* Set the message length to maximum so that the procedure does not abort while receiving the header. */
		uiUbxMsgLength = MSG_BUFFER_LEN - 5;
		
		uiANDataLength = 0;
		
		return true;
	}

	/* Put the data byte into the buffer. */
	*puiUbxMsgWritePtr = uiDataByte;
	
	/* Check that the message starts with SYNC_CHAR_1 and SYNC_CHAR_2 characters. */
	if ( ( puiUbxMsgWritePtr - puiUbxMsgReadPtr == 0 ) && ( uiDataByte != SYNC_CHAR_1 ) )
	{
		NRF_LOG_ERROR( "%i Error when parsing UBX message. Sync character 1 not present.", ulReadRTC() );
		NRF_LOG_FLUSH();
		return false;
	}
	if ( ( puiUbxMsgWritePtr - puiUbxMsgReadPtr == 1 ) && ( uiDataByte != SYNC_CHAR_2 ) )
	{
		NRF_LOG_ERROR( "%i Error when parsing UBX message. Sync character 2 not present.", ulReadRTC() );
		NRF_LOG_FLUSH();
		return false;
	}	
	
	/* Get the message length. */
	if ( puiUbxMsgWritePtr - puiUbxMsgReadPtr == 4 )
	{
		uiUbxMsgLength = uiDataByte;
	}
	if ( puiUbxMsgWritePtr - puiUbxMsgReadPtr == 5 )
	{
		uiUbxMsgLength += uiDataByte << 8;
		
		if ( uiUbxMsgLength > MSG_BUFFER_LEN )
		{
			NRF_LOG_ERROR( "%i Error when sending UBX message. Message too long: 0x%x.", ulReadRTC(), uiUbxMsgLength );
			NRF_LOG_FLUSH();
			return false;
		}
	}
	
	/* Verify the checksum. */
	if ( puiUbxMsgWritePtr - puiUbxMsgReadPtr == 6 + uiUbxMsgLength + 0 )
	{
		uiMsgChkSum = uiDataByte;
	}
	if ( puiUbxMsgWritePtr - puiUbxMsgReadPtr == 6 + uiUbxMsgLength + 1 )
	{
		uiMsgChkSum += uiDataByte << 8;
		uiLocalChkSum = uiCalcChkSum( puiUbxMsgReadPtr, uiUbxMsgLength + 8 );
	
		/* Compare checksum. */
		if ( uiMsgChkSum != uiLocalChkSum )
		{			
			NRF_LOG_ERROR( "%i Error while checking UBX message. Checksum mismatch. Expected: 0x%04x, calculated: 0x%04x.", ulReadRTC(), uiMsgChkSum, uiCalcChkSum );
			NRF_LOG_FLUSH();
			return false;
		}

		/* Update pointer to point to the start of the next message. */
		puiUbxMsgReadPtr += uiUbxMsgLength + 6 + 2;						
		
		/* Set the message length to maximum so that the procedure does not abort while receiving the header. */
		uiUbxMsgLength = AN_BUFFER_LEN - 5;
	}

	/* Increment the write index for the next message. */
	puiUbxMsgWritePtr++;
	uiANDataLength++;

	if ( uiANDataLength > AN_BUFFER_LEN )
	{
		NRF_LOG_WARNING( "%i AssistNow data buffer overflow!", ulReadRTC() );
		NRF_LOG_FLUSH();
		return false;
	}

	return true;
}
/*-----------------------------------------------------------*/


/* Send the GPS AssistNow data to the GPS module.

   The data is stored as a blob in a byte array. It need to be parsed byte-by-byte, delineated into UBX messages, check 
   for integrity (even though this has already been done before) and sent via I2C to the GPS module.
*/
static void vSendAssistNowData( void )
{
    ret_code_t				xErrCode;
	uint16_t				uiLocalChkSum;	
	uint8_t					*puiUbxMsgReadPtr;
	
	/* Initialise read pointer to the message buffer start. */
	puiUbxMsgReadPtr = uiAssistNowData;
	
	/* Walk through the entire AssistNow data file. */
	while ( puiUbxMsgReadPtr < uiAssistNowData + uiANDataLength )
	{
		/* Check that the message starts with SYNC_CHAR_1 and SYNC_CHAR_2 characters. */
		if ( *( puiUbxMsgReadPtr + 0 ) != SYNC_CHAR_1 )
		{
			NRF_LOG_ERROR( "%i Error when sending UBX message. Sync character 1 not present.", ulReadRTC() );
			NRF_LOG_FLUSH();
			return;
		}
		if ( *( puiUbxMsgReadPtr + 1 ) != SYNC_CHAR_2 )
		{
			NRF_LOG_ERROR( "%i Error when sending UBX message. Sync character 2 not present.", ulReadRTC() );
			NRF_LOG_FLUSH();
			return;
		}	
		
		/* Get the message length. */
		uiUbxMsgLength =   *( puiUbxMsgReadPtr + 4 )
						 + ( *( puiUbxMsgReadPtr + 5 ) << 8 );
		
		/* Verify the checksum. */
		uiMsgChkSum =   *( puiUbxMsgReadPtr + 6 + uiUbxMsgLength + 0 )
					  + ( *( puiUbxMsgReadPtr + 6 + uiUbxMsgLength + 1 ) << 8 );
		uiLocalChkSum = uiCalcChkSum( puiUbxMsgReadPtr, uiUbxMsgLength + 8 );

		/* Compare checksum. */
		if ( uiMsgChkSum != uiLocalChkSum )
		{			
			NRF_LOG_ERROR( "%i Error while checking UBX message. Checksum mismatch. Expected: 0x%04x, calculated: 0x%04x.", ulReadRTC(), uiMsgChkSum, uiCalcChkSum );
			NRF_LOG_FLUSH();
			return ;
		}

		/* The function xGpsWriteRaw() updates the read pointer to point behind that last byte read. */
		xErrCode = xGpsWriteRaw( &puiUbxMsgReadPtr );
		if ( xErrCode != NRF_SUCCESS )
		{
			NRF_LOG_ERROR( "%i TWI error when sending UBX message: %x", ulReadRTC(), xErrCode );
			NRF_LOG_FLUSH();
			return;
		}
	}
}
/*-----------------------------------------------------------*/


/* Verify a UBX message checksum. 
   Input:
		*ppuiUbxMessage			points to a UBX message.
   output:
		*ppuiUbxMessage			points to the next bytes following the UBX message.
		Returns true if the message checksum is good.
*/
static bool bVerifyUbxMessage( uint8_t **ppuiUbxMessage )
{
	uint16_t		uiMessageLength;
	uint16_t		uiChkSum;
	uint16_t		uiRecChkSum;
	
	/* Check that the message starts with SYNC_CHAR_1 and SYNC_CHAR_2 characters. */
	if ( *( *ppuiUbxMessage + 0 ) != SYNC_CHAR_1 )
	{
		NRF_LOG_ERROR( "%i Error while checking UBX message. Sync character 1 not present.", ulReadRTC() );
		NRF_LOG_FLUSH();
		return false;
	}
	if ( *( *ppuiUbxMessage + 1 ) != SYNC_CHAR_2 )
	{
		NRF_LOG_ERROR( "%i Error while checking UBX message. Sync character 2 not present.", ulReadRTC() );
		NRF_LOG_FLUSH();
		return false;
	}
	
	/* Get the message length. */
	uiMessageLength = *( *ppuiUbxMessage + 4 ) + ( *( *ppuiUbxMessage + 5 ) << 8 );
	if ( uiMessageLength > MSG_BUFFER_LEN )
	{
		NRF_LOG_ERROR( "%i Error while checking UBX message. Message too long: 0x%x.", ulReadRTC(), uiMessageLength );
		NRF_LOG_FLUSH();
		return false;
	}
	
	/* Calculate checksum. */
	uiChkSum = uiCalcChkSum( *ppuiUbxMessage, uiMessageLength + 8 );
	uiRecChkSum =     *( *ppuiUbxMessage + 6 + uiMessageLength + 0 )
				  + ( *( *ppuiUbxMessage + 6 + uiMessageLength + 1 ) << 8 );
	
	/* Compare checksum. */
	if ( uiChkSum != uiRecChkSum )
	{
		
		NRF_LOG_ERROR( "%i Error while checking UBX message. Checksum mismatch. Expected: 0x%04x, received: 0x%04x.", ulReadRTC(), uiChkSum, uiRecChkSum );
		NRF_LOG_FLUSH();
		return false;
	}

	/* The message is good. Update the pointer and return. */
	*ppuiUbxMessage = *ppuiUbxMessage + 6 + uiMessageLength + 2;

	return true;
}
/*-----------------------------------------------------------*/


/* Walk through the AssistNow data set and check it for consistency.
   Rules:
   - The dataset starts with a UBX message.
   - The dataset ends with a UBX message.
   - The is no data in between UBX messages.
   - The checksum of all UBX mesages passes.
*/
bool bCheckAssistNowDataConsistency( void )
{
	uint8_t		*puiDataSet;
	bool		bSuccess;
	uint32_t	uiMsgCount;
	
	puiDataSet = uiAssistNowData;
	
	uiMsgCount = 0;
	do
	{
		uiMsgCount++;
		bSuccess = bVerifyUbxMessage( &puiDataSet );
	}
	while ( ( puiDataSet < uiAssistNowData + uiANDataLength ) && bSuccess );

	NRF_LOG_DEBUG( "Verfified %i UBX messages.", uiMsgCount );
	NRF_LOG_FLUSH();

	return bSuccess;
}
/*-----------------------------------------------------------*/


/* Send the AssistNow data set to the GPS module. */
bool bSendAssistNowToGPS( void )
{
	uint8_t					*puiDataSet;
	uint8_t					*puiUbxMsgStart;
	uint32_t				uiLocalUbxMsgLen;
    ret_code_t				xErrCode;
	
	/* Initialise read and write pointers to the message buffer start. */
	puiDataSet = uiAssistNowData;
	puiUbxMsgStart = uiAssistNowData;
		
	/* Send all messages bytes to the GPS receiver. Parse the message to know when a new one
	   starts and insert a delay just before. This delay helps the receiver keep up with the
	   data flow. */
	do
	{		
		/* Calculate the message length to know when to insert a delay. */		
		if ( puiDataSet == puiUbxMsgStart )
		{
			uiLocalUbxMsgLen = *( puiUbxMsgStart + 4 ) + ( *( puiUbxMsgStart + 5 ) << 8 );
			
			/* Advance the pointer to the start of the next message. */
			puiUbxMsgStart = puiUbxMsgStart + 6 + uiLocalUbxMsgLen + 2;

			/* Add a delay in between two UBX messages as per ublox specification. */
			vTaskDelay( GPS_UBX_MSG_DLY );
		}

		/* Send the byte. The function xGpsWriteRaw() increments the pointer. */
		xErrCode = xGpsWriteRaw( &puiDataSet );		
	}
	while ( ( puiDataSet < uiAssistNowData + uiANDataLength ) && ( xErrCode == NRF_SUCCESS ) );

	return ( xErrCode == NRF_SUCCESS );
}
/*-----------------------------------------------------------*/


/* Extract data from the NMEA $G*GSA message contained in +GNGSA notifications.
   The format is fixed:
               0 1 2  3  4  5 ...
		$GNGSA,A,3,25,26,31,14,,,,,,,,,3.97,2.35,3.20*0E
		       | | |  |       
		       | | |  2nd satellite number
		       | | 1st satellite number
		       | navMode (1 = no fix, 2 = 2D, 3 = 3D) 
		       opMode (M = manual, A = automatic)
   This message must be the first in the sequence of NMEA sentences read as it contains information
   on how to interpret the others.
*/
static void vParseGNGSA( signed char *pcNmeaStrg )
{
	/* Modification of the GSM data is protected by a mutex which avoids that both Parser and GSM 
	   task try to modify the data at the same time. */
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );

	/* Get fix quality indicator. */
	cGsm_FIX[ 0 ] = *pcFindNComma( pcNmeaStrg, 1, strlen( pcNmeaStrg ) );
	cGsm_FIX[ 1 ] = 0;
	cGsm_BLE_FIX[ 0 ] = cGsm_FIX[ 0 ];
	cGsm_BLE_FIX[ 1 ] = 0;
	
	/* No fix is coded as '0'. */
	if ( cGsm_FIX[ 0 ] == '1' )
	{
		cGsm_FIX[ 0 ] = 0;
		cGsm_BLE_FIX[ 0 ] = 0;
	}
	uiGPSReportStatus |= RPT_NMEA_GNGSA;

	if ( cGsm_FIX[ 0 ] != 0 )
	{
		xGpsFixTimeStamp = xTaskGetTickCount();
	}

	xSemaphoreGive( xMutexGsmData );
}
/*-----------------------------------------------------------*/


/* Extract SVN and CN0 data from the NMEA $G*GSV message.
		
	Format (NMEA 4.10):
		09:59:24.50   131.53  $GPGSV,3,1,10,01,50,297,23,03,24,227,,08,68,174,25,10,21,050,21,0*6A
		09:59:24.69   131.71  $GPGSV,3,2,10,11,77,314,21,21,70,056,25,22,48,232,26,27,34,149,26,0*61
		09:59:24.82   131.85  $GPGSV,3,3,10,28,18,315,17,32,39,087,23,0*63
		09:59:24.92   131.95  $GLGSV,3,1,11,66,09,028,,67,58,053,,68,56,174,,69,08,197,,0*75
		09:59:25.06   132.09  $GLGSV,3,2,11,73,06,265,,74,12,311,,75,00,002,,81,05,118,,0*7C
		09:59:25.19   132.22  $GLGSV,3,3,11,82,50,097,,83,63,351,,84,12,310,,0*4C
		09:59:25.31   132.33  $GAGSV,3,1,09,01,20,283,,03,25,047,,07,29,174,21,08,53,108,25,0*7F
		09:59:25.45   132.47  $GAGSV,3,2,09,13,73,055,25,15,25,100,,21,13,234,,26,41,301,20,0*7A
		09:59:25.59   132.62  $GAGSV,3,3,09,31,06,332,,0*4B
		
		                 1  1   1
		       0 2 4  7  0  3   7 ...
		$GAGSV,3,2,09,13,73,055,25,15,25,100,,21,13,234,,26,41,301,20,0*7A
		       | | |  |  |  |   |                                     |
		       | | |  |  |  |   |                                     signal identifier
		       | | |  |  |  |   CN0 signal strength in dBHz
		       | | |  |  |  azimuth
		       | | |  |  elevation
		       | | |  satellite ID
		       | | number of GPS or GLONASS satellites
			   | number of this message
		       total number of GSV messages
		
   There is only one parser for $GPGSV, $GLGSV and $GAGSV messages as the format is the same.
   Galileo satellite numbering goes from 1 to 32. As this overlaps with the GPS satellite numbering, 
   the ublox convention of adding an offset of 210 to the Galileo satellite numbers is used.
*/
static void vParseGPGSV( signed char *pcNmeaStrg )
{
	vParseG_GSV( pcNmeaStrg, GPS );
	uiGPSReportStatus |= RPT_NMEA_GPGSV;
}

static void vParseGLGSV( signed char *pcNmeaStrg )
{
	vParseG_GSV( pcNmeaStrg, GLONASS );
	uiGPSReportStatus |= RPT_NMEA_GPGSV;
}

static void vParseGAGSV( signed char *pcNmeaStrg )
{
	vParseG_GSV( pcNmeaStrg, GALILEO );
	uiGPSReportStatus |= RPT_NMEA_GAGSV;
}

static void vParseG_GSV( signed char *pcNmeaStrg, enum xGNSS_CONSTELLATION_ID xConstellationId )
{
	unsigned portBASE_TYPE		uxTblIdx;
	unsigned portBASE_TYPE		uxNumReports;
	unsigned portBASE_TYPE		uxReportsIdx;
	signed char					cNumberBuffer[ 3 ];
	signed char					cChar;
	unsigned portBASE_TYPE		uxSvn;
	unsigned portBASE_TYPE		uxCn0;

	/* Modification of the GSM data is protected by a mutex which avoids that both Parser and GSM 
	   task try to modify the data at the same time. */
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );
	
	/* Extract the total number of satellites visible. */
	strncpy( cNumberBuffer, pcFindNComma( pcNmeaStrg, 1, strlen( pcNmeaStrg ) ), 2 );
	if ( cNumberBuffer[ 0 ] == '1' )
	{
		/* This is the first report: Copy the number of satellites visible. */
		strncpy( cNumberBuffer, pcFindNComma( pcNmeaStrg, 2, strlen( pcNmeaStrg ) ), 2 );
		uxGnssVisible[ xConstellationId ] = ucIntStrgToByte( cNumberBuffer, 2 );
	}	

	/* Count the number of fields in the report to calculate the number of contained reports. 
	   NMEA 4.10 reports contain one additional field. This does not affect the number of
	   reports calculated below as the value is truncated when dividing the number of commas by 4. */
	uxNumReports = 0;
	uxReportsIdx = 0;
	
	while (	( cChar = *( pcNmeaStrg + uxReportsIdx ) ) != 0 )
	{
		if ( cChar == ',' )
		{
			uxNumReports++;
		}
		uxReportsIdx++;
	}
	uxNumReports = ( uxNumReports - 2 ) >> 2;
	
	/* Check if the satellite string contains at least one satellite report. */
	if ( uxNumReports == 0 )
	{
		xSemaphoreGive( xMutexGsmData );
		return;
	}
	
	/* Walk through all reports. */
	for ( uxReportsIdx = 0; uxReportsIdx < uxNumReports; uxReportsIdx++ )
	{
		/* Copy the data from the buffer. */
		strncpy( cNumberBuffer, pcFindNComma( pcNmeaStrg, 3 + 4 * uxReportsIdx + 0, strlen( pcNmeaStrg ) ), 2 );
		uxSvn = 10 * ( cNumberBuffer[ 0 ] - '0' ) + ( cNumberBuffer[ 1 ] - '0' );
		if ( xConstellationId == GALILEO )
		{
			uxSvn += 210;
		}
		strncpy( cNumberBuffer, pcFindNComma( pcNmeaStrg, 3 + 4 * uxReportsIdx + 3, strlen( pcNmeaStrg ) ), 2 );
		if ( ( cNumberBuffer[ 0 ] != '*' ) && ( cNumberBuffer[ 0 ] != ',' ) && (cNumberBuffer[ 0 ] != 0 ) )
		{
			uxCn0 = 10 * ( cNumberBuffer[ 0 ] - '0' ) + ( cNumberBuffer[ 1 ] - '0' );
		}
		else
		{
			uxCn0 = 0;
		}
		
		/* If there is valid data, enter it in the SVN table. */
		if ( uxCn0 != 0 )
		{
			/* Find a first free entry in the list. */
			uxTblIdx = 0;
			while (   ( uxTblIdx < GPSDEB_SVN )
				   && ( xGpsSvnData[ uxTblIdx ].uxSvn != 0 ) 
				   && ( xGpsSvnData[ uxTblIdx ].uxSvn != uxSvn )
			   )
			{
				uxTblIdx++;
			}
			if ( uxTblIdx == GPSDEB_SVN )
			{
				/* Nothing left. */
				xSemaphoreGive( xMutexGsmData );
				return;
			}
		
			/* uxTblIdx now indexes the first empty entry in the xGpsSvnData table. */
			xGpsSvnData[ uxTblIdx ].uxSvn = uxSvn;
			xGpsSvnData[ uxTblIdx ].uxCn0 = uxCn0;
		}
	}
	
	xSemaphoreGive( xMutexGsmData );
}
/*-----------------------------------------------------------*/

/* Locate a data field by number in a comma-separated string and copy the field to a specified destination.
   Stop at uxMaxCnt characters. Append a 0x00 byte. The first field has field number uxFieldNum = 0.
   Returns the number of characters copied to the destination.
*/
static unsigned portBASE_TYPE xCopyFieldFromStrg( signed char *pcSrc, unsigned portBASE_TYPE uxFieldNum, unsigned portBASE_TYPE uxOffset, signed char *pcDest, unsigned portBASE_TYPE uxMaxCnt )
{
	unsigned portBASE_TYPE	uxCnt;
	unsigned portBASE_TYPE	uxSrcIdx;
	signed char				cChar;
	
	uxSrcIdx = 0;
	
	/* Locate the uxFieldNum-th comma-separated data field, i.e. find and skip uxFieldNum commas. */
	for ( uxCnt = 0; uxCnt < uxFieldNum; uxCnt++ )
	{
		cChar = 0;
		
		while (   ( ( cChar = *( pcSrc + uxSrcIdx++ ) ) != ',' ) 
			   && ( cChar != 0 ) )
		{
			;
		}
		
		/* Abort if there is are not enough fields in the string. */
		if ( cChar == 0 )
		{
			*( pcDest ) = 0;
			return 0;
		}
	}
	
	/* Add the character offset. */
	uxSrcIdx += uxOffset;
	
	/* Copy the data field. */
	uxCnt = 0;
	
	while (   ( uxCnt < uxMaxCnt ) 
		   && ( ( cChar = *( pcSrc + uxSrcIdx++ ) ) != ',' )
		   && ( cChar != 0 ) )
	{
		*( pcDest + uxCnt++ ) = cChar;
	}
	
	if ( uxCnt < uxMaxCnt )
	{
		*( pcDest + uxCnt ) = 0;	
	}
	
	return uxCnt;
}
/*-----------------------------------------------------------*/

/* Extract data from the NMEA $G*GGA message contained in +UGGGA notifications. 
   The format is fixed:
               0         1          2 3           4 5 6  7    8
		$GPGGA,084449.00,4337.65648,N,00702.47019,E,1,04,2.58,183.2,M,47.3,M,,*5E
		 |         |        |       |     |       | | |  |    |      
		 |         |        |       |     |       | | |  |    ALT in m
		 |         |        |       |     |       | | |  HDOP
		 |         |        |       |     |       | | SV number of satellites used
		 |         |        |       |     |       | quality (1 for 2D/3D fix, 3 for differential)
		 |         |        |       |     |       E/W east/west 
		 |         |        |       |     LON in dddmm.mmmmm
		 |         |        |       N/S north/south indicator      
		 |         |        LAT in ddmm.mmmmm
		 |         UTC time in hhmmss.ss
		 NMEA message ID
*/
static void vParseGNGGA( signed char *pcNmeaStrg )
{
	unsigned portBASE_TYPE		uxIdx;
	unsigned short				usMinutes;
	signed char					cCharBuffer[ 6 ];
	
	/* Modification of the GSM data is protected by a mutex which avoids that both Parser and GSM 
	   task try to modify the data at the same time. */
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );

	/* Only use the information in this message if there was either a 2D or 3D fix reported in the 
	   previous GSA message. */
	if ( cGsm_FIX[ 0 ] != 0 )
	{
		/* Fix OK: Copy the data fields. */
		/* Get ALT and copy the data if the fis is 3D. */
		if ( cGsm_FIX[ 0 ] == '3' )
		{
			( void )xCopyFieldFromStrg( pcNmeaStrg, 8, 0, cGsm_ALT, LEN_GSM_ALT );
		}
		
		/* Get north/south indicator. */
		( void )xCopyFieldFromStrg( pcNmeaStrg, 2, 0, cGsm_LAT, 1 );
		*( cGsm_LAT + 1 ) = ' ';
		( void )xCopyFieldFromStrg( pcNmeaStrg, 2, 0, cGsm_BLE_LAT, 1 );
		*( cGsm_BLE_LAT + 1 ) = ' ';
		
		/* Get LAT and copy the data. */
		( void )xCopyFieldFromStrg( pcNmeaStrg, 1, 0, cGsm_LAT + 2, LEN_GSM_LAT - 2 );
		( void )xCopyFieldFromStrg( pcNmeaStrg, 1, 0, cGsm_BLE_LAT + 2, LEN_GSM_LAT - 2 );
		
		/* Get east/west indicator. */
		( void )xCopyFieldFromStrg( pcNmeaStrg, 4, 0, cGsm_LON, 1 );		
		*( cGsm_LON + 1 ) = ' ';
		( void )xCopyFieldFromStrg( pcNmeaStrg, 4, 0, cGsm_BLE_LON, 1 );
		*( cGsm_BLE_LON + 1 ) = ' ';
	
		/* Get LON and copy the data. */
		( void )xCopyFieldFromStrg( pcNmeaStrg, 3, 0, cGsm_LON + 2, LEN_GSM_LON - 2 );
		( void )xCopyFieldFromStrg( pcNmeaStrg, 3, 0, cGsm_BLE_LON + 2, LEN_GSM_LON - 2 );

		/* Position onto start of SAT and copy the data. */
		( void )xCopyFieldFromStrg( pcNmeaStrg, 6, 0, cGsm_SAT, LEN_GSM_SAT );
		( void )xCopyFieldFromStrg( pcNmeaStrg, 6, 0, cGsm_BLE_SAT, LEN_GSM_SAT );

		/* Position onto start of HDOP and copy the data. */
		( void )xCopyFieldFromStrg( pcNmeaStrg, 7, 0, cGsm_HDOP, LEN_GSM_HDOP );
		( void )xCopyFieldFromStrg( pcNmeaStrg, 7, 0, cGsm_BLE_HDOP, LEN_GSM_HDOP );
	}
	uiGPSReportStatus |= RPT_NMEA_GNGGA;
	
	xSemaphoreGive( xMutexGsmData );
}
/*-----------------------------------------------------------*/

/* Extract data from the NMEA $G*VTG message contained in +UGVTG notifications.
   The format is fixed:
               01 23 4     5 6     7 8
		$GNVTG,,T,,M,0.699,N,1.295,K,A*2A
		       |  |  |       |       |
		       |  |  |       |       posMode: A = autonomous, D = differential
		       |  |  |       VEL speed in kmh
		       |  |  speed in knots
		       |  magnetic heading in degrees
		       HDG true heading in degrees
		
*/
static void vParseGNVTG( signed char *pcNmeaStrg )
{
	signed char		cDifferential[ 2 ];
	
	/* Modification of the GSM data is protected by a mutex which avoids that both Parser and GSM 
	   task try to modify the data at the same time. */
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );

	/* Only use the information in this message if there was either a 2D or 3D fix reported in the 
	   previous GSA message. */
	if ( cGsm_FIX[ 0 ] != 0 )
	{		
		/* Position onto start of HDG and copy the data. */
		( void )xCopyFieldFromStrg( pcNmeaStrg, 0, 0, cGsm_HDG, LEN_GSM_HDG );

		/* Position onto start of VEL and copy the data. */
		( void )xCopyFieldFromStrg( pcNmeaStrg, 6, 0, cGsm_VEL, LEN_GSM_VEL );
			
		/* Get the differential fix indication. Optional flag. */
		( void )xCopyFieldFromStrg( pcNmeaStrg, 8, 0, cDifferential, 1 );
			
		/* Only takes fixes with 'A' or 'D' posMode (autonomous or differential).
		   Modify the fix value if the fix was differential. */
		if ( cDifferential[ 0 ] == 'D' )
		{
			cGsm_FIX[ 0 ] += 2;			/* For a differential fix, '2' becomes '4' and '3' becomes '5'. */
			cGsm_BLE_FIX[ 0 ] += 2;
		}
		else
		{
			if ( cDifferential[ 0 ] != 'A' )
			{
				cGsm_FIX[ 0 ] = 0;
				cGsm_BLE_FIX[ 0 ] = 0;
			}
		}
	}
	uiGPSReportStatus |= RPT_NMEA_GNVTG;

	xSemaphoreGive( xMutexGsmData );
}
/*-----------------------------------------------------------*/

/* Get a precision value from an UBX-format response. The value is 4 bytes long (32-bit) and in
   little Endian notation. The precision in the UBX response is in mm. This function converts it to 
   meters (returned as ASCII string) and centimeters (returned as unsigned short) and stores them at the given destinations.
*/
static void vGetPrecision( uint8_t *uiDataPtr, signed char *pcDestM, unsigned short *usDestCM )
{
	uint32_t				uiPrecision;
	
	/* Get precision in mm. Note that the precision is ordered in little endian in the data array. */
	uiPrecision =     *( uiDataPtr )
				  + ( *( uiDataPtr + 1 ) <<  8 )
				  + ( *( uiDataPtr + 2 ) << 16 )
				  + ( *( uiDataPtr + 3 ) << 24 );
	
	/* Add 0.5m (= 500) to round the value correctly when truncating to meters and then convert to meters.
	   The precision is now capped to a short range, i.e. to 65535(m). This gives 5 string characters maximum. */
	vShortToIntStrg( pcDestM, ( unsigned short )( ( uiPrecision + 500 ) / 1000 ), true );	

	/* Add 0.5cm (= 5) to round the value correctly when truncating to centimeters. */
	uiPrecision = ( uiPrecision + 5 ) / 10;
	
	/* Cap the value to 65535 = 6.5535km - just in case. */
	if ( uiPrecision > 65535 )
	{
		uiPrecision = 65535;
	}

	/* The precision in centimeters is stored as 16-bit value. */	
	*usDestCM = ( unsigned short )uiPrecision;	
}
/*-----------------------------------------------------------*/

/* Extract data from the UGUbxNavPos response. The response is a UBX protocol in binary (HEX-ASCII) format.
   All numbers are either signed or unsigned 32-bit in little Endian order.
                                1       1       2       2       2       3
		0       4       8       2       6       0       4       8       2
	    B56201021C0068EA4E1804223204539D001A427E03004FC5020097E617007FE610001F28
		|       |   |       |       |       |       |       |       |       | 
		|       |   |       |       |       |       |       |       |       CK_A + CK_B
		|       |   |       |       |       |       |       |       vertical accuracy (m)
		|       |   |       |       |       |       |       horizontal accuracy (mm)
		|       |   |       |       |       |       height above mean sea level (mm)
		|       |   |       |       |       height above ellipsoid (mm)
		|       |   |       |       latitude * 1e7 (deg)
		|       |   |       longitude * 1e7 (deg)
		|       |   iTOW 
		|       length
		header + ID
	
*/
static void vParseUbxNavPos( uint8_t *puiUbxMessage )
{
	/* Modification of the GSM data is protected by a mutex which avoids that both Parser and GSM 
	   task try to modify the data at the same time. */
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );

	/* Get horizontal accuracy if we have a (any) fix. Save the prevous value so that the bHasGoodFix() function can calculate the 
	   precision improvement.
	   Also, copy the longitude/latitude/altitude/height/accuracy string to the variable containing the initial position estimate
	   for the next GPS assistance data request. Record the current time stamp. */
	usPrevHorizontalAccuracy = usGps_ACCHCM;	   
	vGetPrecision( puiUbxMessage + 26, cGsm_ACCHM, &usGps_ACCHCM );
	strncpy( cGsm_BLE_ACCHM, cGsm_ACCHM, LEN_GSM_ACCHM  + 1 );

	/* Get vertical accuracy if we have a 3-D fix (differential or not). */
	vGetPrecision( puiUbxMessage + 30, cGsm_ACCVM, &usGps_ACCVCM );
	
	uiGPSReportStatus |= RPT_UBX_NAV_POSLLH;
	
	xSemaphoreGive( xMutexGsmData );
}
/*-----------------------------------------------------------*/

/* Extract hardware status data from the UBX-MON_HW response. The response is a UBX protocol in binary (HEX-ASCII) format.
   All numbers are either signed or unsigned 32-bit in little Endian order.
                                1       1       2       2       2       3       3       4       4       4       5       5       6       6     
		0       4       8       2       6       0       4       8       2       6       0       4       8       2       6       0       4     
	    B5620A093C00C0F401000000000040000100AFD7010064006A14020109847FEB01000A0B0C0D0E0F37FF0203FF10FF121336353E0F5C0000000080F7010000000000EE4A
		|       |   |       |       |       |       |   |   | | |   |       |                                 |     |       |       |       |
		|       |   |       |       |       |       |   |   | | |   |       |                                 |     |       |       |       CK_A + CK_B
		|       |   |       |       |       |       |   |   | | |   |       |                                 |     |       |       pullL
		|       |   |       |       |       |       |   |   | | |   |       |                                 |     |       pullH
		|       |   |       |       |       |       |   |   | | |   |       |                                 |     pinIrq
		|       |   |       |       |       |       |   |   | | |   |       |                                 jamInd
		|       |   |       |       |       |       |   |   | | |   |       VP
		|       |   |       |       |       |       |   |   | | |   usedMask
		|       |   |       |       |       |       |   |   | | flags
		|       |   |       |       |       |       |   |   | aPower
		|       |   |       |       |       |       |   |   aStatus
		|       |   |       |       |       |       |   agcCnt
		|       |   |       |       |       |       noisePerMs
		|       |   |       |       |       pinVal
		|       |   |       |       pinDir
		|       |   |       pinBank
		|       |   pinSel 
		|       length
		header + ID
	
*/
static void vParseUbxMonHw( uint8_t *puiUbxMessage )
{
	/* Modification of the GSM data is protected by a mutex which avoids that both Parser and GSM 
	   task try to modify the data at the same time. */
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );

	/* Get noise level (16-bit) as measured by the GPS Core. */
	usGpsNoisePerMS = *( puiUbxMessage + 22 ) + ( *( puiUbxMessage + 23 ) << 8 );
	
	/* Get AGC Monitor (16-bit). */
	usGpsAgcCnt = *( puiUbxMessage + 24 ) + ( *( puiUbxMessage + 25 ) << 8 );
	
	/* Get HW flags (8-bit). */
	ucGpsHwFlags = *( puiUbxMessage + 28 );
	
	/* Get CW Jamming indicator (8-bit). */
	ucGpsJamInd = *( puiUbxMessage + 51 );
	
	uiGPSReportStatus |= RPT_UBX_MON_HW;
	
	xSemaphoreGive( xMutexGsmData );
}
/*-----------------------------------------------------------*/

/* Extract hardware status data from the UBX-MON_HW2 response. The response is a UBX protocol in binary (HEX-ASCII) format.
   All numbers are either signed or unsigned 32-bit in little Endian order.
                                     1       1       2       2       2       3
	    0       4       8       2       6       0       4       8       2
	    B5620A0B1C00019C389E6F080700FFFFFFFFFFFFFFFFFFFFFFFF00000000000000001697
	    |       |   | | | | |       |                       |               | 
	    |       |   | | | | |       |                       |               CK_A + CK_B
	    |       |   | | | | |       |                       POST status word
	    |       |   | | | | |       lowLevCfg (obsolete)
	    |       |   | | | | cfgSource
	    |       |   | | | magQ
	    |       |   | | ofsQ
	    |       |   | magI
	    |       |   ofsI 
	    |       length
	    header + ID
	
*/
static void vParseUbxMonHw2( uint8_t *puiUbxMessage )
{
	/* Modification of the GSM data is protected by a mutex which avoids that both Parser and GSM 
	   task try to modify the data at the same time. */
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );

	/* Get Imbalance of I-part of complex signal (8-bit). */
	ucGpsOfsI = *( puiUbxMessage + 6 );	
	
	/* Get Magnitude of I-part of complex signal. */
	ucGpsMagI = *( puiUbxMessage + 7 );	
	
	/*Get Imbalance of Q-part of complex signal. */
	ucGpsOfsQ = *( puiUbxMessage + 8 );	

	/*Get Magnitude of Q-part of complex signal. */
	ucGpsMagQ = *( puiUbxMessage + 9 );	
	
	uiGPSReportStatus |= RPT_UBX_MON_HW2;

	xSemaphoreGive( xMutexGsmData );
}
/*-----------------------------------------------------------*/

/* Process a received UBX message.

   Extract the relevant information and store it for later sending. 
   Depending on the xGpsFixReportLevel, also send it raw to the serial interface. */
ret_code_t xProcessUbxMessage( uint8_t *puiUbxMessage, uint32_t uiUbxMessageLen )
{
	/* Send the UBX message to the TRC serial interface if the xGpsFixReportLevel is set appropriately. */
	if ( xGpsFixReportLevel >= FIXRPT_NMEA_UBX )
	{
		if (   ( ( *( puiUbxMessage + 2 ) == UBX_NAV ) && ( *( puiUbxMessage + 3 ) == UBX_NAV_POSLLH ) )
			|| ( ( *( puiUbxMessage + 2 ) == UBX_MON ) && ( *( puiUbxMessage + 3 ) == UBX_MON_HW ) )
			|| ( ( *( puiUbxMessage + 2 ) == UBX_MON ) && ( *( puiUbxMessage + 3 ) == UBX_MON_HW2 ) ) 
			|| ( xGpsFixReportLevel >= FIXRPT_ALL ) )
		{
			char		cUbxAsciiSendBuffer[ 146 ] = "+GPS:";
			uint32_t	uiBufIdx;
			
			for ( uiBufIdx = 0; uiBufIdx < uiUbxMessageLen; uiBufIdx++ )
			{
				vByteToHexStrg( cUbxAsciiSendBuffer + ( 2 * uiBufIdx + 5 ), *( puiUbxMessage + uiBufIdx ) );
			}
			cUbxAsciiSendBuffer[ 2 * uiUbxMessageLen + 5 ] = '\r';
			cUbxAsciiSendBuffer[ 2 * uiUbxMessageLen + 6 ] = '\n';
			cUbxAsciiSendBuffer[ 2 * uiUbxMessageLen + 7 ] = 0;

			xComSendString( COM_TRC, cUbxAsciiSendBuffer );
		}					
	}
	
	/* Parse individual UBX NAV_POSLLH messages. */
	if ( ( *( puiUbxMessage + 2 ) == UBX_NAV ) && ( *( puiUbxMessage + 3 ) == UBX_NAV_POSLLH ) )
	{
		vParseUbxNavPos( puiUbxMessage );
	}

	/* Parse individual UBX MON_HW messages. */
	if ( ( *( puiUbxMessage + 2 ) == UBX_MON ) && ( *( puiUbxMessage + 3 ) == UBX_MON_HW ) )
	{
		vParseUbxMonHw( puiUbxMessage );
	}

	/* Parse individual UBX MON_HW2 messages. */
	if ( ( *( puiUbxMessage + 2 ) == UBX_MON ) && ( *( puiUbxMessage + 3 ) == UBX_MON_HW2 ) )
	{
		vParseUbxMonHw2( puiUbxMessage );
	}

	return NRF_SUCCESS;
}
/*-----------------------------------------------------------*/

/* Process a received NMEA sentence.

   Extract the relevant information and store it for later sending. 
   Depending on the xGpsFixReportLevel, also send it raw to the TRC serial interface. */
ret_code_t	xProcessNmeaSentence( signed char *pcNmeaSentenceBuffer )
{
	uint32_t				uiNmeaIdx = 0;
	bool					bNmeaFound = false;
	uint32_t				uiStrgIdx;
	signed char				cNmea;
	signed char				cNmeaRef;
		
	/* Identify the NMEA sentence and call the parser. */
	while ( ( !bNmeaFound ) && ( uiNmeaIdx < ( portBASE_TYPE )( sizeof( xNmeaId ) / sizeof( xNmeaId[ 0 ] ) ) ) )
	{
		/* Check if the response pointed to by uiNmeaIdx is a match. */
		uiStrgIdx = 0;
		do
		{
			cNmea = *( pcNmeaSentenceBuffer + uiStrgIdx );
			cNmeaRef = *( xNmeaId[ uiNmeaIdx ].pcNmeaStrg + uiStrgIdx );
			/* A '*' means: match any character. */
			if ( ( cNmea != 0 ) && ( cNmeaRef == '*' ) )
			{
				cNmea = cNmeaRef;
			}
			uiStrgIdx++;
		} 
		while (    ( cNmea != 0 ) 
				&& ( cNmeaRef != 0 )
				&& ( cNmeaRef == cNmea ) );

		/* A match has been found if the end of the reference string has been reached and 
		   no mismatch has been encountered before. */
		if ( cNmeaRef == 0 )
		{
			bNmeaFound = true;
			uiStrgIdx--;
		}
		else
		{
			/* No match: select next entry in the NMEA parser table. */
			uiNmeaIdx++;
		}
	}
	
	/* Parse the NMEA sentence, if a match was found in the table. */
	if ( uiNmeaIdx < sizeof( xNmeaId ) / sizeof( xNmeaId[ 0 ] ) )
	{
		void			( *pvNmeaParser )( signed char *pcNmeaStrg );
	
		/* If there is a parameter to be taken, store it in the indicated location. */
		pvNmeaParser = xNmeaId[ uiNmeaIdx ].pvNmeaParser;
		if ( pvNmeaParser != NULL )
		{
			/* Updating the parameters need to be protected by a mutex as they might be read anytime by the GSM task.
			   The mutex avoids that the GSM task reads inconsistent data. Care must be taken that the GSM task
			   takes the mutex only for a very short, determined time as the mutex may result in priority inversion. 
			   Also, it must be ensured that there is only one mutex used between GSM and Parser task, else the system
			   might enter a deadlock. */
			pvNmeaParser( pcNmeaSentenceBuffer + uiStrgIdx );
		}
	}
	
	return NRF_SUCCESS;
}
/*-----------------------------------------------------------*/

/* Invalidate all data fields to be sent to the server. This function is called as soon as 
   a tracker packet has been sent to the server. Doing this allows the GSM task to see which fields have really
   been updated since the last packet. 
*/
void vInvalidateGpsData( void )
{
	unsigned portBASE_TYPE		uxIdx;
	
	/* Modification of the GSM data is protected by a mutex which avoids that both Parser and GSM 
	   task try to modify the data at the same time. */
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );
	
	/* Invalidating a field is done by poking 0 (end-of-string) into the first position of the
	   string. */
	cGsm_LAT	[ 0 ] = 0;		/* field  5: latitude */
	cGsm_LON	[ 0 ] = 0;		/* field  6: longitude */
	cGsm_ALT	[ 0 ] = 0;		/* field  7: altitude (4 digits + sign) */
	cGsm_VEL	[ 0 ] = 0;		/* field  8: velocity */
	cGsm_HDG	[ 0 ] = 0;		/* field  9: heading */
	cGsm_FIX	[ 0 ] = 0;		/* field 10: GPS fix type */
	cGsm_HDOP	[ 0 ] = 0;		/* field 11: Horizontal Dilution of Precision */
	cGsm_LOCM	[ 0 ] = 0;		/* field 12: Localization method */
	cGsm_SAT	[ 0 ] = 0;		/* field 13: satellites in view and used */

	cGsm_ACCHM	[ 0 ] = 0;		/* field 16: GPS fix horizontal accuracy */
	cGsm_ACCVM	[ 0 ] = 0;		/* field 17: GPS fix vertical accuracy */

	/* field 26: SVN data list. */
	usGpsNoisePerMS = 0;
    usGpsAgcCnt = 0;	
    ucGpsJamInd = 0;	
	ucGpsHwFlags = 0;
    ucGpsOfsI = 0;		
    ucGpsMagI = 0;		
    ucGpsOfsQ = 0;		
    ucGpsMagQ = 0;		
	for ( uxIdx = 0; uxIdx < GPSDEB_SVN; uxIdx++ )
	{
		xGpsSvnData[ uxIdx ].uxSvn = 0;
		xGpsSvnData[ uxIdx ].uxCn0 = 0;
	}
	
	/* Reset the previous horizontal accuracy to calculate improvement. */
	usPrevHorizontalAccuracy = 0;
	usGps_ACCHCM = 0;	
	
	/* Reset the total number of visible satellites. */
	uxGnssVisible[ 0 ] = 0;
	uxGnssVisible[ 1 ] = 0;
    uxGnssVisible[ 2 ] = 0;
	
	xSemaphoreGive( xMutexGsmData );
}
/*-----------------------------------------------------------*/


/* Function called once a complete update of the GPS data has been received from the GPS module.

   This function pushes a synopsis of the data to the trace interface for debug purposes.    
   Only send this report if all parts of its message have been updated.
*/
void vPushGpsFixSynopsis( void )
{
	unsigned portBASE_TYPE		uxTotalSatVisible;
	
	if ( uiGPSReportStatus == (   RPT_NMEA_GNGSA		
							    | RPT_NMEA_GPGSV    
							    | RPT_NMEA_GAGSV    
							    | RPT_NMEA_GNGGA    
							    | RPT_NMEA_GNVTG    
							    | RPT_UBX_NAV_POSLLH
							    | RPT_UBX_MON_HW    
							    | RPT_UBX_MON_HW2   ) )
	{	
		uxTotalSatVisible = uxGnssVisible[ 0 ] + uxGnssVisible[ 1 ] + uxGnssVisible[ 2 ];	
	
		vTracePushGpsFixSynopsis( cGsm_FIX, cGsm_HDOP, usGps_ACCHCM, cGsm_SAT, uxTotalSatVisible );
		
		uiGPSReportStatus = 0;	
	}
}
/*-----------------------------------------------------------*/


/* Read the GPS position data from the GPS module and return true if the GPS receiver has a good fix. 
   cGsm_Fix being not 0 signifies a fix. A good fix is qualified with the HDOP data. An HDOP 
   below the threshold is assumed to be good enough.
*/
uint32_t	uiGpsFailReason = 0;

bool bHasGPSGoodFix( void )
{
	unsigned short		usGPSFixQualityThres;
	unsigned short		usGPSFixQuality;
	signed long			lHorizontalAccuracyImprovement;
	
	/* Check if there is a fix at all. */
	if ( cGsm_FIX[ 0 ] == 0 )
	{
		uiGpsFailReason = 1;	
		return false;
	}
	
	/* Read GPS quality criterion from NVDS. */
	usGPSFixQualityThres = usConfigReadShort( &xNvdsConfig.usGPSMinQuality );
	
	/* If the required quality is set to 0, do not apply any quality criterion. */
	if ( usGPSFixQualityThres == 0 )
	{
		return true;
	}
	
	/* Calculate the precision improvement over the last fix. If it is lower than 1m (even negative), accept the fix. 
	   The current horizontal precision is available as 16-bit value in usGps_ACCHCM. 
	   A value of 0 means that the accuracy is not valid (set). */
	if ( ( usGps_ACCHCM == 0 ) || ( usPrevHorizontalAccuracy == 0 ) ) 
	{
		uiGpsFailReason = 2;	
		return false;
	}
	lHorizontalAccuracyImprovement = ( signed long )usPrevHorizontalAccuracy - ( signed long )usGps_ACCHCM;
	if ( lHorizontalAccuracyImprovement > ( signed long )usConfigReadShort( &xNvdsConfig.usGPSMaxHAcc ) )
	{
		uiGpsFailReason = 3;	
		return false;
	}
	
	/* Compare the HDOP to the predefined acceptance level. 
	   A value of 0 means that the HDOP is not valid (set). */
	if ( cGsm_HDOP[ 0 ] == 0 )
	{
		uiGpsFailReason = 4;	
		return false;
	}
	/* Convert the HDOP to hex(10*HDOP).  The decimal point of HDOP is either in the second (cGsm_HDOP[1]) 
	   or third position (cGsm_HDOP[2]). */
	if ( cGsm_HDOP[ 2 ] == '.' )
	{
		usGPSFixQuality = 100 * cCharToNibble( cGsm_HDOP[0] )
						 + 10 * cCharToNibble( cGsm_HDOP[1] );
	}
	else
	{
		usGPSFixQuality = 10 * cCharToNibble( cGsm_HDOP[0] )
							 + cCharToNibble( cGsm_HDOP[2] );
	}	

	/* Return 'bad fix' if the fix is older than 10min. */
	if ( xTaskGetTickCount() - xGpsFixTimeStamp > GPS_MAX_FIX_AGE )
	{
		return false;
	}
	
	/* Compare current HDOP with the threshold in the parameter set. */
	if ( usGPSFixQuality <= usGPSFixQualityThres )
	{
		/* Quality is at least as good as required: return good. */
		return true;
	}
	else
	{
		/* Return quality is bad. */
		uiGpsFailReason = 5;	
		return false;
	}
}
/*-----------------------------------------------------------*/

/* Trace the failure reason for a GPS fix fail. */
void vTraceGpsFixFail( void )
{
	unsigned short		usGPSFixQuality;

	V_TRACE_PRINT_BYTE( TRACE_GSM_GPS_NO_RECEPTION, ( char )uiGpsFailReason, TRACE_UART_AND_FILE );
	V_TRACE_PRINT_BYTE( TRACE_GSM_GPS_NO_RECEPTION, cCharToNibble( cGsm_FIX[ 0 ] ), TRACE_UART_AND_FILE );
	V_TRACE_PRINT_SHORT( TRACE_GSM_GPS_NO_RECEPTION, ( unsigned short )uxNumSvnEntries(), TRACE_UART_AND_FILE );

	if ( cGsm_HDOP[ 0 ] == 0 )
	{
		usGPSFixQuality = 0;
	}
	else
	{
		if ( cGsm_HDOP[ 2 ] == '.' )
		{
			usGPSFixQuality = 100 * cCharToNibble( cGsm_HDOP[0] )
							 + 10 * cCharToNibble( cGsm_HDOP[1] );
		}
		else
		{
			usGPSFixQuality = 10 * cCharToNibble( cGsm_HDOP[0] )
								 + cCharToNibble( cGsm_HDOP[2] );
		}	
	}
	V_TRACE_PRINT_SHORT( TRACE_GSM_GPS_NO_RECEPTION, usGPSFixQuality, 									TRACE_UART_AND_FILE );
	V_TRACE_PRINT_SHORT( TRACE_GSM_GPS_NO_RECEPTION, usConfigReadShort( &xNvdsConfig.usGPSMinQuality ), TRACE_UART_AND_FILE );
	V_TRACE_PRINT_SHORT( TRACE_GSM_GPS_NO_RECEPTION, usGps_ACCHCM, 										TRACE_UART_AND_FILE );	
	V_TRACE_PRINT_SHORT( TRACE_GSM_GPS_NO_RECEPTION, usIntStrgToShort( cGsm_ACCHM, 5 ), 				TRACE_UART_AND_FILE );
	V_TRACE_PRINT_SHORT( TRACE_GSM_GPS_NO_RECEPTION, usIntStrgToShort( cGsm_ACCVM, 5 ), 				TRACE_UART_AND_FILE );
}
/*-----------------------------------------------------------*/


/* Store the current GPS position in the GPS position record array. 

   The store is managed by the use of read and write pointers. The read pointer is initially at 0. The write pointer always points to the 
   next entry to be overwritten. If the store is full, the read pointer is always set to one ahead the write pointer so that the oldest
   entry is overwritten. 
   
   Each entry of the store is a string containing the preformatted position data. All data in the string are delineated by ';'. The format is as follows
   (the max. data item string length is indicated):
   
	time stamp								LEN_GSM_TIM		+ 1
	latitude                       			LEN_GSM_LAT     + 1 
	longitude                      			LEN_GSM_LON     + 1 
	altitude (4 digits + sign)     			LEN_GSM_ALT     + 1 
	velocity                       			LEN_GSM_VEL     + 1 
	heading                        			LEN_GSM_HDG     + 1 
	GPS fix type                   			LEN_GSM_FIX     + 1 
	Horizontal Dilution of Precision HDOP	LEN_GSM_HDOP    + 1 
	Localization method            			LEN_GSM_LOCM    + 1 
	satellites in view and used    			LEN_GSM_SAT     + 1 
	GPS fix horizontal accuracy    			LEN_GSM_ACCHM   + 1 
	GPS fix vertical accuracy      			LEN_GSM_ACCVM   + 1 
	Absolute step count            			LEN_GSM_STEPS   + 1   
*/
void vStoreGPSPosition( void )
{
	/* Lock access to the GSM data. */
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );
	
	if ( bHasGPSGoodFix() )
	{
		/* Write the current GPS position information to the GPS position store. */
		strncpy( xGpsPosStore[ uxPosStoreWrIdx ].cPosLAT,   cGsm_LAT,   LEN_GSM_LAT   );
		strncpy( xGpsPosStore[ uxPosStoreWrIdx ].cPosLON,   cGsm_LON,   LEN_GSM_LON   );
		strncpy( xGpsPosStore[ uxPosStoreWrIdx ].cPosALT,   cGsm_ALT,   LEN_GSM_ALT   );
		strncpy( xGpsPosStore[ uxPosStoreWrIdx ].cPosHDG,   cGsm_HDG,   LEN_GSM_HDG   );
		strncpy( xGpsPosStore[ uxPosStoreWrIdx ].cPosFIX,   cGsm_FIX,   LEN_GSM_FIX   );
		strncpy( xGpsPosStore[ uxPosStoreWrIdx ].cPosHDOP,  cGsm_HDOP,  LEN_GSM_HDOP  );
		strncpy( xGpsPosStore[ uxPosStoreWrIdx ].cPosLOCM,  cGsm_LOCM,  LEN_GSM_LOCM  );
		strncpy( xGpsPosStore[ uxPosStoreWrIdx ].cPosSV,    cGsm_SAT,   LEN_GSM_SAT   );
		strncpy( xGpsPosStore[ uxPosStoreWrIdx ].cPosACCH,  cGsm_ACCHM, LEN_GSM_ACCHM );
		strncpy( xGpsPosStore[ uxPosStoreWrIdx ].cPosACCV,  cGsm_ACCVM, LEN_GSM_ACCVM );

		xGpsPosStore[ uxPosStoreWrIdx ].bPosValidEntry = true;
	}
	else
	{
		xGpsPosStore[ uxPosStoreWrIdx ].bPosValidEntry = false;
	}
	
	xSemaphoreGive( xMutexGsmData );
}
/*-----------------------------------------------------------*/

/* 
 * GPS task function.
 */
void vGpsTask( void * pvParameter )
{
	enum xGPS_CMD			xGpsCmd;
	bool					bSuccess;
	unsigned portBASE_TYPE	uxIdx;
	TickType_t				xPollInterval;

    ( void )pvParameter;
	
	vTaskSetApplicationTaskTag( NULL, ( void * ) GPS_TASK_TAG );
	
	/* Initialise message length to 0 as we have nothing received yet. */
	uiUbxMessageLen = 0;
	
	/* Wait untile the configuration handler is initialised. */
	while ( !bCheckConfigInitialised() || !bCheckTraceInitialised() )
	{
		vTaskDelay( 1 );
	}

	NRF_LOG_INFO( "Task started." );
	NRF_LOG_FLUSH();
	
	xPollInterval = portMAX_DELAY;

	/* Task main loop. */
    while ( true )
	{
		/* Wait until there is a command to treat or until the queue receive times out. Then poll the
		   GPS module. */
		if ( xQueueReceive( xGpsCmdQueue, &xGpsCmd, xPollInterval ) != errQUEUE_EMPTY )
		{
			switch ( xGpsCmd )
			{
				case GPS_START:				/* Start the GPS module, unless already on. */											
											if ( xGetGpsState() == GPS_OFF )
											{
												V_TRACE_PRINT( TRACE_GPS_LOCALISATION_START, TRACE_UART );
												NRF_LOG_DEBUG( "%i Start.", ulReadRTC() );
												NRF_LOG_FLUSH();
												
												/* Initialise message length to 0 as we have nothing received yet. */
												uiUbxMessageLen = 0;
												
												xPollInterval = GPS_POLL_INTERVAL;
												
												/* Start the GPS module. */
												vGpsStart();

												/* Configure the GPS. */
												if ( !bConfigureGPS() )
												{
													/* The GPS module did not respond to a configuration request. 
													   Something is very fishy here. Reboot the entire hardware as it is not clear
													   how to resolve the issue. As far as investigations show, the nRF TWI might not
													   work. */
													V_TRACE_PRINT( TRACE_GPS_NO_I2C_RESPONSE, TRACE_UART_AND_FILE );
													V_SYSTEM_RESET( RESET_BAD_GPS_MODULE );
												}
												
												/* The validity of AN data is handled in the GSM module as its request is tight to the 
												   status of the GSM (i.e., internet must be connected). */
												
												/* Delete all GPS position information. */
												vInvalidateGpsData();

												uiGPSReportStatus = 0;
											}
											else
											{
												NRF_LOG_DEBUG( "%i Already on.", ulReadRTC() );
												NRF_LOG_FLUSH();												
											}
											
											break;		

				case GPS_STOP:				/* Stop the GPS module. */
											if ( xGetGpsState() == GPS_ON )
											{
												V_TRACE_PRINT( TRACE_GPS_LOCALISATION_STOP, TRACE_UART );
												NRF_LOG_DEBUG( "%i Stop.", ulReadRTC() );
												NRF_LOG_FLUSH();
												vGpsStop();
												xPollInterval = portMAX_DELAY;
											}
											else
											{
												NRF_LOG_DEBUG( "%i Already off.", ulReadRTC() );
												NRF_LOG_FLUSH();												
											}
											break;		
											
				case GPS_ANDATA_UPDATED:	/* The GPS assistance data has been updated. Upload it to the GPS module. */
											NRF_LOG_DEBUG( "%i Update AssistNow data.", ulReadRTC() );
											NRF_LOG_FLUSH();
											
											vSendAssistNowData();
											break;
											
				default:					break;
			}
		}

		/* Poll any messages the GPS module might want to send. */
        if ( xGetGpsState() == GPS_ON )
		{
			( void )xGpsGetAvailBytesAndRead( NULL, NULL );
			
			/* Trash any received UBX message we have received in the process. The Function xGpsGetAvailBytesAndRead() already
			   sends the data we need at  the main controller to the UART. */
			uiUbxMessageLen = 0;
		}
    }

    /* Tasks must be implemented to never return... */
}
/*-----------------------------------------------------------*/