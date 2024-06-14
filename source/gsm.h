/*
 * Tracker Firmware
 *
 * GSM/GPS driver header file
 *
 */
 
#ifndef GSM_H
#define GSM_H

#include <stdbool.h>
#include "parser.h"
#include "semphr.h"

#include "custom_board.h"


/* Size of the GSM UART Rx ring buffer. Must be a power of 2. */
#define	GSM_UART_RX_BUFFER_SIZE		( 256 )

#define GPS_ENABLE			( true )							/* Default GPS enable. */
		
/* Define timers */		
#define T_GSM_EXT_PWR_ON 	(  0.5 * portTICKS_PER_SEC )		/* Delay (ms) for setting GSM module external power on. */
#define T_GSM_REBOOT		(   10 * portTICKS_PER_SEC )		/* Delay (s) allowing SARA/QTEL to reboot from +CFUN=15. */
#define	T_GSM_CPOF			(    2 * portTICKS_PER_SEC )		/* Delay (ms) giving the AT+CPOF command to the GSM/GPS module. */
#define	T_GSM_PWR_OFF		(    5 * portTICKS_PER_SEC )		/* Delay (ms) after switching power to the GSM/GPS module off. */
#define	T_GSM_BAT_TEST		(   60 * portTICKS_PER_SEC )		/* If battery is low, delay (s) between testing the battery. */
#define	T_BAUD_SWITCH		(  0.5 * portTICKS_PER_SEC )		/* Time required by the cellular module for a baud switch. */
#define	T_GSM_EXIT_SLEEP	(  0.1 * portTICKS_PER_SEC )		/* Time required by the cellular module to exit sleep mode (estimated, no datasheet spec). */
	
#define	T_TCP_DISC			(    2 * portTICKS_PER_SEC ) 		/* Delay (s) allowing the TCP connection to properly disconnect. */

#define	T_FTP_LOGIN			(    2 * portTICKS_PER_SEC ) 		/* Delay (s) (apparently) required by Quectel between FTP login and any remote operation. Not documented. */
							
#define	TO_GSM_INT_PWR_ON	(  3.2 * portTICKS_PER_SEC )		/* Time-out (ms) for GSM module internal power on. */
#define	TO_GSM_AT			(    1 * portTICKS_PER_SEC ) 		/* Time-out (s) for getting a response from the GSM/GPS module. */
#define	TO_GSM_SIM_AT		(   20 * portTICKS_PER_SEC ) 		/* Time-out (s) for the GSM/GPS module to identify the (U)SIM card. */
#define	TO_GSM_NVDS_AT		(   20 * portTICKS_PER_SEC ) 		/* Time-out (s) for a GSM command resulting in writes nto the NVDS. */
#define	TO_GSM_AT_FAST		(  0.2 * portTICKS_PER_SEC ) 		/* Time-out (s) for getting a response to a simple AT from the GSM/GPS module. */
#define	TO_GSM_CGREG		(    2 * portTICKS_PER_SEC ) 		/* Time-out (s) for receiving a positive response to a GPRS attach request. */
#define TO_GSM_REG_ABANDON	(   10 * portTICKS_PER_SEC ) 		/* Time-out (s) for rebooting while the GSM module has abandoned any attach attempts. */
#define TO_GSM_REG			(  600 * portTICKS_PER_SEC ) 		/* Time-out (s) for attaching to the network. */
#define	TO_GSM_PAACT		(  180 * portTICKS_PER_SEC ) 		/* Time-out (s) for activating the GPRS PDP context (uBlox documentation). */
#define	TO_GSM_GPRS			(  180 * portTICKS_PER_SEC ) 		/* Time-out (s) for most GPRS actions (uBlox documentation). */
#define TO_GSM_TCPCNX		(  150 * portTICKS_PER_SEC ) 		/* Time-out (s) for getting a TCP connect confirmation. */
#define	TO_GSM_SRV_ACK		(   50 * portTICKS_PER_SEC )		/* Time-out (s) for getting response from the remote server. */
#define	TO_GSM_CGED			(   10 * portTICKS_PER_SEC )		/* Time-out (s) for getting a CGED response from the GSM/GPS module. */
#define	TO_GSM_PWROFF		(    5 * portTICKS_PER_SEC )		/* Time-out (s) for getting a response to the power-off command from the GSM/GPS module. */
#define	TO_GSM_TCPWR		(    5 * portTICKS_PER_SEC )		/* Time-out (s) for sending data via the TCP connection. */
#define	TO_GSM_VINT_DWN		(    5 * portTICKS_PER_SEC )		/* Time-out (s) for letting the module shut-down VINT. */
#define	TO_GSM_VGSM_DWN		(   20 * portTICKS_PER_SEC )		/* Time-out (s) for letting VGSM go down after shutting of the power switch. */
#define	TO_GSM_FTPLOGIN		(   90 * portTICKS_PER_SEC )		/* Time-out (s) for logging into the FTP server. */
#define	TO_GSM_FTPOP		(   10 * portTICKS_PER_SEC )		/* Time-out (s) for an FTP operation. */
#define	TO_GSM_FTPTRSF		(  360 * portTICKS_PER_SEC )		/* Time-out (s) for getting the FW update file (~120KB). */
#define	TO_GSM_FTPLOGOUT	(   20 * portTICKS_PER_SEC )		/* Time-out (s) for logging out. */
#define	TO_GSM_PING			(   10 * portTICKS_PER_SEC )		/* Time-out (s) for Ping response. */
#define	TO_GSM_HTTP			(   80 * portTICKS_PER_SEC )		/* Time-out (s) for HTTP GET (ticks). */
#define	TO_GSM_HTTPGET					"79"					/* Time-out (s) for HTTP GET command. */
#define	TO_GSM_HTTPURL		(    4 * portTICKS_PER_SEC )		/* Time-out (s) for configuring the HTTP URL. */
#define	TO_GSM_FS			(    2 * portTICKS_PER_SEC )		/* Time-out (s) for file system actions. */
#define	TO_GSM_SEC			(   10 * portTICKS_PER_SEC )		/* Time-out (s) for security manager actions. */
#define	TO_GSM_TCP_PRMPT	(    5 * portTICKS_PER_SEC )		/* Time-out (s) for receive a TCP send prompt. */
#define	TO_GSM_SEC_CO		(    2 * portTICKS_PER_SEC )		/* Time-out (s) for an unsolicited socket close after the connect when using TLS. */
#define TO_GSM_SOCL			(  120 * portTICKS_PER_SEC ) 		/* Time-out (s) for closing a TCP socket. */

#define TO_MUTEX_GSMDATA	(   21 * portTICKS_PER_SEC ) 		/* Time-out (s) for gaining access to the GSM data. */
	
#define MAX_VIBR_DUR					( 60 )					/* Maximum duration (s) for which to switch on the vibrator on server command. */
#define MAX_LED_DUR						( 655 )					/* Maximum duration (s) for which to switch on the LED(s) on server command. */
		
#define TCP_RETRY_LIMIT					( 4 )					/* Number of times a TCP connection is retried without resetting the GSM/GPS module. */

#define TCP_CONN_CLEAR					( 0x01 )				/* TCP connection: unsecured connection. */
#define TCP_CONN_SEC					( 0x02 )				/* TCP connection: secure connection. */
#define TCP_ESTABLISHED					( 0x04 )				/* TCP connection: initial connection established. */
#define TCP_SOCKET_CLOSE				( 0x08 )				/* TCP connection: unsolicited socket close after initial connection. */

		
#define T_GPS_POLL			(  2.5 * portTICKS_PER_SEC )		/* Delay (s) between two GPS position data polls. */
#define	TO_GPS_FIX			(  90  * portTICKS_PER_SEC )		/* Time-out (s) for obtaining a good GPS fix before sending a position packet. */
#define	TO_GPS_SV_RECVD		(  30  * portTICKS_PER_SEC )		/* Time-out (s) to receive at least MIN_SV_RECVD satellites before sending a position packet. */
#define	MIN_SV_RECVD					( 2 )					/* Required minimum number of received satellites to not abandon a GPS search prematurely. */

#define T_GPS_ASSISTED_DATA_VALIDITY	( 2 * 3600 )			/* (s) Minimum usability period of assisted GPS data downloaded from the ublox server. */
#define INITIAL_POS_VALIDITY			( 30 * 60 )				/* Time period in s during which the initial position is assumed to be valid. It will be reloaded
																   from the server if expired. */
#define INITIAL_POS_LENGTH				( 32 )							
#define POS_FILE_OFFSET					( 33 )

	
#define LEN_DFMAP						(  8 )					/* Length of the DFMAP parameter without end-of-string */
#define LEN_TCP_SOCKET_ID				(  2 )					/* Length of the TCP socket ID parameter without end-of-string. Range 0 to 11. */
#define LEN_TCP_RX_DATA_CNT				(  2 )					/* Length of the TCP RX data length parameter without end-of-string */
#define LEN_GSM_ID						( 15 )					/* Length of the GSM ID parameter without end-of-string */
#define LEN_GSM_ICCID					( 20 )					/* length of the GSM ICCID parameter without end-of-string */
#define LEN_GSM_IND						(  5 )					/* Length of the GSM tracker indication without end-of-string */
#define LEN_GSM_TIM						(  4 )					/* Length of the time stamp parameter without end-of-string */
#define LEN_GSM_BAT						(  4 )					/* Length of the GSM battery parameter without end-of-string */
#define LEN_GSM_LAT						( 12 )					/* Length of the GPS latitude parameter without end-of-string for uBlox */
#define LEN_GSM_LON						( 13 )					/* Length of the GPS longitude parameter without end-of-string */
#define LEN_GSM_ALT						(  6 )					/* Length of the GPS altitude parameter without end-of-string */
#define LEN_GSM_VEL						(  6 )					/* Length of the GPS velocity parameter without end-of-string */
#define LEN_GSM_HDG						(  6 )					/* Length of the GPS heading parameter without end-of-string */
#define LEN_GSM_FIX						(  1 )					/* Length of the GPS fix type parameter without end-of-string */
#define LEN_GSM_HDOP					(  4 )					/* Length of the GPS HDOP parameter without end-of-string */
#define LEN_GSM_LOCM					(  1 )					/* Length of the localization method without end-of-string */
#define LEN_GSM_SAT						(  2 )					/* Length of the GPS # of satellites parameter without end-of-string */
#define LEN_GSM_TEMP					(  6 )					/* Length of the GSM temperature parameter without end-of-string */
#define LEN_GSM_STATE					(  1 )					/* Length of the tracket device state without end-of-string */
#define LEN_GSM_ACCHM					(  5 )					/* Length of the GPS fix horizontal accuracy in meters without end-of-string */
#define LEN_GSM_ACCVM					(  5 )					/* Length of the GPS fix vertial accuracy in meters without end-of-string */
#define LEN_GSM_RSSI					(  2 )					/* Length of the GSM Received Signal Strength Indicator without end-of-string */
#define LEN_GSM_QUAL					(  2 )					/* Length of the GSM GPRS signal quality (BEP) without end-of-string */
#define LEN_GSM_STEPS					(  4 )					/* Length of the absolute step count without end-of-string */
#define LEN_GSM_MCC						(  3 )					/* Length of the MCC without end-of-string */ 
#define LEN_GSM_MNC                     (  3 )					/* Length of the MNC without end-of-string */
#define LEN_GSM_TAC                     (  4 )					/* Length of the TAC without end-of-string */
#define LEN_GSM_CI                      (  8 )					/* Length of the CI without end-of-string */
#define LEN_GSM_RAT                     (  1 )					/* Length of the RAT without end-of-string */
#define LEN_GSM_EARFCN                  (  4 )					/* Length of the eARFCN without end-of-string */
#define LEN_GSM_TXINST                  (  4 )					/* Length of the TX instant delay without end-of-string */
#define LEN_GSM_CHRGI                	(  3 )					/* Length of the charge current without end-of-string */	
#define LEN_GSM_EVAC_ID                	(  2 )					/* Length of the evacuation ID without end-of-string */

#define gsmCMD_QUEUE_SIZE				( 20 )					/* Length of the GSM command queue. */
				                                            
#define MAX_LEN_PARAMS				   ( 350 )					/* Maximum length of any variable parameters to be added to the AT commands. */

#define LEN_IP_ADDR						( 16 )					/* Maximum length of an IP address in ASCII. 4 groups of 8-bit integer numbers separated by '.'. */
																
#define GSM_DEFAULT_DFMAP				( 0x767ffffd )			/* Default DFMap value for enabled fields. */
#define GSM_GPS_POS_FIELDS_MASK			( 0x04032fe0 )			/* Bit mask which describes all GPS-position-related fields in the DFMAP. */
#define GSM_HELLO_FIELDS_MASK			( 0x324cc01d )			/* Bit mask which describes all fields in the DFMAP relevant for GPS recording HELLO packets. */
	
#define GSM_DEFAULT_MOD_PWR_DOWN_INT   ( 330 )					/* Default min. interval (s) between transmissions to power-down the GSM/GPS module. */
#define GPS_DEFAULT_MOD_PWR_DOWN_INT   (  10 )					/* Default min. interval (s) between transmissions to power-down the GPS module only. */	

#define GPS_PSM_CONTINUOUS				(  0 )					/* GPS PSM: Continuous mode. All PSM internal blocks are always active. Best performance,
																            but highest power consumption. */
#define GPS_PSM_BALANCED				(  1 )					/* GPS PSM: Balanced mode. GPS internal cyclic PSM module is used. No concessions concerning
																			performance but only little power saving. */
#define GPS_PSM_AGGESSIVE_ACTIVE		(  2 )					/* GPS PSM: Aggessive iun ACTIVE mode. GPS internal cyclic PSM module is used in aggressive mode,
																			but only in ACTIVE state once there had been a good fix. Good power saving in ACTIVE state. */
#define GPS_PSM_AGGESSIVE_ALWAYS		(  3 )					/* GPS PSM: Always aggessive. Best power consumption but performance is compromised. */
#define GPS_DEFAULT_PSM					( GPS_PSM_CONTINUOUS )	/* Default GPS Power save mode. */

#define GPS_RECORDING_DISABLE			(  0 )					/* Default for GPSposition recording: disabled. */		
#define GPS_RECORDING_ENABLE			(  1 )					/* GPSposition recording enabled (bitmap). */		
#define GPS_RECORDING_DEBUG				(  2 )					/* GPSposition recording debug mode: simultaneous position recording and sending (bitmap). */		

#define GSM_GPS_RECORDING_PACKET_INT	( 0xFFFF )				/* GPSposition recording packet send interval in seconds. 0 means never, 0xFFFF means always 
																   when GPS assistance data is updated. */
#define GSM_GPS_RECORDING_SLEEP_INT		( 140 * 60 )			/* GPSposition recording packet send interval in SLEEP mode in seconds. Here, the interval is set to 
																   something slightly larger than 2h. */

#define	ID_POS_DFMAP					(  0 )					/* Position of the GSM ID field in the DFMAP. */
#define	BCN_POS_DFMAP					( 20 )					/* Position of the locator beacon field in the DFMAP. */
#define	GSMDEB_POS_DFMAP				( 22 )					/* Position of the GSM debug data field in the DFMAP. */
#define	FTL_POS_DFMAP					( 25 )					/* Position of the TL beacon field in the DFMAP. */
#define	GPSDEB_POS_DFMAP				( 26 )					/* Position of the GPS debug data field in the DFMAP. */
#define	EVAC_ID_POS_DFMAP				( 28 )					/* Position of the EVAC_ID field in the DFMAP. */
#define	RFON_POS_DFMAP					( 29 )					/* Position of the RF-ON field in the DFMAP. */
#define	UUID_BCN_POS_DFMAP				( 30 )					/* Position of the UUID-filtered locator beacon field in the DFMAP. */

#define	POS_STORE_LEN					( 10 )					/* Depth of the location store. */

#define	PTIM_POS_PDFMAP					(  0 )					/* Position of the PTIM field in the PDFMAP. */
#define	PSTATE_POS_PDFMAP				(  1 )					/* Position of the PSTATE field in the PDFMAP. */
#define	PLAT_POS_PDFMAP					(  2 )					/* Position of the PLAT field in the PDFMAP. */
#define	PLON_POS_PDFMAP					(  3 )					/* Position of the PLON field in the PDFMAP. */
#define	PALT_POS_PDFMAP					(  4 )					/* Position of the PALT field in the PDFMAP. */
#define	PHDG_POS_PDFMAP					(  5 )					/* Position of the PHDG field in the PDFMAP. */
#define	PFIX_POS_PDFMAP					(  6 )					/* Position of the PFIX field in the PDFMAP. */
#define	PHDOP_POS_PDFMAP				(  7 )					/* Position of the PHDOP field in the PDFMAP. */
#define	PLOCM_POS_PDFMAP				(  8 )					/* Position of the PLOCM field in the PDFMAP. */
#define	PSV_POS_PDFMAP					(  9 )					/* Position of the PSV field in the PDFMAP. */
#define	PACCH_POS_PDFMAP				( 10 )					/* Position of the PACCH field in the PDFMAP. */
#define	PACCV_POS_PDFMAP				( 11 )					/* Position of the PACCV field in the PDFMAP. */
#define	PBCN_POS_PDFMAP					( 12 )					/* Position of the locator beacon field in the PDFMAP. */
#define	PSTEPS_POS_PDFMAP				( 16 )					/* Position of the PSTEPS field in the PDFMAP. */
#define	PBCN_UUID_POS_PDFMAP			( 17 )					/* Position of the UUID-filtered locator beacon field in the PDFMAP. */
	
	
#define MOD_SHUTDWN_DELAY_POS	(  2 * portTICKS_PER_SEC )		/* Length of window to allow the server sending new commands after termination of the 
																   last position transmission transaction. The GSM/GPS module is shut down after this delay. */
#define MOD_SHUTDWN_DELAY_CMD	( 30 * portTICKS_PER_SEC )		/* Length of window to allow the server sending new commands after termination of the 
																   last server command transaction. The GSM/GPS module is shut down after this delay. */
#define MOD_INTER_PKT_DELAY_CMD			( 10 )					/* Length of window (n* 1sec) to allow the server sending new commands between base position packet
																   and FTL packets.  */
#define MAX_MODULE_START_ATTPTS			(  5 )					/* Number of GSM module start attempts before giving up. */
#define GSM_PWRDWN_RETRY				(  3 )					/* Number of times a power-down is attempted before hard
																   re-setting the GSM/GPS module. */
#define GSM_PACKET_RETRY_LIM			(  3 )					/* Number of times a re-transmission of a packet is attempted before hard 
																   re-setting the GSM/GPS module. */

#define FWUPD_BATT_THRES				( ( short )2661 )		/* Battery voltage threshold for allowing FTP downloads in Volts: 3.9V / ADC_GAIN_VBAT / ADC_REF * ADC_MAX_VALUE = 2661 */
#define VGSM_OFF_THRES					( 1452 )				/* VGSM is below 1.5V (1.5V * ADC_MAX_VALUE / ADC_VREF / ADC_GAIN_VGSM = 1.5 * 4095 / 0.6 / 7.05 = 1452). */
																
#define DFMAP_IND						(  2 )					/* Mask for the indications bit field in the DFMAP. */														   
#define IND_ALERT			  			( 0x00001 )				/* Flag in the tracker indication bit map for an alert message. */
#define IND_SOS				   			( 0x00002 )				/* Flag in the tracker indication bit map for an SOS message. */
#define IND_ERROR			   			( 0x00004 )				/* Flag in the tracker indication bit map for an error. */
#define IND_BATT_FULL					( 0x00008 )				/* Flag in the tracker indication bit map for battery fully charged. */
#define IND_REATTACH					( 0x00010 )				/* Flag in the tracker indication bit map indicating a GSM re-attach. */
#define IND_NVDS_REINIT					( 0x00020 )				/* Flag in the tracker indication bit map indicating an NVDS reinitialisation. */
#define IND_FOREIGN_ALERT				( 0x00040 )				/* Flag in the tracker indication bit map indicating a foreign alert. */
#define IND_FOREIGN_SOS					( 0x00080 )				/* Flag in the tracker indication bit map indicating a foreign SOS. */
#define IND_EXTREME_TEMP				( 0x00100 )				/* Flag in the tracker indication bit map for extreme temperatures. */
#define IND_CHARGING					( 0x00200 )				/* Flag in the tracker indication bit map set if the device is on the charger. */
#define IND_NOABN_BCN					( 0x00400 )				/* Flag in the tracker indication bit map set if the device has seen a no abnormal position detection beacon. */
#define IND_DANGER_BCN					( 0x00800 )				/* Flag in the tracker indication bit map set if the device has seen a danger zone beacon. */
#define IND_PRIVATE_BCN					( 0x01000 )				/* Flag in the tracker indication bit map set if the device has seen a private zone beacon. */
#define	IND_VIBR_ON						( 0x02000 )				/* Flag in the tracker indication bit map set if the device had activated the vibration motor. */
#define	IND_ABNDET_OFF					( 0x04000 )				/* Flag in the tracker indication bit map set if the abnormal position (tilt) detection is off by user command. */
#define IND_IMMOBILITY_BCN				( 0x08000 )				/* Flag in the tracker indication bit map set if the device has seen an immobility detection beacon. */
#define IND_AUTOTEST					( 0x10000 )				/* Flag in the tracker indication bit map set if the device is on charger and in autotest. */

#define FTP_MAX_FILNAME					( 32 )					/* Maximum length of update FW path-/filename. */
#define FWUPD_MAX_SRV_URL				( 60 )					/* Maximum length of the URL for the FW FTP server. */
#define FTP_MAX_UID						( 10 )					/* Maximum length of the UID for the FW FTP server. */
#define FTP_MAX_PW						( 10 )					/* Maximum length of the password for the FW FTP server. */
#define FTP_MAX_CFG						( 48 )					/* Maximum length of the FTP configuration string. */
				
#define FWUPD_ESTABL_RETRY				( 10 )					/* Number of times a connection establishment with the server is attempted before rolling-back the FW. */
		
#define SEC_ENABLE						( true )				/* Default value for using TLS over TCP. */
			
#define MAX_GPS_ASSISTED_DATA_DOWNLOAD_ATTEMPTS	( 2 )			/* Maximum number of attempts to download GPS assisted data from the ublox server. */

#define AN_SYNC_CHAR_1					( 0xb5 )				/* ublox sync character. */
#define AN_SYNC_CHAR_2					( 0x62 )				/* ublox sync character. */

#define RAT_LTE_M1						( 7 )					/* LTE-M1 RAT. */
#define RAT_LTE_NB1						( 8 )					/* LTE-NB1 (NB-IoT) RAT. */
#define RAT_GPRS						( 9 )					/* GPRS RAT. */

																/* Bitmask for defaults RATs:
																		[12]		if set, the RAT selection is not to be changed by the application
																		[11:8]		1st AcT (LTE-M1)
																		[ 7:4]		2nd AcT (GPRS)
																		[ 3:0]		3rd AcT (LTE-NB1)	*/
// #define DEFAULT_RATS					( 0x0790 )
#define DEFAULT_RATS					( 0x0700 )				/* DEBUG DEBUG DEBUG Default RAT for Sierra Wireless SIM V4 LWPA tests. */

#define MODULE_VER_LEN					( 50 )					/* Length of the QTEL module version string. */

/* Typedef for a structure field used for generating the GSM/GPS module init sequence. */
struct xAT_CMD	
{	
	const signed char 			*pcAtCommand;					/* Pointer to AT command string. */
	void						( *prvAddParam )( void );		/* Function to add the parameter; NULL if none. */
	enum xAT_MSG_ID				xExpectAtRespMsgId[ 2 ];		/* Expected AT response message IDs. */
	enum xAT_MSG_ID				xErrorAtRespMsgId;				/* AT response message ID in case of errors. */
	unsigned portBASE_TYPE		xRepetitions;					/* Number of retries. */
	unsigned int				uiTimeOut;						/* Response time-out in ms. */
};

extern const struct xAT_CMD xAtLstFreeU;

/* States of the GSM/GPS module.
   Coded as bit fields:
		bit 0:	0 - Module is off.
				1 - Module is powered up.
		bit 1:	0 - GPS is off.
				1 - GPS is on.
		bit 2:	0 - PDP context is deactivated.
				1 - PDP context is activated.
		bit 3:	0 - No TCP connection established.
				1 - TCP connection is established.
		bit 4:	0 - Module is in IDLE state (RTS off).
				1 - Module is in ACTIVE state.
*/

#define	GSM_PWR_OFF			0x00								/* Power is off. */
#define	GSM_PWR_ON			0x01								/* Power is on. */
#define	GSM_PWR_MSK			0x01								/* Bit mask for the power status. */
						
#define	GPS_PWR_OFF			0x00								/* GPS is off. */
#define	GPS_PWR_ON			0x02								/* GPS is on. */
#define	GPS_PWR_MSK			0x02								/* Bit mask for the GPS status. */
						
#define	GSM_PDP_DEACT		0x00								/* PDP context is deactivated (no packet data connection). */
#define	GSM_PDP_ACT			0x04								/* PDP context is activated, packet data connection is available. */
#define	GSM_PDP_ACT_MSK		0x04								/* Bit mask for the PDP activation status. */
						
#define	GSM_TCP_DISC		0x00								/* No TCP connection established. */
#define	GSM_TCP_CONN		0x08								/* TCP connection is established. */
#define	GSM_TCP_CONN_MSK	0x08								/* Bit mask for the TCP connection status. */

#define	GSM_TLS_DISC		0x00								/* No TLS connection established. */
#define	GSM_TLS_CONN		0x10								/* TLS connection is established. */
#define	GSM_TLS_CONN_MSK	0x10								/* Bit mask for the TLS connection status. */
						
#define GSM_IDLE			0x00								/* Module is in IDLE state. */
#define GSM_ACTIVE		 	0x20								/* Module is in ACTIVE or CONNECTED state. */
#define GSM_ACTIVE_MSK	   	0x20								/* Bit mask for the ACTIVE status. */

/* GSM module powerdown type. */
enum xGSM_PWRDOWN_TYPE
{
	RESET_ONLY,					/* The GSM module is only reset but not power-cycled. */
	SOFT_POWER_OFF,				/* Power-cycle the GSM module. Do a soft shut-down, i.e. close all connection properly. */
	HARD_POWER_OFF				/* Power-cycle the GSM module. Do a hard shut-down without attempting to send any commands to the module. 
								   This option is to be used in case the module became unresponsive. */
};

/* GSM module shutdown request type. This request type determines if a connection window for further server commands will be kept 
   open. */
enum xGSM_SHUTDOWN_REQ_TYPE
{
	FROM_POSITION_UPDATE,		/* The GSM module power-down is requested following a position update to the server (no subsequent command window). */
	FROM_SERVER_COMMAND,		/* The GSM module power-down is requested following termination of a server command transaction (subsequent command window). */
	FORCE_SHUTDOWN				/* The GSM module power-down is requested while position recording is on. */
};

/* Commands from the CTRL task or the remote server to the GSM task. */
enum xGSM_CMD
{
	GSM_SEND_ALERT_SOS,			/* Send immediately an alert or SOS message over the air to the server. */
	GSM_SEND_FOREIGN_ALERT_SOS,	/* Send immediately a foreign alert or SOS message over the air to the server. */
	GSM_SEND_POSITION,			/* Send a position packet to the server. */
	GSM_POS_INT_UPDATE,			/* The time interval to send a position message to the server has been updated. */
	GSM_SERVER_CMD,				/* Received a command from the remote server. The server command might already have been decoded 
								   or might still be needing to be fetched. */
	GSM_SHUTDOWN_MODULE,		/* Shutdown the GSM/GPS module. */
	GSM_ACC_VIBR_TEST_DONE,		/* Informs the GSM task that the accelerometer/vibrator test, which runs in the CTRL task, has finished. */
	GSM_IN_PRIVATE_ZONE, 		/* Stop reporting location. The module is in a private zone. */
	GSM_IN_STANDBY				/* The module has been set to STANDBY state. Stop all GSM activity. */
};

/* AT command success codes. */
enum xAT_RESULT
{
	AT_SUCCESS,					/* The reponse to the AT command / sequence of commands was correct. */
	AT_RESPONSE_TO,				/* Time-out in response to the AT command. */
	AT_RESPONSE_ERR,			/* The GSM/GPS module returned the defined response error. */
	AT_MODULE_ERR,				/* The GSM/GPS module returned an error (usually CME or CMS error). */
	AT_MODULE_STUCK				/* The GSM/GPS module appears to be stuck, i.e. it does not respond to simple AT requests. */
};

/* AT command success codes. */
enum xGPRS_CONNECT_RESULT
{
	CONNECT_SUCCESS,			/* Attached successfully to the network */
	CONNECT_FAILED,				/* The connection attempt failed for another reason than finding no BTS to attach to. */
	CONNECT_NO_NW				/* The cellular module did not find any network to attach to. */
};

/* States when decoding the GPS assistance data. */
enum xAN_PHASE
{
	HUNT_FOR_QUOTE_1,			/* Hunt for the first quote. */
	HUNT_FOR_QUOTE_2,			/* Hunt for the second quote. */
	HUNT_FOR_QUOTE_3,			/* Hunt for the third quote which indicates the file start. */
	HUNT_FOR_SYNC_CHAR_1,		/* Parsing the AN file: looking for the SYNC1 character. */
	HUNT_FOR_SYNC_CHAR_2,		/* Parsing the AN file: looking for the SYNC2 character. */
	DATA_TRANSFER				/* Transferring the binary data. */	
};

/* Typedef for a GSM entry in the position store. */
struct xGSM_POS_ENTRY	
{	
	signed char 			cPosTIM  [ LEN_GSM_TIM  	+ 1	];	/* Time stamp. */
	signed char 			cPosSTATE[ LEN_GSM_STATE  	+ 1 ];	/* Device state. */
	signed char 			cPosSTEPS[ LEN_GSM_STEPS  	+ 1	];	/* Step count. */
};

/* Flags (bitmasks) for communicating with the bootloader via the GPREGRET registers. */
/* GPREGRET */
#define BOOTLOADER_DFU_GPREGRET_MASK	( 0xF0 )
#define BOOTLOADER_DFU_GPREGRET			( 0xB0 )
#define BOOTLOADER_DFU_START_BIT_MASK   ( 0x01 )

#define BL_UPDATE_FW					( 0x02 )
#define BL_UPDATE_SD					( 0x04 )
#define BL_UPDATE_BL					( 0x08 )

#define BOOTLOADER_DFU_START_MASK    	( BOOTLOADER_DFU_GPREGRET_MASK | BOOTLOADER_DFU_START_BIT_MASK )
#define BOOTLOADER_DFU_START    		( BOOTLOADER_DFU_GPREGRET | BOOTLOADER_DFU_START_BIT_MASK )

/* GPREGRET2 */
#define BL_FW_UPDATED					( 0x01 )
#define BL_NEW_FW_NOT_FUNCTIONAL		( 0x02 )
#define BL_FW_ROLLED_BACK				( 0x04 )
/*-----------------------------------------------------------*/

/* Public function prototypes. */
extern void vGsmInit( UBaseType_t uxPriority );

/* Return the TCP connection status of the cellular module. If returned true, the module is supposed to be on and connected. */
extern bool bGetTcpConnected( void );

/* Request the Parser to store the next string from the GSM module in the version string buffer. */
extern void vReqStorage( void );

/* Count the number of times a GPRS connection attempt fails. */
extern void vCntCnxFail( void );

/* Update the module status on an unsolicited PDP context deactivation. */
extern void vUnsolicitedPdpDeactivate( unsigned short usStrgIdx );

/* Update the module status on an unsolicited TCP socket close. */
extern void vUnsolicitedTcpSocketClose( unsigned short usStrgIdx );

/* Wait for a specific response from the GSM/GPS module. If the response is 
   not received within xGsmBlockTime ticks, the function returns false. 
   Parameters:
		xExpectedMsg		ID of expected AT message
		xErrorAtRespMsgId	ID of error AT message (additionally to CME or CMS error)
		xWaitTime			wait time for message in ticks 
   Returns an AT_RESULT. 
*/
extern enum xAT_RESULT xReceiveResponse( enum xAT_MSG_ID xExpectedMsg0, enum xAT_MSG_ID xExpectedMsg1, enum xAT_MSG_ID xErrorAtRespMsgId, TickType_t xWaitTime );

/* Power-down the GSM/GPS module. */
extern void vGSMModulePowerDown( enum xGSM_PWRDOWN_TYPE xType );

/* Get the GSM module state. */
extern portBASE_TYPE xGetGsmModuleState( void );

/* Checks if there are enough BLE localiser beacons received to replace a GPS fix by a BLE server-supported fix. */
extern bool bEnoughBleBcnsForFix( void );

/* Check, if the packet currently scheduled for sending is a simple 'Hello' packet while in GPS recording mode. */
extern bool bIsHelloPacketInGpsRecording( void );

/* Count the number of received satellites. */
extern unsigned portBASE_TYPE uxNumSvnEntries( void );

/* Read the battery temperature, convert the result to ASCII and store it in the ASCII field pointed to by pcStrg. */
extern void vGetBatteryTemperature( signed char *pcStrg );

/* Send one AT command to the GSM/GPS module. Take into account any repetitions,
   response time-outs and expected responses / error responses. */
extern enum xAT_RESULT xSendAtCommand( const struct xAT_CMD *pxAtCmd, bool bCrLf, bool bReportTO );

/* Send a batch of AT commands to the GSM module. If xGsmModuleColdStart is set to false, commands which are marked with 
   'required form warm start' = false will not be executed. */
enum xAT_RESULT xSendBatchAtCommand( const struct xAT_CMD *xAtGSMCmdSeq, unsigned portBASE_TYPE xNumCmds, bool bReportTO );
/*-----------------------------------------------------------*/

/* Global variables. */
/* Mutex handle to protect GSM data access. */
extern SemaphoreHandle_t	xMutexGsmData;		

/* GSM command queue handle. The queue is read in the GSM task and filled in the CTRL task. */
extern QueueHandle_t 		xGsmCmdQueue; 

/* Initialisation values for GSM GPRS and TCP connection parameters. */
extern const signed char 	cGsmGPRSDefaultConnParam[];	
extern const signed char 	cGsmTCPDefaultConnParam[];	
extern const signed char 	cGsmSecTCPConnParam[];
extern const signed char 	cGpsTCPStreamingServer[];
extern const signed char 	cGPSLocEstSrvAddr[];

/* Initialisation values for FTP connection parameters. */
extern const signed char 	cFWDefaultFileName[];	
extern const signed char 	cFTPServerDefaultURL[];	
extern const signed char 	cFTPServerDummyLogin[];	
extern const signed char 	cFTPDefaultCfg[];	


/* Parameters received from the GSM/GPS module. Implemented as global variables
   to save program space otherwise need for encapsulation. */
extern signed char 			cGsmTcpSocketId   [ LEN_TCP_SOCKET_ID  + 1 ];

																	/* field  0: tracker identifier (IMEI), in NVDS. */
extern signed char          cGsm_IND[	LEN_GSM_IND    + 1 ];		/* field  2: tracker indication */
extern signed char          cGsm_TIM[  	LEN_GSM_TIM    + 1 ];		/* field  3: time stamp */
extern signed char          cGsm_BAT[  	LEN_GSM_BAT    + 1 ];		/* field  4: battery level in Volt */
extern signed char          cGsm_LAT[  	LEN_GSM_LAT    + 1 ];		/* field  5: latitude */
extern signed char          cGsm_LON[  	LEN_GSM_LON    + 1 ];		/* field  6: longitude */
extern signed char          cGsm_ALT[  	LEN_GSM_ALT    + 1 ];		/* field  7: altitude (4 digits + sign) */
extern signed char          cGsm_VEL[  	LEN_GSM_VEL    + 1 ];		/* field  8: velocity */
extern signed char          cGsm_HDG[  	LEN_GSM_HDG    + 1 ];		/* field  9: heading */
extern signed char          cGsm_FIX[  	LEN_GSM_FIX    + 1 ];		/* field 10: GPS fix type */
extern signed char          cGsm_HDOP[ 	LEN_GSM_HDOP   + 1 ];		/* field 11: Horizontal Dilution of Precision */
extern signed char          cGsm_LOCM[ 	LEN_GSM_LOCM   + 1 ];		/* field 12: Localization method */
extern signed char          cGsm_SAT[  	LEN_GSM_SAT    + 1 ];		/* field 13: satellites in view and used */
extern signed char          cGsm_TEMP[ 	LEN_GSM_TEMP   + 1 ];		/* field 14: device temperature	*/
extern signed char          cGsm_STATE[	LEN_GSM_STATE  + 1 ];		/* field 15: device state */
extern signed char          cGsm_ACCHM[	LEN_GSM_ACCHM  + 1 ];		/* field 16: GPS fix horizontal accuracy */
extern signed char          cGsm_ACCVM[ LEN_GSM_ACCVM  + 1 ];		/* field 17: GPS fix vertical accuracy */
extern signed char          cGsm_RSSI[ 	LEN_GSM_RSSI   + 1 ];		/* field 18: GSM RSSI */
extern signed char          cGsm_QUAL[ 	LEN_GSM_QUAL   + 1 ];		/* field 19: GSM bit error quality indicator */

extern signed char 			cGsm_STEPS[ LEN_GSM_STEPS  + 1 ];		/* field 21: Absolute step count */

extern signed char 			cGsm_MCC[   LEN_GSM_MCC    + 1 ];		/* field 22a: GSM debug data: MCC Mobile Country Code. */
extern signed char 			cGsm_MNC[   LEN_GSM_MNC    + 1 ];		/* field 22b: GSM debug data: MNC Mobile Network Code. */
extern signed char 			cGsm_TAC[   LEN_GSM_TAC    + 1 ];		/* field 22c: GSM debug data: TAC Tracking Area Code. */
extern signed char 			cGsm_CI[    LEN_GSM_CI     + 1 ];		/* field 22d: GSM debug data: CI Cell Identification. */
extern signed char 			cGsm_RAT[   LEN_GSM_RAT    + 1 ];		/* field 22e: GSM debug data: RAT Radio Access Technology. */
extern signed char 			cGsm_EARFCN[LEN_GSM_EARFCN + 1 ];		/* field 22f: GSM debug data: EARFCN Extended Absolute Radio Frequency Channel Number. */
extern signed char 			cGsm_TXINST[LEN_GSM_TXINST + 1 ];		/* field 22g: GSM debug data: TxInst Delay in seconds since transmit instant. */
extern signed char 			cGsm_CHRGI[ LEN_GSM_CHRGI  + 1 ];		/* field 22h: GSM debug data: ChrgCurr Charge current if changring is throttled due to temperature. */

extern signed char 			cGsm_FID[   LEN_GSM_ID     + 1 ];		/* field 23: foreign distress tracker indication */
extern signed char 			cGsm_FTIM[  LEN_GSM_TIM    + 1 ];		/* field 24: foreign distress tracker reception time stamp */

extern unsigned short		usGpsNoisePerMS;						/* field 26: Noise Level as measured by the GPS Core. */
extern unsigned short		usGpsAgcCnt;							/*           GPS AGC Monitor. */
extern unsigned char		ucGpsHwFlags;							/* 			 GPS HW flags. */
extern unsigned char		ucGpsJamInd;							/* 			 GPS CW Jamming indicator. */
extern unsigned char		ucGpsOfsI;								/* 			 GPS Imbalance of I-part of complex signal. */
extern unsigned char		ucGpsMagI;								/* 			 GPS Magnitude of I-part of complex signal. */
extern unsigned char		ucGpsOfsQ;								/* 			 GPS Imbalance of Q-part of complex signal. */
extern unsigned char		ucGpsMagQ;								/* 			 GPS Magnitude of Q-part of complex signal. */

/* Copies of the above data fileds which are going to be sent out via BLE beacons.
   The only difference with the above data fileds is that these here are set to 0 only
   after having been sent out via BLE. */
extern signed char			cGsm_BLE_LAT  	[ LEN_GSM_LAT    + 1 ];	/* field  5: latitude */
extern signed char			cGsm_BLE_LON  	[ LEN_GSM_LON    + 1 ];	/* field  6: longitude */
extern signed char			cGsm_BLE_HDOP 	[ LEN_GSM_HDOP   + 1 ];	/* field 11: Horizontal Dilution of Precision */
extern signed char 			cGsm_BLE_FIX  	[ LEN_GSM_FIX    + 1 ];	/* field 10: GPS fix type */
extern signed char			cGsm_BLE_LOCM 	[ LEN_GSM_LOCM   + 1 ];	/* field 12: Localization method */
extern signed char			cGsm_BLE_SAT  	[ LEN_GSM_SAT    + 1 ];	/* field 13: satellites in view and used */
extern signed char			cGsm_BLE_TEMP 	[ LEN_GSM_TEMP   + 1 ];	/* field 14: device temperature	*/
extern signed char			cGsm_BLE_STATE	[ LEN_GSM_STATE  + 1 ];	/* field 15: device state */
extern signed char			cGsm_BLE_ACCHM 	[ LEN_GSM_ACCHM  + 1 ];	/* field 16: GPS fix horizontal accuracy */
extern signed char			cGsm_BLE_RSSI 	[ LEN_GSM_RSSI   + 1 ];	/* field 18: GSM RSSI */
extern signed char			cGsm_BLE_QUAL 	[ LEN_GSM_QUAL   + 1 ];	/* field 19: GSM bit error quality indicator */
extern signed char 			cGsm_BLE_MCC  	[ LEN_GSM_MCC    + 1 ];	/* field 22a: GSM debug data: MCC Mobile Country Code. */
extern signed char 			cGsm_BLE_MNC  	[ LEN_GSM_MNC    + 1 ];	/* field 22b: GSM debug data: MNC Mobile Network Code. */
extern signed char 			cGsm_BLE_TAC  	[ LEN_GSM_TAC    + 1 ];	/* field 22c: GSM debug data: TAC Tracking Area Code. */
extern signed char 			cGsm_BLE_CI   	[ LEN_GSM_CI     + 1 ];	/* field 22d: GSM debug data: CI Cell Identification. */
extern signed char 			cGsm_BLE_RAT  	[ LEN_GSM_RAT    + 1 ];	/* field 22e: GSM debug data: RAT Radio Access Technology. */
extern signed char 			cGsm_BLE_EARFCN	[ LEN_GSM_EARFCN + 1 ];	/* field 22f: GSM debug data: EARFCN Extended Absolute Radio Frequency Channel Number. */
extern signed char 			cGsm_BLE_TXINST	[ LEN_GSM_TXINST + 1 ];	/* field 22g: GSM debug data: TxInst Delay in seconds since transmit instant. */
extern signed char 			cGsm_BLE_CHRGI	[ LEN_GSM_CHRGI  + 1 ];	/* field 22h: GSM debug data: ChrgCurr Charge current if changring is throttled due to temperature. */


/* Buffer for server command parameters. The buffer is filled once a server command is recognised in the 
   parser task. When the command is treated in the GSM task, the buffer is read and the parameters 
   used to execute the command. */
extern signed char 			cServerCmdParams[ MAX_LEN_PARAMS + 1 ];

/* Boolean which inhibits waiting for a fix on the next transmission slot. This mechanism is used when CTRL detects
   an alert and wants to immediately push a packet regardless of the position fix status. 
   This variable might be set anytime by CTRL. */
extern volatile bool 		bPushPacketImmediately;

/* GPS/GSM module state: off requiring cold or warm start, TCP disconnected with GPS on or off, TCP connected with GPS on or off. */
extern portBASE_TYPE		xModuleState;

/* Boolean which indicates that the device is in test mode, i.e. sending position packets and scanning for
   BLE beacons even though it is on the charger. */
extern bool					bTestMode;

/* Position store write index. The indices are used for both the GPS and the BLE position stores. */
extern unsigned portBASE_TYPE uxPosStoreWrIdx;
/*-----------------------------------------------------------*/

#endif