/*
 * Tracker Firmware
 *
 * GSM task
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "tracker.h"

/* nRF SDK files. */
#include "nordic_common.h"
#include "nrf.h"
#include "ble_gap.h"
#undef  NRF_LOG_DEFAULT_LEVEL
#define NRF_LOG_DEFAULT_LEVEL		NRF_LOG_DEFAULT_DEBUG
#define NRF_LOG_MODULE_NAME 		GSM
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_power.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/* Device specific include files. */
#include "custom_board.h"
#include "drv_uart.h"
#include "drv_accelerometer.h"
#include "drv_adc.h"
#include "drv_gps.h"
#include "drv_nvm.h"
#include "drv_pwm.h"
#include "drv_vibrator.h"

#include "config.h"
#include "ble_adreport.h"
#include "ble_cmd.h"
#include "ble_ctrl.h"
#include "ble_main.h"
#include "ctrl.h"
#include "charger.h"
#include "evacuation.h"
#include "gps.h"
#include "gsm.h"
#include "drv_led.h"
#include "motiondet.h"
#include "rtc.h"
#include "parser.h"
#include "trace.h"
#include "tracemsg.h"
#include "utils.h"
#include "version.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* The transmit task as described at the top of the file. */
static portTASK_FUNCTION_PROTO( vGSMTask, pvParameters );

/* Initialise the GSM task. */
void vGsmInit( UBaseType_t uxPriority );

/* Start the GPS PVT output. */
enum xAT_RESULT xStartGPSOutput( void );

/* Check if battery voltage and temperature allow for booting the GSM module. If not, wait until both
   conditions are good. */
void vCheckVoltageAndTemperature( void );

/* Wait for a rising edge in the RxD signal from the GSM module indicating that the module has finished booting. */
bool bWaitModuleBooted( void );

/* Set the QTEL one-time configuration, if not already done before. */
enum xAT_RESULT xOneTimeConfigureQTEL( void );

/* Select a RAT or sequence of preferred RATs. */
enum xAT_RESULT xSelectRAT( void );

/* Start and configure the GSM/GPS module. */
enum xGPRS_CONNECT_RESULT xStartCellularModule( void );

/* Perform the base starty-up configuration of QTEL. */
enum xAT_RESULT xConfigureQTEL( void );

/* Prepare the cellular module for a reboot. */
void prvPrepareCellularModuleReboot( enum xGSM_PWRDOWN_TYPE xType );

/* Test if the PDP context is still active, i.e. the packet data connection is still intact. 
   If not, try to re-establish the PDP context and thus the packet switched data connection.
   Returns true if PDP is established.
*/
bool bTestAndReActivatePDP( void );

/* Test if the TCP connection is still established.
   If not, try to re-establish TCP.
*/
enum xAT_RESULT xTestAndReConnectTCP( void );

/* Establish TCP connection. 
   The function assumes that the GSM/GPS module is already powered up and configured.
   Returns true if several attempts to establish connection finally succeeded.
*/
bool bStartGsmTCPConn( void );

/* Disable the module's sleep mode. */
enum xAT_RESULT xModuleDisableSleepMode( void );

/* Enable the module's sleep mode. */
enum xAT_RESULT xModuleEnableSleepMode( void );

/* Return the module's sleep state. Returns true if in sleep mode. */
bool xModuleGetSleepMode( void );

/* Request the module to go to ACTIVE mode. */
void vModuleExitPSM( void );

/* Return true if the module is in power-save mode. */
bool bCheckModulePowerSaveMode( void );

/* Update the module status on an unsolicited PDP context deactivation. */
void vUnsolicitedPdpDeactivate( unsigned short usStrgIdx );

/* Update the module status on an unsolicited TCP socket close. */
void vUnsolicitedTcpSocketClose( unsigned short usStrgIdx );

/* Power-up the GSM/GPS module. */
void prvGSMModulePowerUp( void );

/* Power-down the GSM/GPS module. */
void vGSMModulePowerDown( enum xGSM_PWRDOWN_TYPE xType );

/* Return the TCP connection status of the cellular module. If returned true, the module is supposed to be on and connected. */
bool bGetTcpConnected( void );

/* Timer callback for GSM/GPS module shutdown. 
   The timer is started once either the position transmit has finished or a server command transaction is terminated.
   During the timer expiry delay, the server is given the oportunity to send more commands to the module. In that case,
   the timer is stopped and relaunched after the new transaction is terminated.
*/
void prvModuleShutdownCallback( TimerHandle_t xTimer );

/* Invalidate GSM data once they have been sent to the server. */
void vInvalidateGsmData( void );

/* Wait for a specific message from the GSM/GPS module. */
enum xAT_RESULT xReceiveResponse( enum xAT_MSG_ID xExpectedMsg0, enum xAT_MSG_ID xExpectedMsg1, enum xAT_MSG_ID xErrorMsg, TickType_t xGsmBlockTime );

/* Send one AT command to the GSM/GPS module. Take into account any repetitions,
   response time-outs and expected responses / error responses. */
enum xAT_RESULT xSendAtCommand( const struct xAT_CMD *pxAtCmd, bool bCrLf, bool bReportError );

/* Send a batch of AT commands to the GSM module. If xGsmModuleColdStart is set to false, commands which are marked with 
   'required form warm start' = false will not be executed. */
enum xAT_RESULT xSendBatchAtCommand( const struct xAT_CMD *xAtGSMCmdSeq, unsigned portBASE_TYPE xNumCmds, bool bReportTO );

/* Request an initial geoposition estimation from the server based on the cellular connnection data. */
void vRequestInitialPosition( void );

/* Request the assistance data from the ublox server. */
bool bRequestAssistanceData( void );

/* Invalidate all data fields to be sent to the server. This function is called as soon as 
   a tracker packet has been sent to the server. Doing this allows the GSM task to see which fields have really
   been updated since the last packet. */
void prvInvalidateData( void );

/* Get battery voltage measurement. */
void vGetBatteryVoltage( void );

/* Count the number of received satellites. */
unsigned portBASE_TYPE uxNumSvnEntries( void );

/* Read the battery temperature, convert the result to ASCII and store it in the temperature field. */
void vGetBatteryTemperature( signed char *pcStrg );

/* Format and send the location beacon BCN field. */
void vSendBCNField( volatile struct xBLE_LOC_BEACON *pxBleLocBcn, enum xBLE_BCN_FORMAT xBleBcnFormat );

/* Format and send the GPS satellite reception quality field. */
void vSendGpsSvnField( void );

/* Calculate location beacon TL field length. */
unsigned short usCalculateFTLLength( unsigned portBASE_TYPE uxTblIdx );

/* Format and send the TL beacon FTL field. */
void vSendFTLField( unsigned portBASE_TYPE	uxTblIdx );

/* Fill in a bit in an ASCII-character field. */
void vSetFieldBit( bool bCondition, unsigned portBASE_TYPE uxBitMask, signed char *pcField, unsigned portBASE_TYPE *uxIdx, bool bLastBit );

/* Depending on clear TCP or TLS, prepare the data transmission to the server. */
enum xAT_RESULT prvPrepareTCPWrite( void );

/* Send a position message to the server. */
enum xAT_RESULT prvSendBasePacket( void );

/* Send one or more FTL message to the server if there are any FTL fields pending. */
enum xAT_RESULT prvSendFtlPackets( void );

/* Remove all entries in the GPS recording positon store. */
void vDeletePosStore( void );

/* Send one or more packets containing one position store entry each to the server. */
enum xAT_RESULT prvSendPosStoreEntryPacket( void );

/* Check if conditions for shutdown of the GSM/GPS module are met. */
void vCheckConditionsAndShutdownModule( enum xGSM_SHUTDOWN_REQ_TYPE xGsmShutdownReqType );

/* Check is GPS assisted Data is outdated. Returns true, if data is still valid. */
bool bCheckAssistedDataValid( void );

/* Checks if there are enough BLE localiser beacons received to replace a GPS fix by a BLE server-supported fix. */
bool bEnoughBleBcnsForFix( void );

/* Check if sufficient time remains until the next position transmission to shutdown the GPS module. */
void vCheckConditionsAndShutdownGPS( void );

/* Update the timer interval to align with the device state. */
void prvUpdateTransmissionTimer( unsigned long ulNewPositionInterval );

/* Download a new set of GPS assistance data and send it to the nRF. */
bool bDownloadGpsAssistanceData( void );

/* Start the GPS module and get a GPS fix. */
bool bGetGpsPosition( void );

/* Check, if GPS recording is allowed and, if yes, record the position. */
bool bRecordPosition( void );

/* Check, if the packet currently scheduled for sending is a simple 'Hello' packet while in GPS recording mode. */
bool bIsHelloPacketInGpsRecording( void );

/* Get the GSM module state. */
portBASE_TYPE xGetGsmModuleState( void );

/* Estimate the time to the next Hello packet in GPS recording mode. */
TickType_t xGetTimeToNextHello( void );

/* Transmit a position packet to the server. Make sure the packet gets delivered. Handle GSM/GPS module power-down. */
void prvTransmitPosition( void );

/* Send a raw packet to the server. */
enum xAT_RESULT prvSendRawPacket( const signed char *pcServerRespROM, signed char *cServerRespRAM, signed char *cServerRespNVDS  );

/* Return a hexadecimal byte parameter read from NVDS to the server in a raw packet. */
enum xAT_RESULT prvSendByteHexNvdsParameter( const signed char *pcRespStrg, unsigned char *ucNVDSAddress );

/* Send a hexadecimal parameter value to the server. */
enum xAT_RESULT prvSendShortHexNvdsParameter( const signed char *pcRespStrg, unsigned short *usNVDSAddress );

/* Send a hexadecimal parameter value divided by 10 to the server. */
enum xAT_RESULT prvSendShortHexNvdsParameterDiv10( const signed char *pcRespStrg, unsigned short *usNVDSAddress );

/* Return a long hexadecimal parameter read from NVDS to the server in a raw packet. */
enum xAT_RESULT prvSendLongHexNvdsParameter( const signed char *pcRespStrg, unsigned long *ulNVDSAddress );

/* Return a hexadecimal short parameter to the server in a raw packet. */
enum xAT_RESULT prvSendShortHexParameter( const signed char *pcRespStrg, unsigned short usValue );

/* Send some bytes from memory to the server. */
enum xAT_RESULT prvSendMemoryBytes( signed char *pucAddressValue, signed char *pucResponse, signed char *pucCount );

/* Write a byte to memory from a server command. */
enum xAT_RESULT prvWriteByteParameter( unsigned char *pucNVDSValue, signed char *pcParamValue, enum xMEM_TYPE xMemType );

/* Write a parameter of type short given by the server to the NVDS. */
enum xAT_RESULT prvWriteShortParameter( unsigned short *pusNVDSValue, signed char *pcParamValue, enum xMEM_TYPE xMemType );

/* Write 10x of a parameter of type short given by the server to the NVDS. */
enum xAT_RESULT prvWriteShortParameterx10( unsigned short *pusNVDSValue, signed char *pcParamValue );

/* Write a parameter given by the server to the memory. */
enum xAT_RESULT prvWriteLongParameter( unsigned long *pulNVDSValue, signed char *pcParamValue, enum xMEM_TYPE xMemType );

/* Write 10x of a parameter given by the server to the memory. */
enum xAT_RESULT prvWriteLongParameterx10( unsigned long *pulNVDSValue, signed char *pcParamValue );

/* Send the version information to the server. */
enum xAT_RESULT prvSendVersionInformation( void );

/* Send device state to server. */
enum xAT_RESULT prvSendDeviceState( void );

/* Send device statistics to server. */
enum xAT_RESULT prvSendStatsBytes( void );

/* Vibrate for the requested duration on server command and return an 'OK'. */
enum xAT_RESULT prvServerReqVibrate( signed char *pcParamValue );

/* Switch on the LED(s) for the requested duration on server command and return an 'OK'. */
enum xAT_RESULT prvServerReqLed( signed char *pcParamValue );

/* Return the vibration sequence for evacuation alert to the server. */
enum xAT_RESULT prvSendEvacVibrParameters( void );

/* Parse the arguments of the FWUPD command and extract filename, UID, PW and CRC for the 
   following FTP transfer. */
void prvParseFTPParams( void );

/* Parse the arguments of the RELAY_CMD command */
void prvParseRelayCmdParams( void );

/* Initiate a Firmware update. */
void prvFirmwareUpdate( void );

/* Update the BLE module firmware. */
void prvBleFirmwareUpdate( void );

/* Return the listing of the local file system to the server. */
void prvFileSystemListing( void );

/* Download file from local file system to FTP server. */
void prvFileSystemFileGet( void );

/* Upload file from FTP server to local file system. */
void prvFileSystemFilePut( void );

/* Delete file from local file system. */
void prvFileSystemFileDelete( void );

/* Send an event based on a server command to the controlstate machine. */
enum xAT_RESULT prvSendEventToCtrl( enum xCTRL_EVENT xCtrlEvent );

/* Treat a command from the server. */
bool bTreatServerCmd( void );

/* Check, if a server command has arrived. If this is the case, treat it. The GSM module state is not altered. */
bool bCheckForServerCommand( void );

/* Functions to add specific parameters to the AT command string. */
void prvAddGPRS_CONN_CFG( void );
void prvAddTCP_CONN_CFG( void );
void prvAddTLS_CONN_CFG( void );
void prvAddTlsServer( void );
void prvAddTcpSocketId( void );
void prvAddTcpSocketQ( void );
void prvAddRcvTCPPacketLen( void );
void prvAddTcpPacketLength( unsigned short usPacketLength );
void prvAddSrvUrl( void );
void prvAddFtpAcc( void );
void prvAddFtpPutFilenames( void );
void prvAddFtpGetFilenames( void );
void prvAddFtpCfg( void );
void prvAddFLstParams( void );
void prvAddFDelParams( void );
void prvAddRatScanSequence( void );
void prvAddRatScanMode( void );
void prvAddRatIotMode( void );
void prvAddLocServerAddr( void );
void prvAddLocReqLen( void );
void prvAddANReqLen( void );
void prvAddInitalPos( void );

/* Request the Parser to store the next string from the GSM module in the version string buffer. */
void vReqStorage( void );

/* Request the Parser to store the next strings from the GSM module in the parameter string buffer. */
void prvReqMultipleStorage( void );

/* Timer callback for periodic transmissions to the server. */
void prvPosTxCallback( TimerHandle_t xTimer );

/* 	This function is run only after coming out of reset. Check, if the bootloader ran just before and
	send results correspondingly to the server. */
void vSendFWUpdateResultToServer( void );
/*-----------------------------------------------------------*/

/* Global variables. */
/* UART Rx character ring buffer */
signed char			cGsmUartRxBuffer[ GSM_UART_RX_BUFFER_SIZE ];

/* Mutex handle to protect GSM data access. */
SemaphoreHandle_t	xMutexGsmData;

/* GSM command queue handle. The queue is read in the GSM task and filled in the CTRL task. */
QueueHandle_t 		xGsmCmdQueue; 

/* Parameters received from the GSM/GPS module. Implemented as global variables
   to save program space otherwise need for encapsulation. */
signed char 		cGsmTcpSocketId[ LEN_TCP_SOCKET_ID  + 1 ];

/* Buffer for server command parameters. The buffer is filled once a server command is recognised in the 
   parser task. When the command is treated in the GSM task, the buffer is read and the parameters 
   used to execute the command. */
signed char			cServerCmdParams[ MAX_LEN_PARAMS + 1 ];

/* Data fields to be sent to the server.
   The data definition and format is specified in the AS_Tracker_Packer_Format specification.
   An additional byte per field is added for the stop code (0x0). 
   The data is written in the parser task. Reading and writing the data is protected by a semaphore. */
signed char 		cGsm_DFMAP[ LEN_DFMAP       + 1 ];			/* data field bit map. */

																/* field  0: tracker identifier (IMEI), in NVDS. */
																/* field  1: CRC (not used). */
signed char 		cGsm_IND  [ LEN_GSM_IND     + 1 ];			/* field  2: tracker indication */
signed char 		cGsm_TIM  [ LEN_GSM_TIM     + 1 ];			/* field  3: time stamp */

signed char 		cGsm_BAT  [ LEN_GSM_BAT     + 1 ];			/* field  4: battery level in Volt */
signed char 		cGsm_LAT  [ LEN_GSM_LAT     + 1 ];			/* field  5: latitude */
signed char 		cGsm_LON  [ LEN_GSM_LON     + 1 ];			/* field  6: longitude */
signed char 		cGsm_ALT  [ LEN_GSM_ALT     + 1 ];			/* field  7: altitude (4 digits + sign) */

signed char 		cGsm_VEL  [ LEN_GSM_VEL     + 1 ];			/* field  8: velocity */
signed char 		cGsm_HDG  [ LEN_GSM_HDG     + 1 ];			/* field  9: heading */
signed char 		cGsm_FIX  [ LEN_GSM_FIX     + 1 ];			/* field 10: GPS fix type */
signed char 		cGsm_HDOP [ LEN_GSM_HDOP    + 1 ];			/* field 11: Horizontal Dilution of Precision */

signed char 		cGsm_LOCM [ LEN_GSM_LOCM    + 1 ];			/* field 12: Localization method */
signed char 		cGsm_SAT  [ LEN_GSM_SAT     + 1 ];			/* field 13: satellites in view and used */
signed char 		cGsm_TEMP [ LEN_GSM_TEMP    + 1 ];			/* field 14: device temperature	*/
signed char 		cGsm_STATE[ LEN_GSM_STATE   + 1 ];			/* field 15: device state */

signed char 		cGsm_ACCHM[ LEN_GSM_ACCHM   + 1 ];			/* field 16: GPS fix horizontal accuracy */
signed char 		cGsm_ACCVM[ LEN_GSM_ACCVM   + 1 ];			/* field 17: GPS fix vertical accuracy */
signed char 		cGsm_RSSI [ LEN_GSM_RSSI    + 1 ];			/* field 18: GSM RSSI */
signed char 		cGsm_QUAL [ LEN_GSM_QUAL    + 1 ];			/* field 19: GSM bit error quality indicator */

																/* field 20: BLE list of observed beacons */
signed char 		cGsm_STEPS[ LEN_GSM_STEPS   + 1 ];			/* field 21: Absolute step count */
signed char 		cGsm_MCC  [ LEN_GSM_MCC     + 1 ];			/* field 22a: GSM debug data: MCC Mobile Country Code. */
signed char 		cGsm_MNC  [ LEN_GSM_MNC     + 1 ];			/* field 22b: GSM debug data: MNC Mobile Network Code. */
signed char 		cGsm_TAC  [ LEN_GSM_TAC     + 1 ];			/* field 22c: GSM debug data: TAC Tracking Area Code. */
signed char 		cGsm_CI   [ LEN_GSM_CI      + 1 ];			/* field 22d: GSM debug data: CI Cell Identification. */
signed char 		cGsm_RAT  [ LEN_GSM_RAT     + 1 ];			/* field 22e: GSM debug data: RAT Radio Access Technology. */
signed char 		cGsm_EARFCN[LEN_GSM_EARFCN  + 1 ];			/* field 22f: GSM debug data: EARFCN Extended Absolute Radio Frequency Channel Number. */
signed char 		cGsm_TXINST[LEN_GSM_TXINST  + 1 ];			/* field 22g: GSM debug data: TxInst Delay in seconds since transmit instant. */
signed char 		cGsm_CHRGI[ LEN_GSM_CHRGI   + 1 ];			/* field 22h: GSM debug data: ChrgCurr Charge current if charging is throttled due to temperature. */
																/* field 23: tracker identifier (IMEI) of foreign tracker in distress (not used) */
																
																/* field 24: reception time stamp of foreign tracker distress beacon (not used) */
																/* field 25: BLE list of received distress/TL beacons. */												
unsigned short		usGpsNoisePerMS;							/* field 26: Noise Level as measured by the GPS Core. */
unsigned short		usGpsAgcCnt;								/*           GPS AGC Monitor. */
unsigned char		ucGpsHwFlags;								/* 			 GPS HW flags. */
unsigned char		ucGpsJamInd;								/* 			 GPS CW Jamming indicator. */
unsigned char		ucGpsOfsI;									/* 			 GPS Imbalance of I-part of complex signal. */
unsigned char		ucGpsMagI;									/* 			 GPS Magnitude of I-part of complex signal. */
unsigned char		ucGpsOfsQ;									/* 			 GPS Imbalance of Q-part of complex signal. */
unsigned char		ucGpsMagQ;									/* 			 GPS Magnitude of Q-part of complex signal. */
																/* field 27: List of recorded GPS positions and BLE reports. */
																
																/* field 28: Evacuation ID, present in case of an ongoing evacuation alert. */
																/* field 29: RF-on counters. */
																/* field 30: BLE list of observed, UUID-filtered beacons */																

/* Copies of the above data fields which are going to be sent out via BLE beacons.
   The only difference with the above data fields is that these here are set to 0 only
   after having been sent out via BLE. */
signed char 		cGsm_BLE_LAT  [ LEN_GSM_LAT     + 1 ];		/* field  5: latitude */
signed char 		cGsm_BLE_LON  [ LEN_GSM_LON     + 1 ];		/* field  6: longitude */
signed char 		cGsm_BLE_HDOP [ LEN_GSM_HDOP    + 1 ];		/* field 11: Horizontal Dilution of Precision */
signed char 		cGsm_BLE_FIX  [ LEN_GSM_FIX     + 1 ];		/* field 10: GPS fix type */
signed char 		cGsm_BLE_LOCM [ LEN_GSM_LOCM    + 1 ];		/* field 12: Localization method */
signed char 		cGsm_BLE_SAT  [ LEN_GSM_SAT     + 1 ];		/* field 13: satellites in view and used */
signed char 		cGsm_BLE_TEMP [ LEN_GSM_TEMP    + 1 ];		/* field 14: device temperature	*/
signed char 		cGsm_BLE_STATE[ LEN_GSM_STATE   + 1 ];		/* field 15: device state */
signed char 		cGsm_BLE_ACCHM[ LEN_GSM_ACCHM   + 1 ];		/* field 16: GPS fix horizontal accuracy */
signed char 		cGsm_BLE_RSSI [ LEN_GSM_RSSI    + 1 ];		/* field 18: GSM RSSI */
signed char 		cGsm_BLE_QUAL [ LEN_GSM_QUAL    + 1 ];		/* field 19: GSM bit error quality indicator */
signed char 		cGsm_BLE_MCC  [ LEN_GSM_MCC     + 1 ];		/* field 22a: GSM debug data: MCC Mobile Country Code. */
signed char 		cGsm_BLE_MNC  [ LEN_GSM_MNC     + 1 ];		/* field 22b: GSM debug data: MNC Mobile Network Code. */
signed char 		cGsm_BLE_TAC  [ LEN_GSM_TAC     + 1 ];		/* field 22c: GSM debug data: TAC Tracking Area Code. */
signed char 		cGsm_BLE_CI   [ LEN_GSM_CI      + 1 ];		/* field 22d: GSM debug data: CI Cell Identification. */
signed char 		cGsm_BLE_RAT  [ LEN_GSM_RAT     + 1 ];		/* field 22e: GSM debug data: RAT Radio Access Technology. */
signed char 		cGsm_BLE_EARFCN[LEN_GSM_EARFCN  + 1 ];		/* field 22f: GSM debug data: EARFCN Extended Absolute Radio Frequency Channel Number. */

/* ROM-table with pointers to the data fields above. */
static const signed char * const pcDataFields[] = 
{
	NULL,   					/* field  0 */
	NULL, 						/* field  1 */
	cGsm_IND,					/* field  2 */
	cGsm_TIM,  					/* field  3 */
	cGsm_BAT,  					/* field  4 */
	cGsm_LAT,  					/* field  5 */
	cGsm_LON,  					/* field  6 */
	cGsm_ALT,  					/* field  7 */
	cGsm_VEL,  					/* field  8 */
	cGsm_HDG,  					/* field  9 */
	cGsm_FIX,  					/* field 10 */
	cGsm_HDOP, 					/* field 11 */
	cGsm_LOCM,					/* field 12 */
	cGsm_SAT,  					/* field 13 */
	cGsm_TEMP, 					/* field 14 */
	cGsm_STATE,					/* field 15 */
	cGsm_ACCHM,					/* field 16 */
	cGsm_ACCVM,					/* field 17 */
	cGsm_RSSI,					/* field 18 */
	cGsm_QUAL,					/* field 19 */
	NULL,						/* field 20 */
	cGsm_STEPS,					/* field 21 */
	cGsm_MCC,					/* field 22 */
	NULL, 						/* field 23 */
	NULL,						/* field 24 */
	NULL, 						/* field 25 */
	NULL, 						/* field 26 */
	NULL, 						/* field 27 */
	NULL, 						/* field 28 */
	NULL, 						/* field 29 */
	NULL, 						/* field 30 */
	NULL, 						/* field 31 */
};

/* Definition of the AT command strings. The strings are stored in program
   memory. */
static const char pcAt_At[] 			= "";			
static const char pcAt_E0[] 			= "E0";												/* Echo off */
static const char pcAt_IFC[] 			= "+IFC=0,0";										/* Flow control off. */
static const char pcAt_CMee[]			= "+CMEE=2";										/* Enable verbose error reporting. */
static const char pcAt_AtQGmr[]			= "+QGMR";											/* Query the Quectel module version identification. */
static const char pcAt_CPin[]	 		= "+CPIN?";											/* Query (U)SIM state. */
					
static const char pcAt_QNwScanModeQ[]	= "+QCFG=\"nwscanmode\"";							/* Query RAT scan mode selection. */
static const char pcAt_QNwScanSeqQ[]	= "+QCFG=\"nwscanseq\"";							/* Query RAT scan sequence selection. */
static const char pcAt_QIotOpModeQ[]	= "+QCFG=\"iotopmode\"";							/* Query RATs to be scanned selection. */
static const char pcAt_QNwScanMode[]	= "+QCFG=\"nwscanmode\",";							/* Set RAT scan mode selection. */
static const char pcAt_QNwScanSeq[]		= "+QCFG=\"nwscanseq\",0";							/* Set RAT scan sequence selection. */
static const char pcAt_QIotOpMode[]		= "+QCFG=\"iotopmode\",";							/* Set RATs to be scanned selection. */

static const char pcAt_QEmmCause[]		= "+QCFG=\"emmcause\",1";							/* Configure output for EMM cause query to alphanumeric. */
static const char pcAt_QEmmCauseQ[]		= "+QCFG=\"emmcause\"";								/* Query EMM cause. */

static const char pcAt_CPSms0[]			= "+CPSMS=0";										/* ?? Disable Power Saving Mode. Requires reboot to become effective. */
static const char pcAt_CPSms1[]			= "+CPSMS=1";										/* ?? Enable Power Saving Mode. Requires reboot to become effective unless globally enabled. DEBUG DEBUG DEBUG */
static const char pcAt_CPSmsQ[] 		= "+CPSMS?";										/* ?? Query  Power Saving Mode. */
static const char pcAt_CFun0[] 			= "+CFUN=0";										/* Set the MT to minimum functionality (disable both TX and RX RF circuits). */
static const char pcAt_CFun1[] 			= "+CFUN=1";										/* Set the MT to full functionality. */
static const char pcAt_CFun1_1[] 		= "+CFUN=1,1";										/* Reboot module. */
static const char pcAt_CGsn[]	 		= "+CGSN";											/* Query IMEI. */
static const char pcAt_CCid[]	 		= "+QCCID";											/* Query ICCID. */
static const char pcAt_SleepClkEn[]		= "+QSCLK=1";										/* Enable entering sleep mode by pulling GSM_DTR low. */
static const char pcAt_SleepClkDis[]	= "+QSCLK=0";										/* Disable entering sleep mode. */
static const char pcAt_CGERep[] 		= "+CGEREP=1,1";									/* Enable signalling connection event reporting. */
static const char pcAt_CGReg[]	 		= "+CGREG=2";										/* Set GPRS registration report status. */
static const char pcAt_CEReg[]	 		= "+CEREG=2";										/* Set EPS registration report status. */
static const char pcAt_QCfgBandQ[]		= "+QCFG=\"band\"";									/* Query band settings. */
static const char pcAt_QCfgBand[]		= "+QCFG=\"band\",0xF,0x8080084,0x8080084,1";		/* Select bands 3, 8, 20 and 28 for both M1 and NB-IoT. */
static const char pcAt_CGRegQ[]	 		= "+CGREG?";										/* Query GPRS registration status. */
static const char pcAt_CERegQ[]	 		= "+CEREG?";										/* Query EPS registration status. */
static const char pcAt_COps[]	 		= "+COPS?";											/* Query operator name. */
static const char pcAt_CGAttQ[]			= "+CGATT?";										/* Query GPRS atach state. */
					
static const char pcAt_CGDCont[]		= "+CGDCONT=1,\"IPV4V6\",";							/* Configure PDP context with CID=1. */
static const char pcAt_CGActQ[]			= "+CGACT?";										/* Query PDP context state. */
static const char pcAt_CGAct[]			= "+CGACT=1,1";										/* Activate PDP context with CID=1. */
static const char pcAt_QIAct[]			= "+QIACT=1";										/* Activate PDP context with CID=1. */
static const char pcAt_CGDeact[]		= "+QIDEACT=1";										/* Deactivate PDP context with CID=1. */

static const char pcAt_QSSLCfg00[]		= "+QSSLCFG=\"sslversion\",1,3";                  	/* Configure SSL Context 1: SSL version 1 = TLS1.2. */	
static const char pcAt_QSSLCfg01[]		= "+QSSLCFG=\"ciphersuite\",1,0X0035";            	/* Configure SSL Context 1: Cipher suite is TLS_RSA_WITH_AES_256_CBC_SHA. */
static const char pcAt_QSSLCfg02[]		= "+QSSLCFG=\"cacert\",1,\"ca.pem\"";             	/* Configure SSL Context 1: Set CA certificate. */
static const char pcAt_QSSLCfg03[]		= "+QSSLCFG=\"clientcert\",1,\"client-cert.pem\""; 	/* Configure SSL Context 1: Set client certificate. */
static const char pcAt_QSSLCfg04[]		= "+QSSLCFG=\"clientkey\",1,\"client-key.pem\"";  	/* Configure SSL Context 1: Set client key. */
static const char pcAt_QSSLCfg05[]		= "+QSSLCFG=\"seclevel\",1,2";						/* Configure SSL Context 1: Server and client authentication if requested by the remote server. */
static const char pcAt_QSSLCfg06[]		= "+QSSLCFG=\"sni\",1,1";							/* Configure SSL Context 1: Enable Server Name Indication feature. */
static const char pcAt_QSSLCfg07[]		= "+QSSLCFG=\"checkhost\",1,1";						/* Configure SSL Context 1: Enable hostname validation feature (Subject CommonName(CN) matches the specified host name). */
static const char pcAt_QIDnsGIP[]		= "+QIDNSGIP=1,";									/* PDP context 1: Query the IP address of the server. */
static const char pcAt_QIOpen[]			= "+QIOPEN=1,1,\"TCP\",";							/* Create clear TCP socket with PDP CID=1, SSL CID=1 and socket ID=4. */
static const char pcAt_QSSLOpen[]		= "+QSSLOPEN=1,1,4,";								/* Create TLS socket with PDP CID=1, SSL CID=1 and socket ID=4. */
static const char pcAt_QISend[]			= "AT+QISEND=";										/* Write clear TCP socket data. */
static const char pcAt_QSSLSend[]		= "AT+QSSLSEND=";									/* Write TLS socket data. */
static const char pcAt_QIRecv[]			= "+QIRD=";											/* Receive clear TCP socket data. */
static const char pcAt_QSSLRecv[]		= "+QSSLRECV=";										/* Receive TLS socket data. */
static const char pcAt_QIClose[]		= "+QICLOSE=";										/* Close clear TCP socket. */
static const char pcAt_QSSLClose[]		= "+QSSLCLOSE=";									/* Close TLS socket. */
static const char pcAt_QIState[]		= "+QISTATE=0,";									/* Query clear TCP socket status. */
static const char pcAt_QSSLState[]		= "+QSSLSTATE=";									/* Query TLS socket status. */
static const char pcAt_QIGetError[]		= "+QIGETERROR";									/* Query last TCP socket error. */
static const char pcAt_CSQ[]			= "+CSQ";											/* Read the Received Signal Strength Indication (RSSI) <rssi> and <qual> from the MT. */
static const char pcAt_QNwInfoQ[]	 	= "+QNWINFO";										/* Query network information. */
static const char pcAt_COps3[] 			= "+COPS=3,2";										/* Set output format to short alphanumeric. */
static const char pcAt_QPowD[]			= "+QPOWD";											/* Power-down. */
			
static const char pcAt_FtpCfgA[]		= "+QFTPCFG=\"account\",";							/* Configure FTP client: UID and PW. */
static const char pcAt_FtpCfgF[]		= "+QFTPCFG=\"filetype\",0";						/* Configure FTP client: file type (0 = binary, 1 = ASCII). */
static const char pcAt_FtpCfg[]			= "+QFTPCFG=";										/* Generic FTP client configuration, generally for the FTP mode. */
static const char pcAt_FtpCWD[]			= "+QFTPCWD=\"/\"";									/* FTP set current working directory to '/'. */
static const char pcAt_FtpLogin[]		= "+QFTPOPEN=";										/* FTP login to server. */
static const char pcAt_FtpLogout[]		= "+QFTPCLOSE";										/* FTP logout. */
static const char pcAt_FtpPut[]			= "+QFTPPUT=";										/* FTP upload file from local file system to FTP server. */
static const char pcAt_FtpGet[]			= "+QFTPGET=";										/* FTP download file from FTP server to local file system. */
static const char pcAt_FtpErr[]			= "+QFTPSTAT";										/* FTP error query. */
			
static const char pcAt_DelFwUpd[]		= "+QFDEL=\"fwupd.scr\"";							/* Delete firmware update file from local file system. */
							
static const char pcAt_LstFreeU[]		= "+QFLDS=\"UFS\"";									/* List free space on local UFS file system (User File System on modem side). */
static const char pcAt_LstFreeE[]		= "+QFLDS=\"EUFS\"";								/* List free space on local EUFS file system (Extended User File System on application side). */
static const char pcAt_LstFile[]		= "+QFLST=";										/* List files on local file system . */
static const char pcAt_DelFile[]		= "+QFDEL=";										/* Delete file from local file system. */
							
static const char pcAt_QHttpCfg[]		= "+QHTTPCFG=\"contenttype\",2";					/* HTTP: Set contenttype to application octet stream. */
static const char pcAt_QHttpCfg1[]		= "+QHTTPCFG=\"sslctxid\",2";						/* HTTPS request: use SSL context 2. */
static const char pcAt_QHttpUrl[]		= "+QHTTPURL=";										/* HTTP: Initiate writing of URL. */
static const char pcAt_QHttpUrlQ[]		= "+QHTTPURL?";										/* HTTP: Query URL. */
static const char pcAt_QHttpGet[]		= "+QHTTPGET=" TO_GSM_HTTPGET;						/* HTTP: GET */
static const char pcAt_QHttpRd[]		= "+QHTTPREAD";										/* HTTP: Read result. */
static const char pcAt_QSslCfg10[]		= "+QSSLCFG=\"sslversion\",2,4";					/* SSL profile for HTTPS request: all TLS versions. */
static const char pcAt_QSslCfg11[]		= "+QSSLCFG=\"ciphersuite\",2,0XFFFF";				/* SSL profile for HTTPS request: all cipher suites. */
static const char pcAt_QSslCfg12[]		= "+QSSLCFG=\"seclevel\",2,0";						/* SSL profile for HTTPS request: no security. */
static const char pcAt_QSslCfg13[]		= "+QSSLCFG=\"sni\",2,1";							/* SSL profile for HTTPS request: SNI required (does not work otherwise!). */


/* Table containing the GSM/GPS module initialisation sequence.
   The sequence is determined by the AT commands, its parameters (if they are variable),
   the number of times the command has to be repeated and a good/bad response.
   Format:
		Pointer to AT command string
		Pointer to parameter string (NULL if none)
		2 AT Response IDs for success: expected in this sequence.
		AT Response ID for failure requiring restart
		Number of repetitions
		Time-out in ticks
*/
/* Part 0: Boot GSM/LTE module.
   The last command represents the procedure to check the (U)SIM status as recommended by Quectel. If this command fails after 20s, the 
   module must be rebooted. */
static const struct xAT_CMD xAtGSMCmd_P0[] 		= 
{
	/* AT cmd string	function to add			expected					error			repe-	timeout				
						parameters				responses					response		titions	(ticks) */
	{ pcAt_E0,		 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		2,		TO_GSM_AT		},	
	{ pcAt_IFC,		 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		2,		TO_GSM_AT		},	
	{ pcAt_AtQGmr,	 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		2,		TO_GSM_AT		},	
	{ pcAt_CMee,	 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		2,		TO_GSM_AT		},	
	{ pcAt_CPin,	 	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_SIM_AT	}	
};

/* Part 1: Configure the GSM/LTE module.
   The last command reads the IMEI and stores the result. It is used to update the IMEI stored in the NVDS. */
static const struct xAT_CMD xAtGSMCmd_P1[]		=
{
	{ pcAt_CFun1,	 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		2,		TO_GSM_AT		},	
	{ pcAt_SleepClkEn, 	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_AT		},
	{ pcAt_CGReg,	 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		2,		TO_GSM_AT		},
	{ pcAt_CEReg,	 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		2,		TO_GSM_AT		},
	{ pcAt_QEmmCause, 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		2,		TO_GSM_AT		},	
	{ pcAt_COps3,	 	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_AT		},
	{ pcAt_CCid,	 	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_NVDS_AT	},
	{ pcAt_CGERep,	 	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_AT		},
	{ pcAt_CGsn,	 	vReqStorage,			{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_AT		}
};


/* Select the radio access technology and scan sequence to use for the next attempt to attach to a network. 
		AT+QCFG	nwscanmode		scan LTE, GSM or both
				iotopmode		search for eMTC, NB-IoT or both
				nwscanseq		GSM, eMTC NB-IoT
*/
static const struct xAT_CMD xAtGSMSelectRAT[]	= 
{
	/* AT cmd string	function to add			expected					error			repe-	timeout				
						parameters				responses					response		titions	(ticks) */
	{ pcAt_QNwScanMode,	prvAddRatScanMode,		{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		2,		TO_GSM_AT		},
	{ pcAt_QNwScanSeq, 	prvAddRatScanSequence,	{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		2,		TO_GSM_AT		},	
	{ pcAt_QIotOpMode, 	prvAddRatIotMode,		{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		2,		TO_GSM_AT		}
};


/* Part 2: Attach to network. */
static const struct xAT_CMD xAtGSMCmd_P21		=
	{ pcAt_CGRegQ,	 	NULL,					{ AT_CGREG_REG, AT_OK }, 	AT_CGREG_ABAND,	1,		TO_GSM_CGREG	};

static const struct xAT_CMD xAtGSMCmd_P22		=
	{ pcAt_CERegQ,	 	NULL,					{ AT_CGREG_REG, AT_OK }, 	AT_CGREG_ABAND,	1,		TO_GSM_CGREG	};

/* Part 3: Activate context. */
static const struct xAT_CMD xAtGSMCmd_P3[] 		= 
{
	{ pcAt_COps,	 	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_AT		},
	{ pcAt_CGDCont,	 	prvAddGPRS_CONN_CFG,	{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_CGREG	},	
	{ pcAt_CGActQ,		NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_CGREG	},
};	 

/* Part 4: TCP connection (unsecured). */
static const struct xAT_CMD xAtGSMCmd_P4[] 		= 
{
	/* AT cmd string	function to add			expected					error			repe-	timeout				
						parameters				responses					response		titions	(ticks) */
	{ pcAt_QIOpen, 		prvAddTCP_CONN_CFG,		{ AT_OK, AT_USOCRID },		AT_NOMSG,		2,		TO_GSM_AT		},	
};	

/* Part 4: TCP connection (secured). */
static const struct xAT_CMD xAtGSMCmd_P4sec[] 	= 
{
	/* AT cmd string	function to add			expected					error			repe-	timeout				
						parameters				responses					response		titions	(ticks) */
	{ pcAt_QSSLCfg00,	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_SEC		},
	{ pcAt_QSSLCfg01,	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_SEC		},
	{ pcAt_QSSLCfg02,	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_SEC		},
	{ pcAt_QSSLCfg03,	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_SEC		},
	{ pcAt_QSSLCfg04,	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_SEC		},
	{ pcAt_QSSLCfg05,	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_SEC		},
//	{ pcAt_QSSLCfg06,	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_SEC		},		SNI is not yet implemented
//	{ pcAt_QSSLCfg07,	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_SEC		},		Host name verification to be tested later
	{ pcAt_QSSLOpen,	prvAddTLS_CONN_CFG,		{ AT_OK, AT_USOCRID },		AT_NOMSG,		2,		TO_GSM_TCPCNX	}
};	 

/* Get IP address of the server (secured). */
static const struct xAT_CMD xAtSecDnsGIP 		= 
	/* AT cmd string	function to add			expected					error			repe-	timeout				
						parameters				responses					response		titions	(ticks) */
	{ pcAt_QIDnsGIP,	prvAddTlsServer,		{ AT_OK, AT_URC },			AT_NOMSG,		2,		4*TO_GSM_AT		};

/* Send on the clear TCP socket. */
static const struct xAT_CMD xAtQIRecv			=
	{ pcAt_QIRecv,		prvAddTcpSocketId,		{ AT_OK, AT_NOMSG },		AT_NOMSG,		1,		2*TO_GSM_AT		};

/* Send on the TLS socket. */
static const struct xAT_CMD xAtQSSLRecv			=
	{ pcAt_QSSLRecv,	prvAddTcpSocketId,		{ AT_OK, AT_NOMSG },		AT_NOMSG,		1,		5*TO_GSM_AT		};

/* Close the clear TCP socket. */
static const struct xAT_CMD xAtQIClose			=
	{ pcAt_QIClose,		prvAddTcpSocketId,		{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_SOCL		};

/* Close the TLS socket. */
static const struct xAT_CMD xAtQSSLClose		=
	{ pcAt_QSSLClose,	prvAddTcpSocketId,		{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_SOCL		};

/* Query the clear TCP socket state. */
static const struct xAT_CMD xAtQIState			=
	{ pcAt_QIState,		prvAddTcpSocketId,		{ AT_OK, AT_NOMSG },		AT_ERROR,		1,		2*TO_GSM_AT		};

/* Query SSL socket state. */
static const struct xAT_CMD xAtQSSLState		=
	{ pcAt_QSSLState,	prvAddTcpSocketId,		{ AT_OK, AT_NOMSG },		AT_ERROR,		1,		2*TO_GSM_AT		};
	
/* Query last TCP socket error. */
static const struct xAT_CMD xAtQIGetError		=
	{ pcAt_QIGetError,	NULL,					{ AT_OK, AT_NOMSG },		AT_ERROR,		1,		2*TO_GSM_AT		};
	

/* Miscellaneous AT commands. */	

static const struct xAT_CMD xAtAt				=
	{ pcAt_At, 	 		NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		1,		TO_GSM_AT_FAST	};

static const struct xAT_CMD xAtE0				=
	{ pcAt_E0,		 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		2,		TO_GSM_AT		};
	
/* PDP context re-activation. */	
static const struct xAT_CMD xAtPdpAct			=
	{ pcAt_CGAct, 		NULL,					{ AT_OK, AT_NOMSG },  		AT_NOMSG,		2,		TO_GSM_PAACT	};
static const struct xAT_CMD xAtPdpQIAct			=
	{ pcAt_QIAct, 		NULL,					{ AT_OK, AT_NOMSG },  		AT_NOMSG,		2,		TO_GSM_PAACT	};
	
/* The sole AT command serves as a ping to QTEL. If there is no response or the AT command is echoed back, QTEL is 
   rebooted. */
static const struct xAT_CMD xAtAtEcho			=
	{ pcAt_At, 	 		NULL,					{ AT_OK, AT_NOMSG },		AT_ATECHO,		2,		TO_GSM_AT		};

/* Query RATs for RAT management. */
static const struct xAT_CMD xAtQNwScanModeQ		=
	{ pcAt_QNwScanModeQ,vReqStorage,			{ AT_NWCFG, AT_OK },		AT_NOMSG,		2,		TO_GSM_AT		};	
static const struct xAT_CMD xAtQNwScanSeqQ		=
	{ pcAt_QNwScanSeqQ,	vReqStorage,			{ AT_NWCFG, AT_OK },		AT_NOMSG,		2,		TO_GSM_AT		};	
static const struct xAT_CMD xAtQIotOpModeQ		=
	{ pcAt_QIotOpModeQ,	vReqStorage,			{ AT_NWCFG, AT_OK },		AT_NOMSG,		2,		TO_GSM_AT		};	

static const struct xAT_CMD xAtQCfgBandQ		=
	{ pcAt_QCfgBandQ, 	vReqStorage,			{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		2,		TO_GSM_AT		};
static const struct xAT_CMD xAtQCfgBand			=
	{ pcAt_QCfgBand, 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		2,		TO_GSM_AT		};


static const struct xAT_CMD xAtModVersion		=
	{ pcAt_AtQGmr, 		prvReqMultipleStorage,	{ AT_OK, AT_NOMSG },  		AT_NOMSG,		2,		TO_GSM_AT		};
	
static const struct xAT_CMD xAtCGAttQ			=
	{ pcAt_CGAttQ,	 	NULL,					{ AT_OK, AT_NOMSG },  		AT_NOMSG,		2,		TO_GSM_AT		};
		
static const struct xAT_CMD xAtPdpActQ			=
	{ pcAt_CGActQ,		NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_AT		};
 
static const struct xAT_CMD xAtPdpDeact			=
	{ pcAt_CGDeact,		NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_TCPCNX	};

static const struct xAT_CMD xAtQPowD			=
	{ pcAt_QPowD,		NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		4,		TO_GSM_PWROFF	};

static const struct xAT_CMD xAtStat[]			=
{
	{ pcAt_COps,	 	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		1,		TO_GSM_AT		},
	{ pcAt_CGRegQ,	 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		1,		TO_GSM_AT		},
	{ pcAt_CERegQ,	 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		1,		TO_GSM_AT		},
	{ pcAt_CSQ,			NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		1,		TO_GSM_GPRS		}
};

static const struct xAT_CMD xAtQNwInfoQ			=
	{ pcAt_QNwInfoQ,	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		1,		TO_GSM_AT		};

static const struct xAT_CMD xAtQEmmCauseQ		=
	{ pcAt_QEmmCauseQ,	vReqStorage,			{ AT_OK, AT_NOMSG },		AT_NOMSG,		1,		TO_GSM_AT		};	


/* Connect to the XSole server via HTTP to obtain a location estimation based on the cell data. */
static const struct xAT_CMD xAtGetLocEst0[]		=
{
	{ pcAt_QHttpCfg, 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		1,		TO_GSM_AT		},
	{ pcAt_QHttpUrl, 	prvAddLocReqLen,		{ AT_CONNECT, AT_NOMSG },	AT_NOMSG,		1,		TO_GSM_HTTPURL	}
};
static const struct xAT_CMD xAtGetLocEst1[]		=
{
	{ pcAt_QHttpGet, 	NULL,					{ AT_OK, AT_HTTP_GET_OK }, 	AT_HTTP_GET_KO,	1,		TO_GSM_HTTP		},
	{ pcAt_QHttpRd,		prvReqMultipleStorage,	{ AT_CONNECT, AT_OK }, 		AT_NOMSG,		1,		TO_GSM_AT		}
};

/* Connect to the ublox server via HTTP to download the GPS assistance data. */
static const struct xAT_CMD xAtGetGpsANData0[]	=
{
	{ pcAt_QSslCfg10, 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		3,		TO_GSM_AT		},
	{ pcAt_QSslCfg11, 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		3,		TO_GSM_AT		},
	{ pcAt_QSslCfg12, 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		3,		TO_GSM_AT		},
	{ pcAt_QSslCfg13, 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		3,		TO_GSM_AT		},
	{ pcAt_QHttpCfg, 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		3,		TO_GSM_AT		},
	{ pcAt_QHttpCfg1, 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		3,		TO_GSM_AT		},
	{ pcAt_QHttpUrl, 	prvAddANReqLen,			{ AT_CONNECT, AT_NOMSG },	AT_NOMSG,		1,		TO_GSM_AT		}
};
static const struct xAT_CMD xAtQHttpUrlQ		=
	{ pcAt_QHttpUrlQ, 	NULL,					{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		1,		TO_GSM_HTTP		};

static const struct xAT_CMD xAtGetGpsANData1	=
	{ pcAt_QHttpGet, 	NULL,					{ AT_OK, AT_HTTP_GET_OK }, 	AT_HTTP_GET_KO,	1,		TO_GSM_HTTP		};
	
/* FTP Login. */
static const struct xAT_CMD xAtFtpLogin[] 		= 
{
	/* AT cmd string	function to add			expected					error			repe-	timeout				
						parameters				responses					response		titions	(ticks) 		*/
	{ pcAt_FtpCfgA, 	prvAddFtpAcc,			{ AT_OK, AT_NOMSG }, 		AT_NOMSG,		2,		TO_GSM_AT		},	
	{ pcAt_FtpCfgF, 	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_AT		},
	{ pcAt_FtpCfg, 		prvAddFtpCfg,			{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_AT		},
	{ pcAt_FtpLogin,	prvAddSrvUrl,			{ AT_OK, AT_FTP_OK },		AT_FTP_KO,		2,		TO_GSM_FTPLOGIN	}
};

/* FW Update. */
static const struct xAT_CMD xAtFwUpLoad[] 		= 
{
	/* AT cmd string	function to add			expected					error			repe-	timeout				
						parameters				responses					response		titions	(ticks) 		*/
	{ pcAt_FtpCWD,	 	NULL,					{ AT_OK, AT_FTP_OK },		AT_FTP_KO,		2,		TO_GSM_FTPOP	},
	{ pcAt_FtpGet,		prvAddFtpGetFilenames,	{ AT_OK, AT_FTP_OK },		AT_FTP_KO,		2,		TO_GSM_FTPTRSF	},
	{ pcAt_FtpLogout,	NULL,					{ AT_OK, AT_FTP_OK },		AT_NOMSG,		2,		TO_GSM_FTPLOGOUT}
};	

static const struct xAT_CMD xAtDelFwUpd			=
	{ pcAt_DelFwUpd,	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_AT		};

static const struct xAT_CMD xAtFtpErr			=
	{ pcAt_FtpErr,		NULL,					{ AT_OK, AT_FTP_STAT },		AT_NOMSG,		2,		TO_GSM_AT		};
	
static const struct xAT_CMD xAtUFtpLogout		=
	{ pcAt_FtpLogout,	NULL,					{ AT_OK, AT_FTP_OK },		AT_NOMSG,		2,		TO_GSM_FTPLOGOUT};
	
/* Local GSM Module file system API. */
const struct xAT_CMD xAtLstFreeU				=
	{ pcAt_LstFreeU,	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_FS		};
	
const struct xAT_CMD xAtLstFreeE				=
	{ pcAt_LstFreeE,	NULL,					{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_FS		};
	
const struct xAT_CMD xAtLstFile					=
	{ pcAt_LstFile,		prvAddFLstParams,		{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_FS		};

static const struct xAT_CMD xAtDelFile			=
	{ pcAt_DelFile,		prvAddFDelParams,		{ AT_OK, AT_NOMSG },		AT_NOMSG,		2,		TO_GSM_FS		};

/* FS_GET is from the server point of view ('get a file from the device'. From the device (Quectel) point of view,
   this is a put (send a file to the FTP server). */
static const struct xAT_CMD xAtFsGet[] 			= 
{
	/* AT cmd string	function to add			expected					error			repe-	timeout				
						parameters				responses					response		titions	(ticks) 		*/
	{ pcAt_FtpCWD,	 	NULL,					{ AT_OK, AT_FTP_OK },		AT_FTP_KO,		2,		TO_GSM_FTPOP	},
	{ pcAt_FtpPut,		prvAddFtpPutFilenames,	{ AT_OK, AT_FTP_OK },		AT_FTP_KO,		2,		TO_GSM_FTPTRSF	},
	{ pcAt_FtpLogout,	NULL,					{ AT_OK, AT_FTP_OK },		AT_NOMSG,		2,		TO_GSM_FTPLOGOUT}
};	

/* FS_PUT is from the server point of view ('put(write) a file to the device'. From the device (Quectel) point of view,
   this is a get (fetch a file from the FTP serer). */
static const struct xAT_CMD xAtFsPut[] 			= 
{
	/* AT cmd string	function to add			expected					error			repe-	timeout				
						parameters				responses					response		titions	(ticks) 		*/
	{ pcAt_FtpCWD,	 	NULL,					{ AT_OK, AT_FTP_OK },		AT_FTP_KO,		2,		TO_GSM_FTPOP	},
	{ pcAt_FtpGet,		prvAddFtpGetFilenames,	{ AT_OK, AT_FTP_OK },		AT_FTP_KO,		2,		TO_GSM_FTPTRSF	},
	{ pcAt_FtpLogout,	NULL,					{ AT_OK, AT_FTP_OK },		AT_NOMSG,		2,		TO_GSM_FTPLOGOUT}
};	

/* Initialisation values for GSM GPRS and TCP connection parameters. 
   ATTENTION: The maximum string length must not exceed MAX_LEN_PARAMS! */
// const signed char 				cGsmGPRSDefaultConnParam[] 	= "'internet.sierrawireless.com'";	
const signed char 				cGsmGPRSDefaultConnParam[] 	= "'lp.swir'";					/* DEBUG DEBUG DEBUG Default APN for Sierra Wireless SIM V4 LWPA tests. */
const signed char 				cGsmTCPDefaultConnParam[]	= "'lnr.xsole.net',9000";		
const signed char 				cGsmSecTCPConnParam[]		= "'lnr.xsole.net',9003";	
const signed char 				cGpsTCPStreamingServer[]	= "'devlog.xsole.fr'";
const signed char 				cGPSLocEstSrvAddr[]			= "xsole.net";

/* TLS server IP address. Will be filled in as soon as the DNS query has run. */
signed char 					cGsmTlsServerIP[ LEN_IP_ADDR ];

/* Initialisation values for FW update. */
const signed char 				cFTPServerDefaultURL[] 		= "'firmup.xsole.net'";	
const signed char 				cFWDefaultFileName[]		= "''";	
const signed char 				cFTPServerDummyLogin[]		= "''";	
const signed char 				cFTPDefaultCfg[]			= "'transmode',1";				/* Default FTP client configuration: FTP transfer mode (0 = active, 1 = passive). */

/* Number of failed subsequent attempts when connecting to server. */
unsigned portBASE_TYPE			uxGsmAttachAttemptCnt;
unsigned portBASE_TYPE			uxGsmTCPConnectAttemptCnt;

/* GPS/GSM module state: off requiring cold or warm start, TCP disconnected with GPS on or off, TCP connected with GPS on or off. */
portBASE_TYPE					xModuleState;

/* Connection retry counter. */
unsigned portBASE_TYPE			uxTcpConnectionRetryCounter;

/* Variable telling if a server command transaction is ongoing. */
bool							bCommandTransactionOngoing;

/* Timer handle for the position transmission timer. */
static TimerHandle_t 			xPosTxTimer;	

/* Boolean which inhibits waiting for a fix on the next transmission slot. This mechanism is used when CTRL detects
   an alert and wants to immediately push a packet regardless of the position fix status. 
   This variable might be set anytime by CTRL. */
volatile bool 					bPushPacketImmediately;

/* Timer handle for the GSM/GPS module shutdown timer. */
static TimerHandle_t 			xModuleShutdownTimer;	

/* Boolean which indicates that the device is in test mode, i.e. sending position packets and scanning for
   BLE beacons even though it is on the charger. */
bool							bTestMode;

/* Variable is true whenever the GSM had to reattach to the BTS in order to send the packet.
   The value of this variable is ent as flag in the IND field of the packet. */
bool							bGsmReattach;

/* Flag which indicates an unsolicited closure of the TCP/TLS socket. */
bool							bTcpUnsolicitedClose = false;

/* Time stamp recording the moment of the send positon instant. */
unsigned short					usTxInstantTS;

/* Initial position string in ublox format. */
signed char						cGpsInitialPosEstimate[ INITIAL_POS_LENGTH + 1 ];
bool							bGpsInitialPosEstimateInitialised;

/* Time stamp of the moment the initial position string has been updated last. */
unsigned short 					usGpsInitialPosEstimateTS;

/* RTC-timestamp of the last GPS online-assisted GPS database download. The database has a lifetime of about
   two hours. The module requests a database download whenever it detects that the current database is out-
   of-date. */
unsigned long					ulAssistedGPSUpdateTimeStamp;

/* Temporary path-/filename of the source file. */
signed char						cFTPSourceFileName[ FTP_MAX_FILNAME + 1 ];				
/* Temporary path-/filename of the target file. */
signed char						cFTPTargetFileName[ FTP_MAX_FILNAME + 1 ];				
/* Temporary UID for the FTP transfer. */					
signed char						cFTPUid[ FTP_MAX_UID + 1 ];						
/* Temporary password for the FTP transfer. */					
signed char						cFTPPw[ FTP_MAX_PW + 1 ];						

/* Pointers to source and target file names for FTP commands. */
signed char						*pcFTPSourceFileName;
signed char						*pcFTPTargetFileName;

/* File handle for opened BLE FW update file. */
unsigned portBASE_TYPE			uxFileHandle;

/* Position store read and write indices. The indices are used for both the GPS and the BLE position stores. */
unsigned portBASE_TYPE			uxPosStoreRdIdx;
unsigned portBASE_TYPE			uxPosStoreWrIdx;

/* Absolute step count in between entries of the position store. */
unsigned short					usPosStoreAbsStepCount;

/* Timestamp of the last time a packet has been sent in GPS recording mode. */
TickType_t						xPacketSentInGpsRecordingTime;

/* Array to store GSM data accompanying recorded positions if recording is required. */
struct xGSM_POS_ENTRY			xGsmPosStore[ POS_STORE_LEN ];

/* On-duration counters for the cellular module. The counter is reset every time a packet has been 
   sent to the server using the interface function. */
TickType_t						xGsmOnTimeStamp;
TickType_t						xGsmOnDuration;
/*-----------------------------------------------------------*/

/* Initialise the GSM task. */
void vGsmInit( UBaseType_t uxPriority )
{
	/* Initialise the UART port. */
  	/* TXD as output. */
	nrf_gpio_cfg_output( GSM_TXD );
	
	/* Initisalise the outputs to drive the module control lines. */
	nrf_gpio_cfg_output( GSM_PWR_CTRL );						/* Set to output. */
	nrf_gpio_pin_clear( GSM_PWR_CTRL );         				/* Set to 0: power off. */
	
	nrf_gpio_cfg_output( GSM_PWR_KEY );							/* Set to output. */
	nrf_gpio_pin_clear( GSM_PWR_KEY );							/* Low level. */

	nrf_gpio_cfg_output( GSM_DTR );								/* Set to output. */
	nrf_gpio_pin_clear( GSM_DTR );								/* Low level. */

	nrf_gpio_cfg_input( STATUS, NRF_GPIO_PIN_NOPULL );			/* Set to input. */

	/* Create a mutex to protect access to the data variables shared with the GSM task. */
	xMutexGsmData = xSemaphoreCreateMutex();
	
	/* Create a queue from the CTRL task to the GSM task for commands. */
	xGsmCmdQueue = xQueueCreate( gsmCMD_QUEUE_SIZE, ( unsigned portBASE_TYPE ) sizeof( enum xGSM_CMD ) );	
	
	/* The GSM task is spawned here. */
	xTaskCreate( vGSMTask, "GSM", gsmSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
	
	/* Connection retry counter is 0 (never tried to connect yet). */
	uxTcpConnectionRetryCounter = 0;
	
	/* Initialise module state. */
	xModuleState = 0;
	
	/* Initialise the connection attempt counters. */
	uxGsmAttachAttemptCnt = 0;
	uxGsmTCPConnectAttemptCnt = 0;
	
	/* No server command transaction is pending. */
	bCommandTransactionOngoing = false;
	
	/* The first packet will send this flag telling that it is a 're-attach'. */
	bGsmReattach = true;
	
	/* Set the flag which indicates an unsolicited closure of the TCP/TLS socket. */
	bTcpUnsolicitedClose = false;
	
	/* Do not allow for the next packet to be sent without position fix. */
	bPushPacketImmediately = false;
	
	/* Reset the time-stamp of the last GPS assisted data update to a value which will assure that the
	   download of Assisted data is requested the next time the module boots. */
	ulAssistedGPSUpdateTimeStamp = 0xfffffffe - T_GPS_ASSISTED_DATA_VALIDITY;

	/* Initialise GSM data fields. */
	prvInvalidateData();
	uxBleTLBcnCnt = 0;			/* field 25: BLE list of received distress/TL beacons. */
	uxForeignAlertType = 0;
	cGsm_IND[ 0 ]  = 0;	
	cGsm_IND[ 1 ]  = 0;		/* field  2: tracker indication: not covered by the invalidate function. */
	
	/* Initialise the statistics variables. */
	uStats.xStats.usGprsConnectFail = 0;
	uStats.xStats.usGsmModuleCrash = 0;	
	uStats.xStats.usGsmModuleReboot = 0;
	uStats.xStats.usGsmAttachFail = 0;
	
	/* The device is not in test mode. */
	bTestMode = false;
	
	/* Initialise the time stamp recording the moment of the send position instant. */
	usTxInstantTS = 0;
	
	/* Initialise the access variabes to the position store. */
	uxPosStoreRdIdx = 0;
	uxPosStoreWrIdx = 0;
	usPosStoreAbsStepCount = 0;
	
	/* Initialise the initial position estimation with a location corresponding to the centre of France.
			0x1C154B38 = 47.115756deg  	WGS84 Latitude
            0x016F5940 =  2.407456deg 	WGS84 Longitude
            0x204E     = 200m	      	WGS84 Altitude
            0x04801d2C = 700km    		Position accuracy (stddev)	*/ 
	strcpy( cGpsInitialPosEstimate, "46,2,0,800000" );
	bGpsInitialPosEstimateInitialised = false;

	/* Set the update time stamp so that the initial position will be fetched during the next opportunity. */
	usGpsInitialPosEstimateTS = usReadRTC() - INITIAL_POS_VALIDITY - 1;
	
	/* Initialise the timestamp of the last time a packet has been sent in GPS recording mode. */
	xPacketSentInGpsRecordingTime = xTaskGetTickCount();
	
	/* Initialise GPS on-duration counter. */
	xGsmOnTimeStamp = 0;
	xGsmOnDuration = 0;
	
	/* Initialise the server IP address buffer until we have read it once. */
	cGsmTlsServerIP[ 0 ] = 0;

	/* Create position transmit timer. */
	xPosTxTimer = xTimerCreate
							( "POSTX",		 				/* Timer name for debug. */
							  2 * portTICKS_PER_SEC,		/* Initial timer period in ticks. The first packet is transmitted after this delay to ensure that
							  								   any charger conditions are settled before the big GSM current hits the battery. */
							  pdTRUE,						/* Auto-reload. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvPosTxCallback				/* Callback for the position transmit timer. */
							);
							
	/* Create GSM/GPS module shutdown timer. */
	xModuleShutdownTimer = xTimerCreate
							( "SHUTDWN",	 				/* Timer name for debug. */
							  MOD_SHUTDWN_DELAY_POS,		/* Initial timer period in ticks. Dummy here as the time is not yet started. */
							  pdFALSE,						/* Single-shot. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvModuleShutdownCallback		/* Callback for the GSM/GPS module shutdown timer. */
							);

	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* Wait for a AT_QBOOT message from the GSM module indicating that the module has finished booting. 
   Under certain circumstances the module is not configured to the standard baudrate when booted. */
bool bWaitModuleBooted( void )
{
	return ( xReceiveResponse( AT_QBOOT, AT_NOMSG, AT_NOMSG, T_GSM_REBOOT ) == AT_SUCCESS );
}
/*-----------------------------------------------------------*/

/* Power-up the GSM/GPS module. */
void prvGSMModulePowerUp()
{
	enum xAT_RESULT			xSuccess;
	TickType_t				xStartTime;	
	unsigned portBASE_TYPE	uxLoopCnt;
	
	/* To be on the safe side, disable PSM (pull GSM_DTR low). */
	xModuleDisableSleepMode();
	
	NRF_LOG_INFO( "%i Booting the cellular module.", ulReadRTC() );
	NRF_LOG_FLUSH();
	
	uxLoopCnt = 0;
	do
	{
		/* Apply a low level to GSM_PWR_ON_N in order to switch the GSM module on. */
		nrf_gpio_cfg_output( GSM_PWR_CTRL );					/* Set GSM_PWR_CTRL to output and low. */
		nrf_gpio_pin_clear( GSM_PWR_CTRL );
		
		nrf_gpio_cfg_output( GSM_PWR_KEY );						/* Set GSM_PWR_ON_N to input */
		nrf_gpio_pin_clear( GSM_PWR_KEY );						/* Low level on GSM_PWR_ON_N, no pull-up on. */

		nrf_gpio_cfg_input( GSM_RXD, NRF_GPIO_PIN_NOPULL );		/* Set RXD to input, no pull-up on. */
		
		nrf_gpio_cfg_output( GSM_TXD );
		nrf_gpio_pin_clear( GSM_TXD );							/* Set TXD to '0' */

		nrf_gpio_pin_set( GSM_PWR_CTRL );						/* Switch-on the GSM module power supply */
		nrf_gpio_cfg_input( GSM_PWR_KEY, NRF_GPIO_PIN_NOPULL );	/* Set GSM_PWR_ON_N to input without pull-up */

		vTaskDelay( T_GSM_EXT_PWR_ON );							/* Delay for the external power-on. */		

		nrf_gpio_cfg_output( GSM_PWR_KEY );						/* Pulse GSM_PWR_ON low: Set GSM_PWR_KEY to output */

		xStartTime = xTaskGetTickCount();						/* Wait for STATUS to become '1'. */
		do														
		{
			vTaskDelay( 10 );
		} while (    !nrf_gpio_pin_read( STATUS )
				  && ( xTaskGetTickCount() - xStartTime < TO_GSM_INT_PWR_ON ) );	
		
		nrf_gpio_cfg_input( GSM_PWR_KEY, NRF_GPIO_PIN_NOPULL );	/* Set GSM_PWR_KEY to input. */
			  
		nrf_gpio_pin_set( GSM_TXD );							/* Set TXD to '1' (inactive state) */	
		
		vCOMOpen( COM_GSM, cGsmUartRxBuffer, GSM_UART_RX_BUFFER_SIZE - 1, COM_ASCII );
																/* Open the GSM AT COM port with the default 115200. */

		V_TRACE_PRINT_SHORT( TRACE_GSM_BAUDSWITCH, 11520, TRACE_UART );
				  
		/* Wait for the module to boot. The end of the process is indicated by the 'APP RDY' being sent by the module. */
		( void )bWaitModuleBooted();
		
		xSuccess = xSendAtCommand( &xAtAt, true, true );
		
		if ( xSuccess != AT_SUCCESS )
		{
			NRF_LOG_WARNING( "%i Could not contact cellular module.", ulReadRTC() );
			NRF_LOG_FLUSH();
			vGSMModulePowerDown( HARD_POWER_OFF );
		}
	
		uxLoopCnt++;
	} while ( ( uxLoopCnt < 2 ) && ( xSuccess != AT_SUCCESS ) );	

	/* Set the bit indicating that the module is powered up and no future cold start is required. */
	if ( xSuccess == AT_SUCCESS )
	{
		if ( ( xModuleState & GSM_PWR_MSK ) == GSM_PWR_OFF )
		{
			/* Remember the time stamp when the module was started. */
			xGsmOnTimeStamp = xTaskGetTickCount();
		}

		xModuleState |= GSM_PWR_ON | GSM_ACTIVE_MSK;
	}
}
/*-----------------------------------------------------------*/

/* Power-off the GSM/GPS module. 
   Note that the QUectel BG500L has no real reset input. A RESET_ONLY request is thus leading to a soft power-down. */
void vGSMModulePowerDown( enum xGSM_PWRDOWN_TYPE xType )
{
	TickType_t			xStartTime;
	enum xGSM_CMD		xGsmCmd;
	
	/* Check if the module is already off. If yes, return without any further action. */
	if ( ( xModuleState & GSM_PWR_MSK ) != GSM_PWR_ON )
	{
		return;
	}

	NRF_LOG_INFO( "%i Shutting-down the cellular module.", ulReadRTC() );
	NRF_LOG_FLUSH();

	xModuleDisableSleepMode();

	/* Trace module power-down type. */
	if ( xType == SOFT_POWER_OFF )
	{
		V_TRACE_PRINT( TRACE_GSM_MODULE_SOFT_FULL_POWER_DOWN, TRACE_UART_AND_FILE );
	}
	else if ( xType == HARD_POWER_OFF )
	{
		V_TRACE_PRINT( TRACE_GSM_MODULE_HARD_FULL_POWER_DOWN, TRACE_UART_AND_FILE );
	}
	else
	{
		V_TRACE_PRINT( TRACE_GSM_MODULE_INTERNAL_POWER_DOWN_ONLY, TRACE_UART_AND_FILE );
	}
	
	/* Check if the module is still responsive. If not, convert the power-down type to HARD_POWER_OFF
	   to shorten the shut-down time. */
	if ( xType == SOFT_POWER_OFF )
	{
		if ( xSendAtCommand( &xAtAt, true, true ) != AT_SUCCESS )
		{
			xType = HARD_POWER_OFF;
			V_TRACE_PRINT( TRACE_GSM_MODULE_POWER_DOWN_CONVERTED_TO_HARD, TRACE_UART );
		}
	}

	/* The following steps perform a gracious shut-down of all connections and the module.
	   We only do this if the module is still responsive. */
	if ( xType != HARD_POWER_OFF )
	{
		/* Open the GSM AT COM port in case we closed it before in an attempt to shut down the module. */
		vCOMOpen( COM_GSM, cGsmUartRxBuffer, GSM_UART_RX_BUFFER_SIZE - 1, COM_ASCII );								
		
		/* Stop the GPS if it is running. */
		vCheckConditionsAndShutdownGPS();
			
		/* If there is a TCP connection, tear it down. */
		if (    ( ( xModuleState & GSM_TCP_CONN_MSK ) == GSM_TCP_CONN )
			 || ( ( xModuleState & GSM_TLS_CONN_MSK ) == GSM_TLS_CONN ) )
		{
			if ( ( xModuleState & GSM_TCP_CONN_MSK ) == GSM_TCP_CONN )
			{
				xSendAtCommand( &xAtQIClose, true, true );		
			}
			else
			{
				xSendAtCommand( &xAtQSSLClose, true, true );	
			}
			vTaskDelay( T_TCP_DISC );

			/* Check if the module is still responsive. If not, convert the power-down type to HARD_POWER_OFF
			   to shorten the shut-down time. 
			   Otherwise, continue with the soft-power-down. */
			if ( xSendAtCommand( &xAtAt, true, true ) != AT_SUCCESS )
			{
				xType = HARD_POWER_OFF;
				V_TRACE_PRINT( TRACE_GSM_MODULE_POWER_DOWN_CONVERTED_TO_HARD, TRACE_UART );
			}
			else
			{
				xSendAtCommand( &xAtPdpDeact, true, true );
			}
			xModuleState &= ( unsigned portBASE_TYPE )~( GSM_PDP_ACT | GSM_TCP_CONN_MSK | GSM_TLS_CONN_MSK );
		}
		
		/* Shut down module. */
		xSendAtCommand( &xAtQPowD, true, true );	
	}
			
	/* Close the COM port. */
	vCOMClose( COM_GSM );				
	
  	/* Set TXD as output again as disabling the UART resets it to input. */
	nrf_gpio_cfg_output( GSM_TXD );
	nrf_gpio_pin_clear( GSM_TXD );

	/* Wait for max. TO_GSM_VINT_DWN for STATUS to become '0'. */
	if ( xType != HARD_POWER_OFF )
	{
		xStartTime = xTaskGetTickCount();
		while ( nrf_gpio_pin_read( STATUS ) && ( xTaskGetTickCount() - xStartTime < TO_GSM_VINT_DWN ) )
		{
			;
		}

		/* Check if the command was successful. If not, convert the power-down type to HARD_POWER_OFF. */
		if ( nrf_gpio_pin_read( STATUS ) && ( xType != HARD_POWER_OFF ) )
		{
			xType = HARD_POWER_OFF;
			V_TRACE_PRINT( TRACE_GSM_MODULE_POWER_DOWN_CONVERTED_TO_HARD, TRACE_UART );
		}
	}
	
	/* Set all signals to the GSM/GPS module to output and 0. */
	nrf_gpio_cfg_output( GSM_RXD );					
	nrf_gpio_pin_clear( GSM_RXD );         			

	/* If requested, shut down the module's power supply. */
	if ( ( xType == SOFT_POWER_OFF ) || ( xType == HARD_POWER_OFF ) )
	{
		nrf_gpio_pin_clear( GSM_PWR_CTRL );
		
		/* Wait for max. TO_GSM_VGSM_DWN for VGSM to get below 1.2V. */
		xStartTime = xTaskGetTickCount();
		while ( ( ( unsigned long )uiReadADC( VGSM_ADC, PROTECTED ) > VGSM_OFF_THRES ) && ( xTaskGetTickCount() - xStartTime < TO_GSM_VGSM_DWN ) )
		{
			vTaskDelay( 20 );
		}
		
		V_TRACE_PRINT( TRACE_GSM_MODULE_POWERED_DOWN, TRACE_UART );
	}
	
	if ( ( xModuleState & GSM_PWR_MSK ) == GSM_PWR_ON )
	{
		/* Accumulated the time duration during which the module was running. */
		xGsmOnDuration += xTaskGetTickCount() - xGsmOnTimeStamp;
	}

	/* Reset the bit indicating that the module is not powered up. */
	xModuleState &=   ( unsigned portBASE_TYPE )~GSM_PWR_MSK
					& ( unsigned portBASE_TYPE )~GPS_PWR_MSK
					& ( unsigned portBASE_TYPE )~GSM_PDP_ACT_MSK
					& ( unsigned portBASE_TYPE )~GSM_TCP_CONN_MSK
					& ( unsigned portBASE_TYPE )~GSM_TLS_CONN_MSK
					& ( unsigned portBASE_TYPE )~GSM_ACTIVE_MSK;	
}
/*-----------------------------------------------------------*/

/* Timer callback for GSM/GPS module shutdown. 
   The timer is started once either the position transmit has finished or a server command transaction is terminated.
   During the timer expiry delay, the server is given the oportunity to send more commands to the module. In that case,
   the timer is stopped and relaunched after the new transaction is terminated.
   
   CAUTION: This function is running in the timer task!
*/
void prvModuleShutdownCallback( TimerHandle_t xTimer )
{
	enum xGSM_CMD		xGsmCmd;
	
	( void )xTimer;
	
	/* Send a message to the GSM task to shutdown the GSM/GPS module. */
	if ( !bQueueCheckIfDataItemInQueue( xGsmCmdQueue, GSM_SEND_POSITION )  && !bCommandTransactionOngoing )
	{
		/* Only queue a shutdown request if there is no transaction ongoing. */
		xGsmCmd = GSM_SHUTDOWN_MODULE;
		xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmCmd, 0 );		
	}
}
/*-----------------------------------------------------------*/

/* Add GPRS Connection Configuration parameter. */
void prvAddGPRS_CONN_CFG( void )
{
	signed char		cTmpStrg[ LEN_GSM_GPRS_CONN_CFG + 1 ];
	
	/* Every attempt where bit 4 of the attach attempt counter is 0 (i.e. 0 to 15, 32 to 47, etc.) 
	   use the regular parameter string from NVDS. In all other cases (attempts 16 to 31, 48 to 63, etc.) 
	   use the default value stored in ROM. */
	if ( ( uxGsmAttachAttemptCnt & 0b00010000 ) == 0 )
	{
		/* Read the configuration from the NVDS. 
		ATTENTION: It is assumed here that the read process is sufficiently fast.
				   If this turn out not to be the case, the value needs to be shadowed 
				   in RAM. */
		( void )pcConfigReadString( xNvdsConfig.pcGsmGPRSConnCfg, cTmpStrg, LEN_GSM_GPRS_CONN_CFG + 1, TRANSLATE_APOSTR );
	}
	else
	{
		/* Read the configuration string from ROM and copy it to the parameter buffer.
		   Translate any "'" (apostrophe) characters to '"' (quotation mark). */
		strcpy( cTmpStrg, cGsmGPRSDefaultConnParam );
		vTranslateSingleQuote( cTmpStrg );
	}
	
	/* Send the parameter. */
	xComSendString( COM_GSM, cTmpStrg );
	
	/* Add APN (empty), data and header compression. */
	xComSendChar( COM_GSM, ',' );
	xComSendChar( COM_GSM, ',' );
	xComSendChar( COM_GSM, '1' );
	/* PDP data compression <d_comp> = 1: on (predefined compression type i.e. V.42bis data compression) */
	xComSendChar( COM_GSM, ',' );
	/* PDP header compression <h_comp> = 1: on (predefined compression type, i.e. RFC1144) */
	xComSendChar( COM_GSM, '1' );
}
/*-----------------------------------------------------------*/

/* Add TCP Connection Configuration parameter. */
void prvAddTCP_CONN_CFG( void )
{
	signed char		cTmpStrg[ LEN_GSM_TCP_CONN_CFG + 1 ];
	
	/* Every attempt where bit 3 of the server connection attempt counter is 0 (i.e. 0 to 7, 16 to 23, 32 to 39, etc.)
	   use the regular parameter string from NVDS. In all other cases (attempts 8 to 15, 24 to 31, etc.) 
	   use the default value stored in ROM. */
	if ( ( uxGsmTCPConnectAttemptCnt & 0b00000010 ) == 0 )
	{
		/* Read the configuration from the NVDS. */
		( void )pcConfigReadString( xNvdsConfig.pcGsmTCPConnCfg, cTmpStrg, LEN_GSM_TCP_CONN_CFG + 1, TRANSLATE_APOSTR );
	}
	else
	{
		strcpy( cTmpStrg, cGsmTCPDefaultConnParam );
		vTranslateSingleQuote( cTmpStrg );
	}
	
	/* Send the parameter. */
	xComSendString( COM_GSM, cTmpStrg );

	/* Add parameters for local port (0 means to be assigned automatically) and to request buffer access mode. */
	xComSendString( COM_GSM, ",0,0" );	
}
/*-----------------------------------------------------------*/

/* Add TLS Connection Configuration parameter. */
void prvAddTLS_CONN_CFG( void )
 {
 	signed char		cTmpStrg[ LEN_GSM_TCP_CONN_CFG + 1 ];
 	
	/* Send the TLS IP address. */
	xComSendString( COM_GSM, "\"" );	
 	xComSendString( COM_GSM, cGsmTlsServerIP );
	xComSendString( COM_GSM, "\"" );	
	
	/* Read the configuration from the NVDS. */
 	( void )pcConfigReadString( xNvdsConfig.pcGsmSecTCPConnCfg, cTmpStrg, LEN_GSM_TCP_CONN_CFG + 1, TRANSLATE_APOSTR );
 	
	/* Add the TCP socket ID with leading ','. */
 	xComSendString( COM_GSM, pcFindNComma( cTmpStrg, 1, LEN_GSM_TCP_CONN_CFG ) - 1 );
	
	/* Add parameter to request buffer access mode. */
	xComSendString( COM_GSM, ",0" );	
 }
 /*-----------------------------------------------------------*/

/* Add TLS server name to DNS query. */
void prvAddTlsServer( void )
{
	signed char		cTmpStrg[ LEN_GSM_TCP_CONN_CFG + 1 ];
	
	/* Read the server name from the NVDS. */
	( void )pcConfigReadString( xNvdsConfig.pcGsmSecTCPConnCfg, cTmpStrg, LEN_GSM_TCP_CONN_CFG + 1, TRANSLATE_APOSTR );
	
	/* Cut off the port, identified by a ','. */
	*( pcFindNComma( cTmpStrg, 1, LEN_GSM_TCP_CONN_CFG ) - 1 ) = 0;

	/* Add the TCP socket ID. */
	xComSendString( COM_GSM, cTmpStrg );
}
/*-----------------------------------------------------------*/

/* Add TCP socket ID parameter. */
void prvAddTcpSocketId( void )
{
	/* Add the TCP socket ID. */
	xComSendString( COM_GSM, cGsmTcpSocketId );
}
/*-----------------------------------------------------------*/

/* Add FTP server URL. */
void prvAddSrvUrl( void )
{
	signed char		cTmpStrg[ LEN_GSM_TCP_CONN_CFG + 1 ];
	
	/* The server ID is a URL. Add it to the current FTP open command. */
	( void )pcConfigReadString( xNvdsConfig.pcFTPServerURL, cTmpStrg, LEN_GSM_TCP_CONN_CFG + 1, TRANSLATE_APOSTR );
	
	/* Send the parameter. */
	xComSendString( COM_GSM, cTmpStrg );
}
/*-----------------------------------------------------------*/

/* Add FTP user ID for logon. */
void prvAddFtpAcc( void )
{
	/* Add the user ID to the current FTP configuration command. */
	xComSendString( COM_GSM, cFTPUid );
	xComSendChar( COM_GSM, ',' );
	
	/* Add the password to the current FTP configuration command. */
	xComSendString( COM_GSM, cFTPPw );
}
/*-----------------------------------------------------------*/

/* Add generic FTP PUT remote and local filenames for file upload to FTP server. */
void prvAddFtpPutFilenames( void )
{
	/* Add the local name to the current FTP configuration command. */
	xComSendString( COM_GSM, pcFTPTargetFileName );
	xComSendChar( COM_GSM, ',' );
	xComSendString( COM_GSM, pcFTPSourceFileName );
}
/*-----------------------------------------------------------*/

/* Add generic FTP GET remote and local filenames for file download from FTP server. */
void prvAddFtpGetFilenames( void )
{
	/* Add the local name to the current FTP configuration command. */
	xComSendString( COM_GSM, pcFTPSourceFileName );
	xComSendChar( COM_GSM, ',' );
	xComSendString( COM_GSM, pcFTPTargetFileName );
}
/*-----------------------------------------------------------*/

/* Add FTP configuration string. */
void prvAddFtpCfg( void )
{
	signed char		cTmpStrg[ FTP_MAX_CFG + 1 ];

	/* Add the configuration string to the FTP configuration command. */
	( void )pcConfigReadString( xNvdsConfig.pcFTPCfg, cTmpStrg, FTP_MAX_CFG + 1, TRANSLATE_APOSTR );
	xComSendString( COM_GSM, cTmpStrg );
}
/*-----------------------------------------------------------*/

/* Add generic parameters to local filesystem listing. The parameters are in the cServerCmdParams string. */
void prvAddFLstParams( void )
{
	signed char 	*pcChar;
	
	/* Tell the parser that we want the following strings from the module to be stored. */
	pcExpectedParamStrg = cServerCmdParams;
    uxExpectedParamLen = MAX_LEN_PARAMS;	
	bStoreSeveralStrings = true;
	
	/* Distinguish between generic and specific queries. In the former, a '*' name pattern is added. */
	if ( cServerCmdParams[ 1 ] == '}' )
	{
		xComSendString( COM_GSM, "\"*\"" );
	}
	else
	{
		/* Remove the residual server command closing bracket from the parameters. */
		pcChar = &cServerCmdParams[ strlen( cServerCmdParams ) - 1 ];
		if ( *pcChar == '}' )
		{
			*pcChar = 0;
		}
		/* Attach the remainder directly as parameter string while translating ' to ". */
		vTranslateSingleQuote( cServerCmdParams );
		xComSendString( COM_GSM, cServerCmdParams + 2 );
	}
}
/*-----------------------------------------------------------*/

/* Add generic parameters to deleting a local file. The parameters are in the cServerCmdParams string. */
void prvAddFDelParams( void )
{
	signed char 	*pcChar;
	
	/* Remove the residual server command closing bracket from the parameters. */
	pcChar = &cServerCmdParams[ strlen( cServerCmdParams ) - 1 ];
	if ( *pcChar == '}' )
	{
		*pcChar = 0;
	}
	
	/* Attach the remainder directly as parameter string while translating ' to ". */
	vTranslateSingleQuote( cServerCmdParams );
	xComSendString( COM_GSM, cServerCmdParams  );
}
/*-----------------------------------------------------------*/

/* Add required RAT scan sequence to the +QCFG command. */
void prvAddRatScanSequence( void )
{
	unsigned short			usRequiredRATs;
	unsigned portBASE_TYPE	uxRat1st;
	unsigned portBASE_TYPE	uxRat2nd;
	unsigned portBASE_TYPE	uxRat3rd;
	
	/* Get the list of required RATs from the NVDS. */
	usRequiredRATs = usConfigReadShort( &xNvdsConfig.usCfgRat );
	
	/* Map the requirements to QTEL configurations. */
	uxRat1st = ( usRequiredRATs >> 8 ) & 0xf;
	uxRat2nd = ( usRequiredRATs >> 4 ) & 0xf;
	uxRat3rd = ( usRequiredRATs >> 0 ) & 0xf;
	
	/* Derive the desired scanseq. */
	switch ( uxRat1st )
	{
		case RAT_GPRS:		xComSendChar( COM_GSM, '1' );		break;
		case RAT_LTE_M1:	xComSendChar( COM_GSM, '2' );		break;
		case RAT_LTE_NB1:	xComSendChar( COM_GSM, '3' );		break;
		default:			xComSendChar( COM_GSM, '2' );		break;			/* M1 */
	}
	xComSendChar( COM_GSM, '0' );
	switch ( uxRat2nd )
	{
		case RAT_GPRS:		xComSendChar( COM_GSM, '1' );		break;
		case RAT_LTE_M1:	xComSendChar( COM_GSM, '2' );		break;
		case RAT_LTE_NB1:	xComSendChar( COM_GSM, '3' );		break;
		default:			xComSendChar( COM_GSM, '1' );		break;			/* GPRS */
	}
	xComSendChar( COM_GSM, '0' );
	switch ( uxRat3rd )
	{
		case RAT_GPRS:		xComSendChar( COM_GSM, '1' );		break;
		case RAT_LTE_M1:	xComSendChar( COM_GSM, '2' );		break;
		case RAT_LTE_NB1:	xComSendChar( COM_GSM, '3' );		break;
		default:			xComSendChar( COM_GSM, '3' );		break;			/* NB1 */
	}	
}
/*-----------------------------------------------------------*/

/* Add required RAT scan sequence to the +QCFG command. */
void prvAddRatScanMode( void )
{
	unsigned short			usRequiredRATs;
	unsigned portBASE_TYPE	uxRat1st;
	unsigned portBASE_TYPE	uxRat2nd;
	unsigned portBASE_TYPE	uxRat3rd;
	bool					bRatLteM1;
	bool					bRatLteNB1;
	bool					bRatGprs;

	/* Get the list of required RATs from the NVDS. */
	usRequiredRATs = usConfigReadShort( &xNvdsConfig.usCfgRat );
	
	/* Map the requirements to QTEL configurations. */
	uxRat1st = ( usRequiredRATs >> 8 ) & 0xf;
	uxRat2nd = ( usRequiredRATs >> 4 ) & 0xf;
	uxRat3rd = ( usRequiredRATs >> 0 ) & 0xf;

	/* First, identify which RATs are required at all. */
	bRatLteM1  = ( uxRat1st == RAT_LTE_M1 )  || ( uxRat2nd == RAT_LTE_M1 )  || ( uxRat3rd == RAT_LTE_M1 );
	bRatLteNB1 = ( uxRat1st == RAT_LTE_NB1 ) || ( uxRat2nd == RAT_LTE_NB1 ) || ( uxRat3rd == RAT_LTE_NB1 );
	bRatGprs   = ( uxRat1st == RAT_GPRS )    || ( uxRat2nd == RAT_GPRS )    || ( uxRat3rd == RAT_GPRS );
	
	/* Derive desired nwscanmode configuration. */
	if ( ( bRatLteM1 || bRatLteNB1 ) && bRatGprs )
	{
		/* Automatic. */
		xComSendChar( COM_GSM, '0' );
	}
	if ( !( bRatLteM1 || bRatLteNB1 ) && bRatGprs )
	{
		/* GPRS only. */
		xComSendChar( COM_GSM, '1' );
	}
	if ( ( bRatLteM1 || bRatLteNB1 ) && !bRatGprs )
	{
		/* LTE only. */
		xComSendChar( COM_GSM, '3' );
	}
}
/*-----------------------------------------------------------*/

/* Add required network category to be searched under LTE RAT to the +QCFG command. */
void prvAddRatIotMode( void )
{
	unsigned short			usRequiredRATs;
	unsigned portBASE_TYPE	uxRat1st;
	unsigned portBASE_TYPE	uxRat2nd;
	unsigned portBASE_TYPE	uxRat3rd;
	bool					bRatLteM1;
	bool					bRatLteNB1;
	
	/* Get the list of required RATs from the NVDS. */
	usRequiredRATs = usConfigReadShort( &xNvdsConfig.usCfgRat );	
	
	/* Map the requirements to QTEL configurations. */
	uxRat1st = ( usRequiredRATs >> 8 ) & 0xf;
	uxRat2nd = ( usRequiredRATs >> 4 ) & 0xf;
	uxRat3rd = ( usRequiredRATs >> 0 ) & 0xf;
	
	/* First, identify which RATs are required at all. */
	bRatLteM1  = ( uxRat1st == RAT_LTE_M1 )  || ( uxRat2nd == RAT_LTE_M1 )  || ( uxRat3rd == RAT_LTE_M1 );
	bRatLteNB1 = ( uxRat1st == RAT_LTE_NB1 ) || ( uxRat2nd == RAT_LTE_NB1 ) || ( uxRat3rd == RAT_LTE_NB1 );
	
	/* Derive network category to be searched under LTE RAT. */
	if ( !bRatLteM1 && bRatLteNB1 )
	{
		/* NB1 only. */
		xComSendChar( COM_GSM, '1' );
	}
	else
	{
		if ( bRatLteM1 && bRatLteNB1 )
		{
			/* M1 and NB1. */
			xComSendChar( COM_GSM, '2' );
		}
		else
		{
			/* Default: M1 only. */
			xComSendChar( COM_GSM, '0' );
		}
	}
}
/*-----------------------------------------------------------*/

/* Calculate the length of a HTTP location request to the XSole server based on cellular information
   and add the result to the command.

   The complete URL of the HTTP GET request (example):
		http://local.xsole.net:8088/api/cell_towers/location?mcc=208&net=1&area=984&cell=a25706&format=ascii
				
   The fix part of this request ("http://local.xsole.net:8088/api/cell_towers/location?format=ascii") has a length of 65 characters.
*/
void prvAddLocReqLen( void )
{
	unsigned portBASE_TYPE		uxlocLen;
	signed char					sStrgBuffer[ 4 ];
	
	uxlocLen = 59;											/* "http://local.xsole.net:8880/api/cell_towers/location?format=ascii" */
	
	if ( cGsm_MCC[ 0 ] != 0 )
 	{
		uxlocLen += 5 + strlen( cGsm_MCC );					/* "mcc=" + <mcc> + "&" */

		if ( cGsm_MNC[ 0 ] != 0 )
		{
			uxlocLen += 5 + strlen( cGsm_MNC );				/* "net=" + <mnc> + "&" */
		}
		
		if ( cGsm_TAC[ 0 ] != 0 )
		{
			uxlocLen += 6 + strlen( cGsm_TAC );				/* "area=" + <tac> + "&" */
		}
		
		if ( cGsm_CI[ 0 ] != 0 )
		{
			uxlocLen += 6 + strlen( cGsm_CI );				/* "cell=" + <ci> + "&" */
		}
	}
	vByteToIntStrg( sStrgBuffer, uxlocLen, true );
	xComSendString( COM_GSM, sStrgBuffer);
	
	/* 80s timeout for the HTTP GET */
	xComSendString( COM_GSM, "," TO_GSM_HTTPGET "\r\n" );	
}

/*-----------------------------------------------------------*/

/* Calculate the length of a HTTP AssistNow request to the ublox server and add the result to the command.

   The complete URL of the HTTP GET request (example):
		https://online-live1.services.u-blox.com/GetOnlineData.ashx?token=JUPt9E_AqU2T4zCMBruKTg;gnss=gps,qzss,glo,gal;datatype=eph,alm,aux,pos;filteronpos;lat=46;lon=6;alt=0;pacc=800000
		https://online-live2.services.u-blox.com/GetOnlineData.ashx?token=JUPt9E_AqU2T4zCMBruKTg;gnss=gps,qzss,glo,gal;datatype=eph,alm,aux,pos;filteronpos;lat=;lon=;alt=;pacc=
				
   The fix part of this request ("https://online-live1.services.u-blox.com/GetOnlineData.ashx?token=JUPt9E_AqU2T4zCMBruKTg;gnss=gps,qzss,glo,gal;datatype=eph,alm,aux,pos;filteronpos;lat=;lon=;alt=;pacc=")
   has a length of 168 characters.
   
   The format of the location estimate is: "46,2,0,800000"   
*/
void prvAddANReqLen( void )
{
	unsigned portBASE_TYPE		uxlocLen;
	signed char					cStrgBuffer[ 4 ];
	
	/* The base request length. */
	uxlocLen = 168;
	
	/* Add the length of the location estimate minus the separating commas (3). */
	uxlocLen += strlen( cGpsInitialPosEstimate ) - 3;		

	vByteToIntStrg( cStrgBuffer, uxlocLen, true );
	xComSendString( COM_GSM, cStrgBuffer );
	
	/* 80s timeout for the HTTP GET */
	xComSendString( COM_GSM, "," TO_GSM_HTTPGET "\r\n" );	
}
/*-----------------------------------------------------------*/
 
/* Request the Parser to store the next string from the GSM module in the parameter string buffer. 
   This mechanism is used for (amongst others): version string, CCID, ... */
void vReqStorage( void )
{
	/* Tell the parser that we want the next string from the module to be stored. 
	   By default, only one string is stored. */
	pcExpectedParamStrg = cServerCmdParams;
    uxExpectedParamLen = MAX_LEN_PARAMS;	
	bStoreSeveralStrings = false;
}
/*-----------------------------------------------------------*/
 
/* Request the Parser to store the next strings from the GSM module in the parameter string buffer. 
   This mechanism is used for the version strings.
   The next 'OK' message terminates the storage. */
void prvReqMultipleStorage( void )
{
	/* Tell the parser that we want the next string from the module to be stored. 
	   By default, only one string is stored. */
	pcExpectedParamStrg = cServerCmdParams;
    uxExpectedParamLen = MAX_LEN_PARAMS;	
	bStoreSeveralStrings = true;
}
/*-----------------------------------------------------------*/

/* Send one AT command to the GSM/GPS module. 
   The command is sent according to the specification in xAtCmd
   (number of repetitions, good/bad response, function to add 
   a parameter string). 
   Parameters:
		pxAtCmd				Structure containing all parameters for the
							AT comand to be sent and the response to be received.
   Returns an AT_RESULT: 
		AT_SUCCESS 
		AT_RESPONSE_TO
		AT_RESPONSE_ERR
		AT_MODULE_ERROR  
   
   The AT command structure defines up to two responses required to pass successfully. In case two ar defined,
   the implementation of the function only requires two of the messages to arrive in any sequence (even twice
   the same) to return success.
   
   In case the defined error response is received, the function exits without waiting for other messages
   to arrive. The caller has to make sure all expected subsequent messages are flushed before continuing.
   
   In case of AT_MODULE_ERROR, a serious error resulted from the command and usually the 
   module needs to be restarted. 
*/
enum xAT_RESULT xSendAtCommand( const struct xAT_CMD *pxAtCmd, bool bCrLf, bool bReportError )
{
	void					( *pvAddParameter )( void );
	TickType_t				xGsmBlockTime;
	TickType_t				xStartTime;
	unsigned portBASE_TYPE	xRepetitions;
	enum xAT_RESULT			xAtResult;
	
	/* Remove all previous AT responses from the queue to make sure our state machine is 
	   synchronised with the GSM module. Normally there should not be any left at this point. 
	   Server commands and TCP RX data indications are conserved. */
	vRemoveAllMsgsFromQueue( xParserAtRespQueue );
	
	xRepetitions = pxAtCmd->xRepetitions;
	
	xAtResult = AT_MODULE_ERR;
	
	while ( xRepetitions-- > 0 )
	{
		/* Add a short delay to make sure the GSM module will correctly receive the command. */
		vTaskDelay( 2 );
		
		/* Send base command. */
		xComSendString( COM_GSM, "AT" );
		xComSendString( COM_GSM, pxAtCmd->pcAtCommand );
		
		/* Initialise the length of additional expected parameters to 0. This value might be modified later
		   in the optional command-specific function which allows to add parameters to the command or 
		   configure the command return. */
		uxExpectedParamLen = 0;
		bStoreSeveralStrings = false;
		
		pvAddParameter = ( void ( * )( void ) )pxAtCmd->prvAddParam;
		if ( pvAddParameter != NULL )
		{
			/* Call the function specific for the parameter to be added. */
			pvAddParameter();
		}
		
		if ( bCrLf )
		{
			/* Add CRLF. */
			xComSendString( COM_GSM, "\r\n" );
		}
		
		/* Wait for the first response from the module. 
		   xStartTime is used to record when we first started waiting for a 
		   reponse. If we receive a response which bears no relevance to the 
		   ongoing AT request, we use this time to calculate the remaining wait
		   time so that the time-out is not refreshed. */		   
		xGsmBlockTime = pxAtCmd->uiTimeOut;
		xStartTime = xTaskGetTickCount();
		xAtResult = xReceiveResponse( pxAtCmd->xExpectAtRespMsgId[ 0 ],
								      pxAtCmd->xExpectAtRespMsgId[ 1 ],
									  pxAtCmd->xErrorAtRespMsgId,
									  xGsmBlockTime );
		
		if ( ( xAtResult == AT_SUCCESS ) && ( pxAtCmd->xExpectAtRespMsgId[ 1 ] != AT_NOMSG ) )
		{
			/* If the first response was successfully received and a second one is required, wait for the 
			   second one with the remaining block time. */
			xGsmBlockTime = pxAtCmd->uiTimeOut - ( xStartTime - xTaskGetTickCount() );
			if ( xGsmBlockTime > ( TickType_t )0 )
			{
				xAtResult = xReceiveResponse( pxAtCmd->xExpectAtRespMsgId[ 0 ],
											  pxAtCmd->xExpectAtRespMsgId[ 1 ],
									 		  pxAtCmd->xErrorAtRespMsgId,
											  xGsmBlockTime );
				if ( bReportError )
				{
					if ( xAtResult == AT_MODULE_ERR )
					{
						V_TRACE_PRINT_STRG( TRACE_GSM_AT_MODULE_ERR, ( signed char * )pxAtCmd->pcAtCommand, TRACE_UART_AND_FILE );
						V_TRACE_PRINT_STRG( TRACE_GSM_AT_MODULE_ERR, cServerCmdParams, TRACE_UART_AND_FILE );
					}
					if ( xAtResult == AT_RESPONSE_ERR )
					{
						V_TRACE_PRINT_STRG( TRACE_GSM_AT_RESPONSE_ERR, ( signed char * )pxAtCmd->pcAtCommand, TRACE_UART_AND_FILE );
					}
				}
					
				if ( ( xAtResult == AT_SUCCESS ) || ( xAtResult == AT_MODULE_ERR ) || ( xAtResult == AT_RESPONSE_ERR ) )
				{
					/* We received a second response. Return the result. */
					return xAtResult;
				}
			}
		}
		
		if ( bReportError )
		{
			if ( xAtResult == AT_MODULE_ERR )
			{
				V_TRACE_PRINT_STRG( TRACE_GSM_AT_MODULE_ERR, ( signed char * )pxAtCmd->pcAtCommand, TRACE_UART_AND_FILE );
			}
			if ( xAtResult == AT_RESPONSE_ERR )
			{
				V_TRACE_PRINT_STRG( TRACE_GSM_AT_RESPONSE_ERR, ( signed char * )pxAtCmd->pcAtCommand, TRACE_UART_AND_FILE );
			}
		}
		
		if ( ( xAtResult == AT_SUCCESS ) || ( xAtResult == AT_MODULE_ERR ) || ( xAtResult == AT_RESPONSE_ERR ) )
		{
			/* Only one response was required and we received a result. */
			return xAtResult;
		}
	}
	
	if ( bReportError )
	{
		/* Return an error: no valid response after pxAtCmd->xRepetitions attempts. */
		V_TRACE_PRINT_STRG( TRACE_GSM_AT_RESPONSE_TO, ( signed char * )pxAtCmd->pcAtCommand, TRACE_UART_AND_FILE );
	}
	
	return xAtResult;
}
/*-----------------------------------------------------------*/

/* Send a batch of AT commands to the GSM module. */
enum xAT_RESULT xSendBatchAtCommand( const struct xAT_CMD *xAtGSMCmdSeq, unsigned portBASE_TYPE xNumCmds, bool bReportTO )
{
	enum xAT_RESULT			xInitResult;
	unsigned char 			xCmdIdx;

	xInitResult = AT_SUCCESS;	

	for ( xCmdIdx = 0; ( xCmdIdx < xNumCmds ) && ( xInitResult == AT_SUCCESS ); xCmdIdx++ )
	{
		/* Send the command. */
		xInitResult = xSendAtCommand( xAtGSMCmdSeq + xCmdIdx, true, bReportTO );
	}	
	
	return xInitResult;
}
/*-----------------------------------------------------------*/

/* Request an initial geoposition estimation from the server based on the cellular connnection data. */
void vRequestInitialPosition( void )
{
	unsigned portBASE_TYPE	uxRun;
	bool					bResult;

	/* Check if the current initial position estimate is still valid. If not, request a new estimate from the server. */
	if ( ( signed short )usReadRTC() < ( signed short )usGpsInitialPosEstimateTS + INITIAL_POS_VALIDITY )
	{
		return;
	}
	
	V_TRACE_PRINT( TRACE_GSM_CELLULAR_POSITION_REQUEST, TRACE_UART_AND_FILE );
	
	/* Query the module's cellular network connection data and environment. */ 
	if ( xSendBatchAtCommand( xAtStat, sizeof( xAtStat ) / sizeof( struct xAT_CMD ), true ) != AT_SUCCESS )
	{
		V_TRACE_PRINT( TRACE_GSM_LOST_NETWORK_CONNECTION, TRACE_UART_AND_FILE );
		return;
	}
		
	/* Try a number of times to get a rough location estimation from the server. */
	uxRun = 0;
	do
	{
		signed char				cTmpStrg[ LEN_GPS_LOCSRV_ADDR + 1 ];

		uxRun++;

		/* Construct the HTTP GET query to the server with the data and obtain the recovered location data file size in cServerCmdParams. */
		if ( xSendBatchAtCommand( xAtGetLocEst0, sizeof( xAtGetLocEst0 ) / sizeof( struct xAT_CMD ), true ) != AT_SUCCESS )
		{
			/* Skip all other steps if we received an error already here. */
			continue;			
		}
		
		/* If we get here, we have received the "CONNECT" and can now send the HTTP request. 
		   Note that the fields to beput into the request cannot change as only the GSM process will launch their update. */
		xComSendString( COM_GSM, "http://" );
		( void )pcConfigReadString( xNvdsConfig.pcGPSLocEstSrvAddr, cTmpStrg, LEN_GPS_LOCSRV_ADDR + 1, TRANSLATE_APOSTR );
		xComSendString( COM_GSM, cTmpStrg );
		xComSendString( COM_GSM, ":8880/api/cell_towers/location?" );
		if ( cGsm_MCC[ 0 ] != 0 )
		{
			xComSendString( COM_GSM, "mcc=" );
			xComSendString( COM_GSM, cGsm_MCC );

			if ( cGsm_MNC[ 0 ] != 0 )
			{
				xComSendString( COM_GSM, "&net=" );
				xComSendString( COM_GSM, cGsm_MNC );
			}
			
			if ( cGsm_TAC[ 0 ] != 0 )
			{
				xComSendString( COM_GSM, "&area=" );
				xComSendString( COM_GSM, cGsm_TAC );
			}
			
			if ( cGsm_CI[ 0 ] != 0 )
			{
				xComSendString( COM_GSM, "&cell=" );
				xComSendString( COM_GSM, cGsm_CI );
			}
			
			xComSendChar( COM_GSM, '&' );
		}
		
		xComSendString( COM_GSM, "format=ascii\r\n" );
		
		/* Wait for the OK. */
		bResult = xReceiveResponse( AT_OK, AT_NOMSG, AT_NOMSG, TO_GSM_AT ) == AT_SUCCESS;
		
		/* Read back the URL just written for debug purposes (it appears that the QTEL sometimes does not understand the URL correctly). */
		( void )xSendAtCommand( &xAtQHttpUrlQ, true, true );
		
		if ( bResult )
		{
			/* Execute the HTTP GET query to the server and wait for the 'OK'. The result is stored in cServerCmdParams. */
			bResult = xSendBatchAtCommand( xAtGetLocEst1, sizeof( xAtGetLocEst1 ) / sizeof( struct xAT_CMD ), true ) == AT_SUCCESS;
		}
	}
	while ( !bResult && ( uxRun < MAX_GPS_ASSISTED_DATA_DOWNLOAD_ATTEMPTS ) );

	if ( !bResult )
	{
		/* No estimate could be obtained from the server. Return without touching cGpsInitialPosEstimate[] 
		   as it contains the last acquired GPS position. */
 	    V_TRACE_PRINT( TRACE_GSM_CELLULAR_POSITION_REQ_FAILED, TRACE_UART_AND_FILE );
		return;
	}
	
	/* Copy the wanted string enclosed by inverted commas to the location string.
	   The text received from the GSM module, as response to the file read, is after having been 
	   split into several strings by the UART driver: 
			Tx	AT+QHTTPREAD
			Rx	CONNECT\0
			Rx	"43.621134,7.063414,0,4059"\0
			Rx	OK\0	
			Rx	+QHTTPREAD: 0\0
	   Here, we are interested in the string in between the quotation marks. The 'OK' is required to end the sequence.
	   
	   The command triggers prvReqMultipleStorage() to store intermediate results until the next 'OK' 
	   message is received. The result is in cServerCmdParams[] in the form: 
	             0 1
				'\n"43.621134,7.063414,0,4059"'	
				
	   The first part until the quotation mark is fix. The 'CONNECT' string has been converted into a message and thus 
	   consumed. It is therefore not present here. */
	
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );

	if (    ( cServerCmdParams[ 1 ] == '"' )
		 && isdigit( cServerCmdParams[ 2 ] ) )
	{
		unsigned portBASE_TYPE	uxStartIdx;
		unsigned long 			ulAcc = 100000;

		/* We got the expected string. Unless it is has a precision above 100km, copy it from position 1 to the next inverted comma. 
		   Else, do not touch cGpsInitialPosEstimate[] as it contains the last acquired GPS position. */
		/* Locate the 3rd comma. */
		uxStartIdx = strlen( cServerCmdParams + 2 );
		do
		{
			uxStartIdx--;
		}
		while (    ( uxStartIdx > 0 )
			    && ( cServerCmdParams[ 2 + uxStartIdx ] != ',' ) );
			
		/* Use the rest of the string as accuracy value. */
		if ( cServerCmdParams[ 2 + uxStartIdx ] == ',' )
		{
			ulAcc = ulIntStrgToLong( cServerCmdParams + 2 + uxStartIdx + 1, 7 );
		}
		
		/* Use the returned value, if:
				- The accuracy is less that 50km.
		   OR:	- The local position estimation has not been updated yet. */
		if (    ( ulAcc < 50000 ) 
			 || !bGpsInitialPosEstimateInitialised )
		{
			uxStartIdx = 0;
			while (    ( cServerCmdParams[ 2 + uxStartIdx ] != '"' ) 
					&& ( uxStartIdx < INITIAL_POS_LENGTH + 1 ) )
			{
				cGpsInitialPosEstimate[ uxStartIdx  ] = cServerCmdParams[ 2 + uxStartIdx ];
				uxStartIdx++;
			}
			cGpsInitialPosEstimate[ uxStartIdx ] = 0;
		}
	}
	
	xSemaphoreGive( xMutexGsmData );
}
/*-----------------------------------------------------------*/

/* Request the assistance data from the ublox server.

   The request has this format:
   https://online-live1.services.u-blox.com/GetOnlineData.ashx?token=JUPt9E_AqU2T4zCMBruKTg;gnss=gps,qzss,glo,gal;datatype=eph,alm,aux,pos;filteronpos;lat=46;lon=6;alt=0;pacc=800000
   
   The parameter sUbloxServer may be '1' or '2' and denotes the ublox server to contact with the request.
   
   The GPS receiver will be started by this procedure.
   
   The mgaonline.ubx file is streamed directly after having been downloaded from the server.
*/
bool bRequestAssistanceData( void )
{
	unsigned portBASE_TYPE	uxIdx;
	unsigned portBASE_TYPE	uxRun;
	unsigned portBASE_TYPE	uxUrlTry;
	signed char				*pcANFileLen;
	unsigned short			usAnRemainingBytes;
	TickType_t				xCharReadTime;
	bool					bGotByte;
	bool					bDataStart;
	unsigned char			ucByte;
	bool					bResult;	

	V_TRACE_PRINT( TRACE_GSM_ASSISTED_GPS_DATA_REQUEST, TRACE_UART_AND_FILE );

	uxRun = 0;
	do
	{
		uxRun++;
		uxUrlTry = 0;
		bResult = false;
		while ( ( uxUrlTry++ < 2 ) && !bResult )
		{
			/* Construct the HTTP GET query to the ublox server using the location estimation.
			   Try both alternative servers. If both fail, abandon. */
			if ( xSendBatchAtCommand( xAtGetGpsANData0, sizeof( xAtGetGpsANData0 ) / sizeof( struct xAT_CMD ), true ) != AT_SUCCESS )
			{
				/* Skip all other steps if we received an error already here. */
				continue;
			}
			
			/* If we get here, we have received the "CONNECT" and can now send the HTTP request. 
			   Note that the fields to beput into the request cannot change as only the GSM process will launch their update. 
			   There are two ublox servers. The second is tested if the first one failed. */
			xComSendString( COM_GSM, "https://online-live" );
			if ( uxRun == 1 )
			{
				xComSendChar( COM_GSM, '1' );
			}
			else
			{
				xComSendChar( COM_GSM, '2' );
			}
			xComSendString( COM_GSM, ".services.u-blox.com/GetOnlineData.ashx?token=JUPt9E_AqU2T4zCMBruKTg;gnss=gps,qzss,glo,gal;datatype=eph,alm,aux,pos;filteronpos;lat=" );
			
			/* The first parameter in the cGpsInitialPosEstimate string is the latitude. */
			uxIdx = 0;
			while (    ( uxIdx < INITIAL_POS_LENGTH )
					&& ( cGpsInitialPosEstimate[ uxIdx ] != ',' )
					&& ( cGpsInitialPosEstimate[ uxIdx ] != 0 ) )
			{
				( void )xComSendChar( COM_GSM, cGpsInitialPosEstimate[ uxIdx++ ] );				
			}

			uxIdx++;
			
			/* The second parameter in the cGpsInitialPosEstimate string is the longitude. */
			( void )xComSendString( COM_GSM, ";lon=" );
			while (    ( uxIdx < INITIAL_POS_LENGTH )
					&& ( cGpsInitialPosEstimate[ uxIdx ] != ',' ) 
					&& ( cGpsInitialPosEstimate[ uxIdx ] != 0 ) )
			{
				( void )xComSendChar( COM_GSM, cGpsInitialPosEstimate[ uxIdx++ ] );				
			}

			uxIdx++;
			
			/* The third parameter in the cGpsInitialPosEstimate string is the altitude. */
			( void )xComSendString( COM_GSM, ";alt=" );
			while (    ( uxIdx < INITIAL_POS_LENGTH )
					&& ( cGpsInitialPosEstimate[ uxIdx ] != ',' ) 
					&& ( cGpsInitialPosEstimate[ uxIdx ] != 0 ) )
			{
				( void )xComSendChar( COM_GSM, cGpsInitialPosEstimate[ uxIdx++ ] );				
			}

			uxIdx++;

			/* The last parameter in the cGpsInitialPosEstimate string is the horizontal accuracy. */
			( void )xComSendString( COM_GSM, ";pacc=" );
			while (    ( uxIdx < INITIAL_POS_LENGTH )
					&& ( cGpsInitialPosEstimate[ uxIdx ] != ',' ) 
					&& ( cGpsInitialPosEstimate[ uxIdx ] != 0 ) )
			{
				( void )xComSendChar( COM_GSM, cGpsInitialPosEstimate[ uxIdx++ ] );				
			}	
			xComSendString( COM_GSM, "\r\n" );
			
			/* Wait for the OK. */
			bResult = xReceiveResponse( AT_OK, AT_NOMSG, AT_NOMSG, TO_GSM_AT ) == AT_SUCCESS;
		
			/* Read back the URL just written for debug purposes (it appears that the QTEL sometimes does not understand the URL correctly). */
			( void )xSendAtCommand( &xAtQHttpUrlQ, true, true );
		}
		
		/* Execute the HTTP GET query to the server are read the result. Once the command sequence here is finished,
		   cServerCmdParams contains the results of the HTTP GET in the form of "<httprspcode>,<content_length>". */
		bResult = xSendAtCommand( &xAtGetGpsANData1, true, true ) == AT_SUCCESS;
	}
	while ( !bResult && ( uxRun < MAX_GPS_ASSISTED_DATA_DOWNLOAD_ATTEMPTS ) );

	if ( !bResult )
	{
		return false;
	}		

	/* Get the length of the AssistNow data file. */
	pcANFileLen = pcFindNComma( cServerCmdParams, 1, 8 );
	usAnRemainingBytes = usIntStrgToShort( pcANFileLen, 5 );

	/* The assistance data file should be at least 3000 bytes long, probably rather in the range of 5...7Kbytes. */
	if ( usAnRemainingBytes < 3000 )
	{
		return false;
	}		

	/* The AssistNow data is binary, so open UART in binary mode. */
	vCOMOpen( COM_GSM, cGsmUartRxBuffer, GSM_UART_RX_BUFFER_SIZE - 1, COM_BINARY );

	/* Initialise the GPS UBX message accumulator. */
	bStoreANData( 0, true );

	/* Request reading of the AN data. */
	xComSendString( COM_GSM, "AT+QHTTPREAD\r\n" );
	
	/* The AssistNow data is now being streamed. Loop over the data and send it to the BLE once the start character 0xb5
	   has been found. */
	bGotByte = true;
	bDataStart = false;
	bResult = true;
	while ( ( usAnRemainingBytes > 0 ) && bGotByte && bResult )
	{
		xCharReadTime = xTaskGetTickCount();
		do
		{
			bGotByte = bGetRxCharFromBuffer( COM_GSM, ( signed char * )&ucByte );
		}
		while ( !bGotByte && ( xTaskGetTickCount() < xCharReadTime + TO_GSM_FS ) );
		
		if ( bGotByte )
		{
			if ( ucByte == 0xb5 )
			{
				bDataStart = true;
			}
			if ( bDataStart )
			{
				/* Store the data byte in the Assistance data store. */
				bResult = bStoreANData( ucByte, false );
				usAnRemainingBytes--;
			}
		}		
	}
	
	/* Return the GSM UART to ASCII mode. */
	vCOMOpen( COM_GSM, cGsmUartRxBuffer, GSM_UART_RX_BUFFER_SIZE - 1, COM_ASCII );

	if ( bResult )
	{
		/* Set the AssistNow updated timestamp. */
		ulAssistedGPSUpdateTimeStamp = ulReadRTC();
	}

	return bResult;
}
/*-----------------------------------------------------------*/

/* Set the QTEL one-time configuration, if not already done before.

   The base configuration comprises configuring the region and power saving. These settings are required only once as QTEL stores 
   these parameters in NVDS. However, a module reboot is required for them to become effective.
   
   The procedure first checks if the module had already been confiured before. It only proceeds if the module is no still in
   virgin state.
   
   Later (if necessary), this procedure could also be used to apply changes to these parts of the parameters.
   
   The procedure assumes that the module is up and running.
*/
enum xAT_RESULT xOneTimeConfigureQTEL( void )
{
	enum xAT_RESULT			xInitResult;
	bool					bModuleConfigured;

	bModuleConfigured = false;

	/* First, query the band settings. Program only if it is set to all bands,  i.e. unconfigured. 
			+QCFG: "band",0xf,0x100002000000000f0e189f,0x10004200000000090e189f		
			                    ^    ^         ^ ^
								20   25       35 37
	   To check for the (rather lengthy) configuration, only key parameters are verified (^). */
	xInitResult = xSendAtCommand( &xAtQCfgBandQ, true, true );	
	if ( xInitResult == AT_SUCCESS )
	{
		if (    ( cServerCmdParams[ 20 ] == '1' )
			 && ( cServerCmdParams[ 25 ] == '2' )
		     && ( cServerCmdParams[ 35 ] == 'f' )
			 && ( cServerCmdParams[ 37 ] == 'e' ) )
		{
			/* Send the one-time configuration to QTEL which is followed by a reboot - part 1. */
			xInitResult = xSendAtCommand( &xAtQCfgBand, true, true );	
			if ( xInitResult != AT_SUCCESS )
			{
				return xInitResult;
			}
			
			bModuleConfigured = true;
		}
	}
			
	/* If the module has been reconfigured, power-cycle it and apply the base configuration for further operation. */
	if ( bModuleConfigured )
	{
		/* Shut-down. */
		vGSMModulePowerDown( SOFT_POWER_OFF );		
		
		vTaskDelay( 2 * portTICKS_PER_SEC );
		
		/* Power up. */
		prvGSMModulePowerUp();

		/* Reconfigure the module for operation. */
		xInitResult = xConfigureQTEL();
	}

	return xInitResult;
}
/*-----------------------------------------------------------*/

/* Check enabled RATs and sequence of preferred RATs against the configuration. Reconfigure the QTEL module if there is a mismatch.

   The RAT command has been designed along the ublox specification for the R4 series. For QTEL, the RAT configuration needs to be
   mapped to the three relevant QTEL commands.
	
		AT+QCFG="nwscanmode"[,<scan_mode>[,<effect>]]		Configure RAT(s) to be Searched
			<scan_mode> Integer type. RAT(s) to be searched.
					0 	Automatic (GSM and LTE)
					1 	GSM only
					3 	LTE only
	
		AT+QCFG="nwscanseq"[,<scanseq>[,effect]]			Configure RATs Searching Sequence
			<scanseq> Integer type. RATs search sequence. (e.g.: 020301 stands for eMTC -> NB-IoT -> GSM)
					00 	Automatic (eMTC -> NB-IoT -> GSM)
					01 	GSM
					02 	eMTC
					03 	NB-IoT
	
		AT+QCFG="iotopmode"[,<mode>[,<effect>]]				Configure Network Category to be Searched under LTE RAT
			<mode> Integer type. Network category to be searched under LTE RAT.
					0 	eMTC
					1 	NB-IoT
					2 	eMTC and NB-IoT   
					
   The current RAT command definition is:
		RAT=0x<force><1st_RAT><2nd_RAT><3rd_RAT>
			<force>
					0   run FW-controlled RAT selection
					1   let SARA/QTEL control RAT
			<RAT>	
					7   LTE-M1
					8   NB-IOT1
					9   GPRS
					0   none
	Highest priority RAT <1st RAT> cannot be <none>. 
*/
enum xAT_RESULT xSelectRAT()
{
	enum xAT_RESULT			xInitResult;
	unsigned short 			usRequiredRATs;
	unsigned portBASE_TYPE	uxRat1st;
	unsigned portBASE_TYPE	uxRat2nd;
	unsigned portBASE_TYPE	uxRat3rd;
	bool					bRatLteM1;
	bool					bRatLteNB1;
	bool					bRatGprs;
	unsigned portBASE_TYPE	uxReqNwScanMode;
	unsigned short			usReqScanSeq;
	unsigned portBASE_TYPE	uxReqIotOpMode;
	unsigned portBASE_TYPE	uxCurrNwScanMode;
	unsigned short			usCurrScanSeq;
	unsigned portBASE_TYPE	uxCurrIotOpMode;
	bool					bReconfigReq;
	
	/* Get the list of required RATs from the NVDS. */
	usRequiredRATs = usConfigReadShort( &xNvdsConfig.usCfgRat );
	
	/* Map the requirements to QTEL configurations. */
	uxRat1st = ( usRequiredRATs >> 8 ) & 0xf;
	uxRat2nd = ( usRequiredRATs >> 4 ) & 0xf;
	uxRat3rd = ( usRequiredRATs >> 0 ) & 0xf;
	
	/* First, identify which RATs are required at all. */
	bRatLteM1  = ( uxRat1st == RAT_LTE_M1 )  || ( uxRat2nd == RAT_LTE_M1 )  || ( uxRat3rd == RAT_LTE_M1 );
	bRatLteNB1 = ( uxRat1st == RAT_LTE_NB1 ) || ( uxRat2nd == RAT_LTE_NB1 ) || ( uxRat3rd == RAT_LTE_NB1 );
	bRatGprs   = ( uxRat1st == RAT_GPRS )    || ( uxRat2nd == RAT_GPRS )    || ( uxRat3rd == RAT_GPRS );
	
	/* Derive desired nwscanmode configuration. */
	if ( ( bRatLteM1 || bRatLteNB1 ) && bRatGprs )
	{
		/* Automatic. */
		uxReqNwScanMode = 0;
	}
	if ( !( bRatLteM1 || bRatLteNB1 ) && bRatGprs )
	{
		/* GPRS only. */
		uxReqNwScanMode = 1;
	}
	if ( ( bRatLteM1 || bRatLteNB1 ) && !bRatGprs )
	{
		/* LTE only. */
		uxReqNwScanMode = 3;
	}
	
	/* Derive Network Category to be Searched under LTE RAT */
	/* Default: M1. */
	uxReqIotOpMode = 0;
	if ( !bRatLteM1 && bRatLteNB1 )
	{
		uxReqIotOpMode = 1;
	}
	if ( bRatLteM1 && bRatLteNB1 )
	{
		uxReqIotOpMode = 2;
	}
	
	/* Derive the desired scan sequence. */
	switch ( uxRat1st )
	{
		case RAT_GPRS:		usReqScanSeq =  0x0100;		break;
		case RAT_LTE_M1:	usReqScanSeq =  0x0200;		break;
		case RAT_LTE_NB1:	usReqScanSeq =  0x0300;		break;
		default:			usReqScanSeq =  0x0200;		break;			/* M1 */
	}
	switch ( uxRat2nd )
	{
		case RAT_GPRS:		usReqScanSeq |= 0x0010;		break;
		case RAT_LTE_M1:	usReqScanSeq |= 0x0020;		break;
		case RAT_LTE_NB1:	usReqScanSeq |= 0x0030;		break;
		default:			usReqScanSeq |= 0x0010;		break;			/* GPRS */
	}
	switch ( uxRat3rd )
	{
		case RAT_GPRS:		usReqScanSeq |= 0x0001;		break;
		case RAT_LTE_M1:	usReqScanSeq |= 0x0002;		break;
		case RAT_LTE_NB1:	usReqScanSeq |= 0x0003;		break;
		default:			usReqScanSeq |= 0x0003;		break;			/* NB1 */
	}
	
	/* Compare the module's current settings with the requirement. */
	/* Query the module's current network scan sequence setting and compare it with the required settings. 
								 0 2 4
			 +QCFG: "nwscanseq",020103						*/
	usCurrScanSeq = 0;
	xInitResult = xSendAtCommand( &xAtQNwScanSeqQ, true, true );
	if ( xInitResult == AT_SUCCESS )
	{
		/* Parse the module's response and fill in the current RAT settings. */
		if ( cServerCmdParams[ 0 ] != 0 )
		{
			usCurrScanSeq += ( cServerCmdParams[ 0 ] - '0' ) << 8;
			if ( ( cServerCmdParams[ 1 ] != 0 ) && ( cServerCmdParams[ 2 ] != 0 ) )
			{
				usCurrScanSeq += ( cServerCmdParams[ 2 ] - '0' ) << 4;
				if ( ( cServerCmdParams[ 3 ] != 0 ) && ( cServerCmdParams[ 4 ] != 0 ) )
				{
					usCurrScanSeq += ( cServerCmdParams[ 4 ] - '0' );
				}		
			}
		}
	}
	else
	{
		return xInitResult;
	}
	bReconfigReq = ( usCurrScanSeq != usReqScanSeq );
	
	/* Query the module's current network scan mode setting and compare it with the required settings. 
								 0
			 +QCFG: "nwscanmode",0						*/
	uxCurrNwScanMode = 0;
	xInitResult = xSendAtCommand( &xAtQNwScanModeQ, true, true );
	if ( xInitResult == AT_SUCCESS )
	{
		/* Parse the module's response and fill in the current RAT settings. */
		if ( cServerCmdParams[ 0 ] != 0 )
		{
			uxCurrNwScanMode = ( cServerCmdParams[ 0 ] - '0' );
		}
	}
	else
	{
		return xInitResult;
	}
	bReconfigReq |= ( uxCurrNwScanMode != uxReqNwScanMode );
	
	/* Query the module's current IoT scan mode setting and compare it with the required settings. 
								0
			 +QCFG: "iotopmode",2						*/
	uxCurrIotOpMode = 0;
	xInitResult = xSendAtCommand( &xAtQIotOpModeQ, true, true );
	if ( xInitResult == AT_SUCCESS )
	{
		/* Parse the module's response and fill in the current RAT settings. */
		if ( cServerCmdParams[ 0 ] != 0 )
		{
			uxCurrIotOpMode = ( cServerCmdParams[ 0 ] - '0' ) << 8;
		}
	}
	else
	{
		return xInitResult;
	}
	bReconfigReq |= ( uxCurrIotOpMode != uxReqIotOpMode );
	
	
	/* If a difference in the RAT settings has been found. Reconfigure the QTEL module. */
	if ( bReconfigReq )
	{
		xInitResult = xSendBatchAtCommand( xAtGSMSelectRAT, sizeof( xAtGSMSelectRAT ) / sizeof( struct xAT_CMD ), true );
	}
	else
	{
		xInitResult = AT_SUCCESS;
	}	
	
	return xInitResult;
}
/*-----------------------------------------------------------*/

/* Perform the base start-up configuration of QTEL. */
enum xAT_RESULT xConfigureQTEL( void )
{
	enum xAT_RESULT			xInitResult;

	/* Send GSM initialisation commands - part 0. */
	xInitResult = xSendBatchAtCommand( xAtGSMCmd_P0, sizeof( xAtGSMCmd_P0 ) / sizeof( struct xAT_CMD ), true );
	if ( xInitResult != AT_SUCCESS )
	{
		/* Something really bad has happened, including the module being unable to identify the SIM.
		   According to the Quectel specification (BG95&BG77_TCP/IP_Application_Note V1.0), the module must be rebooted. */
		return AT_MODULE_STUCK;
	}
	
	/* Send GSM initialisation commands - part 1. */
	xInitResult = xSendBatchAtCommand( xAtGSMCmd_P1, sizeof( xAtGSMCmd_P1 ) / sizeof( struct xAT_CMD ), true );
	
	if ( xInitResult == AT_SUCCESS )
	{
		/* We should have captured the IMEI in the cServerCmdParams buffer from the +CGSN command in the xAtGSMCmd_P0 command sequence. 
		   Check if the first entry is a valid digit. */
		if ( ( cServerCmdParams[ 0 ] >= '0' ) && ( cServerCmdParams[ 0 ] <= '9' ) && ( strlen( cServerCmdParams ) == LEN_GSM_ID ) )
		{
			unsigned portBASE_TYPE		uxIdx;
			bool						bIdDifferent;
			
			/* The IMEI appears to be good. Check, if it is the same one as we have already stored in NVDS. */
			bIdDifferent = false;
			for ( uxIdx = 0; uxIdx < LEN_GSM_ID; uxIdx++ )
			{
				if ( cServerCmdParams[ uxIdx ] != ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID + uxIdx ) )
				{
					bIdDifferent = true;
				}
			}
			
			/* If the copy in NVDS is different from the one just received, overwrite it with the received one. */
			if ( bIdDifferent )
			{
				/* Just to be sure... */
				cServerCmdParams[ LEN_GSM_ID ] = 0;
			
				vConfigWriteString( xNvdsConfig.pcGsm_ID, cServerCmdParams );
			}
		}
	}
	
	return xInitResult;
}
/*-----------------------------------------------------------*/

/* Prepare the cellular module for a reboot. 
   Set the statistics counters, empty the AT response queue and shut down the cellular module.
*/
void prvPrepareCellularModuleReboot( enum xGSM_PWRDOWN_TYPE xType )
{
	/* Increment the statistics counter. */
	uStats.xStats.usGsmModuleReboot++;

	/* Remove any pending messages from the AT command queue, especially the 'OK' which remains after an error message. 
	   First wait for the 'OK' response to appear (which might take a couple of ms). */
	vTaskDelay( 0.2 * portTICKS_PER_SEC );
	vRemoveAllMsgsFromQueue( xParserAtRespQueue );
	
	/* Hard power-cycle the GSM module. */
	vGSMModulePowerDown( xType );

	xModuleState = GSM_PWR_OFF | GPS_PWR_OFF | GSM_PDP_DEACT | GSM_TCP_DISC | GSM_TLS_DISC;
}
/*-----------------------------------------------------------*/

/* Start and configure the GSM/GPS module. 
   The function takes as parameter an enum telling if a cold or a warm start is required. 
   Returns:
		CONNECT_SUCCESS		all OK
		CONNECT_NO_NW		module did not find a network to attach to
		CONNECT_FAILED		all other failure cases
	Except for success, the cellular module is always off when leaving this function.
	Except for success, the cellular module is always off when leaving this function.

   	 true when successful (always the case).
   Except for success, the cellular module is always off when leaving this function.

   	 true when successful (always the case).
*/
enum xGPRS_CONNECT_RESULT xStartCellularModule( void )
{
	enum xAT_RESULT			xInitResult;
	TickType_t				xStartTime;
	bool					bCellularModuleCrashed;
	
	xInitResult = AT_MODULE_ERR;
	
	/* First perform the module power-up and local initialisation sequence. Under any normal circumstances, this is not supposed to fail.
	   Do this forever until there is a success. */
	while ( xInitResult != AT_SUCCESS )
	{
		/* Clear the contents of the reponse parameters. If we get a CME ERROR now, we capture the error report. */
		cServerCmdParams[0] = 0;
		
		/* Loop as long as the module is not started. */
		while ( ( xModuleState & GSM_PWR_MSK ) == GSM_PWR_OFF )
		{
			/* Make sure the battery is fit for operation (voltage high enough and temperature within
			   operating range). As we are in a while-loop here, we need to continuously monitor
			   conditions. */
			vCheckVoltageAndTemperature();
			
			/* Power up the module. */
			prvGSMModulePowerUp();

			/* Once the module is powered up, check again if the voltage is still good. 
			   If not, the function will turn the module off again. */
			vCheckVoltageAndTemperature();
		}
		
		/* Send GSM initialisation commands - part 0. */
		xInitResult = xConfigureQTEL();
		
		/* Check, if the GSM module boot was successful. */
		if ( xInitResult == AT_SUCCESS )
		{
			/* Write the NVDS log buffer content (if any) to the trace file. */
			vTraceFlushLog();
			
			/* Check if the one-time configuration of QTEL has been done. If not, configure it. */
			xInitResult = xOneTimeConfigureQTEL();
			
			/* Select (preferred) RAT for this connection. */
			if ( xInitResult == AT_SUCCESS )
			{
				xInitResult = xSelectRAT();
			}
		}

		/* In case any of the above configuration steps failed, reboot the module. */
		if ( xInitResult != AT_SUCCESS )
		{
			V_TRACE_PRINT( TRACE_GSM_UNSUCESSFUL_BOOT, TRACE_UART_AND_FILE );
			prvPrepareCellularModuleReboot( HARD_POWER_OFF );
		}
	}
		
	/* The cellular module is now running and configured. It should now start the network registration process all on its own. */
	/* Send GSM initialisation commands - part 1 for up to 300s. */
	xStartTime = xTaskGetTickCount();
	bCellularModuleCrashed = false;
	do 
	{
		xInitResult = xSendAtCommand( &xAtAt, true, true );
		if ( xInitResult != AT_SUCCESS )
		{
			/* Did not get a response to a simple AT. QTEL has crashed. */
			bCellularModuleCrashed = true;
		}
		
		/* Poll both GPRS and LTE registration results. 
		   In this particular case, the xSendAtCommand() function returns:
					AT_SUCCESS 			if there was an 'attached' response. 
					AT_RESPONSE_TO		if there was either no response at all or an 'status unknown' response.
					AT_RESPONSE_ERR		if the moule indicates that it is not searching (and in this case the OK must be polled) */
		xInitResult = xSendAtCommand( &xAtGSMCmd_P21, true, false );
		if ( xInitResult != AT_SUCCESS )
		{
			if ( xInitResult == AT_RESPONSE_ERR )
			{
				/* A response error has been received. The response error here is "not registered, the MT is 
				   not currently searching an operator to register to".
				   Wait a short time until the expected 'OK' message has been received. */
				( void )xReceiveResponse( AT_OK, AT_NOMSG, AT_NOMSG, 0.5 * portTICKS_PER_SEC );
			}

			if ( xInitResult != AT_SUCCESS )
			{
				xInitResult = xSendAtCommand( &xAtGSMCmd_P22, true, false );
				if ( xInitResult != AT_SUCCESS )
				{
					if ( xInitResult == AT_RESPONSE_ERR )
					{
						/* Wait a short time until the expected 'OK' message has been received. 
						   Retain the previous error message in xInitResult so that the while-loop
						   will continue. */
						( void )xReceiveResponse( AT_OK, AT_NOMSG, AT_NOMSG, 0.5 * portTICKS_PER_SEC );
					}

					/* DEBUG DEBUG DEBUG Query the cause for the registration failure. Relevant only in rejects. */
					( void )xSendAtCommand( &xAtQEmmCauseQ, true, false );
				}
			}
		}
	} 
	/* Continue the registration process as long as the module did not attach and the process did not time-out (or the configured
	   time-out is 0). */
	while (    ( xInitResult != AT_SUCCESS ) 
			&& !bCellularModuleCrashed
			&& (    ( xTaskGetTickCount() - xStartTime < ulConfigReadLong( &xNvdsConfig.ulRegTimeOut ) ) 
				 || ( ulConfigReadLong( &xNvdsConfig.ulRegTimeOut ) == 0 ) 
			   )
		  );

	if ( xInitResult != AT_SUCCESS )
	{
		/* The network attach was unsuccessful. Shut down the cellular module.
		   Return the status so that the caller can defer the next attach attempt if the device status permits it. */
		uStats.xStats.usGsmAttachFail++;	
		V_TRACE_PRINT_SHORT( TRACE_GSM_UNSUCESSFUL_GPRS_ATTACH, uStats.xStats.usGsmAttachFail, TRACE_UART_AND_FILE );

		/* Read the reason for the last unsuccessful connection attempt for debug purposes. */
		V_TRACE_PRINT_STRG( TRACE_GSM_UNSUCESSFUL_GPRS, cServerCmdParams, TRACE_UART_AND_FILE );

		if ( bCellularModuleCrashed )
		{
			prvPrepareCellularModuleReboot( HARD_POWER_OFF );
		}
		else
		{
			prvPrepareCellularModuleReboot( SOFT_POWER_OFF );
		}

		return CONNECT_NO_NW;	
	}

	/* Registration was successful: Continue process. */
	/* Send GSM initialisation commands - part 2. */
	xInitResult = xSendBatchAtCommand( xAtGSMCmd_P3, sizeof( xAtGSMCmd_P3 ) / sizeof( struct xAT_CMD ), true );
	
	/* The last command was a query for the status of the PDP activation. Check the status and activate the PDP,
	   if necessary. */
	if ( xInitResult == AT_SUCCESS )
	{
		/* Activating the PDP context using 3GPP commands does not work if the module is on 2G. Use QIACT instead. 
		   3GPP command:		xInitResult = xSendAtCommand( &xAtPdpAct, true, false ); */
		if ( cServerCmdParams[ 0 ] == '0' )
		{
			/* A returned status of '0' means that the PDP is not yet activated. Do it manually now. */
			xInitResult = xSendAtCommand( &xAtPdpQIAct, true, false );
		}
	}

	if ( xInitResult != AT_SUCCESS )
	{
		/* Increment the number of failed attempts to establish a GPRS connection. */
		uStats.xStats.usGprsConnectFail++;
		
		/* Read the reason for the last unsuccessful connection attempt for debug purposes. */
		V_TRACE_PRINT_STRG( TRACE_GSM_UNSUCESSFUL_GPRS, cServerCmdParams, TRACE_UART_AND_FILE );

		prvPrepareCellularModuleReboot( SOFT_POWER_OFF );

		return CONNECT_FAILED;	
	}				

	/* Everything went just fine. */
	xModuleState = GSM_PWR_ON | GSM_PDP_ACT | GSM_ACTIVE;
	
	/* We just (re-)attached to the BTS, so set this variable. */
	bGsmReattach = true;
	
	return CONNECT_SUCCESS;
}
/*-----------------------------------------------------------*/

/* Test if the PDP context is still active, i.e. the packet data connection is still intact. 
   If not, try to re-establish the PDP context and thus the packet switched data connection.
*/
bool bTestAndReActivatePDP( void )
{
	enum xAT_RESULT		xAtCommandResult;
	
	portENTER_CRITICAL();
	bool bModulePDPDeact = ( ( xModuleState & GSM_PDP_ACT_MSK ) == GSM_PDP_DEACT );
	portEXIT_CRITICAL();	

	if ( !bModulePDPDeact )
	{
		/* Check explicitly if the PDP context is still active. */
		xAtCommandResult = xSendAtCommand( &xAtPdpActQ, true, true );
	}
	
	if (    bModulePDPDeact
         || ( xAtCommandResult != AT_SUCCESS )
		 || ( cServerCmdParams[ 0 ] != '1' )
	   )
	{
		/* Re-activate the PDP context. */
		if ( xSendAtCommand( &xAtPdpAct, true, true ) == AT_SUCCESS )
		{
			/* Success: Update the module status. */
			portENTER_CRITICAL();										
			xModuleState |= GSM_PDP_ACT;
			portEXIT_CRITICAL();	
			V_TRACE_PRINT( TRACE_GSM_PDP_REACTIVATED, TRACE_UART_AND_FILE );
			return true;
		}				
		else
		{
			xModuleState &= ( unsigned portBASE_TYPE )~( GSM_PDP_ACT_MSK );
			V_TRACE_PRINT( TRACE_GSM_PDP_REACTIVATED_FAILED, TRACE_UART_AND_FILE );
			return false;
		}
	}
	return true;
}
/*-----------------------------------------------------------*/

/* Test if the TCP connection is still established.
   If not, try to re-establish TCP.
*/
enum xAT_RESULT xTestAndReConnectTCP( void )
{
	enum xAT_RESULT		xAtCommandResult;
	signed char			*pcChar;
	signed char			cSocketState;
	
	xAtCommandResult = AT_SUCCESS;
	cSocketState = '0';
	
	/* First, check if the FW thinks that the TCP is still connected. */
	if ( bGetTcpConnected() )
	{
		/* Yes: Now check explicitly if the TCP socket is still in ESTABLISHED state. */
		if ( xModuleState & GSM_TCP_CONN )
		{
			xAtCommandResult = xSendAtCommand( &xAtQIState, true, true );
		}
		else
		{
			xAtCommandResult = xSendAtCommand( &xAtQSSLState, true, true );
		}

		/* If the TCP socket status query returns a simple 'ERROR', the module needs a reboot to function correctly. Why that is,
		   nobody knows... */
		if ( ( xAtCommandResult == AT_MODULE_ERR ) || ( xAtCommandResult == AT_RESPONSE_ERR ) )
		{
			V_TRACE_PRINT( TRACE_GSM_TCP_MODULE_ERR, TRACE_UART_AND_FILE );
			return xAtCommandResult;
		}

		/* If the TCP is disconnected, connect now.
		   The result of the TCP/TSL query has this format:
				+QISTATE: <connectID>,<service_type>,<IP_address>,<remote_port>,<local_port>,<socket_state>,<contextID>,<serverID>,<access_mode>,<AT_port>
				+QSSLSTATE: <clientID>,"SSLClient",<IP_address>,<remote_port>,<local_port>,<socket_state>,<pdpctxID>,<serverID>,<access_mode>,<AT_port>,<sslctxID>
		   with:
				socket_state	0 "Initial": connection has not been established
								1 "Opening": client is connecting or server is trying to listen
								2 "Connected": client/incoming connection has been established
								3 "Listening": server is listening
								4 "Closing": connection is closing
		   Example:
				+QSSLSTATE: 4,"SSLClient","34.250.63.210",9003,23335,2,1,4,0,"uart1",1		*/
		pcChar = pcFindNComma( cServerCmdParams, 5, MAX_LEN_PARAMS + 1 );
		if ( pcChar < cServerCmdParams + MAX_LEN_PARAMS + 1 )
		{
			/* There is a 5th field. */
			cSocketState = *pcChar;
		}
		else
		{
			/* No valid response to the state query: Simulate disconnected state. */
			cSocketState = '0';
		}
	}
	
	if (	!bGetTcpConnected()
		 || ( xAtCommandResult != AT_SUCCESS )
		 || ( cSocketState == '0' )		/* INACTIVE status (CLOSED status defined in RFC793 "TCP Protocol Specification") */
		 || ( cSocketState == '4' )		/* CLOSING status */
	   )
	{
		/* If the TCP/TLS was closed without module action, try to re-open it. */
		if ( bTcpUnsolicitedClose )
		{
			V_TRACE_PRINT( TRACE_GSM_TCP_REACTIVATION, TRACE_UART_AND_FILE );
			
			/* Close the current SSL socket to free its identity. */
			if ( ( xModuleState & GSM_TCP_CONN_MSK ) == GSM_TCP_CONN )
			{
				xSendAtCommand( &xAtQIClose, true, true );		
			}
			else
			{
				xSendAtCommand( &xAtQSSLClose, true, true );	
			}
			vTaskDelay( T_TCP_DISC );
		}
		
		/* Start the TCP connection to the server. */
		if ( bStartGsmTCPConn() )
		{
			return AT_SUCCESS;
		}
		else
		{
			return AT_RESPONSE_ERR;
		}
	}
	
	bTcpUnsolicitedClose = false;
	
	return AT_SUCCESS;
}
/*-----------------------------------------------------------*/

/* Establish TCP connection. 
   The function assumes that the GSM/GPS module is already powered up and configured.
   Returns true if several attempts to establish connection finally succeeded.
*/
bool bStartGsmTCPConn( void )
{
	enum xAT_RESULT			xInitResult;
	unsigned portBASE_TYPE	xTCPRetryCnt;
	
	/* Repeat the TCP connection sequence until the connection is established. If the module state meanwhile indicates that the module has
	   no power, it may be that it has crashed and the crash detection has reset the module status. 

	   The choice of TLS vs. clear connections and user-configured vs. default destination address is as follows:

		attempt												secured				destination
		uxGsmTCPConnectAttemptCnt	xTCPRetryCnt			yes		no			config	default
		 0		0000				0						X					X
		 1		0001				1						X					X
		 2		0010				2						X					X
		 3		0011				3						X					X
		 4		0100				4								X			X
		 5		0101				5								X			X
		 6		0110				6								X					X
		 7		0111				7								X					X

		secured conn.:  ( uxGsmTCPConnectAttemptCnt & 0b0100 ) == 0
		default dest.:  secured connection AND ( ( uxGsmTCPConnectAttemptCnt & 0b0010 ) == 1 )

		The destination configured vs. default is selected in prvAddTCP_CONN_CFG().
		This scheme repeats for all further attemps. */
	xModuleState &= ( unsigned portBASE_TYPE )~( GSM_TCP_CONN | GSM_TLS_CONN );
	for ( xTCPRetryCnt = 0; ( xTCPRetryCnt < 2 * TCP_RETRY_LIMIT ) && ( xModuleState & GSM_PWR_ON ); xTCPRetryCnt++ )
	{
		/* Send GSM initialisation commands - part 4. 
		   Every attempt where bit 1 of the server connection attempt counter is 1 (i.e. 2 to 3, 6 to 7, etc.),
		   connect using clear connections. In all other attempts, use security if enabled. */
		if ( ( usConfigReadShort( &xNvdsConfig.usSecEnable ) == false ) || ( ( uxGsmTCPConnectAttemptCnt & 0b00000100 ) != 0 ) )
		{
			/* No security required, send in clear. */
			xInitResult = xSendBatchAtCommand( xAtGSMCmd_P4, sizeof( xAtGSMCmd_P4 ) / sizeof( struct xAT_CMD ), true );

			if ( xInitResult == AT_SUCCESS )
			{
				portENTER_CRITICAL();										
				xModuleState |= GSM_TCP_CONN;		
				portEXIT_CRITICAL();							
			}
		}
		else
		{
			/* Check if an IP adress is already known. If not, perform a DNS query. */
			if ( cGsmTlsServerIP[ 0 ] == 0 )
			{
				xInitResult = xSendAtCommand( &xAtSecDnsGIP, true, true ); 
				if ( xInitResult == AT_SUCCESS )
				{
					/* Copy the IP address. The address is terminated by a double quote ('"'). */
					/* Remove the double quote. */
					cServerCmdParams[ strlen( cServerCmdParams ) - 1 ] = 0;
					strncpy( cGsmTlsServerIP, cServerCmdParams, LEN_IP_ADDR );
				}				
			}
			else
			{
				xInitResult = AT_SUCCESS;				
			}

			/* Establish an encrypted (TLS) TCP connection. */
			if ( xInitResult == AT_SUCCESS )
			{
				xInitResult = xSendBatchAtCommand( xAtGSMCmd_P4sec, sizeof( xAtGSMCmd_P4sec ) / sizeof( struct xAT_CMD ), true );
			}
			
			if ( xInitResult == AT_SUCCESS )
			{
				portENTER_CRITICAL();										
				xModuleState |= GSM_TLS_CONN;		
				portEXIT_CRITICAL();							
			}
			else
			{
				/* Connection error: Invalidate the IP address. */
				cGsmTlsServerIP[ 0 ] = 0;
			}
		}
		
		( void )xSendAtCommand( &xAtQIGetError, true, true );
		
		/* Check, if the total initialisation was successful. */
		if ( xInitResult == AT_SUCCESS )
		{
			/* Reset the number of failed subsequent attempts when connecting to server. */
			uxGsmTCPConnectAttemptCnt = 0;
			
			return true;
		}
		
		/* No: Try again. */
		/* Increment the number of failed subsequent attempts when connecting to server. */
		uxGsmTCPConnectAttemptCnt++;
		V_TRACE_PRINT_STRG( TRACE_GSM_TCP_SOCKET_ERROR, cServerCmdParams, TRACE_UART_AND_FILE );
		V_TRACE_PRINT_BYTE( TRACE_GSM_TCP_CONNECT_FAILED, uxGsmTCPConnectAttemptCnt, TRACE_UART_AND_FILE );
		
		/* Close TCP socket to free up memory in QTEL. */
		/* ( void )xSendAtCommand( &xAtTcpSocClose, true, false ); */ /* TCP disconnect takes a long time. Prefer to dirty quit. */
		
		/* On AT_RESPONSE_TO re-boot the module between attempts. On AT_MODULE_ERR or AT_RESPONSE_ERR just continue in the for-loop. */
		if ( xInitResult == AT_RESPONSE_TO )
		{
			prvPrepareCellularModuleReboot( HARD_POWER_OFF );

			return false;
		}
	}
	
	return false;
}
/*-----------------------------------------------------------*/

/* Disable the module's power save modes (deep-sleep and idle). */
enum xAT_RESULT xModuleDisableSleepMode( void )
{
	/* Set GSM_DTR low. */
	nrf_gpio_pin_clear( GSM_DTR );	
	
	vTaskDelay( T_GSM_EXIT_SLEEP );
	
	return AT_SUCCESS;	
}
/*-----------------------------------------------------------*/

/* Enable the module's power save modes (deep-sleep and idle). */
enum xAT_RESULT xModuleEnableSleepMode( void )
{
	/* Set GSM_DTR high. */
	nrf_gpio_pin_set( GSM_DTR );	
	
	return AT_SUCCESS;	
}
/*-----------------------------------------------------------*/

/* Return the module's power save state. Returns true if in sleep mode. */
bool xModuleGetSleepMode( void )
{
	/* Read the value driven to GSM_DTR. */
	return ( nrf_gpio_pin_out_read ( GSM_DTR ) );	
}
/*-----------------------------------------------------------*/

/* Request the module to go to ACTIVE mode by strobing the PWR_ON signal low. 
   In ACTIVE mode, the 26MHz oscillator is turned on and the module runs normally. 
   The AT interface is operational.
*/
void vModuleExitPSM( void )
{
	TickType_t				xStartTime;
	
	/* Pulse GSM_PWR_KEY low: Set GSM_PWR_KEY to output */
	nrf_gpio_cfg_output( GSM_PWR_KEY );						/* Set GSM_PWR_ON_N to output */
	nrf_gpio_pin_clear( GSM_PWR_KEY );						/* Low level on GSM_PWR_ON_N. */

	/* Wait for STATUS to become high. */
	xStartTime = xTaskGetTickCount();						
	do														
	{
		vTaskDelay( 10 );
	} while (    !( nrf_gpio_pin_read ( STATUS ) )
		      && ( xTaskGetTickCount() - xStartTime < TO_GSM_INT_PWR_ON ) );	
	
	/* Set GSM_PWR_KEY_N to input. */
	nrf_gpio_cfg_input( GSM_PWR_KEY, NRF_GPIO_PIN_NOPULL );	/* Set GSM_PWR_ON_N to output */
	
	/* Set TXD to '1' (inactive state) */					
	nrf_gpio_pin_set( GSM_TXD );							
	
	/* Wait for the module to boot. The end of the process is indicated by the +UPSMR being sent by the module. */
	( void )bWaitModuleBooted();							
}
/*-----------------------------------------------------------*/

/* Return true if the module is in power-save mode. */
bool bCheckModulePowerSaveMode( void )
{
	return ( nrf_gpio_pin_read( STATUS ) );
}
/*-----------------------------------------------------------*/

/* Update the module status on an unsolicited PDP context deactivation. */
void vUnsolicitedPdpDeactivate( unsigned short usStrgIdx )
{
	/* The parameter is not used. */
	( void )usStrgIdx;
	
	V_TRACE_PRINT( TRACE_GSM_UNSOLICITED_PDP_DEACTIVATION, TRACE_UART_AND_FILE );
	
	/* Update the module status: neither PDP context is active nor TCP connection established. */
	portENTER_CRITICAL();							
	xModuleState &= ( unsigned portBASE_TYPE )~( GSM_PDP_ACT_MSK | GSM_TCP_CONN_MSK | GSM_TLS_CONN_MSK );
	portEXIT_CRITICAL();
	V_TRACE_PRINT( TRACE_GSM_UNSOLICITED_PDP_DEACTIVATION, TRACE_UART_AND_FILE );
}
/*-----------------------------------------------------------*/

/* Update the module status on an unsolicited TCP socket close. */
void vUnsolicitedTcpSocketClose( unsigned short usStrgIdx )
{
	/* The parameter is not used. */
	( void )usStrgIdx;
	
	/* Update the module status:  TCP connection no lopnger established. */
	portENTER_CRITICAL();							
	xModuleState &= ( unsigned portBASE_TYPE )~( GSM_TCP_CONN_MSK | GSM_TLS_CONN_MSK );
	portEXIT_CRITICAL();							
	V_TRACE_PRINT( TRACE_GSM_UNSOLICITED_TCP_SOCKET_CLOSE, TRACE_UART_AND_FILE );
	bTcpUnsolicitedClose = true;
}
/*-----------------------------------------------------------*/

/* Return the TCP connection status of the cellular module. If returned true, the module is supposed to be on and connected. */
bool bGetTcpConnected( void )
{
	portENTER_CRITICAL();
	bool bModuleTCPConn =    ( ( xModuleState & GSM_TCP_CONN_MSK ) == GSM_TCP_CONN )
						  || ( ( xModuleState & GSM_TLS_CONN_MSK ) == GSM_TLS_CONN );
	portEXIT_CRITICAL();	

	return bModuleTCPConn;
}
/*-----------------------------------------------------------*/

/* Invalidate all data fields to be sent to the server. This function is called as soon as 
   a tracker packet has been sent to the server. Doing this allows the GSM task to see which fields have really
   been updated since the last packet. 
*/
void prvInvalidateData( void )
{
	unsigned portBASE_TYPE		uxIdx;
	
	/* Invalidate the GPS data. */
	vInvalidateGpsData();

	/* Modification of the GSM data is protected by a mutex which avoids that both Parser and GSM 
	   task try to modify the data at the same time. */
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );
	
	/* Invalidating a field is done by poking 0 (end-of-string) into the first position of the
	   string. */
	cGsm_TIM	[ 0 ] = 0;		/* field  3: time stamp */
	cGsm_BAT	[ 0 ] = 0;		/* field  4: battery level in Volt */

	xBleLocBcnGSM.uxBleStdLocBcnCnt 		= 0;	/* field 20: BLE list of observed beacons. */
	xBleLocBcnGSM.uxBleSwissPhoneLocBcnCnt	= 0;	/* field 20: BLE list of observed beacons. */
	
	cGsm_STEPS  [ 0 ] = 0;		/* field 21: Absolute step count */
	cGsm_MCC    [ 0 ] = 0;		/* field 22a: GSM debug data: MCC Mobile Country Code. */
	cGsm_MNC    [ 0 ] = 0;		/* field 22b: GSM debug data: MNC Mobile Network Code. */
	cGsm_TAC    [ 0 ] = 0;		/* field 22c: GSM debug data: TAC Tracking Area Code. */
	cGsm_CI     [ 0 ] = 0;		/* field 22d: GSM debug data: CI Cell Identification. */
	cGsm_RAT    [ 0 ] = 0;		/* field 22e: GSM debug data: RAT Radio Access Technology. */
	cGsm_EARFCN [ 0 ] = 0;		/* field 22f: GSM debug data: EARFCN Extended Absolute Radio Frequency Channel Number. */
	cGsm_TXINST [ 0 ] = 0;		/* field 22g: GSM debug data: TxInst Delay in seconds since transmit instant. */
	cGsm_CHRGI  [ 0 ] = 0;		/* field 22h: GSM debug data: ChrgCurr Charge current if changring is throttled due to temperature. */

								/* field 23: foreign distress tracker indication */
								/* field 24: foreign distress tracker reception time stamp */

	/* Flags indicating the received special purpose beacons.. */
	xSpecialBcnRcvd.bPrivate 	= false;
	xSpecialBcnRcvd.bDanger  	= false;
	xSpecialBcnRcvd.bNoAbn   	= false;
	xSpecialBcnRcvd.bImmobility	= false;
	
	xSemaphoreGive( xMutexGsmData );
}
/*-----------------------------------------------------------*/

/* Wait for a specific response from the GSM/GPS module. If the response is 
   not received within xGsmBlockTime ticks, the function returns false. 
   Parameters:
		xExpectedMsg0/1		IDs of expected AT messages. Either will let the function return.
		xErrorAtRespMsgId	ID of error AT message (additionally to CME or CMS error)
		xWaitTime			wait time for message in ticks 
   Returns an AT_RESULT.

   Any server messages (between AT_REMOTE_OK and SRVCMD_OTHER) or AT_TCP_RX_DATA_IND encountered 
   while waiting for the expected message to be received are stored in a 5-deep FIFO. Once the 
   expected message is received, these messages are stored back into xParserAtRespQueue.  
*/
enum xAT_RESULT xReceiveResponse( enum xAT_MSG_ID xExpectedMsg0, enum xAT_MSG_ID xExpectedMsg1, enum xAT_MSG_ID xErrorAtRespMsgId, TickType_t xWaitTime )
{
	enum xAT_MSG_ID 		xMsgId;
	enum xAT_MSG_ID 		xBackedUpMsgId[ 5 ];
	unsigned portBASE_TYPE	uxBackedUpMsgCnt;
	unsigned portBASE_TYPE	uxIdx;
	enum xAT_RESULT			xAtReturnCode;	
	TickType_t				xGsmBlockTime;
	TickType_t				xStartTime;
	TickType_t				xTimeElapsed;
	portBASE_TYPE			xQueueStatus;
	
	/* xStartTime is used to record when we first started waiting for a 
	   reponse. If we receive a response which bears no relevance to the 
	   ongoing AT request, we use this time to calculate the remaining wait
	   time so that the time-out is not refreshed. */		   
	xGsmBlockTime = xWaitTime;
	xStartTime = xTaskGetTickCount();
	xQueueStatus = pdPASS;
	uxBackedUpMsgCnt = 0;
	xAtReturnCode = AT_RESPONSE_TO;

	while ( ( xGsmBlockTime > ( TickType_t )0 ) && ( xQueueStatus != errQUEUE_EMPTY ) && ( xAtReturnCode == AT_RESPONSE_TO ) )		
	{
		xQueueStatus = xQueueReceive( xParserAtRespQueue, &xMsgId, xGsmBlockTime );

		if ( xQueueStatus != errQUEUE_EMPTY )			
		{
			/* Received a response from the module. Check, if it was expected, else just discard it. */
			if ( ( xMsgId == xExpectedMsg0 ) || ( xMsgId == xExpectedMsg1 ) )
			{
				/* Success: Response matches the expected value. */
				xAtReturnCode = AT_SUCCESS;
			}
			if ( xMsgId == AT_ERROR )
			{
				/* Failure: Response matches the ID for an error response. Restart the module. */
				xAtReturnCode = AT_MODULE_ERR;
			}
			if ( xMsgId == xErrorAtRespMsgId )
			{
				/* Failure: Response matches the ID for an error response. */
				xAtReturnCode = AT_RESPONSE_ERR;
			}
			
			/* All indications that the server has sent data as well as actual server commands are kept in the queue. */
			if (   (   ( ( xMsgId >= AT_REMOTE_OK ) && ( xMsgId <= SRVCMD_OTHER ) )
				    || ( xMsgId == AT_TCP_RX_DATA_IND )
			       )
			    && ( xAtReturnCode == AT_RESPONSE_TO )
 			   )
			{
				/* The message is a server command or a TCP Rx data indication while we are waiting for another, protocol related
				   command. Remember the server command to shove it back into the queue later. Make sure to only remember the 
				   message if it was not already matched before. */
				xBackedUpMsgId[ uxBackedUpMsgCnt++ ] = xMsgId;
			}
			
			/* In any other case, the received response has no relevance for the execution of the 
			   command. Update the timer and continue waiting. 
			   For safety reasons we check that the timer can never underflow, i.e. wrap-around as
			   it is an unsigned value. */
			xTimeElapsed = xTaskGetTickCount() - xStartTime;
			if ( xTimeElapsed < xWaitTime )
			{
				xGsmBlockTime = xWaitTime - xTimeElapsed;
			}
			else
			{
				xGsmBlockTime = ( TickType_t )0;
			}
		}
		else
		{
			/* errQUEUE_EMPTY:	time-out in waiting for the response from the module. */
		}
	}

	/* Check, if we got a server command or a TCP Rx data indication while waiting for the expected message. Shove the 
	   server command back into the queue. */
	for ( uxIdx = 0; uxIdx < uxBackedUpMsgCnt; uxIdx++ )
	{
		if ( xQueueSend( xParserAtRespQueue, &xBackedUpMsgId[ uxIdx ], 0 ) != pdPASS )
		{
			V_TRACE_PRINT( TRACE_PAR_XPARSERATRESPQUEUE_FULL, TRACE_UART_AND_FILE );
		}
	}
	
	/* Return the return code. If there was no valid response within xWaitTime, the code is AT_RESPONSE_TO. */
	return xAtReturnCode;
}
/*-----------------------------------------------------------*/

/* Get battery voltage measurement. */
void vGetBatteryVoltage( void )
{
	unsigned long	ulLocalBattVoltage;
	unsigned char	ucDigit;

	/* Read the battery voltage from the ADC (ADC1, Vref = 1.1V band gap) */
	ulLocalBattVoltage = ( unsigned long )sGetBattVoltage();
	
	/* Convert it to a fixed-point integer with 1mV precision.
	   The resolution of sGetBattVoltage() is 1000 * ADC_VREF * ADC_GAIN_VBAT / ADC_MAX_VALUE mV = 1.465mV per step. */
	ulLocalBattVoltage *= 1465;						/* value in uV */
	
	/* Store the result as ASCII. */
	ucDigit = ( unsigned char )( ulLocalBattVoltage / 1000000l );
	cGsm_BAT[ 0 ] = cNibbleToChar( ucDigit );
	ulLocalBattVoltage -= ( unsigned long )ucDigit * 1000000l;
	
	cGsm_BAT[ 1 ] = '.';
	
	ucDigit = ( unsigned char )( ulLocalBattVoltage / 100000l );
	cGsm_BAT[ 2 ] = cNibbleToChar( ucDigit );
	ulLocalBattVoltage -= ( unsigned long )ucDigit * 100000l;
	
	ucDigit = ( unsigned char )( ulLocalBattVoltage / 10000l );
	cGsm_BAT[ 3 ] = cNibbleToChar( ucDigit );
}
/*-----------------------------------------------------------*/

/* Count the number of received satellites. */
unsigned portBASE_TYPE uxNumSvnEntries( void )
{
	unsigned portBASE_TYPE	uxNumSvnEntriesCnt;
	unsigned portBASE_TYPE	uxNumSvnEntriesIdx;
	
	uxNumSvnEntriesCnt = 0;
	for ( uxNumSvnEntriesIdx = 0; uxNumSvnEntriesIdx < GPSDEB_SVN; uxNumSvnEntriesIdx++ )
	{
		if ( xGpsSvnData[ uxNumSvnEntriesCnt ].uxSvn != 0 )
		{
			uxNumSvnEntriesCnt++;
		}
	}
	
	return uxNumSvnEntriesCnt;
}
/*-----------------------------------------------------------*/

/* Read the battery temperature, convert the result to ASCII and store it in the ASCII field pointed to by pcStrg. */
void vGetBatteryTemperature( signed char *pcStrg )
{
	short 					sTemperature;
	unsigned char			ucDigit;
	
	/* Get the binary temperature value from the charger module. */
	sTemperature = sGetBatteryTemperature();
	
	/* Convert it to ASCII and store it in the destination. 
	   Format: tt.t or -tt.t */
	/* Sign */
	if ( sTemperature < 0 )
	{
		sTemperature = -sTemperature;
		*( pcStrg++ ) = '-';
	}
	
	if ( sTemperature >= 1000 )
	{
		ucDigit = ( unsigned char )( sTemperature / 1000 );
		*( pcStrg++ ) = cNibbleToChar( ucDigit );
		sTemperature -= ( short )ucDigit * 1000;
	}
	
	ucDigit = ( unsigned char )( sTemperature / 100 );
	*( pcStrg++ ) = cNibbleToChar( ucDigit );
	sTemperature -= ( short )ucDigit * 100;
	
	ucDigit = ( unsigned char )( sTemperature / 10 );
	*( pcStrg++ ) = cNibbleToChar( ucDigit );
	sTemperature -= ( short )ucDigit * 10;
	
	*( pcStrg++ ) = '.';
	ucDigit = ( unsigned char )( sTemperature );	
	*( pcStrg++ ) = cNibbleToChar( ucDigit );
	
	*pcStrg = 0;
}
/*-----------------------------------------------------------*/

/* Format and send the location beacon BCN field. */
void vSendBCNField( volatile struct xBLE_LOC_BEACON *pxBleLocBcn, enum xBLE_BCN_FORMAT xBleBcnFormat )
{
	unsigned portBASE_TYPE						uxTblIdx;
	unsigned portBASE_TYPE						uxIdx;
	signed char									cHexStrg[ 5 ];
	volatile struct xBLE_LOC_BEACON_DATA_ENTRY	*pxBleLocBeaconDataEntry;
	unsigned portBASE_TYPE						uxBcnCnt;

	uxBcnCnt = 0;
	for ( uxTblIdx = 0; uxTblIdx < ( pxBleLocBcn->uxBleStdLocBcnCnt + pxBleLocBcn->uxBleSwissPhoneLocBcnCnt ); uxTblIdx++ )
	{
		/* Now filter the entries which match the format specified in xBleBcnFormat. */
		if ( pxBleLocBcn->xBleLocBcn[ uxTblIdx ].bIsSwissPhoneBeacon == ( xBleBcnFormat == BLE_SWISSPHONE_BEACON ) )
		{
			uxBcnCnt++;

			/* Construct the auxiliary pointer. */
			pxBleLocBeaconDataEntry = &pxBleLocBcn->xBleLocBcn[ uxTblIdx ].uBleLocBeaconData.xBleLocBeaconData ;
			
			if (xBleBcnFormat == BLE_SWISSPHONE_BEACON )
			{
				/* UUID filtered (SwissPhone) beacon: Send UUID index, Major and Minor fields.
				These fiels are stored for simplicity inside the beacon address field. */
				/* BCN_UUID_IDX */
				vByteToHexStrg( cHexStrg, pxBleLocBeaconDataEntry->ucBeaconAddr[ 0 ] );
				xComSendString( COM_GSM, cHexStrg );
				xComSendChar( COM_GSM, '/' );

				/* BCN_MAJOR 
				   Note that the value is stored in big Endian. */
				vShortToHexStrg( cHexStrg, 256 * pxBleLocBeaconDataEntry->ucBeaconAddr[ 1 ]
										       + pxBleLocBeaconDataEntry->ucBeaconAddr[ 2 ] );
				xComSendString( COM_GSM, cHexStrg );
				xComSendChar( COM_GSM, '/' );

				/* BCN_MINOR */
				vShortToHexStrg( cHexStrg, 256 * pxBleLocBeaconDataEntry->ucBeaconAddr[ 3 ]
										       + pxBleLocBeaconDataEntry->ucBeaconAddr[ 4 ] );
				xComSendString( COM_GSM, cHexStrg );
			}
			else
			{
				/* Standard beacon: send the BCN_ID */
				for ( uxIdx = 0; uxIdx < 6; uxIdx++ )
				{
					vByteToHexStrg( cHexStrg, pxBleLocBeaconDataEntry->ucBeaconAddr[ uxIdx ] );
					xComSendString( COM_GSM, cHexStrg );
				}
			}
			xComSendChar( COM_GSM, '/' );
			
			/* BCN_TIM */
			vShortToHexStrg( cHexStrg, pxBleLocBeaconDataEntry->usTimeStamp );
			xComSendString( COM_GSM, cHexStrg );
			xComSendChar( COM_GSM, '/' );
			
			/* BCN_RSSI */
			vByteToHexStrg( cHexStrg, pxBleLocBeaconDataEntry->cRSSI );
			xComSendString( COM_GSM, cHexStrg );
			xComSendChar( COM_GSM, '/' );
			
			/* BCN_REF_RSSI */
			vByteToHexStrg( cHexStrg, pxBleLocBeaconDataEntry->cRefRSSI );
			xComSendString( COM_GSM, cHexStrg );
			xComSendChar( COM_GSM, '/' );
			
			/* BCN_BAT */
			vByteToHexStrg( cHexStrg, pxBleLocBeaconDataEntry->ucBatteryLevel );
			xComSendString( COM_GSM, cHexStrg );
			
			if (xBleBcnFormat == BLE_SWISSPHONE_BEACON )
			{
				if ( uxBcnCnt != pxBleLocBcn->uxBleSwissPhoneLocBcnCnt )
				{
					xComSendChar( COM_GSM, ';' );				
				}				
			}
			else
			{
				if ( uxBcnCnt != pxBleLocBcn->uxBleStdLocBcnCnt )
				{
					xComSendChar( COM_GSM, ';' );				
				}				
			}
		}
	}
}
/*-----------------------------------------------------------*/

/* Format and send the GPS satellite reception quality field. */
void vSendGpsSvnField( void )
{
	unsigned portBASE_TYPE	uxIdx;
	signed char				cHexStrg[ 5 ];
	
	/* Add GNSS receiver HW data. */
	vShortToHexStrg( cHexStrg, usGpsNoisePerMS );
	xComSendString( COM_GSM, cHexStrg );
	xComSendChar( COM_GSM, ';' );
	
	vShortToHexStrg( cHexStrg, usGpsAgcCnt );
	xComSendString( COM_GSM, cHexStrg );
	xComSendChar( COM_GSM, ';' );
	
	vByteToHexStrg( cHexStrg, ucGpsHwFlags );
	xComSendString( COM_GSM, cHexStrg );
	xComSendChar( COM_GSM, ';' );
	
	vByteToHexStrg( cHexStrg, ucGpsJamInd );
	xComSendString( COM_GSM, cHexStrg );
	xComSendChar( COM_GSM, ';' );
	
	vByteToHexStrg( cHexStrg, ucGpsOfsI );
	xComSendString( COM_GSM, cHexStrg );
	xComSendChar( COM_GSM, ';' );
	
	vByteToHexStrg( cHexStrg, ucGpsMagI );
	xComSendString( COM_GSM, cHexStrg );
	xComSendChar( COM_GSM, ';' );
	
	vByteToHexStrg( cHexStrg, ucGpsOfsQ );
	xComSendString( COM_GSM, cHexStrg );
	xComSendChar( COM_GSM, ';' );
	
	vByteToHexStrg( cHexStrg, ucGpsMagQ );
	xComSendString( COM_GSM, cHexStrg );
		
	/* Walk through all entries in the SVN data list. */
	for ( uxIdx = 0; uxIdx < GPSDEB_SVN; uxIdx++ )
	{
		if ( xGpsSvnData[ uxIdx ].uxSvn != 0 )
		{
			xComSendChar( COM_GSM, ';' );

			/* Send satellite number. */
			vByteToHexStrg( cHexStrg, xGpsSvnData[ uxIdx ].uxSvn );
			xComSendString( COM_GSM, cHexStrg );
			xComSendChar( COM_GSM, '/' );
			
			/* Send satellite reception CN0. */
			vByteToHexStrg( cHexStrg, xGpsSvnData[ uxIdx ].uxCn0 );
			xComSendString( COM_GSM, cHexStrg );
		}
	}
}
/*-----------------------------------------------------------*/

/* Calculate length of one entry in the location beacon FTL field. 
   The field is composed of one or several TL beacons, each of which is formatted like this:
		'('					 	 1 character
		FID						15 characters
		','						 1 character
		FTIM					 4 characters
		','						 1 character
		FDESC					 2 characters
		','						 1 character
		FRSSI					 2 characters
		','						 1 character
		FLR						 1 characters
		','						 1 character
		FDFMAP					 8 characters
		','						 1 character
		variable part of the data specified by FDFMAP
		')'			 		 	 1 character
		
   Total:						--
								40 characters overhead
								+ variable part
								
   The result does not include the ',' which separates the field from the next one.								
   
   Note:
		Locator beacons are reported only for TL230 or newer.
*/
unsigned short usCalculateFTLLength( unsigned portBASE_TYPE uxTblIdx )
{
	unsigned short			usTlBcnFieldLen;
	unsigned portBASE_TYPE	uxNumFields;
	

	usTlBcnFieldLen = 0;
	
	/* Add the overhead for one TL beacon report. */
	usTlBcnFieldLen = 40;
	
	/* Walk through the FDFMAP and add the lengths od data fields present there up the command relay fields.
	   Each data byte is represented by 2 ASCII characters. */
	usTlBcnFieldLen += 2 * uxCalculateVarBcnFieldLen( xBleTLBcn[ uxTblIdx ].ulDFMap, BLE_BCN_RLYADDR_IDX, &uxNumFields );
	
	/* Take the separators into account. Each data fields adds exactly one separator, except for the last. */
	usTlBcnFieldLen += uxNumFields - 1;
	
	return usTlBcnFieldLen;
}
/*-----------------------------------------------------------*/

/* Format and send one entry in TL beacon FTL field. */
void vSendFTLField( unsigned portBASE_TYPE	uxTblIdx )
{
	unsigned portBASE_TYPE	uxDFMapIdx;
	unsigned portBASE_TYPE	uxVarBcnDataIdx;
	unsigned portBASE_TYPE	uxIdx;
	unsigned portBASE_TYPE	uxTlFieldLen;
	unsigned portBASE_TYPE	uxBcnNum;
	signed char				cHexStrg[ 9 ];
	bool					bFirstField ;
	
	xComSendChar( COM_GSM, '(' );				
	
	/* FID */
	xComSendChar( COM_GSM, cNibbleToChar( xBleTLBcn[ uxTblIdx ].ucFID[ 0 ] & 0x0f ) );
	for ( uxIdx = 1; uxIdx < 8; uxIdx++ )
	{
		vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucFID[ uxIdx ] );
		xComSendString( COM_GSM, cHexStrg );
	}
	xComSendChar( COM_GSM, ',' );
	
	/* FRXTIM */
	vShortToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].usRxTimeStamp );
	xComSendString( COM_GSM, cHexStrg );
	xComSendChar( COM_GSM, ',' );
	
	/* FDESC */
	vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucFDesc );
	xComSendString( COM_GSM, cHexStrg );
	xComSendChar( COM_GSM, ',' );
	
	/* FRSSI */
	vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].cRSSI );
	xComSendString( COM_GSM, cHexStrg );
	xComSendChar( COM_GSM, ',' );
	
	/* FLR */
	xComSendChar( COM_GSM, cNibbleToChar( xBleTLBcn[ uxTblIdx ].ucLongRange & 0xf ) );
	xComSendChar( COM_GSM, ',' );
	
	/* FDFMAP */
	vLongToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ulDFMap );
	xComSendString( COM_GSM, cHexStrg );
	xComSendChar( COM_GSM, ',' );

	/* Data fields specified by FDFMAP. */
	/* Walk through the FDFMAP and add the data fields present there. Include a separator for each data item, except for the last. */
	uxVarBcnDataIdx = 0;
	bFirstField = true;
	
	for ( uxDFMapIdx = 0; uxDFMapIdx < 31; uxDFMapIdx++ )				/* Bit 32 is reserved. */
	{
		if (    ( uxDFMapIdx != BLE_BCN_BCN_ONE_IDX ) 
		     && ( uxDFMapIdx != BLE_BCN_BCN_TWO_IDX ) 
		     && ( uxDFMapIdx != BLE_BCN_BCN_THREE_IDX ) 
		     && ( uxDFMapIdx != BLE_BCN_BCN_FOUR_IDX ) 
		     && ( uxDFMapIdx != BLE_BCN_UUID_BCN_ONE_IDX ) 
		     && ( uxDFMapIdx != BLE_BCN_UUID_BCN_TWO_IDX ) 
		     && ( uxDFMapIdx != BLE_BCN_UUID_BCN_THREE_IDX ) 
		     && ( uxDFMapIdx != BLE_BCN_UUID_BCN_FOUR_IDX ) )
		{
			/* Ordinary data field, not a BCN. */
			if ( ( xBleTLBcn[ uxTblIdx ].ulDFMap >> uxDFMapIdx ) & 1l )
			{
				if ( !bFirstField )
				{
					/* Send the separator. */
					xComSendChar( COM_GSM, ',' );				
				}
				
				/* Add the length of the data expressed as HEX-ASCII. */
				uxTlFieldLen = uxTLBcnDataFieldsLen[ uxDFMapIdx ];
				
				/* Check for any little-endian code data fields. */
				if ( uxDFMapIdx == BLE_BCN_TXTIM_IDX )
				{
					/* Reverse the order of little-endian coded fields. All those fields are 16-bit. */
					vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx + 1 ] );
					xComSendString( COM_GSM, cHexStrg );
					vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx ] );
					xComSendString( COM_GSM, cHexStrg );
					uxVarBcnDataIdx += 2;
				}
				else
				{
					/* The bytes order in all other fields is the same as in the BLE packet. */
					for ( uxIdx = 0; uxIdx < uxTlFieldLen; uxIdx++ )
					{
						vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
						xComSendString( COM_GSM, cHexStrg );
					}
				}
				
				bFirstField = false;
			}

			continue;
		}

		/* Treat standard beacons. */
		if ( uxDFMapIdx == BLE_BCN_BCN_ONE_IDX )
		{
			/* BCN data. Send the data once for all four beacon length bits. */
			uxBcnNum =   ( ( xBleTLBcn[ uxTblIdx ].ulDFMap & BLE_BCN_BCN_ALL_MSK ) >> BLE_BCN_BCN_ONE_IDX )
						+ ( ( ( xBleTLBcn[ uxTblIdx ].ulDFMap & BLE_BCN_BCN2_MSK ) >> BLE_BCN_BCN_FOUR_IDX ) << 3 );
			
			if ( ( uxBcnNum != 0 ) && !bFirstField )
			{
				/* Send the datafield separator. */
				xComSendChar( COM_GSM, ',' );
				bFirstField = false;
			}
			
			/* Loop over each standard beacon. */
			for ( uxIdx = 0; uxIdx < uxBcnNum; uxIdx++ )
			{
				/* Send the BCN separator. */
				if ( uxIdx != 0 )
				{
					xComSendChar( COM_GSM, ';' );				
				}
				
				/* FBCN_ID, 6 bytes */
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
				xComSendChar( COM_GSM, '/' );				
				
				/* FBCN_TIM, 2 bytes. */
				vShortToHexStrg( cHexStrg,    ( unsigned short )xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx ] 
											+ ( ( unsigned short )xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx + 1 ] << 8 ) ) ;
				uxVarBcnDataIdx += 2;
				xComSendString( COM_GSM, cHexStrg );
				xComSendChar( COM_GSM, '/' );				

				/* FBCN_RSSI, 1 byte. */
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
				xComSendChar( COM_GSM, '/' );				

				/* FBCN_REF_RSSI, 1 byte. */
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
				xComSendChar( COM_GSM, '/' );				

				/* FBCN_BAT, 1 byte. */
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
			}				
		}

		/* Treat UUID-filtered beacons. */
		if ( uxDFMapIdx == BLE_BCN_UUID_BCN_ONE_IDX )
		{
			/* BCN data. Send the data once for all four beacon length bits. */
			uxBcnNum = ( ( xBleTLBcn[ uxTblIdx ].ulDFMap & BLE_BCN_UUID_BCN_ALL_MSK ) >> BLE_BCN_UUID_BCN_ONE_IDX );
			
			if ( ( uxBcnNum != 0 ) && !bFirstField )
			{
				/* Send the datafield separator. */
				xComSendChar( COM_GSM, ',' );
				bFirstField = false;
			}
			
			/* Loop over each standard beacon. */
			for ( uxIdx = 0; uxIdx < uxBcnNum; uxIdx++ )
			{
				/* Send the BCN separator. */
				if ( uxIdx != 0 )
				{
					xComSendChar( COM_GSM, ';' );				
				}
				
				/* FBCN_UUID_IDX, 1 bytes */
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
				xComSendChar( COM_GSM, '/' );		

				/* FBCN_MAJOR, 2 bytes */
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
				xComSendChar( COM_GSM, '/' );		

				/* FBCN_MINOR, 2 bytes */
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
				xComSendChar( COM_GSM, '/' );				
				
				/* Skip over the next byte as it bears no significance for UUID-filtered beacons. */
				uxVarBcnDataIdx++;
				
				/* FBCN_TIM, 2 bytes. */
				vShortToHexStrg( cHexStrg,    ( unsigned short )xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx ] 
											+ ( ( unsigned short )xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx + 1 ] << 8 ) ) ;
				uxVarBcnDataIdx += 2;
				xComSendString( COM_GSM, cHexStrg );
				xComSendChar( COM_GSM, '/' );				

				/* FBCN_RSSI, 1 byte. */
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
				xComSendChar( COM_GSM, '/' );				

				/* FBCN_REF_RSSI, 1 byte. */
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
				xComSendChar( COM_GSM, '/' );				

				/* FBCN_BAT, 1 byte. */
				vByteToHexStrg( cHexStrg, xBleTLBcn[ uxTblIdx ].ucVarBcnData[ uxVarBcnDataIdx++ ] );
				xComSendString( COM_GSM, cHexStrg );
			}				
		}
	}
			
	xComSendChar( COM_GSM, ')' );				
}
/*-----------------------------------------------------------*/

/* Send one position store entry. */
void vSendPOSLField( unsigned portBASE_TYPE	uxPosStoreIdx )
{
	unsigned long				ulPDFMap;				/* Recorded position DF map including only data fields really being sent to the server. */
	unsigned portBASE_TYPE		uxDFIdx;
	signed char					cPDFMAP[ LEN_DFMAP ];
	unsigned portBASE_TYPE		uxDigitPos;
	unsigned portBASE_TYPE  	uxIdx;

	xComSendChar( COM_GSM, '(' );				
	
	/* Construct the PDFMAP. Note that PTIM and PSTATE are always present. */
	ulPDFMap = ( 1ul << PTIM_POS_PDFMAP ) | ( 1ul << PSTATE_POS_PDFMAP );
	
	/* Evaluate the contents of the GPS record. */
	if ( xGpsPosStore[ uxPosStoreIdx ].bPosValidEntry )
	{
		ulPDFMap |= ( xGpsPosStore[ uxPosStoreIdx ].cPosLAT[  0 ] != 0 ) ? ( 1ul << PLAT_POS_PDFMAP  ) : 0;
		ulPDFMap |= ( xGpsPosStore[ uxPosStoreIdx ].cPosLON[  0 ] != 0 ) ? ( 1ul << PLON_POS_PDFMAP  ) : 0;
		ulPDFMap |= ( xGpsPosStore[ uxPosStoreIdx ].cPosALT[  0 ] != 0 ) ? ( 1ul << PALT_POS_PDFMAP  ) : 0;
		ulPDFMap |= ( xGpsPosStore[ uxPosStoreIdx ].cPosHDG[  0 ] != 0 ) ? ( 1ul << PHDG_POS_PDFMAP  ) : 0;
		ulPDFMap |= ( xGpsPosStore[ uxPosStoreIdx ].cPosFIX[  0 ] != 0 ) ? ( 1ul << PFIX_POS_PDFMAP  ) : 0;
		ulPDFMap |= ( xGpsPosStore[ uxPosStoreIdx ].cPosHDOP[ 0 ] != 0 ) ? ( 1ul << PHDOP_POS_PDFMAP ) : 0;
		ulPDFMap |= ( xGpsPosStore[ uxPosStoreIdx ].cPosLOCM[ 0 ] != 0 ) ? ( 1ul << PLOCM_POS_PDFMAP ) : 0;
		ulPDFMap |= ( xGpsPosStore[ uxPosStoreIdx ].cPosSV[   0 ] != 0 ) ? ( 1ul << PSV_POS_PDFMAP   ) : 0;
		ulPDFMap |= ( xGpsPosStore[ uxPosStoreIdx ].cPosACCH[ 0 ] != 0 ) ? ( 1ul << PACCH_POS_PDFMAP ) : 0;
		ulPDFMap |= ( xGpsPosStore[ uxPosStoreIdx ].cPosACCV[ 0 ] != 0 ) ? ( 1ul << PACCV_POS_PDFMAP ) : 0;
	}

	/* Evaluate the contents of the BLE Location Beacon Store. */
	ulPDFMap |= ( xBleLocBeaconStore[ uxPosStoreIdx ].uxBleStdLocBcnCnt & 0x0f ) << PBCN_POS_PDFMAP;
	ulPDFMap |= ( xBleLocBeaconStore[ uxPosStoreIdx ].uxBleSwissPhoneLocBcnCnt & 0x0f ) << PBCN_UUID_POS_PDFMAP;
	
	/* Set the PSTEP bit if there are any to send. */
	ulPDFMap |= ( xGsmPosStore[ uxPosStoreIdx ].cPosSTEPS[  0 ] != 0 ) ? ( 1ul << PSTEPS_POS_PDFMAP ) : 0;

	/* Fill in PDFMap into the corresponding data fields. The format is Hex-ASCII. 
	   Leading '0's are suppressed. */
	uxDigitPos = 0;
	for ( uxDFIdx = 0; uxDFIdx < 8; uxDFIdx++ )
	{
		unsigned char	ucDfmapDigit;
		
		ucDfmapDigit = cNibbleToChar( ( ulPDFMap >> ( 28 - 4 * uxDFIdx ) ) & 0xf );
		if ( ( uxDigitPos != 0 ) || ( ucDfmapDigit != '0' ) )
		{	
			*( cPDFMAP + uxDigitPos ) = ucDfmapDigit;
			uxDigitPos++;
		}
	}
	*( cPDFMAP + uxDigitPos ) = 0;
	
	/* Send the PDFMAP field. */
	xComSendString( COM_GSM, cPDFMAP );
	
	/* Send all fields in sequence. */
	/* Send PTIM. */
	xComSendChar( COM_GSM, ',' );
	xComSendString( COM_GSM, xGsmPosStore[ uxPosStoreIdx ].cPosTIM );
	
	/* Send PSTATE. */
	xComSendChar( COM_GSM, ',' );
	xComSendString( COM_GSM, xGsmPosStore[ uxPosStoreIdx ].cPosSTATE );
	
	/* GPS position fields. PLAT_POS_PDFMAP is the first GPS field in the sequence. */
	for ( uxDFIdx = PLAT_POS_PDFMAP - PLAT_POS_PDFMAP; 
		  uxDFIdx < PACCV_POS_PDFMAP - PLAT_POS_PDFMAP + 1; 
		  uxDFIdx++ )
	{
		if ( ( ulPDFMap >> PLAT_POS_PDFMAP ) & ( 1ul << uxDFIdx ) )
		{
			xComSendChar( COM_GSM, ',' );
			xComSendString( COM_GSM, ( signed char * )&xGpsPosStore[ uxPosStoreIdx ] + cPosFieldOffset[ uxDFIdx ] );
		}
	}

	/* BLE location beacon list. */
	if ( xBleLocBeaconStore[ uxPosStoreIdx ].uxBleStdLocBcnCnt  ) 
	{
		xComSendChar( COM_GSM, ',' );
		vSendBCNField( &xBleLocBeaconStore[ uxPosStoreIdx ], BLE_ALTBEACON );
	}

	/* Set the PSTEP. */
	if ( xGsmPosStore[ uxPosStoreIdx ].cPosSTEPS[  0 ] != 0 )
	{
		xComSendChar( COM_GSM, ',' );
		xComSendString( COM_GSM, xGsmPosStore[ uxPosStoreIdx ].cPosSTEPS );
	}

	/* BLE UUID-filtered (SwissPhone) location beacon list. */
	if ( xBleLocBeaconStore[ uxPosStoreIdx ].uxBleSwissPhoneLocBcnCnt ) 
	{
		xComSendChar( COM_GSM, ',' );
		vSendBCNField( &xBleLocBeaconStore[ uxPosStoreIdx ], BLE_SWISSPHONE_BEACON );
	}

	vTaskDelay ( 100 );
			
	xComSendChar( COM_GSM, ')' );				
}
/*-----------------------------------------------------------*/

/* Fill in a bit in an ASCII-character field. The destination is pointed to by pcField with an offset in uxIdx.
   The bit identified by uxBitMask is set when bCondition is true. 
   If the bit is the last in the field (bLastBit is true) and the field contains a value, uxIdx is incremented. 
*/
void vSetFieldBit( bool bCondition, unsigned portBASE_TYPE uxBitMask, signed char *pcField, unsigned portBASE_TYPE *uxIdx, bool bLastBit )
{
	if ( bCondition )
	{
		*( pcField + *uxIdx )  = cNibbleToChar( cCharToNibble( *( pcField + *uxIdx ) ) | uxBitMask );
	}
	
	if ( bLastBit )
	{
		if ( *pcField != '0' )
		{
			( *uxIdx )++;
			*( pcField + *uxIdx ) = '0';
		}
	}
}
/*-----------------------------------------------------------*/

/* Depending on clear TCP or TLS, prepare the data transmission to the server. 
   After successful termination of this function, the caller can send ASCII characters to the server. 
   
   The AT command to the module looks like this :
   
		TX:		'AT+QSSLSEND=4'
		RX:		'> '
*/
enum xAT_RESULT prvPrepareTCPWrite( void )
{
	TickType_t				xCharReadTime;	
	bool					bGotChar;
	signed char 			sChar;
	
	/* Give the TCP send (socket write) command. Note that the command does not expect any return here and the
	   timeout is set to 0. */
	if ( xModuleState & GSM_TCP_CONN )
	{
		/* Clear TCP socket. */
		xComSendString( COM_GSM, pcAt_QISend );
	}
	else
	{
		if ( xModuleState & GSM_TLS_CONN )
		{
			/* TLS socket. */
			xComSendString( COM_GSM, pcAt_QSSLSend );
		}
		else
		{
			/* No socket open. */
			return AT_RESPONSE_ERR;
		}
	}
	
	/* Add the TCP socket ID. */
	xComSendString( COM_GSM, cGsmTcpSocketId );
	xComSendString( COM_GSM, "\r\n" );
	
	/* Poll now the AT interface for the send prompt. */
	xCharReadTime = xTaskGetTickCount();
	do
	{
		bGotChar = bGetRxCharFromBuffer( COM_GSM, &sChar );
		vTaskDelay( 1 );
	}
	while (    !bGotChar 
		    && ( sChar != '>' ) 
			&& ( xTaskGetTickCount() < xCharReadTime + TO_GSM_FS ) );	
	
	if ( bGotChar && ( sChar == '>' ) )
	{
		return AT_SUCCESS;
	}
	
	NRF_LOG_WARNING( "%i Failed to prepare TCP write.", ulReadRTC() );
	NRF_LOG_PROCESS();
	
	return AT_RESPONSE_TO;
}
/*-----------------------------------------------------------*/

/* Send an position base message to the server. 
   This message does not contain any FTL fields. Those will be sent in a later message.
   
   Return false, if there was a serious error while accessing the GSM/GPS module. In this case, the 
   calling function will re-try and eventually reset the module.
*/
enum xAT_RESULT prvSendBasePacket( void )
{
	unsigned long				ulConfiguredDFMap;	/* DFMap as configured by the server. */
	unsigned long				ulActualDFMap;		/* Real DF map including only data fields really being sent to the server. */
	unsigned portBASE_TYPE		uxDFIdx;
	enum xAT_RESULT				xAtResult;
	enum xCTRL_STATE 			xCtrlState;
	unsigned portBASE_TYPE		uxDigitPos;
	TickType_t					xWaitForOkStart;
	TickType_t					xOnDurationSendTimeStamp;
	bool						bOkReceived;
	signed char					cHexStrg[ 5 ];
	unsigned portBASE_TYPE  	uxIdx;
	
	/* Fill in the device state. 
	   ATTENTION: Only works up to 16 device states! */
	xCtrlState = xGetCtrlState();
	cGsm_STATE[ 0 ] = cNibbleToChar( xCtrlState );
	cGsm_STATE[ 1 ] = 0;
	
	/* If there is no good fix, clobber the GPS data which have been received so far. 
	   The GPS module sends accuracy even if there was no good fix. */
	if ( !bHasGPSGoodFix() )
	{
		cGsm_LAT[ 0 ]  = 0;
		cGsm_LON[ 0 ]  = 0;
		cGsm_ALT[ 0 ]  = 0;
		cGsm_VEL[ 0 ]  = 0;
		cGsm_HDG[ 0 ]  = 0;
		cGsm_FIX[ 0 ]  = 0;
		cGsm_HDOP[ 0 ] = 0;
		cGsm_LOCM[ 0 ] = 0;
	}
	
	/* Test, if the packet is the first one to send in ALERT state. */
	if ( bPushPacketImmediately )
	{
		/* We are going to push the first packet in alert state without position.
		   Now immediately let it follow a second packet timer 1s, this time with position. */
		prvUpdateTransmissionTimer( 1 );
	
		/* Now that we are have captured the alert state, reset the 'push immediate' flag. */
		bPushPacketImmediately = false;
	}

	/* Lock access to the GSM data. Neither parser nor BLE parser are now allowed to write the data. */
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );
	
	/* Set the IND field. Note, that cGsm_IND[0] is the MSB! So fill in the fields left-to-right starting 
	   with the MSB. */
	uxIdx = 0;
	cGsm_IND[ 0 ] = '0';
	
	/* Set flags bits[19:16]. */
	/* Set the 'abnormal position detection off' bit. */
	vSetFieldBit( bTestMode, IND_AUTOTEST >> 16, cGsm_IND, &uxIdx, true );
	
	/* Set flags bits[15:12]. */
	/* Set the 'abnormal position detection off' bit. */
	vSetFieldBit( xSpecialBcnRcvd.bImmobility, IND_IMMOBILITY_BCN >> 12, cGsm_IND, &uxIdx, false );
	
	/* Set the 'abnormal position detection off' bit. */
	vSetFieldBit( bNoAbnActive, IND_ABNDET_OFF >> 12, cGsm_IND, &uxIdx, false );

	/* Set the 'vibration motor activated' bit. */
	vSetFieldBit( bVibrationMotorOn, IND_VIBR_ON >> 12, cGsm_IND, &uxIdx, false );

	/* Set the private zone beacon received bit. */
	vSetFieldBit( xSpecialBcnRcvd.bPrivate, IND_PRIVATE_BCN >> 12, cGsm_IND, &uxIdx, true );

	/* Set flags bits[11:8]. */
	/* Set the private zone beacon received bit. */
	vSetFieldBit( xSpecialBcnRcvd.bDanger, IND_DANGER_BCN >> 8, cGsm_IND, &uxIdx, false );

	/* Set the no abnormal position detection beacon received bit. */
	vSetFieldBit( xSpecialBcnRcvd.bNoAbn, IND_NOABN_BCN >> 8, cGsm_IND, &uxIdx, false );

	/* Set the charging bit. */
	vSetFieldBit( bGetOnCharger(), IND_CHARGING >> 8, cGsm_IND, &uxIdx, false );

	/* Set the extreme temperature bit (temperature outside charging conditions). */
	vSetFieldBit( bTemperatureOutsideChrg(), IND_EXTREME_TEMP >> 8, cGsm_IND, &uxIdx, true );

	/* Set flags bits[7:4]. */
	/* Set the foreign alert type, if any. */
	vSetFieldBit( uxForeignAlertType & BLE_ALERT, IND_FOREIGN_ALERT >> 4, cGsm_IND, &uxIdx, false );
	if ( uxForeignAlertType & BLE_ALERT )
	{
		V_TRACE_PRINT( TRACE_GSM_FOREIGN_ALERT, TRACE_UART );
	}
	vSetFieldBit( uxForeignAlertType & BLE_SOS, IND_FOREIGN_SOS >> 4, cGsm_IND, &uxIdx, false );
	if ( uxForeignAlertType & BLE_SOS )
	{
		V_TRACE_PRINT( TRACE_GSM_FOREIGN_SOS, TRACE_UART );
	}

	/* Set the NVDS_REINIT bit in the IND field if required. */
	vSetFieldBit( bNVDSReInitialised, IND_NVDS_REINIT >> 4, cGsm_IND, &uxIdx, false );
	
	/* Set the REATTACH bit in the IND field if required. */
	vSetFieldBit( bGsmReattach, IND_REATTACH >> 4, cGsm_IND, &uxIdx, true );

	/* Set flags bits[3:0]. */
	/* Set the BATT_FULL bit in the IND field if required. */
	vSetFieldBit( bIsChargingAndBatteryFull(), IND_BATT_FULL, cGsm_IND, &uxIdx, false );
	
	/* Set the SOS bit in the IND field if required. */
	vSetFieldBit( xCtrlState == CTRL_SOS, IND_SOS, cGsm_IND, &uxIdx, false );
	
	/* Set the ALERT bit in the IND field if required. */
	vSetFieldBit( xCtrlState == CTRL_ALERT, IND_ALERT, cGsm_IND, &uxIdx, true );
	
	/* Add the end-of-string 0x0 in the appropriate place. */
	cGsm_IND[ uxIdx ] = 0;

	
	/* Fill in the absolute step count unless it is 0 (where the value is not sent). */
	if ( usAbsStepCount != 0 )
	{
		vShortToHexStrg( cGsm_STEPS, usAbsStepCount );
	}	
	
	/* Read the configured DFMap from NVDS. */
	ulConfiguredDFMap = ulConfigReadLong( &xNvdsConfig.ulDFMap );
	/* Always send indications when available. This makes sure that alerts cannot be disabled from the server. */
	ulConfiguredDFMap |= ( 1l << DFMAP_IND );
	ulActualDFMap = 0l;
	
	/* If the device state does not require the transmission of position data, mask all relevant fields. */
	if ( !bCtrlGpsPositionRequired() )
	{
		ulConfiguredDFMap &= ( unsigned long )~GSM_GPS_POS_FIELDS_MASK;
	}
	
	/* If the packet is a HELLO packet in GPS recording, mask all fields which are not to be transmitted. */
	if ( bIsHelloPacketInGpsRecording() )
	{
		ulConfiguredDFMap &= GSM_HELLO_FIELDS_MASK;
	}
	
	/* Get the RSSI. */
	/* First, restore write access to the GSM data. */
	xSemaphoreGive( xMutexGsmData );
	( void )xSendBatchAtCommand( xAtStat, sizeof( xAtStat ) / sizeof( struct xAT_CMD ), true );
	( void )xSendAtCommand( &xAtQNwInfoQ, true, false );
	
	/* Then, lock access to the GSM data again. */
	configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );
	
	/* Get RTC time string. */
	vShortToHexStrg( cGsm_TIM, usReadRTC() );

	/* Get battery and temperature measurements. If recovering
	   of any of the data fails, return failure to the caller. */
	vGetBatteryVoltage();
	
	vGetBatteryTemperature( cGsm_TEMP );
	
	/* Add elapsed time since GSM TX instant to GSM debug field. */
	vShortToHexStrg( cGsm_TXINST, usReadRTC() - usTxInstantTS );
	
	/* The following section adds the battery charger current setting to the GSM debug field. */
	if ( xGetChargerStatus() == CHRG_FW_CONTROLLED )
	{
		unsigned short		usActBattChrgCurrent;
		
		/* Get the current setting of the battery charge current. */
		usActBattChrgCurrent = 100 - ucReadPwmDutyCycle();
		/* Write the current setting to the cGsm_CHRGI field. */
		vByteToHexStrg( cGsm_CHRGI, usActBattChrgCurrent );		
	}

	/* Step through the configured DFMap to see which data fields need to be sent. 
	   In the same loop calculate the total length of the all data fields including separators. */
	for ( uxDFIdx = 0; uxDFIdx < 32; uxDFIdx++ )
	{
		if ( ( ( ulConfiguredDFMap >> uxDFIdx ) & 1 ) == 1 )
		{
			/* The data field needs to be sent. 
			   Some data fields such as 0 (IMEI), 20 (list of BLE location beacons) and 27 (list of BLE distress and TL beacons) need to be treated
			   differently as the data source is not stored as ASCII in RAM for space reasons. */
			switch ( uxDFIdx )
			{
				case ID_POS_DFMAP:		if ( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID ) != 0 )
										{
											/* Mark the data field to be sent. */
											ulActualDFMap |= ( 1l << ID_POS_DFMAP );
										}
										break;
			
				case BCN_POS_DFMAP:		/* BLE standard location beacon list. */
										if ( xBleLocBcnGSM.uxBleStdLocBcnCnt > 0 )
										{
											/* Mark the data field to be sent. */
											ulActualDFMap |= ( 1l << BCN_POS_DFMAP );
										}
										break;

				case UUID_BCN_POS_DFMAP:/* BLE standard location beacon list. */
										if ( xBleLocBcnGSM.uxBleSwissPhoneLocBcnCnt > 0 )
										{
											/* Mark the data field to be sent. */
											ulActualDFMap |= ( 1l << UUID_BCN_POS_DFMAP );
										}
										break;

				case GSMDEB_POS_DFMAP:	/* GSM debug information. */
										ulActualDFMap |= ( 1l << GSMDEB_POS_DFMAP );
										break;

				case GPSDEB_POS_DFMAP:	/* GPS debug SVN data list. */
										if (    bCtrlGpsPositionRequired() 
											 && ( uxNumSvnEntries() > 0 ) 
										   )
 										{
											/* Mark the data field to be sent. */
											ulActualDFMap |= ( 1l << GPSDEB_POS_DFMAP );
										}
										break;
				
				case EVAC_ID_POS_DFMAP:	/* Evacuation ID. */
										if ( bGetEvacOngoing() )
 										{
											/* Mark the data field to be sent. */
											ulActualDFMap |= ( 1l << EVAC_ID_POS_DFMAP );
										}
										break;
										
										
				case RFON_POS_DFMAP:	/* RF-ON. */
										/* Mark the data field to be sent. */
										ulActualDFMap |= ( 1l << RFON_POS_DFMAP );
										break;
				
				default:				/* Do not treat any FTL fields here. Those will be sent later. */
										if ( uxDFIdx != FTL_POS_DFMAP )
										{
											/* Standard (ASCII) data field.
											   See, if the corresponding data field has actually been filled since the last position transmission. */
											if (   (  pcDataFields[ uxDFIdx ] != NULL )
												&& ( *pcDataFields[ uxDFIdx ] != 0 ) )
											{
												/* Mark the data field to be sent. */
												ulActualDFMap |= ( 1l << uxDFIdx );
											}
										}
										break;
			}
		}
	}
	
	/* Fill in DFMap into the corresponding data fields. The format is Hex-ASCII. 
	   Leading '0's are suppressed. */
	uxDigitPos = 0;
	for ( uxDFIdx = 0; uxDFIdx < 8; uxDFIdx++ )
	{
		unsigned char	ucDfmapDigit;
		
		ucDfmapDigit = cNibbleToChar( ( ulActualDFMap >> ( 28 - 4 * uxDFIdx ) ) & 0xf );
		if ( ( uxDigitPos != 0 ) || ( ucDfmapDigit != '0' ) )
		{	
			*( cGsm_DFMAP + uxDigitPos ) = ucDfmapDigit;
			uxDigitPos++;
		}
	}
	*( cGsm_DFMAP + uxDigitPos ) = 0;
	
	/* Give the TCP send (socket write) command. */
	if ( ( xAtResult = prvPrepareTCPWrite() ) != AT_SUCCESS )
	{
		/* Restore write access to the GSM data. */
		xSemaphoreGive( xMutexGsmData );
		
		return xAtResult;
	}
	
	/* Now send the data fields in the order of the bits in ulActualDFMap. */
	/* Send the header. For the time being, the FRMT field is fixed to 0. */
	xComSendString( COM_GSM, "[" TL500_FRMT "," );

	/* Send the DFMAP field. */
	xComSendString( COM_GSM, cGsm_DFMAP ); 
	
	/* Send the data fields one-by-one. Add a separator before each data field. */
	for ( uxDFIdx = 0; uxDFIdx < 32; uxDFIdx++ )
	{
		if ( ( ( ulActualDFMap >> uxDFIdx ) & 1 ) == 1 )
		{
			xComSendChar( COM_GSM, ',' );	
			
			/* Some data fields sunch as 20 (list of BLE location beacons) and 27 (list of BLE distress and TL beacons) 
			   need to be treated differently because either as the data source contains binary data for RAM space reasons or 
			   the data is collected and formatted on-the-fly instead of being stored as preformatted string in pcDataFields[]. */
			switch ( uxDFIdx )
			{
				case ID_POS_DFMAP:		/* GSM ID */
										/* The data field 0 (IMEI) is sent from NVDS. */
										{
											signed char		cTmpStrg[ LEN_GSM_ID + 1 ];
								
											/* Read the configuration from the EEPROM. */
											( void )pcConfigReadString( xNvdsConfig.pcGsm_ID, cTmpStrg, LEN_GSM_ID + 1, TRANSLATE_APOSTR );
	
											/* Send the parameter. */
											xComSendString( COM_GSM, cTmpStrg );
										}
										break;
										
				case BCN_POS_DFMAP:		/* BLE location beacon list. */
										vSendBCNField( &xBleLocBcnGSM, BLE_ALTBEACON );
										break;

				case UUID_BCN_POS_DFMAP:/* BLE location beacon list. */
										vSendBCNField( &xBleLocBcnGSM, BLE_SWISSPHONE_BEACON );
										break;

				case GSMDEB_POS_DFMAP:	/* GSM debug information:
												MCC				0...3 characters 	+ ';'
												MNC				0...2 characters 	+ ';'
												LAC/TAC			0...4 characters 	+ ';'
												CI				0...8 characters 	+ ';'
												CI				0...1 characters 	+ ';'
												EARFCN			0...8 characters 	+ ';'
												TXINST			0...4 characters 	+ ';'
												CHRGI			0...4 characters 	+ ','			*/
										xComSendString( COM_GSM, cGsm_MCC );	xComSendChar( COM_GSM, ';' );
										xComSendString( COM_GSM, cGsm_MNC );	xComSendChar( COM_GSM, ';' );
										xComSendString( COM_GSM, cGsm_TAC );	xComSendChar( COM_GSM, ';' );
										xComSendString( COM_GSM, cGsm_CI );		xComSendChar( COM_GSM, ';' );
										xComSendString( COM_GSM, cGsm_RAT );	xComSendChar( COM_GSM, ';' );
										xComSendString( COM_GSM, cGsm_EARFCN );	xComSendChar( COM_GSM, ';' );
										xComSendString( COM_GSM, cGsm_TXINST );	xComSendChar( COM_GSM, ';' );
										xComSendString( COM_GSM, cGsm_CHRGI );
										break;
										
				case GPSDEB_POS_DFMAP:	/* GPS debug SVN data list. */
										vSendGpsSvnField();
										break;

				case EVAC_ID_POS_DFMAP:	/* Evacuation ID. */
										vByteToHexStrg( cHexStrg, ( unsigned char )uxGetEvacId() );
										xComSendString( COM_GSM, cHexStrg );
										break;
										
				case RFON_POS_DFMAP:	/* RF-ON. */
										/* Cellular module on-duration. */
										vShortToIntStrg( cHexStrg, ( unsigned short )( ( ( xGsmOnDuration + xTaskGetTickCount() - xGsmOnTimeStamp ) / portTICKS_PER_SEC ) & 0xffff ), true );
										xComSendString( COM_GSM, cHexStrg );	
										xComSendChar( COM_GSM, ';' );

										/* GPS module on-duration. */
										vShortToIntStrg( cHexStrg, ( unsigned short )( ( xGetGpsOnDuration() / portTICKS_PER_SEC ) & 0xffff ), true );
										xComSendString( COM_GSM, cHexStrg );	
										xComSendChar( COM_GSM, ';' );

										/* Bluetooth module on-duration. */
										vShortToIntStrg( cHexStrg, ( unsigned short )( ( xGetBleOnDuration() / portTICKS_PER_SEC ) & 0xffff ), true );
										xComSendString( COM_GSM, cHexStrg );	
										
										/* Remember the time when the module on times were captured. */
										xOnDurationSendTimeStamp = xTaskGetTickCount();
										break;
										
				default:				xComSendString( COM_GSM, pcDataFields[ uxDFIdx ] );
										break;

			}
		}
	}
	
	/* Restore write access to the GSM data. */
	xSemaphoreGive( xMutexGsmData );
	
	/* Send the trailer followed by CTRL-Z to exit from data mode. */
	xComSendString( COM_GSM, "]\x1a\r\n" );	
	
	/* Wait for the SEND OK message from the GSM/GPS module. */
	xAtResult = xReceiveResponse( AT_SEND_OK, AT_NOMSG, AT_SEND_FAIL, TO_GSM_TCPWR );
	if ( xAtResult != AT_SUCCESS )
	{ 
		if ( xAtResult == AT_RESPONSE_TO )
		{
			V_TRACE_PRINT_STRG( TRACE_GSM_AT_RESPONSE_TO, ( signed char * )pcAt_QSSLSend, TRACE_UART_AND_FILE );
		}
		else
		{
			V_TRACE_PRINT( TRACE_GSM_SEND_MODULE_ERR, TRACE_UART_AND_FILE );
		}

		/* Received no positive response from the module. Return the error. */
		return xAtResult;
	}
	
	/* Wait for the acknowledgement from the server. In case get a server command first, we need to wait for
	   several data packets from the server. */
	/* Record  start time stamp. */
	xWaitForOkStart =  xTaskGetTickCount();
	bOkReceived = false;

	/* Loop while polling messages received from the server until we received the expected '[OK]'. */
	while ( !bOkReceived && ( xTaskGetTickCount() - xWaitForOkStart <  TO_GSM_SRV_ACK ) )
	{
		/* First, wait for a TCP RX data indication telling us that the server has sent data. */
		xAtResult = xReceiveResponse( AT_TCP_RX_DATA_IND, AT_NOMSG, AT_NOMSG, TO_GSM_SRV_ACK );
		if ( xAtResult != AT_SUCCESS )
		{
			if ( xAtResult == AT_RESPONSE_TO )
			{
				/* A response timeout could also mean that the GSM module had issued previously a data received
				   indication which the FW failed to treat (which should not really happen but does...). In this
				   case, the receive indication is not renewed when the GSM module receives any further data. So
				   just attempt to read receive data from the GSM module. If there aren't any, the read will fail. */
				V_TRACE_PRINT( TRACE_GSM_RX_OK_RESPONSE_TIMEOUT, TRACE_UART_AND_FILE );
			}
			else
			{
				if ( xAtResult == AT_RESPONSE_ERR )
				{
					V_TRACE_PRINT( TRACE_GSM_RX_OK_RESPONSE_ERR, TRACE_UART_AND_FILE );
				}
				else
				{
					V_TRACE_PRINT( TRACE_GSM_RX_OK_MODULE_ERR, TRACE_UART_AND_FILE );
				}

				/* Received no indication that the server has sent us any data. Return the error. */
				return xAtResult;
			}
		}

		/* Now recover the data and compare it against the expected '[OK]' message.
		   First, send command to read the received message. 
		   Note that (as explained before), there is the possibility that there is no receive data available. */
		if ( xModuleState & GSM_TCP_CONN )
		{
			/* Clear TCP socket. */
			( void )xSendAtCommand( &xAtQIRecv, true, true );
		}
		else
		{
			/* TLS socket. */
			( void )xSendAtCommand( &xAtQSSLRecv, true, true );
		}

		if ( xAtResult != AT_SUCCESS )
		{
			return xAtResult;
		}

		/* The actual server command has now been parsed and the command indication been stuffed back into the
		   xParserAtRespQueue queue. We can retrieve it now. 
		   All messages other than the requested type are kept intact in the queue. Thus, if we accidentally receive
		   a server command instead of an [OK], it is preseved in the queue and will be treated later. */
		if ( bQueueCheckIfDataItemInQueue( xParserAtRespQueue, AT_REMOTE_OK ) )
		{
			vRemoveTypeFromQueue( xParserAtRespQueue, AT_REMOTE_OK );
			bOkReceived = true;
			
			/* Reset the RF module ON duration counters. */
			xGsmOnDuration = xTaskGetTickCount() - xOnDurationSendTimeStamp;
			xGsmOnTimeStamp = xTaskGetTickCount() - xGsmOnDuration;
			vResetGpsOnDuration( xTaskGetTickCount() - xOnDurationSendTimeStamp );
			vResetBleOnDuration( xTaskGetTickCount() - xOnDurationSendTimeStamp );
		}
	}

	if ( !bOkReceived )
 	{
		V_TRACE_PRINT( TRACE_GSM_NO_SERVER_OK, TRACE_UART_AND_FILE );

		return AT_RESPONSE_TO;
 	}	
	
	/* If we get here, everything was okay. */
	return AT_SUCCESS;
}
/*-----------------------------------------------------------*/

/* Send one or more FTL message to the server if there are any FTL fields pending. 
   
   Return false, if there was a serious error while accessing the GSM/GPS module. In this case, the 
   calling function will re-try and eventually reset the module.
*/
enum xAT_RESULT prvSendFtlPackets( void )
{
	enum xAT_RESULT				xAtResult;
	unsigned portBASE_TYPE		uxTlTblIdx;
	TickType_t					xWaitForOkStart;
	bool						bOkReceived;
	signed char					cTmpStrg[ LEN_GSM_ID + 1 ];
	
	/* Walk through all received FTL beacons and send them one-by-one in a dedicated packet. */
	for ( uxTlTblIdx = 0; uxTlTblIdx < uxBleTLBcnCnt; uxTlTblIdx++ )
	{
		/* Should the GSM module be in sleep mode, bring it out of sleep. */
		if ( xModuleGetSleepMode() )
		{
			/* The module is in power-save mode. Bring it out of the sleep mode. */
			xModuleDisableSleepMode();
		}
		
		/* Give the TCP send (socket write) command. */
		if ( prvPrepareTCPWrite() != AT_SUCCESS )
		{
			return xAtResult;
		}

		/* Lock access to the GSM data. Neither parser nor BLE parser are now allowed to write the data. */
		configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );
		
		/* Now send the data fields in the order of the bits in ulActualDFMap. */
		/* Send the header. For the time being, the FRMT field is fixed to 0. */
		xComSendString( COM_GSM, "[" TL500_FRMT "," );


		/* Send the DFMAP field. */
		xComSendString( COM_GSM, "02000009" );
		xComSendChar( COM_GSM, ',' );	
		
		/* Send the ID. */
		( void )pcConfigReadString( xNvdsConfig.pcGsm_ID, cTmpStrg, LEN_GSM_ID + 1, TRANSLATE_APOSTR );
		xComSendString( COM_GSM, cTmpStrg );
		xComSendChar( COM_GSM, ',' );	
		
		/* Get RTC time string. */
		vShortToHexStrg( cGsm_TIM, usReadRTC() );
		xComSendString( COM_GSM, cGsm_TIM );
		xComSendChar( COM_GSM, ',' );	
		
		/* BLE distress/TL beacon list. */
		vSendFTLField( uxTlTblIdx );
				
		/* Restore write access to the GSM data. */
		xSemaphoreGive( xMutexGsmData );
		
		/* Send the trailer followed by CTRL-Z to exit from data mode. */
		xComSendString( COM_GSM, "]\x1A" );	
		
		/* Wait for the OK message from the GSM/GPS module. */
		xAtResult = xReceiveResponse( AT_SEND_OK, AT_NOMSG, AT_SEND_FAIL, TO_GSM_TCPWR );
		if ( xAtResult != AT_SUCCESS )
		{ 
			if ( xAtResult == AT_RESPONSE_TO )
			{
				V_TRACE_PRINT_STRG( TRACE_GSM_AT_RESPONSE_TO, ( signed char * )pcAt_QSSLSend, TRACE_UART_AND_FILE );
			}
			else
			{
				V_TRACE_PRINT( TRACE_GSM_SEND_MODULE_ERR, TRACE_UART_AND_FILE );
			}
			/* Received no positive response from the module. Return the error. */
			return xAtResult;
		}
		
		/* Wait for the acknowledgement from the server. In case get a server command first, we need to wait for
		   several data packets from the server. */
		/* Record  start time stamp. */
		xWaitForOkStart =  xTaskGetTickCount();
		bOkReceived = false;
	
		/* Loop while polling messages received from the server until we received the expected '[OK]'. */
		while ( !bOkReceived && ( xTaskGetTickCount() - xWaitForOkStart <  TO_GSM_SRV_ACK ) )
		{
			/* First, wait for a TCP RX data indication telling us that the server has sent data. */
			xAtResult = xReceiveResponse( AT_TCP_RX_DATA_IND, AT_NOMSG, AT_NOMSG, TO_GSM_SRV_ACK );
			if ( xAtResult != AT_SUCCESS )
			{
				if ( xAtResult == AT_RESPONSE_TO )
				{
					/* A response timeout could also mean that the GSM module had issued previously a data received
					   indication which the FW failed to treat (which should not really happen but does...). In this
					   case, the receive indication is not renewed when the GSM module receives any further data. So
					   just attempt to read receive data from the GSM module. If there aren't any, the read will fail. */
					V_TRACE_PRINT( TRACE_GSM_RX_OK_RESPONSE_TIMEOUT, TRACE_UART_AND_FILE );
				}
				else
				{
					if ( xAtResult == AT_RESPONSE_ERR )
					{
						V_TRACE_PRINT( TRACE_GSM_RX_OK_RESPONSE_ERR, TRACE_UART_AND_FILE );
					}
					else
					{
						V_TRACE_PRINT( TRACE_GSM_RX_OK_MODULE_ERR, TRACE_UART_AND_FILE );
					}

					/* Received no indication that the server has sent us any data. Return the error. */
					return xAtResult;
				}
			}
	
			/* Now recover the data and compare it against the expected '[OK]' message.
			   First, send command to read the received message. 
			   Note that (as explained before), there is the possibility that there is no receive data available. */
			if ( xModuleState & GSM_TCP_CONN )
			{
				/* Clear TCP socket. */
				xAtResult = xSendAtCommand( &xAtQIRecv, true, true );
			}
			else
			{
				/* TLS socket. */
				xAtResult = xSendAtCommand( &xAtQSSLRecv, true, true );
			}

			if ( xAtResult != AT_SUCCESS )
			{
				return xAtResult;
			}

			/* The actual server command has now been parsed and the command indication been stuffed back into the
			   xParserAtRespQueue queue. We can retrieve it now. 
			   All messages other than the requested type are kept intact in the queue. Thus, if we accidentally receive
			   a server command instead of an [OK], it is preseved in the queue and will be treated later. */
			if ( bQueueCheckIfDataItemInQueue( xParserAtRespQueue, AT_REMOTE_OK ) )
			{
				vRemoveTypeFromQueue( xParserAtRespQueue, AT_REMOTE_OK );
				bOkReceived = true;
			}
		}

		if ( !bOkReceived )
		{
			V_TRACE_PRINT( TRACE_GSM_NO_SERVER_OK, TRACE_UART_AND_FILE );

			return AT_RESPONSE_TO;
		}
	}
	return AT_SUCCESS;
}
/*-----------------------------------------------------------*/

/* Remove all entries in the GPS recording positon store. */
void vDeletePosStore( void )
{
	uxPosStoreWrIdx = 0;
	uxPosStoreRdIdx = 0;
}
/*-----------------------------------------------------------*/

/* Send one or more packets containing one position store entry each to the server. 
   
   Return false, if there was a serious error while accessing the GSM/GPS module. In this case, the 
   calling function will re-try and eventually reset the module.
*/
enum xAT_RESULT prvSendPosStoreEntryPacket( void )
{
	enum xAT_RESULT				xAtResult;
	unsigned portBASE_TYPE		uxTlTblIdx;
	TickType_t					xWaitForOkStart;
	bool						bOkReceived;
	signed char					cTmpStrg[ LEN_GSM_ID + 1 ];
	
	/* Send one packet for each position store entry. */
	while ( uxPosStoreRdIdx != uxPosStoreWrIdx )
	{
		V_TRACE_PRINT_SHORT( TRACE_GSM_POSITION_RECORDED, ( ( unsigned short )uxPosStoreRdIdx << 8 ) + ( unsigned short )uxPosStoreWrIdx, TRACE_UART );			

		/* Should the GSM module be in sleep mode, bring it out of sleep. */
		if ( xModuleGetSleepMode() )
		{
			/* The module is in power-save mode. Bring it out of the sleep mode. */
			xModuleDisableSleepMode();
		}

		/* Give the TCP send (socket write) command. */
		if ( prvPrepareTCPWrite() != AT_SUCCESS )
		{
			return xAtResult;
		}

		/* Lock access to the GSM data. Neither parser nor BLE parser are now allowed to write the data. */
		configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );
		
		/* Now send the data fields in the order of the bits in ulActualDFMap. */
		/* Send the header. For the time being, the FRMT field is fixed to 0. */
		xComSendString( COM_GSM, "[" TL500_FRMT "," );


		/* Send the DFMAP field. */
		xComSendString( COM_GSM, "08000009" );
		xComSendChar( COM_GSM, ',' );	
		
		/* Send the ID. */
		( void )pcConfigReadString( xNvdsConfig.pcGsm_ID, cTmpStrg, LEN_GSM_ID + 1, TRANSLATE_APOSTR );
		xComSendString( COM_GSM, cTmpStrg );
		xComSendChar( COM_GSM, ',' );	
		
		/* Get RTC time string. */
		vShortToHexStrg( cGsm_TIM, usReadRTC() );
		xComSendString( COM_GSM, cGsm_TIM );
		xComSendChar( COM_GSM, ',' );	
		
		/* Send one position store entry. */
		vSendPOSLField( uxPosStoreRdIdx );
				
		/* Restore write access to the GSM data. */
		xSemaphoreGive( xMutexGsmData );
		
		/* Send the trailer followed by CTRL-Z to exit from data mode. */
		xComSendString( COM_GSM, "]\x1A" );	
		
		/* Wait for the OK message from the cellular module. */
		xAtResult = xReceiveResponse( AT_SEND_OK, AT_NOMSG, AT_SEND_FAIL, TO_GSM_TCPWR );
		if ( xAtResult != AT_SUCCESS )
		{ 
			if ( xAtResult == AT_RESPONSE_TO )
			{
				V_TRACE_PRINT_STRG( TRACE_GSM_AT_RESPONSE_TO, ( signed char * )pcAt_QSSLSend, TRACE_UART_AND_FILE );
			}
			else
			{
				V_TRACE_PRINT( TRACE_GSM_SEND_MODULE_ERR, TRACE_UART_AND_FILE );
			}
			/* Received no positive response from the module. Return the error. */
			return xAtResult;
		}
		
		/* Wait for the acknowledgement from the server. In case get a server command first, we need to wait for
		   several data packets from the server. */
		/* Record  start time stamp. */
		xWaitForOkStart =  xTaskGetTickCount();
		bOkReceived = false;
	
		/* Loop while polling messages received from the server until we received the expected '[OK]'. */
		while ( !bOkReceived && ( xTaskGetTickCount() - xWaitForOkStart <  TO_GSM_SRV_ACK ) )
		{
			/* First, wait for a TCP RX data indication telling us that the server has sent data. */
			xAtResult = xReceiveResponse( AT_TCP_RX_DATA_IND, AT_NOMSG, AT_NOMSG, TO_GSM_SRV_ACK );
			if ( xAtResult != AT_SUCCESS )
			{
				if ( xAtResult == AT_RESPONSE_TO )
				{
					/* A response timeout could also mean that the GSM module had issued previously a data received
					   indication which the FW failed to treat (which should not really happen but does...). In this
					   case, the receive indication is not renewed when the GSM module receives any further data. So
					   just attempt to read receive data from the GSM module. If there aren't any, the read will fail. */
					V_TRACE_PRINT( TRACE_GSM_RX_OK_RESPONSE_TIMEOUT, TRACE_UART_AND_FILE );
				}
				else
				{
					if ( xAtResult == AT_RESPONSE_ERR )
					{
						V_TRACE_PRINT( TRACE_GSM_RX_OK_RESPONSE_ERR, TRACE_UART_AND_FILE );
					}
					else
					{
						V_TRACE_PRINT( TRACE_GSM_RX_OK_MODULE_ERR, TRACE_UART_AND_FILE );
					}

					/* Received no indication that the server has sent us any data. Return the error. */
					return xAtResult;
				}
			}
	
			/* Now recover the data and compare it against the expected '[OK]' message.
			   First, send command to read the received message. 
			   Note that (as explained before), there is the possibility that there is no receive data available. */
			if ( xModuleState & GSM_TCP_CONN )
			{
				/* Clear TCP socket. */
				( void )xSendAtCommand( &xAtQIRecv, true, true );
			}
			else
			{
				/* TLS socket. */
				( void )xSendAtCommand( &xAtQSSLRecv, true, true );
			}

			if ( xAtResult != AT_SUCCESS )
			{
				return xAtResult;
			}

			/* The actual server command has now been parsed and the command indication been stuffed back into the
			   xParserAtRespQueue queue. We can retrieve it now. 
			   All messages other than the requested type are kept intact in the queue. Thus, if we accidentally receive
			   a server command instead of an [OK], it is preseved in the queue and will be treated later. */
			if ( bQueueCheckIfDataItemInQueue( xParserAtRespQueue, AT_REMOTE_OK ) )
			{
				vRemoveTypeFromQueue( xParserAtRespQueue, AT_REMOTE_OK );
				bOkReceived = true;
			}
		}

		if ( !bOkReceived )
		{
			V_TRACE_PRINT( TRACE_GSM_NO_SERVER_OK, TRACE_UART_AND_FILE );

			return AT_RESPONSE_TO;
		}

		/* Let the read index point to the next entry. */
		uxPosStoreRdIdx  = ( uxPosStoreRdIdx + 1 ) % POS_STORE_LEN;
	}
	
	return AT_SUCCESS;
}
/*-----------------------------------------------------------*/

/* Timer callback for periodic transmissions to the server. 
   CAUTION: This function is running in the timer task!
*/
void prvPosTxCallback( TimerHandle_t xTimer )
{
	enum xGSM_CMD			xGsmCmd;
	
	( void )xTimer;
	
	/* Timer expired:
	   Send a message to the GSM task to transmit a position packet unless there is already a transmission pending
	   or the device is in STANDBY state. */
	if ( !bQueueCheckIfDataItemInQueue( xGsmCmdQueue, GSM_SEND_POSITION ) && ( xGetCtrlState() != CTRL_STANDBY ) )
	{
		/* Only queue a new request to send a position packet if there is no attempt already ongoing.
		   This avoids filling up the queue when a previous packet is stuck. */			   
		xGsmCmd = GSM_SEND_POSITION;
		xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmCmd, 0 );		
	}
}
/*-----------------------------------------------------------*/

/* Update the timer interval to align with the device state. 
   Priority between ne value and remaining time before timeout has already been sorted out.
*/
void prvUpdateTransmissionTimer( unsigned long ulNewPositionInterval )
{
	unsigned long	ulPositionSendInterval;
	
	/* A ulNewPositionInterval of 1 means : ASAP. */
	if ( ulNewPositionInterval != 1 )
	{
		/* Convert position send interval from seconds to ticks. */
		ulPositionSendInterval = ulNewPositionInterval * portTICKS_PER_SEC;
		
		/* Safety net: xTimerChangePeriod crashes with new period set to 0. */
		ulPositionSendInterval = ( ulPositionSendInterval == 0 ) ? 1 : ulPositionSendInterval;		
		
		/* Reprogram the timer. */
		( void )xTimerChangePeriod( xPosTxTimer, ulPositionSendInterval, portMAX_DELAY );
	}
	else
	{
		/* Program the timeout to 1s (i.e. ASAP). */
		if ( xTimerGetExpiryTime( xPosTxTimer ) - xTaskGetTickCount() > 1 * portTICKS_PER_SEC )
		{
			( void )xTimerChangePeriod( xPosTxTimer, 1 * portTICKS_PER_SEC, portMAX_DELAY );
		}
		else
		{
			( void )xTimerStart( xPosTxTimer, portMAX_DELAY );
		}
	}
}
/*-----------------------------------------------------------*/


/* Check if conditions for shutdown of the GSM/GPS module are met:
	- No position transmission is pending (should not be the case).
	- No server command transaction is ongoing (should not be the case).
	- Sufficient time remaining until the next position transmission.
*/
void vCheckConditionsAndShutdownModule( enum xGSM_SHUTDOWN_REQ_TYPE xGsmShutdownReqType )
{
	TickType_t 	xRemainingTime;
	TickType_t	xModuleShutdownDelay;

	/* Cancel the module shut-down timer, should it be running. */
	( void )xTimerStop( xModuleShutdownTimer, 0 );
	/* Remove any pending requests to shutdown the GSM/GPS module. We will decide now what to do. */
	vRemoveTypeFromQueue( xGsmCmdQueue, GSM_SHUTDOWN_MODULE );
	
	/* Calculate the time necessary for the module to stay on a bit longer so that it will receive potential further server commands. */
	if (   ( xGsmShutdownReqType == FROM_POSITION_UPDATE )
	    || ( xGsmShutdownReqType == FORCE_SHUTDOWN ) )
	{
		/* The last transaction was for a position transmission, so use the appropriate delay. */
		xModuleShutdownDelay = MOD_SHUTDWN_DELAY_POS;
	}
	else
	{
		/* The last transaction was for a server command, so use the appropriate delay. */
		xModuleShutdownDelay = MOD_SHUTDWN_DELAY_CMD;
	}		
	
	/* Check, if there are any updates to the position interval timer pending due to e.g. detected alerts.
	   If there are those messages, update the timer immediately. */
	if (   bQueueCheckIfDataItemInQueue( xGsmCmdQueue, GSM_SEND_ALERT_SOS ) 
		|| bQueueCheckIfDataItemInQueue( xGsmCmdQueue, GSM_SEND_FOREIGN_ALERT_SOS ) )		
	{
		/* Send immediately an alert or SOS message. The alert/SOS is sent as a default message
		   with the alert or SOS bit set in the indications bit map field. */
		/* Send the message as soon as possible. Set the timer interval to 1s. */
		prvUpdateTransmissionTimer( 1 );
		V_TRACE_PRINT( TRACE_GSM_ALERT_OR_SOS_DETECTED, TRACE_UART_AND_FILE );
	}
	
    /* Calculate the time that remains before the xPosTxTimer timer expires.  TickType_t is an unsigned (32-bit) type, 
	   so the subtraction will result in the correct answer even if the timer will not expire until after the tick
       count has overflowed. 
	   In GPS recording, the time to the next Hello packet is used, unless the module is on the charger, in alert/SOS or has received
	   a foreign distress. In all those cases, the normal send interval is applied. */
	if (    ( bool )( usConfigReadShort( &xNvdsConfig.usGpsRecording ) & GPS_RECORDING_ENABLE )
		 && !( bool )( usConfigReadShort( &xNvdsConfig.usGpsRecording ) & GPS_RECORDING_DEBUG )
		 && ( uxBleTLBcnCnt == 0 )
		 && ( !bGetEvacOngoing() )
		 && ( xGetCtrlState() != CTRL_ALERT )
		 && ( xGetCtrlState() != CTRL_SOS )
		 && ( xGetCtrlState() != CTRL_CHRG_NRML )		 
		  ) 
	{			 
		/* In GPS recording, the time until the next 'HELLO' packet needs to be used instead of the remaining time of the 
		   xPosTxTimer. */
		xRemainingTime = xGetTimeToNextHello();
	}
	else
	{
		xRemainingTime = xTimerGetExpiryTime( xPosTxTimer ) - xTaskGetTickCount();
	}
	V_TRACE_PRINT_LONG( TRACE_DBG5, xRemainingTime, TRACE_UART_AND_FILE );
		
	/* Subtract the shutdown delay from the remaining time. Caution: if there is no remaining time left, 
	   the operation has to yield 0 (no underflow!). */
	xRemainingTime = ( xRemainingTime > xModuleShutdownDelay ) ? xRemainingTime - xModuleShutdownDelay : 0;		
	
	/* Test if there are no transactions ongoing, that no GPS is required and that the interval to the next 
	   transmission is large enough so that we can shut down the GSM module. The limit for the module 
	   shutdown is read directly from the NVDS. 
	   In STANDBY state, immediately shut down the GSM module. */
	if (   !bQueueCheckIfDataItemInQueue( xGsmCmdQueue, GSM_SEND_POSITION )
		&& !bCommandTransactionOngoing )
	{
		if (    ( xRemainingTime > ( TickType_t )usConfigReadShort( &xNvdsConfig.usModulePwrDownInt ) * portTICKS_PER_SEC )
			 || ( xGsmShutdownReqType == FORCE_SHUTDOWN ) 
			 || ( xGetCtrlState() == CTRL_STANDBY ) )
		{
			/* Start the shutdown timer. If the timer was already running, it is reset to the default expiry time. */
			xTimerStart( xModuleShutdownTimer, portMAX_DELAY );

			/* Program the shutdown timer for the appropriate delay. */
			( void )xTimerChangePeriod( xModuleShutdownTimer, xModuleShutdownDelay, portMAX_DELAY );
		}
	}
	
	/* Enable the module's power save mode so that it will go into low-power mode after about 20s. */
	xModuleEnableSleepMode();
}
/*-----------------------------------------------------------*/

/* Check is GPS assisted Data is outdated. 

   Returns true, if data is still valid.
*/
bool bCheckAssistedDataValid( void )
{
	unsigned long			ulCurrentTime;
	unsigned long			ulAssistedGPSDataAge;

	/* Check how much time has elapsed since the GPS assisted database was last updated. The time delta
	   is calculated in seconds. */
	ulCurrentTime = ulReadRTC();
	ulAssistedGPSDataAge = ulCurrentTime - ulAssistedGPSUpdateTimeStamp;

	return ( ulAssistedGPSDataAge <= T_GPS_ASSISTED_DATA_VALIDITY );
}
/*-----------------------------------------------------------*/

/* Checks if there are enough BLE localiser beacons received to replace a GPS fix by a BLE server-supported fix. */
bool bEnoughBleBcnsForFix( void )
{
	return (    ( ( xBleLocBcnGSM.uxBleStdLocBcnCnt + xBleLocBcnGSM.uxBleSwissPhoneLocBcnCnt ) >= usConfigReadShort( &xNvdsConfig.usBleMinBcn ) )
			 && ( bool )usConfigReadShort( &xNvdsConfig.usBleEnable ) );
}
/*-----------------------------------------------------------*/

/* Check if sufficient time remains until the next position transmission to shutdown the GPS module. */
void vCheckConditionsAndShutdownGPS( void )
{
	TickType_t			xRemainingTime;
	enum xGPS_CMD		xGpsCmd;
	bool				bShutDownPermitted;

    /* Calculate the time that remains before the xPosTxTimer timer expires.  TickType_t is an unsigned type, 
  	   so the subtraction will result in the correct answer even if the timer will not expire until after the tick
       count has overflowed. */
	xRemainingTime = xTimerGetExpiryTime( xPosTxTimer ) - xTaskGetTickCount();
	
	/* Check if the GPS module needs to be kept running. This is the case in emergency (ALERT/SOS state or an distress beacon has been received)
	   or the module is in ACTIVE state without GPS or other fix.
	   If there are enough BLE beacons or there was a previous GPS fix or if the module is in SLEEP, it can be shut down unconditionally.
	   Also, if there was no GPS or BLE fix and the module is in either INACTIVE or STILL states, it can be shut down. In those states, 
	   the GPS conditions are not expected to improve rapidly. In all other module states, either the wearer is moving which might result 
	   in changing RX conditions or there is an emergency during which the GPS module needs to keep running. */
	if (   ( xGetCtrlState() == CTRL_SLEEP ) 
		|| bHasGPSGoodFix()
		|| bEnoughBleBcnsForFix()
	   ) 
	{
	   bShutDownPermitted = true;
	}
	else
	{
		if (    !bHasGPSGoodFix()
			 && ( uxBleTLBcnCnt == 0 )
			 && ( !bGetEvacOngoing() )
		     && (    ( xGetCtrlState() == CTRL_INACTIVE )
			      || ( xGetCtrlState() == CTRL_STILL ) 
			    )
		   )
		{
			bShutDownPermitted = true;
		}
		else
		{
			bShutDownPermitted = false;
		}		
	}
	
	/* Test if  the interval to the next transmission is large enough so that we can shut down the GPS module. The 
	   limit for the GPS shutdown is read directly from the NVDS. */
	if ( 	bShutDownPermitted
		 && ( xRemainingTime > ( TickType_t )usConfigReadShort( &xNvdsConfig.usGpsPwrDownInt ) * portTICKS_PER_SEC ) )	
	{
		xGpsCmd = GPS_STOP;
		xQueueSend( xGpsCmdQueue, &xGpsCmd, 0 ); 
	}
}
/*-----------------------------------------------------------*/

/* Download a new set of GPS assistance data and send it to the nRF.

   The GPS receiver is assumed to be running at this point. 
*/
bool bDownloadGpsAssistanceData( void )
{
	/* Get an initial geolocalisation estimation from the server based on the cellular data. */
	vRequestInitialPosition();

	/* Request the assistance data from the ublox server and Send the data to the nRF so that it 
	   can be forwarded to the GPS module. */
	if ( !bRequestAssistanceData() )
	{
		return false;
	}
	
	usGpsInitialPosEstimateTS = usReadRTC();
	
	return true;
}
/*-----------------------------------------------------------*/

/* Start the GPS module and get a GPS fix.

   This function is called in two different scenarios:
   
   1. GPS assistance data is available and up-to-date. The GPS receiver can be run stand-alone and does not require any
      interaction with the GSM module.
   2. GPS assistance data was not available and had just been downloaded (happens only once every ~2h). This means that 
      the GSM module is on now. 
	  With the GSM module being on, check for module crashes and server commands while waiting for a GPS fix.
   
   If bGsmModuleIsOn, it returns the GSM module state (true, if the module is alive). 
   If bGsmModuleIsOn is set false, returns always true. 
*/
bool bGetGpsPosition( void )
{
	TickType_t				xGPSStartTime;
	enum xGPS_CMD			xGpsCmd;
	unsigned portBASE_TYPE	uxGreenLedBlinkCnt;	
	bool					bModuleReady;
	bool					bGoodGPSFix;

	
	/* Provided that the GSM module is on, bModuleReady tells that the GSM/GPS module is powered up, 
	   GPS has a good fix if required and TCP is established. The bModuleReady status is returned by the function. */
	/* From here on, bModuleReady is set false whenever there is an init error. */
	bModuleReady = true;

	/* Assisted GPS data is available so the GPS can run stand-alone. */
	/* Check the GPS assistance data validity and download a new set, if required. The GPS receiver is started in the process. */
	/* Switch on the GPS receiver. Record any GPS module start-up crashes. */
	xGpsCmd = GPS_START;
	xQueueSend( xGpsCmdQueue, &xGpsCmd, 0 ); 
	
	/* Wait for a certain time until a good GPS fix is available. If the GPS switch-on was not successful, skip this stage.
	   Continously check if the module is still considered configured. */
	uxGreenLedBlinkCnt = 0;

	/* Suspend sending any BLE in order to reduce noise on the GPS receiver. 
	   TODO: What shall we do in ALERT or SOS state? Stop advertising distress beacons
			 in favor of a potential GPS fix (which might never happen) or continue advertising 
			 and allow being found while compromising the GPS reception? */
	if ( ( xGetCtrlState() != CTRL_ALERT ) && ( xGetCtrlState() != CTRL_SOS ) )
	{
		vNonBlockingSuspendBleTxActivity();
	}
	
	/* Note that checking for GPS ON state while waiting for a fix is required. If the module entered SLEEP while being in this loop, 
	   the GPS gets shut down and the loop would continue to wait. */
	xGPSStartTime = xTaskGetTickCount();
	bGoodGPSFix = false;
	while (    bModuleReady
	        && ( xTaskGetTickCount() - xGPSStartTime < ( TickType_t )ulConfigReadLong( &xNvdsConfig.ulGPSMaxWaitForFix ) ) 
			&& (    ( xTaskGetTickCount() - xGPSStartTime < ( TickType_t ) ulConfigReadLong( &xNvdsConfig.ulGPSMaxWaitForSat ) ) 
			     || ( uxNumSvnEntries() >= MIN_SV_RECVD ) )
			&& !bGoodGPSFix 
			&& !bPushPacketImmediately 
			&& bCtrlGpsPositionRequired() )
	{
		/* Just stay here and wait. If the loop times out, then go ahead and send a packet without waiting further for a fix. */
		
		/* Check for data reception from a satellite. This means that we send a series of commands to the GSM/GPS module to 
		   query its status. */
		/* Do the following actions only once, i.e. while the module is still in IDLE mode. */
		bGoodGPSFix = bHasGPSGoodFix();

		/* If the GSM module is on and connected, check if a server command has been received while waiting for the GPS fix. If this is the case, 
		   treat it. */
		if ( bGetTcpConnected() )
		{
			( void )bCheckForServerCommand();

			if ( ( xModuleState & GSM_PWR_MSK ) == GSM_PWR_OFF )
			{
				/* The module has crashed. Exit the GPS waiting loop now and let the main loop restart the module. */
				bModuleReady = false;
			}		
		}
							
		/* Add a delay if the fix was not good. */
		if ( !bGoodGPSFix && bModuleReady )
		{
			/* Blink the green LED every third loop in the case where no 1pps signal is received
			   or every loop if the 1pps signal is received. */
			if ( uxGreenLedBlinkCnt >= 3 )
			{
				uxGreenLedBlinkCnt = 0;
				
				vSwitchOnLedGreen( ledGREEN_GSM_REQ );
				vTaskDelay( 0.02 * portTICKS_PER_SEC );
				vSwitchOffLedGreen( ledGREEN_GSM_REQ );							
			}
			else
			{
				uxGreenLedBlinkCnt++;
			}
				
			/* Wait before polling the next fix data. */
			vTaskDelay( T_GPS_POLL );
		}
	}
	
	/* Test, if GPS is still required. If not, shut the GPS module down unconditionally. */
	if ( !bCtrlGpsPositionRequired() )
	{
		vCheckConditionsAndShutdownGPS();
	}
	else
	{
		/* Shut down the GPS module while checking conditions like time-to-next-fix, if either:
			- we got a good fix
			- or the module is in SLEEP state. 
			- or the GPS assisted data is outdated (because the previous download has failed)
			If the module is not ready (bModuleReady is false), something bad has happened before and we are going to reset
			the module anyway. So in this case do not bother about doing a proper shutdown. */
		if (    bModuleReady
			 && (   bGoodGPSFix 
				 || ( xGetCtrlState() == CTRL_SLEEP )
				 || !bCheckAssistedDataValid()
				 || bEnoughBleBcnsForFix()
				)
			)
		{
			/* We we got a GPS fix or it is unlikely to get one anytime soon, so stop the GPS module now, if permitted. */
			vCheckConditionsAndShutdownGPS();
		}
	}
	
	/* Save the GPS fix data if it might be used later as location estimation for the next Assistance Data download. */
	if ( bGoodGPSFix && bModuleReady )
	{
		/* Copy the GPS location data to the estimation string. 
		   The format is:
					lat, lon,alt[m],prec[m]
					43.621134,7.063414,0,5000
		   Altitude is always set to 0 and precision to 5km. */
		char		*pcStrPos;
		short		sGpsTmp;
	   
		/* The cGsm_BLE_LAT data field is coded in format 1, for example: N 4337.421095 */
		pcStrPos = cGpsInitialPosEstimate;
		if ( cGsm_BLE_LAT[ 0 ] == 'S' )
		{
			*( pcStrPos++ ) = '-';
		}
		pcStrPos = stpncpy( pcStrPos, cGsm_BLE_LAT + 2, 2 );
		*( pcStrPos++ ) = '.';
		pcStrPos = stpncpy( pcStrPos, cGsm_BLE_LAT + 4, 2 );
		pcStrPos = stpncpy( pcStrPos, cGsm_BLE_LAT + 7, 2 );

		*( pcStrPos++ ) = ',';

		/* The cGsm_BLE_LON data field is coded in format 1, for example: E 00702.338457.
		   Convert that field into a float string. */
		if ( cGsm_BLE_LON[ 0 ] == 'W' )
		{
			*( pcStrPos++ ) = '-';
		}
		pcStrPos = stpncpy( pcStrPos, cGsm_BLE_LON + 2, 3 );
		*( pcStrPos++ ) = '.';
		pcStrPos = stpncpy( pcStrPos, cGsm_BLE_LON + 5, 2 );
		pcStrPos = stpncpy( pcStrPos, cGsm_BLE_LON + 8, 2 );
		   
		strcpy( pcStrPos, ",0,5000" );
		bGpsInitialPosEstimateInitialised = true;
	}

	/* Resume sending BLE. */
	vResumeBleTxActivity();

	return bModuleReady;
}
/*-----------------------------------------------------------*/

/* Check, if GPS recording is allowed and, if yes, record the position. 

   GPS recording is allowed if:
		- Enabled in the configuration.
		- Device is not on charger.
		- Device is not in ALERT.
		- Device is not in SOS.
		- No foreign TL distress beacon has been received.
		- Battery is not almost dead in which case the alert must be communicated to the server.
		
   Return true to tell the calling function that a position as been recorded successfully and no position must be 
   sent to the server. 		
*/
bool bRecordPosition( void )
{
	if (    ( usConfigReadShort( &xNvdsConfig.usGpsRecording ) & GPS_RECORDING_ENABLE )
		 && !bGetOnCharger()
		 && ( xGetCtrlState() != CTRL_ALERT ) 
		 && ( xGetCtrlState() != CTRL_SOS ) 
		 && ( uxBleTLBcnCnt == 0 )
		 && ( !bGetEvacOngoing() )
		 && !bBatteryLastGasp() )
	{		
		/* usPosStoreAbsStepCount counts the steps in between position recording entries. */
		usPosStoreAbsStepCount += usAbsStepCount;
		
		/* Store the current GPS and BLE position in the position record. */
		if ( bHasGPSGoodFix() || ( ( xBleLocBcnGSM.uxBleStdLocBcnCnt + xBleLocBcnGSM.uxBleSwissPhoneLocBcnCnt ) > 0 ) )
		{
			vShortToHexStrg( xGsmPosStore[ uxPosStoreWrIdx ].cPosTIM, usReadRTC() );
			xGsmPosStore[ uxPosStoreWrIdx ].cPosSTATE[ 0 ] = cNibbleToChar( xGetCtrlState() );
			xGsmPosStore[ uxPosStoreWrIdx ].cPosSTATE[ 1 ] = 0;
			if ( usPosStoreAbsStepCount > 0 )
			{
				vShortToHexStrg( xGsmPosStore[ uxPosStoreWrIdx ].cPosSTEPS, usPosStoreAbsStepCount );
			}
			else
			{
				xGsmPosStore[ uxPosStoreWrIdx ].cPosSTEPS[ 0 ] = 0;
			}
			
			vStoreGPSPosition();
			vStoreBLEAdvertisers();
		
			/* Increment the write pointer. If the write pointer is at the last usable index (POS_STORE_LEN - 1),
			   wrap it around and push the read pointer ahead. */
			uxPosStoreWrIdx  = ( uxPosStoreWrIdx + 1 ) % POS_STORE_LEN;
			if ( uxPosStoreWrIdx == uxPosStoreRdIdx )
			{
				uxPosStoreRdIdx  = ( uxPosStoreRdIdx + 1 ) % POS_STORE_LEN;
			}	
			
			/* Reset the step count as it has been recorded in the position store. */
			usPosStoreAbsStepCount = 0;
			
			V_TRACE_PRINT_SHORT( TRACE_GSM_POSITION_RECORDED, ( ( unsigned short )uxPosStoreRdIdx << 8 ) + ( unsigned short )uxPosStoreWrIdx, TRACE_UART );			
		}		
		
		/* In position recording debug mode, do not shut down the cellular module and GPS receiver yet
		   as a packet will be set to the server. */
		if ( usConfigReadShort( &xNvdsConfig.usGpsRecording ) & GPS_RECORDING_DEBUG )
		{
			return false;
		}

		/* Shut down the cellular module if it is on. */
		if ( xModuleState & GSM_PWR_ON )
		{
			vCheckConditionsAndShutdownModule( FORCE_SHUTDOWN );
		}

		/* Stop the GPS module now, if permitted. */
		vCheckConditionsAndShutdownGPS();
		
		/* Clear all GPS data. */
		prvInvalidateData();

		/* Reset absolute step count (see motiondet.c). */
		usAbsStepCount = 0;

		/* Return without performing the transmission to the server. */
		return true;
	}
	
	/* Recording is not enabled or a position must be sent to the server. Return false. */
	return false;
}
/*-----------------------------------------------------------*/

/* Check, if the packet currently scheduled for sending is a simple 'Hello' packet while in GPS recording mode.
   These packets are regularly sent to tell the server that the module is alive and check in for pending commands.

   Return true if this is a 'Hello' packet.
*/
bool bIsHelloPacketInGpsRecording( void )
{
	if (    ( usConfigReadShort( &xNvdsConfig.usGpsRecording ) & GPS_RECORDING_ENABLE )
		 && !bGetOnCharger()
		 && ( xGetCtrlState() != CTRL_ALERT ) 
		 && ( xGetCtrlState() != CTRL_SOS ) 
		 && ( uxBleTLBcnCnt == 0 )
	   	 && ( !bGetEvacOngoing() )
	   )
	{	
		return true;
	}
	
	return false;
}
/*-----------------------------------------------------------*/

/* Get the GSM module state. */
portBASE_TYPE xGetGsmModuleState( void )
{
	return xModuleState;
}
/*-----------------------------------------------------------*/

/* Estimate the time to the next Hello packet in GPS recording mode. */
TickType_t xGetTimeToNextHello( void )
{
	TickType_t		xEstimatedTimeToNextHello;	
	
	if ( usConfigReadShort( &xNvdsConfig.usGpsRecordingPacketInterval ) == 0 )
	{
		/* Never send a Hello packet. */
		xEstimatedTimeToNextHello = portMAX_DELAY;
	}
	else
	{
		if ( ( TickType_t )usConfigReadShort( &xNvdsConfig.usGpsRecordingPacketInterval ) == 0xffff ) 
		{
			/* GPS recording w/ synchronized Hello packets: The interval is smaller than the GSM_GPS_RECORDING_SLEEP_INT 
			   constant (typically ~150min). If the module is not in SLEEP, the interval is determined by the GPS
			   assistance data update interval (T_GPS_ASSISTED_DATA_VALIDITY = 120min). */
			xEstimatedTimeToNextHello =   xPacketSentInGpsRecordingTime 
										+ T_GPS_ASSISTED_DATA_VALIDITY * portTICKS_PER_SEC 
										- xTaskGetTickCount();
		}
		else
		{
			/* If Hello packets are not synchronised, their interval is a configuration item. */
			xEstimatedTimeToNextHello =   xPacketSentInGpsRecordingTime
										+ ( TickType_t )usConfigReadShort( &xNvdsConfig.usGpsRecordingPacketInterval ) * portTICKS_PER_SEC 
										- xTaskGetTickCount();
		}
	}
	
	return xEstimatedTimeToNextHello;
}
/*-----------------------------------------------------------*/

/* Timer expired: Transmit a position packet to the server. Make sure the packet gets delivered. Handle GSM/GPS module power-down. 

   The transmit procedure uses two options for obtaining a GPS fix:
		1. Before switching on the cellular module, if all information required for the GPS to run is already available
		2. After attaching the cellular module to the wireless network. In this case, the information necessary for AssistedGPS
		   is downloaded and installed before powering up the GPS. 
*/
void prvTransmitPosition( void )
{
	enum xAT_RESULT			xAtResult;
	bool					bModuleReady;
	enum xGPS_CMD			xGpsCmd;
	bool					bCfgIsGpsRecordingActive;
	bool					bCfgIsGpsRecordingDebugActive;
	bool					bCfgNeverSendHelloInRecording;
	bool					bCfgSyncHelloWithGpsInRecording;
	bool					bCfgBleScanDuringFixOnly;
	bool					bPositionAlreadyRecordedInDebug;
	bool					bTimeToNextHelloNotYetElapsed;
	bool					bSleepAndTimeToNextHelloNotYetElapsed;
	bool					bServerCommandTreated;
			
	
	/* Cancel the module shut-down timer, should it be running. */
	( void )xTimerStop( xModuleShutdownTimer, 0 );
	/* Remove any pending requests to shutdown the GSM/GPS module. */
	vRemoveTypeFromQueue( xGsmCmdQueue, GSM_SHUTDOWN_MODULE );
	
	/* Update the timer interval to avoid having the timer trigger too often when a shortened interval was requested. */
	prvUpdateTransmissionTimer( xGetCtrlPositionSendInterval() );	
	
	/* Blink the green LED once for 200ms. */
	vSwitchOnLedGreen( ledGREEN_GSM_REQ );
	vTaskDelay( 0.2 * portTICKS_PER_SEC );
	vSwitchOffLedGreen( ledGREEN_GSM_REQ );
	
	/* Fill in a couple of boolean variable which make reading of the (complicated) decision making for recording and sending
	   packets a little easier. */
	bCfgIsGpsRecordingActive      	= ( bool )( usConfigReadShort( &xNvdsConfig.usGpsRecording ) & GPS_RECORDING_ENABLE );
	bCfgIsGpsRecordingDebugActive 	= ( bool )( usConfigReadShort( &xNvdsConfig.usGpsRecording ) & GPS_RECORDING_DEBUG );
	bCfgNeverSendHelloInRecording 	= usConfigReadShort( &xNvdsConfig.usGpsRecordingPacketInterval ) == 0;
	bCfgSyncHelloWithGpsInRecording = usConfigReadShort( &xNvdsConfig.usGpsRecordingPacketInterval ) == 0xffff;
	bCfgBleScanDuringFixOnly	  	= ( bool )( usConfigReadShort( &xNvdsConfig.usBleScanDuringFixOnly ) );
	
	if ( bCfgIsGpsRecordingActive )																				// DEBUG DEBUG DEBUG
	{
		V_TRACE_PRINT_BYTE( TRACE_DBG0,  bCfgIsGpsRecordingDebugActive, TRACE_UART_AND_FILE );					// DEBUG DEBUG DEBUG
		V_TRACE_PRINT_BYTE( TRACE_DBG0,  bCfgNeverSendHelloInRecording, TRACE_UART_AND_FILE );					// DEBUG DEBUG DEBUG
		V_TRACE_PRINT_BYTE( TRACE_DBG0,  bCfgSyncHelloWithGpsInRecording, TRACE_UART_AND_FILE );				// DEBUG DEBUG DEBUG
		V_TRACE_PRINT_BYTE( TRACE_DBG0,  bCfgBleScanDuringFixOnly, TRACE_UART_AND_FILE );						// DEBUG DEBUG DEBUG
	}

	/* Default: no position recorded yet when in recording mode with debug. */
	bPositionAlreadyRecordedInDebug = false;
	
	/* In position recording mode, nothing needs to be done in SLEEP, OOO and STANDBY states. 
	   In OOO and STANDBY states, the module will never again communicate with the server until placed on the charger. */
	bSleepAndTimeToNextHelloNotYetElapsed =    ( xGetCtrlState() == CTRL_SLEEP )
										    && ( xTaskGetTickCount() - xPacketSentInGpsRecordingTime )
										         < ( GSM_GPS_RECORDING_SLEEP_INT * portTICKS_PER_SEC );

	if ( bCfgIsGpsRecordingActive )																				// DEBUG DEBUG DEBUG
	{
		V_TRACE_PRINT_LONG(  TRACE_DBG1, xTaskGetTickCount(), TRACE_UART_AND_FILE );							// DEBUG DEBUG DEBUG
		V_TRACE_PRINT_LONG(  TRACE_DBG1, xPacketSentInGpsRecordingTime, TRACE_UART_AND_FILE );					// DEBUG DEBUG DEBUG
		V_TRACE_PRINT_SHORT( TRACE_DBG1, bSleepAndTimeToNextHelloNotYetElapsed, TRACE_UART_AND_FILE );			// DEBUG DEBUG DEBUG
		V_TRACE_PRINT_BYTE(  TRACE_DBG1, xGetCtrlState(), TRACE_UART_AND_FILE );								// DEBUG DEBUG DEBUG
	}

	/* Check if the packet transmission can be aborted here as there is neither the need to capture a position nor 
	   send a Hello packet. */
	if (    bCfgIsGpsRecordingActive
		 && !bCfgIsGpsRecordingDebugActive
		 && (    bSleepAndTimeToNextHelloNotYetElapsed
		      || ( xGetCtrlState() == CTRL_OOO ) 
		      || ( xGetCtrlState() == CTRL_STANDBY )
			)
		 && !bBatteryLastGasp() )
	{
		/* Shut down the cellular module if it is on. */
		if ( xModuleState & GSM_PWR_ON )
		{
			vCheckConditionsAndShutdownModule( FORCE_SHUTDOWN );
		}

		/* Stop the GPS module now, if permitted. */
		vCheckConditionsAndShutdownGPS();
		
		/* Clear all GPS data. */
		prvInvalidateData();

		/* Reset absolute step count (see motiondet.c). */
		usAbsStepCount = 0;

		/* Return without performing the transmission to the server. */
	   	return;
	}


	/* Decide, if the position acquisition can be run before powering up the cellular module. 
	   Note that bCtrlGpsPositionRequired() returns false if there are sufficient BLE beacons for positioning. */
	if ( bCfgIsGpsRecordingActive )																				// DEBUG DEBUG DEBUG
	{
		V_TRACE_PRINT_SHORT( TRACE_DBG2, bCheckAssistedDataValid(), TRACE_UART_AND_FILE );						// DEBUG DEBUG DEBUG
		V_TRACE_PRINT_SHORT( TRACE_DBG2, bCtrlGpsPositionRequired(), TRACE_UART_AND_FILE );						// DEBUG DEBUG DEBUG	
		V_TRACE_PRINT_SHORT( TRACE_DBG2, bCtrlPositionRequired(), TRACE_UART_AND_FILE );						// DEBUG DEBUG DEBUG
		V_TRACE_PRINT_SHORT( TRACE_DBG2, bPushPacketImmediately, TRACE_UART_AND_FILE );							// DEBUG DEBUG DEBUG
		V_TRACE_PRINT_LONG(  TRACE_DBG2, ulAssistedGPSUpdateTimeStamp, TRACE_UART_AND_FILE );					// DEBUG DEBUG DEBUG
	}

	if (    (    bCheckAssistedDataValid() 
		      || !bCtrlGpsPositionRequired() )
		 && bCtrlPositionRequired()
		 && !bPushPacketImmediately )
	{
		/* Make sure the battery is fit for operation (voltage high enough and temperature within
		   operating range). */
		vCheckVoltageAndTemperature();
		
		/* If BLE scans only during GPS fixes are configured, enable BLE and scan for beacons for a certain (short) time. */
		if ( 	bCfgBleScanDuringFixOnly 
		     && ( bool )( usConfigReadShort( &xNvdsConfig.usBleEnable ) )
		   )
		{
			TickType_t			xBLEStartTime;
			enum xBLE_CMD		xBleCmd;
			
			/* Activate BLE. */
			xBleCmd = BLE_LOC_START;
			xQueueSend( xBleCmdQueue, &xBleCmd, 0 ); 

			xBLEStartTime = xTaskGetTickCount();
			while (   !bEnoughBleBcnsForFix()
				   && ( xTaskGetTickCount() - xBLEStartTime < ( TickType_t )usConfigReadShort( &xNvdsConfig.usBLEMaxWaitForBcn ) * portTICKS_PER_SEC )
				  )
			{
				;
			}
		}
	
		/* If there are not enough BLE beacons for a position fix, the GPS needs to run. */
		if ( !bEnoughBleBcnsForFix() )
		{
			/* The GSM module is off but assistance data is available and valid. 
			   Request a GPS fix from the GPS module. */
			( void )bGetGpsPosition();
		}

		/* If scan is enabled only during GPS fix attempts, disable BLE again - unless in ALERT or SOS or a distress beacon has been received. */
		if ( 	bCfgBleScanDuringFixOnly
			 && ( uxBleTLBcnCnt == 0 )
			 && ( !bGetEvacOngoing() )
		     && ( xGetCtrlState() != CTRL_ALERT )
			 && ( xGetCtrlState() != CTRL_SOS )
		   )
		{
			enum xBLE_CMD		xBleCmd;

			xBleCmd = BLE_LOC_STOP;
			xQueueSend( xBleCmdQueue, &xBleCmd, 0 ); 
		}
		
		if ( bCfgIsGpsRecordingActive )																				// DEBUG DEBUG DEBUG
		{			
			V_TRACE_PRINT_SHORT( TRACE_DBG3, uxBleTLBcnCnt, TRACE_UART_AND_FILE );									// DEBUG DEBUG DEBUG
			V_TRACE_PRINT_SHORT( TRACE_DBG3, bGetEvacOngoing(), TRACE_UART_AND_FILE );								// DEBUG DEBUG DEBUG
		}
		
		/* Check, if GPS recording is required and all conditions fulfilled.
		   If this is the case, record the position and exit the function without pushing a packet to the server. */
		if ( bRecordPosition() )
		{
			if ( bCfgSyncHelloWithGpsInRecording ) 
			{
				bTimeToNextHelloNotYetElapsed =   ( xTaskGetTickCount() - xPacketSentInGpsRecordingTime )
												< ( ( TickType_t )GSM_GPS_RECORDING_SLEEP_INT * portTICKS_PER_SEC );
			}
			else
			{
				bTimeToNextHelloNotYetElapsed =   ( xTaskGetTickCount() - xPacketSentInGpsRecordingTime )
												< ( ( TickType_t )usConfigReadShort( &xNvdsConfig.usGpsRecordingPacketInterval ) * portTICKS_PER_SEC );				
			}
			
			V_TRACE_PRINT_SHORT( TRACE_DBG3, bTimeToNextHelloNotYetElapsed, TRACE_UART_AND_FILE );					// DEBUG DEBUG DEBUG
	
			/* If not in position recording *debug* mode, and sending a 'hello' packet in GPS recording mode is not yet due (or disabled or
			   sync'ed with GPS Assisted data download), abort procedure and return. Do not send a packet. */
			if (    !bCfgIsGpsRecordingDebugActive 
				 && (    bTimeToNextHelloNotYetElapsed 
				      || bCfgNeverSendHelloInRecording 
					)
			   )				
			{
				/* Shut down the cellular module if it is on. */
				if ( xModuleState & GSM_PWR_ON )
				{
					vCheckConditionsAndShutdownModule( FORCE_SHUTDOWN );
				}

				/* Invalidate the data we just put into the position store. */
				prvInvalidateData();

				return;
			}
			else
			{
				/* Else, just remember that a position has already been recorded so it won't be done twice. */
				bPositionAlreadyRecordedInDebug = true;
			}
		}
	}
	
	/* Now switch on the GSM module and prepare it for transmitting data. */
	do
	{
		/* ModuleReady tells that the GSM/GPS module is powered up, GPS has a good fix if required and TCP is established.
		   Loop as long as this is not the case. As each separate module start-up retries its portion several times, 
		   hard-reset the module in this loop. Looping more than once means that something is seriously going wrong. */
		/* From here on, bModuleReady is set false whenever there is an init error. */
		bModuleReady = true;
		
		/* Make sure the battery is fit for operation (voltage high enough and temperature within
		   operating range). As we are in a while-loop here, we need to continuously monitor
		   conditions. */
		vCheckVoltageAndTemperature();

		/* If necessary, switch on the module. If we come back from this function, we are 100% sure that the module is up and
		   running. */
		portENTER_CRITICAL();
		bool bModulePwrOff = ( ( xModuleState & GSM_PWR_MSK ) == GSM_PWR_OFF );
		portEXIT_CRITICAL();
		
		/* If the module is off, switch it on now. */
		if ( bModulePwrOff && bModuleReady ) 
		{
			enum xGPRS_CONNECT_RESULT 	xConnectSuccess;

			xConnectSuccess = xStartCellularModule();

			if ( xConnectSuccess == CONNECT_NO_NW )
			{
				/* No network found. 
				   Verify that there is not any emergency condition, including requests to relay foreign TL (emergency) beacons. */
				if (    ( !bGetEvacOngoing() )
					 && ( uxBleTLBcnCnt == 0 )
					 && ( xGetCtrlState() != CTRL_ALERT )
					 && ( xGetCtrlState() != CTRL_SOS ) )
				{
					/* In that case, defer the next connection attempt.
					   For this, first remove any other packet push requests which may have accumulated during the attach attempt. */
					vRemoveTypeFromQueue( xGsmCmdQueue, GSM_SEND_POSITION );

					/* Avoid in GPS recording that the cellular module attempts another connection at the next time a position is required.
					   This would happen if the GPS assistance data has expired. Instead, the goal is to update the GPS assistance data
					   at the next time a hello packet is due. For this, fake an update of the data now. */
					if ( bCfgIsGpsRecordingActive )
					{
						if ( !bCheckAssistedDataValid() )
						{
							/* Set the AssistNow updated timestamp to now. */
							ulAssistedGPSUpdateTimeStamp = ulReadRTC();
						}
					}

					/* Then, return without any further action and wait for the next TX packet instant. */
					return;
				}

				/* Themodule did not succeed in connecting to the network but there is an ongoing emergency.
				   Continue attempting to push the TX packet. */
				bModuleReady = false;
			}
			else
			{
				bModuleReady = ( xConnectSuccess == CONNECT_SUCCESS );
			}

			/* In GPS recording, we do not attempt any further to push the packet to the server, unless the module is in 
			   any emergency state (ALERT, SOS, foreign alert, evac, etc).
			   Instead, we advance the GPS assitance data validity time stamp and fake that we just sent a packet to the 
			   server. That way, a new attempt is only made once the next HELLO packet is due. Note that in case the GPS
			   assistance data download is coupled with HELLO packets, we have no new assistance data to work with so that 
			   the GPS quality is degraded until the next successful update. */
			if ( !bModuleReady )
			{
				xPacketSentInGpsRecordingTime = xTaskGetTickCount();
				return;
			}
		}
		
		/* The module should be on now. Check it's power save mode status and bring it out of sleep, if necessary. */
		if ( bModuleReady && xModuleGetSleepMode() )
		{
			/* The module is in power-save mode. Bring it out of the sleep mode. */
			xModuleDisableSleepMode();
		}

		/* Disable the module's power save mode so that it will not shut down while attempting to push the packet. 
		   In border-line cases it could happen that the module has shut down between the last test above and 
		   the disable command. In this case, try the sequence once more. */
//		if ( bModuleReady )
//		{
//			bModuleReady = ( vModuleExitPSM() == AT_SUCCESS );	TODO
//		}
		
		/* Write the NVDS log buffer contents (if any) to the trace file. */
		if ( bModuleReady )
		{
			vTraceFlushLog();
		}
	
		/* If BLE scans only during GPS fixes are configured, enable BLE and scan for beacons for a certain (short) time. */
		if (	bModuleReady
			 && bCfgBleScanDuringFixOnly 
		     && bCtrlGpsPositionRequired() 
			 && ( bool )( usConfigReadShort( &xNvdsConfig.usBleEnable ) )
		   )
		{
			TickType_t			xBLEStartTime;
			enum xBLE_CMD		xBleCmd;
			
			/* Activate BLE. */
			xBleCmd = BLE_LOC_START;
			xQueueSend( xBleCmdQueue, &xBleCmd, 0 ); 

			xBLEStartTime = xTaskGetTickCount();
			while (   !bEnoughBleBcnsForFix()
				   && ( xTaskGetTickCount() - xBLEStartTime < ( TickType_t )usConfigReadShort( &xNvdsConfig.usBLEMaxWaitForBcn ) * portTICKS_PER_SEC )
				  )
			{
				;
			}
		}
	
		/* Check, if the GPS position is required in this system state. */
		/* Receiving enough BLE beacons replaces a GPS fix. In this case, if the receiver is already running, stop it now. */
		if ( bEnoughBleBcnsForFix() )
		{
			/* We we got a GPS fix or it is unlikely to get one anytime soon, so stop the GPS module now, if permitted. */
			vCheckConditionsAndShutdownGPS();
		}

		/* If the assistance data are invalid, download a fresh set of data and get a GPS fix.
		   However, if the module is in ALERT state and the packet is the first to transmit in this state, do not attempt to obtain
		   a position fix but push the packet ASAP (variable bPushPacketImmediately). */
		if (    !bCheckAssistedDataValid()
			 && bCtrlGpsPositionRequired() 
			 && !bHasGPSGoodFix()
			 && !bPushPacketImmediately 
			 && bModuleReady )
		{
			/* The Internet is connected. Download the GPS assistance data. */
			if ( !bDownloadGpsAssistanceData() )
			{
				/* Trace status of GPS assistance request and success. */
				V_TRACE_PRINT( TRACE_GSM_GPS_ASSISTED_DATA_DOWNLOAD_FAILED, TRACE_UART_AND_FILE );
			}
			else
			{
				V_TRACE_PRINT( TRACE_GSM_GPS_ASSISTED_DATA_DOWNLOADED, TRACE_UART_AND_FILE );				
			}
			
			/* Start the GPS receiver. */
			xGpsCmd = GPS_START;
			xQueueSend( xGpsCmdQueue, &xGpsCmd, 0 ); 
			
			/* Inform the GPS receiver that new assistance data is available. */
			xGpsCmd = GPS_ANDATA_UPDATED;
			xQueueSend( xGpsCmdQueue, &xGpsCmd, 0 ); 
			
			/* Request a GPS fix from the GPS receiver. As assistance data was not available when
			   entering this function, a GPS fix could not yet be obtained. So it has to be done now. */
			bModuleReady = bGetGpsPosition();
		}
		
		/* If scan is enabled only during GPS fix attempts, disable BLE again - unless in ALERT or SOS or a distress beacon has been received. */
		if ( 	bCfgBleScanDuringFixOnly
			 && ( uxBleTLBcnCnt == 0 )
			 && ( !bGetEvacOngoing() )
		     && ( xGetCtrlState() != CTRL_ALERT )
			 && ( xGetCtrlState() != CTRL_SOS )
		   )
		{
			enum xBLE_CMD		xBleCmd;

			xBleCmd = BLE_LOC_STOP;
			xQueueSend( xBleCmdQueue, &xBleCmd, 0 ); 
		}
		
		/* With position recording enabled, the STEPS field in first packet when entering ALERT/SOS state contains the number of steps since 
		   the last recorded position (different from the normal case where STEPS contains the number of steps since the last packet). */		
		if (    bCfgIsGpsRecordingActive
		     && (    bPushPacketImmediately 
				  || ( uxBleTLBcnCnt != 0 )
				  || bGetEvacOngoing()
				) 
			 && ( usPosStoreAbsStepCount != 0 ) )
		{
			vShortToHexStrg( cGsm_STEPS, usPosStoreAbsStepCount );
		}	
				
		/* Log an error if a GPS position is required but could not be acquired. */
		if (    !bHasGPSGoodFix()
			 && !bIsHelloPacketInGpsRecording()
			 && bCtrlGpsPositionRequired() 
			 && !bPushPacketImmediately 
			 && bModuleReady )
		{
			unsigned short		usGPSFixQuality;
			
			V_TRACE_PRINT( TRACE_GSM_GPS_NO_RECEPTION, TRACE_UART_AND_FILE );
			vTraceGpsFixFail();
		}
		
		/* Check, if GPS recording is required and all conditions for recording only are fulfilled.
		   If this is the case, record the position and exit the function without pushing a packet to
		   the server. 
		   The flag bPositionAlreadyRecordedInDebug is true, if a position has already been recorded in debug
		   mode. This avoids recording the same position twice. */
		if ( bCfgIsGpsRecordingActive )																				// DEBUG DEBUG DEBUG
		{			
			V_TRACE_PRINT_SHORT( TRACE_DBG4, bPositionAlreadyRecordedInDebug, TRACE_UART_AND_FILE );				// DEBUG DEBUG DEBUG

			if ( !bPositionAlreadyRecordedInDebug )
			{
				bool		bNoPositionTransmissionToServerRequired;
				
				bNoPositionTransmissionToServerRequired = bRecordPosition();
				
				V_TRACE_PRINT_SHORT( TRACE_DBG4, bNoPositionTransmissionToServerRequired, TRACE_UART_AND_FILE );	// DEBUG DEBUG DEBUG
				
				if ( bNoPositionTransmissionToServerRequired )
				{
					bTimeToNextHelloNotYetElapsed =   ( xTaskGetTickCount() - xPacketSentInGpsRecordingTime )
													< ( ( TickType_t )usConfigReadShort( &xNvdsConfig.usGpsRecordingPacketInterval ) * portTICKS_PER_SEC );

					V_TRACE_PRINT_SHORT( TRACE_DBG4, bTimeToNextHelloNotYetElapsed, TRACE_UART_AND_FILE );			// DEBUG DEBUG DEBUG
					
					/* Now abort only processing if not in debug mode and no 'hello' packet needs to be sent to the server. 
					   If 'Hello' packets are sync'ed with downloading GPS assisted data, continue processing to push the 'Hello' packet. */
					if (    !bCfgIsGpsRecordingDebugActive
						 && (    bTimeToNextHelloNotYetElapsed 
							  || bCfgNeverSendHelloInRecording 
							)
						 && !bCfgSyncHelloWithGpsInRecording
					   )
					{
						/* Shut down the cellular module unconditionally. In GPS recording, the next transmission event is in any case way too far away
						   to keep the cellular module alive until then. */
						vCheckConditionsAndShutdownModule( FORCE_SHUTDOWN );
						prvInvalidateData();

						return;
					}
				}
			}
		}
		
		/* Check, if the GSM module is still responsive. For this, we have a simple AT-test here. If it fails (e.g. by 
		   time-out), we will convert the error to a severe module error and thus force rebooting the module. 
		   If the module echoes back the AT command itself, also reboot as it seems that the module has silently reset
		   and lost the echo configuration. */
		if ( bModuleReady )
		{
			if ( xSendAtCommand( &xAtAtEcho, true, true ) != AT_SUCCESS )
			{
				bModuleReady = false;
			}
		}
		
		/* Test and reconnect PDP, if necessary. */
		if ( bModuleReady ) 
		{
			/* Test if the PDP context is still active, i.e. the packet data connection is still intact. Else, re-establish. */
			bModuleReady = bTestAndReActivatePDP();
		}
		
		/* Connect TCP if everything went well so far. */
		if ( bModuleReady ) 
		{
			/* Test if the TCP connection is still established. If not, try to re-establish TCP.
			   If querying the TCP status returned 'ERROR', xTestAndReConnectTCP() returns AT_MODULE_ERR. Something bad went on with QTEL, 
			   so that it needs rebooting. This will be accomplished by setting bModuleReady to false. */
			bModuleReady = ( xTestAndReConnectTCP() == AT_SUCCESS );
		}
		
		/* If there was an error in the procedure so far, restart from scratch. Note, that each stage in the procedure tries everything to succeed,
		   i.e. makes several attempts. */
		if ( !bModuleReady )
		{
			/* Something went wrong. If the module is on, increment the statistics counter and power cycle the module. Hopefully this will clear 
			   the source of the problem. 
			   Note that only a module-internal power cycle is performed as opposed to an external (via power switch) in order to keep 
			   the GPS data intact. */
			V_TRACE_PRINT( TRACE_GSM_BOOT_DATA_CONNECTION_FAILED, TRACE_UART_AND_FILE );

			if ( xModuleState & GSM_PWR_MSK )
			{
				uStats.xStats.usGsmModuleReboot++;			
				vGSMModulePowerDown( SOFT_POWER_OFF );
			}

			/* Blink the green LED once more for 200ms to indicate the next round (the FW will try again). */
			vSwitchOnLedGreen( ledGREEN_GSM_REQ );
			vTaskDelay( 0.2 * portTICKS_PER_SEC );
			vSwitchOffLedGreen( ledGREEN_GSM_REQ );
		}
	} 
	while ( !bModuleReady );
	
	/* Write the NVDS log buffer contents (if any) to the trace file. */
	vTraceFlushLog();
	
	/* At this point we are sure that the GSM/GPS module is ready, i.e. it is:
		- Powered-up.
		- GPS is on and has a good fix, if required.
		- TCP is established. 
		- The module is alive.
	*/

	/* Check if a server command has been received. If this is the case, treat it. */
	( void )bCheckForServerCommand();
	
	V_TRACE_PRINT_BYTE( TRACE_DBG5, bHasGPSGoodFix(), TRACE_UART_AND_FILE );		// DEBUG DEBUG DEBUG
	V_TRACE_PRINT_BYTE( TRACE_DBG5, bEnoughBleBcnsForFix(), TRACE_UART_AND_FILE );	// DEBUG DEBUG DEBUG
	
	/* Send position base message to the server. */
	xAtResult = prvSendBasePacket();

	/* Any server command treated from here on will prolong the shutdown phase. */
	bServerCommandTreated = false;
	
	/* In case there are any FTL fields waiting to be transmitted, give the server some time to 
	   send any commands now. We definitely want to avoid mixing sending FTL packets with incoming
	   commands. */
	if ( xAtResult == AT_SUCCESS )
	{
		V_TRACE_PRINT( TRACE_GSM_PACKET_SENT, TRACE_UART_AND_FILE );

		/* Invalidate the data we just successfully sent. */
		prvInvalidateData();
	
		if ( uxBleTLBcnCnt != 0 )
		{
			unsigned portBASE_TYPE		uxCnt;
			
			for ( uxCnt = 0; uxCnt < MOD_INTER_PKT_DELAY_CMD; uxCnt++ )
			{
				vTaskDelay( 1 * portTICKS_PER_SEC );
				if ( bCheckForServerCommand() )
				{
					/* Each time a server command has been treated, the timer is reset. */
					uxCnt = 0;
					bServerCommandTreated = true;
				}
			}
		}

		/* Send the FTL fields if there are any. */
		xAtResult = prvSendFtlPackets();		
		
		/* Treat any server commands received meanwhile. */
		bServerCommandTreated |= bCheckForServerCommand();
	}
	
	/* Send list of recorded positions. */
	if ( xAtResult == AT_SUCCESS )
	{
		/* The position store is never sent if the device is on the charger.
		   Else, if usGpsRecording is not in debug mode (i.e. it is either off or in normal mode), the store is always sent. */
		if ( bCfgIsGpsRecordingActive )
		{
			/* We are in GPS recording mode (normal or debug). */			
			if ( !bCfgIsGpsRecordingDebugActive )
			{
				/* We are in normal GPS recording mode. */
				if (    ( xGetCtrlState() == CTRL_ALERT ) 
				     || ( xGetCtrlState() == CTRL_SOS ) 
				     || ( uxBleTLBcnCnt != 0 ) 
					 || bGetEvacOngoing()
				   )
				{
					/* The device is either in ALERT or SOS state or received a foreign TL distress beacon. 
  					   In either of these cases, send the position store and empty it afterwards. */
					xAtResult = prvSendPosStoreEntryPacket();

					/* Treat any server commands received while sending the position store. */
					bServerCommandTreated |= bCheckForServerCommand();
					
					/* Empty the position store. */
					vDeletePosStore();
				}
				else
				{
					/* Nothing special going on so do not send anything. 
					   We get here when the device is in normal GPS recording mode and (periodically) just says hello to the 
					   server. */
				}
			}
			else
			{
				/* In GPS debug recording mode. Here, the store is always sent but not deleted afterwards. 
				   This means that we get an ever growing list of positions in the store until itis full
				   and the oldest entry is removed. */
				xAtResult = prvSendPosStoreEntryPacket();

				/* Treat any server commands received while sending the position store. */
				bServerCommandTreated |= bCheckForServerCommand();
			}
		}
		else
		{
			/* In normal mode, do nothing. */
		}

		/* Make sure the list of recorded positions is emptied while on charger. */		
		if ( bGetOnCharger() )
		{
			vDeletePosStore();
		}
	}

	/* For position recoding, remember the time that a packet has been pushed to the server to know when the next is due. */
	xPacketSentInGpsRecordingTime = xTaskGetTickCount();

	/* Shut-down procedure. */
	if ( xAtResult == AT_SUCCESS )
	{
		bool		bEnoughBleBcnsForFixBckup;
		bool		bHadGoodFix;
		
		/* Back up the decision if there are enough BLE beacons to replace a GPS fix before erasing the data. */
		bEnoughBleBcnsForFixBckup = bEnoughBleBcnsForFix();
		bHadGoodFix = bHasGPSGoodFix();

		/* The foreign ALERT/SOS has just been sent, so invalidate the flag. */
		uxBleTLBcnCnt = 0;			/* field 25: BLE list of received distress/TL beacons. */
		uxForeignAlertType = 0;
	
		/* FW-update: Reset the NVDS-initialised flag only once the packet has been transmitted to the server and acknowledged. */
		bNVDSReInitialised = false;
	
		/* The re-attach flag has just been sent, so clear it. If ever the GSM module needs to
		   re-attach, it will be set again. */
		bGsmReattach = false;

		/* Reset the vibration motor indication. */
		bVibrationMotorOn = false;

		/* Send succeeded: Shut down the GSM module.
		   Note, that different from previous modules (TL321 and earlier) the cellular module can be shut down even if the GPS module needs 
		   to continue running. */
		if ( !bServerCommandTreated )
		{
			vCheckConditionsAndShutdownModule( FROM_POSITION_UPDATE );
		}
		else
		{
			vCheckConditionsAndShutdownModule( FROM_SERVER_COMMAND );
		}
		
		/* Reset the re-try counter for the next transmission. */
		uxTcpConnectionRetryCounter = 0;
		
		/* Remove any other position transmit requests from the command queue as we just finished transmitting one. 
		   Normally there shouldn't be any but sometimes a request manages to slip through. */
		vRemoveTypeFromQueue( xGsmCmdQueue, GSM_SEND_POSITION );
		
		/* Reset absolute step count (see motiondet.c). */
		usAbsStepCount = 0;
	
		/* Blink the green LED twice for 100ms. */
		vSwitchOnLedGreen( ledGREEN_GSM_REQ );
		vTaskDelay( 0.1 * portTICKS_PER_SEC );
		vSwitchOffLedGreen( ledGREEN_GSM_REQ );
		vTaskDelay( 0.1 * portTICKS_PER_SEC );
		vSwitchOnLedGreen( ledGREEN_GSM_REQ );
		vTaskDelay( 0.1 * portTICKS_PER_SEC );
		vSwitchOffLedGreen( ledGREEN_GSM_REQ );
		
		/* Set the bad boot counter to 0 as at this point we are fairly sure that the current FW is working correctly. */
		vResetBadBootCounter(); 
	}
	else
	{
		/* Send failed. */		
		/* Increment the re-try counter. */
		uxTcpConnectionRetryCounter++;
		
		V_TRACE_PRINT_BYTE( TRACE_GSM_SEND_MODULE_ERR, uxTcpConnectionRetryCounter, TRACE_UART_AND_FILE );
		
		if ( ( xAtResult == AT_MODULE_ERR ) || ( uxTcpConnectionRetryCounter > GSM_PACKET_RETRY_LIM ) )
		{
			/* In case of a serious module error, shut-down the GSM/GPS module. */
			/* Re-try the transmission. For this, force the next loop to occur immediately. */						   
			prvPrepareCellularModuleReboot( HARD_POWER_OFF );
			
			/* Reset the re-try counter as we have just hard-reset the module. We cannot do more than this. */
			uxTcpConnectionRetryCounter = 0;
		}
		
		/* Retry immediately. Set the timer interval to retry in 1s. */
		prvUpdateTransmissionTimer( 1 );
		
		/* Erase GSM debug data */
		cGsm_MCC    [ 0 ] = 0;		/* field 22a: GSM debug data: MCC Mobile Country Code. */
		cGsm_MNC    [ 0 ] = 0;		/* field 22b: GSM debug data: MNC Mobile Network Code. */
		cGsm_TAC    [ 0 ] = 0;		/* field 22c: GSM debug data: TAC Tracking Area Code. */
		cGsm_CI     [ 0 ] = 0;		/* field 22d: GSM debug data: CI Cell Identification. */
		cGsm_RAT    [ 0 ] = 0;		/* field 22e: GSM debug data: RAT Radio Access Technology. */
		cGsm_EARFCN [ 0 ] = 0;		/* field 22f: GSM debug data: EARFCN Extended Absolute Radio Frequency Channel Number. */
		cGsm_TXINST [ 0 ] = 0;		/* field 22g: GSM debug data: TxInst Delay in seconds since transmit instant. */
		cGsm_CHRGI  [ 0 ] = 0;		/* field 22h: GSM debug data: ChrgCurr Charge current if changring is throttled due to temperature. */		
	}
}
/*-----------------------------------------------------------*/

/* Send a raw message to the server, usually containing responses to server commands.
   Return false, if there was a serious error while accessing the GSM/GPS module. In this case, the 
   calling function will re-try and eventually reset the module.
*/
enum xAT_RESULT prvSendRawPacket( const signed char *pcServerRespROM, signed char *pcServerRespRAM, signed char *pcServerRespNVDS )
{
	enum xAT_RESULT			xAtResult;
	bool					bWasInSleep;
	
	/* Exit the module from sleep state and remember the previous state. */
	bWasInSleep = xModuleGetSleepMode();
	xModuleDisableSleepMode();

	/* Give the TCP send (socket write) command. */
	if ( ( xAtResult = prvPrepareTCPWrite() ) != AT_SUCCESS )
	{
		return xAtResult;
	}

	/* Send the header if not already included in the response string at the first position. */
	if ( *pcServerRespROM != '{' )
	{
		xComSendChar( COM_GSM, '{' );
	}
	/* Send the message body. */
	xComSendString( COM_GSM, pcServerRespROM );
	xComSendString( COM_GSM, pcServerRespRAM );
	xComSendString( COM_GSM, pcServerRespNVDS );
	
	/* Send the trailer. */
	xComSendString( COM_GSM, "}\x1a\r\n" );	

	/* Wait for the OK message from the GSM/GPS module. */
	xAtResult = xReceiveResponse( AT_SEND_OK, AT_NOMSG, AT_SEND_FAIL, TO_GSM_TCPWR );
	
	/* Restore the previous sleep state. */
	if ( bWasInSleep )
	{
		xModuleEnableSleepMode();
	}
	
	if ( xAtResult != AT_SUCCESS )
	{ 
		V_TRACE_PRINT( TRACE_GSM_SEND_MODULE_ERR, TRACE_UART_AND_FILE );

		/* Received no positive response from the module. Return the error. */
		return xAtResult;
	}

	/* Return the response from the module. */
	return AT_SUCCESS;
}
/*-----------------------------------------------------------*/

/* Return a hexadecimal byte parameter read from NVDS to the server in a raw packet. 
   The first parameter points to string in ROM (e.g. "BLE_TXPWR="), the second
   parameter to a location in NVDS.
*/
enum xAT_RESULT prvSendByteHexNvdsParameter( const signed char *pcRespStrg, unsigned char *ucNVDSAddress )
{
	signed char		cStrg[ 3 ];
	unsigned char	ucParameter;
	enum xAT_RESULT	xAtResult;
	
	/* Read the configured parameter from NVDS. */
	ucParameter = ucConfigReadByte( ucNVDSAddress );
	
	/* Add the parameter to the string in format Hex-ASCII. */
	vByteToHexStrg( cStrg, ucParameter );
	
	/* Send the packet. */
	xAtResult = prvSendRawPacket( pcRespStrg, cStrg, NULL );
	
	return xAtResult;
}
/*-----------------------------------------------------------*/

/* Return a hexadecimal short parameter read from NVDS to the server in a raw packet. 
   The first parameter points to string in ROM (e.g. "DFMAP="), the second
   parameter to a location in NVDS.
   The length of the string in ROM is limited to 12 characters!
*/
enum xAT_RESULT prvSendShortHexNvdsParameter( const signed char *pcRespStrg, unsigned short *usNVDSAddress )
{
	signed char		cStrg[ 5 ];
	unsigned short	usParameter;
	enum xAT_RESULT	xAtResult;
	
	/* Read the configured parameter from NVDS. */
	usParameter = usConfigReadShort( usNVDSAddress );
	
	/* Add the parameter to the string in format Hex-ASCII. */
	vShortToHexStrg( cStrg, usParameter );
	
	/* Send the packet. */
	xAtResult = prvSendRawPacket( pcRespStrg, cStrg, NULL );
	
	return xAtResult;
}
/*-----------------------------------------------------------*/

/* Return a hexadecimal short parameter read from NVDS divided by 10 to the server in a raw packet. 
   The first parameter points to string in ROM (e.g. "DFMAP="), the second
   parameter to a location in NVDS.
   The length of the string in ROM is limited to 12 characters!
*/
enum xAT_RESULT prvSendShortHexNvdsParameterDiv10( const signed char *pcRespStrg, unsigned short *usNVDSAddress )
{
	signed char		cStrg[ 5 ];
	unsigned short	usParameter;
	enum xAT_RESULT	xAtResult;
	
	/* Read the configured parameter from NVDS. */
	usParameter = usConfigReadShort( usNVDSAddress );
	
	/* Add the parameter to the string in format Hex-ASCII. */
	vShortToHexStrg( cStrg, usParameter / 10 );
	
	/* Send the packet. */
	xAtResult = prvSendRawPacket( pcRespStrg, cStrg, NULL );
	
	return xAtResult;
}
/*-----------------------------------------------------------*/

/* Return a long hexadecimal parameter read from NVDS to the server in a raw packet. 
   The first parameter points to string in ROM (e.g. "DFMAP="), the second
   parameter to a location in NVDS.
   The length of the string in ROM is limited to 12 characters!
*/
enum xAT_RESULT prvSendLongHexNvdsParameter( const signed char *pcRespStrg, unsigned long *ulNVDSAddress )
{
	signed char		cStrg[9];
	unsigned long	ulParameter;
	enum xAT_RESULT	xAtResult;
	
	/* Read the configured parameter from NVDS. */
	ulParameter = ulConfigReadLong( ulNVDSAddress );
	
	/* Add the parameter to the string in format Hex-ASCII. */
	vLongToHexStrg( cStrg, ulParameter );
	
	/* Send the packet. */
	xAtResult = prvSendRawPacket( pcRespStrg, cStrg, NULL );
	
	return xAtResult;
}
/*-----------------------------------------------------------*/

/* Return a hexadecimal short parameter to the server in a raw packet. 
   The first parameter points to string in ROM (e.g. "DFMAP="), the second
   is the value to send.
   The length of the string in ROM is limited to 12 characters!
*/
enum xAT_RESULT prvSendShortHexParameter( const signed char *pcRespStrg, unsigned short usValue )
{
	signed char		cStrg[ 5 ];
	enum xAT_RESULT	xAtResult;
	
	/* Add the parameter to the string in format Hex-ASCII. */
	vShortToHexStrg( cStrg, usValue );
	
	/* Send the packet. */
	xAtResult = prvSendRawPacket( pcRespStrg, cStrg, NULL );
	
	return xAtResult;
}
/*-----------------------------------------------------------*/

/* Read serveral bytes from the memory type given in the argument. */
enum xAT_RESULT prvSendMemoryBytes( signed char *pucAddressValue, signed char *pucResponse, signed char *pucCount )
{
	unsigned long			ulMemAddress;
	unsigned char			*pucMemAddress;
	unsigned short			usByteCount;
	unsigned char			ucValue;
	enum xAT_RESULT			xAtResult;
	
	ulMemAddress = ulHexStrgToLong( pucAddressValue );
	pucMemAddress = ( unsigned char * )ulMemAddress;
	usByteCount = ucHexStrgToByte( pucCount );
	
	/* Give the TCP send (socket write) command. */
	if ( ( xAtResult = prvPrepareTCPWrite() ) != AT_SUCCESS )
	{
		return xAtResult;
	}

	xComSendString( COM_GSM, pucResponse );
	
	/* Remove the ',' and the rest from the command arguments and use the string for the response. */
	cServerCmdParams[ 8 ] = 0;
	xComSendString( COM_GSM, cServerCmdParams );
	xComSendString( COM_GSM, "=0x" );

	while ( usByteCount > 0 )
	{
		/* 	Read the value from the target. */
		ucValue = *( pucMemAddress++ );
		
		/* Send the value as HEX-ASCII. */
		xComSendChar( COM_GSM, cNibbleToChar( ( ucValue >> 4 ) & 0xf ) );
		xComSendChar( COM_GSM, cNibbleToChar( ( ucValue >> 0 ) & 0xf ) );
		
		usByteCount--;
		
		/* Add a ',' if the current value is not the last one. */
		if ( usByteCount > 0 )
		{
			xComSendString( COM_GSM, ",0x" );
		}
	}
		
	/* Send the trailer. */
	xComSendString( COM_GSM, "}\x1a\r\n" );	
	
	/* Wait for the OK message from the GSM/GPS module. */
	xAtResult = xReceiveResponse( AT_SEND_OK, AT_NOMSG, AT_SEND_FAIL, TO_GSM_TCPWR );
	if ( xAtResult != AT_SUCCESS )
	{ 
		V_TRACE_PRINT( TRACE_GSM_SEND_MODULE_ERR, TRACE_UART_AND_FILE );

		/* Received no positive response from the module. Return the error. */
		return xAtResult;
	}

	/* Return the response from the module. */
	return AT_SUCCESS;
}
/*-----------------------------------------------------------*/

/* Write a parameter given by the server to the memory of the given type. 
   The first parameter to a location in memory and the second is the value to write as HEX-ASCII
   string.
*/
enum xAT_RESULT prvWriteByteParameter( unsigned char *pucAddressValue, signed char *pcParamValue, enum xMEM_TYPE xMemType )
{
	unsigned char	ucParameter;
	enum xAT_RESULT	xAtResult;
	
	/* Convert the string in format Hex-ASCII to a binary parameter. */
	ucParameter = ucHexStrgToByte( pcParamValue );

	if ( xMemType == RAM )
	{
		/* Write the new parameter to RAM. */
		*pucAddressValue = ucParameter;
	}
	else
	{
		/* Write the new parameter to NVDS. */
		vConfigWriteByte( pucAddressValue, ucParameter );
	}
	
	/* Send the acknowledgement packet. */
	xAtResult = prvSendRawPacket( "OK", NULL, NULL );
	
	return xAtResult;
}
/*-----------------------------------------------------------*/

/* Write a parameter given by the server to the memory. 
   The first parameter to a location in memory and the second is the value to write as HEX-ASCII
   string.
*/
enum xAT_RESULT prvWriteShortParameter( unsigned short *pusAddressValue, signed char *pcParamValue, enum xMEM_TYPE xMemType  )
{
	unsigned short	usParameter;
	enum xAT_RESULT	xAtResult;
	
	/* Convert the string in format Hex-ASCII to a binary parameter. */
	usParameter = usHexStrgToShort( pcParamValue );
	
	if ( xMemType == RAM )
	{
		/* Write the new parameter to RAM. */
		*pusAddressValue = usParameter;
	}
	else
	{
		/* Write the new parameter to NVDS. */
		vConfigWriteShort( pusAddressValue, usParameter );
	}

	/* Send the acknowledgement packet. */
	xAtResult = prvSendRawPacket( "OK", NULL, NULL );
	
	return xAtResult;
}
/*-----------------------------------------------------------*/

/* Write a parameter given by the server to the memory. Multiply the value from the server
   by 10 to compensate the difference in configTICK_RATE_HZ between TL500 and previous versions.
   The first parameter to a location in memory and the second is the value to write as HEX-ASCII
   string.
*/
enum xAT_RESULT prvWriteShortParameterx10( unsigned short *pusNVDSValue, signed char *pcParamValue )
{
	unsigned short	usParameter;
	enum xAT_RESULT	xAtResult;
	
	/* Convert the string in format Hex-ASCII to a binary parameter. */
	usParameter = usHexStrgToShort( pcParamValue );
	
	/* Write the new parameter to NVDS. */
	vConfigWriteShort( pusNVDSValue, 10ul * usParameter );
	
	/* Send the acknowledgement packet. */
	xAtResult = prvSendRawPacket( "OK", NULL, NULL );
	
	return xAtResult;
}
/*-----------------------------------------------------------*/

/* Write a parameter given by the server to the memory. 
   The first parameter to a location in memory and the second is the value to write as HEX-ASCII
   string.
*/
enum xAT_RESULT prvWriteLongParameter( unsigned long *pulNVDSValue, signed char *pcParamValue, enum xMEM_TYPE xMemType )
{
	unsigned long	ulParameter;
	enum xAT_RESULT	xAtResult;
	
	/* Convert the string in format Hex-ASCII to a binary parameter. */
	ulParameter = ulHexStrgToLong( pcParamValue );
	
	if ( xMemType == RAM )
	{
		/* Write the new parameter to RAM. */
		*pulNVDSValue = ulParameter;
	}
	else
	{
		/* Write the new parameter to NVDS. */
		vConfigWriteLong( pulNVDSValue, ulParameter );
	}

	/* Send the acknowledgement packet. */
	xAtResult = prvSendRawPacket( "OK", NULL, NULL );
	
	return xAtResult;
}/*-----------------------------------------------------------*/

/* Write a short parameter given by the server to the memory. Multiply the value from the server
   by 10 to compensate the difference in configTICK_RATE_HZ between TL500 and previous versions.
   The first parameter to a location in memory and the second is the value to write as HEX-ASCII
   string.
*/
enum xAT_RESULT prvWriteLongParameterx10( unsigned long *pulNVDSValue, signed char *pcParamValue )
{
	unsigned long	ulParameter;
	enum xAT_RESULT	xAtResult;
	
	/* Convert the string in format Hex-ASCII to a binary parameter. */
	ulParameter = ( unsigned long )usHexStrgToShort( pcParamValue );
	
	/* Write the new parameter to NVDS. */
	vConfigWriteLong( pulNVDSValue, 10ul * ulParameter );
	
	/* Send the acknowledgement packet. */
	xAtResult = prvSendRawPacket( "OK", NULL, NULL );
	
	return xAtResult;
}/*-----------------------------------------------------------*/

/* Send the version information to the server. */
enum xAT_RESULT prvSendVersionInformation( void )
{
	enum xAT_RESULT			xAtResult;
	unsigned portBASE_TYPE 	uxPacketLength;	
	signed char				cModuleVer[ MODULE_VER_LEN ];
	signed char				uxCnt;
	static const char 		pcFwVersion[] = ", FW=";
	ble_gap_addr_t			xBleGapAddr;
	signed char				cBtAddr[ 3 ];
	ble_version_t 			xSoftDeviceVersion;
	signed char				*pcBootloaderVersionStrg;
	
	/* Get GSM module version. The result is stored in cServerCmdParams. */
	cModuleVer[ 0 ] = 0;
	xAtResult = xSendAtCommand( &xAtModVersion, true, true );
	if ( xAtResult != AT_SUCCESS )
	{
		return xAtResult;
	}	
	
	/* Copy version string to buffer and count the characters at the same time. Replace any LF '\n' with ' '. */
	uxPacketLength = 0;
	uxCnt = 0;
	while ( ( cServerCmdParams[ uxCnt ] != 0 ) && ( uxCnt < MODULE_VER_LEN ) )
	{
		signed char		cChar;
		
		cChar = cServerCmdParams[ uxCnt++ ];
		if ( cChar == '\n' )
		{
			cChar = ' ';
		}
		cModuleVer[ uxPacketLength++ ] = cChar;
	}

	/* Remove the last character as it is a ' '. */
	cModuleVer[ uxPacketLength - 1 ] = 0;
	
	/* Give the TCP send (socket write) command. */
	if ( prvPrepareTCPWrite() != AT_SUCCESS )
	{
		return xAtResult;
	}

	/* Send the header and HW information string. */
	xComSendString( COM_GSM, "{HW=" HW_VERSION ", IMEI=" );	
	
	/* Add the IMEI. */
	if ( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID ) != 0 )
	{
		signed char		cTmpStrg[ LEN_GSM_ID + 1 ];

		( void )pcConfigReadString( xNvdsConfig.pcGsm_ID, cTmpStrg, LEN_GSM_ID + 1, TRANSLATE_APOSTR );
		xComSendString( COM_GSM, cTmpStrg );
	}
	
	/* Send the header and HW information string. */
	xComSendString( COM_GSM, ", ICCID=" );	
	
	/* Add the ICCID. */
	if ( ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ID ) != 0 )
	{
		signed char		cTmpStrg[ LEN_GSM_ICCID + 1 ];

		( void )pcConfigReadString( xNvdsConfig.pcGsm_ICCID, cTmpStrg, LEN_GSM_ICCID + 1, TRANSLATE_APOSTR );
		xComSendString( COM_GSM, cTmpStrg );
	}
	
	/* Send the FW version. */
	xComSendString( COM_GSM, pcFwVersion );											/* GIT version */
	xComSendString( COM_GSM, cGitVersion );											/* __DATE__: The compile date: "Mmm dd yyyy",
																					   __TIME__: The compile time: "hh:mm:ss" */	
	xComSendString( COM_GSM, ", Module=" );											
	xComSendString( COM_GSM, cModuleVer );											/* Module version: 08.45 */
	
	xComSendString( COM_GSM, ", Chip=" );
	cModuleVer[ 0 ] = ( NRF_FICR->INFO.VARIANT & 0xff000000 ) >> 24;
	cModuleVer[ 1 ] = ( NRF_FICR->INFO.VARIANT & 0x00ff0000 ) >> 16;
	cModuleVer[ 2 ] = ( NRF_FICR->INFO.VARIANT & 0x0000ff00 ) >>  8;
	cModuleVer[ 3 ] = ( NRF_FICR->INFO.VARIANT & 0x000000ff ) >>  0;
	cModuleVer[ 4 ] = 0;
	xComSendString( COM_GSM, cModuleVer );

	xComSendString( COM_GSM, ", Softdevice=0x" );
	sd_ble_version_get( &xSoftDeviceVersion );
	vShortToHexStrg( cModuleVer, xSoftDeviceVersion.subversion_number );
	xComSendString( COM_GSM, cModuleVer );
		
	/* Retrieve and send BLE GAP address. */
	xComSendString( COM_GSM, ", BLE=0x" );
	sd_ble_gap_addr_get( &xBleGapAddr );
		
	for ( uxCnt = 0; uxCnt < 6; uxCnt++)
	{
		vByteToHexStrg( cBtAddr, xBleGapAddr.addr[ 5 - uxCnt ] );
		xComSendString( COM0, cBtAddr );
	}

	/* Search for the bootloader version and send it. */
	if ( bLocateBootloaderVersion( &pcBootloaderVersionStrg ) )
	{
		xComSendString( COM_GSM, ", BL=" );
		xComSendString( COM_GSM, pcBootloaderVersionStrg );
	}
	
	/* Send the trailer. */
	xComSendString( COM_GSM, "}\x1a\r\n" );	
	
	/* Wait for the OK message from the GSM/GPS module. */
	xAtResult = xReceiveResponse( AT_SEND_OK, AT_NOMSG, AT_SEND_FAIL, TO_GSM_TCPWR );
	if ( xAtResult != AT_SUCCESS )
	{ 
		V_TRACE_PRINT( TRACE_GSM_SEND_MODULE_ERR, TRACE_UART_AND_FILE );

		/* Received no positive response from the module. Return the error. */
		return xAtResult;
	}

	/* Return the response from the module. */
	return AT_SUCCESS;
}
/*-----------------------------------------------------------*/

/* Send device state to server. */
enum xAT_RESULT prvSendDeviceState( void )
{
	enum xAT_RESULT			xAtResult;
	signed char				*pcStateName;
	unsigned portBASE_TYPE 	uxPacketLength;
	
	/* Get the device state name from the control module. */
	vGetCtrlStateName( &pcStateName, &uxPacketLength );
	
	/* Give the TCP send (socket write) command. */
	if ( ( xAtResult = prvPrepareTCPWrite() ) != AT_SUCCESS )
	{
		return xAtResult;
	}

	/* Send the actual packet. */
	xComSendChar( COM_GSM, '{' );
	xComSendString( COM_GSM, pcStateName );
	xComSendString( COM_GSM, "}\x1a\r\n" );	
	
	/* Wait for the OK message from the GSM/GPS module. */
	xAtResult = xReceiveResponse( AT_SEND_OK, AT_NOMSG, AT_SEND_FAIL, TO_GSM_TCPWR );
	if ( xAtResult != AT_SUCCESS )
	{ 
		V_TRACE_PRINT( TRACE_GSM_SEND_MODULE_ERR, TRACE_UART_AND_FILE );

		/* Received no positive response from the module. Return the error. */
		return xAtResult;
	}

	/* Return the response from the module. */
	return AT_SUCCESS;
}
/*-----------------------------------------------------------*/

/* Send device statistics to server. 
   The result is returned in form of: '{STATS: 0x1234=0x5678,0xABCD}'		*/
enum xAT_RESULT prvSendStatsBytes( void )
{
	enum xAT_RESULT					xAtResult;
	unsigned portBASE_TYPE			uxIdx;
	signed char						cHexStrg[5];

	vSystemStateStats( SYSSTATS_UPDATEDWELL, 0 );				
	if ( bNoAbnActive )
	{
		vSystemNoAbnStats( SYSSTATS_NOABN_UPDATEDWELL );	
	}
	
	/* Give the TCP send (socket write) command. */
	if ( ( xAtResult = prvPrepareTCPWrite() ) != AT_SUCCESS )
	{
		return xAtResult;
	}
	
	/* Send the start of the data string. */
	xComSendString( COM_GSM, "{STATS: 0x" );
	
	/* As first parameter, send the RTC. */
	vShortToHexStrg( cHexStrg, usReadRTC() );
	xComSendString( COM_GSM, cHexStrg );	

	/* Send the actual packet. */
	for ( uxIdx = 0; uxIdx < XSTATS_ELEMENTS; uxIdx++ )
	{
		/* Add separator and header of next parameter. */		
		xComSendString( COM_GSM, ",0x" );
		
		/* Send the value as HEX-ASCII. */
		vShortToHexStrg( cHexStrg, uStats.usStats[uxIdx] );
		xComSendString( COM_GSM, cHexStrg );	
	}

	/* Add separator and header of next parameter. */
	xComSendString( COM_GSM, ",0x" );
	
	/* Send the software reset count as HEX-ASCII. */
	vLongToHexStrg( cHexStrg, uxErrorRecord.ulSWResetCount );
	xComSendString( COM_GSM, cHexStrg );	

	/* Add separator and header of next parameter. */
	xComSendString( COM_GSM, ",0x" );
	
	/* Send the watchdog reset count as HEX-ASCII. */
	vLongToHexStrg( cHexStrg, uxErrorRecord.ulWDResetCount );
	xComSendString( COM_GSM, cHexStrg );	

	/* Add separator and header of next parameter. */
	xComSendString( COM_GSM, ",0x" );
	
	/* Send the power-on reset count as HEX-ASCII. */
	vLongToHexStrg( cHexStrg, uxErrorRecord.ulPOResetCount );
	xComSendString( COM_GSM, cHexStrg );	

	/* Add separator and header of next parameter. */
	xComSendString( COM_GSM, ",0x" );
	
	/* Send the other reset count as HEX-ASCII. */
	vLongToHexStrg( cHexStrg, uxErrorRecord.ulOtherResetCount );
	xComSendString( COM_GSM, cHexStrg );	
	
	/* Add separator and header of next parameter. */
	xComSendString( COM_GSM, ",0x" );
	
	/* Send the stack overflow count as HEX-ASCII. */
	vLongToHexStrg( cHexStrg, uxErrorRecord.ulStackOverflowCount );
	xComSendString( COM_GSM, cHexStrg );	

	/* Add separator and header of next parameter. */
	xComSendString( COM_GSM, ",0x" );
	
	/* Send the RTOS system error count as HEX-ASCII. */
	vLongToHexStrg( cHexStrg, uxErrorRecord.ulSystemErrorCount );
	xComSendString( COM_GSM, cHexStrg );	

	/* Add separator and header of next parameter. */
	xComSendString( COM_GSM, ",0x" );
	
	/* Send the SoftDevice assertion failed count as HEX-ASCII. */
	vLongToHexStrg( cHexStrg, uxErrorRecord.ulSDAssertCount );
	xComSendString( COM_GSM, cHexStrg );	

	/* Add separator and header of next parameter. */
	xComSendString( COM_GSM, ",0x" );
	
	/* Send the SoftDevice invalid memory access count as HEX-ASCII. */
	vLongToHexStrg( cHexStrg, uxErrorRecord.ulSDMemAccCount );
	xComSendString( COM_GSM, cHexStrg );	

	/* Add separator and header of next parameter. */
	xComSendString( COM_GSM, ",0x" );
	
	/* Send the nRF SDK assertion failed count as HEX-ASCII. */
	vLongToHexStrg( cHexStrg, uxErrorRecord.ulnRFSDKAssertCount );
	xComSendString( COM_GSM, cHexStrg );	

	/* Add separator and header of next parameter. */
	xComSendString( COM_GSM, ",0x" );
	
	/* Send the nRF SDK error count as HEX-ASCII. */
	vLongToHexStrg( cHexStrg, uxErrorRecord.ulnRFSDKErrorCount );
	xComSendString( COM_GSM, cHexStrg );	

	/* Add separator and header of next parameter. */
	xComSendString( COM_GSM, ",0x" );
	
	/* Send the nRF unknown error count count as HEX-ASCII. */
	vLongToHexStrg( cHexStrg, uxErrorRecord.ulnRFUnknownErrorCount );
	xComSendString( COM_GSM, cHexStrg );	

	/* Send the trailer. */
	xComSendString( COM_GSM, "}\x1a\r\n" );	
	
	/* Wait for the OK message from the GSM/GPS module. */
	xAtResult = xReceiveResponse( AT_SEND_OK, AT_NOMSG, AT_SEND_FAIL, TO_GSM_TCPWR );
	if ( xAtResult != AT_SUCCESS )
	{ 
		V_TRACE_PRINT( TRACE_GSM_SEND_MODULE_ERR, TRACE_UART_AND_FILE );

		/* Received no positive response from the module. Return the error. */
		return xAtResult;
	}

	/* Return the response from the module. */
	return AT_SUCCESS;
}
/*-----------------------------------------------------------*/\

/* Vibrate for the requested duration on server command and return an 'OK'.
   The command parameters are:
		<ON-DURATION>[,<OFF-DURATION>[,<REPETITIONS>]]
	
   with: 
		<ON-DURATION>  (byte) the vibrator on duration in 100mseconds 
		<OFF-DURATION> (byte) the vibrator off duration in 100mseconds, optional
		<REPETITIONS>  (byte) the number of times the pattern is to repeat, optional
*/
enum xAT_RESULT prvServerReqVibrate( signed char *pcParamValue )
{
	enum xAT_RESULT			xAtResult;
	unsigned portBASE_TYPE	uxOnDuration;
	unsigned portBASE_TYPE	uxOffDuration;
	unsigned portBASE_TYPE	uxRepCnt;
	unsigned portBASE_TYPE	uxOkRepCnt;
	
	/* Obtain the ON-DURATION command parameter and convert it to OS ticks. */
	uxOnDuration = ucHexStrgToByte( pcParamValue );
	
	/* Check, if the OFF-DURATION command parameter is given and, if yes, read it. */
	if ( pcParamValue[ 2 ] == ',' )
	{
		uxOffDuration = ucHexStrgToByte( pcParamValue + 5 );
	}
	else
	{
		uxOffDuration = 1;
	}
	
	/* Check, if the REPETITIONS command parameter is given and, if yes, read it. */
	if ( pcParamValue[ 7 ] == ',' )
	{
		uxRepCnt = ucHexStrgToByte( pcParamValue + 10 );
	}
	else
	{
		uxRepCnt = 1;
	}
	
	/* Check parameter range. */
	if ( ( uxOnDuration > MAX_VIBR_DUR ) || ( uxOffDuration > MAX_VIBR_DUR ) )
	{
		/* Return an error. */
		xAtResult = prvSendRawPacket( "KO-VIBR_TOO_LONG", NULL, NULL );
	}
	else
	{
		/* Execute the vibration. */
		vCustomVibrate( uxOnDuration, uxOffDuration, uxRepCnt );
		
		/* Try to send the acknowledgement packet for at most 5 times. While the vibrator is
		   on, the GSM module does not seem to receive and thus may lose the connection. 
		   Re-attempting and adding delay might allow it to re-establish. */
		uxOkRepCnt = 0;
		do
		{
			/* Check GPRS / TCP status before sending OK. */
			bTestAndReActivatePDP();
			( void )xTestAndReConnectTCP();
			xAtResult = prvSendRawPacket( "OK", NULL, NULL );
			if ( xAtResult != AT_SUCCESS )
			{
					vTaskDelay( 3 * portTICKS_PER_SEC );
			}

			uxOkRepCnt++;
		}
		while ( ( xAtResult != AT_SUCCESS ) && ( uxOkRepCnt < 5 ) );
	}
	
	return xAtResult;
}
/*-----------------------------------------------------------*/

/* Switch on the LED(s) for the requested duration on server command and return an 'OK'. 
   The command parameters are:
		<DURATION>,<PATTERN>
	
   with: 
		<DURATION> (short) the LED on duration in seconds 
		<PATTERN> (byte) the definition which LEDs to switch on with:
					bit2 = LED blue
					bit1 = LED green
					bit0 = LED red
   An LED defined OFF might still blink if requested by another process (especially red and blue).   
*/
enum xAT_RESULT prvServerReqLed( signed char *pcParamValue )
{
	enum xAT_RESULT			xAtResult;
	unsigned long 			ulDuration;
	unsigned portBASE_TYPE	uxLedPattern;
	
	/* Obtain the command parameter and convert it to OS ticks. */
	ulDuration = usHexStrgToShort( pcParamValue );
	if ( strlen( pcParamValue ) > 7 )
	{
		uxLedPattern = ucHexStrgToByte( pcParamValue + 7 );
	}
	else
	{
		uxLedPattern = 0x01;
	}
	
	if ( ulDuration > MAX_LED_DUR )
	{
		/* Return an error. */
		xAtResult = prvSendRawPacket( "KO-LED_TOO_LONG", NULL, NULL );
	}
	else
	{
		/* Convert the parameter to OS ticks. */
		ulDuration *= portTICKS_PER_SEC;
	
		/* Request switching on the LEDs. */
		if ( uxLedPattern & 0x01 )
		{
			vSwitchOnLedRed ( ledRED_GSM_REQ );
		}
		if ( uxLedPattern & 0x02 )
		{
			vSwitchOnLedGreen ( ledGREEN_GSM_REQ );
		}
		if ( uxLedPattern & 0x04 )
		{
			vSwitchOnLedBlue ( ledBLUE_GSM_REQ );
		}
		
		/* Keep it on for the requested duration. */
		vTaskDelay( ulDuration );
		
		/* Switch off the LED. */
		vSwitchOffLedRed( ledRED_GSM_REQ );
		vSwitchOffLedGreen( ledGREEN_GSM_REQ );
		vSwitchOffLedBlue( ledBLUE_GSM_REQ );
		
		/* Send the acknowledgement packet. */
		xAtResult = prvSendRawPacket( "OK", NULL, NULL );
	}
	
	return xAtResult;
}
/*-----------------------------------------------------------*/

/* Return the vibration sequence for evacuation alert. The parameters are:
		<ON-DURATION>,<OFF-DURATION>,<REPETITIONS>										   
   with: 
		<ON-DURATION>  (byte) the vibrator on duration in 100 ms steps 
		<OFF-DURATION> (byte) the vibrator off duration in 100 ms steps
		<REPETITIONS>  (byte) the number of times the pattern is to repeat	
*/
enum xAT_RESULT prvSendEvacVibrParameters( void )
{
	signed char		cStrg[ 13 ];
	unsigned char	ucParameter;
	enum xAT_RESULT	xAtResult;
	
	/* Prepare parameter strings for response. */	
	ucParameter = ucConfigReadByte( &xNvdsConfig.ucEvacVibrOn );
	vByteToHexStrg( cStrg + 0, ucParameter );
	cStrg[ 2 ] = ',';
	cStrg[ 3 ] = '0';
	cStrg[ 4 ] = 'x';
	ucParameter = ucConfigReadByte( &xNvdsConfig.ucEvacVibrOff );
	vByteToHexStrg( cStrg + 5, ucParameter );
	cStrg[ 7 ] = ',';
	cStrg[ 8 ] = '0';
	cStrg[ 9 ] = 'x';
	ucParameter = ucConfigReadByte( &xNvdsConfig.ucEvacVibrRep );
	vByteToHexStrg( cStrg + 10, ucParameter );
	cStrg[ 12 ] = 0;

	/* Send the packet. */
	xAtResult = prvSendRawPacket( pcAt_SrvWrEvacVibr, cStrg, NULL );
	
	return xAtResult;
}
/*-----------------------------------------------------------*/

/* Parse the arguments of FTP commands and extract source filename, UID, PW and optionally the target filename for the 
   following FTP transfer.
   Format:	   
		FS_PUT=<source filename>,<uid>,<pw>[,<target filename>]
*/
void prvParseFTPParams( void )
{
	signed char			*pcFieldStartPtr;
	signed char			*pcFieldEndPtr;
	
	/* Copy source filename. Convert single quotes to double quotes. */
	pcFieldEndPtr = pcFindNComma( cServerCmdParams, 1, FTP_MAX_FILNAME );
	*( pcFieldEndPtr - 1 ) = 0;
	strncpy( cFTPSourceFileName, cServerCmdParams, FTP_MAX_FILNAME + 1 );
	vTranslateSingleQuote( cFTPSourceFileName );
	
	/* Copy UID. Convert single quotes to double quotes. */
	pcFieldStartPtr = pcFieldEndPtr;
	pcFieldEndPtr = pcFindNComma( pcFieldStartPtr, 1, FTP_MAX_UID );
	*( pcFieldEndPtr - 1 ) = 0;
	strncpy( cFTPUid, pcFieldStartPtr, FTP_MAX_UID + 1 );
	vTranslateSingleQuote( cFTPUid );

	/* Copy PW. Convert single quotes to double quotes. */
	pcFieldStartPtr = pcFieldEndPtr;
	pcFieldEndPtr = pcFindNComma( pcFieldStartPtr, 1, FTP_MAX_PW );
	if ( *( pcFieldEndPtr - 1 ) != ',' )
	{
		/* No further field was found. Patch a string end '0' at the place of the last character which is a '}'. */
		*( pcFieldEndPtr - 1 ) = 0;
		strncpy( cFTPPw, pcFieldStartPtr, FTP_MAX_PW + 1 );
		vTranslateSingleQuote( cFTPPw );
	}
	else
	{
		/* There is an additional field, use it as target filename. */
		*( pcFieldEndPtr - 1 ) = 0;
		strncpy( cFTPPw, pcFieldStartPtr, FTP_MAX_PW + 1 );
		vTranslateSingleQuote( cFTPPw );

		/* Patch a string end '0' at the place of the last character which is a '}'. */
		*( pcFieldEndPtr + strlen( pcFieldEndPtr ) - 1 ) = 0;
		strncpy( cFTPTargetFileName, pcFieldEndPtr, FTP_MAX_FILNAME + 1 );
		vTranslateSingleQuote( cFTPTargetFileName );		
	}
}
/*-----------------------------------------------------------*/

/* Parse the arguments of the RELAY_CMD command and extract destination IMEI, command index and optional command 
   parameter.
   Format:	   
		RELAY_CMD=<imei>,<cmd>,<cmd param> 

   Examples:
		RELAY_CMD=358578080179290,ALERT_ACK
		RELAY_CMD=358578080179290,LED,0x0107
*/
void prvParseRelayCmdParams( void )
{
	enum xBLE_CMD				xBleCmd;
	unsigned portBASE_TYPE		uxIdx;
	signed char 				*pcCmdStrg;
	
	/* Capture the command desination address (IMEI). */
	cRelayCmdDestination[ 0 ] = cCharToNibble( cServerCmdParams[ 0 ] );
	for ( uxIdx = 1; uxIdx < 8; uxIdx++ )
	{
		cRelayCmdDestination[ uxIdx ] =   16 * cCharToNibble( cServerCmdParams[ 2 * uxIdx - 1 ] )
										+      cCharToNibble( cServerCmdParams[ 2 * uxIdx     ] );
	}

	/* Capture the command index. */
	if ( cServerCmdParams[ 15 ] != ',' )
	{
		( void )prvSendRawPacket( "KO-SYNTAX_ERROR", NULL, NULL );
		return;
	}
	
	pcCmdStrg = cServerCmdParams + 16;
	uxRelayCmdIdx = xParseBleCmd( &pcCmdStrg );
	
	if ( uxRelayCmdIdx != AT_NOMSG )
	{
		/* If present, capture the command parameter. */
		if ( ( *pcCmdStrg == ',' ) || ( *pcCmdStrg == '=' ) )
		{
			bRelayCmdParamPresent = true;
			/* Skip over the ',0x'. */
			usRelayCmdParam = usHexStrgToShort( pcCmdStrg + 3 );
		}
		else
		{
			bRelayCmdParamPresent = false;
		}
		
		/* Send the command to relay to the BLE process. */
		xBleCmd = BLE_RELAY_CMD;
		xQueueSend( xBleCmdQueue, &xBleCmd, 0 );
		
		/* Send the acknowledgement packet. */
		( void )prvSendRawPacket( "OK", NULL, NULL );											
	}
	else
	{
		( void )prvSendRawPacket( "KO-COMMAND_SYNTAX_ERROR", NULL, NULL );													
	}
}
/*-----------------------------------------------------------*/

/* Initiate a Firmware update. */
void prvFirmwareUpdate( void )
{
	ret_code_t		xErrCode;
	unsigned char	ucUpdateMode;
	
	/* Check power conditions before continuing. */
	if ( !bGetOnCharger() )
	{
		( void )prvSendRawPacket( "KO-FWUPD_NOT_ON_CHARGER", NULL, NULL );
		return;
	}
	/* At this point, we are sure that both BLE and GPS are deactivated as the module is on the charger. */
	
	/* Read the battery voltage from the ADC (ADC1, Vref = 1.1V band gap) */
	if ( sGetBattVoltage() < FWUPD_BATT_THRES )
	{
		( void )prvSendRawPacket( "KO-FWUPD_BATT_NOT_FULL", NULL, NULL );
		return;
	}
	
	/* Parse the arguments of the FWUPD command and extract filename, UID, PW and CRC for the 
	   following FTP transfer. The optional <mode> parameter gets copied into cFTTargetFileName.
	   Format:	   
			FWUPD=[<filename>],[<uid>],[<pw>][,mode] */
	cFTPTargetFileName[ 0 ] = 0;
	prvParseFTPParams();
	
	/* Fill in the mode. */
	if ( cFTPTargetFileName[ 0 ] == 0 )
	{
		/* Default case: Application firmware update. */
		ucUpdateMode = BL_UPDATE_FW;
	}
	else
	{
		/* Get the mode from the string parameter. */
		ucUpdateMode = ucIntStrgToByte( cFTPTargetFileName, 2 );
	}

	/* Send a packet telling the server that the request has been taken into account. */
	( void )prvSendRawPacket( "FWUPD_ONGOING", NULL, NULL );
	
	if ( ( ucUpdateMode & BL_UPDATE_FW ) && ( cFTPSourceFileName[ 0 ] != 0 ) )
	{
		/* Fetch a new application firmware file. */
		
		/* Delete any old fwupd.scr file. */
		( void )xSendAtCommand( &xAtDelFwUpd, true, true );
	
		/* Configure and establish the FTP connection, download the file and log out. */
		{
			enum xAT_RESULT	xSuccess;
			
			{
				signed char		cErrCode[ 40 ];
				signed char		*pcStrg;
				bool			bFtpLoggedIn;
				
				bFtpLoggedIn = false;
				
				xSuccess = xSendBatchAtCommand( xAtFtpLogin, sizeof( xAtFtpLogin ) / sizeof( struct xAT_CMD ), true );
				if ( xSuccess == AT_SUCCESS )
				{
					/* Delay required between FTP login and any remote operations. Seems to be an undocumented Quectel requirement.
					   If not respected, Network Error 605 is returned. */
					vTaskDelay( T_FTP_LOGIN );

					/* Configure and establish the FTP connection, download the file and log out. */
					pcFTPSourceFileName = cFTPSourceFileName;
					pcFTPTargetFileName = "\"fwupd.scr\"";
					xSuccess = xSendBatchAtCommand( xAtFwUpLoad, sizeof( xAtFwUpLoad ) / sizeof( struct xAT_CMD ), true );
					if ( xSuccess != AT_SUCCESS )
					{
						/* Save the first error code. */
						pcStrg = stpncpy( ( char * )cErrCode, ( char * )cServerCmdParams, 39 );
						*( pcStrg++ ) = ',';
						*pcStrg = 0;

						bFtpLoggedIn = true;
					}
				}

				/* Retrieve the error code. The code is stored in cServerCmdParams. */
				if ( xSuccess != AT_SUCCESS )
				{
					if ( xSendAtCommand( &xAtFtpErr, true, true ) == AT_SUCCESS )
					{
						unsigned portBASE_TYPE		ucFirstErrLen;
						
						/* Append the second error code. */
						ucFirstErrLen = strlen( cErrCode );
						strncpy( ( char * )pcStrg, ( char * )cServerCmdParams, 40 - ucFirstErrLen - 1 );

						( void )prvSendRawPacket( "KO-FWUPD_FTP_ERROR:", cErrCode, NULL );
					}
					else
					{
						( void )prvSendRawPacket( "KO-FWUPD_FTP_UNSPECIFIED_ERROR", NULL, NULL );
					}
				}
					
				/* Close the FTP connection, if it existed. */
				if ( bFtpLoggedIn )
				{
					( void )xSendAtCommand( &xAtUFtpLogout, true, true );
				}

				/* If the FTP was not successful, return without rebooting. */
				if ( xSuccess != AT_SUCCESS )
				{
					return;
				}
			}
		}
	}
	
	/* Shut down the GSM module. It gets rebooted in the bootloader with the parameters required then. */
	vGSMModulePowerDown( SOFT_POWER_OFF );	
	
	/* Set FW update flags in the GPRETREG registers. */
	#if defined ( NO_SOFTDEVICE )
		nrf_power_gpregret_set( BOOTLOADER_DFU_START | ucUpdateMode );
		nrf_power_gpregret2_set( 0 );
	#else
		xErrCode = sd_power_gpregret_set( 0, BOOTLOADER_DFU_START | ucUpdateMode );
		APP_ERROR_CHECK( xErrCode );
		xErrCode = sd_power_gpregret_set( 1, 0 );
		APP_ERROR_CHECK( xErrCode );
	#endif
	
	/* Invalidate the NVDS. */
	vInvalidateConfig();
	
	/* Reboot in bootloader. */
	V_SYSTEM_RESET( RESET_FOR_FW_UPDATE );
}
/*-----------------------------------------------------------*/

/* Return the listing of the local file system to the server. */
void prvFileSystemListing( void )
{
	signed char 			cFreeSpaceStrg[ 40 ];
	unsigned portBASE_TYPE	uxStrgIdx;
	
	/* Handle file listing queries. 
	   '0' is the generic file listing. '2' is the size of a specific file. Here, both are mapped to the 
	    same query and the file specification in the command is used as name pattern. */
	if ( ( cServerCmdParams[ 0 ] == '0' ) || ( cServerCmdParams[ 0 ] == '2' ) )
	{
		if ( xSendAtCommand( &xAtLstFile, true, true ) == AT_SUCCESS )
		{
			( void )prvSendRawPacket( "FS_LST=", cServerCmdParams, NULL );
		}
		else
		{
			( void )prvSendRawPacket( "KO-ERROR_READING_FS", NULL, NULL );
		}
	}

	/* Handle free file system space queries. */
	if ( cServerCmdParams[ 0 ] == '1' )
	{
		if ( xSendAtCommand( &xAtLstFreeU, true, true ) == AT_SUCCESS )
		{
			strncpy( ( char * )cFreeSpaceStrg, ( char * )cServerCmdParams, 20 );
			
			if ( xSendAtCommand( &xAtLstFreeE, true, true ) == AT_SUCCESS )
			{
				uxStrgIdx = strlen( cFreeSpaceStrg );
				strcpy( cFreeSpaceStrg + uxStrgIdx, ",EUFS:" );
				strncpy( ( char * )( cFreeSpaceStrg + uxStrgIdx + 6 ), ( char * )( cServerCmdParams ), 40 - uxStrgIdx - 6 );
				( void )prvSendRawPacket( "FS_LST=UFS:", cFreeSpaceStrg, NULL );
			}
		}
		else
		{
			( void )prvSendRawPacket( "KO-ERROR_READING_FS", NULL, NULL );
		}
	}
}
/*-----------------------------------------------------------*/

/* Download file from local file system to FTP server (get). */
void prvFileSystemFileGet( void )
{
	signed char		cErrCode[ 40 ];
	signed char		*pcStrg;
	bool			bFtpLoggedIn;
	
	bFtpLoggedIn = false;
	cErrCode[ 0 ] = 0;
	pcStrg = cErrCode;

	/* Parse the arguments of the FWUPD command and extract filename, UID, PW and CRC for the 
	   following FTP transfer.
	   Format:	   
			FS_GET=<source filename>,<uid>,<pw>[,<target filename>] */
	cFTPTargetFileName[ 0 ] = 0;
	prvParseFTPParams();
	
	/* Configure and establish the FTP connection, download the file and log out. */
	if ( xSendBatchAtCommand( xAtFtpLogin, sizeof( xAtFtpLogin ) / sizeof( struct xAT_CMD ), true ) == AT_SUCCESS )
	{
		bFtpLoggedIn = true;
		
		/* Delay required between FTP login and any remote operations. Seems to be an undocumented Quectel requirement.
		   If not respected, Network Error 605 is returned. */
		vTaskDelay( T_FTP_LOGIN );

		/* Configure and establish the FTP connection, download the file and log out. */
		pcFTPSourceFileName = cFTPSourceFileName;
		if ( cFTPTargetFileName[ 0 ] == 0 )
		{
			pcFTPTargetFileName = cFTPSourceFileName;
		}
		else
		{
			pcFTPTargetFileName = cFTPTargetFileName;
		}
			
		if ( xSendBatchAtCommand( xAtFsGet, sizeof( xAtFsGet ) / sizeof( struct xAT_CMD ), true ) == AT_SUCCESS )
		{
			( void )prvSendRawPacket( "OK", NULL, NULL );
			
			return;
		}
		
		/* Save the first error code. */
		pcStrg = stpncpy( ( char * )cErrCode, ( char * )cServerCmdParams, 39 );
		*( pcStrg++ ) = ',';
		*pcStrg = 0;
	}

	/* Retrieve the error code. The code is stored in cServerCmdParams. */
	if ( xSendAtCommand( &xAtFtpErr, true, true ) == AT_SUCCESS )
	{
		unsigned portBASE_TYPE		ucFirstErrLen;

		/* Append the second error code. */
		ucFirstErrLen = strlen( cErrCode );
		strncpy( ( char * )pcStrg, ( char * )cServerCmdParams, 40 - ucFirstErrLen - 1 );

		( void )prvSendRawPacket( "KO-FTP_ERROR:", cErrCode, NULL );
	}
	else
	{
		( void )prvSendRawPacket( "KO-FTP_UNSPECIFIED_ERROR", NULL, NULL );
	}
	/* Close the FTP connection, if it existed. */
	if ( bFtpLoggedIn )
	{
		( void )xSendAtCommand( &xAtUFtpLogout, true, true );
	}

	return;
}
/*-----------------------------------------------------------*/

/* Upload file from FTP server to local file system. */
void prvFileSystemFilePut( void )
{
	signed char		cErrCode[ 40 ];
	signed char		*pcStrg;
	bool			bFtpLoggedIn;
	
	bFtpLoggedIn = false;
	cErrCode[ 0 ] = 0;
	pcStrg = cErrCode;
	
	/* Parse the arguments of the FWUPD command and extract filename, UID, PW and CRC for the 
	   following FTP transfer.
	   Format:	   
			FTP_PUT=<source filename>,<uid>,<pw>[,<target filename>] */
	cFTPTargetFileName[ 0 ] = 0;
	prvParseFTPParams();
	
	if ( xSendBatchAtCommand( xAtFtpLogin, sizeof( xAtFtpLogin ) / sizeof( struct xAT_CMD ), true ) == AT_SUCCESS )
	{
		bFtpLoggedIn = true;
		
		/* Delay required between FTP login and any remote operations. Seems to be an undocumented Quectel requirement.
		   If not respected, Network Error 605 is returned. */
		vTaskDelay( T_FTP_LOGIN );

		/* Configure and establish the FTP connection, download the file and log out. */
		pcFTPSourceFileName = cFTPSourceFileName;
		if ( cFTPTargetFileName[ 0 ] == 0 )
		{
			pcFTPTargetFileName = cFTPSourceFileName;
		}
		else
		{
			pcFTPTargetFileName = cFTPTargetFileName;
		}
		if ( xSendBatchAtCommand( xAtFsPut, sizeof( xAtFsPut ) / sizeof( struct xAT_CMD ), true ) == AT_SUCCESS )
		{
			( void )prvSendRawPacket( "OK", NULL, NULL );
			
			return;
		}
		
		/* Save the first error code. */
		pcStrg = stpncpy( ( char * )cErrCode, ( char * )cServerCmdParams, 39 );
		*( pcStrg++ ) = ',';
		*pcStrg = 0;
	}

	/* Retrieve the error code. The code is stored in cServerCmdParams. */
	if ( xSendAtCommand( &xAtFtpErr, true, true ) == AT_SUCCESS )
	{
		unsigned portBASE_TYPE		ucFirstErrLen;
		
		/* Append the second error code. */
		ucFirstErrLen = strlen( cErrCode );
		strncpy( ( char * )pcStrg, ( char * )cServerCmdParams, 40 - ucFirstErrLen - 1 );

		( void )prvSendRawPacket( "KO-FTP_ERROR:", cErrCode, NULL );
	}
	else
	{
		( void )prvSendRawPacket( "KO-FTP_UNSPECIFIED_ERROR", NULL, NULL );
	}
	/* Close the FTP connection, if it existed. */
	if ( bFtpLoggedIn )
	{
		( void )xSendAtCommand( &xAtUFtpLogout, true, true );
	}

	return;
}
/*-----------------------------------------------------------*/

/* Delete file from local file system. */
void prvFileSystemFileDelete( void )
{
	if ( xSendAtCommand( &xAtDelFile, true, true ) == AT_SUCCESS )
	{
		( void )prvSendRawPacket( "OK", NULL, NULL );
	}
	else
	{
		( void )prvSendRawPacket( "KO-ERROR_ACCESSING_FS", NULL, NULL );
	}
}
/*-----------------------------------------------------------*/

/* Send an event based on a server command to the control state machine. */
enum xAT_RESULT prvSendEventToCtrl( enum xCTRL_EVENT xCtrlEvent )
{
	if ( xQueueSend( xCtrlEventQueue, &xCtrlEvent, ctrlEVENT_BLOCKTIME ) == pdPASS )
	{
		/* Send the acknowledgement packet. */
		( void )prvSendRawPacket( "OK", NULL, NULL );
		return AT_SUCCESS;
	}
	else
	{
		/* Error case: the event could not be sent to the CTRL process. */
		( void )prvSendRawPacket( "KO-CTRL_DID_NOT_ACCEPT_CMD", NULL, NULL );
		return AT_RESPONSE_TO;			/* Not quite the right response as here the control task produces a TO... */
	}
}
/*-----------------------------------------------------------*/

/* Pull a server command and treat it. */
bool bTreatServerCmd( void )
{
	enum xAT_MSG_ID			xServerCmd;
	enum xBLE_CMD			xBleCmd;
	enum xCTRL_EVENT		xCtrlEvent;
	enum xGSM_CMD			xGsmCmd;
	unsigned short			usRssiParameter;
	unsigned portBASE_TYPE	uxRssiFilterMethod;
	unsigned portBASE_TYPE	uxRssiFilterWindow;
	unsigned long			ulAddress;
	bool					bGsmShutDownWasRequested;
	
	/* Depending on when the server command was received, there are different possibilities:
	   1. The Server command was received in between two transactions to send the position (default):
			xParserAtRespQueue:  AT_TCP_RX_DATA_IND
		  --> request a reception to obtain the server command.
		  
	   2. The Server command was received in between sending a position packet while waiting for the [OK]:
			xParserAtRespQueue:  SRVCMD_RD_VERSION ... SRVCMD_OTHER
		   --> trash the server command. This is done in prvSendPacket().
	   
	   3. The Server command was received attached to the [OK] packet in response to a position packet:
			xParserAtRespQueue:  SRVCMD_RD_VERSION ... SRVCMD_OTHER
		  --> skip the reception and directly process the server command. */
	   
	/* First, check if the AT data indication which has caused the server command 
	   indication, is still in the AT queue. If not, it means that the data indication
	   has been part of a command sequence and has already been treated elsewhere (e.g.
	   in prvSendBasePacket). 
	   In this case, simply exit the function. */
	if ( xQueueReceive( xParserAtRespQueue, &xServerCmd, 0 ) == errQUEUE_EMPTY )
	{
		return false;
	}
	
	/* Set flag to indicate that a server command transaction is ongoing. */
	bCommandTransactionOngoing = true;
	
	/* Cancel the module shut-down timer, should it be running. 
	   But remember that there was a shutdown request pending. */
	bGsmShutDownWasRequested = xTimerIsTimerActive( xModuleShutdownTimer );
	( void )xTimerStop( xModuleShutdownTimer, 0 );
	/* Remove any pending requests to shutdown the GSM/GPS module but also remember if there were any. 
	   That way, the shutdown request can be reevaluated and restored once all commands have been treated. */
	xGsmCmd = GSM_SHUTDOWN_MODULE;
	bGsmShutDownWasRequested |= bMsgTypeInQueue( xGsmCmdQueue, &xGsmCmd );
	vRemoveTypeFromQueue( xGsmCmdQueue, GSM_SHUTDOWN_MODULE );
	
	/* Make sure the module does not enter idle PSM. */
	xModuleDisableSleepMode();
	
	do
	{
		/* If we have just got a data indication, we now need to pull in the actual command string - unless this has already been done 
		   and we can skip this phase.
		   In the other case, the command had been attached to an OK message and is thus already in the command queue. */
		if ( xServerCmd == AT_TCP_RX_DATA_IND )
		{
			/* Now that we have established that the GSM base station still has data for us, send receive command. 
			   The parser will send an indication corresponding to the server command in the AT command queue. */
			if ( xModuleState & GSM_TCP_CONN )
			{
				/* Clear TCP socket. */
				( void )xSendAtCommand( &xAtQIRecv, true, true );
			}
			else
			{
				/* TLS socket. */
				( void )xSendAtCommand( &xAtQSSLRecv, true, true );
			}
		
			/* Then we have to wait for the actual server command to be delivered. If we time out, we just return
			   and let the server retry. */
			if ( xQueueReceive( xParserAtRespQueue, &xServerCmd, TO_GSM_SRV_ACK ) == errQUEUE_EMPTY )
			{
				/* Reset flag to indicate that a server command transaction is finished. */
				bCommandTransactionOngoing = false;
				
				return false;
			}
			
			/* The parser sent another server command message to the command queue - in case the server command was piggy-backed
			   onto an 'OK' response. Here we are already treating the command, so remove it from the queue. */
			vRemoveTypeFromQueue( xGsmCmdQueue, GSM_SERVER_CMD );
		}
		
		/* Here is a safety patch for the GSM module operation:
		   It might be that the GSM module crashed after basic initialisation w/o the this FW detecting it. This leads to the AT echo be 
		   put on again. If the FW now reponds with '{<any reponse}' to the server, the reponds is echoed back locally and is treated as
		   command from the server. As the string is not a valid command, the module reponds with 'KO-SYNTAX_ERROR' which is also echoed
		   back and responded to - ad infinitum. 
		   So we send a precautionary ATE0 here - just in case. */
		( void )xSendAtCommand( &xAtE0, true, true );
	
		/* Got an AT indication. Treat it, if it is a server command. */
		switch( xServerCmd )
		{
			case SRVCMD_RD_VERSION:			/* Read Version information. */
											( void )prvSendVersionInformation();
											break;
			case SRVCMD_RESET:				/* Reset device. */
											{
												( void )prvSendRawPacket( "OK", NULL, NULL );
												/* Wait for a short while (e.g. 3sec) to let the response go through. */
												vTaskDelay( 300 );
												/* Shut down the GSM/GSP module. */
												vGSMModulePowerDown( SOFT_POWER_OFF );
												/* Execute reset. */
												V_SYSTEM_RESET( RESET_FROM_COMMAND );
											}
											break;
			case SRVCMD_RD_GPRSCFG:			/* Read/write GPRS connection configuration. */
											( void )prvSendRawPacket( "GPRSCFG=", NULL, ( signed char * )&xNvdsConfig.pcGsmGPRSConnCfg );
											break;
			case SRVCMD_WR_GPRSCFG:			cServerCmdParams[ strlen( cServerCmdParams ) - 1 ] = 0;		/* remove the residual curly bracket. */
											vConfigWriteString( xNvdsConfig.pcGsmGPRSConnCfg, cServerCmdParams );
											( void ) prvSendRawPacket( "OK", NULL, NULL );
											break;
			case SRVCMD_RD_TCPCFG:			/* Read/write TCP connection configuration. */
											( void )prvSendRawPacket( pcAt_SrvWrTcpCfg, NULL, ( signed char * )&xNvdsConfig.pcGsmTCPConnCfg );
											break;
			case SRVCMD_WR_TCPCFG:			cServerCmdParams[ strlen( cServerCmdParams ) - 1 ] = 0;		/* remove the residual curly bracket. */
											vConfigWriteString( xNvdsConfig.pcGsmTCPConnCfg, cServerCmdParams );
											( void ) prvSendRawPacket( "OK", NULL, NULL );
											break;
			case SRVCMD_RD_SECTCPCFG:		/* Read/write secure TCP connection configuration. */
											( void )prvSendRawPacket( pcAt_SrvWrSecTcpCfg, NULL, ( signed char * )&xNvdsConfig.pcGsmSecTCPConnCfg );
											break;
			case SRVCMD_WR_SECTCPCFG:		cServerCmdParams[ strlen( cServerCmdParams ) - 1 ] = 0;		/* remove the residual curly bracket. */
											vConfigWriteString( xNvdsConfig.pcGsmSecTCPConnCfg, cServerCmdParams );
											( void ) prvSendRawPacket( "OK", NULL, NULL );
											break;
			case SRVCMD_RD_DFMAP:			/* Read/write DFMAP. */
											( void )prvSendLongHexNvdsParameter( pcAt_SrvWrDFMap, &xNvdsConfig.ulDFMap );
											break;
			case SRVCMD_WR_DFMAP:			( void )prvWriteLongParameter( &xNvdsConfig.ulDFMap, cServerCmdParams, NVDS );
											/* Make sure that ID and IND are never disabled. */
											vConfigWriteLong( &xNvdsConfig.ulDFMap, ulConfigReadLong( &xNvdsConfig.ulDFMap ) | 0x3 );	
											break;
			case SRVCMD_RD_RAT:				/* Read/write Radio Access Technology. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrRat, &xNvdsConfig.usCfgRat );
											break;
			case SRVCMD_WR_RAT:				( void )prvWriteShortParameter( &xNvdsConfig.usCfgRat, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_ATT_TO:			/* Read/write time-out parameter for network attachment. */
											/* ulRegTimeOut is a long value and contains 10x the value which needs to be sent to the server. */
											( void )prvSendShortHexParameter( pcAt_SrvWrAttTo, ( unsigned short )( ulConfigReadLong( &xNvdsConfig.ulRegTimeOut ) / 10ul ) );
											break;
			case SRVCMD_WR_ATT_TO:			( void )prvWriteLongParameterx10( &xNvdsConfig.ulRegTimeOut, cServerCmdParams );
											break;

			case SRVCMD_RD_ACT_INT:			/* Read/write position update interval in active state. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrActInt, &xNvdsConfig.usActivePositionSendInterval );
											break;
			case SRVCMD_WR_ACT_INT:			( void )prvWriteShortParameter( &xNvdsConfig.usActivePositionSendInterval, cServerCmdParams, NVDS );
											xGsmCmd = GSM_POS_INT_UPDATE;	
											xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmCmd, 0 );		/* Notify position interval update. */
											break;
			case SRVCMD_RD_INACT_INT:		/* Read/write position update interval in inactive state. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrInactInt, &xNvdsConfig.usInactivePositionSendInterval );
											break;
			case SRVCMD_WR_INACT_INT:		( void )prvWriteShortParameter( &xNvdsConfig.usInactivePositionSendInterval, cServerCmdParams, NVDS );
											xGsmCmd = GSM_POS_INT_UPDATE;	
											xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmCmd, 0 );		/* Notify position interval update. */
											break;
			case SRVCMD_RD_STILL_INT:		/* Read/write position update interval in still state. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrStillInt, &xNvdsConfig.usStillPositionSendInterval );
											break;
			case SRVCMD_WR_STILL_INT:		( void )prvWriteShortParameter( &xNvdsConfig.usStillPositionSendInterval, cServerCmdParams, NVDS );
											xGsmCmd = GSM_POS_INT_UPDATE;	
											xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmCmd, 0 );		/* Notify position interval update. */
											break;
			case SRVCMD_RD_SLEEP_INT:		/* Read/write position update interval in sleep state. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrSleepInt, &xNvdsConfig.usSleepPositionSendInterval );
											break;
			case SRVCMD_WR_SLEEP_INT:		( void )prvWriteShortParameter( &xNvdsConfig.usSleepPositionSendInterval, cServerCmdParams, NVDS );
											xGsmCmd = GSM_POS_INT_UPDATE;	
											xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmCmd, 0 );		/* Notify position interval update. */
											break;
			case SRVCMD_RD_OOO_INT:			/* Read/write position update interval while out-of-office. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrOooInt, &xNvdsConfig.usOooPositionSendInterval );
											break;
			case SRVCMD_WR_OOO_INT:			( void )prvWriteShortParameter( &xNvdsConfig.usOooPositionSendInterval, cServerCmdParams, NVDS );
											xGsmCmd = GSM_POS_INT_UPDATE;	
											xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmCmd, 0 );		/* Notify position interval update. */
											break;
			case SRVCMD_RD_CHRG_INT:		/* Read/write position update interval while on charger. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrChrgInt, &xNvdsConfig.usChargingPositionSendInterval );
											break;
			case SRVCMD_WR_CHRG_INT:		( void )prvWriteShortParameter( &xNvdsConfig.usChargingPositionSendInterval, cServerCmdParams, NVDS );
											xGsmCmd = GSM_POS_INT_UPDATE;	
											xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmCmd, 0 );		/* Notify position interval update. */
											break;
			case SRVCMD_RD_ALERT_INT:		/* Read/write position update interval while in alert state. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrAlertInt, &xNvdsConfig.usAlertPositionSendInterval );
											break;
			case SRVCMD_WR_ALERT_INT:		( void )prvWriteShortParameter( &xNvdsConfig.usAlertPositionSendInterval, cServerCmdParams, NVDS );
											xGsmCmd = GSM_POS_INT_UPDATE;	
											xSendUniqueMsgToQueue( xGsmCmdQueue, &xGsmCmd, 0 );		/* Notify position interval update. */
											break;
			case SRVCMD_RD_PD_INT:			/* Read/write min. interval between position updates for module power-down. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrPwrDwnInt, &xNvdsConfig.usModulePwrDownInt );
											break;
			case SRVCMD_WR_PD_INT:			( void )prvWriteShortParameter( &xNvdsConfig.usModulePwrDownInt, cServerCmdParams, NVDS );
											break;
										
			case SRVCMD_RD_STEP_ACC_THR:	/* Read/write acceleration threshold for step detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrStepAccThr, &xNvdsConfig.usStepAccelerationThreshold );
											break;
			case SRVCMD_WR_STEP_ACC_THR:	( void )prvWriteShortParameter( &xNvdsConfig.usStepAccelerationThreshold, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_STEP_MIN_INT:	/* Read/write minimum interval for motion detection. */
											( void )prvSendShortHexNvdsParameterDiv10( pcAt_SrvWrStepMinInt, &xNvdsConfig.usStepMinInterval );
											break;
			case SRVCMD_WR_STEP_MIN_INT:	( void )prvWriteShortParameterx10( &xNvdsConfig.usStepMinInterval, cServerCmdParams );
											break;
			case SRVCMD_RD_STEP_MAX_INT:	/* Read/write maximum interval for step detection. */
											( void )prvSendShortHexNvdsParameterDiv10( pcAt_SrvWrStepMaxInt, &xNvdsConfig.usStepMaxInterval );
											break;
			case SRVCMD_WR_STEP_MAX_INT:	( void )prvWriteShortParameterx10( &xNvdsConfig.usStepMaxInterval, cServerCmdParams );
											break;
			case SRVCMD_RD_ACTIVE_NUM_STEPS:	/* Read/write number of steps for ACTIVE detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrActiveNumSteps, &xNvdsConfig.usActiveNumberSteps );
											break;
			case SRVCMD_WR_ACTIVE_NUM_STEPS:	( void )prvWriteShortParameter( &xNvdsConfig.usActiveNumberSteps, cServerCmdParams, NVDS );
											break;
									
			case SRVCMD_RD_SOS_ACC_THR:		/* Read/write acceleration threshold for SOS detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrSosAccThr, &xNvdsConfig.usSosAccelerationThreshold );
											break;
			case SRVCMD_WR_SOS_ACC_THR:		( void )prvWriteShortParameter( &xNvdsConfig.usSosAccelerationThreshold, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_SOS_MIN_X_ACC_THR: /* Read/write min. x-acceleration threshold for SOS detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrSosMinXAccThr, &xNvdsConfig.usSosMinXAccelerationThreshold );
											break;
			case SRVCMD_WR_SOS_MIN_X_ACC_THR: ( void )prvWriteShortParameter( &xNvdsConfig.usSosMinXAccelerationThreshold, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_SOS_MIN_Z_ACC_THR: /* Read/write min. z-acceleration threshold for SOS detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrSosMinZAccThr, &xNvdsConfig.usSosMinZAccelerationThreshold );
											break;
			case SRVCMD_WR_SOS_MIN_Z_ACC_THR: ( void )prvWriteShortParameter( &xNvdsConfig.usSosMinZAccelerationThreshold, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_SOS_MAX_Z_ACC_THR: /* Read/write max. z-acceleration threshold for SOS detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrSosMaxZAccThr, &xNvdsConfig.usSosMaxZAccelerationThreshold );
											break;
			case SRVCMD_WR_SOS_MAX_Z_ACC_THR: ( void )prvWriteShortParameter( &xNvdsConfig.usSosMaxZAccelerationThreshold, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_SOS_MIN_INT:		/* Read/write minimum interval for SOS detection. */
											( void )prvSendShortHexNvdsParameterDiv10( pcAt_SrvWrSosMinInt, &xNvdsConfig.usSosMinInterval );
											break;
			case SRVCMD_WR_SOS_MIN_INT:		( void )prvWriteShortParameterx10( &xNvdsConfig.usSosMinInterval, cServerCmdParams );
											break;
			case SRVCMD_RD_SOS_MAX_INT:		/* Read/write maximum interval for SOS detection. */
											( void )prvSendShortHexNvdsParameterDiv10( pcAt_SrvWrSosMaxInt, &xNvdsConfig.usSosMaxInterval );
											break;
			case SRVCMD_WR_SOS_MAX_INT:		( void )prvWriteShortParameterx10( &xNvdsConfig.usSosMaxInterval, cServerCmdParams );
											break;
			case SRVCMD_RD_SOS_NUM_PEAKS:	/* Read/write number of peaks for SOS detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrSosNumPeaks, &xNvdsConfig.usSosNumberPeaks );
											break;
			case SRVCMD_WR_SOS_NUM_PEAKS:	( void )prvWriteShortParameter( &xNvdsConfig.usSosNumberPeaks, cServerCmdParams, NVDS );
											break;
									
			case SRVCMD_RD_STILL_DUR:		/* Read/write duration for still detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrStillDur, &xNvdsConfig.usStillDuration );
											break;
			case SRVCMD_WR_STILL_DUR:		( void )prvWriteShortParameter( &xNvdsConfig.usStillDuration, cServerCmdParams, NVDS );
											break;						
			case SRVCMD_RD_IMMOBILITY_DUR:	/* Read/write duration for immobility detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrImmobilityDur, &xNvdsConfig.usImmobilityDuration );
											break;
			case SRVCMD_WR_IMMOBILITY_DUR:	( void )prvWriteShortParameter( &xNvdsConfig.usImmobilityDuration, cServerCmdParams, NVDS );
											break;	

			case SRVCMD_RD_SLEEP_DUR:		/* Read/write duration for sleep detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrSleepDur, &xNvdsConfig.usSleepDuration );
											break;
			case SRVCMD_WR_SLEEP_DUR:		( void )prvWriteShortParameter( &xNvdsConfig.usSleepDuration, cServerCmdParams, NVDS );
											break;
						
			case SRVCMD_RD_ABNPOS_Z_ACC_THR:	/* Read/write Z-acceleration threshold for abnormal position detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrAbnPosZAccThr, &xNvdsConfig.usAbnormalPositionZAccelerationThreshold );
											break;
			case SRVCMD_WR_ABNPOS_Z_ACC_THR:	( void )prvWriteShortParameter( &xNvdsConfig.usAbnormalPositionZAccelerationThreshold, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_ABNPOS_DUR:		/* Read/write duration for abnormal position detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrAbnPosDur, &xNvdsConfig.usAbnormalPositionDuration );
											break;
			case SRVCMD_WR_ABNPOS_DUR:		( void )prvWriteShortParameter( &xNvdsConfig.usAbnormalPositionDuration, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_ABNPOS_MAX_ORIENT:	/* Read/write acceleration threshold for tolerated spatial orientation change during abnormal position detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrAbnMaxOrient, &xNvdsConfig.usAbnormalMaxOrientationChg );
											break;
			case SRVCMD_WR_ABNPOS_MAX_ORIENT:	( void )prvWriteShortParameter( &xNvdsConfig.usAbnormalMaxOrientationChg, cServerCmdParams, NVDS );
											break;

			case SRVCMD_RD_ALERT_CNCL_METHOD:	/* Read/write method for alert cancel detection. */
											( void )prvSendByteHexNvdsParameter( pcAt_SrvWrAlertCnclMethod, &xNvdsConfig.ucAlertCancelMethod );
											break;
			case SRVCMD_WR_ALERT_CNCL_METHOD:	( void )prvWriteByteParameter( ( unsigned char * )&xNvdsConfig.ucAlertCancelMethod, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_ALERT_CNCL_THR:	/* Read/write acceleration threshold for alert cancel detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrAlertCnclThr, &xNvdsConfig.usAlertCancelThreshold );
											break;
			case SRVCMD_WR_ALERT_CNCL_THR:	( void )prvWriteShortParameter( &xNvdsConfig.usAlertCancelThreshold, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_ALERT_CNCL_MIN_INT:	/* Read/write minimum interval for alert cancel detection. */
											( void )prvSendShortHexNvdsParameterDiv10( pcAt_SrvWrAlertCnclMinInt, &xNvdsConfig.usAlertCancelMinInterval );
											break;
			case SRVCMD_WR_ALERT_CNCL_MIN_INT:	( void )prvWriteShortParameterx10( &xNvdsConfig.usAlertCancelMinInterval, cServerCmdParams );
											break;
			case SRVCMD_RD_ALERT_CNCL_MAX_INT:	/* Read/write maximum interval for alert cancel detection. */
											( void )prvSendShortHexNvdsParameterDiv10( pcAt_SrvWrAlertCnclMaxInt, &xNvdsConfig.usAlertCancelMaxInterval );
											break;
			case SRVCMD_WR_ALERT_CNCL_MAX_INT:	( void )prvWriteShortParameterx10( &xNvdsConfig.usAlertCancelMaxInterval, cServerCmdParams );
											break;
			case SRVCMD_RD_ALERT_CNCL_NUM_PEAKS:	/* Read/write number of peaks for alert cancel detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrAlertCnclNumPeaks, &xNvdsConfig.usAlertCancelNumberPeaks );
											break;
			case SRVCMD_WR_ALERT_CNCL_NUM_PEAKS:	( void )prvWriteShortParameter( &xNvdsConfig.usAlertCancelNumberPeaks, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_ALERT_CNCL_TIMEOUT:	/* Read/write timeout for alert cancel detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrAlertCnclTimeout, &xNvdsConfig.usAlertCancelTimeout );
											break;
			case SRVCMD_WR_ALERT_CNCL_TIMEOUT:	( void )prvWriteShortParameter( &xNvdsConfig.usAlertCancelTimeout, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_SOS_CNCL_TIMEOUT:/* Read/write timeout for SOS cancel detection. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrSosCnclTimeout, &xNvdsConfig.usSosCancelTimeout );
											break;
			case SRVCMD_WR_SOS_CNCL_TIMEOUT:( void )prvWriteShortParameter( &xNvdsConfig.usSosCancelTimeout, cServerCmdParams, NVDS );
											break;
											
			case SRVCMD_RD_NOABN_ENABLE:	/* Read/write enable for no abnormal position detection (NOABN). */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrNoAbnEn, &xNvdsConfig.usNoAbnEnabled );
											break;
			case SRVCMD_WR_NOABN_ENABLE:	( void )prvWriteShortParameter( &xNvdsConfig.usNoAbnEnabled, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_NOABN_NUM_PEAKS:	/* Read/write command tap peaks for NOABN command. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrNoAbnNumPeaks, &xNvdsConfig.usNoAbnCmdPeaks );
											break;
			case SRVCMD_WR_NOABN_NUM_PEAKS:	( void )prvWriteShortParameter( &xNvdsConfig.usNoAbnCmdPeaks, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_NOABN_MAX_PERIOD:/* Read/write maximum period for NOABN mode. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrNoAbnMaxPeriod, &xNvdsConfig.usNoAbnMaxPeriod );
											break;
			case SRVCMD_WR_NOABN_MAX_PERIOD:( void )prvWriteShortParameter( &xNvdsConfig.usNoAbnMaxPeriod, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_NOABN_WINDOW:	/* Read/write window width for prolonging the NOABN mode. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrNoAbnWindow, &xNvdsConfig.usAbnPosDetectedWindow );
											break;
			case SRVCMD_WR_NOABN_WINDOW:	( void )prvWriteShortParameter( &xNvdsConfig.usAbnPosDetectedWindow, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_NOABN_ACK_ON:	/* Read/write length of acknowledge vibrations for NOABN command. */
											( void )prvSendByteHexNvdsParameter( pcAt_SrvWrNoAbnCmdAckOn, &xNvdsConfig.ucCmdVibrationParamOn );
											break;
			case SRVCMD_WR_NOABN_ACK_ON:	( void )prvWriteByteParameter( &xNvdsConfig.ucCmdVibrationParamOn, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_NOABN_ACK_OFF:	/* Read/write pause between acknowledge vibrations for NOABN command. */
											( void )prvSendByteHexNvdsParameter( pcAt_SrvWrNoAbnCmdAckOff, &xNvdsConfig.ucCmdVibrationParamOff );
											break;
			case SRVCMD_WR_NOABN_ACK_OFF:	( void )prvWriteByteParameter( &xNvdsConfig.ucCmdVibrationParamOff, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_NOABN_ACK_REP:	/* Read/write number of vibhrations for NOABN command. */
											( void )prvSendByteHexNvdsParameter( pcAt_SrvWrNoAbnCmdAckRep, &xNvdsConfig.ucCmdVibrationParamRep );
											break;
			case SRVCMD_WR_NOABN_ACK_REP:	( void )prvWriteByteParameter( &xNvdsConfig.ucCmdVibrationParamRep, cServerCmdParams, NVDS );
											break;											
											
			case SRVCMD_RD_BATT_EMPTY_THRES:	/* Read/write battery low threshold for power-safe mode. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrBattEmptyThres, &xNvdsConfig.usBattEmptyThres );
											break;
			case SRVCMD_WR_BATT_EMPTY_THRES:	( void )prvWriteShortParameter( &xNvdsConfig.usBattEmptyThres, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_BATT_LOW_THRES:	/* Read/write battery low threshold. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrBattLowThres, &xNvdsConfig.usBattLowThres );
											break;
			case SRVCMD_WR_BATT_LOW_THRES:	( void )prvWriteShortParameter( &xNvdsConfig.usBattLowThres, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_BATT_FULL_THRES:	/* Read/write battery full threshold. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrBattFullThres, &xNvdsConfig.usBattFullThres );
											break;
			case SRVCMD_WR_BATT_FULL_THRES:	( void )prvWriteShortParameter( &xNvdsConfig.usBattFullThres, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_BATT_LOOP_INT:	/* Read/write battery voltage/temperature read loop interval. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrBattLoopInt, &xNvdsConfig.usBattLoopInt );
											break;
			case SRVCMD_WR_BATT_LOOP_INT:	( void )prvWriteShortParameter( &xNvdsConfig.usBattLoopInt, cServerCmdParams, NVDS );
											break;

			case SRVCMD_RD_GPS_ENABLE:		/* Read/write GPS enable. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrGpsEnable, &xNvdsConfig.usGpsEnable );
											break;
			case SRVCMD_WR_GPS_ENABLE:		( void )prvWriteShortParameter( &xNvdsConfig.usGpsEnable, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_GPS_FIX_QUALITY:	/* Read/write minimum HDOP (times 10) quality for an acceptable GPS fix. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrGpsFixQuality, &xNvdsConfig.usGPSMinQuality );
											break;
			case SRVCMD_WR_GPS_FIX_QUALITY:	( void )prvWriteShortParameter( &xNvdsConfig.usGPSMinQuality, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_GPS_MAX_HACC:	/* Read/write maximum horizontal accuracy improvement within 2 seconds for an acceptable GPS fix. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrGpsMaxHAcc, &xNvdsConfig.usGPSMaxHAcc );
											break;
			case SRVCMD_WR_GPS_MAX_HACC:	( void )prvWriteShortParameter( &xNvdsConfig.usGPSMaxHAcc, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_GPS_WAIT_FOR_FIX:/* Read/write maximum time waiting for a good fix before sending a position packet without position. */
											/* ulGPSMaxWaitForFix is a long value in ticks and contains 10x the value which needs to be sent to the server. */
											( void )prvSendShortHexParameter( pcAt_SrvWrMaxGpsWaitTime, ( unsigned short )( ulConfigReadLong( &xNvdsConfig.ulGPSMaxWaitForFix ) / 10ul ) );
											break;
			case SRVCMD_WR_GPS_WAIT_FOR_FIX:( void )prvWriteLongParameterx10( &xNvdsConfig.ulGPSMaxWaitForFix, cServerCmdParams );
											break;
			case SRVCMD_RD_GPS_WAIT_FOR_SAT:/* Read/write maximum time waiting for a GPS satellite to be received before sending a position packet without position. */
											/* ulGPSMaxWaitForSat is a long value in ticks and contains 10x the value which needs to be sent to the server. */
											( void )prvSendShortHexParameter( pcAt_SrvWrMaxGpsSatWaitTime, ( unsigned short )( ulConfigReadLong( &xNvdsConfig.ulGPSMaxWaitForSat ) / 10ul ) );
											break;
			case SRVCMD_WR_GPS_WAIT_FOR_SAT:( void )prvWriteLongParameterx10( &xNvdsConfig.ulGPSMaxWaitForSat, cServerCmdParams );
											break;
			case SRVCMD_RD_GPS_PD_INT:		/* Read/write min. interval between position updates for GPS only power-down. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrGpsPwrDwnInt, &xNvdsConfig.usGpsPwrDownInt );
											break;
			case SRVCMD_WR_GPS_PD_INT:		( void )prvWriteShortParameter( &xNvdsConfig.usGpsPwrDownInt, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_GPS_PSM:			/* Read/write GPS power save mode. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrGpsPsm, &xNvdsConfig.usGpsPsm );
											break;
			case SRVCMD_WR_GPS_PSM:			( void )prvWriteShortParameter( &xNvdsConfig.usGpsPsm, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_GPS_LOCEST_SRV:	/* Read/write location estimation server address. */
											( void )prvSendRawPacket( pcAt_SrvWrGpsLocEstSrv, NULL, ( signed char * )&xNvdsConfig.pcGPSLocEstSrvAddr );
											break;
			case SRVCMD_WR_GPS_LOCEST_SRV:	cServerCmdParams[ strlen( cServerCmdParams ) - 1 ] = 0;		/* remove the residual curly bracket. */
											vConfigWriteString( xNvdsConfig.pcGPSLocEstSrvAddr, cServerCmdParams );
											( void ) prvSendRawPacket( "OK", NULL, NULL );
											break;
			case SRVCMD_RD_GPS_POSREC:		/* Read/write position recording mode. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrGpsPosRec, &xNvdsConfig.usGpsRecording );
											break;
			case SRVCMD_WR_GPS_POSREC:		( void )prvWriteShortParameter( &xNvdsConfig.usGpsRecording, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_GPS_POSREC_INT:	/* Read/write position recording packet interval. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrGpsPosRecInt, &xNvdsConfig.usGpsRecordingPacketInterval );
											break;
			case SRVCMD_WR_GPS_POSREC_INT:	( void )prvWriteShortParameter( &xNvdsConfig.usGpsRecordingPacketInterval, cServerCmdParams, NVDS );
											break;											

			case SRVCMD_RD_BLE_ENABLE:		/* Read/write BLE enable. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrBleEnable, &xNvdsConfig.usBleEnable );
											break;
			case SRVCMD_WR_BLE_ENABLE:		( void )prvWriteShortParameter( &xNvdsConfig.usBleEnable, cServerCmdParams, NVDS );
											if ( usHexStrgToShort( cServerCmdParams ) == 0x0001 ) 
											{
												/* Request starting the BLE, if indoor positioning (BLE) is required in the current state. */
												if ( bCtrlBlePositionRequired() )
												{
													/* Activate BLE. */
													xBleCmd = BLE_LOC_START;
													xQueueSend( xBleCmdQueue, &xBleCmd, 0 ); 
												}
											}
											else
											{
												/* Deactivate BLE. */
												xBleCmd = BLE_LOC_STOP;
												xQueueSend( xBleCmdQueue, &xBleCmd, 0 ); 
												/* Delete all stored BLE positions. */
												vBleDeleteLocBcnStore();
											}
											break;
			case SRVCMD_RD_BLE_UPD_INT:		/* Read/write BLE beacon data update interval. */
											( void )prvSendByteHexNvdsParameter( pcAt_SrvWrBleUpdInt, &xNvdsConfig.ucBleBcnUpdInterval );
											break;
			case SRVCMD_WR_BLE_UPD_INT:		( void )prvWriteByteParameter( &xNvdsConfig.ucBleBcnUpdInterval, cServerCmdParams, NVDS );
											break;
			case SRVCMD_WR_BLE_KEY:			/* Write the BLE AES-128 encryption key for beacon encryption. As there are two keys, the first parameter is the 
											   key index. Boths key may be used for decryption. The last key written becomes the active key for ewncryption. 
											   When getting here, the string cServerCmdParams has the form:
													01,0x112233445566778899AABBCCDDEEFF00 									
													|    |
													+0   +5								*/
											/* Store active key index. */
											vConfigWriteByte( &xNvdsConfig.ucActiveAESKey, ucHexStrgToByte( cServerCmdParams ) );
											/* Store the key itself. */
											{
												unsigned portBASE_TYPE		uxIdx;
												unsigned portBASE_TYPE		uxKeyIdx;
												signed char					cNewAESKey[ AES_BLOCK_LENGTH ];
												
												uxKeyIdx = ucHexStrgToByte( cServerCmdParams );
												
												/* Copy the key to a temporary buffer. */
												for ( uxIdx = 0; uxIdx < AES_BLOCK_LENGTH; uxIdx++ )
												{
													cNewAESKey[ uxIdx ] = ucHexStrgToByte( cServerCmdParams + 5 + 2 * uxIdx );
												}
												vConfigWriteArray( xNvdsConfig.pcAESKey[ uxKeyIdx ], cNewAESKey, AES_BLOCK_LENGTH );
											}
											/* Send the acknowledgement packet. */
											( void )prvSendRawPacket( "OK", NULL, NULL );
											break;
			case SRVCMD_RD_BLE_DFMAP:		/* Read/write BLE DFMAP. */
											( void )prvSendLongHexNvdsParameter( pcAt_SrvWrBleDFMap, &xNvdsConfig.ulBleDFMap );
											break;
			case SRVCMD_WR_BLE_DFMAP:		( void )prvWriteLongParameter( &xNvdsConfig.ulBleDFMap, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_BLE_TXPWR:		/* Read/write BLE transmit power. */
											( void )prvSendByteHexNvdsParameter( pcAt_SrvWrBleTxPwr, &xNvdsConfig.ucBleTxPwr );
											break;
			case SRVCMD_WR_BLE_TXPWR:		( void )prvWriteByteParameter( &xNvdsConfig.ucBleTxPwr, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_BLE_RSSI_CFG:	/* Read/write BLE RSSI filter configuration. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrBleRssiCfg, &xNvdsConfig.usBleRssiCfg );
											/* Configure the RSSI filtering algorithm. */
											usRssiParameter = usConfigReadShort( &xNvdsConfig.usBleRssiCfg );
											uxRssiFilterMethod = ( unsigned char )( ( usRssiParameter >> 13 ) & 0x07 );
											uxRssiFilterWindow = ( unsigned char )( ( usRssiParameter >>  8 ) & 0x1F );				
											vSetRssiCfg( uxRssiFilterMethod, uxRssiFilterWindow );
											break;
			case SRVCMD_WR_BLE_RSSI_CFG:	( void )prvWriteShortParameter( &xNvdsConfig.usBleRssiCfg, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_BLE_MIN_BCN:		/* Read/write BLE minimum number of beacons to turn off GPS. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrBleMinBcn, &xNvdsConfig.usBleMinBcn );
											break;
			case SRVCMD_WR_BLE_MIN_BCN:		( void )prvWriteShortParameter( &xNvdsConfig.usBleMinBcn, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_BLE_IN_DANGER_DUR: /* Read/write BLE time during which the module is assumed to be in a danger zone after
											     reception of a danger beacon. During this period, reception of further danger beacon messages is blocked
											     and the DANGER bit on the BLE beacon is sent. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrBleInDangerDur, &xNvdsConfig.usBleInDangerDur );
											break;
			case SRVCMD_WR_BLE_IN_DANGER_DUR: ( void )prvWriteShortParameter( &xNvdsConfig.usBleInDangerDur, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_BLE_FOREIGN_ALERT: /* Read/write configuration which determines if the device reacts upon a danger indication received from 
											     another module via BLE (DANGER bit in the BLE beacon). */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrBleForeignAlert, &xNvdsConfig.usBleForeignAlert );
											break;
			case SRVCMD_WR_BLE_FOREIGN_ALERT: ( void )prvWriteShortParameter( &xNvdsConfig.usBleForeignAlert, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_BLE_SCAN_FIX_ONLY: /* Read/write configuration which determines if the device scans only for BLE beacons while attempting 
											     a GPS fix or if it scans always. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrBleScanFixOnly, &xNvdsConfig.usBleScanDuringFixOnly );
											break;
			case SRVCMD_WR_BLE_SCAN_FIX_ONLY: ( void )prvWriteShortParameter( &xNvdsConfig.usBleScanDuringFixOnly, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_BLE_MAX_WAIT_BCN: /* Read/write configuration which determines if the device scans only for BLE beacons while attempting 
											     a GPS fix or if it scans always. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrBleMaxWaitForBcn, &xNvdsConfig.usBLEMaxWaitForBcn );
											break;
			case SRVCMD_WR_BLE_MAX_WAIT_BCN: ( void )prvWriteShortParameter( &xNvdsConfig.usBLEMaxWaitForBcn, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_BLE_STD_ADV_INT: /* Read/write BLE standard advertising interval configuration. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrBleStdAdvInt, &xNvdsConfig.usBleStdAdvInterval );
											break;
			case SRVCMD_WR_BLE_STD_ADV_INT: ( void )prvWriteShortParameter( &xNvdsConfig.usBleStdAdvInterval, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_BLE_UUID_FILTER: /* Read/write BLE beacon UUID filter. */
											{
												unsigned portBASE_TYPE		uxUuidIdx;
												unsigned portBASE_TYPE		uxByteIdx;
												signed char 				*pcStrg;
												signed char					cStrg[ ( 2 * BLE_LEN_UUID_FILTER + 8 ) * BLE_NUM_UUID_FILTER ];

												/* Prepare the string to send as response. */																								
												pcStrg = cStrg;
												for ( uxUuidIdx = 0; uxUuidIdx < BLE_NUM_UUID_FILTER; uxUuidIdx++ )
												{
													/* Set the UUID filter index. */
													vByteToHexStrg( pcStrg, uxUuidIdx );
													pcStrg += 2;

													/* Set the filter data. */
													strcpy( pcStrg, ":0x" );
													pcStrg += 3;

													for ( uxByteIdx = 0; uxByteIdx < BLE_LEN_UUID_FILTER; uxByteIdx++ )
													{
														vByteToHexStrg( pcStrg, xNvdsConfig.ucBleBeaconUuidFilter[ uxUuidIdx ][ uxByteIdx ] );
														pcStrg += 2;
													}

													if ( uxUuidIdx != BLE_NUM_UUID_FILTER - 1 )
													{
														strcpy( pcStrg , ",0x" );
														pcStrg += 3;
													}
												}
												*pcStrg = 0;

												( void )prvSendRawPacket( pcAt_SrvWrBleUuidFilter, cStrg, NULL );

											}
											break;
			case SRVCMD_WR_BLE_UUID_FILTER: /* Writing the UUID filter is of the format:
											   00,0x05A106683FC743B0B6830B1E6A06F73D */
											{
												unsigned portBASE_TYPE		uxIdx;
												unsigned portBASE_TYPE		uxUuidIdx;
												signed char					cNewUuidFilter[ BLE_LEN_UUID_FILTER ];
												
												uxUuidIdx = ucHexStrgToByte( cServerCmdParams );
												
												/* Copy the UUID filter value to a temporary buffer. */
												for ( uxIdx = 0; uxIdx < BLE_LEN_UUID_FILTER; uxIdx++ )
												{
													cNewUuidFilter[ uxIdx ] = ucHexStrgToByte( cServerCmdParams + 5 + 2 * uxIdx );
												}
												vConfigWriteArray( xNvdsConfig.ucBleBeaconUuidFilter[ uxUuidIdx ], cNewUuidFilter, BLE_LEN_UUID_FILTER );

												/* Update the UUID-filtering enabled variable. */
												bCheckAndSetUUIDFilterEnabled();

												/* Send the acknowledgement packet. */
												( void )prvSendRawPacket( "OK", NULL, NULL );
											}
											break;
			case SRVCMD_RD_BLE_BCN_FILTER:	/* Read/write BLE localiser beacon filter configuration. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrBleBcnFilter, &xNvdsConfig.usBleBcnFilter );
											break;
			case SRVCMD_WR_BLE_BCN_FILTER:	( void )prvWriteShortParameter( &xNvdsConfig.usBleBcnFilter, cServerCmdParams, NVDS );
											break;


			case SRVCMD_PEEK_RAM:			/* Peek/poke a byte from/to a memory address. */
											/* Peeking can be several bytes. cServerCmdParams is "12345678,0x04". The separation character between the two parameters might be any. */
											( void )prvSendMemoryBytes( cServerCmdParams, "{PEEK_RAM: 0x", &cServerCmdParams[ 11 ] );
											break;
			case SRVCMD_PEEK_NVDS:			/* Peek/poke a byte from/to a memory address. */
											/* Peeking can be several bytes. cServerCmdParams is "12345678,0x04". The separation character between the two parameters might be any. */
											( void )prvSendMemoryBytes( cServerCmdParams, "{PEEK_NVDS: 0x", &cServerCmdParams[ 11 ] );
											break;
			case SRVCMD_POKE_RAM:			/* Format in cServerCmdParams is "12345678,0x12345678". The separation character between the two parameters might be any. */
											{
												ulAddress = ulHexStrgToLong( cServerCmdParams );		/* Put this into a separate line just to calm down GCC on a silly warning. */
												switch ( strlen( &cServerCmdParams[ 11 ] ) - 1 )		/* strlen()-1 get rid of the trailing '}'. */
												{
													case 2:		/* Byte */
													default:	/* Everything elseis taken as byte. */
																( void )prvWriteByteParameter( ( unsigned char * )ulAddress, &cServerCmdParams[ 11 ], RAM );
																break;
													case 4:		/* Short */
																( void )prvWriteShortParameter( ( unsigned short * )ulAddress, &cServerCmdParams[ 11 ], RAM );
																break;
													case 8:		/* Long */
																( void )prvWriteLongParameter( ( unsigned long * )ulAddress, &cServerCmdParams[ 11 ], RAM );
																break;
												}
											}
											break;
			case SRVCMD_POKE_NVDS:			/* Format in cServerCmdParams is "12345678,0x12345678". The separation character between the two parameters might be any. */
											ulAddress = ulHexStrgToLong( cServerCmdParams );		/* Put this into a separate line just to calm down GCC on a silly warning. */
											switch ( strlen( &cServerCmdParams[ 11 ] ) )
											{
												case 2:		/* Byte */
												default:	/* Everything elseis taken as byte. */
															( void )prvWriteByteParameter( ( unsigned char * )ulAddress, &cServerCmdParams[ 11 ], NVDS );
															break;
												case 4:		/* Short */
															( void )prvWriteShortParameter( ( unsigned short * )ulAddress, &cServerCmdParams[ 11 ], NVDS );
															break;
												case 8:		/* Long */
															( void )prvWriteLongParameter( ( unsigned long * )ulAddress, &cServerCmdParams[ 11 ], NVDS );
															break;
											}
											break;
			case SRVCMD_RDSTATS:			/* Read statistics. */
											( void )prvSendStatsBytes();
											break;
			case SRVCMD_RESSTATS:			/* Reset the system statistics counters to 0. */
											vSystemStateStats( SYSSTATS_INIT, 0 );
											vSystemNoAbnStats( SYSSTATS_NOABN_INIT );
											if ( bNoAbnActive )
											{
												vSystemNoAbnStats( SYSSTATS_NOABN_MODESTART );
											}
											/* Reset the reset cause counters maintained in NVDS. */
											uxErrorRecord.ulSWResetCount	 	 = 0;
											uxErrorRecord.ulWDResetCount	 	 = 0;
											uxErrorRecord.ulPOResetCount	 	 = 0;
											uxErrorRecord.ulOtherResetCount  	 = 0;
											uxErrorRecord.ulLockupResetCount  	 = 0;													
											uxErrorRecord.ulStackOverflowCount   = 0;									
											uxErrorRecord.ulSystemErrorCount  	 = 0;											
											uxErrorRecord.ulSDAssertCount  		 = 0;										
											uxErrorRecord.ulSDMemAccCount  		 = 0;											
											uxErrorRecord.ulnRFSDKAssertCount  	 = 0;											
											uxErrorRecord.ulnRFSDKErrorCount  	 = 0;											
											uxErrorRecord.ulnRFUnknownErrorCount = 0;										
											vWriteSystemErrorRecord( &uxErrorRecord, FSTORAGE_SD );
											/* Count one dummy transition into the current state to get correct (or better) results for the average state dwell time. */
											vSystemStateStats( SYSSTATS_UPDATE, xGetCtrlState() );
											/* Send the acknowledgement packet. */
											( void )prvSendRawPacket( "OK", NULL, NULL );
											break;
			case SRVCMD_VIBRATE:			/* Vibrate for the requested duration. */
											( void )prvServerReqVibrate( cServerCmdParams );
											break;
			case SRVCMD_LED:				/* Switch on the LED for the requested duration. */
											( void )prvServerReqLed( cServerCmdParams );
											break;
											
			case SRVCMD_EVACUATE:			if (   ( xGetCtrlState() != CTRL_CHRG_OOO ) 		
												&& ( xGetCtrlState() != CTRL_OOO )
												&& ( xGetCtrlState() != CTRL_CHRG_NRML ) )
											{
												/* Give evacuation alert: Extract evcuation ID. */
												vIndicateEvacuation( ucHexStrgToByte( cServerCmdParams ) );
												
												/* Send the acknowledgement packet. */
												( void )prvSendRawPacket( "OK", NULL, NULL );
											}
											else
											{
												( void )prvSendRawPacket( "KO-ON_CHARGER_OR_OOO", NULL, NULL );
											}
											break;
			case SRVCMD_EVACUATION_STOP:	/* Stop evacuation alert. This will mainly stop the distress beacon. */
											vStopTxEvacuation();
											/* Send the acknowledgement packet. */
											( void )prvSendRawPacket( "OK", NULL, NULL );
											break;
			case SRVCMD_RD_EVAC_VIBR:		/* Read/write EVACUATION vibration sequence parameters. */
											( void )prvSendEvacVibrParameters();
											break;
			case SRVCMD_WR_EVAC_VIBR:		/* Store the vibration sequence for evacuation alert and return an 'OK'. The command parameters are:
													<ON-DURATION>,<OFF-DURATION>,<REPETITIONS>										   
										       with: 
													<ON-DURATION>  (byte) the vibrator on duration in 100 ms steps 
													<OFF-DURATION> (byte) the vibrator off duration in 100 ms steps
													<REPETITIONS>  (byte) the number of times the pattern is to repeat	*/
											vConfigWriteByte( &xNvdsConfig.ucEvacVibrOn , ucHexStrgToByte( cServerCmdParams +  0 ) );
											vConfigWriteByte( &xNvdsConfig.ucEvacVibrOff, ucHexStrgToByte( cServerCmdParams +  5 ) );
											vConfigWriteByte( &xNvdsConfig.ucEvacVibrRep, ucHexStrgToByte( cServerCmdParams + 10 ) );
											/* Send the acknowledgement packet. */
											( void )prvSendRawPacket( "OK", NULL, NULL );
											break;
																						
			case SRVCMD_STATE:				/* Query device state. */
											( void )prvSendDeviceState();
											break;
			case SRVCMD_STANDBY_MODE:		/* Go to standby mode. */
											if (   ( xGetCtrlState() == CTRL_ACTIVE	  ) 
											    || ( xGetCtrlState() == CTRL_INACTIVE ) 
											    || ( xGetCtrlState() == CTRL_STILL	  ) 
											    || ( xGetCtrlState() == CTRL_SLEEP	  ) )
											{
												prvSendEventToCtrl( CTRL_GOTO_STBY );
												/* Invalidate the GPS and BLE data. 
											       There is a slight chance that new beacons are received until the BLE is stopped. */
												prvInvalidateData();
												uxBleTLBcnCnt = 0;			/* field 25: BLE list of received distress/TL beacons. */
												uxForeignAlertType = 0;
											}
											else
											{
												if (   ( xGetCtrlState() == CTRL_CHRG_NRML  ) 
													|| ( xGetCtrlState() == CTRL_CHRG_OOO   ) ) 
												{
													( void )prvSendRawPacket( "KO-ON_CHARGER", NULL, NULL );
												}
												else
												{
													( void )prvSendRawPacket( "KO-INVALID_STATE", NULL, NULL );
												}
											}
											break;
			case SRVCMD_NORMAL_MODE:		/* Go to normal (operating) mode. */
											prvSendEventToCtrl( CTRL_GOTO_NORMAL );
											break;
			case SRVCMD_OOO_MODE:			/* Go to Out-of-office mode. */
											prvSendEventToCtrl( CTRL_GOTO_OOO );
											break;
			case SRVCMD_ALERT_ACK:			/* Server acknowledged Alert. */
											prvSendEventToCtrl( CTRL_ALERT_ACK );
											break;
			case SRVCMD_GOTO_PREALERT:		/* Server forced prealert. 
											   Only accept the  command in certain states. Return an error in all other states. The ctrl state
											   machine would ignore CTRL_GOTO_PREALERT in unsupported states. */
											if (   ( xGetCtrlState() == CTRL_ACTIVE	  ) 
											    || ( xGetCtrlState() == CTRL_INACTIVE ) 
											    || ( xGetCtrlState() == CTRL_STILL	  ) 
											    || ( xGetCtrlState() == CTRL_SLEEP	  )
											    || ( xGetCtrlState() == CTRL_ALERT	  ) 
											    || ( xGetCtrlState() == CTRL_SOS	  ) )
											{
												prvSendEventToCtrl( CTRL_GOTO_PREALERT );
											}
											else
											{
												( void )prvSendRawPacket( "KO-INVALID_STATE", NULL, NULL );												
											}
											break;											
			case SRVCMD_TEST_START:			/* If the device is on the charger, put it into test mode and launch the accelerometer/vibration motor auto test. */
											if ( ( xGetCtrlState() == CTRL_CHRG_NRML ) || ( xGetCtrlState() == CTRL_CHRG_OOO ) )
											{
												/* The device is now in test mode. */
												bTestMode = true;
												
												/* Activate BLE. */
												xBleCmd = BLE_LOC_START;
												xQueueSend( xBleCmdQueue, &xBleCmd, 0 ); 
												
												/* Launch the accelerometer/vibration motor test in the CTRL task. */
												xCtrlEvent = CTRL_ACC_VIBR_TEST;
												xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 ); 
											}
											else
											{
												( void )prvSendRawPacket( "KO-NOT_ON_CHARGER", NULL, NULL );
											}
											break;
			case SRVCMD_TEST_STOP:			/* End of test mode. */
											/* Deactivate BLE. */
											xBleCmd = BLE_LOC_STOP;
											xQueueSend( xBleCmdQueue, &xBleCmd, 0 ); 
											if ( bTestMode == true )
											{
												bTestMode = false;
												( void )prvSendRawPacket( "OK", NULL, NULL );
											}
											else
											{
												( void )prvSendRawPacket( "KO-NOT_IN_TEST_MODE", NULL, NULL );
											}
											V_TRACE_PRINT( TRACE_GSM_SELF_TEST_STOP, TRACE_UART );
											break;													
			case SRVCMD_RD_FTPSRV:			/* Read/write FTP server configuration. */
											( void )prvSendRawPacket( "FTPSRV=", NULL, ( signed char * )&xNvdsConfig.pcFTPServerURL );
											break;
			case SRVCMD_WR_FTPSRV:			cServerCmdParams[ strlen( cServerCmdParams ) - 1 ] = 0;		/* remove the residual curly bracket. */
											vConfigWriteString( xNvdsConfig.pcFTPServerURL, cServerCmdParams );
											( void ) prvSendRawPacket( "OK", NULL, NULL );
											break;
			case SRVCMD_RD_FTP_CFG:			/* Read/write FTP configuration. */
											( void )prvSendRawPacket( pcAt_SrvWrFtpCfg, NULL, ( signed char * )&xNvdsConfig.pcFTPCfg );
											break;
			case SRVCMD_WR_FTP_CFG:			cServerCmdParams[ strlen( cServerCmdParams ) - 1 ] = 0;		/* remove the residual curly bracket. */
											vConfigWriteString( xNvdsConfig.pcFTPCfg, cServerCmdParams );
											( void ) prvSendRawPacket( "OK", NULL, NULL );
											break;
										
			case SRVCMD_FWUPD:				/* FW Update. */
											prvFirmwareUpdate();
											break;
											
			case SRVCMD_FS_LST:				/* GSM module file system file listing. */
											V_TRACE_PRINT( TRACE_GSM_FS_LISTING, TRACE_UART );
											prvFileSystemListing();
											break;											
			case SRVCMD_FS_GET:				/* GSM module file system file download (send to FTP server). */
											V_TRACE_PRINT( TRACE_GSM_FS_GET_FILE, TRACE_UART );
											prvFileSystemFileGet();
											break;											
			case SRVCMD_FS_PUT:				/* GSM module file system file upload (received from FTP server). */
											V_TRACE_PRINT( TRACE_GSM_FS_PUT_FILE, TRACE_UART );
											prvFileSystemFilePut();
											break;											
			case SRVCMD_FS_DEL:				/* GSM module file system file file delete. */
											V_TRACE_PRINT( TRACE_GSM_FS_FILE_DELETE, TRACE_UART );
											prvFileSystemFileDelete();
											break;
											
			case SRVCMD_RD_SEC_ENABLE:		/* Read/write security enable. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrSecEnable, &xNvdsConfig.usSecEnable );
											break;
			case SRVCMD_WR_SEC_ENABLE:		( void )prvWriteShortParameter( &xNvdsConfig.usSecEnable, cServerCmdParams, NVDS );
											break;
											
			case SRVCMD_RD_LOG_ENABLE:		/* Read/write log enable. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrLogEnable, &xNvdsConfig.usLogEnable );
											break;
			case SRVCMD_WR_LOG_ENABLE:		( void )prvWriteShortParameter( &xNvdsConfig.usLogEnable, cServerCmdParams, NVDS );
											break;
										
			case SRVCMD_RD_MOD_SIDE:		/* Read/write module side. */
											( void )prvSendShortHexNvdsParameter( pcAt_SrvWrModuleSide, &xNvdsConfig.usModuleSide );
											break;
			case SRVCMD_WR_MOD_SIDE:		( void )prvWriteShortParameter( &xNvdsConfig.usModuleSide, cServerCmdParams, NVDS );
											break;
											
			case SRVCMD_WR_RELAY_CMD:		/* Write relay command. */
											prvParseRelayCmdParams();
											break;

			case SRVCMD_RD_DANGER_BCN_THRES:/* Read/write 'danger zone' beacon threshold. */
											( void )prvSendByteHexNvdsParameter( pcAt_SrvWrDangerBcnThres, ( unsigned char * )&xNvdsConfig.cDangerBcnThres );
											break;
			case SRVCMD_WR_DANGER_BCN_THRES:( void )prvWriteByteParameter( ( unsigned char * )&xNvdsConfig.cDangerBcnThres, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_NOABN_BCN_THRES:	/* Read/write 'no abnormal position zone' beacon threshold. */
											( void )prvSendByteHexNvdsParameter( pcAt_SrvWrNoAbnBcnThres, ( unsigned char * )&xNvdsConfig.cNoAbnBcnThres );
											break;
			case SRVCMD_WR_NOABN_BCN_THRES:	( void )prvWriteByteParameter( ( unsigned char * )&xNvdsConfig.cNoAbnBcnThres, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_PRIVATE_BCN_THRES:/* Read/write 'private zone' beacon threshold. */
											( void )prvSendByteHexNvdsParameter( pcAt_SrvWrPrivateBcnThres, ( unsigned char * )&xNvdsConfig.cPrivateBcnThres );
											break;
			case SRVCMD_WR_PRIVATE_BCN_THRES:( void )prvWriteByteParameter( ( unsigned char * )&xNvdsConfig.cPrivateBcnThres, cServerCmdParams, NVDS );
											break;
			case SRVCMD_RD_IMMOBILITY_BCN_THRES:/* Read/write 'immobility zone' beacon threshold. */
											( void )prvSendByteHexNvdsParameter( pcAt_SrvWrImmobilityBcnThres, ( unsigned char * )&xNvdsConfig.cImmobilityBcnThres );
											break;
			case SRVCMD_WR_IMMOBILITY_BCN_THRES:( void )prvWriteByteParameter( ( unsigned char * )&xNvdsConfig.cImmobilityBcnThres, cServerCmdParams, NVDS );
											break;
											
			case SRVCMD_OTHER:				/* Received string from server starts like a command but is not recognised. */
											( void )prvSendRawPacket( "KO-SYNTAX_ERROR", NULL, NULL );
											break;
			default:						/* Do nothing for all commands / AT messages not recognised. */
											break;
		}
	}	
	/* Continue looping until all AT commands have been pulled off the queue. */
	while ( xQueueReceive( xParserAtRespQueue, &xServerCmd, 0 ) == pdPASS );

	/* Reset flag to indicate that a server command transaction is finished. */
	bCommandTransactionOngoing = false;
	
	/* Command transaction terminated: If there was a GSM shutdown requested before treating the command, restore it now.
	   Re-entreing the vCheckConditionsAndShutdownModule() verifies if the timing conditions are still met. */
	if ( bGsmShutDownWasRequested )
	{
		/* Finally, shut down the GSM module. */
		vCheckConditionsAndShutdownModule( FROM_SERVER_COMMAND );
	}	

	/* Allow the module to enter idle PSM. */
	xModuleEnableSleepMode();

	/* Return telling that a command has been treated.*/
	return true;
}
/*-----------------------------------------------------------*/

/* Check, if a server command has arrived. If this is the case, treat it. The GSM module state is not altered. 
   Returns true, if a server command has been received and treated.
*/
bool bCheckForServerCommand( void )
{
	bool	bCmdFound;
	
	bCmdFound = false;
	
	/* Check if a server command has been received. If this is the case, treat it. */
	if (    bQueueCheckIfDataItemInQueue( xGsmCmdQueue, GSM_SERVER_CMD )
		 || bQueueCheckIfDataItemInQueue( xParserAtRespQueue, AT_TCP_RX_DATA_IND )
	   )
	{
		/* Remove the command from the queue. */
		vRemoveTypeFromQueue( xGsmCmdQueue, GSM_SERVER_CMD );
		
		/* Treat the command. */
		bCmdFound = bTreatServerCmd();
	}
	
	return bCmdFound;
}
/*-----------------------------------------------------------*/

/* 	This function is run only after coming out of reset. Check, if the bootloader ran just before and
	send results correspondingly to the server. 
	
	The bootloader communicates with the application by the means of the GPREGRET and GPREGRET2 8-bit registers.
	Those registers are not reset when the nRF reboots (except for power cycles, though).
	
	GPREGRET if for sending commands to the bootloader:
		GPREGRET(7:4)		bFWUpdOngoing			BOOTLOADER_DFU_GPREGRET (0xB0), see nrf_bootloader_info.
		GPREGRET(2)									Update softdevice (SD).
		GPREGRET(1)									Update application (FW).
		GPREGRET(0)									Enter bootloader (BOOTLOADER_DFU_START_BIT_MASK).

	GPREGRET2 sends information back from the bootloader to the application. This register is valid only if the 
	bootloader just ran before. The bootloader writes BOOTLOADER_DFU_GPREGRET_MASK into GPREGRET(7:4) to notify the application.
		GPREGRET2(7:4)								Extended error code (cf. nrf_dfu_ext_error_code_t), to APP
		GPREGRET2(2)			bFWRolledBack		FW rolled back. Extended error code is valid.
		GPREGRET2(1)			bNewFWNotFunctional	New FW not functional (returned from APP)
		GPREGRET2(0)			bFWUpdated			FW updated													
		
	Not all error cases are caught with this mechanism. 
	
	Coding and meaning of flags:
	
								|  bFWUpdOngoing  |    bFWUpdated   | bNewFWNotFunctional | bFWRolledBack   |   message to server
	----------------------------+-----------------+-----------------+---------------------+-----------------+---------------------------
	normal boot         		|      false      |      false      |         false       |      false      |       n/a       
		                        |                 |                 |                     |                 |
	FW update request   		|      TRUE       |      false      |         false       |      false      |       n/a       
		                        |                 |                 |                     |                 |
	FW rolled-back (bootloader  |      false      |      false      |         false       |      TRUE       |   KO-FWUPD_FILECHECK_ERROR       	
		file load/check failure)|                 |                 |                     |                 |
	FW updated             		|      false      |      TRUE       |         false       |      false      |       OK	       	
		                        |                 |                 |                     |                 |
	FW not functional, roll-back|      TRUE       |      false      |         TRUE        |      false      |       n/a	       	
		requested               |                 |                 |                     |                 |
	FW rolled-back (not         |      TRUE       |      false      |         TRUE        |      TRUE       |   KO-FWUPD_COMM_ERROR    	       	
		functional)             |                 |                 |                     |                 |
	
	The bNewFWNotFunctional is first set by the faulty FW. Then the bootloader loads the roll-back FW without touching that flag.
	
	Note that the flags are read and set also by the bootloader.
*/
void vSendFWUpdateResultToServer( void )
{
	ret_code_t				xErrCode;
	bool					bSuccess;
	unsigned portBASE_TYPE	uxAttemptCnt;
	enum xAT_RESULT			xAtResult;
	const signed char		*pcResultStrg;		/* Pointer to the result string. */
	bool					bFWUpdOngoing;
	bool					bFWUpdated;
	bool					bNewFWNotFunctional;
	bool					bFWRolledBack;
	signed char				cExtErrorCode[ 10 ];
	signed char				*pcExtErrorCode;
	uint32_t				uxGpregret;
	uint32_t				uxGpregret2;

	/* Read the GPREGRET registers. */
	#if defined ( NO_SOFTDEVICE )
		uxGpregret = nrf_power_gpregret_get();
		uxGpregret2 = nrf_power_gpregret2_get();
	#else
		xErrCode = sd_power_gpregret_get( 0, &uxGpregret );
		APP_ERROR_CHECK(xErrCode );
		xErrCode = sd_power_gpregret_get( 1, &uxGpregret2 );
		APP_ERROR_CHECK(xErrCode );
	#endif
	
	bFWUpdOngoing 		= ( ( uxGpregret & BOOTLOADER_DFU_START_MASK ) == BOOTLOADER_DFU_GPREGRET );
	bFWUpdated 			= uxGpregret2 & BL_FW_UPDATED;
	bNewFWNotFunctional	= uxGpregret2 & BL_NEW_FW_NOT_FUNCTIONAL;
	bFWRolledBack 		= uxGpregret2 & BL_FW_ROLLED_BACK;	
	
	/* If the GPREGRET registers do not contain the magic value, the registers are not assumed to be valid. In that case, exit 
	   immediately. */
	if ( !bFWUpdOngoing )
	{
		#if defined ( NO_SOFTDEVICE )
			nrf_power_gpregret_set( 0 );
			nrf_power_gpregret2_set( 0 );
		#else
			xErrCode = sd_power_gpregret_clr( 0, 0xffffffff );
			APP_ERROR_CHECK( xErrCode );
			xErrCode = sd_power_gpregret_clr( 0, 0xffffffff );
			APP_ERROR_CHECK( xErrCode );
		#endif
		
		return;
	}
	
	V_TRACE_PRINT_BYTE( TRACE_FWUPDATED, bFWUpdated, TRACE_UART );
	V_TRACE_PRINT_BYTE( TRACE_FWROLLEDBACK, bFWRolledBack, TRACE_UART );
	
	/* Check the NVDS for the FW updated flag. */
	if (    !bFWUpdated
		 && !bFWRolledBack )
	{
		/* Neiter the FW was updated nor rolled back. So we came out of a normal reset
		   and continue without any special action. */
		return;
	}
		   
	/* Just before rebooting the module, the bootloader ran. We now need to
	   establish a connection with the server and return a result.
	   If we cannot send a result, something is very wrong and we roll-back to the 
		previous FW version. */
		
	/* Switch on the module. */
	/* Check battery and - if low - loop until the battery is good again. */
	while ( bBatteryEmpty() )
	{
		V_TRACE_PRINT( TRACE_GSM_LOW_BATTERY, TRACE_UART );
		
		/* Wait a bit before testing the battery status again. */
		vTaskDelay( T_GSM_BAT_TEST );
	}
	
	/* Select the appropriate result string. */
	if ( bFWUpdated )
	{
		pcResultStrg = "OK";
	}
	if ( bFWRolledBack )
	{
		ret_code_t					xErrCode;
		uint32_t					uxGpregret2;
		unsigned portBASE_TYPE		uxExtErrorCode;
		
		if ( bNewFWNotFunctional )
		{
			pcResultStrg = "KO-FWUPD_NOT_FUNCTIONAL,EXT_ERR_CODE=0x";
		}
		else
		{
			pcResultStrg = "KO-FWUPD_ROLLED_BACK,EXT_ERR_CODE=0x";
		}
		
		#if defined ( NO_SOFTDEVICE )
			uxGpregret2 = nrf_power_gpregret2_get();
		#else
			xErrCode = sd_power_gpregret_get( 1, &uxGpregret2 );
			APP_ERROR_CHECK( xErrCode );
		#endif
		uxExtErrorCode = ( uxGpregret2 & 0xf0 ) >> 4;
		vByteToHexStrg( cExtErrorCode, uxExtErrorCode );
		pcExtErrorCode = cExtErrorCode;
	}
	else
	{
		pcExtErrorCode = NULL;
	}

	/* Establish TCP connection to the server. */
	uxAttemptCnt = 0;
	do
	{
		/* Attempt to start the GSM module and attach to the network. */
		bSuccess = ( xStartCellularModule() == CONNECT_SUCCESS );
		/* Start the TCP connection to the server. */
		if ( bSuccess )
		{
			bSuccess &= bStartGsmTCPConn();
		}
		/* Send the version information to the server. */
		if ( bSuccess )
		{
			xAtResult = prvSendVersionInformation();
		}
		bSuccess &= ( xAtResult == AT_SUCCESS );
		/* Send the result code to the server. */
		if ( bSuccess )
		{
			xAtResult = prvSendRawPacket( pcResultStrg, pcExtErrorCode, NULL );
		}
		bSuccess &= ( xAtResult == AT_SUCCESS );
		
		uxAttemptCnt++;
	}
	while ( !bSuccess && uxAttemptCnt < FWUPD_ESTABL_RETRY );
	
	/* We could not send a packet to the server. Something is probably seriously wrong.
	   Roll-back the FW to a safe version. 
	   This is achieved by setting the BL_NEW_FW_NOT_FUNCTIONAL flag. */
	if ( !bSuccess )
	{
		/* Set the flags. */
		#if defined ( NO_SOFTDEVICE )
			nrf_power_gpregret_set( 0 );
			nrf_power_gpregret2_set( 1 );
			nrf_power_gpregret_set( BOOTLOADER_DFU_START );
			nrf_power_gpregret2_set( BL_NEW_FW_NOT_FUNCTIONAL );
		#else
			xErrCode = sd_power_gpregret_clr( 0, 0xffffffff );
			APP_ERROR_CHECK( xErrCode );
			xErrCode = sd_power_gpregret_clr( 1, 0xffffffff );
			APP_ERROR_CHECK( xErrCode );
			xErrCode = sd_power_gpregret_set( 0, BOOTLOADER_DFU_START );
			APP_ERROR_CHECK(xErrCode );
			xErrCode = sd_power_gpregret_set( 1, BL_NEW_FW_NOT_FUNCTIONAL );
			APP_ERROR_CHECK( xErrCode );
		#endif
		
		/* Reset to get the bootloader running. */
		V_SYSTEM_RESET( NEW_FW_NOT_FUNCTIONAL );
	}
	
	/* Clear the GPREGRET/GPREGRET2 registers. The update was successful. */
	#if defined ( NO_SOFTDEVICE )
		nrf_power_gpregret_set( 0 );
		nrf_power_gpregret2_set( 0 );
	#else
		xErrCode = sd_power_gpregret_clr( 0, 0xffffffff );
		APP_ERROR_CHECK( xErrCode );
		xErrCode = sd_power_gpregret_clr( 1, 0xffffffff );
		APP_ERROR_CHECK( xErrCode );
	#endif
}
/*-----------------------------------------------------------*/

/* Main GSM task function. */
static portTASK_FUNCTION( vGSMTask, pvParameters )
{
	enum xGSM_CMD			xGsmCmd;
	
	/* Just to stop compiler warnings. */
	( void )pvParameters;

	vTaskSetApplicationTaskTag( NULL, ( void * ) GSM_TASK_TAG );
	
	/* Wait untile the configuration handler is initialised. */
	while ( !bCheckConfigInitialised() || !bCheckTraceInitialised() )
	{
		vTaskDelay( 1 );
	}

	NRF_LOG_INFO( "Task started." );
	NRF_LOG_FLUSH();

	/* At this point, we are coming just out of reset. Check, if the bootloader ran just before and
	   send results correspondingly to the server. */
	#if !defined ( NO_SOFTDEVICE )
		vSendFWUpdateResultToServer();
	#endif
	
	/* Start the position transmit timer. */
	( void )xTimerStart( xPosTxTimer, portMAX_DELAY );	

	while ( 1 )
	{
		if ( xQueueReceive( xGsmCmdQueue, &xGsmCmd, portMAX_DELAY ) != errQUEUE_EMPTY )
		{
			/* Received a message from the CTRL task.
			   +UUSORD indications from the parser will also end up here as GSM_SERVER_CMD. The actual command string
			   will need to be read from the GSM module. Note that it happens, though, that server acknowledgements
			   are copied as commands. */
			switch ( xGsmCmd )
			{
				case GSM_SEND_FOREIGN_ALERT_SOS:
				case GSM_SEND_ALERT_SOS:	/* Send immediately an alert or SOS message. The alert/SOS is sent as a default message
											   with the alert or SOS bit set in the indications bit map field. */
											/* Add a small delay to make sure the state transition in CTRL has happened. */
											vTaskDelay( 2 );
											/* Send the message as soon as possible. Set the timer interval to 1s. */
											prvUpdateTransmissionTimer( 1 );
											V_TRACE_PRINT( TRACE_GSM_ALERT_OR_SOS_DETECTED, TRACE_UART );
											break;
											
				case GSM_SEND_POSITION:		/* Send a position packet to the server. */
											V_TRACE_PRINT( TRACE_GSM_SEND_POSITION, TRACE_UART );
											( void )xTimerStop( xPosTxTimer, portMAX_DELAY );
											/* Record the Tx instant time stamp to calculate the elapsed time once the packet is pushed. This is mainly
											   for debug and power calculation (on the server) purposes. */
											usTxInstantTS = usReadRTC();
											prvTransmitPosition();
											break;
											
				case GSM_POS_INT_UPDATE:	/* The time interval to send a position message to the server has been updated. */
											{
												TickType_t		xRemainingTime;
												unsigned long	ulNewPositionInterval;
												
												/* Get snapshots of the current time and the time the timer will expire. */
												portENTER_CRITICAL();
												xRemainingTime = xTimerGetExpiryTime( xPosTxTimer ) - xTaskGetTickCount();
												portEXIT_CRITICAL();
												
												/* Convert the remaining timer time to seconds. */
												xRemainingTime /= portTICKS_PER_SEC;
												
												/* Check if the remaining timer time (in seconds) is larger than the new interval. If this is the case, 
												   shorten the timer. 
												   CTRL_STANDBY is a special case as the timer callback never updates the transmission time: the call is blocked
												   in this state. Make sure the timer gets nevertheless the new value. */
												ulNewPositionInterval = ( unsigned long )xGetCtrlPositionSendInterval();
												if (    ( xRemainingTime > ulNewPositionInterval )
													 || ( xGetCtrlState() == CTRL_STANDBY )
												   )
												{
													/* Make sure that any shortening of the send interval is immediately taken into account. */
													prvUpdateTransmissionTimer( ulNewPositionInterval );
												}
											}
											break;
											
				case GSM_SERVER_CMD:		/* Received a command from the remote server. */
											/* Pull the command from the server and treat it. */
											( void )bTreatServerCmd();
											break;
											
				case GSM_SHUTDOWN_MODULE:	/* Shutdown the GSM/GPS module. If, do a last safety check that no transactions are pending
											   (should not be the case by construction. */
											if ( !bQueueCheckIfDataItemInQueue( xGsmCmdQueue, GSM_SEND_POSITION ) && !bCommandTransactionOngoing )
											{
												/* Fully shut-down the module and set the status to not-connected. */
												vGSMModulePowerDown( SOFT_POWER_OFF );
											}
											break;
											
				case GSM_ACC_VIBR_TEST_DONE:/* The accelerometer/vibrator test, which runs in the CTRL task, has finished. Take the
											   result and send it to the server. */
											if ( xCtrlAccVibrTestResult != COULD_NOT_RUN )
											{
												signed char		cTestResult[ 11 ] = "F/F";
												
												/* Accelerometer self-test result. */
												if ( ( xCtrlAccVibrTestResult & ACC_PASSED ) == ACC_PASSED )
												{
													cTestResult[ 0 ] = 'P';
												}
												else
												{
													cTestResult[  3 ] = ' ';
													cTestResult[  4 ] = '(';
													vShortToHexStrg( cTestResult + 5, usAccErrorCode );
													cTestResult[  9 ] = ')';
													cTestResult[ 10 ] = 0;
												}
												
												/* Vibrator self-test result. */
												if ( ( xCtrlAccVibrTestResult & VIBR_PASSED ) == VIBR_PASSED )
												{
													cTestResult[ 2 ] = 'P';
												}
													
												/* Send the packet with the self-test result. */
												( void )prvSendRawPacket( "OK-ACC/VIBR TEST RESULT: ", cTestResult, NULL );
											}
											else
											{
												( void )prvSendRawPacket( "KO-NOT_ON_CHARGER", NULL, NULL );
											}
											break;
											
				case GSM_IN_PRIVATE_ZONE:	/* The module is in a private zone. Stop all GSM reporting by resetting the next report
											   time. Thus, every time this message is received, the report time gets pushed out further. */		
											if (   ( xGetCtrlState() != CTRL_ALERT )
												&& ( xGetCtrlState() != CTRL_SOS ) )
											{
												V_TRACE_PRINT( TRACE_GSM_PRIVATE_ZONE, TRACE_UART );

												prvUpdateTransmissionTimer( xGetCtrlPositionSendInterval() );

												/* Invalidate the collected data. */
												prvInvalidateData();

												/* Remove any pending requests to transmit messages. */
												vRemoveTypeFromQueue( xGsmCmdQueue, GSM_SEND_POSITION );
	
												/* Shut down the GSM/GPS module should it be running. At this point we are sure that there
												   is no transmission going on (else we would be in prvTransmitPosition()). */
												if ( xModuleState & GSM_PWR_ON )
												{
													xGsmCmd = GSM_SHUTDOWN_MODULE;
													xQueueSend( xGsmCmdQueue, &xGsmCmd, 0 );
												}
												
												/* Suspend BLE heartbeat transmissions. The transmissions will be resumed in the prvTransmitPosition()
												   function the next time a server packet will be sent out. */
												vNonBlockingSuspendBleTxActivity();
											}
											break;
											
				case GSM_IN_STANDBY:		/* The module has been set to STANDBY state. Stop all GSM activity.
											   If the position transmission timer expires, the handler will not call the prvTransmitPosition() function. 
											   The BLE is being taken care of by the CTRL task. */		
											/* Invalidate the collected data. */
											prvInvalidateData();
											uxBleTLBcnCnt = 0;			/* field 25: BLE list of received distress/TL beacons. */
											uxForeignAlertType = 0;
											
											/* Remove any pending requests to transmit messages. */
											vRemoveTypeFromQueue( xGsmCmdQueue, GSM_SEND_POSITION );

											/* Shut down the GSM/GPS module should it be running. At this point we are sure that there
											   is no transmission going on (else we would be in prvTransmitPosition()). */
											if ( xModuleState & GSM_PWR_ON )
											{
												xGsmCmd = GSM_SHUTDOWN_MODULE;
												xQueueSend( xGsmCmdQueue, &xGsmCmd, 0 );
											}
											break;
											
				default:					break;
			}
		}
	}
}
/*-----------------------------------------------------------*/