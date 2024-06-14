/*
 * Tracker Firmware
 *
 * AT message parser task
 *
 * This task runs with the highest priority. It is triggered by the UART RX interrupt
 * which detects a carriage-return line-feed indicating end of an AT response.
 * The RX interrut routine sends a semaphore triggering the parser. 
 * The parser compares the received string simultaneously against all reference
 * strings. If a match is found, it stores the remainder of the received string as
 * parameter into the location pointed to in the table containing all references
 * and sends an indication to the GSM task.
 *
 * The parser task is stateless.
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "tracker.h"

/* nRF SDK include files */
#define NRF_LOG_MODULE_NAME 		PAR
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* Scheduler include files */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Device specific include files. */
#include "tracker.h"

#include "custom_board.h"
#include "drv_uart.h"
#include "drv_nvm.h"

#include "config.h"
#include "parser.h"
#include "gsm.h"
#include "ctrl.h"
#include "charger.h"
#include "rtc.h"
#include "trace.h"
#include "tracemsg.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Private function prototypes. */
/* The parser task as described at the top of the file. */
static portTASK_FUNCTION_PROTO( vParserTask, pvParameters );
static void prvParseCREG( unsigned short usStrgIdx );
static void prvParseSocId( unsigned short usStrgIdx );
static void prvParseTcpRxData( unsigned short usStrgIdx );
static void prvParseCCID( unsigned short usStrgIdx );
static void prvParseSignalQuality( unsigned short usStrgIdx );
static void prvParseQNwInfo( unsigned short usStrgIdx );
static void prvParseCOps( unsigned short usStrgIdx );
static void prvParseQFTPGet( unsigned short usStrgIdx );
static void prvClrStorage( unsigned short usStrgIdx );
static void prvStoreRemoteArgs( unsigned short usStrgIdx );
static void prvAppendRemoteArgs( unsigned short usStrgIdx );
static void prvStoreStrings( unsigned short usStrgIdx );
static unsigned portBASE_TYPE prvCopyFromUARTBuffer( unsigned short usStrgIdx, unsigned portBASE_TYPE uxFieldNum, 
													 unsigned portBASE_TYPE uxOffset, signed char *pcDest, unsigned portBASE_TYPE uxMaxCnt );
/*-----------------------------------------------------------*/

/* Global variables. */
QueueHandle_t 			xParserAtRespQueue; 	/* AT response indicator queue handle. */
signed char  			*pcExpectedParamStrg;	/* Pointer to a buffer to store the next non-empty string in. Set in the GSM task. */
unsigned portBASE_TYPE	uxExpectedParamLen;		/* Maximum number of characters to store in the buffer, set in the GSM task. If 0,
												   nothing is stored. */
bool					bStoreSeveralStrings;	/* Request storage of several strings. */
TaskHandle_t 			xParserTaskHandle;		/* Task handle of the parser task so that it can be controlled. */
												   
/*-----------------------------------------------------------*/

/* Module scope variables. */
/* Definition of the AT response strings used to compare the actual
   responses received over the UART. The strings are stored in program
   memory. */
const char pcAt_Ok[] 						= "OK";			
const char pcAt_CReg0[] 					= "+C*REG: *,0";	
const char pcAt_CReg1[] 					= "+C*REG: *,1";	
const char pcAt_CReg5[] 					= "+C*REG: *,5";	

const char pcAt_CPSmsRes[]	 				= "+CPSMS:";
const char pcAt_QNwScanModeRes[] 			= "+QCFG: \"nwscanmode\",";
const char pcAt_QNwScanSeqRes[] 			= "+QCFG: \"nwscanseq\",0";
const char pcAt_QIotOpModeRes[] 			= "+QCFG: \"iotopmode\",";
const char pcAt_CGActRes[] 					= "+CGACT: *,";
const char pcAt_QBoot[]						= "APP";
const char pcAt_CGAttRes[] 					= "+CGATT: ";
const char pcAt_SendOk[]					= "SEND OK";
const char pcAt_SendFail[]					= "SEND FAIL";
const char pcAt_Connect[]					= "CONNECT";
const char pcAt_QHttpGetOK[]				= "+QHTTPGET: 0,";
const char pcAt_QHttpGetKO[]				= "+QHTTPGET: *";					/* This check needs to come *after* the OK check. */

const char pcAt_QSSLOpen[]		 			= "+QSSLOPEN: *,0";	
const char pcAt_QSSLOpen2[]		 			= "+QSSLOPEN: **,0";	
const char pcAt_QIOpen[]		 			= "+QIOPEN: *,0";	
const char pcAt_QIOpen2[]		 			= "+QIOPEN: **,0";	
const char pcAt_QSSLUrcRx[] 				= "+QSSLURC: \"recv\",";
const char pcAt_QIUrcRx[] 					= "+QIURC: \"recv\",";
const char pcAt_QSSLUrcCl[] 				= "+QSSLURC: \"closed\",";
const char pcAt_QIUrcCl[] 					= "+QIURC: \"closed\",";
const char pcAt_QSSLState[]					= "+QSSLSTATE: ";
const char pcAt_QIState[]					= "+QISTATE: ";
const char pcAt_QIError[] 					= "+QIGETERROR: ";
const char pcAt_QIURC[] 					= "+QIURC: \"dnsgip\",\"";

const char pcAt_CSQ[]	 					= "+CSQ: ";
const char pcAt_CCID[]	 					= "+QCCID: ";
const char pcAt_QNWINFO[]	 				= "+QNWINFO: ";
const char pcAt_COps[]	 					= "+COPS: 0,2,\"";
const char pcAt_QPingRsp[]					= "+QPING: 0,4,";

const char pcAt_GsmCGEvHwDetach[]			= "+CGEV: NW DETACH";				/* The network has forced a GPRS detach. */
const char pcAt_FOpen[]						= "+QFOPEN: ";						/* File OPEN result (file handle). */
const char pcAt_FTPOpenOK[]					= "+QFTPOPEN: 0,";					/* FTP OPEN successful. */
const char pcAt_FTPOpenKO[]					= "+QFTPOPEN: ";					/* FTP OPEN unsuccessful. */
const char pcAt_FTPCloseOK[]				= "+QFTPCLOSE: 0,";					/* FTP CLOSE successful. */
const char pcAt_FTPCloseKO[]				= "+QFTPCLOSE: ";					/* FTP CLOSE unsuccessful. */
const char pcAt_FTPCwdOK[]					= "+QFTPCWD: 0,";					/* FTP CWD successful. */
const char pcAt_FTPCwdKO[]					= "+QFTPCWD: ";						/* FTP CWD unsuccessful. */
const char pcAt_FTPGetOK[]					= "+QFTPGET: 0,";					/* FTP GET successful. */
const char pcAt_FTPGetKO[]					= "+QFTPGET: ";						/* FTP GET unsuccessful. */
const char pcAt_FTPPutOK[]					= "+QFTPPUT: 0,";					/* FTP PUT successful. */
const char pcAt_FTPPutKO[]					= "+QFTPPUT: ";						/* FTP PUT unsuccessful. */
const char pcAt_FTPER[]						= "+QFTPSTAT: ";					/* FTP error code. */
const char pcAt_LstFree[]					= "+QFLDS: ";
const char pcAt_LstFile[]					= "+QFLST: ";
const char pcAt_GsmCEER[]					= "+CEER: ";
const char pcAt_Error[] 					= "ERROR";	
const char pcAt_CMEError[] 					= "+CM* ERROR: ";	
const char pcAt_AtEcho[]					= "AT";			

/* Acknowledgement from the remote server. */	
const char pcAt_RemoteOK[] 					= "[OK]";
const char pcAt_RemoteKO[] 					= "[KO";

/* Commands / indications from the remote server. */	
const char pcAt_SrvRdVersion[] 				= "{VERSION?}";
const char pcAt_SrvReset[]	 				= "{RESET}";
const char pcAt_SrvRdGPRSCfg[] 				= "{GPRSCFG?}";
const char pcAt_SrvWrGPRSCfg[]				= "{GPRSCFG=";
const char pcAt_SrvRdTcpCfg[] 				= "{TCPCFG?}";
const char pcAt_SrvWrTcpCfg[]				= "{TCPCFG=";
const char pcAt_SrvRdSecTcpCfg[] 			= "{SECTCPCFG?}";
const char pcAt_SrvWrSecTcpCfg[]			= "{SECTCPCFG=";
const char pcAt_SrvRdDFMap[] 				= "{DFMAP?}";
const char pcAt_SrvWrDFMap[]				= "{DFMAP=0x";
const char pcAt_SrvRdRat[] 					= "{RAT?}";
const char pcAt_SrvWrRat[]					= "{RAT=0x";
const char pcAt_SrvRdAttTo[] 				= "{ATT_TO?}";
const char pcAt_SrvWrAttTo[]				= "{ATT_TO=0x";

const char pcAt_SrvRdActInt[] 				= "{ACT_POS_INT?}";
const char pcAt_SrvWrActInt[]				= "{ACT_POS_INT=0x";
const char pcAt_SrvRdInactInt[] 			= "{INACT_POS_INT?}";
const char pcAt_SrvWrInactInt[]				= "{INACT_POS_INT=0x";
const char pcAt_SrvRdStillInt[] 			= "{STILL_POS_INT?}";
const char pcAt_SrvWrStillInt[]				= "{STILL_POS_INT=0x";
const char pcAt_SrvRdSleepInt[] 			= "{SLEEP_POS_INT?}";
const char pcAt_SrvWrSleepInt[]				= "{SLEEP_POS_INT=0x";
const char pcAt_SrvRdOooInt[] 				= "{OOO_POS_INT?}";
const char pcAt_SrvWrOooInt[]				= "{OOO_POS_INT=0x";
const char pcAt_SrvRdChrgInt[] 				= "{CHRG_POS_INT?}";
const char pcAt_SrvWrChrgInt[]				= "{CHRG_POS_INT=0x";
const char pcAt_SrvRdAlertInt[] 			= "{ALERT_POS_INT?}";
const char pcAt_SrvWrAlertInt[]				= "{ALERT_POS_INT=0x";
const char pcAt_SrvRdPwrDwnInt[]			= "{MOD_PWRDWN_INT?}";
const char pcAt_SrvWrPwrDwnInt[]			= "{MOD_PWRDWN_INT=0x";

const char pcAt_SrvRdStepAccThr[]			= "{STEP_ACC_THRES?}";				/* Acceleration and duration for STEP_DET. */
const char pcAt_SrvWrStepAccThr[]			= "{STEP_ACC_THRES=0x";
const char pcAt_SrvRdStepMinInt[]			= "{STEP_MIN_INTERVAL?}";
const char pcAt_SrvWrStepMinInt[]			= "{STEP_MIN_INTERVAL=0x";
const char pcAt_SrvRdStepsMaxInt[]			= "{STEP_MAX_INTERVAL?}";
const char pcAt_SrvWrStepMaxInt[]			= "{STEP_MAX_INTERVAL=0x";
const char pcAt_SrvRdActiveNumSteps[]		= "{ACTIVE_NUM_STEPS?}";
const char pcAt_SrvWrActiveNumSteps[]		= "{ACTIVE_NUM_STEPS=0x";

const char pcAt_SrvRdSosAccThr[]			= "{SOS_ACC_THRES?}";				/* Acceleration and duration for SOS_DET. */
const char pcAt_SrvWrSosAccThr[]			= "{SOS_ACC_THRES=0x";
const char pcAt_SrvRdSosMinXAccThr[]		= "{SOS_MIN_X_ACC_THRES?}";		
const char pcAt_SrvWrSosMinXAccThr[]		= "{SOS_MIN_X_ACC_THRES=0x";
const char pcAt_SrvRdSosMinZAccThr[]		= "{SOS_MIN_Z_ACC_THRES?}";		
const char pcAt_SrvWrSosMinZAccThr[]		= "{SOS_MIN_Z_ACC_THRES=0x";
const char pcAt_SrvRdSosMaxZAccThr[]		= "{SOS_MAX_Z_ACC_THRES?}";		
const char pcAt_SrvWrSosMaxZAccThr[]		= "{SOS_MAX_Z_ACC_THRES=0x";
const char pcAt_SrvRdSosMinInt[]			= "{SOS_MIN_INTERVAL?}";
const char pcAt_SrvWrSosMinInt[]			= "{SOS_MIN_INTERVAL=0x";
const char pcAt_SrvRdSosMaxInt[]			= "{SOS_MAX_INTERVAL?}";
const char pcAt_SrvWrSosMaxInt[]			= "{SOS_MAX_INTERVAL=0x";
const char pcAt_SrvRdSosNumPeaks[]			= "{SOS_NUM_PEAKS?}";
const char pcAt_SrvWrSosNumPeaks[]			= "{SOS_NUM_PEAKS=0x";

const char pcAt_SrvRdStillDur[]				= "{STILL_DURATION?}";
const char pcAt_SrvWrStillDur[]				= "{STILL_DURATION=0x";
const char pcAt_SrvRdImmobilityDur[]		= "{IMMOBILITY_DURATION?}";
const char pcAt_SrvWrImmobilityDur[]		= "{IMMOBILITY_DURATION=0x";
const char pcAt_SrvRdSleepDur[]				= "{SLEEP_DURATION?}";			
const char pcAt_SrvWrSleepDur[]				= "{SLEEP_DURATION=0x";

const char pcAt_SrvRdAbnPosZAccThr[]		= "{ABNORMAL_POS_Z_ACC_THRES?}";	/* Acceleration and duration for final abnormal position detection. */
const char pcAt_SrvWrAbnPosZAccThr[]		= "{ABNORMAL_POS_Z_ACC_THRES=0x";
const char pcAt_SrvRdAbnPosDur[]			= "{ABNORMAL_DURATION?}";
const char pcAt_SrvWrAbnPosDur[]			= "{ABNORMAL_DURATION=0x";
const char pcAt_SrvRdAbnMaxOrient[]			= "{ABNORMAL_MAX_ORIENT_CHG?}";		/* Tolerated maximum spatial orientation change. */
const char pcAt_SrvWrAbnMaxOrient[]			= "{ABNORMAL_MAX_ORIENT_CHG=0x";

const char pcAt_SrvRdAlertCnclMethod[]		= "{ALERT_CNCL_METHOD?}";			/* Method used for ALERT_CNCL_DET. */
const char pcAt_SrvWrAlertCnclMethod[]		= "{ALERT_CNCL_METHOD=0x";
const char pcAt_SrvRdAlertCnclThr[]			= "{ALERT_CNCL_ACC_THRES?}";		/* Acceleration and duration for ALERT_CNCL_DET. */
const char pcAt_SrvWrAlertCnclThr[]			= "{ALERT_CNCL_ACC_THRES=0x";
const char pcAt_SrvRdAlertCnclMinInt[]		= "{ALERT_CNCL_MIN_INTERVAL?}";
const char pcAt_SrvWrAlertCnclMinInt[]		= "{ALERT_CNCL_MIN_INTERVAL=0x";
const char pcAt_SrvRdAlertCnclMaxInt[]		= "{ALERT_CNCL_MAX_INTERVAL?}";
const char pcAt_SrvWrAlertCnclMaxInt[]		= "{ALERT_CNCL_MAX_INTERVAL=0x";
const char pcAt_SrvRdAlertCnclNumPeaks[]	= "{ALERT_CNCL_NUM_PEAKS?}";
const char pcAt_SrvWrAlertCnclNumPeaks[]	= "{ALERT_CNCL_NUM_PEAKS=0x";
const char pcAt_SrvRdAlertCnclTimeout[]		= "{ALERT_CNCL_TIMEOUT?}";
const char pcAt_SrvWrAlertCnclTimeout[]		= "{ALERT_CNCL_TIMEOUT=0x";
const char pcAt_SrvRdSosCnclTimeout[]		= "{SOS_CNCL_TIMEOUT?}";
const char pcAt_SrvWrSosCnclTimeout[]		= "{SOS_CNCL_TIMEOUT=0x";

const char pcAt_SrvRdNoAbnEn[]				= "{NOABN_ENABLE?}";				/* Command for disabling abnormal position (tilt) detection. */
const char pcAt_SrvWrNoAbnEn[]				= "{NOABN_ENABLE=0x";
const char pcAt_SrvRdNoAbnNumPeaks[]		= "{NOABN_NUM_PEAKS?}";
const char pcAt_SrvWrNoAbnNumPeaks[]		= "{NOABN_NUM_PEAKS=0x";
const char pcAt_SrvRdNoAbnMaxPeriod[]		= "{NOABN_MAX_PERIOD?}";
const char pcAt_SrvWrNoAbnMaxPeriod[]		= "{NOABN_MAX_PERIOD=0x";
const char pcAt_SrvRdNoAbnWindow[]			= "{NOABN_WINDOW?}";
const char pcAt_SrvWrNoAbnWindow[]			= "{NOABN_WINDOW=0x";
const char pcAt_SrvRdNoAbnCmdAckOn[]		= "{NOABN_CMDACK_ON?}";
const char pcAt_SrvWrNoAbnCmdAckOn[]		= "{NOABN_CMDACK_ON=0x";
const char pcAt_SrvRdNoAbnCmdAckOff[]		= "{NOABN_CMDACK_OFF?}";
const char pcAt_SrvWrNoAbnCmdAckOff[]		= "{NOABN_CMDACK_OFF=0x";
const char pcAt_SrvRdNoAbnCmdAckRep[]		= "{NOABN_CMDACK_REP?}";
const char pcAt_SrvWrNoAbnCmdAckRep[]		= "{NOABN_CMDACK_REP=0x";

const char pcAt_SrvRdBattEmptyThres[]		= "{BATT_EMPTY_THRES?}";			/* Battery empty threshold for power-safe mode. */	
const char pcAt_SrvWrBattEmptyThres[]		= "{BATT_EMPTY_THRES=0x";
const char pcAt_SrvRdBattLowThres[]			= "{BATT_LOW_THRES?}";				/* Battery low threshold. */	
const char pcAt_SrvWrBattLowThres[]			= "{BATT_LOW_THRES=0x";	
const char pcAt_SrvRdBattFullThres[]		= "{BATT_FULL_THRES?}";				/* Battery full threshold. */	
const char pcAt_SrvWrBattFullThres[]		= "{BATT_FULL_THRES=0x";
const char pcAt_SrvRdBattLoopInt[]			= "{BATT_LOOP_INTERVAL?}";
const char pcAt_SrvWrBattLoopInt[]			= "{BATT_LOOP_INTERVAL=0x";

const char pcAt_SrvRdGpsEnable[]			= "{GPS_ENABLE?}";
const char pcAt_SrvWrGpsEnable[]			= "{GPS_ENABLE=0x";
const char pcAt_SrvRdGpsFixQuality[]		= "{GPS_FIX_QUALITY?}";				/* Minimum HDOP (times 10) quality for an acceptable GPS fix. */
const char pcAt_SrvWrGpsFixQuality[]		= "{GPS_FIX_QUALITY=0x";
const char pcAt_SrvRdGpsMaxHAcc[]			= "{GPS_H_ACCURACY?}";				/* Maximum horizontal accuracy improvement within 2 seconds for an acceptable GPS fix. */
const char pcAt_SrvWrGpsMaxHAcc[]			= "{GPS_H_ACCURACY=0x";
const char pcAt_SrvRdMaxGpsWaitTime[]		= "{GPS_MAX_TIME_WAIT_FOR_FIX?}";	/* Maximum time waiting for a good fix before sending a position packet without position. */	
const char pcAt_SrvWrMaxGpsWaitTime[]		= "{GPS_MAX_TIME_WAIT_FOR_FIX=0x";
const char pcAt_SrvRdMaxGpsSatWaitTime[]	= "{GPS_MAX_TIME_WAIT_FOR_SAT?}";	/* Maximum time waiting for GPS satellites to be received before sending a position packet without position. */	
const char pcAt_SrvWrMaxGpsSatWaitTime[]	= "{GPS_MAX_TIME_WAIT_FOR_SAT=0x";
const char pcAt_SrvRdGpsPwrDwnInt[]			= "{GPS_PWRDWN_INT?}";
const char pcAt_SrvWrGpsPwrDwnInt[]			= "{GPS_PWRDWN_INT=0x";
const char pcAt_SrvRdGpsPsm[]				= "{GPS_PSM?}";
const char pcAt_SrvWrGpsPsm[]				= "{GPS_PSM=0x";
const char pcAt_SrvRdGpsLocEstSrv[]			= "{GPS_LOCESTSRV?}";
const char pcAt_SrvWrGpsLocEstSrv[]			= "{GPS_LOCESTSRV=";
const char pcAt_SrvRdGpsPosRec[]			= "{GPS_POSREC?}";
const char pcAt_SrvWrGpsPosRec[]			= "{GPS_POSREC=0x";
const char pcAt_SrvRdGpsPosRecInt[]			= "{GPS_POSREC_PKT_INT?}";
const char pcAt_SrvWrGpsPosRecInt[]			= "{GPS_POSREC_PKT_INT=0x";

const char pcAt_SrvRdBleEnable[]			= "{BLE_ENABLE?}";
const char pcAt_SrvWrBleEnable[]			= "{BLE_ENABLE=0x";
const char pcAt_SrvRdBleUpdInt[]			= "{BLE_UPDATE_INT?}";
const char pcAt_SrvWrBleUpdInt[]			= "{BLE_UPDATE_INT=0x";
const char pcAt_SrvWrBleKey[]				= "{BLE_KEY=0x";
const char pcAt_SrvRdBleDFMap[] 			= "{BLE_DFMAP?}";
const char pcAt_SrvWrBleDFMap[]				= "{BLE_DFMAP=0x";
const char pcAt_SrvRdBleTxPwr[] 			= "{BLE_TXPWR?}";
const char pcAt_SrvWrBleTxPwr[]				= "{BLE_TXPWR=0x";
const char pcAt_SrvRdBleRssiCfg[]			= "{BLE_RSSI_CFG?}";
const char pcAt_SrvWrBleRssiCfg[]			= "{BLE_RSSI_CFG=0x";
const char pcAt_SrvRdBleMinBcn[]			= "{BLE_MIN_BCN?}";
const char pcAt_SrvWrBleMinBcn[]			= "{BLE_MIN_BCN=0x";
const char pcAt_SrvRdBleInDangerDur[]		= "{BLE_IN_DANGER_DUR?}";
const char pcAt_SrvWrBleInDangerDur[]		= "{BLE_IN_DANGER_DUR=0x";
const char pcAt_SrvRdBleForeignAlert[]		= "{BLE_FOREIGN_DANGER?}";
const char pcAt_SrvWrBleForeignAlert[]		= "{BLE_FOREIGN_DANGER=0x";
const char pcAt_SrvRdBleScanFixOnly[]		= "{BLE_SCAN_FIX_ONLY?}";
const char pcAt_SrvWrBleScanFixOnly[]		= "{BLE_SCAN_FIX_ONLY=0x";
const char pcAt_SrvRdBleMaxWaitForBcn[]		= "{BLE_MAX_WAIT_FOR_BCN?}";
const char pcAt_SrvWrBleMaxWaitForBcn[]		= "{BLE_MAX_WAIT_FOR_BCN=0x";
const char pcAt_SrvRdBleStdAdvInt[]			= "{BLE_STD_ADV_INT?}";
const char pcAt_SrvWrBleStdAdvInt[]			= "{BLE_STD_ADV_INT=0x";
const char pcAt_SrvRdBleUuidFilter[]		= "{BLE_BCN_UUID?";
const char pcAt_SrvWrBleUuidFilter[]		= "{BLE_BCN_UUID=0x";
const char pcAt_SrvRdBleBcnFilter[]			= "{BLE_BCN_FILTER?";
const char pcAt_SrvWrBleBcnFilter[]			= "{BLE_BCN_FILTER=0x";

const char pcAt_SrvPeekRAM[] 				= "{PEEK_RAM=0x";
const char pcAt_SrvPokeRAM[] 				= "{POKE_RAM=0x";
const char pcAt_SrvPeekNVDS[] 				= "{PEEK_NVDS=0x";
const char pcAt_SrvPokeNVDS[] 				= "{POKE_NVDS=0x";
const char pcAt_SrvRdStats[]	 			= "{STATS?}";
const char pcAt_SrvResStats[]	 			= "{RESSTATS}";
const char pcAt_SrvVibrate[] 				= "{VIBRATE=0x";
const char pcAt_SrvLed[]	 				= "{LED=0x";

const char pcAt_SrvEvacuate[] 				= "{EVACUATE=0x";
const char pcAt_SrvEvacuationStop[]			= "{EVACUATION_STOP}";
const char pcAt_SrvRdEvacVibr[]				= "{EVAC_VIBR?}";
const char pcAt_SrvWrEvacVibr[]				= "{EVAC_VIBR=0x";

const char pcAt_SrvState[]	 				= "{STATE?}";
const char pcAt_SrvStandby[] 				= "{STANDBY_MODE}";
const char pcAt_SrvNormal[] 				= "{NORMAL_MODE}";
const char pcAt_SrvOoo[]		 			= "{OOO_MODE}";
const char pcAt_SrvAlertAck[] 				= "{ALERT_ACK}";
const char pcAt_SrvGotoPreAlert[]			= "{GOTO_PREALERT}";
const char pcAt_SrvTestStart[] 				= "{TEST_START}";
const char pcAt_SrvTestStop[] 				= "{TEST_STOP}";

const char pcAt_SrvFwUpd[]	 				= "{FWUPD=";
const char pcAt_SrvRdFtpSrv[]				= "{FTPSRV?}";
const char pcAt_SrvWrFtpSrv[]				= "{FTPSRV=";
const char pcAt_SrvRdFtpCfg[]				= "{FTP_CFG?}";
const char pcAt_SrvWrFtpCfg[]				= "{FTP_CFG=";

const char pcAt_SrvFsDir[]	 				= "{FS_LST=";
const char pcAt_SrvFsGet[]					= "{FS_GET=";
const char pcAt_SrvFsPut[]					= "{FS_PUT=";
const char pcAt_SrvFsDel[]					= "{FS_DEL=";

const char pcAt_SrvRdSecEnable[]			= "{SEC_ENABLE?}";
const char pcAt_SrvWrSecEnable[]			= "{SEC_ENABLE=0x";

const char pcAt_SrvRdLogEnable[]			= "{LOG_ENABLE?}";
const char pcAt_SrvWrLogEnable[]			= "{LOG_ENABLE=0x";

const char pcAt_SrvRdModuleSide[]			= "{MOD_SIDE?}";
const char pcAt_SrvWrModuleSide[]			= "{MOD_SIDE=0x";

const char pcAt_SrvWrRelayCmd[]				= "{RELAY_CMD=";

const char pcAt_SrvRdDangerBcnThres[] 		= "{DANGER_BCN_THRES?}";
const char pcAt_SrvWrDangerBcnThres[]		= "{DANGER_BCN_THRES=0x";
const char pcAt_SrvRdNoAbnBcnThres[]		= "{NOABN_BCN_THRES?}";
const char pcAt_SrvWrNoAbnBcnThres[]		= "{NOABN_BCN_THRES=0x";
const char pcAt_SrvRdPrivateBcnThres[]		= "{PRIVATE_BCN_THRES?}";
const char pcAt_SrvWrPrivateBcnThres[]		= "{PRIVATE_BCN_THRES=0x";
const char pcAt_SrvRdImmobilityBcnThres[]	= "{IMMOBILITY_BCN_THRES?}";
const char pcAt_SrvWrImmobilityBcnThres[]	= "{IMMOBILITY_BCN_THRES=0x";

const char pcAt_SrvOther[]	 				= "{";
	
/* Table containing pointers to the AT response strings, parameters to be stored
   (if any) and the AT response ID to be sent to the GSM task via queue. */
const struct xAT_RESP xAtResp[] = 
{
	 { pcAt_SendOk,					NULL,						AT_SEND_OK						},
	 { pcAt_Ok, 	 				prvClrStorage,				AT_OK 							},
	 { pcAt_CReg1, 					prvParseCREG,				AT_CGREG_REG 					},
	 { pcAt_CReg0, 					NULL,						AT_CGREG_ABAND					},
	 { pcAt_CReg5,	 				prvParseCREG,				AT_CGREG_REG 					},
	 { pcAt_CPSmsRes,				prvStoreRemoteArgs,			AT_NOMSG	 					},
	 { pcAt_QNwScanModeRes,			prvStoreRemoteArgs,			AT_NWCFG	 					},	
	 { pcAt_QNwScanSeqRes,			prvStoreRemoteArgs,			AT_NWCFG	 					},	
	 { pcAt_QIotOpModeRes,			prvStoreRemoteArgs,			AT_NWCFG	 					},	
	 { pcAt_CGActRes,				prvStoreRemoteArgs,			AT_NOMSG	 					},	
	 { pcAt_CGAttRes,				prvStoreRemoteArgs,			AT_NOMSG	 					},	
	 { pcAt_QBoot,					NULL,						AT_QBOOT						},
	 
	 { pcAt_SendFail,				NULL,						AT_SEND_FAIL					},
	 { pcAt_Connect,				prvStoreStrings,			AT_CONNECT						},	 
	 { pcAt_QHttpGetOK,				prvStoreRemoteArgs,			AT_HTTP_GET_OK					},
	 { pcAt_QHttpGetKO,				prvStoreRemoteArgs,			AT_HTTP_GET_KO					},		/* This check needs to come *after* the OK check. */
	 	 
	 { pcAt_QSSLOpen, 				prvParseSocId,				AT_USOCRID 						},
	 { pcAt_QSSLOpen2, 				prvParseSocId,				AT_USOCRID 						},
	 { pcAt_QIOpen, 				prvParseSocId,				AT_USOCRID 						},
	 { pcAt_QIOpen2, 				prvParseSocId,				AT_USOCRID 						},
	 { pcAt_QSSLUrcRx,				NULL,						AT_TCP_RX_DATA_IND				},
	 { pcAt_QIUrcRx,				NULL,						AT_TCP_RX_DATA_IND				},
	 { pcAt_QSSLUrcCl,				vUnsolicitedTcpSocketClose,	AT_USOCL						},
	 { pcAt_QIUrcCl,				vUnsolicitedTcpSocketClose,	AT_USOCL						},
	 { pcAt_QSSLState,				prvStoreRemoteArgs,			AT_NOMSG						},
	 { pcAt_QIState,				prvStoreRemoteArgs,			AT_NOMSG						},
	 { pcAt_QIError,				prvStoreRemoteArgs,			AT_NOMSG						},
	 { pcAt_QIURC,					prvStoreRemoteArgs,			AT_URC							},
	 
	 { pcAt_CSQ, 					prvParseSignalQuality,		AT_NOMSG	 					},
	 { pcAt_QNWINFO,				prvParseQNwInfo,			AT_NOMSG	 					},
	 { pcAt_CCID, 					prvParseCCID,				AT_NOMSG	 					},
	 { pcAt_GsmCGEvHwDetach,		vUnsolicitedPdpDeactivate,	AT_NOMSG						},
	 { pcAt_QPingRsp,				prvStoreRemoteArgs,			AT_PING							},

	 { pcAt_FOpen,		 			prvStoreRemoteArgs,			AT_NOMSG						},
	 { pcAt_FTPOpenOK,	 			NULL,						AT_FTP_OK						},
	 { pcAt_FTPOpenKO,	 			prvStoreRemoteArgs,			AT_FTP_KO						},
	 { pcAt_FTPCloseOK,	 			NULL,						AT_FTP_OK						},
	 { pcAt_FTPCloseKO,	 			prvStoreRemoteArgs,			AT_FTP_KO						},
	 { pcAt_FTPCwdOK,	 			NULL,						AT_FTP_OK						},
	 { pcAt_FTPCwdKO,	 			prvStoreRemoteArgs,			AT_FTP_KO						},
	 { pcAt_FTPGetOK,	 			prvParseQFTPGet,			AT_NOMSG						},
	 { pcAt_FTPGetKO,	 			prvStoreRemoteArgs,			AT_FTP_KO						},
	 { pcAt_FTPPutOK,	 			NULL,						AT_FTP_OK						},
	 { pcAt_FTPPutKO,	 			prvStoreRemoteArgs,			AT_FTP_KO						},
	 { pcAt_FTPER,		 			prvStoreRemoteArgs,			AT_FTP_STAT	 					},
	 { pcAt_LstFree,				prvStoreRemoteArgs,			AT_NOMSG						},
	 { pcAt_LstFile,				prvAppendRemoteArgs,		AT_NOMSG						},
	   
	 { pcAt_Error,					prvStoreRemoteArgs,			AT_ERROR						},
	 { pcAt_CMEError,				prvStoreRemoteArgs,			AT_ERROR						},
	 { pcAt_GsmCEER,				prvStoreRemoteArgs,			AT_NOMSG						},
	 { pcAt_AtEcho,					NULL,						AT_ATECHO						},	 
	 { pcAt_COps,					prvParseCOps,				AT_NOMSG						},

	 { pcAt_RemoteOK,				NULL,						AT_REMOTE_OK 					},
	 { pcAt_RemoteKO,				NULL,						AT_REMOTE_KO 					},
	                                                                                            
	 { pcAt_SrvRdVersion,			NULL,						SRVCMD_RD_VERSION				},		 
	 { pcAt_SrvReset,				prvStoreRemoteArgs,			SRVCMD_RESET 					},		 
	 { pcAt_SrvRdGPRSCfg,			NULL,						SRVCMD_RD_GPRSCFG				},		 
	 { pcAt_SrvWrGPRSCfg,			prvStoreRemoteArgs,			SRVCMD_WR_GPRSCFG				},		 
	 { pcAt_SrvRdTcpCfg,			NULL,						SRVCMD_RD_TCPCFG				},		 
	 { pcAt_SrvWrTcpCfg,			prvStoreRemoteArgs,			SRVCMD_WR_TCPCFG				},		 
	 { pcAt_SrvRdSecTcpCfg,			NULL,						SRVCMD_RD_SECTCPCFG				},		 
	 { pcAt_SrvWrSecTcpCfg,			prvStoreRemoteArgs,			SRVCMD_WR_SECTCPCFG				},		 
	 { pcAt_SrvRdDFMap,				NULL,						SRVCMD_RD_DFMAP					},		 
	 { pcAt_SrvWrDFMap,				prvStoreRemoteArgs,			SRVCMD_WR_DFMAP					},		 
	 { pcAt_SrvRdRat,				NULL,						SRVCMD_RD_RAT					},		 
	 { pcAt_SrvWrRat,				prvStoreRemoteArgs,			SRVCMD_WR_RAT					},		 
	 { pcAt_SrvRdAttTo,				NULL,						SRVCMD_RD_ATT_TO				},		 
	 { pcAt_SrvWrAttTo,				prvStoreRemoteArgs,			SRVCMD_WR_ATT_TO				},		 
	 { pcAt_SrvRdActInt,			NULL,						SRVCMD_RD_ACT_INT				},		 
	 { pcAt_SrvWrActInt,			prvStoreRemoteArgs,			SRVCMD_WR_ACT_INT				},		 
	 { pcAt_SrvRdInactInt,			NULL,						SRVCMD_RD_INACT_INT				},		 
	 { pcAt_SrvWrInactInt,			prvStoreRemoteArgs,			SRVCMD_WR_INACT_INT				},		 
	 { pcAt_SrvRdStillInt,			NULL,						SRVCMD_RD_STILL_INT				},		 
	 { pcAt_SrvWrStillInt,			prvStoreRemoteArgs,			SRVCMD_WR_STILL_INT				},		 
	 { pcAt_SrvRdSleepInt,			NULL,						SRVCMD_RD_SLEEP_INT				},		 
	 { pcAt_SrvWrSleepInt,			prvStoreRemoteArgs,			SRVCMD_WR_SLEEP_INT				},		 
	 { pcAt_SrvRdOooInt,			NULL,						SRVCMD_RD_OOO_INT				},		 
	 { pcAt_SrvWrOooInt,			prvStoreRemoteArgs,			SRVCMD_WR_OOO_INT				},		 
	 { pcAt_SrvRdChrgInt,			NULL,						SRVCMD_RD_CHRG_INT				},		 
	 { pcAt_SrvWrChrgInt,			prvStoreRemoteArgs,			SRVCMD_WR_CHRG_INT				},		 
	 { pcAt_SrvRdAlertInt,			NULL,						SRVCMD_RD_ALERT_INT				},		 
	 { pcAt_SrvWrAlertInt,			prvStoreRemoteArgs,			SRVCMD_WR_ALERT_INT				},		 
	 { pcAt_SrvRdPwrDwnInt,			NULL,						SRVCMD_RD_PD_INT				},		 
	 { pcAt_SrvWrPwrDwnInt,			prvStoreRemoteArgs,			SRVCMD_WR_PD_INT				},		
	                                                                                            
	 { pcAt_SrvRdStepAccThr,		NULL,						SRVCMD_RD_STEP_ACC_THR			},
	 { pcAt_SrvWrStepAccThr,		prvStoreRemoteArgs,			SRVCMD_WR_STEP_ACC_THR  	  	},
	 { pcAt_SrvRdStepMinInt,		NULL,						SRVCMD_RD_STEP_MIN_INT    		},
	 { pcAt_SrvWrStepMinInt,		prvStoreRemoteArgs,			SRVCMD_WR_STEP_MIN_INT    		},
	 { pcAt_SrvRdStepsMaxInt,		NULL,						SRVCMD_RD_STEP_MAX_INT    		},
	 { pcAt_SrvWrStepMaxInt,		prvStoreRemoteArgs,			SRVCMD_WR_STEP_MAX_INT    		},
	 { pcAt_SrvRdActiveNumSteps,	NULL,						SRVCMD_RD_ACTIVE_NUM_STEPS  	},
	 { pcAt_SrvWrActiveNumSteps,	prvStoreRemoteArgs,			SRVCMD_WR_ACTIVE_NUM_STEPS  	},
	                                                                                            
	 { pcAt_SrvRdSosAccThr,			NULL,						SRVCMD_RD_SOS_ACC_THR			},
	 { pcAt_SrvWrSosAccThr,			prvStoreRemoteArgs,			SRVCMD_WR_SOS_ACC_THR   		},
	 { pcAt_SrvRdSosMinXAccThr,  	NULL,						SRVCMD_RD_SOS_MIN_X_ACC_THR		},
	 { pcAt_SrvWrSosMinXAccThr,  	prvStoreRemoteArgs,			SRVCMD_WR_SOS_MIN_X_ACC_THR   	},
	 { pcAt_SrvRdSosMinZAccThr,  	NULL,						SRVCMD_RD_SOS_MIN_Z_ACC_THR		},
	 { pcAt_SrvWrSosMinZAccThr,  	prvStoreRemoteArgs,			SRVCMD_WR_SOS_MIN_Z_ACC_THR   	},
	 { pcAt_SrvRdSosMaxZAccThr,  	NULL,						SRVCMD_RD_SOS_MAX_Z_ACC_THR		},
	 { pcAt_SrvWrSosMaxZAccThr,  	prvStoreRemoteArgs,			SRVCMD_WR_SOS_MAX_Z_ACC_THR   	},
	 { pcAt_SrvRdSosMinInt,			NULL,						SRVCMD_RD_SOS_MIN_INT   		},
	 { pcAt_SrvWrSosMinInt,			prvStoreRemoteArgs,			SRVCMD_WR_SOS_MIN_INT   		},
	 { pcAt_SrvRdSosMaxInt,			NULL,						SRVCMD_RD_SOS_MAX_INT   		},
	 { pcAt_SrvWrSosMaxInt,			prvStoreRemoteArgs,			SRVCMD_WR_SOS_MAX_INT   		},
	 { pcAt_SrvRdSosNumPeaks,		NULL,						SRVCMD_RD_SOS_NUM_PEAKS 		},
	 { pcAt_SrvWrSosNumPeaks,		prvStoreRemoteArgs,			SRVCMD_WR_SOS_NUM_PEAKS 		},
	                                                                                            
	 { pcAt_SrvRdStillDur,			NULL,						SRVCMD_RD_STILL_DUR         	},
	 { pcAt_SrvWrStillDur,			prvStoreRemoteArgs,			SRVCMD_WR_STILL_DUR  			},       	
	 { pcAt_SrvRdImmobilityDur,		NULL,						SRVCMD_RD_IMMOBILITY_DUR        },
	 { pcAt_SrvWrImmobilityDur,		prvStoreRemoteArgs,			SRVCMD_WR_IMMOBILITY_DUR  		},  
	 { pcAt_SrvRdSleepDur,			NULL,						SRVCMD_RD_SLEEP_DUR 		   	},
	 { pcAt_SrvWrSleepDur,			prvStoreRemoteArgs,			SRVCMD_WR_SLEEP_DUR		      	},
	                                                                                            
	 { pcAt_SrvRdAbnPosZAccThr,		NULL,						SRVCMD_RD_ABNPOS_Z_ACC_THR  	},
	 { pcAt_SrvWrAbnPosZAccThr,		prvStoreRemoteArgs,			SRVCMD_WR_ABNPOS_Z_ACC_THR		},
	 { pcAt_SrvRdAbnPosDur,			NULL,						SRVCMD_RD_ABNPOS_DUR           	},
	 { pcAt_SrvWrAbnPosDur,			prvStoreRemoteArgs,			SRVCMD_WR_ABNPOS_DUR           	},
	 { pcAt_SrvRdAbnMaxOrient,		NULL,						SRVCMD_RD_ABNPOS_MAX_ORIENT		},
	 { pcAt_SrvWrAbnMaxOrient,		prvStoreRemoteArgs,			SRVCMD_WR_ABNPOS_MAX_ORIENT		},			
	                                                                                            
	 { pcAt_SrvRdAlertCnclMethod,	NULL,						SRVCMD_RD_ALERT_CNCL_METHOD	    },
	 { pcAt_SrvWrAlertCnclMethod,	prvStoreRemoteArgs,			SRVCMD_WR_ALERT_CNCL_METHOD 	},
	 { pcAt_SrvRdAlertCnclThr,		NULL,						SRVCMD_RD_ALERT_CNCL_THR    	},
	 { pcAt_SrvWrAlertCnclThr,		prvStoreRemoteArgs,			SRVCMD_WR_ALERT_CNCL_THR    	},
	 { pcAt_SrvRdAlertCnclMinInt,	NULL,						SRVCMD_RD_ALERT_CNCL_MIN_INT	},
	 { pcAt_SrvWrAlertCnclMinInt,	prvStoreRemoteArgs,			SRVCMD_WR_ALERT_CNCL_MIN_INT	},
	 { pcAt_SrvRdAlertCnclMaxInt,	NULL,						SRVCMD_RD_ALERT_CNCL_MAX_INT	},
	 { pcAt_SrvWrAlertCnclMaxInt,	prvStoreRemoteArgs,			SRVCMD_WR_ALERT_CNCL_MAX_INT	},
	 { pcAt_SrvRdAlertCnclNumPeaks,	NULL,						SRVCMD_RD_ALERT_CNCL_NUM_PEAKS	},
	 { pcAt_SrvWrAlertCnclNumPeaks,	prvStoreRemoteArgs,			SRVCMD_WR_ALERT_CNCL_NUM_PEAKS  },
	 { pcAt_SrvRdAlertCnclTimeout,	NULL,						SRVCMD_RD_ALERT_CNCL_TIMEOUT    },
	 { pcAt_SrvWrAlertCnclTimeout,	prvStoreRemoteArgs,			SRVCMD_WR_ALERT_CNCL_TIMEOUT    },
	 { pcAt_SrvRdSosCnclTimeout,	NULL,						SRVCMD_RD_SOS_CNCL_TIMEOUT    	},
	 { pcAt_SrvWrSosCnclTimeout,	prvStoreRemoteArgs,			SRVCMD_WR_SOS_CNCL_TIMEOUT    	},
	 
	 { pcAt_SrvRdNoAbnEn,			NULL,						SRVCMD_RD_NOABN_ENABLE			},
	 { pcAt_SrvWrNoAbnEn,			prvStoreRemoteArgs,			SRVCMD_WR_NOABN_ENABLE			},
	 { pcAt_SrvRdNoAbnNumPeaks,     NULL,						SRVCMD_RD_NOABN_NUM_PEAKS		},
	 { pcAt_SrvWrNoAbnNumPeaks,	    prvStoreRemoteArgs,			SRVCMD_WR_NOABN_NUM_PEAKS		},
	 { pcAt_SrvRdNoAbnMaxPeriod,	NULL,						SRVCMD_RD_NOABN_MAX_PERIOD		}, 
	 { pcAt_SrvWrNoAbnMaxPeriod,    prvStoreRemoteArgs,			SRVCMD_WR_NOABN_MAX_PERIOD		},
	 { pcAt_SrvRdNoAbnWindow,		NULL,						SRVCMD_RD_NOABN_WINDOW			}, 
	 { pcAt_SrvWrNoAbnWindow,		prvStoreRemoteArgs,			SRVCMD_WR_NOABN_WINDOW			}, 
	 { pcAt_SrvRdNoAbnCmdAckOn,	    NULL,						SRVCMD_RD_NOABN_ACK_ON			},
	 { pcAt_SrvWrNoAbnCmdAckOn,	    prvStoreRemoteArgs,			SRVCMD_WR_NOABN_ACK_ON			},
	 { pcAt_SrvRdNoAbnCmdAckOff,	NULL,						SRVCMD_RD_NOABN_ACK_OFF			}, 
	 { pcAt_SrvWrNoAbnCmdAckOff,    prvStoreRemoteArgs,			SRVCMD_WR_NOABN_ACK_OFF			},
	 { pcAt_SrvRdNoAbnCmdAckRep,	NULL,						SRVCMD_RD_NOABN_ACK_REP			}, 
	 { pcAt_SrvWrNoAbnCmdAckRep,	prvStoreRemoteArgs,			SRVCMD_WR_NOABN_ACK_REP			}, 	 
	 
	 { pcAt_SrvRdBattEmptyThres,	NULL,						SRVCMD_RD_BATT_EMPTY_THRES	    },
	 { pcAt_SrvWrBattEmptyThres,	prvStoreRemoteArgs,			SRVCMD_WR_BATT_EMPTY_THRES	    },
	 { pcAt_SrvRdBattLowThres,		NULL,						SRVCMD_RD_BATT_LOW_THRES	    },
	 { pcAt_SrvWrBattLowThres,		prvStoreRemoteArgs,			SRVCMD_WR_BATT_LOW_THRES	    },
	 { pcAt_SrvRdBattFullThres,		NULL,						SRVCMD_RD_BATT_FULL_THRES	    },
	 { pcAt_SrvWrBattFullThres,		prvStoreRemoteArgs,			SRVCMD_WR_BATT_FULL_THRES	    },
	 { pcAt_SrvRdBattLoopInt,		NULL,						SRVCMD_RD_BATT_LOOP_INT		    },
	 { pcAt_SrvWrBattLoopInt,		prvStoreRemoteArgs,			SRVCMD_WR_BATT_LOOP_INT		    },

	 { pcAt_SrvRdGpsEnable,			NULL,						SRVCMD_RD_GPS_ENABLE	    	},
	 { pcAt_SrvWrGpsEnable,			prvStoreRemoteArgs,			SRVCMD_WR_GPS_ENABLE	    	},
	 { pcAt_SrvRdGpsFixQuality,		NULL,						SRVCMD_RD_GPS_FIX_QUALITY	    },
	 { pcAt_SrvWrGpsFixQuality,		prvStoreRemoteArgs,			SRVCMD_WR_GPS_FIX_QUALITY	    },
	 { pcAt_SrvRdGpsMaxHAcc,		NULL,						SRVCMD_RD_GPS_MAX_HACC		    },
	 { pcAt_SrvWrGpsMaxHAcc,		prvStoreRemoteArgs,			SRVCMD_WR_GPS_MAX_HACC		    },	 
	 { pcAt_SrvRdMaxGpsWaitTime,	NULL,						SRVCMD_RD_GPS_WAIT_FOR_FIX	    },
	 { pcAt_SrvWrMaxGpsWaitTime,	prvStoreRemoteArgs,			SRVCMD_WR_GPS_WAIT_FOR_FIX	    },
	 { pcAt_SrvRdMaxGpsSatWaitTime,	NULL,						SRVCMD_RD_GPS_WAIT_FOR_SAT	    },
	 { pcAt_SrvWrMaxGpsSatWaitTime,	prvStoreRemoteArgs,			SRVCMD_WR_GPS_WAIT_FOR_SAT	    },
	 { pcAt_SrvRdGpsPwrDwnInt,		NULL,						SRVCMD_RD_GPS_PD_INT			},		 
	 { pcAt_SrvWrGpsPwrDwnInt,		prvStoreRemoteArgs,			SRVCMD_WR_GPS_PD_INT			},		
	 { pcAt_SrvRdGpsPsm,			NULL,						SRVCMD_RD_GPS_PSM				},		 
	 { pcAt_SrvWrGpsPsm,			prvStoreRemoteArgs,			SRVCMD_WR_GPS_PSM				},		
	 { pcAt_SrvRdGpsLocEstSrv,		NULL,						SRVCMD_RD_GPS_LOCEST_SRV		},		 
	 { pcAt_SrvWrGpsLocEstSrv,		prvStoreRemoteArgs,			SRVCMD_WR_GPS_LOCEST_SRV		},
	 { pcAt_SrvRdGpsPosRec,			NULL,						SRVCMD_RD_GPS_POSREC			},		 
	 { pcAt_SrvWrGpsPosRec,			prvStoreRemoteArgs,			SRVCMD_WR_GPS_POSREC			},
	 { pcAt_SrvRdGpsPosRecInt,		NULL,						SRVCMD_RD_GPS_POSREC_INT		},		 
	 { pcAt_SrvWrGpsPosRecInt,		prvStoreRemoteArgs,			SRVCMD_WR_GPS_POSREC_INT		},
	 
	 { pcAt_SrvRdBleEnable,			NULL,						SRVCMD_RD_BLE_ENABLE	    	},
	 { pcAt_SrvWrBleEnable,			prvStoreRemoteArgs,			SRVCMD_WR_BLE_ENABLE	    	},
	 { pcAt_SrvRdBleUpdInt,			NULL,						SRVCMD_RD_BLE_UPD_INT	    	},
	 { pcAt_SrvWrBleUpdInt,			prvStoreRemoteArgs,			SRVCMD_WR_BLE_UPD_INT  		 	},
	 { pcAt_SrvWrBleKey,			prvStoreRemoteArgs,			SRVCMD_WR_BLE_KEY	   		 	},
	 { pcAt_SrvRdBleDFMap,			NULL,						SRVCMD_RD_BLE_DFMAP				},		 
	 { pcAt_SrvWrBleDFMap,			prvStoreRemoteArgs,			SRVCMD_WR_BLE_DFMAP				},		 
	 { pcAt_SrvRdBleTxPwr,			NULL,						SRVCMD_RD_BLE_TXPWR				},		 
	 { pcAt_SrvWrBleTxPwr,			prvStoreRemoteArgs,			SRVCMD_WR_BLE_TXPWR				},		 
	 { pcAt_SrvRdBleRssiCfg,		NULL,						SRVCMD_RD_BLE_RSSI_CFG			},		 
	 { pcAt_SrvWrBleRssiCfg,		prvStoreRemoteArgs,			SRVCMD_WR_BLE_RSSI_CFG			},		 
	 { pcAt_SrvRdBleMinBcn,			NULL,						SRVCMD_RD_BLE_MIN_BCN			},		 
	 { pcAt_SrvWrBleMinBcn,			prvStoreRemoteArgs,			SRVCMD_WR_BLE_MIN_BCN			},
	 { pcAt_SrvRdBleInDangerDur,	NULL,						SRVCMD_RD_BLE_IN_DANGER_DUR		},		 
	 { pcAt_SrvWrBleInDangerDur,	prvStoreRemoteArgs,			SRVCMD_WR_BLE_IN_DANGER_DUR		},		 
	 { pcAt_SrvRdBleForeignAlert,	NULL,						SRVCMD_RD_BLE_FOREIGN_ALERT		},		 
	 { pcAt_SrvWrBleForeignAlert,	prvStoreRemoteArgs,			SRVCMD_WR_BLE_FOREIGN_ALERT		},		 
	 { pcAt_SrvRdBleScanFixOnly,	NULL,						SRVCMD_RD_BLE_SCAN_FIX_ONLY		},		 
	 { pcAt_SrvWrBleScanFixOnly,	prvStoreRemoteArgs,			SRVCMD_WR_BLE_SCAN_FIX_ONLY		},		 	 
	 { pcAt_SrvRdBleMaxWaitForBcn,	NULL,						SRVCMD_RD_BLE_MAX_WAIT_BCN		},		 
	 { pcAt_SrvWrBleMaxWaitForBcn,	prvStoreRemoteArgs,			SRVCMD_WR_BLE_MAX_WAIT_BCN		},		 	 
	 { pcAt_SrvRdBleStdAdvInt,		NULL,						SRVCMD_RD_BLE_STD_ADV_INT		},		 
	 { pcAt_SrvWrBleStdAdvInt,		prvStoreRemoteArgs,			SRVCMD_WR_BLE_STD_ADV_INT		},
	 { pcAt_SrvRdBleUuidFilter,		NULL,						SRVCMD_RD_BLE_UUID_FILTER		},		 
	 { pcAt_SrvWrBleUuidFilter,		prvStoreRemoteArgs,			SRVCMD_WR_BLE_UUID_FILTER		},
	 { pcAt_SrvRdBleBcnFilter,		NULL,						SRVCMD_RD_BLE_BCN_FILTER		},		 
	 { pcAt_SrvWrBleBcnFilter,		prvStoreRemoteArgs,			SRVCMD_WR_BLE_BCN_FILTER		},

	 { pcAt_SrvPeekRAM,				prvStoreRemoteArgs,			SRVCMD_PEEK_RAM					},		 
	 { pcAt_SrvPokeRAM,				prvStoreRemoteArgs,			SRVCMD_POKE_RAM 				},		 
	 { pcAt_SrvPeekNVDS,			prvStoreRemoteArgs,			SRVCMD_PEEK_NVDS				},
	 { pcAt_SrvPokeNVDS,			prvStoreRemoteArgs,			SRVCMD_POKE_NVDS				},
	 { pcAt_SrvRdStats,				NULL,						SRVCMD_RDSTATS					},
	 { pcAt_SrvResStats,			NULL,						SRVCMD_RESSTATS					},
	 { pcAt_SrvVibrate,				prvStoreRemoteArgs,			SRVCMD_VIBRATE					},
	 { pcAt_SrvLed,					prvStoreRemoteArgs,			SRVCMD_LED						},
	 
	 { pcAt_SrvEvacuate,			prvStoreRemoteArgs,			SRVCMD_EVACUATE					},	 
	 { pcAt_SrvEvacuationStop,		NULL,						SRVCMD_EVACUATION_STOP			},	 
	 { pcAt_SrvRdEvacVibr,			NULL,						SRVCMD_RD_EVAC_VIBR		    	},
	 { pcAt_SrvWrEvacVibr,			prvStoreRemoteArgs,			SRVCMD_WR_EVAC_VIBR	   		 	},
	 	 
	 { pcAt_SrvState,				NULL,						SRVCMD_STATE					},
	 { pcAt_SrvStandby,				NULL,						SRVCMD_STANDBY_MODE				},
	 { pcAt_SrvNormal,				NULL,						SRVCMD_NORMAL_MODE				},
	 { pcAt_SrvOoo,					NULL,						SRVCMD_OOO_MODE					},
	 { pcAt_SrvAlertAck,			NULL,						SRVCMD_ALERT_ACK				},
	 { pcAt_SrvGotoPreAlert,		NULL,						SRVCMD_GOTO_PREALERT			},
	 { pcAt_SrvTestStart,			NULL,						SRVCMD_TEST_START				},
	 { pcAt_SrvTestStop,			NULL,						SRVCMD_TEST_STOP				},
	 
	 { pcAt_SrvWrFtpSrv,			prvStoreRemoteArgs,			SRVCMD_WR_FTPSRV				},
	 { pcAt_SrvRdFtpSrv,			NULL,						SRVCMD_RD_FTPSRV				},
	 { pcAt_SrvWrFtpCfg,			prvStoreRemoteArgs,			SRVCMD_WR_FTP_CFG				},
	 { pcAt_SrvRdFtpCfg,			NULL,						SRVCMD_RD_FTP_CFG				},

	 { pcAt_SrvFwUpd,				prvStoreRemoteArgs,			SRVCMD_FWUPD					},
	 
	 { pcAt_SrvFsDir,				prvStoreRemoteArgs,			SRVCMD_FS_LST					},
	 { pcAt_SrvFsGet,				prvStoreRemoteArgs,			SRVCMD_FS_GET					},
	 { pcAt_SrvFsPut,				prvStoreRemoteArgs,			SRVCMD_FS_PUT					},
	 { pcAt_SrvFsDel,				prvStoreRemoteArgs,			SRVCMD_FS_DEL					},

	 { pcAt_SrvRdSecEnable,			NULL,						SRVCMD_RD_SEC_ENABLE	    	},
	 { pcAt_SrvWrSecEnable,			prvStoreRemoteArgs,			SRVCMD_WR_SEC_ENABLE	    	},

 	 { pcAt_SrvRdLogEnable,			NULL,						SRVCMD_RD_LOG_ENABLE	    	},
	 { pcAt_SrvWrLogEnable,			prvStoreRemoteArgs,			SRVCMD_WR_LOG_ENABLE	    	},

 	 { pcAt_SrvRdModuleSide,		NULL,						SRVCMD_RD_MOD_SIDE		    	},
	 { pcAt_SrvWrModuleSide,		prvStoreRemoteArgs,			SRVCMD_WR_MOD_SIDE		    	},

	 { pcAt_SrvWrRelayCmd,			prvStoreRemoteArgs,			SRVCMD_WR_RELAY_CMD		    	},
	 
	 { pcAt_SrvRdDangerBcnThres,	NULL,						SRVCMD_RD_DANGER_BCN_THRES		},		 
	 { pcAt_SrvWrDangerBcnThres,	prvStoreRemoteArgs,			SRVCMD_WR_DANGER_BCN_THRES		},		
	 { pcAt_SrvRdNoAbnBcnThres,		NULL,						SRVCMD_RD_NOABN_BCN_THRES		},		 
	 { pcAt_SrvWrNoAbnBcnThres,		prvStoreRemoteArgs,			SRVCMD_WR_NOABN_BCN_THRES		},		
	 { pcAt_SrvRdPrivateBcnThres,	NULL,						SRVCMD_RD_PRIVATE_BCN_THRES		},		 
	 { pcAt_SrvWrPrivateBcnThres,	prvStoreRemoteArgs,			SRVCMD_WR_PRIVATE_BCN_THRES		},		
	 { pcAt_SrvRdImmobilityBcnThres,NULL,						SRVCMD_RD_IMMOBILITY_BCN_THRES	},		 
	 { pcAt_SrvWrImmobilityBcnThres,prvStoreRemoteArgs,			SRVCMD_WR_IMMOBILITY_BCN_THRES	},
	 
	 { pcAt_SrvOther,				NULL,						SRVCMD_OTHER					}	 
};

const size_t xNumAtResp = sizeof( xAtResp ) / sizeof( xAtResp[ 0 ] );
/*-----------------------------------------------------------*/

void vParserInit( UBaseType_t uxPriority )
{
	/* Create a queue to the GSM task for AT response indications. */
	xParserAtRespQueue = xQueueCreate( parserAT_RESP_QUEUE_SIZE, ( unsigned portBASE_TYPE ) sizeof( enum xAT_MSG_ID ) );
	
	/* The Parser task is spawned here. */
	xTaskCreate( vParserTask, "PAR", parserSTACK_SIZE, NULL, uxPriority, &xParserTaskHandle );

	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* Extract the TCP socket ID from the data from the +QIOPEN or +QSSLOPEN response. 
   Note that the socket ID ranges from 0 to 11. 
   usStrgIdx points to the next offset *behind* the string.
   Possible returns are:
		 +QIOPEN: 4,0
		 +QIOPEN: 11,0
*/
static void prvParseSocId( unsigned short usStrgIdx )
{
	signed char				cChar;
	
	usStrgIdx -= 4;

	*( cGsmTcpSocketId + 0 ) = cGetRxCharFromBufferWithIndex( COM_GSM, usStrgIdx++ );
	if ( *( cGsmTcpSocketId + 0 ) == ' ' )
	{
		*( cGsmTcpSocketId + 0 ) = cGetRxCharFromBufferWithIndex( COM_GSM, usStrgIdx++ );
	}

	cChar = cGetRxCharFromBufferWithIndex( COM_GSM, usStrgIdx );
	if ( ( cChar >= '0' ) && ( cChar <= '9' ) )
	{
		*( cGsmTcpSocketId + 1 ) = cChar;
		usStrgIdx++;
	}
	else
	{
		*( cGsmTcpSocketId + 1 ) = 0;
	}
	
	*( cGsmTcpSocketId + 2 ) = 0;
}
/*-----------------------------------------------------------*/

/* Locate a data field by number in a comma-separated string in the UART buffer and copy the field to a specified destination.
   Stop at uxMaxCnt characters. Append a 0x00 byte. The first field has field number uxFieldNum = 0.
   Returns the number of characters copied to the destination.
*/
static unsigned portBASE_TYPE prvCopyFromUARTBuffer( unsigned short usStrgIdx, unsigned portBASE_TYPE uxFieldNum, unsigned portBASE_TYPE uxOffset, signed char *pcDest, unsigned portBASE_TYPE uxMaxCnt )
{
	unsigned portBASE_TYPE	uxCnt;
	signed char				cChar;
	
	/* Locate the uxFieldNum-th comma-separated data field, i.e. find and skip uxFieldNum commas. */
	for ( uxCnt = 0; uxCnt < uxFieldNum; uxCnt++ )
	{
		cChar = 0;
		
		while (   ( ( cChar = cGetRxCharFromBufferWithIndex( COM_GSM, usStrgIdx++ ) ) != ',' ) 
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
	usStrgIdx += uxOffset;
	
	/* Copy the data field. */
	uxCnt = 0;
	
	while (   ( uxCnt < uxMaxCnt ) 
		   && ( ( cChar = cGetRxCharFromBufferWithIndex( COM_GSM, usStrgIdx++ ) ) != ',' )
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

/* Parse +CEREG and +CGREG messages.
   Format: +CEREG: 2,<stat>[,[<tac>],[<ci>],[<AcT>[,<cause_type>,<reject_cause>]]]
		   +CGREG: 2,<stat>[,[<lac>],[<ci>],[<AcT>],[<rac>][,<cause_type>,<reject_cause>]]
   
   Interpretation:
		<stat>			(E)PS registration status
							0: not registered, the MT is not currently searching an operator to register to
							1: registered, home network
							2: not registered, but MT is currently searching a new operator to register to
							3: registration denied
							4: unknown (e.g. out of GERAN/UTRAN coverage)
							5: registered, roaming
		<tac>			Two bytes tracking area code in hexadecimal format.
		<lac> 			Two bytes location area in hexadecimal format; it is optionally provided in the URC
						and in the response to the read command.
		<ci>			Two to four bytes GERAN/E-UTRAN cell-id in hexadecimal format.
		<AcT>			Access technology of the serving cell.
							0: GSM
							8: E-UTRAN (M1)
							9: E-UTRAN (NB-S1 mode)
							255: the current <AcT> value is invalid
		<rac>			One byte routing area code.
		<cause_type>	<reject_cause> type.
		<reject_cause>	Cause of the failed registration.
		
	Example:
		+CGREG: 2,5,"93","E08B",3
*/
static void prvParseCREG( unsigned short usStrgIdx )
{
	signed char					cNumberBuffer[ 9 ];
	signed char					cChar;
	unsigned portBASE_TYPE		uxIdx;
	
	/* Copy LAC/TAC to cGsm_TAC. Take into account that the value is enclosed by double quotes. The maximum length is 4 characters. */
	prvCopyFromUARTBuffer( usStrgIdx, 1, 1, cNumberBuffer, LEN_GSM_TAC );
	
	uxIdx = 0;
	cChar = cNumberBuffer[ uxIdx ];
	while (    ( uxIdx < LEN_GSM_TAC )    
		    && ( cChar != 0 )
		    && ( cChar != '\"' ) )
	{
		cGsm_TAC[ uxIdx++ ] = cChar;
		cChar = cNumberBuffer[ uxIdx ];
	}
	cGsm_TAC[ uxIdx ] = 0;
	strncpy( ( char * )cGsm_BLE_TAC, ( char * )cGsm_TAC, LEN_GSM_TAC );
	
	/* Copy CI to cGsm_CI. */
	prvCopyFromUARTBuffer( usStrgIdx, 2, 1, cNumberBuffer, LEN_GSM_CI );

	uxIdx = 0;
	cChar = cNumberBuffer[ uxIdx ];
	while (    ( uxIdx < LEN_GSM_CI )    
		    && ( cChar != 0 )
		    && ( cChar != '\"' ) )
	{
		cGsm_CI[ uxIdx++ ] = cChar;
		cChar = cNumberBuffer[ uxIdx ];
	}
	cGsm_CI[ uxIdx ] = 0;
	strncpy( ( char * )cGsm_BLE_CI, ( char * )cGsm_CI, LEN_GSM_CI );

	/* Copy RAT to cGsm_RAT of not already done elsewhere. */
	if ( cGsm_RAT[ 0 ] == 0 ) 
	{
		prvCopyFromUARTBuffer( usStrgIdx, 3, 0, cGsm_RAT, LEN_GSM_RAT );
		/* Change type from '0' (GSM) to '3' (GSM EDGE) and '8' to '7' to be compatible with SARA. */
		if ( cGsm_RAT[ 0 ] == '0' )
		{
			cGsm_RAT[ 0 ] = '3';
		}
		if ( cGsm_RAT[ 0 ] == '8' )
		{
			cGsm_RAT[ 0 ] = '7';
		}
		cGsm_BLE_RAT[ 0 ] = cGsm_RAT[ 0 ];
		cGsm_BLE_RAT[ 1 ] = 0;
	}
}
/*-----------------------------------------------------------*/

/* Store the ICCID in the NVDS. */
static void prvParseCCID( unsigned short  usStrgIdx )
{
	unsigned portBASE_TYPE		uxIdx;
	bool						bIdDifferent;
	
	/* Check, if the ICCID we just received is the same one as we have already stored in NVDS. */
	bIdDifferent = false;
	for ( uxIdx = 0; uxIdx < LEN_GSM_ICCID; uxIdx++ )
	{
		if ( cGetRxCharFromBufferWithIndex( COM_GSM, usStrgIdx + uxIdx ) != ucConfigReadByte( ( uint8_t * )xNvdsConfig.pcGsm_ICCID + uxIdx ) )
		{
			bIdDifferent = true;
		}
	}

	/* If the copy in NVDS is different from the one just received, overwrite it with the received one. */
	if ( bIdDifferent )
	{
		signed char			cBuffer_ICCID[ LEN_GSM_ICCID + 1 ];
		
		for ( uxIdx = 0; uxIdx < LEN_GSM_ICCID; uxIdx++ )
		{
			cBuffer_ICCID[ uxIdx ] = cGetRxCharFromBufferWithIndex( COM_GSM, usStrgIdx + uxIdx );
		}
		cBuffer_ICCID[ LEN_GSM_ICCID ] = 0;
		vConfigWriteString( ( uint8_t * )xNvdsConfig.pcGsm_ICCID, ( uint8_t * )cBuffer_ICCID );
	}
}
/*-----------------------------------------------------------*/

/* Extract the received signal quality and RSSI from the +CESQ response to the AT+CESQ request.
   Format: +CSQ: <rxlev>,<ber>
   
    Interpretation:
		<rxlev> 	Received Signal Strength Indication (RSSI) - GSM only.
						0: 			less than -113 dBm
						1..30: 		from -111 to -53 dBm with 2 dBm steps
						31: 		-51 dBm or greater
						99: 		not known or not detectable
		<ber> 		Bit Error Rate (BER) - GSM only:
						0..7: 		as RXQUAL values in the table in 3GPP TS 45.008 [125], subclause 8.2.4
						99: 		not known or not detectable    
						
	Example:
		+CSQ: 28,99
*/
static void prvParseSignalQuality( unsigned short usStrgIdx )
{
	signed char					cNumberBuffer[ 4 ];
	
	cNumberBuffer[ 0 ] = 0;
	
	/* Get the RSSI. No need to keep the volatile here as parser is the highest-priority task to access this variable. */
	( void )prvCopyFromUARTBuffer( usStrgIdx, 0, 0, cNumberBuffer, LEN_GSM_RSSI );

	if ( ( ucIntStrgToByte( cNumberBuffer, 2 ) != 99 ) && ( ucIntStrgToByte( cNumberBuffer, LEN_GSM_RSSI + 1 ) != 255 ) )
	{
		strncpy( ( char * )cGsm_RSSI, 	  ( char * )cNumberBuffer, LEN_GSM_RSSI );
		strncpy( ( char * )cGsm_BLE_RSSI, ( char * )cNumberBuffer, LEN_GSM_RSSI );
	}	
	
	/* Get the bit error quality indicator. */
	( void )prvCopyFromUARTBuffer( usStrgIdx, 1, 0, cNumberBuffer, LEN_GSM_QUAL );

	if ( ( ucIntStrgToByte( cNumberBuffer, LEN_GSM_QUAL ) != 99 ) && ( ucIntStrgToByte( cNumberBuffer, LEN_GSM_QUAL + 1 ) != 255 ) )
	{
		strncpy( ( char * )cGsm_QUAL,     ( char * )cNumberBuffer, LEN_GSM_QUAL );
		strncpy( ( char * )cGsm_BLE_QUAL, ( char * )cNumberBuffer, LEN_GSM_QUAL );
	}
}
/*-----------------------------------------------------------*/

/* Parse +QNWINFO response.
   Format: +QNWINFO: <AcT>,<oper>,<band>,<channel>
   
   Interpretation:
		<AcT> 			Access technology selected.
							"No Service"
							"GSM"
							"GPRS"
							"EDGE"
							"eMTC"
							"NBIoT"
		<oper> 			Operator in numeric format.
		<band> 			Band selected.
							"GSM 850"
							"GSM 900"
							"GSM 1800"
							"GSM 1900"
							"LTE BAND 1"
							– 
							"LTE BAND 85"
		<channel>		Channel ID.   

	Example:
		+QNWINFO: "EDGE","46001","GSM 1800",653
		
	The only information used here is the eARFCN (channel).
*/
static void prvParseQNwInfo( unsigned short usStrgIdx )
{
	/* Copy field 3 to EARFCN. */
	prvCopyFromUARTBuffer( usStrgIdx, 3, 0, cGsm_EARFCN, LEN_GSM_EARFCN );	
	prvCopyFromUARTBuffer( usStrgIdx, 3, 0, cGsm_BLE_EARFCN, LEN_GSM_EARFCN );	
}
/*-----------------------------------------------------------*/

/* Parse the +COPS string containing information about the serving cell.
	+COPS: <mode>[,<format>,<oper>[,<AcT>]]
	
	Example:			+COPS: 0,2,"20820",8
									20820 --> 208 = MCC, 20 = MNC ('Bouygues'), 8 = AcT (RAT = M1)
*/
static void prvParseCOps( unsigned short usStrgIdx )
{
	/* The next three characters are the mobile country code. Copy them to MCC. */
	prvCopyFromUARTBuffer( usStrgIdx, 0, 0, cGsm_MCC, LEN_GSM_MCC );
	strncpy( ( char * )cGsm_BLE_MCC, 	  ( char * )cGsm_MCC, LEN_GSM_MCC );
	
	/* Then, two characters for MNC. */
	prvCopyFromUARTBuffer( usStrgIdx, 0, 3, cGsm_MNC, LEN_GSM_MNC );
	if ( cGsm_MNC[ LEN_GSM_MNC - 1 ] == '"' )
	{
		cGsm_MNC[ LEN_GSM_MNC - 1 ] = 0;
	}	
	strncpy( ( char * )cGsm_BLE_MNC, 	  ( char * )cGsm_MNC, LEN_GSM_MNC );
	
	/* Also copy the RAT, if not already done elsewhere. */
	if ( cGsm_RAT[ 0 ] == 0 ) 
	{
		prvCopyFromUARTBuffer( usStrgIdx, 1, 0, cGsm_RAT, LEN_GSM_RAT );
		/* Change type from '0' (GSM) to '3' (GSM EDGE) and '8' to '7' to be compatible with SARA. */
		if ( cGsm_RAT[ 0 ] == '0' )
		{
			cGsm_RAT[ 0 ] = '3';
		}
		if ( cGsm_RAT[ 0 ] == '8' )
		{
			cGsm_RAT[ 0 ] = '7';
		}
		cGsm_BLE_RAT[ 0 ] = cGsm_RAT[ 0 ];
		cGsm_BLE_RAT[ 1 ] = 0;
	}
}
/*-----------------------------------------------------------*/

/* Parse the +QFTPGET indication to check if the file received has a certain minimum length. */
static void prvParseQFTPGet( unsigned short usStrgIdx )
{
	signed char					cNumberBuffer[ 7 ];
	enum xAT_MSG_ID 			xMsgId;
	
	cNumberBuffer[ 0 ] = 0;
	
	/* Get the RSSI. No need to keep the volatile here as parser is the highest-priority task to access this variable. */
	( void )prvCopyFromUARTBuffer( usStrgIdx, 0, 0, cNumberBuffer, 7 );

	if ( ulIntStrgToLong( cNumberBuffer, 7 ) > 0 )
	{
		xMsgId = AT_FTP_OK;
	}	
	else
	{
		*cServerCmdParams = 0;
		xMsgId = AT_FTP_KO;
	}

	if ( xQueueSend( xParserAtRespQueue, &xMsgId, parserAT_RESP_BLOCKTIME ) != pdPASS )
	{
		/* The indication could not be sent for a certain time. 
		   TODO; Check, if this could be normal. The GSM task might not be interested in the 
		   indications from the GSM module for some time. */
		V_TRACE_PRINT( TRACE_PAR_XPARSERATRESPQUEUE_FULL, TRACE_UART_AND_FILE );
	}	
}
/*-----------------------------------------------------------*/

/* Terminate storage of strings and arguments. */
static void prvClrStorage( unsigned short usStrgIdx )
{
	( void )usStrgIdx;
	
	/* If the storage of intermediate strings had been requested, make sure that the buffer is correctly terminated. */
	if ( bStoreSeveralStrings )
	{
		*pcExpectedParamStrg = 0;
	}
	/* Reset the request to store several strings. */
	uxExpectedParamLen = 0;
	bStoreSeveralStrings = false;
}
/*-----------------------------------------------------------*/

/* Copy the arguments of the remote command. The actual command is treated in the 
   GSM task. 
*/
static void prvStoreRemoteArgs( unsigned short usStrgIdx )
{
	signed char				cCmdParam;
	signed char				*pcParamBuf;
	portBASE_TYPE			xMaxCharCnt;
	
	xMaxCharCnt = MAX_LEN_PARAMS;
	pcParamBuf = cServerCmdParams;
	bStoreSeveralStrings = false;
	
	/* Copy the the string as parameter. */
	do
	{
		cCmdParam = cGetRxCharFromBufferWithIndex( COM_GSM, usStrgIdx++ );
		*( pcParamBuf++ ) = cCmdParam;
		xMaxCharCnt--;
	} while ( ( cCmdParam != 0 ) && ( xMaxCharCnt != 0 ) );
	
	/* Make sure that there is always a end-of-string marker. */
	*( pcParamBuf++ ) = 0;
	
	bStoreSeveralStrings = false;
}
/*-----------------------------------------------------------*/
/* Copy the arguments of the remote command and append it to the storage buffer. 
   The storage buffer location must be initialised before.
   
   Example:
   
		AT+QFLST="*"\r\n
		+QFLST: "ca.pem",2338\r\n
		+QFLST: "client-cert.pem",1346\r\n
		+QFLST: "client-key.pem",887\r\n
		+QFLST: "mgaonline.ubx",5540\r\n
		OK\r\n
		
	gives:
		"ca.pem",2338\n
		"client-cert.pem",1346\n
		"client-key.pem",887\n
		"mgaonline.ubx",5540\n
		
   A function called when sending "AT+QFLST=\"*\"\r\n" initialises the variables
		uxExpectedParamLen
		pcExpectedParamStrg 
*/
static void prvAppendRemoteArgs( unsigned short usStrgIdx )
{
	signed char				cCmdParam;
	
	/* Copy the the string as parameter. */
	do
	{
		cCmdParam = cGetRxCharFromBufferWithIndex( COM_GSM, usStrgIdx++ );
		*( pcExpectedParamStrg++ ) = cCmdParam;
		uxExpectedParamLen--;
	} while ( ( cCmdParam != 0 ) && ( uxExpectedParamLen != 0 ) );
	
	/* Allow for several strings to be stored. The strings are separated by '\n' */
	*( pcExpectedParamStrg - 1 ) = '\n';
}
/*-----------------------------------------------------------*/

/* Copy the arguments of the remote command and request storage of all following strings. 
   For strings to be stored, they must not be preceded by a key word.
   
   Example:
   
		CONNECT abcdef\r\n
		ghijkl\r\n
		mnopqr\r\n
		stuvwx\r\n
		OK\r\n
		
	gives:
		abcdef\n
		ghijkl\n
		mnopqr\n
		stuvwx\n
		
   Matching the CONNECT key word calls this function and initialises storage of all following strings.
*/
static void prvStoreStrings( unsigned short usStrgIdx )
{
	signed char				cCmdParam;
	
	uxExpectedParamLen = MAX_LEN_PARAMS;
	pcExpectedParamStrg = cServerCmdParams;
	bStoreSeveralStrings = false;
	
	/* Copy the the string as parameter. */
	do
	{
		cCmdParam = cGetRxCharFromBufferWithIndex( COM_GSM, usStrgIdx++ );
		*( pcExpectedParamStrg++ ) = cCmdParam;
		uxExpectedParamLen--;
	} while ( ( cCmdParam != 0 ) && ( uxExpectedParamLen != 0 ) );
	
	/* Allow for several strings to be stored. The strings are separated by '\n' */
	*( pcExpectedParamStrg - 1 ) = '\n';
	
	bStoreSeveralStrings = true;
}
/*-----------------------------------------------------------*/

static portTASK_FUNCTION( vParserTask, pvParameters )
{
	/* Just to stop compiler warnings. */
	( void ) pvParameters;

	vTaskSetApplicationTaskTag( NULL, ( void * ) PAR_TASK_TAG );

	/* Wait untile the configuration handler is initialised. */
	while ( !bCheckConfigInitialised() || !bCheckTraceInitialised() )
	{
		vTaskDelay( 1 );
	}

	NRF_LOG_INFO( "Task started." );
	NRF_LOG_FLUSH();

	while ( 1 )
	{
		unsigned portBASE_TYPE	xRespIdx;
		bool					atRespFound;
		unsigned short			usStrgIdx;
		signed char				cAtResp;
		signed char				*pcAtRef;
		signed char				cAtRef;
		
		/* Wait indefinitely for a string to arrive on the UART. */
		if ( bComReceiveString( COM_GSM, portMAX_DELAY ) )
		{		
			xRespIdx = 0;
			atRespFound = false;
			
			/* Scan all strings in the AT response table to find a match. */
			while ( ( !atRespFound ) && ( xRespIdx < ( unsigned portBASE_TYPE )( xNumAtResp ) ) )
			{				
				/* Check if the response pointed to by xRespIdx is a match. */
				usStrgIdx = 0;
				do
				{
					cAtResp = cGetRxCharFromBufferWithIndex( COM_GSM, usStrgIdx );
					pcAtRef = ( signed char * )xAtResp[ xRespIdx ].pcAtResponse;
					cAtRef = *( pcAtRef + usStrgIdx );
					/* A '*' means: match any character. */
					if ( ( cAtResp != 0 ) && ( cAtRef == '*' ) )
					{
						cAtResp = cAtRef;
					}
					usStrgIdx++;
				} 
				while (    ( cAtResp != 0 ) 
						&& ( cAtRef != 0 )
						&& ( cAtRef == cAtResp ) );

				/* A match has been found if the end of the reference string has been reached and 
				   no mismatch has been encountered before. */
				if ( cAtRef == 0 )
				{
					atRespFound = true;
					usStrgIdx--;
				}
				else
				{
					/* No match: select next entry in the response table. */
					xRespIdx++;
				}
			}
			
			/* Treat the response, if a match was found in the table. The resulting index is xRespIdx. */
			if ( xRespIdx < sizeof( xAtResp ) / sizeof( xAtResp[ 0 ] ) )
			{
				enum xAT_MSG_ID xMsgId;
				void			( *pcParameterParser )( unsigned short usStrgIdx );
				
				/* If there is a parameter to be taken, store it in the indicated location. */
				pcParameterParser = (void ( * )( unsigned short usStrgIdx ) )xAtResp[ xRespIdx ].prvParamParser;
				if ( pcParameterParser != NULL )
				{
					/* Updating the parameters need to be protected by a mutex as they might be read anytime by the GSM task.
					   The mutex avoids that the GSM task reads inconsistent data. Care must be taken that the GSM task
					   takes the mutex only for a very short, determined time as the mutex may result in priority inversion. 
					   Also, it must be ensured that there is only one mutex used between GSM and Parser task, else the system
					   might enter a deadlock. */
					configASSERT( xSemaphoreTake( xMutexGsmData, TO_MUTEX_GSMDATA ) );
					pcParameterParser( usStrgIdx );
					xSemaphoreGive( xMutexGsmData );
				}
				
				/* If required, send an indication to the GSM task. */
				xMsgId = xAtResp[ xRespIdx ].xAtMsgId;
				if ( xMsgId != AT_NOMSG )
				{
					if ( xQueueSend( xParserAtRespQueue, &xMsgId, parserAT_RESP_BLOCKTIME ) != pdPASS )
					{
						/* The indication could not be sent for a certain time. */
						V_TRACE_PRINT( TRACE_PAR_XPARSERATRESPQUEUE_FULL, TRACE_UART_AND_FILE );
					}
				}
				
				/* In case of an AT_TCP_RX_DATA_IND, also send a indication to the GSM command queue. 
				
				   If we got a server command attached to an 'OK' message, send a 'raw command' indication to the GSM command
				   queue. That way it knows that the command does not have to be pulled from the server. */
				if (    ( xMsgId == AT_TCP_RX_DATA_IND )
					 || ( ( xMsgId >= SRVCMD_RD_VERSION ) && ( xMsgId < SRVCMD_OTHER ) ) 
				   )
				{
					enum xGSM_CMD		xGsmCmd;
					
					/* Send the command w/o blocking to the GSM task. The Parser task has higher priority
					   and thus cannot wait on the queue to become available. Worst case, we are losing a 
					   command. */
					xGsmCmd = GSM_SERVER_CMD;
					if ( xQueueSend( xGsmCmdQueue, &xGsmCmd, 0 ) != pdPASS )
					{
						V_TRACE_PRINT( TRACE_PAR_GSMCMDQUEUE_FULL, TRACE_UART_AND_FILE );
					}
				}
			}
			else
			{
				/* The received string has not been found in the reference table. Now it may be, that the string
				   is a single line without any known structure containing a response to the last command.
				   In that case, the GSM task which has sent the command, has set the uxExpectedParamLen variable with the maximum length 
				   of the expected parameter string and the associated pcExpectedParamStrg with the pointer to the parameter buffer.
				   Now simply copy the string to the parameter buffer and let the GSM task worry what to do with it. 
				   If bStoreSeveralStrings is set true, several strings are concatenated until the next matching key word is received. */
				if ( uxExpectedParamLen > 0 )
				{
					usStrgIdx = 0;
					
					do
					{
						cAtResp = cGetRxCharFromBufferWithIndex( COM_GSM, usStrgIdx++ );
						*( pcExpectedParamStrg++ ) = cAtResp;
						uxExpectedParamLen--;
					}
					while ( ( uxExpectedParamLen > 0 ) && ( cAtResp != 0 ) );
					
					if ( bStoreSeveralStrings && ( uxExpectedParamLen > 0 ) && ( cAtResp == 0 ) )
					{
						/* Replace the terminating 0x00 with a '\n'
						   with the next string. */
						*( pcExpectedParamStrg - 1 ) = '\n';
					}
					else
					{
						/* Clear the flag. */
						uxExpectedParamLen = 0;			
					}
				}
			}
			
			/* Remove the received string from the UART RX ring buffer. */
			vUartRemoveString( COM_GSM );
		}
	}
}		
/*-----------------------------------------------------------*/