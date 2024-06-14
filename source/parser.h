/*
 * Tracker Firmware
 *
 * Parser header file
 *
 */ 
#ifndef PARSER_H
#define PARSER_H

#include "queue.h"
#include "task.h"

#define parserAT_RESP_QUEUE_SIZE	( 15 )
#define parserAT_RESP_BLOCKTIME		( ( TickType_t ) ( 5 * portTICKS_PER_SEC ) )	/* wait max. 5 second for At response indication queue to become available */

/* Enumerate type containing identifiers for all AT messages to be received from the GSM/GPS device and server commands. 
   The enumeration value is fixed as the index is sent as relayed commend over BLE. Fixing the value allows devices wih different FW version to assign the same 
   meaning to a relayed command. 
   SRVCMD_OTHER must always have the highest number of all server commands. */
enum xAT_MSG_ID
{
	AT_NOMSG						=  0,							
	AT_OK							=  1, 									
	AT_CGREG_ABAND					=  2, 			        
	AT_CGREG_UNKNOWN				=  3, 			        
	AT_CGREG_REG					=  4, 	
	AT_QBOOT						= 230,
	AT_SEND_OK						= 232,
	AT_SEND_FAIL					= 233,
	AT_USOCRID						=  5,	                        
	AT_TCP_RX_DATA_IND				=  6,	                
	AT_USOCL						=  7,	                        
	AT_FTP_STAT						=  8,	                        
	AT_FTP_OK						= 14,	                        
	AT_FTP_KO						= 15, 	                        
	AT_FS_LST						= 16,                          
	AT_ERROR						= 17,                           
	AT_ATECHO						= 18,  
	AT_CONNECT						= 231,
	AT_HTTP_GET_OK					= 234,
	AT_HTTP_GET_KO					= 235,
	AT_PING							= 236,
	AT_NWCFG						= 237,
	AT_URC							= 238,
										
	AT_REMOTE_OK					= 19, 		                
	AT_REMOTE_KO					= 20,		                
												
	SRVCMD_RD_VERSION				= 21,	 		        
	SRVCMD_RESET					= 22, 		 		        
	SRVCMD_RD_GPRSCFG				= 23,	 		        
	SRVCMD_WR_GPRSCFG				= 24,	 		        
	SRVCMD_RD_TCPCFG				= 25,	 		        
	SRVCMD_WR_TCPCFG				= 26,	 		        
	SRVCMD_RD_SECTCPCFG				= 27,                
	SRVCMD_WR_SECTCPCFG				= 28,                
	SRVCMD_RD_DFMAP					= 29,	 		        
	SRVCMD_WR_DFMAP					= 30,	 		        
	SRVCMD_RD_RAT              		= 161,
	SRVCMD_WR_RAT					= 162,	
	SRVCMD_RD_ATT_TO				= 165,
	SRVCMD_WR_ATT_TO				= 166,

	SRVCMD_RD_ACT_INT				= 31,	 		        
	SRVCMD_WR_ACT_INT				= 32,	 		        
	SRVCMD_RD_INACT_INT				= 33, 		        
	SRVCMD_WR_INACT_INT				= 34, 		        
	SRVCMD_RD_STILL_INT				= 35, 		        
	SRVCMD_WR_STILL_INT				= 36, 		        
	SRVCMD_RD_SLEEP_INT				= 37, 		        
	SRVCMD_WR_SLEEP_INT				= 38, 		        
	SRVCMD_RD_OOO_INT				= 39,	 		        
	SRVCMD_WR_OOO_INT				= 40,	 		        
	SRVCMD_RD_CHRG_INT				= 41,	 		        
	SRVCMD_WR_CHRG_INT				= 42,	 		        
	SRVCMD_RD_ALERT_INT				= 43,		        
	SRVCMD_WR_ALERT_INT				= 44,		        
	SRVCMD_RD_PD_INT				= 45,	 		        
	SRVCMD_WR_PD_INT				= 46,	 	            
											
    SRVCMD_RD_STEP_ACC_THR			= 47,		        
    SRVCMD_WR_STEP_ACC_THR			= 48,    	        
    SRVCMD_RD_STEP_MAX_INT			= 49,    	        
    SRVCMD_WR_STEP_MAX_INT			= 50,    	        
    SRVCMD_RD_STEP_MIN_INT			= 51,    	        
    SRVCMD_WR_STEP_MIN_INT			= 52,    	        
    SRVCMD_RD_ACTIVE_NUM_STEPS		= 53,  	    
    SRVCMD_WR_ACTIVE_NUM_STEPS		= 54,  	    
									
    SRVCMD_RD_SOS_ACC_THR			= 55,		        
    SRVCMD_WR_SOS_ACC_THR			= 56,    	        
	SRVCMD_RD_SOS_MIN_X_ACC_THR		= 212,
	SRVCMD_WR_SOS_MIN_X_ACC_THR		= 213,	
	SRVCMD_RD_SOS_MIN_Z_ACC_THR		= 193,
	SRVCMD_WR_SOS_MIN_Z_ACC_THR		= 194,
	SRVCMD_RD_SOS_MAX_Z_ACC_THR		= 210,
	SRVCMD_WR_SOS_MAX_Z_ACC_THR		= 211,	
    SRVCMD_RD_SOS_MAX_INT			= 57,    	        
    SRVCMD_WR_SOS_MAX_INT			= 58,    	        
    SRVCMD_RD_SOS_MIN_INT			= 59,    	        
    SRVCMD_WR_SOS_MIN_INT			= 60,    	        
    SRVCMD_RD_SOS_NUM_PEAKS			= 61,  	        
    SRVCMD_WR_SOS_NUM_PEAKS			= 62,  	        
											
    SRVCMD_RD_STILL_DUR				= 63,         	    
    SRVCMD_WR_STILL_DUR				= 64,         	    
	SRVCMD_RD_IMMOBILITY_DUR		= 206,
	SRVCMD_WR_IMMOBILITY_DUR		= 207,
    SRVCMD_RD_SLEEP_DUR				= 65,      	        
    SRVCMD_WR_SLEEP_DUR				= 66,      	        
                                        
    SRVCMD_RD_ABNPOS_Z_ACC_THR		= 67,  	    
    SRVCMD_WR_ABNPOS_Z_ACC_THR		= 68,		    
    SRVCMD_RD_ABNPOS_DUR			= 69,           	
    SRVCMD_WR_ABNPOS_DUR			= 70,           	
	SRVCMD_RD_ABNPOS_MAX_ORIENT		= 71,        
	SRVCMD_WR_ABNPOS_MAX_ORIENT		= 72,        
                                         
	SRVCMD_RD_ALERT_CNCL_METHOD		= 73,
	SRVCMD_WR_ALERT_CNCL_METHOD		= 74,
    SRVCMD_RD_ALERT_CNCL_THR		= 75,    	    
    SRVCMD_WR_ALERT_CNCL_THR		= 76,    	    
    SRVCMD_RD_ALERT_CNCL_MIN_INT	= 77,	    
    SRVCMD_WR_ALERT_CNCL_MIN_INT	= 78,	    
    SRVCMD_RD_ALERT_CNCL_MAX_INT	= 79,	    
    SRVCMD_WR_ALERT_CNCL_MAX_INT	= 80,	    
    SRVCMD_RD_ALERT_CNCL_NUM_PEAKS	= 81,     
    SRVCMD_WR_ALERT_CNCL_NUM_PEAKS	= 82,     
    SRVCMD_RD_ALERT_CNCL_TIMEOUT	= 83,       
    SRVCMD_WR_ALERT_CNCL_TIMEOUT	= 84,       
    SRVCMD_RD_SOS_CNCL_TIMEOUT		= 85,         
    SRVCMD_WR_SOS_CNCL_TIMEOUT		= 86,         
	                                    
	SRVCMD_RD_NOABN_ENABLE		 	= 177,
    SRVCMD_WR_NOABN_ENABLE			= 178,
    SRVCMD_RD_NOABN_NUM_PEAKS	    = 179,
    SRVCMD_WR_NOABN_NUM_PEAKS	    = 180,
    SRVCMD_RD_NOABN_MAX_PERIOD	    = 181,
    SRVCMD_WR_NOABN_MAX_PERIOD	    = 182,
    SRVCMD_RD_NOABN_WINDOW		    = 183,
    SRVCMD_WR_NOABN_WINDOW		    = 184,
    SRVCMD_RD_NOABN_ACK_ON		    = 185,
    SRVCMD_WR_NOABN_ACK_ON		    = 186,
    SRVCMD_RD_NOABN_ACK_OFF		    = 187,
    SRVCMD_WR_NOABN_ACK_OFF		    = 188,
    SRVCMD_RD_NOABN_ACK_REP		    = 189,
	SRVCMD_WR_NOABN_ACK_REP		    = 190,
	                                    
	SRVCMD_RD_BATT_EMPTY_THRES		= 91,         
	SRVCMD_WR_BATT_EMPTY_THRES		= 92,         
	SRVCMD_RD_BATT_LOW_THRES		= 155,
	SRVCMD_WR_BATT_LOW_THRES		= 156,
	SRVCMD_RD_BATT_FULL_THRES		= 157,
	SRVCMD_WR_BATT_FULL_THRES		= 158,
	SRVCMD_RD_BATT_LOOP_INT			= 218,
	SRVCMD_WR_BATT_LOOP_INT			= 219,

	SRVCMD_RD_GPS_FIX_QUALITY		= 87,          
	SRVCMD_WR_GPS_FIX_QUALITY		= 88,          
	SRVCMD_RD_GPS_MAX_HACC			= 175,
	SRVCMD_WR_GPS_MAX_HACC			= 176,
	SRVCMD_RD_GPS_WAIT_FOR_FIX		= 89,         
	SRVCMD_WR_GPS_WAIT_FOR_FIX		= 90,         
	SRVCMD_RD_GPS_WAIT_FOR_SAT		= 195,         
	SRVCMD_WR_GPS_WAIT_FOR_SAT		= 196,   
	SRVCMD_RD_GPS_ENABLE			= 93,               
	SRVCMD_WR_GPS_ENABLE			= 94,               
	SRVCMD_RD_GPS_PD_INT			= 95,               
	SRVCMD_WR_GPS_PD_INT			= 96,               
	SRVCMD_RD_GPS_PSM				= 163,
	SRVCMD_WR_GPS_PSM				= 164,
	SRVCMD_RD_GPS_LOCEST_SRV		= 173,
	SRVCMD_WR_GPS_LOCEST_SRV		= 174,
	SRVCMD_RD_GPS_POSREC			= 197,
	SRVCMD_WR_GPS_POSREC			= 198,
	SRVCMD_RD_GPS_POSREC_INT		= 214,
	SRVCMD_WR_GPS_POSREC_INT		= 215,
	                                   
	SRVCMD_RD_BLE_ENABLE			= 97,               
	SRVCMD_WR_BLE_ENABLE			= 98,               
	SRVCMD_RD_BLE_UPD_INT			= 153,                  
	SRVCMD_WR_BLE_UPD_INT			= 154,                  
	SRVCMD_WR_BLE_KEY				= 103,                  
	SRVCMD_RD_BLE_DFMAP				= 104,                
	SRVCMD_WR_BLE_DFMAP				= 105,                
	SRVCMD_RD_BLE_TXPWR				= 106,                
	SRVCMD_WR_BLE_TXPWR				= 107,     
	SRVCMD_RD_BLE_RSSI_CFG			= 159,
	SRVCMD_WR_BLE_RSSI_CFG			= 160,
	SRVCMD_RD_BLE_MIN_BCN			= 167,
	SRVCMD_WR_BLE_MIN_BCN           = 168,      
	SRVCMD_RD_BLE_IN_DANGER_DUR		= 169,
	SRVCMD_WR_BLE_IN_DANGER_DUR		= 170,
	SRVCMD_RD_BLE_FOREIGN_ALERT		= 171,
	SRVCMD_WR_BLE_FOREIGN_ALERT		= 172,
	SRVCMD_RD_BLE_SCAN_FIX_ONLY		= 200,
	SRVCMD_WR_BLE_SCAN_FIX_ONLY		= 201,
	SRVCMD_RD_BLE_MAX_WAIT_BCN		= 202,
	SRVCMD_WR_BLE_MAX_WAIT_BCN		= 203,
	SRVCMD_RD_BLE_STD_ADV_INT		= 204,
	SRVCMD_WR_BLE_STD_ADV_INT		= 205,
	SRVCMD_RD_BLE_UUID_FILTER		= 216,
	SRVCMD_WR_BLE_UUID_FILTER		= 217,
	SRVCMD_RD_BLE_BCN_FILTER		= 220,
	SRVCMD_WR_BLE_BCN_FILTER		= 221,
	
	SRVCMD_PEEK_RAM					= 108,	 		        
	SRVCMD_POKE_RAM					= 109,	 		        
	SRVCMD_PEEK_NVDS				= 110,	 		        
	SRVCMD_POKE_NVDS				= 111,	                
	SRVCMD_RDSTATS					= 112,                     
	SRVCMD_RESSTATS					= 113,	                
	SRVCMD_VIBRATE					= 114,		 		        
	SRVCMD_LED						= 115,		                    
										
	SRVCMD_EVACUATE					= 116,                    
	SRVCMD_EVACUATION_STOP			= 117,             
	SRVCMD_RD_EVAC_VIBR				= 118,                
	SRVCMD_WR_EVAC_VIBR				= 119,                
	                                  
	SRVCMD_STATE					= 120,				        
	SRVCMD_STANDBY_MODE				= 121,		        
	SRVCMD_NORMAL_MODE				= 122,			        
	SRVCMD_OOO_MODE					= 123,			        
	SRVCMD_ALERT_ACK				= 124,			        
	SRVCMD_GOTO_PREALERT			= 199,
	SRVCMD_TEST_START				= 125,                  
	SRVCMD_TEST_STOP				= 126,                   
	                                   
	SRVCMD_WR_FTPSRV				= 127,                   
	SRVCMD_RD_FTPSRV				= 128,                   
	SRVCMD_WR_FTP_CFG				= 191,
	SRVCMD_RD_FTP_CFG				= 192,
	SRVCMD_FWUPD					= 129,                       
                                      
	SRVCMD_FS_LST					= 131,                      
    SRVCMD_FS_GET					= 132,                      
    SRVCMD_FS_PUT					= 133,                      
	SRVCMD_FS_DEL					= 134,                      
									 
	SRVCMD_RD_SEC_ENABLE			= 136,               
	SRVCMD_WR_SEC_ENABLE			= 137,               
	                                  
	SRVCMD_RD_LOG_ENABLE			= 138,               
	SRVCMD_WR_LOG_ENABLE			= 139,               
									
	SRVCMD_RD_GPS_STREAM_PORT		= 140,
	SRVCMD_WR_GPS_STREAM_PORT		= 141,
	SRVCMD_RD_GPS_STREAM_ADDR		= 142,
	SRVCMD_WR_GPS_STREAM_ADDR		= 143,
	                                  
	SRVCMD_RD_MOD_SIDE				= 144,                 
	SRVCMD_WR_MOD_SIDE				= 145,                 
	                                  
	SRVCMD_WR_RELAY_CMD				= 146,                
									 
	SRVCMD_RD_DANGER_BCN_THRES		= 147,	
	SRVCMD_WR_DANGER_BCN_THRES		= 148,	
	SRVCMD_RD_NOABN_BCN_THRES		= 149,	
	SRVCMD_WR_NOABN_BCN_THRES		= 150,	
	SRVCMD_RD_PRIVATE_BCN_THRES		= 151,
	SRVCMD_WR_PRIVATE_BCN_THRES		= 152,	
	SRVCMD_RD_IMMOBILITY_BCN_THRES	= 208,
	SRVCMD_WR_IMMOBILITY_BCN_THRES	= 209,

	SRVCMD_OTHER				    = 222
};

/* Definition of a structure field used for parsing and classifying incoming messages. */
struct xAT_RESP 
{
	const signed char	*pcAtResponse;
	void				( *prvParamParser )( unsigned short usStrgIdx );
	enum xAT_MSG_ID		xAtMsgId;	
};

/* Public function prototypes. */
void vParserInit( UBaseType_t uxPriority );

/* Global variables. */
/* AT response indicator queue handle. */
extern QueueHandle_t 			xParserAtRespQueue; 
/* Pointer to a buffer to store the next non-empty string in. */
extern signed char 				*pcExpectedParamStrg;	
/* Maximum number of characters to store in the buffer. If 0, nothing is stored. */
extern unsigned portBASE_TYPE 	uxExpectedParamLen;		
/* Request storage of several strings. */
extern bool						bStoreSeveralStrings;
/* Task handle of the parser task so that it can be controlled. */
extern TaskHandle_t 			xParserTaskHandle;		

/* Table containing pointers to the AT response strings, parameters to be stored
   (if any) and the AT response ID to be sent to the GSM task via queue. */
extern const struct xAT_RESP 	xAtResp[];
extern const size_t 			xNumAtResp;


/* Pointers to strings, re-used to save program space. */
extern const char pcAt_SrvWrTcpCfg[];
extern const char pcAt_SrvWrSecTcpCfg[];
extern const char pcAt_SrvWrDFMap[];
extern const char pcAt_SrvWrRat[];
extern const char pcAt_SrvWrAttTo[];
extern const char pcAt_SrvWrActInt[];			
extern const char pcAt_SrvWrInactInt[];			
extern const char pcAt_SrvWrStillInt[];			
extern const char pcAt_SrvWrSleepInt[];			
extern const char pcAt_SrvWrOooInt[];			
extern const char pcAt_SrvWrChrgInt[];			
extern const char pcAt_SrvWrAlertInt[];			
extern const char pcAt_SrvWrPwrDwnInt[];		

extern const char pcAt_SrvWrStepAccThr[];		
extern const char pcAt_SrvWrStepMinInt[];		
extern const char pcAt_SrvWrStepMaxInt[];		
extern const char pcAt_SrvWrActiveNumSteps[];	

extern const char pcAt_SrvWrSosAccThr[];		
extern const char pcAt_SrvWrSosMinXAccThr[];
extern const char pcAt_SrvWrSosMinZAccThr[];
extern const char pcAt_SrvWrSosMaxZAccThr[];
extern const char pcAt_SrvWrSosMinInt[];		
extern const char pcAt_SrvWrSosMaxInt[];		
extern const char pcAt_SrvWrSosNumPeaks[];	

extern const char pcAt_SrvWrStillDur[];			
extern const char pcAt_SrvWrImmobilityDur[];	
extern const char pcAt_SrvWrSleepDur[];		

extern const char pcAt_SrvWrAbnPosZAccThr[];	
extern const char pcAt_SrvWrAbnPosDur[];		
extern const char pcAt_SrvWrAbnMaxOrient[];			

extern const char pcAt_SrvWrAlertCnclMethod[];
extern const char pcAt_SrvWrAlertCnclThr[];		
extern const char pcAt_SrvWrAlertCnclMinInt[];
extern const char pcAt_SrvWrAlertCnclMaxInt[];
extern const char pcAt_SrvWrAlertCnclNumPeaks[];
extern const char pcAt_SrvWrAlertCnclTimeout[];
extern const char pcAt_SrvWrSosCnclTimeout[];

extern const char pcAt_SrvWrNoAbnEn[];		
extern const char pcAt_SrvWrNoAbnNumPeaks[];
extern const char pcAt_SrvWrNoAbnMaxPeriod[];
extern const char pcAt_SrvWrNoAbnWindow[];	
extern const char pcAt_SrvWrNoAbnCmdAckOn[];
extern const char pcAt_SrvWrNoAbnCmdAckOff[];
extern const char pcAt_SrvWrNoAbnCmdAckRep[];

extern const char pcAt_SrvWrBattEmptyThres[];
extern const char pcAt_SrvWrBattLowThres[];
extern const char pcAt_SrvWrBattFullThres[];
extern const char pcAt_SrvWrBattLoopInt[];

extern const char pcAt_SrvWrGpsEnable[];
extern const char pcAt_SrvWrGpsFixQuality[];
extern const char pcAt_SrvWrGpsMaxHAcc[];
extern const char pcAt_SrvWrMaxGpsWaitTime[];
extern const char pcAt_SrvWrMaxGpsSatWaitTime[];
extern const char pcAt_SrvWrGpsPwrDwnInt[];		
extern const char pcAt_SrvWrGpsPsm[];		
extern const char pcAt_SrvWrGpsLocEstSrv[];
extern const char pcAt_SrvWrGpsPosRec[];
extern const char pcAt_SrvWrGpsPosRecInt[];

extern const char pcAt_SrvWrBleEnable[];
extern const char pcAt_SrvWrBleUpdInt[];
extern const char pcAt_SrvWrBleDFMap[];
extern const char pcAt_SrvWrBleTxPwr[];
extern const char pcAt_SrvWrBleRssiCfg[];
extern const char pcAt_SrvWrBleMinBcn[];
extern const char pcAt_SrvWrBleInDangerDur[];
extern const char pcAt_SrvWrBleForeignAlert[];
extern const char pcAt_SrvWrBleScanFixOnly[];
extern const char pcAt_SrvWrBleMaxWaitForBcn[];
extern const char pcAt_SrvWrBleStdAdvInt[];
extern const char pcAt_SrvWrBleUuidFilter[];
extern const char pcAt_SrvWrBleBcnFilter[];

extern const char pcAt_SrvWrEvacVibr[];

extern const char pcAt_SrvWrFtpSrv[];
extern const char pcAt_SrvWrFtpCfg[];

extern const char pcAt_SrvWrSecEnable[];

extern const char pcAt_SrvWrLogEnable[];

extern const char pcAt_SrvWrModuleSide[];

extern const char pcAt_SrvWrDangerBcnThres[];
extern const char pcAt_SrvWrNoAbnBcnThres[];
extern const char pcAt_SrvWrPrivateBcnThres[];
extern const char pcAt_SrvWrImmobilityBcnThres[];

#endif