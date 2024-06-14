/*
 * Tracker Firmware
 *
 * Trace message definitions
 *
 */ 
#ifndef TRACEMSG_H
#define TRACEMSG_H
/*-----------------------------------------------------------*/

#define		TRACE_UART				( 1 )
#define		TRACE_FILE				( 2 )
#define		TRACE_UART_AND_FILE		( 3 )

#define		TRACE_RAW												(   0 )		/* TRACE_STRING ""		 												*/
#define		TRACE_DBG0												( 240 )		/* TRACE_STRING "DBG0 GPS Recording config: "							*/
#define		TRACE_DBG1												( 241 )		/* TRACE_STRING "DBG1 Exit w/o recording: "								*/
#define		TRACE_DBG2												( 242 )		/* TRACE_STRING "DBG2 Record before GSM: "								*/
#define		TRACE_DBG3												( 243 )		/* TRACE_STRING "DBG3 Exit recording before GSM: "						*/
#define		TRACE_DBG4												( 244 )		/* TRACE_STRING "DBG4 Decision to shut-down cellular module once on: "	*/
#define		TRACE_DBG5												( 245 )		/* TRACE_STRING "DBG5 GSM Shutdown process: "													*/
#define		TRACE_DBG6												( 246 )		/* TRACE_STRING "DBG6 "													*/
#define		TRACE_DBG7												( 247 )		/* TRACE_STRING "DBG7 "													*/
#define		TRACE_DBG8												( 248 )		/* TRACE_STRING "DBG8 "													*/
#define		TRACE_DBG9												( 249 )		/* TRACE_STRING "DBG9 "													*/
#define		TRACE_DBG10												( 250 )		/* TRACE_STRING "DBG10 "												*/
#define		TRACE_DBG11												( 251 )		/* TRACE_STRING "DBG11 "												*/
#define		TRACE_DBG12												( 252 )		/* TRACE_STRING "DBG12 "												*/
#define		TRACE_DBG13												( 253 )		/* TRACE_STRING "DBG13 "												*/
#define		TRACE_DBG14												( 254 )		/* TRACE_STRING "DBG14 "												*/
#define		TRACE_DBG15												( 255 )		/* TRACE_STRING "DBG15 "												*/
	
#define		TRACE_FWUPDATED											(   1 )		/* TRACE_STRING "bFWUpdated: " 											*/
#define		TRACE_FWROLLEDBACK										(   2 )		/* TRACE_STRING "bFWRolledBack: " 										*/
#define		TRACE_FWNOTFUNCTIONAL									(   3 )		/* TRACE_STRING "bNewFWNotFunctional: " 								*/
#define		TRACE_ACC_SELF_TEST_FAILED								(   4 )		/* TRACE_STRING "Accelerator self test failed!" 						*/
#define		TRACE_HEAP												(   5 )		/* TRACE_STRING "Heap: " 												*/
#define		TRACE_WD_RESETS      									(   6 )		/* TRACE_STRING "WD resets:                          "					*/
#define		TRACE_SW_RESETS      									(   7 )		/* TRACE_STRING "SW resets:                          "					*/
#define		TRACE_POR_RESETS     									(   8 )		/* TRACE_STRING "POR resets:                         "					*/
#define		TRACE_OTHER_RESETS										(   9 )		/* TRACE_STRING "Other resets:                       "					*/
#define		TRACE_RTOS_SYSTEM_ERRORS 								(  10 )		/* TRACE_STRING "RTOS system errors:                 "					*/
#define		TRACE_LOCKUP_RESETS										(  11 )		/* TRACE_STRING "Lockup resets:                      "					*/
#define		TRACE_STACK_OVERFLOWS									(  12 )		/* TRACE_STRING "Stack overflows:                    "					*/
#define		TRACE_VERSION											(  13 )		/* TRACE_STRING "Version: " 											*/
#define		TRACE_SYSTEM_ERROR										(  14 )		/* TRACE_STRING "System error at 0x" 									*/
#define		TRACE_RESET												(  15 )		/* TRACE_STRING "System reset, processor reset source: 0x" 				*/
#define		TRACE_NVDS_INIT											(  16 )		/* TRACE_STRING "NVDS re-initialised."					 				*/
#define 	TRACE_FDS_FAILURE										(  17 )		/* TRACE_STRING "FDS failure."							 				*/
#define		TRACE_RESET_EXT_INFO									(  18 )		/* TRACE_STRING "System reset, extended debug info: " 					*/
#define		TRACE_SD_ASSERT											(  19 )		/* TRACE_STRING "SoftDevice asserts:                 "					*/
#define		TRACE_SD_MEM_ERROR										( 120 )		/* TRACE_STRING "SoftDevice invalid memory accesses: "					*/
#define		TRACE_NRF_SDK_ASSERT									( 121 )		/* TRACE_STRING "nRF SDK asserts:                    "					*/
#define		TRACE_NRF_SDK_ERROR										( 122 )		/* TRACE_STRING "nRF SDK errors:                     "					*/
#define		TRACE_NRF_UNKNOWN_ERROR									( 123 )		/* TRACE_STRING "nRF unknown errors:                 "					*/
#define		TRACE_BOOT_ERROR										( 124 )		/* TRACE_STRING "Consecutive boot failures:          "					*/
#define		TRACE_BL_VERSION										( 125 )		/* TRACE_STRING "Bootloader version: "									*/
#define		TRACE_ORIGINATING_ADDRESS								( 126 )		/* TRACE_STRING "Reset origin address:    0x" 							*/
#define		TRACE_HARDFAULTS										( 127 )		/* TRACE_STRING "Hardfaults:                         "					*/

#define		TRACE_GSM_FORCE_RAT										(  20 )		/* TRACE_STRING "GSM: Forcing RAT: "									*/
#define		TRACE_GSM_RESTORE_RAT									(  21 )		/* TRACE_STRING "GSM: Returning to configured RAT: "					*/
#define		TRACE_GSM_NO_INTERNET									(  22 )		/* TRACE_STRING "GSM: No Internet connectivity. "						*/

#define		TRACE_GPS_FIX_SYNOPSIS									(  23 )		/* TRACE_STRING "GPS: Fix synopsis: "									*/

#define		TRACE_GSM_NO_SERVER_OK									(  24 )		/* TRACE_STRING "GSM: No [OK] from server received."					*/
#define		TRACE_GSM_RX_OK_RESPONSE_TIMEOUT						(  25 )		/* TRACE_STRING "GSM: Timeout when receiving server OK."				*/
#define		TRACE_GSM_GPS_ASSISTED_DATA_DOWNLOADED					(  26 )		/* TRACE_STRING "GSM: Downloaded AssistedGPS."							*/
#define		TRACE_GPS_LOCALISATION_START							(  27 )		/* TRACE_STRING "GPS: Localisation started."							*/
#define		TRACE_GSM_TCP_REACTIVATION								(  28 )		/* TRACE_STRING "GSM: Request TCP reactivation." 						*/
#define		TRACE_GSM_CELLULAR_POSITION_REQUEST						(  29 )		/* TRACE_STRING "GSM: Request rough cellular position from server." 	*/
#define		TRACE_GSM_LOST_NETWORK_CONNECTION						(  30 )		/* TRACE_STRING "GSM: Lost network connection."	 						*/
#define		TRACE_GSM_CELLULAR_POSITION_REQ_FAILED					(  31 )		/* TRACE_STRING "GSM: Cellular position request from server failed." 	*/
#define		TRACE_GSM_TCP_MODULE_ERR								(  32 )		/* TRACE_STRING "GSM: TCP connection check returned module error."		*/
#define		TRACE_GPS_LOCALISATION_STOP								(  33 )		/* TRACE_STRING "GPS: Localisation stopped."							*/
#define		TRACE_GPS_NO_I2C_RESPONSE								(  34 )		/* TRACE_STRING "GPS: No response on I2C bus from GPS module. Reboot."	*/
#define		TRACE_GSM_TCP_SOCKET_ERROR								(  39 )		/* TRACE_STRING "GSM: TCP socket error: "								*/
#define		TRACE_GSM_MODULE_SOFT_FULL_POWER_DOWN					(  40 )		/* TRACE_STRING "GSM: Module soft full power-down." 					*/
#define		TRACE_GSM_MODULE_HARD_FULL_POWER_DOWN					(  41 )		/* TRACE_STRING "GSM: Module hard full power-down." 					*/
#define		TRACE_GSM_MODULE_INTERNAL_POWER_DOWN_ONLY				(  42 )		/* TRACE_STRING "GSM: Module internal power-down only." 				*/
#define		TRACE_GSM_AT_RESPONSE_TO								(  43 )		/* TRACE_STRING "GSM: AT response TO: " 								*/
#define		TRACE_GSM_LOW_BATTERY									(  44 )		/* TRACE_STRING "GSM: Low battery: " 									*/
#define		TRACE_GSM_MODULE_IDLE_MODE								(  45 )		/* TRACE_STRING "GSM: Module IDLE mode." 								*/
#define		TRACE_GSM_MODULE_ACTIVE_MODE							(  46 )		/* TRACE_STRING "GSM: Module ACTIVE mode." 								*/
#define		TRACE_GSM_1PPS_SIGNAL_ALIVE								(  47 )		/* TRACE_STRING "GSM: 1pps signal alive." 								*/
#define		TRACE_GSM_SELF_TEST_STOP								(  48 )		/* TRACE_STRING "GSM: Self-test stop." 									*/
#define		TRACE_GSM_FW_UPDATE										(  49 )		/* TRACE_STRING "GSM: FW Update." 										*/
#define		TRACE_GSM_FS_LISTING									(  50 )		/* TRACE_STRING "GSM: FS listing." 										*/
#define		TRACE_GSM_FS_GET_FILE									(  51 )		/* TRACE_STRING "GSM: FS get file." 									*/
#define		TRACE_GSM_FS_PUT_FILE									(  52 )		/* TRACE_STRING "GSM: FS put file." 									*/
#define		TRACE_GSM_FS_FILE_DELETE								(  53 )		/* TRACE_STRING "GSM: FS file delete." 									*/
#define		TRACE_GSM_TLS_CERTIFICATE_INSTALLATION					(  54 )		/* TRACE_STRING "GSM: TLS certificate installation." 					*/
#define		TRACE_GSM_ALERT_OR_SOS_DETECTED							(  55 )		/* TRACE_STRING "GSM: Alert or SOS detected!" 							*/
#define		TRACE_GSM_SEND_POSITION									(  56 )		/* TRACE_STRING "GSM: Send position." 									*/
#define		TRACE_GSM_UNSUCESSFUL_BOOT								(  57 )		/* TRACE_STRING "GSM: Error when booting GSM module." 					*/
#define		TRACE_GSM_UNSUCESSFUL_GPRS_ATTACH						(  58 )		/* TRACE_STRING "GSM: Unsuccessful GPRS attach." 						*/
#define		TRACE_GSM_UNSUCESSFUL_CONTEXT_ACTIVATION				(  59 )		/* TRACE_STRING "GSM: Unsuccessful GPRS context activation."			*/
#define		TRACE_GSM_UNSUCESSFUL_GPRS								(  60 )		/* TRACE_STRING "GSM: Unsuccessful GPRS activation, CME error: "		*/
#define		TRACE_GSM_PDP_REACTIVATED								(  61 )		/* TRACE_STRING "GSM: PDP reactivated." 								*/
#define		TRACE_GSM_PDP_REACTIVATED_FAILED						(  62 )		/* TRACE_STRING "GSM: PDP reactivation failed."	 						*/
#define		TRACE_GSM_TCP_CONNECT_FAILED							(  63 )		/* TRACE_STRING "GSM: GSM TCP connect failed." 							*/
#define		TRACE_GSM_UNSOLICITED_PDP_DEACTIVATION					(  64 )		/* TRACE_STRING "GSM: Unsolicited PDP deactivation." 					*/
#define		TRACE_GSM_UNSOLICITED_TCP_SOCKET_CLOSE					(  65 )		/* TRACE_STRING "GSM: Unsolicited TCP socket close." 					*/
#define		TRACE_GSM_BOOT_DATA_CONNECTION_FAILED					(  66 )		/* TRACE_STRING "GSM: Boot/ data connection failed." 					*/
#define		TRACE_GSM_NO_RESPONSE_ON_CEER_PROBE						(  67 )		/* TRACE_STRING "GSM: No response on CEER probe."    					*/
#define		TRACE_GSM_ERROR_DURING_GPS_READ							(  68 )		/* TRACE_STRING "GSM: Error when reading GPS device: " 					*/
#define		TRACE_GSM_ASSISTED_GPS_DATA_REQUEST						(  69 )		/* TRACE_STRING "GSM: Downloading assisted GPS data." 					*/
#define		TRACE_GSM_PACKET_SENT									(  70 )		/* TRACE_STRING "GSM: Packet sent to server." 							*/
#define		TRACE_GSM_AT_MODULE_ERR									(  71 )		/* TRACE_STRING "GSM: Module error received on command: " 				*/
#define		TRACE_GSM_AT_RESPONSE_ERR								(  72 )		/* TRACE_STRING "GSM: Error response received on command: "				*/
#define		TRACE_GSM_SEND_MODULE_ERR								(  73 )		/* TRACE_STRING "GSM: Module error when sending packet data. "			*/
#define		TRACE_GSM_SEND_RESPONSE_ERR								(  74 )		/* TRACE_STRING "GSM: Error response when sending data."				*/
#define		TRACE_GSM_RX_OK_MODULE_ERR								(  75 )		/* TRACE_STRING "GSM: Module error when receiving server OK."			*/
#define		TRACE_GSM_RX_OK_RESPONSE_ERR							(  76 )		/* TRACE_STRING "GSM: Error response when receiving server OK."			*/
#define		TRACE_GSM_MODULE_RESET_DETECTED							(  77 )		/* TRACE_STRING "GSM: Module reset detected."							*/
#define		TRACE_GSM_FOREIGN_ALERT									(  78 )		/* TRACE_STRING "GSM: Foreign alert forwarded."							*/
#define		TRACE_GSM_FOREIGN_SOS									(  79 )		/* TRACE_STRING "GSM: Foreign SOS forwarded."							*/
#define		TRACE_GSM_MODULE_POWER_DOWN_CONVERTED_TO_HARD			(  80 )		/* TRACE_STRING "GSM: Int. power-down converted to hard power-down."	*/
#define		TRACE_GSM_MODULE_POWERED_DOWN							(  81 )		/* TRACE_STRING "GSM: GSM module hard powered-down."					*/
#define		TRACE_GSM_MAXCRASH_DETECT								(  82 )		/* TRACE_STRING "GSM: MAX module crash detected #"						*/
#define		TRACE_GSM_GPSRESETSTART									(  83 )		/* TRACE_STRING "GSM: MAX deep reset startup."							*/
#define		TRACE_GSM_GPSRESETSTARTSUCC								(  84 )		/* TRACE_STRING "GSM: MAX deep reset startup succeeded."				*/
#define		TRACE_GSM_BAUDSWITCH									(  85 )		/* TRACE_STRING "GSM: GSM module baudrate switch to "					*/
#define		TRACE_GSM_FOREIGN_ALERT_OR_SOS_DETECTED					(  86 )		/* TRACE_STRING "GSM: Foreign Alert or SOS detected!" 					*/
#define		TRACE_GSM_GPSSTARTCRASH									(  87 )		/* TRACE_STRING "GSM: GPS startup crash. Requested assistance: "		*/
#define		TRACE_GSM_GPS_ASSISTED_DATA_DOWNLOAD_FAILED				(  89 )		/* TRACE_STRING "GSM: Failed to download AssistedGPS. "					*/
#define		TRACE_GSM_ABANDONED_ATTACH_ATTEMPT						(  90 )		/* TRACE_STRING "GSM: Abandoned any attempt to attach to the network."	*/

#define		TRACE_GSM_TIME_TO_NEXT_TX								(  91 )		/* TRACE_STRING "GSM: Time to next transmission in ticks: "				*/
#define		TRACE_GSM_EXCESSIVE_TEMPERATURE							(  92 )		/* TRACE_STRING "GSM: Temperature exceeds limits for operation: "		*/
#define		TRACE_GSM_PRIVATE_ZONE									(  93 )		/* TRACE_STRING "GSM: Private zone: Keeping GSM shut down."				*/
#define		TRACE_GSM_GPS_NO_RECEPTION								(  94 )		/* TRACE_STRING "GSM: GPS has not received any satellite."				*/
#define		TRACE_GSM_GPSSTARTLOCAL_FAILED							(  95 )		/* TRACE_STRING "GSM: GPS failed to start with local assistance data. Attempt: " */
#define		TRACE_GSM_POSITION_RECORDED								(  96 )		/* TRACE_STRING "GSM: Recorded position, read/write index: "			*/
#define		TRACE_GSM_CGATT_RESULT									(  97 )		/* TRACE_STRING "GSM: +CGATT result: "									*/
#define		TRACE_GSM_ATTACH_REJECT									(  98 )		/* TRACE_STRING "GSM: Received network registration reject."			*/
#define		TRACE_GSM_GPSSTARTRAW_FAILED							(  99 )		/* TRACE_STRING "GSM: GPS failed to raw start. Attempt: "				*/
	
#define		TRACE_CTRL_STANDBY										( 100 )		/* TRACE_STRING "CTRL: STANDBY, event: 0x"								*/
#define		TRACE_CTRL_ACTIVE										( 101 )		/* TRACE_STRING "CTRL: ACTIVE, event: 0x" 								*/
#define		TRACE_CTRL_INACTIVE										( 102 )		/* TRACE_STRING "CTRL: INACTIVE, event: 0x" 							*/
#define		TRACE_CTRL_SLEEP										( 103 )		/* TRACE_STRING "CTRL: SLEEP, event: 0x" 								*/
#define		TRACE_CTRL_OUT_OF_OFFICE								( 104 )		/* TRACE_STRING "CTRL: OUT-OF-OFFICE, event: 0x" 						*/
#define		TRACE_CTRL_CHARGING_OOO									( 105 )		/* TRACE_STRING "CTRL: CHARGING OOO, event: 0x" 						*/
#define		TRACE_CTRL_CHARGING_NORMAL								( 106 )		/* TRACE_STRING "CTRL: CHARGING NORMAL, event: 0x" 						*/
#define		TRACE_CTRL_STILL										( 107 )		/* TRACE_STRING "CTRL: STILL, event: 0x" 								*/
#define		TRACE_CTRL_PREALERT										( 108 )		/* TRACE_STRING "CTRL: PREALERT, event: 0x" 							*/
#define		TRACE_CTRL_ALERT										( 109 )		/* TRACE_STRING "CTRL: ALERT, event: 0x" 								*/
#define		TRACE_CTRL_PRESOS										( 110 )		/* TRACE_STRING "CTRL: PRESOS, event: 0x" 								*/
#define		TRACE_CTRL_SOS											( 111 )		/* TRACE_STRING "CTRL: SOS, event: 0x" 									*/
#define		TRACE_CTRL_ACCELEROMETER_SELF_TEST 						( 112 )		/* TRACE_STRING "CTRL: Accelerometer self-test." 						*/
#define		TRACE_CTRL_VIBRATION_MOTOR_SELF_TEST					( 113 )		/* TRACE_STRING "CTRL: Vibration motor self-test." 						*/
#define		TRACE_CTRL_ABNORMAL_POS_DET_BLOCKED						( 114 )		/* TRACE_STRING "CTRL: Abnormal position detection blocked (beacon)."	*/
#define		TRACE_CTRL_IMMOBILITY									( 115 )		/* TRACE_STRING "CTRL: Immobility zone (beacon)."						*/
																		
#define		TRACE_OS_SEMAPHORE_CREATE_FAILED						( 140 )		/* TRACE_STRING "OS: Semaphore create failed: insufficient heap!" 		*/
#define		TRACE_OS_QUEUE_CREATE_FAILED							( 141 )		/* TRACE_STRING "OS: Queue create failed: insufficient heap!" 			*/
#define		TRACE_OS_TASK_CREATE_FAILED								( 142 )		/* TRACE_STRING "OS: Task create failed: insufficient heap!" 			*/
#define		TRACE_OS_TIMER_CREATE_FAILED							( 143 )		/* TRACE_STRING "OS: Timer create failed: insufficient heap!" 			*/
#define		TRACE_OS_TIMER_QUEUE_FULL								( 144 )		/* TRACE_STRING "OS: Timer command queue full!" 						*/
#define		TRACE_OS_QUEUE_SEND_FAILED								( 145 )		/* TRACE_STRING "OS: Queue send failed: queue full! "					*/
																			
#define		TRACE_VIBR_EVACUATION									( 150 )		/* TRACE_STRING "VIBR: Evacuation request: 0x"	 						*/
#define		TRACE_VIBR_DANGER_ZONE									( 151 )		/* TRACE_STRING "VIBR: In danger zone (beacon)."						*/
#define		TRACE_VIBR_FOREIGN_DANGER_ZONE							( 152 )		/* TRACE_STRING "VIBR: Foreign danger zone (distress beacon)."			*/
																			
#define		TRACE_PAR_XPARSERATRESPQUEUE_FULL						( 160 )		/* TRACE_STRING "PAR: xParserAtRespQueue full!" 						*/
#define		TRACE_PAR_GSMCMDQUEUE_FULL								( 161 )		/* TRACE_STRING "PAR: GsmCmdQueue full!" 								*/
																			
#define		TRACE_MD_NOABN_EXIT										( 170 )		/* TRACE_STRING "MD: Exiting commanded abnormal position detection stop." */
#define		TRACE_MD_ALERT_MSG_FAIL									( 180 )		/* TRACE_STRING "MD: Error: when sending ALERT message to CTRL." 		*/
#define		TRACE_MD_SPATIAL_ORIENTATION_CHANGED					( 181 )		/* TRACE_STRING "MD: Spatial orientation change detected." 				*/
#define		TRACE_MD_ABNORMAL_POSITION								( 182 )		/* TRACE_STRING "MD: Abnormal position detected, count: " 				*/
#define		TRACE_MD_PREALERT										( 186 )		/* TRACE_STRING "MD: Prealert"			 								*/
#define		TRACE_MD_PREALERT_CNCL									( 187 )		/* TRACE_STRING "MD: Prealert cancel timeout."							*/
#define		TRACE_MD_IRQ_LOST										( 188 )		/* TRACE_STRING "MD: Lost accelerometer interrupt."						*/
#define		TRACE_MD_STEP											( 189 )		/* TRACE_STRING "MD: Step detected, count: " 							*/
#define		TRACE_MD_STEP_INACT										( 190 )		/* TRACE_STRING "MD: Step detected in INACTIVCE, count: " 				*/
#define		TRACE_MD_SOS											( 191 )		/* TRACE_STRING "MD: SOS detected, count: " 							*/
#define		TRACE_MD_CANCEL											( 192 )		/* TRACE_STRING "MD: Cancel detected, count: " 							*/
#define		TRACE_MD_ABN_RESET_TILT									( 193 )		/* TRACE_STRING "MD: Abnormal count reset - no tilt: "					*/
#define		TRACE_MD_ABN_RESET_ACC_DELTA							( 194 )		/* TRACE_STRING "MD: Abnormal count reset - acc. delta exceeded limit." */
#define		TRACE_MD_ABN_RESET_MOTION								( 195 )		/* TRACE_STRING "MD: Abnormal count reset - motion detected: "			*/
#define		TRACE_MD_ABN_DET										( 196 )		/* TRACE_STRING "MD: Abnormal position detected -> prealert"			*/
#define		TRACE_MD_CTRL_QUEUE_FULL								( 197 )		/* TRACE_STRING "MD: CTRL event queue full!"							*/
#define		TRACE_MD_CMD											( 198 )		/* TRACE_STRING "MD: Command tap detected, count: " 					*/
#define		TRACE_MD_NOABN_ENTER									( 199 )		/* TRACE_STRING "MD: Entering commanded abnormal position detection stop."*/
		
#define		TRACE_BLE_AT_RESPONSE_TIME_OUT							( 200 )		/* TRACE_STRING "BLE: AT response Time-out: " 							*/
#define		TRACE_BLE_SCAN_ERROR									( 201 )		/* TRACE_STRING "BLE: Error while starting scan." 						*/
#define		TRACE_BLE_LOCALISATION_START							( 202 )		/* TRACE_STRING "BLE: Localisation start." 								*/
#define		TRACE_BLE_LOCALISATION_STOP								( 203 )		/* TRACE_STRING "BLE: Localisation stop." 								*/
#define		TRACE_BLE_ADVERTISING_ERROR								( 204 )		/* TRACE_STRING "BLE: Error while starting advertising." 				*/
#define		TRACE_BLE_LOCKED										( 205 )		/* TRACE_STRING "BLE: Module locked for FW update."						*/
#define		TRACE_BPR_XBLEPARSERATRESPQUEUE_FULL					( 206 )		/* TRACE_STRING "BPR: xBleParserAtRespQueue full." 						*/
#define		TRACE_BLE_SLIP_DECODE_INVALID							( 207 )		/* TRACE_STRING "BLE: SLIP decoding error: invalid packet at line "		*/
#define		TRACE_BLE_PROG_RX_TIMEOUT								( 208 )		/* TRACE_STRING "BLE: FW programmer TO in response at line "			*/
#define		TRACE_BLE_START_SR_SOS									( 209 )		/* TRACE_STRING "BLE: Start BLE short range alert/SOS advertising."		*/
#define		TRACE_BLE_STOP_SR_SOS									( 210 )		/* TRACE_STRING "BLE: Stop BLE short range alert/SOS advertising."		*/
#define		TRACE_BLE_START_SR_EVAC									( 211 )		/* TRACE_STRING "BLE: Start BLE short range EVAC advertising."			*/
#define		TRACE_BLE_STOP_SR_EVAC									( 212 )		/* TRACE_STRING "BLE: Stop BLE short range EVAC advertising."			*/
#define		TRACE_BLE_PROG_RX_MISMATCH								( 213 )		/* TRACE_STRING "BLE: FW programmer response mismatch at line "			*/
#define		TRACE_BLE_PROG_BAD_STATE								( 214 )		/* TRACE_STRING "BLE: FW programmer bad internal state at line "		*/
#define		TRACE_BLE_ENCRYPTION_FAIL								( 215 )		/* TRACE_STRING "BLE: Encryption HW failure."							*/
#define		TRACE_BLE_SLIP_PACKET_TOO_LONG							( 216 )		/* TRACE_STRING "BLE: FW programmer slip packet too long at line "		*/
#define		TRACE_BLE_LR_BEACON_CHKSUM_FAIL							( 217 )		/* TRACE_STRING "BLE: Long range distress beacon checksum failure."		*/
#define		TRACE_BLE_NRF_REBOOT									( 218 )		/* TRACE_STRING "BLE: nRF reboot detected. Reconfiguring."				*/
#define		TRACE_BLE_EXCESSIVE_TEMPERATURE							( 219 )		/* TRACE_STRING "BLE: Temperature exceeds limits for operation: "		*/
#define		TRACE_BLE_SHUTDOWN										( 220 )		/* TRACE_STRING "BLE: Shutdown requested."								*/
#define		TRACE_BLE_SUSPEND_ALL									( 221 )		/* TRACE_STRING "BLE: Suspend all, semaphore count: "					*/
#define		TRACE_BLE_RESUME_ALL									( 222 )		/* TRACE_STRING "BLE: Resume all, semaphore count: "					*/
#define		TRACE_BLE_SUSPEND_TX									( 223 )		/* TRACE_STRING "BLE: Suspend TX, semaphore count: "					*/
#define		TRACE_BLE_RESUME_TX										( 224 )		/* TRACE_STRING "BLE: Resume TX, semaphore count: "						*/
	
#define		TRACE_CHRG_DISABLED										( 230 )		/* TRACE_STRING "CHRG: Charger disabled."								*/
#define		TRACE_CHRG_ENABLED										( 231 )		/* TRACE_STRING "CHRG: Charger enabled (TL501/502/512: controlled for nominal)."*/
#define		TRACE_CHRG_FWCTRL										( 232 )		/* TRACE_STRING "CHRG: Charger set to FW control."						*/
#define		TRACE_CHRG_BATTVOLTAGE									( 233 )		/* TRACE_STRING "CHRG: Battery voltage: "								*/
#define		TRACE_CHRG_BATTCHRGCURRENT								( 234 )		/* TRACE_STRING "CHRG: Battery charge current: "						*/
#define		TRACE_CHRG_BATTTEMPERATURE								( 235 )		/* TRACE_STRING "CHRG: Battery temperature: "							*/
#define		TRACE_CHRG_BATTCHRGPWM									( 236 )		/* TRACE_STRING "CHRG: Battery charge PWM setting: "					*/
#define		TRACE_CHRG_OFF											( 237 )		/* TRACE_STRING "CHRG: Charger off (TL501/502: autonomous)."			*/
/*-----------------------------------------------------------*/

#endif


