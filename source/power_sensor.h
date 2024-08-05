/*
 * Tracker Firmware
 *
 * Parser header file
 *
 */ 
#ifndef POWER_SENSOR_H
#define POWER_SENSOR_H

#include "drv_uart.h"
/*-----------------------------------------------------------*/

/* Size of the Sensor UART Rx ring buffer. */
#define	powerSensorUART_RX_BUFFER_SIZE			uartRX_BUFFER_SIZE

#define ADV_UPDATE_INTERVAL						2  * portTICKS_PER_SEC

/* Sensor data as received by the UART. */
#define SENSOR_PACKET_INTERVAL					100							/* ms */
#define SENSOR_PACKET_DURATION					55							/* ms */
#define SENSOR_RX_TIMEOUT						15							/* ms */
#define SENSOR_RX_LENGTH						24							/* Octets */

#define SENSOR_HEADER1_OFFS						0							/* Header1 */
#define SENSOR_HEADER1_VALID					0x55
#define SENSOR_HEADER1_NOT_CALIBRATED			0xaa
#define SENSOR_HEADER1_ABNORMAL_MSK				0xf0
#define SENSOR_HEADER1_ABNORMAL					0xf0
#define SENSOR_HEADER1_ERROR_MSK				0x0f
#define SENSOR_HEADER1_VCYCL_EXCEED				0x08						/* Voltage cycle exceeds range. */
#define SENSOR_HEADER1_ICYCL_EXCEED				0x04						/* Current cycle exceeds range. */
#define SENSOR_HEADER1_PCYCL_EXCEED				0x02						/* Power cycle exceeds range. */
#define SENSOR_HEADER1_COEFF_ERROR				0x01						/* Coefficient storage area (3n0h~3nFh) is abnormal (n=A, B, C, D, E, F). */

#define SENSOR_HEADER2_OFFS						1							/* Header2 */
#define SENSOR_HEADER2_VALID					0x5a

#define SENSOR_UKH_OFFS							2							/* Voltage coefficient */
#define SENSOR_UKM_OFFS							3
#define SENSOR_UKL_OFFS							4

#define SENSOR_UTH_OFFS							5							/* Voltage cycle */
#define SENSOR_UTM_OFFS							6
#define SENSOR_UTL_OFFS							7

#define SENSOR_IKH_OFFS							8							/* Current coefficient */
#define SENSOR_IKM_OFFS							9
#define SENSOR_IKL_OFFS							10

#define SENSOR_ITH_OFFS							11							/* Current cycle */
#define SENSOR_ITM_OFFS							12
#define SENSOR_ITL_OFFS							13

#define SENSOR_PKH_OFFS							14							/* Power coefficient */
#define SENSOR_PKM_OFFS							15
#define SENSOR_PKL_OFFS							16

#define SENSOR_PTH_OFFS							17							/* Power cycle */
#define SENSOR_PTM_OFFS							18
#define SENSOR_PTL_OFFS							19

#define SENSOR_ADJ_OFFS							20							/* Adjust */

#define SENSOR_CFM_OFFS							21							/* Number of CF pulses */
#define SENSOR_CFL_OFFS							22

#define SENSOR_CHKSUM_OFFS						23							/* Packet checksum */

/* Sensor data as transmitted via BLE. */
#define SENSOR_HEADER1_ERROR					0x10
#define SENSOR_HEADER2_ERROR					0x20
#define SENSOR_UNCALIBRATED_ERROR				0x40
#define SENSOR_CHKSUM_ERROR						0x80

#define SENSOR_TX_SYSTEM_ID_ID					0							/* System ID field. */
#define SENSOR_TX_SENSOR_ID_ID					1							/* Sensor ID field. */
#define SENSOR_TX_STATUS_ID						2							/* Status field. */
#define SENSOR_TX_VOLTAGE_ID					3							/* Voltage field. */
#define SENSOR_TX_CURRENT_ID					4							/* Current field. */
#define SENSOR_TX_POWER_ID						5							/* Power field. */

#define SENSOR_TX_SYSTEM_ID_OFFS				0							/* System ID field. */
#define SENSOR_TX_SENSOR_ID_OFFS				3							/* Sensor ID field. */
#define SENSOR_TX_STATUS_OFFS					6							/* Status field. */
#define SENSOR_TX_VOLTAGE_OFFS					10							/* Voltage field. */
#define SENSOR_TX_CURRENT_OFFS					16							/* Current field. */
#define SENSOR_TX_POWER_OFFS					22							/* Power field. */

#define ID_OFFS									0
#define LEN_OFFS								1
#define DATA_OFFS								2

#define SYSTEM_ID								0x55						/* System ID field. */
#define SENSOR_ID								1							/* Sensor ID field. */
/*-----------------------------------------------------------*/

/* Public function prototypes. */
extern void vPowerSensorInit( UBaseType_t uxPriority );
/*-----------------------------------------------------------*/

/* Global variables. */
/*-----------------------------------------------------------*/

#endif