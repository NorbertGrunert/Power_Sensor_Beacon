/**
 * Copyright (c) 2012 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#ifndef BOARD_CUSTOM_H
#define BOARD_CUSTOM_H

/* TL5xx board definition. */
#include "nrf_gpio.h"
#include "nrf_saadc.h"

/* Tracker packet format ID */
#define TL500_FRMT			"2"

/* Hardware version */
#if defined( TL500 )
	#if  defined ( RELEASE )
		#define HW_VERSION		"TL500-RELEASE"
		#define LEN_HW_VERSION	( 13 )			/* Length of the HW version string. */
	#elif  defined ( DEBUG )
		#define HW_VERSION		"TL500-DEBUG"
		#define LEN_HW_VERSION	( 11 )			/* Length of the HW version string. */
	#endif
#elif defined ( TL501 )
	#if  defined ( RELEASE )
		#define HW_VERSION		"TL501-RELEASE"
		#define LEN_HW_VERSION	( 13 )			/* Length of the HW version string. */
	#elif  defined ( DEBUG )
		#define HW_VERSION		"TL501-DEBUG"
		#define LEN_HW_VERSION	( 11 )			/* Length of the HW version string. */
	#endif
#elif defined ( TL502 )
	#if  defined ( RELEASE )
		#define HW_VERSION		"TL502-RELEASE"
		#define LEN_HW_VERSION	( 13 )			/* Length of the HW version string. */
	#elif  defined ( DEBUG )
		#define HW_VERSION		"TL502-DEBUG"
		#define LEN_HW_VERSION	( 11 )			/* Length of the HW version string. */
	#endif
#elif defined ( TL510 )
	#if  defined ( RELEASE )
		#define HW_VERSION		"TL510-RELEASE"
		#define LEN_HW_VERSION	( 13 )			/* Length of the HW version string. */
	#elif  defined ( DEBUG )
		#define HW_VERSION		"TL510-DEBUG"
		#define LEN_HW_VERSION	( 11 )			/* Length of the HW version string. */
	#endif
#elif defined ( TL512 )
	#if  defined ( RELEASE )
		#define HW_VERSION		"TL512-RELEASE"
		#define LEN_HW_VERSION	( 13 )			/* Length of the HW version string. */
	#elif  defined ( DEBUG )
		#define HW_VERSION		"TL512-DEBUG"
		#define LEN_HW_VERSION	( 11 )			/* Length of the HW version string. */
	#endif
#else
	#error ERROR: Target not defined in makefile!
#endif


/* Signals used for voltage supervision. */
#define VCHRG_SENSE			NRF_GPIO_PIN_MAP( 0,  3 )			/* Analog In  - Voltage measurement on wireless charger input. */
#define VGSM_SENSE			NRF_GPIO_PIN_MAP( 0, 29 )			/* Analog In  - Voltage measurement on cellular supply. */

/* Signals used for the charger control. */	
#define NTC					NRF_GPIO_PIN_MAP( 0,  5 )			/* Analog In  - Battery NTC temperature measurement. */
#define BATT_CHRG			NRF_GPIO_PIN_MAP( 0, 24 )			/* PWM Out    - Battery charge current control. */
#define CHRG_STATE			NRF_GPIO_PIN_MAP( 0, 30 )			/* Analog In  - Read-back from battery charger CHRG output. ATTENTION: shared with buttons/LEDs on DK. */
#define QIEN_N				NRF_GPIO_PIN_MAP( 1,  3 )			/* Out 		  - Qi charger enable. */
												    
/* Signals used for the control of the Cellular module. */
#define GSM_PWR_CTRL		NRF_GPIO_PIN_MAP( 0, 18 )			/* Out 		  - Cellular module power supply control. */
#define STATUS				NRF_GPIO_PIN_MAP( 0, 19 )			/* In		  - Cellular status signal. ATTENTION: shared with QSPI on DK. */
#define GSM_PWR_KEY			NRF_GPIO_PIN_MAP( 0, 20 )			/* Out 		  - Open-drain output power ON cellular module. ATTENTION: shared with QSPI on DK. */
#define GSM_RXD				NRF_GPIO_PIN_MAP( 1,  8 )			/* In 		  - Input UART - cellular. */
#define GSM_TXD				NRF_GPIO_PIN_MAP( 1,  9 )			/* Out 		  - Output UART - cellular. */
#if defined ( TL500 ) || defined ( TL501 ) || defined ( TL502 )
	#define GSM_PONTRIG		NRF_GPIO_PIN_MAP( 0, 21 )			/* Out		  - Wake up the cellular module from PSM. ATTENTION: shared with QSPI on DK. */
	#define GSM_CTS			NRF_GPIO_PIN_MAP( 0, 14 )			/* Out 		  - UART - CTS. ATTENTION: shared with buttons/LEDs on DK. */
	#define GSM_RTS			NRF_GPIO_PIN_MAP( 0, 13 )			/* Out 		  - UART - RTS. ATTENTION: shared with buttons/LEDs on DK. */
#endif
#define GSM_DTR				NRF_GPIO_PIN_MAP( 0, 12 )			/* Out		  - Output UART - GSM_DTR (sleep control). ATTENTION: shared with buttons/LEDs on DK. */
#define HWFC 				false                   
												    
/* Signals used for the accelerometer. */	        
#define ACC_INT1			NRF_GPIO_PIN_MAP( 1, 15 )			/* In  		  - Input Accelerometer interrupt 1. */
#if defined ( TL500 ) || defined ( TL501 ) || defined ( TL502 )
	#define ACC_INT2		NRF_GPIO_PIN_MAP( 1, 13 )			/* In  		  - Input Accelerometer interrupt 2. */
#endif
#define SPI_CS				NRF_GPIO_PIN_MAP( 0, 26 )			/* Out 		  - Output SPI CS. */
#define SPI_SCK				NRF_GPIO_PIN_MAP( 0,  7 )			/* Out 		  - Output SPI SCK. */
#define SPI_MOSI			NRF_GPIO_PIN_MAP( 0, 27 )			/* Out 		  - Output SPI MOSI. */
#define SPI_MISO			NRF_GPIO_PIN_MAP( 0,  6 )			/* In  		  - Input SPI MISO. */
												    
/* Signals used for LED control. */	                
/* LEDs */                                          
#define LED_RED         	NRF_GPIO_PIN_MAP( 0, 10 )			/* Out 		  - Output LED red. ATTENTION: shared with NFC2. */	
#define LED_GREEN       	NRF_GPIO_PIN_MAP( 1, 10 )			/* Out 		  - Output LED green. */
#define LED_BLUE        	NRF_GPIO_PIN_MAP( 1, 11 )			/* Out 		  - Output LED blue. */
												    
/* Signals used for the Trace UART. */	            
#define TRC_RXD				NRF_GPIO_PIN_MAP( 0, 22 )			/* In		  - Input UART - trace. ATTENTION: shared with QSPI on DK. */
#define TRC_TXD				NRF_GPIO_PIN_MAP( 0, 15 )			/* Out		  - Output UART - trace. ATTENTION: shared with buttons/LEDs on DK. */
												    
/* Signals used for the vibration motor. */	        
#define VIBR				NRF_GPIO_PIN_MAP( 1,  5 )			/* Out		  - Output Vibrator. ATTENTION: shared with QSPI on DK. */
	
/* ZOE GPS interface. */                            
#define GPS_SCL_PIN 		NRF_GPIO_PIN_MAP( 0,  8 )			/* Out		  - SCL signal pin. */
#define GPS_SDA_PIN 		NRF_GPIO_PIN_MAP( 0, 11 )			/* InOut	  - SDA signal pin. ATTENTION: shared with buttons/LEDs on DK. */
#define GPS_CTRL_PWR		NRF_GPIO_PIN_MAP( 0, 17 )			/* Out		  - DC/DC power control. ATTENTION: shared with QSPI on DK. */	
#if defined ( TL500 ) || defined ( TL501 ) || defined ( TL502 )
	#define GPS_RXD			NRF_GPIO_PIN_MAP( 0,  4 )			/* Out		  - UART RxD. */	
	#define GPS_EXTINT		NRF_GPIO_PIN_MAP( 0, 31 )			/* Out		  - GPS external interrupt. */	
	#define GPS_TX_RDY		NRF_GPIO_PIN_MAP( 1, 14 )			/* In		  - GPS Tx ready. */	
#endif

/* Debug */
#define DEBUG0	         	NRF_GPIO_PIN_MAP( 0, 23 )			/* Out 		  - Debug. TL500 net: D_IO_Gen1, T63, debug board: (J33-10, BleBlue) */
#define DEBUG1	         	NRF_GPIO_PIN_MAP( 0, 25 )			/* Out 		  - Debug. TL500 net: D_IO_Gen2, T16, debug board: (J35-13) */
#define DEBUG2	         	NRF_GPIO_PIN_MAP( 0, 16 )			/* Out 		  - Debug. TL500 net: D_IO_Gen3, T9,  debug board: (J35-10, VBatSense) */
#define DEBUG3	         	NRF_GPIO_PIN_MAP( 1, 00 )			/* Out 		  - Debug. TL500 net: SWO_DBG, JT1-4, debug board: PDI_DAT */
#define DEBUG4	         	NRF_GPIO_PIN_MAP( 1, 01 )			/* Out 		  - Debug. TL500 net: D_IO_LF, JT1-3, debug board: PDI_CLKRST */
#define DEBUG5	         	NRF_GPIO_PIN_MAP( 1, 12 )			/* Out 		  - Debug. TL500 net: D_IO_LF1, solder island */
#define DEBUG6	         	NRF_GPIO_PIN_MAP( 0, 28 )			/* Out 		  - Debug. TL500 net: A_IN_LF1, solder island */
#define DEBUG7	         	NRF_GPIO_PIN_MAP( 0, 02 )			/* Out 		  - Debug. TL500 net: A_IN_LF2, solder island */

/* ADC channel and pin allocations. The pin numbering for the ADC does not follow the GPIO numbering scheme. 
   Analog positive and negative input channels have strange numbering:				
			NRF_SAADC_INPUT_DISABLED          	not connected                        
			NRF_SAADC_INPUT_AIN0  		 		AIN0			P0.02		A12         
			NRF_SAADC_INPUT_AIN1  		 		AIN1			P0.03		B13         
			NRF_SAADC_INPUT_AIN2  		 		AIN2			P0.04		J1          
			NRF_SAADC_INPUT_AIN3  		 		AIN3			P0.05		K2          
			NRF_SAADC_INPUT_AIN4  		 		AIN4			P0.28		B11         
			NRF_SAADC_INPUT_AIN5  		 		AIN5			P0.29		A10		    
			NRF_SAADC_INPUT_AIN6  		 		AIN6			P0.30		B9          
			NRF_SAADC_INPUT_AIN7  		 		AIN7			P0.31		A8          
			NRF_SAADC_INPUT_VDD          		VDD									
			NRF_SAADC_INPUT_VDDHDIV5     		VDDH/5								*/
#define VBAT_ADC_PIN			NRF_SAADC_INPUT_VDDHDIV5
#define VCC_ADC_PIN				NRF_SAADC_INPUT_VDD
#define VCHRG_ADC_PIN			NRF_SAADC_INPUT_AIN1
#define VCHRG_STATE_ADC_PIN		NRF_SAADC_INPUT_AIN6
#define VGSM_ADC_PIN			NRF_SAADC_INPUT_AIN5
#define NTC_ADC_PIN				NRF_SAADC_INPUT_AIN3

/* LPCOMP pin allocation. The pin numbering for the LPCOMP does not follow the GPIO numbering scheme. 
			NRF_LPCOMP_INPUT_0  		 		AIN0			P0.02		A12         
			NRF_LPCOMP_INPUT_1  		 		AIN1			P0.03		B13         
			NRF_LPCOMP_INPUT_2  		 		AIN2			P0.04		J1          
			NRF_LPCOMP_INPUT_3  		 		AIN3			P0.05		K2          
			NRF_LPCOMP_INPUT_4  		 		AIN4			P0.28		B11         
			NRF_LPCOMP_INPUT_5  		 		AIN5			P0.29		A10		    
			NRF_LPCOMP_INPUT_6  		 		AIN6			P0.30		B9          
			NRF_LPCOMP_INPUT_7  		 		AIN7			P0.31		A8      */
#define VCHRG_LPCOMP_PIN		NRF_LPCOMP_INPUT_1

/* Define the ADC channels. The ADC channel is used as index into the channel configuration table, thus be careful that the index defined
   here matches that table. */
enum xADC_CHANNEL
{
	NTC_ADC			= 0,		/*  Battery NTC. */
	CHRG_STATE_ADC	= 1,		/*  Charge state pin for the LiIon charge controller. */
	VGSM_ADC		= 2,		/*  GSM module supply voltage. */
	VBAT_ADC		= 3,		/*  Battery voltage. */
	VCHRG_ADC		= 4,		/*  Wireless charger voltage. */
	VCC_ADC			= 5			/*  VCC */
};	
#define ADC_CHANNEL_LAST_IDX	5

/* Define which physical UART is going to be used for the AT-interface to the cellular module and the trace interface. */
#define COM_GSM					COM0   
#define COM_TRC					COM1

#endif