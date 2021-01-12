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

#ifdef __cplusplus
extern "C" {
#endif

#ifdef NINA_B1
	// NINA-B1 board definition
	#include "nrf_gpio.h"

	// In this case PIN 16 is used as button SW1, if the green led
	// should be used it is possible to defined that one instead.
	#define LEDS_NUMBER 	3

	#define LED_1 			8 						// Red
	#define LED_2 			18 						// Blue
	#define LED_3 			16 						// Green

	#define LEDS_ACTIVE_STATE 0

	#define LEDS_LIST { LED_1, LED_2 }

	#define BSP_LED_0 		LED_1
	#define BSP_LED_1 		LED_2
	#define BSP_LED_2 		LED_3

	#define LEDS_INV_MASK 	LEDS_MASK

	#define BUTTONS_NUMBER 	2

	//#define BUTTON_1 		16 						// SW1
	#define BUTTON_2 		30 						// SW2
	#define BUTTON_PULL 	NRF_GPIO_PIN_PULLUP

	#define BUTTONS_ACTIVE_STATE 0

	#define BUTTONS_LIST { BUTTON_2 }

	#define BSP_BUTTON_0 	BUTTON_2

	#define RX_PIN_NUMBER 	5
	#define TX_PIN_NUMBER 	6
	#define CTS_PIN_NUMBER 	7
	#define RTS_PIN_NUMBER 	31
	#define DTR_PIN_NUMBER 	28
	#define DSR_PIN_NUMBER 	29
	#define HWFC 			true

	#define SPIS_MISO_PIN 	12 						// SPI MISO signal.
	#define SPIS_CSN_PIN 	11 						// SPI CSN signal.
	#define SPIS_MOSI_PIN 	13 						// SPI MOSI signal.
	#define SPIS_SCK_PIN 	14 						// SPI SCK signal.
	#define SPIM0_SCK_PIN 	14 						// SPI clock GPIO pin number.

	#define SPIM0_MOSI_PIN 	13 						// SPI Master Out Slave In GPIO pin number.
	#define SPIM0_MISO_PIN 	12 						// SPI Master In Slave Out GPIO pin number.
	#define SPIM0_SS_PIN 	11 						// SPI Slave Select GPIO pin number.

	// Low frequency clock source to be used by the SoftDevice
	#define NRF_CLOCK_LFCLKSRC {.source = NRF_CLOCK_LF_SRC_XTAL, 					\
								.rc_ctiv = 0, 										\
								.rc_temp_ctiv = 0, 									\
								.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
								
#else
	// NINA-B3 board definition
	#include "nrf_gpio.h"
	
	// In this file PIN 25 is used as button SWITCH_1, if the GREEN led
	// should be used it is possible to defined that one instead.
	#define LEDS_NUMBER 	3
	#define LED_1 			NRF_GPIO_PIN_MAP(0, 13)		// RED
	#define LED_2 			NRF_GPIO_PIN_MAP(1, 00)		// BLUE
	#define LED_3 			NRF_GPIO_PIN_MAP(0, 25)		// GREEN	
	
	#define LEDS_ACTIVE_STATE 0	
	
	#define LEDS_LIST 		{ LED_1, LED_2, LED_3 }
	#define LEDS_INV_MASK 	LEDS_MASK	
	
	#define BSP_LED_0 		LED_1
	#define BSP_LED_1 		LED_2
	#define BSP_LED_2 		LED_3

	// #define BUTTONS_NUMBER 	2
	// #define BUTTON_1 		25 						// SWITCH_1
	// #define BUTTON_2 		2 						// SWITCH_2
	#define BUTTONS_NUMBER 	1
	#define BUTTON_1 		2							// SWITCH_2
	
	#define BUTTON_PULL 	NRF_GPIO_PIN_PULLUP
	
	#define BUTTONS_ACTIVE_STATE 0
	
	// #define BUTTONS_LIST 	{ BUTTON_1, BUTTON_2 }
	#define BUTTONS_LIST 	{ BUTTON_1 }
	
	#define BSP_BUTTON_0 	BUTTON_1
	// #define BSP_BUTTON_1 	BUTTON_2
	
	#define RX_PIN_NUMBER 	NRF_GPIO_PIN_MAP(0, 29)
	#define TX_PIN_NUMBER 	NRF_GPIO_PIN_MAP(1, 13)
	#define CTS_PIN_NUMBER 	NRF_GPIO_PIN_MAP(1, 12)
	#define RTS_PIN_NUMBER 	NRF_GPIO_PIN_MAP(0, 31)
	#define HWFC 			true
	
	#define BSP_QSPI_SCK_PIN 19
	#define BSP_QSPI_CSN_PIN 17
	#define BSP_QSPI_IO0_PIN 20
	#define BSP_QSPI_IO1_PIN 21
	#define BSP_QSPI_IO2_PIN 22
	#define BSP_QSPI_IO3_PIN 23
	
	// Arduino board mappings
	#define ARDUINO_SCL_PIN 24							// SCL signal pin
	#define ARDUINO_SDA_PIN 16							// SDA signal pin
	#define ARDUINO_13_PIN 	NRF_GPIO_PIN_MAP(0, 7)
	#define ARDUINO_12_PIN 	NRF_GPIO_PIN_MAP(0, 2)
	#define ARDUINO_11_PIN 	NRF_GPIO_PIN_MAP(0, 15)
	#define ARDUINO_10_PIN 	NRF_GPIO_PIN_MAP(0, 14)
	#define ARDUINO_9_PIN 	NRF_GPIO_PIN_MAP(0, 12)
	#define ARDUINO_8_PIN 	NRF_GPIO_PIN_MAP(1, 9)
	#define ARDUINO_7_PIN 	NRF_GPIO_PIN_MAP(0, 10)
	#define ARDUINO_6_PIN 	NRF_GPIO_PIN_MAP(0, 9)
	#define ARDUINO_5_PIN 	NRF_GPIO_PIN_MAP(0, 11)
	#define ARDUINO_4_PIN 	NRF_GPIO_PIN_MAP(0, 13)
	#define ARDUINO_3_PIN 	NRF_GPIO_PIN_MAP(0, 31)
	#define ARDUINO_2_PIN 	NRF_GPIO_PIN_MAP(1, 12)
	#define ARDUINO_1_PIN 	NRF_GPIO_PIN_MAP(1, 13)	
#endif
								
#ifdef __cplusplus
}

#endif

#endif // NINA_B1_H