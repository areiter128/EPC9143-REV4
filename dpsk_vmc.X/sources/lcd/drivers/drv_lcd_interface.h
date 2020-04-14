//======================================================================================================================
// Copyright(c) 2018 Microchip Technology Inc. and its subsidiaries.
// Subject to your compliance with these terms, you may use Microchip software and any derivatives exclusively with
// Microchip products. It is your responsibility to comply with third party license terms applicable to your use of
// third-party software (including open source software) that may accompany Microchip software.
// THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO
// THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR
// PURPOSE.
// IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE,
// COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED
// OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY
// ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE
// PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
//======================================================================================================================

//======================================================================================================================
// @file drv_lcd_interface.h
//
// @brief driver for the spi-interface to access the lcd controller
//
//======================================================================================================================

#ifndef _Dev_LCD_INTERFACE_H_
#define	_Dev_LCD_INTERFACE_H_

#include <stdint.h>
//#include <xc.h>
//#include "delay.h"

#ifdef	__cplusplus
extern "C" {
#endif

//======================================================================================================================
//	defines to access the pins
//======================================================================================================================
/*
#define LCD_RS_COMMAND     LCD_RS_SetLow
#define LCD_RS_DATA        LCD_RS_SetHigh

#define LCD_CS_nSELECTED   LCD_CS_SetHigh
#define LCD_CS_SELECTED    LCD_CS_SetLow
*/

//======================================================================================================================
//  @brief  this function initialized the lcd interface driver
//  @note   use this one time after booting up your system to initialize before calling something else
//======================================================================================================================
extern void Drv_Lcd_Interface_Init(void);

//======================================================================================================================
//  @brief  this function resets the lcd controller
//  @note   use this one time after booting up your system to reset the lcd controller and get into a defined state
//======================================================================================================================
extern void Drv_Lcd_Interface_Reset(void);

//======================================================================================================================
//  @brief  this functions sends a command to the lcd controller
//======================================================================================================================
extern void Drv_Lcd_Interface_SendCmd(const uint8_t cmd);

//======================================================================================================================
//  @brief this function sends one character to the display controller to be displayed on the ldc screen
//======================================================================================================================
extern void Drv_Lcd_Interface_SendChar(const char c);

#ifdef	__cplusplus
}
#endif

#endif // _Dev_LCD_INTERFACE_H_