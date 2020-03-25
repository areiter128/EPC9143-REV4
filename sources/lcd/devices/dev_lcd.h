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

#ifndef _Dev_LCD_H_
#define	_Dev_LCD_H_

#include <stdint.h>
#include <stdio.h>

#include "lcd/drivers/drv_lcd_interface.h"

#ifdef	__cplusplus
extern "C" {
#endif
    
#define TEMPSTR_LCD_SIZE    40
    
#define PrintLcd(LINE, ...) \
    do{ \
        char __print_utils_string[TEMPSTR_LCD_SIZE]; \
        sprintf(__print_utils_string, __VA_ARGS__); \
        Dev_Lcd_WriteStringXY(0, LINE, __print_utils_string); \
    } while(0)
    
//======================================================================================================================
// @brief initializes the LCD Device, needs to be called once at bootup before that device can be used
//======================================================================================================================
extern void Dev_Lcd_Init(void);

//======================================================================================================================
// @brief   sets the cursor position to the given x- and y-coordinates starting with zero
// @param   x x-coordinates for the new cursor position starting with zero
// @param   y y-coordinates for the new cursor position starting with zero
//======================================================================================================================
extern void Dev_Lcd_GotoXY(uint8_t x,uint8_t y);

//======================================================================================================================
// @brief   writes the character on the lcd screen
// @param    ch is the charactert be written on the lcd screen
// @note    '\f' clears the screen and positions the cursor on the upper left corner,
// @note    '\r' sets the x position of the cursor to 0
// @note    '\n' poairiona the cursor on the next line (without changing the x-position)
//======================================================================================================================
extern void Dev_Lcd_WriteChar(const char ch);

//======================================================================================================================
// @brief   writes the given string on the lcd screen
// @param   str is the string to be written on the lcd screen
// @note    '\f' clears the screen and positions the cursor on the upper left corner,
// @note    '\r' sets the x position of the cursor to 0
// @note    '\n' poairiona the cursor on the next line (without changing the x-position)
//======================================================================================================================
extern void Dev_Lcd_WriteString(const char *str);

//======================================================================================================================
// @brief   sets the cursor position to the given x- and y-coordinates and writes the given string on the lcd screen
// @param   x x-coordinates for the new cursor position starting with zero
// @param   y y-coordinates for the new cursor position starting with zero
// @param   str is the string to be written on the lcd screen
// @note    '\f' clears the screen and positions the cursor on the upper left corner,
// @note    '\r' sets the x position of the cursor to 0
// @note    '\n' poairiona the cursor on the next line (without changing the x-position)
//======================================================================================================================
extern void Dev_Lcd_WriteStringXY(uint8_t column_index, uint8_t line_index, const char *str);


#ifdef	__cplusplus
}
#endif

#endif	// _Dev_LCD_H_

