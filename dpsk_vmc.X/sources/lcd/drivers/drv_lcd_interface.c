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
// @file drv_lcd_interface.c
//
// @brief driver for the spi-interface to access the lcd controller
//
//======================================================================================================================

#include <stdint.h>
#include "common/delay.h"

//======================================================================================================================
//	defines: to access the pins
//======================================================================================================================

#define LCD_SCL_SetHigh()           _LATC5 = 1
#define LCD_SCL_SetLow()            _LATC5 = 0
#define LCD_SCL_Toggle()            _LATC5 ^= 1
#define LCD_SCL_GetValue()          _RC5
#define LCD_SCL_SetDigitalInput()   _TRISC5 = 1
#define LCD_SCL_SetDigitalOutput()  _TRISC5 = 0

#define LCD_SDI_SetHigh()           _LATC4 = 1
#define LCD_SDI_SetLow()            _LATC4 = 0
#define LCD_SDI_Toggle()            _LATC4 ^= 1
#define LCD_SDI_GetValue()          _RC4
#define LCD_SDI_SetDigitalInput()   _TRISC4 = 1
#define LCD_SDI_SetDigitalOutput()  _TRISC4 = 0

#define LCD_CS_SetHigh()            _LATD8 = 1
#define LCD_CS_SetLow()             _LATD8 = 0
#define LCD_CS_Toggle()             _LATD8 ^= 1
#define LCD_CS_GetValue()           _RD8
#define LCD_CS_SetDigitalInput()    _TRISD8 = 1
#define LCD_CS_SetDigitalOutput()   _TRISD8 = 0

#define LCD_RST_SetHigh()           _LATC8 = 1
#define LCD_RST_SetLow()            _LATC8 = 0
#define LCD_RST_Toggle()            _LATC8 ^= 1
#define LCD_RST_GetValue()          _RC8
#define LCD_RST_SetDigitalInput()   _TRISC8 = 1
#define LCD_RST_SetDigitalOutput()  _TRISC8 = 0

#define LCD_RS_SetHigh()            _LATC9 = 1
#define LCD_RS_SetLow()             _LATC9 = 0
#define LCD_RS_Toggle()             _LATC9 ^= 1
#define LCD_RS_GetValue()           _RC9
#define LCD_RS_SetDigitalInput()    _TRISC9 = 1
#define LCD_RS_SetDigitalOutput()   _TRISC9 = 0


#define LCD_CS_nSELECTED    LCD_CS_SetHigh
#define LCD_CS_SELECTED     LCD_CS_SetLow

#define LCD_RS_COMMAND      LCD_RS_SetLow
#define LCD_RS_DATA         LCD_RS_SetHigh

//======================================================================================================================
//  @brief  this function initialized the lcd interface driver
//  @note   use this one time after booting up your system to initialize before calling something else
//======================================================================================================================
void Drv_Lcd_Interface_Init(void)
{
    LCD_RST_SetHigh();
    LCD_CS_nSELECTED();
    LCD_SCL_SetLow();
    LCD_SDI_SetLow();
    LCD_RS_SetLow();
    
    LCD_RST_SetDigitalOutput();
    LCD_CS_SetDigitalOutput();
    LCD_SCL_SetDigitalOutput();
    LCD_SDI_SetDigitalOutput();
    LCD_RS_SetDigitalOutput();
}

//======================================================================================================================
//  @brief  this function sends data through the spi interface to the lcd controller
//  @note   local function
//======================================================================================================================
void Drv_Lcd_Interface_Spi_Send(uint8_t data)
{
    uint8_t mask;

    for(mask = 0x80; mask != 0; mask >>= 1)
    {
        LCD_SCL_SetLow();
        Nop();
        __delay_us(1);
        if(data & mask)
            LCD_SDI_SetHigh();
        else
            LCD_SDI_SetLow();
        LCD_SCL_SetHigh();
        Nop();
        __delay_us(1);
    }
    LCD_CS_nSELECTED();
    __delay_us(50);
}


//======================================================================================================================
//  @brief  this function resets the lcd controller
//  @note   use this one time after booting up your system to reset the lcd controller and get into a defined state
//======================================================================================================================
void Drv_Lcd_Interface_Reset(void)
{
    LCD_RST_SetLow();
    __delay_ms(25);
    LCD_RST_SetHigh();
}


//======================================================================================================================
//  @brief  this functions sends a command to the lcd controller
//======================================================================================================================
void Drv_Lcd_Interface_SendCmd(uint8_t cmd)
{		
   LCD_CS_SELECTED();
   LCD_RS_COMMAND();
   Drv_Lcd_Interface_Spi_Send(cmd);
}


//======================================================================================================================
//  @brief this function sends one character to the display controller to be displayed on the lcd screen
//======================================================================================================================
extern void Drv_Lcd_Interface_SendChar(const char c)
{
   LCD_CS_SELECTED();
   LCD_RS_DATA();
   Drv_Lcd_Interface_Spi_Send(c);
}


