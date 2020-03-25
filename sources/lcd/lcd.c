/*
 * File:   lcd.c
 * Author: M91406
 *
 * Created on March 12, 2020, 12:10 PM
 */

#include "lcd/lcd.h"

// Additional header files required by this app
#include "config/dpsk3_hwdescr.h"
#include "pwr_control/pwr_control.h"
#include "fault_handler/faults.h"

// PRIVATE VARIABLE DELARATIONS
volatile uint16_t lcd_cnt = 0;  // local counter determining LCD refresh rate
#define LCD_STARTUP   30000
#define LCD_REFRESH   2000

volatile LCD_t lcd;

volatile uint16_t appLCD_Initialize(void) 
{
    volatile uint16_t fres = 1;
    
    if (lcd.refresh == 0)
        lcd.refresh = LCD_STARTUP;
    
    Dev_Lcd_Init();
    Dev_Lcd_WriteStringXY(0,0,"==== DPSK-3 ====");
    Dev_Lcd_WriteStringXY(0,1," CE201  VMC+AGC ");

    lcd_cnt = 0;
    lcd.enabled = true;
    
    return(fres);
}

volatile uint16_t appLCD_Execute(void) 
{
    volatile uint16_t fres = 1;
    volatile float vi=0.0, vo=0.0;
    
    // IF LCD output is disabled, exit here
    if(!lcd.enabled)
        return(fres);
    
    // Refresh LCD and reset refresh counter
    lcd_cnt++;
    
    // If REFRESH period has expired, update LCD contents
    if(lcd_cnt == lcd.refresh) {
        
        // Input voltage display
        vi = ((buck.data.v_in << 3) * ADC_GRAN);
        if(vi < 10.0)
            PrintLcd(0, "VIN     = %2.2f V", (double)vi);
        else
            PrintLcd(0, "VIN     =%2.2f V", (double)vi);

        
        // Output voltage display
        vo = ((buck.data.v_out << 1) * ADC_GRAN);
        if (!buck.status.bits.fault_active)
            PrintLcd(1, "VOUT    = %2.2f V", (double)vo);
        else {
            if (fltobj_BuckUVLO.status.bits.fault_status)
                PrintLcd(1, "VOUT(UV)= %2.2f V", (double)vo);
            else if (fltobj_BuckOVLO.status.bits.fault_status)
                PrintLcd(1, "VOUT(OV)= %2.2f V", (double)vo);
            else if (fltobj_BuckRegErr.status.bits.fault_status)
                PrintLcd(1, "VOUT(RE)= %2.2f V", (double)vo);
        }
        
        lcd.refresh = LCD_REFRESH;
        
        lcd_cnt = 0; // Reset internal interval counter
    }
    
    return(fres);
}

volatile uint16_t appLCD_Dispose(void) 
{
    volatile uint16_t fres = 1;
    
    /* PLACE DISPOSE CODE HERE */

    return(fres);
}
