/*
 * File:   switch.c
 * Author: M91406
 *
 * Created on March 12, 2020, 12:10 PM
 */


#include "switch.h"
#include "config/dpsk3_hwdescr.h"
#include "pwr_control/pwr_control.h"
#include "lcd/lcd.h"

// PRIVATE VARIABLE DELARATIONS

#define SWITCH_DEBOUNCE_DELAY_DEFAULT    199    // Switch needs to be pressed >20ms to trip a SWITCH BUTTON event
#define SWITCH_LONG_PRESS_DELAY_DEFAULT  4999   // Switch needs to be pressed >500ms to trip a SWITCH BUTTON event

volatile SWITCH_OBJECT_t switch_button;

volatile uint16_t appSwitch_Initialize(void) 
{
    volatile uint16_t fres = 1;
    
    fres &= drv_Switch_Initialize(&switch_button);   
    
    switch_button.debounce_delay = SWITCH_DEBOUNCE_DELAY_DEFAULT;
    switch_button.long_press_delay = SWITCH_LONG_PRESS_DELAY_DEFAULT;
    switch_button.status.bits.enabled = true;
    
    switch_button.event_btn_down = &appSwitch_EventButtonDown;
    switch_button.event_long_press = &appSwitch_EventButtonLongPress;
    switch_button.event_pressed = NULL; // Event not used
    switch_button.event_release = &appSwitch_EventButtonUp;
    
    return(fres);
}

volatile uint16_t appSwitch_Execute(void) 
{
    volatile uint16_t fres = 1;

    fres &= drv_Switch_Execute(&switch_button);

    return(fres);
}

volatile uint16_t appSwitch_EventButtonDown(void) {
    Nop();
    return(1);
}

volatile uint16_t appSwitch_EventButtonLongPress(void) {
    
    lcd.screen ^= 1;
    return(1);
}

volatile uint16_t appSwitch_EventButtonUp(void) {

    return(1);
}

volatile uint16_t appSwitch_Dispose(void) 
{
    volatile uint16_t fres = 1;
    
    fres &= drv_Switch_Dispose(&switch_button);

    return(fres);
}
