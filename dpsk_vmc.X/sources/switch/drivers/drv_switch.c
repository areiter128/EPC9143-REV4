/*
 * File:   switch.c
 * Author: M91406
 *
 * Created on March 12, 2020, 12:10 PM
 */


#include "switch/app_switch.h"
#include "config/dpsk3_hwdescr.h"

// PRIVATE VARIABLE DELARATIONS

volatile uint16_t drv_Switch_Initialize(volatile SWITCH_OBJECT_t* swbtn) 
{
    volatile uint16_t fres = 1;
    
    SW_USER_INIT;   // Initialize GPIO used to read switch

    // Initializing switch object 
    swbtn->debounce_delay = 0;              // Clear debounce delay
    swbtn->long_press_delay = 0;            // Clear 'long press" delay
    swbtn->status.bits.pressed = false;     // Reset PRESSED status
    swbtn->status.bits.long_press = false;  // Reset LONG_PRESS status
    swbtn->status.bits.enabled = false;     // Turn off Switch Button
    swbtn->status.bits.sw_event = false;    // Clear SWITCH EVENT flag 
    
    return(fres);
}

volatile uint16_t drv_Switch_Execute(volatile SWITCH_OBJECT_t* swbtn) 
{
    volatile uint16_t fres = 1; // Return value
    static uint16_t press_cnt, release_cnt;     // local counters of SWITCH_USER being pressed/released
    static bool pre_pressed = false, pre_long_press = false;
    
    // If switch is disabled, exit here
    if (!swbtn->status.bits.enabled)
    {
        swbtn->status.bits.pressed = false; // Mark Switch as RELEASED
        return(1);  // Exit function
    }
    
	// Trigger on a PRESSED event with applied debouncing
	if ((!SW_USER_PORTx) && (!swbtn->status.bits.pressed)) { 
		
        // switch button PRESSED event
        if (++press_cnt > swbtn->debounce_delay) {
            swbtn->status.bits.pressed = true; // Set PRESSED flag
            if (swbtn->event_btn_down != NULL)   // Raise Button Event
                swbtn->event_btn_down();
        }
	}
	// Trigger on a LONG PRESS event with applied debouncing
    else if ((!SW_USER_PORTx) && (swbtn->status.bits.pressed)) {

        // switch button LONG PRESS event
        if (++press_cnt > swbtn->long_press_delay) {
            
            if (!swbtn->status.bits.long_press)    // Long Press Event is triggered for the first time
            if (swbtn->event_long_press != NULL)   // Raise Button Long Press Event
                swbtn->event_long_press();
                
            swbtn->status.bits.long_press = true;  // Set LONG_PRESS flag
            press_cnt = swbtn->long_press_delay; // Clamp counter to threshold
            
            if (swbtn->event_pressed != NULL)   // Raise Button Pressed Event
                swbtn->event_pressed();
            
        }
        
    }
    // Trigger on a RELEASE event with applied debouncing
	else if ((SW_USER_PORTx) && (swbtn->status.bits.pressed)) {  
        
        // switch button RELEASE event
        if (++release_cnt > swbtn->debounce_delay) {
            if (swbtn->event_release != NULL)       // Raise Button Event
                swbtn->event_release();
            swbtn->status.bits.pressed = false;      // Clear PRESSED flag
            swbtn->status.bits.long_press = false;   // Clear LONG_PRESS flag
            release_cnt = swbtn->debounce_delay; // Clamp counter to threshold
        }
    }
    else {
        press_cnt = 0;   // Clear switch debounce counter PRESSED
        release_cnt = 0; // Clear switch debounce counter RELEASE
    }
    
    // Trigger on switch events
    swbtn->status.bits.sw_event = (bool) //(pre_pressed != swbtn->status.pressed) ;
        ((pre_pressed != swbtn->status.bits.pressed) || (pre_long_press != swbtn->status.bits.long_press));
    
    pre_pressed = swbtn->status.bits.pressed;
    pre_long_press = swbtn->status.bits.long_press;
        
    return(fres);
}

volatile uint16_t drv_Switch_Dispose(volatile SWITCH_OBJECT_t* swbtn) 
{
    volatile uint16_t fres = 1;
    
    swbtn->debounce_delay = 0;
    swbtn->status.bits.pressed = false;
    swbtn->status.bits.long_press = false;
    swbtn->status.bits.enabled = false;
    SW_USER_INIT;

    return(fres);
}
