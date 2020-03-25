/*
 * File:   switch.c
 * Author: M91406
 *
 * Created on March 12, 2020, 12:10 PM
 */


#include "switch.h"
#include "config/dpsk3_hwdescr.h"
#include "pwr_control/pwr_control.h"

// PRIVATE VARIABLE DELARATIONS

volatile uint16_t sw_cnt = 0; // local counter of SWITCH_USER being pressed
#define SWITCH_DEBOUNCE_DELAY_DEFAULT    199 // Switch needs to be pressed >200ms to trip a SWITCH BUTTON event

volatile SWITCH_OBJECT_t switch_button;

volatile uint16_t appSwitch_Initialize(void) 
{
    volatile uint16_t fres = 1;
    
    if(switch_button.debounce_delay == 0) // if debounce delay has not been configured...
        switch_button.debounce_delay = SWITCH_DEBOUNCE_DELAY_DEFAULT; // Set default debounce delay
    SW_USER_INIT;

    switch_button.status.enabled = true; // Turn on Switch Button
    
    return(fres);
}

volatile uint16_t appSwitch_Execute(void) 
{
    volatile uint16_t fres = 1;

    if (!switch_button.status.enabled)
    {
        switch_button.status.pressed = false; // Mark Switch as RELEASED
        return(1);  // Exit function
    }
    
	// Enable/Disable AGM when the user pushes the SW_USER button
	if(!SW_USER_PORTx){ 
		
		if ((sw_cnt++ > SWITCH_DEBOUNCE_DELAY_DEFAULT) && (!switch_button.status.pressed)){
			// Add code for switch button event
			switch_button.status.pressed = true;
		}
	}
	else if (SW_USER_PORTx && switch_button.status.pressed) {
        switch_button.status.pressed = false; 
    }
	else
	{ sw_cnt = 0; } // Reset switch counter

    return(fres);
}

volatile uint16_t appSwitch_Dispose(void) 
{
    volatile uint16_t fres = 1;
    
    switch_button.debounce_delay = 0;
    SW_USER_INIT;

    return(fres);
}
