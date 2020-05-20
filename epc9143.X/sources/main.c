/*
 * File:   main.c
 * Author: M91406
 *
 * Created on July 8, 2019, 1:52 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "main.h"

#include "config/init/init_fosc.h"
#include "config/init/init_timer1.h"
#include "config/init/init_gpio.h"

#include "config/init/init_opa.h"
#include "config/init/init_dac.h"


#define  TMR1_TIMEOUT 30000   // Timeout protection for Timer1 interrupt flag bit
volatile bool LOW_PRIORITY_GO = false;  // Flag allowing low priority tasks to be executed

int main(void) {

    volatile uint16_t test=0;
    
    test=BUCK_PWM_PHASE_SHIFT;
    Nop();
    Nop();
    Nop();
    
    volatile uint16_t timeout = 0;
    
    init_fosc();        // Set up system oscillator for 100 MIPS operation
    init_aclk();        // Set up Auxiliary PLL for 500 MHz (source clock to PWM module)
    init_timer1();      // Set up Timer1 as scheduler time base
    init_gpio();        // Initialize common device GPIOs
    
    
    init_opa(); // Initialize op-amp #2 used to drive the reference voltage for current sense amplifiers
    
    init_dac_module();  // Initialize DAC module
    init_dac_channel(1); // Initialize DAC #1 used to generate the reference voltage for current sense amplifiers
    init_dac_enable(); // Enable DAC setting the reference for current sense amplifiers
    
    appPowerSupply_Initialize(); // Initialize BUCK converter object and state machine
    appFaults_Initialize(); // Initialize fault objects and fault handler task
    
    
    // Enable Timer1
    T1CONbits.TON = 1; 

    _T1IP = 2;  // Set interrupt priority to zero
    _T1IF = 0;  // Reset interrupt flag bit
    _T1IE = 1;  // Enable/Disable Timer1 interrupt
    
    DBGPIN_2_CLEAR;

    
    while (1) {

        // wait for timer1 to overrun
        while ((!LOW_PRIORITY_GO) && (timeout++ < TMR1_TIMEOUT));
        LOW_PRIORITY_GO = false;
        timeout = 0;    // Reset timeout counter

        DBGPIN_2_TOGGLE; // indicate main() loop activity
        
        // Execute non-time critical, low-priority tasks
        /* PLACE LOW_PRIORITY TASKS CALLS HERE */

        Nop();
    }

    return (0);
}

void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    DBGPIN_3_SET; // Set DEBUG-PIN
    
    // Execute high priority, time critical tasks
    appPowerSupply_Execute();
    appFaults_Execute();
  
    LOW_PRIORITY_GO = true; // Set GO trigger for low priority tasks
    
    _T1IF = 0;
    DBGPIN_3_CLEAR; // Clear DEBUG-PIN
    
}


