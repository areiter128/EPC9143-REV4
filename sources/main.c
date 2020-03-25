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


#define TMR1_TIMEOUT        30000   // Timeout protection for Timer1 interrupt flag bit
volatile bool LOW_PRIO_GO = false;  // Flag allowing low priority tasks to be executed

int main(void) {

    volatile uint16_t timeout = 0;
    
    init_fosc();        // Set up system oscillator for 100 MIPS operation
    init_aclk();        // Set up Auxiliary PLL for 500 MHz (source clock to PWM module)
    init_timer1();      // Set up Timer1 as scheduler time base
    init_gpio();        // Initialize common device GPIOs
    
    
    appLCD_Initialize(); // Initialize LC Display task
    appLED_Initialize(); // Initialize Debugging LED task
    appSwitch_Initialize(); // Initialize user switch button
    appPowerSupply_Initialize(); // Initialize BUCK converter object and state machine
    appFaults_Initialize(); // Initialize fault objects and fault handler task
    
    
    // Enable Timer1
    T1CONbits.TON = 1; 

    _T1IP = 1;  // Set interrupt priority to zero
    _T1IF = 0;  // Reset interrupt flag bit
    _T1IE = 1;  // Disable Timer1 interrupt
    
    DBGPIN_2_CLEAR;
    DBGPIN_3_CLEAR;
    
    while (1) {

        // wait for timer1 to overrun
        while ((!LOW_PRIO_GO) && (timeout++ < TMR1_TIMEOUT));
        LOW_PRIO_GO = false;
        timeout = 0;    // Reset timeout counter

        DBGPIN_3_SET; // Set DEBUG-PIN

        // Execute non-time critical, low-priority tasks
        appLCD_Execute();
        appLED_Execute();
        appSwitch_Execute();
        
        DBGPIN_3_CLEAR; // Clear DEBUG-PIN
        Nop();
    }


    return (0);
}

void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    DBGPIN_1_SET; // Set DEBUG-PIN
    
    // Execute high priority, time critical tasks
    appPowerSupply_Execute();
    appFaults_Execute();
    
    LOW_PRIO_GO = true; // Set GO trigger for low priority tasks
    
    _T1IF = 0;
    DBGPIN_1_CLEAR; // Clear DEBUG-PIN
}


