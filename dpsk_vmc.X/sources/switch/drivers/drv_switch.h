/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   led.h
 * Author: M91406
 * Comments: LCD application layer
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef SWITCH_BUTTON_DRIVER_H
#define	SWITCH_BUTTON_DRIVER_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h> // include standard integer data types
#include <stdbool.h> // include standard boolean data types
#include <stddef.h> // include standard definition data types


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    typedef enum {
        SWITCH_STAT_PRESSED    = 0b1100000000000001,
        SWITCH_STAT_LONG_PRESS = 0b1110000000000001,
        SWITCH_STAT_RELEASED   = 0b1000000000000001
    } SWITCH_STATUS_FLAGS_e;
    
typedef union {
    struct{
        volatile bool sw_event :1;  // Bit 0:  Event bit indicating a state has changed (cleared automatically)
        volatile unsigned     :1;   // Bit 1:  (reserved)
        volatile unsigned     :1;   // Bit 2:  (reserved)
        volatile unsigned     :1;   // Bit 3:  (reserved)
        volatile unsigned     :1;   // Bit 4:  (reserved)
        volatile unsigned     :1;   // Bit 5:  (reserved)
        volatile unsigned     :1;   // Bit 6:  (reserved)
        volatile unsigned     :1;   // Bit 7:  (reserved)
        volatile unsigned     :1;   // Bit 8:  (reserved)
        volatile unsigned     :1;   // Bit 9:  (reserved)
        volatile unsigned     :1;   // Bit 10: (reserved)
        volatile unsigned     :1;   // Bit 11: (reserved)
        volatile unsigned     :1;   // Bit 12: (reserved)
        volatile bool long_press :1;   // Bit 13: Indicates if switch has been pressed for a longer time
        volatile bool pressed :1;   // Bit 14: Indicates if the button is pressed or not
        volatile bool enabled :1;   // Bit 15: Enables/disables the Switch button object
    }__attribute__((packed)) bits;  // Status bit field
    volatile uint16_t value;        // Status word
} SWITCH_STATUS_t;                  // Switch status
    
// PUBLIC DATA TYPE DECLARATION
typedef struct {
    volatile SWITCH_STATUS_t status;    // Status word of the switch object
    volatile uint16_t debounce_delay;   // Number of call cycles until a switch event is triggered
    volatile uint16_t long_press_delay; // Number of call cycles until a "long press" switch event is triggered
    volatile uint16_t (*event_btn_down)(void);    // Function pointer to user function triggering a PRESSED event
    volatile uint16_t (*event_long_press)(void); // Function pointer to user function triggering a LONG_PRESS event
    volatile uint16_t (*event_pressed)(void);    // Function pointer to user function triggering a LONG_PRESS event
    volatile uint16_t (*event_release)(void);    // Function pointer to user function triggering a RELEASE event
}SWITCH_OBJECT_t;
    
// PUBLIC FUNCTION PROTOTYPE DECLARATION
extern volatile uint16_t drv_Switch_Initialize(volatile SWITCH_OBJECT_t* swbtn);
extern volatile uint16_t drv_Switch_Execute(volatile SWITCH_OBJECT_t* swbtn);
extern volatile uint16_t drv_Switch_Dispose(volatile SWITCH_OBJECT_t* swbtn);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* SWITCH_BUTTON_DRIVER_H */

