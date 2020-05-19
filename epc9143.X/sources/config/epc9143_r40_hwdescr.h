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
 * File:   epc9143_R40_hwdescr.h
 * Author: M91406
 * Comments: EPC9143 16th Brick Reference Design Hardware Descriptor header file
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef EPC9143_R40_HARDWARE_DESCRIPTOR_H
#define	EPC9143_R40_HARDWARE_DESCRIPTOR_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h> // include standard integer data types
#include <stdbool.h> // include standard boolean data types
#include <stddef.h> // include standard definition data types
#include <math.h> // include standard math functoins library

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */


/*!Microcontroller Abstraction
 * *************************************************************************************************
 * Summary:
 * Global defines for device specific parameters
 * 
 * Description:
 * This section is used to define device specific parameters like ADC reference and
 * resolution. Pre-compiler macros are used to translate physical values into binary 
 * (integer) numbers to be written to SFRs
 * 
 * *************************************************************************************************/
#define CPU_FREQUENCY        100000000  // CPU frequency in [Hz]
#define CPU_TCY              (float)(1.0/CPU_FREQUENCY) // Instruction period

// ADC(DAC Reference and Resolution Settings    
#define ADC_REF              (float)3.300 // ADC reference voltage in V
#define ADC_RES              (float)12.0  // ADC resolution in [bit]
#define ADC_GRAN             (float)(ADC_REF / pow(2.0, ADC_RES)) // ADC granularity in [V/tick]
    
// PWM/ADC Clock Settings    
#define PWM_CLOCK_FREQUENCY     (float)4.0e+9   // PWM Clock Frequency in [Hz]
#define PWM_CLOCK_PERIOD        (float)(1.0/PWM_CLOCK_FREQUENCY) // PWM Clock Period in [sec]
    
/*!State Machine Settings
 * *************************************************************************************************
 * Summary:
 * Global defines for state-machine specific parameters
 * 
 * Description:
 * This section is used to define state-machine settings such as the main execution call interval. 
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers to be 
 * written to SFRs and variables.
 * 
 * *************************************************************************************************/
#define MAIN_EXECUTION_PERIOD   (float)100e-6     // main state machine pace period in [sec]
#define MAIN_EXEC_PER           (uint16_t)((CPU_FREQUENCY * MAIN_EXECUTION_PERIOD)-1)

    
/*!Hardware Abstraction
 * *************************************************************************************************
 * Summary:
 * Global defines for hardware specific parameters
 * 
 * Description:
 * This section is used to define hardware specific parameters such as output voltage dividers,
 * reference levels or feedback gains. Pre-compiler macros are used to translate physical  
 * values into binary (integer) numbers to be written to SFRs
 * 
 * *************************************************************************************************/

#ifdef __EPC9143_R40__
    
    // Device Pin #1 on EPC9143
    #define DBGPIN_1_SET	{ _LATB14 = 1; }
    #define DBGPIN_1_CLEAR	{ _LATB14 = 0; }
    #define DBGPIN_1_TOGGLE	{ _LATB14 ^= 1; }
    #define DBGPIN_1_INIT	{ _LATB14 = 0; _TRISB14 = 0; }

    // Device Pin #2 on EPC9143
    #define DBGPIN_2_SET	{ _LATB15 = 1; }
    #define DBGPIN_2_CLEAR	{ _LATB15 = 0; }
    #define DBGPIN_2_TOGGLE	{ _LATB15 ^= 1; }
    #define DBGPIN_2_INIT	{ _LATB15 = 0; _TRISB15 = 0; }

    // Device Pin #25 on EPC9143
    #define DBGPIN_3_SET	{ _LATB10 = 1; }
    #define DBGPIN_3_CLEAR	{ _LATB10 = 0; }
    #define DBGPIN_3_TOGGLE	{ _LATB10 ^= 1; }
    #define DBGPIN_3_INIT	{ _LATB10 = 0; _TRISB10 = 0; }
#endif

/*!Power Control Parameter Declaration
 * *************************************************************************************************
 * Summary:
 * Global defines for Buck Converter Power Control parameters
 * 
 * Description:
 * This section is used to define hardware specific parameters such as output voltage dividers,
 * reference levels or feedback gains. Pre-compiler macros are used to translate physical  
 * values into binary (integer) numbers to be written to SFRs
 * 
 * *************************************************************************************************/

#define SWITCHING_FREQUENCY     (float)500e+3   // Switching frequency in [Hz]
#define SWITCHING_PERIOD        (float)(1.0/SWITCHING_FREQUENCY)    // Switching period in [sec]

#define SWITCHING_PHASE_SHIFT   (float)0        // Phase Shift of PWM output in [sec]
#define SWITCHING_LEB           (float)120.0e-9 // Leading Edge Blanking in [sec]
#define SWITCHING_DEAD_TIME_LE  (float)20e-9    // Leading Edge Dead Time in [sec]
#define SWITCHING_DEAD_TIME_FE  (float)30e-9    // Falling Edge Dead Time in [sec]
    


/*!Fundamental PWM Settings
 * *************************************************************************************************
 * Summary:
 * Global defines for PWM settings of DV330101
 * 
 * Description:
 * This section defines fundamental PWM settings required for the low-voltage interleaved boost
 * PFC development board. These settings are determined by hardware. User-selectable settings 
 * are made on application layer level (e.g. in apl.h), outside this header file.
 * 
 * Pre-compiler macros are used to convert physical values into binary (integer) numbers to 
 * be written to SFRs
 * 
 * *************************************************************************************************/

#define BUCK_NO_OF_PHASES              1U     // Number of power converter phases of this design
    
#define BUCK_PWM_DUTY_CYCLE_MIN        (float)0.010 // ~1.0% On Time 
#define BUCK_PWM_DUTY_CYCLE_MAX        (float)0.400 // ~90% On Time 
#define BUCK_LEADING_EDGE_BLANKING     (float)120.0e-9 // Leading Edge Blanking in [sec]
#define BUCK_DEAD_TIME_LEADING_EDGE    (float)100.0e-9 // Leading Edge Dead Time in [sec]
#define BUCK_DEAD_TIME_FALLING_EDGE    (float)100.0e-9 // Falling Edge Dead Time in [sec]

// PWM Phase #1 Configuration
#define BUCK_PWM1_CHANNEL              2U // PWM Instance Index (e.g. 1=PWM1, 2=PWM2, etc.)
#define BUCK_PWM1_GPIO_INSTANCE        2U // Number indicating device port, where 1=A, 2=B, 3=C, etc.
#define BUCK_PWM1_GPIO_PORT_PINH       12U // Port Pin Number
#define BUCK_PWM1_GPIO_PORT_PINL       13U // Port Pin Number
    
#define BUCK_PWM1_PDC                  PG2DC    // PWM Instance Duty Cycle Register
#define BUCK_PWM1H_TRIS                _TRISB12 // Device Port TRIS register
#define BUCK_PWM1H_WR                  _LATB12  // Device Pin WRITE
#define BUCK_PWM1H_RD                  _RB12    // Device Pin READ
#define BUCK_PWM1L_TRIS                _TRISB13 // Device Port TRIS register
#define BUCK_PWM1L_WR                  _LATB13  // Device Pin WRITE
#define BUCK_PWM1L_RD                  _RB13    // Device Pin READ

#define _BUCK_PWM1_Interrupt           _PWM2Interrupt // PWM Interrupt Serivice Routine label
#define BUCK_PWM1_IF                   _PWM2IF  // PWM Interrupt Flag Bit
#define BUCK_PWM1_IE                   _PWM2IE  // PWM Interrupt Enable Bit
#define BUCK_PWM1_IP                   _PWM2IP  // PWM Interrupt Priority
#define BUCK_PWM1_TRGSRC_TRG1          0b00110  // PWM Trigger #1 Trigger Source of this channel
#define BUCK_PWM1_TRGSRC_TRG2          0b00111  // PWM Trigger #2 Trigger Source of this channel
#define BUCK_PWM1_PGxTRIGA             PG2TRIGA // PWM trigger register A
#define BUCK_PWM1_PGxTRIGB             PG2TRIGB // PWM trigger register B
#define BUCK_PWM1_PGxTRIGC             PG2TRIGC // PWM trigger register C
    
#define BUCK_PWM1_ADTR1OFS             0U // ADC Trigger 1 Offset:  0...31
#define BUCK_PWM1_ADTR1PS              0U // ADC Trigger 1 Postscaler: 0...31

// PWM Phase #2 Configuration
#define BUCK_PWM2_CHANNEL              4U    // PWM Instance Index (e.g. 1=PWM1, 2=PWM2, etc.)
#define BUCK_PWM2_GPIO_INSTANCE        2U // Number indicating device port, where 1=A, 2=B, 3=C, etc.
#define BUCK_PWM2_GPIO_PORT_PINH       9U // Port Pin Number
#define BUCK_PWM2_GPIO_PORT_PINL       8U // Port Pin Number

#define BUCK_PWM2H_PDC                 PG4DC    // PWM Instance Duty Cycle Register
#define BUCK_PWM2H_TRIS                _TRISB9  // Device Port TRIS register
#define BUCK_PWM2H_WR                  _LATB9   // Device Pin WRITE
#define BUCK_PWM2H_RD                  _RB9     // Device Pin READ
#define BUCK_PWM2L_TRIS                _TRISB8  // Device Port TRIS register
#define BUCK_PWM2L_WR                  _LATB8   // Device Pin WRITE
#define BUCK_PWM2L_RD                  _RB8     // Device Pin READ
    
#define _BUCK_PWM2_Interrupt           _PWM4Interrupt // PWM Interrupt Serivice Routine label
#define BUCK_PWM2_IF                   _PWM4IF        // PWM Interrupt Flag Bit
#define BUCK_PWM2_IE                   _PWM4IE        // PWM Interrupt Enable Bit
#define BUCK_PWM2_IP                   _PWM4IP        // PWM Interrupt Priority
#define BUCK_PWM2_TRGSRC_TRG1          0b01010  // PWM2 Trigger #1 Trigger Source of this channel
#define BUCK_PWM2_TRGSRC_TRG2          0b01011  // PWM2 Trigger #2 Trigger Source of this channel
#define BUCK_PWM2_PGxTRIGA             PG4TRIGA // PWM2 trigger register A
#define BUCK_PWM2_PGxTRIGB             PG4TRIGB // PWM2 trigger register B
#define BUCK_PWM2_PGxTRIGC             PG4TRIGC // PWM2 trigger register C

#define BUCK_PWM2_ADTR1OFS             0 // ADC Trigger 1 Offset:  0...31
#define BUCK_PWM2_ADTR1PS              0 // ADC Trigger 1 Postscaler: 0...31
    

// ~~~ conversion macros ~~~~~~~~~~~~~~~~~~~~~~~~~
#define BUCK_SWITCHING_PERIOD      (float)(1.0/SWITCHING_FREQUENCY)   // Switching period in [sec]
#define BUCK_PWM_PERIOD            (uint16_t)(float)(BUCK_SWITCHING_PERIOD / PWM_CLOCK_PERIOD)
#define BUCK_PWM_PHASE_SHIFT       (uint16_t)((float)BUCK_SWITCHING_PERIOD / (float)BUCK_NO_OF_PHASES)      
#define BUCK_PWM_DC_MIN            (uint16_t)(BUCK_PWM_DUTY_CYCLE_MIN * (float)BUCK_PWM_PERIOD) // This sets the minimum duty cycle
#define BUCK_PWM_DC_MAX            (uint16_t)(BUCK_PWM_DUTY_CYCLE_MAX * (float)BUCK_PWM_PERIOD) // This sets the maximum duty cycle
#define BUCK_LEB_PERIOD            (uint16_t)(BUCK_LEADING_EDGE_BLANKING / (float)PWM_CLOCK_PERIOD) // Leading Edge Blanking = n x PWM resolution (here: 50 x 2ns = 100ns)
#define BUCK_PWM_DEAD_TIME_LE      (uint16_t)(BUCK_DEAD_TIME_LEADING_EDGE / (float)PWM_CLOCK_PERIOD) // Rising edge dead time [tick = 250ps]
#define BUCK_PWM_DEAD_TIME_FE      (uint16_t)(BUCK_DEAD_TIME_FALLING_EDGE / (float)PWM_CLOCK_PERIOD) // Falling edge dead time [tick = 250ps]
// ~~~ conversion macros end ~~~~~~~~~~~~~~~~~~~~~


    
/*!Input Voltage Feedback
 * *************************************************************************************************
 * Summary:
 * 
 * Description:
 * 
 * *************************************************************************************************/
    
#define BUCK_VIN_MINIMUM        (float)6.000    // Minimum input voltage
#define BUCK_VIN_NOMINAL        (float)9.000    // Nominal input voltage
#define BUCK_VIN_MAXIMUM        (float)13.80    // Maximum input voltage

#define BUCK_VIN_UNDER_VOLTAGE  (float)6.000    // Under Voltage Lock Out Cut Off
#define BUCK_VIN_OVER_VOLTAGE   (float)14.00    // Over Voltage Lock Out Cut Off
#define BUCK_VIN_HYSTERESIS     (float)1.000    // UVLO/OVLO Hysteresis
    
#define BUCK_VIN_R1             (float)(110.0)  // Upper voltage divider resistor in kOhm
#define BUCK_VIN_R2             (float)(4.870)  // Lower voltage divider resistor in kOhm
#define BUCK_VIN_FB_GAIN        (float)((BUCK_VIN_R2) / (BUCK_VIN_R1 + BUCK_VIN_R2))
    
#define BUCK_VIN_FEEDBACK_OFFSET    (float)(0.0)
#define BUCK_VIN_ADC_TRG_DELAY      (float)(120.0e-9) // ADC trigger delay in [sec]
    
// ~ conversion macros ~~~~~~~~~~~~~~~~~~~~~
    
#define BUCK_VIN_MIN            (uint16_t)(BUCK_VIN_MINIMUM * BUCK_VIN_FB_GAIN / ADC_GRAN)   // Minimum input voltage
#define BUCK_VIN_NOM            (uint16_t)(BUCK_VIN_NOMINAL * BUCK_VIN_FB_GAIN / ADC_GRAN)   // Nominal input voltage
#define BUCK_VIN_MAX            (uint16_t)(BUCK_VIN_MAXIMUM * BUCK_VIN_FB_GAIN / ADC_GRAN)   // Maximum input voltage
#define BUCK_VIN_HYST           (uint16_t)(BUCK_VIN_HYSTERESIS * BUCK_VIN_FB_GAIN / ADC_GRAN)  // Over Voltage LOck Out voltage    
#define BUCK_VIN_UVLO_TRIP      (uint16_t)(BUCK_VIN_UNDER_VOLTAGE * BUCK_VIN_FB_GAIN / ADC_GRAN) // Under Voltage LOck Out voltage
#define BUCK_VIN_UVLO_RELEASE   (uint16_t)((BUCK_VIN_UNDER_VOLTAGE + BUCK_VIN_HYSTERESIS) * BUCK_VIN_FB_GAIN / ADC_GRAN) // Under Voltage LOck Out voltage
#define BUCK_VIN_OVLO_TRIP      (uint16_t)(BUCK_VIN_OVER_VOLTAGE * BUCK_VIN_FB_GAIN / ADC_GRAN)  // Over Voltage LOck Out voltage
#define BUCK_VIN_OVLO_RELEASE   (uint16_t)((BUCK_VIN_OVER_VOLTAGE - BUCK_VIN_HYSTERESIS) * BUCK_VIN_FB_GAIN / ADC_GRAN)  // Over Voltage LOck Out voltage
#define BUCK_VIN_ADC_TRGDLY     (uint16_t)(BUCK_VIN_ADC_TRG_DELAY / PWM_CLOCK_PERIOD) // Input voltage ADC trigger delay
#define BUCK_VIN_FB_OFFSET      (uint16_t)(BUCK_VIN_OVER_VOLTAGE / ADC_GRAN) // Input voltage feedback offset

#define BUCK_VIN_NORM_SCALER (int16_t)(floor(log(BUCK_VIN_FB_GAIN)) + 1) // VIN normalization  
#define BUCK_VIN_NORM_FACTOR (int16_t)((BUCK_VIN_FB_GAIN / pow(2.0, BUCK_VIN_NORM_SCALER)) * pow(2.0, 15)) // VIN normalization factor scaled in Q15

// ~ conversion macros end ~~~~~~~~~~~~~~~~~
    
#define _BUCK_VIN_ADCInterrupt      _ADCAN9Interrupt   
#define _BUCK_VIN_ADCISR_IF         _ADCAN9IF

#define BUCK_VIN_ANSEL              _ANSELA2    // GPIO analog function mode enable bit
#define BUCK_VIN_ADCCORE            8           // 0=Dedicated Core #0, 1=Dedicated Core #1, 8=Shared ADC Core
#define BUCK_VIN_ADCIN              9           // Analog input number (e.g. '5' for 'AN5')
#define BUCK_VIN_ADCBUF             ADCBUF9     // ADC input buffer of this ADC channel
#define BUCK_VIN_ADCTRIG            PG2TRIGB    // Register used for trigger placement
#define BUCK_VIN_TRGSRC             0b00111     // THis aDC channel trigger source: PWM2 Trigger 2

/*!Output Voltage Feedback
 * *************************************************************************************************
 * Summary:
 * 
 * Description:
 * 
 * *************************************************************************************************/

// Feedback Declarations
#define BUCK_VOUT_NOMINAL           (float)3.000   // Nominal output voltage
#define BUCK_VOUT_TOLERANCE_MAX     (float)0.500   // Output voltage tolerance [+/-]
#define BUCK_VOUT_TOLERANCE_MIN     (float)0.100   // Output voltage tolerance [+/-]
    
#define BUCK_VOUT_R1                (float)(18.00) // Upper voltage divider resistor in kOhm
#define BUCK_VOUT_R2                (float)(4.750) // Lower voltage divider resistor in kOhm
#define BUCK_VOUT_FB_GAIN           (float)((BUCK_VOUT_R2) / (BUCK_VOUT_R1 + BUCK_VOUT_R2))
#define BUCK_VOUT_FB_OFFSET         (float)(0.0)
#define BUCK_VOUT_ADC_TRG_DELAY     (float)(120.0e-9) // Trigger delay in [sec]

// Peripheral Assignments
#define BUCK_VOUT_ANSEL             _ANSELA0    // GPIO analog function mode enable bit
#define BUCK_VOUT_ADCCORE           0           // 0=Dedicated Core #0, 1=Dedicated Core #1, 8=Shared ADC Core
#define BUCK_VOUT_ADCIN             0           // Analog input number (e.g. '5' for 'AN5')
#define BUCK_VOUT_ADCBUF            ADCBUF0     // ADC input buffer of this ADC channel
#define BUCK_VOUT_ADCTRIG           PG2TRIGA    // Register used for trigger placement
#define BUCK_VOUT_TRGSRC            0b00110     // PWM2 Trigger 1
    
// ~ conversion macros ~~~~~~~~~~~~~~~~~~~~~

#define BUCK_VOUT_REF           (uint16_t)(BUCK_VOUT_NOMINAL * BUCK_VOUT_FB_GAIN / ADC_GRAN)
#define BUCK_VOUT_DEV_TRIP      (uint16_t)(BUCK_VOUT_TOLERANCE_MAX * BUCK_VOUT_FB_GAIN / ADC_GRAN)
#define BUCK_VOUT_DEV_RELEASE   (uint16_t)(BUCK_VOUT_TOLERANCE_MIN * BUCK_VOUT_FB_GAIN / ADC_GRAN)
#define BUCK_VOUT_OFFSET        (uint16_t)(BUCK_VOUT_FB_OFFSET / ADC_GRAN)
#define BUCK_VOUT_ADC_TRGDLY    (uint16_t)(BUCK_VOUT_ADC_TRG_DELAY / PWM_CLOCK_PERIOD)

#define BUCK_VOUT_NORM_SCALER   (int16_t)(floor(log(BUCK_VOUT_FB_GAIN)) + 1) // VOUT normalization  
#define BUCK_VOUT_NORM_FACTOR   (int16_t)((BUCK_VOUT_FB_GAIN / pow(2.0, BUCK_VOUT_NORM_SCALER)) * pow(2.0, 15)) // VOUT normalization factor scaled in Q15

// ~ conversion macros end ~~~~~~~~~~~~~~~~~


/*!Phase Current Feedback
 * *************************************************************************************************
 * Summary:
 * 
 * Description:
 * 
 * *************************************************************************************************/

// Feedback Declarations
#define BUCK_ISNS_FB_GAIN           (float) 0.050       // Current Gain in V/A
#define BUCK_ISNS_MAXIMUM           (float) 2.500       // absolute maximum output current (average)
#define BUCK_ISNS_REFERENCE         (float) 1.000       // output current reference (average)
#define BUCK_ISNS_ADC_TRG_DELAY     (float) 120.0e-9    // ADC trigger delay for current sense in [sec]

#define BUCK_ISNS1_FB_OFFSET        (float) 1.650       // current sense #1 feedback offset (average)
#define BUCK_ISNS2_FB_OFFSET        (float) 1.650       // current sense #2 feedback offset (average)
    
// Peripheral Assignments
#define _BUCK_ISNS1_ADCInterrupt    _ADCAN1Interrupt   
#define _BUCK_ISNS1_ADCISR_IF       _ADCAN1IF

#define BUCK_ISNS1_ANSEL            _ANSELA1    // GPIO analog function mode enable bit
#define BUCK_ISNS1_ADCCORE          0           // 0=Dedicated Core #0, 1=Dedicated Core #1, 2=Shared ADC Core
#define BUCK_ISNS1_ADCIN            1           // Analog input number (e.g. '5' for 'AN5')
#define BUCK_ISNS1_ADCBUF           ADCBUF1     // ADC input buffer of this ADC channel
#define BUCK_ISNS1_ADCTRIG          PG2TRIGA    // Register used for trigger placement
#define BUCK_ISNS1_TRGSRC           0b00110     // PWM2 Trigger 1

#define _BUCK_ISNS2_ADCInterrupt    _ADCAN4Interrupt   
#define _BUCK_ISNS2_ADCISR_IF       _ADCAN4IF

#define BUCK_ISNS2_ANSEL            _ANSELA4    // GPIO analog function mode enable bit
#define BUCK_ISNS2_ADCCORE          0           // 0=Dedicated Core #0, 1=Dedicated Core #1, 2=Shared ADC Core
#define BUCK_ISNS2_ADCIN            0           // Analog input number (e.g. '5' for 'AN5')
#define BUCK_ISNS2_ADCBUF           ADCBUF4     // ADC input buffer of this ADC channel
#define BUCK_ISNS2_ADCTRIG          PG4TRIGA    // Register used for trigger placement
#define BUCK_ISNS2_TRGSRC           0b01010     // PWM4 Trigger 1

// ~ conversion macros ~~~~~~~~~~~~~~~~~~~~~

#define BUCK_ISNS_OCL           (uint16_t)(BUCK_ISNS_MAXIMUM * BUCK_ISNS_FB_GAIN / ADC_GRAN)  // Over Current Limit
#define BUCK_ISNS_REF           (uint16_t)(BUCK_ISNS_REFERENCE * BUCK_ISNS_FB_GAIN / ADC_GRAN)  // Output Current Reference
#define BUCK_ISNS1_OFFFSET      (uint16_t)(BUCK_ISNS1_FB_OFFSET / ADC_GRAN)
#define BUCK_ISNS2_OFFFSET      (uint16_t)(BUCK_ISNS2_FB_OFFSET / ADC_GRAN)
#define BUCK_ISNS_ADC_TRGDLY    (uint16_t)(BUCK_ISNS_ADC_TRG_DELAY / PWM_CLOCK_PERIOD)

// ~ conversion macros end ~~~~~~~~~~~~~~~~~

    
/*!Adaptive Gain Control Feed Forward Control
 * *************************************************************************************************
 * Summary:
 * 
 * Description:
 * 
 * *************************************************************************************************/
    
#define BUCK_VL_MINIMUM         (float)(BUCK_VIN_MINIMUM - BUCK_VOUT_NOMINAL) // Minimum input voltage - output voltate
#define BUCK_VL_NOMINAL         (float)(BUCK_VIN_NOMINAL - BUCK_VOUT_NOMINAL) // Nominal input voltage - output voltate
#define BUCK_VL_MAXIMUM         (float)(BUCK_VIN_MAXIMUM - BUCK_VOUT_NOMINAL) // Maximum input voltage - output voltate

#define BUCK_VIN_NORM_FCT       (float)(BUCK_VOUT_FB_GAIN / BUCK_VIN_FB_GAIN)   // VIN-2-VOUT Normalization Factor

#define BUCK_AGC_NORM_SCALER    (int16_t)(floor(log(BUCK_VIN_NORM_FCT)) + 1) // Nominal VL Q15 scaler  
#define BUCK_AGC_NORM_FACTOR    (int16_t)((BUCK_VIN_NORM_FCT / pow(2.0, BUCK_AGC_NORM_SCALER)) * pow(2.0, 15)) // Nominal VL Q15 factor 

    
/*!Startup Behavior
 * *************************************************************************************************
 * Summary:
 * Global defines for soft-start specific parameters
 * 
 * Description:
 * This section is used to define power supply startup timing setting. The soft-start sequence 
 * is part of the power controller. It allows to program specific timings for Power On Delay,
 * Ramp Period and Power Good Delay. After the startup has passed these three timing periods,
 * the power supply is ending up in "normal" operation, continuously regulating the output until 
 * a fault is detected or the operating state is changed for any other reason.
 * 
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers to 
 * be written to SFRs and variables.
 * 
 * *************************************************************************************************/

#define BUCK_POWER_ON_DELAY          (float) 500e-3 // power on delay in [sec]
#define BUCK_VRAMP_PERIOD            (float) 200e-3 // ramp period in [sec]
#define BUCK_IRAMP_PERIOD            (float)  50e-3 // ramp period in [sec]
#define BUCK_POWER_GOOD_DELAY        (float) 100e-3 // power good in [sec]

// ~ conversion macros ~~~~~~~~~~~~~~~~~~~~~

#define BUCK_POD       (uint16_t)(((float)BUCK_POWER_ON_DELAY / (float)MAIN_EXECUTION_PERIOD)-1)
#define BUCK_VRAMP_PER (uint16_t)(((float)BUCK_VRAMP_PERIOD / (float)MAIN_EXECUTION_PERIOD)-1)
#define BUCK_VREF_STEP (uint16_t)((float)BUCK_VOUT_REF / (float)(BUCK_VRAMP_PER + 1))
#define BUCK_IRAMP_PER (uint16_t)(((float)BUCK_IRAMP_PERIOD / (float)MAIN_EXECUTION_PERIOD)-1)
#define BUCK_IREF_STEP (uint16_t)((float)BUCK_ISNS_REF / (float)(BUCK_VRAMP_PER + 1))
#define BUCK_PGD       (uint16_t)(((float)BUCK_POWER_GOOD_DELAY / (float)MAIN_EXECUTION_PERIOD)-1)

// ~ conversion macros end ~~~~~~~~~~~~~~~~~

    
/*!Controller Declarations
 * *************************************************************************************************
 * Summary:
 * 
 * 
 * Description:
 * 
 * 
 * *************************************************************************************************/

    
// Hardware-dependent defines
#define BUCK_VOUT_TRIG_PWM  0   // Buck VOUT control loop is called in PWM interrupt
#define BUCK_VOUT_TRIG_ADC  1   // Buck VOUT control loop is called in ADC interrupt

#define BUCK_VOUT_TRIGGER_MODE  BUCK_VOUT_TRIG_PWM
    
#if (BUCK_VOUT_TRIGGER_MODE == BUCK_VOUT_TRIG_ADC)    
  #define _BUCK_VLOOP_Interrupt     _ADCAN0Interrupt   
  #define _BUCK_VLOOP_ISR_IP        _ADCAN0IP
  #define _BUCK_VLOOP_ISR_IF        _ADCAN0IF
  #define _BUCK_VLOOP_ISR_IE        _ADCAN0IE
#elif (BUCK_VOUT_TRIGGER_MODE == BUCK_VOUT_TRIG_PWM)
  #define _BUCK_VLOOP_Interrupt     _PWM2Interrupt   
  #define _BUCK_VLOOP_ISR_IP        _PWM2IP
  #define _BUCK_VLOOP_ISR_IF        _PWM2IF
  #define _BUCK_VLOOP_ISR_IE        _PWM2IE
#endif



#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* EPC9143_R40_HARDWARE_DESCRIPTOR_H */
