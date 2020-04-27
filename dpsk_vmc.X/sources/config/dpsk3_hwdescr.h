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
 * File:   dpsk3_hwdescr.h
 * Author: M91406
 * Comments: DPSK3 Hardware Descriptor header file
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef DPSK3_HARDWARE_DESCRIPTOR_H
#define	DPSK3_HARDWARE_DESCRIPTOR_H

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
#define ADC_REF              (float)3.300 // ADC reference voltage in V
#define ADC_RES              (float)12.0  // ADC resolution in [bit]
#define ADC_GRAN             (float)(ADC_REF / pow(2.0, ADC_RES)) // ADC granularity in [V/tick]
    
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

#ifdef _DPSK3_BUCK_
    // TP51 on DPSK3 (Red LED)
    #define DBGLED_SET		{ _LATB6 = 1; }
    #define DBGLED_CLEAR	{ _LATB6 = 0; }
    #define DBGLED_TOGGLE	{ _LATB6 ^= 1; }
    #define DBGLED_INIT		{ _LATB6 = 0; _TRISB6 = 0; }
    #define DBGLED_DISPOSE  { _LATB6 = 1; _TRISB6 = 1; }
    
    // TP50 on DPSK3
    #define DBGPIN_1_SET	{ _LATB5 = 1; }
    #define DBGPIN_1_CLEAR	{ _LATB5 = 0; }
    #define DBGPIN_1_TOGGLE	{ _LATB5 ^= 1; }
    #define DBGPIN_1_INIT	{ _LATB5 = 0; _TRISB5 = 0; }

    // TP52 on DPSK3
    #define DBGPIN_2_SET	{ _LATB11 = 1; }
    #define DBGPIN_2_CLEAR	{ _LATB11 = 0; }
    #define DBGPIN_2_TOGGLE	{ _LATB11 ^= 1; }
    #define DBGPIN_2_INIT	{ _LATB11 = 0; _TRISB11 = 0; }

    // TP53 on DPSK3
    #define DBGPIN_3_SET	{ _LATB12 = 1; }
    #define DBGPIN_3_CLEAR	{ _LATB12 = 0; }
    #define DBGPIN_3_TOGGLE	{ _LATB12 ^= 1; }
    #define DBGPIN_3_INIT	{ _LATB12 = 0; _TRISB12 = 0; }
#endif

#define SW_USER_TRISx   _TRISD1
#define SW_USER_LATx    _LATD1
#define SW_USER_PORTx   _RD1
#define SW_USER_INIT    { SW_USER_LATx = 1; SW_USER_TRISx = 1; }
    
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
#define SWITCHING_DEAD_TIME_FE  (float)60e-9    // Falling Edge Dead Time in [sec]
    
//#define __VMC   0   // Voltage Mode Control
//#define __PCMC  1   // Peak Current Mode Control
//#define __ACMC  2   // Average Current Mode Control
//    
//#define BUCK_CONTROL_MODE   __VMC   // Control Mode
    
#define VIN_MINIMUM         (float)6.000    // Minimum input voltage
#define VIN_NOMINAL         (float)9.000    // Nominal input voltage
#define VIN_MAXIMUM         (float)13.80    // Maximum input voltage

#define VIN_UNDER_VOLTAGE   (float)6.000    // Under Voltage Lock Out Cut Off
#define VIN_OVER_VOLTAGE    (float)14.00    // Over Voltage Lock Out Cut Off
#define VIN_HYSTERESIS      (float)1.000    // UVLO/OVLO Hysteresis
    
#define BUCK_VIN_R1         (float)(6.98)   // Upper voltage divider resistor in kOhm
#define BUCK_VIN_R2         (float)(1.00)   // Lower voltage divider resistor in kOhm
#define BUCK_VIN_FB_GAIN    (float)((BUCK_VIN_R2) / (BUCK_VIN_R1 + BUCK_VIN_R2))
    
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define VIN_MIN             (uint16_t)(VIN_MINIMUM * BUCK_VIN_FB_GAIN / ADC_GRAN)   // Minimum input voltage
#define VIN_NOM             (uint16_t)(VIN_NOMINAL * BUCK_VIN_FB_GAIN / ADC_GRAN)   // Nominal input voltage
#define VIN_MAX             (uint16_t)(VIN_MAXIMUM * BUCK_VIN_FB_GAIN / ADC_GRAN)   // Maximum input voltage
#define VIN_HYST            (uint16_t)(VIN_HYSTERESIS * BUCK_VIN_FB_GAIN / ADC_GRAN)  // Over Voltage LOck Out voltage    
#define VIN_UVLO_TRIP       (uint16_t)(VIN_UNDER_VOLTAGE * BUCK_VIN_FB_GAIN / ADC_GRAN) // Under Voltage LOck Out voltage
#define VIN_UVLO_RELEASE    (uint16_t)((VIN_UNDER_VOLTAGE + VIN_HYSTERESIS) * BUCK_VIN_FB_GAIN / ADC_GRAN) // Under Voltage LOck Out voltage
#define VIN_OVLO_TRIP       (uint16_t)(VIN_OVER_VOLTAGE * BUCK_VIN_FB_GAIN / ADC_GRAN)  // Over Voltage LOck Out voltage
#define VIN_OVLO_RELEASE    (uint16_t)((VIN_OVER_VOLTAGE - VIN_HYSTERESIS) * BUCK_VIN_FB_GAIN / ADC_GRAN)  // Over Voltage LOck Out voltage
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
#define BUCK_VOUT_NOMINAL       (float)3.300   // Nominal output voltage
#define BUCK_VOUT_TOLERANCE_MAX (float)0.500   // Output voltage tolerance [+/-]
#define BUCK_VOUT_TOLERANCE_MIN (float)0.100   // Output voltage tolerance [+/-]
    
#define BUCK_VOUT_R1            (float)(1.0)    // Upper voltage divider resistor in kOhm
#define BUCK_VOUT_R2            (float)(1.0)    // Lower voltage divider resistor in kOhm
#define BUCK_VOUT_FB_GAIN       (float)((BUCK_VOUT_R2) / (BUCK_VOUT_R1 + BUCK_VOUT_R2))

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define BUCK_VOUT_REF           (uint16_t)(BUCK_VOUT_NOMINAL * BUCK_VOUT_FB_GAIN / ADC_GRAN)
#define BUCK_VOUT_DEV_TRIP      (uint16_t)(BUCK_VOUT_TOLERANCE_MAX * BUCK_VOUT_FB_GAIN / ADC_GRAN)
#define BUCK_VOUT_DEV_RELEASE   (uint16_t)(BUCK_VOUT_TOLERANCE_MIN * BUCK_VOUT_FB_GAIN / ADC_GRAN)
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define BUCK_VL_MINIMUM         (float)(VIN_MINIMUM - BUCK_VOUT_NOMINAL) // Minimum input voltage - output voltate
#define BUCK_VL_NOMINAL         (float)(VIN_NOMINAL - BUCK_VOUT_NOMINAL) // Nominal input voltage - output voltate
#define BUCK_VL_MAXIMUM         (float)(VIN_MAXIMUM - BUCK_VOUT_NOMINAL) // Maximum input voltage - output voltate


#define BUCK_VIN_NORM_FCT       (float)(BUCK_VOUT_FB_GAIN / BUCK_VIN_FB_GAIN)   // VIN-2-VOUT Normalization Factor
#define BUCK_AGC_VL_NOMINAL     (float)(VIN_NOMINAL - BUCK_VOUT_NOMINAL)
    
#define BUCK_AGC_VL_NOM         (uint16_t)(BUCK_AGC_VL_NOMINAL / ADC_GRAN) // Common Volt-Seconds ratio at VIN=9V/VOUT=3.§V

#define BUCK_VIN_NORM_SCALER    3 // VIN normalization  
#define BUCK_VIN_NORM_FACTOR    0x7FFF // VIN normalization factor (Q15)
#define BUCK_VOUT_NORM_SCALER   1 // VOUT normalization  
#define BUCK_VOUT_NORM_FACTOR   0x7FFF // VOUT normalization factor (Q15)

#define BUCK_IOUT_FB_GAIN       (float) 0.2     // Current Gain in V/A
#define BUCK_IOUT_MAXIMUM       (float) 2.500   // absolute maximum output current (average)
#define BUCK_IOUT_REFERENCE     (float) 2.500   // output current reference (average)

#define BUCK_IOUT_OCL           (uint16_t)(BUCK_IOUT_MAXIMUM * BUCK_IOUT_FB_GAIN / ADC_GRAN)  // Over Current Limit
#define BUCK_IOUT_REF           (uint16_t)(BUCK_IOUT_REFERENCE * BUCK_IOUT_FB_GAIN / ADC_GRAN)  // Output Current Reference



#define BUCK_TEMPCAL_ZERO       (float) 0.500   // Temperature sense signal zero point voltage in [V]
#define BUCK_TEMPCAL_SLOPE      (float) 0.010   // Temperature sense signal slope in [V/K]
    
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define BUCK_FB_TEMP_ZERO       (uint16_t)(BUCK_TEMPCAL_ZERO / ADC_GRAN)
#define BUCK_FB_TEMP_SLOPE      (float)(BUCK_TEMPCAL_SLOPE / ADC_GRAN)
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
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

#define BUCK_POWER_ON_DELAY          500e-3     // power on delay in [sec]
#define BUCK_VRAMP_PERIOD             50e-3     // ramp period in [sec]
#define BUCK_IRAMP_PERIOD             50e-3     // ramp period in [sec]
#define BUCK_POWER_GOOD_DELAY        100e-3     // power good in [sec]

#define BUCK_POD       (uint16_t)(((float)BUCK_POWER_ON_DELAY / (float)MAIN_EXECUTION_PERIOD)-1)
#define BUCK_VRAMP_PER (uint16_t)(((float)BUCK_VRAMP_PERIOD / (float)MAIN_EXECUTION_PERIOD)-1)
#define BUCK_VREF_STEP (uint16_t)((float)BUCK_VOUT_REF / (float)(BUCK_VRAMP_PER + 1))
#define BUCK_IRAMP_PER (uint16_t)(((float)BUCK_IRAMP_PERIOD / (float)MAIN_EXECUTION_PERIOD)-1)
#define BUCK_IREF_STEP (uint16_t)((float)BUCK_IOUT_REF / (float)(BUCK_VRAMP_PER + 1))
#define BUCK_PGD       (uint16_t)(((float)BUCK_POWER_GOOD_DELAY / (float)MAIN_EXECUTION_PERIOD)-1)


    
/*!Peripheral Declarations
 * *************************************************************************************************
 * Summary:
 * 
 * 
 * Description:
 * 
 * 
 * *************************************************************************************************/

// PWM/ADC Clock Settings    
#define PWM_CLOCK_FREQUENCY     (float)4.0e+9   // PWM Clock Frequency in [Hz]
#define PWM_CLOCK_PERIOD        (float)(1.0/PWM_CLOCK_FREQUENCY) // PWM Clock Period in [sec]
    
// Hardware-dependent defines
#define BUCK_VOUT_TRIG_PWM  0   // Buck VOUT control loop is called in PWM interrupt
#define BUCK_VOUT_TRIG_ADC  1   // Buck VOUT control loop is called in ADC interrupt

#define BUCK_VOUT_TRIGGER_MODE  BUCK_VOUT_TRIG_PWM
    
#if (BUCK_VOUT_TRIGGER_MODE == BUCK_VOUT_TRIG_ADC)    
  #define _BUCK_VLOOP_Interrupt     _ADCAN13Interrupt   
  #define _BUCK_VLOOP_ISR_IP        _ADCAN13IP
  #define _BUCK_VLOOP_ISR_IF        _ADCAN13IF
  #define _BUCK_VLOOP_ISR_IE        _ADCAN13IE
#elif (BUCK_VOUT_TRIGGER_MODE == BUCK_VOUT_TRIG_PWM)
  #define _BUCK_VLOOP_Interrupt     _PWM1Interrupt   
  #define _BUCK_VLOOP_ISR_IP        _PWM1IP
  #define _BUCK_VLOOP_ISR_IF        _PWM1IF
  #define _BUCK_VLOOP_ISR_IE        _PWM1IE
#endif

#define BUCK_VOUT_ANSEL             _ANSELC1
#define BUCK_VOUT_ADCCORE           8           // 0=Dedicated Core #0, 1=Dedicated Core #1, 8=Shared ADC Core
#define BUCK_VOUT_ADCIN             13          // Analog input number (e.g. '5' for 'AN5')
#define BUCK_VOUT_ADCBUF            ADCBUF13
#define BUCK_VOUT_ADCTRIG           PG1TRIGA
#define BUCK_VOUT_TRGSRC            0b00100     // PWM1 Trigger 1
#define BUCK_VOUT_OFFSET            0

#define _BUCK_VIN_ADCInterrupt      _ADCAN12Interrupt   
#define _BUCK_VIN_ADCISR_IF         _ADCAN12IF

#define BUCK_VIN_ANSEL              _ANSELC0
#define BUCK_VIN_ADCCORE            8           // 0=Dedicated Core #0, 1=Dedicated Core #1, 8=Shared ADC Core
#define BUCK_VIN_ADCIN              12          // Analog input number (e.g. '5' for 'AN5')
#define BUCK_VIN_ADCBUF             ADCBUF12
#define BUCK_VIN_ADCTRIG            PG1TRIGB
#define BUCK_VIN_TRGSRC             0b00101     // PWM1 Trigger 2
#define BUCK_VIN_OFFSET             0
    
#define _BUCK_TEMP_ADCInterrupt     _ADCAN2Interrupt   
#define _BUCK_TEMP_ADCISR_IF        _ADCAN2IF

#define BUCK_TEMP_ANSEL             _ANSELB7
#define BUCK_TEMP_ADCCORE           8           // 0=Dedicated Core #0, 1=Dedicated Core #1, 8=Shared ADC Core
#define BUCK_TEMP_ADCIN             2           // Analog input number (e.g. '5' for 'AN5')
#define BUCK_TEMP_ADCBUF            ADCBUF2
#define BUCK_TEMP_ADCTRIG           PG1TRIGB
#define BUCK_TEMP_TRGSRC            0b00101     // PWM1 Trigger 2
#define BUCK_TEMP_OFFSET            0
        
#define _BUCK_IOUT_ADCInterrupt     _ADCAN2Interrupt   
#define _BUCK_IOUT_ADCISR_IF        _ADCAN2IF

#define BUCK_I_CT_ANSEL             _ANSELA0
#define BUCK_I_CT_ADCCORE           0           // 0=Dedicated Core #0, 1=Dedicated Core #1, 2=Shared ADC Core
#define BUCK_I_CT_ADCIN             0           // Analog input number (e.g. '5' for 'AN5')
#define BUCK_I_CT_ADCBUF            ADCBUF0
#define BUCK_I_CT_ADCTRIG           PG1TRIGA
#define BUCK_I_CT_TRGSRC            0b00100     // PWM1 Trigger 2
#define BUCK_I_CT_OFFSET            0
        
#define BUCK_I_AMP_ANSEL             _ANSELB2
#define BUCK_I_AMP_ADCCORE          1           // 0=Dedicated Core #0, 1=Dedicated Core #1, 2=Shared ADC Core
#define BUCK_I_AMP_ADCIN            1           // Analog input number (e.g. '5' for 'AN5')
#define BUCK_I_AMP_ADCBUF           ADCBUF1
#define BUCK_I_AMP_ADCTRIG          PG1TRIGA
#define BUCK_I_AMP_TRGSRC           0b00100     // PWM1 Trigger 2
#define BUCK_I_AMP_OFFSET           0


    
#define BUCK_PWM_CHANNEL            1           // PWM Instance Index (e.g. 1=PWM1, 2=PWM2, etc.)
#define BUCK_PWM_PDC                PG1DC
    
#define BUCK_PWM_ADTR1OFS           0           // ADC Trigger 1 Offset:  0...31
#define BUCK_PWM_ADTR1PS            0           // ADC Trigger 1 Postscaler: 0...31

#define BUCK_PWM_DUTY_CYCLE_MIN     (float)0.01 // ~1% On Time 
#define BUCK_PWM_DUTY_CYCLE_MAX     (float)0.90 // ~90% On Time 

#define BUCK_PWM_PERIOD             (uint16_t)(SWITCHING_PERIOD / PWM_CLOCK_PERIOD) // Period in [tick = 250ps] -> 400 kHz 
#define BUCK_PWM_PHASE_SHIFT        (uint16_t)(SWITCHING_PHASE_SHIFT / PWM_CLOCK_PERIOD) // Phase Shift in [tick = 250ps]
#define BUCK_PWM_DC_MIN             (uint16_t)(BUCK_PWM_DUTY_CYCLE_MIN * (float)BUCK_PWM_PERIOD) // This sets the minimum duty cycle
#define BUCK_PWM_DC_MAX             (uint16_t)(BUCK_PWM_DUTY_CYCLE_MAX * (float)BUCK_PWM_PERIOD) // This sets the maximum duty cycle
#define BUCK_LEB_PERIOD             (uint16_t)(SWITCHING_LEB / (float)PWM_CLOCK_PERIOD) // Leading Edge Blanking = n x PWM resolution (here: 50 x 2ns = 100ns)
#define BUCK_PWM_DEAD_TIME_RISING   (uint16_t)(SWITCHING_DEAD_TIME_LE / (float)PWM_CLOCK_PERIOD) // Rising edge dead time [tick = 250ps]
#define BUCK_PWM_DEAD_TIME_FALLING  (uint16_t)(SWITCHING_DEAD_TIME_FE / (float)PWM_CLOCK_PERIOD) // Falling edge dead time [tick = 250ps]

#define BUCK_VOUT_ADCTRG_DELAY      (uint16_t)120         // With respect to the start of the PWM cycle 

#define BUCK_PWM_OUT_PORT           (uint16_t)1           // Device Port No: 0=A, 1=B, 2=C, 3=D, etc
#define BUCK_PWMxH_OUT_PINNO        (uint16_t)14          // Device Pin No:  10=Rx10 where X is the port index A, B, C, etc
#define BUCK_PWMxL_OUT_PINNO        (uint16_t)15          // Device Pin No:  10=Rx10 where X is the port index A, B, C, etc



#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* DPSK3_HARDWARE_DESCRIPTOR_H */

