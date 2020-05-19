/*
 * File:   pwr_control.c
 * Author: M91406
 *
 * Created on March 12, 2020, 11:55 AM
 */

#include "./pwr_control/drivers/v_loop.h"
#include "./pwr_control/devices/dev_buck_typedef.h"
#include "devices/dev_buck_converter.h"
#include "config/epc9143_r40_hwdescr.h"

#include "fault_handler/app_faults.h"

/*!BUCK_POWER_CONTROLLER_t data structure
 * *************************************************************************************************
 * Summary:
 * Global data object for a BUCK CONVERTER 
 * 
 * Description:
 * The 'buck' data object holds all status, control and monitoring values of the BUCK power 
 * controller. The BUCK_POWER_CONTROLLER_t data structure is defined in dev_buck_converter.h.
 * Please refer to the comments on top of this file for further information.
 *  
 * *************************************************************************************************/
volatile BUCK_POWER_CONTROLLER_t buck;

/* PRIVATE FUNCTION PROTOTYPES */
volatile uint16_t appPowerSupply_ConverterObjectInitialize(void);
volatile uint16_t appPowerSupply_ControllerInitialize(void);
volatile uint16_t appPowerSupply_PeripheralsInitialize(void);

/* CUSTOM RUNTIME OPTIONS */
#define PLANT_MEASUREMENT   false

/* *************************************************************************************************
 * PUBLIC FUNCTIONS
 * ************************************************************************************************/

volatile uint16_t appPowerSupply_Initialize(void)
{ 
    volatile uint16_t fres=1;

    // Run initialization sequence
    fres &= appPowerSupply_ConverterObjectInitialize();
    fres &= appPowerSupply_ControllerInitialize();
    fres &= appPowerSupply_PeripheralsInitialize();

    // Sequence Peripheral Startup
    fres &= buckPWM_Start(buck.sw_node.pwm_instance);   // Start PWM (No Outputs Enabled)
    if (fres) buck.status.bits.pwm_active = 1;
    
    fres &= buckADC_Start();                            // Start ADC
    
    // Initialize Control Interrupt
    _BUCK_VLOOP_ISR_IP = 5;
    _BUCK_VLOOP_ISR_IF = 0;
    _BUCK_VLOOP_ISR_IE = 1;
    
    // Enable Buck Converter
    buck.status.bits.enabled = true;
    
    return(fres); 
}

volatile uint16_t appPowerSupply_Execute(void)
{ 
    volatile uint16_t fres=1;

    // Capture data values
    buck.data.v_in = BUCK_VIN_ADCBUF;
    buck.data.i_out = BUCK_VIN_ADCBUF;
/*    
    // Check conditional parameters and fault flags
    buck.status.bits.power_source_detected = (bool)
        ((VIN_UVLO_TRIP < buck.data.v_in) && (buck.data.v_in<VIN_OVLO_TRIP));
    
    // Combine individual fault bits to a common fault indicator
    buck.status.bits.fault_active = (bool) (
        fltobj_BuckUVLO.status.bits.fault_status | 
        fltobj_BuckOVLO.status.bits.fault_status |
        fltobj_BuckRegErr.status.bits.fault_status 
        );
*/
    buck.status.bits.fault_active = false;
    buck.status.bits.power_source_detected = true;
    
    // Execute buck converter state machine
    fres &= drv_BuckConverter_Execute(&buck);

    // Buck regulation error is only active while controller is running
    // and while being tied to a valid reference
    if(buck.mode >= BUCK_STATE_V_RAMP_UP) {
        fltobj_BuckRegErr.ref_obj = buck.v_loop.controller->Ports.ptrControlReference;
        #if (PLANT_MEASUREMENT == false)
        fltobj_BuckRegErr.status.bits.enabled = buck.v_loop.controller->status.bits.enabled;
        #endif
    }
    else {
        fltobj_BuckRegErr.status.bits.enabled = false;
    }
    
    return(fres); 
}

volatile uint16_t appPowerSupply_Dispose(void)
{ 
    volatile uint16_t fres=1;

    buck.status.value = 0;
    buck.data.i_out = 0;
    buck.data.v_out = 0;
    buck.data.v_in = 0;
    buck.mode = BUCK_STATE_INITIALIZE; // Set state machine
    
    return(fres); 
}

volatile uint16_t appPowerSupply_Suspend(void)
{ 
    volatile uint16_t fres=1;

    buckPWM_Suspend(buck.sw_node.pwm_instance); // Shut down PWM immediately
    buck.status.bits.fault_active = true; // Set FAULT flag
    buck.mode = BUCK_STATE_RESET; // Reset State Machine (causes loop reset)

    return(fres); 
}

volatile uint16_t appPowerSupply_Resume(void)
{ 
    volatile uint16_t fres=0;

    buck.mode = BUCK_STATE_RESET;       // Ensure State Machine is RESET
    buck.status.bits.enabled = true;    // Ensure Buck Converter is enabled
    
    return(fres); 
}

/* *************************************************************************************************
 * PRIVATE FUNCTIONS
 * ************************************************************************************************/

volatile uint16_t appPowerSupply_ConverterObjectInitialize(void)
{
    volatile uint16_t fres = 1;
    
    // Initialize Buck Converter Object Status
    buck.status.bits.adc_active = false; // Clear ADC STARTED flag
    buck.status.bits.pwm_active = false; // clear PWM STARTED flag
    buck.status.bits.power_source_detected = false; // Clear POWER SOURCE DETECTED flag
    buck.status.bits.fault_active = true; // Set global FAULT flag
    buck.status.bits.autorun = true;    // Allow the buck converter to run when cleared of faults
    buck.status.bits.enabled = false; // Disable buck converter
 
    // Set Initial State Machine State
    buck.mode = BUCK_STATE_INITIALIZE; // Reset Buck State Machine
    
    // Set Reference values
    buck.set_values.control_mode = BUCK_CONTROL_MODE_VMC; // Set Control Mode
    buck.set_values.i_ref = BUCK_ISNS_REF; // Set current loop reference
    buck.set_values.v_ref = BUCK_VOUT_REF; // Set voltage loop reference
    
    // Clear Runtime Data
    buck.data.v_out = 0; // Reset output voltage value
    buck.data.i_out = 0; // Reset output current value
    buck.data.v_in = 0;  // Reset input voltage value
    buck.data.temp = 0;  // Reset output temperature value
    
    // Initialize Switch Node
    buck.sw_node.pwm_instance = BUCK_PWM1_CHANNEL;
    buck.sw_node.gpio_instance = BUCK_PWM1_GPIO_INSTANCE;
    buck.sw_node.gpio_high = BUCK_PWM1_GPIO_PORT_PINH;
    buck.sw_node.gpio_low = BUCK_PWM1_GPIO_PORT_PINL;
    buck.sw_node.master_period = false;
    buck.sw_node.period = BUCK_PWM_PERIOD;
    buck.sw_node.phase = BUCK_PWM_PHASE_SHIFT;
    buck.sw_node.duty_ratio_min = BUCK_PWM_DC_MIN;
    buck.sw_node.duty_ratio_init = BUCK_PWM_DC_MIN;
    buck.sw_node.duty_ratio_max = BUCK_PWM_DC_MAX;
    buck.sw_node.dead_time_rising = BUCK_PWM_DEAD_TIME_LE;
    buck.sw_node.dead_time_falling = BUCK_PWM_DEAD_TIME_FE;
    buck.sw_node.leb_period = BUCK_LEB_PERIOD;
    buck.sw_node.trigger_offset = BUCK_PWM1_ADTR1OFS;
    buck.sw_node.trigger_scaler = BUCK_PWM1_ADTR1PS;

    // Initialize Feedback Channels
    
    // ~~~ OUTPUT VOLTAGE FEEDBACK ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    buck.feedback.ad_vout.enabled = true;   // Use this channel

    buck.feedback.ad_vout.adc_input = BUCK_VOUT_ADCIN;
    buck.feedback.ad_vout.adc_core = BUCK_VOUT_ADCCORE;
    buck.feedback.ad_vout.adc_buffer = &BUCK_VOUT_ADCBUF;
    buck.feedback.ad_vout.trigger_source = BUCK_VOUT_TRGSRC;

    buck.feedback.ad_vout.differential_input = false;
    buck.feedback.ad_vout.interrupt_enable = false;
    buck.feedback.ad_vout.early_interrupt_enable = false;
    buck.feedback.ad_vout.level_trigger = false;
    buck.feedback.ad_vout.signed_result = false;
    
    BUCK_VOUT_ANSEL = buck.feedback.ad_vout.enabled;
    
    // ~~~ OUTPUT VOLTAGE FEEDBACK END ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    // ~~~ INPUT VOLTAGE FEEDBACK ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    buck.feedback.ad_vin.enabled = true;   // Use this channel

    buck.feedback.ad_vin.adc_input = BUCK_VIN_ADCIN;
    buck.feedback.ad_vin.adc_core = BUCK_VIN_ADCCORE;
    buck.feedback.ad_vin.adc_buffer = &BUCK_VIN_ADCBUF;
    buck.feedback.ad_vin.trigger_source = BUCK_VIN_TRGSRC;

    buck.feedback.ad_vin.differential_input = false;
    buck.feedback.ad_vin.interrupt_enable = false;
    buck.feedback.ad_vin.early_interrupt_enable = false;
    buck.feedback.ad_vin.level_trigger = false;
    buck.feedback.ad_vin.signed_result = false;

    BUCK_VIN_ANSEL = buck.feedback.ad_vin.enabled;
    
    // ~~~ INPUT VOLTAGE FEEDBACK END ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // ~~~ OUTPUT CURRENT FEEDBACK ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    buck.feedback.ad_iphs.enabled = false;   // Use this channel

    buck.feedback.ad_iphs.adc_input = BUCK_ISNS1_ADCIN;
    buck.feedback.ad_iphs.adc_core = BUCK_ISNS1_ADCCORE;
    buck.feedback.ad_iphs.adc_buffer = &BUCK_ISNS1_ADCBUF;
    buck.feedback.ad_iphs.trigger_source = BUCK_ISNS1_TRGSRC;

    buck.feedback.ad_iphs.differential_input = false;
    buck.feedback.ad_iphs.interrupt_enable = false;
    buck.feedback.ad_iphs.early_interrupt_enable = false;
    buck.feedback.ad_iphs.level_trigger = false;
    buck.feedback.ad_iphs.signed_result = false;

    BUCK_ISNS1_ANSEL = buck.feedback.ad_iphs.enabled;
    
    // ~~~ OUTPUT CURRENT FEEDBACK END ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Initialize Startup Settings
    
    buck.startup.power_on_delay.counter = 0;
    buck.startup.power_on_delay.period = BUCK_POD;
    buck.startup.power_on_delay.ref_inc_step = 0;
    buck.startup.power_on_delay.reference = 0;
    
    buck.startup.v_ramp.counter = 0;
    buck.startup.v_ramp.period = BUCK_VRAMP_PER;
    buck.startup.v_ramp.ref_inc_step = BUCK_VREF_STEP;
    if (buck.startup.v_ramp.ref_inc_step == 0)
        buck.startup.v_ramp.ref_inc_step = 1;
    buck.startup.v_ramp.reference = 0;
    
    if (buck.set_values.control_mode == BUCK_CONTROL_MODE_ACMC) 
    {
        buck.startup.i_ramp.counter = 0;
        buck.startup.i_ramp.period = BUCK_IRAMP_PER;
        buck.startup.i_ramp.ref_inc_step = BUCK_IREF_STEP;
        buck.startup.i_ramp.reference = 0;
    }
    
    buck.startup.power_good_delay.counter = 0;
    buck.startup.power_good_delay.period = BUCK_PGD;
    buck.startup.power_good_delay.ref_inc_step = 0;
    buck.startup.power_good_delay.reference = BUCK_VOUT_REF;
    
    
    return(fres);
}

volatile uint16_t appPowerSupply_PeripheralsInitialize(void)
{
    volatile uint16_t fres=1;
    
 
    fres &= buckPWM_ModuleInitialize(&buck); // Initialize PWM Module
    fres &= buckPWM_VMC_Initialize(&buck);  // Initialize PWM Channel of Buck Converter
    
    fres &= buckADC_ModuleInitialize();     // Initialize ADC Module
    
    fres &= buckADC_Channel_Initialize(&buck.feedback.ad_vout); // Initialize Output Voltage Channel
    fres &= buckADC_Channel_Initialize(&buck.feedback.ad_vin);  // Initialize Input Voltage Channel
    fres &= buckADC_Channel_Initialize(&buck.feedback.ad_iphs); // Initialize Output Current Channel
    fres &= buckADC_Channel_Initialize(&buck.feedback.ad_temp); // Initialize Temperature Channel
    
    // Custom configurations
    ADCON4Hbits.C1CHS = 1; // Set ADC channel input to ANA1
    
    return(fres);
}

volatile uint16_t appPowerSupply_ControllerInitialize(void)
{
    volatile uint16_t fres = 1;
    
    // ~~~ VOLTAGE LOOP CONFIGURATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    // Initialize Default Loop Configuration
    buck.v_loop.feedback_offset = BUCK_VOUT_OFFSET;
    buck.v_loop.reference = BUCK_VOUT_REF;
    buck.v_loop.minimum = BUCK_PWM_DC_MIN;
    buck.v_loop.maximum = BUCK_PWM_DC_MAX;
    buck.v_loop.trigger_offset = ((BUCK_PWM_PERIOD >> 1) + BUCK_VOUT_ADC_TRGDLY);
    
     // Set Controller Object of Voltage Loop
    buck.v_loop.controller = &v_loop;
    buck.v_loop.ctrl_Initialization = &v_loop_Initialize;
    buck.v_loop.ctrl_Update = &v_loop_Update;
    buck.v_loop.ctrl_Reset = &v_loop_Reset;
    buck.v_loop.ctrl_Precharge = &v_loop_Precharge;
    
    // Configure Voltage Loop Controller Object
    buck.v_loop.ctrl_Initialization(&v_loop);   // Call Initialization Routine setting histories and scaling
    
    // Configure controller input ports
    buck.v_loop.controller->Ports.Source.ptrAddress = &BUCK_VOUT_ADCBUF; // Output Voltage is Common Source
    buck.v_loop.controller->Ports.Source.Offset = buck.v_loop.feedback_offset; // Output Voltage feedback signal offset 
    buck.v_loop.controller->Ports.Source.NormScaler = BUCK_VOUT_NORM_SCALER; // Output voltage normalization factor bit-shift scaler 
    buck.v_loop.controller->Ports.Source.NormFactor = BUCK_VOUT_NORM_FACTOR; // Output voltage normalization factor fractional
    
    buck.v_loop.controller->Ports.AltSource.ptrAddress = &BUCK_VIN_ADCBUF; // Input Voltage Is Alternate Source
    buck.v_loop.controller->Ports.AltSource.Offset = BUCK_VIN_FB_OFFSET; // Input Voltage feedback signal offset 
    buck.v_loop.controller->Ports.AltSource.NormScaler = BUCK_VIN_NORM_SCALER; // Input voltage normalization factor bit-shift scaler 
    buck.v_loop.controller->Ports.AltSource.NormFactor = BUCK_VIN_NORM_FACTOR; // Input voltage normalization factor fractional

    // Configure controller output ports
    buck.v_loop.controller->Ports.Target.ptrAddress = &BUCK_PWM1_PDC; // PWM Duty Cycle is Control Target
    buck.v_loop.controller->Ports.Target.Offset = 0; // Static primary output value offset
    buck.v_loop.controller->Ports.Target.NormScaler = 0; // Primary control output normalization factor bit-shift scaler 
    buck.v_loop.controller->Ports.Target.NormFactor = 0x7FFF; // Primary control output normalization factor fractional 

    buck.v_loop.controller->Ports.AltTarget.ptrAddress = NULL; // No alternate target used
    buck.v_loop.controller->Ports.AltTarget.Offset = 0; // Static secondary output value offset
    buck.v_loop.controller->Ports.AltTarget.NormScaler = 0; // Secondary control output normalization factor bit-shift scaler
    buck.v_loop.controller->Ports.AltTarget.NormFactor = 0x7FFF; // Secondary control output normalization factor fractional 
    
    // Configure controller control ports
    buck.v_loop.controller->Ports.ptrControlReference = &buck.set_values.v_ref; // Set pointer to Reference
    
    // Data Input/Output Limit Configuration
    buck.v_loop.controller->Limits.MinOutput = buck.v_loop.minimum;
    buck.v_loop.controller->Limits.MaxOutput = buck.v_loop.maximum;
    buck.v_loop.controller->Limits.AltMinOutput = 0; // not used
    buck.v_loop.controller->Limits.AltMaxOutput = 0; // not used

    // ADC Trigger Control Configuration
    buck.v_loop.controller->ADCTriggerControl.ptrADCTriggerARegister = &BUCK_VOUT_ADCTRIG;
    buck.v_loop.controller->ADCTriggerControl.ADCTriggerAOffset = buck.v_loop.trigger_offset;
    buck.v_loop.controller->ADCTriggerControl.ptrADCTriggerBRegister = NULL;
    buck.v_loop.controller->ADCTriggerControl.ADCTriggerBOffset = 0;
    
    // Data Provider Configuration
    buck.v_loop.controller->DataProviders.ptrDProvControlInput = &buck.data.v_out;
    buck.v_loop.controller->DataProviders.ptrDProvControlError = NULL;
    buck.v_loop.controller->DataProviders.ptrDProvControlOutput = NULL;
    
    // Cascaded Function Configuration
    buck.v_loop.controller->CascadeTrigger.ptrCascadedFunction = NULL;
    buck.v_loop.controller->CascadeTrigger.CascadedFunParam = 0;
    
    // Initialize Advanced Control Settings (not used in this code example)
    buck.v_loop.controller->GainControl.AgcFactor = 0x7FFF; // Adaptive Gain Control factor fractional
    buck.v_loop.controller->GainControl.AgcScaler = 0x0000; // Adaptive Gain Control factor bit-shift scaler
    buck.v_loop.controller->GainControl.AgcMedian = 0x0000; // Q15 number representing normalized Nominal Operating Point

    // Custom Advanced Control Settings
    buck.v_loop.controller->Advanced.advParam1 = 0; // No additional advanced control options used
    buck.v_loop.controller->Advanced.advParam2 = 0; // No additional advanced control options used
    buck.v_loop.controller->Advanced.advParam3 = 0; // No additional advanced control options used
    buck.v_loop.controller->Advanced.advParam4 = 0; // No additional advanced control options used
    
    // Reset Controller Status
    buck.v_loop.controller->status.bits.enabled = false; // Keep controller disabled
    buck.v_loop.controller->status.bits.swap_source = false; // use SOURCE as major control input
    buck.v_loop.controller->status.bits.swap_target = false; // use TARGET as major control output
    buck.v_loop.controller->status.bits.invert_input = false; // Do not invert input value
    buck.v_loop.controller->status.bits.flt_clamp_min = false; // Reset Anti-Windup Minimum Status bit
    buck.v_loop.controller->status.bits.flt_clamp_max = false; // Reset Anti-Windup Minimum Status bits
    buck.v_loop.controller->status.bits.agc_enabled = false;   // Enable Adaptive Gain Modulation by default

    // ~~~ VOLTAGE LOOP CONFIGURATION END ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    
    return(fres);
}

/*!Power Converter Control Loop Interrupt
 * **************************************************************************************************
 * 
 * **************************************************************************************************/

void __attribute__((__interrupt__, auto_psv, context))_BUCK_VLOOP_Interrupt(void)
{
DBGPIN_2_SET;

    buck.status.bits.adc_active = true;
    #if (PLANT_MEASUREMENT == false)
    buck.v_loop.ctrl_Update(buck.v_loop.controller);
    #else
    v_loop_PTermUpdate(&v_loop);
    #endif
    PG1STATbits.UPDREQ = 1;  // Force PWM timing update
    _BUCK_VLOOP_ISR_IF = 0;  // Clear the ADCANx interrupt flag 

DBGPIN_2_CLEAR;
}

