/*
 * File:   pwr_control.c
 * Author: M91406
 *
 * Created on March 12, 2020, 11:55 AM
 */

#include <p33CK256MP505.h>

#include "pwr_control/devices/dev_buck_typedef.h"
#include "pwr_control/devices/dev_buck_converter.h"
#include "pwr_control/drivers/v_loop.h"
#include "config/dpsk3_hwdescr.h"

#include "fault_handler/faults.h"

/*!BUCK_POWER_CONTROLLER_t data structure
 * *************************************************************************************************
 * Summary:
 * Global data object for a BUCK CONVERTER 
 * 
 * Description:
 * The 'buck' data object holds all status, control and monitoring values of the BUCK power 
 * controller. The BUCK_POWER_CONTROLLER_t data structure is defined in drv_buck_converter.h.
 * Please refer to the comments on top of this file for further information.
 *  
 * *************************************************************************************************/
volatile BUCK_POWER_CONTROLLER_t buck;

/* PRIVATE FUNCTION PROTOTYPES */
volatile uint16_t appPowerSupply_ConverterObjectInitialize(void);
volatile uint16_t appPowerSupply_ControllerInitialize(void);
volatile uint16_t appPowerSupply_PeripheralsInitialize(void);


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
    buck.data.temp = BUCK_TEMP_ADCBUF;
    
    // Check conditional parameters and fault flags
    buck.status.bits.power_source_detected = (bool)
        ((VIN_UVLO_TRIP < buck.data.v_in) && (buck.data.v_in<VIN_OVLO_TRIP));
    
    // Combine individual fault bits to a common fault indicator
    buck.status.bits.fault_active = (bool) (
        fltobj_BuckUVLO.status.bits.fault_status | 
        fltobj_BuckOVLO.status.bits.fault_status |
        fltobj_BuckRegErr.status.bits.fault_status 
        );
    
    // Execute buck converter state machine
    fres &= drvBuckConverter_Execute(&buck);

    // Buck regulation error is only active while controller is running
    // and while being tied to a valid reference
    if(buck.mode >= BUCK_STATE_V_RAMP_UP) {
        fltobj_BuckRegErr.ref_obj = buck.v_loop.controller->Ports.ptrControlReference;
        fltobj_BuckRegErr.status.bits.enabled = buck.v_loop.controller->status.bits.enabled;
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
    buck.set_values.i_ref = BUCK_IOUT_REF; // Set current loop reference
    buck.set_values.v_ref = BUCK_VOUT_REF; // Set voltage loop reference
    
    // Clear Runtime Data
    buck.data.v_out = 0; // Reset output voltage value
    buck.data.i_out = 0; // Reset output current value
    buck.data.v_in = 0;  // Reset input voltage value
    buck.data.temp = 0;  // Reset output temperature value
    
    // Initialize Switch Node
    buck.sw_node.pwm_instance = BUCK_PWM_CHANNEL;
    buck.sw_node.gpio_instance = BUCK_PWM_OUT_PORT;
    buck.sw_node.gpio_high = BUCK_PWMxH_OUT_PINNO;
    buck.sw_node.gpio_low = BUCK_PWMxL_OUT_PINNO;
    buck.sw_node.master_period = false;
    buck.sw_node.period = BUCK_PWM_PERIOD;
    buck.sw_node.phase = BUCK_PWM_PHASE_SHIFT;
    buck.sw_node.duty_ratio_min = BUCK_PWM_DC_MIN;
    buck.sw_node.duty_ratio_init = BUCK_PWM_DC_MIN;
    buck.sw_node.duty_ratio_max = BUCK_PWM_DC_MAX;
    buck.sw_node.dead_time_rising = BUCK_PWM_DEAD_TIME_RISING;
    buck.sw_node.dead_time_falling = BUCK_PWM_DEAD_TIME_FALLING;
    buck.sw_node.leb_period = BUCK_LEB_PERIOD;
    buck.sw_node.trigger_offset = BUCK_PWM_ADTR1OFS;
    buck.sw_node.trigger_scaler = BUCK_PWM_ADTR1PS;

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

    buck.feedback.ad_iphs.adc_input = BUCK_I_CT_ADCIN;
    buck.feedback.ad_iphs.adc_core = BUCK_I_CT_ADCCORE;
    buck.feedback.ad_iphs.adc_buffer = &BUCK_I_CT_ADCBUF;
    buck.feedback.ad_iphs.trigger_source = BUCK_I_CT_TRGSRC;

    buck.feedback.ad_iphs.differential_input = false;
    buck.feedback.ad_iphs.interrupt_enable = false;
    buck.feedback.ad_iphs.early_interrupt_enable = false;
    buck.feedback.ad_iphs.level_trigger = false;
    buck.feedback.ad_iphs.signed_result = false;

    BUCK_I_CT_ANSEL = buck.feedback.ad_iphs.enabled;
    
    // ~~~ OUTPUT CURRENT FEEDBACK END ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // ~~~ TEMPERATURE FEEDBACK ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    buck.feedback.ad_temp.enabled = false;   // Use this channel

    buck.feedback.ad_temp.adc_input = BUCK_TEMP_ADCIN;
    buck.feedback.ad_temp.adc_core = BUCK_TEMP_ADCCORE;
    buck.feedback.ad_temp.adc_buffer = &BUCK_TEMP_ADCBUF;
    buck.feedback.ad_temp.trigger_source = BUCK_TEMP_TRGSRC;

    buck.feedback.ad_temp.differential_input = false;
    buck.feedback.ad_temp.interrupt_enable = false;
    buck.feedback.ad_temp.early_interrupt_enable = false;
    buck.feedback.ad_temp.level_trigger = false;
    buck.feedback.ad_temp.signed_result = false;

    BUCK_TEMP_ANSEL = buck.feedback.ad_temp.enabled;
    
    // ~~~ TEMPERATURE FEEDBACK END ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
    buck.v_loop.trigger_offset = ((BUCK_PWM_PERIOD >> 1) + BUCK_VOUT_ADCTRG_DELAY);
    
     // Set Controller Object of Voltage Loop
    buck.v_loop.controller = &v_loop;
    buck.v_loop.ctrl_Initialization = &v_loop_Initialize;
    buck.v_loop.ctrl_Update = &v_loop_Update;
    buck.v_loop.ctrl_Reset = &v_loop_Reset;
    buck.v_loop.ctrl_Precharge = &v_loop_Precharge;
    
    // Configure Voltage Loop Controller Object
    buck.v_loop.ctrl_Initialization(&v_loop);   // Call Initialization Routine setting histories and scaling
    
    // Configure controller input/output ports
    buck.v_loop.controller->Ports.ptrSource = &BUCK_VOUT_ADCBUF; // Output Voltage is Common Source
    buck.v_loop.controller->Ports.ptrAltSource = &BUCK_VIN_ADCBUF; // Input Voltage Is Alternate Source
    buck.v_loop.controller->Ports.ptrTarget = &BUCK_PWM_PDC; // PWM Duty Cycle is Control Target
    buck.v_loop.controller->Ports.ptrAltTarget = NULL; // No alternate target used
    buck.v_loop.controller->Ports.ptrControlReference = &buck.set_values.v_ref; // Set pointer to Reference
    
    // Data Input/Output Limit Configuration
    buck.v_loop.controller->Limits.InputOffset = buck.v_loop.feedback_offset;
    buck.v_loop.controller->Limits.MinOutput = buck.v_loop.minimum;
    buck.v_loop.controller->Limits.MaxOutput = buck.v_loop.maximum;

    // ADC Trigger Control Configuration
    buck.v_loop.controller->TriggerControl.ptrADCTriggerARegister = &BUCK_VOUT_ADCTRIG;
    buck.v_loop.controller->TriggerControl.ADCTriggerAOffset = buck.v_loop.trigger_offset;
    buck.v_loop.controller->TriggerControl.ptrADCTriggerBRegister = NULL;
    buck.v_loop.controller->TriggerControl.ADCTriggerBOffset = 0;
    
    // Data Provider Configuration
    buck.v_loop.controller->DataProviders.ptrDataProviderControlInput = &buck.data.v_out;
    buck.v_loop.controller->DataProviders.ptrDataProviderControlError = NULL;
    buck.v_loop.controller->DataProviders.ptrDataProviderControlOutput = NULL;
    
    // Cascaded Function Configuration
    buck.v_loop.controller->CascadeTrigger.CascadedFunction = NULL;
    buck.v_loop.controller->CascadeTrigger.CascadedFunParam = 0;
    
    // Initialize Advanced Control Settings (not used in this code example)
    buck.v_loop.controller->Advanced.GainModulationFactor = 0x7FFF; // Normalized Adaptive Gain Modulation Factor
    buck.v_loop.controller->Advanced.GainModulationScaler = 0x0000; // Normalization Scaler
    buck.v_loop.controller->Advanced.GainModulationNorm = (0x0000); // Normalization Bit-Shift Scaler 
    buck.v_loop.controller->Advanced.AltSourceNormShift = BUCK_VIN_SCALER;
    buck.v_loop.controller->Advanced.SourceNormShift = BUCK_VOUT_SCALER;
        
    // Reset Controller Status
    buck.v_loop.controller->status.bits.enabled = false; // Keep controller disabled
    buck.v_loop.controller->status.bits.swap_source = false; // use SOURCE as major control input
    buck.v_loop.controller->status.bits.swap_target = false; // use TARGET as major control output
    buck.v_loop.controller->status.bits.invert_input = false; // Do not invert input value
    buck.v_loop.controller->status.bits.flt_clamp_min = false; // Reset Anti-Windup Minimum Status bit
    buck.v_loop.controller->status.bits.flt_clamp_max = false; // Reset Anti-Windup Minimum Status bits
    buck.v_loop.controller->status.bits.agm_enable = true;   // Enable Adaptive Gain Modulation by default

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
    buck.v_loop.ctrl_Update(buck.v_loop.controller);
    PG1STATbits.UPDREQ = 1;  // Force PWM timing update
    _BUCK_VLOOP_ISR_IF = 0;  // Clear the ADCANx interrupt flag 
DBGPIN_2_CLEAR;
}

