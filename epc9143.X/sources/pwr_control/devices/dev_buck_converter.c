/*
 * File:   dev_buck_converter.c
 * Author: M91406
 *
 * Created on July 9, 2019, 1:10 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "pwr_control/devices/dev_buck_typedef.h"
#include "pwr_control/devices/dev_buck_converter.h"

volatile uint16_t drv_BuckConverter_Initialize(volatile BUCK_POWER_CONTROLLER_t* buckInstance) {

    volatile uint16_t fres = 1;
    
    buckInstance->v_loop.controller->status.bits.enabled = false; // Disable voltage loop
    
    if (buckInstance->set_values.control_mode == BUCK_CONTROL_MODE_ACMC) // In current mode...
        buckInstance->i_loop.controller->status.bits.enabled = false; // Disable current loop
    
    buckInstance->status.bits.enabled = false;  // Disable Buck Converter
    buckInstance->mode = BUCK_STATE_INITIALIZE; // Reset state machine
    
    return(fres);
}

volatile uint16_t drv_BuckConverter_Execute(volatile BUCK_POWER_CONTROLLER_t* buckInstance) {
    
    volatile uint16_t fres = 1;
    volatile float fdummy = 0.0;
    volatile uint16_t int_dummy = 0;

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    /* DISABLE-RESET                                                                      */
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    // When enable status has changed from ENABLED to DISABLED, reset the state machine
    if ((!buckInstance->status.bits.enabled) || 
        (!buckInstance->status.bits.power_source_detected) ||
        (buckInstance->status.bits.fault_active))
    {
        if (!buckInstance->status.bits.ready)
            buckInstance->mode = BUCK_STATE_INITIALIZE;
        else
            buckInstance->mode = BUCK_STATE_RESET;
    }    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    /* EXECUTE STATE MACHINE                                                              */
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    switch (buckInstance->mode) {

        /*!BUCK_STATE_INITIALIZE
         * ============================
         * If the controller has not been run yet, the POWER ON and POWER GOOD delay
         * counters are reset and conditional flag bits are cleared. Status of 
         * power source, ADC and current sensor calibration have to be set during
         * runtime by system check routines. 
         * */
        case BUCK_STATE_INITIALIZE:

            // Set the BUSY bit indicating a delay/ramp period being executed
            buckInstance->status.bits.busy = true;
            
            buckInstance->startup.power_on_delay.counter = 0;   // Reset power on counter
            buckInstance->startup.power_good_delay.counter = 0; // Reset power good counter

            // Reset all status bits
            buckInstance->status.bits.power_source_detected = false;
            buckInstance->status.bits.adc_active = false;

            // Initiate current sensor calibration flag bit
            if (buckInstance->set_values.control_mode == BUCK_CONTROL_MODE_VMC)
                buckInstance->status.bits.cs_calib_ready = true; 
            else
                buckInstance->status.bits.cs_calib_ready = false; 
                
            // switch to soft-start phase RESUME
            buckInstance->mode = BUCK_STATE_RESET;

            break;

        /*!BUCK_STATE_RESET
         * ============================
         * After successful initialization or after an externally triggered state machine reset,
         * the state machine returns to this RESET mode, re-initiating control mode, references 
         * and status bits before switching further into STANDBY mode
         * */
        case BUCK_STATE_RESET:

            // Set the BUSY bit indicating a delay/ramp period being executed
            buckInstance->status.bits.busy = true;

            // Disable PWM outputs & control loops (immediate power cut-off)
            buckPWM_Suspend(buckInstance->sw_node.pwm_instance); // Disable PWM outputs

            // Disable voltage loop controller and reset control loop histories
            buckInstance->v_loop.controller->status.bits.enabled = false; // disable voltage control loop
            buckInstance->v_loop.ctrl_Reset(buckInstance->v_loop.controller); // Reset control histories of outer voltage controller
            *buckInstance->v_loop.controller->Ports.Target.ptrAddress = 
                buckInstance->v_loop.controller->Limits.MinOutput;
            
            // Disable current loop controller and reset control loop histories
            if (buck.set_values.control_mode == BUCK_CONTROL_MODE_ACMC) 
            {   // Disable all current control loops and reset control loop histories
                buckInstance->i_loop.controller->status.bits.enabled = false; 
                buckInstance->i_loop.ctrl_Reset(buckInstance->i_loop.controller); 
                *buckInstance->i_loop.controller->Ports.Target.ptrAddress = 
                    buckInstance->i_loop.controller->Limits.MinOutput;
            }
                
            // Switch to STANDBY mode
            buckInstance->mode = BUCK_STATE_STANDBY;  
            
            break;
            
        /*!BUCK_STATE_STANDBY
         * ============================
         * After a successful state machine reset, the state machine waits in  
         * STANDBY mode until all conditional flag bits are set/cleared allowing  
         * the converter to run.
         * */
        case BUCK_STATE_STANDBY:
            
            // Set the BUSY bit indicating a delay/ramp period being executed
            buckInstance->status.bits.busy = false;

            // if the 'autorun' option is set, automatically set the GO bit when the 
            // converter is enabled
            if ((buckInstance->status.bits.enabled) && (buckInstance->status.bits.autorun))
            { buckInstance->status.bits.GO = true; }
            
            // Wait for all startup conditions to be met
            if ((buckInstance->status.bits.enabled) &&          // state machine needs to be enabled
                (buckInstance->status.bits.GO) &&               // GO-bit needs to be set
                (buckInstance->status.bits.adc_active) &&       // ADC needs to be running
                (buckInstance->status.bits.pwm_active) &&       // PWM needs to be running 
                (!buckInstance->status.bits.fault_active) &&    // No active fault is present
                (buckInstance->status.bits.cs_calib_ready)      // Current Sensor Calibration complete
                )
            {
                // switch to soft-start phase POWER-ON DELAY
                buckInstance->status.bits.GO = false;   // Clear the GO bit
                buckInstance->mode = BUCK_STATE_POWER_ON_DELAY; // Step over of POD
            }
                
            break;
            
            
        /*!BUCK_STATE_POWER_ON_DELAY
         * ================================
         * After the converter has been cleared to get started, the power-on 
         * delay counter until the defined power-on delay period has expired.
         * */
        case BUCK_STATE_POWER_ON_DELAY:

            buckInstance->status.bits.busy = true;  // Set the BUSY bit

            // delay startup until POWER ON DELAY has expired
            
            if(buckInstance->startup.power_on_delay.counter++ > buckInstance->startup.power_on_delay.period)
            {
                // Clamp POD counter to EXPIRED
                buckInstance->startup.power_on_delay.counter = 
                    (buckInstance->startup.power_on_delay.period + 1); // Saturate power on counter

                if (buckInstance->status.bits.cs_calib_ready) { // if current sensors is calibrated
                    buckInstance->mode = BUCK_STATE_LAUNCH_V_RAMP; // ramp up output
                }

            }
                
            break;

        /*!BUCK_STATE_LAUNCH_V_RAMP
         * ================================
         * After the POWER ON DELAY has expired, the ramp up starting point is determined by measuring the input and output 
         * voltage and calculates the ideal duty ratio of the PWM. This value is then programmed into
         * the PWM module duty cycle register and is also used to pre-charge the control loop output
         * history. In addition the measured output voltage also set as reference to ensure the loop 
         * starts without jerks and jumps.
         * When voltage mode control is enabled, the voltage loop control history is charged, 
         * when average current mode control is enabled, the current loop control history is charged.
         *  */
        case BUCK_STATE_LAUNCH_V_RAMP:

            buckInstance->status.bits.busy = true;  // Set the BUSY bit
            
            // Hijack voltage loop controller reference
            buckInstance->startup.v_ramp.reference = 0; // Reset Soft-Start Voltage Reference
            buckInstance->startup.i_ramp.reference = 0; // Reset Soft-Start Current Reference
            buckInstance->v_loop.controller->Ports.ptrControlReference = 
                &buckInstance->startup.v_ramp.reference; // Voltage loop is pointing to Soft-Start Reference

            // Pre-charge reference
            // Never start above the pre-biased output voltage.
            // Always start at or slightly below the pre-biased output voltage
            buckInstance->startup.v_ramp.reference = buckInstance->data.v_out;
            
            // In average current mode, set current reference limit to max startup current level
            if (buck.set_values.control_mode == BUCK_CONTROL_MODE_ACMC) 
            {   // Disable all current control loops and reset control loop histories
                buckInstance->v_loop.maximum = buckInstance->set_values.i_ref;
                buckInstance->v_loop.controller->Limits.MaxOutput = buckInstance->v_loop.maximum;
            }
                
            // Pre-charge PWM and control loop history ToDo: FIX VIN-2-VOUT SCALING ISSUE !!!!
            fdummy = (float)(buckInstance->data.v_out) / (float)(buckInstance->data.v_in << 1);
            int_dummy = (uint16_t)(fdummy * (float)buckInstance->sw_node.period);
            
            if (buckInstance->set_values.control_mode == BUCK_CONTROL_MODE_VMC)
            {
                if(int_dummy < buckInstance->v_loop.minimum) 
                { int_dummy = buckInstance->v_loop.minimum; }
                else if(int_dummy > buckInstance->v_loop.maximum) 
                { int_dummy = buckInstance->v_loop.maximum; }

                buckInstance->v_loop.ctrl_Precharge(buckInstance->v_loop.controller, 0, int_dummy);
                *buckInstance->v_loop.controller->Ports.Target.ptrAddress = int_dummy; // set initial PWM duty ratio

            }
            else if (buckInstance->set_values.control_mode == BUCK_CONTROL_MODE_ACMC) 
            {   
                if(int_dummy < buckInstance->i_loop.minimum) 
                { int_dummy = buckInstance->i_loop.minimum; }
                else if(int_dummy > buckInstance->i_loop.maximum) 
                { int_dummy = buckInstance->i_loop.maximum; }

                buckInstance->i_loop.ctrl_Precharge(
                            buckInstance->i_loop.controller, 0, int_dummy
                        );

                *buckInstance->i_loop.controller->Ports.Target.ptrAddress = int_dummy; // set initial PWM duty ratio

            }

            // switch to soft-start phase RAMP UP
            buckInstance->mode = BUCK_STATE_V_RAMP_UP;
            
            break;
            
        /*!BUCK_STATE_V_RAMP_UP
         * ===========================
         * This is the essential step in which the output voltage is ramped up by incrementing the 
         * outer control loop reference. In voltage mode the output voltage will ramp up to the 
         * nominal regulation point. 
         * In average current mode the inner loop will limit the voltage as soon as the current 
         * reference limit is hit and the output is switched to constant current mode. 
         * */
        case BUCK_STATE_V_RAMP_UP:

            buckInstance->status.bits.busy = true;  // Set the BUSY bit

            // enable control loop
            if(!buckInstance->v_loop.controller->status.bits.enabled) {

                // Enable input power source
                buckPWM_Resume(buckInstance->sw_node.pwm_instance);  // Enable PWM outputs

                if (buckInstance->set_values.control_mode == BUCK_CONTROL_MODE_VMC)
                {   
                    buckInstance->v_loop.controller->status.bits.enabled = true; // enable voltage loop controller
                }
                else if (buckInstance->set_values.control_mode == BUCK_CONTROL_MODE_ACMC)
                {
                    buckInstance->v_loop.controller->status.bits.enabled = true; // enable voltage loop controller
                    buckInstance->i_loop.controller->status.bits.enabled = true; // enable phase current loop controller
                }

            }
            
            // increment reference
            buckInstance->startup.v_ramp.reference += buckInstance->startup.v_ramp.ref_inc_step;
            
            // check if ramp is complete
            if (buckInstance->startup.v_ramp.reference > buckInstance->v_loop.reference) 
            {
                // Set reference to the desired level
                buckInstance->startup.v_ramp.reference = buckInstance->v_loop.reference;
                
                // Reconnect API reference to controller
                buckInstance->v_loop.controller->Ports.ptrControlReference = &buckInstance->v_loop.reference;
                
                if (buckInstance->set_values.control_mode == BUCK_CONTROL_MODE_VMC)
                    buckInstance->mode = BUCK_STATE_PWRGOOD_DELAY;
                else if (buckInstance->set_values.control_mode == BUCK_CONTROL_MODE_ACMC)
                    buckInstance->mode = BUCK_STATE_I_RAMP_UP;
                
            }

            break;

        /*!BUCK_STATE_I_RAMP_UP
         * ===========================
         * This phase of the soft-start ramp is only executed in average current mode and will 
         * only take effect when the current limit is hit before the nominal voltage regulation 
         * point. In this case the constant output current is ramped up to from the startup current
         * to the nominal constant charging current. 
         * */
        case BUCK_STATE_I_RAMP_UP:

            if (buckInstance->set_values.control_mode == BUCK_CONTROL_MODE_ACMC)
            {
            
                buckInstance->status.bits.busy = true;  // Set the BUSY bit

                // increment current limit
                buckInstance->v_loop.controller->Limits.MaxOutput += buckInstance->startup.i_ramp.ref_inc_step; // Increment maximum current limit

                // check if ramp is complete
                if (buckInstance->v_loop.controller->Limits.MaxOutput >= buckInstance->set_values.i_ref)
                {
                    buckInstance->v_loop.maximum = buckInstance->set_values.i_ref;
                    buckInstance->v_loop.controller->Limits.MaxOutput = buckInstance->v_loop.maximum;
                    buckInstance->mode = BUCK_STATE_PWRGOOD_DELAY;
                }
            
            }
            else // Non-Current Loops Ending up here need to be lifted to PG_DELAY
            { 
                buckInstance->v_loop.controller->Limits.MaxOutput = buckInstance->v_loop.maximum;
                buckInstance->mode = BUCK_STATE_PWRGOOD_DELAY; 
            }
            
            break;

        /*!BUCK_STATE_PWRGOOD_DELAY
         * =============================
         * In this phase of the soft-start procedure a counter is incremented until the power good 
         * delay has expired before the soft-start process is marked as COMPLETEd 
         * */
        case BUCK_STATE_PWRGOOD_DELAY:
            // POWER GOOD Delay

            buckInstance->status.bits.busy = true;  // Set the BUSY bit
            
            // increment delay counter until the GOWER GOOD delay has expired
            if(buckInstance->startup.power_good_delay.counter++ > buckInstance->startup.power_good_delay.period)
            {
                buckInstance->mode = BUCK_STATE_ONLINE; // Set COMPLETE flag
                buckInstance->startup.power_good_delay.counter = 
                    (buckInstance->startup.power_good_delay.period + 1); // Clamp to PERIOD_EXPIRED for future startups
            }

            break;
            
        /*!BUCK_STATE_COMPLETE
         * =============================
         * When the soft-start step is set to BUCK_STATE_CHARGING, the soft-start state machine 
         * will not be executed any further and the converter has entered normal operation.  
         * */
        case BUCK_STATE_ONLINE:
            
            /*!Runtime Reference Tuning
             * ==================================================================================
             * Description:
             * If the user reference setting has been changed and is different from the most recent 
             * controller reference/current clamping, the state machine will tune the controller 
             * reference/current clamping into the new user control reference level. 
             * While ramping the output voltage up or down, the BUSY bit will be set and any new 
             * changes to the reference will be ignored until the ramp up/down is complete.
             * =================================================================================*/

            if(buckInstance->set_values.v_ref != buckInstance->v_loop.reference) 
            {
                // Set the BUSY bit indicating a delay/ramp period being executed
                buckInstance->status.bits.busy = true;
                
                // New reference value is less than controller reference value => ramp down
                if(buckInstance->set_values.v_ref < buckInstance->v_loop.reference){
                    // decrement reference until new reference level is reached
                    buckInstance->v_loop.reference -= buckInstance->startup.v_ramp.ref_inc_step; // decrement reference
                    if(buckInstance->v_loop.reference < buckInstance->set_values.v_ref) { // check if ramp is complete
                        buckInstance->v_loop.reference = buckInstance->set_values.v_ref; // clamp reference level to setting
                    }
                        
                }
                // New reference value is greater than controller reference value => ramp up
                else if(buckInstance->set_values.v_ref > buckInstance->v_loop.reference){
                    // increment reference until new reference level is reached
                    buckInstance->v_loop.reference += buckInstance->startup.v_ramp.ref_inc_step; // increment reference
                    if(buckInstance->v_loop.reference > buckInstance->set_values.v_ref) { // check if ramp is complete
                        buckInstance->v_loop.reference = buckInstance->set_values.v_ref; // clamp reference level to setting
                    }
                        
                }
                
            }
            else{
                // Clear the BUSY bit indicating "no state machine activity"
                buckInstance->status.bits.busy = false;
            }
              
            break;

        /*!BUCK_STATE_SUSPEND
         * =============================
         * When the state machine step is set to BATCRG_STEP_SUSPEND, the PWM and control
         * loops are reset and the state machine is reset to RESUME mode WITH all
         * delay counters pre-charged. When re-enabled, the state machine will perform 
         * a soft-start without POWER ON and POWER GOOD delays.
         * */
        case BUCK_STATE_SUSPEND:

            // Disable PWM outputs & control loops (immediate power shut-down)
            buckPWM_Stop(buckInstance->sw_node.pwm_instance); // Disable PWM outputs
            
            buckInstance->v_loop.controller->status.bits.enabled = false;   // disable voltage control loop
            
            if (buckInstance->set_values.control_mode == BUCK_CONTROL_MODE_ACMC)
                buckInstance->i_loop.controller->status.bits.enabled = false;  // disable current control loop
            
            // Reset the state machine to repeat the soft-start without delays (ramp only)
            buckInstance->startup.power_on_delay.counter = (buckInstance->startup.power_on_delay.period + 1); // Clamp POD counter to PERIOD_EXPIRED for future startups
            buckInstance->startup.power_good_delay.counter = (buckInstance->startup.power_good_delay.period + 1); // Clamp PG counter to PERIOD_EXPIRED for future startups

            buckInstance->mode = BUCK_STATE_RESET; // Reset state machine to RESUME
            
            break;

        /*!BUCK_STATE_UNDEFINED
         * ===========================
         * When this state-machine step is executed, something went wrong in the master state machine.
         * To fix this issue the state-machine will be reset to INITIALIZATE.  
         * */
        default:
            buckInstance->mode = BUCK_STATE_INITIALIZE;
            break;
    }            
    
    return(fres);
}

volatile uint16_t drv_BuckConverter_Start(volatile BUCK_POWER_CONTROLLER_t* buckInstance) {

    volatile uint16_t fres=1;
    
    // Start PWM with its outputs disabled
    fres &= buckPWM_Start(buckInstance->sw_node.pwm_instance);
    
    buckInstance->v_loop.controller->status.bits.enabled = false; // Disable voltage loop
    buckInstance->v_loop.ctrl_Reset(buckInstance->v_loop.controller); // Reset voltage loop histories

    if (buckInstance->set_values.control_mode == BUCK_CONTROL_MODE_ACMC) { // In current mode...
        buckInstance->i_loop.controller->status.bits.enabled = false; // Disable current loop
        buckInstance->i_loop.ctrl_Reset(buckInstance->i_loop.controller); // Reset current loop histories
    }
    
    buckInstance->status.bits.enabled = true;   // Enable Buck Converter
    buckInstance->mode = BUCK_STATE_INITIALIZE; // Reset state machine

    return(fres);
}

volatile uint16_t drv_BuckConverter_Stop(volatile BUCK_POWER_CONTROLLER_t* buckInstance) {

    volatile uint16_t fres=1;
    
    // Stop PWM completely (shuts down PWM generator)
    fres &= buckPWM_Stop(buckInstance->sw_node.pwm_instance); // Stop PWM
    
    buckInstance->v_loop.controller->status.bits.enabled = false; // Disable voltage loop
    
    if (buckInstance->set_values.control_mode == BUCK_CONTROL_MODE_ACMC) // In current mode...
        buckInstance->i_loop.controller->status.bits.enabled = false; // Disable current loop
    
    buckInstance->status.bits.enabled = false;  // Disable Buck Converter
    buckInstance->mode = BUCK_STATE_INITIALIZE; // Reset state machine

    return(fres);
}

volatile uint16_t drv_BuckConverter_Suspend(volatile BUCK_POWER_CONTROLLER_t* buckInstance) {
    
    volatile uint16_t fres=1;
    
    buckInstance->mode = BUCK_STATE_SUSPEND;
    fres &= drv_BuckConverter_Execute(buckInstance);
    
    return(fres);
}

volatile uint16_t drv_BuckConverter_Resume(volatile BUCK_POWER_CONTROLLER_t* buckInstance) {
    
    volatile uint16_t fres=1;
    
    buckInstance->mode = BUCK_STATE_RESET;
    fres &= drv_BuckConverter_Execute(buckInstance);
    
    return(fres);
}

// END OF FILE