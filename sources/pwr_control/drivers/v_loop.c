/* **********************************************************************************
 * z-Domain Control Loop Designer, Version 0.9.6.97
 * **********************************************************************************
 * 3p3z compensation filter coefficients derived for following operating
 * conditions:
 * **********************************************************************************
 *
 *  Controller Type:    3P3Z - Basic Voltage Mode Compensator
 *  Sampling Frequency: 500000 Hz
 *  Fixed Point Format: 15
 *  Scaling Mode:       4 - Fast Floating Point Coefficient Scaling
 *  Input Gain:         0.5
 *
 * *********************************************************************************
 * CGS Version:         2.0.3
 * CGS Date:            03/31/2020
 * *********************************************************************************
 * User:                M91406
 * Date/Time:           03/31/2020 5:32:23 PM
 * ********************************************************************************/

#include "./pwr_control/drivers/v_loop.h"

/* *********************************************************************************
 * Data Arrays:
 * This source file declares the default parameters of the z-domain compensation
 * filter. The cNPNZ_t data structure contains two pointers to A- and B-
 * coefficient arrays and two pointers to control and error history arrays.
 *
 * For optimized data processing during DSP computations, these arrays must be
 * located in specific memory locations (X-space for coefficient arrays and
 * Y-space for control and error history arrays).
 *
 * The following declarations are used to define the array data contents, their
 * length and memory location. These declarations are made publicly accessible
 * through defines in source file v_loop.c
 * ********************************************************************************/

volatile V_LOOP_CONTROL_LOOP_COEFFICIENTS_t __attribute__((space(xmemory), near)) v_loop_coefficients; // A/B-Coefficients
volatile uint16_t v_loop_ACoefficients_size = (sizeof(v_loop_coefficients.ACoefficients)/sizeof(v_loop_coefficients.ACoefficients[0])); // A-coefficient array size
volatile uint16_t v_loop_BCoefficients_size = (sizeof(v_loop_coefficients.BCoefficients)/sizeof(v_loop_coefficients.BCoefficients[0])); // B-coefficient array size

volatile V_LOOP_CONTROL_LOOP_HISTORIES_t __attribute__((space(ymemory), far)) v_loop_histories; // Control/Error Histories
volatile uint16_t v_loop_ControlHistory_size = (sizeof(v_loop_histories.ControlHistory)/sizeof(v_loop_histories.ControlHistory[0])); // Control history array size
volatile uint16_t v_loop_ErrorHistory_size = (sizeof(v_loop_histories.ErrorHistory)/sizeof(v_loop_histories.ErrorHistory[0])); // Error history array size

/* *********************************************************************************
 * Pole&Zero Placement:
 * *********************************************************************************
 *
 *    fP0:    1334 Hz
 *    fP1:    104500 Hz
 *    fP2:    250000 Hz
 *    fZ1:    3020 Hz
 *    fZ2:    5033 Hz
 *
 * *********************************************************************************
 * Filter Coefficients and Parameters:
 * ********************************************************************************/
volatile int32_t v_loop_ACoefficients [3] =
{
    0x7E1E0000, // Coefficient A1 will be multiplied with controller output u(n-1)
    0x7C700004, // Coefficient A2 will be multiplied with controller output u(n-2)
    0xA1BD0004  // Coefficient A3 will be multiplied with controller output u(n-3)
};

volatile int32_t v_loop_BCoefficients [4] =
{
    0x71CBFFFD, // Coefficient B0 will be multiplied with error input e(n-0)
    0x996DFFFD, // Coefficient B1 will be multiplied with error input e(n-1)
    0x8E78FFFD, // Coefficient B2 will be multiplied with error input e(n-2)
    0x66D7FFFD  // Coefficient B3 will be multiplied with error input e(n-3)
};

// Coefficient normalization factors
volatile int16_t v_loop_pre_scaler = 3;
volatile int16_t v_loop_post_shift_A = 0;
volatile int16_t v_loop_post_shift_B = 0;
volatile fractional v_loop_post_scaler = 0x0000;

volatile cNPNZ16b_t v_loop; // user-controller data object

/* ********************************************************************************/

/*!v_loop_Init()
 * *********************************************************************************
 * Summary: Initializes controller coefficient arrays and normalization
 *
 * Parameters:
 *     - cNPNZ16b_t* controller
 *
 * Returns:
 *     - uint16_t:  0->failure
 *                  1->success

 * Description:
 * This function needs to be called from user code once to initialize coefficient
 * arrays and number normalization settings of the v_loop controller
 * object.
 *
 * PLEASE NOTE:
 * This routine DOES NOT initialize the complete controller object.
 * User-defined settings such as pointers to the control reference, source and
 * target registers, output minima and maxima and further, design-dependent
 * settings, need to be specified in user code.
 * ********************************************************************************/
volatile uint16_t v_loop_Initialize(volatile cNPNZ16b_t* controller)
{
    volatile uint16_t i=0;

    // Initialize controller data structure at runtime with pre-defined default values
    controller->status.value = CONTROLLER_STATUS_CLEAR;  // clear all status flag bits (will turn off execution))
    
    controller->Filter.ptrACoefficients = &v_loop_coefficients.ACoefficients[0]; // initialize pointer to A-coefficients array
    controller->Filter.ptrBCoefficients = &v_loop_coefficients.BCoefficients[0]; // initialize pointer to B-coefficients array
    controller->Filter.ptrControlHistory = &v_loop_histories.ControlHistory[0]; // initialize pointer to control history array
    controller->Filter.ptrErrorHistory = &v_loop_histories.ErrorHistory[0]; // initialize pointer to error history array
    controller->Filter.normPostShiftA = v_loop_post_shift_A; // initialize A-coefficients/single bit-shift scaler
    controller->Filter.normPostShiftB = v_loop_post_shift_B; // initialize B-coefficients/dual/post scale factor bit-shift scaler
    controller->Filter.normPostScaler = v_loop_post_scaler; // initialize control output value normalization scaling factor
    controller->Filter.normPreShift = v_loop_pre_scaler; // initialize A-coefficients/single bit-shift scaler
    
    controller->Filter.ACoefficientsArraySize = v_loop_ACoefficients_size; // initialize A-coefficients array size
    controller->Filter.BCoefficientsArraySize = v_loop_BCoefficients_size; // initialize A-coefficients array size
    controller->Filter.ControlHistoryArraySize = v_loop_ControlHistory_size; // initialize control history array size
    controller->Filter.ErrorHistoryArraySize = v_loop_ErrorHistory_size; // initialize error history array size
    
    
    // Load default set of A-coefficients from user RAM into X-Space controller A-array
    for(i=0; i<controller->Filter.ACoefficientsArraySize; i++)
    {
        v_loop_coefficients.ACoefficients[i] = v_loop_ACoefficients[i];
    }

    // Load default set of B-coefficients from user RAM into X-Space controller B-array
    for(i=0; i<controller->Filter.BCoefficientsArraySize; i++)
    {
        v_loop_coefficients.BCoefficients[i] = v_loop_BCoefficients[i];
    }

    // Clear error and control histories of the 3P3Z controller
    v_loop_Reset(&v_loop);
    
    return(1);
}


//**********************************************************************************
// Download latest version of this tool here: https://areiter128.github.io/DCLD
//**********************************************************************************

