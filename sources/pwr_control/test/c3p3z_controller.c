/* **********************************************************************************
 * z-Domain Control Loop Designer, Version 0.9.3.89
 * **********************************************************************************
 * 3p3z compensation filter coefficients derived for following operating
 * conditions:
 * **********************************************************************************
 *
 *  Controller Type:    3P3Z - Basic Voltage Mode Compensator
 *  Sampling Frequency: 250000 Hz
 *  Fixed Point Format: 15
 *  Scaling Mode:       1 - Single Bit-Shift Scaling
 *  Input Gain:         1
 *
 * *********************************************************************************
 * CGS Version:         1.1.1
 * CGS Date:            03/19/2020
 * *********************************************************************************
 * User:                M91406
 * Date/Time:           03/24/2020 1:03:38 AM
 * ********************************************************************************/

#include "./pwr_control/test/c3p3z_controller.h"

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
 * through defines in source file c3p3z_controller.c
 * ********************************************************************************/

volatile C3P3Z_CONTROL_LOOP_COEFFICIENTS_t __attribute__((space(xmemory), near)) c3p3z_coefficients; // A/B-Coefficients
volatile uint16_t c3p3z_ACoefficients_size = (sizeof(c3p3z_coefficients.ACoefficients)/sizeof(c3p3z_coefficients.ACoefficients[0])); // A-coefficient array size
volatile uint16_t c3p3z_BCoefficients_size = (sizeof(c3p3z_coefficients.BCoefficients)/sizeof(c3p3z_coefficients.BCoefficients[0])); // B-coefficient array size

volatile C3P3Z_CONTROL_LOOP_HISTORIES_t __attribute__((space(ymemory), far)) c3p3z_histories; // Control/Error Histories
volatile uint16_t c3p3z_ControlHistory_size = (sizeof(c3p3z_histories.ControlHistory)/sizeof(c3p3z_histories.ControlHistory[0])); // Control history array size
volatile uint16_t c3p3z_ErrorHistory_size = (sizeof(c3p3z_histories.ErrorHistory)/sizeof(c3p3z_histories.ErrorHistory[0])); // Error history array size

/* *********************************************************************************
 * Pole&Zero Placement:
 * *********************************************************************************
 *
 *    fP0:    650 Hz
 *    fP1:    30000 Hz
 *    fP2:    125000 Hz
 *    fZ1:    3000 Hz
 *    fZ2:    6000 Hz
 *
 * *********************************************************************************
 * Filter Coefficients and Parameters:
 * ********************************************************************************/
volatile fractional c3p3z_ACoefficients [3] =
{
    0x4EC0, // Coefficient A1 will be multiplied with controller output u(n-1)
    0xF7AF, // Coefficient A2 will be multiplied with controller output u(n-2)
    0xF993  // Coefficient A3 will be multiplied with controller output u(n-3)
};

volatile fractional c3p3z_BCoefficients [4] =
{
    0x2256, // Coefficient B0 will be multiplied with error input e(n-0)
    0xE4FA, // Coefficient B1 will be multiplied with error input e(n-1)
    0xDE05, // Coefficient B2 will be multiplied with error input e(n-2)
    0x1B60  // Coefficient B3 will be multiplied with error input e(n-3)
};

// Coefficient normalization factors
volatile int16_t c3p3z_pre_scaler = 3;
volatile int16_t c3p3z_post_shift_A = -1;
volatile int16_t c3p3z_post_shift_B = 0;
volatile fractional c3p3z_post_scaler = 0x0000;

volatile cNPNZ16b_t c3p3z_controller; // user-controller data object

/* ********************************************************************************/

/*!c3p3z_controller_Init()
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
 * arrays and number normalization settings of the c3p3z_controller controller
 * object.
 *
 * PLEASE NOTE:
 * This routine DOES NOT initialize the complete controller object.
 * User-defined settings such as pointers to the control reference, source and
 * target registers, output minima and maxima and further, design-dependent
 * settings, need to be specified in user code.
 * ********************************************************************************/
volatile uint16_t c3p3z_controller_Initialize(volatile cNPNZ16b_t* controller)
{
    volatile uint16_t i=0;

    // Initialize controller data structure at runtime with pre-defined default values
    controller->status.value = CONTROLLER_STATUS_CLEAR;  // clear all status flag bits (will turn off execution))
    
    controller->ptrACoefficients = &c3p3z_coefficients.ACoefficients[0]; // initialize pointer to A-coefficients array
    controller->ptrBCoefficients = &c3p3z_coefficients.BCoefficients[0]; // initialize pointer to B-coefficients array
    controller->ptrControlHistory = &c3p3z_histories.ControlHistory[0]; // initialize pointer to control history array
    controller->ptrErrorHistory = &c3p3z_histories.ErrorHistory[0]; // initialize pointer to error history array
    controller->normPostShiftA = c3p3z_post_shift_A; // initialize A-coefficients/single bit-shift scaler
    controller->normPostShiftB = c3p3z_post_shift_B; // initialize B-coefficients/dual/post scale factor bit-shift scaler
    controller->normPostScaler = c3p3z_post_scaler; // initialize control output value normalization scaling factor
    controller->normPreShift = c3p3z_pre_scaler; // initialize A-coefficients/single bit-shift scaler
    
    controller->ACoefficientsArraySize = c3p3z_ACoefficients_size; // initialize A-coefficients array size
    controller->BCoefficientsArraySize = c3p3z_BCoefficients_size; // initialize A-coefficients array size
    controller->ControlHistoryArraySize = c3p3z_ControlHistory_size; // initialize control history array size
    controller->ErrorHistoryArraySize = c3p3z_ErrorHistory_size; // initialize error history array size
    
    
    // Load default set of A-coefficients from user RAM into X-Space controller A-array
    for(i=0; i<controller->ACoefficientsArraySize; i++)
    {
        c3p3z_coefficients.ACoefficients[i] = c3p3z_ACoefficients[i];
    }

    // Load default set of B-coefficients from user RAM into X-Space controller B-array
    for(i=0; i<controller->BCoefficientsArraySize; i++)
    {
        c3p3z_coefficients.BCoefficients[i] = c3p3z_BCoefficients[i];
    }

    // Clear error and control histories of the 3P3Z controller
    c3p3z_controller_Reset(&c3p3z_controller);
    
    return(1);
}
 
//**********************************************************************************
// Download latest version of this tool here: https://areiter128.github.io/DCLD
//**********************************************************************************
 
