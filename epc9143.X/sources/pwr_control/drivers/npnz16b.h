/* *********************************************************************************
 * z-Domain Control Loop Designer, Version 0.9.7.102
 * *********************************************************************************
 * Generic library header for z-domain compensation filter assembly functions
 * CGS Version: 2.0.9
 * CGS Date:    05/20/2020
 * ********************************************************************************/
#ifndef __SPECIAL_FUNCTION_LAYER_LIB_NPNZ_H__
#define __SPECIAL_FUNCTION_LAYER_LIB_NPNZ_H__

#include <xc.h> // include processor files - each processor file is guarded
#include <dsp.h> // include DSP data types (e.g. fractional)
#include <stdint.h> // include standard integer number data types
#include <stdbool.h> // include standard boolean data types (true/false)

// Generic macro allowing to identify the file version of  'npnz16b.h'
// This version key represents the product version of DCLD as integer number
// of the form [MAJOR][MINOR][REVISION] => version 0.9.3.xxx would be shown as 903
#ifndef __DCLD_VERSION
    #define __DCLD_VERSION    907
#endif  // end of __DCLD_VERSION

/* Status flags (Single Bit) */
#define NPNZ16_STATUS_LSAT_SET             1
#define NPNZ16_STATUS_LSAT_CLEAR           0
#define NPNZ16_STATUS_USAT_SET             1
#define NPNZ16_STATUS_USAT_CLEAR           0
#define NPNZ16_STATUS_AGC_ENABLED          1
#define NPNZ16_STATUS_AGC_DISABLED         0
#define NPNZ16_STATUS_TARGET_SWAPPED       1
#define NPNZ16_STATUS_TARGET_NOT_SWAPPED   0
#define NPNZ16_STATUS_SOURCE_SWAPPED       1
#define NPNZ16_STATUS_SOURCE_NOT_SWAPPED   0
#define NPNZ16_STATUS_INPUT_INVERTED       1
#define NPNZ16_STATUS_INPUT_NOT_INVERTED   0
#define NPNZ16_STATUS_ENABLED              1
#define NPNZ16_STATUS_DISABLED             0

/* Status flags (bit-field) */
typedef enum {
    CONTROLLER_STATUS_CLEAR           = 0b0000000000000000,
    CONTROLLER_STATUS_SATUATION_MSK   = 0b0000000000000011,
    CONTROLLER_STATUS_LSAT_ACTIVE     = 0b0000000000000001,
    CONTROLLER_STATUS_LSAT_CLEAR      = 0b0000000000000000,
    CONTROLLER_STATUS_USAT_ACTIVE     = 0b0000000000000010,
    CONTROLLER_STATUS_USAT_CLEAR      = 0b0000000000000000,
    CONTROLLER_STATUS_AGC_DISABLE     = 0b0000000000000000,
    CONTROLLER_STATUS_AGC_ENABLED     = 0b0000100000000000,
    CONTROLLER_STATUS_TARGET_DEFAULT  = 0b0000000000000000,
    CONTROLLER_STATUS_TARGET_SWAPED   = 0b0001000000000000,
    CONTROLLER_STATUS_SOURCE_DEFAULT  = 0b0000000000000000,
    CONTROLLER_STATUS_SOURCE_SWAPED   = 0b0010000000000000,
    CONTROLLER_STATUS_INV_INPUT_OFF   = 0b0000000000000000,
    CONTROLLER_STATUS_INV_INPUT_ON    = 0b0100000000000000,
    CONTROLLER_STATUS_ENABLE_OFF      = 0b0000000000000000,
    CONTROLLER_STATUS_ENABLE_ON       = 0b1000000000000000
} CONTROLLER_STATUS_FLAGS_e;

typedef union {
    struct {
        volatile unsigned flt_clamp_min : 1; // Bit 0: control loop is clamped at minimum output level
        volatile unsigned flt_clamp_max : 1; // Bit 1: control loop is clamped at maximum output level
        volatile unsigned : 1; // Bit 2: reserved
        volatile unsigned : 1; // Bit 3: reserved
        volatile unsigned : 1; // Bit 4: reserved
        volatile unsigned : 1; // Bit 5: reserved
        volatile unsigned : 1; // Bit 6: reserved
        volatile unsigned : 1; // Bit 7: reserved
        volatile unsigned : 1; // Bit 8: reserved
        volatile unsigned : 1; // Bit 9: reserved
        volatile unsigned : 1; // Bit 11: reserved
        volatile unsigned agc_enabled: 1; // Bit 11: when set, Adaptive Gain Control Modulation is enabled
        volatile unsigned swap_target: 1; // Bit 12: when set, AltTarget is used as data output of controller
        volatile unsigned swap_source: 1; // Bit 13: when set, AltSource is used as data input to controller
        volatile unsigned invert_input: 1; // Bit 14: when set, most recent error input value to controller is inverted
        volatile unsigned enabled : 1; // Bit 15: enables/disables control loop execution
    } __attribute__((packed))bits;    // Controller status bit-field for direct bit access
    volatile uint16_t value;          // Controller status full register access
} __attribute__((packed))CONTROLLER_STATUS_t; // Controller status data structure

typedef struct {
    volatile uint16_t* ptrAddress; // Pointer to register or variable where the value is read from (e.g. ADCBUFx) or written to (e.g. PGxDC)
    volatile uint16_t  NormScaler; // Bit-shift scaler of the Q15 normalization factor
    volatile fractional NormFactor; // Q15 normalization factor
    volatile uint16_t  Offset; // Value/signal offset of this port
} __attribute__((packed))CONTROLLER_PORT_t;

typedef struct {
    // External control and monitoring
    volatile CONTROLLER_STATUS_t status; // Control Loop Status and Control flags

    // Input/Output to controller
    struct {
        volatile CONTROLLER_PORT_t Source; // Primary data input port declaration
        volatile CONTROLLER_PORT_t AltSource; // Secondary data input port declaration
        volatile CONTROLLER_PORT_t Target; // Primary data output port declaration
        volatile CONTROLLER_PORT_t AltTarget; // Secondary data output port declaration
        volatile uint16_t* ptrControlReference; // Pointer to global variable of input register holding the controller reference value (e.g. uint16_t my_ref)
    } __attribute__((packed))Ports; // Controller block input and output port definitions

    // Filter coefficients and input/output histories
    struct {
        volatile int32_t* ptrACoefficients; // Pointer to A coefficients located in X-space
        volatile int32_t* ptrBCoefficients; // Pointer to B coefficients located in X-space
        volatile fractional* ptrControlHistory; // Pointer to n delay-line samples located in Y-space with first sample being the most recent
        volatile fractional* ptrErrorHistory; // Pointer to n+1 delay-line samples located in Y-space with first sample being the most recent

        // Array size information
        volatile uint16_t ACoefficientsArraySize; // Size of the A coefficients array in X-space
        volatile uint16_t BCoefficientsArraySize; // Size of the B coefficients array in X-space
        volatile uint16_t ControlHistoryArraySize; // Size of the control history array in Y-space
        volatile uint16_t ErrorHistoryArraySize; // Size of the error history array in Y-space

        // Feedback scaling Input/Output Normalization
        volatile int16_t normPreShift; // Normalization of ADC-resolution to Q15 (R/W)
        volatile int16_t normPostShiftA; // Normalization of A-term control output to Q15 (R/W)
        volatile int16_t normPostShiftB; // Normalization of B-term control output to Q15 (R/W)
        volatile int16_t normPostScaler; // Control output normalization factor (Q15) (R/W)

        // P-Term Coefficients (for plant measurements only)
        volatile int16_t PTermScaler; // Q15 P-Term Coefficient Bit-Shift Scaler (R/W)
        volatile int16_t PTermFactor; // Q15 P-Term Coefficient Factor (R/W)
    } __attribute__((packed))Filter; // Filter parameters such as pointer to history and coefficient arrays and number scaling

    // System clamping/Anti-windup
    struct {
        volatile int16_t MinOutput; // Minimum output value used for clamping (R/W)
        volatile int16_t MaxOutput; // Maximum output value used for clamping (R/W)
        volatile int16_t AltMinOutput; // Alternate minimum output value used for clamping (R/W)
        volatile int16_t AltMaxOutput; // Alternate maximum output value used for clamping (R/W)
    } __attribute__((packed))Limits; // Input and output clamping values

    // Voltage/Average Current Mode Control Trigger handling
    struct {
        volatile uint16_t* ptrADCTriggerARegister; // Pointer to ADC trigger #1 register (e.g. TRIG1)
        volatile uint16_t ADCTriggerAOffset; // ADC trigger #1 offset to compensate propagation delays
        volatile uint16_t* ptrADCTriggerBRegister; // Pointer to ADC trigger #2 register (e.g. TRIG2)
        volatile uint16_t ADCTriggerBOffset; // ADC trigger #2 offset to compensate propagation delays
    } __attribute__((packed))ADCTriggerControl; // Automatic ADC trigger placement options for ADC Trigger A and B

    // Data Provider Sources
    struct {
        volatile uint16_t* ptrDProvControlInput; // Pointer to external data buffer of most recent control input
        volatile uint16_t* ptrDProvControlError; // Pointer to external data buffer of most recent control error
        volatile uint16_t* ptrDProvControlOutput; // Pointer to external data buffer of most recent control output
    } __attribute__((packed))DataProviders; // Automated data sources pushing data points to user-defined variables

    // Cascaded Function Call Parameters
    struct {
        volatile uint16_t ptrCascadedFunction; // Pointer to Function which should be called at the end of the control loop
        volatile uint16_t CascadedFunParam; // Parameter of function called (can be a pointer to a data structure)
    } __attribute__((packed))CascadeTrigger; // Cascade triggers with parameters for next function call

    // Adaptive Gain Control Modulation
    struct {
        volatile uint16_t AgcScaler; // Bit-shift scaler of Adaptive Gain Modulation factor
        volatile fractional AgcFactor; // Q15 value of Adaptive Gain Modulation factor
        volatile fractional AgcMedian; // Q15 value of Adaptive Gain Modulation nominal operating point
        volatile uint16_t ptrAgcObserverFunction; // Function Pointer to Observer function updating the AGC modulation factor
    } __attribute__((packed))GainControl; // Parameter section for advanced control options

    // User Data Space for Advanced Control Functions
    struct {
        volatile uint16_t advParam1; // generic 16-bit wide, user-defined parameter #1 for advanced control options
        volatile uint16_t advParam2; // generic 16-bit wide, user-defined parameter #2 for advanced control options
        volatile uint16_t advParam3; // generic 16-bit wide, user-defined parameter #3 for advanced control options
        volatile uint16_t advParam4; // generic 16-bit wide, user-defined parameter #4 for advanced control options
    } __attribute__((packed))Advanced; // Parameter section for advanced control options

} __attribute__((packed))cNPNZ16b_t; // Generic nPnZ Controller Object with 16-bit fixed point coefficients, data input and data output

/* ********************************************************************************/
#endif  // end of __SPECIAL_FUNCTION_LAYER_LIB_NPNZ_H__ header file section


//**********************************************************************************
// Download latest version of this tool here: https://areiter128.github.io/DCLD
//**********************************************************************************

