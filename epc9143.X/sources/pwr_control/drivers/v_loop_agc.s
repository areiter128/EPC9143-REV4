;LICENSE / DISCLAIMER
; **********************************************************************************
;  SDK Version: z-Domain Control Loop Designer v0.9.7.99
;  AGS Version: Assembly Generator Script v2.0.8 (04/21/2020)
;  Author:      M91406
;  Date/Time:   04/24/2020 3:26:49 PM
; **********************************************************************************
;  3P3Z Control Library File (Fast Floating Point Coefficient Scaling Mode)
; **********************************************************************************
	
;------------------------------------------------------------------------------
;file start
	.nolist
	.list
	
;------------------------------------------------------------------------------
;local inclusions.
	.section .data    ; place constant data in the data section
	
;------------------------------------------------------------------------------
; Define status flags bit positions
	.equ NPNZ16_STATUS_ENABLED,      15    ; bit position of the ENABLE control bit
	.equ NPNZ16_STATUS_INVERT_INPUT, 14    ; bit position of the INVERT_INPUT control bit
	.equ NPNZ16_STATUS_SWAP_SOURCE,  13    ; bit position of the SWAP_SOURCE control bit
	.equ NPNZ16_STATUS_SWAP_TARGET,  12    ; bit position of the SWAP_TARGET control bit
	.equ NPNZ16_STATUS_AGC_ENABLED,  11    ; bit position of the AGC_ENABLED control bit
	.equ NPNZ16_STATUS_USAT,         1    ; bit position of the UPPER_SATURATION_FLAG status bit
	.equ NPNZ16_STATUS_LSAT,         0    ; bit position of the LOWER_SATURATION_FLAG status bit
	
;------------------------------------------------------------------------------
; Address offset declarations for data structure addressing (double bit-shift scaling)
	.equ Status,                    0    ; controller object status word at address-offset = 0
	.equ ptrSourceRegister,         2    ; parameter group Ports.Source: pointer to source memory address
	.equ SourceNormShift,           4    ; parameter group Ports.Source: bit-shift scaler of normalization factor
	.equ SourceNormFactor,          6    ; parameter group Ports.Source: Q15 normalization factor
	.equ SourceOffset,              8    ; parameter group Ports.Source: value of source input signal/value offset
	.equ ptrAltSourceRegister,      10    ; parameter group Ports.AltSource: pointer to alternate source memory address
	.equ AltSourceNormShift,        12    ; parameter group Ports.AltSource: bit-shift scaler of normalization factor
	.equ AltSourceNormFactor,       14    ; parameter group Ports.AltSource: Q15 normalization factor
	.equ AltSourceOffset,           16    ; parameter group Ports.AltSource: value of alternate source input signal/value offset
	.equ ptrTargetRegister,         18    ; parameter group Ports.Target: pointer to target memory address
	.equ TargetNormShift,           20    ; parameter group Ports.Target: bit-shift scaler of normalization factor
	.equ TargetNormFactor,          22    ; parameter group Ports.Target: Q15 normalization factor
	.equ TargetOffset,              24    ; parameter group Ports.Target: value of target output signal/value offset
	.equ ptrAltTargetRegister,      26    ; parameter group Ports.AltTarget: pointer to alternate target memory address
	.equ AltTargetNormShift,        28    ; parameter group Ports.AltTarget: bit-shift scaler of normalization factor
	.equ AltTargetNormFactor,       30    ; parameter group Ports.AltTarget: Q15 normalization factor
	.equ AltTargetOffset,           32    ; parameter group Ports.AltTarget: value of alternate target output sigal/value offset
	.equ ptrCtrlReference,          34    ; parameter group Ports.ConrolReference: pointer to control reference variable/register memory address
	.equ ptrACoefficients,          36    ; parameter group Filter: pointer to A-coefficients array start address
	.equ ptrBCoefficients,          38    ; parameter group Filter: pointer to B-coefficients array start address
	.equ ptrControlHistory,         40    ; parameter group Filter: pointer to control history array start address
	.equ ptrErrorHistory,           42    ; parameter group Filter: pointer to error history array start address
	.equ ACoeffArraySize,           44    ; parameter group Filter: size of the A-coefficients array
	.equ BCoeffArraySize,           46    ; parameter group Filter: size of the B-coefficients array
	.equ CtrlHistArraySize,         48    ; parameter group Filter: size of the control history array
	.equ ErrHistArraySize,          50    ; parameter group Filter: size of the error history array
	.equ PreShift,                  52    ; parameter group Filter: value of input value normalization bit-shift scaler
	.equ PostShiftA,                54    ; parameter group Filter: value of A-term normalization bit-shift scaler
	.equ PostShiftB,                56    ; parameter group Filter: value of B-term normalization bit-shift scaler
	.equ reserved_2,                58    ; parameter group Filter: (reserved)
	.equ pterm_scaler,              60    ; parameter group Filter: P-Term coefficient scaler
	.equ pterm_factor,              62    ; parameter group Filter: P-Term coefficient fractional factor
	.equ MinOutput,                 64    ; parameter group Limits: minimum clamping value of primary control output
	.equ MaxOutput,                 66    ; parameter group Limits: maximum clamping value of primary control output
	.equ AltMinOutput,              68    ; parameter group Limits: minimum clamping value of alternate control output
	.equ AltMaxOutput,              70    ; parameter group Limits: maximum clamping value of alternate control output
	.equ ptrADCTriggerARegister,    72    ; parameter group ADCTriggerControl: pointer to ADC trigger A register memory address
	.equ ADCTriggerAOffset,         74    ; parameter group ADCTriggerControl: value of ADC trigger A offset
	.equ ptrADCTriggerBRegister,    76    ; parameter group ADCTriggerControl: pointer to ADC trigger B register memory address
	.equ ADCTriggerBOffset,         78    ; parameter group ADCTriggerControl: value of ADC trigger B offset
	.equ ptrDProvControlInput,      80    ; parameter group DataProviders: pointer to external variable/register the most recent control input will be pushed to
	.equ ptrDProvControlError,      82    ; parameter group DataProviders: pointer to external variable/register the most recent control error will be pushed to
	.equ ptrDProvControlOutput,     84    ; parameter group DataProviders: pointer to external variable/register the most recent control output will be pushed to
	.equ ptrCascadedFunction,       86    ; parameter group CascadeTrigger: pointer to external, cascaded function which will be called by this controller
	.equ CascadedFunParam,          88    ; parameter group CascadeTrigger: 16-bit wide function parameter or pointer to a parameter data structure of cascaded function
	.equ agcGainModScaler,          90    ; parameter group GainControl: bit-shift scaler of Adaptive Gain Control Modulation factor
	.equ agcGainModFactor,          92    ; parameter group GainControl: Q15 value of Adaptive Gain Control Modulation factor
	.equ agcGainModMedian,          94    ; parameter group GainControl: Q15 value of Adaptive Gain Control Modulation norminal operating point
	.equ ptrAgcObserverFunction,    96    ; parameter group GainControl: function pointer to observer function updating the AGC modulation factor
	.equ AdvParam1,                 98    ; parameter group Advanced: generic 16-bit wide, user-defined parameter #1 for advanced control options
	.equ AdvParam2,                 100    ; parameter group Advanced: generic 16-bit wide, user-defined parameter #2 for advanced control options
	.equ AdvParam3,                 102    ; parameter group Advanced: generic 16-bit wide, user-defined parameter #3 for advanced control options
	.equ AdvParam4,                 104    ; parameter group Advanced: generic 16-bit wide, user-defined parameter #4 for advanced control options
	
;------------------------------------------------------------------------------
;local inclusions.
	.section .text    ; place code in the code section
	
;------------------------------------------------------------------------------
; Global function declaration
; This function calls the z-domain controller processing the latest data point input
;------------------------------------------------------------------------------
	
	.global _v_loop_GetAGCFactor
_v_loop_GetAGCFactor:
    
    nop ; (debugging break point anchor)

    return
    
    ; determine most recent VL
    
    ; read and normalize input voltage 
    mov [w0 + #ptrAltSourceRegister], w1    ; load pointer to most recent input voltage register
    mov [w1], w1                            ; load value into w1
    mov [w0 + AltSourceNormShift], w3       ; load most recent input voltage normalization scaler
    sl  w1, w3, w1                          ; normalize input voltage value
    
    ; read and normalize output voltage 
    mov [w0 + #ptrSourceRegister], w2       ; load pointer to most recent output voltage register
    mov [w2], w2                            ; load value into w2
    mov [w0 + SourceNormShift], w3          ; load most recent input voltage normalization scaler
    sl  w2, w3, w2                          ; normalize output voltage value
    
    ; Calculate instantaneous VL
    sub w1, w2, w6                          ; calculate most recent VL, place result in w6
    
    ; Load modulation median
    mov [w0 + #agcGainModMedian], w4        ; load pointer to nominal VL
    
    ; Divide median by instatneous VL
    push.s      ; Save pointer to cNPNZ16b_t data structure
    repeat #5   ; run divide in 6 steps
    divf w4, w6 ; divide VL_nom/VL
    mov w0, w4  ; move result to w4
    pop.s       ; restore pointer to cNPNZ16b_t data structure
    mov w4, [w0 + #agcGainModFactor] ; load result into cNPNZ16b_t data structure
    
;------------------------------------------------------------------------------
; End of routine
	return
;------------------------------------------------------------------------------
    
;------------------------------------------------------------------------------
; End of file
	.end
;------------------------------------------------------------------------------


