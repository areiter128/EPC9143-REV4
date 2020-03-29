;LICENSE / DISCLAIMER
; **********************************************************************************
;  SDK Version: z-Domain Control Loop Designer v0.9.5.95
;  AGS Version: Assembly Generator Script v2.0.2 (03/30/2020)
;  Author:      M91406
;  Date/Time:   03/30/2020 1:37:21 AM
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
; Address offset declarations for data structure addressing
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
	.equ reserved_0,                54    ; parameter group Filter: value of A-term normalization bit-shift scaler
	.equ reserved_1,                56    ; parameter group Filter: (reserved)
	.equ reserved_2,                58    ; parameter group Filter: (reserved)
	.equ MinOutput,                 60    ; parameter group Limits: minimum clamping value of primary control output
	.equ MaxOutput,                 62    ; parameter group Limits: maximum clamping value of primary control output
	.equ AltMinOutput,              64    ; parameter group Limits: minimum clamping value of alternate control output
	.equ AltMaxOutput,              66    ; parameter group Limits: maximum clamping value of alternate control output
	.equ ptrADCTriggerARegister,    68    ; parameter group ADCTriggerControl: pointer to ADC trigger A register memory address
	.equ ADCTriggerAOffset,         70    ; parameter group ADCTriggerControl: value of ADC trigger A offset
	.equ ptrADCTriggerBRegister,    72    ; parameter group ADCTriggerControl: pointer to ADC trigger B register memory address
	.equ ADCTriggerBOffset,         74    ; parameter group ADCTriggerControl: value of ADC trigger B offset
	.equ ptrDProvControlInput,      76    ; parameter group DataProviders: pointer to external variable/register the most recent control input will be pushed to
	.equ ptrDProvControlError,      78    ; parameter group DataProviders: pointer to external variable/register the most recent control error will be pushed to
	.equ ptrDProvControlOutput,     80    ; parameter group DataProviders: pointer to external variable/register the most recent control output will be pushed to
	.equ ptrCascadedFunction,       82    ; parameter group CascadeTrigger: pointer to external, cascaded function which will be called by this controller
	.equ CascadedFunParam,          84    ; parameter group CascadeTrigger: 16-bit wide function parameter or pointer to a parameter data structure of cascaded function
	.equ agcGainModScaler,          86    ; parameter group GainControl: bit-shift scaler of Adaptive Gain Control Modulation factor
	.equ agcGainModFactor,          88    ; parameter group GainControl: Q15 value of Adaptive Gain Control Modulation factor
	.equ agcGainModMedian,          90    ; parameter group GainControl: Q15 value of Adaptive Gain Control Modulation norminal operating point
	.equ AdvParam1,                 92    ; parameter group Advanced: generic 16-bit wide, user-defined parameter #1 for advanced control options
	.equ AdvParam2,                 94    ; parameter group Advanced: generic 16-bit wide, user-defined parameter #2 for advanced control options
	.equ AdvParam3,                 96    ; parameter group Advanced: generic 16-bit wide, user-defined parameter #3 for advanced control options
	.equ AdvParam4,                 98    ; parameter group Advanced: generic 16-bit wide, user-defined parameter #4 for advanced control options
	
;------------------------------------------------------------------------------
;local inclusions.
	.section .text    ; place code in the code section
	
;------------------------------------------------------------------------------
; Global function declaration
; This function calls the z-domain controller processing the latest data point input
;------------------------------------------------------------------------------
	
	.global _v_loop_Update
_v_loop_Update:    ; provide global scope to routine
	push w12    ; save working register used for status flag tracking
	
;------------------------------------------------------------------------------
; Check status word for Enable/Disable flag and bypass computation, if disabled
	mov [w0 + #Status], w12    ; load value of status word into working register
	btss w12, #NPNZ16_STATUS_ENABLED    ; check ENABLED bit state, skip (do not execute) next instruction if set
	bra V_LOOP_BYPASS_LOOP    ; if ENABLED bit is cleared, jump to end of control code
	
;------------------------------------------------------------------------------
; Configure DSP for fractional operation with normal saturation (Q1.31 format)
	mov #0x00E4, w4    ; load default value of DSP core configuration enabling saturation and signed fractional multiply
	mov w4, _CORCON    ; load default configuration into CORCON register
	
;------------------------------------------------------------------------------
; Setup pointers to A-Term data arrays
	mov [w0 + #ptrACoefficients], w8    ; load pointer to first index of A coefficients array
	
;------------------------------------------------------------------------------
; Load pointer to first element of control history array
	mov [w0 + #ptrControlHistory], w10    ; load pointer address into wreg
	
;------------------------------------------------------------------------------
; Compute compensation filter term
	clr b, [w8]+=2, w5    ; clear both accumulators and prefetch first operands
	clr a, [w8]+=4, w4, [w10]+=2, w6
	mpy w4*w6, a, [w8]+=4, w4, [w10]+=2, w6    ; multiply control output (n-%INDEX%) from the delay line with coefficient X%INDEX%
	sftac a, w5    ; shift accumulator to post-scale floating number
	add b    ; adding accumulator b to a
	mov [w8 - #6], w5    ; load scaler into wreg
	mpy w4*w6, a, [w8]+=4, w4, [w10]+=2, w6    ; multiply control output (n-1) from the delay line with coefficient X1
	sftac a, w5    ; shift accumulator to post-scale floating number
	add b    ; adding accumulator b to a
	mov [w8 - #6], w5    ; load scaler into wreg
	mpy w4*w6, a    ; multiply & accumulate last control output with coefficient of the delay line (no more prefetch)
	sftac a, w5    ; shift accumulator to post-scale floating number
	add b    ; adding accumulator b to a
	
;------------------------------------------------------------------------------
; Read data from input source and calculate error input to transfer function
	mov [w0 + #ptrSourceRegister], w2    ; load pointer to input source register
	mov [w2], w1    ; move value from input source into working register
	mov [w0 + #ptrDProvControlInput], w2    ; load pointer address of target buffer of most recent controller input from data structure
	mov w1, [w2]    ; copy most recent controller input value to given data buffer target
	mov [w0 + #ptrCtrlReference], w2    ; move pointer to control reference into working register
	subr w1, [w2], w1    ; calculate error (=reference - input)
	mov [w0 + #PreShift], w2    ; move error input scaler into working register
	sl w1, w2, w1    ; normalize error result to fractional number format
	
;------------------------------------------------------------------------------
; Setup pointers to B-Term data arrays
	mov [w0 + #ptrBCoefficients], w8    ; load pointer to first index of B coefficients array
	
;------------------------------------------------------------------------------
; Setup pointer to first element of error history array
	mov [w0 + #ptrErrorHistory], w10    ; load pointer address into wreg
	
;------------------------------------------------------------------------------
; Update error history (move error one tick along the delay line)
	mov [w10 + #4], w6    ; move entry (n-3) into buffer
	mov w6, [w10 + #6]    ; move buffered value one tick down the delay line
	mov [w10 + #2], w6    ; move entry (n-2) into buffer
	mov w6, [w10 + #4]    ; move buffered value one tick down the delay line
	mov [w10 + #0], w6    ; move entry (n-1) into buffer
	mov w6, [w10 + #2]    ; move buffered value one tick down the delay line
	mov w1, [w10]    ; add most recent error input to history array
	
;------------------------------------------------------------------------------
; Compute compensation filter term
	movsac b, [w8]+=2, w5    ; leave contents of accumulator B unchanged
	clr a, [w8]+=4, w4, [w10]+=2, w6    ; clear accumulator A and prefetch first operands
	mpy w4*w6, a, [w8]+=4, w4, [w10]+=2, w6    ; multiply control output (n-%INDEX%) from the delay line with coefficient X%INDEX%
	sftac a, w5    ; shift accumulator to post-scale floating number
	add b    ; adding accumulator b to a
	mov [w8 - #6], w5    ; load scaler into wreg
	mpy w4*w6, a, [w8]+=4, w4, [w10]+=2, w6    ; multiply control output (n-0) from the delay line with coefficient X0
	sftac a, w5    ; shift accumulator to post-scale floating number
	add b    ; adding accumulator b to a
	mov [w8 - #6], w5    ; load scaler into wreg
	mpy w4*w6, a, [w8]+=4, w4, [w10]+=2, w6    ; multiply control output (n-1) from the delay line with coefficient X1
	sftac a, w5    ; shift accumulator to post-scale floating number
	add b    ; adding accumulator b to a
	mov [w8 - #6], w5    ; load scaler into wreg
	mpy w4*w6, a    ; multiply & accumulate last control output with coefficient of the delay line (no more prefetch)
	sftac a, w5    ; shift accumulator to post-scale floating number
	add b    ; adding accumulator b to a
	; Backwards normalization of the controller output
	sac.r b, w4    ; store most recent accumulator result in working register
	
;------------------------------------------------------------------------------
; Controller Anti-Windup (control output value clamping)
	; Check for upper limit violation
	mov [w0 + #MaxOutput], w6    ; load upper limit value
	cpslt w4, w6    ; compare values and skip next instruction if control output is within operating range (control output < upper limit)
	mov w6, w4    ; override controller output
	; Check for lower limit violation
	mov [w0 + #MinOutput], w6    ; load lower limit value
	cpsgt w4, w6    ; compare values and skip next instruction if control output is within operating range (control output > lower limit)
	mov w6, w4    ; override controller output
	
;------------------------------------------------------------------------------
; Write control output value to target
	mov [w0 + #ptrTargetRegister], w8    ; move pointer to target in to working register
	mov w4, [w8]    ; move control output into target address
	
;------------------------------------------------------------------------------
; Update ADC trigger A position
	asr w4, #1, w6    ; half control output by shifting value one bit to the right
	mov [w0 + #ADCTriggerAOffset], w8    ; load user-defined ADC trigger A offset value into working register
	add w6, w8, w10    ; add user-defined ADC trigger A offset to half of control output
	mov [w0 + #ptrADCTriggerARegister], w8    ; load pointer to ADC trigger A register into working register
	mov w10, [w8]    ; push new ADC trigger value to ADC trigger A register
	
;------------------------------------------------------------------------------
; Load pointer to first element of control history array
	mov [w0 + #ptrControlHistory], w10    ; load pointer address into wreg
	
;------------------------------------------------------------------------------
; Update control output history
	mov [w10 + #2], w6    ; move entry (n-2) one tick down the delay line
	mov w6, [w10 + #4]
	mov [w10 + #0], w6    ; move entry (n-1) one tick down the delay line
	mov w6, [w10 + #2]
	mov w4, [w10]    ; add most recent control output to history
	
;------------------------------------------------------------------------------
; Enable/Disable bypass branch target with dummy read of source buffer
	goto V_LOOP_EXIT_LOOP    ; when enabled, step over dummy read and go straight to EXIT
	V_LOOP_BYPASS_LOOP:    ; Enable/Disable bypass branch target to perform dummy read of source to clear the source buffer
	mov [w0 + #ptrSourceRegister], w2    ; load pointer to input source register
	mov [w2], w1    ; move value from input source into working register
	mov [w0 + #ptrDProvControlInput], w2    ; load pointer address of target buffer of most recent controller input from data structure
	mov w1, [w2]    ; copy most recent controller input value to given data buffer target
	V_LOOP_EXIT_LOOP:    ; Exit control loop branch target
	pop w12    ; restore working register used for status flag tracking
	
;------------------------------------------------------------------------------
; End of routine
	return
;------------------------------------------------------------------------------
	
;------------------------------------------------------------------------------
; Global function declaration _v_loop_Reset
; This function clears control and error histories enforcing a reset
;------------------------------------------------------------------------------
	
	.global _v_loop_Reset
_v_loop_Reset:
	
;------------------------------------------------------------------------------
; Clear control history array
	push w0    ; save contents of working register WREG0
	mov  [w0 + #ptrControlHistory], w0    ; set pointer to the base address of control history array
	clr [w0++]    ; clear next address of control history array
	clr [w0++]    ; clear next address of control history array
	clr [w0]    ; clear last address of control history array
	pop w0    ; restore contents of working register WREG0
	
;------------------------------------------------------------------------------
; Clear error history array
	push w0    ; save contents of working register WREG0
	mov [w0 + #ptrErrorHistory], w0    ; set pointer to the base address of error history array
	clr [w0++]    ; Clear next address of error history array
	clr [w0++]    ; Clear next address of error history array
	clr [w0++]    ; Clear next address of error history array
	clr [w0]    ; clear last address of error history array
	pop w0    ; restore contents of working register WREG0
	
;------------------------------------------------------------------------------
; End of routine
	return
;------------------------------------------------------------------------------
	
;------------------------------------------------------------------------------
; Global function declaration _v_loop_Precharge
; This function loads user-defined default values into control and error histories
;------------------------------------------------------------------------------
	
	.global _v_loop_Precharge
_v_loop_Precharge:
	
;------------------------------------------------------------------------------
; Charge error history array with defined value
	push w0    ; save contents of working register WREG0
	push w1    ; save contents of working register WREG1
	mov  [w0 + #ptrErrorHistory], w0    ; set pointer to the base address of error history array
	mov w1, [w0++]    ; Load user value into next address of error history array
	mov w1, [w0++]    ; Load user value into next address of error history array
	mov w1, [w0++]    ; Load user value into next address of error history array
	mov w1, [w0]    ; load user value into last address of error history array
	pop w1    ; restore contents of working register WREG1
	pop w0    ; restore contents of working register WREG0
	
;------------------------------------------------------------------------------
; Charge control history array with defined value
	push w0    ; save contents of working register WREG0
	push w2    ; save contents of working register WREG2
	mov  [w0 + #ptrControlHistory], w0    ; set pointer to the base address of control history array
	mov w2, [w0++]    ; Load user value into next address of control history array
	mov w2, [w0++]    ; Load user value into next address of control history array
	mov w2, [w0]    ; Load user value into last address of control history array
	pop w2    ; restore contents of working register WREG2
	pop w0    ; restore contents of working register WREG0
	
;------------------------------------------------------------------------------
; End of routine
	return
;------------------------------------------------------------------------------
	
;------------------------------------------------------------------------------
; End of file
	.end
;------------------------------------------------------------------------------
	