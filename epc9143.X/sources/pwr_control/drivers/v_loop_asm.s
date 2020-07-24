;LICENSE / DISCLAIMER ************************************************************
;  Microchip Technology Inc. and its subsidiaries.  You may use this software 
;  and any derivatives exclusively with Microchip products. 
;  
;  THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
;  EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
;  WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
;  PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
;  WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
;  
;  IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
;  INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
;  WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
;  BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
;  FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
;  IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
;  ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE. 
;  
;  MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
;  TERMS. 
; **********************************************************************************
; **********************************************************************************
;  SDK Version: z-Domain Control Loop Designer v0.9.9.312
;  AGS Version: Assembly Generator Script v2.0.16 (07/22/2020)
;  Author:      M91406
;  Date/Time:   07/24/2020 11:19:07
; **********************************************************************************
;  4P4Z Control Library File (Dual Bitshift-Scaling Mode)
; **********************************************************************************
	
;------------------------------------------------------------------------------
;file start
	.nolist
	.list
	
;------------------------------------------------------------------------------
;local inclusions.
	.section .data    ; place constant data in the data section
	
    ; include NPNZ16b_t object data structure value offsets and status flag labels
    .include "./pwr_control/drivers/npnz16b.inc"
    
;------------------------------------------------------------------------------
;local variables.
    
    .section .bss
    ; no variables declared

	
;------------------------------------------------------------------------------
;local inclusions.
	.section .text    ; place code in the code section
	
;------------------------------------------------------------------------------
; Global function declaration
; This function calls the z-domain controller processing the latest data point input
;------------------------------------------------------------------------------
	
	.global _v_loop_Update
_v_loop_Update:    ; provide global scope to routine
	
;------------------------------------------------------------------------------
; Check status word for Enable/Disable flag and bypass computation, if disabled
	mov [w0 + #Status], w12    ; load value of status word into working register
	btss w12, #NPNZ16_STATUS_ENABLED    ; check ENABLED bit state, skip (do not execute) next instruction if set
	bra V_LOOP_LOOP_BYPASS    ; if ENABLED bit is cleared, jump to end of control code
	
;------------------------------------------------------------------------------
; Setup pointers to A-Term data arrays
	mov [w0 + #ptrACoefficients], w8    ; load pointer to first index of A coefficients array
	
;------------------------------------------------------------------------------
; Load pointer to first element of control history array
	mov [w0 + #ptrControlHistory], w10    ; load pointer address into wreg
	
;------------------------------------------------------------------------------
; Compute compensation filter term
	clr a, [w8]+=4, w4, [w10]+=2, w6    ; clear accumulator A and prefetch first operands
	mac w4*w6, a, [w8]+=4, w4, [w10]+=2, w6    ; multiply control output (n-1) from the delay line with coefficient A1
	mac w4*w6, a, [w8]+=4, w4, [w10]+=2, w6    ; multiply control output (n-2) from the delay line with coefficient A2
	mac w4*w6, a, [w8]+=4, w4, [w10]+=2, w6    ; multiply control output (n-3) from the delay line with coefficient A3
	mac w4*w6, a    ; multiply & accumulate last control output with coefficient of the delay line (no more prefetch)
	
;------------------------------------------------------------------------------
; Backward normalization of recent result
	mov [w0 + #PostShiftA], w6    ; load A-coefficients post bit-shift scaler value into working register
	sftac a, w6    ; shift accumulator A by number of bits loaded in working register
	
;------------------------------------------------------------------------------
; Setup pointer to first element of error history array
	mov [w0 + #ptrErrorHistory], w10    ; load pointer address into wreg
	
;------------------------------------------------------------------------------
; Update error history (move error one tick along the delay line)
	mov [w10 + #6], w6    ; move entry (n-4) into buffer
	mov w6, [w10 + #8]    ; move buffered value one tick down the delay line
	mov [w10 + #4], w6    ; move entry (n-3) into buffer
	mov w6, [w10 + #6]    ; move buffered value one tick down the delay line
	mov [w10 + #2], w6    ; move entry (n-2) into buffer
	mov w6, [w10 + #4]    ; move buffered value one tick down the delay line
	mov [w10 + #0], w6    ; move entry (n-1) into buffer
	mov w6, [w10 + #2]    ; move buffered value one tick down the delay line
	
;------------------------------------------------------------------------------
; Read data from input source and calculate error input to transfer function
	mov [w0 + #ptrSourceRegister], w2    ; load pointer to input source register
	mov [w2], w1    ; move value from input source into working register
	mov [w0 + #ptrDProvControlInput], w2    ; load pointer address of target buffer of most recent raw controller input from data structure
	mov w1, [w2]    ; copy most recent controller input value to given data buffer target
	mov [w0 + #ptrCtrlReference], w2    ; move pointer to control reference into working register
	subr w1, [w2], w1    ; calculate error (=reference - input)
	mov [w0 + #PreShift], w2    ; move error input scaler into working register
	sl w1, w2, w1    ; normalize error result to fractional number format
	
;------------------------------------------------------------------------------
; Setup pointers to B-Term data arrays
	mov [w0 + #ptrBCoefficients], w8    ; load pointer to first index of B coefficients array
	mov w1, [w10]    ; add most recent error input to history array
	
;------------------------------------------------------------------------------
; Compute B-Term of the compensation filter
	clr b, [w8]+=4, w4, [w10]+=2, w6    ; clear accumulator B and prefetch first operands
	mac w4*w6, b, [w8]+=4, w4, [w10]+=2, w6    ; multiply & accumulate error input (n-0) from the delay line with coefficient B0 and prefetch next operands
	mac w4*w6, b, [w8]+=4, w4, [w10]+=2, w6    ; multiply & accumulate error input (n-1) from the delay line with coefficient B1 and prefetch next operands
	mac w4*w6, b, [w8]+=4, w4, [w10]+=2, w6    ; multiply & accumulate error input (n-2) from the delay line with coefficient B2 and prefetch next operands
	mac w4*w6, b, [w8]+=4, w4, [w10]+=2, w6    ; multiply & accumulate error input (n-3) from the delay line with coefficient B3 and prefetch next operands
	mac w4*w6, b    ; multiply & accumulate last error input with coefficient of the delay line (no more prefetch)
	
;------------------------------------------------------------------------------
; Backward normalization of recent result
	mov [w0 + #PostShiftB], w6    ; load B-coefficients post bit-shift scaler value into working register
	sftac b, w6    ; shift accumulator B by number of bits loaded in working register
	
;------------------------------------------------------------------------------
; Adaptive Loop Gain Modulation
	sac.r b, w6    ; store result of accumulator B in working register
	mov [w0 + ptrAgcObserverFunction], w1    ; load function pointer to observer function
	call w1    ; call extern observer function to update gain modulation factor
	mov [w0 + #agcGainModFactor], w4    ; load AGC modulation factor into working register
	mov [w0 + #agcGainModScaler], w2    ; load AGC modulation factor scaler into working register
	mpy w4*w6, b    ; multiply accumulator B result with AGC modulation factor
	sftac b, w2    ; shift result by AGC scaler
	
;------------------------------------------------------------------------------
; Add accumulators finalizing LDE computation
	add a    ; add accumulator b to accumulator a
	sac.r a, w4    ; store most recent accumulator result in working register
	
;------------------------------------------------------------------------------
; Controller Anti-Windup (control output value clamping)
	; Check for upper limit violation
	mov [w0 + #MaxOutput], w6    ; load upper limit value
	cpslt w4, w6    ; compare values and skip next instruction if control output is within operating range (control output < upper limit)
	mov w6, w4    ; override controller output
	V_LOOP_CLAMP_MAX_EXIT:
	; Check for lower limit violation
	mov [w0 + #MinOutput], w6    ; load lower limit value
	cpsgt w4, w6    ; compare values and skip next instruction if control output is within operating range (control output > lower limit)
	mov w6, w4    ; override controller output
	V_LOOP_CLAMP_MIN_EXIT:
	
;------------------------------------------------------------------------------
; Write control output value to target
	mov [w0 + #ptrTargetRegister], w8    ; move pointer to target to working register
	mov w4, [w8]    ; move control output to target address
	mov [w0 + #ptrAltTargetRegister], w8    ; move pointer to alternate target to working register
	mov w4, [w8]    ; move control output to alternate target address
	
;------------------------------------------------------------------------------
; Update ADC trigger locations
	asr w4, #1, w6    ; half control output by shifting value one bit to the right
	; Update ADC trigger A position
	mov [w0 + #ADCTriggerAOffset], w8    ; load user-defined ADC trigger A offset value into working register
	add w6, w8, w10    ; add user-defined ADC trigger A offset to half of control output
	mov [w0 + #ptrADCTriggerARegister], w8    ; load pointer to ADC trigger A register into working register
	mov w10, [w8]    ; push new ADC trigger value to ADC trigger A register
	
;------------------------------------------------------------------------------
; Load pointer to first element of control history array
	mov [w0 + #ptrControlHistory], w10    ; load pointer address into wreg
	
;------------------------------------------------------------------------------
; Update control output history
	mov [w10 + #4], w6    ; move entry (n-3) one tick down the delay line
	mov w6, [w10 + #6]
	mov [w10 + #2], w6    ; move entry (n-2) one tick down the delay line
	mov w6, [w10 + #4]
	mov [w10 + #0], w6    ; move entry (n-1) one tick down the delay line
	mov w6, [w10 + #2]
	mov w4, [w10]    ; add most recent control output to history
	
;------------------------------------------------------------------------------
; Update status flag bitfield
	mov w12, [w0 + #Status]    ; update value of the status word in data structure
	
;------------------------------------------------------------------------------
; Enable/Disable bypass branch target with dummy read of source buffer
	goto V_LOOP_LOOP_EXIT    ; when enabled, step over dummy read and go straight to EXIT
	V_LOOP_LOOP_BYPASS:    ; Enable/Disable bypass branch target to perform dummy read of source to clear the source buffer
	mov [w0 + #ptrSourceRegister], w2    ; load pointer to input source register
	mov [w2], w1    ; move value from input source into working register
	mov [w0 + #ptrDProvControlInput], w2    ; load pointer address of target buffer of most recent raw controller input from data structure
	mov w1, [w2]    ; copy most recent controller input value to given data buffer target
	V_LOOP_LOOP_EXIT:    ; Exit control loop branch target
	
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
	mov w2, [w0++]    ; Load user value into next address of control history array
	mov w2, [w0]    ; Load user value into last address of control history array
	pop w2    ; restore contents of working register WREG2
	pop w0    ; restore contents of working register WREG0
	
;------------------------------------------------------------------------------
; End of routine
	return
;------------------------------------------------------------------------------
	
;------------------------------------------------------------------------------
; Global function declaration _v_loop_PTermUpdate
; This function executes a P-term based control loop used for plant measurements only.
; THIS LOOP IS NOT SUITED FOR STABLE OPERATION
;------------------------------------------------------------------------------
	
	.global _v_loop_PTermUpdate
_v_loop_PTermUpdate:
	
;------------------------------------------------------------------------------
; Check status word for Enable/Disable flag and bypass computation when disabled
	mov [w0 + #Status], w12    ; load value of status word into working register
	btss w12, #NPNZ16_STATUS_ENABLED    ; check ENABLED bit state, skip (do not execute) next instruction if set
	bra V_LOOP_PTERM_LOOP_BYPASS    ; if ENABLED bit is cleared, jump to end of control code
	
;------------------------------------------------------------------------------
; Read data from input source and calculate error input to transfer function
	mov [w0 + #ptrSourceRegister], w2    ; load pointer to input source register
	mov [w2], w1    ; move value from input source into working register
	mov [w0 + #ptrDProvControlInput], w2    ; load pointer address of target buffer of most recent raw controller input from data structure
	mov w1, [w2]    ; copy most recent controller input value to given data buffer target
	mov [w0 + #ptrCtrlReference], w2    ; move pointer to control reference into working register
	subr w1, [w2], w1    ; calculate error (=reference - input)
	mov [w0 + #PreShift], w2    ; move error input scaler into working register
	sl w1, w2, w1    ; normalize error result to fractional number format
	
;------------------------------------------------------------------------------
; Load P-gain factor from data structure
	mov [w0 + #pterm_factor], w6    ; move P-coefficient fractional into working register
	mov [w0 + #pterm_scaler], w2    ; move P-coefficient scaler into working register
	mov w1, w4    ; move error to MPY working register
	; calculate P-control result
	mpy w4*w6, a    ; multiply most recent error with P-coefficient
	sftac a, w2    ; shift accumulator to post-scale floating number
	sac.r a, w4    ; store accumulator result to working register
	
;------------------------------------------------------------------------------
; Controller Anti-Windup (control output value clamping)
	; Check for upper limit violation
	mov [w0 + #MaxOutput], w6    ; load upper limit value
	cpslt w4, w6    ; compare values and skip next instruction if control output is within operating range (control output < upper limit)
	mov w6, w4    ; override controller output
	V_LOOP_PTERM_CLAMP_MAX_EXIT:
	; Check for lower limit violation
	mov [w0 + #MinOutput], w6    ; load lower limit value
	cpsgt w4, w6    ; compare values and skip next instruction if control output is within operating range (control output > lower limit)
	mov w6, w4    ; override controller output
	V_LOOP_PTERM_CLAMP_MIN_EXIT:
	
;------------------------------------------------------------------------------
; Write control output value to target
	mov [w0 + #ptrTargetRegister], w8    ; move pointer to target to working register
	mov w4, [w8]    ; move control output to target address
	mov [w0 + #ptrAltTargetRegister], w8    ; move pointer to alternate target to working register
	mov w4, [w8]    ; move control output to alternate target address
	
;------------------------------------------------------------------------------
; Update ADC trigger locations
	asr w4, #1, w6    ; half control output by shifting value one bit to the right
	; Update ADC trigger A position
	mov [w0 + #ADCTriggerAOffset], w8    ; load user-defined ADC trigger A offset value into working register
	add w6, w8, w10    ; add user-defined ADC trigger A offset to half of control output
	mov [w0 + #ptrADCTriggerARegister], w8    ; load pointer to ADC trigger A register into working register
	mov w10, [w8]    ; push new ADC trigger value to ADC trigger A register
	
;------------------------------------------------------------------------------
; Update status flag bitfield
	mov w12, [w0 + #Status]    ; update value of the status word in data structure
	
;------------------------------------------------------------------------------
; Enable/Disable bypass branch target with dummy read of source buffer
	goto V_LOOP_PTERM_LOOP_EXIT    ; when enabled, step over dummy read and go straight to EXIT
	V_LOOP_PTERM_LOOP_BYPASS:    ; Enable/Disable bypass branch target to perform dummy read of source to clear the source buffer
	mov [w0 + #ptrSourceRegister], w2    ; load pointer to input source register
	mov [w2], w1    ; move value from input source into working register
	mov [w0 + #ptrDProvControlInput], w2    ; load pointer address of target buffer of most recent raw controller input from data structure
	mov w1, [w2]    ; copy most recent controller input value to given data buffer target
	V_LOOP_PTERM_LOOP_EXIT:    ; Exit P-Term control loop branch target
	
;------------------------------------------------------------------------------
; End of routine
	return
;------------------------------------------------------------------------------
	
;------------------------------------------------------------------------------
; End of file
	.end
;------------------------------------------------------------------------------
	