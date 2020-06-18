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
	
    ; include NPNZ object data structure value offsets and status flag labels
    .include "./pwr_control/drivers/npnz16b.inc"

;------------------------------------------------------------------------------
;local variables.
    
    .section .bss
    ; no variables declared

;------------------------------------------------------------------------------
;code section.
	.section .text    ; place code in the code section
	
;------------------------------------------------------------------------------
; Global function declaration
; This function calls the z-domain controller processing the latest data point input
;------------------------------------------------------------------------------
	
	.global _v_loop_AGCFactorUpdate
_v_loop_AGCFactorUpdate:
    
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


