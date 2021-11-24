;MAB8049H, 10.000MHz
;Gdansk 2021
	.cr	8048
	.tf	rom.bin,BIN
	.lf	meteo.lst
;==================Defines=====================
;Pins

;Fixed purpose registers/flags

;RAM variables
tmp_mul .eq $20 ;32-bit

cal_T1  .eq $24 ;16-bit
cal_T2  .eq $26 ;16-bit
cal_T3  .eq $28 ;16-bit
cal_P1  .eq $2A ;16-bit
cal_P2  .eq $2C ;16-bit
cal_P3  .eq $2E ;16-bit
cal_P4  .eq $30 ;16-bit
cal_P5  .eq $32 ;16-bit
cal_P6  .eq $34 ;16-bit
cal_P7  .eq $36 ;16-bit
cal_P8  .eq $38 ;16-bit
cal_P9  .eq $3A ;16-bit

temp_adc    .eq $3C ;32-bit
temp_real   .eq $40 ;32-bit
pres_adc    .eq $44 ;32-bit
pres_real   .eq $48 ;32-bit

tmp1    .eq $4C ;32-bit
tmp2    .eq $50 ;32-bit
tmp3    .eq $54 ;32-bit
tmp4    .eq $58 ;32-bit

;Macros
swap    .ma Rx,Ry ;Swaps two registers
        mov A,]1 ;Move Rx to A
        xch A,]2 ;Swap Ry with A
        mov ]1,A ;Store Ry in Rx
        .em

;Set vectors
	.no $00 ;Set jump to main at reset vector (00h)
	jmp main

main:
    call lcd_init
    
    mov R1,#1
    mov R0,#'R'
    call lcd_write
    mov R0,#'e'
    call lcd_write
    mov R0,#'s'
    call lcd_write
    mov R0,#'u'
    call lcd_write
    mov R0,#'l'
    call lcd_write
    mov R0,#'t'
    call lcd_write
    mov R0,#':'
    call lcd_write
    mov R0,#' '
    call lcd_write

    ; mov R0,#temp_adc
    ; mov @R0,#$D0
    ; inc R0
    ; mov @R0,#$EE
    ; inc R0
    ; mov @R0,#$07
    ; inc R0
    ; mov @R0,#$00

    mov R0,#tmp1
    mov @R0,#$00
    inc R0
    mov @R0,#$00
    inc R0
    mov @R0,#$00
    inc R0
    mov @R0,#$81

    mov R1,#1
    mov R0,#tmp2
    mov A,@R0
    mov R0,A
    call lcd_write

    mov R0,#tmp2
    inc R0
    mov A,@R0
    mov R0,A
    call lcd_write

    mov R0,#tmp2
    inc R0
    inc R0
    mov A,@R0
    mov R0,A
    call lcd_write

    mov R0,#tmp2
    inc R0
    inc R0
    inc R0
    mov A,@R0
    mov R0,A
    call lcd_write
loop:
	jmp loop

;Constants

;Subroutines

;R3 - pointer to value to be zeroed, uses R0,R3,R7
zero_32bit:
    mov A,R3
    mov R0,A
    mov R7,#4
    clr A
zero_32bit_loop:
    mov @R0,A
    inc R0
    djnz R7,zero_32bit_loop
    ret

;R4 - pointer to destination, R5 - pointer to source, R6 - number of bytes to copy, uses R0,R1,R4,R5,R6,R7
copy_32bit:
    call zero_32bit
    mov A,R4
    mov R0,A ;Copy R4 to R0, to preserve R4, also only R0 and R1 can be used to access RAM
    mov A,R5
    mov R1,A ;Copy R5 to R1, the same reason as above
copy_32bit_loop:
    mov A,@R1
    mov @R0,A ;Copy [R1] to [R0]
    inc R0
    inc R1 ;Move pointers to next byte
    djnz R6,copy_32bit_loop
    ret

;R5 - pointer to value to be shifted (LSB first), R6 - number of positions to shift left, uses R0,R5,R6,R7
shl_32bit:
    clr C ;Clear carry bit
    mov R7,#4 ;Load loop counter
    mov A,R5
    mov R0,A ;Copy R5 to R0, so that R5 won't be changed in inner loop - needed to shift by multiple positions
shl_32bit_loop:
    mov A,@R0 ;Load byte from RAM to A
    rlc A ;Rotate A left through carry - C->A0, A7->C
    mov @R0,A ;Store result back in RAM
    inc R0 ;Move pointer to next byte
    djnz R7,shl_32bit_loop ;Repeat for all bytes
    djnz R6,shl_32bit ;Repeat the whole process required number of times
    ret

;F0 - operation signedness, if set, unsigned, R5 - pointer to value to be shifted (LSB first), R6 - number of positions to shift right, uses F0,R0,R5,R6,R7
shr_32bit:
    clr C ;Clear carry bit
    mov R7,#4 ;Load loop counter
    mov A,R5 ;Load R5 to A
    add A,#3 ;Move pointer to MSB
    mov R0,A ;Store result in R0, so that R5 won't be changed - needed to shift by multiple positions
    jf0 shr_32bit_loop ;If flag set, do not set carry - perform unsigned shift
    mov A,@R0 ;Load MSB to A
    cpl A ;Complement A because of lack of jnbx instruction
    jb7 shr_32bit_loop ;If sign bit is not set (negation of sign bit is set, actually...), value is positive, perform unsigned shift
    cpl C ;Otherwise set carry - perform signed shift
shr_32bit_loop:
    mov A,@R0 ;Load byte from RAM to A
    rrc A ;Rotate A right through carry - C->A7, A0->C
    mov @R0,A ;Store result back in RAM
    dec R0 ;Move pointer to previous byte
    djnz R7,shr_32bit_loop ;Repeat for all bytes
    djnz R6,shr_32bit ;Repeat the whole operation required number of times
    ret

;R3 - pointer to first addend and result, R4 - pointer to second addend, uses R0,R1,R3,R4,R7
add_32bit:
    clr C ;Clear carry
    mov R7,#4 ;Load loop counter
    mov A,R3
    mov R0,A ;Copy R3 to R0, to preserve R3, also only R0 and R1 can be used to access RAM
    mov A,R4
    mov R1,A ;Copy R4 to R1, the same reason as above
add_32bit_loop:
    mov A,@R1 ;Load byte from RAM to A
    addc A,@R0 ;Add byte from second added
    mov @R0,A ;Store back in RAM
    inc R0 
    inc R1 ;Move both pointers to next byte
    djnz R7,add_32bit_loop ;Repeat for all bytes
    ret

;R3 - pointer to minuend and result, R4 - pointer to second subtrahend, uses R0,R1,R3,R4,R7
sub_32bit:
    clr C ;Clear carry
    mov R7,#4 ;Load loop counter
    mov A,R3
    mov R0,A ;Copy R3 to R0, to preserve R3, also only R0 and R1 can be used to access RAM
    mov A,R4
    mov R1,A ;Copy R4 to R1, the same reason as above
sub_32bit_loop:
    mov A,@R0 ;A = [R0]
    jnc sub_32bit_no_borrow ;If no borrow from previous subtraction, continue with algorithm
    dec A ;If previous subtraction caused borrow, apply it here: A = [R0]-1 
sub_32bit_no_borrow:
    cpl A ;A = -[R0]-1, that's how two's complement works
    add A,@R1 ;A = -[R0]-1+[R1]
    cpl A ;A = -(-[R0]-1+[R1])-1 = [R0]+1-[R1]-1 = [R0]-[R1]
    mov @R0,A ;[R0] = [R0]-[R1]
    inc R0
    inc R1
    djnz R7,sub_32bit_loop
    ret

;R3 - pointer to result, R4 - pointer to multiplicand, R5 - pointer to multiplier, LSB first, uses ALL registers and F0
mul_32bit:
    call check_sign ;Check sign of the result and complement inputs if needed
    call zero_32bit ;Clear result
    mov R2,#32 ;Set loop counter
    clr F0
    cpl F0 ;Set F0 - perform unsigned right shifts
mul_32bit_loop:
    mov R6,#1
    call shr_32bit ;Shift multiplier 1 time right
    jnc mul_32bit_no_carry ;If carry not set, don't add multiplicand to result
    call add_32bit ;Add multiplicand to result
mul_32bit_no_carry:
    >swap R4,R5 ;Swap R4 and R5
    mov R6,#1
    call shl_32bit ;Shift multiplicand 1 time left
    >swap R4,R5 ;Revert R4 and R5 to original state
    djnz R2,mul_32bit_loop ;Repeat for every multiplier bit
    jf1 mul_32bit_end ;If there's no need to change sign of the result, finish
    call neg_32bit ;Change sign of the result
mul_32bit_end:
    ret

;R4 - pointer to first value, R5 - pointer to second value, F1 - sign flag, if cleared, result sign has to be changed
check_sign:
    clr F1
    cpl F1 ;Set sign flag
    mov A,R4 ;Load pointer to first value to A
    add A,#3 ;Move pointer to MSB
    mov R0,A ;Load pointer to MSB to R0
    mov A,@R0 ;Load MSB to A
    cpl A ;Complement A
    jb7 check_sign_first_pos ;If sign bit is not set, proceed to check second value
    inc A ;Finish two's complement
    mov @R0,A ;Store two's complement of value back in RAM
    cpl F1 ;Set sign flag
check_sign_first_pos:
    mov A,R5 ;Load pointer to second value to A
    add A,#3 ;Move pointer to MSB
    mov R0,A ;Load pointer to MSB to R0
    mov A,@R0 ;Load MSB to A
    cpl A ;Complement A
    jb7 check_sign_done ;If sign bit is not set, finish
    inc A ;Finish two's complement
    mov @R0,A ;Store two's complement of value back in RAM
    cpl F1 ;Set sign flag
check_sign_done:
    ret

;R3 - value to change sign of, uses R0,R3,R4,R7
neg_32bit:
    mov R7,#4 ;Load loop counter
    mov A,R3
    mov R0,A ;Copy R3 to R0, only R0 and R1 can be used to access RAM
neg_32bit_loop:
    mov A,@R0 ;Load byte to A
    cpl A ;Complement A
    mov @R0,A ;Store value back in RAM
    inc R0
    djnz R7,neg_32bit_loop ;Repeat for every byte
    mov A,R3
    mov R4,A ;Preserve R3 in R4
    mov R3,#tmp_mul
    call zero_32bit ;Clear tmp_mul variable
    mov R0,#tmp_mul
    mov @R0,#1 ;tmp_mul = 1
    mov A,R4
    mov R3,A ;Restore R3
    mov R4,#tmp_mul
    call add_32bit ;Add 1 to result to finish two's complement
    ret

;TODO signed division

;R6 - delay time in msec, uses R6,R7
delay_ms:
	mov R7,#146
delay_ms_loop:
	nop
	nop
	djnz R7,delay_ms_loop
	djnz R6,delay_ms
	ret

;~500uS delay, uses R7
delay_500us:
	mov R7,#71
delay_500us_loop:
	nop
	nop
	djnz R7,delay_500us_loop
	ret

;R0 - byte, R1 - cmd/data switch, uses R0,R1
lcd_write:
	anl P2,#%11011111 ;Clear RS
	;Test whether data or cmd will be sent
	mov A,R1 ;Load R1 to A to test if zero
	jz skip_rs ;Skip RS line setting - cmd will be sent
	orl P2,#%00100000 ;Set RS line - data will be sent
skip_rs:
	;Send upper nibble
	mov A,R0 ;Load byte to A
	anl A,#%11110000 ;Mask lower nibble
	outl P1,A ;Send data to P1
	
	orl P2,#%00010000 ;Set E line
	call delay_500us ;Wait for LCD	
	anl P2,#%11101111 ;Clear E line
	call delay_500us ;Wait for LCD
	
	;Send lower nibble
	mov A,R0 ;Load byte to A
	swap A ;Swap nibbles
	anl A,#%11110000 ;Mask lower nibble
	outl P1,A ;Send data to P1
	
	orl P2,#%00010000 ;Set E line
	call delay_500us ;Wait for LCD	
	anl P2,#%11101111 ;Clear E line
	call delay_500us ;Wait for LCD	
	ret
	
;R0 - y, R1 - x, uses R0,R1	
lcd_gotoxy:
	mov A,R1
	jnz second_row ;Check row
	mov A,#$80 ;If first, load address of its first position
	jmp lcd_gotoxy_write
second_row:
	mov A,#$C0 ;If second, load address of its first position
lcd_gotoxy_write:
	add A,R0 ;Add offset (y)
	mov R0,A
	mov R1,#0
	call lcd_write ;Send command
	ret
	
;Uses R0,R1,R6,R7	
lcd_init:
	mov R1,#0 ;Whole subroutine will be sending commands
	
	mov R0,#$30	
	call lcd_write ;Weird 4-bit init command first time...
	mov R6,#5
	call delay_ms ;Wait 5ms
	
	mov R0,#$30
	call lcd_write ;Weird repeated 4-bit init command second time...
	mov R6,#1
	call delay_ms ;Wait 1ms
	
	mov R0,#$30
	call lcd_write ;Weird repeated 4-bit init command third time...
	mov R6,#1
	call delay_ms ;Wait 1ms

	mov R0,#$02
	call lcd_write ;Init 4-bit mode
	
	mov R0,#$28
	call lcd_write ;2 lines, 5*8 matrix, 4-bit
	
	mov R0,#$0C
	call lcd_write ;Display on, cursor off
	
	mov R0,#$06
	call lcd_write ;Autoincrement cursor position, text scroll off
	
	call lcd_cls ;Clear screen
	ret
	
;Uses R0,R1,R6,R7	
lcd_cls:
	mov R1,#0
	mov R0,#$01	
	call lcd_write ;Clear display
	mov R6,#1
	call delay_ms ;Wait 1ms
	
	mov R0,#$80
	call lcd_write ;Set cursor at first place in upper row
	mov R6,#1
	call delay_ms ;Wait 1ms
	ret	
