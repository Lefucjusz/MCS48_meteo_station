;MAB8048H, 4.000MHz
;Gdansk 2021
	.cr	8048
	.tf	rom.bin,BIN
	.lf	dac.lst
;==================Defines=====================
;Pins

;Fixed purpose registers/flags
;R0 - LUT pointer

;RAM variables
cal_T1  .eq $20 ;16-bit
cal_T2  .eq $22 ;16-bit
cal_T3  .eq $24 ;16-bit
cal_P1  .eq $26 ;16-bit
cal_P2  .eq $28 ;16-bit
cal_P3  .eq $2A ;16-bit
cal_P4  .eq $2C ;16-bit
cal_P5  .eq $2E ;16-bit
cal_P6  .eq $30 ;16-bit
cal_P7  .eq $32 ;16-bit
cal_P8  .eq $34 ;16-bit
cal_P9  .eq $36 ;16-bit

tmp1    .eq $38 ;32-bit
tmp2    .eq $42 ;32-bit
adc_T    .eq $46 ;32-bit   
t_fine    .eq $50 ;32-bit

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

    call load_cal_data
    call compute_T

    mov R0,#var1
    mov @R0,#$CC
    inc R0
    mov @R0,#$F7
    inc R0
    mov @R0,#$06
    inc R0
    mov @R0,#$88

    mov R0,#var2
    mov @R0,#$FC
    inc R0
    mov @R0,#$7A
    inc R0
    mov @R0,#$34
    inc R0
    mov @R0,#$AC

    ; mov R3,#var3
    mov R4,#var2
    mov R5,#var1
    call sub_32bit

    mov R1,#1
    mov R0,#var2
    mov A,@R0
    mov R0,A
    call lcd_write

    mov R0,#var2
    inc R0
    mov A,@R0
    mov R0,A
    call lcd_write

    mov R0,#var2
    inc R0
    inc R0
    mov A,@R0
    mov R0,A
    call lcd_write

    mov R0,#var2
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

compute_T:
    mov R4,#adc_T
    mov R5,#tmp1
    call copy_32bit
    clr F0 ;Signed shift
    mov R6,#3
    call shr_32bit ;tmp1 = adc_T>>3

    mov R4,#cal_T1
    mov R5,#tmp2
    call copy_32bit
    mov R6,#1
    call shl_32bit ;tmp2 = cal_T1<<1

    mov R4,#tmp1
    mov R5,#tmp2
    call sub_32bit ;tmp1 = adc_T>>3 - cal_T1<<1

    mov R3,#tmp2
    mov R4,#tmp1
    mov R5,#cal_T2


    ret

load_cal_data:
    mov R0,#cal_T1
    mov @R0,#$70
    inc R0
    mov @R0,#$6B

    mov R0,#cal_T2
    mov @R0,#$43
    inc R0
    mov @R0,#$67

    mov R0,#cal_T3
    mov @R0,#$18
    inc R0
    mov @R0,#$FC

    mov R0,#cal_P1
    mov @R0,#$7D
    inc R0
    mov @R0,#$8E

    mov R0,#cal_P2
    mov @R0,#$43
    inc R0
    mov @R0,#$D6

    mov R0,#cal_P3
    mov @R0,#$D0
    inc R0
    mov @R0,#$0B

    mov R0,#cal_P4
    mov @R0,#$27
    inc R0
    mov @R0,#$0B

    mov R0,#cal_P5
    mov @R0,#$8C
    inc R0
    mov @R0,#$00

    mov R0,#cal_P6
    mov @R0,#$F9
    inc R0
    mov @R0,#$FF

    mov R0,#cal_P7
    mov @R0,#$8C
    inc R0
    mov @R0,#$3C

    mov R0,#cal_P8
    mov @R0,#$F8
    inc R0
    mov @R0,#$C6

    mov R0,#cal_P9
    mov @R0,#$70
    inc R0
    mov @R0,#$17
    ret

;R4 - pointer to source, R5 - pointer to destination, uses R0,R1,R4,R5,R7
copy_32bit:
    mov R7,#4 ;Load loop counter
    mov A,R4
    mov R0,A ;Copy R4 to R0, to preserve R4, also only R0 and R1 can be used to access RAM
    mov A,R5
    mov R1,A ;Copy R5 to R1, the same reason as above
copy_32bit_loop:
    mov A,@R0
    mov @R1,A ;Copy [R0] to [R1]
    inc R0
    inc R1 ;Move pointers to next byte
    djnz R7,copy_32bit_loop
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

;R4 - pointer to first addend and result, R5 - pointer to second addend, uses R0,R1,R4,R5,R7
add_32bit:
    clr C ;Clear carry
    mov R7,#4 ;Load loop counter
    mov A,R4
    mov R0,A ;Copy R4 to R0, to preserve R4, also only R0 and R1 can be used to access RAM
    mov A,R5
    mov R1,A ;Copy R5 to R1, the same reason as above
add_32bit_loop:
    mov A,@R1 ;Load byte from RAM to A
    addc A,@R0 ;Add byte from second added
    mov @R0,A ;Store back in RAM
    inc R0 
    inc R1 ;Move both pointers to next byte
    djnz R7,add_32bit_loop ;Repeat for all bytes
    ret

;R4 - pointer to minuend and result, R5 - pointer to second subtrahend, uses R0,R1,R4,R5,R7
sub_32bit:
    clr C ;Clear carry
    mov R7,#4 ;Load loop counter
    mov A,R4
    mov R0,A ;Copy R4 to R0, to preserve R4, also only R0 and R1 can be used to access RAM
    mov A,R5
    mov R1,A ;Copy R5 to R1, the same reason as above
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

;R3 - pointer to result, R4 - pointer to multiplicand, R5 - pointer to multiplier, LSB first TODO signed multiplication
mul_32bit:
    mov R2,#4 ;Set loop counter
    mov A,R3
    mov R0,A ;Copy R3 to R0, to preserve R3, also only R0 and R1 can be used to access RAM
    clr A ;Clear A
mul_32bit_clear_result:
    mov @R0,A ;Clear byte
    inc R0 ;Move pointer to next byte
    djnz R2,mul_32bit_clear_result ;Repeat for every byte

    mov R2,#32 ;Set loop counter
    clr F0
    cpl F0 ;Set F0 - perform unsigned right shifts
mul_32bit_loop:
    mov R6,#1
    call shr_32bit ;Shift multiplier 1 time right
    jnc mul_32bit_no_carry ;If carry not set, don't add multiplicand to result
    >swap R3,R4
    >swap R3,R5 ;Swap pointers, so that R3 = R5, R4 = R3, R5 = R4
    call add_32bit ;Perform addition
    >swap R3,R5
    >swap R3,R4 ;Revert pointers to original state
mul_32bit_no_carry:
    >swap R4,R5 ;Swap R4 and R5
    mov R6,#1
    call shl_32bit ;Shift multiplicand 1 time left
    >swap R4,R5 ;Revert R4 and R5 to original state
    djnz R2,mul_32bit_loop
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
