;MAB8048H, 4.000MHz
;Gdansk 2021
	.cr	8048
	.tf	rom.bin,BIN
	.lf	dac.lst
;==================Defines=====================
;Pins

;Fixed purpose registers/flags
;R0 - LUT pointer

;Set vectors
	.no $00 ;Set jump to main at reset vector (00h)
	jmp main

main:
    call lcd_init

    mov R4,#$56
    mov R5,#$C1
    mov R6,#$FC
    mov R7,#$FF

    clr F0 ;Perform signed right shift
    mov R3,#1 ;Shift 1 position
    call shr_32bit

    mov R0,#$00
    mov R1,#$FA
    mov R2,#$00
    mov R3,#$00
    call sub_32bit

    mov R1,#1

    mov A,R4
    mov R0,A
    call lcd_write

    mov A,R5
    mov R0,A
    call lcd_write

    mov A,R6
    mov R0,A
    call lcd_write

    mov A,R7
    mov R0,A
    call lcd_write
loop:
	jmp loop

;Constants


;R3 - number of positions to shift left, R4:R7 - 32-bit value to be shifted
; shl_32bit:
;     clr C ;Clear carry bit
;     mov A,R4 ;Load R4 to A
;     rlc A ;Rotate A left through carry - C->A0, A7->C
;     mov R4,A ;Store result back in source register

;     mov A,R5 ;Repeat the process for all the remaining bytes
;     rlc A
;     mov R5,A

;     mov A,R6
;     rlc A
;     mov R6,A

;     mov A,R7
;     rlc A
;     mov R7,A

;     djnz R3,shl_32bit ;Repeat the whole operation required number of times 
;     ret

;F0 - operation signedness, if set, unsigned, R3 - number of positions to shift right, R4:R7 - 32-bit value to be shifted
; shr_32bit:
;     clr C ;Clear carry
;     mov A,R7 ;Load R7 to A
;     jf0 shr_32bit_unsigned ;If flag set, perform unsigned shift
;     cpl A ;Complement A because of lack of jnbx instruction
;     jb7 shr_32bit_unsigned ;If sign bit is not set (negation of sign bit is set, actually...), value is positive, perform unsigned shift
;     cpl A ;Complement A again to restore original value
;     cpl C ;Set carry to be shifted to sign bit position - perform signed shift
; shr_32bit_unsigned:
;     rrc A ;Rotate A right through carry - C->A7, A0->C
;     mov R7,A ;Store result back in source register

;     mov A,R6 ;Continue the process for all the remaining bytes
;     rrc A
;     mov R6,A

;     mov A,R5
;     rrc A
;     mov R5,A

;     mov A,R4
;     rrc A
;     mov R4,A
;     djnz R3,shr_32bit ;Repeat the whole operation required number of times
;     ret

;F0 - operation signedness, if set, unsigned, R0 - pointer to value to be shifted, R1 - number of positions to shift right
shr_32bit:
    clr C ;Clear carry
    mov R2,#4 ;Load loop counter
    mov A,@R0 ;Load MSB to A
    jf0 shr_32bit_unsigned ;If flag set, perform unsigned shift
    cpl A ;Complement A because of lack of jnbx instruction
    jb7 shr_32bit_unsigned ;If sign bit is not set (negation of sign bit is set, actually...), value is positive, perform unsigned shift
    cpl A ;Complement A again to restore original value
    cpl C ;Set carry to be shifted to sign bit position - perform signed shift
shr_32bit_unsigned:
    rrc A ;Rotate A right through carry - C->A7, A0->C
    mov R7,A ;Store result back in source register

    mov A,R6 ;Continue the process for all the remaining bytes
    rrc A
    mov R6,A

    mov A,R5
    rrc A
    mov R5,A

    mov A,R4
    rrc A
    mov R4,A
    djnz R3,shr_32bit ;Repeat the whole operation required number of times
    ret

;R0 - pointer to value to be shifted, R1 - number of positions to shift right
shr_32bit:
    clr C ;Clear carry
    mov R2,#4 ;Load loop counter
shr_32bit_byte_loop:
    mov A,@R0 ;Load byte from RAM to A
    rrc A ;Rotate A right through carry - C->A7, A0->C
    mov @R0,A ;Store result back in RAM
    inc R0 ;Set pointer to next address
    djnz R2,shr_32bit_byte_loop ;Repeat for all bytes
    djnz R1,shr_32bit ;Repeat the whole operation required number of times
    ret

;R4:R7 - first addend and result, R0:R3 - second addend
add_32bit:
    mov A,R0
    add A,R4
    mov R4,A ;Add first bytes without carry and store in R4

    mov A,R1
    addc A,R5
    mov R5,A ;Add second bytes with carry and store in R5

    mov A,R2
    addc A,R6
    mov R6,A ;Add third bytes with carry and store in R6

    mov A,R3
    addc A,R7
    mov R7,A ;Add fourth bytes with carry and store in R7
    ret

;R0 - pointer to multiplicand, R1 - pointer to multiplier, R4:R7 - result, MSB first
mul_32bit:
    mov R3,#32 ;Set loop counter
    mov R4,#0 ;Clear result register
    mov R5,#0
    mov R6,#0
    mov R7,#0
mul_32bit_loop:
    clr C
    mov A,@R0 ;Load byte of multiplicand
    rrc A ;Rotate A right through carry - C->A7, A0->C
    mov @R0,A ;Store result back in source address
    inc R0 ;Move multiplicand pointer to next byte



    ret

;R4:R7 - minuend and result, R0:R3 - subtrahend
sub_32bit:
    mov A,R4 ;A = R4
    cpl A ;A = -R4-1, that's how two's complement works
    add A,R0 ;A = -R4-1+R0
    cpl A ;A = -(-R4-1+R0)-1 = R4+1-R0-1 = R4-R0
    mov R4,A ;Store result back in register

    mov A,R5 ;A = R5
    jnc sub_32bit_no_borrow_1 ;If no borrow from previous subtraction, continue with algorithm
    dec A ;If previous subtraction caused borrow, apply it here: A = R5-1 
sub_32bit_no_borrow_1:
    cpl A ;A = -R5-1 or A = -R5+1-1 = -R5
    add A,R1 ;A = -R5-1+R1 or A = -R5+R1
    cpl A ;A = R5-R1 or A = R5-R1-1
    mov R5,A ;Store result back in register

    mov A,R6 ;Continue the process for all the remaining bytes
    jnc sub_32bit_no_borrow_2
    dec A
sub_32bit_no_borrow_2:
    cpl A
    add A,R2
    cpl A
    mov R6,A

    mov A,R7
    jnc sub_32bit_no_borrow_3
    dec A
sub_32bit_no_borrow_3:
    cpl A
    add A,R3
    cpl A
    mov R7,A
    ret

;R3 - delay time in msec, uses R2,R3
delay_ms:
	mov R2,#146
delay_ms_loop:
	nop
	nop
	djnz R2,delay_ms_loop
	djnz R3,delay_ms
	ret

;~500uS delay, uses R2
delay_500us:
	mov R2,#71
delay_500us_loop:
	nop
	nop
	djnz R2,delay_500us_loop
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
	mov R3,#5
	call delay_ms ;Wait 5ms
	
	mov R0,#$30
	call lcd_write ;Weird repeated 4-bit init command second time...
	mov R3,#1
	call delay_ms ;Wait 1ms
	
	mov R0,#$30
	call lcd_write ;Weird repeated 4-bit init command third time...
	mov R3,#1
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
	mov R3,#1
	call delay_ms ;Wait 1ms
	
	mov R0,#$80
	call lcd_write ;Set cursor at first place in upper row
	mov R3,#1
	call delay_ms ;Wait 1ms
	ret	
