;MAB8049H, 10.000MHz
;Gdansk 2021
	.cr	8048
	.tf	rom.bin,BIN
	.lf	meteo.lst
;==================Defines=====================
;Pins
tx  .eq %10000000 ;Tx pin at P2.7

;RAM variables, little endian
tmp_neg .eq $20 ;32-bit

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

temp_read   .eq $3C ;32-bit
temp_real   .eq $40 ;32-bit
pres_read   .eq $44 ;32-bit
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

movr    .ma Rx,Ry ;Moves Ry to Rx (Rx = Ry)
        mov A,]2 ;Move Ry to A
        mov ]1,A ;Move A to Rx
        .em

;Set vectors
	.no $00 ;Set jump to main at reset vector (00h)
	jmp main

main:
    mov R0,#temp_read
    mov @R0,#$D0
    inc R0
    mov @R0,#$EE
    inc R0
    mov @R0,#$07
    inc R0
    mov @R0,#$00

    call load_cal_data
    call bmp280_compute_compensation

    mov A,#temp_real
    add A,#3
    mov R1,A
    mov A,@R1
    mov R0,A
    call uart_write_byte

    dec R1
    mov A,@R1
    mov R0,A
    call uart_write_byte

    dec R1
    mov A,@R1
    mov R0,A
    call uart_write_byte

    dec R1
    mov A,@R1
    mov R0,A
    call uart_write_byte

loop:
	jmp loop

;Constants

;Subroutines

;R0	- byte to send, uses R0,R6,R7
uart_write_byte:
	mov R6,#8 ;Load bit counter	
	mov A,R0 ;Move byte to be send to A	
	anl P2,#~tx ;Set tx pin low - start bit
	call delay_100us
uart_write_loop:
	jb0 uart_write_one ;Check if LSB of A is set
	anl P2,#~tx ;Set tx pin low
	jmp uart_write_delay	
uart_write_one:
	orl P2,#tx ;Set tx pin high
uart_write_delay:
	call delay_100us
	rr A ;Shift byte one bit right
	djnz R6,uart_write_loop

	orl P2,#tx ;Set tx pin high - stop bit
	call delay_100us
	ret
    
;~100uS delay, uses R7
delay_100us:
	mov R7,#28
delay_100us_loop:
	djnz R7,delay_500us_loop
	ret

;~500uS delay, uses R7
delay_500us:
	mov R7,#164
delay_500us_loop:
	djnz R7,delay_500us_loop
	ret

;R6 - delay time in msec, uses R6,R7
delay_ms:
	mov R7,#228
delay_ms_loop:
	nop
	djnz R7,delay_ms_loop
	djnz R6,delay_ms
	ret

;R0 - pointer to value to be zeroed, uses and destroys R0,R7
zero_32bit:
    clr A ;Clear A
    mov R7,#4 ;Load loop counter
zero_32bit_loop:
    mov @R0,A ;Load A = 0 to byte
    inc R0 ;Move pointer to next byte
    djnz R7,zero_32bit_loop ;Repeat for every byte
    ret

;R0 - pointer to value to be filled, uses and destroys R0,R7
fill_32bit:
    clr A ;Clear A
    cpl A ;Complement A
    mov R7,#4 ;Load loop counter
fill_32bit_loop:
    mov @R0,A ;Load A = FF to byte
    inc R0 ;Move pointer to next byte
    djnz R7,fill_32bit_loop ;Repeat for every byte
    ret

;F0 - signedness, if set, unsigned, R0 - pointer to destination, R1 - pointer to source, R6 - number of bytes to copy, uses F0,R0,R1,R5,R6,R7, destroys R0,R1,R5,R6,R7
copy_32bit:
    >movr R5,R0 ;Preserve R0 in R5
    >movr R4,R1 ;Preserve R1 in R4
    jf0 copy_32bit_unsigned ;If flag set, unsigned
    mov A,R1 ;Load pointer to source to A
    add A,R6
    dec A ;Move pointer to MSB
    mov R1,A ;Store value back in register
    mov A,@R1 ;Load MSB to A
    cpl A ;Complement A
    jb7 copy_32bit_unsigned ;If sign bit not set, value is positive, sign extension as for unsigned
    call fill_32bit ;Perform sign extension for signed value
    jmp copy_32bit_continue ;Continue with algorithm
copy_32bit_unsigned:
    call zero_32bit ;Perform sign extension for unsigned value
copy_32bit_continue:
    >movr R1,R4 ;Restore R1 from R4
    >movr R0,R5 ;Restore R0 from R5
copy_32bit_loop:
    mov A,@R1
    mov @R0,A ;Copy [R1] to [R0]
    inc R0
    inc R1 ;Move pointers to next byte
    djnz R6,copy_32bit_loop
    ret

;R5 - pointer to value to be shifted, R6 - number of positions to shift, uses R0,R5,R6,R7, destroys R0,R6,R7
shl_32bit:
    clr C ;Clear carry bit
shlc_32bit:
    mov R7,#4 ;Load loop counter
    >movr R0,R5 ;Copy R5 to R0, so that R5 won't be changed in inner loop - needed to shift by multiple positions
shl_32bit_loop:
    mov A,@R0 ;Load byte from RAM to A
    rlc A ;Rotate A left through carry - C->A0, A7->C
    mov @R0,A ;Store result back in RAM
    inc R0 ;Move pointer to next byte
    djnz R7,shl_32bit_loop ;Repeat for all bytes
    djnz R6,shl_32bit ;Repeat the whole process required number of times
    ret

;F0 - signedness, if set, unsigned, R5 - pointer to value to be shifted, R6 - number of positions to shift, uses F0,R0,R5,R6,R7, destroys F0,R0,R6,R7
shr_32bit:
    clr C ;Clear carry bit
shrc_32bit:
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

;R0 - pointer to first addend and result, R1 - pointer to second addend, uses and destroys R0,R1,R7
add_32bit:
    clr C ;Clear carry
    mov R7,#4 ;Load loop counter
add_32bit_loop:
    mov A,@R1 ;Load byte from RAM to A
    addc A,@R0 ;Add byte from second added
    mov @R0,A ;Store back in RAM
    inc R0 
    inc R1 ;Move both pointers to next byte
    djnz R7,add_32bit_loop ;Repeat for all bytes
    ret

;R0 - pointer to minuend and result, R1 - pointer to second subtrahend, uses and destroys R0,R1,R7
sub_32bit:
    clr C ;Clear carry
    mov R7,#4 ;Load loop counter
sub_32bit_loop: 
    mov A,@R0 ;A = [R0]
    cpl A ;A = -[R0]-1, that's how two's complement works
    addc A,@R1 ;A = -[R0]-1+[R1]+C
    cpl A ;A = -(-[R0]-1+[R1]+C)-1 = [R0]+1-[R1]-C-1 = [R0]-[R1]-C
    mov @R0,A ;[R0] = [R0]-[R1]-C
    inc R0
    inc R1
    djnz R7,sub_32bit_loop
    ret

;R0 - pointer to value to change sign of, uses and destroys R0,R1,R7
neg_32bit:
    mov R7,#4 ;Load loop counter
    >movr R1,R0 ;Preserve pointer to value in R1 - it will be used again later
neg_32bit_loop:
    mov A,@R0 ;Load byte to A
    cpl A ;Complement A
    mov @R0,A ;Store value back in RAM
    inc R0
    djnz R7,neg_32bit_loop ;Repeat for every byte
    mov R0,#tmp_neg
    call zero_32bit ;Clear tmp_neg variable
    mov R0,#tmp_neg
    mov @R0,#1 ;tmp_neg = 1
    >movr R0,R1 ;Load pointer to value back to R0
    mov R1,#tmp_neg
    call add_32bit ;Add 1 to result to finish two's complement
    ret

;R3 - pointer to result, R4 - pointer to multiplicand, R5 - pointer to multiplier, uses F0 and ALL registers, destroys F0,R0,R1,R2,R6,R7
mul_32bit:
    >movr R0,R3 ;Load R3 to R0
    call zero_32bit ;Clear result
    mov R2,#32 ;Set loop counter
    clr F0
    cpl F0 ;Set F0 - perform unsigned right shifts
mul_32bit_loop: 
    mov R6,#1 
    call shr_32bit ;Shift multiplier 1 time right
    jnc mul_32bit_no_carry ;If carry not set, don't add multiplicand to result
    >movr R0,R3 ;Load R3 to R0
    >movr R1,R4 ;Load R4 to R1
    call add_32bit ;Add multiplicand to result
mul_32bit_no_carry:
    >swap R4,R5 ;Swap R4 and R5
    mov R6,#1
    call shl_32bit ;Shift multiplicand 1 time left
    >swap R4,R5 ;Revert swap
    djnz R2,mul_32bit_loop ;Repeat for every multiplier bit
    ret

;R3 - pointer to remainder, R4 - pointer to divisor, R5 - pointer to dividend and result, uses BOTH flags and ALL registers, destroys F0,F1,R0,R1,R2,R6,R7
div_32bit:
    call check_sign ;Check sign of operands, complement them if needed
    >movr R0,R3 ;Load pointer to remainder to R0
    call zero_32bit ;Clear remainder
    mov R2,#32 ;Set loop counter
    clr C ;Clear carry
    clr F0
    cpl F0 ;Set F0 - perform unsigned right shifts
div_32bit_loop:
    mov R6,#1
    call shlc_32bit ;Shift dividend left with carry
    >swap R3,R5 ;Swap R3 and R5
    mov R6,#1
    call shlc_32bit ;Shift carry (MSB of dividend) into remainder
    >swap R3,R5 ;Revert swap
    >movr R0,R3 ;Load pointer to remainder to R0
    >movr R1,R4 ;Load pointer to divisor to R1
    call sub_32bit ;remainder = remainder - divisor
    jnc div_32bit_fit ;If result positive, divisor fits into remainder
    >movr R0,R3 ;Load pointer to remainder to R0
    >movr R1,R4 ;Load pointer to divisor to R1
    call add_32bit ;Restore remainder    
    clr C ;Clear carry to be shifted into result
    jmp div_32bit_continue
div_32bit_fit:
    clr C
    cpl C ;Set carry to be shifted into result
div_32bit_continue:
    djnz R2,div_32bit_loop
    mov R6,#1
    call shlc_32bit ;Shift dividend one last time
    jf1 div_32bit_end ;If there's no need to change sign of the result, finish
    >movr R0,R5 ;Load pointer to result to R0
    call neg_32bit ;Change sign of the result
div_32bit_end:
    ret

;R4 - pointer to first value, R5 - pointer to second value, F1 - sign flag, if cleared, result sign has to be changed, uses F1,R0,R1,R4,R5,R7, destroys F1,R0,R1,R7
check_sign:
    clr F1
    cpl F1 ;Set sign flag
    mov A,R4 ;Load pointer to first value to A
    add A,#3 ;Move pointer to MSB
    mov R0,A ;Load pointer to MSB to R0
    mov A,@R0 ;Load MSB to A
    cpl A ;Complement A
    jb7 check_sign_first_pos ;If sign bit is not set, proceed to check second value
    >movr R0,R4 ;Load pointer to first value to R0
    call neg_32bit ;Change sign of value
    cpl F1 ;Complement sign flag
check_sign_first_pos:
    mov A,R5 ;Load pointer to second value to A
    add A,#3 ;Move pointer to MSB
    mov R0,A ;Load pointer to MSB to R0
    mov A,@R0 ;Load MSB to A
    cpl A ;Complement A
    jb7 check_sign_done ;If sign bit is not set, finish
    >movr R0,R5 ;Load pointer to second value to R0
    call neg_32bit ;Change sign of value
    cpl F1 ;Complement sign flag
check_sign_done:
    ret

bmp280_compute_compensation:
    mov R0,#tmp1
    mov R1,#temp_read
    mov R6,#4
    call copy_32bit ;Copy 4 bytes, tmp1 = temp_read
    clr F0 ;Perform signed shift
    mov R5,#tmp1
    mov R6,#3
    call shr_32bit ;tmp1 = temp_read>>3
    cpl F0 ;Perform unsigned sign extension
    mov R0,#tmp2
    mov R1,#cal_T1
    mov R6,#2
    call copy_32bit ;Copy 2 bytes - tmp2 (32-bit) = cal_T1 (16-bit)
    mov R5,#tmp2
    mov R6,#1
    call shl_32bit ;tmp2 = cal_T1<<1
    mov R0,#tmp1
    mov R1,#tmp2
    call sub_32bit ;tmp1 = temp_read>>3 - cal_T1<<1
    clr F0 ;Perform signed sign extension
    mov R0,#tmp2
    mov R1,#cal_T2
    mov R6,#2
    call copy_32bit ;Copy 2 bytes, tmp2 = cal_T2
    mov R3,#tmp3
    mov R4,#tmp1
    mov R5,#tmp2
    call mul_32bit ;tmp3 = (temp_read>>3 - cal_T1<<1) * cal_T2
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it
    mov R5,#tmp3
    mov R6,#11
    call shr_32bit ;tmp3 = ((temp_read>>3 - cal_T1<<1) * cal_T2) >> 11

    mov R0,#tmp1
    mov R1,#temp_read
    mov R6,#4
    call copy_32bit ;Copy 4 bytes, tmp1 = temp_read
    ;Perform signed shift, flag already cleared
    mov R5,#tmp1
    mov R6,#4
    call shr_32bit ;tmp1 = temp_read>>4
    cpl F0 ;Perform unsigned sign extension
    mov R0,#tmp2
    mov R1,#cal_T1
    mov R6,#2
    call copy_32bit ;Copy 2 bytes - tmp2 (32-bit) = cal_T1 (16-bit)
    mov R0,#tmp1
    mov R1,#tmp2
    call sub_32bit ;tmp1 = temp_read>>4 - cal_T1
    mov R0,#tmp2
    mov R1,#tmp1
    mov R6,#4
    call copy_32bit ;Copy 4 bytes, tmp2 = tmp1 = temp_read>>4 - cal_T1
    mov R3,#tmp4
    mov R4,#tmp1
    mov R5,#tmp2
    call mul_32bit ;tmp4 = (temp_read>>4 - cal_T1) * (temp_read>>4 - cal_T1)
    clr F0 ;Perform signed shift
    mov R5,#tmp4
    mov R6,#12
    call shr_32bit ;tmp4 = ((temp_read>>4 - cal_T1) * (temp_read>>4 - cal_T1)) >> 12
    ;Perform signed sign extension, flag already cleared
    mov R0,#tmp1
    mov R1,#cal_T3
    mov R6,#2
    call copy_32bit ;Copy 2 bytes - tmp1 (32-bit) = cal_T3 (16-bit)
    mov R3,#tmp2
    mov R4,#tmp4
    mov R5,#tmp1
    call mul_32bit ;tmp2 = (((temp_read>>4 - cal_T1)*(temp_read>>4 - cal_T1)) >> 12) * cal_T3
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it 
    mov R5,#tmp2
    mov R6,#14
    call shr_32bit ;tmp2 = ((((temp_read>>4 - cal_T1)*(temp_read>>4 - cal_T1)) >> 12) * cal_T3) >> 14
    mov R0,#tmp3
    mov R1,#tmp2
    call add_32bit ;tmp3 = t_fine = var1 + var2, see BMP280 datasheet
    mov R0,#tmp1
    call zero_32bit
    mov R0,#tmp1
    mov @R0,#5 ;tmp1 = 5
    mov R3,#temp_real
    mov R4,#tmp3
    mov R5,#tmp1
    call mul_32bit ;temp_real = t_fine * 5
    mov R0,#tmp1
    call zero_32bit
    mov R0,#tmp1
    mov @R0,#128 ;tmp1 = 128
    mov R0,#temp_real
    mov R1,#tmp1
    call add_32bit ;temp_real = t_fine * 5 + 128
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it 
    mov R5,#temp_real
    mov R6,#8
    call shr_32bit ;temp_real = (t_fine * 5 + 128) >> 8
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