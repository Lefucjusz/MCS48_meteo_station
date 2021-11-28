;MAB8049H, 10.000MHz
;Gdansk 2021
;TODO add uses/destroys
;TODO optimize RAM usage (overlap variables)
	.cr	8048
	.tf	rom.bin,BIN
	.lf	meteo.lst
;==================Defines=====================
;Pins
e   .eq %00001000 ;RS pin at P1.3
rs  .eq %00000100 ;E pin at P1.2

tx  .eq %10000000 ;Tx pin at P2.7
sda .eq %01000000 ;SDA pin at P2.6
scl .eq %00100000 ;SCL pin at P2.5

;BMP280 addresses
bmp280_wr_addr  .eq %11101100 ;0x76 << 1 | 0
bmp280_rd_addr  .eq %11101101 ;0x76 << 1 | 1

bmp280_cal_regs_size    .eq 24 ;12 registers 2 bytes each = 24 bytes
bmp280_dig_T1_LSB       .eq $88 ;First calibration register address
bmp280_pres_MSB         .eq $F7 ;First raw measurement register address

;RAM variables, little endian
dig_T1  .eq $20 ;16-bit
dig_T2  .eq $22 ;16-bit
dig_T3  .eq $24 ;16-bit
dig_P1  .eq $26 ;16-bit
dig_P2  .eq $28 ;16-bit
dig_P3  .eq $2A ;16-bit
dig_P4  .eq $2C ;16-bit
dig_P5  .eq $2E ;16-bit
dig_P6  .eq $30 ;16-bit
dig_P7  .eq $32 ;16-bit
dig_P8  .eq $34 ;16-bit
dig_P9  .eq $36 ;16-bit

temp_raw    .eq $38 ;32-bit
temp_real   .eq $3C ;32-bit
pres_raw    .eq $40 ;32-bit
pres_real   .eq $44 ;32-bit

tmp1    .eq $48 ;32-bit
tmp2    .eq $4C ;32-bit
tmp3    .eq $50 ;32-bit
tmp4    .eq $54 ;32-bit
tmp5    .eq $58 ;32-bit

num_ascii .eq $5C ;6 bytes

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
    call bmp280_configure
    call bmp280_get_cal_regs

    call lcd_init
    mov R3,#temp_text
    call lcd_string
    mov R0,#0
    mov R1,#1
    call lcd_gotoxy
    mov R3,#pres_text
    call lcd_string
loop:
    call bmp280_get_raw_meas
    call bmp280_compute
    call bmp280_display

    ; mov R1,#pres_real
    ; call uart_write_32bit

    ; mov R3,#deg_c_text
    ; call lcd_string
   
	jmp loop

;Constants
;DEBUG 1520 bytes before changes
;DEBUG 1446 after adding div10

;TODO - split in loop if possible

;Subroutines

;R0 - pointer to value to be zeroed, uses and destroys R0,R6
zero_32bit:
    clr A ;Clear A
    mov R6,#4 ;Load loop counter
zero_32bit_loop:
    mov @R0,A ;Load A = 0 to byte
    inc R0 ;Move pointer to next byte
    djnz R6,zero_32bit_loop ;Repeat for every byte
    ret

;R0 - pointer to value to be filled, uses and destroys R0,R6
fill_32bit:
    clr A ;Clear A
    cpl A ;Complement A
    mov R6,#4 ;Load loop counter
fill_32bit_loop:
    mov @R0,A ;Load A = FF to byte
    inc R0 ;Move pointer to next byte
    djnz R6,fill_32bit_loop ;Repeat for every byte
    ret

;F0 - signedness, if set, unsigned, R0 - pointer to destination, R1 - pointer to source, R7 - number of bytes to copy, uses F0,R0,R1,R5,R6,R7, destroys R0,R1,R4,R5,R6,R7
copy_32bit:
    >movr R5,R0 ;Preserve R0 in R5
    >movr R4,R1 ;Preserve R1 in R4
    jf0 copy_32bit_unsigned ;If flag set, unsigned
    mov A,R1 ;Load pointer to source to A
    add A,R7
    dec A ;Move pointer to MSB
    mov R1,A ;Store value back in register
    mov A,@R1 ;Load MSB to A
    cpl A ;Complement A
    jb7 copy_32bit_unsigned ;If sign bit not set, value is positive
    call fill_32bit ;Otherwise perform sign extension for signed value
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
    djnz R7,copy_32bit_loop
    ret

;R5 - pointer to value to be shifted, R6 - number of positions to shift, uses R0,R5,R6,R7, destroys R0,R6,R7
shl_32bit:
    clr C ;Clear carry bit
shlc_32bit:
    mov R1,#4 ;Load loop counter
    >movr R0,R5 ;Copy R5 to R0, so that R5 won't be changed in inner loop - needed to shift by multiple positions
shl_32bit_loop:
    mov A,@R0 ;Load byte from RAM to A
    rlc A ;Rotate A left through carry - C->A0, A7->C
    mov @R0,A ;Store result back in RAM
    inc R0 ;Move pointer to next byte
    djnz R1,shl_32bit_loop ;Repeat for all bytes
    djnz R6,shl_32bit ;Repeat the whole process required number of times
    ret

;F0 - signedness, if set, unsigned, R5 - pointer to value to be shifted, R6 - number of positions to shift, uses F0,R0,R5,R6,R7, destroys F0,R0,R6,R7
shr_32bit:
    clr C ;Clear carry bit
shrc_32bit:
    mov R1,#4 ;Load loop counter
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
    djnz R1,shr_32bit_loop ;Repeat for all bytes
    djnz R6,shr_32bit ;Repeat the whole operation required number of times
    ret

;R0 - pointer to first addend and result, R1 - pointer to second addend, uses and destroys R0,R1,R6
add_32bit:
    clr C ;Clear carry
    mov R6,#4 ;Load loop counter
add_32bit_loop:
    mov A,@R1 ;Load byte from RAM to A
    addc A,@R0 ;Add byte from second added
    mov @R0,A ;Store back in RAM
    inc R0 
    inc R1 ;Move both pointers to next byte
    djnz R6,add_32bit_loop ;Repeat for all bytes
    ret

;R0 - pointer to minuend and result, R1 - pointer to second subtrahend, uses and destroys R0,R1,R6
sub_32bit:
    clr C ;Clear carry
    mov R6,#4 ;Load loop counter
sub_32bit_loop: 
    mov A,@R0 ;A = [R0]
    cpl A ;A = -[R0]-1, that's how two's complement works
    addc A,@R1 ;A = -[R0]-1+[R1]+C
    cpl A ;A = -(-[R0]-1+[R1]+C)-1 = [R0]+1-[R1]-C-1 = [R0]-[R1]-C
    mov @R0,A ;[R0] = [R0]-[R1]-C
    inc R0
    inc R1
    djnz R6,sub_32bit_loop
    ret

;R3 - pointer to result, R4 - pointer to multiplicand, R5 - pointer to multiplier, uses F0,R0,R1,R2,R3,R4,R5,R6, destroys F0,R0,R1,R2,R6
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

;R3 - pointer to remainder, R4 - pointer to divisor, R5 - pointer to dividend and result, uses F1 and ALL registers, destroys F1,R0,R1,R2,R6,R7 TODO proper regs
div_32bit:
    >movr R0,R3 ;Load pointer to remainder to R0
    call zero_32bit ;Clear remainder
    mov R2,#32 ;Set loop counter
    clr C ;Clear carry
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
    call shlc_32bit ;Align result
div_32bit_end:
    ret

;R5 - pointer to variable to divide
div10_32bit:
    mov R0,#tmp2
    call zero_32bit ;Clear tmp2
    mov R0,#tmp2
    mov @R0,#$0A ;tmp2 = 0x0A = 10
    mov R3,#tmp3
    mov R4,#tmp2
    call div_32bit
    ret

;R5 - pointer to value to be split (max 6 digits)
split_32bit: 
    mov R7,#6 ;Load loop counter
split_32bit_loop:
    call div10_32bit ;temp_real = temp_real/10, tmp3 = temp_real%10
    mov A,#num_ascii ;Load pointer to result array to A
    add A,R7 ;Move pointer to proper position
    dec A ;Decrement 1 because array is indexed from 0 (sixth digit is at address+5)
    mov R0,A ;Store address in R0
    mov R1,#tmp3 ;Load pointer to digit to R1
    mov A,@R1 ;Load digit
    add A,#$30 ;Add ASCII code of '0'
    mov @R0,A ;Load digit to array
    djnz R7,split_32bit_loop
    ret

;R1 - pointer to value to write, uses R0,R1,R5,R6,R7
uart_write_32bit:
    mov R5,#4 ;Load loop counter
    mov A,R1 ;Load pointer to value to A
    add A,#3 ;Move pointer to MSB
    mov R1,A ;Store value in R1
uart_write_32bit_loop:
    mov A,@R1
    mov R0,A
    call uart_write_byte
    dec R1
    djnz R5,uart_write_32bit_loop
    ret
        
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


bmp280_compute:
    ;============ Temperature compensation ============
    mov R0,#tmp1
    mov R1,#temp_raw
    mov R7,#4
    call copy_32bit ;Copy 4 bytes, tmp1 = temp_raw
    clr F0 ;Perform signed shift
    mov R5,#tmp1
    mov R6,#3
    call shr_32bit ;tmp1 = temp_raw>>3
    cpl F0 ;Perform unsigned sign extension
    mov R0,#tmp2
    mov R1,#dig_T1
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp2 (32-bit) = dig_T1 (16-bit)    
    mov R5,#tmp2
    mov R6,#1
    call shl_32bit ;tmp2 = dig_T1<<1
    mov R0,#tmp1
    mov R1,#tmp2
    call sub_32bit ;tmp1 = temp_raw>>3 - dig_T1<<1
    clr F0 ;Perform signed sign extension
    mov R0,#tmp2
    mov R1,#dig_T2
    mov R7,#2
    call copy_32bit ;Copy 2 bytes, tmp2 = dig_T2
    mov R3,#tmp3
    mov R4,#tmp1
    mov R5,#tmp2
    call mul_32bit ;tmp3 = (temp_raw>>3 - dig_T1<<1) * dig_T2
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it
    mov R5,#tmp3
    mov R6,#11
    call shr_32bit ;tmp3 = ((temp_raw>>3 - dig_T1<<1) * dig_T2) >> 11
    mov R0,#tmp1
    mov R1,#temp_raw
    mov R7,#4
    call copy_32bit ;Copy 4 bytes, tmp1 = temp_raw
    ;Perform signed shift, flag already cleared
    mov R5,#tmp1
    mov R6,#4
    call shr_32bit ;tmp1 = temp_raw>>4
    cpl F0 ;Perform unsigned sign extension
    mov R0,#tmp2
    mov R1,#dig_T1
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp2 (32-bit) = dig_T1 (16-bit)
    mov R0,#tmp1
    mov R1,#tmp2
    call sub_32bit ;tmp1 = temp_raw>>4 - dig_T1
    mov R0,#tmp2
    mov R1,#tmp1 
    mov R7,#4
    call copy_32bit ;Copy 4 bytes, tmp2 = tmp1 = temp_raw>>4 - dig_T1
    mov R3,#tmp4
    mov R4,#tmp1
    mov R5,#tmp2
    call mul_32bit ;tmp4 = (temp_raw>>4 - dig_T1) * (temp_raw>>4 - dig_T1)
    clr F0 ;Perform signed shift
    mov R5,#tmp4
    mov R6,#12
    call shr_32bit ;tmp4 = ((temp_raw>>4 - dig_T1) * (temp_raw>>4 - dig_T1)) >> 12
    ;Perform signed sign extension, flag already cleared
    mov R0,#tmp1
    mov R1,#dig_T3
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp1 (32-bit) = dig_T3 (16-bit)
    mov R3,#tmp2
    mov R4,#tmp4
    mov R5,#tmp1
    call mul_32bit ;tmp2 = (((temp_raw>>4 - dig_T1)*(temp_raw>>4 - dig_T1)) >> 12) * dig_T3
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it 
    mov R5,#tmp2
    mov R6,#14
    call shr_32bit ;tmp2 = ((((temp_raw>>4 - dig_T1)*(temp_raw>>4 - dig_T1)) >> 12) * dig_T3) >> 14
    mov R0,#tmp3
    mov R1,#tmp2
    call add_32bit ;tmp3 = tmp3 + tmp2
    mov R0,#tmp4
    mov R1,#tmp3
    mov R7,#4
    call copy_32bit ;tmp4 = tmp3, this value is needed during pressure compensation (t_fine, see BMP280 datasheet) so preserve it
    ; mov R0,#tmp1
    ; call zero_32bit ;last time tmp1 was used as argument to mul_32bit, so it's already zeroed
    mov R0,#tmp1
    mov @R0,#5 ;tmp1 = 5
    mov R3,#temp_real
    mov R4,#tmp3
    mov R5,#tmp1
    call mul_32bit ;temp_real = t_fine * 5
    ; mov R0,#tmp1
    ; call zero_32bit ;last time tmp1 was used as argument to mul_32bit, so it's already zeroed
    mov R0,#tmp1
    mov @R0,#128 ;tmp1 = 128
    mov R0,#temp_real
    mov R1,#tmp1
    call add_32bit ;temp_real = t_fine * 5 + 128
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it 
    mov R5,#temp_real
    mov R6,#8
    call shr_32bit ;temp_real = (t_fine * 5 + 128) >> 8
    ;============ Pressure compensation ============
    ;Perform signed shift, flag already cleared
    mov R5,#tmp4
    mov R6,#1
    call shr_32bit ;tmp4 = tmp4 >> 1
    mov R0,#tmp1
    call zero_32bit ;Clear tmp1
    mov R0,#tmp1
    inc R0
    mov @R0,#$FA ;tmp1 = 0x0000FA00 = 64000
    mov R0,#tmp4
    mov R1,#tmp1
    call sub_32bit ;tmp4 = tmp4 - tmp1 = t_fine>>1 - 64000
    mov R0,#tmp1
    mov R1,#tmp4
    mov R7,#4
    call copy_32bit ;tmp1 = tmp4, preserve tmp4 as it is needed later
    ;Perform signed shift, flag already cleared
    mov R5,#tmp1
    mov R6,#2
    call shr_32bit ;tmp1 = tmp4>>2
    mov R0,#tmp2
    mov R1,#tmp1
    mov R7,#4
    call copy_32bit ;tmp2 = tmp1 = tmp4>>2
    mov R3,#tmp3
    mov R4,#tmp1
    mov R5,#tmp2
    call mul_32bit ;tmp3 = tmp1*tmp2 = (tmp4>>2)*(tmp4>>2)
    clr F0 ;Perform signed shift
    mov R5,#tmp3
    mov R6,#11
    call shr_32bit ;tmp3 = (tmp4>>2)*(tmp4>>2) >> 11
    ;Perform signed sign extension, flag already cleared
    mov R0,#tmp1
    mov R1,#dig_P6
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp1 (32-bit) = dig_P6 (16-bit)
    mov R3,#tmp2
    mov R4,#tmp3
    mov R5,#tmp1
    call mul_32bit ;tmp2 = ((tmp4>>2)*(tmp4>>2) >> 11)*dig_P6
    mov R0,#tmp1
    mov R1,#tmp4
    mov R7,#4
    call copy_32bit ;tmp1 = tmp4, tmp4 is still needed later
    clr F0 ;Perform signed sign extension, clear flag after mul_32bit has set it 
    mov R0,#tmp3
    mov R1,#dig_P5
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp3 (32-bit) = dig_P5 (16-bit)
    mov R3,#tmp5
    mov R4,#tmp1
    mov R5,#tmp3
    call mul_32bit ;tmp5 = tmp1*tmp3 = tmp4*dig_P5
    mov R5,#tmp5
    mov R6,#1
    call shl_32bit ;tmp5 = (tmp4*dig_P5)<<1
    mov R0,#tmp2
    mov R1,#tmp5
    call add_32bit ;tmp2 = tmp2 + (tmp4*dig_P5)<<1
    clr F0 ;Perform signed sign extension, clear flag after mul_32bit has set it
    mov R0,#tmp1
    mov R1,#dig_P4
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp1 (32-bit) = dig_P4 (16-bit)
    mov R5,#tmp1
    mov R6,#16
    call shl_32bit ;tmp1 = dig_P4<<16
    ;Perform signed shift, flag already cleared
    mov R5,#tmp2
    mov R6,#2
    call shr_32bit ;tmp2 = tmp2>>2
    mov R0,#tmp2
    mov R1,#tmp1
    call add_32bit ;tmp2 = tmp2>>2 + dig_P4<<16
    ;Perform signed sign extension, flag already cleared
    mov R0,#tmp1
    mov R1,#dig_P2
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp1 (32-bit) = dig_P2 (16-bit)
    mov R0,#tmp3
    mov R1,#tmp4
    mov R7,#4
    call copy_32bit ;tmp3 = tmp4, still needed...
    ; mov R3,#tmp5 ;Optimization - already loaded with tmp5
    mov R4,#tmp1
    mov R5,#tmp3
    call mul_32bit ; tmp5 = tmp1*tmp3 = dig_P2*tmp4
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it
    mov R5,#tmp5
    mov R6,#1
    call shr_32bit
    ;Perform signed shift, flag already cleared
    mov R5,#tmp4
    mov R6,#2
    call shr_32bit ;tmp4 = tmp4>>2
    mov R0,#tmp1
    mov R1,#tmp4
    mov R7,#4
    call copy_32bit ;tmp1 = tmp4
    mov R3,#tmp3
    mov R4,#tmp1
    mov R5,#tmp4
    call mul_32bit ;tmp3 = tmp1*tmp4 = (tmp4>>2)*(tmp4>>2)
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it
    mov R5,#tmp3
    mov R6,#13
    call shr_32bit ;tmp3 = tmp3>>13 = (tmp4>>2)*(tmp4>>2)>>13
    ;Perform signed sign extension, flag already cleared
    mov R0,#tmp1
    mov R1,#dig_P3
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp1 (32-bit) = dig_P3 (16-bit)
    mov R3,#tmp4
    mov R4,#tmp1
    mov R5,#tmp3
    call mul_32bit ;tmp4 = tmp1*tmp3 = dig_P3*((tmp4>>2)*(tmp4>>2)>>13)
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it
    mov R5,#tmp4
    mov R6,#3
    call shr_32bit ;tmp4 = (dig_P3*((tmp4>>2)*(tmp4>>2)>>13))>>3
    mov R0,#tmp4
    mov R1,#tmp5
    call add_32bit ;tmp4 = dig_P3*((tmp4>>2)*(tmp4>>2)>>13) + (dig_P2*tmp4)>>1
    ;mov R5,#tmp4 ;Optimization - already loaded with tmp4
    mov R6,#18
    call shr_32bit ;tmp4 = (dig_P3*((tmp4>>2)*(tmp4>>2)>>13) + (dig_P2*tmp4)>>1)>>18 (WTF Bosch, who the hell has come up with such equation...)
    ; mov R0,#tmp1
    ; call zero_32bit ;Clear tmp1 ;last time tmp1 was used as argument to mul_32bit, so it's already zeroed
    mov R0,#tmp1
    inc R0
    mov @R0,#$80 ;tmp1 = 0x00008000 = 32768
    mov R0,#tmp1
    mov R1,#tmp4
    call add_32bit ;tmp1 = tmp4 + 32768
    cpl F0 ;Perform unsigned sign extension, flag was cleared so set it
    mov R0,#tmp3
    mov R1,#dig_P1
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp3 (32-bit) = dig_P1 (16-bit)
    ; mov R3,#tmp4 ;Optimization - already loaded with tmp4
    mov R4,#tmp1
    mov R5,#tmp3
    call mul_32bit ;tmp4 = tmp1*tmp3 = tmp1*dig_P1
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it
    mov R5,#tmp4
    mov R6,#15
    call shr_32bit ;tmp4 = (tmp1*dig_P1)>>15
    ; mov R0,#tmp1
    ; call zero_32bit ;Clear tmp1 ;last time tmp1 was used as argument to mul_32bit, so it's already zeroed
    mov R0,#tmp1
    mov @R0,#1 ;tmp1 = 1
    mov R0,#tmp4
    mov R1,#tmp1
    call sub_32bit ;tmp4 = tmp4 - 1 to check if tmp4 == 0
    jnc bmp280_compute_not_zero ;If carry not set, tmp4 != 0
    mov R0,#pres_real
    call zero_32bit ;Set result to 0
    ret ;Abort further steps to prevent division by 0; first exit point
bmp280_compute_not_zero:
    mov R0,#tmp4
    mov R1,#tmp1
    call add_32bit ;Revert tmp4 state after subtraction
    mov R0,#tmp1
    call zero_32bit ;Clear tmp1
    mov R0,#tmp1
    inc R0
    inc R0
    mov @R0,#$10 ;tmp1 = 0x00100000 = 1048576
    mov R0,#tmp1
    mov R1,#pres_raw
    call sub_32bit ;tmp1 = 1048576 - pres_raw
    ;Perform signed shift, flag already cleared
    mov R5,#tmp2
    mov R6,#12
    call shr_32bit ;tmp2 = tmp2>>12
    mov R0,#tmp1
    mov R1,#tmp2
    call sub_32bit ;tmp1 = (1048576 - pres_raw) - (tmp2>>12)
    mov R0,#tmp2
    call zero_32bit ;Clear tmp2
    mov R0,#tmp2
    mov @R0,#$35
    inc R0
    mov @R0,#$0C ;tmp2 = 0x00000C35 = 3125
    mov R3,#pres_real
    ;mov R4,#tmp1 ;Optimization - already loaded with tmp1
    ;mov R5,#tmp2 ;Optimization - already loaded with tmp2
    call mul_32bit ;pres_real = ((1048576 - pres_raw) - (tmp2>>12))*3125
    mov R0,#pres_real ;Load pres_real pointer to R0
    mov A,R0 ;Load R0 to A
    add A,#3 ;Move pointer to MSB
    mov R0,A ;Load A to R0
    mov A,@R0 ;Load MSB to A
    jb7 bmp280_compute_msb_set ;If MSB set
    mov R5,#pres_real ;If MSB not set
    mov R6,#1
    call shl_32bit ;pres_real = pres_real<<1
    mov R3,#tmp1
    mov R4,#tmp4
    ; mov R5,#pres_real ;Optimization - already loaded with pres_real
    call div_32bit ;pres_real = pres_real/tmp4
    jmp bmp280_compute_continue
bmp280_compute_msb_set:
    ; mov R3,#tmp1 ;Optimization - already loaded with pres_real
    ; mov R4,#tmp4 ;Optimization - already loaded with tmp4
    ; mov R5,#pres_real ;Optimization - already loaded with pres_real
    call div_32bit ;pres_real = pres_real/tmp4
    ; mov R5,#pres_real ;Optimization - already loaded with pres_real
    mov R6,#1
    call shl_32bit ;pres_real = pres_real<<1
bmp280_compute_continue:
    mov R0,#tmp1
    mov R1,#pres_real
    mov R7,#4
    call copy_32bit ;tmp1 = pres_real, preserve pres_real as it is needed later
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it
    mov R5,#tmp1
    mov R6,#3
    call shr_32bit ;tmp1 = tmp1>>3 = pres_real>>3
    mov R0,#tmp2
    mov R1,#tmp1
    mov R7,#4
    call copy_32bit ;tmp2 = tmp1 = pres_real>>3
    mov R3,#tmp4
    mov R4,#tmp1
    mov R5,#tmp2
    call mul_32bit ;tmp4 = tmp1*tmp2 = (pres_real>>3)*(pres_real>>3)
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it
    mov R5,#tmp4
    mov R6,#13
    call shr_32bit ;tmp4 = ((pres_real>>3)*(pres_real>>3))>>13
    ;Perform signed sign extension, flag already cleared
    mov R0,#tmp1
    mov R1,#dig_P9
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp1 (32-bit) = dig_P9 (16-bit)
    mov R3,#tmp2
    mov R4,#tmp1
    mov R5,#tmp4
    call mul_32bit ;tmp2 = dig_P9*(((pres_real>>3)*(pres_real>>3))>>13)
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it
    mov R5,#tmp2
    mov R6,#12
    call shr_32bit ;tmp2 = (dig_P9*(((pres_real>>3)*(pres_real>>3))>>13))>>12, again great equation!
    mov R0,#tmp1
    mov R1,#pres_real
    mov R7,#4
    call copy_32bit ;tmp1 = pres_real, preserve pres_real as it is needed later
    ;Perform signed shift, flag already cleared
    mov R5,#tmp1
    mov R6,#2
    call shr_32bit ;tmp1 = tmp1>>2 = pres_real>>2
    ;Perform signed sign extension, flag already cleared
    mov R0,#tmp4
    mov R1,#dig_P8
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp1 (32-bit) = dig_P8 (16-bit)
    mov R3,#tmp5
    mov R4,#tmp1
    mov R5,#tmp4
    call mul_32bit ;tmp5 = tmp1*tmp4 = (pres_real>>2)*dig_P8
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it
    mov R5,#tmp5
    mov R6,#13
    call shr_32bit ;tmp5 = ((pres_real>>2)*dig_P8)>>13
    mov R0,#tmp2
    mov R1,#tmp5
    call add_32bit ;tmp2 = tmp2 + tmp5
    ;Perform signed sign extension, flag already cleared
    mov R0,#tmp1
    mov R1,#dig_P7
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp1 (32-bit) = dig_P7 (16-bit)
    mov R0,#tmp2
    mov R1,#tmp1
    call add_32bit ;tmp2 = tmp2 + dig_P7
    ;Perform signed shift, flag already cleared
    mov R5,#tmp2
    mov R6,#4
    call shr_32bit ;tmp2 = tmp2>>4
    mov R0,#pres_real
    mov R1,#tmp2
    call add_32bit ; pres_real = pres_real + tmp2
    ret


bmp280_configure:
    call i2c_start ;Start transmission	
	mov R0,#bmp280_wr_addr
	call i2c_write_byte ;Send BMP280 write address	
	mov R0,#0xF4
	call i2c_write_byte ;Set register pointer 

    mov R0,#%00100111
    call i2c_write_byte
    mov R0,#%11100000
    call i2c_write_byte

    call i2c_stop

    ret

bmp280_get_cal_regs:
    call i2c_start ;Start transmission	
	mov R0,#bmp280_wr_addr
	call i2c_write_byte ;Send BMP280 write address	
	mov R0,#bmp280_dig_T1_LSB
	call i2c_write_byte ;Set register pointer to first config register
    call i2c_start ;Restart transmission	
	mov R0,#bmp280_rd_addr
	call i2c_write_byte ;Send BMP280 read address

    mov R6,#bmp280_cal_regs_size-1 ;Load loop counter
    mov R1,#dig_T1 ;Load RAM pointer
    clr F0 ;Get size-1 bytes with ACK
read_dig_loop:
    call i2c_read_byte ;Read byte
    mov A,R0 
    mov @R1,A ;Store read byte at address pointer by R1
    inc R1 ;Move R1 to next address
    djnz R6,read_dig_loop ;Repeat size-1 times

    cpl F0 ;Get last byte with NACK 
    call i2c_read_byte ;Read byte
    mov A,R0
    mov @R1,A ;Store read byte at address pointer by R1
    ret

bmp280_get_raw_meas:
    mov R0,#pres_raw
    call zero_32bit ;Clear pres_raw
    mov R0,#temp_raw
    call zero_32bit ;Clear temp_raw
    mov R0,#tmp1
    call zero_32bit ;Clear tmp1
    mov R0,#tmp2
    call zero_32bit ;Clear tmp2

    call i2c_start ;Start transmission	
	mov R0,#bmp280_wr_addr
	call i2c_write_byte ;Send BMP280 write address	
	mov R0,#bmp280_pres_MSB
	call i2c_write_byte ;Set register pointer to first measurement register
    call i2c_start ;Restart transmission	
	mov R0,#bmp280_rd_addr
	call i2c_write_byte ;Send BMP280 read address
    clr F0 ;Send ACK
    call i2c_read_byte ;Read byte
    mov A,R0
    mov R0,#pres_raw
    mov @R0,A ;Load read byte to pres_raw
    call i2c_read_byte ;Read byte
    mov A,R0
    mov R0,#tmp1
    mov @R0,A ;Load read byte to tmp1
    call i2c_read_byte ;Read byte
    mov A,R0
    mov R0,#tmp2
    mov @R0,A ;Load read byte to tmp2
    mov R2,#pres_raw
    mov R3,#tmp1
    mov R4,#tmp2
    call bmp280_merge_reg ;Merge read values into one and store in pres_raw

    mov R0,#tmp1
    call zero_32bit ;Clear tmp1
    mov R0,#tmp2
    call zero_32bit ;Clear tmp2
    call i2c_read_byte ;Read byte
    mov A,R0
    mov R0,#temp_raw
    mov @R0,A ;Load read byte to temp_raw
    call i2c_read_byte ;Read byte
    mov A,R0
    mov R0,#tmp1
    mov @R0,A ;Load read byte to tmp1
    cpl F0 ;Send NACK
    call i2c_read_byte ;Read byte
    mov A,R0
    mov R0,#tmp2
    mov @R0,A ;Load read byte to tmp2
    mov R2,#temp_raw
    mov R3,#tmp1
    mov R4,#tmp2
    call bmp280_merge_reg ;Merge read values into one and store in temp_raw
    call i2c_stop ;Finish transmission
    ret

;R2 - pointer to MSB and result, R3 - pointer to LSB, R4 - pointer to XLSB
bmp280_merge_reg:
    >movr R5,R2 ;Load pointer to MSB to R5
    mov R6,#12
    call shl_32bit ;MSB = MSB<<12
    >movr R5,R3 ;Load pointer to LSB to R5
    mov R6,#4
    call shl_32bit ;LSB = LSB<<4
    >movr R0,R2
    >movr R1,R3
    call add_32bit ;MSB = MSB + LSB
    clr F0 ;Perform signed shift
    >movr R5,R4 ;Load pointer to XLSB to R5
    mov R6,#4
    call shr_32bit ;XLSB = XLSB>>4
    >movr R0,R2
    >movr R1,R4
    call add_32bit ;MSB = MSB + XLSB = MSB<<12 + LSB<<4 + XLSB>>4
    ret

bmp280_display:
    mov R0,#13
    mov R1,#0
    call lcd_gotoxy

    mov R5,#temp_real
    call split_32bit

    mov R4,#4
    mov R1,#num_ascii+2
bmp280_display_loop1:
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    inc R1
    djnz R4,bmp280_display_loop1

    mov R0,#10
    mov R1,#1
    call lcd_gotoxy

    mov R5,#pres_real
    call split_32bit

    mov R4,#6
    mov R1,#num_ascii
bmp280_display_loop2:
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    inc R1
    djnz R4,bmp280_display_loop2
    ret

    .ot
pres_text   .az /Pressure: /
temp_text   .az /Temperature: /
hpa_text    .az /hPa/
deg_c_text  .az #$DF,/C/

;R2 - pointer to string in ROM
lcd_string:
    mov R1,#1 ;Send data to LCD
lcd_string_loop:
    mov A,R3 ;Load string pointer to A
    movp A,@A ;Load char from ROM to A
    .ct
    jz lcd_string_end ;If end of the string - finish
    mov R0,A ;Load A to R0
    call lcd_write ;Send char to LCD
    inc R3 ;Move pointer to next char
    jmp lcd_string_loop ;Loop until end of the string
lcd_string_end:
    ret

;R0 - digit to be displayed
lcd_digit:
    mov A,R0
    add A,#$30 ;ASCII code for '0'
    mov R0,A
    mov R1,#1
    call lcd_write
    ret

;R0 - byte, R1 - cmd/data switch, uses R0,R1,R2
lcd_write:
	anl P1,#~rs ;Clear RS
	;Test whether data or cmd will be sent
	mov A,R1 ;Load R1 to A to test if zero
	jz skip_rs ;Skip RS line setting - cmd will be sent
	orl P1,#rs ;Set RS line - data will be sent
skip_rs:
	;Send upper nibble
    in A,P1 ;Load port state to A
    anl A,#%00001111 ;Mask upper nibble
    mov R2,A ;Store lower nibble in R2
	mov A,R0 ;Load byte to A
    anl A,#%11110000 ;Mask lower nibble
    orl A,R2 ;Add preserved lower nibble state
    outl P1,A ;Write A to P1
	
	orl P1,#e ;Set E line
	call delay_500us ;Wait for LCD	
	anl P1,#~e ;Clear E line
	call delay_500us ;Wait for LCD
	
	;Send lower nibble
	in A,P1 ;Load port state to A
    anl A,#%00001111 ;Mask upper nibble
    mov R2,A ;Store lower nibble in R2
	mov A,R0 ;Load byte to A
    swap A ;Swap nibbles
    anl A,#%11110000 ;Mask lower nibble
    orl A,R2 ;Add preserved lower nibble state
    outl P1,A ;Write A to P1
	
	orl P1,#e ;Set E line
	call delay_500us ;Wait for LCD	
	anl P1,#~e ;Clear E line
	call delay_500us ;Wait for LCD	
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

;==================I2C routines=================	
;No registers used
i2c_start:
	orl P2,#sda
	orl P2,#scl ;SDA = 1, SCL = 1 -> idle state
	anl P2,#~sda ;SDA = 1->0 while SCL = 1 -> START condition
	anl P2,#~scl ;SCL to zero
	ret
	
;No registers used	
i2c_stop:
	anl P2,#~sda ;SDA to zero
	orl P2,#scl ;SCL to one
	orl P2,#sda ;SDA to one - leave both lines in high state (bus idle)
	ret
	
;R0 - byte to be sent, uses R0,R7	
i2c_write_byte:
	mov R7,#8 ;Load bit counter
	mov A,R0 ;Load byte to be sent do A	
i2c_write_loop:
	jb7 i2c_write_one ;If MSB = 1, send one
	anl P2,#~sda ;Otherwise send zero -> SDA = 0
	jmp i2c_write_zero ;Skip part sending one
i2c_write_one:
	orl P2,#sda ;Send one -> SDA = 1
i2c_write_zero:
	orl P2,#scl ;SCL = 1
	anl P2,#~scl ;SCL = 0
	rl A ;Prepare next bit
	djnz R7,i2c_write_loop ;Repeat for all 8 bits	
	;Generate clock for ACK/NACK slave bit, but don't check its state
	orl P2,#scl ;SCL = 1
	anl P2,#~scl ;SCL = 0
	ret		

;R0 - received byte, F0 - ACK/NACK to be sent, uses F0,R0,R7
i2c_read_byte:
	mov R0,#0 ;Clear result
	mov R7,#8 ;Load bit counter
	orl P2,#sda ;SDA = 1
i2c_read_loop:
	orl P2,#scl ;SCL = 1	
	mov A,R0
	rl A
	mov R0,A ;Shift bits in result left	
	in A,P2
	anl A,#sda ;Obtain SDA line state
	jnz i2c_read_one ;If SDA == 1
	jmp i2c_read_zero ;If SDA == 0	
i2c_read_one: ;If SDA == 1...
	mov A,R0
	inc A
	mov R0,A ;...set last bit in result
i2c_read_zero: ;If SDA == 0 do nothing with result
	anl P2,#~scl ;SCL = 0
	djnz R7,i2c_read_loop ;Repeat for all 8 bits	
    ;Send ACK/NACK
	jf0 i2c_read_send_nack ;If requested to send NACK
	anl P2,#~sda ;If requested to send ACK, SDA = 0 -> send ACK
	jmp i2c_read_end 	
i2c_read_send_nack:
	orl P2,#sda ;SDA = 1 -> send NACK
i2c_read_end:
	orl P2,#scl ;SCL = 1
	anl P2,#~scl ;SCL = 0
	ret


