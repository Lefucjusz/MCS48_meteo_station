;MAB8049H, 10.000MHz
;Gdansk 2021
;TODO add uses/corrupts
;TODO optimize RAM usage (overlap variables)
;TODO check if result variables passed to mul are not double-zeroed
	.cr	8048
	.tf	rom.bin,BIN
	.lf	meteo.lst

;======================== Defines ========================
;Pins
uart_tx_pin .eq %00000010 ;UART Tx pin at P1.1
ow_pin      .eq %00000100 ;1-Wire pin at P1.2
dht11_pin   .eq %00001000 ;DHT11 data pin at P1.3
scl_pin     .eq %00010000 ;I2C SCL pin at P1.4
sda_pin     .eq %00100000 ;I2C SDA pin at P1.5
rs_pin      .eq %01000000 ;LCD E pin at P1.6
e_pin       .eq %10000000 ;LCD RS pin at P1.7

;BMP280
bmp280_wr_addr          .eq %11101100 ;0x76 << 1 | 0
bmp280_rd_addr          .eq %11101101 ;0x76 << 1 | 1
bmp280_cal_regs_size    .eq 24 ;12 registers 2 bytes each = 24 bytes
bmp280_dig_T1_LSB_reg   .eq $88 ;First calibration register address
bmp280_ctrl_meas_reg    .eq $F4 ;Measurement control register address
bmp280_config_reg       .eq $F5 ;Config register address
bmp280_pres_MSB_reg     .eq $F7 ;First raw measurement register address

;DS18b20
ds18b20_convert_t		.eq $44
ds18b20_read_scratchpad	.eq $BE
ds18b20_skip_rom		.eq $CC

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

tmp1    .eq $38 ;32-bit
tmp2    .eq $3C ;32-bit
tmp3    .eq $40 ;32-bit
tmp4    .eq $44 ;32-bit
tmp5    .eq $48 ;32-bit

bmp280_temp_raw     .eq $4C ;32-bit
bmp280_temp_real    .eq $50 ;32-bit
bmp280_pres_raw     .eq $54 ;32-bit
bmp280_pres_real    .eq $58 ;32-bit
dht11_humid         .eq $5C ;8-bit
ds18b20_temp_raw    .eq $5D ;16-bit

ascii_num .eq $5F ;6 bytes

;======================== Macros ========================
swap    .ma Rx,Ry ;Swaps two registers
        mov A,]1 ;Move Rx to A
        xch A,]2 ;Swap Ry with A
        mov ]1,A ;Store Ry in Rx
        .em

movr    .ma Rx,Ry ;Moves Ry to Rx (Rx = Ry)
        mov A,]2 ;Move Ry to A
        mov ]1,A ;Move A to Rx
        .em

;======================== Vectors ========================
	.no $00 ;Set jump to main at reset vector (00h)
	jmp main

;======================== Main ========================
main:
    call lcd_init
    call bmp280_write_config
    call bmp280_read_cal_regs
loop:
    call bmp280_read_raw_meas
    call bmp280_compute_values
    call disp_bmp

    call ds18b20_read_temp
    call ds18b20_value_to_ascii
    call disp_ds

    call dht11_read_rh
    call dht11_value_to_ascii
    call disp_dht

    mov R6,#250
    call delay_ms
    mov R6,#250
    call delay_ms

	jmp loop

;TODO add "corrupts"
;======================== 1-Wire routines ========================	
;No registers used
ow_reset:
	anl P1,#~ow_pin ;Clear 1-Wire pin
	call delay_500us ;Hold low for 500us
	orl P1,#ow_pin ;Set 1-Wire pin
	call delay_500us ;Wait for 500us for timeslot to end
	ret

;R0 - received byte, uses R0,R1,R7
ow_read_byte:
	mov R0,#0 ;Clear result
	mov R1,#8 ;Load bit loop counter
ow_read_loop:
	mov R7,#10 ;Load delay loop counter; ~3us
	;Shift result one bit right
	mov A,R0 ;~1.5us
	rr A ;~1.5us
	mov R0,A ;~1.5us	
	;Request read - 1-Wire pin >1us low
	anl P1,#~ow_pin ;Clear 1-Wire pin; ~3us
	nop ;Wait for ~1.5us
	orl P1,#ow_pin ;Set 1-Wire pin; ~3us
	;Read bit and complete 60us timeslot
	in A,P1 ;Read P1; ~3us
	anl A,#ow_pin ;Read 1-Wire pin; ~3us
	jz ow_read_zero ;~3us
ow_read_one:
	mov A,R0 ;~1.5us
	orl A,#%10000000 ;~3us
	mov R0,A ;Set bit in result; ~1.5us
ow_read_zero:
	djnz R7,ow_read_zero ;Wait for ~30us; ~3us	
	djnz R1,ow_read_loop ;Receive next bit; ~3us
	ret

;R0 - byte to be written, uses R0,R1,R7	
ow_write_byte:
	mov A,R0 ;Load byte to A
	cpl A ;Because of 8049 limitations - there's no jnbx instruction...
	mov R1,#8 ;Load bit loop counter
ow_write_loop:
	mov R7,#16 ;Load delay loop counter; ~3us
	anl P1,#~ow_pin ;Clear 1-Wire pin; ~3us
	jb0 ow_write_zero ;Check LSB, if not set - send zero; ~3us
ow_write_one:
	orl P1,#ow_pin ;Set 1-Wire pin; ~3us
ow_write_zero:
	djnz R7,ow_write_zero ;Wait for ~50us	
	orl P1,#ow_pin ;Set 1-Wire pin; ~3us
	rr A ;Shift byte one bit right; ~1.5us
	djnz R1,ow_write_loop ;Write next bit; ~3us
	ret

;======================== I2C routines ========================	
;No registers used
i2c_start:
	orl P1,#sda_pin
	orl P1,#scl_pin ;SDA = 1, SCL = 1 -> idle state
	anl P1,#~sda_pin ;SDA = 1->0 while SCL = 1 -> START condition
	anl P1,#~scl_pin ;SCL to zero
	ret
	
;No registers used	
i2c_stop:
	anl P1,#~sda_pin ;SDA to zero
	orl P1,#scl_pin ;SCL to one
	orl P1,#sda_pin ;SDA to one - leave both lines in high state (bus idle)
	ret
	
;R0 - byte to be sent, uses R0,R7, corrupts R7
i2c_write_byte:
	mov R7,#8 ;Load bit counter
	mov A,R0 ;Load byte to be sent do A	
i2c_write_loop:
	jb7 i2c_write_one ;If MSB = 1, send one
	anl P1,#~sda_pin ;Otherwise send zero -> SDA = 0
	jmp i2c_write_zero ;Skip part sending one
i2c_write_one:
	orl P1,#sda_pin ;Send one -> SDA = 1
i2c_write_zero:
	orl P1,#scl_pin ;SCL = 1
	anl P1,#~scl_pin ;SCL = 0
	rl A ;Prepare next bit
	djnz R7,i2c_write_loop ;Repeat for all 8 bits	
	;Generate clock for ACK/NACK slave bit, but don't check its state
	orl P1,#scl_pin ;SCL = 1
	anl P1,#~scl_pin ;SCL = 0
	ret		

;R0 - received byte, F0 - ACK/NACK to be sent, uses F0,R0,R7, corrupts R0,R7
i2c_read_byte:
	mov R0,#0 ;Clear result
	mov R7,#8 ;Load bit counter
	orl P1,#sda_pin ;SDA = 1
i2c_read_loop:
	orl P1,#scl_pin ;SCL = 1	
	mov A,R0
	rl A
	mov R0,A ;Shift bits in result left	
	in A,P1
	anl A,#sda_pin ;Obtain SDA line state
	jnz i2c_read_one ;If SDA == 1
	jmp i2c_read_zero ;If SDA == 0	
i2c_read_one: ;If SDA == 1...
	mov A,R0
	inc A
	mov R0,A ;...set last bit in result
i2c_read_zero: ;If SDA == 0 do nothing with result
	anl P1,#~scl_pin ;SCL = 0
	djnz R7,i2c_read_loop ;Repeat for all 8 bits	
    ;Send ACK/NACK
	jf0 i2c_read_send_nack ;If requested to send NACK
	anl P1,#~sda_pin ;If requested to send ACK, SDA = 0 -> send ACK
	jmp i2c_read_end 	
i2c_read_send_nack:
	orl P1,#sda_pin ;SDA = 1 -> send NACK
i2c_read_end:
	orl P1,#scl_pin ;SCL = 1
	anl P1,#~scl_pin ;SCL = 0
	ret

;======================== LCD routines ========================
;R0 - byte, R1 - cmd/data switch, uses R0,R1, corrupts nothing
lcd_write:
	anl P1,#~rs_pin ;Clear RS
	;Test whether data or cmd will be sent
	mov A,R1 ;Load R1 to A to test if zero
	jz skip_rs ;Skip RS pin setting - cmd will be sent
	orl P1,#rs_pin ;Set RS pin - data will be sent
skip_rs:
	;Send upper nibble
	mov A,R0 ;Load byte to A
    outl P2,A ;Write A to P2
	orl P1,#e_pin ;Set E pin	
	anl P1,#~e_pin ;Clear E pin
	;Send lower nibble
    swap A ;Swap nibbles
    outl P2,A ;Write A to P2
	orl P1,#e_pin ;Set E pin	
	anl P1,#~e_pin ;Clear E pin	
	ret
	
;Uses and corrupts R0,R1,R6,R7	
lcd_cls:
	mov R1,#0 ;Send commands
	mov R0,#$01	
	call lcd_write ;Clear display
	mov R6,#1
	call delay_ms ;Wait 1ms
	mov R0,#$80
	call lcd_write ;Set cursor at first place in upper row
	mov R6,#1
	call delay_ms ;Wait 1ms
	ret

;R1 - row, R0 - column, uses and corrupts R0,R1	
lcd_gotoxy:
	mov A,R1
	jnz lcd_gotoxy_sec_row ;Check row
	mov A,#$80 ;If first, load address of its first position
	jmp lcd_gotoxy_write
lcd_gotoxy_sec_row:
	mov A,#$C0 ;If second, load address of its first position
lcd_gotoxy_write:
	add A,R0 ;Add offset (y)
	mov R0,A
	mov R1,#0
	call lcd_write ;Send command
	ret
	
;Uses and corrupts R0,R1,R6,R7	
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

;======================== 32-bit math routines ========================
;R0 - pointer to value to be zeroed, uses and corrupts R0,R6
zero_32bit:
    clr A ;Clear A
    mov R6,#4 ;Load loop counter
zero_32bit_loop:
    mov @R0,A ;Load A = 0 to byte
    inc R0 ;Move pointer to next byte
    djnz R6,zero_32bit_loop ;Repeat for every byte
    ret

;R0 - pointer to value to be filled, uses and corrupts R0,R6
fill_32bit:
    clr A ;Clear A
    cpl A ;Complement A
    mov R6,#4 ;Load loop counter
fill_32bit_loop:
    mov @R0,A ;Load A = FF to byte
    inc R0 ;Move pointer to next byte
    djnz R6,fill_32bit_loop ;Repeat for every byte
    ret

;F0 - signedness, if set, unsigned, R0 - pointer to destination, R1 - pointer to source, R7 - number of bytes to copy, uses F0,R0,R1,R4,R5,R6,R7, corrupts R0,R1,R4,R5,R6,R7
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

;R5 - pointer to value to be shifted, R6 - number of positions to shift, uses R0,R1,R5,R6, corrupts R0,R1,R6
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

;F0 - signedness, if set, unsigned, R5 - pointer to value to be shifted, R6 - number of positions to shift, uses F0,R0,R1,R5,R6, corrupts F0,R0,R1,R6
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

;R0 - pointer to first addend and result, R1 - pointer to second addend, uses and corrupts R0,R1,R6
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

;R0 - pointer to minuend and result, R1 - pointer to second subtrahend, uses and corrupts R0,R1,R6
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

;R3 - pointer to result, R4 - pointer to multiplicand, R5 - pointer to multiplier, uses F0,R0,R1,R2,R3,R4,R5,R6, corrupts F0,R0,R1,R2,R6
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

;R3 - pointer to remainder, R4 - pointer to divisor, R5 - pointer to dividend and result, uses R0,R1,R2,R3,R4,R5,R6, corrupts R0,R1,R2,R6
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

;R5 - pointer to variable to divide, uses R0,R1,R2,R3,R4,R5,R6, corrupts R0,R1,R2,R3,R4,R6
div10_32bit:
    mov R0,#tmp2
    call zero_32bit ;Clear tmp2
    mov R0,#tmp2
    mov @R0,#$0A ;tmp2 = 0x0A = 10
    mov R3,#tmp3
    mov R4,#tmp2
    call div_32bit
    ret

;R5 - pointer to value to be split (6 digits max) uses ALL registers, corrupts R0,R1,R2,R3,R4,R6,R7
split_32bit:
    mov R7,#6
split_32bit_loop:
    call div10_32bit ;bmp280_temp_real = bmp280_temp_real/10, tmp3 = bmp280_temp_real%10
    mov A,#ascii_num ;Load pointer to result array to A
    add A,R7 ;Move pointer to proper position
    dec A ;Decrement 1 because array is indexed from 0 (n-th digit is at address+(n-1))
    mov R0,A ;Store address in R0
    mov R1,#tmp3 ;Load pointer to digit to R1
    mov A,@R1 ;Load digit
    add A,#$30 ;Add ASCII code of '0'
    mov @R0,A ;Load digit to array
    djnz R7,split_32bit_loop
    ret

;======================== Delay routines ========================
;~100uS delay, uses and corrupts R7
delay_100us:
	mov R7,#28
delay_100us_loop:
	djnz R7,delay_100us_loop
	ret

;~500uS delay, uses and corrupts R7
delay_500us:
	mov R7,#164
delay_500us_loop:
	djnz R7,delay_500us_loop
	ret
   
;R6 - delay time in msec, uses and corrupts R6,R7
delay_ms:
	mov R7,#228
delay_ms_loop:
	nop
	djnz R7,delay_ms_loop
	djnz R6,delay_ms
	ret 

;======================== BMP280 routines ========================
;Uses and corrupts R0,R7
bmp280_write_config:
    call i2c_start ;Start transmission	
	mov R0,#bmp280_wr_addr
	call i2c_write_byte ;Send BMP280 write address	
	mov R0,#bmp280_ctrl_meas_reg
	call i2c_write_byte ;Set register pointer
    mov R0,#%00100111 ;Toversampling = 1x, Poversampling = 1x, power mode = normal
    call i2c_write_byte
    mov R0,#%11101000 ;Tstandby = 4000ms, IIR filter coefficient = 2, 3-wire SPI turned off
    call i2c_write_byte
    call i2c_stop
    ret

;Uses and corrupts F0,R0,R1,R6,R7
bmp280_read_cal_regs:
    call i2c_start ;Start transmission	
	mov R0,#bmp280_wr_addr
	call i2c_write_byte ;Send BMP280 write address	
	mov R0,#bmp280_dig_T1_LSB_reg
	call i2c_write_byte ;Set register pointer to first config register
    call i2c_start ;Restart transmission	
	mov R0,#bmp280_rd_addr
	call i2c_write_byte ;Send BMP280 read address
    mov R6,#bmp280_cal_regs_size-1 ;Load loop counter
    mov R1,#dig_T1 ;Load RAM pointer
    clr F0 ;Get size-1 bytes with ACK
read_cal_loop:
    call i2c_read_byte ;Read byte
    mov A,R0 
    mov @R1,A ;Store read byte at address pointed by R1
    inc R1 ;Move R1 to next address
    djnz R6,read_cal_loop ;Repeat size-1 times
    cpl F0 ;Get last byte with NACK 
    call i2c_read_byte ;Read byte
    mov A,R0
    mov @R1,A ;Store read byte at address pointed by R1
    ret

;Uses and corrupts F0 and ALL registers
bmp280_read_raw_meas:
    mov R0,#bmp280_pres_raw
    call zero_32bit ;Clear bmp280_pres_raw
    mov R0,#bmp280_temp_raw
    call zero_32bit ;Clear bmp280_temp_raw
    mov R0,#tmp1
    call zero_32bit ;Clear tmp1
    mov R0,#tmp2
    call zero_32bit ;Clear tmp2

    call i2c_start ;Start transmission	
	mov R0,#bmp280_wr_addr
	call i2c_write_byte ;Send BMP280 write address	
	mov R0,#bmp280_pres_MSB_reg
	call i2c_write_byte ;Set register pointer to first measurement register
    call i2c_start ;Restart transmission	
	mov R0,#bmp280_rd_addr
	call i2c_write_byte ;Send BMP280 read address
    clr F0 ;Send ACK
    call i2c_read_byte ;Read byte
    mov A,R0
    mov R0,#bmp280_pres_raw
    mov @R0,A ;Load read byte to bmp280_pres_raw
    call i2c_read_byte ;Read byte
    mov A,R0
    mov R0,#tmp1
    mov @R0,A ;Load read byte to tmp1
    call i2c_read_byte ;Read byte
    mov A,R0
    mov R0,#tmp2
    mov @R0,A ;Load read byte to tmp2
    mov R2,#bmp280_pres_raw
    mov R3,#tmp1
    mov R4,#tmp2
    call bmp280_merge_reg ;Merge read values into one and store in bmp280_pres_raw

    mov R0,#tmp1
    call zero_32bit ;Clear tmp1
    mov R0,#tmp2
    call zero_32bit ;Clear tmp2
    call i2c_read_byte ;Read byte
    mov A,R0
    mov R0,#bmp280_temp_raw
    mov @R0,A ;Load read byte to bmp280_temp_raw
    call i2c_read_byte ;Read byte
    mov A,R0
    mov R0,#tmp1
    mov @R0,A ;Load read byte to tmp1
    cpl F0 ;Send NACK
    call i2c_read_byte ;Read byte
    mov A,R0
    mov R0,#tmp2
    mov @R0,A ;Load read byte to tmp2
    mov R2,#bmp280_temp_raw
    mov R3,#tmp1
    mov R4,#tmp2
    call bmp280_merge_reg ;Merge read values into one and store in bmp280_temp_raw
    call i2c_stop ;Finish transmission
    ret

;R2 - pointer to MSB and result, R3 - pointer to LSB, R4 - pointer to XLSB, uses F0,R0,R1,R2,R3,R4,R5,R6, corrupts F0,R0,R1,R5,R6
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

;Uses and corrupts F0 and ALL registers 
bmp280_compute_values:
    ;============ Temperature compensation ============
    mov R0,#tmp1
    mov R1,#bmp280_temp_raw
    mov R7,#4
    call copy_32bit ;Copy 4 bytes, tmp1 = bmp280_temp_raw
    clr F0 ;Perform signed shift
    mov R5,#tmp1
    mov R6,#3
    call shr_32bit ;tmp1 = bmp280_temp_raw>>3
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
    call sub_32bit ;tmp1 = bmp280_temp_raw>>3 - dig_T1<<1
    clr F0 ;Perform signed sign extension
    mov R0,#tmp2
    mov R1,#dig_T2
    mov R7,#2
    call copy_32bit ;Copy 2 bytes, tmp2 = dig_T2
    mov R3,#tmp3
    mov R4,#tmp1
    mov R5,#tmp2
    call mul_32bit ;tmp3 = (bmp280_temp_raw>>3 - dig_T1<<1) * dig_T2
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it
    mov R5,#tmp3
    mov R6,#11
    call shr_32bit ;tmp3 = ((bmp280_temp_raw>>3 - dig_T1<<1) * dig_T2) >> 11
    mov R0,#tmp1
    mov R1,#bmp280_temp_raw
    mov R7,#4
    call copy_32bit ;Copy 4 bytes, tmp1 = bmp280_temp_raw
    ;Perform signed shift, flag already cleared
    mov R5,#tmp1
    mov R6,#4
    call shr_32bit ;tmp1 = bmp280_temp_raw>>4
    cpl F0 ;Perform unsigned sign extension
    mov R0,#tmp2
    mov R1,#dig_T1
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp2 (32-bit) = dig_T1 (16-bit)
    mov R0,#tmp1
    mov R1,#tmp2
    call sub_32bit ;tmp1 = bmp280_temp_raw>>4 - dig_T1
    mov R0,#tmp2
    mov R1,#tmp1 
    mov R7,#4
    call copy_32bit ;Copy 4 bytes, tmp2 = tmp1 = bmp280_temp_raw>>4 - dig_T1
    mov R3,#tmp4
    mov R4,#tmp1
    mov R5,#tmp2
    call mul_32bit ;tmp4 = (bmp280_temp_raw>>4 - dig_T1) * (bmp280_temp_raw>>4 - dig_T1)
    clr F0 ;Perform signed shift
    mov R5,#tmp4
    mov R6,#12
    call shr_32bit ;tmp4 = ((bmp280_temp_raw>>4 - dig_T1) * (bmp280_temp_raw>>4 - dig_T1)) >> 12
    ;Perform signed sign extension, flag already cleared
    mov R0,#tmp1
    mov R1,#dig_T3
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp1 (32-bit) = dig_T3 (16-bit)
    mov R3,#tmp2
    mov R4,#tmp4
    mov R5,#tmp1
    call mul_32bit ;tmp2 = (((bmp280_temp_raw>>4 - dig_T1)*(bmp280_temp_raw>>4 - dig_T1)) >> 12) * dig_T3
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it 
    mov R5,#tmp2
    mov R6,#14
    call shr_32bit ;tmp2 = ((((bmp280_temp_raw>>4 - dig_T1)*(bmp280_temp_raw>>4 - dig_T1)) >> 12) * dig_T3) >> 14
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
    mov R3,#bmp280_temp_real
    mov R4,#tmp3
    mov R5,#tmp1
    call mul_32bit ;bmp280_temp_real = t_fine * 5
    ; mov R0,#tmp1
    ; call zero_32bit ;last time tmp1 was used as argument to mul_32bit, so it's already zeroed
    mov R0,#tmp1
    mov @R0,#128 ;tmp1 = 128
    mov R0,#bmp280_temp_real
    mov R1,#tmp1
    call add_32bit ;bmp280_temp_real = t_fine * 5 + 128
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it 
    mov R5,#bmp280_temp_real
    mov R6,#8
    call shr_32bit ;bmp280_temp_real = (t_fine * 5 + 128) >> 8
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
    mov R0,#bmp280_pres_real
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
    mov R1,#bmp280_pres_raw
    call sub_32bit ;tmp1 = 1048576 - bmp280_pres_raw
    ;Perform signed shift, flag already cleared
    mov R5,#tmp2
    mov R6,#12
    call shr_32bit ;tmp2 = tmp2>>12
    mov R0,#tmp1
    mov R1,#tmp2
    call sub_32bit ;tmp1 = (1048576 - bmp280_pres_raw) - (tmp2>>12)
    mov R0,#tmp2
    call zero_32bit ;Clear tmp2
    mov R0,#tmp2
    mov @R0,#$35
    inc R0
    mov @R0,#$0C ;tmp2 = 0x00000C35 = 3125
    mov R3,#bmp280_pres_real
    ;mov R4,#tmp1 ;Optimization - already loaded with tmp1
    ;mov R5,#tmp2 ;Optimization - already loaded with tmp2
    call mul_32bit ;bmp280_pres_real = ((1048576 - bmp280_pres_raw) - (tmp2>>12))*3125
    mov R0,#bmp280_pres_real ;Load bmp280_pres_real pointer to R0
    mov A,R0 ;Load R0 to A
    add A,#3 ;Move pointer to MSB
    mov R0,A ;Load A to R0
    mov A,@R0 ;Load MSB to A
    jb7 bmp280_compute_msb_set ;If MSB set
    mov R5,#bmp280_pres_real ;If MSB not set
    mov R6,#1
    call shl_32bit ;bmp280_pres_real = bmp280_pres_real<<1
    mov R3,#tmp1
    mov R4,#tmp4
    ; mov R5,#bmp280_pres_real ;Optimization - already loaded with bmp280_pres_real
    call div_32bit ;bmp280_pres_real = bmp280_pres_real/tmp4
    jmp bmp280_compute_continue
bmp280_compute_msb_set:
    ; mov R3,#tmp1 ;Optimization - already loaded with bmp280_pres_real
    ; mov R4,#tmp4 ;Optimization - already loaded with tmp4
    ; mov R5,#bmp280_pres_real ;Optimization - already loaded with bmp280_pres_real
    call div_32bit ;bmp280_pres_real = bmp280_pres_real/tmp4
    ; mov R5,#bmp280_pres_real ;Optimization - already loaded with bmp280_pres_real
    mov R6,#1
    call shl_32bit ;bmp280_pres_real = bmp280_pres_real<<1
bmp280_compute_continue:
    mov R0,#tmp1
    mov R1,#bmp280_pres_real
    mov R7,#4
    call copy_32bit ;tmp1 = bmp280_pres_real, preserve bmp280_pres_real as it is needed later
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it
    mov R5,#tmp1
    mov R6,#3
    call shr_32bit ;tmp1 = tmp1>>3 = bmp280_pres_real>>3
    mov R0,#tmp2
    mov R1,#tmp1
    mov R7,#4
    call copy_32bit ;tmp2 = tmp1 = bmp280_pres_real>>3
    mov R3,#tmp4
    mov R4,#tmp1
    mov R5,#tmp2
    call mul_32bit ;tmp4 = tmp1*tmp2 = (bmp280_pres_real>>3)*(bmp280_pres_real>>3)
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it
    mov R5,#tmp4
    mov R6,#13
    call shr_32bit ;tmp4 = ((bmp280_pres_real>>3)*(bmp280_pres_real>>3))>>13
    ;Perform signed sign extension, flag already cleared
    mov R0,#tmp1
    mov R1,#dig_P9
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp1 (32-bit) = dig_P9 (16-bit)
    mov R3,#tmp2
    mov R4,#tmp1
    mov R5,#tmp4
    call mul_32bit ;tmp2 = dig_P9*(((bmp280_pres_real>>3)*(bmp280_pres_real>>3))>>13)
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it
    mov R5,#tmp2
    mov R6,#12
    call shr_32bit ;tmp2 = (dig_P9*(((bmp280_pres_real>>3)*(bmp280_pres_real>>3))>>13))>>12, again great equation!
    mov R0,#tmp1
    mov R1,#bmp280_pres_real
    mov R7,#4
    call copy_32bit ;tmp1 = bmp280_pres_real, preserve bmp280_pres_real as it is needed later
    ;Perform signed shift, flag already cleared
    mov R5,#tmp1
    mov R6,#2
    call shr_32bit ;tmp1 = tmp1>>2 = bmp280_pres_real>>2
    ;Perform signed sign extension, flag already cleared
    mov R0,#tmp4
    mov R1,#dig_P8
    mov R7,#2
    call copy_32bit ;Copy 2 bytes - tmp1 (32-bit) = dig_P8 (16-bit)
    mov R3,#tmp5
    mov R4,#tmp1
    mov R5,#tmp4
    call mul_32bit ;tmp5 = tmp1*tmp4 = (bmp280_pres_real>>2)*dig_P8
    clr F0 ;Perform signed shift, clear flag after mul_32bit has set it
    mov R5,#tmp5
    mov R6,#13
    call shr_32bit ;tmp5 = ((bmp280_pres_real>>2)*dig_P8)>>13
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
    mov R0,#bmp280_pres_real
    mov R1,#tmp2
    call add_32bit ; bmp280_pres_real = bmp280_pres_real + tmp2
    ret

;======================== DHT11 routines ========================
;R0 - value of RH in percent, uses and corrupts R0,R6,R7 TODO write value to RAM
dht11_read_rh:
    mov R0,#0
    anl P1,#~dht11_pin
    mov R6,#20
    call delay_ms ;Pull DHT11 bus low for at least 18ms (20ms here to be sure)
    orl P1,#dht11_pin ;Release bus
    call delay_100us 
    call delay_100us ;Wait for 200us - 40us + 80us + 80us, see datasheet; ignore sensor response
    mov R6,#8 ;Load loop counter
dht11_read_rh_loop:
    mov A,R0
    rl A
    mov R0,A ;R0 = R0<<1
    mov R7,#22 ;Load delay loop counter
dht11_read_rh_delay:
    djnz R7,dht11_read_rh_delay ;Wait for ~70us
    in A,P1 ;Load port state to A
    anl A,#dht11_pin ;Mask all pins except for DHT11 bus
    jnz dht11_read_one ;If '1' bit was received
    jmp dht11_read_rh_continue ; If '0' bit was received
dht11_read_one:
    inc R0 ;R0 = R0+1 (= R0|1)
dht11_sync_loop:
    in A,P1 ;Load port state to A
    anl A,#dht11_pin ;Mask all pins except for DHT11 bus
    jnz dht11_sync_loop ;Self-synchronize timings - on every '1' bit received, wait for start-of-transmission bit
dht11_read_rh_continue:
    djnz R6,dht11_read_rh_loop ;Repeat for all bits
    mov R1,#dht11_humid
    mov A,R0
    mov @R1,A ;Store read value in dht11_humid
    ret

;TODO uses
dht11_value_to_ascii:
    mov R0,#tmp1
    mov R1,#dht11_humid
    mov R7,#1
    call copy_32bit

    mov R5,#tmp1
    call split_32bit ;Split value to digits
    ret

;======================== DS18b20 routines ========================
;Uses R0,R1,R2,R3,R6,R7 TODO add uses
ds18b20_read_temp:
	call ow_reset ;Send bus reset condition
	mov R0,#ds18b20_skip_rom 
	call ow_write_byte ;Send skip ROM command
	mov R0,#ds18b20_read_scratchpad 
	call ow_write_byte ;Send read scratchpad command

	call ow_read_byte ;Read temperature LSB
    mov R1,#ds18b20_temp_raw ;TODO use R0 in place of R1
    mov A,R0
    mov @R1,A ;Load LSB to first byte of ds18b20_temp_raw
    
    call ow_read_byte ;Read temperature MSB
    mov R1,#ds18b20_temp_raw+1 ;TODO use R0 in place of R1
    mov A,R0
    mov @R1,A ;Load MSB to first byte of ds18b20_temp_raw

	call ow_reset ;Send bus reset condition
	mov R0,#ds18b20_skip_rom ;Skip ROM
	call ow_write_byte
	mov R0,#ds18b20_convert_t ;Convert temp - prepare to read next time
	call ow_write_byte
	ret	

;TODO add uses, add negative numbers support
ds18b20_value_to_ascii:
    mov R0,#tmp2
    mov R1,#ds18b20_temp_raw
    mov R7,#2
    call copy_32bit ;Copy 2 bytes

    mov R0,#tmp3
    call zero_32bit ;Clear tmp3

    mov R0,#tmp3
    mov @R0,#$19 ;tmp1 = 0x19 = 25

    mov R3,#tmp1
    mov R4,#tmp2
    mov R5,#tmp3
    call mul_32bit ;tmp2 = ds18b20_temp_raw*25

    mov R5,#tmp1
    mov R6,#2
    call shr_32bit ;tmp2 = (ds18b20_temp_raw*25)>>2 = ds18b20_temp_raw*(25/4)
    call split_32bit
    ret

disp_bmp:
    mov R0,#0
    mov R1,#0
    call lcd_gotoxy

    mov R5,#bmp280_temp_real
    call split_32bit

    mov R1,#1
    mov R0,#'I'
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#':'
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#' '
    call lcd_write ;TODO no stupid trick with R1
    mov R1,#ascii_num+2
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    inc R1
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#'.'
    call lcd_write ;TODO no stupid trick with R1
    inc R1
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    inc R1
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#'C'
    call lcd_write ;TODO no stupid trick with R1


    mov R0,#0
    mov R1,#1
    call lcd_gotoxy

    mov R5,#bmp280_pres_real
    call split_32bit

    mov R1,#1
    mov R0,#'P'
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#':'
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#' '
    call lcd_write ;TODO no stupid trick with R1
    mov R1,#ascii_num
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    inc R1
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    inc R1
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    inc R1
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#'.'
    call lcd_write ;TODO no stupid trick with R1
    inc R1
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    inc R1
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#'h'
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#'P'
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#'a'
    call lcd_write ;TODO no stupid trick with R1
    ret

disp_ds:
    mov R0,#11
    mov R1,#0
    call lcd_gotoxy

    mov R1,#1
    mov R0,#'O'
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#':'
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#' '
    call lcd_write ;TODO no stupid trick with R1
    mov R1,#ascii_num+2
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    inc R1
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#'.'
    call lcd_write ;TODO no stupid trick with R1
    inc R1
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    inc R1
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#'C'
    call lcd_write ;TODO no stupid trick with R1
    ret

disp_dht:
    mov R0,#14
    mov R1,#1
    call lcd_gotoxy

    mov R1,#1
    mov R0,#'H'
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#':'
    call lcd_write ;TODO no stupid trick with R1
    mov R0,#' '
    call lcd_write ;TODO no stupid trick with R1

    mov R4,#2
    mov R1,#ascii_num+4
disp_loop:
    mov A,@R1
    mov R0,A
    call lcd_write ;TODO no stupid trick with R1
    inc R1
    djnz R4,disp_loop
    mov R0,#'%'
    call lcd_write ;TODO no stupid trick with R1
    ret

; ;R0 - byte to send, uses R0,R6,R7
; uart_write_byte:
; 	mov R6,#8 ;Load bit counter	
; 	mov A,R0 ;Move byte to be send to A	
; 	anl P1,#~uart_tx_pin ;Set tx pin low - start bit
; 	call delay_100us
; uart_write_loop:
; 	jb0 uart_write_one ;Check if LSB of A is set
; 	anl P1,#~uart_tx_pin ;Set tx pin low
; 	jmp uart_write_delay	
; uart_write_one:
; 	orl P1,#uart_tx_pin ;Set tx pin high
; uart_write_delay:
; 	call delay_100us
; 	rr A ;Shift byte one bit right
; 	djnz R6,uart_write_loop

; 	orl P1,#uart_tx_pin ;Set tx pin high - stop bit
; 	call delay_100us
; 	ret

; bmp280_display:
;     mov R0,#13
;     mov R1,#0
;     call lcd_gotoxy

;     mov R5,#bmp280_temp_real
;     call split_32bit

;     mov R4,#4
;     mov R1,#ascii_num+2
; bmp280_display_loop1:
;     mov A,@R1
;     mov R0,A
;     call lcd_write ;TODO no stupid trick with R1
;     inc R1
;     djnz R4,bmp280_display_loop1

;     mov R0,#10
;     mov R1,#1
;     call lcd_gotoxy

;     mov R5,#bmp280_pres_real
;     call split_32bit

;     mov R4,#6
;     mov R1,#ascii_num
; bmp280_display_loop2:
;     mov A,@R1
;     mov R0,A
;     call lcd_write ;TODO no stupid trick with R1
;     inc R1
;     djnz R4,bmp280_display_loop2
;     ret