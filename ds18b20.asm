;MAB8049H, 10.000MHz
;Gdansk 2021
;TODO add uses/destroys
;TODO optimize RAM usage (overlap variables)
	.cr	8048
	.tf	rom.bin,BIN
	.lf	meteo.lst
;==================Defines=====================
;Pins
ow_pin  .eq %10000000 ;1-Wire pin at P1.7

uart_tx .eq %10000000 ;Tx pin at P2.7

skip_rom		.eq $CC
convert_t		.eq $44
read_scratchpad	.eq $BE

temp_int    .eq $20
temp_dec    .eq $21

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
    
loop:
    call temp_get
    mov R0,#temp_int
    mov A,@R0
    mov R0,A
    call uart_write_byte
  
    mov R0,#temp_dec
    mov A,@R0
    mov R0,A
    call uart_write_byte

    mov R6,#250
    call delay_ms
    mov R6,#250
    call delay_ms
	jmp loop

;Constants

;Subroutines



;R0 - MSB, R2 - LSB, uses R0,R2
temp_convert:
	;Compute integral part
	mov A,R0 ;Load MSB
	rl A
	rl A
	rl A
	rl A ;Shift 4 times left
	anl A,#%01110000 ;Mask unneeded bits
	mov R0,A ;Store result in R0
	
	mov A,R2 ;Load LSB
	rr A
	rr A
	rr A
	rr A ;Shift 4 times right
	anl A,#%00001111 ;Mask unneeded bits
	orl A,R0 ;Add MSBs to LSBs to create result
	
	mov R0,#temp_int
	mov @R0,A ;Store computed value
	
	;Compute decimal part - ultra evil math hacking to overcome lack of division, multiplication and floats!
	;Computes 6.25*decimal_bits as (25/4)*decimal_bits, what gives 2 decimal places
	;Multiplication by 25 is done by shifting left and adding (25*x = 16*x + 8*x + x = (x << 4) + (x << 3) + x)
	;The equation has to be transformed so that the addition potentially causing overflow will be the last one, to use carry as the ninth bit of result (25*15 > 255)
	;Division by 4 is performed by right shifting, but to use carry set by addition it has to be RRC (rotate right through carry), not RR
	;So the final equation: (25/4)*x = (((x << 4) + x + (x << 3)) >>> 2; where >>> - RRC
	mov A,R2 ;Load LSB
	anl A,#%00001111 ;Get decimal bits
	mov R2,A ;Store just those bits in R2
	rl A
	rl A
	rl A
	rl A
	mov R0,A ;R0 = x << 4
	
	mov A,R2
	rl A
	rl A
	rl A ;A = x << 3
	
	add A,R2 ;First perform the addition that won't cause overflow
	add A,R0 ;Now perform the addition potentially causing overflow
	rrc A
	rrc A ;Divide by 4 with carry
	anl A,#%01111111 ;Mask MSB - result will be in 0...93 interval, so it should never be set
	mov R0,#temp_dec
	mov @R0,A ;Store computed value
	ret

;R0	- byte to send, uses R0,R6,R7
uart_write_byte:
	mov R6,#8 ;Load bit counter	
	mov A,R0 ;Move byte to be send to A	
	anl P2,#~uart_tx ;Set Tx pin low - start bit
	call delay_100us
uart_write_loop:
	jb0 uart_write_one ;Check if LSB of A is set
	anl P2,#~uart_tx ;Set Tx pin low
	jmp uart_write_delay	
uart_write_one:
	orl P2,#uart_tx ;Set Tx pin high
uart_write_delay:
	call delay_100us
	rr A ;Shift byte one bit right
	djnz R6,uart_write_loop

	orl P2,#uart_tx ;Set Tx pin high - stop bit
	call delay_100us
	ret

;~100uS delay, uses R7
delay_100us:
	mov R7,#28
delay_100us_loop:
	djnz R7,delay_100us_loop
	ret

;~500uS delay, uses R7
delay_500us:
	mov R7,#164
delay_500us_loop:
	djnz R7,delay_500us_loop
	ret

;R6 - delay time in ms, uses R6,R7
delay_ms:
	mov R7,#228
delay_ms_loop:
	nop
	djnz R7,delay_ms_loop
	djnz R6,delay_ms
	ret 