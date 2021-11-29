;MAB8049H, 10.000MHz
;Gdansk 2021
;TODO add uses/destroys
;TODO optimize RAM usage (overlap variables)
	.cr	8048
	.tf	rom.bin,BIN
	.lf	meteo.lst
;==================Defines=====================
;Pins
dht11_data_pin  .eq %10000000 ;DHT11 data pin at P1.7

uart_tx .eq %10000000 ;Tx pin at P2.7

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
    call dht11_read_rh
    call uart_write_byte
    mov R6,#250
    call delay_ms
    mov R6,#250
    call delay_ms
	jmp loop

;Constants

;Subroutines
;TODO move delay_70us into dht11_read_rh, as it is needed only once
;TODO check if clearing result is needed
;R0 - value of RH in percent, uses and destroys R0,R6,R7
dht11_read_rh:
    mov R0,#0
    anl P1,#~dht11_data_pin
    mov R6,#20
    call delay_ms ;Pull DHT11 bus low for at least 18ms (20ms here to be sure)
    orl P1,#dht11_data_pin ;Release bus
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
    anl A,#dht11_data_pin ;Mask all pins except for DHT11 bus
    jnz dht11_read_one ;If '1' bit was received
    jmp dht11_read_rh_continue ; If '0' bit was received
dht11_read_one:
    inc R0 ;R0 = R0+1 (= R0|1)
dht11_sync_loop:
    in A,P1 ;Load port state to A
    anl A,#dht11_data_pin ;Mask all pins except for DHT11 bus
    jnz dht11_sync_loop ;Self-synchronize timings - on every '1' bit received, wait for start-of-transmission bit
dht11_read_rh_continue:
    djnz R6,dht11_read_rh_loop ;Repeat for all bits
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

;R6 - delay time in ms, uses R6,R7
delay_ms:
	mov R7,#228
delay_ms_loop:
	nop
	djnz R7,delay_ms_loop
	djnz R6,delay_ms
	ret 