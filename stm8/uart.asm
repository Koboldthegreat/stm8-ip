;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.6.8 #9946 (Linux)
;--------------------------------------------------------
	.module uart
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _uart_init
	.globl _uart_write
	.globl _uart_read
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	../../stm8/uart.c: 4: void uart_init() {
;	-----------------------------------------
;	 function uart_init
;	-----------------------------------------
_uart_init:
;	../../stm8/uart.c: 8: UART1_BRR2 = ((div >> 8) & 0xF0) + (div & 0x0F);
	mov	0x5233+0, #0x03
;	../../stm8/uart.c: 9: UART1_BRR1 = div >> 4;
	ld	a, #0x68
	ldw	x, #0x5232
	ld	(x), a
;	../../stm8/uart.c: 11: UART1_CR2 = (1 << UART1_CR2_TEN) | (1 << UART1_CR2_REN);
	mov	0x5235+0, #0x0c
	ret
;	../../stm8/uart.c: 14: void uart_write(uint8_t data) {
;	-----------------------------------------
;	 function uart_write
;	-----------------------------------------
_uart_write:
;	../../stm8/uart.c: 15: UART1_DR = data;
	ldw	x, #0x5231
	ld	a, (0x03, sp)
	ld	(x), a
;	../../stm8/uart.c: 16: while (!(UART1_SR & (1 << UART1_SR_TC)));
00101$:
	ldw	x, #0x5230
	ld	a, (x)
	bcp	a, #0x40
	jreq	00101$
	ret
;	../../stm8/uart.c: 19: uint8_t uart_read() {
;	-----------------------------------------
;	 function uart_read
;	-----------------------------------------
_uart_read:
;	../../stm8/uart.c: 20: while (!(UART1_SR & (1 << UART1_SR_RXNE)));
00101$:
	ldw	x, #0x5230
	ld	a, (x)
	bcp	a, #0x20
	jreq	00101$
;	../../stm8/uart.c: 21: return UART1_DR;
	ldw	x, #0x5231
	ld	a, (x)
	ret
	.area CODE
	.area INITIALIZER
	.area CABS (ABS)
