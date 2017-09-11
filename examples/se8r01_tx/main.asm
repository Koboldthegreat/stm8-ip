;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.6.8 #9946 (Linux)
;--------------------------------------------------------
	.module main
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _se8r01_tx
	.globl _se8r01_init
	.globl _uart_init
	.globl _printf
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
;--------------------------------------------------------
; Stack segment in internal ram 
;--------------------------------------------------------
	.area	SSEG
__start__stack:
	.ds	1

;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)
;--------------------------------------------------------
; interrupt vector 
;--------------------------------------------------------
	.area HOME
__interrupt_vect:
	int s_GSINIT ; reset
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
__sdcc_gs_init_startup:
__sdcc_init_data:
; stm8_genXINIT() start
	ldw x, #l_DATA
	jreq	00002$
00001$:
	clr (s_DATA - 1, x)
	decw x
	jrne	00001$
00002$:
	ldw	x, #l_INITIALIZER
	jreq	00004$
00003$:
	ld	a, (s_INITIALIZER - 1, x)
	ld	(s_INITIALIZED - 1, x), a
	decw	x
	jrne	00003$
00004$:
; stm8_genXINIT() end
	.area GSFINAL
	jp	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
__sdcc_program_startup:
	jp	_main
;	return from main will return to caller
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	../../stm8/delay.h: 11: static inline void delay_ms(uint32_t ms) {
;	-----------------------------------------
;	 function delay_ms
;	-----------------------------------------
_delay_ms:
	sub	sp, #8
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	x
	ldw	(0x03, sp), x
	ldw	(0x01, sp), x
	ldw	x, (0x0d, sp)
	pushw	x
	ldw	x, (0x0d, sp)
	pushw	x
	push	#0x6f
	clrw	x
	pushw	x
	push	#0x00
	call	__mullong
	addw	sp, #8
	ldw	(0x07, sp), x
	ldw	(0x05, sp), y
00103$:
	ldw	x, (0x03, sp)
	cpw	x, (0x07, sp)
	ld	a, (0x02, sp)
	sbc	a, (0x06, sp)
	ld	a, (0x01, sp)
	sbc	a, (0x05, sp)
	jrnc	00105$
;	../../stm8/delay.h: 13: __asm__("nop");
	nop
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	ldw	y, (0x03, sp)
	addw	y, #0x0001
	ld	a, (0x02, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x01, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x03, sp), y
	ldw	(0x01, sp), x
	jra	00103$
00105$:
	addw	sp, #8
	ret
;	../../stm8/delay.h: 17: static inline void delay_us(uint32_t us) {
;	-----------------------------------------
;	 function delay_us
;	-----------------------------------------
_delay_us:
;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
	clrw	y
	clrw	x
00103$:
	cpw	y, #0x0000
	ld	a, xl
	sbc	a, #0x00
	ld	a, xh
	sbc	a, #0x00
	jrc	00115$
	ret
00115$:
;	../../stm8/delay.h: 19: __asm__("nop");
	nop
;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
	incw	y
	jrne	00103$
	incw	x
	jra	00103$
	ret
;	main.c: 10: void main() {
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
	sub	sp, #14
;	main.c: 11: uint8_t counter = 0;
	clr	(0x0b, sp)
;	main.c: 15: CLK_CKDIVR = 3;
	mov	0x50c6+0, #0x03
;	main.c: 17: uart_init();
	call	_uart_init
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	y
	clrw	x
00110$:
	cpw	y, #0x2b5c
	ld	a, xl
	sbc	a, #0x00
	ld	a, xh
	sbc	a, #0x00
	jrnc	00106$
;	../../stm8/delay.h: 13: __asm__("nop");
	nop
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	incw	y
	jrne	00110$
	incw	x
	jra	00110$
;	main.c: 18: delay_ms(100);
00106$:
;	main.c: 19: printf("Hello\n");
	ldw	x, #___str_0+0
	pushw	x
	call	_printf
	addw	sp, #2
;	main.c: 21: se8r01_init('t');
	push	#0x74
	call	_se8r01_init
	pop	a
;	main.c: 23: while (1) {
00103$:
;	main.c: 24: for (uint8_t i=0; i<TX_PLOAD_WIDTH; i++)
	clr	(0x04, sp)
	ldw	x, sp
	addw	x, #5
	ldw	(0x0d, sp), x
00113$:
	ld	a, (0x04, sp)
	cp	a, #0x06
	jrnc	00101$
;	main.c: 25: tx_buf[i] = counter++;
	clrw	x
	ld	a, (0x04, sp)
	ld	xl, a
	addw	x, (0x0d, sp)
	ld	a, (0x0b, sp)
	ld	(0x0c, sp), a
	inc	(0x0b, sp)
	ld	a, (0x0c, sp)
	ld	(x), a
;	main.c: 24: for (uint8_t i=0; i<TX_PLOAD_WIDTH; i++)
	inc	(0x04, sp)
	jra	00113$
00101$:
;	main.c: 26: se8r01_tx(tx_buf);
	ldw	x, (0x0d, sp)
	pushw	x
	call	_se8r01_tx
	addw	sp, #2
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	x
	clr	(0x02, sp)
	clr	a
00116$:
	push	a
	cpw	x, #0x15ae
	ld	a, (0x03, sp)
	sbc	a, #0x00
	ld	a, (1, sp)
	sbc	a, #0x00
	pop	a
	jrnc	00103$
;	../../stm8/delay.h: 13: __asm__("nop");
	nop
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	addw	x, #0x0001
	push	a
	ld	a, (0x03, sp)
	adc	a, #0x00
	ld	yl, a
	pop	a
	adc	a, #0x00
	exg	a, yl
	ld	(0x02, sp), a
	exg	a, yl
	jra	00116$
;	main.c: 32: delay_ms(50);
	addw	sp, #14
	ret
	.area CODE
___str_0:
	.ascii "Hello"
	.db 0x0a
	.db 0x00
	.area INITIALIZER
	.area CABS (ABS)
