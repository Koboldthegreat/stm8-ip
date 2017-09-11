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
	.globl _se8r01_wait_rx
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
	sub	sp, #30
;	main.c: 11: uint8_t rx_buf[TX_PLOAD_WIDTH] = {0};
	ldw	x, sp
	addw	x, #14
	ldw	(0x18, sp), x
	clr	(x)
	ldw	x, (0x18, sp)
	incw	x
	clr	(x)
	ldw	x, (0x18, sp)
	incw	x
	incw	x
	clr	(x)
	ldw	x, (0x18, sp)
	addw	x, #0x0003
	clr	(x)
	ldw	x, (0x18, sp)
	addw	x, #0x0004
	clr	(x)
	ldw	x, (0x18, sp)
	addw	x, #0x0005
	ldw	(0x1a, sp), x
	clr	(x)
;	main.c: 12: uint32_t total = 0;
	clrw	x
	ldw	(0x0c, sp), x
	ldw	(0x0a, sp), x
;	main.c: 13: uint32_t dropped = 0;
	clrw	x
	ldw	(0x08, sp), x
	ldw	(0x06, sp), x
;	main.c: 14: uint8_t last = 0;
	clr	(0x05, sp)
;	main.c: 16: CLK_CKDIVR = 3;
	mov	0x50c6+0, #0x03
;	main.c: 18: uart_init();
	call	_uart_init
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clr	(0x04, sp)
	clr	(0x03, sp)
	clr	a
	rlwa	x
	clr	a
	rrwa	x
00115$:
	push	a
	ld	a, (0x05, sp)
	cp	a, #0x5c
	ld	a, (0x04, sp)
	sbc	a, #0x2b
	ld	a, (1, sp)
	sbc	a, #0x00
	ld	a, xh
	sbc	a, #0x00
	pop	a
	jrnc	00111$
;	../../stm8/delay.h: 13: __asm__("nop");
	nop
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	ldw	y, (0x03, sp)
	addw	y, #0x0001
	adc	a, #0x00
	push	a
	ld	a, xh
	adc	a, #0x00
	ld	xh, a
	pop	a
	ldw	(0x03, sp), y
	jra	00115$
;	main.c: 19: delay_ms(100);
00111$:
;	main.c: 21: se8r01_init('r');
	push	#0x72
	call	_se8r01_init
	pop	a
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	y
	clrw	x
00118$:
	cpw	y, #0xb198
	ld	a, xl
	sbc	a, #0x01
	ld	a, xh
	sbc	a, #0x00
	jrnc	00108$
;	../../stm8/delay.h: 13: __asm__("nop");
	nop
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	incw	y
	jrne	00118$
	incw	x
	jra	00118$
;	main.c: 24: while (1) {
00108$:
;	main.c: 25: if (se8r01_wait_rx(rx_buf)){
	ldw	x, (0x18, sp)
	pushw	x
	call	_se8r01_wait_rx
	addw	sp, #2
	ld	(0x1e, sp), a
;	main.c: 26: PD_ODR |= (1<<4);
	ldw	x, #0x500f
	ld	a, (x)
	ld	(0x14, sp), a
;	main.c: 25: if (se8r01_wait_rx(rx_buf)){
	tnz	(0x1e, sp)
	jrne	00157$
	jp	00105$
00157$:
;	main.c: 26: PD_ODR |= (1<<4);
	ld	a, (0x14, sp)
	or	a, #0x10
	ldw	x, #0x500f
	ld	(x), a
;	main.c: 32: if (rx_buf[TX_PLOAD_WIDTH-1] != last+6 && last != 0){
	ldw	x, (0x1a, sp)
	ld	a, (x)
	ld	(0x15, sp), a
	clrw	x
	ld	a, (0x05, sp)
	ld	xl, a
	addw	x, #0x0006
	ldw	(0x1c, sp), x
	clrw	x
	ld	a, (0x15, sp)
	ld	xl, a
	cpw	x, (0x1c, sp)
	jreq	00102$
	tnz	(0x05, sp)
	jreq	00102$
;	main.c: 33: dropped++;
	ldw	y, (0x08, sp)
	addw	y, #0x0001
	ld	a, (0x07, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x06, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x08, sp), y
	ldw	(0x06, sp), x
00102$:
;	main.c: 35: total++;
	ldw	y, (0x0c, sp)
	addw	y, #0x0001
	ld	a, (0x0b, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x0a, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x0c, sp), y
	ldw	(0x0a, sp), x
;	main.c: 36: last = rx_buf[TX_PLOAD_WIDTH-1];
	ld	a, (0x15, sp)
	ld	(0x05, sp), a
;	main.c: 37: printf("total: %d, ", (int) total);
	ldw	y, (0x0c, sp)
	ldw	(0x16, sp), y
	ldw	x, #___str_0+0
	ldw	y, (0x16, sp)
	pushw	y
	pushw	x
	call	_printf
	addw	sp, #4
;	main.c: 38: printf("dropped: %d, ", (int) dropped);
	ldw	x, (0x08, sp)
	ldw	y, #___str_1+0
	pushw	x
	pushw	x
	pushw	y
	call	_printf
	addw	sp, #4
	popw	x
;	main.c: 39: printf("->: %d\n", ((int) dropped*100)/(int) total);
	pushw	x
	push	#0x64
	push	#0x00
	call	__mulint
	addw	sp, #4
	ldw	y, (0x16, sp)
	pushw	y
	pushw	x
	call	__divsint
	addw	sp, #4
	ldw	y, #___str_2+0
	pushw	x
	pushw	y
	call	_printf
	addw	sp, #4
	jp	00108$
00105$:
;	main.c: 41: PD_ODR &= ~(1<<4);
	ld	a, (0x14, sp)
	and	a, #0xef
	ldw	x, #0x500f
	ld	(x), a
	jp	00108$
	addw	sp, #30
	ret
	.area CODE
___str_0:
	.ascii "total: %d, "
	.db 0x00
___str_1:
	.ascii "dropped: %d, "
	.db 0x00
___str_2:
	.ascii "->: %d"
	.db 0x0a
	.db 0x00
	.area INITIALIZER
	.area CABS (ABS)
