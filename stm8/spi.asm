;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.6.8 #9946 (Linux)
;--------------------------------------------------------
	.module spi
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _SPI_init
	.globl _SPI_read
	.globl _SPI_write
	.globl _SPI_write_read
	.globl _init_bit
	.globl _SPI_write_read_bit
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
;	../../stm8/spi.c: 5: void SPI_init() {
;	-----------------------------------------
;	 function SPI_init
;	-----------------------------------------
_SPI_init:
;	../../stm8/spi.c: 6: SPI_CR1 |= (1 << SPI_CR1_MSTR) | (1<<SPI_CR1_BR0); 
	ldw	x, #0x5200
	ld	a, (x)
	or	a, #0x0c
	ld	(x), a
;	../../stm8/spi.c: 7: SPI_CR2 |= (1 << SPI_CR2_SSM) | (1<<SPI_CR2_SSI);
	ldw	x, #0x5201
	ld	a, (x)
	or	a, #0x03
	ld	(x), a
;	../../stm8/spi.c: 8: SPI_CR1 |= (1 << SPI_CR1_SPE);
	ldw	x, #0x5200
	ld	a, (x)
	or	a, #0x40
	ld	(x), a
	ret
;	../../stm8/spi.c: 11: uint8_t SPI_read() {
;	-----------------------------------------
;	 function SPI_read
;	-----------------------------------------
_SPI_read:
;	../../stm8/spi.c: 12: SPI_write(0xFF);
	push	#0xff
	call	_SPI_write
	pop	a
;	../../stm8/spi.c: 13: while (!(SPI_SR & (1 << SPI_SR_RXNE))); //wait till byte received
00101$:
	ldw	x, #0x5203
	ld	a, (x)
	srl	a
	jrnc	00101$
;	../../stm8/spi.c: 14: return SPI_DR;
	ldw	x, #0x5204
	ld	a, (x)
	ret
;	../../stm8/spi.c: 17: void SPI_write(uint8_t data) {
;	-----------------------------------------
;	 function SPI_write
;	-----------------------------------------
_SPI_write:
;	../../stm8/spi.c: 18: SPI_DR = data;
	ldw	x, #0x5204
	ld	a, (0x03, sp)
	ld	(x), a
;	../../stm8/spi.c: 19: while (!(SPI_SR & (1 << SPI_SR_TXE)));
00101$:
	ldw	x, #0x5203
	ld	a, (x)
	bcp	a, #0x02
	jreq	00101$
;	../../stm8/spi.c: 20: while ((SPI_SR & (1<<SPI_SR_BSY))); //wait till byte Send`
00104$:
	tnz	a
	jrmi	00104$
	ret
;	../../stm8/spi.c: 23: uint8_t SPI_write_read(uint8_t data) {
;	-----------------------------------------
;	 function SPI_write_read
;	-----------------------------------------
_SPI_write_read:
	push	a
;	../../stm8/spi.c: 26: SPI_DR = data;
	ldw	x, #0x5204
	ld	a, (0x04, sp)
	ld	(x), a
;	../../stm8/spi.c: 27: while (!(SPI_SR & (1 << SPI_SR_RXNE))); //wait till byte received
00101$:
	ldw	x, #0x5203
	ld	a, (x)
	srl	a
	jrnc	00101$
;	../../stm8/spi.c: 28: rcv = SPI_DR;
	ldw	x, #0x5204
	ld	a, (x)
	ld	(0x01, sp), a
;	../../stm8/spi.c: 29: while (!(SPI_SR & (1<<SPI_SR_TXE))); //wait till byte Send`
00104$:
;	../../stm8/spi.c: 27: while (!(SPI_SR & (1 << SPI_SR_RXNE))); //wait till byte received
	ldw	x, #0x5203
	ld	a, (x)
;	../../stm8/spi.c: 29: while (!(SPI_SR & (1<<SPI_SR_TXE))); //wait till byte Send`
	bcp	a, #0x02
	jreq	00104$
;	../../stm8/spi.c: 30: while ((SPI_SR & (1<<SPI_SR_BSY))); //wait till byte Send`
00107$:
	tnz	a
	jrmi	00107$
;	../../stm8/spi.c: 31: return rcv;
	ld	a, (0x01, sp)
	addw	sp, #1
	ret
;	../../stm8/spi.c: 35: void init_bit() {
;	-----------------------------------------
;	 function init_bit
;	-----------------------------------------
_init_bit:
;	../../stm8/spi.c: 36: PC_DDR |= (1<<5) | (1<<6); //output
	ldw	x, #0x500c
	ld	a, (x)
	or	a, #0x60
	ld	(x), a
;	../../stm8/spi.c: 37: PC_CR1 |= (1<<5) | (1<<6); //push-pull
	ldw	x, #0x500d
	ld	a, (x)
	or	a, #0x60
	ld	(x), a
;	../../stm8/spi.c: 38: PC_CR2 |= (1<<5) | (1<<6); //10Mhz speed
	ldw	x, #0x500e
	ld	a, (x)
	or	a, #0x60
	ld	(x), a
;	../../stm8/spi.c: 40: PC_DDR &= ~(1<<7); //Input
	bres	0x500c, #7
;	../../stm8/spi.c: 41: PC_CR1 &= ~(1<<7); //No pull-up
	bres	0x500d, #7
;	../../stm8/spi.c: 42: PC_CR2 &= ~(1<<7); //disable external interupt
	bres	0x500e, #7
	ret
;	../../stm8/spi.c: 45: uint8_t SPI_write_read_bit(uint8_t byte) {
;	-----------------------------------------
;	 function SPI_write_read_bit
;	-----------------------------------------
_SPI_write_read_bit:
;	../../stm8/spi.c: 46: for(uint8_t i=0; i<8; i++) {
	clr	a
	ld	yl, a
00108$:
	ld	a, yl
	cp	a, #0x08
	jrnc	00106$
;	../../stm8/spi.c: 48: PC_ODR |= (1<<6);
	ldw	x, #0x500a
	ld	a, (x)
;	../../stm8/spi.c: 47: if(byte & 0x80) {
	tnz	(0x03, sp)
	jrpl	00102$
;	../../stm8/spi.c: 48: PC_ODR |= (1<<6);
	or	a, #0x40
	ldw	x, #0x500a
	ld	(x), a
	jra	00103$
00102$:
;	../../stm8/spi.c: 50: PC_ODR &= ~(1<<6);
	and	a, #0xbf
	ldw	x, #0x500a
	ld	(x), a
00103$:
;	../../stm8/spi.c: 52: PC_ODR |= (1<<5);
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x20
	ld	(x), a
;	../../stm8/spi.c: 53: byte <<= 1;
	sll	(0x03, sp)
;	../../stm8/spi.c: 54: if(PC_IDR & (1<<7)) {
	ldw	x, #0x500b
	ld	a, (x)
	jrpl	00105$
;	../../stm8/spi.c: 55: byte |=1;
	ld	a, (0x03, sp)
	or	a, #0x01
	ld	(0x03, sp), a
00105$:
;	../../stm8/spi.c: 57: PC_ODR &= ~(1<<5);
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xdf
	ld	(x), a
;	../../stm8/spi.c: 46: for(uint8_t i=0; i<8; i++) {
	incw	y
	jra	00108$
00106$:
;	../../stm8/spi.c: 59: return byte;
	ld	a, (0x03, sp)
	ret
	.area CODE
	.area INITIALIZER
	.area CABS (ABS)
