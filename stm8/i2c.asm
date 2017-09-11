;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.6.8 #9946 (Linux)
;--------------------------------------------------------
	.module i2c
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _i2c_init
	.globl _i2c_start
	.globl _i2c_stop
	.globl _i2c_write
	.globl _i2c_write_addr
	.globl _i2c_read
	.globl _i2c_read_arr
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
;	../../stm8/i2c.c: 4: void i2c_init() {
;	-----------------------------------------
;	 function i2c_init
;	-----------------------------------------
_i2c_init:
;	../../stm8/i2c.c: 5: I2C_FREQR = (1 << I2C_FREQR_FREQ1);
	mov	0x5212+0, #0x02
;	../../stm8/i2c.c: 6: I2C_CCRL = 0x0A; // 100kHz
	mov	0x521b+0, #0x0a
;	../../stm8/i2c.c: 7: I2C_OARH = (1 << I2C_OARH_ADDMODE); // 7-bit addressing
	mov	0x5214+0, #0x80
;	../../stm8/i2c.c: 8: I2C_CR1 = (1 << I2C_CR1_PE);
	mov	0x5210+0, #0x01
	ret
;	../../stm8/i2c.c: 11: void i2c_start() {
;	-----------------------------------------
;	 function i2c_start
;	-----------------------------------------
_i2c_start:
;	../../stm8/i2c.c: 12: I2C_CR2 |= (1 << I2C_CR2_START);
	bset	0x5211, #0
;	../../stm8/i2c.c: 13: while (!(I2C_SR1 & (1 << I2C_SR1_SB)));
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	srl	a
	jrnc	00101$
	ret
;	../../stm8/i2c.c: 16: void i2c_stop() {
;	-----------------------------------------
;	 function i2c_stop
;	-----------------------------------------
_i2c_stop:
;	../../stm8/i2c.c: 17: I2C_CR2 |= (1 << I2C_CR2_STOP);
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x02
	ld	(x), a
;	../../stm8/i2c.c: 18: while (!(I2C_SR3 & (1 << I2C_SR3_MSL)));
00101$:
	ldw	x, #0x5219
	ld	a, (x)
	srl	a
	jrnc	00101$
	ret
;	../../stm8/i2c.c: 21: void i2c_write(uint8_t data) {
;	-----------------------------------------
;	 function i2c_write
;	-----------------------------------------
_i2c_write:
;	../../stm8/i2c.c: 22: I2C_DR = data;
	ldw	x, #0x5216
	ld	a, (0x03, sp)
	ld	(x), a
;	../../stm8/i2c.c: 23: while (!(I2C_SR1 & (1 << I2C_SR1_TXE)));
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	jrpl	00101$
	ret
;	../../stm8/i2c.c: 26: void i2c_write_addr(uint8_t addr) {
;	-----------------------------------------
;	 function i2c_write_addr
;	-----------------------------------------
_i2c_write_addr:
;	../../stm8/i2c.c: 27: I2C_DR = addr;
	ldw	x, #0x5216
	ld	a, (0x03, sp)
	ld	(x), a
;	../../stm8/i2c.c: 28: while (!(I2C_SR1 & (1 << I2C_SR1_ADDR)));
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	bcp	a, #0x02
	jreq	00101$
;	../../stm8/i2c.c: 29: (void) I2C_SR3; // check BUS_BUSY
	ldw	x, #0x5219
	ld	a, (x)
;	../../stm8/i2c.c: 30: I2C_CR2 |= (1 << I2C_CR2_ACK);
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
	ret
;	../../stm8/i2c.c: 33: uint8_t i2c_read() {
;	-----------------------------------------
;	 function i2c_read
;	-----------------------------------------
_i2c_read:
;	../../stm8/i2c.c: 34: I2C_CR2 &= ~(1 << I2C_CR2_ACK);
	ldw	x, #0x5211
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
;	../../stm8/i2c.c: 35: i2c_stop();
	call	_i2c_stop
;	../../stm8/i2c.c: 36: while (!(I2C_SR1 & (1 << I2C_SR1_RXNE)));
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	bcp	a, #0x40
	jreq	00101$
;	../../stm8/i2c.c: 37: return I2C_DR;
	ldw	x, #0x5216
	ld	a, (x)
	ret
;	../../stm8/i2c.c: 40: void i2c_read_arr(uint8_t *buf, int len) {
;	-----------------------------------------
;	 function i2c_read_arr
;	-----------------------------------------
_i2c_read_arr:
	sub	sp, #2
;	../../stm8/i2c.c: 41: while (len-- > 1) {
00104$:
	ldw	y, (0x07, sp)
	ldw	x, y
	decw	x
	ldw	(0x07, sp), x
;	../../stm8/i2c.c: 44: *(buf++) = I2C_DR;
	ldw	x, (0x05, sp)
	ldw	(0x01, sp), x
;	../../stm8/i2c.c: 41: while (len-- > 1) {
	cpw	y, #0x0001
	jrsle	00106$
;	../../stm8/i2c.c: 42: I2C_CR2 |= (1 << I2C_CR2_ACK);
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
;	../../stm8/i2c.c: 43: while (!(I2C_SR1 & (1 << I2C_SR1_RXNE)));
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	bcp	a, #0x40
	jreq	00101$
;	../../stm8/i2c.c: 44: *(buf++) = I2C_DR;
	ldw	x, #0x5216
	ld	a, (x)
	ldw	x, (0x01, sp)
	ld	(x), a
	ldw	x, (0x01, sp)
	incw	x
	ldw	(0x05, sp), x
	jra	00104$
00106$:
;	../../stm8/i2c.c: 46: *buf = i2c_read();
	call	_i2c_read
	ldw	x, (0x01, sp)
	ld	(x), a
	addw	sp, #2
	ret
	.area CODE
	.area INITIALIZER
	.area CABS (ABS)
