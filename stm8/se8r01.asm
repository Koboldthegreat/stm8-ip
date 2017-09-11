;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.6.8 #9946 (Linux)
;--------------------------------------------------------
	.module se8r01
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _printf
	.globl _uart_read
	.globl _uart_write
	.globl _init_bit
	.globl _SPI_write_read_bit
	.globl _putchar
	.globl _getchar
	.globl _SPI_write_read_reg
	.globl _SPI_write_buf
	.globl _SPI_read_buf
	.globl _se8r01_init
	.globl _se8r01_wait_rx
	.globl _se8r01_tx
	.globl _se8r01_get_status
	.globl _init_io
	.globl _clear_IRQ_flags
	.globl _rf_switch_bank
	.globl _se8r01_powerup
	.globl _se8r01_calibration
	.globl _se8r01_setup
	.globl _se8r01_settings
	.globl _se8r01_flush_TX
	.globl _se8r01_flush_RX
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
;	../../stm8/se8r01.c: 13: int putchar(int c) {
;	-----------------------------------------
;	 function putchar
;	-----------------------------------------
_putchar:
;	../../stm8/se8r01.c: 14: uart_write(c);
	ld	a, (0x04, sp)
	push	a
	call	_uart_write
	pop	a
;	../../stm8/se8r01.c: 15: return 0;
	clrw	x
	ret
;	../../stm8/se8r01.c: 18: char getchar() {
;	-----------------------------------------
;	 function getchar
;	-----------------------------------------
_getchar:
;	../../stm8/se8r01.c: 19: return uart_read();
	jp	_uart_read
;	../../stm8/se8r01.c: 22: uint8_t SPI_write_read_reg(uint8_t reg, uint8_t value) {
;	-----------------------------------------
;	 function SPI_write_read_reg
;	-----------------------------------------
_SPI_write_read_reg:
	push	a
;	../../stm8/se8r01.c: 25: CSN_L(); //enable spi
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
;	../../stm8/se8r01.c: 26: status = SPI_write_read_bit(reg); //select register
	ld	a, (0x04, sp)
	push	a
	call	_SPI_write_read_bit
	addw	sp, #1
	ld	(0x01, sp), a
;	../../stm8/se8r01.c: 27: SPI_write_read_bit(value); //write value to it
	ld	a, (0x05, sp)
	push	a
	call	_SPI_write_read_bit
	pop	a
;	../../stm8/se8r01.c: 28: CSN_H(); //disable spi
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x10
	ld	(x), a
;	../../stm8/se8r01.c: 30: return status;
	ld	a, (0x01, sp)
	addw	sp, #1
	ret
;	../../stm8/se8r01.c: 33: uint8_t SPI_write_buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes) {
;	-----------------------------------------
;	 function SPI_write_buf
;	-----------------------------------------
_SPI_write_buf:
	push	a
;	../../stm8/se8r01.c: 36: CSN_L();
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
;	../../stm8/se8r01.c: 37: status = SPI_write_read_bit(reg);
	ld	a, (0x04, sp)
	push	a
	call	_SPI_write_read_bit
	addw	sp, #1
	ld	(0x01, sp), a
;	../../stm8/se8r01.c: 38: for (uint8_t i=0; i<bytes; i++) {
	clr	a
00103$:
	cp	a, (0x07, sp)
	jrnc	00101$
;	../../stm8/se8r01.c: 39: SPI_write_read_bit(pBuf[i]);
	clrw	x
	ld	xl, a
	addw	x, (0x05, sp)
	push	a
	ld	a, (x)
	ld	xl, a
	pushw	x
	addw	sp, #1
	call	_SPI_write_read_bit
	pop	a
	pop	a
;	../../stm8/se8r01.c: 38: for (uint8_t i=0; i<bytes; i++) {
	inc	a
	jra	00103$
00101$:
;	../../stm8/se8r01.c: 41: CSN_H();
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x10
	ld	(x), a
;	../../stm8/se8r01.c: 42: return status;
	ld	a, (0x01, sp)
	addw	sp, #1
	ret
;	../../stm8/se8r01.c: 45: uint8_t SPI_read_buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes) {
;	-----------------------------------------
;	 function SPI_read_buf
;	-----------------------------------------
_SPI_read_buf:
	sub	sp, #2
;	../../stm8/se8r01.c: 47: CSN_L();
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
;	../../stm8/se8r01.c: 48: status = SPI_write_read_bit(reg); 
	ld	a, (0x05, sp)
	push	a
	call	_SPI_write_read_bit
	addw	sp, #1
	ld	(0x01, sp), a
;	../../stm8/se8r01.c: 49: for(uint8_t i=0; i<bytes; i++) {
	clr	a
00103$:
	cp	a, (0x08, sp)
	jrnc	00101$
;	../../stm8/se8r01.c: 50: pBuf[i] = SPI_write_read_bit(0xff);
	clrw	x
	ld	xl, a
	addw	x, (0x06, sp)
	push	a
	pushw	x
	push	#0xff
	call	_SPI_write_read_bit
	addw	sp, #1
	ld	(0x05, sp), a
	popw	x
	ld	a, (0x03, sp)
	ld	(x), a
	pop	a
;	../../stm8/se8r01.c: 49: for(uint8_t i=0; i<bytes; i++) {
	inc	a
	jra	00103$
00101$:
;	../../stm8/se8r01.c: 52: CSN_H();
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x10
	ld	(x), a
;	../../stm8/se8r01.c: 53: return status;
	ld	a, (0x01, sp)
	addw	sp, #2
	ret
;	../../stm8/se8r01.c: 57: void se8r01_init(uint8_t mode) {
;	-----------------------------------------
;	 function se8r01_init
;	-----------------------------------------
_se8r01_init:
	sub	sp, #4
;	../../stm8/se8r01.c: 59: printf("Initializing SPI\n");
	ldw	x, #___str_0+0
	pushw	x
	call	_printf
	addw	sp, #2
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	x
	clr	a
	clr	(0x01, sp)
00109$:
	push	a
	cpw	x, #0x2b5c
	ld	a, (1, sp)
	sbc	a, #0x00
	ld	a, (0x02, sp)
	sbc	a, #0x00
	pop	a
	jrnc	00105$
;	../../stm8/delay.h: 13: __asm__("nop");
	nop
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	addw	x, #0x0001
	adc	a, #0x00
	ld	yl, a
	ld	a, (0x01, sp)
	adc	a, #0x00
	ld	(0x01, sp), a
	ld	a, yl
	jra	00109$
;	../../stm8/se8r01.c: 61: delay_ms(100);
00105$:
;	../../stm8/se8r01.c: 62: init_bit(); //bit bashing SPI
	call	_init_bit
;	../../stm8/se8r01.c: 63: init_io();
	call	_init_io
;	../../stm8/se8r01.c: 64: printf("Status: %02x \n", se8r01_get_status());
	call	_se8r01_get_status
	clrw	x
	ld	xl, a
	ldw	y, #___str_1+0
	pushw	x
	pushw	y
	call	_printf
	addw	sp, #4
;	../../stm8/se8r01.c: 66: CE_L();
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	y
	clrw	x
00112$:
	cpw	y, #0x0456
	ld	a, xl
	sbc	a, #0x00
	ld	a, xh
	sbc	a, #0x00
	jrnc	00107$
;	../../stm8/delay.h: 13: __asm__("nop");
	nop
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	incw	y
	jrne	00112$
	incw	x
	jra	00112$
;	../../stm8/se8r01.c: 67: delay_ms(10);
00107$:
;	../../stm8/se8r01.c: 68: se8r01_powerup();
	call	_se8r01_powerup
;	../../stm8/se8r01.c: 69: se8r01_calibration();
	call	_se8r01_calibration
;	../../stm8/se8r01.c: 70: se8r01_setup();
	call	_se8r01_setup
;	../../stm8/se8r01.c: 71: se8r01_settings();
	call	_se8r01_settings
;	../../stm8/se8r01.c: 72: if (mode == 'r') { //rx mode
	ld	a, (0x07, sp)
	cp	a, #0x72
	jrne	00102$
;	../../stm8/se8r01.c: 73: printf("RX mode\n"); 
	ldw	x, #___str_2+0
	pushw	x
	call	_printf
	addw	sp, #2
;	../../stm8/se8r01.c: 74: SPI_write_read_reg(WRITE_REG | iRF_BANK0_CONFIG, 0x3f);
	push	#0x3f
	push	#0x20
	call	_SPI_write_read_reg
	addw	sp, #2
	jra	00103$
00102$:
;	../../stm8/se8r01.c: 77: SPI_write_read_reg(WRITE_REG | iRF_BANK0_CONFIG, 0x3e)
	push	#0x3e
	push	#0x20
	call	_SPI_write_read_reg
	addw	sp, #2
	clrw	x
	ld	xl, a
;	../../stm8/se8r01.c: 76: printf("TX mode, status %02x\n",
	ldw	y, #___str_3+0
	pushw	x
	pushw	y
	call	_printf
	addw	sp, #4
;	../../stm8/se8r01.c: 79: printf("connected? %02x\n", SPI_write_read_reg(SETUP_AW, 0xff));
	push	#0xff
	push	#0x03
	call	_SPI_write_read_reg
	addw	sp, #2
	clrw	x
	ld	xl, a
	ldw	y, #___str_4+0
	pushw	x
	pushw	y
	call	_printf
	addw	sp, #4
00103$:
;	../../stm8/se8r01.c: 81: CE_H();
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
	addw	sp, #4
	ret
;	../../stm8/se8r01.c: 84: uint8_t se8r01_wait_rx(uint8_t *rx_buf) {
;	-----------------------------------------
;	 function se8r01_wait_rx
;	-----------------------------------------
_se8r01_wait_rx:
;	../../stm8/se8r01.c: 87: while (PB_IDR & (1<<IRQ_PIN));
00101$:
	ldw	x, #0x5006
	ld	a, (x)
	bcp	a, #0x20
	jrne	00101$
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	y
	clrw	x
00110$:
	cpw	y, #0x006f
	ld	a, xl
	sbc	a, #0x00
	ld	a, xh
	sbc	a, #0x00
	jrnc	00108$
;	../../stm8/delay.h: 13: __asm__("nop");
	nop
;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	incw	y
	jrne	00110$
	incw	x
	jra	00110$
;	../../stm8/se8r01.c: 88: delay_ms(1);
00108$:
;	../../stm8/se8r01.c: 89: status = se8r01_get_status();  
	call	_se8r01_get_status
;	../../stm8/se8r01.c: 90: if (status & STA_MARK_RX) {
	bcp	a, #0x40
	jreq	00105$
;	../../stm8/se8r01.c: 91: SPI_read_buf(RD_RX_PLOAD, rx_buf, TX_PLOAD_WIDTH);
	push	#0x06
	ldw	x, (0x04, sp)
	pushw	x
	push	#0x61
	call	_SPI_read_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 92: se8r01_flush_RX();
	call	_se8r01_flush_RX
;	../../stm8/se8r01.c: 93: SPI_write_read_reg(WRITE_REG|STATUS, 0xff);//  ,0xff);
	push	#0xff
	push	#0x27
	call	_SPI_write_read_reg
	addw	sp, #2
;	../../stm8/se8r01.c: 94: return 1;
	ld	a, #0x01
	ret
00105$:
;	../../stm8/se8r01.c: 96: SPI_write_read_reg(WRITE_REG|STATUS, 0xff);
	push	#0xff
	push	#0x27
	call	_SPI_write_read_reg
	addw	sp, #2
;	../../stm8/se8r01.c: 97: return 0;
	clr	a
	ret
;	../../stm8/se8r01.c: 104: void se8r01_tx(uint8_t *tx_buf) {
;	-----------------------------------------
;	 function se8r01_tx
;	-----------------------------------------
_se8r01_tx:
	push	a
;	../../stm8/se8r01.c: 107: status = se8r01_get_status();
	call	_se8r01_get_status
	ld	(0x01, sp), a
;	../../stm8/se8r01.c: 109: PD_ODR |= (1<<4);
	ldw	x, #0x500f
	ld	a, (x)
;	../../stm8/se8r01.c: 108: if (status & (1<<5)) {
	push	a
	ld	a, (0x02, sp)
	bcp	a, #0x20
	pop	a
	jreq	00102$
;	../../stm8/se8r01.c: 109: PD_ODR |= (1<<4);
	or	a, #0x10
	ldw	x, #0x500f
	ld	(x), a
	jra	00103$
00102$:
;	../../stm8/se8r01.c: 111: PD_ODR &= ~(1<<4);
	and	a, #0xef
	ldw	x, #0x500f
	ld	(x), a
00103$:
;	../../stm8/se8r01.c: 113: se8r01_flush_TX();
	call	_se8r01_flush_TX
;	../../stm8/se8r01.c: 114: SPI_write_buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
	push	#0x06
	ldw	x, (0x05, sp)
	pushw	x
	push	#0xa0
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 115: SPI_write_read_reg(WRITE_REG+STATUS, 0xff);
	push	#0xff
	push	#0x27
	call	_SPI_write_read_reg
	addw	sp, #2
;	../../stm8/se8r01.c: 116: printf("Status: %02x \n", status);
	clrw	x
	ld	a, (0x01, sp)
	ld	xl, a
	ldw	y, #___str_1+0
	pushw	x
	pushw	y
	call	_printf
	addw	sp, #5
	ret
;	../../stm8/se8r01.c: 119: uint8_t se8r01_get_status() {
;	-----------------------------------------
;	 function se8r01_get_status
;	-----------------------------------------
_se8r01_get_status:
;	../../stm8/se8r01.c: 120: return SPI_write_read_reg(STATUS, 0);
	push	#0x00
	push	#0x07
	call	_SPI_write_read_reg
	addw	sp, #2
	ret
;	../../stm8/se8r01.c: 123: void init_io() {
;	-----------------------------------------
;	 function init_io
;	-----------------------------------------
_init_io:
;	../../stm8/se8r01.c: 125: PB_DDR &= ~(1<<IRQ_PIN); //Input
	ldw	x, #0x5007
	ld	a, (x)
	and	a, #0xdf
	ld	(x), a
;	../../stm8/se8r01.c: 126: PB_CR1 &= ~(1<<IRQ_PIN); //No pull-up
	ldw	x, #0x5008
	ld	a, (x)
	and	a, #0xdf
	ld	(x), a
;	../../stm8/se8r01.c: 127: PB_CR2 &= ~(1<<IRQ_PIN); //disable externernal interupt
	ldw	x, #0x5009
	ld	a, (x)
	and	a, #0xdf
	ld	(x), a
;	../../stm8/se8r01.c: 130: PC_DDR |= (1<<CE_PIN); //output
	ldw	x, #0x500c
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	../../stm8/se8r01.c: 131: PC_CR1 |= (1<<CE_PIN); //push-pull
	ldw	x, #0x500d
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	../../stm8/se8r01.c: 132: PC_CR2 |= (1<<CE_PIN); //10Mhz speed
	ldw	x, #0x500e
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	../../stm8/se8r01.c: 135: PC_DDR |= (1<<CSN_PIN); //output
	ldw	x, #0x500c
	ld	a, (x)
	or	a, #0x10
	ld	(x), a
;	../../stm8/se8r01.c: 136: PC_CR1 |= (1<<CSN_PIN); //push-pull
	ldw	x, #0x500d
	ld	a, (x)
	or	a, #0x10
	ld	(x), a
;	../../stm8/se8r01.c: 137: PC_CR2 |= (1<<CSN_PIN); //10Mhz speed
	ldw	x, #0x500e
	ld	a, (x)
	or	a, #0x10
	ld	(x), a
;	../../stm8/se8r01.c: 140: PD_DDR |= (1<<4);
	ldw	x, #0x5011
	ld	a, (x)
	or	a, #0x10
	ld	(x), a
;	../../stm8/se8r01.c: 141: PD_CR1 |= (1<<4);
	ldw	x, #0x5012
	ld	a, (x)
	or	a, #0x10
	ld	(x), a
;	../../stm8/se8r01.c: 143: CSN_H(); //disable spi
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x10
	ld	(x), a
;	../../stm8/se8r01.c: 144: CE_L(); //power down at startup
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
	ret
;	../../stm8/se8r01.c: 147: void clear_IRQ_flags() {
;	-----------------------------------------
;	 function clear_IRQ_flags
;	-----------------------------------------
_clear_IRQ_flags:
;	../../stm8/se8r01.c: 150: status = se8r01_get_status();
	call	_se8r01_get_status
;	../../stm8/se8r01.c: 151: SPI_write_read_reg(STATUS, status|0x70);
	or	a, #0x70
	push	a
	push	#0x07
	call	_SPI_write_read_reg
	addw	sp, #2
;	../../stm8/se8r01.c: 152: printf("clearing irq flag\n");
	ldw	x, #___str_5+0
	pushw	x
	call	_printf
	addw	sp, #2
	ret
;	../../stm8/se8r01.c: 155: void rf_switch_bank(uint8_t bankindex)
;	-----------------------------------------
;	 function rf_switch_bank
;	-----------------------------------------
_rf_switch_bank:
	sub	sp, #2
;	../../stm8/se8r01.c: 159: temp0 = SPI_write_read_bit(iRF_BANK0_STATUS);
	push	#0x07
	call	_SPI_write_read_bit
	addw	sp, #1
;	../../stm8/se8r01.c: 160: printf("Temp 0: %02x \n", temp0);
	ld	(0x02, sp), a
	clr	(0x01, sp)
	ldw	x, #___str_6+0
	push	a
	ldw	y, (0x02, sp)
	pushw	y
	pushw	x
	call	_printf
	addw	sp, #4
	pop	a
;	../../stm8/se8r01.c: 161: if((temp0 & 0x80) != bankindex)
	and	a, #0x80
	cp	a, (0x05, sp)
	jreq	00103$
;	../../stm8/se8r01.c: 163: SPI_write_read_reg(iRF_CMD_ACTIVATE,0x53);
	push	#0x53
	push	#0x50
	call	_SPI_write_read_reg
	addw	sp, #2
00103$:
	addw	sp, #2
	ret
;	../../stm8/se8r01.c: 167: void se8r01_powerup()
;	-----------------------------------------
;	 function se8r01_powerup
;	-----------------------------------------
_se8r01_powerup:
;	../../stm8/se8r01.c: 169: rf_switch_bank(iBANK0);
	push	#0x00
	call	_rf_switch_bank
	pop	a
;	../../stm8/se8r01.c: 170: SPI_write_read_reg(iRF_CMD_WRITE_REG|iRF_BANK0_CONFIG,0x03);
	push	#0x03
	push	#0x20
	call	_SPI_write_read_reg
	addw	sp, #2
;	../../stm8/se8r01.c: 171: SPI_write_read_reg(iRF_CMD_WRITE_REG|iRF_BANK0_RF_CH,0x32);
	push	#0x32
	push	#0x25
	call	_SPI_write_read_reg
	addw	sp, #2
;	../../stm8/se8r01.c: 172: SPI_write_read_reg(iRF_CMD_WRITE_REG|iRF_BANK0_RF_SETUP,0x48);
	push	#0x48
	push	#0x26
	call	_SPI_write_read_reg
	addw	sp, #2
;	../../stm8/se8r01.c: 173: SPI_write_read_reg(iRF_CMD_WRITE_REG|iRF_BANK0_PRE_GURD,0x77); //2450 calibration
	push	#0x77
	push	#0x3f
	call	_SPI_write_read_reg
	addw	sp, #2
	ret
;	../../stm8/se8r01.c: 177: void se8r01_calibration()
;	-----------------------------------------
;	 function se8r01_calibration
;	-----------------------------------------
_se8r01_calibration:
	sub	sp, #17
;	../../stm8/se8r01.c: 182: rf_switch_bank(iBANK1);
	push	#0x80
	call	_rf_switch_bank
	pop	a
;	../../stm8/se8r01.c: 184: gtemp[0]=0x40;
	ldw	x, sp
	addw	x, #5
	ldw	(0x0a, sp), x
	ld	a, #0x40
	ld	(x), a
;	../../stm8/se8r01.c: 185: gtemp[1]=0x00;
	ldw	x, (0x0a, sp)
	incw	x
	ldw	(0x0c, sp), x
	clr	(x)
;	../../stm8/se8r01.c: 186: gtemp[2]=0x10;
	ldw	x, (0x0a, sp)
	incw	x
	incw	x
	ldw	(0x10, sp), x
	ld	a, #0x10
	ld	(x), a
;	../../stm8/se8r01.c: 187: gtemp[3]=0xE6;
	ldw	x, (0x0a, sp)
	addw	x, #0x0003
	ldw	(0x0e, sp), x
	ld	a, #0xe6
	ld	(x), a
;	../../stm8/se8r01.c: 188: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, gtemp, 4);
	ldw	x, (0x0a, sp)
	push	#0x04
	pushw	x
	push	#0x21
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 190: gtemp[0]=0x20;
	ldw	x, (0x0a, sp)
	ld	a, #0x20
	ld	(x), a
;	../../stm8/se8r01.c: 191: gtemp[1]=0x08;
	ldw	x, (0x0c, sp)
	ld	a, #0x08
	ld	(x), a
;	../../stm8/se8r01.c: 192: gtemp[2]=0x50;
	ldw	x, (0x10, sp)
	ld	a, #0x50
	ld	(x), a
;	../../stm8/se8r01.c: 193: gtemp[3]=0x40;
	ldw	x, (0x0e, sp)
	ld	a, #0x40
	ld	(x), a
;	../../stm8/se8r01.c: 194: gtemp[4]=0x50;
	ldw	x, (0x0a, sp)
	ld	a, #0x50
	ld	(0x0004, x), a
;	../../stm8/se8r01.c: 195: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, gtemp, 5);
	ldw	x, (0x0a, sp)
	push	#0x05
	pushw	x
	push	#0x23
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 197: gtemp[0]=0x00;
	ldw	x, (0x0a, sp)
	clr	(x)
;	../../stm8/se8r01.c: 198: gtemp[1]=0x00;
	ldw	x, (0x0c, sp)
	clr	(x)
;	../../stm8/se8r01.c: 199: gtemp[2]=0x1E;
	ldw	x, (0x10, sp)
	ld	a, #0x1e
	ld	(x), a
;	../../stm8/se8r01.c: 200: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_IF_FREQ, gtemp, 3);
	ldw	x, (0x0a, sp)
	push	#0x03
	pushw	x
	push	#0x2a
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 202: gtemp[0]=0x29;
	ldw	x, (0x0a, sp)
	ld	a, #0x29
	ld	(x), a
;	../../stm8/se8r01.c: 203: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, gtemp, 1);
	ldw	x, (0x0a, sp)
	push	#0x01
	pushw	x
	push	#0x2c
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 205: gtemp[0]=0x00;
	ldw	x, (0x0a, sp)
	clr	(x)
;	../../stm8/se8r01.c: 206: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_LOW, gtemp, 1);
	ldw	x, (0x0a, sp)
	push	#0x01
	pushw	x
	push	#0x37
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 208: gtemp[0]=0x7F;
	ldw	x, (0x0a, sp)
	ld	a, #0x7f
	ld	(x), a
;	../../stm8/se8r01.c: 209: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_HI, gtemp, 1);
	ldw	x, (0x0a, sp)
	push	#0x01
	pushw	x
	push	#0x38
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 211: gtemp[0]=0x02;
	ldw	x, (0x0a, sp)
	ld	a, #0x02
	ld	(x), a
;	../../stm8/se8r01.c: 212: gtemp[1]=0xC1;
	ldw	x, (0x0c, sp)
	ld	a, #0xc1
	ld	(x), a
;	../../stm8/se8r01.c: 213: gtemp[2]=0xEB;
	ldw	x, (0x10, sp)
	ld	a, #0xeb
	ld	(x), a
;	../../stm8/se8r01.c: 214: gtemp[3]=0x1C;
	ldw	x, (0x0e, sp)
	ld	a, #0x1c
	ld	(x), a
;	../../stm8/se8r01.c: 215: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, gtemp, 4);
	ldw	x, (0x0a, sp)
	push	#0x04
	pushw	x
	push	#0x3d
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 217: gtemp[0]=0x97;
	ldw	x, (0x0a, sp)
	ld	a, #0x97
	ld	(x), a
;	../../stm8/se8r01.c: 218: gtemp[1]=0x64;
	ldw	x, (0x0c, sp)
	ld	a, #0x64
	ld	(x), a
;	../../stm8/se8r01.c: 219: gtemp[2]=0x00;
	ldw	x, (0x10, sp)
	clr	(x)
;	../../stm8/se8r01.c: 220: gtemp[3]=0x81;
	ldw	x, (0x0e, sp)
	ld	a, #0x81
	ld	(x), a
;	../../stm8/se8r01.c: 221: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, gtemp, 4);
	ldw	x, (0x0a, sp)
	push	#0x04
	pushw	x
	push	#0x3e
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 223: rf_switch_bank(iBANK0);
	push	#0x00
	call	_rf_switch_bank
	pop	a
;	../../stm8/se8r01.c: 225: CE_H();
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
	clrw	x
	clr	a
	clr	(0x01, sp)
00110$:
	push	a
	cpw	x, #0x0000
	ld	a, (1, sp)
	sbc	a, #0x00
	ld	a, (0x02, sp)
	sbc	a, #0x00
	pop	a
	jrnc	00102$
;	../../stm8/delay.h: 19: __asm__("nop");
	nop
;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
	addw	x, #0x0001
	adc	a, #0x00
	ld	yl, a
	ld	a, (0x01, sp)
	adc	a, #0x00
	ld	(0x01, sp), a
	ld	a, yl
	jra	00110$
;	../../stm8/se8r01.c: 226: delay_us(30);
00102$:
;	../../stm8/se8r01.c: 227: CE_L();
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
	clrw	x
	ldw	(0x03, sp), x
	ldw	(0x01, sp), x
00113$:
	ldw	x, (0x03, sp)
	cpw	x, #0x0000
	ld	a, (0x02, sp)
	sbc	a, #0x00
	ld	a, (0x01, sp)
	sbc	a, #0x00
	jrnc	00104$
;	../../stm8/delay.h: 19: __asm__("nop");
	nop
;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
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
	jra	00113$
;	../../stm8/se8r01.c: 229: delay_us(50);                            // delay 50ms waiting for calibaration.
00104$:
;	../../stm8/se8r01.c: 231: CE_H();
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
	clrw	x
	ldw	(0x03, sp), x
	ldw	(0x01, sp), x
00116$:
	ldw	x, (0x03, sp)
	cpw	x, #0x0000
	ld	a, (0x02, sp)
	sbc	a, #0x00
	ld	a, (0x01, sp)
	sbc	a, #0x00
	jrnc	00106$
;	../../stm8/delay.h: 19: __asm__("nop");
	nop
;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
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
	jra	00116$
;	../../stm8/se8r01.c: 232: delay_us(30);
00106$:
;	../../stm8/se8r01.c: 233: CE_L();
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
	clrw	y
	clrw	x
00119$:
	cpw	y, #0x0000
	ld	a, xl
	sbc	a, #0x00
	ld	a, xh
	sbc	a, #0x00
	jrnc	00121$
;	../../stm8/delay.h: 19: __asm__("nop");
	nop
;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
	incw	y
	jrne	00119$
	incw	x
	jra	00119$
;	../../stm8/se8r01.c: 235: delay_us(50);                            // delay 50ms waiting for calibaration.
00121$:
	addw	sp, #17
	ret
;	../../stm8/se8r01.c: 240: void se8r01_setup()
;	-----------------------------------------
;	 function se8r01_setup
;	-----------------------------------------
_se8r01_setup:
	sub	sp, #15
;	../../stm8/se8r01.c: 244: gtemp[0]=0x28;
	ldw	x, sp
	incw	x
	ldw	(0x0e, sp), x
	ld	a, #0x28
	ld	(x), a
;	../../stm8/se8r01.c: 245: gtemp[1]=0x32;
	ldw	x, (0x0e, sp)
	incw	x
	ldw	(0x0c, sp), x
	ld	a, #0x32
	ld	(x), a
;	../../stm8/se8r01.c: 246: gtemp[2]=0x80;
	ldw	x, (0x0e, sp)
	incw	x
	incw	x
	ldw	(0x0a, sp), x
	ld	a, #0x80
	ld	(x), a
;	../../stm8/se8r01.c: 247: gtemp[3]=0x90;
	ldw	x, (0x0e, sp)
	addw	x, #0x0003
	ldw	(0x08, sp), x
	ld	a, #0x90
	ld	(x), a
;	../../stm8/se8r01.c: 248: gtemp[4]=0x00;
	ldw	x, (0x0e, sp)
	addw	x, #0x0004
	ldw	(0x06, sp), x
	clr	(x)
;	../../stm8/se8r01.c: 249: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK0_SETUP_VALUE, gtemp, 5);
	ldw	x, (0x0e, sp)
	push	#0x05
	pushw	x
	push	#0x3e
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
	clrw	y
	clrw	x
00104$:
	cpw	y, #0x0000
	ld	a, xl
	sbc	a, #0x00
	ld	a, xh
	sbc	a, #0x00
	jrnc	00102$
;	../../stm8/delay.h: 19: __asm__("nop");
	nop
;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
	incw	y
	jrne	00104$
	incw	x
	jra	00104$
;	../../stm8/se8r01.c: 251: delay_us(2);
00102$:
;	../../stm8/se8r01.c: 253: rf_switch_bank(iBANK1);
	push	#0x80
	call	_rf_switch_bank
	pop	a
;	../../stm8/se8r01.c: 255: gtemp[0]=0x40;
	ldw	x, (0x0e, sp)
	ld	a, #0x40
	ld	(x), a
;	../../stm8/se8r01.c: 256: gtemp[1]=0x01;
	ldw	x, (0x0c, sp)
	ld	a, #0x01
	ld	(x), a
;	../../stm8/se8r01.c: 257: gtemp[2]=0x30;
	ldw	x, (0x0a, sp)
	ld	a, #0x30
	ld	(x), a
;	../../stm8/se8r01.c: 258: gtemp[3]=0xE2;
	ldw	x, (0x08, sp)
	ld	a, #0xe2
	ld	(x), a
;	../../stm8/se8r01.c: 259: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, gtemp, 4);
	ldw	x, (0x0e, sp)
	push	#0x04
	pushw	x
	push	#0x21
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 261: gtemp[0]=0x29;
	ldw	x, (0x0e, sp)
	ld	a, #0x29
	ld	(x), a
;	../../stm8/se8r01.c: 262: gtemp[1]=0x89;
	ldw	x, (0x0c, sp)
	ld	a, #0x89
	ld	(x), a
;	../../stm8/se8r01.c: 263: gtemp[2]=0x55;
	ldw	x, (0x0a, sp)
	ld	a, #0x55
	ld	(x), a
;	../../stm8/se8r01.c: 264: gtemp[3]=0x40;
	ldw	x, (0x08, sp)
	ld	a, #0x40
	ld	(x), a
;	../../stm8/se8r01.c: 265: gtemp[4]=0x50;
	ldw	x, (0x06, sp)
	ld	a, #0x50
	ld	(x), a
;	../../stm8/se8r01.c: 266: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, gtemp, 5);
	ldw	x, (0x0e, sp)
	push	#0x05
	pushw	x
	push	#0x23
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 268: gtemp[0]=0x29;
	ldw	x, (0x0e, sp)
	ld	a, #0x29
	ld	(x), a
;	../../stm8/se8r01.c: 269: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, gtemp, 1);
	ldw	x, (0x0e, sp)
	push	#0x01
	pushw	x
	push	#0x2c
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 271: gtemp[0]=0x55;
	ldw	x, (0x0e, sp)
	ld	a, #0x55
	ld	(x), a
;	../../stm8/se8r01.c: 272: gtemp[1]=0xC2;
	ldw	x, (0x0c, sp)
	ld	a, #0xc2
	ld	(x), a
;	../../stm8/se8r01.c: 273: gtemp[2]=0x09;
	ldw	x, (0x0a, sp)
	ld	a, #0x09
	ld	(x), a
;	../../stm8/se8r01.c: 274: gtemp[3]=0xAC;
	ldw	x, (0x08, sp)
	ld	a, #0xac
	ld	(x), a
;	../../stm8/se8r01.c: 275: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_RX_CTRL, gtemp, 4);
	ldw	x, (0x0e, sp)
	push	#0x04
	pushw	x
	push	#0x31
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 277: gtemp[0]=0x00;
	ldw	x, (0x0e, sp)
	clr	(x)
;	../../stm8/se8r01.c: 278: gtemp[1]=0x14;
	ldw	x, (0x0c, sp)
	ld	a, #0x14
	ld	(x), a
;	../../stm8/se8r01.c: 279: gtemp[2]=0x08;
	ldw	x, (0x0a, sp)
	ld	a, #0x08
	ld	(x), a
;	../../stm8/se8r01.c: 280: gtemp[3]=0x29;
	ldw	x, (0x08, sp)
	ld	a, #0x29
	ld	(x), a
;	../../stm8/se8r01.c: 281: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_FAGC_CTRL_1, gtemp, 4);
	ldw	x, (0x0e, sp)
	push	#0x04
	pushw	x
	push	#0x33
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 283: gtemp[0]=0x02;
	ldw	x, (0x0e, sp)
	ld	a, #0x02
	ld	(x), a
;	../../stm8/se8r01.c: 284: gtemp[1]=0xC1;
	ldw	x, (0x0c, sp)
	ld	a, #0xc1
	ld	(x), a
;	../../stm8/se8r01.c: 285: gtemp[2]=0xCB;
	ldw	x, (0x0a, sp)
	ld	a, #0xcb
	ld	(x), a
;	../../stm8/se8r01.c: 286: gtemp[3]=0x1C;
	ldw	x, (0x08, sp)
	ld	a, #0x1c
	ld	(x), a
;	../../stm8/se8r01.c: 287: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, gtemp, 4);
	ldw	x, (0x0e, sp)
	push	#0x04
	pushw	x
	push	#0x3d
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 289: gtemp[0]=0x97;
	ldw	x, (0x0e, sp)
	ld	a, #0x97
	ld	(x), a
;	../../stm8/se8r01.c: 290: gtemp[1]=0x64;
	ldw	x, (0x0c, sp)
	ld	a, #0x64
	ld	(x), a
;	../../stm8/se8r01.c: 291: gtemp[2]=0x00;
	ldw	x, (0x0a, sp)
	clr	(x)
;	../../stm8/se8r01.c: 292: gtemp[3]=0x01;
	ldw	x, (0x08, sp)
	ld	a, #0x01
	ld	(x), a
;	../../stm8/se8r01.c: 293: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, gtemp, 4);
	ldw	x, (0x0e, sp)
	push	#0x04
	pushw	x
	push	#0x3e
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 295: gtemp[0]=0x2A;
	ldw	x, (0x0e, sp)
	ld	a, #0x2a
	ld	(x), a
;	../../stm8/se8r01.c: 296: gtemp[1]=0x04;
	ldw	x, (0x0c, sp)
	ld	a, #0x04
	ld	(x), a
;	../../stm8/se8r01.c: 297: gtemp[2]=0x00;
	ldw	x, (0x0a, sp)
	clr	(x)
;	../../stm8/se8r01.c: 298: gtemp[3]=0x7D;
	ldw	x, (0x08, sp)
	ld	a, #0x7d
	ld	(x), a
;	../../stm8/se8r01.c: 299: SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_TEST_PKDET, gtemp, 4);
	ldw	x, (0x0e, sp)
	push	#0x04
	pushw	x
	push	#0x3f
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 301: rf_switch_bank(iBANK0);
	push	#0x00
	call	_rf_switch_bank
	addw	sp, #16
	ret
;	../../stm8/se8r01.c: 305: void se8r01_settings() {
;	-----------------------------------------
;	 function se8r01_settings
;	-----------------------------------------
_se8r01_settings:
	sub	sp, #6
;	../../stm8/se8r01.c: 306: uint8_t TX_ADDRESS[TX_ADR_WIDTH]  = 
	ldw	x, sp
	incw	x
	ldw	(0x05, sp), x
	ld	a, #0x34
	ld	(x), a
	ldw	x, (0x05, sp)
	incw	x
	ld	a, #0x43
	ld	(x), a
	ldw	x, (0x05, sp)
	incw	x
	incw	x
	ld	a, #0x10
	ld	(x), a
	ldw	x, (0x05, sp)
	ld	a, #0x10
	ld	(0x0003, x), a
;	../../stm8/se8r01.c: 311: SPI_write_read_reg(WRITE_REG|iRF_BANK0_EN_AA, 0x01);          //enable auto acc on pip 1
	push	#0x01
	push	#0x21
	call	_SPI_write_read_reg
	addw	sp, #2
;	../../stm8/se8r01.c: 312: SPI_write_read_reg(WRITE_REG|iRF_BANK0_EN_RXADDR, 0x01);      //enable pip 1
	push	#0x01
	push	#0x22
	call	_SPI_write_read_reg
	addw	sp, #2
;	../../stm8/se8r01.c: 313: SPI_write_read_reg(WRITE_REG|iRF_BANK0_SETUP_AW, 0x02);        //4 byte adress
	push	#0x02
	push	#0x23
	call	_SPI_write_read_reg
	addw	sp, #2
;	../../stm8/se8r01.c: 315: SPI_write_read_reg(WRITE_REG|iRF_BANK0_SETUP_RETR, 0x0a);        //lowest 4 bits 0-15 rt transmisston higest 4 bits 256-4096us Auto Retransmit Delay
	push	#0x0a
	push	#0x24
	call	_SPI_write_read_reg
	addw	sp, #2
;	../../stm8/se8r01.c: 316: SPI_write_read_reg(WRITE_REG|iRF_BANK0_RF_CH, 40);
	push	#0x28
	push	#0x25
	call	_SPI_write_read_reg
	addw	sp, #2
;	../../stm8/se8r01.c: 317: SPI_write_read_reg(WRITE_REG|iRF_BANK0_RF_SETUP, 0x5f);        //500kps 0x4f
	push	#0x5f
	push	#0x26
	call	_SPI_write_read_reg
	addw	sp, #2
;	../../stm8/se8r01.c: 321: SPI_write_buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);  //from tx
	ldw	x, (0x05, sp)
	push	#0x04
	pushw	x
	push	#0x30
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 322: SPI_write_buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // Use the same address on the RX device as the TX device
	ldw	x, (0x05, sp)
	push	#0x04
	pushw	x
	push	#0x2a
	call	_SPI_write_buf
	addw	sp, #4
;	../../stm8/se8r01.c: 323: SPI_write_read_reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width
	push	#0x06
	push	#0x31
	call	_SPI_write_read_reg
	addw	sp, #8
	ret
;	../../stm8/se8r01.c: 327: void se8r01_flush_TX() {
;	-----------------------------------------
;	 function se8r01_flush_TX
;	-----------------------------------------
_se8r01_flush_TX:
;	../../stm8/se8r01.c: 328: SPI_write_read_reg(FLUSH_TX, 0xff);
	push	#0xff
	push	#0xe1
	call	_SPI_write_read_reg
	addw	sp, #2
	ret
;	../../stm8/se8r01.c: 331: void se8r01_flush_RX() {
;	-----------------------------------------
;	 function se8r01_flush_RX
;	-----------------------------------------
_se8r01_flush_RX:
;	../../stm8/se8r01.c: 332: SPI_write_read_reg(FLUSH_RX, 0xff);
	push	#0xff
	push	#0xe2
	call	_SPI_write_read_reg
	addw	sp, #2
	ret
	.area CODE
___str_0:
	.ascii "Initializing SPI"
	.db 0x0a
	.db 0x00
___str_1:
	.ascii "Status: %02x "
	.db 0x0a
	.db 0x00
___str_2:
	.ascii "RX mode"
	.db 0x0a
	.db 0x00
___str_3:
	.ascii "TX mode, status %02x"
	.db 0x0a
	.db 0x00
___str_4:
	.ascii "connected? %02x"
	.db 0x0a
	.db 0x00
___str_5:
	.ascii "clearing irq flag"
	.db 0x0a
	.db 0x00
___str_6:
	.ascii "Temp 0: %02x "
	.db 0x0a
	.db 0x00
	.area INITIALIZER
	.area CABS (ABS)
