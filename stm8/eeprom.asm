;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.6.8 #9946 (Linux)
;--------------------------------------------------------
	.module eeprom
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _eeprom_unlock
	.globl _option_bytes_unlock
	.globl _eeprom_lock
	.globl _eeprom_wait_busy
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
;	../../stm8/eeprom.c: 3: void eeprom_unlock() {
;	-----------------------------------------
;	 function eeprom_unlock
;	-----------------------------------------
_eeprom_unlock:
;	../../stm8/eeprom.c: 4: FLASH_DUKR = FLASH_DUKR_KEY1;
	mov	0x5064+0, #0xae
;	../../stm8/eeprom.c: 5: FLASH_DUKR = FLASH_DUKR_KEY2;
	mov	0x5064+0, #0x56
;	../../stm8/eeprom.c: 6: while (!(FLASH_IAPSR & (1 << FLASH_IAPSR_DUL)));
00101$:
	ldw	x, #0x505f
	ld	a, (x)
	bcp	a, #0x08
	jreq	00101$
	ret
;	../../stm8/eeprom.c: 9: void option_bytes_unlock() {
;	-----------------------------------------
;	 function option_bytes_unlock
;	-----------------------------------------
_option_bytes_unlock:
;	../../stm8/eeprom.c: 10: FLASH_CR2 |= (1 << FLASH_CR2_OPT);
	bset	0x505b, #7
;	../../stm8/eeprom.c: 11: FLASH_NCR2 &= ~(1 << FLASH_NCR2_NOPT);
	bres	0x505c, #7
	ret
;	../../stm8/eeprom.c: 14: void eeprom_lock() {
;	-----------------------------------------
;	 function eeprom_lock
;	-----------------------------------------
_eeprom_lock:
;	../../stm8/eeprom.c: 15: FLASH_IAPSR &= ~(1 << FLASH_IAPSR_DUL);
	ldw	x, #0x505f
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
	ret
;	../../stm8/eeprom.c: 18: void eeprom_wait_busy() {
;	-----------------------------------------
;	 function eeprom_wait_busy
;	-----------------------------------------
_eeprom_wait_busy:
;	../../stm8/eeprom.c: 19: while (!(FLASH_IAPSR & (1 << FLASH_IAPSR_EOP)));
00101$:
	ldw	x, #0x505f
	ld	a, (x)
	bcp	a, #0x04
	jreq	00101$
	ret
	.area CODE
	.area INITIALIZER
	.area CABS (ABS)
