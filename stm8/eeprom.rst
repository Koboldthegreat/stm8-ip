                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.6.8 #9946 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module eeprom
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _eeprom_unlock
                                     12 	.globl _option_bytes_unlock
                                     13 	.globl _eeprom_lock
                                     14 	.globl _eeprom_wait_busy
                                     15 ;--------------------------------------------------------
                                     16 ; ram data
                                     17 ;--------------------------------------------------------
                                     18 	.area DATA
                                     19 ;--------------------------------------------------------
                                     20 ; ram data
                                     21 ;--------------------------------------------------------
                                     22 	.area INITIALIZED
                                     23 ;--------------------------------------------------------
                                     24 ; absolute external ram data
                                     25 ;--------------------------------------------------------
                                     26 	.area DABS (ABS)
                                     27 ;--------------------------------------------------------
                                     28 ; global & static initialisations
                                     29 ;--------------------------------------------------------
                                     30 	.area HOME
                                     31 	.area GSINIT
                                     32 	.area GSFINAL
                                     33 	.area GSINIT
                                     34 ;--------------------------------------------------------
                                     35 ; Home
                                     36 ;--------------------------------------------------------
                                     37 	.area HOME
                                     38 	.area HOME
                                     39 ;--------------------------------------------------------
                                     40 ; code
                                     41 ;--------------------------------------------------------
                                     42 	.area CODE
                                     43 ;	../../stm8/eeprom.c: 3: void eeprom_unlock() {
                                     44 ;	-----------------------------------------
                                     45 ;	 function eeprom_unlock
                                     46 ;	-----------------------------------------
      008966                         47 _eeprom_unlock:
                                     48 ;	../../stm8/eeprom.c: 4: FLASH_DUKR = FLASH_DUKR_KEY1;
      008966 35 AE 50 64      [ 1]   49 	mov	0x5064+0, #0xae
                                     50 ;	../../stm8/eeprom.c: 5: FLASH_DUKR = FLASH_DUKR_KEY2;
      00896A 35 56 50 64      [ 1]   51 	mov	0x5064+0, #0x56
                                     52 ;	../../stm8/eeprom.c: 6: while (!(FLASH_IAPSR & (1 << FLASH_IAPSR_DUL)));
      00896E                         53 00101$:
      00896E AE 50 5F         [ 2]   54 	ldw	x, #0x505f
      008971 F6               [ 1]   55 	ld	a, (x)
      008972 A5 08            [ 1]   56 	bcp	a, #0x08
      008974 27 F8            [ 1]   57 	jreq	00101$
      008976 81               [ 4]   58 	ret
                                     59 ;	../../stm8/eeprom.c: 9: void option_bytes_unlock() {
                                     60 ;	-----------------------------------------
                                     61 ;	 function option_bytes_unlock
                                     62 ;	-----------------------------------------
      008977                         63 _option_bytes_unlock:
                                     64 ;	../../stm8/eeprom.c: 10: FLASH_CR2 |= (1 << FLASH_CR2_OPT);
      008977 72 1E 50 5B      [ 1]   65 	bset	0x505b, #7
                                     66 ;	../../stm8/eeprom.c: 11: FLASH_NCR2 &= ~(1 << FLASH_NCR2_NOPT);
      00897B 72 1F 50 5C      [ 1]   67 	bres	0x505c, #7
      00897F 81               [ 4]   68 	ret
                                     69 ;	../../stm8/eeprom.c: 14: void eeprom_lock() {
                                     70 ;	-----------------------------------------
                                     71 ;	 function eeprom_lock
                                     72 ;	-----------------------------------------
      008980                         73 _eeprom_lock:
                                     74 ;	../../stm8/eeprom.c: 15: FLASH_IAPSR &= ~(1 << FLASH_IAPSR_DUL);
      008980 AE 50 5F         [ 2]   75 	ldw	x, #0x505f
      008983 F6               [ 1]   76 	ld	a, (x)
      008984 A4 F7            [ 1]   77 	and	a, #0xf7
      008986 F7               [ 1]   78 	ld	(x), a
      008987 81               [ 4]   79 	ret
                                     80 ;	../../stm8/eeprom.c: 18: void eeprom_wait_busy() {
                                     81 ;	-----------------------------------------
                                     82 ;	 function eeprom_wait_busy
                                     83 ;	-----------------------------------------
      008988                         84 _eeprom_wait_busy:
                                     85 ;	../../stm8/eeprom.c: 19: while (!(FLASH_IAPSR & (1 << FLASH_IAPSR_EOP)));
      008988                         86 00101$:
      008988 AE 50 5F         [ 2]   87 	ldw	x, #0x505f
      00898B F6               [ 1]   88 	ld	a, (x)
      00898C A5 04            [ 1]   89 	bcp	a, #0x04
      00898E 27 F8            [ 1]   90 	jreq	00101$
      008990 81               [ 4]   91 	ret
                                     92 	.area CODE
                                     93 	.area INITIALIZER
                                     94 	.area CABS (ABS)
