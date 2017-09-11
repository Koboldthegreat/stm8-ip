                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.6.8 #9946 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module main
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _main
                                     12 	.globl _se8r01_tx
                                     13 	.globl _se8r01_init
                                     14 	.globl _uart_init
                                     15 	.globl _printf
                                     16 ;--------------------------------------------------------
                                     17 ; ram data
                                     18 ;--------------------------------------------------------
                                     19 	.area DATA
                                     20 ;--------------------------------------------------------
                                     21 ; ram data
                                     22 ;--------------------------------------------------------
                                     23 	.area INITIALIZED
                                     24 ;--------------------------------------------------------
                                     25 ; Stack segment in internal ram 
                                     26 ;--------------------------------------------------------
                                     27 	.area	SSEG
      000001                         28 __start__stack:
      000001                         29 	.ds	1
                                     30 
                                     31 ;--------------------------------------------------------
                                     32 ; absolute external ram data
                                     33 ;--------------------------------------------------------
                                     34 	.area DABS (ABS)
                                     35 ;--------------------------------------------------------
                                     36 ; interrupt vector 
                                     37 ;--------------------------------------------------------
                                     38 	.area HOME
      008000                         39 __interrupt_vect:
      008000 82 00 80 07             40 	int s_GSINIT ; reset
                                     41 ;--------------------------------------------------------
                                     42 ; global & static initialisations
                                     43 ;--------------------------------------------------------
                                     44 	.area HOME
                                     45 	.area GSINIT
                                     46 	.area GSFINAL
                                     47 	.area GSINIT
      008007                         48 __sdcc_gs_init_startup:
      008007                         49 __sdcc_init_data:
                                     50 ; stm8_genXINIT() start
      008007 AE 00 00         [ 2]   51 	ldw x, #l_DATA
      00800A 27 07            [ 1]   52 	jreq	00002$
      00800C                         53 00001$:
      00800C 72 4F 00 00      [ 1]   54 	clr (s_DATA - 1, x)
      008010 5A               [ 2]   55 	decw x
      008011 26 F9            [ 1]   56 	jrne	00001$
      008013                         57 00002$:
      008013 AE 00 00         [ 2]   58 	ldw	x, #l_INITIALIZER
      008016 27 09            [ 1]   59 	jreq	00004$
      008018                         60 00003$:
      008018 D6 92 24         [ 1]   61 	ld	a, (s_INITIALIZER - 1, x)
      00801B D7 00 00         [ 1]   62 	ld	(s_INITIALIZED - 1, x), a
      00801E 5A               [ 2]   63 	decw	x
      00801F 26 F7            [ 1]   64 	jrne	00003$
      008021                         65 00004$:
                                     66 ; stm8_genXINIT() end
                                     67 	.area GSFINAL
      008021 CC 80 04         [ 2]   68 	jp	__sdcc_program_startup
                                     69 ;--------------------------------------------------------
                                     70 ; Home
                                     71 ;--------------------------------------------------------
                                     72 	.area HOME
                                     73 	.area HOME
      008004                         74 __sdcc_program_startup:
      008004 CC 80 81         [ 2]   75 	jp	_main
                                     76 ;	return from main will return to caller
                                     77 ;--------------------------------------------------------
                                     78 ; code
                                     79 ;--------------------------------------------------------
                                     80 	.area CODE
                                     81 ;	../../stm8/delay.h: 11: static inline void delay_ms(uint32_t ms) {
                                     82 ;	-----------------------------------------
                                     83 ;	 function delay_ms
                                     84 ;	-----------------------------------------
      008024                         85 _delay_ms:
      008024 52 08            [ 2]   86 	sub	sp, #8
                                     87 ;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      008026 5F               [ 1]   88 	clrw	x
      008027 1F 03            [ 2]   89 	ldw	(0x03, sp), x
      008029 1F 01            [ 2]   90 	ldw	(0x01, sp), x
      00802B 1E 0D            [ 2]   91 	ldw	x, (0x0d, sp)
      00802D 89               [ 2]   92 	pushw	x
      00802E 1E 0D            [ 2]   93 	ldw	x, (0x0d, sp)
      008030 89               [ 2]   94 	pushw	x
      008031 4B 6F            [ 1]   95 	push	#0x6f
      008033 5F               [ 1]   96 	clrw	x
      008034 89               [ 2]   97 	pushw	x
      008035 4B 00            [ 1]   98 	push	#0x00
      008037 CD 8A 66         [ 4]   99 	call	__mullong
      00803A 5B 08            [ 2]  100 	addw	sp, #8
      00803C 1F 07            [ 2]  101 	ldw	(0x07, sp), x
      00803E 17 05            [ 2]  102 	ldw	(0x05, sp), y
      008040                        103 00103$:
      008040 1E 03            [ 2]  104 	ldw	x, (0x03, sp)
      008042 13 07            [ 2]  105 	cpw	x, (0x07, sp)
      008044 7B 02            [ 1]  106 	ld	a, (0x02, sp)
      008046 12 06            [ 1]  107 	sbc	a, (0x06, sp)
      008048 7B 01            [ 1]  108 	ld	a, (0x01, sp)
      00804A 12 05            [ 1]  109 	sbc	a, (0x05, sp)
      00804C 24 17            [ 1]  110 	jrnc	00105$
                                    111 ;	../../stm8/delay.h: 13: __asm__("nop");
      00804E 9D               [ 1]  112 	nop
                                    113 ;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      00804F 16 03            [ 2]  114 	ldw	y, (0x03, sp)
      008051 72 A9 00 01      [ 2]  115 	addw	y, #0x0001
      008055 7B 02            [ 1]  116 	ld	a, (0x02, sp)
      008057 A9 00            [ 1]  117 	adc	a, #0x00
      008059 97               [ 1]  118 	ld	xl, a
      00805A 7B 01            [ 1]  119 	ld	a, (0x01, sp)
      00805C A9 00            [ 1]  120 	adc	a, #0x00
      00805E 95               [ 1]  121 	ld	xh, a
      00805F 17 03            [ 2]  122 	ldw	(0x03, sp), y
      008061 1F 01            [ 2]  123 	ldw	(0x01, sp), x
      008063 20 DB            [ 2]  124 	jra	00103$
      008065                        125 00105$:
      008065 5B 08            [ 2]  126 	addw	sp, #8
      008067 81               [ 4]  127 	ret
                                    128 ;	../../stm8/delay.h: 17: static inline void delay_us(uint32_t us) {
                                    129 ;	-----------------------------------------
                                    130 ;	 function delay_us
                                    131 ;	-----------------------------------------
      008068                        132 _delay_us:
                                    133 ;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
      008068 90 5F            [ 1]  134 	clrw	y
      00806A 5F               [ 1]  135 	clrw	x
      00806B                        136 00103$:
      00806B 90 A3 00 00      [ 2]  137 	cpw	y, #0x0000
      00806F 9F               [ 1]  138 	ld	a, xl
      008070 A2 00            [ 1]  139 	sbc	a, #0x00
      008072 9E               [ 1]  140 	ld	a, xh
      008073 A2 00            [ 1]  141 	sbc	a, #0x00
      008075 25 01            [ 1]  142 	jrc	00115$
      008077 81               [ 4]  143 	ret
      008078                        144 00115$:
                                    145 ;	../../stm8/delay.h: 19: __asm__("nop");
      008078 9D               [ 1]  146 	nop
                                    147 ;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
      008079 90 5C            [ 1]  148 	incw	y
      00807B 26 EE            [ 1]  149 	jrne	00103$
      00807D 5C               [ 1]  150 	incw	x
      00807E 20 EB            [ 2]  151 	jra	00103$
      008080 81               [ 4]  152 	ret
                                    153 ;	main.c: 10: void main() {
                                    154 ;	-----------------------------------------
                                    155 ;	 function main
                                    156 ;	-----------------------------------------
      008081                        157 _main:
      008081 52 0E            [ 2]  158 	sub	sp, #14
                                    159 ;	main.c: 11: uint8_t counter = 0;
      008083 0F 0B            [ 1]  160 	clr	(0x0b, sp)
                                    161 ;	main.c: 15: CLK_CKDIVR = 3;
      008085 35 03 50 C6      [ 1]  162 	mov	0x50c6+0, #0x03
                                    163 ;	main.c: 17: uart_init();
      008089 CD 88 1C         [ 4]  164 	call	_uart_init
                                    165 ;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      00808C 90 5F            [ 1]  166 	clrw	y
      00808E 5F               [ 1]  167 	clrw	x
      00808F                        168 00110$:
      00808F 90 A3 2B 5C      [ 2]  169 	cpw	y, #0x2b5c
      008093 9F               [ 1]  170 	ld	a, xl
      008094 A2 00            [ 1]  171 	sbc	a, #0x00
      008096 9E               [ 1]  172 	ld	a, xh
      008097 A2 00            [ 1]  173 	sbc	a, #0x00
      008099 24 08            [ 1]  174 	jrnc	00106$
                                    175 ;	../../stm8/delay.h: 13: __asm__("nop");
      00809B 9D               [ 1]  176 	nop
                                    177 ;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      00809C 90 5C            [ 1]  178 	incw	y
      00809E 26 EF            [ 1]  179 	jrne	00110$
      0080A0 5C               [ 1]  180 	incw	x
      0080A1 20 EC            [ 2]  181 	jra	00110$
                                    182 ;	main.c: 18: delay_ms(100);
      0080A3                        183 00106$:
                                    184 ;	main.c: 19: printf("Hello\n");
      0080A3 AE 81 06         [ 2]  185 	ldw	x, #___str_0+0
      0080A6 89               [ 2]  186 	pushw	x
      0080A7 CD 8A 50         [ 4]  187 	call	_printf
      0080AA 5B 02            [ 2]  188 	addw	sp, #2
                                    189 ;	main.c: 21: se8r01_init('t');
      0080AC 4B 74            [ 1]  190 	push	#0x74
      0080AE CD 82 0D         [ 4]  191 	call	_se8r01_init
      0080B1 84               [ 1]  192 	pop	a
                                    193 ;	main.c: 23: while (1) {
      0080B2                        194 00103$:
                                    195 ;	main.c: 24: for (uint8_t i=0; i<TX_PLOAD_WIDTH; i++)
      0080B2 0F 04            [ 1]  196 	clr	(0x04, sp)
      0080B4 96               [ 1]  197 	ldw	x, sp
      0080B5 1C 00 05         [ 2]  198 	addw	x, #5
      0080B8 1F 0D            [ 2]  199 	ldw	(0x0d, sp), x
      0080BA                        200 00113$:
      0080BA 7B 04            [ 1]  201 	ld	a, (0x04, sp)
      0080BC A1 06            [ 1]  202 	cp	a, #0x06
      0080BE 24 14            [ 1]  203 	jrnc	00101$
                                    204 ;	main.c: 25: tx_buf[i] = counter++;
      0080C0 5F               [ 1]  205 	clrw	x
      0080C1 7B 04            [ 1]  206 	ld	a, (0x04, sp)
      0080C3 97               [ 1]  207 	ld	xl, a
      0080C4 72 FB 0D         [ 2]  208 	addw	x, (0x0d, sp)
      0080C7 7B 0B            [ 1]  209 	ld	a, (0x0b, sp)
      0080C9 6B 0C            [ 1]  210 	ld	(0x0c, sp), a
      0080CB 0C 0B            [ 1]  211 	inc	(0x0b, sp)
      0080CD 7B 0C            [ 1]  212 	ld	a, (0x0c, sp)
      0080CF F7               [ 1]  213 	ld	(x), a
                                    214 ;	main.c: 24: for (uint8_t i=0; i<TX_PLOAD_WIDTH; i++)
      0080D0 0C 04            [ 1]  215 	inc	(0x04, sp)
      0080D2 20 E6            [ 2]  216 	jra	00113$
      0080D4                        217 00101$:
                                    218 ;	main.c: 26: se8r01_tx(tx_buf);
      0080D4 1E 0D            [ 2]  219 	ldw	x, (0x0d, sp)
      0080D6 89               [ 2]  220 	pushw	x
      0080D7 CD 83 1C         [ 4]  221 	call	_se8r01_tx
      0080DA 5B 02            [ 2]  222 	addw	sp, #2
                                    223 ;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0080DC 5F               [ 1]  224 	clrw	x
      0080DD 0F 02            [ 1]  225 	clr	(0x02, sp)
      0080DF 4F               [ 1]  226 	clr	a
      0080E0                        227 00116$:
      0080E0 88               [ 1]  228 	push	a
      0080E1 A3 15 AE         [ 2]  229 	cpw	x, #0x15ae
      0080E4 7B 03            [ 1]  230 	ld	a, (0x03, sp)
      0080E6 A2 00            [ 1]  231 	sbc	a, #0x00
      0080E8 7B 01            [ 1]  232 	ld	a, (1, sp)
      0080EA A2 00            [ 1]  233 	sbc	a, #0x00
      0080EC 84               [ 1]  234 	pop	a
      0080ED 24 C3            [ 1]  235 	jrnc	00103$
                                    236 ;	../../stm8/delay.h: 13: __asm__("nop");
      0080EF 9D               [ 1]  237 	nop
                                    238 ;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0080F0 1C 00 01         [ 2]  239 	addw	x, #0x0001
      0080F3 88               [ 1]  240 	push	a
      0080F4 7B 03            [ 1]  241 	ld	a, (0x03, sp)
      0080F6 A9 00            [ 1]  242 	adc	a, #0x00
      0080F8 90 97            [ 1]  243 	ld	yl, a
      0080FA 84               [ 1]  244 	pop	a
      0080FB A9 00            [ 1]  245 	adc	a, #0x00
      0080FD 61               [ 1]  246 	exg	a, yl
      0080FE 6B 02            [ 1]  247 	ld	(0x02, sp), a
      008100 61               [ 1]  248 	exg	a, yl
      008101 20 DD            [ 2]  249 	jra	00116$
                                    250 ;	main.c: 32: delay_ms(50);
      008103 5B 0E            [ 2]  251 	addw	sp, #14
      008105 81               [ 4]  252 	ret
                                    253 	.area CODE
      008106                        254 ___str_0:
      008106 48 65 6C 6C 6F         255 	.ascii "Hello"
      00810B 0A                     256 	.db 0x0a
      00810C 00                     257 	.db 0x00
                                    258 	.area INITIALIZER
                                    259 	.area CABS (ABS)
