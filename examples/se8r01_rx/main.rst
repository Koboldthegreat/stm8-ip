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
                                     12 	.globl _se8r01_wait_rx
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
      008018 D6 93 22         [ 1]   61 	ld	a, (s_INITIALIZER - 1, x)
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
      008037 CD 8B 50         [ 4]   99 	call	__mullong
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
      008081 52 1E            [ 2]  158 	sub	sp, #30
                                    159 ;	main.c: 11: uint8_t rx_buf[TX_PLOAD_WIDTH] = {0};
      008083 96               [ 1]  160 	ldw	x, sp
      008084 1C 00 0E         [ 2]  161 	addw	x, #14
      008087 1F 18            [ 2]  162 	ldw	(0x18, sp), x
      008089 7F               [ 1]  163 	clr	(x)
      00808A 1E 18            [ 2]  164 	ldw	x, (0x18, sp)
      00808C 5C               [ 1]  165 	incw	x
      00808D 7F               [ 1]  166 	clr	(x)
      00808E 1E 18            [ 2]  167 	ldw	x, (0x18, sp)
      008090 5C               [ 1]  168 	incw	x
      008091 5C               [ 1]  169 	incw	x
      008092 7F               [ 1]  170 	clr	(x)
      008093 1E 18            [ 2]  171 	ldw	x, (0x18, sp)
      008095 1C 00 03         [ 2]  172 	addw	x, #0x0003
      008098 7F               [ 1]  173 	clr	(x)
      008099 1E 18            [ 2]  174 	ldw	x, (0x18, sp)
      00809B 1C 00 04         [ 2]  175 	addw	x, #0x0004
      00809E 7F               [ 1]  176 	clr	(x)
      00809F 1E 18            [ 2]  177 	ldw	x, (0x18, sp)
      0080A1 1C 00 05         [ 2]  178 	addw	x, #0x0005
      0080A4 1F 1A            [ 2]  179 	ldw	(0x1a, sp), x
      0080A6 7F               [ 1]  180 	clr	(x)
                                    181 ;	main.c: 12: uint32_t total = 0;
      0080A7 5F               [ 1]  182 	clrw	x
      0080A8 1F 0C            [ 2]  183 	ldw	(0x0c, sp), x
      0080AA 1F 0A            [ 2]  184 	ldw	(0x0a, sp), x
                                    185 ;	main.c: 13: uint32_t dropped = 0;
      0080AC 5F               [ 1]  186 	clrw	x
      0080AD 1F 08            [ 2]  187 	ldw	(0x08, sp), x
      0080AF 1F 06            [ 2]  188 	ldw	(0x06, sp), x
                                    189 ;	main.c: 14: uint8_t last = 0;
      0080B1 0F 05            [ 1]  190 	clr	(0x05, sp)
                                    191 ;	main.c: 16: CLK_CKDIVR = 3;
      0080B3 35 03 50 C6      [ 1]  192 	mov	0x50c6+0, #0x03
                                    193 ;	main.c: 18: uart_init();
      0080B7 CD 88 ED         [ 4]  194 	call	_uart_init
                                    195 ;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0080BA 0F 04            [ 1]  196 	clr	(0x04, sp)
      0080BC 0F 03            [ 1]  197 	clr	(0x03, sp)
      0080BE 4F               [ 1]  198 	clr	a
      0080BF 02               [ 1]  199 	rlwa	x
      0080C0 4F               [ 1]  200 	clr	a
      0080C1 01               [ 1]  201 	rrwa	x
      0080C2                        202 00115$:
      0080C2 88               [ 1]  203 	push	a
      0080C3 7B 05            [ 1]  204 	ld	a, (0x05, sp)
      0080C5 A1 5C            [ 1]  205 	cp	a, #0x5c
      0080C7 7B 04            [ 1]  206 	ld	a, (0x04, sp)
      0080C9 A2 2B            [ 1]  207 	sbc	a, #0x2b
      0080CB 7B 01            [ 1]  208 	ld	a, (1, sp)
      0080CD A2 00            [ 1]  209 	sbc	a, #0x00
      0080CF 9E               [ 1]  210 	ld	a, xh
      0080D0 A2 00            [ 1]  211 	sbc	a, #0x00
      0080D2 84               [ 1]  212 	pop	a
      0080D3 24 13            [ 1]  213 	jrnc	00111$
                                    214 ;	../../stm8/delay.h: 13: __asm__("nop");
      0080D5 9D               [ 1]  215 	nop
                                    216 ;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0080D6 16 03            [ 2]  217 	ldw	y, (0x03, sp)
      0080D8 72 A9 00 01      [ 2]  218 	addw	y, #0x0001
      0080DC A9 00            [ 1]  219 	adc	a, #0x00
      0080DE 88               [ 1]  220 	push	a
      0080DF 9E               [ 1]  221 	ld	a, xh
      0080E0 A9 00            [ 1]  222 	adc	a, #0x00
      0080E2 95               [ 1]  223 	ld	xh, a
      0080E3 84               [ 1]  224 	pop	a
      0080E4 17 03            [ 2]  225 	ldw	(0x03, sp), y
      0080E6 20 DA            [ 2]  226 	jra	00115$
                                    227 ;	main.c: 19: delay_ms(100);
      0080E8                        228 00111$:
                                    229 ;	main.c: 21: se8r01_init('r');
      0080E8 4B 72            [ 1]  230 	push	#0x72
      0080EA CD 82 DE         [ 4]  231 	call	_se8r01_init
      0080ED 84               [ 1]  232 	pop	a
                                    233 ;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0080EE 90 5F            [ 1]  234 	clrw	y
      0080F0 5F               [ 1]  235 	clrw	x
      0080F1                        236 00118$:
      0080F1 90 A3 B1 98      [ 2]  237 	cpw	y, #0xb198
      0080F5 9F               [ 1]  238 	ld	a, xl
      0080F6 A2 01            [ 1]  239 	sbc	a, #0x01
      0080F8 9E               [ 1]  240 	ld	a, xh
      0080F9 A2 00            [ 1]  241 	sbc	a, #0x00
      0080FB 24 08            [ 1]  242 	jrnc	00108$
                                    243 ;	../../stm8/delay.h: 13: __asm__("nop");
      0080FD 9D               [ 1]  244 	nop
                                    245 ;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0080FE 90 5C            [ 1]  246 	incw	y
      008100 26 EF            [ 1]  247 	jrne	00118$
      008102 5C               [ 1]  248 	incw	x
      008103 20 EC            [ 2]  249 	jra	00118$
                                    250 ;	main.c: 24: while (1) {
      008105                        251 00108$:
                                    252 ;	main.c: 25: if (se8r01_wait_rx(rx_buf)){
      008105 1E 18            [ 2]  253 	ldw	x, (0x18, sp)
      008107 89               [ 2]  254 	pushw	x
      008108 CD 83 A1         [ 4]  255 	call	_se8r01_wait_rx
      00810B 5B 02            [ 2]  256 	addw	sp, #2
      00810D 6B 1E            [ 1]  257 	ld	(0x1e, sp), a
                                    258 ;	main.c: 26: PD_ODR |= (1<<4);
      00810F AE 50 0F         [ 2]  259 	ldw	x, #0x500f
      008112 F6               [ 1]  260 	ld	a, (x)
      008113 6B 14            [ 1]  261 	ld	(0x14, sp), a
                                    262 ;	main.c: 25: if (se8r01_wait_rx(rx_buf)){
      008115 0D 1E            [ 1]  263 	tnz	(0x1e, sp)
      008117 26 03            [ 1]  264 	jrne	00157$
      008119 CC 81 AE         [ 2]  265 	jp	00105$
      00811C                        266 00157$:
                                    267 ;	main.c: 26: PD_ODR |= (1<<4);
      00811C 7B 14            [ 1]  268 	ld	a, (0x14, sp)
      00811E AA 10            [ 1]  269 	or	a, #0x10
      008120 AE 50 0F         [ 2]  270 	ldw	x, #0x500f
      008123 F7               [ 1]  271 	ld	(x), a
                                    272 ;	main.c: 32: if (rx_buf[TX_PLOAD_WIDTH-1] != last+6 && last != 0){
      008124 1E 1A            [ 2]  273 	ldw	x, (0x1a, sp)
      008126 F6               [ 1]  274 	ld	a, (x)
      008127 6B 15            [ 1]  275 	ld	(0x15, sp), a
      008129 5F               [ 1]  276 	clrw	x
      00812A 7B 05            [ 1]  277 	ld	a, (0x05, sp)
      00812C 97               [ 1]  278 	ld	xl, a
      00812D 1C 00 06         [ 2]  279 	addw	x, #0x0006
      008130 1F 1C            [ 2]  280 	ldw	(0x1c, sp), x
      008132 5F               [ 1]  281 	clrw	x
      008133 7B 15            [ 1]  282 	ld	a, (0x15, sp)
      008135 97               [ 1]  283 	ld	xl, a
      008136 13 1C            [ 2]  284 	cpw	x, (0x1c, sp)
      008138 27 18            [ 1]  285 	jreq	00102$
      00813A 0D 05            [ 1]  286 	tnz	(0x05, sp)
      00813C 27 14            [ 1]  287 	jreq	00102$
                                    288 ;	main.c: 33: dropped++;
      00813E 16 08            [ 2]  289 	ldw	y, (0x08, sp)
      008140 72 A9 00 01      [ 2]  290 	addw	y, #0x0001
      008144 7B 07            [ 1]  291 	ld	a, (0x07, sp)
      008146 A9 00            [ 1]  292 	adc	a, #0x00
      008148 97               [ 1]  293 	ld	xl, a
      008149 7B 06            [ 1]  294 	ld	a, (0x06, sp)
      00814B A9 00            [ 1]  295 	adc	a, #0x00
      00814D 95               [ 1]  296 	ld	xh, a
      00814E 17 08            [ 2]  297 	ldw	(0x08, sp), y
      008150 1F 06            [ 2]  298 	ldw	(0x06, sp), x
      008152                        299 00102$:
                                    300 ;	main.c: 35: total++;
      008152 16 0C            [ 2]  301 	ldw	y, (0x0c, sp)
      008154 72 A9 00 01      [ 2]  302 	addw	y, #0x0001
      008158 7B 0B            [ 1]  303 	ld	a, (0x0b, sp)
      00815A A9 00            [ 1]  304 	adc	a, #0x00
      00815C 97               [ 1]  305 	ld	xl, a
      00815D 7B 0A            [ 1]  306 	ld	a, (0x0a, sp)
      00815F A9 00            [ 1]  307 	adc	a, #0x00
      008161 95               [ 1]  308 	ld	xh, a
      008162 17 0C            [ 2]  309 	ldw	(0x0c, sp), y
      008164 1F 0A            [ 2]  310 	ldw	(0x0a, sp), x
                                    311 ;	main.c: 36: last = rx_buf[TX_PLOAD_WIDTH-1];
      008166 7B 15            [ 1]  312 	ld	a, (0x15, sp)
      008168 6B 05            [ 1]  313 	ld	(0x05, sp), a
                                    314 ;	main.c: 37: printf("total: %d, ", (int) total);
      00816A 16 0C            [ 2]  315 	ldw	y, (0x0c, sp)
      00816C 17 16            [ 2]  316 	ldw	(0x16, sp), y
      00816E AE 81 BC         [ 2]  317 	ldw	x, #___str_0+0
      008171 16 16            [ 2]  318 	ldw	y, (0x16, sp)
      008173 90 89            [ 2]  319 	pushw	y
      008175 89               [ 2]  320 	pushw	x
      008176 CD 8B 3A         [ 4]  321 	call	_printf
      008179 5B 04            [ 2]  322 	addw	sp, #4
                                    323 ;	main.c: 38: printf("dropped: %d, ", (int) dropped);
      00817B 1E 08            [ 2]  324 	ldw	x, (0x08, sp)
      00817D 90 AE 81 C8      [ 2]  325 	ldw	y, #___str_1+0
      008181 89               [ 2]  326 	pushw	x
      008182 89               [ 2]  327 	pushw	x
      008183 90 89            [ 2]  328 	pushw	y
      008185 CD 8B 3A         [ 4]  329 	call	_printf
      008188 5B 04            [ 2]  330 	addw	sp, #4
      00818A 85               [ 2]  331 	popw	x
                                    332 ;	main.c: 39: printf("->: %d\n", ((int) dropped*100)/(int) total);
      00818B 89               [ 2]  333 	pushw	x
      00818C 4B 64            [ 1]  334 	push	#0x64
      00818E 4B 00            [ 1]  335 	push	#0x00
      008190 CD 8B 04         [ 4]  336 	call	__mulint
      008193 5B 04            [ 2]  337 	addw	sp, #4
      008195 16 16            [ 2]  338 	ldw	y, (0x16, sp)
      008197 90 89            [ 2]  339 	pushw	y
      008199 89               [ 2]  340 	pushw	x
      00819A CD 8B CC         [ 4]  341 	call	__divsint
      00819D 5B 04            [ 2]  342 	addw	sp, #4
      00819F 90 AE 81 D6      [ 2]  343 	ldw	y, #___str_2+0
      0081A3 89               [ 2]  344 	pushw	x
      0081A4 90 89            [ 2]  345 	pushw	y
      0081A6 CD 8B 3A         [ 4]  346 	call	_printf
      0081A9 5B 04            [ 2]  347 	addw	sp, #4
      0081AB CC 81 05         [ 2]  348 	jp	00108$
      0081AE                        349 00105$:
                                    350 ;	main.c: 41: PD_ODR &= ~(1<<4);
      0081AE 7B 14            [ 1]  351 	ld	a, (0x14, sp)
      0081B0 A4 EF            [ 1]  352 	and	a, #0xef
      0081B2 AE 50 0F         [ 2]  353 	ldw	x, #0x500f
      0081B5 F7               [ 1]  354 	ld	(x), a
      0081B6 CC 81 05         [ 2]  355 	jp	00108$
      0081B9 5B 1E            [ 2]  356 	addw	sp, #30
      0081BB 81               [ 4]  357 	ret
                                    358 	.area CODE
      0081BC                        359 ___str_0:
      0081BC 74 6F 74 61 6C 3A 20   360 	.ascii "total: %d, "
             25 64 2C 20
      0081C7 00                     361 	.db 0x00
      0081C8                        362 ___str_1:
      0081C8 64 72 6F 70 70 65 64   363 	.ascii "dropped: %d, "
             3A 20 25 64 2C 20
      0081D5 00                     364 	.db 0x00
      0081D6                        365 ___str_2:
      0081D6 2D 3E 3A 20 25 64      366 	.ascii "->: %d"
      0081DC 0A                     367 	.db 0x0a
      0081DD 00                     368 	.db 0x00
                                    369 	.area INITIALIZER
                                    370 	.area CABS (ABS)
