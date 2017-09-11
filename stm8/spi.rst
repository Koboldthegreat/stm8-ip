                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.6.8 #9946 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module spi
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _SPI_init
                                     12 	.globl _SPI_read
                                     13 	.globl _SPI_write
                                     14 	.globl _SPI_write_read
                                     15 	.globl _init_bit
                                     16 	.globl _SPI_write_read_bit
                                     17 ;--------------------------------------------------------
                                     18 ; ram data
                                     19 ;--------------------------------------------------------
                                     20 	.area DATA
                                     21 ;--------------------------------------------------------
                                     22 ; ram data
                                     23 ;--------------------------------------------------------
                                     24 	.area INITIALIZED
                                     25 ;--------------------------------------------------------
                                     26 ; absolute external ram data
                                     27 ;--------------------------------------------------------
                                     28 	.area DABS (ABS)
                                     29 ;--------------------------------------------------------
                                     30 ; global & static initialisations
                                     31 ;--------------------------------------------------------
                                     32 	.area HOME
                                     33 	.area GSINIT
                                     34 	.area GSFINAL
                                     35 	.area GSINIT
                                     36 ;--------------------------------------------------------
                                     37 ; Home
                                     38 ;--------------------------------------------------------
                                     39 	.area HOME
                                     40 	.area HOME
                                     41 ;--------------------------------------------------------
                                     42 ; code
                                     43 ;--------------------------------------------------------
                                     44 	.area CODE
                                     45 ;	../../stm8/delay.h: 11: static inline void delay_ms(uint32_t ms) {
                                     46 ;	-----------------------------------------
                                     47 ;	 function delay_ms
                                     48 ;	-----------------------------------------
      008847                         49 _delay_ms:
      008847 52 08            [ 2]   50 	sub	sp, #8
                                     51 ;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      008849 5F               [ 1]   52 	clrw	x
      00884A 1F 03            [ 2]   53 	ldw	(0x03, sp), x
      00884C 1F 01            [ 2]   54 	ldw	(0x01, sp), x
      00884E 1E 0D            [ 2]   55 	ldw	x, (0x0d, sp)
      008850 89               [ 2]   56 	pushw	x
      008851 1E 0D            [ 2]   57 	ldw	x, (0x0d, sp)
      008853 89               [ 2]   58 	pushw	x
      008854 4B 6F            [ 1]   59 	push	#0x6f
      008856 5F               [ 1]   60 	clrw	x
      008857 89               [ 2]   61 	pushw	x
      008858 4B 00            [ 1]   62 	push	#0x00
      00885A CD 8A 66         [ 4]   63 	call	__mullong
      00885D 5B 08            [ 2]   64 	addw	sp, #8
      00885F 1F 07            [ 2]   65 	ldw	(0x07, sp), x
      008861 17 05            [ 2]   66 	ldw	(0x05, sp), y
      008863                         67 00103$:
      008863 1E 03            [ 2]   68 	ldw	x, (0x03, sp)
      008865 13 07            [ 2]   69 	cpw	x, (0x07, sp)
      008867 7B 02            [ 1]   70 	ld	a, (0x02, sp)
      008869 12 06            [ 1]   71 	sbc	a, (0x06, sp)
      00886B 7B 01            [ 1]   72 	ld	a, (0x01, sp)
      00886D 12 05            [ 1]   73 	sbc	a, (0x05, sp)
      00886F 24 17            [ 1]   74 	jrnc	00105$
                                     75 ;	../../stm8/delay.h: 13: __asm__("nop");
      008871 9D               [ 1]   76 	nop
                                     77 ;	../../stm8/delay.h: 12: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      008872 16 03            [ 2]   78 	ldw	y, (0x03, sp)
      008874 72 A9 00 01      [ 2]   79 	addw	y, #0x0001
      008878 7B 02            [ 1]   80 	ld	a, (0x02, sp)
      00887A A9 00            [ 1]   81 	adc	a, #0x00
      00887C 97               [ 1]   82 	ld	xl, a
      00887D 7B 01            [ 1]   83 	ld	a, (0x01, sp)
      00887F A9 00            [ 1]   84 	adc	a, #0x00
      008881 95               [ 1]   85 	ld	xh, a
      008882 17 03            [ 2]   86 	ldw	(0x03, sp), y
      008884 1F 01            [ 2]   87 	ldw	(0x01, sp), x
      008886 20 DB            [ 2]   88 	jra	00103$
      008888                         89 00105$:
      008888 5B 08            [ 2]   90 	addw	sp, #8
      00888A 81               [ 4]   91 	ret
                                     92 ;	../../stm8/delay.h: 17: static inline void delay_us(uint32_t us) {
                                     93 ;	-----------------------------------------
                                     94 ;	 function delay_us
                                     95 ;	-----------------------------------------
      00888B                         96 _delay_us:
                                     97 ;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
      00888B 90 5F            [ 1]   98 	clrw	y
      00888D 5F               [ 1]   99 	clrw	x
      00888E                        100 00103$:
      00888E 90 A3 00 00      [ 2]  101 	cpw	y, #0x0000
      008892 9F               [ 1]  102 	ld	a, xl
      008893 A2 00            [ 1]  103 	sbc	a, #0x00
      008895 9E               [ 1]  104 	ld	a, xh
      008896 A2 00            [ 1]  105 	sbc	a, #0x00
      008898 25 01            [ 1]  106 	jrc	00115$
      00889A 81               [ 4]  107 	ret
      00889B                        108 00115$:
                                    109 ;	../../stm8/delay.h: 19: __asm__("nop");
      00889B 9D               [ 1]  110 	nop
                                    111 ;	../../stm8/delay.h: 18: for (uint32_t i = 0; i < ((F_CPU / 18 / 1000000UL) * us); i++) {
      00889C 90 5C            [ 1]  112 	incw	y
      00889E 26 EE            [ 1]  113 	jrne	00103$
      0088A0 5C               [ 1]  114 	incw	x
      0088A1 20 EB            [ 2]  115 	jra	00103$
      0088A3 81               [ 4]  116 	ret
                                    117 ;	../../stm8/spi.c: 5: void SPI_init() {
                                    118 ;	-----------------------------------------
                                    119 ;	 function SPI_init
                                    120 ;	-----------------------------------------
      0088A4                        121 _SPI_init:
                                    122 ;	../../stm8/spi.c: 6: SPI_CR1 |= (1 << SPI_CR1_MSTR) | (1<<SPI_CR1_BR0); 
      0088A4 AE 52 00         [ 2]  123 	ldw	x, #0x5200
      0088A7 F6               [ 1]  124 	ld	a, (x)
      0088A8 AA 0C            [ 1]  125 	or	a, #0x0c
      0088AA F7               [ 1]  126 	ld	(x), a
                                    127 ;	../../stm8/spi.c: 7: SPI_CR2 |= (1 << SPI_CR2_SSM) | (1<<SPI_CR2_SSI);
      0088AB AE 52 01         [ 2]  128 	ldw	x, #0x5201
      0088AE F6               [ 1]  129 	ld	a, (x)
      0088AF AA 03            [ 1]  130 	or	a, #0x03
      0088B1 F7               [ 1]  131 	ld	(x), a
                                    132 ;	../../stm8/spi.c: 8: SPI_CR1 |= (1 << SPI_CR1_SPE);
      0088B2 AE 52 00         [ 2]  133 	ldw	x, #0x5200
      0088B5 F6               [ 1]  134 	ld	a, (x)
      0088B6 AA 40            [ 1]  135 	or	a, #0x40
      0088B8 F7               [ 1]  136 	ld	(x), a
      0088B9 81               [ 4]  137 	ret
                                    138 ;	../../stm8/spi.c: 11: uint8_t SPI_read() {
                                    139 ;	-----------------------------------------
                                    140 ;	 function SPI_read
                                    141 ;	-----------------------------------------
      0088BA                        142 _SPI_read:
                                    143 ;	../../stm8/spi.c: 12: SPI_write(0xFF);
      0088BA 4B FF            [ 1]  144 	push	#0xff
      0088BC CD 88 CC         [ 4]  145 	call	_SPI_write
      0088BF 84               [ 1]  146 	pop	a
                                    147 ;	../../stm8/spi.c: 13: while (!(SPI_SR & (1 << SPI_SR_RXNE))); //wait till byte received
      0088C0                        148 00101$:
      0088C0 AE 52 03         [ 2]  149 	ldw	x, #0x5203
      0088C3 F6               [ 1]  150 	ld	a, (x)
      0088C4 44               [ 1]  151 	srl	a
      0088C5 24 F9            [ 1]  152 	jrnc	00101$
                                    153 ;	../../stm8/spi.c: 14: return SPI_DR;
      0088C7 AE 52 04         [ 2]  154 	ldw	x, #0x5204
      0088CA F6               [ 1]  155 	ld	a, (x)
      0088CB 81               [ 4]  156 	ret
                                    157 ;	../../stm8/spi.c: 17: void SPI_write(uint8_t data) {
                                    158 ;	-----------------------------------------
                                    159 ;	 function SPI_write
                                    160 ;	-----------------------------------------
      0088CC                        161 _SPI_write:
                                    162 ;	../../stm8/spi.c: 18: SPI_DR = data;
      0088CC AE 52 04         [ 2]  163 	ldw	x, #0x5204
      0088CF 7B 03            [ 1]  164 	ld	a, (0x03, sp)
      0088D1 F7               [ 1]  165 	ld	(x), a
                                    166 ;	../../stm8/spi.c: 19: while (!(SPI_SR & (1 << SPI_SR_TXE)));
      0088D2                        167 00101$:
      0088D2 AE 52 03         [ 2]  168 	ldw	x, #0x5203
      0088D5 F6               [ 1]  169 	ld	a, (x)
      0088D6 A5 02            [ 1]  170 	bcp	a, #0x02
      0088D8 27 F8            [ 1]  171 	jreq	00101$
                                    172 ;	../../stm8/spi.c: 20: while ((SPI_SR & (1<<SPI_SR_BSY))); //wait till byte Send`
      0088DA                        173 00104$:
      0088DA 4D               [ 1]  174 	tnz	a
      0088DB 2B FD            [ 1]  175 	jrmi	00104$
      0088DD 81               [ 4]  176 	ret
                                    177 ;	../../stm8/spi.c: 23: uint8_t SPI_write_read(uint8_t data) {
                                    178 ;	-----------------------------------------
                                    179 ;	 function SPI_write_read
                                    180 ;	-----------------------------------------
      0088DE                        181 _SPI_write_read:
      0088DE 88               [ 1]  182 	push	a
                                    183 ;	../../stm8/spi.c: 26: SPI_DR = data;
      0088DF AE 52 04         [ 2]  184 	ldw	x, #0x5204
      0088E2 7B 04            [ 1]  185 	ld	a, (0x04, sp)
      0088E4 F7               [ 1]  186 	ld	(x), a
                                    187 ;	../../stm8/spi.c: 27: while (!(SPI_SR & (1 << SPI_SR_RXNE))); //wait till byte received
      0088E5                        188 00101$:
      0088E5 AE 52 03         [ 2]  189 	ldw	x, #0x5203
      0088E8 F6               [ 1]  190 	ld	a, (x)
      0088E9 44               [ 1]  191 	srl	a
      0088EA 24 F9            [ 1]  192 	jrnc	00101$
                                    193 ;	../../stm8/spi.c: 28: rcv = SPI_DR;
      0088EC AE 52 04         [ 2]  194 	ldw	x, #0x5204
      0088EF F6               [ 1]  195 	ld	a, (x)
      0088F0 6B 01            [ 1]  196 	ld	(0x01, sp), a
                                    197 ;	../../stm8/spi.c: 29: while (!(SPI_SR & (1<<SPI_SR_TXE))); //wait till byte Send`
      0088F2                        198 00104$:
                                    199 ;	../../stm8/spi.c: 27: while (!(SPI_SR & (1 << SPI_SR_RXNE))); //wait till byte received
      0088F2 AE 52 03         [ 2]  200 	ldw	x, #0x5203
      0088F5 F6               [ 1]  201 	ld	a, (x)
                                    202 ;	../../stm8/spi.c: 29: while (!(SPI_SR & (1<<SPI_SR_TXE))); //wait till byte Send`
      0088F6 A5 02            [ 1]  203 	bcp	a, #0x02
      0088F8 27 F8            [ 1]  204 	jreq	00104$
                                    205 ;	../../stm8/spi.c: 30: while ((SPI_SR & (1<<SPI_SR_BSY))); //wait till byte Send`
      0088FA                        206 00107$:
      0088FA 4D               [ 1]  207 	tnz	a
      0088FB 2B FD            [ 1]  208 	jrmi	00107$
                                    209 ;	../../stm8/spi.c: 31: return rcv;
      0088FD 7B 01            [ 1]  210 	ld	a, (0x01, sp)
      0088FF 5B 01            [ 2]  211 	addw	sp, #1
      008901 81               [ 4]  212 	ret
                                    213 ;	../../stm8/spi.c: 35: void init_bit() {
                                    214 ;	-----------------------------------------
                                    215 ;	 function init_bit
                                    216 ;	-----------------------------------------
      008902                        217 _init_bit:
                                    218 ;	../../stm8/spi.c: 36: PC_DDR |= (1<<5) | (1<<6); //output
      008902 AE 50 0C         [ 2]  219 	ldw	x, #0x500c
      008905 F6               [ 1]  220 	ld	a, (x)
      008906 AA 60            [ 1]  221 	or	a, #0x60
      008908 F7               [ 1]  222 	ld	(x), a
                                    223 ;	../../stm8/spi.c: 37: PC_CR1 |= (1<<5) | (1<<6); //push-pull
      008909 AE 50 0D         [ 2]  224 	ldw	x, #0x500d
      00890C F6               [ 1]  225 	ld	a, (x)
      00890D AA 60            [ 1]  226 	or	a, #0x60
      00890F F7               [ 1]  227 	ld	(x), a
                                    228 ;	../../stm8/spi.c: 38: PC_CR2 |= (1<<5) | (1<<6); //10Mhz speed
      008910 AE 50 0E         [ 2]  229 	ldw	x, #0x500e
      008913 F6               [ 1]  230 	ld	a, (x)
      008914 AA 60            [ 1]  231 	or	a, #0x60
      008916 F7               [ 1]  232 	ld	(x), a
                                    233 ;	../../stm8/spi.c: 40: PC_DDR &= ~(1<<7); //Input
      008917 72 1F 50 0C      [ 1]  234 	bres	0x500c, #7
                                    235 ;	../../stm8/spi.c: 41: PC_CR1 &= ~(1<<7); //No pull-up
      00891B 72 1F 50 0D      [ 1]  236 	bres	0x500d, #7
                                    237 ;	../../stm8/spi.c: 42: PC_CR2 &= ~(1<<7); //disable external interupt
      00891F 72 1F 50 0E      [ 1]  238 	bres	0x500e, #7
      008923 81               [ 4]  239 	ret
                                    240 ;	../../stm8/spi.c: 45: uint8_t SPI_write_read_bit(uint8_t byte) {
                                    241 ;	-----------------------------------------
                                    242 ;	 function SPI_write_read_bit
                                    243 ;	-----------------------------------------
      008924                        244 _SPI_write_read_bit:
                                    245 ;	../../stm8/spi.c: 46: for(uint8_t i=0; i<8; i++) {
      008924 4F               [ 1]  246 	clr	a
      008925 90 97            [ 1]  247 	ld	yl, a
      008927                        248 00108$:
      008927 90 9F            [ 1]  249 	ld	a, yl
      008929 A1 08            [ 1]  250 	cp	a, #0x08
      00892B 24 36            [ 1]  251 	jrnc	00106$
                                    252 ;	../../stm8/spi.c: 48: PC_ODR |= (1<<6);
      00892D AE 50 0A         [ 2]  253 	ldw	x, #0x500a
      008930 F6               [ 1]  254 	ld	a, (x)
                                    255 ;	../../stm8/spi.c: 47: if(byte & 0x80) {
      008931 0D 03            [ 1]  256 	tnz	(0x03, sp)
      008933 2A 08            [ 1]  257 	jrpl	00102$
                                    258 ;	../../stm8/spi.c: 48: PC_ODR |= (1<<6);
      008935 AA 40            [ 1]  259 	or	a, #0x40
      008937 AE 50 0A         [ 2]  260 	ldw	x, #0x500a
      00893A F7               [ 1]  261 	ld	(x), a
      00893B 20 06            [ 2]  262 	jra	00103$
      00893D                        263 00102$:
                                    264 ;	../../stm8/spi.c: 50: PC_ODR &= ~(1<<6);
      00893D A4 BF            [ 1]  265 	and	a, #0xbf
      00893F AE 50 0A         [ 2]  266 	ldw	x, #0x500a
      008942 F7               [ 1]  267 	ld	(x), a
      008943                        268 00103$:
                                    269 ;	../../stm8/spi.c: 52: PC_ODR |= (1<<5);
      008943 AE 50 0A         [ 2]  270 	ldw	x, #0x500a
      008946 F6               [ 1]  271 	ld	a, (x)
      008947 AA 20            [ 1]  272 	or	a, #0x20
      008949 F7               [ 1]  273 	ld	(x), a
                                    274 ;	../../stm8/spi.c: 53: byte <<= 1;
      00894A 08 03            [ 1]  275 	sll	(0x03, sp)
                                    276 ;	../../stm8/spi.c: 54: if(PC_IDR & (1<<7)) {
      00894C AE 50 0B         [ 2]  277 	ldw	x, #0x500b
      00894F F6               [ 1]  278 	ld	a, (x)
      008950 2A 06            [ 1]  279 	jrpl	00105$
                                    280 ;	../../stm8/spi.c: 55: byte |=1;
      008952 7B 03            [ 1]  281 	ld	a, (0x03, sp)
      008954 AA 01            [ 1]  282 	or	a, #0x01
      008956 6B 03            [ 1]  283 	ld	(0x03, sp), a
      008958                        284 00105$:
                                    285 ;	../../stm8/spi.c: 57: PC_ODR &= ~(1<<5);
      008958 AE 50 0A         [ 2]  286 	ldw	x, #0x500a
      00895B F6               [ 1]  287 	ld	a, (x)
      00895C A4 DF            [ 1]  288 	and	a, #0xdf
      00895E F7               [ 1]  289 	ld	(x), a
                                    290 ;	../../stm8/spi.c: 46: for(uint8_t i=0; i<8; i++) {
      00895F 90 5C            [ 1]  291 	incw	y
      008961 20 C4            [ 2]  292 	jra	00108$
      008963                        293 00106$:
                                    294 ;	../../stm8/spi.c: 59: return byte;
      008963 7B 03            [ 1]  295 	ld	a, (0x03, sp)
      008965 81               [ 4]  296 	ret
                                    297 	.area CODE
                                    298 	.area INITIALIZER
                                    299 	.area CABS (ABS)
