#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stm8s.h>
#include <uart.h>
#include <delay.h>
#include <se8r01.h>
#include <spi.h>

void main() {
    uint8_t rx_buf[TX_PLOAD_WIDTH] = {0};
    uint32_t total = 0;
    uint32_t dropped = 0;
    uint8_t last = 0;

    CLK_CKDIVR = 3;

    uart_init();
    delay_ms(100);

    se8r01_init('r');
    delay_ms(1000);

    while (1) {
        if (se8r01_wait_rx(rx_buf)){
            PD_ODR |= (1<<4);
            //printf("Got packet: ");
            //for (uint8_t i = 0; i<TX_PLOAD_WIDTH; i++){
            //    printf("%d ; ", rx_buf[i]); 
            //}
            //printf("\n");
            if (rx_buf[TX_PLOAD_WIDTH-1] != last+6 && last != 0){
                dropped++;
            }
            total++;
            last = rx_buf[TX_PLOAD_WIDTH-1];
            printf("total: %d, ", (int) total);
            printf("dropped: %d, ", (int) dropped);
            printf("->: %d\n", ((int) dropped*100)/(int) total);
        } else {
            PD_ODR &= ~(1<<4);
        }
        //if (se8r01_wait_rx() == 1){
        //    printf("Got packet\n");
        //    printf("Buf: %.*s\n", 32, rx_buf);
        //} else {
        //}
    }
}
