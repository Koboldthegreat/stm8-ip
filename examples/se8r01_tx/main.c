#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stm8s.h>
#include <uart.h>
#include <delay.h>
#include <se8r01.h>
#include <spi.h>

void main() {
    uint8_t counter = 0;
    uint8_t rcv = 0;
    uint8_t tx_buf[TX_PLOAD_WIDTH];

    CLK_CKDIVR = 3;

    uart_init();
    delay_ms(100);
    printf("Hello\n");

    se8r01_init('t');

    while (1) {
        for (uint8_t i=0; i<TX_PLOAD_WIDTH; i++)
            tx_buf[i] = counter++;
        se8r01_tx(tx_buf);
        //if (se8r01_wait_rx() == 1){
        //    printf("Got packet\n");
        //    printf("Buf: %.*s\n", 32, rx_buf);
        //} else {
        //}
        delay_ms(50);
    }
}
