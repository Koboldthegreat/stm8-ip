#include "spi.h"
#include "stm8s.h"
#include "delay.h"

void SPI_init() {
    SPI_CR1 |= (1 << SPI_CR1_MSTR) | (1<<SPI_CR1_BR0); 
    SPI_CR2 |= (1 << SPI_CR2_SSM) | (1<<SPI_CR2_SSI);
    SPI_CR1 |= (1 << SPI_CR1_SPE);
}

uint8_t SPI_read() {
    SPI_write(0xFF);
    while (!(SPI_SR & (1 << SPI_SR_RXNE))); //wait till byte received
    return SPI_DR;
}

void SPI_write(uint8_t data) {
    SPI_DR = data;
    while (!(SPI_SR & (1 << SPI_SR_TXE)));
    while ((SPI_SR & (1<<SPI_SR_BSY))); //wait till byte Send`
}

uint8_t SPI_write_read(uint8_t data) {
    uint8_t rcv;

    SPI_DR = data;
    while (!(SPI_SR & (1 << SPI_SR_RXNE))); //wait till byte received
    rcv = SPI_DR;
    while (!(SPI_SR & (1<<SPI_SR_TXE))); //wait till byte Send`
    while ((SPI_SR & (1<<SPI_SR_BSY))); //wait till byte Send`
    return rcv;
}


void init_bit() {
    PC_DDR |= (1<<5) | (1<<6); //output
    PC_CR1 |= (1<<5) | (1<<6); //push-pull
    PC_CR2 |= (1<<5) | (1<<6); //10Mhz speed

    PC_DDR &= ~(1<<7); //Input
    PC_CR1 &= ~(1<<7); //No pull-up
    PC_CR2 &= ~(1<<7); //disable external interupt
}

uint8_t SPI_write_read_bit(uint8_t byte) {
    for(uint8_t i=0; i<8; i++) {
        if(byte & 0x80) {
            PC_ODR |= (1<<6);
        } else {
            PC_ODR &= ~(1<<6);
        }
        PC_ODR |= (1<<5);
        byte <<= 1;
        if(PC_IDR & (1<<7)) {
            byte |=1;
        }
        PC_ODR &= ~(1<<5);
    }
    return byte;
}




