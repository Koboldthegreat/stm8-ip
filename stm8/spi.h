#ifndef SPI_H
#define SPI_H

#include <stdint.h>

/*
 * Initialize SPI in MODE1.
 *
 * Pinout:
 * SCK  -> PC5
 * MOSI -> PC6
 * MISO -> PC7
 * CS   -> user defined
 */

void SPI_init();

void SPI_write(uint8_t data);

uint8_t SPI_read();

uint8_t SPI_write_read(uint8_t data);

uint8_t SPI_write_read_bit(uint8_t byte);
void init_bit();


#endif /* SPI_H */

