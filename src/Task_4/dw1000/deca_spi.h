#ifndef DECA_SPI_H
#define DECA_SPI_H

#include <stdint.h>
#include <stddef.h>

// The SPI channel used (typically 0 for CE0)
#define SPI_CHANNEL 0

int init_spi_w_speed(int speed);

#endif // DECA_SPI_H
