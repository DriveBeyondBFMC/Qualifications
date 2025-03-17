#include "deca_spi.h"
#include "deca_mutex.h"
#include <wiringPiSPI.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/*
 * Initialize Raspberry's SPI module with a speed
 *
 */
int init_spi_w_speed(int speed)
{
	if (wiringPiSPISetup(SPI_CHANNEL, speed) == -1)
    {
        fprintf(stderr, "SPI setup failed!\n");
        return -1;
    }
	return 0;
}

/*
 * Write data to the DW1000 via SPI.
 * Uses a mutex to ensure safe SPI transactions.
 */
int writetospi(int headerLength, const uint8_t *header, uint32_t bodyLength, const uint8_t *body)
{
    uint32_t totalLength = headerLength + bodyLength;
    uint8_t *txBuf = (uint8_t *)malloc(totalLength);
    if (txBuf == NULL)
    {
        fprintf(stderr, "writetospi: Memory allocation failed\n");
        return -1;
    }

    // Lock SPI before sending data
    decamutexon();

    // Copy header and body into the transmission buffer
    memcpy(txBuf, header, headerLength);
    memcpy(txBuf + headerLength, body, bodyLength);

    // Perform the SPI transaction
    if (wiringPiSPIDataRW(SPI_CHANNEL, txBuf, totalLength) == -1)
    {
        fprintf(stderr, "writetospi: SPI transaction failed\n");
        free(txBuf);
        decamutexoff(0);
        return -1;
    }

    free(txBuf);

    // Unlock SPI after transaction
    decamutexoff(0);

    return 0;
}

/*
 * Read data from the DW1000 via SPI.
 */
int readfromspi(int headerLength, const uint8_t *header, uint32_t readLength, uint8_t *readBuffer)
{
    uint32_t totalLength = headerLength + readLength;
    uint8_t *txBuf = (uint8_t *)malloc(totalLength);
    if (txBuf == NULL)
    {
        fprintf(stderr, "readfromspi: Memory allocation failed\n");
        return -1;
    }

    // Lock SPI before transaction
    decamutexon();

    // Copy header to transmission buffer
    memcpy(txBuf, header, headerLength);
    memset(txBuf + headerLength, 0, readLength); // Dummy bytes to clock out data

    // Perform SPI transaction
    if (wiringPiSPIDataRW(SPI_CHANNEL, txBuf, totalLength) == -1)
    {
        fprintf(stderr, "readfromspi: SPI transaction failed\n");
        free(txBuf);
        decamutexoff(0);
        return -1;
    }

    // Copy the received data (after the header) into readBuffer
    memcpy(readBuffer, txBuf + headerLength, readLength);
    free(txBuf);

    // Unlock SPI after transaction
    decamutexoff(0);

    return 0;
}