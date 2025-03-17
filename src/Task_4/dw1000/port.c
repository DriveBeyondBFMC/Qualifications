#include "port.h"
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

// Global function pointer for interrupt callback
static void (*dw1000_interrupt_callback)(void) = NULL;

/**
 * @brief Initializes the Raspberry Pi GPIO and SPI.
 */
void port_init(void)
{
    if ( wiringPiSetupGpio() == -1)
    {
        fprintf(stderr, "WiringPi setup failed\n");
        exit(1);
    }

    // Configure GPIOs
    pinMode(DW1000_RST_PIN, OUTPUT);
    pinMode(DW1000_IRQ_PIN, INPUT);
	pullUpDnControl(DW1000_IRQ_PIN, PUD_DOWN);  // Enable pull-down resistor
}

/**
 * @brief Resets the DW1000 module by toggling the reset pin.
 */
void reset_DW1000(void)
{
    digitalWrite(DW1000_RST_PIN, LOW);
    usleep(10000); // 10ms delay
    digitalWrite(DW1000_RST_PIN, HIGH);
    usleep(50000); // 50ms delay for startup
}

/**
 * @brief Returns the current system tick count (milliseconds).
 */
uint32_t portGetTickCount(void)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    return (uint32_t)((now.tv_sec * 1000) + (now.tv_usec / 1000));
}

/**
 * @brief Interrupt Handler for DW1000 IRQ pin.
 */
void dwm1000_irq_handler(void)
{
    if (dw1000_interrupt_callback)
    {
        dw1000_interrupt_callback(); // Call user-defined callback function
    }
}

/**
 * @brief Enable external interrupt for DW1000 on GPIO 16.
 * @param callback Function pointer to the user-defined interrupt handler.
 */
void port_enable_ext_interrupt(void (*callback)(void))
{
    dw1000_interrupt_callback = callback; // Save the callback function

    // Attach the interrupt to DW1000 IRQ pin (GPIO 16)
    if (wiringPiISR(DW1000_IRQ_PIN, INT_EDGE_RISING, &dwm1000_irq_handler) < 0)
    {
        fprintf(stderr, "Failed to setup interrupt on GPIO %d\n", DW1000_IRQ_PIN);
        exit(1);
    }

    printf("DW1000 Interrupt enabled on GPIO %d\n", DW1000_IRQ_PIN);
}