#include "deca_sleep.h"
#include <unistd.h> // For usleep()
#include <stdio.h>

/**
 * @brief Sleep for a given number of milliseconds.
 *
 * @param time_ms Time in milliseconds.
 */
void deca_sleep(unsigned int time_ms)
{
    usleep(time_ms * 1000); // Convert ms to microseconds
}
