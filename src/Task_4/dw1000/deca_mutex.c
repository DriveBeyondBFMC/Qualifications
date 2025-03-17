#include "deca_mutex.h"
#include <pthread.h>
#include <stdio.h>

// Define a mutex
static pthread_mutex_t dwm1000_mutex = PTHREAD_MUTEX_INITIALIZER;

/**
 * @brief Lock the mutex to enter a critical section.
 *
 * @return Always returns 0.
 */
int decamutexon(void)
{
    pthread_mutex_lock(&dwm1000_mutex);
    return 0;
}

/**
 * @brief Unlock the mutex to leave a critical section.
 *
 * @param val Not used, but kept for compatibility.
 */
void decamutexoff(int val)
{
    (void)val; // Avoid unused parameter warning
    pthread_mutex_unlock(&dwm1000_mutex);
}