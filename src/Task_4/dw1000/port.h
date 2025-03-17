#ifndef PORT_H_
#define PORT_H_

#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>


// Define GPIO Pins (Modify according to your wiring)
#define DW1000_RST_PIN  23  // Reset Pin (Board 16)
#define DW1000_IRQ_PIN  24  // IRQ Pin (Board 18)


// Function Prototypes
void port_init(void);
void reset_DW1000(void);
uint32_t portGetTickCount(void);
void port_enable_ext_interrupt(void (*callback)(void));  // NEW: Enable interrupt handling


#endif /* PORT_H_ */
