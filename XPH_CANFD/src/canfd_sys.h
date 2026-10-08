#ifndef CANFD_SYS_H
#define CANFD_SYS_H
#include <stdint.h>

/** Return system uptime in microseconds for protocol timestamps. */
uint64_t canfd_timestamp_us(void);
/** Initialize CAN-FD channels, memory, FIFOs and interrupts. */
void axi_canfd_system_init(void);
/** Run the host/FIFO transfer service loop. */
void axi_canfd_frame_transfer(void);
#endif
