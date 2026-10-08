#ifndef AXIS_FIFO_DRV_H
#define AXIS_FIFO_DRV_H
#include "xil_types.h"
#define AXIS_FIFO_TDFR 0x8U
#define AXIS_FIFO_TDFV 0xCU
#define AXIS_FIFO_TDFD 0x10U
#define AXIS_FIFO_TLR  0x14U
#define AXIS_FIFO_RDFR 0x18U
#define AXIS_FIFO_RDFO 0x1CU
#define AXIS_FIFO_RDFD 0x20U
#define AXIS_FIFO_RLR 0x24U
#define AXIS_FIFO_SRR  0x28U
/** Reset the AXI-Stream FIFO. */
void axis_fifo_reset(UINTPTR base);
/** Return the number of unread 32-bit words in the RX FIFO. */
uint32_t axis_fifo_rx_occupancy(UINTPTR base);
/** Return the length of the next RX packet in bytes. */
uint32_t axis_fifo_read_data_len(UINTPTR base);
/** Read FIFO data, in 32-bit words. */
int axis_fifo_read_data(UINTPTR base, unsigned char *buf, int len);
/** Discard one RX packet without resetting the FIFO. */
int axis_fifo_discard_data(UINTPTR base, uint32_t len);
/** Write FIFO data and commit the transfer. */
int axis_fifo_write_data(UINTPTR base, unsigned char *buf, int len);
#endif
