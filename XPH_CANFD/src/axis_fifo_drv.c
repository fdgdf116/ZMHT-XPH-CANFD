#include "axis_fifo_drv.h"

#include "xil_io.h"

/** Reset the AXI-Stream FIFO TX/RX datapaths and status registers. */
void axis_fifo_reset(UINTPTR base)
{
    Xil_Out32(base + AXIS_FIFO_TDFR, 0xA5U);
    Xil_Out32(base + AXIS_FIFO_RDFR, 0xA5U);
    Xil_Out32(base + AXIS_FIFO_SRR, 0xA5U);
}

/** Return the number of unread 32-bit words in the FIFO RX path. */
uint32_t axis_fifo_rx_occupancy(UINTPTR base)
{
    return Xil_In32(base + AXIS_FIFO_RDFO);
}

/** Return the length of the next packet, in bytes. */
uint32_t axis_fifo_read_data_len(UINTPTR base)
{
    return Xil_In32(base + AXIS_FIFO_RLR);
}

/** Read an integral number of 32-bit words from the FIFO RX data register. */
int axis_fifo_read_data(UINTPTR base, unsigned char *buffer, int length)
{
    int offset;

    if (buffer == NULL || length < 0 || (length & 3) != 0) {
        return -1;
    }
    for (offset = 0; offset < length; offset += 4) {
        *(uint32_t *)(buffer + offset) = Xil_In32(base + AXIS_FIFO_RDFD);
    }
    return length;
}

/** Discard one complete RX packet without flushing other queued packets. */
int axis_fifo_discard_data(UINTPTR base, uint32_t length)
{
    uint32_t offset;

    if ((length & 3U) != 0U) {
        return -1;
    }
    for (offset = 0U; offset < length; offset += 4U) {
        (void)Xil_In32(base + AXIS_FIFO_RDFD);
    }
    return (int)length;
}

/** Write an integral number of words and commit them with the TLR register. */
int axis_fifo_write_data(UINTPTR base, unsigned char *buffer, int length)
{
    int offset;
    uint32_t words;

    if (buffer == NULL || length <= 0 || (length & 3) != 0) {
        return -1;
    }
    words = (uint32_t)length / 4U;
    if (Xil_In32(base + AXIS_FIFO_TDFV) < words) {
        return -1;
    }
    for (offset = 0; offset < length; offset += 4) {
        Xil_Out32(base + AXIS_FIFO_TDFD,
                  *(uint32_t *)(buffer + offset));
    }
    Xil_Out32(base + AXIS_FIFO_TLR, (uint32_t)offset);
    return offset == length ? 0 : -1;
}
