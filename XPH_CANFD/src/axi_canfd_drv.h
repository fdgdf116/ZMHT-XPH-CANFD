#ifndef AXI_CANFD_DRV_H
#define AXI_CANFD_DRV_H

#include "xstatus.h"
#include "xcanfd.h"
#include "ringbuffer.h"
#include "canfd_common.h"

/** Counters for nominal- and data-phase protocol errors. */
typedef struct {
    uint32_t ack_error, bit_error, stuff_error, form_error, crc_error;
    uint32_t fd_bit_error, fd_stuff_error, fd_form_error, fd_crc_error;
} canfd_error_count_t;

/** Counters for controller events exposed to diagnostics. */
typedef struct {
    volatile uint32_t bus_off, rx_overflow, rx_underflow;
    volatile uint32_t tx_cancel_served, tx_ready_served;
    volatile uint32_t arbitration_lost, tx_ok, rx_ok, protocol_error;
} canfd_event_count_t;

/** Software counters used to verify TX and host-feedback loss. */
typedef struct {
    volatile uint32_t tx_requests;
    volatile uint32_t tx_submitted;
    /* Counted after the owning TX buffer is observed released.  This is
     * different from events.tx_ok, which counts interrupt callbacks and may
     * under-count when several completions share one interrupt. */
    volatile uint32_t tx_completed;
    volatile uint32_t tx_submit_failed;
    volatile uint32_t rx_dequeued;
    volatile uint32_t rx_queued;
    volatile uint32_t rx_dropped;
    volatile uint32_t feedback_generated;
    volatile uint32_t feedback_published;
    volatile uint32_t feedback_publish_failed;
    volatile uint32_t feedback_tx;
    volatile uint32_t feedback_rx;
    volatile uint32_t feedback_error;
    uint32_t stats_last_print;
} canfd_diag_count_t;

/** Runtime state for one CAN-FD controller/channel pair. */
typedef struct {
    XCanFd can;
    uint8_t channel;
    uint16_t device_id;
    uint8_t intc_id;
    uint8_t initialized;
    uint8_t mode;
    uint8_t auto_retransmit;
    uint8_t brs_enabled;
    uint32_t nominal_baud;
    uint32_t data_baud;
    uint32_t packet_interval_us;
    uint32_t interrupt_enable;
    ring_buffer_t *rx_rb;
    uint32_t rx_count;
    UINTPTR rx_storage;
    UINTPTR status_base;
    UINTPTR response_base;
    UINTPTR axis_fifo_base;
    uint32_t response_offset;
    canfd_error_count_t errors;
    canfd_event_count_t events;
    canfd_diag_count_t diag;
} canfd_drv_t;

/** Initialize one CAN-FD controller and install callbacks. */
int axi_canfd_init(canfd_drv_t *drv);
/** Apply nominal/data timing, BRS and retransmission settings. */
int axi_canfd_config(canfd_drv_t *drv, uint32_t nominal_baud,
                     uint32_t data_baud, uint8_t brs_enabled);
/** Enter a CAN-FD operating mode and wait for acknowledgement. */
void axi_canfd_set_mode(canfd_drv_t *drv, uint8_t mode);

#endif
