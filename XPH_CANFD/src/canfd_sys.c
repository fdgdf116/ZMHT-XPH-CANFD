#include "canfd_sys.h"

#include <string.h>

#include "axi_canfd_drv.h"
#include "axis_fifo_drv.h"
#include "canfd_common.h"
#include "xil_cache.h"
#include "xil_exception.h"
#include "xil_printf.h"
#include "xintc.h"
#include "xparameters.h"
#include "sleep.h"

/* XCanFd_Send() only reserves a TX buffer.  Retry a full controller and wait
 * for the selected buffer to be released before reporting success to the
 * host.  The timeout prevents a missing ACK or bus-off from blocking forever.
 */
#ifndef CANFD_TX_POLL_US
#define CANFD_TX_POLL_US              100U
#endif
#ifndef CANFD_TX_WAIT_TIMEOUT_US
#define CANFD_TX_WAIT_TIMEOUT_US      1000000U
#endif
#ifndef CANFD_FEEDBACK_WAIT_TIMEOUT_US
#define CANFD_FEEDBACK_WAIT_TIMEOUT_US 1000000U
#endif

/* Set to 0 for a quiet production build.  Keep ISR messages limited to
 * important events; per-frame details are printed by the service path. */
#ifndef CANFD_DEBUG
#define CANFD_DEBUG 1
#endif
#if CANFD_DEBUG
#define CANFD_DBG(...) xil_printf(__VA_ARGS__)
#else
#define CANFD_DBG(...)
#endif

/* Verbose tracing is intentionally off by default.  Enable it temporarily
 * when diagnosing initialization/FIFO details; frame records and counters
 * remain available with the normal debug setting. */
#ifndef CANFD_VERBOSE
#define CANFD_VERBOSE 0
#endif
#if CANFD_DEBUG && CANFD_VERBOSE
#define CANFD_VDBG(...) xil_printf(__VA_ARGS__)
#else
#define CANFD_VDBG(...)
#endif

/* Detailed payload dumps are enabled for loss diagnosis. Set to 0 after the
 * test if the serial output itself becomes a throughput bottleneck. */
#ifndef CANFD_DEBUG_DATA
#define CANFD_DEBUG_DATA 0
#endif

/* Print a cumulative loss-diagnostic line at this interval.  Per-frame TX
 * and feedback lines are always printed when CANFD_DEBUG is enabled. */
#ifndef CANFD_DEBUG_SUMMARY_INTERVAL
#define CANFD_DEBUG_SUMMARY_INTERVAL 0U
#endif

/* The BSP may describe fewer than four instances.  Entries beyond the
 * reported instance count are never used, but zero fallbacks keep this
 * source portable across BSPs generated for 1..4 CAN-FD channels. */
#ifndef XPAR_XCANFD_NUM_INSTANCES
#define XPAR_XCANFD_NUM_INSTANCES CANFD_CHANNEL_COUNT
#endif
#ifndef XPAR_CANFD_0_DEVICE_ID
#define XPAR_CANFD_0_DEVICE_ID 0U
#define XPAR_INTC_0_CANFD_0_VEC_ID 0U
#define XPAR_AXI_FIFO_0_BASEADDR 0U
#define XPAR_AXI_FIFO_1_BASEADDR 0U
#endif
#ifndef XPAR_CANFD_1_DEVICE_ID
#define XPAR_CANFD_1_DEVICE_ID 0U
#define XPAR_INTC_0_CANFD_1_VEC_ID 0U
#define XPAR_AXI_FIFO_2_BASEADDR 0U
#define XPAR_AXI_FIFO_3_BASEADDR 0U
#endif
#ifndef XPAR_CANFD_2_DEVICE_ID
#define XPAR_CANFD_2_DEVICE_ID 0U
#define XPAR_INTC_0_CANFD_2_VEC_ID 0U
#define XPAR_AXI_FIFO_4_BASEADDR 0U
#define XPAR_AXI_FIFO_5_BASEADDR 0U
#endif
#ifndef XPAR_CANFD_3_DEVICE_ID
#define XPAR_CANFD_3_DEVICE_ID 0U
#define XPAR_INTC_0_CANFD_3_VEC_ID 0U
#define XPAR_AXI_FIFO_6_BASEADDR 0U
#define XPAR_AXI_FIFO_7_BASEADDR 0U
#endif

#define CANFD_DDR_BASE 0x20000000U
#define CANFD_DDR_END  0x30000000U

static canfd_drv_t g_can[CANFD_CHANNEL_COUNT];
static XIntc g_intc;
static uint32_t total_count;
static uint32_t channel_count[CANFD_CHANNEL_COUNT];
static uint8_t config_buf[CANFD_MAX_CONFIG_BYTES] __attribute__((aligned(4)));
static uint32_t g_active_channels;

/* Data FIFOs, initialization FIFOs and per-channel status DDR regions. */
static const UINTPTR g_data_fifo[CANFD_CHANNEL_COUNT] = {
    XPAR_AXI_FIFO_0_BASEADDR, XPAR_AXI_FIFO_2_BASEADDR,
    XPAR_AXI_FIFO_4_BASEADDR, XPAR_AXI_FIFO_6_BASEADDR
};
static const UINTPTR g_init_fifo[CANFD_CHANNEL_COUNT] = {
    XPAR_AXI_FIFO_1_BASEADDR, XPAR_AXI_FIFO_3_BASEADDR,
    XPAR_AXI_FIFO_5_BASEADDR, XPAR_AXI_FIFO_7_BASEADDR
};
static const UINTPTR g_status_base[CANFD_CHANNEL_COUNT] = {
    0x21000000U, 0x23000000U, 0x25000000U,
    0x27000000U
};
static const uint16_t g_device_id[CANFD_CHANNEL_COUNT] = {
    XPAR_CANFD_0_DEVICE_ID, XPAR_CANFD_1_DEVICE_ID,
    XPAR_CANFD_2_DEVICE_ID, XPAR_CANFD_3_DEVICE_ID
};
static const uint8_t g_intc_id[CANFD_CHANNEL_COUNT] = {
    XPAR_INTC_0_CANFD_0_VEC_ID,
    XPAR_INTC_0_CANFD_1_VEC_ID,
    XPAR_INTC_0_CANFD_2_VEC_ID,
    XPAR_INTC_0_CANFD_3_VEC_ID
};

static void flush_rx(uint8_t channel);

/** Print cumulative software-side TX/feedback counters for one channel. */
static void canfd_debug_print_counters(uint8_t channel)
{
    canfd_drv_t *drv;
    uint32_t tx_gap;
    uint32_t tx_feedback_gap;
    uint32_t feedback_gap;

    if (channel >= g_active_channels) {
        return;
    }
    drv = &g_can[channel];
    tx_gap = drv->diag.tx_submitted >= drv->diag.tx_completed ?
             drv->diag.tx_submitted - drv->diag.tx_completed : 0U;
    tx_feedback_gap = drv->diag.tx_requests >=
                      (drv->diag.feedback_tx + drv->diag.feedback_error) ?
                      drv->diag.tx_requests - drv->diag.feedback_tx -
                      drv->diag.feedback_error : 0U;
    feedback_gap = drv->diag.feedback_generated >=
                   drv->diag.feedback_published ?
                   drv->diag.feedback_generated -
                   drv->diag.feedback_published : 0U;
    CANFD_DBG("C%u STAT tx=%u/%u/%u fail=%u pend=%u txfb_miss=%u "
              "rx=%u/%u drop=%u ovf=%u fb=%u/%u fail=%u pend=%u\r\n",
              (unsigned)channel,
              (unsigned)drv->diag.tx_requests,
              (unsigned)drv->diag.tx_submitted,
              (unsigned)drv->diag.tx_completed,
              (unsigned)drv->diag.tx_submit_failed,
              (unsigned)tx_gap,
              (unsigned)tx_feedback_gap,
              (unsigned)drv->diag.rx_dequeued,
              (unsigned)drv->diag.rx_queued,
              (unsigned)drv->diag.rx_dropped,
              (unsigned)drv->events.rx_overflow,
              (unsigned)drv->diag.feedback_generated,
              (unsigned)drv->diag.feedback_published,
              (unsigned)drv->diag.feedback_publish_failed,
              (unsigned)feedback_gap);
}

/** Return the number of hardware channels, capped at the software maximum. */
static uint32_t canfd_active_channel_count(void)
{
    uint32_t count = XPAR_XCANFD_NUM_INSTANCES;

    if (count == 0U) {
        count = CANFD_CHANNEL_COUNT;
    }
    return count > CANFD_CHANNEL_COUNT ? CANFD_CHANNEL_COUNT : count;
}

/** Weak timestamp hook; board code should override it with the system timer. */
__attribute__((weak)) uint64_t canfd_timestamp_us(void)
{
    return 0U;
}

/** Calculate the byte-sum checksum stored in the final four bytes. */
static uint32_t protocol_xor(const void *data, uint32_t bytes)
{
    if (data == NULL || bytes < sizeof(uint32_t) || (bytes & 3U) != 0U) {
        return 0U;
    }
    return canfd_xor32(data, bytes - sizeof(uint32_t));
}

/** Reverse the byte order within each four-byte CAN payload word. */
static void reverse_feedback_data_words(uint32_t *data, uint32_t bytes)
{
    uint8_t *p;
    uint32_t i;
    uint8_t tmp;

    if (data == NULL || (bytes & 3U) != 0U) {
        return;
    }
    p = (uint8_t *)data;
    for (i = 0U; i < bytes; i += 4U) {
        tmp = p[i];
        p[i] = p[i + 3U];
        p[i + 3U] = tmp;
        tmp = p[i + 1U];
        p[i + 1U] = p[i + 2U];
        p[i + 2U] = tmp;
    }
}

/** Print one CAN/CAN-FD message in the byte order visible to the host. */
static void canfd_debug_dump_frame(const char *direction, uint8_t channel,
                                   uint32_t sequence, uint8_t type,
                                   uint8_t state, uint32_t total,
                                   uint32_t idr, uint32_t dlcr,
                                   const uint32_t *data)
{
#if CANFD_DEBUG && CANFD_DEBUG_DATA
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t can_id;
    int length;
    int i;

    if (direction == NULL || data == NULL) {
        return;
    }
    if ((idr & XCANFD_IDR_IDE_MASK) != 0U) {
        can_id = (((idr & XCANFD_IDR_ID1_MASK) >> XCANFD_IDR_ID1_SHIFT)
                  << 18U) |
                 ((idr & XCANFD_IDR_ID2_MASK) >> XCANFD_IDR_ID2_SHIFT);
    } else {
        can_id = (idr & XCANFD_IDR_ID1_MASK) >> XCANFD_IDR_ID1_SHIFT;
    }
    length = canfd_dlc_to_len(dlcr);
    if (direction[0] == 'T') {
        xil_printf("C%u TX#%u id=%08x len=%d data=",
                   (unsigned)channel, (unsigned)sequence,
                   (unsigned)can_id, length);
    } else {
        xil_printf("C%u FB#%u t=%02x n=%u st=%u id=%08x len=%d data=",
                   (unsigned)channel, (unsigned)sequence, (unsigned)type,
                   (unsigned)total, (unsigned)state, (unsigned)can_id,
                   length);
    }
    for (i = 0; i < length; ++i) {
        xil_printf("%02x", (unsigned)bytes[i]);
    }
    xil_printf("\r\n");
#else
    (void)direction;
    (void)channel;
    (void)sequence;
    (void)type;
    (void)state;
    (void)total;
    (void)idr;
    (void)dlcr;
    (void)data;
#endif
}

/** Initialize XIntc, connect CAN-FD handlers and enable CPU exceptions. */
static int canfd_interrupt_init(void)
{
    uint32_t i;
    int status;

    status = XIntc_Initialize(&g_intc, XPAR_INTC_0_DEVICE_ID);
    if (status != XST_SUCCESS) {
        CANFD_DBG("CANFD: XIntc init failed (%d)\r\n", status);
        return status;
    }
    for (i = 0U; i < g_active_channels; ++i) {
        if (g_can[i].initialized == 0U) {
            continue;
        }
        status = XIntc_Connect(&g_intc, g_intc_id[i],
                               (XInterruptHandler)XCanFd_IntrHandler,
                               &g_can[i]);
        if (status != XST_SUCCESS) {
            CANFD_DBG("CANFD%u: interrupt connect failed vec=%u (%d)\r\n",
                      (unsigned)i, (unsigned)g_intc_id[i], status);
            return status;
        }
        CANFD_VDBG("CANFD%u: interrupt connected vec=%u\r\n",
                   (unsigned)i, (unsigned)g_intc_id[i]);
    }
    status = XIntc_Start(&g_intc, XIN_REAL_MODE);
    if (status != XST_SUCCESS) {
        CANFD_DBG("CANFD: XIntc start failed (%d)\r\n", status);
        return status;
    }
    for (i = 0U; i < g_active_channels; ++i) {
        if (g_can[i].initialized != 0U) {
            XIntc_Enable(&g_intc, g_intc_id[i]);
        }
    }
    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                  (Xil_ExceptionHandler)XIntc_InterruptHandler,
                                  &g_intc);
    Xil_ExceptionEnable();
    CANFD_VDBG("CANFD: CPU exceptions and CAN-FD interrupts enabled\r\n");
    return XST_SUCCESS;
}

/** Validate a host CAN-FD transmit record before accessing the controller. */
static int canfd_frame_valid(const canfd_frame_t *frame, uint8_t channel)
{
    uint32_t dlc;

    if (frame == NULL || channel >= g_active_channels) {
        CANFD_DBG("CANFD%u: TX validation failed: null/invalid channel\r\n",
                  (unsigned)channel);
        return 0;
    }
    if (frame->head != CANFD_FRAME_HEAD ||
        frame->direction != CANFD_HOST_TO_DEVICE ||
        frame->type != CANFD_CMD_NORMAL ||
        frame->length != CANFD_FRAME_SIZE || frame->ch_id != channel) {
        CANFD_DBG("CANFD%u: TX validation header head=0x%08x dir=%u type=0x%02x len=%u ch=%u\r\n",
                  (unsigned)channel, (unsigned)frame->head,
                  (unsigned)frame->direction, (unsigned)frame->type,
                  (unsigned)frame->length, (unsigned)frame->ch_id);
        return 0;
    }
    dlc = (frame->dlcr & CANFD_DLCR_DLC_MASK) >> CANFD_DLCR_DLC_SHIFT;
    if (!canfd_dlcr_is_fd(frame->dlcr) && dlc > 8U) {
        CANFD_DBG("CANFD%u: TX validation classic CAN DLC=%u\r\n",
                  (unsigned)channel, (unsigned)dlc);
        return 0;
    }
    if (canfd_dlcr_has_brs(frame->dlcr) &&
        (!canfd_dlcr_is_fd(frame->dlcr) ||
         g_can[channel].brs_enabled == 0U)) {
        CANFD_DBG("CANFD%u: TX validation invalid BRS dlcr=0x%08x enabled=%u\r\n",
                  (unsigned)channel, (unsigned)frame->dlcr,
                  (unsigned)g_can[channel].brs_enabled);
        return 0;
    }
    /* ESI, EFC, MM and the low timestamp field are receive/event fields. */
    if ((frame->dlcr & (CANFD_DLCR_ESI_MASK | CANFD_DLCR_EFC_MASK |
                        CANFD_DLCR_MM_MASK | CANFD_DLCR_TIMESTAMP_MASK)) != 0U) {
        CANFD_DBG("CANFD%u: TX validation RX-only DLCR bits=0x%08x\r\n",
                  (unsigned)channel, (unsigned)frame->dlcr);
        return 0;
    }
    if (canfd_dlc_to_len(frame->dlcr) > (int)CANFD_MAX_DATA_LEN) {
        CANFD_DBG("CANFD%u: TX validation payload too long dlcr=0x%08x\r\n",
                  (unsigned)channel, (unsigned)frame->dlcr);
        return 0;
    }
    if (frame->xor_checksum != protocol_xor(frame, sizeof(*frame))) {
        CANFD_DBG("CANFD%u: TX validation checksum got=0x%08x expected=0x%08x\r\n",
                  (unsigned)channel, (unsigned)frame->xor_checksum,
                  (unsigned)protocol_xor(frame, sizeof(*frame)));
        return 0;
    }
    return 1;
}

/** Store one feedback frame in DDR and publish its descriptor through FIFO. */
static void queue_feedback(uint8_t channel, const canfd_frame_t *source,
                           uint8_t type, uint8_t state, uint8_t xor_state)
{
    canfd_feedback_frame_t feedback;
    canfd_fifo_cmd_t command;
    UINTPTR address;
    uint32_t feedback_seq;
    int fifo_status;

    if (source == NULL || channel >= g_active_channels) {
        return;
    }
    memset(&feedback, 0, sizeof(feedback));
    feedback.head = CANFD_FRAME_HEAD;
    feedback.direction = CANFD_DEVICE_TO_HOST;
    feedback.type = type;
    feedback.length = CANFD_FRAME_SIZE;
    feedback.ch_id = channel;
    feedback.total_count = total_count;
    feedback.ch_count = channel_count[channel];
    feedback.idr = source->idr;
    feedback.dlcr = source->dlcr;
    memcpy(feedback.data, source->data, sizeof(feedback.data));
    /* Reverse payload words only for feedback generated from an interrupt-
     * received CAN frame.  Transmit status/error feedback is unchanged. */
    if (type == CANFD_CMD_RX) {
        reverse_feedback_data_words(feedback.data, sizeof(feedback.data));
    }
    feedback_seq = ++g_can[channel].diag.feedback_generated;
    if (type == CANFD_CMD_STATUS || type == CANFD_CMD_SIM_RX) {
        ++g_can[channel].diag.feedback_tx;
    } else if (type == CANFD_CMD_RX) {
        ++g_can[channel].diag.feedback_rx;
    } else {
        ++g_can[channel].diag.feedback_error;
    }
    feedback.states = state;
    feedback.xor_states = xor_state;
    feedback.timestamp = canfd_timestamp_us();
    feedback.xor_checksum = protocol_xor(&feedback, sizeof(feedback));

    address = g_can[channel].response_base + g_can[channel].response_offset;
    memcpy((void *)address, &feedback, sizeof(feedback));
    Xil_DCacheFlushRange((INTPTR)address, sizeof(feedback));

    memset(&command, 0, sizeof(command));
    command.head = CANFD_FIFO_HEAD;
    command.direction = CANFD_FIFO_RX;
    command.type = 1U;
    command.pkt_base_addr = (uint32_t)address;
    command.pkt_len = sizeof(feedback);
    canfd_debug_dump_frame("FB", channel, feedback_seq, type,
                           feedback.states, feedback.total_count,
                           feedback.idr, feedback.dlcr, feedback.data);
    /* Feedback publication is flow-controlled too.  A transiently full
     * AXIS-FIFO must not turn a completed CAN frame into a missing host
     * record, so wait for space before declaring publication failed. */
    {
        uint32_t waited_us = 0U;
        do {
            fifo_status = axis_fifo_write_data(g_data_fifo[channel],
                                                (unsigned char *)&command,
                                                sizeof(command));
            if (fifo_status == 0) {
                break;
            }
            usleep(CANFD_TX_POLL_US);
            waited_us += CANFD_TX_POLL_US;
        } while (waited_us < CANFD_FEEDBACK_WAIT_TIMEOUT_US);
    }
    if (fifo_status != 0) {
        ++g_can[channel].diag.feedback_publish_failed;
        CANFD_DBG("CANFD%u: feedback FIFO write failed (%d)\r\n",
                  (unsigned)channel, fifo_status);
    } else {
        ++g_can[channel].diag.feedback_published;
        CANFD_VDBG("CANFD%u: feedback type=0x%02x addr=0x%08x len=%u\r\n",
                  (unsigned)channel, (unsigned)type, (unsigned)address,
                  (unsigned)sizeof(feedback));
    }

    g_can[channel].response_offset += sizeof(feedback);
    if (g_can[channel].response_offset >= CANFD_RESPONSE_OFFSET) {
        g_can[channel].response_offset = 0U;
    }

    /* Print on feedback cadence as well as TX cadence, so a receive-only
     * workload still produces a useful loss report. */
    if (CANFD_DEBUG_SUMMARY_INTERVAL != 0U &&
        (g_can[channel].diag.feedback_generated %
         CANFD_DEBUG_SUMMARY_INTERVAL) == 0U &&
        g_can[channel].diag.feedback_generated !=
        g_can[channel].diag.stats_last_print) {
        canfd_debug_print_counters(channel);
        g_can[channel].diag.stats_last_print =
            g_can[channel].diag.feedback_generated;
    }
}

/**
 * Submit one frame and wait until its controller TX buffer is released.
 *
 * XCanFd_Send() is non-blocking: success means only that the frame was copied
 * into a free hardware buffer.  Without this wait the host can receive all
 * status records and stop the test while the CAN controller is still draining
 * its buffers.  A bounded retry also prevents a full TX buffer from being
 * treated as a silently accepted frame.
 */
static int canfd_send_and_wait(uint8_t channel, canfd_frame_t *frame,
                               uint32_t *tx_buffer, uint8_t *timed_out,
                               uint8_t *submitted)
{
    uint32_t waited_us = 0U;
    uint32_t bus_off_before;
    uint32_t cancel_before;
    XCanFd *can;
    int status;

    if (timed_out != NULL) {
        *timed_out = 0U;
    }
    if (submitted != NULL) {
        *submitted = 0U;
    }
    if (channel >= g_active_channels || frame == NULL || tx_buffer == NULL) {
        return XST_INVALID_PARAM;
    }
    can = &g_can[channel].can;

    /* Wait for a free hardware buffer rather than dropping the frame when
     * all 32 TX buffers are occupied. */
    do {
        status = XCanFd_Send(&g_can[channel].can, &frame->idr, tx_buffer);
        if (status != XST_FIFO_NO_ROOM) {
            break;
        }
        flush_rx(channel);
        usleep(CANFD_TX_POLL_US);
        waited_us += CANFD_TX_POLL_US;
    } while (waited_us < CANFD_TX_WAIT_TIMEOUT_US);

    if (status != XST_SUCCESS) {
        if (status == XST_FIFO_NO_ROOM && timed_out != NULL) {
            *timed_out = 1U;
        }
        return status;
    }

    if (submitted != NULL) {
        *submitted = 1U;
    }

    /* A bus-off reset releases the buffer without putting the frame on the
     * wire.  Treat that as a failed completion instead of reporting success.
     */
    bus_off_before = g_can[channel].events.bus_off;
    cancel_before = g_can[channel].events.tx_cancel_served;
    waited_us = 0U;
    while (!XCanFd_IsBufferTransmitted(can, *tx_buffer)) {
        if (g_can[channel].events.bus_off != bus_off_before ||
            g_can[channel].events.tx_cancel_served != cancel_before) {
            if (timed_out != NULL) {
                *timed_out = 1U;
            }
            return XST_FAILURE;
        }
        flush_rx(channel);
        usleep(CANFD_TX_POLL_US);
        waited_us += CANFD_TX_POLL_US;
        if (waited_us >= CANFD_TX_WAIT_TIMEOUT_US) {
            if (timed_out != NULL) {
                *timed_out = 1U;
            }
            return XST_FAILURE;
        }
    }

    ++g_can[channel].diag.tx_completed;
    return XST_SUCCESS;
}

/** Send raw XCanFd words and publish a feedback record of the requested type. */
static int send_words(uint8_t channel, canfd_frame_t *frame,
                      uint8_t feedback_type)
{
    uint32_t tx_buffer;
    uint8_t timed_out;
    uint8_t submitted;
    uint8_t feedback_state;
    uint8_t feedback_xor_state;
    int status;

    if (channel >= g_active_channels || frame == NULL) {
        return XST_INVALID_PARAM;
    }
    ++g_can[channel].diag.tx_requests;
    /* Print before validation so malformed host frames remain diagnosable. */
    canfd_debug_dump_frame("TX", channel, g_can[channel].diag.tx_requests,
                           frame->type, 0U, 0U, frame->idr, frame->dlcr,
                           frame->data);
    if (feedback_type == CANFD_CMD_STATUS &&
        !canfd_frame_valid(frame, channel)) {
        ++g_can[channel].diag.tx_submit_failed;
        CANFD_DBG("CANFD%u: invalid TX frame idr=0x%08x dlcr=0x%08x\r\n",
                  (unsigned)channel, (unsigned)frame->idr,
                  (unsigned)frame->dlcr);
        queue_feedback(channel, frame, CANFD_CMD_SINGLE_ERR, 0U, 2U);
        return XST_INVALID_PARAM;
    }
    /* IDR, DLCR and the 64-byte data array are contiguous in the packed
     * protocol frame.  Send directly from DDR, then wait for the selected
     * controller buffer to complete before publishing success feedback. */
    status = canfd_send_and_wait(channel, frame, &tx_buffer, &timed_out,
                                 &submitted);
    if (submitted != 0U) {
        ++g_can[channel].diag.tx_submitted;
    }
    if (status != XST_SUCCESS) {
        ++g_can[channel].diag.tx_submit_failed;
        CANFD_DBG("CANFD%u: TX failed status=%d%s\r\n",
                  (unsigned)channel, status,
                  timed_out != 0U ? " timeout" : "");
        feedback_state = 0U;
        /* Keep the legacy protocol meaning: 1 = not processed/timeout,
         * 2 = immediate controller submission failure. */
        feedback_xor_state = timed_out != 0U ? 1U : 2U;
    } else {
        feedback_state = 1U;
        /* Legacy CAN status frames use xor_states=0 for success. */
        feedback_xor_state = 0U;
        CANFD_VDBG("CANFD%u: TX completed idr=0x%08x dlcr=0x%08x txbuf=%u\r\n",
                  (unsigned)channel, (unsigned)frame->idr,
                  (unsigned)frame->dlcr, (unsigned)tx_buffer);
    }

    total_count = total_count == 0xFFFFFFFFU ? 0U : total_count + 1U;
    channel_count[channel] = channel_count[channel] == 0xFFFFFFFFU ?
                             0U : channel_count[channel] + 1U;
    queue_feedback(channel, frame, feedback_type,
                   feedback_state, feedback_xor_state);
    return status;
}

/** Send one regular 112-byte host CAN-FD frame. */
static int send_one(uint8_t channel, canfd_frame_t *frame)
{
    return send_words(channel, frame, CANFD_CMD_STATUS);
}

/** Validate a DDR descriptor and transmit each contained 112-byte frame. */
static void process_ddr(uint8_t channel, const canfd_fifo_cmd_t *command)
{
    uint32_t i;
    canfd_frame_t *frame;
    if (command == NULL || command->pkt_len == 0U ||
        command->pkt_len % CANFD_FRAME_SIZE != 0U ||
        command->pkt_len > (CANFD_DDR_END - CANFD_DDR_BASE) ||
        command->pkt_base_addr < CANFD_DDR_BASE ||
        command->pkt_base_addr + command->pkt_len < command->pkt_base_addr ||
        command->pkt_base_addr + command->pkt_len > CANFD_DDR_END) {
        CANFD_DBG("CANFD%u: invalid DDR descriptor dir=%u addr=0x%08x len=%u\r\n",
                  (unsigned)channel, command != NULL ? (unsigned)command->direction : 0U,
                  command != NULL ? (unsigned)command->pkt_base_addr : 0U,
                  command != NULL ? (unsigned)command->pkt_len : 0U);
        return;
    }
    CANFD_VDBG("CANFD%u: DDR TX descriptor dir=%u addr=0x%08x len=%u frames=%u\r\n",
               (unsigned)channel, (unsigned)command->direction,
               (unsigned)command->pkt_base_addr, (unsigned)command->pkt_len,
               (unsigned)(command->pkt_len / CANFD_FRAME_SIZE));

    Xil_DCacheInvalidateRange((INTPTR)command->pkt_base_addr,
                              command->pkt_len);
    for (i = 0U; i < command->pkt_len; i += CANFD_FRAME_SIZE) {
        if (i != 0U && g_can[channel].packet_interval_us != 0U) {
            usleep(g_can[channel].packet_interval_us);
        }
        /* The protocol frame's IDR/DLCR/data area is a contiguous 72-byte
         * controller message at offset 16.  Use the DDR frame in place;
         * there is no per-frame 112-byte temporary copy. */
        frame = (canfd_frame_t *)(UINTPTR)(command->pkt_base_addr + i);
        (void)send_one(channel, frame);
        /* Interleave RX forwarding with batch TX so the 1024-entry software
         * queue cannot fill merely because one DDR descriptor is large. */
        flush_rx(channel);
    }
}

/** Convert queued hardware RX messages into host feedback records. */
static void flush_rx(uint8_t channel)
{
    canfd_frame_info_t info;
    canfd_frame_t frame;

    while (g_can[channel].rx_rb != NULL &&
           ringbuffer_get(g_can[channel].rx_rb, &info, sizeof(info)) != 0U) {
        memset(&frame, 0, sizeof(frame));
        memcpy(&frame.idr,
               (void *)(g_can[channel].rx_storage +
                        (UINTPTR)info.offset * CANFD_MESSAGE_SIZE),
               CANFD_MESSAGE_SIZE);
        frame.head = CANFD_FRAME_HEAD;
        frame.direction = CANFD_DEVICE_TO_HOST;
        frame.type = CANFD_CMD_RX;
        frame.length = CANFD_FRAME_SIZE;
        frame.ch_id = channel;
        frame.total_count = total_count;
        frame.ch_count = channel_count[channel];
        frame.xor_checksum = protocol_xor(&frame, sizeof(frame));
        CANFD_VDBG("CANFD%u: RX frame idr=0x%08x dlcr=0x%08x len=%d total=%u ch_count=%u\r\n",
                   (unsigned)channel, (unsigned)frame.idr,
                   (unsigned)frame.dlcr, canfd_dlc_to_len(frame.dlcr),
                   (unsigned)total_count, (unsigned)channel_count[channel]);
        CANFD_VDBG("CANFD%u: RX state=%u\r\n", (unsigned)channel,
                   (unsigned)info.states);
        queue_feedback(channel, &frame, CANFD_CMD_RX, info.states, 1U);
    }
}

/** Parse one init frame or one AXIS-FIFO command for a channel. */
static void parse_command(uint8_t channel, uint8_t *buffer, uint32_t length)
{
    canfd_config_frame_t config;
    canfd_fifo_cmd_t command;

    if (buffer == NULL || length < sizeof(uint32_t) ||
        channel >= g_active_channels) {
        return;
    }

    if (*(uint32_t *)buffer == CANFD_FRAME_HEAD &&
        length >= sizeof(config)) {
        memcpy(&config, buffer, sizeof(config));
        CANFD_VDBG("CANFD%u: protocol frame type=0x%02x len=%u\r\n",
                   (unsigned)channel, (unsigned)config.type,
                   (unsigned)length);
        if (config.direction != CANFD_HOST_TO_DEVICE ||
            config.length != CANFD_FRAME_SIZE || config.ch_id != channel ||
            config.xor_checksum != protocol_xor(&config, sizeof(config))) {
            CANFD_DBG("CANFD%u: invalid protocol frame dir=%u frame_len=%u ch=%u\r\n",
                      (unsigned)channel, (unsigned)config.direction,
                      (unsigned)config.length, (unsigned)config.ch_id);
            CANFD_DBG("CANFD%u: protocol checksum got=0x%08x expected=0x%08x\r\n",
                      (unsigned)channel, (unsigned)config.xor_checksum,
                      (unsigned)protocol_xor(&config, sizeof(config)));
            return;
        }
        if (config.type == CANFD_CMD_INIT) {
            if ((config.mode != XCANFD_MODE_NORMAL &&
                 config.mode != XCANFD_MODE_LOOPBACK) ||
                config.auto_send > 1U || config.brs_en > 1U) {
                CANFD_DBG("CANFD%u: invalid INIT mode=%u auto=%u brs=%u\r\n",
                          (unsigned)channel, (unsigned)config.mode,
                          (unsigned)config.auto_send, (unsigned)config.brs_en);
                return;
            }
            CANFD_VDBG("CANFD%u: INIT nominal=%u data=%u brs=%u mode=%u auto=%u ier=0x%04x interval=%u\r\n",
                       (unsigned)channel, (unsigned)config.baud_rate,
                       (unsigned)config.data_baud_rate, (unsigned)config.brs_en,
                       (unsigned)config.mode, (unsigned)config.auto_send,
                       (unsigned)config.ier, (unsigned)config.packet_interval);
            g_can[channel].mode = config.mode;
            g_can[channel].auto_retransmit = config.auto_send != 0U;
            g_can[channel].packet_interval_us = config.packet_interval;
            g_can[channel].interrupt_enable =
                config.ier & 0x0FFFU & ~XCANFD_IXR_TSCNT_OFLW_MASK;
            if (axi_canfd_config(&g_can[channel], config.baud_rate,
                                 config.data_baud_rate, config.brs_en) ==
                XST_SUCCESS) {
                XCanFd_InterruptDisable(&g_can[channel].can,
                                         XCANFD_IXR_ALL);
                XCanFd_InterruptEnable(&g_can[channel].can,
                                       g_can[channel].interrupt_enable);
                g_can[channel].initialized = 1U;
                CANFD_DBG("CANFD%u: INIT applied\r\n", (unsigned)channel);
            } else {
                CANFD_DBG("CANFD%u: INIT rejected by CAN-FD driver\r\n",
                          (unsigned)channel);
            }
        }
        /* Type 0x02 (PPS) is reserved by the protocol. Its GPIO scheduler is
         * board-specific and therefore does not alter CAN-FD registers here. */
        if (config.type != CANFD_CMD_INIT && config.type != CANFD_CMD_PPS) {
            CANFD_DBG("CANFD%u: unsupported host frame type=0x%02x\r\n",
                      (unsigned)channel, (unsigned)config.type);
        }
        return;
    }

    if (length != sizeof(command)) {
        CANFD_DBG("CANFD%u: ignored FIFO payload length=%u expected=%u\r\n",
                  (unsigned)channel, (unsigned)length,
                  (unsigned)sizeof(command));
        return;
    }
    memcpy(&command, buffer, sizeof(command));
    if (command.head == CANFD_FIFO_HEAD &&
        (command.direction == CANFD_FIFO_TX ||
         command.direction == CANFD_FIFO_HP_TX)) {
        process_ddr(channel, &command);
    } else {
        CANFD_DBG("CANFD%u: unknown FIFO command head=0x%08x dir=%u type=%u\r\n",
                  (unsigned)channel, (unsigned)command.head,
                  (unsigned)command.direction, (unsigned)command.type);
    }
}

/** Initialize all CAN-FD channels, their FIFOs, DDR buffers and interrupts. */
void axi_canfd_system_init(void)
{
    uint32_t i;

    g_active_channels = canfd_active_channel_count();
    /* Release queues if the system is reinitialized at runtime. */
    for (i = 0U; i < CANFD_CHANNEL_COUNT; ++i) {
        if (g_can[i].rx_rb != NULL) {
            ringbuffer_destroy(g_can[i].rx_rb);
            g_can[i].rx_rb = NULL;
        }
    }
    memset(g_can, 0, sizeof(g_can));
    memset(channel_count, 0, sizeof(channel_count));
    total_count = 0U;

    CANFD_VDBG("CANFD: BSP instances=%u, active channels=%u, software maximum=%u\r\n",
               (unsigned)XPAR_XCANFD_NUM_INSTANCES,
               (unsigned)g_active_channels, (unsigned)CANFD_CHANNEL_COUNT);
    for (i = 0U; i < g_active_channels; ++i) {
        g_can[i].channel = (uint8_t)i;
        g_can[i].device_id = g_device_id[i];
        g_can[i].intc_id = g_intc_id[i];
        /* Power-on defaults; the host INIT command applies requested rates. */
        g_can[i].nominal_baud = 500000U;
        g_can[i].data_baud = 2000000U;
        g_can[i].brs_enabled = 1U;
        g_can[i].mode = XCANFD_MODE_NORMAL;
        /* Reliability default: retry a frame after a lost ACK/arbitration.
         * The host INIT command may still explicitly disable this. */
        g_can[i].auto_retransmit = 1U;
        g_can[i].packet_interval_us = 0U;
        g_can[i].interrupt_enable =
            XCANFD_IXR_ALL & ~XCANFD_IXR_TSCNT_OFLW_MASK;
        g_can[i].status_base = g_status_base[i];
        g_can[i].response_base = g_status_base[i] + CANFD_RESPONSE_OFFSET;
        g_can[i].rx_storage = 0x2C000000U +
                              (UINTPTR)i * CANFD_RX_STORAGE_SIZE;
        g_can[i].axis_fifo_base = g_data_fifo[i];
        g_can[i].rx_rb = ringbuffer_create(1024U * sizeof(canfd_frame_info_t),
                                           sizeof(canfd_frame_info_t));
        if (g_can[i].rx_rb == NULL) {
            CANFD_DBG("CANFD%u: RX ring buffer allocation failed\r\n",
                      (unsigned)i);
        } else {
            CANFD_VDBG("CANFD%u: RX ring buffer ready capacity=%u (dynamic)\r\n",
                       (unsigned)i, (unsigned)(g_can[i].rx_rb->mask + 1U));
        }
        axis_fifo_reset(g_data_fifo[i]);
        axis_fifo_reset(g_init_fifo[i]);
        if (axi_canfd_init(&g_can[i]) != XST_SUCCESS) {
            xil_printf("CANFD%u initialization failed\r\n", (unsigned)i);
        } else {
            CANFD_VDBG("CANFD%u: ready dev=%u data_fifo=0x%08x init_fifo=0x%08x\r\n",
                       (unsigned)i, (unsigned)g_device_id[i],
                       (unsigned)g_data_fifo[i], (unsigned)g_init_fifo[i]);
        }
    }
    if (canfd_interrupt_init() != XST_SUCCESS) {
        xil_printf("CANFD interrupt initialization failed\r\n");
    }
    CANFD_DBG("CANFD: system initialization complete\r\n");
}

/** Poll initialization/data FIFOs forever and process host traffic. */
void axi_canfd_frame_transfer(void)
{
    uint32_t i;
    uint32_t length;
    int received;

    for (;;) {
        for (i = 0U; i < g_active_channels; ++i) {
            while (axis_fifo_rx_occupancy(g_init_fifo[i]) != 0U) {
                length = axis_fifo_read_data_len(g_init_fifo[i]);
                CANFD_VDBG("CANFD%u: init FIFO payload bytes=%u\r\n",
                           (unsigned)i, (unsigned)length);
                if (length > sizeof(config_buf) || (length & 3U) != 0U) {
                    CANFD_DBG("CANFD%u: invalid init FIFO length=%u\r\n",
                              (unsigned)i, (unsigned)length);
                    (void)axis_fifo_discard_data(g_init_fifo[i], length);
                } else {
                    received = axis_fifo_read_data(g_init_fifo[i], config_buf,
                                                   (int)length);
                    if (received == (int)length) {
                        parse_command((uint8_t)i, config_buf, length);
                    } else {
                        CANFD_DBG("CANFD%u: init FIFO read failed got=%d expected=%u\r\n",
                                  (unsigned)i, received, (unsigned)length);
                    }
                }
            }

            while (axis_fifo_rx_occupancy(g_data_fifo[i]) != 0U) {
                length = axis_fifo_read_data_len(g_data_fifo[i]);
                CANFD_VDBG("CANFD%u: data FIFO payload bytes=%u\r\n",
                           (unsigned)i, (unsigned)length);
                if (length > sizeof(config_buf) || (length & 3U) != 0U) {
                    CANFD_DBG("CANFD%u: invalid data FIFO length=%u\r\n",
                              (unsigned)i, (unsigned)length);
                    (void)axis_fifo_discard_data(g_data_fifo[i], length);
                    continue;
                }
                received = axis_fifo_read_data(g_data_fifo[i], config_buf,
                                               (int)length);
                if (received == (int)length) {
                    parse_command((uint8_t)i, config_buf, length);
                } else {
                    CANFD_DBG("CANFD%u: data FIFO read failed got=%d expected=%u\r\n",
                              (unsigned)i, received, (unsigned)length);
                }
            }
            flush_rx((uint8_t)i);
        }
    }
}
