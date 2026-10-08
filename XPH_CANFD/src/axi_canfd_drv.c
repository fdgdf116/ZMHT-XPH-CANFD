#include "axi_canfd_drv.h"

#include <string.h>

#include "xil_printf.h"

#ifndef CANFD_DEBUG
#define CANFD_DEBUG 1
#endif
#if CANFD_DEBUG
#define CANFD_DBG(...) xil_printf(__VA_ARGS__)
#else
#define CANFD_DBG(...)
#endif
#ifndef CANFD_VERBOSE
#define CANFD_VERBOSE 0
#endif
#if CANFD_DEBUG && CANFD_VERBOSE
#define CANFD_VDBG(...) xil_printf(__VA_ARGS__)
#else
#define CANFD_VDBG(...)
#endif

/* BSP CAN_CLK_FREQ_HZ is zero.  Both current HDFs request 80 MHz but export
 * can_clk = 79.6875 MHz (can_clk_x2 = 159.375 MHz).  Set this to 80000000U
 * after regenerating hardware with an exact 80/160 MHz clock pair. */
#ifndef CANFD_CAN_CLOCK_HZ
#define CANFD_CAN_CLOCK_HZ 79687500U
#endif

/* Accept the current HDF's -3906 ppm rate error, and report actual rates.
 * This is a software selection limit, not a guarantee of bus tolerance. */
#define CANFD_MAX_BAUD_ERROR_PPM 5000U

/* A missing/unclocked CAN-FD instance must not stall the MicroBlaze forever. */
#define CANFD_MODE_TIMEOUT 1000000U

/** Count software RX drops and rate-limit diagnostics from interrupt context. */
static void canfd_rx_drop(canfd_drv_t *drv)
{
    uint32_t dropped;

    ++drv->diag.rx_dropped;
    dropped = drv->diag.rx_dropped;
    if (dropped == 1U || (dropped & 0xFFU) == 0U) {
        CANFD_DBG("C%u RX_DROP=%u queued=%u\r\n",
                  (unsigned)drv->channel, (unsigned)dropped,
                  (unsigned)ringbuffer_len(drv->rx_rb));
    }
}

/** Wait for a controller mode transition and report a non-responsive IP. */
static int canfd_wait_mode(canfd_drv_t *drv, u8 expected,
                           const char *stage)
{
    uint32_t timeout;

    for (timeout = 0U; timeout < CANFD_MODE_TIMEOUT; ++timeout) {
        if (XCanFd_GetMode(&drv->can) == expected) {
            return XST_SUCCESS;
        }
    }
    CANFD_DBG("CANFD%u: mode timeout stage=%s expected=0x%02x actual=0x%02x base=0x%08x\r\n",
              (unsigned)drv->channel, stage, (unsigned)expected,
              (unsigned)XCanFd_GetMode(&drv->can),
              (unsigned)drv->can.CanFdConfig.BaseAddress);
    return XST_FAILURE;
}

/** Record a successful transmit interrupt for the owning channel. */
static void canfd_send_handler(void *ref)
{
    canfd_drv_t *drv = (canfd_drv_t *)ref;

    if (drv != NULL) {
        drv->events.tx_ok++;
        CANFD_VDBG("CANFD%u: TX complete count=%u\r\n",
                   (unsigned)drv->channel,
                   (unsigned)drv->events.tx_ok);
    }
}

/** Drain the XCanFd receive FIFO and queue message locations for the system task. */
static void canfd_recv_handler(void *ref)
{
    canfd_drv_t *drv = (canfd_drv_t *)ref;
    uint32_t frame_words[18];
    int mailbox_mode;

    if (drv == NULL) {
        return;
    }

    mailbox_mode = drv->can.CanFdConfig.Rx_Mode == 1U;
    while (mailbox_mode ||
           XCanFd_GetNofMessages_Stored_Rx_Fifo(&drv->can,
                                                XCANFD_RX_FIFO_0) > 0) {
        canfd_frame_info_t info;

        {
            int recv_status = (drv->can.CanFdConfig.Rx_Mode == 1U ?
                               XCanFd_Recv_Mailbox(&drv->can, frame_words) :
                               XCanFd_Recv_Sequential(&drv->can, frame_words));
            if (recv_status != XST_SUCCESS) {
                CANFD_DBG("CANFD%u: RX dequeue failed status=%d\r\n",
                          (unsigned)drv->channel, recv_status);
                break;
            }
        }

        ++drv->diag.rx_dequeued;

        if (drv->rx_rb != NULL && !ringbuffer_is_full(drv->rx_rb)) {
            info.offset = drv->rx_count;
            info.states = 0U;
            memcpy((void *)(drv->rx_storage +
                            (UINTPTR)drv->rx_count * CANFD_MESSAGE_SIZE),
                   frame_words, CANFD_MESSAGE_SIZE);
            if (ringbuffer_put(drv->rx_rb, &info, sizeof(info)) != 0U) {
                drv->rx_count = (drv->rx_count + 1U) %
                                (CANFD_RX_STORAGE_SIZE / CANFD_MESSAGE_SIZE);
                CANFD_VDBG("CANFD%u: RX queued slot=%u queued=%u\r\n",
                           (unsigned)drv->channel, (unsigned)info.offset,
                           (unsigned)ringbuffer_len(drv->rx_rb));
                ++drv->diag.rx_queued;
            } else {
                canfd_rx_drop(drv);
            }
        } else if (drv->rx_rb == NULL) {
            canfd_rx_drop(drv);
        } else {
            canfd_rx_drop(drv);
        }

        drv->events.rx_ok++;
        if (mailbox_mode) {
            break;
        }
    }
}

/** Accumulate nominal- and data-phase protocol errors reported by the IP. */
static void canfd_error_handler(void *ref, u32 mask)
{
    canfd_drv_t *drv = (canfd_drv_t *)ref;

    if (drv == NULL) {
        return;
    }

    CANFD_DBG("CANFD%u: protocol error mask=0x%08x\r\n",
              (unsigned)drv->channel, (unsigned)mask);

    if (mask & XCANFD_ESR_ACKER_MASK)   drv->errors.ack_error++;
    if (mask & XCANFD_ESR_BERR_MASK)    drv->errors.bit_error++;
    if (mask & XCANFD_ESR_STER_MASK)    drv->errors.stuff_error++;
    if (mask & XCANFD_ESR_FMER_MASK)    drv->errors.form_error++;
    if (mask & XCANFD_ESR_CRCER_MASK)   drv->errors.crc_error++;
    if (mask & XCANFD_ESR_F_BERR_MASK)  drv->errors.fd_bit_error++;
    if (mask & XCANFD_ESR_F_STER_MASK)  drv->errors.fd_stuff_error++;
    if (mask & XCANFD_ESR_F_FMER_MASK)  drv->errors.fd_form_error++;
    if (mask & XCANFD_ESR_F_CRCER_MASK) drv->errors.fd_crc_error++;
    drv->events.protocol_error++;
}

/** Handle bus-off, overflow and arbitration events generated by the IP. */
static void canfd_event_handler(void *ref, u32 mask)
{
    canfd_drv_t *drv = (canfd_drv_t *)ref;
    XCanFd *can;

    if (drv == NULL) {
        return;
    }
    can = &drv->can;

    if (mask & XCANFD_IXR_BSOFF_MASK) {
        drv->events.bus_off++;
        CANFD_DBG("CANFD%u: bus-off, resetting controller\r\n",
                  (unsigned)drv->channel);
        XCanFd_Reset(can);
        if (axi_canfd_config(drv, drv->nominal_baud, drv->data_baud,
                             drv->brs_enabled) == XST_SUCCESS) {
            XCanFd_InterruptEnable(can, drv->interrupt_enable);
        }
    }
    if (mask & XCANFD_IXR_RXFOFLW_MASK) drv->events.rx_overflow++;
    if (mask & XCANFD_IXR_ARBLST_MASK)  drv->events.arbitration_lost++;
    if (mask & XCANFD_IXR_TXCRS_MASK)   drv->events.tx_cancel_served++;
    if (mask & XCANFD_IXR_TXRRS_MASK)   drv->events.tx_ready_served++;
    if ((mask & ~XCANFD_IXR_TSCNT_OFLW_MASK) != 0U) {
        CANFD_DBG("CANFD%u: event mask=0x%08x\r\n",
                  (unsigned)drv->channel, (unsigned)mask);
    }
}

/**
 * Select bit timing for the CAN clock configured by CANFD_CAN_CLOCK_HZ.
 *
 * The XCanFd register fields are encoded as value minus one, therefore:
 * baud = CANFD_CAN_CLOCK_HZ / ((BRP + 1) * (TS1 + TS2 + 3)).  The selected
 * sample point
 * is 80 percent where the requested rate permits it.
 */
static int canfd_timing(uint32_t baud, int data_phase,
                        u8 *brp, u8 *ts1, u8 *ts2)
{
    static const u8 quanta[] = {20U, 40U, 80U, 16U, 10U, 5U};
    unsigned i;
    u32 ts1_max = data_phase ? XCANFD_F_BTR_TS1_MASK : XCANFD_BTR_TS1_MASK;
    u32 ts2_max = data_phase ?
        (XCANFD_F_BTR_TS2_MASK >> XCANFD_F_BTR_TS2_SHIFT) :
        (XCANFD_BTR_TS2_MASK >> XCANFD_BTR_TS2_SHIFT);

    /* Preserve the protocol's supported rate list. */
    switch (baud) {
    case 5000U: case 10000U: case 20000U: case 50000U:
    case 100000U: case 125000U: case 250000U: case 500000U:
    case 800000U: case 1000000U: case 2000000U:
    case 4000000U: case 5000000U: case 8000000U:
        break;
    default: return XST_INVALID_PARAM;
    }

    /* Prefer 20 TQ, enlarge for low rates and reduce only for high rates.
     * Round the 80% sample point to the nearest whole TQ.  Most candidates
     * are divisible by five; the 16-TQ candidate is needed for 5 Mbps and
     * gives an 81.25% sample point.  Check register masks, not the BSP's
     * MAX constants (some of those include unencodable boundary values). */
    for (i = 0U; i < sizeof(quanta) / sizeof(quanta[0]); ++i) {
        u32 tq = quanta[i];
        u32 sample_tq = (tq * 4U + 2U) / 5U;
        u32 segment1;
        u32 segment2;
        u32 quantum_rate = baud * tq;
        u32 divider = (CANFD_CAN_CLOCK_HZ + quantum_rate / 2U) / quantum_rate;
        u32 target_clock;
        u32 error;

        if (sample_tq < 2U || sample_tq >= tq) {
            continue;
        }
        segment1 = sample_tq - 2U;
        segment2 = tq - sample_tq - 1U;
        if (divider == 0U || divider > 256U ||
            segment1 > ts1_max || segment2 > ts2_max) {
            continue;
        }
        target_clock = quantum_rate * divider;
        error = CANFD_CAN_CLOCK_HZ > target_clock ?
                CANFD_CAN_CLOCK_HZ - target_clock :
                target_clock - CANFD_CAN_CLOCK_HZ;
        if ((uint64_t)error * 1000000U >
            (uint64_t)target_clock * CANFD_MAX_BAUD_ERROR_PPM) {
            continue;
        }
        *brp = (u8)(divider - 1U);
        *ts1 = (u8)segment1;
        *ts2 = (u8)segment2;
        return XST_SUCCESS;
    }
    return XST_INVALID_PARAM;
}

/** Configure nominal/data bit timing, BRS and automatic retransmission. */
int axi_canfd_config(canfd_drv_t *drv, uint32_t nominal_baud,
                     uint32_t data_baud, uint8_t brs_enabled)
{
    u8 nbrp, nts1, nts2;
    u8 dbrp = 0U, dts1 = 0U, dts2 = 0U;
    u8 target_mode;
    u32 mode_select;
    u32 tdc_offset = 0U;
    u32 data_prescaler;

    if (drv == NULL || nominal_baud == 0U || nominal_baud > 1000000U ||
        data_baud > 8000000U || (data_baud != 0U && data_baud < nominal_baud) ||
        (brs_enabled != 0U && data_baud == 0U)) {
        return XST_INVALID_PARAM;
    }

    /* Validate both phases before changing controller mode or registers. */
    if (canfd_timing(nominal_baud, 0, &nbrp, &nts1, &nts2) != XST_SUCCESS ||
        (data_baud != 0U &&
         canfd_timing(data_baud, 1, &dbrp, &dts1, &dts2) != XST_SUCCESS)) {
        CANFD_DBG("CANFD%u: unsupported timing clock=%u nominal=%u data=%u\r\n",
                  (unsigned)drv->channel, (unsigned)CANFD_CAN_CLOCK_HZ,
                  (unsigned)nominal_baud, (unsigned)data_baud);
        return XST_INVALID_PARAM;
    }
    if (brs_enabled != 0U && data_baud > 2000000U) {
        /* SSP = measured loop delay + offset; offset is in CAN clocks,
         * not prescaled TQ.  80 MHz / 4 Mbps gives offset = 16. */
        tdc_offset = ((u32)dbrp + 1U) * ((u32)dts1 + 2U);
        if (tdc_offset > (XCANFD_F_BRPR_TDCMASK >> TDC_SHIFT)) {
            return XST_INVALID_PARAM;
        }
    }

    CANFD_VDBG("CANFD%u: configure nominal=%u data=%u brs=%u auto=%u mode=%u\r\n",
                (unsigned)drv->channel, (unsigned)nominal_baud,
                (unsigned)data_baud, (unsigned)(brs_enabled != 0U),
                (unsigned)drv->auto_retransmit, (unsigned)drv->mode);

    XCanFd_EnterMode(&drv->can, XCANFD_MODE_CONFIG);
    if (canfd_wait_mode(drv, XCANFD_MODE_CONFIG, "enter-config") !=
        XST_SUCCESS) {
        return XST_FAILURE;
    }

    CANFD_VDBG("CANFD%u: nominal timing brp=%u ts1=%u ts2=%u\r\n",
                (unsigned)drv->channel, (unsigned)nbrp,
                (unsigned)nts1, (unsigned)nts2);
    if (XCanFd_SetBaudRatePrescaler(&drv->can, nbrp) != XST_SUCCESS) {
        return XST_FAILURE;
    }
    if (XCanFd_SetBitTiming(&drv->can, nts2, nts2, nts1) != XST_SUCCESS) {
        return XST_FAILURE;
    }

    if (data_baud != 0U) {
        CANFD_VDBG("CANFD%u: data timing brp=%u ts1=%u ts2=%u\r\n",
                    (unsigned)drv->channel, (unsigned)dbrp,
                    (unsigned)dts1, (unsigned)dts2);
        if (XCanFd_SetFBaudRatePrescaler(&drv->can, dbrp) != XST_SUCCESS ||
            XCanFd_SetFBitTiming(&drv->can, dts2, dts2, dts1) != XST_SUCCESS) {
            return XST_FAILURE;
        }
    }

    /* The BSP TDC setter only ORs offsets.  Clear stale fields on every
     * reconfiguration, including transitions back to low speed / BRS off. */
    data_prescaler = XCanFd_ReadReg(drv->can.CanFdConfig.BaseAddress,
                                    XCANFD_F_BRPR_OFFSET);
    data_prescaler &= ~(XCANFD_F_BRPR_TDCMASK | XCANFD_F_BRPR_TDC_ENABLE_MASK);
    if (tdc_offset != 0U) {
        data_prescaler |= (tdc_offset << TDC_SHIFT) |
                           XCANFD_F_BRPR_TDC_ENABLE_MASK;
    }
    XCanFd_WriteReg(drv->can.CanFdConfig.BaseAddress, XCANFD_F_BRPR_OFFSET,
                    data_prescaler);

    /* In the Xilinx API, "EnableNominal" sets the global BRSD bit and forces
     * CAN-FD frames to remain at the nominal rate.  Clear BRSD when protocol
     * configuration enables bit-rate switching. */
    if (brs_enabled != 0U) {
        XCanFd_SetBitRateSwitch_DisableNominal(&drv->can);
    } else {
        XCanFd_SetBitRateSwitch_EnableNominal(&drv->can);
    }

    /* DAR uses inverted protocol semantics: 1 disables retransmission.  Set
     * or clear it explicitly so a later host configuration can re-enable
     * automatic retransmission after it was previously disabled. */
    mode_select = XCanFd_ReadReg(drv->can.CanFdConfig.BaseAddress,
                                 XCANFD_MSR_OFFSET);
    if (drv->auto_retransmit != 0U) {
        mode_select &= ~XCANFD_MSR_DAR_MASK;
    } else {
        mode_select |= XCANFD_MSR_DAR_MASK;
    }
    XCanFd_WriteReg(drv->can.CanFdConfig.BaseAddress, XCANFD_MSR_OFFSET,
                    mode_select);
    CANFD_VDBG("CANFD%u: MSR=0x%08x (auto_retransmit=%u)\r\n",
                (unsigned)drv->channel, (unsigned)mode_select,
                (unsigned)drv->auto_retransmit);

    drv->nominal_baud = nominal_baud;
    drv->data_baud = data_baud;
    drv->brs_enabled = brs_enabled != 0U;
    target_mode = drv->mode != 0U ? drv->mode : XCANFD_MODE_NORMAL;
    XCanFd_EnterMode(&drv->can, target_mode);
    if (canfd_wait_mode(drv, target_mode, "enter-operating") !=
        XST_SUCCESS) {
        return XST_FAILURE;
    }

    CANFD_DBG("CANFD%u: clock=%u nominal=%u actual=%u data=%u actual=%u BRS=%u TDC=%u\r\n",
              (unsigned)drv->channel, (unsigned)CANFD_CAN_CLOCK_HZ,
              (unsigned)nominal_baud,
              (unsigned)(CANFD_CAN_CLOCK_HZ / ((nbrp + 1U) * (nts1 + nts2 + 3U))),
              (unsigned)data_baud, (unsigned)(data_baud == 0U ? 0U :
                  CANFD_CAN_CLOCK_HZ / ((dbrp + 1U) * (dts1 + dts2 + 3U))),
              (unsigned)drv->brs_enabled, (unsigned)tdc_offset);

    return XST_SUCCESS;
}

/** Change the controller operating mode and wait for the hardware response. */
void axi_canfd_set_mode(canfd_drv_t *drv, uint8_t mode)
{
    if (drv == NULL) {
        return;
    }

    drv->mode = mode;
    XCanFd_EnterMode(&drv->can, mode);
    if (canfd_wait_mode(drv, mode, "set-mode") != XST_SUCCESS) {
        CANFD_DBG("CANFD%u: set mode failed requested=0x%02x\r\n",
                  (unsigned)drv->channel, (unsigned)mode);
    }
}

/** Initialize one CAN-FD IP instance, install callbacks and enable interrupts. */
int axi_canfd_init(canfd_drv_t *drv)
{
    XCanFd_Config *config;
    int status;
    u8 operating_mode;

    if (drv == NULL) {
        return XST_INVALID_PARAM;
    }

    config = XCanFd_LookupConfig(drv->device_id);
    if (config == NULL) {
        CANFD_DBG("CANFD%u: CAN-FD config not found dev=%u\r\n",
                  (unsigned)drv->channel, (unsigned)drv->device_id);
        return XST_FAILURE;
    }

    status = XCanFd_CfgInitialize(&drv->can, config, config->BaseAddress);
    if (status != XST_SUCCESS) {
        CANFD_DBG("CANFD%u: CfgInitialize failed (%d)\r\n",
                  (unsigned)drv->channel, status);
        return status;
    }
    status = XCanFd_SelfTest(&drv->can);
    if (status != XST_SUCCESS) {
        CANFD_DBG("CANFD%u: self-test failed (%d)\r\n",
                  (unsigned)drv->channel, status);
        return status;
    }

    drv->mode = XCANFD_MODE_NORMAL;
    operating_mode = drv->mode;
    status = axi_canfd_config(drv,
                              drv->nominal_baud != 0U ? drv->nominal_baud : 500000U,
                              drv->data_baud,
                              drv->brs_enabled);
    if (status != XST_SUCCESS) {
        CANFD_DBG("CANFD%u: initial timing setup failed (%d)\r\n",
                  (unsigned)drv->channel, status);
        return status;
    }

    /* Acceptance filters are configuration registers; set them while the IP
     * is in configuration mode, then restore the requested operating mode. */
    axi_canfd_set_mode(drv, XCANFD_MODE_CONFIG);
    if (drv->can.CanFdConfig.Rx_Mode == 0U) {
        XCanFd_AcceptFilterDisable(&drv->can, XCANFD_AFR_UAF_ALL_MASK);
        XCanFd_AcceptFilterEnable(&drv->can, XCANFD_AFR_UAF_ALL_MASK);
    } else {
        u8 buffer_count = drv->can.CanFdConfig.NumofRxMbBuf;
        u8 buffer;

        for (buffer = 0U; buffer < buffer_count; ++buffer) {
            XCanFd_RxBuff_MailBox_DeActive(&drv->can, buffer);
            /* Mask 0 accepts every standard/extended identifier. */
            XCanFd_Set_MailBox_IdMask(&drv->can, buffer, 0U, 0U);
            XCanFd_RxBuff_MailBox_Active(&drv->can, buffer);
        }
    }
    axi_canfd_set_mode(drv, operating_mode);

    XCanFd_SetHandler(&drv->can, XCANFD_HANDLER_SEND,
                      (void *)canfd_send_handler, drv);
    XCanFd_SetHandler(&drv->can, XCANFD_HANDLER_RECV,
                      (void *)canfd_recv_handler, drv);
    XCanFd_SetHandler(&drv->can, XCANFD_HANDLER_ERROR,
                      (void *)canfd_error_handler, drv);
    XCanFd_SetHandler(&drv->can, XCANFD_HANDLER_EVENT,
                      (void *)canfd_event_handler, drv);
    XCanFd_InterruptEnable(&drv->can, drv->interrupt_enable);
    CANFD_VDBG("CANFD%u: callbacks installed IER=0x%08x rx_mode=%u\r\n",
               (unsigned)drv->channel, (unsigned)drv->interrupt_enable,
               (unsigned)drv->can.CanFdConfig.Rx_Mode);
    drv->initialized = 1U;
    CANFD_VDBG("CANFD%u: driver initialized dev=%u base=0x%08x\r\n",
               (unsigned)drv->channel, (unsigned)drv->device_id,
               (unsigned)config->BaseAddress);
    return XST_SUCCESS;
}
