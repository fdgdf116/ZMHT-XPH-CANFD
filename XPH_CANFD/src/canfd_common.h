#ifndef CANFD_COMMON_H
#define CANFD_COMMON_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "xil_types.h"

#define CANFD_CHANNEL_COUNT       4U  /* software maximum; hardware may expose fewer */
#define CANFD_FRAME_SIZE          112U
#define CANFD_MESSAGE_SIZE        72U
#define CANFD_MAX_DATA_LEN        64U
#define CANFD_CONFIG_SIZE         CANFD_FRAME_SIZE
#define CANFD_RX_STORAGE_SIZE     (2U * 1024U * 1024U)
#define CANFD_RESPONSE_OFFSET     (8U * 1024U * 1024U)
#define CANFD_STATUS_REGION_SIZE  (48U * 128U)
#define CANFD_FIFO_CMD_SIZE       16U
#define CANFD_MAX_CONFIG_BYTES    (CANFD_CHANNEL_COUNT * CANFD_FRAME_SIZE)
#define CANFD_SIM_HEADER_SIZE     32U

#define CANFD_FRAME_HEAD          0x499602D2U
#define CANFD_FIFO_HEAD           0x499601D1U

/* CAN-FD Data Length Code Register bit layout (PG223 / ISO 11898-1). */
#define CANFD_DLCR_DLC_MASK       0xF0000000U
#define CANFD_DLCR_DLC_SHIFT      28U
#define CANFD_DLCR_EDL_MASK       0x08000000U
#define CANFD_DLCR_BRS_MASK       0x04000000U
#define CANFD_DLCR_ESI_MASK       0x02000000U
#define CANFD_DLCR_EFC_MASK       0x01000000U
#define CANFD_DLCR_MM_MASK        0x00FF0000U
#define CANFD_DLCR_TIMESTAMP_MASK 0x0000FFFFU

enum {
    CANFD_CMD_INIT       = 0x00,
    CANFD_CMD_NORMAL     = 0x01,
    CANFD_CMD_PPS        = 0x02,
    CANFD_CMD_SIM        = 0x03,
    CANFD_CMD_STATUS     = 0x80,
    CANFD_CMD_RX         = 0x81,
    CANFD_CMD_SIM_RX     = 0x82,
    CANFD_CMD_MULTI_ERR  = 0xFE,
    CANFD_CMD_SINGLE_ERR = 0xFF
};

enum {
    CANFD_HOST_TO_DEVICE = 0,
    CANFD_DEVICE_TO_HOST = 1
};

enum {
    CANFD_FIFO_TX = 0,
    CANFD_FIFO_HP_TX = 1,
    CANFD_FIFO_RX = 2,
    CANFD_FIFO_SIM = 3
};

/* Protocol frame layouts are packed on purpose: their byte offsets are part
 * of the host/FPGA ABI.  Use memcpy for unaligned external buffers. */
#pragma pack(push, 1)
/** 112-byte host initialization/configuration frame. */
typedef struct {
    uint32_t head;
    uint8_t direction;
    uint8_t type;
    uint8_t length;
    uint8_t ch_id;
    uint32_t total_count;
    uint32_t ch_count;
    uint32_t baud_rate;
    uint8_t mode;
    uint8_t auto_send;
    uint16_t ier;
    uint32_t packet_interval;
    uint32_t data_baud_rate;
    uint8_t brs_en;
    uint8_t reserved[67];
    uint64_t timestamp;
    uint32_t xor_checksum;
} canfd_config_frame_t;

/** 112-byte host transmit frame; message area starts at byte 17. */
typedef struct {
    uint32_t head;
    uint8_t direction;
    uint8_t type;
    uint8_t length;
    uint8_t ch_id;
    uint32_t total_count;
    uint32_t ch_count;
    uint32_t idr;
    uint32_t dlcr;
    uint32_t data[16];
    uint8_t padding[8];
    uint16_t packets_sum;
    uint16_t packets_no;
    uint64_t timestamp;
    uint32_t xor_checksum;
} canfd_frame_t;

/** 112-byte device feedback frame with states at byte 97. */
typedef struct {
    uint32_t head;
    uint8_t direction;
    uint8_t type;
    uint8_t length;
    uint8_t ch_id;
    uint32_t total_count;
    uint32_t ch_count;
    uint32_t idr;
    uint32_t dlcr;
    uint32_t data[16];
    uint8_t padding[8];
    uint8_t states;
    uint8_t xor_states;
    uint16_t reserved;
    uint64_t timestamp;
    uint32_t xor_checksum;
} canfd_feedback_frame_t;

/** 16-byte AXIS-FIFO descriptor pointing to a DDR packet. */
typedef struct {
    uint32_t head;
    uint8_t direction;
    uint8_t hp_ch_num;
    uint16_t type;
    uint32_t pkt_base_addr;
    uint32_t pkt_len;
} canfd_fifo_cmd_t;

/* Variable-length simulator request.  send_data contains 72-byte CAN-FD
 * message units (IDR, DLCR and 64 data bytes). */
typedef struct {
    uint32_t head;
    uint8_t direction;
    uint8_t type;
    uint16_t length;
    uint8_t ch_id;
    uint32_t total_count;
    uint32_t ch_count;
    uint16_t packet_type;
    uint32_t can_id;
    uint32_t data_type;
    uint16_t rcv_len;
    uint16_t send_len;
    uint8_t reserved;
    uint8_t send_data[1];
} canfd_sim_packet_t;
#pragma pack(pop)

typedef struct { uint32_t offset; uint8_t states; } canfd_frame_info_t;

typedef char canfd_config_size_check[(sizeof(canfd_config_frame_t) == 112) ? 1 : -1];
typedef char canfd_frame_size_check[(sizeof(canfd_frame_t) == 112) ? 1 : -1];
typedef char canfd_feedback_size_check[(sizeof(canfd_feedback_frame_t) == 112) ? 1 : -1];
typedef char canfd_fifo_size_check[(sizeof(canfd_fifo_cmd_t) == 16) ? 1 : -1];
typedef char canfd_config_offset_check[
    (offsetof(canfd_config_frame_t, timestamp) == 100U) ? 1 : -1];
typedef char canfd_frame_offset_check[
    (offsetof(canfd_frame_t, idr) == 16U &&
     offsetof(canfd_frame_t, data) == 24U &&
     offsetof(canfd_frame_t, packets_sum) == 96U) ? 1 : -1];
typedef char canfd_feedback_offset_check[
    (offsetof(canfd_feedback_frame_t, states) == 96U) ? 1 : -1];
typedef char canfd_fifo_offset_check[
    (offsetof(canfd_fifo_cmd_t, pkt_base_addr) == 8U &&
     offsetof(canfd_fifo_cmd_t, pkt_len) == 12U) ? 1 : -1];

/** Calculate the protocol checksum as an unsigned byte-wise sum. */
static inline uint32_t canfd_xor32(const void *data, uint32_t bytes)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t v = 0U;
    uint32_t i;

    if (p == NULL) {
        return 0U;
    }
    for (i = 0U; i < bytes; ++i) {
        v += (uint32_t)p[i];
    }
    return v;
}

/** Convert a CAN/CAN-FD DLCR value to its payload length in bytes. */
static inline int canfd_dlc_to_len(uint32_t dlcr)
{
    static const uint8_t fd_len[16] = {
        0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
        8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
    };
    uint32_t dlc = (dlcr & CANFD_DLCR_DLC_MASK) >> CANFD_DLCR_DLC_SHIFT;
    return (dlcr & CANFD_DLCR_EDL_MASK) ?
           fd_len[dlc] : (dlc > 8U ? 8 : (int)dlc);
}

/** Return non-zero when a DLCR describes a CAN-FD (EDL) frame. */
static inline int canfd_dlcr_is_fd(uint32_t dlcr)
{
    return (dlcr & CANFD_DLCR_EDL_MASK) != 0U;
}

/** Return non-zero when bit-rate switching is requested in a DLCR. */
static inline int canfd_dlcr_has_brs(uint32_t dlcr)
{
    return (dlcr & CANFD_DLCR_BRS_MASK) != 0U;
}

/** Return the ESI value captured by the controller on reception. */
static inline int canfd_dlcr_has_esi(uint32_t dlcr)
{
    return (dlcr & CANFD_DLCR_ESI_MASK) != 0U;
}

/** Return the smallest DLC able to carry @p len bytes (0..64). */
static inline uint32_t canfd_len_to_dlc(int len)
{
    static const uint8_t fd_len[16] = {
        0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
        8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
    };
    uint32_t i;
    if (len <= 0) {
        return 0U;
    }
    if (len <= 8) {
        return (uint32_t)len;
    }
    for (i = 9U; i < 16U; ++i) {
        if (len <= fd_len[i]) {
            return i;
        }
    }
    return 15;
}

#endif
