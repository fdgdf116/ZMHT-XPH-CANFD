/* Compatibility include for applications shared with the legacy CAN build. */
#ifndef AXI_CAN_DRV_COMPAT_H
#define AXI_CAN_DRV_COMPAT_H
#include "axi_canfd_drv.h"
typedef canfd_drv_t can_drv_t;
#define axi_can_init       axi_canfd_init
#define axi_can_config(d,b) axi_canfd_config((d),(b),0,0)
#define axi_can_set_mode   axi_canfd_set_mode
#endif
