/* CAN-FD protocol supersedes the legacy CAN wire format in this application. */
#ifndef CAN_COMMON_COMPAT_H
#define CAN_COMMON_COMPAT_H
#include "canfd_common.h"
typedef canfd_frame_t can_request_frame_t;
typedef canfd_feedback_frame_t can_response_frame_t;
#endif
