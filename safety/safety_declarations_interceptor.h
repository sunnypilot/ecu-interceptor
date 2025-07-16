#pragma once

#define ADAS_DRV_INTERCEPTOR_OPT_MSG_ADDR 0x256
#include "safety_declarations.h"

typedef struct {
  uint16_t checksum;
  uint8_t counter;
  uint16_t requested_safety_mode;
  uint16_t requested_safety_param;
} AdasDrvInterceptOptMsg;
#define INTERCEPTOR_HEARTBEAT_MSG_ADDR 0x258

static inline AdasDrvInterceptOptMsg unpack_interceptor_opt_msg(const CANPacket_t * msg) {
  AdasDrvInterceptOptMsg adas = {
    .requested_safety_mode = (uint16_t)GET_BYTES(msg, 3, 4),
    .requested_safety_param = (uint16_t)GET_BYTES(msg, 5, 6),
  };
  return adas;
}

uint32_t sunnypilot_detected_last = 0U;
bool sp_seen_recently = false;
void send_interceptor_heartbeat(void);
