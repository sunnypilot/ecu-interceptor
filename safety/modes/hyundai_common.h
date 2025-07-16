#pragma once

#include "safety/safety_declarations.h"

extern uint16_t hyundai_canfd_crc_lut[256];
uint16_t hyundai_canfd_crc_lut[256];

// common state
extern bool adas_drv_ecu_long_interceptor_enabled;
bool adas_drv_ecu_long_interceptor_enabled = false;

#ifdef CANFD
uint32_t hyundai_common_canfd_compute_checksum(const CANPacket_t *to_push) {
  int len = GET_LEN(to_push);
  uint32_t address = GET_ADDR(to_push);

  uint16_t crc = 0;

  for (int i = 2; i < len; i++) {
    crc = (crc << 8U) ^ hyundai_canfd_crc_lut[(crc >> 8U) ^ GET_BYTE(to_push, i)];
  }

  // Add address to crc
  crc = (crc << 8U) ^ hyundai_canfd_crc_lut[(crc >> 8U) ^ ((address >> 0U) & 0xFFU)];
  crc = (crc << 8U) ^ hyundai_canfd_crc_lut[(crc >> 8U) ^ ((address >> 8U) & 0xFFU)];

  if (len == 24) {
    crc ^= 0x819dU;
  } else if (len == 32) {
    crc ^= 0x9f5bU;
  } else {

  }

  return crc;
}
#endif

void hyundai_common_init(uint16_t param) {
  const int ADAS_DRV_ECU_LONG_INTERCEPTOR = 8;
  adas_drv_ecu_long_interceptor_enabled = GET_FLAG(param, ADAS_DRV_ECU_LONG_INTERCEPTOR);
}
