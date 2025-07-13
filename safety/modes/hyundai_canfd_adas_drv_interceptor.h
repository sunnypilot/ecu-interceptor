#pragma once

#include "safety/safety_declarations.h"
#include "safety/modes/hyundai_canfd.h"
#include "safety/modes/defaults.h"

#define HYUNDAI_CANFD_ADAS_DRV_SCC_MSGS(bus) \
{0x1A0, bus, 8, .check_relay = false},       \

#define HEARTBEAT_MSG_ADDR 0x258
#define HYUNDAI_CANFD_ADAS_INTERCEPTOR_MESSAGES() \
{.msg = {{0x258, 1, 8, .max_counter=0, .ignore_counter = true, .ignore_checksum = true, .ignore_quality_flag = true, .frequency=100U}}}

#define ADAS_DRV_BUS 2
#define CAR_BUS 0
#define COMMA_BUS 1 //A1, or L-can

uint32_t sunnypilot_detected_last = 0;
bool is_comma_alive = false;

static const CanMsg HYUNDAI_CANFD_ADAS_DRV_TX_MSGS[] = {
  HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, false)
};

void hyundai_canfd_adas_drv_interceptor_rx_hook(const CANPacket_t * to_push) {
  const int addr = GET_ADDR(to_push);

  if (addr == HEARTBEAT_MSG_ADDR) {
    #ifdef DEBUG
      print("hyundai_canfd_adas_drv_interceptor_rx_hook: addr=0x"); puth(addr); print(", status="); puth(GET_BYTE(to_push, 3)); print("\n");
    #endif 
    const unsigned short status = GET_BYTE(to_push, 3) & 0x3U;
    if (status > 0) { 
      sunnypilot_detected_last = MICROSECOND_TIMER->CNT;
    }
  }
}

static int hyundai_canfd_adas_drv_interceptor_tamper_hook(int source_bus, int addr, int default_destination_bus) {
  const bool is_scc_msg = addr == 0x1A0;
  if (source_bus == COMMA_BUS || !is_scc_msg)
    return default_destination_bus;
  
  // Update the scc_block_allowed status based on elapsed time
  const uint32_t ts_elapsed = get_ts_elapsed(MICROSECOND_TIMER->CNT, sunnypilot_detected_last);
  is_comma_alive = (ts_elapsed <= 150000);

  // If the source bus is ADAS_DRV_BUS, we assume that the message is coming from the ADAS unit
  const bool should_block = is_comma_alive && source_bus == ADAS_DRV_BUS;
  if (should_block) {
#ifdef DEBUG
    print("hyundai_canfd_adas_drv_interceptor_tamper_hook: redirecting to COMMA_BUS, originally meant for bus "); puth(source_bus); print("\n");
#endif
    return COMMA_BUS;
  }
  return default_destination_bus;
}

safety_config hyundai_canfd_adas_interceptor_init(uint16_t param) {
  UNUSED(param);

  static RxCheck hyundai_canfd_interceptor_rx_checks[] = {
    HYUNDAI_CANFD_ADAS_INTERCEPTOR_MESSAGES()
  };

  safety_config ret;
  SET_RX_CHECKS(hyundai_canfd_interceptor_rx_checks, ret);

  ret.tx_msgs = NULL;
  ret.tx_msgs_len = 0;
  ret.disable_forwarding = false;
  controls_allowed = true;
  print("hyundai_canfd_adas_interceptor_init initiailized with ["); puth(ret.rx_checks_len); print("]\n");
  
  return ret;
}

// Note: tx is not used when the message is being forwarded. It's only used when messages are sent from OP
// but we have a different setup here, and no messages come from OP.
const safety_hooks hyundai_canfd_adas_drv_interceptor_hooks = {
  .init = hyundai_canfd_adas_interceptor_init,
  .rx = hyundai_canfd_adas_drv_interceptor_rx_hook,
  .tamper = hyundai_canfd_adas_drv_interceptor_tamper_hook,
  // .fwd = hyundai_canfd_adas_drv_interceptor_fwd_hook,
  .get_counter = hyundai_canfd_get_counter,
  .get_checksum = hyundai_canfd_get_checksum,
  .compute_checksum = hyundai_common_canfd_compute_checksum,
};
