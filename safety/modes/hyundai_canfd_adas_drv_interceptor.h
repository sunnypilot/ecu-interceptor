#pragma once

#include "main_definitions.h"
#include "safety/modes/hyundai_canfd.h"
#include "safety/modes/defaults.h"
#include "safety/safety_declarations.h"

#define HYUNDAI_CANFD_ADAS_DRV_SCC_MSGS(bus) \
{0x1A0, bus, 8, .check_relay = false},       \

#define HEARTBEAT_MSG_ADDR 0x258
// No need for this, we no longer really checking the rx here
// #define HYUNDAI_CANFD_ADAS_INTERCEPTOR_MESSAGES() 
// {.msg = {{0x258, 1, 8, .max_counter=0, .ignore_counter = true, .ignore_checksum = true, .ignore_quality_flag = true, .frequency=100U}}}

#define ADAS_DRV_BUS 2
#define CAR_BUS 0
#define COMMA_BUS 1 //A1, or L-can

uint16_t init_param = 0;

static const CanMsg HYUNDAI_CANFD_ADAS_DRV_TX_MSGS[] = {
  HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, false)
};

static int hyundai_canfd_adas_drv_interceptor_tamper_hook(int source_bus, int addr, int default_destination_bus) {
  const bool is_scc_msg = addr == 0x1A0;
  if (source_bus == COMMA_BUS || !is_scc_msg)
    return default_destination_bus;
  
  // Update the scc_block_allowed status based on elapsed time
  const uint32_t ts_elapsed = get_ts_elapsed(MICROSECOND_TIMER->CNT, sunnypilot_detected_last);
  sp_seen_recently = (ts_elapsed <= 150000);

  // If the source bus is ADAS_DRV_BUS, we assume that the message is coming from the ADAS unit
  const bool should_block = sp_seen_recently && source_bus == ADAS_DRV_BUS;
  if (should_block) {
#ifdef DEBUG
    print("hyundai_canfd_adas_drv_interceptor_tamper_hook: redirecting to COMMA_BUS, originally meant for bus "); puth(source_bus); print("\n");
#endif
    return COMMA_BUS;
  }
  
  return default_destination_bus;
}

safety_config hyundai_canfd_adas_interceptor_init(uint16_t param) {
  hyundai_common_init(param);
  safety_config ret;

  // static RxCheck hyundai_canfd_interceptor_rx_checks[] = { };
  // SET_RX_CHECKS(hyundai_canfd_interceptor_rx_checks, ret);
  ret.rx_checks = NULL;
  ret.rx_checks_len = 0;

  ret.tx_msgs = NULL;
  ret.tx_msgs_len = 0;

  // with param we will decide what functionalities are allowed, for now, we assume that non zero param means full functionality
  ret.disable_forwarding = !adas_drv_ecu_long_interceptor_enabled;
  controls_allowed = adas_drv_ecu_long_interceptor_enabled;
  print("hyundai_canfd_adas_interceptor_init initialized, forwarding disabled ["); print(ret.disable_forwarding ? "YES" : "NO"); print("]\n");
  
  return ret;
}

// Note: tx is not used when the message is being forwarded. It's only used when messages are sent from OP
// but we have a different setup here, and no messages come from OP.
const safety_hooks hyundai_canfd_adas_drv_interceptor_hooks = {
  .init = hyundai_canfd_adas_interceptor_init,
  .tamper = hyundai_canfd_adas_drv_interceptor_tamper_hook,
  .get_counter = hyundai_canfd_get_counter,
  .get_checksum = hyundai_canfd_get_checksum,
  .compute_checksum = hyundai_common_canfd_compute_checksum,
};
