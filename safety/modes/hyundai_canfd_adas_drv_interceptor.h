#pragma once

#include "safety/safety_declarations.h"
#include "safety/modes/hyundai_canfd.h"

#define HYUNDAI_CANFD_ADAS_DRV_SCC_MSGS(bus) \
{0x1A0, bus, 8, .check_relay = false},  /* CRUISE_BUTTON */   \

#define ADAS_DRV_BUS 2
#define CAR_BUS 0
#define COMMA_BUS 1

uint32_t sunnypilot_detected_last = 0;
bool block_adas_drv_ecu = false;

static const CanMsg HYUNDAI_CANFD_ADAS_DRV_TX_MSGS[] = {
  HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, false)
};

static int hyundai_canfd_adas_drv_interceptor_tamper_hook(int source_bus, int addr, int default_destination_bus) {
  const int is_scc_msg = addr == 0x1A0;
  const uint32_t ts = MICROSECOND_TIMER->CNT;
   
  // Update the last detected timestamp if an SCC message is from CAR_BUS
  if (source_bus == CAR_BUS && is_scc_msg) {
    sunnypilot_detected_last = ts;
  }

  // Default forwarding logic
  int bus_dst = default_destination_bus;

  // Update the scc_block_allowed status based on elapsed time
  const uint32_t ts_elapsed = get_ts_elapsed(ts, sunnypilot_detected_last);
  block_adas_drv_ecu = (ts_elapsed <= 150000);

  // If we are allowed to block, and this is an scc msg coming from ADAS (or somehow we are sending it TO the ADAS) we block
  block_adas_drv_ecu = block_adas_drv_ecu && is_scc_msg && (source_bus == ADAS_DRV_BUS || bus_dst == ADAS_DRV_BUS);
  if (block_adas_drv_ecu) {
    bus_dst = COMMA_BUS; // Change SCC_CONTROL address to avoid being processed by the car
  }

  return bus_dst;
}

// Note: tx is not used when the message is being forwarded. It's only used when messages are sent from OP
// but we have a different setup here, and no messages come from OP.
const safety_hooks hyundai_canfd_adas_drv_interceptor_hooks = {
  .init = alloutput_init,
  //.rx = hyundai_canfd_adas_drv_interceptor_rx_hook,
  .tamper = hyundai_canfd_adas_drv_interceptor_tamper_hook,
  // .fwd = hyundai_canfd_adas_drv_interceptor_fwd_hook,
  .get_counter = hyundai_canfd_get_counter,
  .get_checksum = hyundai_canfd_get_checksum,
  .compute_checksum = hyundai_common_canfd_compute_checksum,
};
