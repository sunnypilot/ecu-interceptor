// ********************* Includes *********************
#include "config.h"

#include "drivers/led.h"
#include "drivers/pwm.h"
#include "drivers/usb.h"
#include "drivers/simple_watchdog.h"
#include "drivers/bootkick.h"

#include "early_init.h"
#include "provision.h"

#include "safety/safety.h"

#include "health.h"

#include "drivers/can_common.h"

#ifdef STM32H7
  #include "drivers/fdcan.h"
#else
  #include "drivers/bxcan.h"
#endif

#include "power_saving.h"

#include "obj/gitversion.h"

#include "can_comms.h"
#include "main_comms.h"

// ********************* Serial debugging *********************

static bool check_started(void) {
  bool started = current_board->check_ignition() || ignition_can;
  return started;
}

void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (get_char(ring, &rcv)) {
    (void)put_char(ring, rcv);  // misra-c2012-17.7: cast to void is ok: debug function

    // only allow bootloader entry on debug builds
    #ifdef ALLOW_DEBUG
      // jump to DFU flash
      if (rcv == 'z') {
        enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
        NVIC_SystemReset();
      }
    #endif

    // normal reset
    if (rcv == 'x') {
      NVIC_SystemReset();
    }
  }
}

void send_interceptor_heartbeat(void) {
  uint8_t dat[8] = {0};
  dat[3] = (current_safety_mode >> 8) & 0xFF;
  dat[4] = current_safety_mode & 0xFF;
  dat[5] = (current_safety_param >> 8) & 0xFF;
  dat[6] = current_safety_param & 0xFF;
  dat[7] = sp_seen_recently ? 1 : 0;

  CANPacket_t to_send;
  to_send.extended = INTERCEPTOR_HEARTBEAT_MSG_ADDR >= 0x800 ? 1 : 0;
  to_send.addr = INTERCEPTOR_HEARTBEAT_MSG_ADDR;
  to_send.bus = 1;
  to_send.data_len_code = sizeof(dat);
  memcpy(to_send.data, dat, sizeof(dat));
  
  can_set_checksum(&to_send);
  can_send(&to_send, 1, true);
}

// ****************************** safety mode ******************************

// this is the only way to leave silent mode
void set_safety_mode(uint16_t mode, uint16_t param) {
  uint16_t mode_copy = mode;
  int err = set_safety_hooks(mode_copy, param);
  if (err == -1) {
    print("Error: safety set mode failed. Falling back to SILENT\n");
    mode_copy = SAFETY_SILENT;
    err = set_safety_hooks(mode_copy, 0U);
    // TERMINAL ERROR: we can't continue if SILENT safety mode isn't succesfully set
    assert_fatal(err == 0, "Error: Failed setting SILENT mode. Hanging\n");
  }
  safety_tx_blocked = 0;
  safety_rx_invalid = 0;

  switch (mode_copy) {
    case SAFETY_SILENT:
      set_intercept_relay(false, false);
      if (current_board->harness_config->has_harness) {
        current_board->set_can_mode(CAN_MODE_NORMAL);
      }
      can_silent = ALL_CAN_LIVE; // stock is ALL_CAN_SILENT
      break;
    case SAFETY_NOOUTPUT:
      set_intercept_relay(false, false);
      if (current_board->harness_config->has_harness) {
        current_board->set_can_mode(CAN_MODE_NORMAL);
      }
      can_silent = ALL_CAN_LIVE;
      break;
    case SAFETY_ELM327:
      set_intercept_relay(false, false);
      heartbeat_counter = 0U;
      heartbeat_lost = false;
      if (current_board->harness_config->has_harness) {
        // Clear any pending messages in the can core (i.e. sending while comma power is unplugged)
        // TODO: rewrite using hardware queues rather than fifo to cancel specific messages
        can_clear_send(CANIF_FROM_CAN_NUM(1), 1);
        if (param == 0U) {
          current_board->set_can_mode(CAN_MODE_OBD_CAN2);
        } else {
          current_board->set_can_mode(CAN_MODE_NORMAL);
        }
      }
      can_silent = ALL_CAN_LIVE;
      break;
    case SAFETY_HKG_ADAS_DRV_INTERCEPTOR:
      const bool is_intercept_active = param != 0U;
      set_intercept_relay(is_intercept_active, false);
      heartbeat_counter = is_intercept_active ? 0U : heartbeat_counter;
      heartbeat_lost = is_intercept_active ? false : heartbeat_lost;
      if (current_board->harness_config->has_harness) {
        current_board->set_can_mode(CAN_MODE_NORMAL);
      }
      can_silent = is_intercept_active ? ALL_CAN_LIVE : ALL_CAN_SILENT;
      break;
    default:
      set_intercept_relay(true, false);
      heartbeat_counter = 0U;
      heartbeat_lost = false;
      if (current_board->harness_config->has_harness) {
        current_board->set_can_mode(CAN_MODE_NORMAL);
      }
      can_silent = ALL_CAN_LIVE;
      break;
  }
  can_init_all();
}

bool is_car_safety_mode(uint16_t mode) {
  return (mode != SAFETY_SILENT) &&
         (mode != SAFETY_NOOUTPUT) &&
         (mode != SAFETY_ALLOUTPUT) &&
         (mode != SAFETY_ELM327);
}

// ***************************** main code *****************************

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
// cppcheck-suppress misra-c2012-8.4
void __initialize_hardware_early(void) {
  early_initialization();
}

static void __attribute__ ((noinline)) enable_fpu(void) {
  // enable the FPU
  SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
}

// go into SILENT when heartbeat isn't received for this amount of seconds.
#define HEARTBEAT_IGNITION_CNT_ON 2U
#define HEARTBEAT_IGNITION_CNT_OFF 2U

// called at 8Hz
static void tick_handler(void) {
  static uint32_t siren_countdown = 0; // siren plays while countdown > 0
  static uint32_t controls_allowed_countdown = 0;
  static uint8_t prev_harness_status = HARNESS_STATUS_NC;
  static uint8_t loop_counter = 0U;

  if (TICK_TIMER->SR != 0U) {

    // siren
    current_board->set_siren((loop_counter & 1U) && (siren_enabled || (siren_countdown > 0U)));

    // tick drivers at 8Hz
    fan_tick();
    harness_tick();
    simple_watchdog_kick();
    sound_tick();
    send_interceptor_heartbeat();

    // re-init everything that uses harness status
    if (harness.status != prev_harness_status) {
      prev_harness_status = harness.status;
      can_set_orientation(harness.status == HARNESS_STATUS_FLIPPED);

      // re-init everything that uses harness status
      can_init_all();
      set_safety_mode(current_safety_mode, current_safety_param);
      // set_power_save_state(power_save_status);
    }

    // decimated to 1Hz
    if (loop_counter == 0U) {
      can_live = pending_can_live;

      //puth(usart1_dma); print(" "); puth(DMA2_Stream5->M0AR); print(" "); puth(DMA2_Stream5->NDTR); print("\n");

      // reset this every 16th pass
      if ((uptime_cnt & 0xFU) == 0U) {
        pending_can_live = 0;
      }
      #ifdef DEBUG
        print("** blink ");
        print("rx:"); puth4(can_rx_q.r_ptr); print("-"); puth4(can_rx_q.w_ptr); print("  ");
        print("tx1:"); puth4(can_tx1_q.r_ptr); print("-"); puth4(can_tx1_q.w_ptr); print("  ");
        print("tx2:"); puth4(can_tx2_q.r_ptr); print("-"); puth4(can_tx2_q.w_ptr); print("  ");
        print("tx3:"); puth4(can_tx3_q.r_ptr); print("-"); puth4(can_tx3_q.w_ptr); print("\n");
      #endif

      // set green LED to be controls allowed
      led_set(LED_GREEN, controls_allowed | green_led_enabled);

      // turn off the blue LED, turned on by CAN
      // unless we are in power saving mode
      led_set(LED_BLUE, (uptime_cnt & 1U) && (power_save_status == POWER_SAVE_STATUS_ENABLED));

      const bool recent_heartbeat = heartbeat_counter == 0U;

      // tick drivers at 1Hz
      bootkick_tick(check_started(), recent_heartbeat);

      // increase heartbeat counter and cap it at the uint32 limit
      if (heartbeat_counter < UINT32_MAX) {
        heartbeat_counter += 1U;
      }

      // disabling heartbeat not allowed while in safety mode
      if (is_car_safety_mode(current_safety_mode)) {
        heartbeat_disabled = false;
      }

      if (siren_countdown > 0U) {
        siren_countdown -= 1U;
      }

      if (controls_allowed || heartbeat_engaged) {
        controls_allowed_countdown = 5U;
      } else if (controls_allowed_countdown > 0U) {
        controls_allowed_countdown -= 1U;
      } else {

      }

      // exit controls allowed if unused by openpilot for a few seconds
      if (controls_allowed && !heartbeat_engaged) {
        heartbeat_engaged_mismatches += 1U;
        if (heartbeat_engaged_mismatches >= 3U) {
          controls_allowed = false;
        }
      } else {
        heartbeat_engaged_mismatches = 0U;
      }

      if (!heartbeat_disabled) {
        // if the heartbeat has been gone for a while, go to SILENT safety mode and enter power save
        if (heartbeat_counter >= (check_started() ? HEARTBEAT_IGNITION_CNT_ON : HEARTBEAT_IGNITION_CNT_OFF)) {
          print("device hasn't sent a heartbeat for 0x");
          puth(heartbeat_counter);
          print(" seconds. Safety is set to SILENT mode.\n");

          if (controls_allowed_countdown > 0U) {
            siren_countdown = 3U;
            controls_allowed_countdown = 0U;
          }

          // set flag to indicate the heartbeat was lost
          if (is_car_safety_mode(current_safety_mode)) {
            heartbeat_lost = true;
          }

          // clear heartbeat engaged state
          heartbeat_engaged = false;

          if (current_safety_mode != SAFETY_SILENT) {
            set_safety_mode(SAFETY_SILENT, 0U);
          }

          // We do NOT want power save because we can't listen to CAN, and thus we can't react to SP requesting safety mode changes.
          // if (power_save_status != POWER_SAVE_STATUS_ENABLED) {
          //   set_power_save_state(POWER_SAVE_STATUS_ENABLED);
          // }

          // Also disable IR when the heartbeat goes missing
          current_board->set_ir_power(0U);

          // Run fan when device is up but not talking to us.
          // The bootloader enables the SOM GPIO on boot.
          fan_set_power(current_board->read_som_gpio() ? 30U : 0U);
        }
      }

      // check registers
      check_registers();

      // set ignition_can to false after 2s of no CAN seen
      if (ignition_can_cnt > 2U) {
        ignition_can = false;
      }

      // on to the next one
      uptime_cnt += 1U;
      safety_mode_cnt += 1U;
      ignition_can_cnt += 1U;

      // synchronous safety check
      safety_tick(&current_safety_config);
    }

    loop_counter++;
    loop_counter %= 8U;
  }
  TICK_TIMER->SR = 0;
}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  // shouldn't have interrupts here, but just in case
  disable_interrupts();

  // init early devices
  clock_init();
  peripherals_init();
  detect_board_type();
  led_init();
  // red+green leds enabled until succesful USB/SPI init, as a debug indicator
  led_set(LED_RED, true);
  led_set(LED_GREEN, true);
  adc_init();

  // print hello
  print("\n\n\n************************ MAIN START ************************\n");

  // check for non-supported board types
  assert_fatal(hw_type != HW_TYPE_UNKNOWN, "Unsupported board type");

  print("Config:\n");
  print("  Board type: 0x"); puth(hw_type); print("\n");

  // init board
  current_board->init();
  current_board->set_can_mode(CAN_MODE_NORMAL);
  if (current_board->harness_config->has_harness) {
    harness_init();
  }

  // panda has an FPU, let's use it!
  enable_fpu();

  microsecond_timer_init();

  current_board->set_siren(false);
  if (current_board->fan_max_rpm > 0U) {
    fan_init();
  }

  // init to SILENT and can silent
  set_safety_mode(SAFETY_SILENT, 0U);

  // enable CAN TXs
  enable_can_transceivers(true);

  // init watchdog for heartbeat loop, fed at 8Hz
  simple_watchdog_init(FAULT_HEARTBEAT_LOOP_WATCHDOG, (3U * 1000000U / 8U));

  // 8Hz timer
  REGISTER_INTERRUPT(TICK_TIMER_IRQ, tick_handler, 10U, FAULT_INTERRUPT_RATE_TICK)
  tick_timer_init();

#ifdef DEBUG
  print("DEBUG ENABLED\n");
#endif
  // enable USB (right before interrupts or enum can fail!)
  usb_init();

#ifdef ENABLE_SPI
  if (current_board->has_spi) {
    gpio_spi_init();
    spi_init();
  }
#endif

  led_set(LED_RED, false);
  led_set(LED_GREEN, false);
  led_set(LED_BLUE, false);

  print("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // LED should keep on blinking all the time
  while (true) {
    if (power_save_status == POWER_SAVE_STATUS_DISABLED) {
      #ifdef DEBUG_FAULTS
      if (fault_status == FAULT_STATUS_NONE) {
      #endif
        // useful for debugging, fade breaks = panda is overloaded
        for (uint32_t fade = 0U; fade < MAX_LED_FADE; fade += 1U) {
          led_set(LED_RED, true);
          delay(fade >> 4);
          led_set(LED_RED, false);
          delay((MAX_LED_FADE - fade) >> 4);
        }

        for (uint32_t fade = MAX_LED_FADE; fade > 0U; fade -= 1U) {
          led_set(LED_RED, true);
          delay(fade >> 4);
          led_set(LED_RED, false);
          delay((MAX_LED_FADE - fade) >> 4);
        }

      #ifdef DEBUG_FAULTS
      } else {
          led_set(LED_RED, 1);
          delay(512000U);
          led_set(LED_RED, 0);
          delay(512000U);
        }
      #endif
    } else {
      __WFI();
      SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    }
  }

  return 0;
}
