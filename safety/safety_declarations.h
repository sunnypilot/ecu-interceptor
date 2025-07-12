#pragma once

#include <stdint.h>
#include <stdbool.h>

// from cereal.car.CarParams.SafetyModel
#define SAFETY_SILENT 0U
#define SAFETY_ELM327 1U
#define SAFETY_ALLOUTPUT 2U
#define SAFETY_NOOUTPUT 3U
#define SAFETY_BODY 4U
//#define SAFETY_HYUNDAI_CANFD 5U

#define GET_BIT(msg, b) ((bool)!!(((msg)->data[((b) / 8U)] >> ((b) % 8U)) & 0x1U))
#define GET_BYTE(msg, b) ((msg)->data[(b)])
#define GET_FLAG(value, mask) (((__typeof__(mask))(value) & (mask)) == (mask)) // cppcheck-suppress misra-c2012-1.2; allow __typeof__

#define BUILD_SAFETY_CFG(rx, tx) ((safety_config){(rx), (sizeof((rx)) / sizeof((rx)[0])), \
                                                  (tx), (sizeof((tx)) / sizeof((tx)[0])), \
                                                  false})
#define SET_RX_CHECKS(rx, config) \
  do { \
    (config).rx_checks = (rx); \
    (config).rx_checks_len = sizeof((rx)) / sizeof((rx)[0]); \
    (config).disable_forwarding = false; \
  } while (0);

#define SET_TX_MSGS(tx, config) \
  do { \
    (config).tx_msgs = (tx); \
    (config).tx_msgs_len = sizeof((tx)) / sizeof((tx)[0]); \
    (config).disable_forwarding = false; \
  } while (0);

uint32_t GET_BYTES(const CANPacket_t *msg, int start, int len);

extern const int MAX_WRONG_COUNTERS;
#define MAX_ADDR_CHECK_MSGS 3U
#define MAX_SAMPLE_VALS 6
// used to represent floating point vehicle speed in a sample_t
#define VEHICLE_SPEED_FACTOR 1000.0
#define MAX_RT_INTERVAL 250000U

// Conversions
#define KPH_TO_MS (1.0 / 3.6)

// sample struct that keeps 6 samples in memory
struct sample_t {
  int values[MAX_SAMPLE_VALS];
  int min;
  int max;
};

// safety code requires floats
struct lookup_t {
  float x[3];
  float y[3];
};

typedef struct {
  int addr;
  int bus;
  int len;
  bool check_relay;              // if true, trigger relay malfunction if existence on destination bus and block forwarding to destination bus
  bool disable_static_blocking;  // if true, static blocking is disabled so safety mode can dynamically handle it (e.g. selective AEB pass-through)
} CanMsg;

typedef struct {
  const int addr;
  const int bus;
  const int len;
  const bool ignore_checksum;        // checksum check is not performed when set to true
  const bool ignore_counter;         // counter check is not performed when set to true
  const uint8_t max_counter;         // maximum value of the counter. 0 means that the counter check is skipped
  const bool ignore_quality_flag;    // true if quality flag check is skipped
  const uint32_t frequency;          // expected frequency of the message [Hz]
} CanMsgCheck;

typedef struct {
  // dynamic flags, reset on safety mode init
  bool msg_seen;
  int index;                         // if multiple messages are allowed to be checked, this stores the index of the first one seen. only msg[msg_index] will be used
  bool valid_checksum;               // true if and only if checksum check is passed
  int wrong_counters;                // counter of wrong counters, saturated between 0 and MAX_WRONG_COUNTERS
  bool valid_quality_flag;           // true if the message's quality/health/status signals are valid
  uint8_t last_counter;              // last counter value
  uint32_t last_timestamp;           // micro-s
  bool lagging;                      // true if and only if the time between updates is excessive
} RxStatus;

// params and flags about checksum, counter and frequency checks for each monitored address
typedef struct {
  const CanMsgCheck msg[MAX_ADDR_CHECK_MSGS];  // check either messages (e.g. honda steer)
  RxStatus status;
} RxCheck;

typedef struct {
  RxCheck *rx_checks;
  int rx_checks_len;
  const CanMsg *tx_msgs;
  int tx_msgs_len;
  bool disable_forwarding;
} safety_config;

typedef uint32_t (*get_checksum_t)(const CANPacket_t *to_push);
typedef uint32_t (*compute_checksum_t)(const CANPacket_t *to_push);
typedef uint8_t (*get_counter_t)(const CANPacket_t *to_push);
typedef bool (*get_quality_flag_valid_t)(const CANPacket_t *to_push);

typedef safety_config (*safety_hook_init)(uint16_t param);
typedef void (*rx_hook)(const CANPacket_t *to_push);
typedef bool (*tx_hook)(const CANPacket_t *to_send);  // returns true if the message is allowed
typedef bool (*fwd_hook)(int bus_num, int addr);      // returns true if the message should be blocked from forwarding

typedef struct {
  safety_hook_init init;
  rx_hook rx;
  tx_hook tx;
  fwd_hook fwd;
  get_checksum_t get_checksum;
  compute_checksum_t compute_checksum;
  get_counter_t get_counter;
  get_quality_flag_valid_t get_quality_flag_valid;
} safety_hooks;

bool safety_rx_hook(const CANPacket_t *to_push);
bool safety_tx_hook(CANPacket_t *to_send);
int to_signed(int d, int bits);
void update_sample(struct sample_t *sample, int sample_new);
bool get_longitudinal_allowed(void);
int ROUND(float val);
void gen_crc_lookup_table_8(uint8_t poly, uint8_t crc_lut[]);
#ifdef CANFD
void gen_crc_lookup_table_16(uint16_t poly, uint16_t crc_lut[]);
#endif

void safety_tick(const safety_config *safety_config);

// This can be set by the safety hooks
extern bool controls_allowed;
extern bool relay_malfunction;
extern bool safety_rx_checks_invalid;

// state for controls_allowed timeout logic
extern bool heartbeat_engaged;             // openpilot enabled, passed in heartbeat USB command
extern uint32_t heartbeat_engaged_mismatches;  // count of mismatches between heartbeat_engaged and controls_allowed

extern int alternative_experience;

// time since safety mode has been changed
extern uint32_t safety_mode_cnt;

typedef struct {
  uint16_t id;
  const safety_hooks *hooks;
} safety_hook_config;

extern uint16_t current_safety_mode;
extern uint16_t current_safety_param;
extern int current_safety_param_sp;
extern safety_config current_safety_config;

int safety_fwd_hook(int bus_num, int addr);
int set_safety_hooks(uint16_t mode, uint16_t param);

extern const safety_hooks body_hooks;
extern const safety_hooks elm327_hooks;
extern const safety_hooks nooutput_hooks;
extern const safety_hooks alloutput_hooks;
//extern const safety_hooks hyundai_canfd_hooks;
