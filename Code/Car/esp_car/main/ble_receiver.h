#pragma once
#include <stdint.h>
#include <stdbool.h>

void     ble_receiver_init(void);
bool     ble_is_connected(void);

extern char     ble_cmd_dir[8];   // "FWD ", "BACK", "LEFT", "RGHT", "IDLE"
extern uint32_t ble_cmd_duty;     // 0 - 1023
extern bool     ble_cmd_updated;  // true when new command arrives