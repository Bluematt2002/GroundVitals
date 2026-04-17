#pragma once
#include <stdint.h>
#include <stdbool.h>

void     ble_receiver_init(void);
bool     ble_is_connected(void);

extern char     ble_cmd_dir[8];
extern uint32_t ble_cmd_duty;
extern bool     ble_cmd_updated;