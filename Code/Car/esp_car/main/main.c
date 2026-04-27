#if 1

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "ble_receiver.h"

#include "car_pins.h"

static const char *TAG = "ALL_WHEEL_TEST";

// ==============================
// PWM Setup
// ==============================
void setup_pwm()
{
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .freq_hz          = 20000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    // 8 channels (2 per motor)
    int pins[8] = {
        WHEEL_FRONT_LEFT_IN1_PIN,
        WHEEL_FRONT_LEFT_IN2_PIN,
        WHEEL_FRONT_RIGHT_IN1_PIN,
        WHEEL_FRONT_RIGHT_IN2_PIN,
        WHEEL_BACK_LEFT_IN1_PIN,
        WHEEL_BACK_LEFT_IN2_PIN,
        WHEEL_BACK_RIGHT_IN1_PIN,
        WHEEL_BACK_RIGHT_IN2_PIN
    };

    for (int i = 0; i < 8; i++) {
        ledc_channel_config_t ch = {
            .gpio_num   = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = i,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
        };
        ledc_channel_config(&ch);
    }
}

// ==============================
// Motor Control Helpers
// Channel mapping:
// FL: 0,1
// FR: 2,3
// BL: 4,5
// BR: 6,7
// ==============================

void set_motor(int ch_forward, int ch_reverse, uint32_t duty, bool forward)
{
    if (forward) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_forward, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_forward);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_reverse, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_reverse);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_forward, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_forward);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_reverse, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_reverse);
    }
}

void stop_all()
{
    ESP_LOGI(TAG, "STOP");
    for (int i = 0; i < 8; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, i, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, i);
    }
}

void move_forward(uint32_t duty)
{
    ESP_LOGI(TAG, "ALL FORWARD");
    set_motor(0, 1, duty, true);  // FL
    set_motor(2, 3, duty, false);  // FR
    set_motor(4, 5, duty, true);  // BL
    set_motor(6, 7, duty, false);  // BR
}

void move_reverse(uint32_t duty)
{
    ESP_LOGI(TAG, "ALL REVERSE");
    set_motor(0, 1, duty, false); // FL
    set_motor(2, 3, duty, true); // FR
    set_motor(4, 5, duty, false); // BL
    set_motor(6, 7, duty, true); // BR
}

void turn_left(uint32_t duty)
{
    ESP_LOGI(TAG, "SHIFT LEFT");
    // set_motor(0, 1, duty, false);  // FL
    // set_motor(2, 3, duty, false);  // FR
    set_motor(4, 5, duty, false); // BL
    set_motor(6, 7, duty, false); // BR
}

void turn_right(uint32_t duty)
{
    ESP_LOGI(TAG, "SHIFT RIGHT");
    // set_motor(0, 1, duty, true); // FL
    // set_motor(2, 3, duty, true); // FR
    set_motor(4, 5, duty, true);  // BL
    set_motor(6, 7, duty, true);  // BR
}

// ==============================
// Main Test Loop
// ==============================
void app_main(void)
{
    ESP_LOGI(TAG, "Starting BLE controlled vehicle");

    setup_pwm();
    ble_receiver_init();
    static bool wasConnected = false;
    // while (1)
    // {
    //     turn_left(700);

    //     vTaskDelay(pdMS_TO_TICKS(4000));

    //     stop_all();
    //     vTaskDelay(pdMS_TO_TICKS(2000));

    //     turn_right(700);

    //     vTaskDelay(pdMS_TO_TICKS(4000));

    //     stop_all();
    //     vTaskDelay(pdMS_TO_TICKS(3000));
    // }
    while (1)
    {
        bool nowConnected = ble_is_connected();

        // Safety stop on disconnect
        if (wasConnected && !nowConnected) {
            ESP_LOGI(TAG, "BLE lost — stopping motors");
            stop_all();
        }
        wasConnected = nowConnected;

        // Process incoming command
        if (ble_cmd_updated) {
            ble_cmd_updated = false;

            ESP_LOGI(TAG, "CMD: %s  DUTY: %lu  CONNECTED: %s",
                ble_cmd_dir,
                ble_cmd_duty,
                nowConnected ? "YES" : "NO");

            if (strcmp(ble_cmd_dir, "FWD ") == 0) {
                move_forward(ble_cmd_duty);
            } else if (strcmp(ble_cmd_dir, "BACK") == 0) {
                move_reverse(ble_cmd_duty);
            } else if (strcmp(ble_cmd_dir, "LEFT") == 0) {
                turn_left(ble_cmd_duty);
            } else if (strcmp(ble_cmd_dir, "RGHT") == 0) {
                turn_right(ble_cmd_duty);
            } else {
                stop_all();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

#else

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "ble_receiver.h"
#include "car_pins.h"

static const char *TAG = "BLE_SYNC_DRIVE";

// ==============================
// CONFIG
// ==============================
#define DUTY_MIN  350
#define DUTY_MAX  1023

#define SYNC_INTERVAL_MS 50

#define KP 3
#define DEADBAND 1
#define FILTER_ALPHA 0.5

#define STARTUP_CYCLES 5
#define MIN_VALID_PULSES 1

// ==============================
// SYNC MODE
// ==============================
typedef enum {
    SYNC_MODE_OFF,
    SYNC_MODE_FORWARD,
    SYNC_MODE_REVERSE
} sync_mode_t;

volatile sync_mode_t sync_mode = SYNC_MODE_OFF;
volatile int base_duty_cmd = 600;

// ==============================
// Wheel struct
// ==============================
typedef struct {
    int ch_fwd;
    int ch_rev;
    int hall_pin;

    volatile int pulse_count;
    int last_pulses;

    float filtered_speed;

    int duty;
} wheel_t;

// Wheels: FL, FR, BL, BR
wheel_t wheels[4] = {
    {0,1, WHEEL_FRONT_LEFT_HALL_A_PIN, 0,0,0,600},
    {2,3, WHEEL_FRONT_RIGHT_HALL_A_PIN,0,0,0,600},
    {4,5, WHEEL_BACK_LEFT_HALL_A_PIN,  0,0,0,600},
    {6,7, WHEEL_BACK_RIGHT_HALL_A_PIN, 0,0,0,600},
};

// Direction mapping (DO NOT CHANGE your mech hack)
bool forward_map[4]  = {true, false, true, false};
bool reverse_map[4]  = {false, true, false, true};

// ==============================
// ISR
// ==============================
static void IRAM_ATTR hall_isr(void *arg)
{
    wheel_t *w = (wheel_t *)arg;
    w->pulse_count++;
}

// ==============================
// PWM SETUP
// ==============================
void setup_pwm()
{
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .freq_hz          = 20000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    int pins[8] = {
        WHEEL_FRONT_LEFT_IN1_PIN,
        WHEEL_FRONT_LEFT_IN2_PIN,
        WHEEL_FRONT_RIGHT_IN1_PIN,
        WHEEL_FRONT_RIGHT_IN2_PIN,
        WHEEL_BACK_LEFT_IN1_PIN,
        WHEEL_BACK_LEFT_IN2_PIN,
        WHEEL_BACK_RIGHT_IN1_PIN,
        WHEEL_BACK_RIGHT_IN2_PIN
    };

    for (int i = 0; i < 8; i++) {
        ledc_channel_config_t ch = {
            .gpio_num   = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = i,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
        };
        ledc_channel_config(&ch);
    }
}

// ==============================
// HALL SETUP
// ==============================
void setup_hall()
{
    gpio_install_isr_service(0);

    for (int i = 0; i < 4; i++) {
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_POSEDGE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << wheels[i].hall_pin),
            .pull_up_en = GPIO_PULLUP_ENABLE
        };
        gpio_config(&io_conf);

        gpio_isr_handler_add(wheels[i].hall_pin, hall_isr, &wheels[i]);
    }
}

// ==============================
// MOTOR APPLY
// ==============================
void apply_motor(int i, bool forward, int duty)
{
    if (forward) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, wheels[i].ch_fwd, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, wheels[i].ch_fwd);
        
        ledc_set_duty(LEDC_LOW_SPEED_MODE, wheels[i].ch_rev, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, wheels[i].ch_rev);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, wheels[i].ch_fwd, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, wheels[i].ch_fwd);
        
        ledc_set_duty(LEDC_LOW_SPEED_MODE, wheels[i].ch_rev, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, wheels[i].ch_rev);
    }
}

// ==============================
// SET MOTOR
// ==============================
void set_motor(int ch_forward, int ch_reverse, uint32_t duty, bool forward)
{
    if (forward) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_forward, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_forward);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_reverse, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_reverse);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_forward, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_forward);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_reverse, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_reverse);
    }
}

// ==============================
// SYNC TASK
// ==============================
void sync_task(void *arg)
{
    int startup_counter = 0;

    while (1)
    {
        if (sync_mode == SYNC_MODE_OFF) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        int speeds[4];

        // Pulse delta
        for (int i = 0; i < 4; i++) {
            int now = wheels[i].pulse_count;
            speeds[i] = now - wheels[i].last_pulses;
            wheels[i].last_pulses = now;

            wheels[i].filtered_speed =
                FILTER_ALPHA * speeds[i] +
                (1 - FILTER_ALPHA) * wheels[i].filtered_speed;
        }

        // Startup ignore
        if (startup_counter < STARTUP_CYCLES) {
            startup_counter++;
            vTaskDelay(pdMS_TO_TICKS(SYNC_INTERVAL_MS));
            continue;
        }

        // Average
        float avg = 0;
        int count = 0;

        for (int i = 0; i < 4; i++) {
            if (speeds[i] >= MIN_VALID_PULSES) {
                avg += wheels[i].filtered_speed;
                count++;
            }
        }

        if (count == 0) {
            vTaskDelay(pdMS_TO_TICKS(SYNC_INTERVAL_MS));
            continue;
        }

        avg /= count;

        // Apply control
        for (int i = 0; i < 4; i++) {

            float error = avg - wheels[i].filtered_speed;

            if (fabs(error) < DEADBAND)
                error = 0;

            int target = base_duty_cmd + (int)(KP * error);

            if (speeds[i] == 0)
                target += 150;

            if (target > DUTY_MAX) target = DUTY_MAX;
            if (target < DUTY_MIN) target = DUTY_MIN;

            wheels[i].duty = target;

            bool forward =
                (sync_mode == SYNC_MODE_FORWARD) ? forward_map[i] : reverse_map[i];

            apply_motor(i, forward, target);
        }

        vTaskDelay(pdMS_TO_TICKS(SYNC_INTERVAL_MS));
    }
}

// ==============================
// ORIGINAL MOVEMENT (UNCHANGED)
// ==============================
void stop_all()
{
    sync_mode = SYNC_MODE_OFF;

    for (int i = 0; i < 8; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, i, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, i);
    }
}

void move_forward(uint32_t duty)
{
    base_duty_cmd = duty;
    sync_mode = SYNC_MODE_FORWARD;
}

void move_reverse(uint32_t duty)
{
    base_duty_cmd = duty;
    sync_mode = SYNC_MODE_REVERSE;
}

// TURNING = NO SYNC (your original behavior)
void turn_left(uint32_t duty)
{
    sync_mode = SYNC_MODE_OFF;

    set_motor(4, 5, duty, false);
    set_motor(6, 7, duty, false);
}

void turn_right(uint32_t duty)
{
    sync_mode = SYNC_MODE_OFF;

    set_motor(4, 5, duty, true);
    set_motor(6, 7, duty, true);
}

// ==============================
// MAIN
// ==============================
void app_main(void)
{
    ESP_LOGI(TAG, "Starting BLE synced vehicle");

    setup_pwm();
    setup_hall();
    ble_receiver_init();

    xTaskCreate(sync_task, "sync_task", 4096, NULL, 5, NULL);

    bool wasConnected = false;

    while (1)
    {
        bool nowConnected = ble_is_connected();

        if (wasConnected && !nowConnected) {
            ESP_LOGI(TAG, "BLE lost — stopping motors");
            stop_all();
        }
        wasConnected = nowConnected;

        if (ble_cmd_updated) {
            ble_cmd_updated = false;

            ESP_LOGI(TAG, "CMD: %s DUTY: %lu",
                ble_cmd_dir, ble_cmd_duty);

            if (strcmp(ble_cmd_dir, "FWD ") == 0) {
                move_forward(ble_cmd_duty);
            }
            else if (strcmp(ble_cmd_dir, "BACK") == 0) {
                move_reverse(ble_cmd_duty);
            }
            else if (strcmp(ble_cmd_dir, "LEFT") == 0) {
                turn_left(ble_cmd_duty);
            }
            else if (strcmp(ble_cmd_dir, "RGHT") == 0) {
                turn_right(ble_cmd_duty);
            }
            else {
                stop_all();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

#endif

#if 0

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "car_pins.h"

static const char *TAG = "SYNC_DRIVE";

// ==============================
// CONFIG
// ==============================
#define BASE_DUTY 600
#define DUTY_MIN  350
#define DUTY_MAX  1023

#define SYNC_INTERVAL_MS 50

#define KP 3
#define DEADBAND 1
#define FILTER_ALPHA 0.5

#define STARTUP_CYCLES 5
#define MIN_VALID_PULSES 1

// ==============================
// Wheel structure
// ==============================
typedef struct {
    int ch_fwd;
    int ch_rev;
    int hall_pin;

    volatile int pulse_count;
    int last_pulses;

    float filtered_speed;

    int duty;
    bool forward;
} wheel_t;

// Per-wheel base offsets (tune these if needed)
int base_duty_offset[4] = {0, 0, 0, 0};

// Wheels: FL, FR, BL, BR
wheel_t wheels[4] = {
    {0,1, WHEEL_FRONT_LEFT_HALL_A_PIN, 0,0, 0, BASE_DUTY, true},
    {2,3, WHEEL_FRONT_RIGHT_HALL_A_PIN,0,0, 0, BASE_DUTY, false},
    {4,5, WHEEL_BACK_LEFT_HALL_A_PIN,  0,0, 0, BASE_DUTY, true},
    {6,7, WHEEL_BACK_RIGHT_HALL_A_PIN, 0,0, 0, BASE_DUTY, false},
};

// ==============================
// ISR
// ==============================
static void IRAM_ATTR hall_isr(void *arg)
{
    wheel_t *w = (wheel_t *)arg;
    w->pulse_count++;
}

// ==============================
// PWM SETUP
// ==============================
void setup_pwm()
{
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .freq_hz          = 20000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    int pins[8] = {
        WHEEL_FRONT_LEFT_IN1_PIN,
        WHEEL_FRONT_LEFT_IN2_PIN,
        WHEEL_FRONT_RIGHT_IN1_PIN,
        WHEEL_FRONT_RIGHT_IN2_PIN,
        WHEEL_BACK_LEFT_IN1_PIN,
        WHEEL_BACK_LEFT_IN2_PIN,
        WHEEL_BACK_RIGHT_IN1_PIN,
        WHEEL_BACK_RIGHT_IN2_PIN
    };

    for (int i = 0; i < 8; i++) {
        ledc_channel_config_t ch = {
            .gpio_num   = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = i,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
        };
        ledc_channel_config(&ch);
    }
}

// ==============================
// HALL SETUP
// ==============================
void setup_hall()
{
    gpio_install_isr_service(0);

    for (int i = 0; i < 4; i++) {
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_POSEDGE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << wheels[i].hall_pin),
            .pull_up_en = GPIO_PULLUP_ENABLE
        };
        gpio_config(&io_conf);

        gpio_isr_handler_add(wheels[i].hall_pin, hall_isr, &wheels[i]);
    }
}

// ==============================
// APPLY MOTOR OUTPUT
// ==============================
void apply_motor(wheel_t *w)
{
    if (w->forward) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, w->ch_fwd, w->duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, w->ch_fwd);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, w->ch_rev, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, w->ch_rev);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, w->ch_fwd, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, w->ch_fwd);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, w->ch_rev, w->duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, w->ch_rev);
    }
}

// ==============================
// SYNC TASK
// ==============================
void sync_task(void *arg)
{
    int startup_counter = 0;

    while (1)
    {
        int speeds[4];

        // Compute pulse deltas
        for (int i = 0; i < 4; i++) {
            int now = wheels[i].pulse_count;
            speeds[i] = now - wheels[i].last_pulses;
            wheels[i].last_pulses = now;

            // Low-pass filter
            wheels[i].filtered_speed =
                FILTER_ALPHA * speeds[i] +
                (1 - FILTER_ALPHA) * wheels[i].filtered_speed;
        }

        // Startup delay (ignore bad initial data)
        if (startup_counter < STARTUP_CYCLES) {
            startup_counter++;
            vTaskDelay(pdMS_TO_TICKS(SYNC_INTERVAL_MS));
            continue;
        }

        // Compute average (only valid speeds)
        float avg = 0;
        int count = 0;

        for (int i = 0; i < 4; i++) {
            if (speeds[i] >= MIN_VALID_PULSES) {
                avg += wheels[i].filtered_speed;
                count++;
            }
        }

        if (count == 0) {
            vTaskDelay(pdMS_TO_TICKS(SYNC_INTERVAL_MS));
            continue;
        }

        avg /= count;

        // Control update
        for (int i = 0; i < 4; i++) {

            float error = avg - wheels[i].filtered_speed;

            // Deadband
            if (fabs(error) < DEADBAND)
                error = 0;

            // Compute target duty (NO accumulation)
            int target = BASE_DUTY + base_duty_offset[i] + (int)(KP * error);

            // Kickstart if stalled
            if (speeds[i] == 0) {
                target += 150;
            }

            // Clamp
            if (target > DUTY_MAX) target = DUTY_MAX;
            if (target < DUTY_MIN) target = DUTY_MIN;

            wheels[i].duty = target;

            apply_motor(&wheels[i]);
        }

        ESP_LOGI(TAG, "FL:%d FR:%d BL:%d BR:%d",
                 speeds[0], speeds[1], speeds[2], speeds[3]);

        vTaskDelay(pdMS_TO_TICKS(SYNC_INTERVAL_MS));
    }
}

// ==============================
// CONTROL API
// ==============================
void stop_all()
{
    ESP_LOGI(TAG, "STOP");
    for (int i = 0; i < 4; i++) {
        wheels[i].duty = 0;
        apply_motor(&wheels[i]);
    }
}

void move_forward(uint32_t duty)
{
    ESP_LOGI(TAG, "FORWARD");

    wheels[0].forward = true;
    wheels[1].forward = false;
    wheels[2].forward = true;
    wheels[3].forward = false;

    for (int i = 0; i < 4; i++) {
        wheels[i].duty = duty;
    }
}

void move_reverse(uint32_t duty)
{
    ESP_LOGI(TAG, "REVERSE");

    wheels[0].forward = false;
    wheels[1].forward = true;
    wheels[2].forward = false;
    wheels[3].forward = true;

    for (int i = 0; i < 4; i++) {
        wheels[i].duty = duty;
    }
}

void turn_left(uint32_t duty)
{
    ESP_LOGI(TAG, "LEFT");

    wheels[0].forward = true;
    wheels[1].forward = true;
    wheels[2].forward = false;
    wheels[3].forward = false;

    for (int i = 0; i < 4; i++) {
        wheels[i].duty = duty;
    }
}

void turn_right(uint32_t duty)
{
    ESP_LOGI(TAG, "RIGHT");

    wheels[0].forward = false;
    wheels[1].forward = false;
    wheels[2].forward = true;
    wheels[3].forward = true;

    for (int i = 0; i < 4; i++) {
        wheels[i].duty = duty;
    }
}

// ==============================
// MAIN
// ==============================
void app_main(void)
{
    setup_pwm();
    setup_hall();

    xTaskCreate(sync_task, "sync_task", 4096, NULL, 5, NULL);

    while (1)
    {
        move_forward(BASE_DUTY);
        vTaskDelay(pdMS_TO_TICKS(5000));

        stop_all();
        vTaskDelay(pdMS_TO_TICKS(2000));

        move_reverse(BASE_DUTY);
        vTaskDelay(pdMS_TO_TICKS(5000));

        stop_all();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

#endif