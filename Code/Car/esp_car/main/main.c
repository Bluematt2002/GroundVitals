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
    set_motor(0, 1, duty, true);  // FL
    set_motor(2, 3, duty, false);  // FR
    set_motor(4, 5, duty, false); // BL
    set_motor(6, 7, duty, true); // BR
}

void turn_right(uint32_t duty)
{
    ESP_LOGI(TAG, "SHIFT RIGHT");
    set_motor(0, 1, duty, true); // FL
    set_motor(2, 3, duty, false); // FR
    set_motor(4, 5, duty, false);  // BL
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
#define DUTY_MIN  300
#define DUTY_MAX  1023

#define SYNC_INTERVAL_MS 200
#define KP 2   // proportional gain (tune this)

// ==============================
// Wheel structure
// ==============================
typedef struct {
    int ch_fwd;
    int ch_rev;
    int hall_pin;

    volatile int pulse_count;
    int last_pulses;

    int duty;
    bool forward;
} wheel_t;

// Wheels: FL, FR, BL, BR
wheel_t wheels[4] = {
    {0,1, WHEEL_FRONT_LEFT_HALL_A_PIN, 0,0, BASE_DUTY, true},
    {2,3, WHEEL_FRONT_RIGHT_HALL_A_PIN,0,0, BASE_DUTY, false},
    {4,5, WHEEL_BACK_LEFT_HALL_A_PIN,  0,0, BASE_DUTY, true},
    {6,7, WHEEL_BACK_RIGHT_HALL_A_PIN, 0,0, BASE_DUTY, false},
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
// SYNC TASK (closed-loop control)
// ==============================
void sync_task(void *arg)
{
    while (1)
    {
        int speeds[4];

        // compute delta pulses
        for (int i = 0; i < 4; i++) {
            int now = wheels[i].pulse_count;
            speeds[i] = now - wheels[i].last_pulses;
            wheels[i].last_pulses = now;
        }

        // compute average speed
        int avg = (speeds[0] + speeds[1] + speeds[2] + speeds[3]) / 4;

        // adjust each wheel
        for (int i = 0; i < 4; i++) {
            int error = avg - speeds[i];

            wheels[i].duty += KP * error;

            if (wheels[i].duty > DUTY_MAX) wheels[i].duty = DUTY_MAX;
            if (wheels[i].duty < DUTY_MIN) wheels[i].duty = DUTY_MIN;

            apply_motor(&wheels[i]);
        }

        ESP_LOGI(TAG, "Speeds FL:%d FR:%d BL:%d BR:%d",
                 speeds[0], speeds[1], speeds[2], speeds[3]);

        vTaskDelay(pdMS_TO_TICKS(SYNC_INTERVAL_MS));
    }
}

// ==============================
// YOUR ORIGINAL API (now synced)
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

    wheels[0].forward = true;   // FL
    wheels[1].forward = false;  // FR
    wheels[2].forward = true;   // BL
    wheels[3].forward = false;  // BR

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

        turn_left(BASE_DUTY);
        vTaskDelay(pdMS_TO_TICKS(5000));

        stop_all();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

#endif