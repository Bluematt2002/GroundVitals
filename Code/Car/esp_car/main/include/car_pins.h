// car_pins.h
// This header file defines the pin mappings for the car's motor control and hall effect sensors.
// The car uses an ESP32-S3 DevKit-1 (WROOM-2) microcontroller, and each motor is driven by a DRV8871 motor driver.

#ifndef CAR_PINS_H
#define CAR_PINS_H

// ========================================= //
//  --- Schematic Lables to Pin Numbers ---  //
// ========================================= //

#define INA1 11  // Changed from 38 - GPIO38 is used by onboard RGB LED on DevKitC-1 v1.1 (WROOM-2 N32R16V)
#define INA2 12  // Changed from 37 - GPIO37 reserved by Octal PSRAM on WROOM-2
#define INA3 13  // Changed from 36 - GPIO36 reserved by Octal PSRAM on WROOM-2
#define INA4 16  // Changed from 35 - GPIO35 reserved by Octal PSRAM on WROOM-2

#define INB1 48
#define INB2 47
#define INB3 21
#define INB4 14

#define O_A1 1
#define O_A2 4
#define O_A3 6
#define O_A4 8

#define O_B1 2
#define O_B2 5
#define O_B3 7
#define O_B4 9

#define TX_0 17
#define RX_0 18

// ========================= //
//  --- Readable Lables ---  //
// ========================= //

// Front Left Wheel
#define WHEEL_FRONT_LEFT_IN1_PIN      INA3
#define WHEEL_FRONT_LEFT_IN2_PIN      INB3
#define WHEEL_FRONT_LEFT_HALL_A_PIN   O_A3
#define WHEEL_FRONT_LEFT_HALL_B_PIN   O_B3

// Front Right Wheel
#define WHEEL_FRONT_RIGHT_IN1_PIN      INA2
#define WHEEL_FRONT_RIGHT_IN2_PIN      INB2
#define WHEEL_FRONT_RIGHT_HALL_A_PIN   O_A2
#define WHEEL_FRONT_RIGHT_HALL_B_PIN   O_B2

// Back Left Wheel
#define WHEEL_BACK_LEFT_IN1_PIN        INA4
#define WHEEL_BACK_LEFT_IN2_PIN        INB4
#define WHEEL_BACK_LEFT_HALL_A_PIN     O_A4
#define WHEEL_BACK_LEFT_HALL_B_PIN     O_B4

// Back Right Wheel
#define WHEEL_BACK_RIGHT_IN1_PIN       INA1
#define WHEEL_BACK_RIGHT_IN2_PIN       INB1
#define WHEEL_BACK_RIGHT_HALL_A_PIN    O_A1
#define WHEEL_BACK_RIGHT_HALL_B_PIN    O_B1

// UART Pins
#define TX_PIN                         TX_0
#define RX_PIN                         RX_0

#endif // CAR_PINS_H