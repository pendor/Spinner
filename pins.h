#ifndef _PINS_H_
#define _PINS_H_

#include <Arduino.h>

// Arduino pin assignments:
// D0   Serial
// D1   Serial
// D2   Hall Sensor
// D3   CAP1188 interrupt
// D4   CAP1188 reset
// D5   
// D6   
// D7   
// D8   Remote1
// D9   Remote2
// D10  Remote3
// D11  Remote4
// D12  
// D13  
//
// A0   
// A1   
// A2   
// A3   
// A4   SDA } I2C feeds screen, motor controller,
// A5   SDC } and button touch sensor.

// Hall sensort
#define PIN_HALL         2

// Interrupt pin (active low) from touch sensor
#define PIN_TOUCH_INT    3

// Reset pin to the touch sensor uC
#define PIN_TOUCH_RESET  4

// Pin assignments for the RF remote.
// Buttons A-D on the remote are 1-4 below.
#define PIN_REMOTE_1    10
#define PIN_REMOTE_2     8
#define PIN_REMOTE_3    11
#define PIN_REMOTE_4     9

// derive the index into the button down array based on
// the button pin # minus the lowest numbered pin.  Assumes
// contiguously mapped buttons & #2 is the lowest.
#define remoteTimeIdx(b) (b - PIN_REMOTE_2)

#define LCD_ROWS  4
#define LCD_COLS 20

#define LCD_I2C   0x27
#define TOUCH_I2C 0x29
#define MOTOR_I2C 0x60

// 1-based port number the motor is plugged into.
#define MOTOR_NUM 4

#endif /* _PINS_H_ */
