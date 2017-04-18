#ifndef _PINS_H_
#define _PINS_H_

#include <Arduino.h>

// Definitions for pin assignments

// FIXME: Need to move Rem1 off D2 so we canuse it for interrupt.
// Tie remote plus interrupt pin of touch sensor to D2 and trigger
// interrupt.

// FIXME: Need the hall sensor on an interrupt.  Gotta get the newer
// motor shield for that.  Grr...

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
// A4   SDA
// A5   SDC

#define PIN_HALL         2
#define PIN_TOUCH_INT    3
#define PIN_TOUCH_RESET  4

#define PIN_REMOTE_1    10
#define PIN_REMOTE_2     8
#define PIN_REMOTE_3    11
#define PIN_REMOTE_4     9

#define LCD_ROWS  4
#define LCD_COLS 20

#define LCD_I2C   0x27
#define TOUCH_I2C 0x29
#define MOTOR_I2C 0x60

#define MOTOR_NUM 4

#endif /* _PINS_H_ */
