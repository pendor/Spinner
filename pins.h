#ifndef _PINS_H_
#define _PINS_H_

#include <Arduino.h>

// Definitions for pin assignments

// D0   Serial
// D1   Serial
// D2   Rem1
// D3   Rem2
// D4   Motor
// D5   Rem3
// D6   Rem4
// D7   Motor
// D8   Motor
// D9   Hall Sensor
// D10  
// D11  Motor
// D12  Motor
// D13  Touch_Reset_
//
// A0   
// A1   
// A2   
// A3   
// A4   SDA
// A5   SDC

#define PIN_TOUCH_RESET 13
#define PIN_REMOTE_1 2
#define PIN_REMOTE_2 3
#define PIN_REMOTE_3 5
#define PIN_REMOTE_4 6

#define PIN_HALL 9

// Library resolves several other pins as above, 
// but this is the one that controls our motor
#define PIN_MOTOR 11
#define MOTOR_NUM 1 
// ^^ FIXME: zero-based?

#define LCD_I2C 0x27
#define LCD_ROWS 4
#define LCD_COLS 20

#define TOUCH_I2C 0x29

#endif /* _PINS_H_ */
