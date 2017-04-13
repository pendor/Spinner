#include <Arduino.h>
//#include <EEPROM.h>
#ifndef DEBUG
#include <avr/wdt.h>
#endif
#include <stddef.h>
#include <stdlib.h>

#include "main.h"
#include "pins.h"
#include "buttons.h"

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Adafruit_CAP1188.h>
#include <AFMotor.h>

LiquidCrystal_I2C lcd(LCD_I2C, LCD_COLS, LCD_ROWS);
Adafruit_CAP1188 cap = Adafruit_CAP1188(PIN_TOUCH_RESET);
AF_DCMotor motor(1, MOTOR12_64KHZ); // create motor #1, 64KHz pwm

uint8_t motor_speed = 100; // 0..255 = 0..100%

long turns = 0l;

uint8_t bumpMotorSpeed(int8_t p_bump) {
  if(p_bump > 0) {
    if(((int)motor_speed + (int)p_bump) > 255) {
      motor_speed = 255;
    } else {
      motor_speed = motor_speed + p_bump;
    }
  } else if(p_bump < 0){
    if(((int)motor_speed - (int)p_bump) < 0) {
      motor_speed = 0;
    } else {
      motor_speed = motor_speed - p_bump;
    }
  } 
  return motor_speed;
};

void setup() {
  lcd.init();                      // initialize the lcd 
 
  // Print a message to the LCD.
  lcd.backlight();
  lcd.print("Hello, world!");
  
  lcd.setCursor ( 0, 0 );            // go to the top left corner
  lcd.print("    Hello,world!    "); // write this string on the top row
  lcd.setCursor ( 0, 1 );            // go to the 2nd row
  lcd.print("   IIC/I2C LCD2004  "); // pad string with spaces for centering
  lcd.setCursor ( 0, 2 );            // go to the third row
  lcd.print("  20 cols, 4 rows   "); // pad with spaces for centering
  lcd.setCursor ( 0, 3 );            // go to the fourth row
  lcd.print(" www.sunfounder.com ");
  
  if(!cap.begin(TOUCH_I2C)) {
    Serial.println("CAP1188 not found");
    while (1);
  }
  Serial.println("CAP1188 found!");
  
  motor.setSpeed(motor_speed);
}
/*
  TODO: Move pins around so hall sensor on D2, toggle for all the inputs to D3.

https://www.arduino.cc/en/Reference/attachInterrupt
https://learn.adafruit.com/multi-tasking-the-arduino-part-2/external-interrupts?gclid=CI3j4NL2odMCFVq4wAod1xULoA
Add interrupt handler for all button pushes and separate one for the hall.
LOW to trigger the interrupt whenever the pin is low,
CHANGE to trigger the interrupt whenever the pin changes value
RISING to trigger when the pin goes from low to high,
FALLING for when the pin goes from high to low.

attachInterrupt(digitalPinToInterrupt(pin), ISR, mode);

Need menu handler routines.

 * Set spin count
 * Reset count
 * Set speed %

Auto slow down as we get near the end.

*/

void loop() {
  int touched = checkTouch();
  lcd.setCursor(0, 0);
  
  switch(touched) {
    case BTN_STOP1:
    case BTN_STOP2:
    case BTN_CANCEL:
    case BTN_OK:
      lcd.print("STOP                ");
      motor.run(RELEASE);
      break;
    
    case BTN_UP:
      motor.setSpeed(bumpMotorSpeed(10));
      lcd.print("SP+ = ");
      lcd.print(motor_speed);
      lcd.print("    ");
      break;
      
    case BTN_DOWN:
      motor.setSpeed(bumpMotorSpeed(-10));
      lcd.print("SP- = ");
      lcd.print(motor_speed);
      lcd.print("    ");
      break;
    
    case BTN_GO_CW:
      lcd.print("GO CW               ");
      motor.run(FORWARD);
      break;
    
    case BTN_GO_CCW:
      lcd.print("GO CCW              ");
      motor.run(BACKWARD);
      break;
    default:
      lcd.print("                    ");
      break;
  }
}

/** Check which button is touched.
 * FIXME: Board allows multi-touch, but buttons are checked in
 * index order.  We should check buttons in priority to make sure
 * STOP wins.
 */ 
int checkTouch() {
  int8_t touched = cap.touched();
  if(touched == 0) {
    // No touch detected
    return -1;
  }
  
  for(uint8_t i=0; i<8; i++) {
    if(touched & (1 << BUTTON_CHECK_ORDER[i])) {
      return BUTTON_CHECK_ORDER[i];
    }
  }
  return -1;
}
