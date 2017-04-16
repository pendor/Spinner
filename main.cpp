#include <Arduino.h>
//#include <EEPROM.h>

#include <stddef.h>
#include <stdlib.h>

#include "main.h"
#include "pins.h"
#include "buttons.h"

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Adafruit_CAP1188.h>
#include <Adafruit_MotorShield.h>

LiquidCrystal_I2C lcd(LCD_I2C, LCD_COLS, LCD_ROWS);
Adafruit_CAP1188 cap = Adafruit_CAP1188(PIN_TOUCH_RESET);
Adafruit_MotorShield motorShield = Adafruit_MotorShield(MOTOR_I2C);
Adafruit_DCMotor *motor = motorShield.getMotor(4);

uint8_t motor_speed = 100; // 0..255 = 0..100%
int direction = RELEASE;

long turns = 0l;

void bumpMotorSpeed(int p_bump) {
  int m = motor_speed;
  m = m + p_bump;
  if(m > 255) {
    m = 255;
  } else if(m < 0) {
    m = 0;
  }
  motor_speed = (uint8_t)m;
  motor->setSpeed(motor_speed);
};

void setDirection(int p_dir) {
  direction = p_dir;
  motor->run(p_dir);
}

void reverse() {
  if(direction == FORWARD) {
    setDirection(BACKWARD);
  } else if(direction == BACKWARD) {
    setDirection(FORWARD);
  }
}

void setup() {
  lcd.init();                      // initialize the lcd 
 
  // Print a message to the LCD.
  lcd.backlight();
  // lcd.print("Hello, world!");
  
  
  // lcd.setCursor ( 0, 1 );
//lcd.print("   IIC/I2C LCD2004  ");
  
  if(!cap.begin(TOUCH_I2C)) {
    Serial.println("CAP1188 not found");
    while (1);
  }
  Serial.println("CAP1188 found!");
  
  pinMode(PIN_REMOTE_1, INPUT);
  pinMode(PIN_REMOTE_2, INPUT);
  pinMode(PIN_REMOTE_3, INPUT);
  pinMode(PIN_REMOTE_4, INPUT);
  
  pinMode(PIN_HALL, INPUT);
  
  motorShield.begin();  // create with the default frequency 1.6KHz
  motor->setSpeed(motor_speed);
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

void faster() {
  lcd.setCursor(0, 0);
  bumpMotorSpeed(10);
  lcd.print("SP+ = ");
  lcd.print(motor_speed);
  lcd.print("    ");
}

void slower() {
  lcd.setCursor(0, 0);
  bumpMotorSpeed(-10);
  lcd.print("SP- = ");
  lcd.print(motor_speed);
  lcd.print("    ");
}

void loop() {
  int touched = checkTouch();
  lcd.setCursor(0, 0);  
  
  switch(touched) {
    case BTN_STOP1:
    case BTN_STOP2:
    case BTN_CANCEL:
    case BTN_OK:
      setDirection(RELEASE);
      break;
    
    case BTN_UP:
      faster();
      break;
      
    case BTN_DOWN:
      slower();
      break;
    
    case BTN_GO_CW:
      setDirection(FORWARD);
      break;
    
    case BTN_GO_CCW:
      setDirection(BACKWARD);
      break;
    default:
      break;
  }
  
  // Remote buttons:
  // Faster, Slower, Reverse, Stop
  if(digitalRead(PIN_REMOTE_2) == HIGH) {
    setDirection(RELEASE);
  }
  
  if(digitalRead(PIN_REMOTE_4) == HIGH) {
    reverse();
  }
  
  if(digitalRead(PIN_REMOTE_1) == HIGH) {
    faster();
  }
  
  if(digitalRead(PIN_REMOTE_3) == HIGH) {
    slower();
  }
  
  lcd.setCursor(0, 2);
  if(digitalRead(PIN_HALL) == HIGH) {
    lcd.print("HALL                ");
  } else {
    lcd.print("                    ");
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
