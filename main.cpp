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
#include "cap1188.h"
#include <Adafruit_MotorShield.h>

// #define RF_INTERRUPTS

LiquidCrystal_I2C lcd(LCD_I2C, LCD_COLS, LCD_ROWS);
Adafruit_CAP1188 cap = Adafruit_CAP1188(PIN_TOUCH_RESET);
Adafruit_MotorShield motorShield = Adafruit_MotorShield(MOTOR_I2C);
Adafruit_DCMotor *motor = motorShield.getMotor(4);

uint8_t motor_speed = 100; // 0..255 = 0..100%
int direction = RELEASE;
long turns = 0l;
long lastTurn = -1l;
bool touchChanged = false;

long lastCommandTime = 0;
const char *lastCommand = "";

void setup() {
  lcd.init();
  lcd.setCursor(0,0);
  lcd.clear();
  lcd.backlight();
  lcd.print("One Sec...");
  
  Serial.begin(9600);
  
  initTouch();
  initRemote();  
  initHall();
  initMotor();
  
  lcd.clear(0,0);  
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
  if(touchChanged) {
    processTouch();
    clearTouchInt();
  }
  
#ifndef RF_INTERRUPTS
  checkRemoteButtons();
#endif
  
  if(millis() - lastCommandTime < 100) {
    lcd.setCursor(0,2);
    lcd.print("Last:");
    lcd.clear(2, 5);
    lcd.setCursor(6, 2);
    lcd.print(lastCommand);
  } else if(millis() - lastCommandTime > 2000 && millis() - lastCommandTime < 2100) {
    lcd.clear(2, 0);
  }
  
  if(lastTurn != turns) {
    lcd.setCursor(0, 1);
    lcd.print("Turns:");
    lcd.clear(1, 6);
    lcd.setCursor(7, 1);
    lcd.print(turns);
    
    lastTurn = turns;
  }
}

void faster() {
  lcd.setCursor(0, 0);
  bumpMotorSpeed(10);
  lcd.print("SP+ = ");
  lcd.print(motor_speed);
  lcd.print("    ");

  lastCommand = "faster";
  lastCommandTime = millis();
}

void slower() {
  lcd.setCursor(0, 0);
  bumpMotorSpeed(-10);
  lcd.print("SP- = ");
  lcd.print(motor_speed);
  lcd.print("    ");
  
  lastCommand = "slower";
  lastCommandTime = millis();
}

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

void motorCcw() {
  setDirection(BACKWARD);
  lastCommand = "backward";
  lastCommandTime = millis();
}

void motorCw() {
  setDirection(FORWARD);
  lastCommand = "forward";
  lastCommandTime = millis();
}

void motorStop() {
  setDirection(RELEASE);
  lastCommand = "stop";
  lastCommandTime = millis();
}

void reverse() {
  if(direction == FORWARD) {
    setDirection(BACKWARD);
  } else if(direction == BACKWARD) {
    setDirection(FORWARD);
  }
  lastCommand = "reverse";
  lastCommandTime = millis();
}

void processTouch() {
  // Interrupt line seems to bounce a little...
  if(millis() - lastCommandTime < 200) {
    return;
  }
  
  int8_t touchTranslated = -1;
  int8_t touched = cap.touched();
  if(touched == 0) {
    // No touch detected
    return;
  }
  
  for(uint8_t i=0; i<8; i++) {
    if(touched & (1 << BUTTON_CHECK_ORDER[i])) {
      touchTranslated = BUTTON_CHECK_ORDER[i];
      break;
    }
  }

  if(touchTranslated < 0) {
    // Should be can't happen.  Means we didn't have a button mapped to this index.
    return;
  }

  switch(touchTranslated) {
    case BTN_STOP1:
    case BTN_STOP2:
    case BTN_CANCEL:
    case BTN_OK:
      motorStop();
      break;
    
    case BTN_UP:
      faster();
      break;
      
    case BTN_DOWN:
      slower();
      break;
    
    case BTN_GO_CW:
      motorCw();
      break;
    
    case BTN_GO_CCW:
      motorCcw();
      break;
    default:
      break;
  }
}

void initTouch() {
  pinMode(PIN_TOUCH_INT, INPUT_PULLUP);
  
  if(!cap.begin(TOUCH_I2C)) {
    Serial.println("CAP1188 not found");
    lcd.setCursor(0,0);
    lcd.print("No Touch Sensor.");
    while (1);
  }
  
  // Enable interrupts
  cap.writeRegister(CAP1188_INTERRUPT, 0xff);
 
  // No multi-touch allowed.
  cap.writeRegister(CAP1188_MTBLK, 0x80);
  
  // Only repeat on up/down buttons
  cap.writeRegister(CAP1188_REPEAT_ENABLE, REPEAT_MASK);
  
  // Slow interrupt repeat time.
  byte rep = cap.readRegister(CAP1188_REPEAT_RATE);
  rep |= 0x0f;
  cap.writeRegister(CAP1188_REPEAT_RATE, rep);
 /* 
  // Read current sensitivity register, zero out the bits we want, 
  // then add back our desired sensitivity setting.
  int8_t sens = cap.readRegister(CAP1188_SENSITIVITY);
  // Bits 6..4 contain sensitivity.  Rest should be left alone.
  sens &= 0x8F;
  // Values are 7..0 for least to most sensitive.  
  // Shift over 4 bits & add in.
  sens += (5 << 4); 
  cap.writeRegister(CAP1188_SENSITIVITY, sens);
   */ 
  // We might want the LED's for testing, but no sense having them eat current
  // sealed in an opaque box...
  cap.writeRegister(CAP1188_LEDLINK, 0);  
  
  // Force recalibration of all inputs.
  //cap.writeRegister(CAP1188_CALIBRATE, 0xff);

  attachInterrupt(digitalPinToInterrupt(PIN_TOUCH_INT), intTouchInput, FALLING);
}

void initHall() {
  pinMode(PIN_HALL, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), intHallSwitch, RISING);
}

void initMotor() {
  motorShield.begin();
  motor->setSpeed(motor_speed);
}

void initRemote() {
  pinMode(PIN_REMOTE_1, INPUT);
  pinMode(PIN_REMOTE_2, INPUT);
  pinMode(PIN_REMOTE_3, INPUT);
  pinMode(PIN_REMOTE_4, INPUT);
  
#ifdef RF_INTERRUPTS
  noInterrupts();
  
  // enable pin interrupts
  *digitalPinToPCMSK(PIN_REMOTE_1) |= (1 << digitalPinToPCMSKbit(PIN_REMOTE_1));
  *digitalPinToPCMSK(PIN_REMOTE_1) |= (1 << digitalPinToPCMSKbit(PIN_REMOTE_2));
  *digitalPinToPCMSK(PIN_REMOTE_1) |= (1 << digitalPinToPCMSKbit(PIN_REMOTE_3));
  *digitalPinToPCMSK(PIN_REMOTE_1) |= (1 << digitalPinToPCMSKbit(PIN_REMOTE_4));

  // Enable global interrupts
  *digitalPinToPCICR(PIN_REMOTE_1) |= (1<<digitalPinToPCICRbit(PIN_REMOTE_1));
  *digitalPinToPCICR(PIN_REMOTE_2) |= (1<<digitalPinToPCICRbit(PIN_REMOTE_2));
  *digitalPinToPCICR(PIN_REMOTE_3) |= (1<<digitalPinToPCICRbit(PIN_REMOTE_3));
  *digitalPinToPCICR(PIN_REMOTE_4) |= (1<<digitalPinToPCICRbit(PIN_REMOTE_4));

  clearPci();
  
  // enable interrupt for the group 
  PCICR |= (1 << digitalPinToPCICRbit(PIN_REMOTE_1));
  PCICR |= (1 << digitalPinToPCICRbit(PIN_REMOTE_2));
  PCICR |= (1 << digitalPinToPCICRbit(PIN_REMOTE_3));
  PCICR |= (1 << digitalPinToPCICRbit(PIN_REMOTE_4));

  interrupts();
#endif
}

/** clear any outstanding interrupts. */
void clearPci() {
#ifdef RF_INTERRUPTS
  PCIFR |= (1 << digitalPinToPCICRbit(PIN_REMOTE_1));
  PCIFR |= (1 << digitalPinToPCICRbit(PIN_REMOTE_2));
  PCIFR |= (1 << digitalPinToPCICRbit(PIN_REMOTE_3));
  PCIFR |= (1 << digitalPinToPCICRbit(PIN_REMOTE_4));
#endif
}

#ifdef RF_INTERRUPTS
ISR(PCINT0_vect) {
  Serial.println("B");
  checkRemoteButtons();
  clearPci();
}
#endif

// Read PINB pins (D8..D13) by direct port access.
// Faster & safer during ISR.
#define digReadPinB(b) (PINB & bit(b - 8))

void checkRemoteButtons() {
  // digitalRead(PIN_REMOTE_2)
  if(digReadPinB(PIN_REMOTE_2)) {
    // FIXME: Restart if held?
    Serial.println("RStop");
    motorStop();
  }
  
  // digitalRead(PIN_REMOTE_4)
  if(digReadPinB(PIN_REMOTE_4)) {
    Serial.println("RRev");
    reverse();
  }
  
  // digitalRead(PIN_REMOTE_1)
  if(digReadPinB(PIN_REMOTE_1)) {
    Serial.println("R+");
    faster();
  }
  
  //if(digitalRead(PIN_REMOTE_3) == HIGH) {
  if(digReadPinB(PIN_REMOTE_3)) {
    Serial.println("R-");
    slower();
  }
}

void intHallSwitch() {
  if(direction == FORWARD) {
    turns++;
  } else if(direction == BACKWARD) {
    turns--;
  } // else turned while not moving?  Ignore...
}

void intTouchInput() {
  touchChanged = true;
}

void clearTouchInt() {
  touchChanged = false;
  byte reg = cap.readRegister(CAP1188_MAIN);
  reg &= 0xFE;
  cap.writeRegister(CAP1188_MAIN, reg);
}

