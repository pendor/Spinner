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

// Read PINB pins (D8..D13) by direct port access.
// Faster & safer during ISR.
#define digReadPinB(b) (PINB & bit(b - 8))

LiquidCrystal_I2C lcd(LCD_I2C, LCD_COLS, LCD_ROWS);
Adafruit_CAP1188 cap = Adafruit_CAP1188(PIN_TOUCH_RESET);
Adafruit_MotorShield motorShield = Adafruit_MotorShield(MOTOR_I2C);
Adafruit_DCMotor *motor = motorShield.getMotor(4);

// Used to de-bounce the touch panel
long m_lastTouchTime = 0;

uint8_t m_motorSpeed = 100; // 0..255 = 0..100%
int m_motorDirection = RELEASE;

// Updated in the interrupt for the hall switch
long m_turnCount = 0l;

// Updated in the 
bool m_touchChanged = false;

long m_lastDisplayMillis = -1000000;

void setup() {
  lcd.init();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.backlight();
  lcd.print("One Sec...");
  
  Serial.begin(9600);
  
  initTouch();
  initRemote();  
  initHall();
  initMotor();
  
  lcd.clear(0,0);  
}

void loop() {
  if(m_touchChanged) {
    processTouch();
    clearTouchInt();
  }
  
#ifndef RF_INTERRUPTS
  checkRemoteButtons();
#endif
  
  if(millis() - m_lastDisplayMillis > 5000) {
    updateDisplay();
    m_lastDisplayMillis = millis();
  }
}

void updateDisplay() {
  lcd.setCursor(0, 1);
  lcd.print("Turns:");
  lcd.clear(1, 6);
  lcd.setCursor(7, 1);
  lcd.print(m_turnCount);

  lcd.setCursor(0, 2);
  lcd.print("Speed:");
  lcd.clear(2, 6);
  lcd.setCursor(7, 2);
  lcd.print(m_motorSpeed);
  switch(m_motorDirection) {
    case FORWARD:
    lcd.print(" CW");
    lcd.setCursor(9, 2);
    break;
    case BACKWARD:
    lcd.print(" CCW");
    lcd.setCursor(10, 2);
    break;
    default:
    lcd.print(" STOP");
    lcd.setCursor(11, 2);
  }
}

void faster() {
  bumpMotorSpeed(10);
}

void slower() {
  bumpMotorSpeed(-10);
}

void bumpMotorSpeed(int p_bump) {
  int m = m_motorSpeed;
  m = m + p_bump;
  if(m > 255) {
    m = 255;
  } else if(m < 0) {
    m = 0;
  }
  m_motorSpeed = (uint8_t)m;
  motor->setSpeed(m_motorSpeed);
  m_lastDisplayMillis = 0;
};

void setDirection(int p_dir) {
  m_motorDirection = p_dir;
  motor->run(p_dir);
  m_lastDisplayMillis = 0;
}

void motorCcw() {
  setDirection(BACKWARD);
}

void motorCw() {
  setDirection(FORWARD);
}

void motorStop() {
  setDirection(RELEASE);
}

void reverse() {
  if(m_motorDirection == FORWARD) {
    setDirection(BACKWARD);
  } else if(m_motorDirection == BACKWARD) {
    setDirection(FORWARD);
  }
}

void processTouch() {
  // Interrupt line seems to bounce a little...
  if(millis() - m_lastTouchTime < 200) {
    return;
  }
  
  m_lastTouchTime = millis();
  
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

  // FIXME: Menu state machine goes here...
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
  motor->setSpeed(m_motorSpeed);
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

#ifdef RF_INTERRUPTS
/** clear any outstanding interrupts. */
void clearPci() {
  PCIFR |= (1 << digitalPinToPCICRbit(PIN_REMOTE_1));
  PCIFR |= (1 << digitalPinToPCICRbit(PIN_REMOTE_2));
  PCIFR |= (1 << digitalPinToPCICRbit(PIN_REMOTE_3));
  PCIFR |= (1 << digitalPinToPCICRbit(PIN_REMOTE_4));
}

ISR(PCINT0_vect) {
  checkRemoteButtons();
  clearPci();
}
#endif

void checkRemoteButtons() {
  if(digReadPinB(PIN_REMOTE_2)) {
    // FIXME: Restart if held?
    motorStop();
  }
  
  if(digReadPinB(PIN_REMOTE_4)) {
    reverse();
  }
  
  if(digReadPinB(PIN_REMOTE_1)) {
    faster();
  }
  
  if(digReadPinB(PIN_REMOTE_3)) {
    slower();
  }
}

// ISR for the hall switch.  Increment or decrement the turn count.
void intHallSwitch() {
  if(m_motorDirection == FORWARD) {
    m_turnCount++;
  } else if(m_motorDirection == BACKWARD) {
    m_turnCount--;
  } // else turned while not moving?  Ignore...
  m_lastDisplayMillis = 0;
}

// ISR for touch sensor.  Flag the touch so the main loop will deal with it.
void intTouchInput() {
  m_touchChanged = true;
}

// Have to call this after we process the touch or we won't get any more.
void clearTouchInt() {
  m_touchChanged = false;
  byte reg = cap.readRegister(CAP1188_MAIN);
  reg &= 0xFE;
  cap.writeRegister(CAP1188_MAIN, reg);
}
