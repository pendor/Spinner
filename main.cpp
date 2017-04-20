#include <Arduino.h>
#include <EEPROM.h>

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

byte m_motorSpeed = 100; // 0..255 = 0..100%
byte m_motorDirection = RELEASE;

// Stored before STOP
byte m_lastMotorDirection = FORWARD;

// Updated in the interrupt for the hall switch
long m_turnCount = 0l;
long m_turnGoal = -1l;

// Updated in the 
bool m_touchChanged = false;

long m_lastDisplayMillis = -1000000;

char lineBuffer[LCD_COLS + 1];

// true, up/down sets turn goal, false they set speed.
bool m_setTurnMode = false;

// ms to hold button on remote before we decide it's held down.
#define REMOTE_HOLD_TIME 300

// Ignore subsequence presses in this time.
#define DEBOUNCE_TIME 200

// Array of last button down times to see if remote is holding its button
// (touch sensor thankfully does that on its own...)
long m_remoteDownTime[] = {-1l, -1l, -1l, -1l};

void setup() {
  lcd.init();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.backlight();
  lcd.print("One Sec...");
  
  Serial.begin(9600);
  
  // Load our last goal & speed
  byte speed;
  EEPROM.get(EEPROM_SPEED_ADDR, speed);
  m_motorSpeed = speed;
  
  long turns;
  EEPROM.get(EEPROM_TURN_ADDR, turns);
  if(turns > 1000000) {
    turns = 100;
  }
  m_turnGoal = turns;
  
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
  
  long mils = millis();
  
  if(mils - m_lastDisplayMillis >= 500 && !m_setTurnMode) {
    saveSettings();
  }
  
  if(isDone()) {
    motorStop();
    m_lastDisplayMillis = 0;
  }
  
  if(mils - m_lastDisplayMillis > 5000) {
    updateDisplay();
    m_lastDisplayMillis = millis();
  }
}

void saveSettings() {
  long cur;
  EEPROM.get(EEPROM_TURN_ADDR, cur);
  if(cur != m_turnGoal) {
    EEPROM.put(EEPROM_TURN_ADDR, m_turnGoal);
  }
  
  byte curSpeed;
  EEPROM.get(EEPROM_SPEED_ADDR, curSpeed);
  if(curSpeed != m_motorSpeed) {
    EEPROM.put(EEPROM_SPEED_ADDR, m_motorSpeed);
  }
}

bool isDone() {
  return
    (m_motorDirection == FORWARD && m_turnGoal > 0 && m_turnCount >= m_turnGoal)
      ||
    (m_motorDirection == BACKWARD && m_turnGoal < 0 && m_turnCount <= m_turnGoal)
      || 
    (m_motorDirection == RELEASE && m_turnCount == m_turnGoal)
  ;
}

void resetBuffer() {
  lineBuffer[LCD_COLS] = 0;
  memset(lineBuffer, ' ', LCD_COLS);
}

void updateDisplay() {
  resetBuffer();
  snprintf(lineBuffer, LCD_COLS, "Turns: %5ld /%5ld", m_turnCount, m_turnGoal);  
  lcd.setCursor(0, 1);
  lcd.print(lineBuffer);

  const char *dir;
  switch(m_motorDirection) {
    case FORWARD:
    dir = "CW  ";
    break;
    case BACKWARD:
    dir = "CCW ";
    break;
    default:
    dir = "STOP";
    break;
  }
  
  resetBuffer();
  snprintf(lineBuffer, LCD_COLS, "Speed: %3d :: %s", m_motorSpeed, dir);
  lcd.setCursor(0, 2);
  lcd.print(lineBuffer);
  
  lcd.setCursor(0, 3);
  if(isDone()) {
    lcd.print("DONE!");
  } else {
    lcd.print("     ");
  }
  
  lcd.setCursor(LCD_COLS-9, 3);
  if(m_setTurnMode) {
    lcd.print("Mode: SET");
  } else {
    lcd.print("Mode: RUN");
  }
}

/** Handle a button push based on state machine mode. */
void button(int p_button, bool p_isHeld) {
  // FIXME: Menu state machine goes here...
  m_lastDisplayMillis = 0;
  switch(p_button) {
  case BTN_STOP1:
  case BTN_STOP2:
    if(m_setTurnMode) {
      m_setTurnMode = false;
    } else {
      if(p_isHeld) {
        // Hold button to resume.
        setDirection(m_lastMotorDirection);
      } else {
        motorStop();
      }
    }
    break;
    
  case BTN_CANCEL:
    m_setTurnMode = false;
    motorStop();
    break;
    
  case BTN_OK:
    m_setTurnMode = !m_setTurnMode;
    if(m_setTurnMode) {
      motorStop();
    }
    break;
  
  case BTN_UP:
    if(m_setTurnMode) {
      m_turnGoal += 10;
    } else {
      faster();
    }
    break;
    
  case BTN_DOWN:
    if(m_setTurnMode) {
      m_turnGoal -= 10;
    } else {
      slower();
    }
    break;
  
  case BTN_GO_CW:
    if(!m_setTurnMode) {
      motorCw();
    }
    break;
  
  case BTN_GO_CCW:
    if(!m_setTurnMode) {
      motorCcw();
    }
    break;
  case BTN_RESET:
    m_turnCount = 0;
    break;
    
  default:
    break;
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
  // Save last moving direction for a restart.
  if(p_dir == RELEASE && m_motorDirection != RELEASE) {
    m_lastMotorDirection = m_motorDirection;
  }
  
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
  if(millis() - m_lastTouchTime < DEBOUNCE_TIME) {
    return;
  }
  
  m_lastTouchTime = millis();
  
  int8_t touchTranslated = -1;
  int8_t touched = cap.touched();
  if(touched == 0) {
    // No touch detected
    return;
  }
  
  // If both stop & reset held, reset the counter.
  if(
    (touched & (1 << BTN_STOP1) || touched & (1 << BTN_STOP2))
      &&
        (touched & (1 << BTN_CANCEL))
  ) {
    button(BTN_RESET, false);
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

  button(touchTranslated, false);
}

byte buttonForRemotePin(byte p_btnPin) {
  switch(p_btnPin) {
  case PIN_REMOTE_2:
    return BTN_STOP1;
  case PIN_REMOTE_4:
    return BTN_REVERSE;
  case PIN_REMOTE_1:
    return BTN_UP;
  case PIN_REMOTE_3:
    return BTN_DOWN;
    
  default: // Just in case...
    return BTN_STOP1;
  }
}

/** @return 0 = not pushed, 1 = down, 2 = repeat. */
void checkRemoteHold(byte p_btnPin) {
  long downTime = m_remoteDownTime[remoteTimeIdx(p_btnPin)];
  if(digReadPinB(p_btnPin)) {
    // Button is down now
    if(downTime >= 0) {
      // It was down last time we looked too.  
      // See if it's long enough to send a "held".
      if(millis() - downTime >= REMOTE_HOLD_TIME) {
        button(buttonForRemotePin(p_btnPin), true);
      }
      // else keep waiting...
    } else if(millis() + downTime > DEBOUNCE_TIME) {
      // downTime is negative which means it wasn't down last time we looked.
      // We'll only trigger if it's been long enough since it was last released
      // to not be a bounce.
      button(buttonForRemotePin(p_btnPin), false);
      m_remoteDownTime[remoteTimeIdx(p_btnPin)] = millis();
    }
  } else if(downTime > 0) {
    // Button isn't down.  If downTime is positive, it WAS down so 
    // store the time we noticed it up as a negative for later debounce
    // checking.
    m_remoteDownTime[remoteTimeIdx(p_btnPin)] = 0l - millis();
  }
}

void checkRemoteButtons() {
  checkRemoteHold(PIN_REMOTE_2);
  checkRemoteHold(PIN_REMOTE_4);
  checkRemoteHold(PIN_REMOTE_1);
  checkRemoteHold(PIN_REMOTE_3);
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
 
  // Only repeat on up/down buttons
  cap.writeRegister(CAP1188_REPEAT_ENABLE, REPEAT_MASK);
  
  // Slow interrupt repeat time.  Bits 3..0 have the repeat rate.
  byte rep = cap.readRegister(CAP1188_REPEAT_RATE);
  rep &= 0b11110000;
  rep |= 0b00001001; // 1001 = 350ms
  cap.writeRegister(CAP1188_REPEAT_RATE, rep);
 
  // Read current sensitivity register, zero out the bits we want, 
  // then add back our desired sensitivity setting.
  int8_t sens = cap.readRegister(CAP1188_SENSITIVITY);
  // Bits 6..4 contain sensitivity.  Rest should be left alone.
  sens &= 0b10001111;
  sens |= 0b00110000;
  cap.writeRegister(CAP1188_SENSITIVITY, sens);
  
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
