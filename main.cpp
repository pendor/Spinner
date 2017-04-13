#include <Arduino.h>
//#include <EEPROM.h>
#ifndef DEBUG
#include <avr/wdt.h>
#endif
#include <stddef.h>
#include <stdlib.h>

#include "main.h"
#include "pins.h"

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Adafruit_CAP1188.h>

LiquidCrystal_I2C lcd(LCD_I2C, LCD_COLS, LCD_ROWS);
Adafruit_CAP1188 cap = Adafruit_CAP1188(PIN_TOUCH_RESET);

long turns = 0l;

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
  uint8_t touched = checkTouch();
  if(touched >= 0) {
    Serial.print("Touched");
    Serial.print(touched);
    Serial.println();
    
    lcd.setCursor(0, 0);
    lcd.print("Touch: ");
    lcd.print(touched);
  }
}

int checkTouch() {
  uint8_t touched = cap.touched();
  if(touched == 0) {
    // No touch detected
    return -1;
  }
  
  for(uint8_t i=0; i<8; i++) {
    if(touched & (1 << i)) {
      return i;
    }
  }
  return -1;
}