#ifndef _MENU_H_
#define _MENU_H_

#include <Arduino.h>
#include "pins.h"
/*
REQUIRED GLOBAL VARIABLES & DEFINITIONS
*/
#define MOVECURSOR 1  // constants for indicating whether cursor should be redrawn
#define MOVELIST 2  // constants for indicating whether cursor should be redrawn

byte totalRows = LCD_ROWS;  // total rows of LCD
byte totalCols = LCD_COLS;  // total columns of LCD

unsigned long timeoutTime = 0;  // this is set and compared to millis to see when the user last did something.
const int menuTimeout = 10000; // time to timeout in a menu when user doesn't do anything.

extern void basicMenu();

#endif /* _MENU_H_ */
