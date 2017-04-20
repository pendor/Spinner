#ifndef _BUTTONS_H_
#define _BUTTONS_H_

#include <Arduino.h>

// Offset of buttons on touch board:
#define BTN_UP      0
#define BTN_DOWN    7
#define BTN_OK      1
#define BTN_CANCEL  6
#define BTN_GO_CW   2
#define BTN_GO_CCW  5
#define BTN_STOP1   3
#define BTN_STOP2   4

// Only on the remote, so not part of the touch indexes.
#define BTN_REVERSE 100
#define BTN_RESET   200

// Allow repeat on the up & down buttons only.
#define REPEAT_MASK 0b10000001

// Order to check buttons in more or less
// least destructive order.
uint8_t BUTTON_CHECK_ORDER[] = {
  BTN_STOP1, BTN_STOP2,
  BTN_CANCEL,
  BTN_UP, BTN_DOWN,
  BTN_OK,
  BTN_GO_CW,
  BTN_GO_CCW,
};

#endif /* _BUTTONS_H_ */
