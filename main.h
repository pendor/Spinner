#ifndef _MAIN_H_
#define _MAIN_H_

#include <Arduino.h>

void slower();
void faster();
void bumpMotorSpeed(int p_bump);
void setDirection(int p_dir);
void motorCcw();
void motorCw();
void motorStop();
void reverse();
void processTouch();
void checkRemoteButtons();

void intTouchInput();
void intHallSwitch();
void clearPci();

void initTouch();
void initRemote();
void initHall();
void initMotor();
void clearTouchInt();

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
  void loop();
  void setup();
#ifdef __cplusplus
} // extern "C"
#endif

#endif /* _MAIN_H_ */
