SHELL := /bin/bash
PATH := ./Arduino-Makefile/bin:$(PATH)

# Arduino Make file. Refer to https://github.com/sudar/Arduino-Makefile
ARDMK_DIR = Arduino-Makefile

ARDUINO_DIR = /Applications/Arduino.app/Contents/Java
BOARD_TAG = uno
USER_LIB_PATH = libs

# dot on the end of libs is important.  It tricks the Makefile mess into including the libs
# directory in include which enables #include <Wire/Wire.h> instead of "Wire.h"
ARDUINO_LIBS = LiquidCrystal_I2C/src Adafruit_Motor_v2 Adafruit_CAP1188 Wire SPI EEPROM .

# PRE_BUILD_HOOK = ./gen-revision.sh

include $(ARDMK_DIR)/Arduino.mk

