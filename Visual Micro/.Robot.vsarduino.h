/* 
	Editor: http://www.visualmicro.com
	        arduino debugger, visual micro +, free forum and wiki
	
	Hardware: Arduino Mega 2560 or Mega ADK, Platform=avr, Package=arduino
*/

#define __AVR_ATmega2560__
#define ARDUINO 101
#define ARDUINO_MAIN
#define F_CPU 16000000L
#define __AVR__
#define __cplusplus
extern "C" void __cxa_pure_virtual() {;}

//
//
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
uint8_t i2cWrite(uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop);
uint8_t i2cRead(uint8_t registerAddress, uint8_t* data, uint8_t nbytes);
void calibrateSensors();
void motorInit();
void motorsFwd(int speed);
void motorsStop(bool brake);

#include "C:\Arduino\hardware\arduino\variants\mega\pins_arduino.h" 
#include "C:\Arduino\hardware\arduino\cores\arduino\arduino.h"
#include "C:\DATA\Dropbox\Robot\Robot.ino"
#include "C:\DATA\Dropbox\Robot\Motor.ino"
