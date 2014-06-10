#define DEBUG 1
#define LED_PIN 13
#define MISO_PIN 12 //MISO
#define MOSI_PIN 11	//MOSI
//pin D3 INT1

#define  HALF 133  //motor speed slow
#define  FULL 255  //motor speed full

#define BRAKE 1		//motors brake
#define NOBRAKE 0	//motors free run

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#include "Kalman.h"
#include "IncFile1.h"

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanZ;

MPU6050 accelgyro;

float    base_x_accel;
float    base_y_accel;
float    base_z_accel;
float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;
float busvoltage = 0;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle, accZangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro
double compAngleX, compAngleY, compAngleZ; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculate the angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

const int loopPeriod = 55;          // a period of 55ms = a frequency of 25Hz
const int slowPeriod = 2000;		// 2sec
unsigned long timeLoopDelay = 0, slowLoopDelay=0;


boolean blinkState = false;
boolean moving = false;
static boolean object = false;
boolean start = false;
char direction =0;		//where to turn after lookaround

volatile boolean isTriggered=false;

enum STATE {STDBY, FWD, LOOKAROUND, TURNRIGHT, TURNLEFT, BACK};
STATE actualState = STDBY;
STATE prevState = STDBY;

void setup() 
{ 
    Serial.begin(9600);
	Wire.begin();
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
	Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed"); 
    // configure Arduino LED
    pinMode(LED_PIN, OUTPUT);

	calibrateSensors();
	 
	 accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
	 accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
	
	 
	kalmanX.setAngle(accXangle); // Set starting angle
	kalmanY.setAngle(accYangle);
	
	
	gyroXangle = accXangle;
	gyroYangle = accYangle;
	gyroZangle = accZangle;
	
	compAngleX = accXangle;
	compAngleY = accYangle;
	
	timer = micros();
	
	motorInit();
	
}

void loop() 
{	  
	
	switch(actualState)	//state machine
	{
		case STDBY:					//initial state		
		
			if(millis() - slowLoopDelay >= slowPeriod) //	.5Hz loop for pinging
			{			
				slowLoopDelay = millis();
			}	
					
		break;
		
		case LOOKAROUND:
			Serial.println("LOOKAROUND");
			readIMU();
			
		break;
		
		case FWD:
			if(millis() - timeLoopDelay >= loopPeriod) //	25Hz loop for pinging
			{
				timeLoopDelay = millis();
			}
			Serial.println("FWD");
			motorsFwd(HALF);
			if(object) 
			{
				motorsStop(BRAKE);
				actualState = LOOKAROUND;
			}
			start=false;
		break;
		
		case TURNLEFT:
			Serial.println("TURNLEFT");
			//-->FWD
		break;
		
		case TURNRIGHT:
			Serial.println("TURNRIGHT");
			turnRight(90);
			//-->FWD
		break;
		
		case BACK:
			Serial.println("BACK");
		break;		
	}
	
	
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
	
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
	return i2cWrite(registerAddress,&data,1,sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop) {
	Wire.beginTransmission(IMUAddress);
	Wire.write(registerAddress);
	Wire.write(data, length);
	return Wire.endTransmission(sendStop); // Returns 0 on success
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t* data, uint8_t nbytes) {
	uint32_t timeOutTimer;
	Wire.beginTransmission(IMUAddress);
	Wire.write(registerAddress);
	if(Wire.endTransmission(false)) // Don't release the bus
	return 1; // Error in communication
	Wire.requestFrom(IMUAddress, nbytes,(uint8_t)true); // Send a repeated start and then release the bus after reading
	for(uint8_t i = 0; i < nbytes; i++) {
		if(Wire.available())
		data[i] = Wire.read();
		else {
			timeOutTimer = micros();
			while(((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
			if(Wire.available())
			data[i] = Wire.read();
			else
			return 2; // Error in communication
		}
	}
	return 0; // Success
}
