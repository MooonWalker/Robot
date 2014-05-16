// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Kalman.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

MPU6050 accelgyro;

float    base_x_accel;
float    base_y_accel;
float    base_z_accel;

float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication


#define LED_PIN 13
int16_t HALF = 127;
int16_t FULL = 255;

bool blinkState = false;



void setup() 
{

	
    // join I2C bus (I2Cdev library doesn't do this automatically)
    

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(19200);
	Wire.begin();
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
	 i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
	 i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
	i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
	i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
	while(i2cWrite(0x19,i2cData,4,false)); // Write to all four registers at once
	while(i2cWrite(0x6B,0x01,true)); // PLL with X axis gyroscope reference and disable sleep mode 
	while(i2cRead(0x75,i2cData,1));
	
	if(i2cData[0] != 0x68) 
	{ // Read "WHO_AM_I" register
		 Serial.print(F("Error reading sensor"));
		 while(1);
	}
 
	 
    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
	
	delay(100);
	
	calibrateSensors();
	
	 while(i2cRead(0x3B,i2cData,6));
	 accX = ((i2cData[0] << 8) | i2cData[1]);
	 accY = ((i2cData[2] << 8) | i2cData[3]);
	 accZ = ((i2cData[4] << 8) | i2cData[5]);
	 
	 accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
	 accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
	 
	kalmanX.setAngle(accXangle); // Set starting angle
	kalmanY.setAngle(accYangle);
	gyroXangle = accXangle;
	gyroYangle = accYangle;
	compAngleX = accXangle;
	compAngleY = accYangle;
	
	timer = micros();
	
	motorInit();
	
	
		
	
}

void loop() 
{
	
	motorsFwd(HALF);
	
	while(i2cRead(0x3B,i2cData,14)); //read mpu6050
	accX = ((i2cData[0] << 8) | i2cData[1]);
	accY = ((i2cData[2] << 8) | i2cData[3]);
	accZ = ((i2cData[4] << 8) | i2cData[5]);
	tempRaw = ((i2cData[6] << 8) | i2cData[7]);
	gyroX = ((i2cData[8] << 8) | i2cData[9]);
	gyroY = ((i2cData[10] << 8) | i2cData[11]);
	gyroZ = ((i2cData[12] << 8) | i2cData[13]);
	
	accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
	accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
	
	 double gyroXrate = ((double)gyroX-base_x_gyro)/131.0;
	 double gyroYrate = (-((double)gyroY-base_y_gyro)/131.0);
	 
	
	 gyroXangle += gyroXrate*((double)(micros()-timer)/1000000); // Calculate gyro angle without any filter
	 gyroYangle += gyroYrate*((double)(micros()-timer)/1000000);
	 
	 //compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(micros()-timer)/1000000)))+(0.07*accXangle); // Calculate the angle using a Complimentary filter
	 //compAngleY = (0.93*(compAngleY+(gyroYrate*(double)(micros()-timer)/1000000)))+(0.07*accYangle);
   //
	 kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
	 kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
	
	timer = micros();
	
	//temp= accelgyro.getTemperature();
	//temp= (temp+12412.0)/340.0;
	temp = ((double)tempRaw + 12412.0) / 340.0;
    // display tab-separated accel/gyro x/y/z values
	
	
	
    Serial.print("a/g/t:\t");
    //Serial.print(compAngleX); Serial.print("\t");
	 Serial.print(kalAngleX); Serial.print("\t");
    Serial.print(accXangle); Serial.print("\t");
	
	//Serial.print(compAngleY); Serial.print("\t");
	Serial.print(kalAngleY); Serial.print("\t");
	Serial.print(accYangle); Serial.print("\t");
   
    //Serial.print(gyroX); Serial.print("\t");
    //Serial.print(gyroY); Serial.print("\t");
    
	Serial.println(temp);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
	
	delay(500);
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

void calibrateSensors()
{
	int                   num_readings = 50;
	float                 x_accel = 0;
	float                 y_accel = 0;
	float                 z_accel = 0;
	float                 x_gyro = 0;
	float                 y_gyro = 0;
	float                 z_gyro = 0;	
	
	while(i2cRead(0x3B,i2cData,14));
	x_accel = ((i2cData[0] << 8) | i2cData[1]);
	y_accel = ((i2cData[2] << 8) | i2cData[3]);
	z_accel = ((i2cData[4] << 8) | i2cData[5]);
	tempRaw = ((i2cData[6] << 8) | i2cData[7]);
	x_gyro = ((i2cData[8] << 8) | i2cData[9]);
	y_gyro = ((i2cData[10] << 8) | i2cData[11]);
	z_gyro = ((i2cData[12] << 8) | i2cData[13]);
	
	x_accel = 0;
	y_accel = 0;
	z_accel = 0;
	x_gyro = 0;
	y_gyro = 0;
	z_gyro = 0;
	
	for(int i=0;i < num_readings;i++) //calibrating
	{
		while(i2cRead(0x3B,i2cData,14));
		x_accel += ((i2cData[0] << 8) | i2cData[1]);
		y_accel += ((i2cData[2] << 8) | i2cData[3]);
		z_accel += ((i2cData[4] << 8) | i2cData[5]);
		tempRaw += ((i2cData[6] << 8) | i2cData[7]);
		x_gyro += ((i2cData[8] << 8) | i2cData[9]);
		y_gyro += ((i2cData[10] << 8) | i2cData[11]);
		z_gyro += ((i2cData[12] << 8) | i2cData[13]);
		
		delay(100);
	}
	
	x_accel /= num_readings;
	y_accel /= num_readings;
	z_accel /= num_readings;
	x_gyro /= num_readings;
	y_gyro /= num_readings;
	z_gyro /= num_readings;
	
	base_x_accel = x_accel;
	base_y_accel = y_accel;
	base_z_accel = z_accel;
	base_x_gyro = x_gyro;
	base_y_gyro = y_gyro;
	base_z_gyro = z_gyro;
	
}


