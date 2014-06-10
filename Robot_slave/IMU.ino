

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

void readIMU()
{
	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); 
		
	accXangle = (atan2(ay,az)+PI)*RAD_TO_DEG;
	accYangle = (atan2(ax,az)+PI)*RAD_TO_DEG;
	accZangle = (atan2(ay,ax)+PI)*RAD_TO_DEG;
	
	double gyroXrate = ((double)gx-base_x_gyro)/131.0;
	double gyroYrate = (-((double)gy-base_y_gyro)/131.0);
	double gyroZrate = ((double)gz-base_z_gyro)/1.0323;
	
	gyroXangle += gyroXrate*((double)(micros()-timer)/1000000); // Calculate gyro angle without any filter
	gyroYangle += gyroYrate*((double)(micros()-timer)/1000000);
	gyroZangle += gyroZrate*((double)(micros()-timer)/1000);
	
	//compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(micros()-timer)/1000000)))+(0.07*accXangle); // Calculate the angle using a Complimentary filter
	//compAngleY = (0.93*(compAngleY+(gyroYrate*(double)(micros()-timer)/1000000)))+(0.07*accYangle);
	//
	kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
	kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
	kalAngleZ = kalmanZ.getAngle(gyroZangle, gyroZrate, (double)(micros()-timer)/1000);	
	
	timer = micros();
	
	//read temperature
	temp = ((double)tempRaw + 12412.0) / 340.0;
	// display tab-separated accel/gyro x/y/z values

#ifdef DEBUG	
	Serial.print("a/g/temp:\t");
	//Serial.print(compAngleX); Serial.print("\t");
	Serial.print(kalAngleX); Serial.print("\t");
	Serial.print(accXangle); Serial.print("\t");
	
	//Serial.print(compAngleY); Serial.print("\t");
	Serial.print(kalAngleY); Serial.print("\t");
	Serial.print(accYangle); Serial.print("\t");
	
	
	Serial.print(kalAngleZ	); Serial.print("\t");
	Serial.print(gyroZangle); Serial.print("\t");
	
	//Serial.print(gyroX); Serial.print("\t");
	//Serial.print(gyroY); Serial.print("\t");
	
#endif // _DEBUG
}



