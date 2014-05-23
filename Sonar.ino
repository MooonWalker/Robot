
void initSonar()
{
	pinMode(TRIG_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);	
}

void doSonarping()
{
	long duration, distanceCm;
	// Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
	digitalWrite(TRIG_PIN, LOW);
	delayMicroseconds(2);
	digitalWrite(TRIG_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG_PIN, LOW);
	duration = pulseIn(ECHO_PIN,HIGH);

	// convert the time into a distance
	distanceCm = duration / 29.1 / 2 ;

	if(distanceCm <=5)
	{
		
	}
	else if(distanceCm<=50)
	{
		object=true;
	}
	else
	{
		object=false;
	}

	if (distanceCm <= 0)
	{
		Serial.println("Out of range");
	}
	else
	{
		Serial.print(distanceCm);
		Serial.print("cm");
		Serial.println();
	}

}
