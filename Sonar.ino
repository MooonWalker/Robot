
void initSonar()
{
	pinMode(TRIG_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);	
}

boolean doSonarping()
{
		duration = sonar.ping();  // Send ping, get ping time in microseconds (uS). 		
		distanceCm = duration / US_ROUNDTRIP_CM; // convert the time into a distance	
	
	
	if(distanceCm <=5)
	{
		
	}
	else if(distanceCm<=40)
	{
		return true;
	}
	else
	{
		return false;
	}

	if (distanceCm <= 0)
	{
		//out of range
	}
	else
	{
		
	}

}

boolean singlePing()
{
	long sumOfDistances=0;
	for (int i=0; i < numberOfPings; i++)
	{
		duration = sonar.ping();  // Send ping, get ping time in microseconds (uS).
		distanceCm = duration / US_ROUNDTRIP_CM; // convert the time into a distance
		sumOfDistances += distanceCm;
		delay(35);
	}
	
	avgDistance=sumOfDistances/numberOfPings;
	
	if(avgDistance <=5)
	{
		
	}
	else if(avgDistance<=40)
	{
		return true;
	}
	else
	{
		return false;
	}

	if (avgDistance <= 0)
	{
		//out of range
	}
	else
	{
		
	}

}
