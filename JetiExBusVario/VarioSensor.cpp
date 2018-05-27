/* 
  Jeti Sensor EX Telemetry C++ Library
  
  JetiExSerial - EX serial output implementation
  --------------------------------------------------------------------
  
  Copyright (C) 2015 Bernd Wokoeck
  
  Version history:
  1.00   11/22/2015  created
  
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation
  the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  IN THE SOFTWARE.

**************************************************************/

#include "VarioSensor.h"



  /*
  jetiEx.SetSensorValue( ID_VOLTAGE,   81 );  // 8.1 volt
  jetiEx.SetSensorValue( ID_ALTITUDE, 250 );  // 250 m
  jetiEx.SetSensorValue( ID_TEMP,     300 );  // 300 degrees celsius
  jetiEx.SetSensorValue( ID_CLIMB,    153 );  // 1.53 m/s
  jetiEx.SetSensorValue( ID_FUEL,     80 );   // 80 %
  jetiEx.SetSensorValue( ID_RPM,      5000 ); // 5000/min
  */
MS5611 sensor;


VarioSensor::VarioSensor()
{

}

void VarioSensor::setup(long normpress) {
#ifdef SENSOR_MS5611
    Wire2.begin(I2C_MASTER, 0x00, I2C_PINS_3_4, I2C_PULLUP_EXT, I2C_RATE_400);
#else
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_47_48, I2C_PULLUP_EXT, I2C_RATE_400);
#endif
    sensor.begin();
	sensor.InitializeSampleStateMachine();
	delay(20);
	sensor.readPressureTemperature(&airPressure, &airTemperature);
	airTemperature0 = 0;
	airPressure0 = 0;
	timeSample = micros() + SAMPLING_PERIOD * 1e6;
	airTemperature0 = 0;
	airPressure0 = 0;
	timeSample = micros() + SAMPLING_PERIOD * 1e6;
	for (int i = 0;  i < 200; i++) { //new pressure value every second time of
		while (long(timeSample - micros()) > 0); //state machine
		timeSample += SAMPLING_PERIOD * 1e6;
		if (sensor.readPressureTemperature(&airPressure, &airTemperature) ) {
			//Serial.printf("P = %8.1f  T = %8.1f\r\n", airPressure, airTemperature);
			airTemperature0 += airTemperature;
			airPressure0 += airPressure;
		}
	}
	airTemperature0 /= 100.0;
	airPressure0 /= 100.0;
	timeSample = micros() + SAMPLING_PERIOD * 1e6;
	previouspress = airPressure0;
	alt = airTemperature0 / 0.0065 * (1 - pow(airPressure0/normpress, 0.1903));
	oldalt = alt;
//	memset(altitude, alt, (SAMPLES_ALTITUDE + 1) * sizeof(float)); // fill altitude sample buffer
//	memset(pressure, airPressure0, sizeof(pressure));
	for (int i = 0;  i < SAMPLES_ALTITUDE; i++) {
		pressure[i] = airPressure0;
		altitude[i] = alt;
	}

	timeSample = micros() + SAMPLING_PERIOD * 1e6;
	timeInterval = 0;
}

long VarioSensor::GetVoltage()
{

  volt = 54;

  return volt;
}

long VarioSensor::GetAltitude()
{

  return long(alt*100.0);
}
float VarioSensor::GetPressure()
{
	return long(pascal);
}

long VarioSensor::GetTemp()
{
  return long(temp-273.15); //in degrees Celsius
}

bool VarioSensor::GetClimb(long* climb, long smoothing, long normpress)
{

	if (long(timeSample - micros()) < 0 ) { //non-blocking
		//this loop is sometimes not repeated within less than SAMPLING_PERIOD
		//(caused by the DoJetiSend when it is active every 150 ms), so that
		//micros() - timeSample > SAMPLING_PERIOD. Therefore, the time between two pressure
		//measurements are not always just 2 * SAMPLING_PERIOD (factor two because of state machine)
		//timeInterval adds the real time spent between two calls and uses it for calculating the
		//climb velocity
		//As of 22.11.2017 this is obsolete. The Serial.flush command introduced by me
		//waits for tx buffer send ready and blocks the serial interrupt loop
		//this is not needed with the modified serial.c according to JetiExProtocol readme_teensy.
		//Serial.printf ("elapsed time (us) %d \n\r", long(SAMPLING_PERIOD*1e6 + micros() -timeSample));
		timeInterval += micros() - timeSample + SAMPLING_PERIOD * 1e6;
		//Serial.printf ("time interval (us) %d \n\r", long(timeInterval));
		//The next statement makes sure that the next cycle starts SAMPLING_PERIOD *1E6
		//microseconds later. If timeSample-micros() is much larger than SAMPLING-PERIOD *1e6
		//(i.e. the if statement was called late), simply adding the cycle time to timeSample
		//could mean that the if statement is called in less than the cycle time again, causing
		// reading of the sensor before the conversion is ready
		timeSample = micros() + SAMPLING_PERIOD * 1e6; //better if overrun

		if (sensor.readPressureTemperature(&press, &temp)) {
			//if true: new pressure value available, false: temperature cycle of state machine
			//always true for BMP280
			//Serial.printf("Pressure %8.2f  \n\r", press );
#ifdef LOWPASSFILTER
			press = FILTERX * previouspress + FILTERY * press;
			previouspress = press;
#endif

			//pressure[SAMPLES_ALTITUDE] = press;
			pressure[smoothing] = press;
			//altitude[SAMPLES_ALTITUDE] =  temp / 0.0065 * (1 - pow(press / airPressure0, 0.1903)); //in cm
			//altitude[SAMPLES_ALTITUDE] =  temp / 0.0065 * (1 - pow(press / NORM_AIRPRESSURE, 0.1903)); //in cm
			altitude[smoothing] =  temp / 0.0065 * (1 - pow(press / float(normpress), 0.1903)); //in cm
			//float x = -0.5 * (SAMPLES_ALTITUDE - 1), sxx = 0, sxy = 0; // init linear regression of altitude samples
			float x = -0.5 * (smoothing - 1), sxx = 0, sxy = 0; // init linear regression of altitude samples
			//for (int i = 0;  i < SAMPLES_ALTITUDE; i++) { // add sums over all altitude samples
			for (int i = 0;  i < smoothing; i++) { // add sums over all altitude samples
			altitude[i] = altitude[i + 1]; // shift altitude data buffer by one sample
				pressure[i] = pressure[i+1];
				sxx += x * x;
				sxy += x * altitude[i];
				x++;
			}
			alt = 0.0;
			pascal = 0.0;
			for (int i = 0;  i < smoothing; i++) {
				alt += altitude[i];
				pascal += pressure[i];

			}
			alt /= smoothing;
			pascal = pascal /float(smoothing);
/*
			diff[SAMPLES_DIFF] = alt-oldalt;
			climbRate1 = 0;
			for (int i = 0;  i < SAMPLES_DIFF; i++) {
				diff[i] = diff[i+1];
				climbRate1 += diff[i];

			}
			oldalt = alt;
*/
			//neue Druckmessung kommt nur bei jedem 2. Zyklus der state machine
			//climbRate = sxy / sxx / (SAMPLING_PERIOD * 2.0); // calculate climb rate as slope of linear regression
			climbRate = sxy / sxx / float(timeInterval/1e6); // calculate climb rate as slope of linear regression
			//climbRate1 = climbRate1 /float(timeInterval/1e6)/SAMPLES_DIFF;
			timeInterval = 0;
			*climb = long(climbRate*100.0);
			//Serial.printf("climb (cm/s) %6.2f  Alt(cm) %8.2f Pressure %8.2f  \r",climbRate,  100*alt, pascal );
			return true;
		}
	}

	return false; //in cm/s as integer
}

