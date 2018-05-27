/* 
  Jeti Sensor EX Telemetry C++ Library
  
  VarioSensor - get some demo values for telemetry display
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

#ifndef VarioSensor_H
#define VarioSensor_H

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <i2c_t3.h>
#include "MS5611.h"
#define SAMPLES_ALTITUDE 100  //maximalwert für smoothing, konstant!
#define SAMPLES_DIFF 10
//#define LOWPASSFILTER
#define SAMPLING_PERIOD 0.01 // in s
#define FILTERX 0.8 //weight on older data
#define FILTERY 0.2 //weight on new data
#define NORM_AIRPRESSURE 101300.0     //in Pa
class VarioSensor
{
public:
  VarioSensor();

  long GetVoltage();
  long GetAltitude();
  long GetTemp();
  bool GetClimb(long* climb, long smoothing, long normpress);
  float GetPressure();
//  long GetFuel();
//  long GetRpm();
  void setup(long normpress);
  long GetVal( int idx ); // values 7..18
  float airTemperature, airTemperature0; // Kelvin
  float airPressure, airPressure0; // Pa

protected:
  enum
  {
    NVALS = 3,
  };


  float volt;
  unsigned long tiVolt;

 /* float alt;
  unsigned long tiAlt;

  float temp;
  unsigned long tiTemp;
*/
  float climb;
  unsigned long tiClimb;

  float fuel;
  unsigned long tiFuel;

  float rpm;
  unsigned long tiRpm;

  float vals[NVALS];
  unsigned long tiVals[NVALS];

  unsigned long timeSample; // us
  unsigned long timeJetiSample;
  unsigned long timeInterval;
  float altitude[SAMPLES_ALTITUDE + 1]; //
  float pressure[SAMPLES_ALTITUDE + 1];

  float diff[SAMPLES_DIFF+1];
  float oldalt;
  float climbRate; // m/s
  float climbRate1;
  float press, temp,pascal;
  float alt, previouspress;

};

#endif // VarioSensor
