/*
 * MS5611.cpp
 *
 *  Created on: 06.11.2017
 *      Author: KarlJ
 */
#include <i2c_t3.h>
#include "MS5611.h"
#define MPU9250_I2C_ADDRESS 0x68


MS5611::MS5611() {
	paSample_ = 0.0f;
	zCmSample_ = 0.0f;
	celsiusSample_ = 0;
	zCmAvg_ = 0.0f;
	}
void MS5611::begin() {

	Reset();
	ReadPROM();
	GetCalibrationCoefficients();
	InitializeSampleStateMachine();

}
uint8_t MS5611::readPressureTemperature(float* press, float* temp) {
	if (SampleStateMachine() ) {
		*press = paSample_;
		*temp = celsiusSample_ + 273.15; // in Kelvin

		return 1;
	}
    return 0;
}
float MS5611::readTemperature() {
	if (!SampleStateMachine() ){
			return celsiusSample_;
		}
}




#if 0

#define MAX_TEST_SAMPLES    100
extern char gszBuf[];
static float pa[MAX_TEST_SAMPLES];
static float z[MAX_TEST_SAMPLES];

void MS5611::Test(int nSamples) {
	int n;
    float paMean, zMean, zVariance, paVariance;
    paMean = 0.0f;
    zMean = 0.0f;
    paVariance = 0.0f;
    zVariance = 0.0f;
    for (n = 0; n < nSamples; n++) {
	    TriggerTemperatureSample();
	    delay(MS5611_SAMPLE_PERIOD_MS);
	    D2_ = ReadSample();
	    CalculateTemperatureCx10();
		TriggerPressureSample();
		delay(MS5611_SAMPLE_PERIOD_MS);
		D1_ = ReadSample();
		pa[n] = CalculatePressurePa();
        z[n] =  Pa2Cm(pa[n]);
        paMean += pa[n];
        zMean += z[n];
        }
    paMean /= nSamples;
    zMean /= nSamples;
    Serial.printf("paMean = %dPa, zMean = %dcm\r\n",(int)paMean,(int)zMean);
    for (n = 0; n < nSamples; n++) {
        paVariance += (pa[n]-paMean)*(pa[n]-paMean);
        zVariance += (z[n]-zMean)*(z[n]-zMean);
        //Serial.printf("%d %d\r\n",(int)pa[n],(int)z[n]);
       }
    paVariance /= (nSamples-1);
    zVariance /= (nSamples-1);
    Serial.printf("\r\npaVariance %d  zVariance %d\r\n",(int)paVariance, (int)zVariance);
	}
#endif








/*float MS5611::Pa2Cm(float paf)  {
   	int32_t pa,inx,pa1,z1,z2;
    float zf;
    pa = (int32_t)(paf);

   	if (pa > PA_INIT) {
      	zf = (float)(gPZTbl[0]);
      	}
   	else {
      	inx = (PA_INIT - pa)>>10;
      	if (inx >= PZLUT_ENTRIES-1) {
         	zf = (float)(gPZTbl[PZLUT_ENTRIES-1]);
         	}
      	else {
         	pa1 = PA_INIT - (inx<<10);
         	z1 = gPZTbl[inx];
         	z2 = gPZTbl[inx+1];
         	zf = (float)(z1) + ( ((float)pa1-paf)*(float)(z2-z1))/1024.0f;
         	}
      	}
   	return zf;
   	}*/

void MS5611::CalculateTemperatureCx10(void) {
	dT_ = (int64_t)D2_ - tref_;
	tempCx100_ = 2000 + ((dT_*((int32_t)cal_[5]))>>23);
    //Serial.printf("tempCx100 %d", tempCx100_);
	}


float MS5611::CalculatePressurePa(void) {
	float pa;
    int64_t offset1, sens,offset2,sens2,t2;
	offset1 = offT1_ + ((((int64_t)cal_[3])*dT_)>>7);
	sens = sensT1_ + ((((int64_t)cal_[2])*dT_)>>8);
    if (tempCx100_ < 2000) { // correction for temperature < 20C
        t2 = ((dT_*dT_)>>31);
        offset2 = (5*(tempCx100_-2000)*(tempCx100_-2000))/2;
        sens2 = offset2/2;
        }
    else {
        t2 = 0;
        sens2 = 0;
        offset2 = 0;
        }
    tempCx100_ -= t2;
    offset1 -= offset2;
    sens -= sens2;
	pa = (((float)((int64_t)D1_ * sens))/2097152.0f - (float)offset1) / 32768.0f;
	//return pa;

#ifndef SENSOR_MS5611
	return 2.0f*pa;  //something is fishy here.... Karl
#else
	return pa;
#endif

	}


/// Trigger a pressure sample with max oversampling rate
void MS5611::TriggerPressureSample(void) {
#ifndef SENSOR_MS5611
	Wire.beginTransmission(MS5611_I2C_ADDRESS);
	Wire.write(MS5611_CONVERT_D1 | MS5611_ADC_4096);  //  pressure conversion, max oversampling
	Wire.endTransmission(I2C_NOSTOP);
#else
	Wire2.beginTransmission(MS5611_I2C_ADDRESS);
	Wire2.write(MS5611_CONVERT_D1 | MS5611_ADC_4096);  //  pressure conversion, max oversampling
	Wire2.endTransmission(I2C_NOSTOP);
#endif
   }

/// Trigger a temperature sample with max oversampling rate
void MS5611::TriggerTemperatureSample(void) {

#ifndef SENSOR_MS5611
	Wire.beginTransmission(MS5611_I2C_ADDRESS);
	Wire.write(MS5611_CONVERT_D2 | MS5611_ADC_4096);   //  temperature conversion, max oversampling
	Wire.endTransmission(I2C_NOSTOP);
#else
	Wire2.beginTransmission(MS5611_I2C_ADDRESS);
	Wire2.write(MS5611_CONVERT_D2 | MS5611_ADC_4096);   //  temperature conversion, max oversampling
	Wire2.endTransmission(I2C_NOSTOP);
#endif
   }

uint32_t MS5611::ReadSample(void)	{

#ifndef SENSOR_MS5611
	Wire.beginTransmission(MS5611_I2C_ADDRESS);  // Initialize the Tx buffer
	Wire.write(0x00);                        // Put ADC read command in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(MS5611_I2C_ADDRESS, 3);     // Read three bytes from slave PROM address
#else
	Wire2.beginTransmission(MS5611_I2C_ADDRESS);  // Initialize the Tx buffer
	Wire2.write(0x00);                        // Put ADC read command in Tx buffer
	Wire2.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
    Wire2.requestFrom(MS5611_I2C_ADDRESS, 3);     // Read three bytes from slave PROM address

#endif

    int inx = 0;
	uint8_t buf[3];
#ifndef SENSOR_MS5611
	while (Wire.available()) {
        buf[inx++] = Wire.read();
		}
#else
	while (Wire2.available()) {
        buf[inx++] = Wire2.read();
		}
#endif
	uint32_t w = (((uint32_t)buf[0])<<16) | (((uint32_t)buf[1])<<8) | (uint32_t)buf[2];
	return w;
   }


void MS5611::InitializeSampleStateMachine(void) {
   TriggerTemperatureSample();
   sensorState = MS5611_READ_TEMPERATURE;
   }

int MS5611::SampleStateMachine(void) {
   if (sensorState == MS5611_READ_TEMPERATURE) {
      D2_ = ReadSample(); //temperature
      //Serial.printf("D2 %d", D2_);
      TriggerPressureSample();
      //DBG_1(); // turn on the debug pulse for timing the critical computation
      CalculateTemperatureCx10();
      celsiusSample_ = tempCx100_/100.0;
      paSample_ = CalculatePressurePa();
	 // zCmSample_ = Pa2Cm(paSample_);
    //zCmSample_ = (3*zCmSample_ + Pa2Cm(paSample_))/4.0f;
   // Serial.printf("Zavg : %dcm\r\n",(int)zCmSample_);
      //DBG_0();
	  sensorState = MS5611_READ_PRESSURE;
      return 1;  // 1 => new altitude sample is available
      }
   else
   if (sensorState == MS5611_READ_PRESSURE) {
      D1_ = ReadSample(); //Pressure
      //Serial.printf("D1 %d", D1_);
      TriggerTemperatureSample();
      sensorState = MS5611_READ_TEMPERATURE;
      return 0; // 0 => intermediate state
      }
   return 0;
   }



void MS5611::Reset() {
#ifndef SENSOR_MS5611
	WriteByte(MPU9250_I2C_ADDRESS, PWR_MGMT_1, 0x80);
	delay(100); // Wait after reset
// as per datasheet all registers are reset to 0 except WHOAMI and PWR_MGMT_1,

	WriteByte(MPU9250_I2C_ADDRESS, PWR_MGMT_1, 0x01);
//set bypass for other devices
	WriteByte(MPU9250_I2C_ADDRESS, INT_PIN_CFG, 0x12);
	Wire.beginTransmission(MS5611_I2C_ADDRESS);
	Wire.write(MS5611_RESET);
	Wire.endTransmission();
#else

	Wire2.beginTransmission(MS5611_I2C_ADDRESS);
	Wire2.write(MS5611_RESET);
	Wire2.endTransmission();
#endif
	delay(100); // 3mS as per app note AN520
    }


void MS5611::GetCalibrationCoefficients(void)  {
    for (int inx = 0; inx < 6; inx++) {
		int promIndex = 2 + inx*2;
		cal_[inx] = (((uint16_t)prom_[promIndex])<<8) | (uint16_t)prom_[promIndex+1];
		}
    //Serial.printf("\r\nCalib Coeffs : %d %d %d %d %d %d\r\n",cal_[0],cal_[1],cal_[2],cal_[3],cal_[4],cal_[5]);
    tref_ = ((int64_t)cal_[4])<<8;
    offT1_ = ((int64_t)cal_[1])<<16;
    sensT1_ = ((int64_t)cal_[0])<<15;
    }

int MS5611::ReadPROM(void)    {
    for (int inx = 0; inx < 8; inx++) {
#ifndef SENSOR_MS5611
    	Wire.beginTransmission(MS5611_I2C_ADDRESS);
		Wire.write(0xA0 + inx*2);
		Wire.endTransmission(false); // restart
		Wire.requestFrom(MS5611_I2C_ADDRESS, 2);
		int cnt = 0;
		while (Wire.available()) {
			prom_[inx*2 + cnt] = Wire.read();
			cnt++;
			}
		}
#else
	Wire2.beginTransmission(MS5611_I2C_ADDRESS);

	Wire2.write(0xA0 + inx*2);
	Wire2.endTransmission(false); // restart
	Wire2.requestFrom(MS5611_I2C_ADDRESS, 2);
	int cnt = 0;
	while (Wire2.available()) {
		prom_[inx*2 + cnt] = Wire2.read();
		cnt++;
		}
	}
#endif
/*
	Serial.printf("\r\nProm : ");

	for (int inx = 0; inx < 14; inx++) {
		Serial.printf("0x%02x ", prom_[inx]);
		}
	Serial.printf("\r\n");*/
	uint8_t crcPROM = prom_[15] & 0x0F;
	uint8_t crcCalculated = CRC4(prom_);

	return (crcCalculated == crcPROM ? 1 : 0);
	}


uint8_t MS5611::CRC4(uint8_t prom[] ) {
	 int cnt, nbit;
	 uint16_t crcRemainder;
	 uint8_t crcSave = prom[15]; // crc byte in PROM
	 //Serial.printf("PROM CRC = 0x%x\r\n", prom[15] & 0x0F);
	 crcRemainder = 0x0000;
	 prom[15] = 0; //CRC byte is replaced by 0

	 for (cnt = 0; cnt < 16; cnt++)  {
		crcRemainder ^= (uint16_t) prom[cnt];
		for (nbit = 8; nbit > 0; nbit--) {
			if (crcRemainder & (0x8000)) {
				crcRemainder = (crcRemainder << 1) ^ 0x3000;
				}
			else {
				crcRemainder = (crcRemainder << 1);
				}
			}
		}
	 crcRemainder= (0x000F & (crcRemainder >> 12)); // final 4-bit reminder is CRC code
	 prom[15] = crcSave; // restore the crc byte
	// Serial.printf("Calculated CRC = 0x%x\r\n",  crcRemainder ^ 0x0);
	 return (uint8_t)(crcRemainder ^ 0x0);

	}

void MS5611::WriteByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t d) {

#ifndef SENSOR_MS5611
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(d);
  Wire.endTransmission();
#else
  Wire2.beginTransmission(deviceAddress);
  Wire2.write(registerAddress);
  Wire2.write(d);
  Wire2.endTransmission();
#endif
}


uint8_t MS5611::ReadByte(uint8_t deviceAddress, uint8_t registerAddress){
  uint8_t d;
#ifndef SENSOR_MS5611
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false); // restart
  Wire.requestFrom(deviceAddress, (uint8_t) 1);
  d = Wire.read();
#else
  Wire2.beginTransmission(deviceAddress);
  Wire2.write(registerAddress);
  Wire2.endTransmission(false); // restart
  Wire2.requestFrom(deviceAddress, (uint8_t) 1);
  d = Wire2.read();
#endif
  return d;
}
