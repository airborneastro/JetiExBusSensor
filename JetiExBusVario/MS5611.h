/*
 * MS5611.h
 *
 *  Created on: 06.11.2017
 *      Author: KarlJ
 */

#ifndef MS5611_H_
#define MS5611_H_
// max conversion time with OSR=4096 is  9.04mS

#define SENSOR_MS5611 //if using external sensor on SCA2/SDA2 instead of MPU9250
#define MS5611_SAMPLE_PERIOD_MS         10

#define MS5611_READ_TEMPERATURE 		11
#define MS5611_READ_PRESSURE			22
#define MS5611_READ_PRESSURE1			23 //arbitrary for state machine

#ifndef SENSOR_MS5611
	#define MS5611_I2C_ADDRESS 0x76 //for sensor on MPU9250 module
#else
	#define  MS5611_I2C_ADDRESS 0x77 //for separate MS5611

#endif
#define MS5611_RESET      	0x1E
#define MS5611_CONVERT_D1 	0x40
#define MS5611_CONVERT_D2 	0x50
#define MS5611_ADC_READ   	0x00

#define MS5611_ADC_4096 	0x08
#define ADC_8192 0x0A
#define PWR_MGMT_1         0x6B
#define MPU9250_I2C_ADDRESS 0x68
#define WHO_AM_I_MPU9250 0x75
#define INT_PIN_CFG        0x37

#define SMPLRT_DIV        0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D


class MS5611 {

public :
MS5611(void);
void begin(void);
float readTemperature(void);

uint8_t readPressureTemperature(float* press, float* temp);
void TriggerPressureSample(void);
void TriggerTemperatureSample(void);
uint32_t  ReadSample(void);

void CalculateTemperatureCx10(void);
float CalculatePressurePa(void);
void CalculateSensorNoisePa(void);
void Reset(void);
void WriteByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t d) ;
uint8_t ReadByte(uint8_t deviceAddress, uint8_t registerAddress);
void GetCalibrationCoefficients(void);
float Pa2Cm(float pa);
void Test(int nSamples);
int  ReadPROM(void);
uint8_t CRC4(uint8_t prom[] );
int  SampleStateMachine(void);
void InitializeSampleStateMachine(void);

volatile float paSample_;
volatile float zCmSample_;
float zCmAvg_;
float  celsiusSample_;
volatile int sensorState;

private :
uint8_t prom_[16];
uint16_t cal_[6];
int64_t tref_;
int64_t offT1_;
int64_t sensT1_;
volatile int32_t tempCx100_;
uint32_t D1_;
uint32_t D2_;
int64_t dT_;
};




#endif /* MS5611_H_ */
