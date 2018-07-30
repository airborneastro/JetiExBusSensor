/*
  ------------------------------------------------------------------
                    Jeti VarioGPS Sensor
  ------------------------------------------------------------------
            *** Universeller Jeti Telemetrie Sensor ***
  Vario, GPS, Strom/Spannung, Empfängerspannungen, Temperaturmessung

*/
#define VARIOGPS_VERSION "Vario ExBus V0.9"
#define JETIEX_DEBUG
#include "WProgram.h"
/*

  ******************************************************************
  Versionen:

  V2.2.1  26.03.18  Bugfix bei float<->int casting Smoothing Factor (by RS)
  V2.2    15.02.18  Vario Tiefpass mit nur einem Smoothing Factor (by RS)
                    Jeder Sensor kann mit #define deaktiviert werden
  V2.1.1  13.01.18  kleine Fehlerbehebung mit libraries
  V2.1    23.12.17  Analog Messeingänge stark überarbeitet, NTC-Temperaturmessung hinzugefügt,
                    startup-/auto-/maual-reset für Kapazitätsanzeige, SRAM-Speicheroptimierung
  V2.0.2  03.12.17  Fehler in GPS Trip behoben
  V2.0.1  21.11.17  Fehler bei Spannungsmessung behoben
  V2.0    20.11.17  GPS Trip[km] und verbrauchte Kapazität[mAh] eingebaut, Stromsensoren ACS712 eingebaut
  V1.9    17.11.17  ACSxxx Stromsensoren eingebaut
  V1.8    17.11.17  Luftdrucksensor MS5611/LPS werden unterstützt
  V1.7.2  15.11.17  Fehlerbehebung mit BME280
  V1.7.1  15.11.17  Speicheroptimierung, kleinere Fehler behoben
  V1.7    12.11.17  Empfängerspannungen können gemessen werden
  V1.6    05.11.17  Luftdrucksensoren BMP085/BMP180/BMP280/BME280 eingebaut, und zu VarioGPS umbenannt
  V1.5    05.11.17  Code von RC-Thoughts(Jeti_GPS-Sensor) übernommen




  ******************************************************************
  Unterstützte Hardware:

  - Arduino Pro Mini 3.3V-8Mhz/5V-16Mhz
  - GPS-Modul mit NMEA Protokoll und UART@9600baud
  - Luftdrucksensoren: BMP280, BME280, MS5611, LPS
  - Stromsensoren @3.3V/5V Betriebsspannung:        AttoPilot Module @3.3V: 45A/13.6V - 90A/50V - 180A/50V (@5V: 45A/20.6V - 90A/60V - 180A/60V)
                                                    APM2.5 PowerModul @5V: 90A/50V (@3.3V: 58A/33.4V)
                                                    ACS758_50B, ACS758_100B, ACS758_150B, ACS758_200B, ACS758_50U, ACS758_100U, ACS758_150U, ACS758_200U
  - zusätzliche Stromsensoren @5V Betriebsspannung: ACS712_05, ACS712_20, ACS712_30



  ******************************************************************
  Anzeige:

  Nur mit Luftdrucksensor werden die Werte angezeigt:
  - Rel. und Abs. Höhe
  - Vario
  - Luftdruck
  - Temperatur
  - Luftfeuchte (nur mit BME280)

  Im GPS Basic Mode werden die Werte angezeigt:
  - Position
  - Geschwindigkeit
  - Rel. und Abs. Höhe
  - Vario

  Im GPS Extended Mode werden zusätzlich die Werte angezeigt:
  - Distanz vom Modell zum Startpunkt (2D oder 3D)
  - zurückgelegte Strecke (Trip)
  - Flugrichtung (Heading)
  - Kurs vom Modell zum Startpunkt
  - Anzahl Satelliten
  - HDOP (Horizontaler Faktor der Positionsgenauigkeit)
  - Luftdruck
  - Temperatur
  - Luftfeuchtigkeit

  An den Analogeingängen sind folgende Messungen möglich:
  - Strom- und Spannung für Hauptantrieb mit verbrauchter Kapazität[mAh] und Leistung[W]
  - 2x Empfängerspannung
  - Temperatur mit NTC-Wiederstand von -55 bis +155°C

  Folgende Einstellungen können per Jetibox vorgenommen werden:
  - GPS: deaktiviert, Basic oder Extended
  - GPS Distanz: 2D oder 3D
  - Vario Filterparameter X, Y und Deadzone
  - Stromsensor für Hauptantrieb
  - Einstellung Reset der Kapazität:
        STARTUP(Wert ist nach jedem Einschalten auf 0)
        AUTO(Wert wird gespeichert und erst zurückgesetzt wenn ein geladener Akku angeschlossen wird)
        MANUAL(Wert muss manuell per Jetibox zurückgesetzt werden mit RESET OFFSET)
  - Rx1, Rx2 Empfängerspannungsmessung aktivieren
  - Temperaturmessung aktivieren

*/

#include <JetiExBusSerial.h>
#include <JetiExProtocolBuf.h>
#include <JetiExBusProtocol.h>
#include <EEPROM.h>
#include <i2c_t3.h>
#include "defaults.h"
#include "HardwareSerial.h"
//#include <stdio.h>
#include "VarioSensor.h"
//See https://forum.pjrc.com/threads/44857-How-to-Reset-Restart-Teensy-3-5-using-sotware
#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

JetiExBusProtocol jetiEx;

#ifdef SUPPORT_GPS
  #include <TinyGPS++.h>
//  #include <AltSoftSerial.h>
  TinyGPSPlus gps;
HardwareSerial gpsSerial = Serial1;

#endif

#ifdef SUPPORT_BMx280
  #include "BMx_Sensor.h"
  BMx_Sensor boschPressureSensor;
#endif

#ifdef SUPPORT_MS5611_LPS
  #include <MS5611.h>
//  #include <LPS.h>
VarioSensor variosensor;
//  LPS lps;
#endif


#if V_REF < 1800 || V_REF > 5500
  #error unsupported supply voltage
#endif

#define MEASURING_INTERVAL        150         //ms NOT USED
#define EEPROM_ADRESS_CAPACITY    20


struct {
  uint8_t mode = DEFAULT_GPS_MODE;
  bool distance3D = DEFAULT_GPS_3D_DISTANCE;
} gpsSettings;

struct {
  uint8_t type = unknown;
  long smoothingValue;
  long normpress; //
  long deadzone;
} pressureSensor;


unsigned long lastTime = 0;

unsigned long lastTimeCapacity = 0;
unsigned long lastTimeCapacityAdd = 0;
uint8_t currentSensor = DEFAULT_CURRENT_SENSOR;
uint8_t capacityMode = DEFAULT_CAPACITY_MODE;
bool enableRx1 = DEFAULT_ENABLE_Rx1;
bool enableRx2 = DEFAULT_ENABLE_Rx2;
bool enableExtTemp = DEFAULT_ENABLE_EXT_TEMP;

const float factorVoltageDivider[] = { float(voltageInputR1[0]+voltageInputR2[0])/voltageInputR2[0],
                                       float(voltageInputR1[1]+voltageInputR2[1])/voltageInputR2[1],
                                       float(voltageInputR1[2]+voltageInputR2[2])/voltageInputR2[2],
                                       float(voltageInputR1[3]+voltageInputR2[3])/voltageInputR2[3],
                                       float(voltageInputR1[4]+voltageInputR2[4])/voltageInputR2[4],
                                       float(voltageInputR1[5]+voltageInputR2[5])/voltageInputR2[5],
                                       float(voltageInputR1[6]+voltageInputR2[6])/voltageInputR2[6],
                                       float(voltageInputR1[7]+voltageInputR2[7])/voltageInputR2[7],
									   float(voltageInputR1[8]+voltageInputR2[7])/voltageInputR2[8]};

const float timefactorCapacityConsumption = (1000.0/MEASURING_INTERVAL*60*60)/1000;
float capacityConsumption = 0;

// Restart by user
//void(* resetFunc) (void) = 0;
void resetFunc(void) {
	// see https://forum.pjrc.com/threads/44857-How-to-Reset-Restart-Teensy-3-5-using-sotware
	WRITE_RESTART(0x5FA0004);

}
#include "HandleMenu.h"

long readAnalog_mV(uint8_t sensorVolt, uint8_t pin){
  return (analogRead(pin)/1023.0)*V_REF*factorVoltageDivider[sensorVolt];
}

uint8_t getVoltageSensorTyp(){
  if (currentSensor <= APM25_A){
    return currentSensor-1;
  #if V_REF >= 4500
  }else if(currentSensor >= ACS712_05 && currentSensor <= ACS712_30){
    return ACS712_voltage;
  #endif
  }else if(currentSensor >= ACS758_50B){
    return  ACS758_voltage;
  }
}

void setup()
{
#ifdef JETIEX_DEBUG
#if defined (CORE_TEENSY) || (__AVR_ATmega32U4__)
	Serial.begin( 38400 );
	 delay(2000);
	 Serial.println("Start Vario, wait 10sec");
#endif
#endif

	// identify sensor
#ifdef SUPPORT_BMx280
	pressureSensor.type = boschPressureSensor.begin(0x76);
	if(pressureSensor.type == unknown){
		pressureSensor.type = boschPressureSensor.begin(0x77);
	}
#endif
#ifdef SUPPORT_MS5611_LPS
	if(pressureSensor.type == unknown){

		pressureSensor.type = MS5611_;

	}

#endif
#if  defined SUPPORT_BMx280 || defined SUPPORT_MS5611_LPS
	switch (pressureSensor.type){
	case BMP280:
		pressureSensor.smoothingValue = BMx280_SMOOTHING;
		pressureSensor.deadzone = BMx280_DEADZONE;
		break;
	case BME280:
		pressureSensor.smoothingValue = BMx280_SMOOTHING;
		pressureSensor.deadzone = BMx280_DEADZONE;
		break;
	case MS5611_:
		pressureSensor.smoothingValue = MS5611_SMOOTHING;
		pressureSensor.normpress = MS5611_NORMPRESS;
		if (EEPROM.read(12) != 0xFF) {
			pressureSensor.normpress = EEPROM.read(12)*100 + 78400;//EEPROM = 229 -> 101300
		}
		//Serial.println("Vor sensoraktivierung");
		variosensor.setup(pressureSensor.normpress);
		break;
	case LPS_ :
		pressureSensor.smoothingValue = LPS_SMOOTHING;
		pressureSensor.deadzone = LPS_DEADZONE;
		break;
	}
#endif
	// read settings from eeprom
#ifdef SUPPORT_GPS

	if (EEPROM.read(1) != 0xFF) {
		gpsSettings.mode = EEPROM.read(1);
	}

	if (EEPROM.read(2) != 0xFF) {
		gpsSettings.distance3D = EEPROM.read(2);
	}
#endif

#ifdef SUPPORT_MAIN_DRIVE
	if (EEPROM.read(3) != 0xFF) {
		currentSensor = EEPROM.read(3);
	}
	if (EEPROM.read(5) != 0xFF) {
		capacityMode = EEPROM.read(5);
	}

#endif

#ifdef SUPPORT_RX_VOLTAGE
		if (EEPROM.read(6) != 0xFF) {
			enableRx1 = EEPROM.read(6);
		}
		if (EEPROM.read(7) != 0xFF) {
			enableRx2 = EEPROM.read(7);
		}
#endif

#ifdef SUPPORT_EXT_TEMP
		if (EEPROM.read(8) != 0xFF) {
			enableExtTemp = EEPROM.read(8);
		}
#endif

		if (EEPROM.read(10) != 0xFF) {
			pressureSensor.smoothingValue = EEPROM.read(10);
		}

#ifdef SUPPORT_GPS
		// init GPS
		gpsSerial.begin(GPSBaud);
//		delay(100);
//		gpsSerial.println("$PMTK251,115200*1F");
//		delay(500);
//		gpsSerial.begin(115200);
// or $PMTK251,57600*2C  $PMTK251,38400*27  $PMTK251,19200*22  $PMTK251,14400*29
// Checksums http://www.hhhh.org/wiml/proj/nmeaxor.html
// MiniGPS GUI http://docs.mirifica.eu/GlobalTop_Technology/tools/MiniGPS/MediaTek/
// GPS chip resets to 9600 after power up
// NMEA update rate in ms $PMTK220,1000*1F  $PMTK220,200*2C   $PMTK220,100*2F
#endif


		// Setup sensors
		if(pressureSensor.type == unknown){
			jetiEx.SetSensorActive( ID_VARIO, false, sensors );
		}

		if(gpsSettings.mode == GPS_basic || pressureSensor.type != BME280){
			jetiEx.SetSensorActive( ID_HUMIDITY, false, sensors );
		}
		if(gpsSettings.mode == GPS_basic || pressureSensor.type == unknown){
			jetiEx.SetSensorActive( ID_PRESSURE, false, sensors );
			jetiEx.SetSensorActive( ID_TEMPERATURE, false, sensors );
		}

		if(gpsSettings.mode == GPS_disabled){
			jetiEx.SetSensorActive( ID_GPSLAT, false, sensors );
			jetiEx.SetSensorActive( ID_GPSLON, false, sensors );
			jetiEx.SetSensorActive( ID_GPSSPEED, false, sensors );
		}

		if(gpsSettings.mode == GPS_disabled && pressureSensor.type == unknown){
			jetiEx.SetSensorActive( ID_ALTREL, false, sensors );
			jetiEx.SetSensorActive( ID_ALTABS, false, sensors );
		}

		if(gpsSettings.mode != GPS_extended){
			jetiEx.SetSensorActive( ID_DIST, false, sensors );
			jetiEx.SetSensorActive( ID_TRIP, false, sensors );
			jetiEx.SetSensorActive( ID_HEADING, false, sensors );
			jetiEx.SetSensorActive( ID_COURSE, false, sensors );
			jetiEx.SetSensorActive( ID_SATS, false, sensors );
			jetiEx.SetSensorActive( ID_HDOP, false, sensors );
		}

		if(currentSensor == mainDrive_disabled){
			jetiEx.SetSensorActive( ID_VOLTAGE, false, sensors );
			jetiEx.SetSensorActive( ID_CURRENT, false, sensors );
			jetiEx.SetSensorActive( ID_CAPACITY, false, sensors );
			jetiEx.SetSensorActive( ID_POWER, false, sensors );
#ifdef SUPPORT_MAIN_DRIVE
		}else{
			if(capacityMode > startup){

				// read capacity from eeprom
				int eeAddress = EEPROM_ADRESS_CAPACITY;
				EEPROM.get(eeAddress, capacityConsumption);
				if(capacityMode == automatic){
					eeAddress += sizeof(float);
					float cuVolt = readAnalog_mV(getVoltageSensorTyp(),VOLTAGE_PIN)/1000.0;
					float oldVolt;
					EEPROM.get(eeAddress, oldVolt);
					if(cuVolt >= oldVolt * ((100.0+VOLTAGE_DIFFERENCE)/100.0)){
						capacityConsumption = 0;
					}
				}
			}
#endif
		}

		if(!enableRx1){
			jetiEx.SetSensorActive( ID_RX1_VOLTAGE, false, sensors );
		}

		if(!enableRx2){
			jetiEx.SetSensorActive( ID_RX2_VOLTAGE, false, sensors );
		}

		if(!enableExtTemp){
			jetiEx.SetSensorActive( ID_EXT_TEMP, false, sensors );
		}

		// init Jeti EX Bus
		jetiEx.SetDeviceId( 0x76, 0x32 );
		jetiEx.Start( "VarioGPS", sensors, 2 );

	}

   int main(void)
   {
	   //Karl start
	   setup();
	   char buf[30];
	   float testCurrent;
	   static uint8_t previous_motor = 0;
	   static unsigned long lastLoop = micros();
	   static unsigned long elapsedmicros = 0;
	   testCurrent = (analogRead(CURRENT_PIN) / 1023.0) * V_REF; //reads zero value current
	   while(1) {	//replaces "loop" in Arduino code, fast loop reading climb etc
		   // channel data
		   //if (false)   //comment to see channel data
		   static uint8_t motor_on;

		   if ( jetiEx.HasNewChannelData() ) //remove comment to see channel data
		   {
/*			   int i;
			   for (i = 0; i < jetiEx.GetNumChannels(); i++)
			   {
				   sprintf(buf, "chan-%d: %d", i, jetiEx.GetChannel(i));
				   Serial.println(buf);
			   }
		   	   if
*/
			   if (jetiEx.GetChannel(7) > 8100) //read channel 7 (switch SE, motor ) = Servo channel 8, off = 8080
				   motor_on = 1;
			   else
				   motor_on = 0;
		   }
		   float testVoltage;

		   //testVoltage = (analogRead(VZCR_PIN) / 1023.0) * V_REF; //in mV, offset reference from chip
		   //testCurrent = (analogRead(CURRENT_PIN) / 1023.0) * V_REF;
		 //  Serial.printf("VCZR (mV) %8.1f  %8.1f", testVoltage, testCurrent);
		 //  Serial.println();

		   static long uRelAltitude = 0;
		   static long uAbsAltitude = 0;
		   static bool setStartAltitude = false;
		   static float lastVariofilter = 0;
		   static long lastAltitude = 0;
		   static long avTemp = 0;
		   static long avVario = 0;
		   static long avPressure = 0;
		   static long startAltitude = 0;
		   static uint8_t numVario = 0;

		   static long avAltitude = 0;
		   long curAltitude;
		   long uPressure;
		   int uTemperature;
		   long uVario;
/*
		   while(gpsSerial.available() )  {
			   char c = gpsSerial.read();
			   Serial.print(c);
			   if(gps.encode(c)){
		   			break;
		   		}else{
		   			//return ;
		   		}
		   }
*/
		   if (jetiEx.IsBusReleased())
		   {
			   // exBus is currently sending an ex packet
			   // do time consuming stuff here (20-30ms)
			   //delay( 30 );
			   //Data actually sent, clear averages
			   avVario=0;
			   avPressure = 0;
			   avTemp = 0;
			   avAltitude = 0;
			   numVario = 0;
#ifdef SUPPORT_GPS
			   while(gpsSerial.available() )
			   {
				   char c = gpsSerial.read();
				   //Serial.print(c);
				   if(gps.encode(c)){
					   break;
				   }else{
					   //return ;
				   }
			   }
#endif
		   }

		   if (variosensor.GetClimb(&uVario, pressureSensor.smoothingValue, pressureSensor.normpress)) {
			   numVario += 1; //Average while waiting for next doJetiSend (150ms)
			   avVario += uVario;
			   avAltitude += variosensor.GetAltitude();
			   //Serial.printf("Alt(cm) %6.2f \r\n", float(avAltitude/numVario));
			   avTemp += variosensor.GetTemp()*10;
			   avPressure += variosensor.GetPressure();

		   }
#ifdef UNIT_US
		   // EU to US conversions
		   // ft/s = m/s / 0.3048
		   // inHG = hPa * 0,029529983071445
		   // ft = m / 0.3048
		   uVario /= 0.3048;
		   uPressure *= 0.029529983071445;
		   uTemperature = uTemperature * 1.8 + 320;
#endif



		   // analog input
#ifdef SUPPORT_MAIN_DRIVE
		   if(currentSensor){
			   // Voltage
			   float cuVolt = readAnalog_mV(getVoltageSensorTyp(),VOLTAGE_PIN)/1000.0;
			   jetiEx.SetSensorValue( ID_VOLTAGE, cuVolt*10);

			   // Current
			   uint16_t ampOffset;

			   if (currentSensor <= APM25_A){
				   ampOffset = Atto_APM_offset;
			   }else if (currentSensor > ACS758_200B){
				   ampOffset = ACS_U_offset;
			   }else{
				   ampOffset = ACS_B_offset;
			   }
			   if (currentSensor == ACS709_35BB){
				   ampOffset = ACS_B_offset;
				   ampOffset = testCurrent;
				   //ampOffset = (analogRead(VZCR_PIN) / 1023.0) * V_REF; //in mV, offset reference from chip

			   }

			   float mVanalogIn = (analogRead(CURRENT_PIN) / 1023.0) * V_REF; // mV
			   //float cuAmp = (mVanalogIn - ampOffset) / mVperAmp[currentSensor-1];
			   float cuAmp = (ampOffset - mVanalogIn) / mVperAmp[currentSensor-1]; //reverse polarity for Sharon wiring


			   cuAmp *= float(motor_on); // measure current only if motor is on
			   if (currentSensor > APM25_A){
				   cuAmp *= 5000.0/V_REF;
			   }

			   //Serial.printf("Current, mv/Amp %8.1f, %3d %3d ", cuAmp, mVperAmp[currentSensor-1] , currentSensor-1);
			   //Serial.println();

			   jetiEx.SetSensorValue( ID_CURRENT, cuAmp*10);

			   if (motor_on) {
 				   if (previous_motor == 0) {
  					   elapsedmicros = micros(); //start clock
  					   previous_motor = 1;
  					   lastLoop = micros();
  				   }

  				   if ((micros() - lastLoop) > 100000) { //needed because Teensy runs at full speed, loop does not wait "MEASURING_INTERVAL
  					 capacityConsumption += cuAmp * float(micros()-lastLoop)/1000000.0*0.277777777;
  					 lastLoop = micros();
  				   }

  			   }
  			   else {
  				   if (previous_motor) {
  					   previous_motor = 0;
  					   //Serial.printf("Elapsed seconds, consumption %8.1f %8.1f ", float(micros()- elapsedmicros)/1000000.0, capacityConsumption);
  					   //Serial.println();
  				   }
  			   }



			   //Serial.printf("Looptime [ms]  %8.2f ",float( micros()-lastLoop));
			   //Serial.println();

			   jetiEx.SetSensorValue( ID_CAPACITY, capacityConsumption);
			   // save capacity and voltage to eeprom
			   if(capacityMode > startup && (millis() - lastTimeCapacity) > CAPACITY_SAVE_INTERVAL){
				   if(cuAmp <= MAX_CUR_TO_SAVE_CAPACITY){
					   int eeAddress = EEPROM_ADRESS_CAPACITY;
					   EEPROM.put(eeAddress, capacityConsumption);
					   eeAddress += sizeof(float);
					   EEPROM.put(eeAddress, cuVolt);;
				   }
				   lastTimeCapacity = millis();
			   }
			   // Power
			   jetiEx.SetSensorValue( ID_POWER, cuAmp*cuVolt);
		   }//if currentSensor
#endif

#ifdef SUPPORT_RX_VOLTAGE
		   if(enableRx1){
			   jetiEx.SetSensorValue( ID_RX1_VOLTAGE, readAnalog_mV(Rx1_voltage,RX1_VOLTAGE_PIN)/10);
		   }

		   if(enableRx2){
			   jetiEx.SetSensorValue( ID_RX2_VOLTAGE, readAnalog_mV(Rx2_voltage,RX2_VOLTAGE_PIN)/10);
		   }
#endif

#ifdef SUPPORT_EXT_TEMP
		   if(enableExtTemp){
			   // convert the value to resistance
			   float aIn = 1023.0 / analogRead(TEMPERATURE_PIN) - 1.0;
			   aIn = SERIESRESISTOR / aIn;

			   // convert resistance to temperature
			   float steinhart;
			   steinhart = aIn / THERMISTORNOMINAL;                // (R/Ro)
			   steinhart = log(steinhart);                         // ln(R/Ro)
			   steinhart /= BCOEFFICIENT;                          // 1/B * ln(R/Ro)
			   steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);   // + (1/To)
			   steinhart = 1.0 / steinhart;                        // Invert
			   steinhart -= 273.15 - SELF_HEAT;                    // convert to °C and self-heat compensation

#ifdef UNIT_US
			   // EU to US conversions
			   steinhart = steinhart * 1.8 + 320;
#endif

			   jetiEx.SetSensorValue( ID_EXT_TEMP, steinhart*10);
		   }//if enableExtTemp
#endif



		   //Serial.println(numVario);
		   if (numVario) { //calculate average from values collected while waiting for doJetiSend
			   uVario = avVario/numVario;
			   uPressure = avPressure/numVario;
			   uTemperature = avTemp/numVario;
			   curAltitude = avAltitude/numVario;

		   }
		   //Serial.printf("Alt(cm) %6.2f \n", float(curAltitude));

		   jetiEx.SetSensorValue( ID_VARIO, uVario );
		   jetiEx.SetSensorValue( ID_PRESSURE, uPressure );
		   jetiEx.SetSensorValue( ID_TEMPERATURE, uTemperature );

		   if (!setStartAltitude  ) {
			   // Set start-altitude in sensor-start
			   if ((numVario > 0) ) {  //schon Werte da?
				   setStartAltitude = true;
				   startAltitude = curAltitude;
				   //Serial.printf("startAlt(cm) %6.2f Curalt %6.2f \r\n", float(startAltitude), float(curAltitude));
				   lastAltitude = curAltitude;
			   }

		   }else{
			   // Altitude
			   uRelAltitude = (curAltitude - startAltitude) / 10;
			   uAbsAltitude = curAltitude / 100;
		   }

		   //Serial.printf("climb (cm/s) %6.2f  RelAlt(cm) %8.2f AbsAlt(m) %8.2f Pressure %8.2f  \r\n", float(uVario)/100.0,  float(uRelAltitude*10.),float(uAbsAltitude), float(uPressure));

#ifdef SUPPORT_GPS
		   if(gpsSettings.mode != GPS_disabled){

			   static int homeSetCount = 0;
			   static float home_lat;
			   static float home_lon;
			   static float last_lat;
			   static float last_lon;
			   static long lastAbsAltitude = 0;
			   static unsigned long tripDist;
			   unsigned long distToHome;

			   // read data from GPS



			   if (gps.location.isValid() && gps.location.age() < 2000) { // if Fix

				   jetiEx.SetSensorValueGPS( ID_GPSLAT, false, gps.location.lat() );
				   jetiEx.SetSensorValueGPS( ID_GPSLON, true, gps.location.lng() );

				   // Altitude
				   uAbsAltitude = gps.altitude.meters();

#ifdef UNIT_US
				   jetiEx.SetSensorValue( ID_GPSSPEED, gps.speed.mph() );
#else
				   jetiEx.SetSensorValue( ID_GPSSPEED, gps.speed.kmph() );
#endif

				   jetiEx.SetSensorValue( ID_HEADING, gps.course.deg() );

				   if (homeSetCount < 3000) {  // set home position
					   ++homeSetCount;
					   home_lat = gps.location.lat();
					   home_lon = gps.location.lng();
					   last_lat = home_lat;
					   last_lon = home_lon;
					   lastAbsAltitude = gps.altitude.meters();
					   tripDist = 0;
					   if(pressureSensor.type == unknown){
						   startAltitude = gps.altitude.meters();
					   }

				   }else{

					   // Rel. Altitude
					   if(pressureSensor.type == unknown){
						   uRelAltitude = (uAbsAltitude - startAltitude)*10;
					   }

					   // Distance to model
					   distToHome = gps.distanceBetween(
							   gps.location.lat(),
							   gps.location.lng(),
							   home_lat,
							   home_lon);
					   if(gpsSettings.distance3D){
						   distToHome = sqrt(pow(uRelAltitude/10,2) + pow(distToHome,2));
					   }

					   // Course from home to model
					   jetiEx.SetSensorValue( ID_COURSE, gps.courseTo(home_lat,home_lon,gps.location.lat(),gps.location.lng()));

					   // Trip distance
					   float distLast = gps.distanceBetween(
							   gps.location.lat(),
							   gps.location.lng(),
							   last_lat,
							   last_lon);
					   if(gpsSettings.distance3D){
						   distLast = sqrt(pow(uAbsAltitude-lastAbsAltitude,2) + pow(distLast,2));
						   lastAbsAltitude = uAbsAltitude;
					   }
					   tripDist += distLast;
					   last_lat = gps.location.lat();
					   last_lon = gps.location.lng();
				   }

			   } else { // If Fix end
				   jetiEx.SetSensorValueGPS( ID_GPSLAT, false, 0 );
				   jetiEx.SetSensorValueGPS( ID_GPSLON, true, 0 );
				   if(pressureSensor.type == unknown){
					   uRelAltitude = 0;
					   uAbsAltitude = 0;
				   }

				   //uAbsAltitude = 0;
				   distToHome = 0;
				   jetiEx.SetSensorValue( ID_COURSE, 0 );
				   jetiEx.SetSensorValue( ID_GPSSPEED, 0 );
				   jetiEx.SetSensorValue( ID_HEADING, 0 );
			   }

			   jetiEx.SetSensorValue( ID_SATS, gps.satellites.value() );
			   jetiEx.SetSensorValue( ID_HDOP, gps.hdop.value());
#ifndef UNIT_US
			   //EU units
			   jetiEx.SetSensorValue( ID_TRIP, tripDist/10 );
			   jetiEx.SetSensorValue( ID_DIST, distToHome );
#endif
#ifdef UNIT_US
			   //US units
			   jetiEx.SetSensorValue( ID_TRIP, tripDist*0.06213 );
			   jetiEx.SetSensorValue( ID_DIST, distToHome*3.2808399);
#endif

		   }// if gps !disabled
#endif

#ifdef UNIT_US
		   // EU to US conversions
		   // ft/s = m/s / 0.3048
		   // inHG = hPa * 0,029529983071445
		   // ft = m / 0.3048
		   uRelAltitude /= 0.3048;
		   uAbsAltitude /= 0.3048;
#endif

		   jetiEx.SetSensorValue( ID_ALTREL, uRelAltitude );
		   jetiEx.SetSensorValue( ID_ALTABS, uAbsAltitude );

#ifdef SUPPORT_JETIBOX_MENU
		   HandleMenu();
#endif
		  jetiEx.DoJetiExBus(); //question indicator if sensor data were sent?
/* for testing without receiver only
		  avVario=0;
		  avPressure = 0;
		  avTemp = 0;
		  avAltitude = 0;
		  numVario = 0;
*/
	   }//while 1
   }//int main
