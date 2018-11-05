/*
  -----------------------------------------------------------
            Menu handler for Jetibox
  -----------------------------------------------------------
*/
#include <avr/eeprom.h>

// Jetibox screen names
enum screenViews {
  aboutScreen,
  resetOffset,
  #ifdef SUPPORT_GPS
  setGpsMode,
  setDistanceMode,
  #endif
  detectedPressureSensor,
  setVarioSmoothingValue,
  setNormPress,
  setIntTime,
  #ifdef SUPPORT_MAIN_DRIVE
  setMainDrive,
  setCapacityMode,
  #endif
  #ifdef SUPPORT_RX_VOLTAGE
  enableRx1Voltage,
  enableRx2Voltage,
  #endif
  #ifdef SUPPORT_EXT_TEMP
  enableExternTemp,
  #endif
  saveSettings,
  defaultSettings
};


const char menuText[][17] PROGMEM=
{
  {"VarioGPS Sensor"},
  {"Reset offset"},
  #ifdef SUPPORT_GPS
  {"GPS mode:"},
  {"GPS distance:"},
  #endif
  {"Pressure sensor:"},
  {"Vario smoothing:"},
  {"Vario NormPress:"},
  {"Vario IntTime:  "},
  #ifdef SUPPORT_MAIN_DRIVE
  {"Main drive:"},
  {"Capacity reset:"},
  #endif
  #ifdef SUPPORT_RX_VOLTAGE
  {"Rx1 voltage:"},
  {"Rx2 voltage:"},
  #endif
  #ifdef SUPPORT_EXT_TEMP
  {"Ext. Temp:"},
  #endif
  {"Save and restart"},
  {"Load defaults"}
};

const char aboutScreenText[17] PROGMEM= {VARIOGPS_VERSION};

const char setGpsModeText[][10] PROGMEM=
{
  {" Disabled"},
  {" Basic"},
  {" Extended"}
};

const char setDistanceModeText[][4] PROGMEM=
{
  {" 3D"},
  {" 2D"}
};

const char detectedPressureSensorText[][9] PROGMEM=
{
  {" unknown"},
  {" BMP280"},
  {" BME280"},
  {" MS5611"},
  {" LPS"}
};

const char setMainDriveText[][16] PROGMEM=
{
  {" Disabled"},
  {" ACS709_BB35"},
  {" AttoPilot 45"},
  {" AttoPilot 90"},
  {" AttoPilot 180"},
  {" APM2.5 90A/50V"},
  #if V_REF > 4500
  {" ACS712-05"},
  {" ACS712-20"},
  {" ACS712-30"},
  #endif
  {" ACS758-50B"},
  {" ACS758-100B"},
  {" ACS758-150B"},
  {" ACS758-200B"},
  {" ACS758-50U"},
  {" ACS758-100U"},
  {" ACS758-150U"},
  {" ACS758-200U"}
};

const char setCapacityModeText[][9] PROGMEM=
{
  {" Startup"},
  {" Auto"},
  {" Manual"}
};

const char enableText[][10] PROGMEM=
{
  {" Disabled"},
  {" Enabled"}
};


void HandleMenu()
{
  static int  _nMenu = aboutScreen;
  static bool _bSetDisplay = true;
  static uint32_t LastKey;
  char _bufferLine1[17];
  char _bufferLine2[17];
  uint8_t c = jetiEx.GetJetiboxKey();

  enum
  {
    keyRight       = 0xe0,
    keyLeft        = 0x70,
    keyUp          = 0xd0,
    keyDown        = 0xb0,
    keyUpDown      = 0x90,
    keyLeftRight   = 0x60
  };

  if( c == 0 && !_bSetDisplay) return;

  if( millis() < LastKey )
    return;
  LastKey = millis() + 200;

  startHandleMenu:

  // Right
  if ( c == keyRight && _nMenu < defaultSettings)
  {
    _nMenu++;
    _bSetDisplay = true;
  }

  // Left
  if ( c == keyLeft &&  _nMenu > aboutScreen )
  {
    _nMenu--;
    _bSetDisplay = true;
  }

  // UP
  if ( c == keyUp )
  {
    switch ( _nMenu )
    {
      #ifdef SUPPORT_GPS
      case setGpsMode:
        if(gpsSettings.mode > GPS_disabled){
          gpsSettings.mode--;
        }
        break;
      #endif
      case setVarioSmoothingValue:
        if (pressureSensor.smoothingValue < 100) {
          pressureSensor.smoothingValue = (pressureSensor.smoothingValue+1);
        }
        break;
      case setNormPress:
        if (pressureSensor.normpress < 103900) { //78400 + 25500
          pressureSensor.normpress += 100;
        }
        break;
      case setIntTime:
    	  if (pressureSensor.inttime < 30) {
    		 pressureSensor.inttime +=1;
    	  }
    	  break;
      #ifdef SUPPORT_MAIN_DRIVE
      case setMainDrive:
        if (currentSensor > mainDrive_disabled) {
          currentSensor--;
        }
        break;
      case setCapacityMode:
        if(capacityMode > startup){
          capacityMode--;
        }
        break;
      #endif
    }

    _bSetDisplay = true;
  }

  // DN
  if ( c == keyDown )
  {
    switch ( _nMenu )
    {
      case resetOffset:

    	EEPROM.put(P_CAPACITY_VALUE, 0.0f);                 // reset capacity in eeprom
    	EEPROM.put(P_VOLT_VALUE, 0.0f);
    	resetFunc();
        break;
      #ifdef SUPPORT_GPS
      case setGpsMode:
        if(gpsSettings.mode < GPS_extended){
          gpsSettings.mode++;
        }
        break;
      case setDistanceMode:
        gpsSettings.distance3D = !gpsSettings.distance3D;
        break;
      #endif
      case setVarioSmoothingValue:
        if (pressureSensor.smoothingValue > 1) {
          pressureSensor.smoothingValue = (pressureSensor.smoothingValue-1);
        }
        break;
      case setNormPress:
        if (pressureSensor.normpress > 78400) {
          pressureSensor.normpress -= 100;  //in 100Pa-Schritten, im EEPROM in 1-er Schritten

        }
        break;
      case setIntTime:
          	  if (pressureSensor.inttime > 5) {
          		 pressureSensor.inttime -=1;
          	  }
          	  break;
      #ifdef SUPPORT_MAIN_DRIVE
      case setMainDrive:
        if (currentSensor < ACS758_200U) {
          currentSensor++;
        }
        break;
      case setCapacityMode:
        if(capacityMode < manual){
          capacityMode++;
        }
        break;
      #endif
      #ifdef SUPPORT_RX_VOLTAGE
      case enableRx1Voltage:
        enableRx1 = !enableRx1;
        break;
      case enableRx2Voltage:
        enableRx2 = !enableRx2;
        break;
      #endif
      #ifdef SUPPORT_EXT_TEMP
      case enableExternTemp:
        enableExtTemp = !enableExtTemp;
        break;
      #endif
      case saveSettings:
        #ifdef SUPPORT_GPS
    	EEPROM.write(P_GPS_MODE, gpsSettings.mode);
        EEPROM.write(P_GPS_3D, gpsSettings.distance3D);
        #endif

        #ifdef SUPPORT_MAIN_DRIVE
        EEPROM.write(P_CURRENT_SENSOR, currentSensor);
        EEPROM.write(P_CAPACITY_MODE, capacityMode);
        #endif

        #ifdef SUPPORT_RX_VOLTAGE
        EEPROM.write(P_ENABLE_RX1, enableRx1);
        EEPROM.write(P_ENABLE_RX2, enableRx2);
        #endif

        #ifdef SUPPORT_EXT_TEMP
        EEPROM.write(P_ENABLE_TEMP, enableExtTemp);
        #endif

        EEPROM.write(P_VARIO_SMOOTHING,uint8_t(pressureSensor.smoothingValue));
        EEPROM.write(P_VARIO_NORMPRESS,(pressureSensor.normpress-78400)/100);  //78400 = 2000m, EEPROM = 0
        EEPROM.write(P_VARIO_INTTIME,uint8_t(pressureSensor.inttime));
        resetFunc();
        break;
      case defaultSettings:
        for(int i=0; i < 50; i++){
          EEPROM.write(i, 0xFF);
        }
        EEPROM.put(P_CAPACITY_VALUE, 0.0f);
        EEPROM.put(P_VOLT_VALUE, 0.0f);
        resetFunc();
    }

    _bSetDisplay = true;
  }

  if ( !_bSetDisplay )
    return;

  // clear buffer
  _bufferLine1[0] = 0;
  _bufferLine2[0] = 0;

  memcpy_P( _bufferLine1, &menuText[_nMenu], 16 );

  switch ( _nMenu )
  {
    case aboutScreen:
      memcpy_P( _bufferLine2, aboutScreenText, 16 );
      break;
    #ifdef SUPPORT_GPS
    case setGpsMode:
      memcpy_P( _bufferLine2, &setGpsModeText[gpsSettings.mode], 16 );
      break;
    case setDistanceMode:
      if(gpsSettings.mode == GPS_disabled)goto startHandleMenu;
      memcpy_P( _bufferLine2, &setDistanceModeText[gpsSettings.distance3D], 16 );
      break;
    #endif
    case detectedPressureSensor:
      memcpy_P( _bufferLine2, &detectedPressureSensorText[pressureSensor.type], 16 );
      break;
    case setVarioSmoothingValue:
      if(pressureSensor.type == unknown)goto startHandleMenu;
      sprintf( _bufferLine2, " %2d%%",int(pressureSensor.smoothingValue));
      break;
    case setNormPress:
      if(pressureSensor.type == unknown)goto startHandleMenu;
      sprintf( _bufferLine2, " %6dPa",pressureSensor.normpress);
      break;
    case setIntTime:
          if(pressureSensor.type == unknown)goto startHandleMenu;
          sprintf( _bufferLine2, " %2d sec",pressureSensor.inttime);
          break;
    #ifdef SUPPORT_MAIN_DRIVE
    case setMainDrive:
      memcpy_P( _bufferLine2, &setMainDriveText[currentSensor], 16 );
      break;
    case setCapacityMode:
      if(currentSensor == mainDrive_disabled)goto startHandleMenu;
      memcpy_P( _bufferLine2, &setCapacityModeText[capacityMode], 16 );
      break;
    #endif
    #ifdef SUPPORT_RX_VOLTAGE
    case enableRx1Voltage:
      memcpy_P( _bufferLine2, &enableText[enableRx1], 16 );
      break;
    case enableRx2Voltage:
      memcpy_P( _bufferLine2, &enableText[enableRx2], 16 );
      break;
    #endif
    #ifdef SUPPORT_EXT_TEMP
    case enableExternTemp:
      memcpy_P( _bufferLine2, &enableText[enableExtTemp], 16 );
      break;
    #endif
  }

  jetiEx.SetJetiboxText( 0, _bufferLine1 );
  jetiEx.SetJetiboxText( 1, _bufferLine2 );

  _bSetDisplay = false;
}
