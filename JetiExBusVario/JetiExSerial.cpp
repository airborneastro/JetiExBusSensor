/* 
  Jeti Sensor EX Telemetry C++ Library
  
  JetiExSerial - EX serial output implementation for AtMega328
  --------------------------------------------------------------------
  
  Copyright (C) 2015 Bernd Wokoeck
  
  Version history:
  0.90   11/22/2015  created
  0.92   12/11/2015  baud rate for 8 MHz Pro mini (thanks to Wolfgang/wiff)
  0.94   12/22/2015  Teensy 3.x support on Serial2
  0.95   12/23/2015  Refactoring
  0.96   02/21/2016  comPort number as parameter for Teensy
  1.00   01/29/2017  Some refactoring:
                     - Optimized half duplex control for AVR CPUs in JetiExHardwareSerialInt class (for improved Jetibox key handling)
                     - Reduced size of serial transmit buffer (128-->64 words) 
                     - Changed bitrates for serial communication for AVR CPUs (9600-->9800 bps)
                     - JETI_DEBUG and BLOCKING_MODE removed (cleanup)
  1.0.1  02/15/2017  Support for ATMega32u4 CPU in Leonardo/Pro Micro
                     GetKey routine optimized 
  1.0.3  07/14/2017  Allow all jetibox key combinations (thanks to ThomasL)
                     Disable RX at startup to prevent reception of receiver identification

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

#include "JetiExSerial.h"

// Teensy
/////////
#ifdef CORE_TEENSY 

  JetiExSerial * JetiExSerial::CreatePort( int comPort )
  {
    return new JetiExTeensySerial( comPort );
  } 

  JetiExTeensySerial::JetiExTeensySerial( int comPort ) : m_bNextIsKey( false )
  {
    switch( comPort )
    {
    default: m_pSerial = &Serial2; break;
    case 2: m_pSerial = &Serial2; break;
    case 1: m_pSerial = &Serial1; break;
    case 3: m_pSerial = &Serial3; break;
    }
  }

  void JetiExTeensySerial::Init()
  {
//Karl start
//	m_pSerial->begin( 9600, SERIAL_9O1 );
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(KINETISL)
	  m_pSerial->begin( 9600, SERIAL_9O2 );
	  //9 bits plus parity plus 2 Stopbits is possible in MK20, MK64, MK 66 CPUs (Teensy 3.5, 3.6)
	  //see 52.4.4.3.5 Non-memory mapped tenth bit for parity on page 1597 of
	  //NXP programming manual K64P144M120SF5RM.pdf
	  //(K64 Sub-Family Reference Manual)
	  //see 49.4.5.3.5 in K20 Sub-Family Reference Manual for MK20 (Teensy 3.1)
#else
	  m_pSerial->begin( 9600, SERIAL_9O1 );
#endif

  }
  void JetiExTeensySerial::Send( uint8_t data, boolean bit8 )
  {
    uint32_t w = data | ( bit8 ? 0x100 : 0x000 );
    m_pSerial->write9bit(w);
  }
  uint8_t JetiExTeensySerial::Getchar(void) // you must modify \Arduino\hardware\teensy\avr\cores\teensy3\serial2.c in order to receive jeti keys 
  {                                         // refer to TeensyReadme.txt
    while( m_pSerial->available() > 0 )
    {
		  int c = m_pSerial->read();
      // Serial.print( c ); 
      // if( c == 0x70 || c == 0xb0 || c == 0xd0 || c == 0xe0 ) // filter for jetibox keys: Left = 0x70, down = 0xb0, up= 0xd0, right = 0xe0
      if( c != 0xf0 && (c & 0x0f) == 0 )   // check upper nibble
        return c;
    }
    return 0;
  }


#endif // CORE_TEENSY 
