# JetiExBusVario
This branch uses the JetiExBus protocol (which also allows to query the "servo" channels) and is augmented with an integral vario function. The integration time can be set via the "JetiBox" function of the DC-16 transmitter


Jeti multi sensor, using a MS5611 or MS5803 pressure sensor for vario function,
GPS and more sensors for Jeti RC control, based on code by nightflyer88 (see cloned repository) and on the JetiExBus libraries by Bernd Wokoeck. This code is for a Teensy 3.5 board and an Eclipse IDE. Code was adapted to my purposes. The GPS module uses RX1, TX1 Pins 0,1, output to the EX receiver input uses RX2,TX2 pin 9,10 (tied together without resistor), the pressure sensor uses pin 3,4 as Wire2 or 37,38 as Wire1, depending on whoch sensor is #defined, the current sensor ACS709 uses A0 pin14 for VIOUT (voltage proportional to sensed current), VCZR on A4 pin 18 (not used in current code) LiPo voltage (added resistor voltage divider on ACS709 board) on A1 pin 15. 
