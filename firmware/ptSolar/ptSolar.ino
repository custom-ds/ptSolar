/*
Project: Traveler ptSolar Firmware
Copywrite 2011-2025 - Zack Clobes (W0ZC), Custom Digital Services, LLC


This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.


Before programming for the first time, the ATmega fuses must be set.
 Low:      0xDF
 High:     0xD6
 Extended: 0xFD

*/


#define FIRMWARE_VERSION "0.9.0"
#define CONFIG_VERSION "PT0100"
#define CONFIG_PROMPT "\n# "


// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>

#include "MemoryFree.h"
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "TNC.h"
#include "GPS.h"

#include "Custom.h"


#include "SparkFunBME280.h"


#include <Wire.h>

//PD0 is Serial Port RX
//PD1 is Serial Port TX
#define PIN_PTT_OUT 2     //PD2
#define PIN_AUDIO_OUT 3   //PD3   - APRS Packet Audio
#define PIN_DRA_EN 4      //PD4
#define PIN_AUDIO 5       //PD5   - Audio Annunciation
#define PIN_GPS_EN 6      //PD6
#define PIN_GPS_TX 7      //PD7

#define PIN_GPS_RX 8      //PB0
#define PIN_DRA_TX 9      //PB1
#define PIN_DRA_RX 10     //PB2
//PB3 is MOSI
//PB4 is MISO
#define PIN_LED 13        //PB5

//Analog Pins
#define PIN_AUDIO_IN A0   //PC0
#define PIN_ANALOG_BATTERY A1   //PC1
//PC2 is not used
//PC3 is not used
//PC4 is SDA
//PC5 is SCL
//PC6 is reset and not available



//How many MS to delay between subsequent packets (as in between GPGGA and GPRMC strings
#define DELAY_MS_BETWEEN_XMITS 1250
#define GPS_TIMEOUT_TIME 45000      //number of milliseconds to wait between error transmissions if the GPS fails
#define GPS_MAX_COLLECTION_TIME 3000    //number of millis to wait while collecting the two GPS strings.
#define METERS_TO_FEET 3.2808399

//Anunciator Settings
#define DELAY_DAH 650
#define DELAY_DIT 200
#define DELAY_GAP 150

struct udtConfig {
  char Callsign[7];    //6 digit callsign + Null
  char CallsignSSID;
  char Destination[7];
  char DestinationSSID;    //Destination SSID
  char Path1[7];
  char Path1SSID;
  char Path2[7];
  char Path2SSID;

  unsigned int DisablePathAboveAltitude;    //the altitude to stop sending path.  If 0, then always send path defined.

  char Symbol;
  char SymbolPage;

  byte BeaconType;    //0=seconds-delay, 1=Speed Smart Beaconing, 2=Altitude Smart Beaconing, 3=Time Slots, 4=Low-power mode
  unsigned long BeaconSimpleDelay;

  unsigned int BeaconAltitudeThreshLow;
  unsigned int BeaconAltitudeThreshHigh;
  unsigned long BeaconAltitudeDelayLow;
  unsigned long BeaconAltitudeDelayMid;
  unsigned long BeaconAltitudeDelayHigh;

  unsigned int BeaconSpeedThreshLow;
  unsigned int BeaconSpeedThreshHigh;
  unsigned long BeaconSpeedDelayLow;
  unsigned long BeaconSpeedDelayMid;
  unsigned long BeaconSpeedDelayHigh;

  byte BeaconSlot1;
  byte BeaconSlot2;

  byte AnnounceMode;    //0=None, 1=LED, 2=Audio, 3=LED+Audio

  byte GPSSerialBaud;    //1=300, 2=1200 3=2400 4=4800 5=9600 6=19200
  bool GPSSerialInvert;    //Invert the incoming serial string.
  byte GPSType;      //0=Generic NMEA, 1=UBlox, 2=ATGM332D

  char StatusMessage[41];
  bool StatusXmitGPSFix;
  bool StatusXmitBurstAltitude;
  bool StatusXmitBatteryVoltage;
  bool StatusXmitTemp;
  bool StatusXmitPressure;

  bool StatusXmitCustom;
  byte RadioType;
  unsigned int RadioTxDelay;
  bool RadioCourtesyTone;
  char RadioFreqTx[9];
  char RadioFreqRx[9];

  //Enable the BME280 temperature/pressure sensor
  bool i2cBME280;

  //Beacon Type 4 Settings
  unsigned int VoltThreshGPS;    //The voltage threshold to activate the GPS and read a position (in millivolts)
  unsigned int VoltThreshXmit;    //The voltage threshold to transmit a packet (in millivolts)
  unsigned int MinTimeBetweenXmits;    //The minimum time between transmissions (in seconds)
  

  unsigned int CheckSum;    //sum of the callsign element.  If it doesn't match, then it reinitializes the EEPROM
};
udtConfig Config;

GPS GPSParser;    //Object that parses the GPS strings
TNC oTNC;            //Object that assembles the packets for the TNC and transmits them

unsigned long timeLastXmit;    //Keeps track of the timestamp of the last transmission
unsigned long iLastErrorTxMillis;    //keep track of the timestamp of the last "lost GPS" transmission

bool bHasBurst;
float fMaxAlt;


BME280 Pressure;      //BMP280 pressure/temp sensor


//------------------------------------------ Variables for the internal modulation ------------------------------------------
#define BAUD_GENERATOR_COUNT 20

#define TONE_HIGH_STEPS_PER_TICK 6001
#define TONE_LOW_STEPS_PER_TICK 3273

//Sinewave lookup tables for high and low tones (the difference is the amplitude)
PROGMEM const unsigned char _arySineHigh[] = {128, 131, 134, 137, 140, 144, 147, 150, 153, 156,
	159, 162, 165, 168, 171, 174, 177, 179, 182, 185,
	188, 191, 193, 196, 199, 201, 204, 206, 209, 211,
	213, 216, 218, 220, 222, 224, 226, 228, 230, 232,
	234, 235, 237, 239, 240, 241, 243, 244, 245, 246,
	248, 249, 250, 250, 251, 252, 253, 253, 254, 254,
	254, 255, 255, 255, 255, 255, 255, 255, 254, 254,
	254, 253, 253, 252, 251, 250, 250, 249, 248, 246,
	245, 244, 243, 241, 240, 239, 237, 235, 234, 232,
	230, 228, 226, 224, 222, 220, 218, 216, 213, 211,
	209, 206, 204, 201, 199, 196, 193, 191, 188, 185,
	182, 179, 177, 174, 171, 168, 165, 162, 159, 156,
	153, 150, 147, 144, 140, 137, 134, 131, 128, 125,
	122, 119, 116, 112, 109, 106, 103, 100, 97, 94,
	91, 88, 85, 82, 79, 77, 74, 71, 68, 65,
	63, 60, 57, 55, 52, 50, 47, 45, 43, 40,
	38, 36, 34, 32, 30, 28, 26, 24, 22, 21,
	19, 17, 16, 15, 13, 12, 11, 10, 8, 7,
	6, 6, 5, 4, 3, 3, 2, 2, 2, 1,
	1, 1, 1, 1, 1, 1, 2, 2, 2, 3,
	3, 4, 5, 6, 6, 7, 8, 10, 11, 12,
	13, 15, 16, 17, 19, 21, 22, 24, 26, 28,
	30, 32, 34, 36, 38, 40, 43, 45, 47, 50,
	52, 55, 57, 60, 63, 65, 68, 71, 74, 77,
	79, 82, 85, 88, 91, 94, 97, 100, 103, 106,
	109, 112, 116, 119, 122, 125};


//Max 192 (3db down from High)
PROGMEM const unsigned char _arySineLow[] = {128, 129, 130, 132, 133, 134, 135, 136, 137, 139,
	140, 141, 142, 143, 144, 145, 146, 147, 149, 150,
	151, 152, 153, 154, 155, 156, 157, 158, 158, 159,
	160, 161, 162, 163, 164, 164, 165, 166, 167, 167,
	168, 169, 169, 170, 170, 171, 171, 172, 172, 173,
	173, 174, 174, 174, 175, 175, 175, 175, 175, 176,
	176, 176, 176, 176, 176, 176, 176, 176, 176, 176,
	175, 175, 175, 175, 175, 174, 174, 174, 173, 173,
	172, 172, 171, 171, 170, 170, 169, 169, 168, 167,
	167, 166, 165, 164, 164, 163, 162, 161, 160, 159,
	158, 158, 157, 156, 155, 154, 153, 152, 151, 150,
	149, 147, 146, 145, 144, 143, 142, 141, 140, 139,
	137, 136, 135, 134, 133, 132, 130, 129, 128, 127,
	126, 124, 123, 122, 121, 120, 119, 117, 116, 115,
	114, 113, 112, 111, 110, 109, 107, 106, 105, 104,
	103, 102, 101, 100, 99, 98, 98, 97, 96, 95,
	94, 93, 92, 92, 91, 90, 89, 89, 88, 87,
	87, 86, 86, 85, 85, 84, 84, 83, 83, 82,
	82, 82, 81, 81, 81, 81, 81, 80, 80, 80,
	80, 80, 80, 80, 80, 80, 80, 80, 81, 81,
	81, 81, 81, 82, 82, 82, 83, 83, 84, 84,
	85, 85, 86, 86, 87, 87, 88, 89, 89, 90,
	91, 92, 92, 93, 94, 95, 96, 97, 98, 98,
	99, 100, 101, 102, 103, 104, 105, 106, 107, 109,
	110, 111, 112, 113, 114, 115, 116, 117, 119, 120,
	121, 122, 123, 124, 126, 127};

//------------------------------------------ END Variables for the internal modulation ------------------------------------------



void setup() {

  //Configure the Transmitter Pins
	pinMode(PIN_PTT_OUT, OUTPUT);
  pinMode(PIN_DRA_EN, OUTPUT);
  pinMode(PIN_AUDIO_OUT, OUTPUT);

  //Configure the GPS Pins
  pinMode(PIN_GPS_EN, OUTPUT);

  //Shut everything down until we get booted
  digitalWrite(PIN_PTT_OUT, LOW);    //Stop the transmit - 
  digitalWrite(PIN_DRA_EN, LOW);    //disable the transmitter
  digitalWrite(PIN_GPS_EN, LOW);    //disable the GPS

  //Configure the Misc Pins
  pinMode(PIN_LED, OUTPUT);
  //pinMode(PIN_AUDIO, OUTPUT);
  
  Serial.begin(19200);

	//Define as internally moduled TNC
  oTNC.initInternal(PIN_PTT_OUT);


  Serial.println(F("pt Flight Computer"));

  Serial.print(F("Firmware Version: "));
  Serial.println(FIRMWARE_VERSION);


  //Init some variables
  fMaxAlt = 0;
  bHasBurst = false;
  timeLastXmit = 0;


  getConfigFromEeprom();

  annunciate('k');

  
  oTNC.setTransmitterType(Config.RadioType);
  oTNC.setTxDelay(Config.RadioTxDelay);
  oTNC.setCourtesyTone(Config.RadioCourtesyTone);


  //init the I2C devices
  if (Config.i2cBME280 == 1) {
    //we're supposed to initialize the BME280
    Serial.println(F("Init BME280"));
    Pressure.setI2CAddress(0x76);
    if (Pressure.beginI2C() == false) //Begin communication over I2C
    {
      Serial.println(F(" Could NOT init!"));
    }
  } else {
    Serial.println(F("No I2C devices to init"));
  }
  
  


  //Check to see if we're going into config mode
  byte byTemp;
  while (millis() < 5000) {
    if (Serial.available() > 0) {
      // read the incoming byte:
      byTemp = Serial.read();

      if (byTemp == '!') {
        doConfigMode();
      }
    }
  }


  customInit();   //Call any custom code to init sensors, initialize variables, etc.

  //Send out an initial packet announcing itself.
  // if (Config.RadioType == 1) initDRA818();    //Configure the transmitter to correct frequency
  // oTNC.xmitStart(Config.Destination, Config.DestinationSSID, Config.Callsign, Config.CallsignSSID, Config.Path1, Config.Path1SSID, Config.Path2, Config.Path2SSID, true);
  // oTNC.xmitString((char *)">Project Traveler ptSolar Flight Computer v");
  // oTNC.xmitString((char *)FIRMWARE_VERSION);
  // oTNC.xmitString((char *)" Initializing...");
  // oTNC.xmitEnd();

  //init the GPS into high altitude mode (Dynamic Model 6 – Airborne < 1g)
  initGPS(Config.GPSType);    //will continually retry this operation until its sucessful
  
  GPSParser.OutputNEMA(true);    ///TODO: Need to pull this from Configuration

  iLastErrorTxMillis = millis();      //set a starting time for the potential error messages
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  float fCurrentAlt, fSpeed, fMaxSpeed, fBattery;
  bool bXmit;
  int iSeconds;
  unsigned long msDelay;    //calculate the number of milliseconds to delay

  //collectGPSStrings();      //listen to the GPS for up to 3 seconds (the function will exit out as soon as a pair of RMC and GGA strings are received)

  fBattery = readBatteryVoltage();
  Serial.print(F("Battery: "));
  Serial.print(fBattery);
  Serial.println("V");

  //check to see if we have sufficient battery to run the GPS
  if (fBattery > 4.0) {
    Serial.println(F("Batt > 4. Checking the GPS"));
    collectGPSStrings();
  } else {
    Serial.println(F("Batt < 4.0.  Not checking GPS"));
    delay(750);   //wait for about the amount of time that we'd normally spend grabbing a GPS reading
  }

  //Check to see if we've decoded a GPS packet recently.
  if ((GPSParser.LastDecodedMillis() + GPS_TIMEOUT_TIME) < millis()) {
    //we haven't decoded anything from the GPS in 45 seconds - we have a problem here

    if ((iLastErrorTxMillis + GPS_TIMEOUT_TIME) < millis()) {
      //it's been 45 seconds since the last time that we transmitted an error - so transmit

      annunciate('g');
      if (Config.RadioType == 1) initDRA818();    //Configure the transmitter to correct frequency
      oTNC.xmitStart(Config.Destination, Config.DestinationSSID, Config.Callsign, Config.CallsignSSID, Config.Path1, Config.Path1SSID, Config.Path2, Config.Path2SSID, true);
      oTNC.xmitString((char *)">Lost GPS for over 45 seconds!");
      oTNC.xmitEnd();

      iLastErrorTxMillis = millis();      //track the fact that we just transmitted
    }
  }

  fCurrentAlt = GPSParser.Altitude();        //get the current altitude
  if (fCurrentAlt > fMaxAlt) {
    fMaxAlt = fCurrentAlt;
  } else {
    if (fMaxAlt > 10000 && (fCurrentAlt < (fMaxAlt - 250))) {
      //Check for burst.  The Burst must be at least 10,000m MSL.
      // To sense a burst, the controller must have fallen at least 250m from the max altitude

      bHasBurst = true;
    }
  }

  bXmit = false;    //assume that we won't transmit this time around

  //Figure out how long to delay before the next packet
  switch (Config.BeaconType) {
  case 0:
    //This is no logic to beacon intervals - just plan old time delays
    msDelay = (unsigned long)Config.BeaconSimpleDelay * 1000;    //cast this to unsigned long
    
     if ((millis() - timeLastXmit) > msDelay) {
      //we've waited long enough - transmit
      bXmit = true;
    }

    break;
  case 1:
    //This is for Speed-based beaconing

    fSpeed = GPSParser.Knots();        //get the current speed
    if (fSpeed > fMaxSpeed) fMaxSpeed = fSpeed;

    if (fMaxSpeed < Config.BeaconSpeedThreshLow) {
      //we're in the slow range
      msDelay = (unsigned long)Config.BeaconSpeedDelayLow * 1000;    //cast this to unsigned long
      
      if ((millis() - timeLastXmit) > msDelay) {
        //we've waited long enough for this speed - transmit
        bXmit = true;
      }
    }

    if (fSpeed >= Config.BeaconSpeedThreshLow && fSpeed < Config.BeaconSpeedThreshHigh) {
      //we're in the medium range
      msDelay = (unsigned long)Config.BeaconSpeedDelayMid * 1000;    //cast this to unsigned long
      
      if ((millis() - timeLastXmit) > msDelay) {
        //we've waited long enough for this speed - transmit
        bXmit = true;
      }
    }

    if (fSpeed >= Config.BeaconSpeedThreshHigh) {
      //we're in the fast range
      msDelay = (unsigned long)Config.BeaconSpeedDelayHigh * 1000;    //cast this to unsigned long
      
      if ((millis() - timeLastXmit) > msDelay) {
        //we've waited long enough for this speed - transmit
        bXmit = true;
      }
    }

    break;
  case 2:
    //This is for Altitude-based beaconing

    if (fCurrentAlt < Config.BeaconAltitudeThreshLow) {
      //we're in the low phase of the flight - we'll typically send packets more frequently close to the ground
      msDelay = (unsigned long)Config.BeaconAltitudeDelayLow * 1000;    //cast this to unsigned long
      
      if ((millis() - timeLastXmit) > msDelay) {
        //we've waited long enough for this speed - transmit
        bXmit = true;
      }
    }
    if (fCurrentAlt >= Config.BeaconAltitudeThreshLow && fCurrentAlt < Config.BeaconAltitudeThreshHigh) {
      //we're in the mid-phase of the flight.  We'll transmit regularly in here
      msDelay = (unsigned long)Config.BeaconAltitudeDelayMid * 1000;    //cast this to unsigned long
      
      if ((millis() - timeLastXmit) > msDelay) {
        //we've waited long enough for this speed - transmit
        bXmit = true;
      }
    }
    if (fCurrentAlt >= Config.BeaconAltitudeThreshHigh) {
      //we're in the top-phase of the flight.  Transmit more frequenly to get better burst resolution?
      msDelay = (unsigned long)Config.BeaconAltitudeDelayHigh * 1000;    //cast this to unsigned long
      
      if ((millis() - timeLastXmit) > msDelay) {
        //we've waited long enough for this speed - transmit
        bXmit = true;
      }
    }

    break;
  case 3:
    //Use Time Slotting to determine when to transmit
    iSeconds = GPSParser.getGPSSeconds();

    if (iSeconds == Config.BeaconSlot1 || iSeconds == (Config.BeaconSlot1 + 1) || iSeconds == Config.BeaconSlot2 || iSeconds == (Config.BeaconSlot2 + 1)) {
      bXmit = true;
    }

    break;
  case 4:
    //This is a voltage-checked time delay.  It will wait X seconds, but then also wait for the system (solar) voltage to be above a threshold before transmitting
    msDelay = (unsigned long)Config.BeaconSimpleDelay * 1000;    //cast this to unsigned long
    
     if ((millis() - timeLastXmit) > msDelay) {
      //we've waited long enough - see if we have the power to transmit
      

      if (fBattery > 3.0) {
        Serial.println("Transmitting");
        bXmit = true;
      } else {
        Serial.println("Low volts - no xmit");
      }
    }

    break;    
  }

  if (customLoop(GPSParser)) {
    bXmit = true;   //allow the customLoop function to set an xmit flag true
  }

  if (bXmit) {
    //we're supposed to transmit now
    sendPositionSingleLine();

    timeLastXmit = millis();
    fMaxSpeed = 0;    //reset the max speed to check again this next cycle (Used for Speed-based smart beaconing)

    delay(DELAY_MS_BETWEEN_XMITS);    //delay about a second - if you don't you can run into multiple packets inside of a 2 second window

    if (!GPSParser.FixQuality() || GPSParser.NumSats() < 4) {
      //we are having GPS fix issues - issue an annunciation
      annunciate('l');
    }
  }
  //see if we're tracking free memory (debugging)
  #ifdef  MEMORY_FREE_H
    Serial.print(F("Free Mem: "));
    Serial.println(freeMemory());
  #endif  
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void sendPositionSingleLine() {
  char szTemp[15];    //largest string held should be the longitude
  int i;
  double insideTemp;    //inside air temp
  
  double airPressure;    //millibars
  float fTemp;    //temporary variable

  char statusIAT = 0;
  
  
  
  if (Config.StatusXmitPressure || Config.StatusXmitTemp) {
    //we're supposed to transmit the air pressure and/or temp - go ahead and pre-fetch it


	  airPressure = (double)Pressure.readFloatPressure();
	  airPressure = airPressure / 100;    //convert back to simple airpressure in hPa
	  insideTemp = (double)Pressure.readTempC();

  }

  if (Config.RadioType == 1) initDRA818();    //Configure the transmitter to correct frequency
  oTNC.xmitStart(Config.Destination, Config.DestinationSSID, Config.Callsign, Config.CallsignSSID, Config.Path1, Config.Path1SSID, Config.Path2, Config.Path2SSID, (GPSParser.Altitude() < Config.DisablePathAboveAltitude));

  //      /155146h3842.00N/09655.55WO301/017/A=058239
  int hh = 0, mm = 0, ss = 0;
  GPSParser.getGPSTime(&hh, &mm, &ss);
  oTNC.xmitString((char *)"/");

  sprintf(szTemp, "%02d", hh);
  oTNC.xmitString(szTemp);
  sprintf(szTemp, "%02d", mm);
  oTNC.xmitString(szTemp);
  sprintf(szTemp, "%02d", ss);
  oTNC.xmitString(szTemp);

  oTNC.xmitString((char *)"h");
  //Latitude
  GPSParser.getLatitude(szTemp);
  i=0;

  while (i<7 && szTemp[i]) {
    oTNC.xmitChar(szTemp[i]);
    i++;
  }
  oTNC.xmitChar(GPSParser.LatitudeHemi());
  oTNC.xmitChar(Config.SymbolPage);

  //Longitude
  GPSParser.getLongitude(szTemp);
  i=0;
  while (i<8 && szTemp[i]) {
    oTNC.xmitChar(szTemp[i]);
    i++;
  }
  oTNC.xmitChar(GPSParser.LongitudeHemi());

  oTNC.xmitChar(Config.Symbol);

  //Course
  fTemp = GPSParser.Course();

  sprintf(szTemp, "%03d", (int)fTemp);
  oTNC.xmitString(szTemp);
  oTNC.xmitChar('/');

  //Speed in knots
  fTemp = GPSParser.Knots();

  sprintf(szTemp, "%03d", (int)fTemp);
  oTNC.xmitString(szTemp);

  oTNC.xmitString((char *)"/A=");
  //Altitude in Feet
  fTemp = GPSParser.AltitudeInFeet();
  oTNC.xmitLong((long)fTemp, true);

  if (Config.StatusXmitGPSFix) {
    //Fix quality and num sats

    if (GPSParser.FixQuality() >= 1 && GPSParser.FixQuality() <=3) {
      //we have a GPS, DGPS, or PPS fix
      oTNC.xmitString((char *)" 3D");
    } else {
      oTNC.xmitString((char *)" na");
    }

    sprintf(szTemp, "%dSats", GPSParser.NumSats());
    oTNC.xmitString(szTemp);
  }
  if (Config.StatusXmitBatteryVoltage) {

    oTNC.xmitString((char *)" Vb=");
    oTNC.xmitFloat(readBatteryVoltage());
  }

  if (Config.StatusXmitTemp) {
    oTNC.xmitString((char *)" IAT=");
    oTNC.xmitFloat((float)insideTemp);
  }

  if (Config.StatusXmitPressure) {
    oTNC.xmitString((char *)" Press=");
    oTNC.xmitFloat((float)airPressure);
  }

  if (Config.StatusXmitBurstAltitude && bHasBurst) {
    oTNC.xmitString((char *)" Burst=");
    fTemp = fMaxAlt * METERS_TO_FEET;
    oTNC.xmitLong((long)fTemp, true);
  }

  customSendPositionSingleLine(Config.StatusXmitCustom, oTNC, GPSParser);     //Xmit any custom telemetry data
  
  oTNC.xmitChar(' ');
  oTNC.xmitString(Config.StatusMessage);

  oTNC.xmitEnd();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void collectGPSStrings() {
  SoftwareSerial GPS(PIN_GPS_RX, PIN_GPS_TX, Config.GPSSerialInvert);    //A True at the end indicates that the serial data is inverted.

  //figure out the baud rate for the data coming in
  switch (Config.GPSSerialBaud) {
    case 0x01:
      GPS.begin(300);
      break;
    case 0x02:
      GPS.begin(1200);
      break;
    case 0x03:
      GPS.begin(2400);
      break;
    case 0x04:
      GPS.begin(4800);
      break;
    case 0x05:
      GPS.begin(9600);
      break;
    case 0x06:
      GPS.begin(19200);
      break;
  }

  digitalWrite(PIN_GPS_EN, HIGH);    //enable the GPS

  GPSParser.clearInputBuffer();
  GPSParser.ClearSentenceFlags();      //clear out the temporary flags to indicate that the new sentences have come in


  //keep track of how long we can listen to the GPS
  unsigned long ulUntil = millis() + GPS_MAX_COLLECTION_TIME;


  while (millis() < ulUntil ) {
    //need to continue looping even if the data isn't coming in.

    //see if there's some new GPS data available
    if (GPS.available()) {
      byte c = GPS.read();

      GPSParser.addChar(c);

      //check the sentence flags to see if both RMC and GGA's have been received in this session
      if (GPSParser.GotNewRMC() && GPSParser.GotNewGGA()) {
        //we got new GGA and RMC strings - exit out now rather than waiting the whole alloted period.
        return;
      }
    }
  }


  digitalWrite(PIN_GPS_EN, LOW);    //shut the GPS back down

  return;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void annunciate(char c) {
  //send an anunciator via LED and/or buzzer, depending on config

  switch (c) {
  case 'c':
  	//Used when entering configuration mode
    audioTone(DELAY_DAH);
    delay(DELAY_GAP);
    audioTone(DELAY_DIT);
    delay(DELAY_GAP);
    audioTone(DELAY_DAH);
    delay(DELAY_GAP);
    audioTone(DELAY_DIT);
    break;
  case 'e':
    //single chirp - Used during initialization of uBlox GPS
    audioTone(DELAY_DIT);
    break;
  case 'g':
  	//Used during complete loss of GPS signal
    audioTone(DELAY_DAH);
    delay(DELAY_GAP);
    audioTone(DELAY_DAH);
    delay(DELAY_GAP);
    audioTone(DELAY_DIT);
    break;
  case 'i':
    //double chirp - Used to confirm initialization of uBlox GPS
    audioTone(DELAY_DIT);
    delay(DELAY_GAP);
    audioTone(DELAY_DIT);
    break;
  case 'k':
  	//Initial "OK"
    audioTone(DELAY_DAH);
    delay(DELAY_GAP);
    audioTone(DELAY_DIT);
    delay(DELAY_GAP);
    audioTone(DELAY_DAH);
    break;
  case 'l':
  	//Used during loss of GPS lock
    audioTone(DELAY_DIT);
    delay(DELAY_GAP);
    audioTone(DELAY_DAH);
    delay(DELAY_GAP);
    audioTone(DELAY_DIT);
    delay(DELAY_GAP);
    audioTone(DELAY_DIT);
    break;
  case 't':
    //Exercising and testing the Transmitter
    audioTone(DELAY_DAH);
    break;
  case 'w':
    //Indicates that configuration settings were written to EEPROM
    audioTone(DELAY_DIT);
    delay(DELAY_GAP);
    audioTone(DELAY_DAH);
    delay(DELAY_GAP);
    audioTone(DELAY_DAH);
    break;
  case 'x':
    //Used when exercising the board to test for functionality
    audioTone(DELAY_DAH);
    delay(DELAY_GAP);
    audioTone(DELAY_DIT);
    delay(DELAY_GAP);
    audioTone(DELAY_DIT);
    delay(DELAY_GAP);
    audioTone(DELAY_DAH);
    break;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void audioTone(int length) {

  //The Config parameter Config.AnnounceMode determines what this function does.  If bit 0 is set, then the LED will flash.
  // If the bit 1 is set, then it will pulse the audio annunciator.  Both or neither bits can also be set and behave accordingly.
  if (Config.AnnounceMode & 0x01) {
    digitalWrite(PIN_LED, HIGH);
  }

  for (int i = 0; i<length; i++) {
    if (Config.AnnounceMode & 0x02) {
      digitalWrite(PIN_AUDIO, HIGH);
    }
    delayMicroseconds(200);

    if (Config.AnnounceMode & 0x02) {
      digitalWrite(PIN_AUDIO, LOW);
    }
    delayMicroseconds(200);
  }

  if (Config.AnnounceMode & 0x01) {
    digitalWrite(PIN_LED, LOW);
  }

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void initDRA818(void) {

  Serial.println(F("Enabling the 818-V"));
  digitalWrite(PIN_DRA_EN, HIGH);
  
  Serial.println(F("Init DRA818"));
  SoftwareSerial DRA(PIN_DRA_RX, PIN_DRA_TX, false);    //A True at the end indicates that the serial data is inverted.
  DRA.begin(9600);
  delay(100);
  
  DRA.print(F("AT+DMOCONNECT\r\n"));
  Serial.println(F(" Connected"));
  delay(350);
  DRA.print(F("AT+DMOSETGROUP=0,"));
  DRA.print(Config.RadioFreqTx);
  DRA.print(",");
  DRA.print(Config.RadioFreqRx);
  DRA.print(F(",0000,4,0000\r\n"));


  delay(100);
  
  DRA.print(F("AT+DMOCONNECT\r\n"));
  Serial.println(F(" Connected"));
  delay(350);
  DRA.print(F("AT+DMOSETGROUP=0,"));
  DRA.print(Config.RadioFreqTx);
  DRA.print(",");
  DRA.print(Config.RadioFreqRx);
  DRA.print(F(",0000,4,0000\r\n"));
  

  Serial.print(F(" Freq: "));
  Serial.println(Config.RadioFreqTx);
  delay(1000);      //wait for transmitter to change frequency

  //Cycle the transmitter quickly.  It seems to take a long time to transmit the first time after inint
  // oTNC.keyTransmitter(true);
  // delay(250);   //not even long enough to actually key up...
  // oTNC.keyTransmitter(false);
  
  //disable the DRA until we're ready to xmit
  Serial.println(F("Powering down the 818-V"));
  digitalWrite(PIN_DRA_EN, LOW);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void initGPS(int GPSType) {
  bool bSuccess = false;

  while(!bSuccess) {
    annunciate('e');    //chirp
    if (GPSType == 1) {
      bSuccess = ubloxSendUBX();
    }
    if (GPSType == 2) {
      //ATGM332D
      bSuccess = atgmSendInit();
    }
  }
  annunciate('i');    //double chirp for success
}

/**
 * @brief  Initialize the uBlox GPS module.
 * @return true if the GPS initialized successfully, false if it did not.
 */
bool ubloxSendUBX() {
  //start up the GPS serial port - always use 9600 to init the UBlox
  SoftwareSerial GPS(PIN_GPS_RX, PIN_GPS_TX, Config.GPSSerialInvert);    //A True at the end indicates that the serial data is inverted.
  GPS.begin(9600);

  
  byte setdm6[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
                   0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };

  byte ackByteID = 0;
  byte ackPacket[10];


  //calculate a response checksum to verify that the config was sent correct.
  // Construct the expected ACK packet
  ackPacket[0] = 0xB5; // header
  ackPacket[1] = 0x62; // header
  ackPacket[2] = 0x05; // class
  ackPacket[3] = 0x01; // id
  ackPacket[4] = 0x02; // length
  ackPacket[5] = 0x00;
  ackPacket[6] = setdm6[2]; // ACK class
  ackPacket[7] = setdm6[3]; // ACK id
  ackPacket[8] = 0; // CK_A
  ackPacket[9] = 0; // CK_B

  // Calculate the checksums
  for (byte i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

	Serial.println(F("Init uBlox"));
  //send the config to the GPS
  GPS.flush();
  GPS.write(0xFF);
  delay(500);

  for (byte i=0; i<44; i++) {
    GPS.write(setdm6[i]);
  }

  //keep track of how long we can listen to the GPS
  unsigned long ulUntil = millis() + 3000;

  while (millis() < ulUntil ) {
    // Test for success
    if (ackByteID > 9) return true;    //we had all 9 bytes come back through - valid response!!!

    // Make sure data is available to read
    if (GPS.available()) {
      byte c = GPS.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (c == ackPacket[ackByteID]) {
        ackByteID++;
      } else {
        ackByteID = 0; // Reset and look again, invalid order
      }
    }
  }
  return false;    //timed out
}

/**
 * @brief  Initialize the ATGM332D GPS module.
 * @return true if the GPS initialized successfully, false if it did not.
 */
bool atgmSendInit() {
  SoftwareSerial GPS(PIN_GPS_RX, PIN_GPS_TX, Config.GPSSerialInvert);    //A True at the end indicates that the serial data is inverted.
  GPS.begin(9600);

	Serial.println(F("Init atgm GPS"));

  return true;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void getConfigFromEeprom() {
  for (unsigned int i=0; i<sizeof(Config); i++) {
    *((char*)&Config + i) = EEPROM.read(i);
  }

  //Check to see if the EEPROM appears to be valid
  unsigned int iCheckSum = 0;
  for (int i=0; i<7; i++) {
    iCheckSum += Config.Callsign[i];
  }

  Serial.println(F("Read EEPROM"));
  Serial.println(Config.Callsign);

  if (iCheckSum != Config.CheckSum) {
    Serial.println(F("Checksum fail. Resetting."));

    //we do NOT have a match - reset the Config variables
    setDefaultConfig();
  }

}
/**
 * @brief  Write the default configuration to the EEPROM.
 * @note   Used when the EEPROM is not initialized or has been corrupted. Checksum is based on the string inside of Config.Callsign.
 */

void setDefaultConfig() {
  strcpy(Config.Callsign, "N0CALL");
  Config.CallsignSSID = '0';
  strcpy(Config.Destination, "APRS  ");
  Config.DestinationSSID = '0';
  strcpy(Config.Path1, "WIDE2 ");
  Config.Path1SSID = '1';
  strcpy(Config.Path2, "      ");
  Config.Path2SSID = '0';
  Config.DisablePathAboveAltitude = 2000;
  Config.Symbol = 'O';    //letter O for balloons
  Config.SymbolPage = '/';
  Config.BeaconType = 4;    //Solar (voltgage) delay
  Config.BeaconSimpleDelay = 30;
  Config.BeaconSpeedThreshLow = 20;
  Config.BeaconSpeedThreshHigh = 50;
  Config.BeaconSpeedDelayLow = 300;
  Config.BeaconSpeedDelayMid = 60;
  Config.BeaconSpeedDelayHigh = 120;
  Config.BeaconAltitudeThreshLow = 5000;
  Config.BeaconAltitudeThreshHigh = 20000;
  Config.BeaconAltitudeDelayLow  = 30;
  Config.BeaconAltitudeDelayMid  = 60;
  Config.BeaconAltitudeDelayHigh = 45;
  Config.BeaconSlot1 = 15;
  Config.BeaconSlot2 = 45;
  strcpy(Config.StatusMessage, "Project Traveler ptSolar");
  Config.StatusXmitGPSFix = 1;
  Config.StatusXmitBurstAltitude = 1;
  Config.StatusXmitBatteryVoltage = 1;
  Config.StatusXmitTemp = 1;
  Config.StatusXmitPressure = 1;
	Config.StatusXmitCustom;
	
	Config.RadioType = 1;   //DRA818V transmitter
	Config.RadioTxDelay = 50;
	Config.RadioCourtesyTone = 0;
  strcpy(Config.RadioFreqTx, "144.3900");
	strcpy(Config.RadioFreqRx, "144.3900");

  Config.GPSSerialBaud = 5;    //1=300, 2=1200, 3=2400, 4=4800, 5=9600, 6=19200
  Config.GPSSerialInvert = 0;    //Invert the incoming signal
  Config.GPSType = 2;      //0=Generic NMEA, 1=UBlox, 2=ATGM332D
  Config.AnnounceMode = 1;

  Config.i2cBME280 = 0;    //initialize the BME280

  Config.VoltThreshGPS = 3500;    //3.5V
  Config.VoltThreshXmit = 4000;    //4.0V
  Config.MinTimeBetweenXmits = 30;    //30 seconds



  Config.CheckSum = 410;		//Checksum for N0CALL

  writeConfigToEeprom();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void writeConfigToEeprom() {
  for (unsigned int i=0; i<sizeof(Config); i++) {
    EEPROM.write(i, *((char*)&Config + i));
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void doConfigMode() {
  byte byTemp;

  Serial.println(F("pt Flight Computer"));
  Serial.print(F("Firmware Version: "));
  Serial.print((char *)FIRMWARE_VERSION);
  Serial.print(F("   Config Version: "));
  Serial.println(CONFIG_VERSION);
  Serial.print(CONFIG_PROMPT);

  delay(750);
  annunciate('c');

  while (byTemp != 'Q') {
    if (Serial.available()) {
      byTemp = Serial.read();

      if (byTemp == '!') {
        Serial.println(F("pt Flight Computer"));
        Serial.print(F("Firmware Version: "));
        Serial.print((char *)FIRMWARE_VERSION);
        Serial.print(F("   Config Version: "));
        Serial.println(CONFIG_VERSION);
      }


      if (byTemp == 'R' || byTemp == 'r') {
        getConfigFromEeprom();    //pull the configs from eeprom
        sendConfigToPC();
      }

      if (byTemp == 'W' || byTemp == 'w') {
        //take the incoming configs and load them into the Config UDT

        Serial.println(F("Entering config write mode..."));

        if (getConfigFromPC()) {
          Serial.println(F("Done reading in configuration data."));

          writeConfigToEeprom();
          Serial.println(F("Written config to eeprom."));

          annunciate('w');
        } else {
          //something failed during the read of the config data
          Serial.println(F("Failure to read in configuration data..."));
        }
      }
      
      if (byTemp == 'D' || byTemp == 'd') {
        //used to reset the tracker back to N0CALL defaults
        Serial.println(F("Reset defaults"));
        setDefaultConfig();        
        annunciate('w');
      }

      if (byTemp == 'T' || byTemp == 't') {
        //exercise the transmitter
        Serial.println(F("Test Transmit"));
        Serial.println(F(""));
        Serial.println(F("1. - 1.5 Seconds"));
        Serial.println(F("2. - 10 Seconds"));
        Serial.println(F("3. - 30 Seconds"));
        Serial.println(F("4. - 60 Seconds"));
        Serial.println(F("5. - 120 Seconds"));

        while (!Serial.available()) {
          //Wait for an input
        }
        byTemp = Serial.read();

        if (byTemp >= '1' && byTemp <= '5') {
          oTNC.setTransmitterType(Config.RadioType);
          oTNC.setTxDelay(Config.RadioTxDelay);
          oTNC.setCourtesyTone(Config.RadioCourtesyTone);
      
          if (Config.RadioType == 1) {
            //this is DRA818
            initDRA818();
          }
          
          annunciate('t');
          oTNC.keyTransmitter(true);
          switch (byTemp) {
          case '1':
            Serial.println(F("1.5 sec..."));
            delay(1500);
            break;
          case '2':
            Serial.println(F("10 sec..."));
            delay(10000);
            break;
          case '3':
            Serial.println(F("30 sec..."));
            delay(30000);
            break;
          case '4':
            Serial.println(F("60 sec..."));
            delay(60000);
            break;
          case '5':
            Serial.println(F("120 sec..."));
            delay(120000);
            break;
          default:
            Serial.println(F("Unknown"));
          }

          oTNC.keyTransmitter(false);
        } else {
          Serial.println(F("Cancelling..."));
        }
      }
      
      if (byTemp == 'E' || byTemp == 'e') {
        //exercise mode to check out all of the I/O ports
        
        Serial.println(F("Exercise"));
        
        Serial.println(F(" annun"));
        Config.AnnounceMode = 0x03;    //temporarily set the announce mode to both
        annunciate('x');
        
        Serial.print(F("Batt: "));
        Serial.println(readBatteryVoltage());

        
  
        collectGPSStrings();   //check the GPS  
  
        double insideTemp;    //inside air temp
        double airPressure;    //millibars
        char status;
  

        airPressure = (double)Pressure.readFloatPressure();
        airPressure = airPressure / 100;    //convert back to simple airpressure in hPa
        insideTemp = (double)Pressure.readTempC();
        //insideTemp = insideTemp / 100;    //convert back to decimal

        Serial.print(F("IAT: "));
        Serial.println(insideTemp);
        Serial.print(F("Press: "));
        Serial.println(airPressure);   
        
        customExercise();
      }

      Serial.print(CONFIG_PROMPT);
    }
  }
  Serial.println(F("Exiting config mode..."));
}


/**
 * @brief Reads in serial line data from the PC until it finds a tab (0x09) or an End of Transmission (0x04) character.
 * @param szParam - The array to store the incoming data while it's being collected. This parameter is by reference and will be modified.
 * @param iMaxLen - The maximum length of the incoming data. Any data exceeding the iMaxLen will be discarded.
 * @note  The function will timeout if it doesn't receive anything within 1 second.
 */
void readConfigParam(char *szParam, int iMaxLen) {
  byte c;
  int iSize;
  unsigned long iMilliTimeout = millis() + 1000;    //wait up to 1 second for this data

  for (iSize=0; iSize<iMaxLen; iSize++) szParam[iSize] = 0x00;    //load the array with nulls just in case we don't find anything
  iSize = 0;    //reset to start counting up for real

  //Serial.println("W: ");
  while (millis() < iMilliTimeout) {

    if (Serial.available()) {
      c = Serial.read();

      if (c == 0x09 || c == 0x04) {
        //this is the end of a data set

        return;
      }
      if (iSize < iMaxLen) {
        //only add to the return array IF there's room.  Even if there's not room, continue to parse the incoming data until a tab is found.
        szParam[iSize] = c;
        iSize++;
      }
    }
  }


  Serial.println(F("timeout"));
}

/**
 * @brief Reads in the configuration data from the PC and loads it into the Config UDT.
 * @return True if the configuration data was successfully read in.  False if there was an error.
 * @note  
 */
bool getConfigFromPC() {

  char szParam[45];
  unsigned long iMilliTimeout = millis() + 10000;    //wait up to 10 seconds for this data

  while (millis() < iMilliTimeout) {

    while (!Serial.available()) {
      //wait
    }

    if (Serial.read() == 0x01) {
      Serial.println(F("Reading..."));

      //we have the start to a config string


      readConfigParam(szParam, sizeof(szParam));    //should be PT01xx for the ptSolar
      if (strcmp(szParam, CONFIG_VERSION) != 0) {
        //not a config string
        Serial.println(F("Failed to find Config Type param."));
        Serial.print(F("Found: "));
        Serial.println(szParam);
        return false;
      }

      readConfigParam(szParam, sizeof(Config.Callsign));    //Callsign
      strcpy(Config.Callsign, szParam);
      readConfigParam(szParam, 1);    //Callsign SSID
      Config.CallsignSSID = szParam[0];

      readConfigParam(szParam, sizeof(Config.Destination));    //Destination
      strcpy(Config.Destination, szParam);
      readConfigParam(szParam, 1);    //SSID
      Config.DestinationSSID = szParam[0];

      readConfigParam(szParam, sizeof(Config.Path1));    //Path1
      strcpy(Config.Path1, szParam);
      readConfigParam(szParam, 1);    //SSID
      Config.Path1SSID = szParam[0];

      readConfigParam(szParam, sizeof(Config.Path2));    //Path2
      strcpy(Config.Path2, szParam);
      readConfigParam(szParam, 1);    //SSID
      Config.Path2SSID = szParam[0];

      //Cutoff altitude to stop using the path
      readConfigParam(szParam, sizeof(szParam));
      Config.DisablePathAboveAltitude = atoi(szParam);

      //Symbol/Page
      readConfigParam(szParam, 1);
      Config.Symbol = szParam[0];
      readConfigParam(szParam, 1);
      Config.SymbolPage = szParam[0];


      //BeaconType
      readConfigParam(szParam, sizeof(szParam));
      Config.BeaconType = atoi(szParam);

      //Simple Beacon Delay
      readConfigParam(szParam, sizeof(szParam));
      Config.BeaconSimpleDelay = atoi(szParam);

      //SpeedBeacon
      readConfigParam(szParam, sizeof(szParam));
      Config.BeaconSpeedThreshLow = atoi(szParam);

      readConfigParam(szParam, sizeof(szParam));
      Config.BeaconSpeedThreshHigh = atoi(szParam);

      readConfigParam(szParam, sizeof(szParam));
      Config.BeaconSpeedDelayLow = atoi(szParam);

      readConfigParam(szParam, sizeof(szParam));
      Config.BeaconSpeedDelayMid = atoi(szParam);

      readConfigParam(szParam, sizeof(szParam));
      Config.BeaconSpeedDelayHigh = atoi(szParam);

      //AltitudeBeacon
      readConfigParam(szParam, sizeof(szParam));
      Config.BeaconAltitudeThreshLow = atoi(szParam);

      readConfigParam(szParam, sizeof(szParam));
      Config.BeaconAltitudeThreshHigh = atoi(szParam);

      readConfigParam(szParam, sizeof(szParam));
      Config.BeaconAltitudeDelayLow  = atoi(szParam);

      readConfigParam(szParam, sizeof(szParam));
      Config.BeaconAltitudeDelayMid  = atoi(szParam);

      readConfigParam(szParam, sizeof(szParam));
      Config.BeaconAltitudeDelayHigh = atoi(szParam);

      //Time Slots
      readConfigParam(szParam, sizeof(szParam));
      Config.BeaconSlot1 = atoi(szParam);

      readConfigParam(szParam, sizeof(szParam));
      Config.BeaconSlot2 = atoi(szParam);


      //Status Message
      readConfigParam(szParam, sizeof(szParam));
      strcpy(Config.StatusMessage, szParam);


      //Misc Flags
      readConfigParam(szParam, sizeof(szParam));
      Config.StatusXmitGPSFix = szParam[0] == '1';

      readConfigParam(szParam, sizeof(szParam));
      Config.StatusXmitBurstAltitude = szParam[0] == '1';

      readConfigParam(szParam, sizeof(szParam));
      Config.StatusXmitBatteryVoltage = szParam[0] == '1';

      readConfigParam(szParam, sizeof(szParam));
      Config.StatusXmitTemp = szParam[0] == '1';

      readConfigParam(szParam, sizeof(szParam));
      Config.StatusXmitPressure = szParam[0] == '1';

      readConfigParam(szParam, sizeof(szParam));
      Config.StatusXmitCustom = szParam[0] == '1';

			
			//Radio Configuration
      readConfigParam(szParam, sizeof(szParam));
      Config.RadioType = atoi(szParam);    //0=Standard Tx-only, 1=DRA818V

      readConfigParam(szParam, sizeof(szParam));
      Config.RadioTxDelay = atoi(szParam);

      readConfigParam(szParam, sizeof(szParam));
      Config.RadioCourtesyTone = atoi(szParam);    //0=off, 1=on

      readConfigParam(szParam, sizeof(Config.RadioFreqTx));    //Transmit Frequency for DRA818
      strcpy(Config.RadioFreqTx, szParam);
      
      readConfigParam(szParam, sizeof(Config.RadioFreqRx));    //Receive Frequency for DRA818
      strcpy(Config.RadioFreqRx, szParam);
      


			//GPS Configuration
      readConfigParam(szParam, sizeof(szParam));
      Config.GPSSerialBaud = atoi(szParam);    //1=300, 2=1200, 3=2400, 4=4800, 5=9600, 6=19200

      readConfigParam(szParam, sizeof(szParam));
      Config.GPSSerialInvert = atoi(szParam);    //Invert the incoming signal

      readConfigParam(szParam, sizeof(szParam));
      Config.GPSType = atoi(szParam);        //0=Generic NMEA, 1=Ublox, 2=ATGM332D


      //Annunciator Type
      readConfigParam(szParam, sizeof(szParam));
      Config.AnnounceMode = atoi(szParam);

      //BME280 Configuration
      readConfigParam(szParam, sizeof(szParam));
      Config.i2cBME280 = szParam[0] == '1';

      //Beacon Type 4 Configuration
      readConfigParam(szParam, sizeof(szParam));
      Config.VoltThreshGPS = atoi(szParam);   //Threshold for voltage before activating the GPS receiver
      readConfigParam(szParam, sizeof(szParam));
      Config.VoltThreshXmit = atoi(szParam);   //Threshold for voltage before transmitting a packet
      readConfigParam(szParam, sizeof(szParam));
      Config.MinTimeBetweenXmits = atoi(szParam);   //Minimum time between transmissions in the event we have solid voltage



      unsigned int iCheckSum = 0;
      for (int i=0; i<7; i++) {
        iCheckSum += Config.Callsign[i];
      }
      Config.CheckSum = iCheckSum;
      return true;    //done reading in the file
    }
  }
  return false;
}

/** 
 * @brief Sends the configuration data to the PC so that the configuration can be verified and managed.
 * @note  This function will send the configuration data to the PC in a tab-delimited format. 
 */
void sendConfigToPC() {
//dump the configs out to the host PC
        Serial.write(0x01);
        Serial.write(CONFIG_VERSION);
        Serial.write(0x09);

        Serial.write(Config.Callsign);
        Serial.write(0x09);
        Serial.write(Config.CallsignSSID);
        Serial.write(0x09);
        Serial.write(Config.Destination);
        Serial.write(0x09);
        Serial.write(Config.DestinationSSID);
        Serial.write(0x09);
        Serial.write(Config.Path1);
        Serial.write(0x09);
        Serial.write(Config.Path1SSID);
        Serial.write(0x09);
        Serial.write(Config.Path2);
        Serial.write(0x09);
        Serial.write(Config.Path2SSID);
        Serial.write(0x09);

        //Allow to disable the path above certain altitude
        Serial.print(Config.DisablePathAboveAltitude, DEC);
        Serial.write(0x09);

        //Symbol
        Serial.write(Config.Symbol);
        Serial.write(0x09);
        Serial.write(Config.SymbolPage);
        Serial.write(0x09);

        //Beacon Type
        Serial.print(Config.BeaconType, DEC);
        Serial.write(0x09);

        //Beacon - Simple Delay
        Serial.print(Config.BeaconSimpleDelay, DEC);
        Serial.write(0x09);


        //Beacon - Speed Beaconing
        Serial.print(Config.BeaconSpeedThreshLow, DEC);
        Serial.write(0x09);
        Serial.print(Config.BeaconSpeedThreshHigh, DEC);
        Serial.write(0x09);
        Serial.print(Config.BeaconSpeedDelayLow, DEC);
        Serial.write(0x09);
        Serial.print(Config.BeaconSpeedDelayMid, DEC);
        Serial.write(0x09);
        Serial.print(Config.BeaconSpeedDelayHigh, DEC);
        Serial.write(0x09);

        //Beacon - Altitude Beaconing
        Serial.print(Config.BeaconAltitudeThreshLow, DEC);
        Serial.write(0x09);
        Serial.print(Config.BeaconAltitudeThreshHigh, DEC);
        Serial.write(0x09);
        Serial.print(Config.BeaconAltitudeDelayLow, DEC);
        Serial.write(0x09);
        Serial.print(Config.BeaconAltitudeDelayMid, DEC);
        Serial.write(0x09);
        Serial.print(Config.BeaconAltitudeDelayHigh, DEC);
        Serial.write(0x09);

        //Beacon - Time Slots
        Serial.print(Config.BeaconSlot1, DEC);
        Serial.write(0x09);
        Serial.print(Config.BeaconSlot2, DEC);
        Serial.write(0x09);

        //Status Message
        Serial.write(Config.StatusMessage);
        Serial.write(0x09);

        //Misc Flags
        if (Config.StatusXmitGPSFix) Serial.write("1");
        else Serial.write("0");
        Serial.write(0x09);

        if (Config.StatusXmitBurstAltitude) Serial.write("1");
        else Serial.write("0");
        Serial.write(0x09);

        if (Config.StatusXmitBatteryVoltage) Serial.write("1");
        else Serial.write("0");
        Serial.write(0x09);

        if (Config.StatusXmitTemp) Serial.write("1");
        else Serial.write("0");
        Serial.write(0x09);

        if (Config.StatusXmitPressure) Serial.write("1");
        else Serial.write("0");
        Serial.write(0x09);

        if (Config.StatusXmitCustom) Serial.write("1");
        else Serial.write("0");
        Serial.write(0x09);

        //Radio Parameters
        Serial.print(Config.RadioType, DEC);      //0=Standard, 1=DRA818
        Serial.write(0x09);
        
        Serial.print(Config.RadioTxDelay, DEC);
        Serial.write(0x09);
        
        if (Config.RadioCourtesyTone) Serial.write("1");
        else Serial.write("0");
        Serial.write(0x09);               

        Serial.write(Config.RadioFreqTx);
        Serial.write(0x09);

        Serial.write(Config.RadioFreqRx);
        Serial.write(0x09);
        
        //GPS Serial Data
        Serial.print(Config.GPSSerialBaud, DEC);      //1=300, 2=1200, 3=2400, 4=4800, 5=9600, 6=19200
        Serial.write(0x09);

        if (Config.GPSSerialInvert) Serial.write("1");   //1=Invert the incoming signal
        else Serial.write("0");
        Serial.write(0x09);

        Serial.print(Config.GPSType, DEC);      //0=Generic NMEA, 1=Ublox, 2=ATGM332D
        Serial.write(0x09);

        //Misc System Configuration
        Serial.print(Config.AnnounceMode, DEC);    //0=No annunciator, 1=LED only, 2=LED and buzzer
        Serial.write(0x09);

        //BME280 Configuration
        if (Config.i2cBME280) Serial.write("1");
        else Serial.write("0");
        Serial.write(0x09);

        //Beacon Type 4 Configuration
        Serial.print(Config.VoltThreshGPS, DEC);
        Serial.write(0x09);

        Serial.print(Config.VoltThreshXmit, DEC);
        Serial.write(0x09);

        Serial.print(Config.MinTimeBetweenXmits, DEC);


        Serial.write(0x04);      //End of string
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float readBatteryVoltage() {
  int iBattery = analogRead(PIN_ANALOG_BATTERY);
  float fVolts = (float)iBattery / 310.3;    //204.8 points per volt for 5.0V systems, 310.3 for 3.3V systems!!!!,
  fVolts = fVolts * 5.545;        //times (122/22) to adjust for the resistor divider (5.545)
//  fVolts = fVolts + 0.19;      //account for the inline diode on the power supply  // not interested in diode drop for solar purposes??????????????????????????????????????????????????
  return fVolts;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	//------------------------------------------ Functions and Timers  for the internal modulation ------------------------------------------
	ISR(TIMER1_COMPA_vect) {
	  static boolean bBaudFlip = 0;
	  static byte iStuffZero = 0;
	  static boolean bStuffBit = false;


	  static byte iRateGen;
	  static unsigned int iTonePhase = 0;      //two byte variable.  The highByte contains the element in arySine that should be output'ed
	  static boolean bToneHigh = 0;


	  //digitalWrite(PIN_TP_ISRTIME, HIGH);    //measure how long the ISR is taking to execute

	  //increment the phase counter.  It will overflow automatically at > 65535
	  if (bToneHigh) {
	    analogWrite(PIN_AUDIO_OUT, (pgm_read_byte_near(_arySineHigh + highByte(iTonePhase))));
	    iTonePhase += TONE_HIGH_STEPS_PER_TICK;
	  } else {
	    analogWrite(PIN_AUDIO_OUT, (pgm_read_byte_near(_arySineLow + highByte(iTonePhase))));
	    iTonePhase += TONE_LOW_STEPS_PER_TICK;
	  }

	  iRateGen--;


	  if (iRateGen == 0) {
	    //it's time for the next bit

	    //used to measure the baud rate frequency.  Can be deleted for production
	    //digitalWrite(PIN_TP_BAUDRATE, bBaudFlip);
	    //bBaudFlip = !bBaudFlip;

	    if (bStuffBit) {
	      //we hit the stuffing counter  - we don't need to get the next bit yet, just change the tone and send one bit
	      bToneHigh = !bToneHigh;
	      iStuffZero = 0;

	      bStuffBit = false;    //reset this so we don't keep stuffing

	    } else {
	      //this is just a normal bit - grab the next bit from the szString

	      if (oTNC.getNextBit() == 0) {
	        //we only flip the output state if we have a zero

	        //Flip Bit
	        bToneHigh = !bToneHigh;
	        iStuffZero = 0;
	      } else {
	        //it's a 1, so send the same tone...

	        iStuffZero++;      //increament the stuffing counter

	        //if there's been 5 in a row, then we need to stuff an extra bit in, and change the tone for that one
	        if (iStuffZero == 5 && !oTNC.noBitStuffing()) {
	          bStuffBit = true;      //once we hit five, we let this fifth (sixth?) one go, then we set a flag to flip the tone and send a bogus extra bit next time
	        }
	      }
	    }

	    iRateGen = BAUD_GENERATOR_COUNT;
	  }

	  //digitalWrite(PIN_TP_ISRTIME, LOW);
	}
