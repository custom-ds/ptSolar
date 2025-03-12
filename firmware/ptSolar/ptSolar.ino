/*
Project: Traveler ptSolar Firmware
Copyright 2011-2025 - Zack Clobes (W0ZC), Custom Digital Services, LLC


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
#define CONFIG_PROMPT "\n# "




#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>

#include "MemoryFree.h"
#include <SoftwareSerial.h>

#include "Modem.h"
#include "GPS.h"
#include "ptTracker.h"
#include "ptConfig.h"

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
#define METERS_TO_FEET 3.2808399




ptConfig Config;      //Configuration object
ptTracker Tracker;    //Object that manages the board-specific functions
GPS GPSParser(PIN_GPS_RX, PIN_GPS_TX, PIN_GPS_EN);        //Object that parses the GPS strings
Modem Aprs;            //Object that assembles the packets for the TNC and transmits them



unsigned long timeLastXmit;    //Keeps track of the timestamp of the last transmission
unsigned long iLastErrorTxMillis;    //keep track of the timestamp of the last "lost GPS" transmission

bool bHasBurst;
float fMaxAlt;


BME280 Pressure;      //BMP280 pressure/temp sensor




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

  Serial.println(F("pt Flight Computer"));

  Serial.print(F("Firmware Version: "));
  Serial.println(FIRMWARE_VERSION);


  //Init some variables
  fMaxAlt = 0;
  bHasBurst = false;
  timeLastXmit = 0;

  Config.init();    //initialize the configuration object and pull the settings from EEPROM
  


  Tracker.init(PIN_LED, PIN_AUDIO, PIN_ANALOG_BATTERY, Config.getAnnounceMode());
  Tracker.annunciate('k');

  //Configure the APRS modem with the pins to connect to the transmitter
  Aprs.init(PIN_DRA_EN, PIN_PTT_OUT, PIN_AUDIO_OUT, PIN_DRA_TX, PIN_DRA_RX);
  Aprs.setDebugLevel(2);
  Aprs.setTxDelay(Config.getRadioTxDelay());


  //init the I2C devices
  if (Config.getI2cBME280() == 1) {
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


  //Send out an initial packet announcing itself.
  // if (Config.RadioType == 1) initDRA818();    //Configure the transmitter to correct frequency
  // oTNC.xmitStart(Config.Destination, Config.DestinationSSID, Config.Callsign, Config.CallsignSSID, Config.Path1, Config.Path1SSID, Config.Path2, Config.Path2SSID, true);
  // oTNC.xmitString((char *)">Project Traveler ptSolar Flight Computer v");
  // oTNC.xmitString((char *)FIRMWARE_VERSION);
  // oTNC.xmitString((char *)" Initializing...");
  // oTNC.xmitEnd();

  //init the GPS into high altitude mode (Dynamic Model 6 – Airborne < 1g)
  GPSParser.initGPS();    //will continually retry this operation until its sucessful
  GPSParser.setDebugNEMA(true);    ///TODO: Need to pull this from Configuration

  iLastErrorTxMillis = millis();      //set a starting time for the potential error messages
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  float fCurrentAlt, fSpeed, fMaxSpeed;
  unsigned long battMillivolts;
  bool bXmit;
  int iSeconds;
  unsigned long msDelay;    //calculate the number of milliseconds to delay


  battMillivolts = (unsigned long)(Tracker.readBatteryVoltage(true) * 1000);  //read the battery voltage and spit it out to the serial port


  //check to see if we have sufficient battery to run the GPS
  if (battMillivolts > Config.getVoltThreshGPS()) {
    Serial.println(F("Batt > Threshold. Checking the GPS"));
    GPSParser.collectGPSStrings();

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
  } else {
    Serial.println(F("Batt below GPS threshold."));
    delay(750);   //wait for about the amount of time that we'd normally spend grabbing a GPS reading
  }


  // //Check to see if we've decoded a GPS packet recently.
  // if ((GPSParser.LastDecodedMillis() + GPS_TIMEOUT_TIME) < millis()) {
  //   //we haven't decoded anything from the GPS in 45 seconds - we have a problem here

  //   if ((iLastErrorTxMillis + GPS_TIMEOUT_TIME) < millis()) {
  //     //it's been 45 seconds since the last time that we transmitted an error - so transmit

  //     Tracker.annunciate('g');
  //     if (Config.RadioType == 1) initDRA818();    //Configure the transmitter to correct frequency
  //     oTNC.xmitStart(Config.Destination, Config.DestinationSSID, Config.Callsign, Config.CallsignSSID, Config.Path1, Config.Path1SSID, Config.Path2, Config.Path2SSID, true);
  //     oTNC.xmitString((char *)">Lost GPS for over 45 seconds!");
  //     oTNC.xmitEnd();

  //     iLastErrorTxMillis = millis();      //track the fact that we just transmitted
  //   }
  // }



  bXmit = false;    //assume that we won't transmit this time around

  //Figure out how long to delay before the next packet
  switch (Config.getBeaconType()) {
  case 0:
    //This is no logic to beacon intervals - just plan old time delays
    msDelay = (unsigned long)Config.getBeaconSimpleDelay() * 1000;    //cast this to unsigned long
    
     if ((millis() - timeLastXmit) > msDelay) {
      //we've waited long enough - transmit
      bXmit = true;
    }

    break;
  case 1:
    //This is for Speed-based beaconing

    fSpeed = GPSParser.Knots();        //get the current speed
    if (fSpeed > fMaxSpeed) fMaxSpeed = fSpeed;

    if (fMaxSpeed < Config.getBeaconSpeedThreshLow()) {
      //we're in the slow range
      msDelay = (unsigned long)Config.getBeaconSpeedDelayLow() * 1000;    //cast this to unsigned long
      
      if ((millis() - timeLastXmit) > msDelay) {
        //we've waited long enough for this speed - transmit
        bXmit = true;
      }
    }

    if (fSpeed >= Config.getBeaconSpeedThreshLow() && fSpeed < Config.getBeaconSpeedThreshHigh()) {
      //we're in the medium range
      msDelay = (unsigned long)Config.getBeaconSpeedDelayMid() * 1000;    //cast this to unsigned long
      
      if ((millis() - timeLastXmit) > msDelay) {
        //we've waited long enough for this speed - transmit
        bXmit = true;
      }
    }

    if (fSpeed >= Config.getBeaconSpeedThreshHigh()) {
      //we're in the fast range
      msDelay = (unsigned long)Config.getBeaconSpeedDelayHigh() * 1000;    //cast this to unsigned long
      
      if ((millis() - timeLastXmit) > msDelay) {
        //we've waited long enough for this speed - transmit
        bXmit = true;
      }
    }

    break;
  case 2:
    //This is for Altitude-based beaconing

    if (fCurrentAlt < Config.getBeaconAltitudeThreshLow()) {
      //we're in the low phase of the flight - we'll typically send packets more frequently close to the ground
      msDelay = (unsigned long)Config.getBeaconAltitudeDelayLow() * 1000;    //cast this to unsigned long
      
      if ((millis() - timeLastXmit) > msDelay) {
        //we've waited long enough for this speed - transmit
        bXmit = true;
      }
    }
    if (fCurrentAlt >= Config.getBeaconAltitudeThreshLow() && fCurrentAlt < Config.getBeaconAltitudeThreshHigh()) {
      //we're in the mid-phase of the flight.  We'll transmit regularly in here
      msDelay = (unsigned long)Config.getBeaconAltitudeDelayMid() * 1000;    //cast this to unsigned long
      
      if ((millis() - timeLastXmit) > msDelay) {
        //we've waited long enough for this speed - transmit
        bXmit = true;
      }
    }
    if (fCurrentAlt >= Config.getBeaconAltitudeThreshHigh()) {
      //we're in the top-phase of the flight.  Transmit more frequenly to get better burst resolution?
      msDelay = (unsigned long)Config.getBeaconAltitudeDelayHigh() * 1000;    //cast this to unsigned long
      
      if ((millis() - timeLastXmit) > msDelay) {
        //we've waited long enough for this speed - transmit
        bXmit = true;
      }
    }

    break;
  case 3:
    //Use Time Slotting to determine when to transmit
    iSeconds = GPSParser.getGPSSeconds();

    if (iSeconds == Config.getBeaconSlot1() || iSeconds == (Config.getBeaconSlot1() + 1) || iSeconds == Config.getBeaconSlot2() || iSeconds == (Config.getBeaconSlot2() + 1)) {
      bXmit = true;
    }

    break;
  case 4:
    //This is a voltage-checked time delay.  It will wait X seconds, but then also wait for the system (solar) voltage to be above a threshold before transmitting
    msDelay = (unsigned long)Config.getMinTimeBetweenXmits() * 1000;    //cast this to unsigned long
    
     if ((millis() - timeLastXmit) > msDelay) {
      //we've waited long enough - see if we have the power to transmit
      
      if (battMillivolts > Config.getVoltThreshXmit()) {
        Serial.println("Transmitting");
        bXmit = true;
      } else {
        Serial.println("Time to transmit, but still Low volts - no xmit");
      }
    }

    break;    
  }


  if (bXmit) {
    //we're supposed to transmit now
    sendPositionSingleLine();

    timeLastXmit = millis();
    fMaxSpeed = 0;    //reset the max speed to check again this next cycle (Used for Speed-based smart beaconing)

    delay(DELAY_MS_BETWEEN_XMITS);    //delay about a second - if you don't you can run into multiple packets inside of a 2 second window

    if (!GPSParser.FixQuality() || GPSParser.NumSats() < 4) {
      //we are having GPS fix issues - issue an annunciation
      Tracker.annunciate('l');
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
  
  
  if (Config.getI2cBME280()) {
    if (Config.getStatusXmitPressure() || Config.getStatusXmitTemp()) {
      //we're supposed to transmit the air pressure and/or temp - go ahead and pre-fetch it
      airPressure = (double)Pressure.readFloatPressure();
      airPressure = airPressure / 100;    //convert back to simple airpressure in hPa
      insideTemp = (double)Pressure.readTempC();
    }
  }

  Aprs.packetHeader(Config.getDestination(), Config.getDestinationSSID(), Config.getCallsign(), Config.getCallsignSSID(), Config.getPath1(), Config.getPath1SSID(), Config.getPath2(), Config.getPath2SSID(), (GPSParser.Altitude() < Config.getDisablePathAboveAltitude()));

  //      /155146h3842.00N/09655.55WO301/017/A=058239
  int hh = 0, mm = 0, ss = 0;
  GPSParser.getGPSTime(&hh, &mm, &ss);
  Aprs.packetAppend((char *)"/");

  sprintf(szTemp, "%02d", hh);
  Aprs.packetAppend(szTemp);
  sprintf(szTemp, "%02d", mm);
  Aprs.packetAppend(szTemp);
  sprintf(szTemp, "%02d", ss);
  Aprs.packetAppend(szTemp);

  Aprs.packetAppend((char *)"h");
  //Latitude
  GPSParser.getLatitude(szTemp);
  i=0;

  while (i<7 && szTemp[i]) {
    Aprs.packetAppend(szTemp[i]);
    i++;
  }
  Aprs.packetAppend(GPSParser.LatitudeHemi());
  Aprs.packetAppend(Config.getSymbolPage());

  //Longitude
  GPSParser.getLongitude(szTemp);
  i=0;
  while (i<8 && szTemp[i]) {
    Aprs.packetAppend(szTemp[i]);
    i++;
  }
  Aprs.packetAppend(GPSParser.LongitudeHemi());

  Aprs.packetAppend(Config.getSymbol());

  //Course
  fTemp = GPSParser.Course();

  sprintf(szTemp, "%03d", (int)fTemp);
  Aprs.packetAppend(szTemp);
  Aprs.packetAppend('/');

  //Speed in knots
  fTemp = GPSParser.Knots();

  sprintf(szTemp, "%03d", (int)fTemp);
  Aprs.packetAppend(szTemp);

  Aprs.packetAppend((char *)"/A=");
  //Altitude in Feet
  fTemp = GPSParser.AltitudeInFeet();
  Aprs.packetAppend((long)fTemp, true);

  if (Config.getStatusXmitGPSFix()) {
    //Fix quality and num sats

    if (GPSParser.FixQuality() >= 1 && GPSParser.FixQuality() <=3) {
      //we have a GPS, DGPS, or PPS fix
      Aprs.packetAppend((char *)" 3D");
    } else {
      Aprs.packetAppend((char *)" na");
    }

    sprintf(szTemp, "%dSats", GPSParser.NumSats());
    Aprs.packetAppend(szTemp);
  }
  if (Config.getStatusXmitBatteryVoltage()) {

    Aprs.packetAppend((char *)" Vb=");
    Aprs.packetAppend(Tracker.readBatteryVoltage(true));
  }

  if (Config.getI2cBME280() && Config.getStatusXmitTemp()) {
    Aprs.packetAppend((char *)" IAT=");
    Aprs.packetAppend((float)insideTemp);
  }

  if (Config.getI2cBME280() && Config.getStatusXmitPressure()) {
    Aprs.packetAppend((char *)" Press=");
    Aprs.packetAppend((float)airPressure);
  }

  if (Config.getStatusXmitBurstAltitude() && bHasBurst) {
    Aprs.packetAppend((char *)" Burst=");
    fTemp = fMaxAlt * METERS_TO_FEET;
    Aprs.packetAppend((long)fTemp, true);
  }

 
  Aprs.packetAppend(' ');
  Aprs.packetAppend(Config.getStatusMessage());

  Aprs.packetSend();
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
  Tracker.annunciate('c');

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
        Config.readEEPROM();    //pull the configs from eeprom
        Config.sendConfigToPC();
      }

      if (byTemp == 'W' || byTemp == 'w') {
        //take the incoming configs and load them into the Config UDT

        Serial.println(F("Entering config write mode..."));

        if (Config.getConfigFromPC()) {
          Serial.println(F("Done reading in configuration data."));

          Config.writeEEPROM();
          Serial.println(F("Written config to eeprom."));

          Tracker.annunciate('w');
        } else {
          //something failed during the read of the config data
          Serial.println(F("Failure to read in configuration data..."));
        }
      }
      
      if (byTemp == 'D' || byTemp == 'd') {
        //used to reset the tracker back to N0CALL defaults
        Serial.println(F("Reset defaults"));
        Config.setDefaultConfig();        
        Tracker.annunciate('w');
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
          Aprs.setTxDelay(Config.getRadioTxDelay());
      

          Tracker.annunciate('t');
          
          Aprs.PTT(true);   //configures the SA818 as part of the transmit process.
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

          Aprs.PTT(false);
        } else {
          Serial.println(F("Cancelling..."));
        }
      }
      
      if (byTemp == 'E' || byTemp == 'e') {
        //exercise mode to check out all of the I/O ports
        
        Serial.println(F("Exercise"));
        
        Serial.println(F(" annun"));
        Config.setAnnounceMode(0x03);    //temporarily set the announce mode to both
        Tracker.annunciate('x');
        
        Serial.println(Tracker.readBatteryVoltage(true));

        
  
        GPSParser.collectGPSStrings();   //check the GPS  
  
        double insideTemp;    //inside air temp
        double airPressure;    //millibars
        char status;
  

        airPressure = (double)Pressure.readFloatPressure();
        airPressure = airPressure / 100;    //convert back to simple airpressure in hPa
        insideTemp = (double)Pressure.readTempC();
        //insideTemp = insideTemp / 100;    //convert back to decimal

        if (Config.getI2cBME280()) {
          Serial.print(F("IAT: "));
          Serial.println(insideTemp);
          Serial.print(F("Press: "));
          Serial.println(airPressure);
        }
        
      }

      Serial.print(CONFIG_PROMPT);
    }
  }
  Serial.println(F("Exiting config mode..."));
}







//------------------------------------------ Functions and Timers  for the internal modulation ------------------------------------------
ISR(TIMER1_COMPA_vect) {
  static uint8_t iStuffZero = 0;
  static bool bStuffBit = false;
  static uint8_t iRateGen;
  static uint16_t iTonePhase = 0;      //two byte variable.  The highByte contains the element in arySine that should be output'ed
  static bool bToneHigh = false;



  //increment the phase counter.  It will overflow automatically at > 65535
  if (bToneHigh) {
    //analogWrite(PIN_AUDIO_OUT, (pgm_read_byte_near(_arySineHigh + highByte(iTonePhase))));
    analogWrite(PIN_AUDIO_OUT, (pgm_read_byte_near(_arySineHigh + highByte(iTonePhase))));
    iTonePhase += Modem::TONE_HIGH_STEPS_PER_TICK;
  } else {
    //analogWrite(PIN_AUDIO_OUT, (pgm_read_byte_near(_arySineLow + highByte(iTonePhase))));
    analogWrite(PIN_AUDIO_OUT, (pgm_read_byte_near(_arySineLow + highByte(iTonePhase))));
    iTonePhase += Modem::TONE_LOW_STEPS_PER_TICK;
  }

  iRateGen--;


  if (iRateGen == 0) {
    //it's time for the next bit

    if (bStuffBit) {
      //we hit the stuffing counter  - we don't need to get the next bit yet, just change the tone and send one bit
      bToneHigh = !bToneHigh;
      iStuffZero = 0;

      bStuffBit = false;    //reset this so we don't keep stuffing

    } else {
      //this is just a normal bit - grab the next bit from the szString

      if (Aprs.getNextBit() == 0) {
        //we only flip the output state if we have a zero

        //Flip Bit
        bToneHigh = !bToneHigh;
        iStuffZero = 0;
      } else {
        //it's a 1, so send the same tone...

        iStuffZero++;      //increament the stuffing counter

        //if there's been 5 in a row, then we need to stuff an extra bit in, and change the tone for that one
        if (iStuffZero == 5 && !Aprs.noBitStuffing()) {
          bStuffBit = true;      //once we hit five, we let this fifth (sixth?) one go, then we set a flag to flip the tone and send a bogus extra bit next time
        }
      }
    }

    iRateGen = Modem::BAUD_GENERATOR_COUNT;
  }
}