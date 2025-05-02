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


#define FIRMWARE_VERSION "0.9.10"
#define CONFIG_PROMPT "\n\n# "




#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>

#include "MemoryFree.h"

#include "ptConfig.h"
#include "Modem.h"
#include "GPS.h"
#include "ptTracker.h"

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
#define METERS_TO_FEET 3.2808399


ptConfig Config;                                                                        //Configuration object
ptTracker Tracker(PIN_LED, PIN_AUDIO, PIN_ANALOG_BATTERY, Config.getAnnounceMode());    //Object that manages the board-specific functions
GPS GPSParser(PIN_GPS_RX, PIN_GPS_TX, PIN_GPS_EN);                                      //Object that parses the GPS strings
Modem Aprs(PIN_DRA_EN, PIN_PTT_OUT, PIN_AUDIO_OUT, PIN_DRA_TX, PIN_DRA_RX);;            //Object that assembles the packets for the TNC and transmits them

BME280 Pressure;      //BMP280 pressure/temp sensor

unsigned long timeLastXmit;    //Keeps track of the timestamp of the last transmission
unsigned long iLastErrorTxMillis;    //keep track of the timestamp of the last "lost GPS" transmission
bool bHasBurst;
float fMaxAlt;


/**
 * @brief  The function that runs first before the main loop() function is called indefinitely.
 * @note   This function is called once at startup and is used to initialize the board and set up the hardware.
 */
void setup() {
  Serial.begin(19200);

  showVersion();    //show the version of the firmware that we're running


  //Init some variables
  fMaxAlt = 0;
  bHasBurst = false;
  timeLastXmit = 0;

  Tracker.annunciate('k');

  //Additional configurations for the APRS Modem
  Aprs.setDebugLevel(2);
  Aprs.setTxDelay(Config.getRadioTxDelay());
  Aprs.setCourtesyTone(Config.getRadioCourtesyTone());

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
  

  //init the GPS into high altitude mode (Dynamic Model 6 – Airborne < 1g)
  //GPSParser.initGPS();        //Initializes the GPS
  GPSParser.setDebugNEMA(true);    ///TODO: Need to pull this from Configuration
  GPSParser.setDebugLevel(2);    //Get full verbose output from the GPS

  iLastErrorTxMillis = millis();      //set a starting time for the potential error messages
}


/**
 * @brief  Main loop for the program.  This is where the main logic of the program is executed.
 * @note   This function will run continuously until the board is powered off or reset.
 */
void loop() {

  float fCurrentAlt, fSpeed, fMaxSpeed;
  unsigned long battMillivolts;
  bool bXmit;
  int iSeconds;
  unsigned long msDelay;    //calculate the number of milliseconds to delay
  byte byTemp;
  char szFreq[9];    //The frequency to transmit on

  //Check to see if we have a command from the serial port to indicate that we need to enter config mode
  if (Serial.available()) {
    byTemp = Serial.read();
    if (byTemp == '!') {
      doConfigMode();
    }
  }

  battMillivolts = (unsigned long)(Tracker.readBatteryVoltage(true) * 1000);  //read the battery voltage and spit it out to the serial port


  //check to see if we have sufficient battery to run the GPS
  if (battMillivolts >= Config.getVoltThreshGPS()) {
    GPSParser.enableGPS(false);    //enable the GPS module if it's not already
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

    //See if the Battery has dropped 100mV below the threshold.  If so, disable the GPS until the battery comes back up
    if (battMillivolts < (Config.getVoltThreshGPS() - 100)) {
      //we don't have enough battery to run the GPS - disable it
      Serial.println(F("Disabling GPS"));
      GPSParser.disableGPS();
    }

    Serial.println(F("Low Batt, no GPS"));
    delay(750);   //wait for about the amount of time that we'd normally spend grabbing a GPS reading
  }



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
        bXmit = true;
      } else {
        Serial.println("Low Batt - no xmit");
      }
    }

    break;    
  }


  if (bXmit) {
    bool bXmitPermitted = true;    //assume that we can transmit

    //Disable the GPS to save power
    GPSParser.disableGPS();    //disable the GPS module

    //Determine the transmit/receive frequency to use
    if (Config.getUseGlobalFreq()) {
      //Look up the current APRS frequency from the GPS Position
      bXmitPermitted = GPSParser.getAPRSFrequency(szFreq);
      Aprs.setTxFrequency(szFreq);    //set the frequency to transmit on
      Aprs.setRxFrequency(szFreq);    //set the frequency to receive on
    } else {
      //we're supposed to use the local frequency - set it up
      Aprs.setTxFrequency(Config.getRadioFreqTx());    //set the frequency to transmit on
      Aprs.setRxFrequency(Config.getRadioFreqRx());    //set the frequency to receive on
    }

    //we're supposed to transmit now
    if (bXmitPermitted) {
      sendPositionSingleLine();
    } else {
      Serial.println(F("Xmit Prohibit"));
    }

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
    Serial.print(F("Mem: "));
    Serial.println(freeMemory());
  #endif  
}


/**
 * @brief sendPositionSingleLine - This function sends the position of the tracker in a single line format. It includes information such as GPS time, latitude, longitude, course, speed, altitude, and other telemetry data.
 * @return void
 */
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

  Tracker.readBatteryVoltage(true);  //read the battery voltage before the transmission
  Aprs.packetSend();
  //Normally seeing about 280mV of drop during the transmission with a 0.5F supercap
  Tracker.readBatteryVoltage(true);  //read the battery voltage after the transmission
}


/**
 * @brief showVersion - Displays the version of the firmware and configuration
 * @return void
 */
void showVersion() {
  Serial.println(F("pt Flight Computer"));
  Serial.print(F("Firmware Version: "));
  Serial.println((char *)FIRMWARE_VERSION);
  Serial.print(F("Config Version: "));
  Serial.println(CONFIG_VERSION);
}


/**
 * @brief doConfigMode - This function is used to enter the configuration mode of the tracker. There are diagnostic routines, and the ability to read/write the EEPROM settings.
 * @return void
 */
void doConfigMode() {
  byte byTemp;

  showVersion();
  Serial.print(CONFIG_PROMPT);

  delay(750);
  Tracker.annunciate('c');

  while (true) {
    //Endless loop. Only exit is to reboot with the 'Q'

    if (Serial.available()) {
      byTemp = Serial.read();


      if (byTemp == '!') {
        showVersion();
      }

      
      if (byTemp == 'D' || byTemp == 'd') {
        //used to reset the tracker back to N0CALL defaults
        Serial.println(F("Clear config"));
        Config.setDefaultConfig();        
        Tracker.annunciate('w');
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


      if (byTemp == 'l' || byTemp == 'L') {
        //Do a long test of the transmitter (useful for spectrum analysis or burn-in testing)
        Serial.println(F("Test Xmit"));
        Serial.println(F("\n1. - 1.5s"));
        Serial.println(F("2. - 10s"));
        Serial.println(F("3. - 30s"));
        Serial.println(F("4. - 60s"));
        Serial.println(F("5. - 120s"));

        while (!Serial.available()) {
          //Wait for an input
        }
        byTemp = Serial.read();

        if (byTemp >= '1' && byTemp <= '5') {
          Aprs.setTxFrequency(Config.getRadioFreqTx());    //set the frequency to transmit on
          Aprs.setRxFrequency(Config.getRadioFreqRx());    //set the frequency to receive on
          Aprs.setTxDelay(Config.getRadioTxDelay());

          Tracker.annunciate('t');
          
          Aprs.PTT(true);   //configures the SA818 as part of the transmit process.
          switch (byTemp) {
          case '1':
            Serial.println(F("1.5s"));
            delay(1500);
            break;
          case '2':
            Serial.println(F("10s"));
            delay(10000);
            break;
          case '3':
            Serial.println(F("30s"));
            delay(30000);
            break;
          case '4':
            Serial.println(F("60s"));
            delay(60000);
            break;
          case '5':
            Serial.println(F("120s"));
            delay(120000);
            break;
          default:
            Serial.println(F("Unk"));
          }

          Aprs.PTT(false);
        }
      }

      if (byTemp == 'P' || byTemp == 'p') {
        //Send a test packet
        Serial.println(F("Test Packet"));
        Aprs.setTxFrequency(Config.getRadioFreqTx());    //set the frequency to transmit on
        Aprs.setRxFrequency(Config.getRadioFreqRx());    //set the frequency to receive on
        Aprs.setTxDelay(Config.getRadioTxDelay());

        Aprs.packetHeader(Config.getDestination(), Config.getDestinationSSID(), Config.getCallsign(), Config.getCallsignSSID(), Config.getPath1(), Config.getPath1SSID(), Config.getPath2(), Config.getPath2SSID(), (GPSParser.Altitude() < Config.getDisablePathAboveAltitude()));
        Aprs.packetAppend((char *)">Project Traveler Test");
        Tracker.readBatteryVoltage(true);  //read the battery voltage before the transmission
        Aprs.packetSend();
        Tracker.readBatteryVoltage(true);  //read the battery voltage after the transmission
      }


      if (byTemp == 'Q' || byTemp == 'q') {
        //Quit the config mode
        Serial.println(F("Rebooting..."));
        Tracker.reboot();
      }


      if (byTemp == 'R' || byTemp == 'r') {
        Config.readEEPROM();    //pull the configs from eeprom
        Config.sendConfigToPC();
      }


      if (byTemp == 'T' || byTemp == 't') {
        //exercise the transmitter
        Aprs.setTxFrequency(Config.getRadioFreqTx());    //set the frequency to transmit on
        Aprs.setRxFrequency(Config.getRadioFreqRx());    //set the frequency to receive on
        Aprs.setTxDelay(Config.getRadioTxDelay());
        Aprs.sendTestDiagnotics();
      }      


      if (byTemp == 'W' || byTemp == 'w') {
        //take the incoming configs and load them into the Config UDT

        Serial.println(F("Config mode..."));    //NOTE: This wording is critical for the ptConfigurator to know that we're in config mode

        if (Config.getConfigFromPC()) {
          Serial.println(F(" loaded"));

          Config.writeEEPROM();
          Serial.println(F(" saved"));

          Tracker.annunciate('w');
        } else {
          //something failed during the read of the config data
          Serial.println(F(" failed"));
        }
      }

      Serial.print(CONFIG_PROMPT);
    }
  }
}


//------------------------------------------ Functions and Timers  for the internal modulation ------------------------------------------
/**
 * @brief ISR - This is the interrupt service routine for the timer.  It is used to generate the audio tones for the VHF transmitter using the AX.25 protocol.
 */
ISR(TIMER1_COMPA_vect) {
  static uint8_t iStuffZero = 0;
  static bool bStuffBit = false;
  static uint16_t iRateGen = 1;
  static uint16_t iTonePhase = 0;      //two byte variable.  The highByte contains the element in arySine that should be output'ed
  static uint8_t toneHigh = 0;

  // digitalWrite(PIN_LED, HIGH);   //Uncomment for troubleshooting ISR Timing


  //increment the phase counter.  It will overflow automatically at > 65535
  switch (toneHigh) {
    case 0:
      iTonePhase += Modem::TONE_LOW_STEPS_PER_TICK; 
      break;
    case 1:
      iTonePhase += Modem::TONE_HIGH_STEPS_PER_TICK;
      break;
    case 2:
      iTonePhase += Modem::TONE_COURTESY_STEPS_PER_TICK;    //courtesy chirp
      break;
    default:
      iTonePhase = Modem::TONE_LOW_STEPS_PER_TICK; 
      break;
  }
  
  OCR2B = Aprs.getDACValue(highByte(iTonePhase));   //analogWrite to PIN PD3, directly using the OCR register

  iRateGen--;
  if (iRateGen == 0) {
    //it's time for the next bit

    if (bStuffBit) {
      //we hit the stuffing counter  - we don't need to get the next bit yet, just change the tone and send one bit
      toneHigh = !toneHigh;
      iStuffZero = 0;

      bStuffBit = false;    //reset this so we don't keep stuffing
      iRateGen = Modem::BAUD_GENERATOR_COUNT;  
    } else {
      //this is just a normal bit - grab the next bit from the szString

      switch(Aprs.getNextBit()) {
        case 0:
          //we only flip the output state if we have a zero
          toneHigh = !toneHigh;   //Flip Bit
          iStuffZero = 0;
          iRateGen = Modem::BAUD_GENERATOR_COUNT;  
          break;     
        case 1:
          //it's a 1, so send the same tone...
          iStuffZero++;      //increament the stuffing counter

          //if there's been 5 in a row, then we need to stuff an extra bit in, and change the tone for that one
          if (iStuffZero == 5 && !Aprs.noBitStuffing()) {
            bStuffBit = true;      //once we hit five, we let this fifth (sixth?) one go, then we set a flag to flip the tone and send a bogus extra bit next time
          }
          iRateGen = Modem::BAUD_GENERATOR_COUNT;  
          break;
        case 2:
          //this is a special case - we need to send a courtesy tone, so set the flag and break out of the loop
          toneHigh = 2;
          bStuffBit = false;
          iRateGen = Modem::COURTESY_TONE_COUNT;  
          break;
      } 
    }
    
  }
  //digitalWrite(PIN_LED, LOW);   //Uncomment for troubleshooting ISR Timing
}