/*
A Configuration object to storing settings for Project: Traveler Flight Controllers
Copyright 2011-2025 - Zack Clobes (W0ZC), Custom Digital Services, LLC

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.

Version History:
Version 1.0.0 - March 9, 2025 - Initial Release.

*/

#include "ptConfig.h"

ptConfig::ptConfig() {
    //Initializer
}

void ptConfig::init() {
    this->readEEPROM();

}


/**
 * @brief  Read the configuration from the EEPROM.
 * @note   This function reads the configuration from the EEPROM.  It is called at startup to load the configuration.
 */
void ptConfig::readEEPROM() {
    for (unsigned int i=0; i<sizeof(this->_config); i++) {
        *((char*)&this->_config + i) = EEPROM.read(i);
    }

    //Check to see if the EEPROM appears to be valid
    unsigned int iCheckSum = 0;
    for (int i=0; i<7; i++) {
        iCheckSum += this->_config.Callsign[i];
    }

    Serial.println(F("Read EEPROM"));
    Serial.println(this->_config.Callsign);

    if (iCheckSum != this->_config.CheckSum) {
        Serial.println(F("Checksum fail. Resetting."));

        //we do NOT have a match - reset the Config variables
        this->setDefaultConfig();
    }

}
/**
 * @brief  Write the default configuration to the EEPROM.
 * @note   Used when the EEPROM is not initialized or has been corrupted. Checksum is based on the string inside of _config.Callsign.
 */
void ptConfig::setDefaultConfig() {
    strcpy(this->_config.Callsign, "N0CALL");
    this->_config.CallsignSSID = '0';
    strcpy(this->_config.Destination, "APRS  ");
    this->_config.DestinationSSID = '0';
    strcpy(this->_config.Path1, "WIDE2 ");
    this->_config.Path1SSID = '1';
    strcpy(this->_config.Path2, "      ");
    this->_config.Path2SSID = '0';
    this->_config.DisablePathAboveAltitude = 2000;
    this->_config.Symbol = 'O';    //letter O for balloons
    this->_config.SymbolPage = '/';
    this->_config.BeaconType = 4;    //Solar (voltgage) delay
    this->_config.BeaconSimpleDelay = 30;
    this->_config.BeaconSpeedThreshLow = 20;
    this->_config.BeaconSpeedThreshHigh = 50;
    this->_config.BeaconSpeedDelayLow = 300;
    this->_config.BeaconSpeedDelayMid = 60;
    this->_config.BeaconSpeedDelayHigh = 120;
    this->_config.BeaconAltitudeThreshLow = 5000;
    this->_config.BeaconAltitudeThreshHigh = 20000;
    this->_config.BeaconAltitudeDelayLow  = 30;
    this->_config.BeaconAltitudeDelayMid  = 60;
    this->_config.BeaconAltitudeDelayHigh = 45;
    this->_config.BeaconSlot1 = 15;
    this->_config.BeaconSlot2 = 45;
    strcpy(this->_config.StatusMessage, "Project Traveler ptSolar");
    this->_config.StatusXmitGPSFix = 1;
    this->_config.StatusXmitBurstAltitude = 1;
    this->_config.StatusXmitBatteryVoltage = 1;
    this->_config.StatusXmitTemp = 1;
    this->_config.StatusXmitPressure = 1;
    this->_config.StatusXmitCustom;

    this->_config.RadioType = 1;   //DRA818V transmitter
    this->_config.RadioTxDelay = 50;
    this->_config.RadioCourtesyTone = 0;
    strcpy(this->_config.RadioFreqTx, "144.3900");
    strcpy(this->_config.RadioFreqRx, "144.3900");

    this->_config.GPSSerialBaud = 5;    //1=300, 2=1200, 3=2400, 4=4800, 5=9600, 6=19200
    this->_config.GPSSerialInvert = 0;    //Invert the incoming signal
    this->_config.GPSType = 2;      //0=Generic NMEA, 1=UBlox, 2=ATGM332D
    this->_config.AnnounceMode = 1;

    this->_config.i2cBME280 = 0;    //initialize the BME280

    this->_config.VoltThreshGPS = 3500;    //3.5V
    this->_config.VoltThreshXmit = 4000;    //4.0V
    this->_config.MinTimeBetweenXmits = 30;    //30 seconds



    this->_config.CheckSum = 410;		//Checksum for N0CALL

    this->writeEEPROM();
}


/**
 * @brief  Write the configuration to the EEPROM.
 * @note   This function writes the configuration to the EEPROM.  It is called after the configuration has been updated.
 */
void ptConfig::writeEEPROM() {
    for (unsigned int i=0; i<sizeof(this->_config); i++) {
        EEPROM.write(i, *((char*)&this->_config + i));
    }
}