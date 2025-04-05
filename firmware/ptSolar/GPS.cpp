/*
GPS Data Parser for Project: Traveler Flight Controllers
Copywrite 2011-2025 - Zack Clobes (W0ZC), Custom Digital Services, LLC

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.

Version History:
Version 1.0.4 - October 18, 2015 - Found bug where GPS latitude and longitude were exceeding the buffer.  Corrected and cleaned up some old code.
Version 1.0.3 - October 7, 2015 - Eliminated excess Serial.print's.  Cleaned up old comments.
Version 1.0.2 - April 27, 2015 - Added the possibility to capture GNGGA and GNRMC strings (instead of just GP***).
Version 1.0.1 - February 27, 2015 - Added initialization to the private member variables.  Cleaned up some old comments.
Version 1.0.0 - January 15, 2015 - Finalized the basic configuration and licensed GPS under the GPL3 license.


*/

#include "GPS.h"
#include <SoftwareSerial.h>


//Public Methods
GPS::GPS(uint8_t pinGPSRx, uint8_t pinGPSTx, uint8_t pinGPSEnable) {
	//Initializer
	this->_pinGPSRx = pinGPSRx;
	this->_pinGPSTx = pinGPSTx;
	this->_pinGPSEnable = pinGPSEnable;

	//Set the pin modes
	pinMode(this->_pinGPSEnable, OUTPUT);
	digitalWrite(this->_pinGPSEnable, LOW);    //disable the GPS until we're ready
	
	//Constructor - initialize the vars
	_szTemp[0] = 0;
	_iTempPtr = 0;
	_bFoundStart = false;
	_bRMCComplete = false;
	_bGGAComplete = false;  

	strcpy(_szLatitude, "1234.5678");
	strcpy(_szLongitude, "12345.6789");
	strcpy(_szGPSDate, "000000"); 
	_currTime.hh =_currTime.mm = _currTime.ss = 0;
	_cLatitudeHemi = 'N';
	_cLongitudeHemi = 'W';
	_iFixQuality = 0;
	_bFixValid = false;
	_iNumSats = 0;
	_fAltitude = 0.0;
	_fKnots = 0.0;
	_fCourse = 0.0;

	_lastDecodedMillis = millis();
	_outputNEMA = false;
}


/**
 * @brief  Initializes the GPS module and sets it to high altitude mode.
 * @note   The purpose of this function varies depending on which type of GPS module is being init'ed.
 */
void GPS::initGPS() {
	bool bSuccess = false;

	SoftwareSerial GPS(this->_pinGPSRx, this->_pinGPSTx, false);
	GPS.begin(9600);
	delay(500);

	// UBlox GPS - set the GPS to high altitude mode (Dynamic Model 6 – Airborne < 1g)
	if (this->_GPSType == 1) {
		if (this->_debugLevel > 0) Serial.println(F("Init UBlox"));

		//digitalWrite(this->_pinGPSEnable, HIGH);    //Enable the GPS		-- not currently implemented in the ublox GPS


		const byte setdm6[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
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
		//digitalWrite(this->_pinGPSEnable, LOW);    //Disable the GPS		-- not currently implemented in the ublox GPS
	}


	// ATGM332D GPS - set the GPS to high altitude mode and disable unnecessary NMEA sentences
	if (this->_GPSType ==2) {
		if (this->_debugLevel > 0) Serial.println(F("Init ATGM332"));

		digitalWrite(this->_pinGPSEnable, HIGH);    //Enable the GPS

		if (this->_debugLevel > 0) Serial.println(F("Config GGA/RMC"));
		Serial.println(F("$PCAS03,1,0,0,0,1,0,0,0,0*1E")); // turns on only $GNGGA and $GNRMC (1 sec)
		delay(100);
		if (this->_debugLevel > 1) Serial.println(F("Enable air"));
		Serial.println(F("$PCAS11,5*18")); // Airborne mode on ATM336H-5N GPS??
		delay(100);

		
	}

	GPS.end();	//close the serial port to the GPS so it doens't draw excess current
}


void GPS::collectGPSStrings() {
	SoftwareSerial GPS(this->_pinGPSRx, this->_pinGPSTx, false);    //A True at the end indicates that the serial data is inverted.
	GPS.begin(9600);

	

	digitalWrite(this->_pinGPSEnable, HIGH);    //enable the GPS

	this->clearInputBuffer();
	this->ClearSentenceFlags();      //clear out the temporary flags to indicate that the new sentences have come in


	//keep track of how long we can listen to the GPS
	unsigned long ulUntil = millis() + GPS_MAX_COLLECTION_TIME;


	while (millis() < ulUntil ) {
		//need to continue looping even if the data isn't coming in.

		//see if there's some new GPS data available
		if (GPS.available()) {
			byte c = GPS.read();

			this->addChar(c);

			//check the sentence flags to see if both RMC and GGA's have been received in this session
			if (this->GotNewRMC() && this->GotNewGGA()) {
				//we got new GGA and RMC strings - exit out now rather than waiting the whole alloted period.
				return;
			}
		}
	}


	digitalWrite(this->_pinGPSEnable, LOW);    //shut the GPS back down
	GPS.end();	//close the serial port to the GPS so it doens't draw excess current
	return;
}

void GPS::clearInputBuffer(void) {
  //need to flush out the old data before importing new, or else you can wind up with odd combinations of old headers and new tails
  _szTemp[0] = 0;
  _iTempPtr = 0;
  _bFoundStart = false;
}
  
  

void GPS::addChar(char c) {
	//first make sure we still have room in the _szTemp for another char (and null termiation)
	if (_iTempPtr >= (_MAX_SENTENCE_LEN - 2)) {
		//we're full and we apparently didn't find an end of string - throw the szTemp away and lets start over
		_szTemp[0] = 0;
		_iTempPtr = 0;
		_bFoundStart = false;
	}

	if (this->_bFoundStart == false) {
		//we are getting a new character, but we don't have a valid start to a sentence yet - see if this is a $
		
		if (c == '$') {
			//we have a new sentence - start storing it to the temp var
			this->_iTempPtr = 0;
			this->_bFoundStart = true;
			
			_szTemp[_iTempPtr++] = c;
			_szTemp[_iTempPtr] = 0;			//always make sure our resulting string is null terminated
		}
		//this was a start, so it couldn't have been the finish - just return;
		
		return;
	} else {
		//we were already collecting a valid sentence - lets add it to the array
		
		//first make sure it's not the END of the sentence.
		if ((int)c <= 0x0d) {
			//this was probably a \r or \n, but could be some other sort of invalid (null or Control) char
			
			//look at the first few chars of the array to see if it's RMC or GGA.
			if (_szTemp[1] == 'G' && (_szTemp[2] == 'P' || _szTemp[2] == 'N') && _szTemp[3] == 'R' && _szTemp[4] == 'M' && _szTemp[5] == 'C') {
				//we have the start of an RMC string

        if (this->_outputNEMA) {
          Serial.println(_szTemp);
        }
				this->_bRMCComplete = true;    //set a flag indicating that an RMC sentence has been received, therefore we have valid data

				this->parseRMC();

				this->_bGotNewRMC = true;      //set a temporary flag indicating that we got a new RMC sentence
				this->_lastDecodedMillis = millis();    //keep track of the time when we last received a sentence
			
			} else if (_szTemp[1] == 'G' && (_szTemp[2] == 'P' || _szTemp[2] == 'N') && _szTemp[3] == 'G' && _szTemp[4] == 'G' && _szTemp[5] == 'A') {
				//we have the start of an GGA string

        if (this->_outputNEMA) {
          Serial.println(_szTemp);
        }
				this->_bGGAComplete = true;
				this->parseGGA();
				this->_bGotNewGGA = true;      //set a temporary flag indicating that we got a new RMC sentence
				this->_lastDecodedMillis = millis();    //keep track of the time when we last received a sentence
			}
			
			//even if we didn't find an RMC or GGA string, we still made it to the end - throw out whatever is still in szTemp
			this->_szTemp[0] = 0;
			this->_iTempPtr = 0;
			this->_bFoundStart = false;
			
		} else {
			//this is just a "normal char".  Add it to the array and quit
			_szTemp[_iTempPtr++] = c;
			_szTemp[_iTempPtr] = 0;			//always make sure our resulting string is null terminated
			return;
		}
	}
}





//Private functions
void GPS::parseRMC() {
//This function parses the time and altitude from the GGA string that is stored in the variable being passed
//0         10        20        30        40        50        60        70
//01234567890123456789012345678901234567890123456789012345678901234567890
//$GPRMC,224831,A,3805.5827,N,09755.0740,W,000.0,000.0,240806,005.9,E*65

	char sz[9];			//temp var
	
	char* ptrTemp;
	
	ptrTemp = &_szTemp[7];			//set this pointer to the hours digit

	
	strncpy(sz, ptrTemp, 2);
	sz[2] = 0;		//null terminate the string
	this->_currTime.hh = atoi(sz);
	ptrTemp += 2;		//incr the pointer to minutes
	
	strncpy(sz, ptrTemp, 2);
	sz[2] = 0;		//null terminate the string
	this->_currTime.mm = atoi(sz);
	ptrTemp += 2;		//incre the pointer to seconds
	
	strncpy(sz, ptrTemp, 2);
	sz[2] = 0;		//null terminate the string
	this->_currTime.ss = atoi(sz);
	
	
	ptrTemp = this->skipToNext(ptrTemp);			//skip thru the rest of the chars in the time

	//see if we have valid fix (A) or invalid (V)
	this->getString(ptrTemp, sz, 2);
	if (sz[0] == 'A') {
		this->_bFixValid = true;
	} else {
		this->_bFixValid = false;
	}
	ptrTemp = this->skipToNext(ptrTemp);


	//get the Latitude
	this->getString(ptrTemp, _szLatitude, _MAX_LATITUDE_LEN);
  if (_szLatitude[0] == '\0') {
    //the longitude was empty
	  strcpy(_szLatitude, "0000.0000");
  }
	ptrTemp = this->skipToNext(ptrTemp);

	//get Latitude Hemisphere
	this->getString(ptrTemp, sz, 2);
	if (sz[0] == 'S') {
		this->_cLatitudeHemi = 'S';
	} else {
		this->_cLatitudeHemi = 'N';
	}
	ptrTemp = this->skipToNext(ptrTemp);


	//get the Longitude
	this->getString(ptrTemp, this->_szLongitude, _MAX_LONGITUDE_LEN);
  if (this->_szLongitude[0] == '\0') {
    //the longitude was empty
	  strcpy(this->_szLongitude, "00000.0000");
  }
	ptrTemp = this->skipToNext(ptrTemp);

	//get Latitude Hemisphere
	this->getString(ptrTemp, sz, 2);
	if (sz[0] == 'E') {
		this->_cLongitudeHemi = 'E';
	} else {
		this->_cLongitudeHemi = 'W';
	}
	ptrTemp = this->skipToNext(ptrTemp);

	//get speed in knots
	this->getString(ptrTemp, sz, 6);
	this->_fKnots = atof(sz);
	ptrTemp = this->skipToNext(ptrTemp);

	//get course of track
	this->getString(ptrTemp, sz, 6);
	this->_fCourse = atof(sz);
	ptrTemp = this->skipToNext(ptrTemp);	
	
	//get date
	this->getString(ptrTemp, this->_szGPSDate, 7);
	
	
}
void GPS::parseGGA() {
	//This function parses the time and altitude from the GGA string that is stored in the variable being passed
	//0         10        20        30        40        50        60        70
	//01234567890123456789012345678901234567890123456789012345678901234567890
	//$GPGGA,232440   ,3804.3322 ,N,09756.1222 ,W,6,03,2.7,485.1,M,-26.7,M,,*
	//$GNGGA,162315.00,3805.57830,N,09755.05519,W,1,03,1.86,577.4,M,-26.1,M,,*76
	char sz[9];			//temp var
	
	char* ptrTemp;
	
	ptrTemp = &_szTemp[7];			//set this pointer to the hours digit

	
	strncpy(sz, ptrTemp, 2);
	sz[2] = 0;		//null terminate the string
	this->_currTime.hh = atoi(sz);
	ptrTemp += 2;		//incr the pointer to minutes
	
	strncpy(sz, ptrTemp, 2);
	sz[2] = 0;		//null terminate the string
	this->_currTime.mm = atoi(sz);
	ptrTemp += 2;		//incre the pointer to seconds
	
	strncpy(sz, ptrTemp, 2);
	sz[2] = 0;		//null terminate the string
	this->_currTime.ss = atoi(sz);
	
	ptrTemp = this->skipToNext(ptrTemp);			//skip thru the rest of the chars in the time

	//get the Latitude
	getString(ptrTemp, _szLatitude, _MAX_LATITUDE_LEN);
  if (this->_szLatitude[0] == '\0') {
    //the longitude was empty
    strcpy(this->_szLatitude, "0000.0000");
  }
	ptrTemp = skipToNext(ptrTemp);

	//get Latitude Hemisphere
	this->getString(ptrTemp, sz, 2);
	if (sz[0] == 'S') {
		this->_cLatitudeHemi = 'S';
	} else {
		this->_cLatitudeHemi = 'N';
	}
	ptrTemp = skipToNext(ptrTemp);


	//get the Longitude
	this->getString(ptrTemp, this->_szLongitude, _MAX_LONGITUDE_LEN);
  if (this->_szLongitude[0] == '\0') {
    //the longitude was empty
	  strcpy(_szLongitude, "00000.0000");
  }
	ptrTemp = this->skipToNext(ptrTemp);

	//get Latitude Hemisphere
	this->getString(ptrTemp, sz, 2);
	if (sz[0] == 'E') {
		this->_cLongitudeHemi = 'E';
	} else {
		this->_cLongitudeHemi = 'W';
	}
	ptrTemp = this->skipToNext(ptrTemp);

	//get position fix quality
	this->getString(ptrTemp, sz, 2);
	this->_iFixQuality = atoi(sz);
	ptrTemp = this->skipToNext(ptrTemp);
	
	//get number of sats received
	this->getString(ptrTemp, sz, 3);
	this->_iNumSats = atoi(sz);
	ptrTemp = this->skipToNext(ptrTemp);
	
	ptrTemp = this->skipToNext(ptrTemp);		//skip over horizontal dilution
	
	this->getString(ptrTemp, sz, 8);
	this->_fAltitude = atof(sz);
}


void GPS::getString(char *ptrHaystack, char *ptrFound, int iMaxLength) {
	//Extracts the string (up to iMaxLength) from the char*, and returns a pointer
	
	*ptrFound = '\0';			//be sure that the Found is null terminated even if we didn't find anything
	
	int i = 0;
	while (ptrHaystack[i] != '\0' && ptrHaystack[i] != ',' && i < (iMaxLength - 1)) {
		
		//this is just a char - copy it to the array
		ptrFound[i] = ptrHaystack[i];
		ptrFound[i+1] = '\0';		//null terminate the return		
		
		i++;
	}
}


bool GPS::validateGPSSentence(char *szGPSSentence, int iNumCommas, int iMinLength) {
	//checks to make sure we have a string that is null terminated, has the appropriate # of commas, and has valid chars within it
	// Pass it the string to test, and the number of commas that should be included for this type of string
	
	int iCommaCount = 0;
	int iCharCount = 0;
	
	while (szGPSSentence[iCharCount] != '\0' && iCharCount < _MAX_SENTENCE_LEN) {
		if (szGPSSentence[iCharCount] == ',') iCommaCount++;			//we found a comma, incre the counter
		
		iCharCount++;
	}
	if (iCharCount < iMinLength) return false;
	if (iNumCommas != iCommaCount) return false;
		
	//we passed all of the tests - return true
	return true;
	
}

char* GPS::skipToNext(char *ptr) {
	//takes a pointer, advances through until it either hits a null or gets past the next comma

	while (*ptr != ',' && *ptr != '\0') {
		//just a char - incr to the next one
		ptr++;	
	}
	
	if (*ptr == ',') ptr++;			//if we landed on a comma, advance one
	return ptr;
}

void GPS::getLatitude(char *sz) {
  byte i = 0;
  sz[0] = 0x00;   //always make sure we return null if no valid data in the source

  if (this->_bGGAComplete || this->_bRMCComplete) {
    while(this->_szLatitude[i] > 0x00 && i < _MAX_LATITUDE_LEN) {
      sz[i] = this->_szLatitude[i];
      i++;
      sz[i] = 0x00;   //always null terminate the end
    }
  }
}

void GPS::getLongitude(char *sz) {
  byte i = 0;
  sz[0] = 0x00;   //always make sure we return null if no valid data in the source

  if (this->_bGGAComplete || this->_bRMCComplete) {
    while(this->_szLongitude[i] > 0x00 && i < _MAX_LONGITUDE_LEN) {
      sz[i] = this->_szLongitude[i];
      i++;
      sz[i] = 0x00;   //always null terminate the end
    }
  }
}

/**
 * @brief   Returns the appopriate APRS frequency for the current location.  This function is used to determine the frequency to use for APRS transmissions.
 * @param   sz: A pointer to a char array to store the frequency in.
 * @return: None.
 * @note    Attempts to determine the frequency based on lat/lon. If there is no valid GPS position, then the function will return the US frequency of 144.390 MHz.
 *            If the position is over the UK, Yemen, or North Korea, a 0.000 MHz frequency is returned indicating that no transmissions should be made.
 */
bool GPS::getAPRSFrequency(char *sz) {

	
	uint8_t freqSelected = 99;	//Default to 99, which is the International Space Station frequency of 145.825 MHz

	if (this->_bFixValid == false) {
		//we don't have a valid fix, so return the default US frequency of 144.3900 MHz
		freqSelected = 1;		// just default to the US Frequency of 144.3900 MHz if we don't have a Sat Lock
	} else {

		//Convert the latitude and longitude to an integer for comparison
		// Lat/Lon are in the format ddmm, and range from -9000 to 9000 and -18000 to 18000 respectively
		int iLat = atoi(this->_szLatitude);
		int iLon = atoi(this->_szLongitude);

		if (this->_cLatitudeHemi == 'S') iLat = -iLat;    //convert to negative if we're in the southern hemisphere
		if (this->_cLongitudeHemi == 'W') iLon = -iLon;    //convert to negative if we're in the western hemisphere


		Serial.print(F("Lat: "));
		Serial.println(iLat);
		Serial.print(F("Lon: "));
		Serial.println(iLon);

		//The table below is a list of the APRS frequencies for different regions of the world. The ordering must go from least specific, to most specific.
		// Any transmit-prohibited areas must be listed LAST, or else they will be overridden by the other regions.

		//Use the included XLSX file to maintain this list of lat/lons and associated frequencies.

		// Transmit geo-lookups from east specific, to most specific. Transmit prohibits must be at the bottom of this list.
		if (iLat >= 0 && iLat <= 8000 && iLon >= -13000 && iLon <= -3400) freqSelected = 1;    //US Canada Mexico on 144.3900MHz
		if (iLat >= -6000 && iLat <= 0 && iLon >= -10300 && iLon <= -3300) freqSelected = 1;    //South America on 144.3900MHz
		if (iLat >= -1000 && iLat <= 2000 && iLon >= 9600 && iLon <= 14200) freqSelected = 1;    //Indonesia/Malaysia/Singapore on 144.3900MHz
		if (iLat >= 2000 && iLat <= 5200 && iLon >= 6900 && iLon <= 13500) freqSelected = 4;    //China/Taiwan on 144.6400MHz
		if (iLat >= -4900 && iLat <= -3200 && iLon >= 16500 && iLon <= 18000) freqSelected = 3;    //New Zealand on 144.5750MHz
		if (iLat >= -4500 && iLat <= -900 && iLon >= 11100 && iLon <= 15400) freqSelected = 7;    //Austrailia on 145.1750MHz
		if (iLat >= 3600 && iLat <= 7300 && iLon >= -1200 && iLon <= 5000) freqSelected = 6;    //Europe on 144.8000MHz
		if (iLat >= -3600 && iLat <= 3600 && iLon >= -2100 && iLon <= 5200) freqSelected = 6;    //Africa on 144.8000MHz
		if (iLat >= 5200 && iLat <= 7500 && iLon >= 5000 && iLon <= 18000) freqSelected = 6;    //Russia on 144.8000MHz
		if (iLat >= 5000 && iLat <= 8000 && iLon >= -16900 && iLon <= -13000) freqSelected = 1;    //Alaska on 144.3900MHz
		if (iLat >= 1500 && iLat <= 2600 && iLon >= -16500 && iLon <= -15300) freqSelected = 1;    //Hawaii on 144.3900MHz
		if (iLat >= -3000 && iLat <= 300 && iLon >= -7000 && iLon <= -3300) freqSelected = 9;    //Brazil on 145.5700MHz
		if (iLat >= 3000 && iLat <= 4500 && iLon >= 12900 && iLon <= 14600) freqSelected = 5;    //Japan on 144.6600MHz
		if (iLat >= 500 && iLat <= 2036 && iLon >= 9700 && iLon <= 10600) freqSelected = 8;    //Thailand on 145.5300MHz
		if (iLat >= 2200 && iLat <= 2230 && iLon >= 11348 && iLon <= 11430) freqSelected = 2;    //Hong Kong on 144.5250MHz
		if (iLat >= 4900 && iLat <= 6100 && iLon >= -800 && iLon <= 200) freqSelected = 0;    //UK on 000.0000MHz
		if (iLat >= 1148 && iLat <= 1912 && iLon >= 4200 && iLon <= 5442) freqSelected = 0;    //Yemen on 000.0000MHz
		if (iLat >= 3742 && iLat <= 4306 && iLon >= 13100 && iLon <= 12400) freqSelected = 0;    //North Korea on 000.0000MHz
	}

	switch (freqSelected) {
	case 0:
		//Transmit Prohibited
		strcpy(sz, "000.0000");
		break;
	case 1:
		//North America, US, Canada, Mexico
		strcpy(sz, "144.3900");
		break;
	case 2:
		//Hong Kong
		strcpy(sz, "144.5250");
		break;
	case 3:	
		//New Zealand
		strcpy(sz, "144.5750");
		break;
	case 4:
		//China/Taiwan
		strcpy(sz, "144.6400");
		break;
	case 5:
		//Japan
		strcpy(sz, "144.6600");
		break;
	case 6:
		//Europe/Africa/Russia
		strcpy(sz, "144.8000");
		break;
	case 7:
		//Australia
		strcpy(sz, "145.1750");
		break;
	case 8:
		//Thailand
		strcpy(sz, "145.5300");
		break;
	case 9:
		//Brazil
		strcpy(sz, "145.5700");
		break;
	case 99:
		//ISS Satellite
		strcpy(sz, "145.8250");
		break;
	default:
		strcpy(sz, "144.3900");
		break;
	}
	
	
	return (freqSelected != 0);		//return false if transmissions are prohibited, true if they are allowed
	
}
