/*
APRS Data Modem for Project: Traveler Flight Controllers
Copyright 2011-2025 - Zack Clobes (W0ZC), Custom Digital Services, LLC

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.

Version History:
Version 2.0.0 - March 9, 2025 - Major refactoring to make use of the DRA/SA818V transmitter module. Based on the prior TNC module from the ptFlex-series of trackers.

*/

#include "Modem.h"
#include "Arduino.h"
#include <SoftwareSerial.h>


Modem::Modem(void) {
  //Initializer
}


/**
 * @brief Initializes the APRS object with the pins for the transmitter and serial port.
 * @param pinEnable The pin to enable the transmitter.
 * @param pinPTT The pin to key up the transmitter.
 * @param pinTxAudio The pin to send audio to the transmitter.
 * @param pinSerialTx The pin to transmit serial data to the 818V module.
 * @param pinSerialRx The pin to receive serial data from the 818V module.
 */
void Modem::init(uint8_t pinEnable, uint8_t pinPTT, uint8_t pinTxAudio, uint8_t pinSerialTx, uint8_t pinSerialRx) {
  this->_pinEnable = pinEnable;
  this->_pinPTT = pinPTT;
  this->_pinTxAudio = pinTxAudio;
  this->_pinSerialTx = pinSerialTx;
  this->_pinSerialRx = pinSerialRx;
  this->_txDelay = 30;    //default to 15 if not otherwise defined.

  this->PTT(false);   //make sure the transmitter is unkeyed

  this->configTimers();
}


/**
 * @brief  Keys up the 818V transmitter. Passes through the Tx/Rx frequency during the process.
 * @param  tx: A boolean indicating whether or not to key up the transmitter or unkey into receive mode.
 * @notes  The configuration of the module should be done each time the chip is taken out of PowerDown mode.
 */
void Modem::PTT(bool tx) {

  char response;
  long start;

  if (tx) {
    //Turn on the transmitter
    digitalWrite(this->_pinEnable, HIGH);
    
    //Configure the serial port
    SoftwareSerial DRA(this->_pinSerialRx, this->_pinSerialTx, false);
    DRA.begin(9600);
    delay(100);

    //Connect to the radio chip
    if (this->_debugLevel >0) Serial.println("Sending DMOConnect...");
    DRA.print(F("AT+DMOCONNECT\r\n"));
    
    //Get the response:
    start = millis();
    do {
      if (DRA.available()) {
        response = DRA.read();
        if (this->_debugLevel == 2) Serial.print(response);     //debug the output from the response
      }
    } while (response != '0' && (millis() - start) < MAX_WAIT_TIMEOUT);
    if (this->_debugLevel ==2) Serial.println("");
    if (this->_debugLevel >0) Serial.println("End Connect Response.");
    delay(100);

    //Configure the transceiver
    if (this->_debugLevel >0) Serial.println("Sending DMO Set Group...");
    DRA.print(F("AT+DMOSETGROUP=0,"));
    DRA.print("144.3900");
    DRA.print(",");
    DRA.print("144.3900");
    DRA.print(F(",0000,4,0000\r\n"));   //No CTCSS Tx, Sql 4, No CTCSS Rx

    //Get the response:
    start = millis();
    do {
      if (DRA.available()) {
        response = DRA.read();
        if (this->_debugLevel == 2) Serial.print(response);     //debug the output from the response
      }
    } while (response != '0' && (millis() - start) < MAX_WAIT_TIMEOUT);    
    if (this->_debugLevel ==2) Serial.println("");
    if (this->_debugLevel >0) Serial.println("End DMO Set Response.");
    delay(100);

    //Push the PTT
    digitalWrite(this->_pinPTT, HIGH);   //There's a delay of about 37mS from PTT going high to when RF is emitted.
  } else {
    //End of transmission. stop the PTT and shut down the transmitter
    digitalWrite(this->_pinPTT, LOW);
    digitalWrite(this->_pinEnable, LOW);
  }

}

/**
 * @brief  Sets the debug level for the APRS object.
 * @param  level: The debug level to set. 0=none, 1=Normal Serial.printlns, 2=Verbose feedback from 818V module
 */
void Modem::setDebugLevel(uint8_t level) {
  this->_debugLevel = level;
}




/**
 * @brief Starts the process of sending an APRS packet. Header information is passed in to address and route the packet appropriately.
 * @param szDest The destination callsign of the packet
 * @param destSSID The destination SSID of the packet
 * @param szCall The source callsign of the packet
 * @param callSSID The source SSID of the packet
 * @param szPath1 The first path of the packet
 * @param path1SSID The first path's SSID
 * @param szPath2 The second path of the packet
 * @param path2SSID The second path's SSID
 * @param bUsePath A boolean indicating whether or not to use the path information
 * @note  This function is called to start the transmission of a packet.  It will start the timer and begin the process of transmitting the packet.  Calling this function indicates that the packet is ready to be transmitted.  The packet will be transmitted in the background, and the function will return immediately.
 */
void Modem::packetHeader(char *szDest, char destSSID, char *szCall, char callSSID, char *szPath1, char path1SSID, char *szPath2, char path2SSID, bool bUsePath) {

  uint8_t i;
  
  //Add the destination address
  for (i=0; i<6; i++) {
    this->packetAppend((char)(szDest[i] << 1));
  }
  this->packetAppend((char)(destSSID << 1));
  
  //Add the from callsign
  for (i=0; i<6; i++) {
    this->packetAppend((char)(szCall[i] << 1));
  }
  
  if ((szPath1[0] != ' ') && (bUsePath)) {
    //there's a path
    
    //don't end the callsign section
    this->packetAppend((char)((callSSID << 1) | 0x00));    //DON'T flag the last bit with a 1 to indicate end of string

    //dump the Path1
    for (int j=0; j<6; j++) {
      this->packetAppend((char)(szPath1[j] << 1));
    }
    if (szPath2[0] != ' ') {
      //there's a second path
      
      //don't end the Path1 section
      this->packetAppend((char)((path1SSID << 1) | 0x00));    //DON'T flag the last bit with a 1 to indicate end of string
      
      for (int k=0; k<6; k++) {
        this->packetAppend((char)(szPath2[k] << 1));
      }
      
      //Max of 2 paths, so always end here
      this->packetAppend((char)((path2SSID << 1) | 0x01));    //flag the last bit with a 1 to indicate end of string
      
    } else {
      //This was the only path - end it
      this->packetAppend((char)((path1SSID << 1) | 0x01));    //flag the last bit with a 1 to indicate end of string
    }
  } else {
    //this is the end of the callsign, there was no Paths
    this->packetAppend((char)((callSSID << 1) | 0x01));    //flag the last bit with a 1 to indicate end of string
  }

  
  this->packetAppend((char)0x03);    //Control Byte
  this->packetAppend((char)0xF0);    //PID    

}


/** 
 * @brief Appends a string to the packet buffer.  This function will append the string to the packet buffer, and the packet will be transmitted when the packetSend() function is called.
 * @param sz The string to append to the packet buffer.
 */
void Modem::packetAppend(char *sz) {
  while (*sz != 0) {
    this->packetAppend(*sz);
    sz++;
  }	
}


/**
 * @brief Appends a character to the packet buffer.  This function will append the character to the packet buffer, and the packet will be transmitted when the packetSend() function is called.
 * @param c The character to append to the packet buffer.
 */
void Modem::packetAppend(char c) {
  if (_iSZLen < (MAX_SZXMIT_SIZE - 2)) {
    //we still have room in the array
    _szXmit[++_iSZLen] = c;
    _szXmit[_iSZLen + 1] = '\0';    //null terminate
  }  
}


/**
 * @brief Appends a float to the packet buffer.  This function will append the float to the packet buffer, and the packet will be transmitted when the packetSend() function is called.
 * @param f The float to append to the packet buffer.
 */
void Modem::packetAppend(float f) {
  //prints a float to the TNC, with a single decimal point of resolution.  Sufficient for voltages and temperatures.
  
  if (f < 0) {
    this->packetAppend('-');    //negative sign
    f = f * -1;        //convert the negative number to positive for the rest of the processing
  }
  
  this->packetAppend((long)int(f), false);  //prints the int part
  this->packetAppend('.'); // print the decimal point
  int iDec = (f - int(f)) * 10;    //calculate the decimal point portion
  this->packetAppend((long)iDec, false) ; 
  
}


/**
 * @brief Appends a long to the packet buffer.  This function will append the long to the packet buffer, and the packet will be transmitted when the packetSend() function is called.
 * @param lNumToSend The long to append to the packet buffer.
 * @param bLeadingZero A boolean indicating whether or not to pad the number with leading zeros.
 */
void Modem::packetAppend(long lNumToSend, bool bLeadingZero) {
//This function writes a long numeric data type to the packet, zero padded to 6 digits (for the /A= Altitude report
  int iCnt = 0;
  
  iCnt = 0;
  while (lNumToSend >= 100000) {
    lNumToSend -= 100000;
    iCnt++;
  }
  if (iCnt == 0) {
    if (bLeadingZero == true) {
      this->packetAppend('0');
    }
  } else {
    this->packetAppend((char)(iCnt + 48));
    bLeadingZero = true;    //we've sent a digit, so now always send subsequent zeros
  }
  
  iCnt = 0;
  while (lNumToSend >= 10000) {
    lNumToSend -= 10000;
    iCnt++;
  }

  if (iCnt == 0) {
    if (bLeadingZero == true) {
      this->packetAppend('0');
    }
  } else {
    this->packetAppend((char)(iCnt + 48));
    bLeadingZero = true;    //we've sent a digit, so now always send subsequent zeros
  }
  
  iCnt = 0;
  while (lNumToSend >= 1000) {
    lNumToSend -= 1000;
    iCnt++;
  }
  if (iCnt == 0) {
    if (bLeadingZero == true) {
      this->packetAppend('0');
    }
  } else {
    this->packetAppend((char)(iCnt + 48));
    bLeadingZero = true;    //we've sent a digit, so now always send subsequent zeros
  }
  
  iCnt = 0;
  while (lNumToSend >= 100) {
    lNumToSend -= 100;
    iCnt++;
  }
  if (iCnt == 0) {
    if (bLeadingZero == true) {
      this->packetAppend('0');
    }
  } else {
    this->packetAppend((char)(iCnt + 48));
    bLeadingZero = true;    //we've sent a digit, so now always send subsequent zeros
  }
  
  iCnt = 0;
  while (lNumToSend >= 10) {
    lNumToSend -= 10;
    iCnt++;
  }
  if (iCnt == 0) {
    if (bLeadingZero == true) {
      this->packetAppend('0');
    }
  } else {
    this->packetAppend((char)(iCnt + 48));
    bLeadingZero = true;    //we've sent a digit, so now always send subsequent zeros
  }
  
  this->packetAppend((char)(lNumToSend + 48));        //always send the ones digit
}


/**
 * @brief This function is called to start the transmission of a packet.  It will start the timer and begin the process of transmitting the packet.
 * @note  Calling this function indicates that the packet is ready to be transmitted.  For internally modulated signals (not KISS), the signal will transmit through the timer ISR, but this function will wait until it's finished to return.
 */
void Modem::packetSend() {

  //Initialize the state machine
  _iTxState = 0;
  _iSZPos = 0;
  _iTxDelayRemaining = _txDelay;    //start off with a txDelay parameter
  _CRC = 0xFFFF;    //init the CRC variable


  //Key the transmitter
  this->PTT(true);

  //Start the interrupt routine to modulate the signal
  this->timer1ISR(true);

  //wait for the state machine to get to a State 5, which is when it shuts down the transmitter.
  while (_iTxState != 5) {
    delay(100);    //just wait patiently until the packet is done
  }

  _iSZLen = -1;      //reset back to a clean buffer
}



/**
 * @brief  Sends a test tone to the transmitter.  This function is used to test the transmitter and the audio path.
 */
void Modem::sendTestDiagnotics() {


  this->PTT(true);
  delay(DIAGNOSTIC_DELAY);   //deadkey before starting the tones.

  this->_iTxState = 11;    //set the state to 11 to indicate a constant test tone
  this->timer1ISR(true);
  delay(DIAGNOSTIC_DELAY);

  //Change to the high tone
  this->_iTxState = 12;    //temporarily set the state to 12 to flip the tone to the opposite.
  delay(DIAGNOSTIC_DELAY);

  //Alternate the tones
  this->_iTxState = 13;    //set the state to 11 to indicate a constant test tone
  delay(DIAGNOSTIC_DELAY);

  this->timer1ISR(false);   //stop the tones
  this->_iTxState = 0;
  delay(DIAGNOSTIC_DELAY);   //dead key for a little bit

  this->PTT(false);   //shut down the transmitter
}


/**
 * @brief  Calculates the CRC Checksums for the AX.25 packet.
 * @param  iBit: The bit to calculate the CRC for.
 */
void Modem::calcCRC(uint8_t iBit) {
  unsigned int xor_int;
  
  //iBit = iBit & 0x01;    //strip of all but LSB
  xor_int = _CRC ^ iBit;				// XOR lsb of CRC with the latest bit
  _CRC >>= 1;									// Shift 16-bit CRC one bit to the right

  if (xor_int & 0x0001) {					// If XOR result from above has lsb set
    _CRC ^= 0x8408;							// Shift 16-bit CRC one bit to the right
  }
  return;
}


/**
 * @brief  The main component of the transmitting State Machine. After a packetSend() command is sent, this function gets called repeatedly from the ISR timer routine to output the packet bit-by-bit..
 * @return The next bit in the packet to be transmitted.
 */
uint8_t Modem::getNextBit(void) {
  static int iRotatePos = 0;
  uint8_t bOut = 0;
  

  switch (_iTxState) {
  case 0:
    //TX Delay transmitting flags
    _bNoStuffing = true;
    bOut = ((0x7e & (1 << iRotatePos)) != 0);    //get the bit
  
    
    if (iRotatePos != 7) iRotatePos++;
    else {
      //rotate around for the next bit

      iRotatePos = 0;
     
      if (_iTxDelayRemaining > 0) {
        //we have more txdelays
        _iTxDelayRemaining--;
      } else {
        _iTxState = 1;    //we need to drop down to the next state
      }
    }
    break;
  case 1:
    //normal packet payload
    _bNoStuffing = false;
    bOut = ((_szXmit[_iSZPos] & (1 << iRotatePos)) != 0);    //get the bit
  
    if (iRotatePos != 7) iRotatePos++;
    else {
      //we need to get the next byte if possible

      iRotatePos = 0;
      
      if (_iSZPos < _iSZLen) {
        //we have more bytes
        _iSZPos++;
      } else {
        _iTxState = 2;    //drop down and transmit the CRC bytes
      }
    }
    
    
    //calculate the CRC for the data portion of the packet
    this->calcCRC(bOut);
    
    break;  
  case 2:  
    //Transmit the CRC Check bits (first byte)
  
    bOut = (((lowByte(_CRC) ^ 0xff) & (1 << iRotatePos)) != 0);    //get the bit
  
    if (iRotatePos != 7) iRotatePos++;
    else {
      iRotatePos = 0;
      _iTxState = 3;    //we need to drop down to the next state
    }  
    break;
  
  case 3:
    //Transmit the CRC Check bits (second byte)
    bOut = (((highByte(_CRC) ^ 0xff) & (1 << iRotatePos)) != 0);    //get the bit
  
    if (iRotatePos != 7) iRotatePos++;
    else {
      iRotatePos = 0;
      _iTxState = 4;    //we need to drop down to the next state
    }  
    break;

  case 4:  
    //send end-of-packet flag
  
    _bNoStuffing = true;
    bOut = ((0x7e & (1 << iRotatePos)) != 0);    //get the bit
  
    if (iRotatePos != 7) iRotatePos++;
    else {
      _iTxState = 5;    //we need to drop down to the next state
    } 
    break;


  case 5:
    //done - shut the transmitter down
    bOut = 1;    //bogus bit so we have something to return.  Should be a moot point, since the ISR is shutting down.
    iRotatePos = 0;    //reset to get ready for next byte
    this->timer1ISR(false);    //stop the timer
    
    //Unkey the transmitter
    this->PTT(false);
  
    break;
  
  case 11:
    //Send a constant tone
    this->_bNoStuffing = true;
    bOut = 1;    //send high which means stay with whatever tone was used last
    break;
  case 12:
    //swap the tone
    bOut = 0;    //swap the tone
    this->_iTxState = 11;    //After it's been swapped, go back to the constant tone
    break;
  case 13:
    //alternate the tones
    bOut = 0;    //send a low, which means switch between high and low tones repeatedly
    break;

  default:
    bOut = 2;
  }

  return bOut;  
  
}

/**
 * @brief  Returns the current state of the noBitStuffing flag.  This flag is used to indicate whether or not to stuff an extra bit into the packet because of an excessive number of 1's in a row.
 * @return A boolean indicating whether or not to stuff an extra bit into the packet.
 * @note  This function is called from the ISR routine.  It must be public for now.
 */
bool Modem::noBitStuffing() {
  return this->_bNoStuffing;  
}




/**
 * @brief  Configures the ISR times to create the AX.25 packet and the audio wave forms.
 */
void Modem::configTimers() {
  TCCR1A = 0x00;
  TCCR1B = 0x09;    //Set WGM12 high, and set CS=001 (which is clk/1);
  
  OCR1A = TIMER1_SEED;

  //Timer2 drives the PWM frequency.  We need it sufficiently higher than the 1200/2200hz tones
  TCCR2B = 0<<CS22 | 0<<CS21 | 1<<CS20;      //sets timer2 prescaler to clk - Approx 32kHz PWM freq.
}

/**
 * @brief  Starts and stops the Timer1 ISR routine.
 * @param  run: A boolean indicating to either start or stop the ISR routine.
 */
void Modem::timer1ISR(bool run) {
  if (run) sbi(TIMSK1, OCIE1A);
  else cbi(TIMSK1, OCIE1A);
}


/**
 * @brief  Sets the delay between the end of the packet and the transmitter being unkeyed.
 * @param  txDelay: The number of bytes to cycle through before the packet is transmitted.
 */
void Modem::setTxDelay(unsigned int txDelay) {
  _txDelay = txDelay;
}


/**
 * @brief  Returns the pin number for the PTT line.
 * @return The pin number for the PTT line.
 */
uint8_t Modem::getPinTxAudio() {
  return this->_pinTxAudio;
}


/**
 * @brief  The ISR routine for the Timer1 interrupt.  This routine is called every time the timer overflows.
 * @note  This function is called from the ISR routine.
 */
/*
void handleInterrupt() {
  // Code to handle the interrupt
  //static APRS* instance;

  static boolean bBaudFlip = 0;
  static byte iStuffZero = 0;
  static boolean bStuffBit = false;


  static byte iRateGen;
  static unsigned int iTonePhase = 0;      //two byte variable.  The highByte contains the element in arySine that should be output'ed
  static boolean bToneHigh = 0;

  Serial.println("c");


  //increment the phase counter.  It will overflow automatically at > 65535
  if (bToneHigh) {
    analogWrite(Modem::instance->getPinTxAudio(), (pgm_read_byte_near(_arySineHigh + highByte(iTonePhase))));
    iTonePhase += TONE_HIGH_STEPS_PER_TICK;
  } else {
    analogWrite(Modem::instance->getPinTxAudio(), (pgm_read_byte_near(_arySineLow + highByte(iTonePhase))));
    iTonePhase += TONE_LOW_STEPS_PER_TICK;
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

      if (Modem::instance->getNextBit() == 0) {
        //we only flip the output state if we have a zero

        //Flip Bit
        bToneHigh = !bToneHigh;
        iStuffZero = 0;
      } else {
        //it's a 1, so send the same tone...

        iStuffZero++;      //increament the stuffing counter

        //if there's been 5 in a row, then we need to stuff an extra bit in, and change the tone for that one
        if (iStuffZero == 5 && !Modem::instance->noBitStuffing()) {
          bStuffBit = true;      //once we hit five, we let this fifth (sixth?) one go, then we set a flag to flip the tone and send a bogus extra bit next time
        }
      }
    }

    iRateGen = BAUD_GENERATOR_COUNT;
  }
}
*/



