/*
APRS Data Modem for Project: Traveler Flight Controllers
Copyright 2011-2025 - Zack Clobes (W0ZC), Custom Digital Services, LLC

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef Modem_h
#define Modem_h


#include <stdint.h>   //standard data types available, such as uint8_t
#include <avr/io.h>
#include <avr/interrupt.h>
#include <arduino.h>

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


//Maximum size of the transmit buffer
#define MAX_SZXMIT_SIZE 200

//Maximum time to wait for a response from the radio transmitter module (in milliseconds)245
#define MAX_WAIT_TIMEOUT 2000

//mS Delay when performaing dianostics test transmission
#define DIAGNOSTIC_DELAY 1500

//Sinewave lookup table
const uint8_t arySin[] PROGMEM = {131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 
  161, 164, 167, 169, 172, 175, 178, 181, 183, 186, 
  189, 191, 194, 196, 199, 201, 204, 206, 208, 211, 
  213, 215, 217, 219, 221, 223, 225, 227, 229, 230, 
  232, 234, 235, 236, 238, 239, 240, 242, 243, 244, 
  245, 246, 247, 247, 248, 249, 249, 250, 250, 250, 
  251, 251, 251, 251, 251, 251, 251, 250, 250, 250, 
  249, 249, 248, 247, 247, 246, 245, 244, 243, 242, 
  240, 239, 238, 236, 235, 234, 232, 230, 229, 227, 
  225, 223, 221, 219, 217, 215, 213, 211, 208, 206,
  204, 201, 199, 196, 194, 191, 189, 186, 183, 181, 
  178, 175, 172, 169, 167, 164, 161, 158, 155, 152, 
  149, 146, 143, 140, 137, 134, 131, 128, 125, 122, 
  119, 116, 113, 110, 107, 104, 101, 98, 95, 92, 
  89, 87, 84, 81, 78, 75, 73, 70, 67, 65, 
  62, 60, 57, 55, 52, 50, 48, 45, 43, 41, 
  39, 37, 35, 33, 31, 29, 27, 26, 24, 22, 
  21, 20, 18, 17, 16, 14, 13, 12, 11, 10, 
  9, 9, 8, 7, 7, 6, 6, 6, 5, 5, 
  5, 5, 5, 5, 5, 6, 6, 6, 7, 7, 
  8, 9, 9, 10, 11, 12, 13, 14, 16, 17, 
  18, 20, 21, 22, 24, 26, 27, 29, 31, 33, 
  35, 37, 39, 41, 43, 45, 48, 50, 52, 55, 
  57, 60, 62, 65, 67, 70, 73, 75, 78, 81, 
  84, 87, 89, 92, 95, 98, 101, 104, 107, 110, 
  113, 116, 119, 122, 125, 128};

class Modem {
  public:
    // Constructor
    Modem(uint8_t pinEnable, uint8_t pinPTT, uint8_t pinTxAudio, uint8_t pinSerialTx, uint8_t pinSerialRx);

    // Public Functions
    void PTT(bool tx);

    void packetHeader(char *szDest, char destSSID, char *szCall, char callSSID, char *szPath1, char path1SSID, char *szPath2, char path2SSID, bool usePath);
    void packetAppend(char *sz);
    void packetAppend(char c);
    void packetAppend(float f);
    void packetAppend(long lNumToSend, bool bLeadingZero);
    void packetSend();

    void sendTestDiagnotics();

    uint8_t getNextBit();       //Called from the ISR routine. Must be public for now
    bool noBitStuffing();    //Called from the ISR routine. Must be public for now

    void setDebugLevel(uint8_t level);
    void setTxDelay(unsigned int txDelay);
    uint8_t getPinTxAudio();

	//Parameters for the Numerically Controlled Oscillator NCO. See notes in the ConfigureTimers() function for details
    static const uint8_t BAUD_GENERATOR_COUNT = 22;
    static const uint16_t TIMER1_OCR = 301;
    static const uint16_t TONE_HIGH_STEPS_PER_TICK = 5461;		//2200Hz Tone
    static const uint16_t TONE_LOW_STEPS_PER_TICK = 2979; 		//1200Hz Tone

/*
// functions to hide the ISR inside of the class. Can't get the self-reference (instance) to work
void handleInterrupt();
static APRS* instance; // Declare the static instance pointer

static void isr() {
  Serial.println("b");
  if (instance) {
    Serial.println("b2");
    instance->handleInterrupt();
  }
}
*/

  private:
    // Private Variables
    uint8_t _pinEnable;
    uint8_t _pinPTT;
    uint8_t _pinTxAudio;
    uint8_t _pinSerialTx;
    uint8_t _pinSerialRx;

    uint8_t _debugLevel;

    char _szXmit[MAX_SZXMIT_SIZE];    //array to hold the data to be transmitted
    int _iSZLen;    //Tracks the current size of the szXmit buffer  

    char _transmitterType;
    unsigned int _txDelay;
    char _courtesyTone;
    uint8_t _iSZPos = 0;    //Tracks the current byte being modulated out of the modem
    int _iTxDelayRemaining = 0;
    bool _bNoStuffing = false;
    unsigned int _CRC;
    uint8_t _iTxState = 0;


    // Private Functions
    void calcCRC(uint8_t iBit);
    void configTimers();
    void timer1ISR(bool run);

};
#endif