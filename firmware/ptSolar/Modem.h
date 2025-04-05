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

const uint8_t arySin[] PROGMEM = {2, 5, 7, 10, 12, 15, 17, 20, 22, 24, 
  27, 29, 31, 34, 36, 38, 41, 43, 45, 47, 
  49, 51, 53, 56, 58, 60, 62, 63, 65, 67, 
  69, 71, 72, 74, 76, 77, 79, 80, 82, 83, 
  84, 86, 87, 88, 89, 90, 91, 92, 93, 94, 
  95, 96, 96, 97, 98, 98, 99, 99, 99, 100, 
  100, 100, 100, 100, 100, 100, 100, 100, 99, 99, 
  99, 98, 98, 97, 96, 96, 95, 94, 93, 92, 
  91, 90, 89, 88, 87, 86, 84, 83, 82, 80, 
  79, 77, 76, 74, 72, 71, 69, 67, 65, 63, 
  62, 60, 58, 56, 53, 51, 49, 47, 45, 43, 
  41, 38, 36, 34, 31, 29, 27, 24, 22, 20, 
  17, 15, 12, 10, 7, 5, 2, 0};
  

//Maximum size of the transmit buffer
#define MAX_SZXMIT_SIZE 200

//Maximum time to wait for a response from the radio transmitter module (in milliseconds)245
#define MAX_WAIT_TIMEOUT 2000

//mS Delay when performaing dianostics test transmission
#define DIAGNOSTIC_DELAY 1500


class Modem {
  public:
    // Constructor
    Modem(uint8_t pinEnable, uint8_t pinPTT, uint8_t pinTxAudio, uint8_t pinSerialTx, uint8_t pinSerialRx);

    // Public Functions
    void PTT(bool tx);

    void setTxFrequency(char *szFreq) { strcpy(this->_szTxFreq, szFreq); }
    void setRxFrequency(char *szFreq) { strcpy(this->_szRxFreq, szFreq); }

    void packetHeader(char *szDest, char destSSID, char *szCall, char callSSID, char *szPath1, char path1SSID, char *szPath2, char path2SSID, bool usePath);
    void packetAppend(char *sz);
    void packetAppend(char c);
    void packetAppend(float f);
    void packetAppend(long lNumToSend, bool bLeadingZero);
    void packetSend();

    void sendTestDiagnotics();

    bool getNextBit();       //Called from the ISR routine. Must be public for now
    bool noBitStuffing();    //Called from the ISR routine. Must be public for now

    void setDebugLevel(uint8_t level);
    void setTxDelay(unsigned int txDelay);
    uint8_t getPinTxAudio();

    inline uint8_t getDACValue(uint8_t iPhase) {
          if (iPhase & 0x80) return 100 + pgm_read_byte(&arySin[(iPhase & 0x7f)]);    //first half of the sine wave
          return 100 - pgm_read_byte(&arySin[(iPhase & 0x7f)]);    //second half of the sine wave
    }


	  //Parameters for the Numerically Controlled Oscillator NCO. See notes in the ConfigureTimers() function for details
    static const uint8_t BAUD_GENERATOR_COUNT = 22;
    static const uint16_t TIMER1_OCR = 303;
    static const uint16_t TONE_HIGH_STEPS_PER_TICK = 5461;		//2200Hz Tone
    static const uint16_t TONE_LOW_STEPS_PER_TICK = 2979; 		//1200Hz Tone

  private:
    // Private Variables
    uint8_t _pinEnable;
    uint8_t _pinPTT;
    uint8_t _pinTxAudio;
    uint8_t _pinSerialTx;
    uint8_t _pinSerialRx;

    uint8_t _debugLevel;

    char _szTxFreq[9];    //array to hold the transmit frequency
    char _szRxFreq[9];    //array to hold the receive frequency


    char _szXmit[MAX_SZXMIT_SIZE];    //array to hold the data to be transmitted
    int _iSZLen;    //Tracks the current size of the szXmit buffer  

    unsigned int _txDelay;
    uint8_t _iSZPos = 0;    //Tracks the current byte being modulated out of the modem
    int _iTxDelayRemaining = 0;
    bool _bNoStuffing = false;
    unsigned int _CRC;
    uint8_t _iTxState = 0;


    // Private Functions
    void calcCRC(bool bit);
    void configTimers();
    void timer1ISR(bool run);

};
#endif