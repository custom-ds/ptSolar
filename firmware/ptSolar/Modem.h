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


//Timer seed for calculating the AFSK and 1200 baud rates
#define TIMER1_SEED 333 // 16MHz / 666 = 24.024kHz.  24kHz / 20 / 1201Hz baud

//Maximum size of the transmit buffer
#define MAX_SZXMIT_SIZE 200

//Maximum time to wait for a response from the radio transmitter module (in milliseconds)
#define MAX_WAIT_TIMEOUT 2000

//mS Delay when performaing dianostics test transmission
#define DIAGNOSTIC_DELAY 5000


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

/*
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
*/



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

    static const uint16_t TONE_HIGH_STEPS_PER_TICK = 6001;
    static const uint16_t TONE_LOW_STEPS_PER_TICK = 3273;
    static const uint8_t BAUD_GENERATOR_COUNT = 20;

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