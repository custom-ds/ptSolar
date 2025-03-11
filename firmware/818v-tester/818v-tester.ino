/* 

This is a similified version of the 818V code to test the transmit functionalityon the ptSolar PCB.

*/




#include "APRS.h"

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

#define DELAY_TX 300
#define DELAY_RX 400
#define DELAY_BETWEEN 3000

APRS aprs;            //APRS and Transmitter object for the 818V



void setup() {
  Serial.begin(19200);
  
  //Configure the Transmitter Pins
	pinMode(PIN_PTT_OUT, OUTPUT);
  pinMode(PIN_DRA_EN, OUTPUT);
  pinMode(PIN_AUDIO_OUT, OUTPUT);

  //Configure the GPS Pins
  pinMode(PIN_GPS_EN, OUTPUT);

  //Configure the APRS modem with the pins to connect to the transmitter
  aprs.init(PIN_DRA_EN, PIN_PTT_OUT, PIN_AUDIO_OUT, PIN_DRA_TX, PIN_DRA_RX);
  aprs.setDebugLevel(2);
/*
  APRS* aprs::instance = nullptr;   // Initialize the static instance pointer
  aprs::instance = &aprs; // Set the instance pointer to the timer object
*/
  //Shut everything down until we get booted
  digitalWrite(PIN_PTT_OUT, LOW);    //Stop the transmit - 
  digitalWrite(PIN_DRA_EN, LOW);    //disable the transmitter
  digitalWrite(PIN_GPS_EN, LOW);    //disable the GPS

  //Configure the Misc Pins
  pinMode(PIN_LED, OUTPUT);
  //pinMode(PIN_AUDIO, OUTPUT);
  
  Serial.begin(19200);
  Serial.println("ptSolar Test Fixture for troubleshooting SA818V Transmit Module");




}

void loop() {

  char Callsign[7];    //6 digit callsign + Null
  strcpy(Callsign, "N0CALL");

  char CallsignSSID = '0';

  char Destination[7];
  strcpy(Destination, "APRS  ");

  char DestinationSSID = '0';

  char Path1[7];
  strcpy(Path1, "WIDE2 ");

  char Path1SSID = '1';

  char Path2[7];
  strcpy(Path2, "      ");

  char Path2SSID = '0';


  Serial.println("Transmitting 6x");
  for (int i=0;i<6;i++) {
    aprs.PTT(true);
    delay(DELAY_TX);
    aprs.PTT(false);
    delay(DELAY_RX);
  }
  Serial.println("Done with the dead keys.");

  delay(DELAY_BETWEEN);   //wait X seconds and repeat

  Serial.println("Transmitting an APRS packet");

  for (int i=1;i<=3;i++) {
    aprs.packetHeader(Destination, DestinationSSID, Callsign, CallsignSSID, Path1, Path1SSID, Path2, Path2SSID, true);
    aprs.packetAppend((char *)">Project Traveler ptSolar Flight Computer ");
    aprs.packetAppend((long)i, false);
    aprs.packetSend();
    delay(DELAY_RX);

  }
  Serial.println("Done with the packet.");

  delay(DELAY_BETWEEN);   //wait X seconds and repeat

}

/*
// Define the ISR
ISR(TIMER1_COMPA_vect) {
  Serial.println("a");
    aprs::isr();    //Call the static ISR
}
*/


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
    iTonePhase += APRS::TONE_HIGH_STEPS_PER_TICK;
  } else {
    //analogWrite(PIN_AUDIO_OUT, (pgm_read_byte_near(_arySineLow + highByte(iTonePhase))));
    analogWrite(PIN_AUDIO_OUT, (pgm_read_byte_near(_arySineLow + highByte(iTonePhase))));
    iTonePhase += APRS::TONE_LOW_STEPS_PER_TICK;
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

      if (aprs.getNextBit() == 0) {
        //we only flip the output state if we have a zero

        //Flip Bit
        bToneHigh = !bToneHigh;
        iStuffZero = 0;
      } else {
        //it's a 1, so send the same tone...

        iStuffZero++;      //increament the stuffing counter

        //if there's been 5 in a row, then we need to stuff an extra bit in, and change the tone for that one
        if (iStuffZero == 5 && !aprs.noBitStuffing()) {
          bStuffBit = true;      //once we hit five, we let this fifth (sixth?) one go, then we set a flag to flip the tone and send a bogus extra bit next time
        }
      }
    }

    iRateGen = APRS::BAUD_GENERATOR_COUNT;
  }
}
 






