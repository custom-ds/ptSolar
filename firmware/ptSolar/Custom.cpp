/*
Custom Sensor Configurations for Project: Traveler Flight Controllers
Copywrite 2011-2019 - Zack Clobes (W0ZC), Custom Digital Services, LLC


This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.



This file is intended to be modified by the end-user to accomidate any custom programming.

*/


#include <Arduino.h>
#include "TNC.h"
#include "GPS.h"



/* Custom Global Variables */





/* customInit()
 *  
 *  Called once during the boot/initialization process.  Generally used to initialize sensors and variables.
 *  
 */
void customInit() {
//  Enviro.initialize();




}


/* customLoop()
 *  
 *  Called continuously during the Arduino "Loop".  Practically speaking, this gets executed approximatley once a second, but can vary.  Returning true from
 *  this function will cause the transmitter to send a Single Line Position report.  Be careful returning true, as packets could be potentially sent continously
 *  which would drain the batter, and jam the APRS channel for other users.
 *  
 */
bool customLoop(GPS& GPSParser) {
  

  return false;   //Returning true will cause the transmitter to send a packet.  Be very careful how often this is transmitted.
}


/* customSendPositionSingleLine()
 *  
 *  Called towards the end of the packet send process, after any built-in telemetry data is sent (e.g. battery, temperature, etc) but before the comment string
 *  is transmitted.  Be aware that any custom output should be limited to roughly 50 characters max.
 *  
 */
void customSendPositionSingleLine(bool transmitCustom, TNC& oTNC, GPS& GPSParser) {

  if (!transmitCustom) return;    //we don't want to transmit anything custom here



    
}


/* customExercise()
 *  
 *  Called during the exercise routines and is useful for testing hardware, and troubleshooting logic.  Any output should be Serial printed out to the 
 *  console.
 *  
 */
void customExercise() {
//  int iHumidity;
//
//  iHumidity = Enviro.readHumidity();   //0-99
//  Serial.print(F("Humidity: "));
//  Serial.println(percentHumidity);

  



}
