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
#ifndef Custom_h
#define Custom_h





void customInit();
bool customLoop(GPS& GPSParser);
void customSendPositionSingleLine(bool transmitCustom, TNC& oTNC, GPS& GPSParser);
void customExercise();

#endif
