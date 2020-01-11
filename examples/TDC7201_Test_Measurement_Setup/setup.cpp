/*
 * TDC7201.h
 * 
 * Copyright 2019 robin <robin.ekeya@gmail.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */
 
#include "setup.h"

// define fuction to print TDCx registers
void printRegisters(int timer=PIN_TDC7201_CSB1, int startReg=0, int stopReg=22)
{
  for(int reg{startReg}; reg < stopReg+1; reg++)
  {
    if(reg < 10)
    {
      Serial.print("Register "); Serial.print(reg);Serial.print(": ");
      Serial.println(TDC7201_1.spiReadReg8(timer, reg), BIN);
      //delay(100);
    }
    else
    { 
      Serial.print("Register "); Serial.print(reg);Serial.print(": ");
      Serial.println(TDC7201_1.spiReadReg24(timer, reg+6), BIN); // +6 so that index aligns with hex addresses
      //delay(100);
    }
  }
}

// define wrapper to setup and start TDC7201 measurement
void startMeasurement(int TDCx_chipSelect = PIN_TDC7201_CSB1, int calibrationPeriods = 20, int averageCycles = 1, int numberOfStops = 1, int measurementMode = 1)
{
  // setup TDC7201
  if(TDC7201_1.setupMeasurement(TDCx_chipSelect, calibrationPeriods, averageCycles, numberOfStops, measurementMode))
    TDC7201_1.startMeasurement(TDCx_chipSelect); // start measurement on TDC setup success
  else
    Serial.println("TDC7201 Setup fail - no measurement");
}

// define wrapper to setup and generate a normalized LSB
void generateNormLSB(int TDCx_chipSelect = PIN_TDC7201_CSB1, int calibrationPeriods = 20, int averageCycles = 1, int numberOfStops = 1, int measurementMode = 1)
{
  // setup TDC7201
  if(TDC7201_1.setupMeasurement(TDCx_chipSelect, calibrationPeriods, averageCycles, numberOfStops, measurementMode))
    TDC7201_1.generateNormLSB(TDCx_chipSelect); // generate normLSB on TDC setup success
  else
    Serial.println("TDC7201 Setup fail - no normLSB");
}

// define wrapper function to read and print a TOF measurement
void readPrintMeasurement(int TDCx_chipSelect, int stopNum)
{
  uint64_t tofOut{0};
  TDC7201_1.readMeasurement(TDCx_chipSelect, stopNum, tofOut);
  double TOF{static_cast<int>(tofOut)};
  Serial.print("TOF_"); Serial.print(stopNum, 1); Serial.print(": "); Serial.println(TOF, 0);
}
