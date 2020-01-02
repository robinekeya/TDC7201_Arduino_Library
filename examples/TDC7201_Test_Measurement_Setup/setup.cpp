#include "setup.h"

#include <TDC7201.h>
#include <Arduino.h>
#include <inttypes.h>

static  TDC7201 TDC7201(PIN_TDC7201_ENABLE, PIN_TDC7201_CSB1, PIN_TDC7201_CSB2, PIN_TDC7201_TIMER1_TRIG,
                        PIN_TDC7201_TIMER2_TRIG, PIN_TDC7201_TIMER1_INT, PIN_TDC7201_TIMER2_INT, 8000000);

// define fuction to print TDCx registers
void printRegisters(int timer=PIN_TDC7201_CSB1, int startReg=0, int stopReg=22)
{
  for(int reg{startReg}; reg < stopReg+1; reg++){
    if(reg < 10){
      Serial.print("Register "); Serial.print(reg);Serial.print(": ");
      Serial.println(TDC7201.spiReadReg8(timer, reg), BIN);
      //delay(100);
    }
    else{ 
      Serial.print("Register "); Serial.print(reg);Serial.print(": ");
      Serial.println(TDC7201.spiReadReg24(timer, reg+6), BIN); // +6 so that index aligns with hex addresses
      //delay(100);
    }
  }
}

// define wrapper to setup and start TDC7201 measurement
void startMeasurement(int TDCx_chipSelect = PIN_TDC7201_CSB1, int calibrationPeriods = 20, int averageCycles = 1, int numberOfStops = 1, int measurementMode = 1)
{
  // setup TDC7201
  if(~TDC7201.setupMeasurement(TDCx_chipSelect, calibrationPeriods, averageCycles, numberOfStops, measurementMode))
    TDC7201.startMeasurement(TDCx_chipSelect); // start measurement on TDC setup success
  else
    Serial.println("TDC7201 Setup fail - no measurement");
}

// define wrapper to setup and generate a normalized LSB
void generateNormLSB(int TDCx_chipSelect = PIN_TDC7201_CSB1, int calibrationPeriods = 20, int averageCycles = 1, int numberOfStops = 1, int measurementMode = 1)
{
  // setup TDC7201
  if(~TDC7201.setupMeasurement(TDCx_chipSelect, calibrationPeriods, averageCycles, numberOfStops, measurementMode))
    TDC7201.generateNormLSB(TDCx_chipSelect); // generate normLSB on TDC setup success
  else
    Serial.println("TDC7201 Setup fail - no normLSB");
}

// define wrapper function to read and print a TOF measurement
void readPrintMeasurement(int TDCx_chipSelect, int stopNum)
{
  uint64_t tofOut{0};
  TDC7201.readMeasurement(TDCx_chipSelect, stopNum, tofOut);
  double TOF{static_cast<int>(tofOut)};
  Serial.print("TOF_: "); Serial.print(stopNum, 0); Serial.println(TOF, 0);
}
