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

// Set up variables for the TDC7201 class member functions check driver code for usage
static int TDCx_CSB{PIN_TDC7201_CSB2}; // Chip select pin
static int TDCx_TIMER_TRIG{PIN_TDC7201_TIMER2_TRIG}; // timer start trigger pin
static int TDCx_TIMER_INT{PIN_TDC7201_TIMER2_INT}; // timer results interrupt pin active low
static int calibrationPeriods{10}; // number of calibration periods for calculating TDC7201 LSB
static int averageCycles{1}; // multicycle averaging only, set to 1 for no averaging
static int numberOfStops{5}; // number of stops
static int measurementMode{1}; // measurment mode

// set up stop variables for code readability
static int Stop_1{1}; // Time of stop signal 1
static int Stop_2{2}; // Time of stop signal 2
static int Stop_3{3}; // Time of stop signal 3
static int Stop_4{4}; // Time of stop signal 4
static int Stop_5{5}; // Time of stop signal 5


// Set up  an instance of class TDC7201
TDC7201 TDC7201_1(PIN_TDC7201_ENABLE, PIN_TDC7201_CSB1, PIN_TDC7201_CSB2, PIN_TDC7201_TIMER1_TRIG,
                       PIN_TDC7201_TIMER2_TRIG, PIN_TDC7201_TIMER1_INT, PIN_TDC7201_TIMER2_INT, 8000000);

// declare some interrupt flags
volatile boolean flag1, flag2, flag3;

    
// define interrupt service routines for push button and timer triggers
void buttonISR()
{
  // mostly button debounce code
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 250)
  {//actual ISR code
    flag1 = true;
  }
  last_interrupt_time = interrupt_time;
}

void timerTrig1ISR()
{
  flag2 = true;
}

void timerTrig2ISR()
{
  flag3 = true;
}

void setup() 
{ 
  uint64_t stopMaskPs{20};
  delay(2500); // delay to allow bringing up the serial monitor
  Serial.begin(9600);

  // Initialize TDC7201
  if(TDC7201_1.begin())
    Serial.println("TDC7201 Initialization Success");
  else
    Serial.println("TDC7201 Initialization Fail");
  
  // setup TDC stop mask
  TDC7201_1.setupStopMask(TDCx_CSB, stopMaskPs);
  printRegisters(TDCx_CSB, 0, 22);
  
  // setup arduino interrupt pins
  pinMode(TDCx_TIMER_TRIG, INPUT);
  attachInterrupt(digitalPinToInterrupt(TDCx_TIMER_TRIG), timerTrig1ISR, RISING); // timerTrig1ISR sets flag2
  
  pinMode(TDCx_TIMER_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TDCx_TIMER_INT), timerTrig2ISR, RISING); // timerTrig2ISR sets flag3, TDCx_TIMER_INT active low
  
  pinMode(PIN_M0_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_M0_BUTTON), buttonISR, RISING); // buttonISR sets flag1
  
  //pinMode(PIN_M0_LED, OUTPUT);
}

void loop() 
{
 if (flag2)
 {
  Serial.println("Timer 1 start triggered");
  printRegisters(TDCx_CSB, 0, 22);
  generateNormLSB(TDCx_CSB, calibrationPeriods, averageCycles, numberOfStops, measurementMode);
  double normLSB{static_cast<int>(TDC7201_1.m_normLSB)};
  Serial.print("normLSB: "); Serial.println(normLSB, 0);
  readPrintMeasurement(TDCx_CSB, Stop_1);
  readPrintMeasurement(TDCx_CSB, Stop_2);
  readPrintMeasurement(TDCx_CSB, Stop_3);
  readPrintMeasurement(TDCx_CSB, Stop_4);
  readPrintMeasurement(TDCx_CSB, Stop_5);
  flag2 = false;
 }
 if (flag3)
 {
  Serial.println("Timer 1 stop triggered");
  //double normLSB{static_cast<int>(TDC7201_1.m_normLSB)};
  //Serial.print("normLSB: "); Serial.println(normLSB, 0);
  flag3 = false;
 }
 if (flag1)
 {
  Serial.println("Button Pushed!");
  printRegisters(TDCx_CSB, 0, 22);
  double normLSB{static_cast<int>(TDC7201_1.m_normLSB)};
  Serial.print("normLSB: "); Serial.println(normLSB, 0);
  startMeasurement(TDCx_CSB, calibrationPeriods, averageCycles, numberOfStops, measurementMode);
  //generateNormLSB(TDCx_CSB, calibrationPeriods, averageCycles, numberOfStops, measurementMode);
  flag1 = false;
  //delay(200);
 } 
}
