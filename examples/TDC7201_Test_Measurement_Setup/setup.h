/*
 * setup.h
 * 
 * Copyright 2020 robin <robin@robin-XPS-13-9365>
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

#ifndef SETUP_H
#define SETUP_H

#include "TDC7201.h"
#include <inttypes.h>
#include <Arduino.h>

/*===========================================================================*/
/* Adafruit M0 constants.                                                    */
/*===========================================================================*/

// Connections
//  TDC7200         AdaFruit Feather M0 pin #
//  OSC Enable      0
//  TDC7201 Enable  1
//  INT1            5
//  TRIG1           6
//  DTG_TRIG        9 -- Combined trigger signal?
//  MSP_START       10 -- Combined mode start signal?
//  TRIG2           11
//  CSB2            12
//  CSB1            14
//  INT2            21

constexpr uint8_t PIN_EVM_OSC_ENABLE{0};       // EVM Oscillator enable
constexpr uint8_t PIN_TDC7201_ENABLE{1};       // TDC7201 device enable
constexpr uint8_t PIN_TDC7201_TIMER1_INT{5};   // TDC7201 clock 1 measurement results interrupt
constexpr uint8_t PIN_TDC7201_TIMER1_TRIG{6};  // TDC7201 clock 1 measurement start trigger
constexpr uint8_t PIN_EVM_DTG_TRIG{9};         // EVM Data gather trigger
constexpr uint8_t PIN_EVM_MSP_START{10};       // EVM Combined mode start
constexpr uint8_t PIN_TDC7201_TIMER2_TRIG{11}; // TDC7201 clock 2 measurement start trigger
constexpr uint8_t PIN_TDC7201_CSB2{12};        // TDC7201 Chip select 2
constexpr uint8_t PIN_TDC7201_CSB1{14};        // TDC7201 Chip select 1
constexpr uint8_t PIN_TDC7201_TIMER2_INT{21};  // TDC7201 clock 2 measurement results interrupt

constexpr uint8_t PIN_M0_LED{13};    // MCU built in LED
constexpr uint8_t PIN_M0_BUTTON{15}; // MCU wired push button

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

// declare TDC7201 instance
extern TDC7201 TDC7201_1;

// forward declaration for function to print TDCx registers
void printRegisters(int timer, int startReg, int stopReg);

// forward declaration for function to setup and start TDC7201 measurement
void startMeasurement(int TDCx_chipSelect, int calibrationPeriods, int averageCycles, int numberOfStops, int measurementMode);

// forward declaration for function to setup and generate a normalized LSB
void generateNormLSB(int TDCx_chipSelect, int calibrationPeriods, int averageCycles, int numberOfStops, int measurementMode);


// forward declaration for function to read and print a TOF measurement
void readPrintMeasurement(int TDCx_chipSelect, int stopNum);

#endif
