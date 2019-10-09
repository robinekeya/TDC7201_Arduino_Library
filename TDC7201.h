/*
 * TDC7201.h
 * 
 * Copyright 2019 robin <robin@robin-XPS-13-9365>
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

#ifndef _TDC7201_H_INCLUDED
#define _TDC7201_H_INCLUDED

#include <inttypes.h> // also includes cstdint.h

class TDC7201
{
    public:
		/**
		* Constructor.
		@param[in] pinEnable   Mcu pin controlling TDC7201 enable input.
		* @param[in] pinCs       Mcu pin controlling TDC7201 SPI CSB input.
		* @param[in] clockFreqHz Clock frequency supplied at TDC7201 clock input, range [1..16] [MHz].
		*/
		TDC7201(const uint8_t pinEnable, const uint8_t pinCSB1, const uint8_t pinCSB2, const uint8_t pinTRIGG1,
				const uint8_t pinTRIGG2, const uint8_t pinINTB1,const uint8_t pinINTB2, const uint32_t clockFreqHz);
		/**
		* Initialize TDC7201.
		*/
		bool begin();

		/**
		* Setup measurement parameters.
		* @param[in] pinCSBx Set which of the two TDC7201 timers to configure
		* @param[in] cal2Periods Set calibration2 periods [2,10,20,40].
		* @param[in] avgCycles   Set number of averaging cycles [1,2,4,8,16,32,64,128]. 
		* @param[in] numStops    Set number of stop pulses [1..5].
		* @param[in] mode		Set measurement mode [1,2]. Mode 1 is for measurements <500 [ns].
		* @return				True, when all parameters were valid and setup succeeded.
		*/
		bool setupMeasurement(const uint8_t pinCSBx, const uint8_t cal2Periods, const uint8_t avgCycles, const uint8_t numStops, const uint8_t mode);

		/**
		* Setup stop mask.
		* @param[in] pinCSBx Set which of the two TDC7201 timers to configure
		* @param[in] stopMaskPs Duration of stop mask, in [ps]. Will be rounded to number of external
		*						clock counts, so actual value used might be slightly different.
		*/
		void setupStopMask(const uint8_t pinCSBx, const uint64_t stopMaskPs);

		/**
		* Setup overflow time.
		* @param[in] pinCSBx Set which of the two TDC7201 timers to configure
		* @param[in] overflowPs  Overflow time, in [ps]. If tof takes longer than overflowPs,
		*						a timeout will occur.
		* @remark Only functional for mode2, datasheet only describes registers for a clock counter stop mask.
		*/
		void setupOverflow(const uint8_t pinCSBx, const uint64_t overflowPs);

		/**
		* Start a new measurement.
		*/
		//void startMeasurement(const uint8_t timerNum);

		/**
		* Read measurement result.
		* Caller must make sure sufficient time has elapsed for all pulses to occur before calling this function.
		* @param[in]  stop       Index of stop pulse to read time for, [1..numStops].
		* @param[out] tof		Measured time-of-flight from start pulse to given stop pulse, or 0 when
		*				      pulse wasn't recorded (didn't occur, or not within overflow time).
		* @return		       True, when all parameters were valid and time-of-flight was retrieved.
		*/
		//bool readMeasurement(const uint8_t stop, uint64_t& tof, const uint8_t timerNum);
		uint8_t  spiReadReg8(const uint8_t pinCSBx, const uint8_t addr);
		uint32_t spiReadReg24(const uint8_t pinCSBx, const uint8_t addr);
		void     spiWriteReg8(const uint8_t pinCSBx, const uint8_t addr, const uint8_t val)	;
    private:
		uint8_t  m_pinEnable;	//< Mcu pin controlling TDC7201 enable input.
		uint8_t  m_pinCSB1;		//< Mcu pin controlling TDC7201 SPI CSB1 input.
		uint8_t  m_pinCSB2;		//< Mcu pin controlling TDC7201 SPI CSB2 input. 
		uint8_t  m_pinTRIGG1;	//< Mcu pin controlling TDC7201 TRIGG1 input.
		uint8_t  m_pinTRIGG2;	//< Mcu pin controlling TDC7201 TRIGG2 input.
		uint8_t  m_pinINTB1;	//< Mcu pin controlling TDC7201 Interrupt 1 input.
		uint8_t  m_pinINTB2;	//< Mcu pin controlling TDC7201 Interrupt 2 input.
		uint32_t m_clkPeriodPs;	//< Clock period in [ps].
		uint8_t  m_cal2Periods;	//< Calibration2, number of measuring clock periods, one of [2,10,20,40].
		uint8_t  m_config1;		//< CONFIG1 register value, used to start measurement.
		uint8_t  m_config2;		//< CONFIG2 register value, used to start measurement.
		uint8_t  m_mode;		//< Measurement mode [1,2].
		uint8_t  m_numStops;	//< Number of stops per measurement.
		int64_t  m_normLsb;		//< Cached normLsb value for tof calculation.
		uint64_t m_overflowPs;	//< Overflow time, in [ps].
		uint64_t m_stopMaskPs;	//< Stop mask time, in [ps].

		//uint8_t  spiReadReg8(const uint8_t pinCSBx, const uint8_t addr);
		//uint32_t spiReadReg24(const uint8_t pinCSBx, const uint8_t addr);
		//void     spiWriteReg8(const uint8_t pinCSBx, const uint8_t addr, const uint8_t val)	;
};

#endif
