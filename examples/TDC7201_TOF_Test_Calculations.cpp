/*
 * TDC7201_TOF_Test_Calculations.cpp
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
// This program tests the TI TDC7201 normalized lsb generation algorithm as
// well as the TOF generation algorithm
#include <iostream>
#include <inttypes.h>
//using namespace std;
int main()
{
    uint8_t shift = 20;
    uint32_t cal1 = 0b100011100000; // contents of TDC7201 calibration register 21
    uint32_t cal2 = 0b101100010011111; // contents of TDC7201 calibration register 22
    uint32_t cal2T = 10; // number of calibration cycles
    uint64_t clock = (1.0 / 8.0E6) / 1.0E-12; // clock period in ps
    /* Datasheet section 7.4.2.1.1 says normLSB = (CLOCKperiod / calCount) and
	 * calCount = (TDCx_CALIBRATION2 - TDCx_CALIBRATION1) / (CALIBRATION2_PERIODS - 1)
	 * where CALIBRATION2_PERIODS is the number of calibration periods set in the TDCx_CONFIG2 register.
	 */
	// calcount scaled by 2^shift to preserve precision
    uint64_t calcount = (uint64_t(cal2 - cal1) << shift) / (uint64_t(cal2T - 1));
    
    // normLSB scaled by 2^shift, divided by calcount (scaled by 2^shift),
	// so multiply by 2^(2*shift) to compensate for divider in calCount
	
    uint64_t lsb = (uint64_t(clock) << (2 * shift)) / calcount; // lsb scaled by 2^shift to preserve precision
    
    uint32_t Time_1 = 0b10010101;
    uint32_t Time_2 = 0b100011101101101;
    uint32_t Time_3 = 0b0;
    uint32_t Time_4 = 0b0;
    uint32_t Time_5 = 0b0;

    std::cout << "calcount:\t\t " << calcount << std::endl << "calcount right shifted 20: " << (calcount >> shift) << std::endl;
    std::cout << "normLSB:\t\t " << lsb << std::endl << "normLSB right shifted 20:  " << (lsb >> shift) << std::endl; // lsb needs to be divided by shift to get normLSB in ps.
    std::cout << "TOF_1 (ps): " << (Time_1 * lsb >> shift) << std::endl;
    std::cout << "TOF_2 (ps): " << (Time_2 * lsb >> shift) << std::endl;
    std::cout << "TOF_3 (ps): " << (Time_3 * lsb >> shift) << std::endl;
    std::cout << "TOF_4 (ps): " << (Time_4 * lsb >> shift) << std::endl;
    std::cout << "TOF_5 (ps): " << (Time_5 * lsb >> shift) << std::endl;

    double calcountf = double(cal2 - cal1) / (double(cal2T - 1));	
    double lsbf = double(clock) / calcountf;
    std::cout << "calcount as a double:\t " << calcountf << std::endl << "normLSB as a double:\t " << lsbf << std::endl  << "TOF as a double:\t " << Time_1*lsbf << std::endl;

    return 0;
}
