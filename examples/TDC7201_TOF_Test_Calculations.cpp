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

#include <iostream>
#include <inttypes.h>
//using namespace std;
int main()
{
    uint8_t shift = 20;
    uint32_t cal2 = 21121;
    uint32_t cal1 = 2110;
    uint32_t cal2T = 10;
    uint64_t clock = (1.0 / 8.0E6) / 1.0E-12;
    uint64_t calcount = (uint64_t(cal2 - cal1) << shift) / (uint64_t(cal2T - 1));
    uint64_t lsb = (uint64_t(clock) << (2 * shift)) / calcount;

    uint32_t TOF = 4175;

    std::cout << "calcount1: " << calcount << std::endl << "calcount2: " << (calcount >> shift) << std::endl << "normLSB1: " << lsb << std::endl << "normLSB2: " << (lsb >> shift) << std::endl << "TOF: " << (TOF * lsb >> shift) << std::endl;

    double calcountf = double(cal2 - cal1) / (double(cal2T - 1));
    double lsbf = double(clock) / calcountf;
    std::cout << "calcount DS: " << calcountf << std::endl << "normLSB DS: " << lsbf << std::endl  << "TOF_DS: " << TOF*lsbf << std::endl;

    return 0;
}
