/*
 * TDC7201.cpp
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

#include "TDC7201.h"
#include <SPI.h>

#define TDC7201_SPI_CLK_MAX			(int32_t (25000000))
#define TDC7201_SPI_MODE			(SPI_MODE0)
#define TDC7201_SPI_ORDER			(MSBFIRST)
#define TDC7201_SPI_REG_ADDR_MASK	(0x1Fu) // datasheet specifies 6 bits but highest address used is 0x1c
#define TDC7201_SPI_REG_READ		(0x00u)
#define TDC7201_SPI_REG_WRITE		(0x40u)
#define TDC7201_SPI_REG_AUTOINC		(0x80u)
#define TDC7201_ENABLE_LOW_MS			(5)
#define TDC7201_ENABLE_T3_LDO_SET3_MS	(2)

#define TDC7201_NOMINAL_LSB					(55)
#define TDC7201_RING_OSC_CELLS				(63) // number of ring oscillator cells
#define TDC7201_RING_OSC_FREQ_HZ               (144000000)
#define TDC7201_RING_OSC_FREQ_MHZ              (TDC7201_RING_OSC_FREQ_HZ/1000000)
#define PS_PER_SEC	(1000000000000)
#define US_PER_SEC	(1000000)

#define TDC7201_REG_TDCx_CONFIG1				(0x00u)
#define TDC7201_REG_TDCx_CONFIG2				(0x01u)
#define TDC7201_REG_TDCx_INT_STATUS				(0x02u)
#define TDC7201_REG_TDCx_INT_MASK				(0x03u)
#define TDC7201_REG_TDCx_COARSE_CNTR_OVF_H		(0x04u)
#define TDC7201_REG_TDCx_COARSE_CNTR_OVF_L		(0x05u)
#define TDC7201_REG_TDCx_CLOCK_CNTR_OVF_H		(0x06u)
#define TDC7201_REG_TDCx_CLOCK_CNTR_OVF_L		(0x07u)
#define TDC7201_REG_TDCx_CLOCK_CNTR_STOP_MASK_H	(0x08u)
#define TDC7201_REG_TDCx_CLOCK_CNTR_STOP_MASK_L	(0x09u)
#define TDC7201_REG_TDCx_LOWER_BITS_MASK		(0xFFu)
#define TDC7201_REG_TDCx_TIME1					(0x10u)
#define TDC7201_REG_TDCx_CLOCK_COUNT1			(0x11u)
#define TDC7201_REG_TDCx_TIME2					(0x12u)
#define TDC7201_REG_TDCx_CLOCK_COUNT2			(0x13u)
#define TDC7201_REG_TDCx_TIME3					(0x14u)
#define TDC7201_REG_TDCx_CLOCK_COUNT3			(0x15u)
#define TDC7201_REG_TDCx_TIME4					(0x16u)
#define TDC7201_REG_TDCx_CLOCK_COUNT4			(0x17u)
#define TDC7201_REG_TDCx_TIME5					(0x18u)
#define TDC7201_REG_TDCx_CLOCK_COUNT5			(0x19u)
#define TDC7201_REG_TDCx_CLOCK_COUNTX(num)      (TDC7201_REG_TDCx_CLOCK_COUNT1+2*((num)-1))
#define TDC7201_REG_TDCx_TIME6					(0x1Au)
#define TDC7201_REG_TDCx_TIMEX(num)             (TDC7201_REG_TDCx_TIME1+2*((num)-1))
#define TDC7201_REG_TDCx_CALIBRATION1			(0x1Bu)
#define TDC7201_REG_TDCx_CALIBRATION2			(0x1Cu)

#define TDC7201_REG_TDCx_CONFIG2_DEFAULTS		(0x40u)      // defaults after reset
#define TDC7201_REG_TDCx_INT_MASK_DEFAULTS		(0x07u)      // defaults after reset

#define TDC7201_REG_SHIFT_CONFIG1_FORCE_CAL    (7u)
#define TDC7201_REG_SHIFT_CONFIG1_PARITY_EN    (6u)
#define TDC7201_REG_SHIFT_CONFIG1_TRIGG_EDGE   (5u)
#define TDC7201_REG_SHIFT_CONFIG1_STOP_EDGE    (4u)
#define TDC7201_REG_SHIFT_CONFIG1_START_EDGE   (3u)
#define TDC7201_REG_SHIFT_CONFIG1_MEAS_MODE    (1u)
#define TDC7201_REG_SHIFT_CONFIG1_START_MEAS   (0u)

#define TDC7201_REG_VAL_CONFIG1_MEAS_MODE_MIN  (1u)
#define TDC7201_REG_VAL_CONFIG1_MEAS_MODE_MAX  (2u)
#define TDC7201_REG_VAL_CONFIG1_MEAS_MODE(num) ((num)-1)


#define TDC7201_REG_SHIFT_CONFIG2_CALIBRATION2_PERIODS  (6u)
#define TDC7201_REG_SHIFT_CONFIG2_AVG_CYCLES            (3u)
#define TDC7201_REG_SHIFT_CONFIG2_NUM_STOP              (0u)

#define TDC7201_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_2  (0u)
#define TDC7201_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_10 (1u)
#define TDC7201_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_20 (2u)
#define TDC7201_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_40 (3u)

#define TDC7201_REG_VAL_CONFIG2_AVG_CYCLES_MIN_VAL      (0u)
#define TDC7201_REG_VAL_CONFIG2_AVG_CYCLES_MAX_VAL      (7u)

#define TDC7201_REG_VAL_CONFIG2_NUM_STOP(num)           ((num)-1)
#define TDC7201_REG_VAL_CONFIG2_NUM_STOP_MAX            (5)


#define TDC7201_REG_SHIFT_INT_STATUS_MEAS_COMPLETE_FLAG  (4)
#define TDC7201_REG_SHIFT_INT_STATUS_MEAS_STARTED_FLAG   (3)
#define TDC7201_REG_SHIFT_INT_STATUS_CLOCK_CNTR_OVF_INT  (2)
#define TDC7201_REG_SHIFT_INT_STATUS_COARSE_CNTR_OVF_INT (1)
#define TDC7201_REG_SHIFT_INT_STATUS_NEW_MEAS_INT        (0)


#define TDC7201_REG_SHIFT_INT_MASK_CLOCK_CNTR_OVF_MASK   (2)
#define TDC7201_REG_SHIFT_INT_MASK_COARSE_CNTR_OVF_MASK  (1)
#define TDC7201_REG_SHIFT_INT_MASK_NEW_MEAS_MASK         (0)

TDC7201::TDC7201(const uint8_t pinEnable, const uint8_t pinCSB1, const uint8_t pinCSB2, 
				const uint8_t pinTRIGG1, const uint8_t pinTRIGG2, const uint8_t pinINTB1,
				const uint8_t pinINTB2, const uint32_t clockFreqHz)
	:	m_pinEnable{pinEnable},
		m_pinCSB1{pinCSB1},
		m_pinCSB2{pinCSB2},
		m_pinTRIGG1{pinTRIGG1},
		m_pinTRIGG2{pinTRIGG2},
		m_pinINTB1{pinINTB1},
		m_pinINTB2{pinINTB2},
		m_clkPeriodPs{uint64_t(PS_PER_SEC) / uint64_t(clockFreqHz)},
		m_overflowPs{0ull},
		m_stopMaskPs{0ull}
{
}

bool TDC7201::begin()
{
    // -- Enable TDC7201 - Reset to default configuration
    digitalWrite(m_pinEnable, LOW);
    pinMode(m_pinEnable, OUTPUT);
    // Disable for a short time, TDC7201 needs to see a clean positive edge after VDD on to power up
    delay(TDC7201_ENABLE_LOW_MS);

    // Enable and wait the maximum time after enabling LDO to assure VREG is stable.
    digitalWrite(m_pinEnable, HIGH);
    delay(TDC7201_ENABLE_T3_LDO_SET3_MS);

    // -- Configure SPI
    digitalWrite(m_pinCSB1, HIGH);
    pinMode(m_pinCSB1, OUTPUT);
    digitalWrite(m_pinCSB2, HIGH);
    pinMode(m_pinCSB2, OUTPUT);
    SPI.begin();

    // -- Comms sanity check
    if (   (spiReadReg8(m_pinCSB1, TDC7201_REG_TDCx_CONFIG2)  != TDC7201_REG_TDCx_CONFIG2_DEFAULTS)
        or (spiReadReg8(m_pinCSB1, TDC7201_REG_TDCx_INT_MASK) != TDC7201_REG_TDCx_INT_MASK_DEFAULTS))
    {
		Serial.println("SPI comms error! Unnable to intialize TDC7201");
        return false;
    }

    // Assert interrupt output on overflow and measurement finished
    spiWriteReg8(m_pinCSB1, TDC7201_REG_TDCx_INT_MASK,   bit(TDC7201_REG_SHIFT_INT_MASK_CLOCK_CNTR_OVF_MASK)
                                           | bit(TDC7201_REG_SHIFT_INT_MASK_COARSE_CNTR_OVF_MASK)
                                           | bit(TDC7201_REG_SHIFT_INT_MASK_NEW_MEAS_MASK) );
	spiWriteReg8(m_pinCSB2, TDC7201_REG_TDCx_INT_MASK,   bit(TDC7201_REG_SHIFT_INT_MASK_CLOCK_CNTR_OVF_MASK)
                                           | bit(TDC7201_REG_SHIFT_INT_MASK_COARSE_CNTR_OVF_MASK)
                                           | bit(TDC7201_REG_SHIFT_INT_MASK_NEW_MEAS_MASK) );

    return true;
}

bool TDC7201::setupMeasurement(const uint8_t pinCSBx, const uint8_t cal2Periods, const uint8_t avgCycles, const uint8_t numStops, const uint8_t mode)
{
    uint8_t config2{0u};

    // Config2 Calibration2 periods
    if      (cal2Periods == 2)  config2 = TDC7201_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_2  << TDC7201_REG_SHIFT_CONFIG2_CALIBRATION2_PERIODS;
    else if (cal2Periods == 10) config2 = TDC7201_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_10 << TDC7201_REG_SHIFT_CONFIG2_CALIBRATION2_PERIODS;
    else if (cal2Periods == 20) config2 = TDC7201_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_20 << TDC7201_REG_SHIFT_CONFIG2_CALIBRATION2_PERIODS;
    else if (cal2Periods == 40) config2 = TDC7201_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_40 << TDC7201_REG_SHIFT_CONFIG2_CALIBRATION2_PERIODS;
    else{
		Serial.println("Invalid calibration2 periods entered!");
		return false;
	}
    m_cal2Periods = cal2Periods;

    // Config2 Avg Cycles
/*	switch (avgCycles)
	{
		case 1:
			config2 |= 0x0 << TDC7201_REG_SHIFT_CONFIG2_AVG_CYCLES;
			break;
		case 2:
			config2 |= 0x1 << TDC7201_REG_SHIFT_CONFIG2_AVG_CYCLES;
			break;
		case 4:
			config2 |= 0x2 << TDC7201_REG_SHIFT_CONFIG2_AVG_CYCLES;
			break;
		case 8:
			config2 |= 0x3 << TDC7201_REG_SHIFT_CONFIG2_AVG_CYCLES;
			break;
		case 16:
			config2 |= 0x4 << TDC7201_REG_SHIFT_CONFIG2_AVG_CYCLES;
			break;
		case 32:
			config2 |= 0x5 << TDC7201_REG_SHIFT_CONFIG2_AVG_CYCLES;
			break;
		case 64:
			config2 |= 0x6 << TDC7201_REG_SHIFT_CONFIG2_AVG_CYCLES;
			break;
		case 128:
			config2 |= 0x7 << TDC7201_REG_SHIFT_CONFIG2_AVG_CYCLES;
			break;
		default:
			Serial.println("Invalid number of averaging cycles entered for Multi-Cycle averaging");
			return false;
	}
	*/
	uint8_t val {TDC7201_REG_VAL_CONFIG2_AVG_CYCLES_MIN_VAL};
	do {
		if ((1 << val) == avgCycles) { //(1 << val) can only equal 2^n
			config2 |= val << TDC7201_REG_SHIFT_CONFIG2_AVG_CYCLES;
			break;
		}
       ++val;
	} while (val <= TDC7201_REG_VAL_CONFIG2_AVG_CYCLES_MAX_VAL);
	
	if (val > TDC7201_REG_VAL_CONFIG2_AVG_CYCLES_MAX_VAL) {
		Serial.println("Invalid number of averaging cycles entered for Multi-Cycle averaging");
		return false;
	}
	
	// Config2 Num Stops 
	if ((numStops == 0) or (numStops > TDC7201_REG_VAL_CONFIG2_NUM_STOP_MAX)) {
		Serial.println("Invalid number of Stops entered");
		return false;
	}
	config2 |= TDC7201_REG_VAL_CONFIG2_NUM_STOP(numStops) << TDC7201_REG_SHIFT_CONFIG2_NUM_STOP;
	m_numStops = numStops;

	// Config1 Mode
	if ((mode < TDC7201_REG_VAL_CONFIG1_MEAS_MODE_MIN) or (mode > TDC7201_REG_VAL_CONFIG1_MEAS_MODE_MAX)) {
		Serial.println("Invalid measure mode entered");
		return false;
	}
	m_config1 = TDC7201_REG_VAL_CONFIG1_MEAS_MODE(mode) << TDC7201_REG_SHIFT_CONFIG1_MEAS_MODE;
	m_mode = mode;
	
	// Save config2 value here
	m_config2 = config2;
	
	// write values to the TDCx_CONFIG registers
	spiWriteReg8(pinCSBx, TDC7201_REG_TDCx_CONFIG2, m_config2);
	spiWriteReg8(pinCSBx, TDC7201_REG_TDCx_CONFIG1, m_config1);
	
	// config1 and config2 values now exist so we can have everything we need to generate 
	// an initial normLSB so lets do that
	if (~(generateNormLSB())){
		Serial.println("normLSB not calculated");
		return false;
	}
	
	// Mode influences overflow, so update now.
	setupOverflow(pinCSBx, m_overflowPs);
	
	// Stop mask is dependent on mode 2 and overflow so set it now.
	if(mode == 2)
		setupStopMask(pinCSBx, m_stopMaskPs);
		
	return true;     
}

void TDC7201::setupStopMask(const uint8_t pinCSBx, const uint64_t stopMaskPs) //*** Function fails if m_overflowPs = 0
{
	/* DS Section 7.3.3.3 says Stop mask value must be less than the clock counter overflow value,
	 * else no measurement will happen. If the value is greater, then lets set it to the CCO
	 * value as upper bound.
	 */
	uint16_t stopMaskClk {m_overflowPs / m_clkPeriodPs}; // initalize to current CCO value.
	
	// clip to upper bound
	if (stopMaskPs < m_overflowPs)
	{
		// Convert duration of stopmask from [ps] to clock increments.
		stopMaskClk = stopMaskPs / m_clkPeriodPs;
	}
	// write value to registers
	spiWriteReg8(pinCSBx, TDC7201_REG_TDCx_CLOCK_CNTR_STOP_MASK_H, stopMaskClk >> 8);
	spiWriteReg8(pinCSBx, TDC7201_REG_TDCx_CLOCK_CNTR_STOP_MASK_L, (stopMaskClk & TDC7201_REG_TDCx_LOWER_BITS_MASK));
	
	// Save value for future reference
	m_stopMaskPs = stopMaskPs;
}

void TDC7201::setupOverflow(const uint8_t pinCSBx, const uint64_t overflowPs)
{

	/* The datasheet does not explicitly say how to calculate a register value for a desired timeout in mode 1.
	 * But it does have the following info.
	 * 
	 * From section 7.4.2.1.1, TOF_n = (TIME_n)(normLSB) where:
	 * -TOF_n is time of flight in seconds 
	 * -TIME_n is the register value is the register value TDCx_TIMEn in the TIME1 to TIME6 registers.
	 * -normLSB is the normalized LSB value
	 *  
	 * From section 7.3.3.2. "if (TDCx_TIMEn / 63) ≥ COARSE_CNTR_OVF, then COARSE_CNTR_OVF_INT = 1 
	 * (this interrupt bit is located in the TDCx_INT_STATUS register)." So TIME_n = COARSE_CNTR_OVF * 63.
	 * 
	 * Putting this information together, the conversion from desired timeout value in seconds 
	 * to the COARSE_CNTR_OVF value is Timeout_value = COARSE_CNTR_OVF * 63 * normLSB and 
	 * COARSE_CNTR_OVF = Timeout_value / (normLSB *63).
	 *  
	 * An estimate using normLSB = 55 ps as stated in the datasheet follows.
	 * COARSE_CNTR_OVF max value is FFFF (from the COARSE_CNTR_OV_H/L registers) so Timeout_value max is 227.08us.
	 * The data sheet also does not recommend using mode 1 for times greater than 2us.
	 * 
	 * An accurate normLSB can be generated by calling the generateNormLSB() function.
	 * 
	 * In mode 2 FFFF from the CLOCK_CNTR_OV_H/L registers is 8.192ms with an 8MHz clock.
	 */


	uint16_t coarseOvf {0xFFFFu};   // For mode 1, maximum 227.08us @ 8MHz clock
	uint16_t clockOvf  {0xFFFFu};   // For mode 2, maximum 8.192ms @ 8MHz clock
	

	// overflowPS of 0 leaves the default overflow values of 0xFFFF for both the COARSE_CNTR_OV_H/L 
	// and the CLOCK_CNTR_OV_H/L registers i.e. the max time out values
	if (overflowPs)
	{
		if (m_mode == 1)
		{
			// multiplier (2^shift) used to prevent rounding errors
			const uint8_t shift = 20;
			
			// Convert overflow value from ps to  number of ring oscillator ticks
			const uint64_t ovf = overflowPs / ((m_normLSB >> shift) * TDC7201_RING_OSC_CELLS);
			
			// Clip to upper bound.
			if (ovf < 0xFFFFu)
			{
				coarseOvf = ovf;
			}
			// write values to the register
			spiWriteReg8(pinCSBx,TDC7201_REG_TDCx_COARSE_CNTR_OVF_H, coarseOvf >> 8);
			spiWriteReg8(pinCSBx,TDC7201_REG_TDCx_COARSE_CNTR_OVF_L, (coarseOvf & TDC7201_REG_TDCx_LOWER_BITS_MASK));
		}
		else
		{
			// convert clock overflow duration from ps to clock increments
			const uint64_t ovf = overflowPs / m_clkPeriodPs;
			// Clip to upper bound.
			if (ovf < 0xFFFFu)
			{
				clockOvf = ovf;
			}
			spiWriteReg8(pinCSBx,TDC7201_REG_TDCx_CLOCK_CNTR_OVF_H, clockOvf >> 8);
			spiWriteReg8(pinCSBx,TDC7201_REG_TDCx_CLOCK_CNTR_OVF_L, (clockOvf & TDC7201_REG_TDCx_LOWER_BITS_MASK));
		}
    }
	// Remember for mode changes
	m_overflowPs = overflowPs;
}

void TDC7201::startMeasurement(const uint8_t pinCSBx)
{
    // Clear status
    // According to Datasheet section 7.6.4, writing a 1 to any of the TDCx_INT_STATUS registers clears the interrupt status
    spiWriteReg8(pinCSBx, TDC7201_REG_TDCx_INT_STATUS,  bit(TDC7201_REG_SHIFT_INT_STATUS_MEAS_COMPLETE_FLAG)
                                             | bit(TDC7201_REG_SHIFT_INT_STATUS_MEAS_STARTED_FLAG)
                                             | bit(TDC7201_REG_SHIFT_INT_STATUS_CLOCK_CNTR_OVF_INT)
                                             | bit(TDC7201_REG_SHIFT_INT_STATUS_COARSE_CNTR_OVF_INT)
                                             | bit(TDC7201_REG_SHIFT_INT_STATUS_NEW_MEAS_INT) );

    // Force recalculation of normLsb after measurement ended
    m_normLsb = 0ull;
    
    // Start measurement
    m_config1 |= bit(TDC7201_REG_SHIFT_CONFIG1_START_MEAS);
    spiWriteReg8(pinCSBx, TDC7201_REG_TDCx_CONFIG1, m_config1);
}

void TDC7201::generateNormLSB(const uint8_t pinCSBx)
{
	/* According to Datasheet section 7.4.1 the TDC7201 performs calibration measurements after every successful
	 * measurement. If there in no stop signal or the measurement is aborted no calibration measurements are performed.
	 * If the FORCE_CAL bit is set in the TDCx_CONFIG1 register then calibration measurements will be perfromed even
	 * after an aborted meaurement.
	 */
	 
	// generate mask to force calibration and start measurement
	config1 = (m_config1	| bit(TDC7201_REG_SHIFT_CONFIG1_FORCE_CAL)
							| bit(TDC7201_REG_SHIFT_CONFIG1_START_MEAS));
	
	// perform a dummy measurement with no stops to generate results for TDCx_CALIBRATION registers
	spiWriteReg8(pinCSBx, TDC7201_REG_TDCx_CONFIG1, config1);
	// wait for measurent to time out - no stops expected
	delay(10);
	
	// multiplier (2^shift) used to prevent rounding errors
	const uint8_t shift = 20;
	
	const uint32_t calibration1 = spiReadReg24(pinCSBx, TDC7201_REG_TDCx_CALIBRATION1);
	const uint32_t calibration2 = spiReadReg24(pinCSBx, TDC7201_REG_TDCx_CALIBRATION2);
	
	/* Datasheet section 7.4.2.1.1 says normLSB = (CLOCKperiod / calCount) and
	 * calCount = (TDCx_CALIBRATION2 - TDCx_CALIBRATION1) / (CALIBRATION2_PERIODS - 1)
	 * where CALIBRATION2_PERIODS is the number of calibration periods set in the TDCx_CONFIG2 register.
	 * The TDCx_CALIBRATION registers are only updated after a calibration measurement hence the dummy measurement
	 * above.
	 */
	// calCount scaled by 2^shift
	const int64_t calCount = ( int64_t(calibration2-calibration1) << shift ) / int64_t(m_cal2Periods - 1);

	// normLsb scaled by 2^shift, divided by calcount (scaled by 2^shift),
	// so multiply by 2^(2*shift) to compensate for divider in calCount
	// needs to be divided by shift to get normLSB in ps.
	m_normLsb  = (uint64_t(m_clkPeriodPs) << (2*shift)) / calCount;
	
}

uint8_t TDC7201::spiReadReg8(const uint8_t pinCSBx, const uint8_t addr)
{
    SPI.beginTransaction(SPISettings(TDC7201_SPI_CLK_MAX, TDC7201_SPI_ORDER, TDC7201_SPI_MODE));
    digitalWrite(pinCSBx, LOW);
	// create and send TDC7210 read command with 6 bit address (lower 6)  and 2 bit read (upper 2 bits)
    SPI.transfer((addr & TDC7201_SPI_REG_ADDR_MASK) | TDC7201_SPI_REG_READ);
	// send 8 '0' bits to read back 8 bits of data from 'addr'
    uint8_t val = SPI.transfer(0u);
	// set CSBx high to return spi interface to high impedance
    digitalWrite(pinCSBx, HIGH);
    SPI.endTransaction();

    return val;
}

uint32_t TDC7201::spiReadReg24(const uint8_t pinCSBx, const uint8_t addr)
{
    SPI.beginTransaction(SPISettings(TDC7201_SPI_CLK_MAX, TDC7201_SPI_ORDER, TDC7201_SPI_MODE));
    digitalWrite(pinCSBx, LOW);

    SPI.transfer((addr & TDC7201_SPI_REG_ADDR_MASK) | TDC7201_SPI_REG_READ);
    uint32_t val;
    val = SPI.transfer(0u);
    val <<= 8; // shift read value over 8 bits to make space for next byte
    val |= SPI.transfer(0u);
    val <<= 8;
    val |= SPI.transfer(0u);

    digitalWrite(pinCSBx, HIGH);
    SPI.endTransaction();

    return val;
}

void TDC7201::spiWriteReg8(const uint8_t pinCSBx, const uint8_t addr, const uint8_t val)
{
    SPI.beginTransaction(SPISettings(TDC7201_SPI_CLK_MAX, TDC7201_SPI_ORDER, TDC7201_SPI_MODE));
    digitalWrite(pinCSBx, LOW);

    (void)SPI.transfer16((((addr & TDC7201_SPI_REG_ADDR_MASK) | TDC7201_SPI_REG_WRITE) << 8) | val);

    digitalWrite(pinCSBx, HIGH);
    SPI.endTransaction();
}


