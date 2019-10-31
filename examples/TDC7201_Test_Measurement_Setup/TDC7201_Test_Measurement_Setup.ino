#include <TDC7201.h>

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
//  CSB1            13
//  INT2            21

#include <TDC7201.h>

constexpr uint8_t PIN_EVM_OSC_ENABLE{0};      // EVM Oscillator enable
constexpr uint8_t PIN_TDC7201_ENABLE{1};      // TDC7201 device enable
constexpr uint8_t PIN_TDC7201_TIMER1_INT{5};    // TDC7201 clock 1 interrupt
constexpr uint8_t PIN_TDC7201_TIMER1_TRIG{6};   // TDC7201 clock 1 trigger
constexpr uint8_t PIN_EVM_DTG_TRIG{9};        // EVM Combined timer trigger
constexpr uint8_t PIN_EVM_MSP_START{10};      // EVM Combined mode start
constexpr uint8_t PIN_TDC7201_TIMER2_TRIG{11};    // TDC7201 clock 2 trigger
constexpr uint8_t PIN_TDC7201_CSB2{12};       // TDC7201 Chip select 2
constexpr uint8_t PIN_TDC7201_CSB1{13};       // TDC7201 Chip select 1
constexpr uint8_t PIN_TDC7201_TIMER2_INT{21};   // TDC7201 clock 1 interrupt


static  TDC7201 TDC7201(PIN_TDC7201_ENABLE, PIN_TDC7201_CSB1, PIN_TDC7201_CSB2, PIN_TDC7201_TIMER1_TRIG,
              PIN_TDC7201_TIMER2_TRIG, PIN_TDC7201_TIMER1_INT, PIN_TDC7201_TIMER2_INT, 8000000);
              
void printRegisters(int timer=PIN_TDC7201_CSB1, int startReg=0, int stopReg=23) {
  
  for(int reg{startReg}; reg < stopReg+1; reg++){
    if(reg < 10){
      Serial.print("Register ");Serial.print(reg);Serial.print(": ");
      Serial.println(TDC7201.spiReadReg8(timer, reg), BIN);
      delay(100);
    }
    else{
      Serial.print("Register ");Serial.print(reg);Serial.print(": ");
      Serial.println(TDC7201.spiReadReg24(timer, reg), BIN);
      delay(100);
    }
  }
}

void setup() {
  int calibrationPeriods{20};
  int averageCycles{16};
  int numberOfStops{1};
  int measurementMode{2};
  uint64_t stopMaskPs{300};
  delay(5000);
  Serial.begin(9600);
 
  if(TDC7201.begin())
    Serial.println("TDC7201 Initialization Success");
  else
    Serial.println("TDC7201 Initialization Fail");
  
  if(TDC7201.setupMeasurement(PIN_TDC7201_CSB1, calibrationPeriods, averageCycles, numberOfStops, measurementMode))
    Serial.println("TDC7201 Setup success");
  else
    Serial.println("TDC7201 Setup fail");
  
  Serial.print("normLSB: "); Serial.print(m_normLSB);

  TDC7201.setupStopMask(PIN_TDC7201_CSB1, stopMaskPs);
  printRegisters(PIN_TDC7201_CSB1, 4, 9);
}

void loop() {
 
}
