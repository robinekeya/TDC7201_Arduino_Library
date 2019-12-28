#include <TDC7201.h>
#include <Arduino.h>
#include <inttypes.h>

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

#include <TDC7201.h>

constexpr uint8_t PIN_EVM_OSC_ENABLE{0};      // EVM Oscillator enable
constexpr uint8_t PIN_TDC7201_ENABLE{1};      // TDC7201 device enable
constexpr uint8_t PIN_TDC7201_TIMER1_INT{5};    // TDC7201 clock 1 measurement results interrupt
constexpr uint8_t PIN_TDC7201_TIMER1_TRIG{6};   // TDC7201 clock 1 measurement start trigger
constexpr uint8_t PIN_EVM_DTG_TRIG{9};        // EVM Data gather trigger
constexpr uint8_t PIN_EVM_MSP_START{10};      // EVM Combined mode start
constexpr uint8_t PIN_TDC7201_TIMER2_TRIG{11};    // TDC7201 clock 2 measurement start trigger
constexpr uint8_t PIN_TDC7201_CSB2{12};       // TDC7201 Chip select 2
constexpr uint8_t PIN_TDC7201_CSB1{14};       // TDC7201 Chip select 1
constexpr uint8_t PIN_TDC7201_TIMER2_INT{21};   // TDC7201 clock 2 measurement results interrupt

constexpr uint8_t PIN_M0_LED{13}; // MCU built in LED
constexpr uint8_t PIN_M0_BUTTON{15}; // MCU wired push button


static  TDC7201 TDC7201(PIN_TDC7201_ENABLE, PIN_TDC7201_CSB1, PIN_TDC7201_CSB2, PIN_TDC7201_TIMER1_TRIG,
                        PIN_TDC7201_TIMER2_TRIG, PIN_TDC7201_TIMER1_INT, PIN_TDC7201_TIMER2_INT, 8000000);

// Set up variables for the TDC7201 class member functions check driver code for usage
static int TDCx_CSB{PIN_TDC7201_CSB2}; // Chip select pin
static int TDCx_TIMER_TRIG{PIN_TDC7201_TIMER2_TRIG}; // timer start trigger pin
static int TDCx_TIMER_INT{PIN_TDC7201_TIMER2_INT}; // timer results interrupt pin active low
static int calibrationPeriods{10}; // number of calibration periods for calculating TDC7201 LSB
static int averageCycles{1}; // multicycle averaging only, set to 1 for no averaging
static int numberOfStops{2}; // number of stops
static int measurementMode{2}; // measurment mode

// declare some interrupt flags
volatile boolean flag1, flag2, flag3;


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
  /* int TDCx_CSB{PIN_TDC7201_CSB2};
  int TDCx_TIMER_TRIG{PIN_TDC7201_TIMER2_TRIG};
  int TDCx_TIMER_INT{PIN_TDC7201_TIMER2_INT}; // active low
  int calibrationPeriods{40};
  int averageCycles{1}; // for multicycle averaging only
  int numberOfStops{2};
  int measurementMode{2};
  */
  uint64_t stopMaskPs{20};
  delay(2500); // delay to allow bringing up the serial monitor
  Serial.begin(9600);

  // Initialize TDC7201
  if(TDC7201.begin())
    Serial.println("TDC7201 Initialization Success");
  else
    Serial.println("TDC7201 Initialization Fail");
  
  // setup TDC stop mask
  TDC7201.setupStopMask(TDCx_CSB, stopMaskPs);
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
 if (flag2){
  //digitalWrite(PIN_M0_LED, flag1);
  //delay(500);
  
  Serial.println("Timer 1 start triggered");
  printRegisters(TDCx_CSB, 0, 22);
  generateNormLSB(TDCx_CSB, calibrationPeriods, averageCycles, numberOfStops, measurementMode);
  double normLSB{static_cast<int>(TDC7201.m_normLSB)};
  Serial.print("normLSB: "); Serial.println(normLSB, 0);
  flag2 = false;
  //digitalWrite(PIN_M0_LED, flag1);
  //delay(1000);
 }
 if (flag3){
  Serial.println("Timer 1 stop triggered");
  //double normLSB{static_cast<int>(TDC7201.m_normLSB)};
  //Serial.print("normLSB: "); Serial.println(normLSB, 0);
  flag3 = false;
 }
 if (flag1){
  Serial.println("Button Pushed!");
  printRegisters(TDCx_CSB, 0, 22);
  double normLSB{static_cast<int>(TDC7201.m_normLSB)};
  Serial.print("normLSB: "); Serial.println(normLSB, 0);
  startMeasurement(TDCx_CSB, calibrationPeriods, averageCycles, numberOfStops, measurementMode);
  //generateNormLSB(TDCx_CSB, calibrationPeriods, averageCycles, numberOfStops, measurementMode);
  flag1 = false;
  //delay(200);
 } 
}
