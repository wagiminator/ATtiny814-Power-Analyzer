// Power Analyzer
//
// Programmable electronic constant current dummy load with two high side voltage and 
// current sensors for an automatic analysis of power supplys, DC/DC converters, voltage
// regulators, batteries, chargers, power consumers and others.
// The device can be controlled via USB serial interface using a serial monitor or the
// provided python skripts. Data can be exported to spread sheet programs or directly be
// analyzed by the python skripts.
//
//                         +-\/-+
//                   Vcc  1|    |14  GND
// Fan -------- (D0) PA4  2|    |13  PA3 (D10) ------- 
// NTC -------- (D1) PA5  3|    |12  PA2  (D9) ------- 
// Load --- DAC (D2) PA6  4|    |11  PA1  (D8) ------- 
// LED -------- (D3) PA7  5|    |10  PA0 (D11) ------- UPDI
// USB ---- RXD (D4) PB3  6|    |9   PB0  (D7) SCL --- INA219
// USB ---- TXD (D5) PB2  7|    |8   PB1  (D6) SDA --- INA219
//                         +----+
//
// Controller:    ATtiny814
// Core:          megaTinyCore v2.0.1 (https://github.com/SpenceKonde/megaTinyCore)
// Clockspeed:    20 MHz internal
// Millis/Micros: enabled (default timer)
// UART voltage:  closer to 5V
//
// 2020 by Stefan Wagner (https://easyeda.com/wagiminator)
// License: http://creativecommons.org/licenses/by-sa/3.0/
//
// ---------------------------------------------------------------------------------------
// Load Test:
//
// TEST-IN <---- Test Power Supply
// PWR-OUT --X-- unconnected
// PWR-IN  --X-- unconnected
//
// Command: "l <maxloadcurrent[mA: 17..5000]> <minloadvoltage[mV: 0..26000]>"
// Example: "l 1000 2900"
// The Power Analyzer continuously increases the load from 17 mA up to <maxloadcurrent>.
// It stops automatically if the voltage drops below <minloadvoltage>. It continuously
// transmits the measured values via the serial interface in the format: 
// current[mA] voltage[mV] power[mW] (seperated by the SEPERATOR string).
// ---------------------------------------------------------------------------------------
// Voltage Regulation Test:
//
// TEST-IN <---- Test Power Supply
// PWR-OUT --X-- unconnected
// PWR-IN  --X-- unconnected
//
// Command: "g <maxloadcurrent[mA: 17..5000]>"
// Example: "g 3000"
// The Power Analyzer changes rapidly the load between 17 mA and <maxloadcurrent>.
// It continuously transmits the measured values via the serial interface in the format: 
// time[ms] current[mA] voltage[mV] (seperated by the SEPERATOR string).
// ---------------------------------------------------------------------------------------
// Efficiency Test:
//
// TEST-IN <---- DC/DC Converter Output
// PWR-OUT ----> DC/DC Converter Input
// PWR-IN  <---- Power Source
//
// Command: "e <maxloadcurrent[mA: 17..5000]> <minloadvoltage[mV: 0..26000]>"
// Example: "e 1000 4700"
// The Power Analyzer continuously increases the load from 17 mA up to <maxloadcurrent>.
// It stops automatically if the voltage at TEST-IN drops below <minloadvoltage>. It 
// continuously transmits the measured values via the serial interface in the format: 
// current[mA] voltage[mV] efficiency[% * 10] (seperated by the SEPERATOR string).
// ---------------------------------------------------------------------------------------
// Low Frequency Ripple Test:
//
// TEST-IN <---- Test Power Supply
// PWR-OUT --X-- unconnected
// PWR-IN  --X-- unconnected
//
// Command: "f <maxloadcurrent[mA: 17..5000]> <minloadvoltage[mV: 0..26000]>"
// Example: "f 1000 4700"
// The Power Analyzer continuously increases the load from 17 mA up to <maxloadcurrent>.
// It stops automatically if the voltage at TEST-IN drops below <minloadvoltage>. It 
// continuously transmits the measured values via the serial interface in the format: 
// current[mA] peak-to-peak voltage[mV] (seperated by the SEPERATOR string).
// ---------------------------------------------------------------------------------------
// Battery Discharge Test:
//
// TEST-IN <---- Battery
// PWR-OUT --X-- unconnected
// PWR-IN  --X-- unconnected
//
// Command: "b <maxloadcurrent[mA: 17..5000]> <minloadvoltage[mV: 0..26000]>"
// Example: "l 1000 2900"
// The Power Analyzer sets a constant current load of <maxloadcurrent>. If the voltage
// drops below <minloadvoltage> it constantly decreases the load to maintain
// <minloadvoltage>. It stops automatically if the load current drops to 0mA. It
// continuously transmits the measured values via the serial interface in the format: 
// time[s] current[mA] voltage[mV] capacity[mAh] (seperated by the SEPERATOR string).
// ---------------------------------------------------------------------------------------
// Long-Term Multimeter:
//
// TEST-IN --X-- unconnected
// PWR-OUT ----> Power Consumer
// PWR-IN  <---- Power Source
//
// Command: "m <interval[ms: 2..65535]> <duration[s: 1..65535]>"
// Example: "m 1000 600"
// The Power Analyzer measures voltage, current and power delivered to the test device at
// every <interval> for a total of <duration>. It continuously transmits the measured
// values via the serial interface in the format: 
// time[ms] current[mA] voltage[mV] (seperated by the SEPERATOR string).
// ---------------------------------------------------------------------------------------
// Calibration:
//
// Set all calibration values in the sketch to 1, compile and upload.
// Connect a power supply to PWR-IN and connect PWR-OUT to TEST-IN with an ammeter in 
// between the positive line. Connect the voltmeter to the TEST-IN terminals. Make sure
// not to exceed the maximum load power.
//
// TEST-IN <---- |Multi-
// PWR-OUT ----> |meter
// PWR-IN  <---- Power Source
//
// Command: "c <loadcurrent[mA: 17..5000]> <duration[s: 1..65535]>"
// Example: "c 1000 10"
// The Power Analyzer sets a constant current load of <loadcurrent> for <duration>.
// It averages the measurements if both voltage/current sensors and transmits the
// values via the serial interface in the format: 
// current1[mA] voltage1[mV] current2[mA] voltage2[mV] (seperated by the SEPERATOR string)
// Measure voltage and current with a trusty multimeter during this time and calculate
// the calibration values as follows:
// ILCAL1 = current measured with multimeter / transmitted value of current1
// ULCAL1 = voltage measured with multimeter / transmitted value of voltage1
// ILCAL2 = current measured with multimeter / transmitted value of current2
// ULCAL2 = voltage measured with multimeter / transmitted value of voltage2
// Change the calibration values in the sketch, compile and upload.
// ---------------------------------------------------------------------------------------
// Commands for Direct Control:
//
// Command                Function
// "i"                    transmit indentification string ("Power Analyzer")
// "v"                    transmit firmware version number
// "x"                    terminate current test program
// "s <loadcurrent[mA]>"  set load to a constant current of <loadcurrent>
// "p <loadpower [mW]>"   set load to a constant power of <loadpower> (not yet implemented)
// "o <resistance [Ohm]>" set load to a constant resistance of <resistance> (nyi)
// "r"                    reset the load to minimum
// "t"                    read current and voltage of both sensors and transmit them
// ---------------------------------------------------------------------------------------
// Notes:
// - Use a good heatsink with a 5V fan for the MOSFET!
// - Be careful with high power loads! Make some tests to figure out what can be achieved
//   with your cooling solution!
// - Due to the limitations of the OpAmp the minimum load current is around 17mA.
//   You can choose a better OpAmp if you like, but for most cases this is not neccessary.
// - The maximum load current is 5A, however for small voltages it might be less.
// - The maximum PWR-IN/PWR-OUT current is 8A.
// - Do not exceed the maximum voltage of 26V on all connectors !
// - In order to make the design much simpler all connectors including USB have a common
//   ground. Keep this in mind when making your test setup in order to avoid ground loops
//   or shorts. Using a USB isolator between the Analyzer and your PC is not a bad idea!  


// Libraries
#include <TinyMegaI2CMaster.h>          // https://github.com/technoblogy/tiny-mega-i2c

// Pins
#define FAN_PIN     0                   // pin the fan is connected to
#define NTC_PIN     1                   // pin the temperature sensor is connected to
#define LED_PIN     3                   // pin the status LED is connected to

// Identifiers
#define VERSION     "1.0"               // version number sent via serial if requested
#define IDENT       "Power Analyzer"    // identifier sent via serial if requested
#define SEPERATOR   "\t"                // seperator string for serial communication
#define DONE        "DONE"              // string sent when operation has finished

// Calibration values
#define ILCAL1      1                   // linear current calibration factor (load)
#define ULCAL1      1                   // linear voltage calibration factor (load)
#define ILCAL2      1                   // linear current calibration factor (supply)
#define ULCAL2      1                   // linear voltage calibration factor (supply)

// Parameters
#define MAXPOWER    25000               // maximum power of the load in mW -> turn off load
#define FANONPOWER  700                 // power in mW to turn fan on
#define FANOFFPOWER 500                 // power in mW to turn fan off
#define MAXTEMP     820                 // max ADC-value of NTC_PIN -> turn off load
#define FANONTEMP   585                 // ADC-value of NTC_PIN to turn fan on
#define FANOFFTEMP  540                 // ADC-value of NTC_PIN to turn fan off
#define SETTLE      25                  // settle time in ms

// INA219 register values
#define INA1ADDR    0b01000000          // I2C address of INA on the load side
#define INA2ADDR    0b01000001          // I2C address of INA on the power side
#define INA1CONFIG  0b0010011001100111  // INA config register according to datasheet
#define INA2CONFIG  0b0010111001100111  // INA config register according to datasheet
#define INAFASTBUS  0b0010110001000110  // INA config register according to datasheet
#define INAFASTBOTH 0b0010110001000111  // INA config register according to datasheet
#define INACALIB    5120                // INA calibration register according to R_SHUNT
#define CONFIG_REG  0x00                // INA configuration register address
#define CALIB_REG   0x05                // INA calibration register address
#define SHUNT_REG   0x01                // INA shunt voltage register address
#define VOLTAGE_REG 0x02                // INA bus voltage register address
#define POWER_REG   0x03                // INA power register address
#define CURRENT_REG 0x04                // INA current register address

// DAC reference voltages (load current = DAC voltage * R16 / (R15 + R16) / R_SHUNT)
// Reference voltages: 0.55V, 1.1V, 1.5V, 2.5V, 4.3V
const uint8_t  DACREF[] = {0x00, 0x01, 0x04, 0x02, 0x03}; // CTRLA.DAC0REFSEL values
const uint16_t DACCUR[] = { 717, 1434, 1956, 3260, 5608}; // max current in mA
uint8_t DACreference = 0;                                 // start with 0.55V reference

// Command buffer
char    cmdBuffer[16];                  // command buffer

// Variables (voltages in mV, currents in mA, power in mW, shunts in 0.01 mV)
uint16_t voltage1, current1, voltage2, current2, shunt1, shunt2, power2, loadtemp;
uint16_t argument1, argument2;
uint16_t loadpower = 0;


void setup() {
  // setup pins
  pinMode(FAN_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(NTC_PIN, INPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);

  // setup serial and I2C communictaion
  Serial.begin(115200);  
  TinyMegaI2C.init();

  // setup DAC and INA
  initDAC();
  initINA();
}

void loop() {
  readCommand();                                          // wait for and read command
  digitalWrite(LED_PIN, HIGH);                            // set BUSY LED
  char cmd = cmdBuffer[0];                                // get command
  switch(cmd) {
    case 'i':   Serial.println(IDENT); break;             // send identification
    case 'v':   Serial.println(VERSION); break;           // send version number
    case 'c':   getArguments(); calibrateLoad(); break;   // set load for calibration
    case 'l':   getArguments(); loadTest(); break;        // perform load test
    case 'g':   getArguments(); regulationTest();break;   // perform regulation test
    case 'e':   getArguments(); efficiencyTest(); break;  // perform efficiency test
    case 'f':   getArguments(); rippleTest(); break;      // perform ripple test
    case 'b':   getArguments(); batteryTest(); break;     // perform battery test
    case 'm':   getArguments(); multimeter(); break;      // long-term multimeter
    case 't':   transmitValues(); break;                  // read and transmit sensor values
    case 'r':   resetLoad(); Serial.println(DONE); break; // reset the load
    case 's':   getArguments(); setLoad(argument1);       // set load current
                Serial.println(DONE); break; 
    default:    break;    
  }
  digitalWrite(LED_PIN, LOW);                             // set READY LED
}


// reads command string via serial connection
void readCommand() {
  for(uint8_t i=0; i< 16; i++) cmdBuffer[i] = 0;  // clear command buffer
  char c; uint8_t idx = 0;                        // initialize variables  
  // read serial data until linebreak or buffer is full
  do {
    if(Serial.available()) {
      c = Serial.read();
      cmdBuffer[idx++] = c;
    }
    else updateLoadSensors();                     // just to maintain fan control
  } while (c != '\n' && idx < 16);
  cmdBuffer[idx - 1] = 0;                         // change last newline to '\0' termination
}

// parse command string for arguments
void getArguments() {
  uint8_t i = 0;
  argument1 = 0; argument2 = 0; 
  while(cmdBuffer[++i] == ' ');
  while(cmdBuffer[i] > ' ') argument1 = argument1 * 10 + cmdBuffer[i++] - '0';
  while(cmdBuffer[++i] == ' ');
  while(cmdBuffer[i] != 0)  argument2 = argument2 * 10 + cmdBuffer[i++] - '0';
}

// checks if termination command was send during a test program
bool isTerminated() {
  if (Serial.available()) return (Serial.read() == 'x');
  else return (false);
}

// writes a register value to the INA219
void writeRegister(uint8_t addr, uint8_t reg, uint16_t value) {
  TinyMegaI2C.start(addr, 0);
  TinyMegaI2C.write(reg);
  TinyMegaI2C.write((value >> 8) & 0xff);
  TinyMegaI2C.write(value & 0xff);
  TinyMegaI2C.stop();
}

// reads a register from the INA219
uint16_t readRegister(uint8_t addr, uint8_t reg) {
  uint16_t result;
  TinyMegaI2C.start(addr, 0);
  TinyMegaI2C.write(reg);
  TinyMegaI2C.restart(addr, 2);
  result = (uint16_t)(TinyMegaI2C.read() << 8) | TinyMegaI2C.read();
  TinyMegaI2C.stop();
  return(result);
}

// writes inital configuration and calibration values to the INAs
void initINA() {
  writeRegister(INA1ADDR, CONFIG_REG, INA1CONFIG);
  writeRegister(INA1ADDR, CALIB_REG,  ILCAL1 * INACALIB);
  writeRegister(INA2ADDR, CONFIG_REG, INA2CONFIG);
  writeRegister(INA2ADDR, CALIB_REG,  ILCAL2 * INACALIB);
}

// reads all sensors of the electronic load and sets the fan
void updateLoadSensors() {
  // read values from INA on the load side
  shunt1   = readRegister(INA1ADDR, SHUNT_REG);
  if (shunt1 > 4095) shunt1 = 0;
  voltage1 = (readRegister(INA1ADDR, VOLTAGE_REG) >> 1) & 0xfffc;
  voltage1 = ((float)shunt1 / 100 + voltage1) * ULCAL1 + 0.5;
  current1 = readRegister(INA1ADDR, CURRENT_REG);
  if (current1 > 32767) current1 = 0;

  // calculate load power and read temperature of the heatsink
  loadpower= ((uint32_t)voltage1 * current1 + 500) / 1000;
  loadtemp = analogRead(NTC_PIN);

  // turn fan on or off depending on load power and temperature
  if ((loadpower > MAXPOWER)    || (loadtemp > MAXTEMP))    resetLoad();
  if ((loadpower > FANONPOWER)  || (loadtemp > FANONTEMP))  digitalWrite(FAN_PIN, HIGH);
  if ((loadpower < FANOFFPOWER) && (loadtemp < FANOFFTEMP)) digitalWrite(FAN_PIN, LOW); 
}

// reads voltage and current of PWR-IN/PWR-OUT
void updatePowerSensors() {
  voltage2 = (readRegister(INA2ADDR, VOLTAGE_REG) >> 1) & 0xfffc;
  voltage2 = (float)voltage2 * ULCAL2 + 0.5;
  current2 = readRegister(INA2ADDR, CURRENT_REG);
  if (current2 > 32767) current2 = 0;
}

// reads all sensors and updates the corresponding variables
void updateSensors() {
  updateLoadSensors();
  updatePowerSensors();
}

// setup the digital to analog converter (DAC) which controls the load
void initDAC() {
  VREF_CTRLA      |= VREF_DAC0REFSEL_0V55_gc;   // select reference voltage
  VREF_CTRLB      |= VREF_DAC0REFEN_bm;         // enable DAC reference
  delay(1);                                     // wait for Vref to start up
  PORTA.PIN6CTRL  &= ~PORT_ISC_gm;              // disable digital input buffer
  PORTA.PIN6CTRL  |= PORT_ISC_INPUT_DISABLE_gc; 
  PORTA.PIN6CTRL  &= ~PORT_PULLUPEN_bm;         // disable pullup
  DAC0.DATA        = 0;                         // set DAC value to zero
  // enable DAC, output buffer and run in standby
  DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm | DAC_RUNSTDBY_bm;
}

// sets the lowest reference voltage possible for the DAC to meet the load current
void setReference(uint16_t current) {
  DACreference = 0;
  if (current > DACCUR[4]) current = DACCUR[4];
  while (current > DACCUR[DACreference]) DACreference++;
  DAC0.DATA = 0;
  VREF_CTRLA &= 0xf8;
  VREF_CTRLA |= DACREF[DACreference];
  delay(1);
}

// sets the DAC within the selected reference to the specified load current
void setDAC(uint16_t current) {
  if (current > 5000) current = 5000;
  if (current > DACCUR[DACreference]) DAC0.DATA = 255;
  else DAC0.DATA = (uint32_t)255 * current / DACCUR[DACreference];
}

// sets the DAC and its reference to the specified load current
void setLoad(uint16_t current) {
  setReference(current);
  setDAC(current);
}

// reset the load to minimum
void resetLoad() {
  setLoad(0);                                   // reset the load to minimum
}

// reads sensor values and transmits them via serial interface
void transmitValues() {
  updateSensors();
  Serial.print(current1); Serial.print(SEPERATOR);
  Serial.print(voltage1); Serial.print(SEPERATOR);
  Serial.print(current2); Serial.print(SEPERATOR);
  Serial.println(voltage2);
}

// performs automatic load test according to minimum load voltage and maximum load current
void loadTest() {
  uint8_t  DACvalue = 0; DAC0.DATA = 0;
  uint16_t maxloadcurrent = argument1;
  uint16_t minloadvoltage = argument2;
  if (maxloadcurrent > 4999) maxloadcurrent = 4999;
  
  setReference(maxloadcurrent);                 // set DAC reference according to max current
  while (++DACvalue) {                          // increase load every cycle
    updateLoadSensors();                        // read all load sensor values
    if (loadpower > MAXPOWER) break;            // stop when power reaches maximum
    if (loadtemp  > MAXTEMP)  break;            // stop when heatsink is to hot
    if (voltage1  < minloadvoltage) break;      // stop when voltage falls below minimum
    if (current1  > maxloadcurrent) break;      // stop when current reaches maximum
    DAC0.DATA = DACvalue;                       // set load for next measurement

    // transmit values via serial interface
    Serial.print(current1); Serial.print(SEPERATOR);
    Serial.print(voltage1); Serial.print(SEPERATOR);
    Serial.println(loadpower);
    
    delay (SETTLE);                             // give everything a little time to settle
  }
  
  resetLoad();                                  // reset the load to minimum
  Serial.println(DONE);                         // transmit end of test
}

// performs a voltage regulation test up to the max load current
void regulationTest() {
  uint16_t maxloadcurrent = argument1;
  uint8_t  profile[] = {0, 2, 5, 7, 10, 10, 5, 0, 10, 0};
  writeRegister(INA1ADDR, CONFIG_REG, INAFASTBOTH); // speed-up sampling rate of INA
  setReference(maxloadcurrent);
  uint32_t startmillis = millis();
  uint32_t nextmillis = startmillis + 100;

  for(uint8_t i=0; i<10; i++) {
    setDAC(maxloadcurrent * profile[i] / 10);
    while (millis() < nextmillis) {
      updateLoadSensors();                      // read all load sensor values
      if (loadpower > MAXPOWER) break;          // stop when power reaches maximum
      if (loadtemp  > MAXTEMP)  break;          // stop when heatsink is to hot
      Serial.print((millis() - startmillis)); Serial.print(SEPERATOR);
      Serial.print(current1); Serial.print(SEPERATOR);
      Serial.println(voltage1);
    }
    nextmillis += 100;
  }
  resetLoad();                                  // reset the load to minimum
  writeRegister(INA1ADDR, CONFIG_REG, INA1CONFIG); // INA back to normal operation
  Serial.println(DONE);                         // transmit end of test
}

// performs automatic efficiency test according to minimum load voltage and maximum load current
void efficiencyTest() {
  uint16_t supplypower;
  uint16_t  efficiency;
  uint8_t  DACvalue = 0; DAC0.DATA = 0;
  uint16_t maxloadcurrent = argument1;
  uint16_t minloadvoltage = argument2;
  if (maxloadcurrent > 4999) maxloadcurrent = 4999;
  
  setReference(maxloadcurrent);                 // set DAC reference according to max current
  while (++DACvalue) {                          // increase load every cycle
    updateSensors();                            // read all sensor values
    if (loadpower > MAXPOWER) break;            // stop when power reaches maximum
    if (loadtemp  > MAXTEMP)  break;            // stop when heatsink is to hot
    if (voltage1  < minloadvoltage) break;      // stop when voltage falls below minimum
    if (current1  > maxloadcurrent) break;      // stop when current reaches maximum
    DAC0.DATA = DACvalue;                       // set load for next measurement

    // calculate efficiency
    supplypower = ((uint32_t)voltage2 * current2 + 500) / 1000;
    efficiency  = (float)loadpower * 1000 / supplypower + 0.5;

    // transmit values via serial interface
    Serial.print(current1); Serial.print(SEPERATOR);
    Serial.print(voltage1); Serial.print(SEPERATOR);
    Serial.println(efficiency);
    
    delay (SETTLE);                             // give everything a little time to settle
  }
  
  resetLoad();                                  // reset the load to minimum
  Serial.println(DONE);                         // transmit end of test
}

// performs low frequency ripple test
void rippleTest() {
  uint8_t  DACvalue = 0; DAC0.DATA = 0;
  uint16_t minvoltage, maxvoltage;
  uint16_t maxloadcurrent = argument1;
  uint16_t minloadvoltage = argument2;
  if (maxloadcurrent > 4999) maxloadcurrent = 4999;
  
  setReference(maxloadcurrent);                 // set DAC reference according to max current
  while (++DACvalue) {                          // increase load every cycle
    updateLoadSensors();                        // read all load sensor values
    if (loadpower > MAXPOWER) break;            // stop when power reaches maximum
    if (loadtemp  > MAXTEMP)  break;            // stop when heatsink is to hot
    if (voltage1  < minloadvoltage) break;      // stop when voltage falls below minimum
    if (current1  > maxloadcurrent) break;      // stop when current reaches maximum
    writeRegister(INA1ADDR, CONFIG_REG, INAFASTBUS); // speed-up sampling rate of INA
    minvoltage = 26000; maxvoltage = 0;         // reset peak voltage values
    for (uint8_t i=0; i<255; i++) {             // get 255 voltage readings
      // read voltage and update peak values
      voltage1 = (readRegister(INA1ADDR, VOLTAGE_REG) >> 1) & 0xfffc;
      if (voltage1 > maxvoltage) maxvoltage = voltage1;
      if (voltage1 < minvoltage) minvoltage = voltage1;
    }
    writeRegister(INA1ADDR, CONFIG_REG, INA1CONFIG); // INA back to normal operation
    DAC0.DATA = DACvalue;                       // set load for next measurement

    // transmit values via serial interface
    Serial.print(current1); Serial.print(SEPERATOR);
    Serial.println(maxvoltage - minvoltage);   
    delay (SETTLE);                             // give everything a little time to settle
  } 
  resetLoad();                                  // reset the load to minimum
  Serial.println(DONE);                         // transmit end of test
}

// performs a battery discharge test
void batteryTest() {
  uint32_t capacity       = 0;
  uint16_t maxloadcurrent = argument1;
  uint16_t minloadvoltage = argument2;
  uint32_t startmillis    = millis();
  uint32_t nextmillis = startmillis + 1000;
  if (maxloadcurrent > 3000) maxloadcurrent = 3000;
  
  setLoad(maxloadcurrent);                      // set the constant load current
  delay (SETTLE);                               // give everything a little time to settle
  while(DAC0.DATA) {                            // repeat until load is zero
    updateLoadSensors();                        // read all load sensor values
    if (loadpower > MAXPOWER) break;            // stop when power reaches maximum
    if (loadtemp  > MAXTEMP)  break;            // stop when heatsink is to hot
    if (isTerminated()) break;                  // stop if termination command was sent

    // decrease load if voltage drops below minloadvoltage
    if (voltage1  < minloadvoltage) DAC0.DATA--;

    // transmit values via serial interface
    Serial.print((millis() - startmillis) / 1000); Serial.print(SEPERATOR);
    Serial.print(current1); Serial.print(SEPERATOR);
    Serial.print(voltage1); Serial.print(SEPERATOR);
    Serial.println(capacity / 3600);
    
    while(millis() < nextmillis);               // wait for the next cycle (one cycle/second)
    nextmillis += 1000;                         // set end time for next cycle
    capacity   += current1;                     // calculate capacity
  }

  resetLoad();                                  // reset the load to minimum
  Serial.println(DONE);                         // transmit end of test
}

// long-term multimeter
void multimeter() {
  resetLoad();
  uint16_t interval   = argument1;
  uint16_t duration   = argument2;
  uint32_t startmillis= millis();
  uint32_t nextmillis = startmillis + interval;
  uint32_t endmillis  = startmillis + (uint32_t)1000 * duration;

  while(millis() <= endmillis) {
    if (isTerminated()) break;                  // stop if termination command was sent
    updatePowerSensors();                       // read all power sensor values
    // transmit values via serial interface
    Serial.print((millis() - startmillis)); Serial.print(SEPERATOR);
    Serial.print(current2); Serial.print(SEPERATOR);
    Serial.println(voltage2);
    while(millis() < nextmillis);               // wait for the next cycle
    nextmillis += interval;                     // set end time for next cycle
  }
  Serial.println(DONE);                         // transmit end of test
}

// sets a load for calibration
void calibrateLoad() {
  uint8_t counter = 0;                          // counts the number of measurements
  uint32_t voltages1 = 0;                       // accumulates all voltage measurements (load)
  uint32_t currents1 = 0;                       // accumulates all current measurements (load)
  uint32_t voltages2 = 0;                       // accumulates all voltage measurements (supply)
  uint32_t currents2 = 0;                       // accumulates all current measurements (supply)
  uint16_t loadcurrent = argument1;             // argument1 is the constant load current
  
  setLoad(loadcurrent);                         // set the constant load current
  delay(10);                                    // a little settle time
  uint32_t endmillis = millis() + (uint32_t)1000 * argument2;  // start the timer
  while(millis() < endmillis) {
    updateSensors();                            // read all sensor values
    if (loadpower > MAXPOWER) break;            // stop when power reaches maximum
    if (loadtemp  > MAXTEMP)  break;            // stop when heatsink is to hot

    // add up the measurements and increase the counter
    voltages1 += voltage1; currents1 += current1; voltages2 += voltage2; currents2 += current2;
    counter++;
    
    // transmit averaged values via serial interface
    Serial.print(currents1 / counter); Serial.print(SEPERATOR);
    Serial.print(voltages1 / counter); Serial.print(SEPERATOR);
    Serial.print(currents2 / counter); Serial.print(SEPERATOR);
    Serial.println(voltages2 / counter);

    delay(100);
  }
  resetLoad();                                  // reset the load to minimum
  Serial.println(DONE);                         // transmit end of test
}
