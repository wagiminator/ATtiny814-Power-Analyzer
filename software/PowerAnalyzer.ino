// ===================================================================================
// Project:   Power Analyzer
// Version:   1.1
// Year:      2020
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// Programmable electronic constant current dummy load with two high side voltage
// and current sensors for an automatic analysis of power supplys, DC/DC converters,
// voltage regulators, batteries, chargers, power consumers and others.
// The device can be controlled via USB serial interface using a serial monitor or
// the provided python skripts. Data can be exported to spread sheet programs or
// directly be analyzed by the python skripts.
//
// Wiring:
// -------
//                            +-\/-+
//                      Vcc  1|°   |14  GND
//     Fan --- !SS AIN4 PA4  2|    |13  PA3 AIN3 SCK ---- 
//     NTC ------- AIN5 PA5  3|    |12  PA2 AIN2 MISO --- 
//    Load --- DAC AIN6 PA6  4|    |11  PA1 AIN1 MOSI --- 
//     LED ------- AIN7 PA7  5|    |10  PA0 AIN0 UPDI --- UPDI
//     USB -------- RXD PB3  6|    |9   PB0 AIN11 SCL --- INA219
//     USB ---------TXD PB2  7|    |8   PB1 AIN10 SDA --- INA219
//                            +----+
//
// Compilation Settings:
// ---------------------
// Core:    megaTinyCore (https://github.com/SpenceKonde/megaTinyCore)
// Board:   ATtiny1614/1604/814/804/414/404/214/204
// Chip:    ATtiny1614 or ATtiny814
// Clock:   20 MHz internal
//
// Leave the rest on default settings. Don't forget to "Burn bootloader"!
// Compile and upload the code.
//
// No Arduino core functions or libraries are used. To compile and upload without
// Arduino IDE download AVR 8-bit toolchain at:
// https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers
// and extract to tools/avr-gcc. Use the makefile to compile and upload.
//
// Fuse Settings: 0:0x00 1:0x00 2:0x02 4:0x00 5:0xC5 6:0x04 7:0x00 8:0x00
//
// ===================================================================================
// Operating Instructions
// ===================================================================================
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


// ===================================================================================
// Libraries, Definitions and Macros
// ===================================================================================

// Libraries
#include <avr/io.h>                     // for GPIO
#include <avr/interrupt.h>              // for interrupts
#include <util/delay.h>                 // for delays

// Pin definitions
#define FAN_PIN     PA4                 // pin the fan is connected to
#define NTC_PIN     PA5                 // pin the temperature sensor is connected to
#define DAC_PIN     PA6                 // DAC output pin, connected to load
#define LED_PIN     PA7                 // pin the status LED is connected to
#define SCL_PIN     PB0                 // I2C serial clock pin, connected to INA219
#define SDA_PIN     PB1                 // I2C serial data pin, connected to INA219
#define TXD_PIN     PB2                 // UART transmit data pin, connected to CH330N
#define RXD_PIN     PB3                 // UART receive data pin, connected to CH330N

// Identifiers
#define VERSION     "1.1"               // version number sent via serial if requested
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
#define INA1ADDR    0b10000000          // I2C write address of INA on the load side 
#define INA2ADDR    0b10000010          // I2C write address of INA on the power side
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

// Pin manipulation macros
enum {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1, PB2, PB3};    // enumerate pin designators
#define pinInput(x)       (&VPORTA.DIR)[((x)&8)>>1] &= ~(1<<((x)&7))  // set pin to INPUT
#define pinOutput(x)      (&VPORTA.DIR)[((x)&8)>>1] |=  (1<<((x)&7))  // set pin to OUTPUT
#define pinLow(x)         (&VPORTA.OUT)[((x)&8)>>1] &= ~(1<<((x)&7))  // set pin to LOW
#define pinHigh(x)        (&VPORTA.OUT)[((x)&8)>>1] |=  (1<<((x)&7))  // set pin to HIGH
#define pinToggle(x)      (&VPORTA.IN )[((x)&8)>>1] |=  (1<<((x)&7))  // TOGGLE pin
#define pinRead(x)        ((&VPORTA.IN)[((x)&8)>>1] &   (1<<((x)&7))) // READ pin
#define pinDisable(x)     (&PORTA.PIN0CTRL)[(((x)&8)<<2)+((x)&7)] |= PORT_ISC_INPUT_DISABLE_gc
#define pinPullup(x)      (&PORTA.PIN0CTRL)[(((x)&8)<<2)+((x)&7)] |= PORT_PULLUPEN_bm
#define pinAIN(x)         ((x)<8 ? (x) : (19-(x)))                    // convert pin to ADC port

// Global variables (voltages in mV, currents in mA, power in mW, shunts in 0.01 mV)
uint16_t voltage1, current1, voltage2, current2, shunt1, shunt2, power2, loadtemp;
uint16_t argument1, argument2;
uint16_t loadpower = 0;
char     cmd;

// ===================================================================================
// UART Implementation - Low Level Functions (8N1, no calibration, with RX-interrupt)
// ===================================================================================

// UART definitions and macros
#define UART_BAUD         115200
#define UART_BAUD_RATE    4.0 * F_CPU / UART_BAUD + 0.5
#define UART_ready()      (USART0.STATUS & USART_DREIF_bm)

// UART init
void UART_init(void) {
  pinOutput(TXD_PIN);                             // set TX pin as output
  USART0.BAUD   = UART_BAUD_RATE;                 // set BAUD
  USART0.CTRLA  = USART_RXCIE_bm;                 // enable RX interrupt
  USART0.CTRLB  = USART_RXEN_bm                   // enable RX
                | USART_TXEN_bm;                  // enable TX
}

// UART transmit data byte
void UART_write(uint8_t data) {
  while(!UART_ready());                           // wait until ready for next data
  USART0.TXDATAL = data;                          // send data byte
}

// ===================================================================================
// UART Implementation - String Functions
// ===================================================================================

// UART print string
void UART_print(const char *str) {
  while(*str) UART_write(*str++);                 // write characters of string
}

// UART print string with new line
void UART_println(const char *str) {
  UART_print(str);                                // print string
  UART_write('\n');                               // send new line command
}

// UART print 16-bit integer as decimal (BCD conversion by substraction method)
void UART_printInt(uint16_t value) {
  static uint16_t divider[5] = {10000, 1000, 100, 10, 1};
  uint8_t leadflag = 0;                           // flag for leading spaces
  for(uint8_t digit = 0; digit < 5; digit++) {    // 5 digits
    uint8_t digitval = 0;                         // start with digit value 0
    while(value >= divider[digit]) {              // if current divider fits into the value
      leadflag = 1;                               // end of leading spaces
      digitval++;                                 // increase digit value
      value -= divider[digit];                    // decrease value by divider
    }
    if(leadflag || (digit == 4)) UART_write(digitval + '0'); // print the digit
  }
}

// ===================================================================================
// UART Implementation - Command Buffer
// ===================================================================================

// UART command buffer and pointer
#define CMD_BUF_LEN 16                            // command buffer length
volatile uint8_t CMD_buffer[CMD_BUF_LEN];         // command buffer
volatile uint8_t CMD_ptr = 0;                     // buffer pointer for writing
volatile uint8_t CMD_compl = 0;                   // command completely received flag

// UART RXC interrupt service routine (read command via UART)
ISR(USART0_RXC_vect) {
  uint8_t data = USART0.RXDATAL;                  // read received data byte
  if(!CMD_compl) {                                // command still incomplete?
    if(data != '\n') {                            // not command end?
      CMD_buffer[CMD_ptr] = data;                 // write received byte to buffer
      if(CMD_ptr < (CMD_BUF_LEN-1)) CMD_ptr++;    // increase and limit pointer
    } else if(CMD_ptr) {                          // received at least one byte?
      CMD_compl = 1;                              // set command complete flag
      CMD_buffer[CMD_ptr] = 0;                    // write string terminator
      CMD_ptr = 0;                                // reset pointer
    }
  }  
}

// ===================================================================================
// I2C Master Implementation (Read/Write, Conservative)
// ===================================================================================

#define I2C_FREQ  400000                          // I2C clock frequency in Hz
#define I2C_BAUD  ((F_CPU / I2C_FREQ) - 10) / 2;  // simplified BAUD calculation

// I2C init function
void I2C_init(void) {
  TWI0.MBAUD   = I2C_BAUD;                        // set TWI master BAUD rate
  TWI0.MCTRLA  = TWI_ENABLE_bm;                   // enable TWI master
  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;            // set bus state to idle
}

// I2C start transmission
void I2C_start(uint8_t addr) {
  TWI0.MADDR = addr;                              // start sending address
  while(!(TWI0.MSTATUS&(TWI_WIF_bm|TWI_RIF_bm))); // wait for transfer to complete
}

// I2C restart transmission
void I2C_restart(uint8_t addr) {
  I2C_start(addr);                                // start sending address
}

// I2C stop transmission
void I2C_stop(void) {
  TWI0.MCTRLB = TWI_MCMD_STOP_gc;                 // send stop condition
}

// I2C transmit one data byte to the slave, ignore ACK bit
void I2C_write(uint8_t data) {
  TWI0.MDATA = data;                              // start sending data byte
  while(~TWI0.MSTATUS & TWI_WIF_bm);              // wait for transfer to complete
}

// I2C receive one data byte from slave; ack=0: last byte, ack>0: more bytes to follow
uint8_t I2C_read(uint8_t ack) {
  while(~TWI0.MSTATUS & TWI_RIF_bm);              // wait for transfer to complete
  uint8_t data = TWI0.MDATA;                      // get received data byte
  if(ack) TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;    // ACK:  read more bytes
  else    TWI0.MCTRLB = TWI_ACKACT_NACK_gc;       // NACK: this was the last byte
  return data;                                    // return received byte
}

// ===================================================================================
// INA219 Implementation
// ===================================================================================

// Writes  register value to the INA219
void INA_write(uint8_t addr, uint8_t reg, uint16_t value) {
  I2C_start(addr);
  I2C_write(reg);
  I2C_write(value >> 8);
  I2C_write(value);
  I2C_stop();
}

// Read a register from the INA219
uint16_t INA_read(uint8_t addr, uint8_t reg) {
  I2C_start(addr);
  I2C_write(reg);
  I2C_restart(addr | 1);
  uint16_t result = (uint16_t)(I2C_read(1) << 8) | I2C_read(0);
  I2C_stop();
  return result;
}

// Write inital configuration and calibration values to the INAs
void INA_init(void) {
  INA_write(INA1ADDR, CONFIG_REG, INA1CONFIG);
  INA_write(INA1ADDR, CALIB_REG,  ILCAL1 * INACALIB);
  INA_write(INA2ADDR, CONFIG_REG, INA2CONFIG);
  INA_write(INA2ADDR, CALIB_REG,  ILCAL2 * INACALIB);
}

// ===================================================================================
// Millis Counter Implementation for TCB0
// ===================================================================================

volatile uint32_t MIL_counter = 0;                // millis counter variable

// Init millis counter
void MIL_init(void) {
  TCB0.CCMP    = (F_CPU / 1000) - 1;              // set TOP value (period)
  TCB0.CTRLA   = TCB_ENABLE_bm;                   // enable timer/counter
  TCB0.INTCTRL = TCB_CAPT_bm;                     // enable periodic interrupt
}

// Read millis counter
uint32_t MIL_read(void) {
  cli();                                          // disable interrupt for atomic read
  uint32_t result = MIL_counter;                  // read millis counter
  sei();                                          // enable interrupt again
  return result;                                  // return millis counter value
}

// TCB0 interrupt service routine (every millisecond)
ISR(TCB0_INT_vect) {
  TCB0.INTFLAGS = TCB_CAPT_bm;                    // clear interrupt flag
  MIL_counter++;                                  // increase millis counter
}

// ===================================================================================
// DAC Implementation and Load Control Functions
// ===================================================================================

// DAC reference voltages (load current = DAC voltage * R16 / (R15 + R16) / R_SHUNT)
// Reference voltages:    0.55V, 1.1V, 1.5V, 2.5V, 4.3V
const uint8_t  DACREF[] = {0x00, 0x01, 0x04, 0x02, 0x03}; // CTRLA.DAC0REFSEL values
const uint16_t DACCUR[] = { 717, 1434, 1956, 3260, 5608}; // max current in mA
uint8_t DACreference = 0;                                 // start with 0.55V reference

// Setup the digital to analog converter (DAC)
void DAC_init(void) {
  VREF_CTRLB |= VREF_DAC0REFEN_bm;                // enable DAC reference
  _delay_us(25);                                  // wait for Vref to start up
  pinDisable(DAC_PIN);                            // disable digital input buffer
  DAC0.CTRLA  = DAC_ENABLE_bm                     // enable DAC
              | DAC_OUTEN_bm;                     // enable output buffer
}

// Set the lowest reference voltage possible for the DAC to meet the load current
void DAC_setReference(uint16_t current) {
  DACreference = 0;
  if(current > DACCUR[4]) current = DACCUR[4];
  while(current > DACCUR[DACreference]) DACreference++;
  DAC0.DATA   = 0;
  VREF_CTRLA &= 0xf8;
  VREF_CTRLA |= DACREF[DACreference];
  _delay_us(25);
}

// Set the DAC within the selected reference to the specified load current
void DAC_set(uint16_t current) {
  if(current > 5000) current = 5000;
  if(current > DACCUR[DACreference]) DAC0.DATA = 255;
  else DAC0.DATA = (uint32_t)255 * current / DACCUR[DACreference];
}

// Set the DAC and its reference to the specified load current
void DAC_setLoad(uint16_t current) {
  DAC_setReference(current);                      // set suitable voltage reference
  DAC_set(current);                               // set DAC according to desired load
}

// Reset the load to minimum
void DAC_resetLoad(void) {
  DAC_setLoad(0);                                 // reset the load to minimum
}

// ===================================================================================
// ADC Implementation
// ===================================================================================

// ADC init
void ADC_init(void) {
  ADC0.CTRLA  = ADC_ENABLE_bm;                    // enable ADC, 10 bits, single shot
  ADC0.CTRLC  = ADC_SAMPCAP_bm                    // reduced size of sampling capacitance
              | ADC_REFSEL_VDDREF_gc              // set VCC as reference
              | ADC_PRESC_DIV16_gc;               // set prescaler for 1.25 MHz ADC clock
}

// ADC sample and read
uint16_t ADC_read(uint8_t port) {
  ADC0.MUXPOS   = port;                           // set muxer to desired port
  ADC0.INTFLAGS = ADC_RESRDY_bm;                  // clear result ready intflag
  ADC0.COMMAND  = ADC_STCONV_bm;                  // start sampling
  while(~ADC0.INTFLAGS & ADC_RESRDY_bm);          // wait for ADC sampling to complete
  return ADC0.RES;                                // read and return sampling result
}

// ===================================================================================
// Sensors Implementation
// ===================================================================================

// Read all sensors of the electronic load and set the fan
void updateLoadSensors(void) { 
  // Read values from INA on the load side
  shunt1   = INA_read(INA1ADDR, SHUNT_REG);
  if(shunt1 > 4095) shunt1 = 0;
  voltage1 = (INA_read(INA1ADDR, VOLTAGE_REG) >> 1) & 0xfffc;
  voltage1 = ((float)shunt1 / 100 + voltage1) * ULCAL1 + 0.5;
  current1 = INA_read(INA1ADDR, CURRENT_REG);
  if(current1 > 32767) current1 = 0;

  // Calculate load power and read temperature of the heatsink
  loadpower= ((uint32_t)voltage1 * current1 + 500) / 1000;
  loadtemp = ADC_read(pinAIN(NTC_PIN));

  // Turn fan on or off depending on load power and temperature
  if((loadpower > MAXPOWER)    || (loadtemp > MAXTEMP))    DAC_resetLoad();
  if((loadpower > FANONPOWER)  || (loadtemp > FANONTEMP))  pinHigh(FAN_PIN);
  if((loadpower < FANOFFPOWER) && (loadtemp < FANOFFTEMP)) pinLow(FAN_PIN); 
}

// Read voltage and current of PWR-IN/PWR-OUT
void updatePowerSensors(void) {
  voltage2 = (INA_read(INA2ADDR, VOLTAGE_REG) >> 1) & 0xfffc;
  voltage2 = (float)voltage2 * ULCAL2 + 0.5;
  current2 = INA_read(INA2ADDR, CURRENT_REG);
  if(current2 > 32767) current2 = 0;
}

// Read all sensors and updates the corresponding variables
void updateSensors(void) {
  updateLoadSensors();
  updatePowerSensors();
}

// Read sensor values and transmit them via serial interface
void transmitSensors(void) {
  updateSensors();
  UART_printInt(current1); UART_print(SEPERATOR);
  UART_printInt(voltage1); UART_print(SEPERATOR);
  UART_printInt(current2); UART_print(SEPERATOR);
  UART_printInt(voltage2); UART_println("");
}

// ===================================================================================
// Command Buffer Parser
// ===================================================================================

// Wait for, read and parse command string
void CMD_read(void) {
  while(!CMD_compl) updateLoadSensors();        // maintain fan control
  uint8_t i = 0;
  cmd = CMD_buffer[0];
  argument1 = 0; argument2 = 0;
  while(CMD_buffer[++i] == ' ');
  while(CMD_buffer[i] > ' ') argument1 = argument1 * 10 + CMD_buffer[i++] - '0';
  while(CMD_buffer[i] == ' ') i++;
  while(CMD_buffer[i] != 0)  argument2 = argument2 * 10 + CMD_buffer[i++] - '0';
  CMD_compl = 0;
}

// Check if termination command was send during a test program
uint8_t CMD_isTerminated(void) {
  if(CMD_compl) return(CMD_buffer[0] == 'x');
  return 0;
}

// ===================================================================================
// Test Algorithms
// ===================================================================================

// Perform automatic load test according to minimum load voltage and maximum load current
void loadTest(void) {
  uint8_t  DACvalue = 0; DAC0.DATA = 0;
  uint16_t maxloadcurrent = argument1;
  uint16_t minloadvoltage = argument2;
  if(maxloadcurrent > 4999) maxloadcurrent = 4999;
  
  DAC_setReference(maxloadcurrent);               // set DAC reference according to max current
  while(++DACvalue) {                             // increase load every cycle
    updateLoadSensors();                          // read all load sensor values
    if(loadpower > MAXPOWER) break;               // stop when power reaches maximum
    if(loadtemp  > MAXTEMP)  break;               // stop when heatsink is to hot
    if(voltage1  < minloadvoltage) break;         // stop when voltage falls below minimum
    if(current1  > maxloadcurrent) break;         // stop when current reaches maximum
    DAC0.DATA = DACvalue;                         // set load for next measurement

    // Transmit values via serial interface
    UART_printInt(current1);  UART_print(SEPERATOR);
    UART_printInt(voltage1);  UART_print(SEPERATOR);
    UART_printInt(loadpower); UART_println("");
    
    _delay_ms(SETTLE);                            // give everything a little time to settle
  }
  DAC_resetLoad();                                // reset the load to minimum
  UART_println(DONE);                             // transmit end of test
}

// Perform a voltage regulation test up to the max load current
void regulationTest(void) {
  uint16_t maxloadcurrent = argument1;
  uint8_t  profile[] = {0, 2, 5, 7, 10, 10, 5, 0, 10, 0};
  INA_write(INA1ADDR, CONFIG_REG, INAFASTBOTH);   // speed-up sampling rate of INA
  DAC_setReference(maxloadcurrent);
  uint32_t startmillis = MIL_read();
  uint32_t nextmillis  = startmillis + 100;

  for(uint8_t i=0; i<10; i++) {
    DAC_set(maxloadcurrent * profile[i] / 10);
    while(MIL_read() < nextmillis) {
      updateLoadSensors();                        // read all load sensor values
      if(loadpower > MAXPOWER) break;             // stop when power reaches maximum
      if(loadtemp  > MAXTEMP)  break;             // stop when heatsink is to hot
      UART_printInt(MIL_read() - startmillis); UART_print(SEPERATOR);
      UART_printInt(current1); UART_print(SEPERATOR);
      UART_printInt(voltage1); UART_println("");
    }
    nextmillis += 100;
  }
  DAC_resetLoad();                                // reset the load to minimum
  INA_write(INA1ADDR, CONFIG_REG, INA1CONFIG);    // INA back to normal operation
  UART_println(DONE);                             // transmit end of test
}

// Perform automatic efficiency test according to minimum load voltage and maximum load current
void efficiencyTest(void) {
  uint16_t supplypower;
  uint16_t efficiency;
  uint8_t  DACvalue = 0; DAC0.DATA = 0;
  uint16_t maxloadcurrent = argument1;
  uint16_t minloadvoltage = argument2;
  if(maxloadcurrent > 4999) maxloadcurrent = 4999;
  
  DAC_setReference(maxloadcurrent);               // set DAC reference according to max current
  while(++DACvalue) {                             // increase load every cycle
    updateSensors();                              // read all sensor values
    if(loadpower > MAXPOWER) break;               // stop when power reaches maximum
    if(loadtemp  > MAXTEMP)  break;               // stop when heatsink is to hot
    if(voltage1  < minloadvoltage) break;         // stop when voltage falls below minimum
    if(current1  > maxloadcurrent) break;         // stop when current reaches maximum
    DAC0.DATA = DACvalue;                         // set load for next measurement

    // Calculate efficiency
    supplypower = ((uint32_t)voltage2 * current2 + 500) / 1000;
    efficiency  = (float)loadpower * 1000 / supplypower + 0.5;

    // Transmit values via serial interface
    UART_printInt(current1);   UART_print(SEPERATOR);
    UART_printInt(voltage1);   UART_print(SEPERATOR);
    UART_printInt(efficiency); UART_println("");
    
    _delay_ms(SETTLE);                            // give everything a little time to settle
  }  
  DAC_resetLoad();                                // reset the load to minimum
  UART_println(DONE);                             // transmit end of test
}

// Perform low frequency ripple test
void rippleTest(void) {
  uint8_t  DACvalue = 0; DAC0.DATA = 0;
  uint16_t minvoltage, maxvoltage;
  uint16_t maxloadcurrent = argument1;
  uint16_t minloadvoltage = argument2;
  if(maxloadcurrent > 4999) maxloadcurrent = 4999;
  
  DAC_setReference(maxloadcurrent);               // set DAC reference according to max current
  while(++DACvalue) {                             // increase load every cycle
    updateLoadSensors();                          // read all load sensor values
    if(loadpower > MAXPOWER) break;               // stop when power reaches maximum
    if(loadtemp  > MAXTEMP)  break;               // stop when heatsink is to hot
    if(voltage1  < minloadvoltage) break;         // stop when voltage falls below minimum
    if(current1  > maxloadcurrent) break;         // stop when current reaches maximum
    INA_write(INA1ADDR, CONFIG_REG, INAFASTBUS);  // speed-up sampling rate of INA
    minvoltage = 26000; maxvoltage = 0;           // reset peak voltage values
    for(uint8_t i=255; i; i--) {                  // get 255 voltage readings
      // Read voltage and update peak values
      voltage1 = (INA_read(INA1ADDR, VOLTAGE_REG) >> 1) & 0xfffc;
      if(voltage1 > maxvoltage) maxvoltage = voltage1;
      if(voltage1 < minvoltage) minvoltage = voltage1;
    }
    INA_write(INA1ADDR, CONFIG_REG, INA1CONFIG);  // INA back to normal operation
    DAC0.DATA = DACvalue;                         // set load for next measurement

    // Transmit values via serial interface
    UART_printInt(current1); UART_print(SEPERATOR);
    UART_printInt(maxvoltage - minvoltage); UART_println("");
    _delay_ms(SETTLE);                            // give everything a little time to settle
  } 
  DAC_resetLoad();                                // reset the load to minimum
  UART_println(DONE);                             // transmit end of test
}

// Perform a battery discharge test
void batteryTest(void) {
  uint32_t capacity       = 0;
  uint16_t maxloadcurrent = argument1;
  uint16_t minloadvoltage = argument2;
  uint32_t startmillis    = MIL_read();
  uint32_t nextmillis     = startmillis + 1000;
  if(maxloadcurrent > 3000) maxloadcurrent = 3000;
  
  DAC_setLoad(maxloadcurrent);                    // set the constant load current
  _delay_ms(SETTLE);                              // give everything a little time to settle
  while(DAC0.DATA) {                              // repeat until load is zero
    updateLoadSensors();                          // read all load sensor values
    if(loadpower > MAXPOWER) break;               // stop when power reaches maximum
    if(loadtemp  > MAXTEMP)  break;               // stop when heatsink is to hot
    if(CMD_isTerminated()) break;                 // stop if termination command was sent

    // Decrease load if voltage drops below minloadvoltage
    if(voltage1 < minloadvoltage) DAC0.DATA--;

    // Transmit values via serial interface
    UART_printInt((MIL_read() - startmillis) / 1000); UART_print(SEPERATOR);
    UART_printInt(current1); UART_print(SEPERATOR);
    UART_printInt(voltage1); UART_print(SEPERATOR);
    UART_printInt(capacity / 3600); UART_println("");
    
    while(MIL_read() < nextmillis);               // wait for the next cycle (one cycle/second)
    nextmillis += 1000;                           // set end time for next cycle
    capacity   += current1;                       // calculate capacity
  }
  DAC_resetLoad();                                // reset the load to minimum
  UART_println(DONE);                             // transmit end of test
}

// Long-term multimeter
void multimeter(void) {
  DAC_resetLoad();
  uint16_t interval    = argument1;
  uint16_t duration    = argument2;
  uint32_t startmillis = MIL_read();
  uint32_t nextmillis  = startmillis + interval;
  uint32_t endmillis   = startmillis + (uint32_t)1000 * duration;

  while(MIL_read() <= endmillis) {
    if(CMD_isTerminated()) break;                 // stop if termination command was sent
    updatePowerSensors();                         // read all power sensor values
    UART_printInt(MIL_read() - startmillis); UART_print(SEPERATOR);
    UART_printInt(current2); UART_print(SEPERATOR);
    UART_printInt(voltage2); UART_println("");
    while(MIL_read() < nextmillis);               // wait for the next cycle
    nextmillis += interval;                       // set end time for next cycle
  }
  UART_println(DONE);                             // transmit end of test
}

// Set a load for calibration
void calibrateLoad(void) {
  uint8_t  counter     = 0;                       // counts the number of measurements
  uint32_t voltages1   = 0;                       // accumulates all voltage measurements (load)
  uint32_t currents1   = 0;                       // accumulates all current measurements (load)
  uint32_t voltages2   = 0;                       // accumulates all voltage measurements (supply)
  uint32_t currents2   = 0;                       // accumulates all current measurements (supply)
  uint16_t loadcurrent = argument1;               // argument1 is the constant load current
  
  DAC_setLoad(loadcurrent);                       // set the constant load current
  _delay_ms(10);                                  // a little settle time
  uint32_t endmillis = MIL_read() + (uint32_t)1000 * argument2;  // start the timer
  while(MIL_read() < endmillis) {
    updateSensors();                              // read all sensor values
    if(loadpower > MAXPOWER) break;               // stop when power reaches maximum
    if(loadtemp  > MAXTEMP)  break;               // stop when heatsink is to hot

    // Add up the measurements and increase the counter
    voltages1 += voltage1; currents1 += current1; voltages2 += voltage2; currents2 += current2;
    counter++;
    
    // Transmit averaged values via serial interface
    UART_printInt(currents1 / counter); UART_print(SEPERATOR);
    UART_printInt(voltages1 / counter); UART_print(SEPERATOR);
    UART_printInt(currents2 / counter); UART_print(SEPERATOR);
    UART_printInt(voltages2 / counter); UART_println("");

    _delay_ms(100);
  }
  DAC_resetLoad();                                // reset the load to minimum
  UART_println(DONE);                             // transmit end of test
}

// ===================================================================================
// Main Function
// ===================================================================================

int main(void) {
  // Setup MCU
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0);         // set clock frequency to 20 MHz

  // Setup modules
  UART_init();                                    // init UART
  I2C_init();                                     // init I2C
  INA_init();                                     // init INA219
  DAC_init();                                     // init DAC
  ADC_init();                                     // init ADC
  MIL_init();                                     // init millis counter
  sei();                                          // enable interrupts

  // Setup pins
  pinOutput(FAN_PIN);                             // set fan pin as output
  pinOutput(LED_PIN);                             // set LED pin as output
  pinDisable(NTC_PIN);                            // disable digital input buffer

  // Loop
  while(1) {
    CMD_read();                                   // wait for and read command
    pinHigh(LED_PIN);                             // set BUSY LED
    switch(cmd) {
      case 'i':   UART_println(IDENT);   break;   // send identification
      case 'v':   UART_println(VERSION); break;   // send version number
      case 'c':   calibrateLoad();       break;   // set load for calibration
      case 'l':   loadTest();            break;   // perform load test
      case 'g':   regulationTest();      break;   // perform regulation test
      case 'e':   efficiencyTest();      break;   // perform efficiency test
      case 'f':   rippleTest();          break;   // perform ripple test
      case 'b':   batteryTest();         break;   // perform battery test
      case 'm':   multimeter();          break;   // long-term multimeter
      case 't':   transmitSensors();     break;   // read and transmit sensor values
      case 'r':   DAC_resetLoad();                // reset the load
                  UART_println(DONE);    break;
      case 's':   DAC_setLoad(argument1);         // set load current
                  UART_println(DONE);    break; 
      default:    break;    
    }
    pinLow(LED_PIN);                              // set READY LED
  }
}
