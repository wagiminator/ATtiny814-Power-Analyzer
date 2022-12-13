# Power Supply Analyzer with USB Interface based on ATtiny814
The Power Analyzer is a programmable electronic constant current dummy load with two high side voltage and current sensors for an automatic analysis of power supplys, DC/DC converters, voltage regulators, batteries, chargers, power consumers and others. The device can be controlled via USB serial interface using a serial monitor or the provided Python skripts. Data can be exported to spread sheet programs or directly be analyzed by the Python skript.

- Project Video (YouTube): https://youtu.be/q4-aywmeqHs
- Design Files (EasyEDA): https://easyeda.com/wagiminator/y-attiny814-power-analyzer

![pic1.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny814-Power-Analyzer/master/documentation/PowerAnalyzer_pic1.jpg)

# Hardware
The electronic load control circuit, which essentially consists of an operational amplifier, a MOSFET and a shunt resistor, ensures that the same current flows regardless of the voltage applied.

![PowerAnalyzer_wiring.png](https://raw.githubusercontent.com/wagiminator/ATtiny814-Power-Analyzer/master/documentation/PowerAnalyzer_wiring.png)

For this purpose, a 100mΩ shunt consisting of three 300mΩ resistors in parallel for proper heat dissipation is located in the load circuit, via which the current is measured. The [LMV321](https://www.onsemi.com/pdf/datasheet/lmv321-d.pdf) rail-to-rail OpAmp compares this with the target value, which is specified by the ATtiny's internal digital to analog converter (DAC) via a voltage divider and accordingly controls the gate of an [IRL540N](https://datasheet.lcsc.com/lcsc/1808281632_Infineon-Technologies-IRL540NPBF_C111607.pdf) logic level power MOSFET, which in turn adjusts the current through its internal resistance set in this way.

Voltage and current are measured via a high side 8mΩ shunt resistor connected to an [INA219](https://www.ti.com/lit/ds/symlink/ina219.pdf) with a resolution of 4mV/1mA. A second INA219 is connected to another 8mΩ shunt resistor between the PWR-IN and PWR-OUT terminal. The INA219 is a current shunt and power monitor with an I²C-compatible interface. The device monitors both shunt voltage drop and bus supply voltage, with programmable conversion times and filtering. A programmable calibration value, combined with an internal multiplier, enables direct readouts of current in amperes. The selected shunt resistance of 8mΩ enables both a very small influence on the circuit and a measurement with a resolution of 1mA. For an accurate measurement, a shunt resistor with a low tolerance (1% or better) should be selected.

The Power Analyzer is connected via USB to a PC or a RaspberryPi. Commands to the Analyzer can be sent via a serial monitor or by the GUI-based Python skript. The Analyzer has different built-in automatic test algorithms. The collected data is sent back via the serial interface/USB to the PC/RaspberryPi. The ATtiny814 constantly measures power and temperature of the heatsink. It controls the fan and cuts off the load when the temperature gets too hot.

![PowerAnalyzer_block.png](https://raw.githubusercontent.com/wagiminator/ATtiny814-Power-Analyzer/master/documentation/PowerAnalyzer_block.png)

# Software
## Load Control via DAC
The ATtiny814 controls the electronic dummy load with its internal digital to analog converter (DAC). All of its 5 internal reference voltages are being used in order to get the maximum accuracy and resolution of the DAC. The DAC is connected to an OpAmp which acts as a unity gain amplifier controlling the resistance of the MOSFET.

```c
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
```

## Parsing Commands via UART
Commands sent via the USB-to-serial converter are stored in a 16-byte command buffer. This is done via interrupts so that load and fan control, for example, can continue to run in parallel. As soon as a command has been completely received, the CMD_compl flag is set. The parser then extracts the command and the arguments.

```c
// UART definitions and macros
#define UART_BAUD         115200
#define UART_BAUD_RATE    4.0 * F_CPU / UART_BAUD + 0.5
#define UART_ready()      (USART0.STATUS & USART_DREIF_bm)

// UART command buffer and pointer
#define CMD_BUF_LEN 16                            // command buffer length
volatile uint8_t CMD_buffer[CMD_BUF_LEN];         // command buffer
volatile uint8_t CMD_ptr = 0;                     // buffer pointer for writing
volatile uint8_t CMD_compl = 0;                   // command completely received flag

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

// Wait for, read and parse command string
void CMD_read(void) {
  while(!CMD_compl) updateLoadSensors();          // maintain fan control
  uint8_t i = 0;
  cmd = CMD_buffer[0];
  argument1 = 0; argument2 = 0;
  while(CMD_buffer[++i] == ' ');
  while(CMD_buffer[i] > ' ') argument1 = argument1 * 10 + CMD_buffer[i++] - '0';
  while(CMD_buffer[i] == ' ') i++;
  while(CMD_buffer[i] != 0)  argument2 = argument2 * 10 + CMD_buffer[i++] - '0';
  CMD_compl = 0;
}
```

## Compiling and Uploading the Firmware
### If using the Arduino IDE
- Open your Arduino IDE.
- Make sure you have installed [megaTinyCore](https://github.com/SpenceKonde/megaTinyCore).
- Go to **Tools -> Board -> megaTinyCore** and select **ATtiny1614/1604/814/804/414/404/214/204**.
- Go to **Tools** and choose the following board options:
  - **Chip:**           ATtiny1614 or ATtiny814
  - **Clock:**          20 MHz internal
  - Leave the rest at the default settings.
- Connect your programmer to your PC and to the UPDI header on the board.
- Go to **Tools -> Programmer** and select your UPDI programmer.
- Go to **Tools -> Burn Bootloader** to burn the fuses.
- Open the sketch and click **Upload**.

### If using the makefile (Linux/Mac)
- Connect your [programmer](https://github.com/wagiminator/AVR-Programmer) (jtag2updi or SerialUPDI) to your PC and to the UPDI header on the board.
- Make sure you have installed the latest [avr-gcc toolchain](http://maxembedded.com/2015/06/setting-up-avr-gcc-toolchain-on-linux-and-mac-os-x/).
- Open a terminal.
- Navigate to the folder with the makefile and the sketch.
- Run `DEVICE=attiny814 PROGRMR=serialupdi PORT=/dev/ttyUSB0 make install` to compile, burn the fuses and upload the firmware (change DEVICE, PROGRMR and PORT accordingly).

## Installing Python and Drivers
Python needs to be installed on your PC in order to use the GUI-based Python application. Most Linux distributions already include this. Windows users can follow these [instructions](https://www.pythontutorial.net/getting-started/install-python/). In addition PySerial and Tkinter (8.6 or newer) must be installed. However, these are already included in most Python installations.

Windows users may also need to install a [driver](http://www.wch.cn/download/CH341SER_ZIP.html) for the CH330N/CH340N USB to serial adapter. This is not necessary for Linux or Mac users.

# Operating Instructions
The device can be operated in two ways:
- Using a serial monitor: Test algorithms can be started by sending the corresponding command via a serial monitor. The collected data will be displayed in the serial monitor and can be exported to a spread sheet program for further analysis.
- Using the GUI-based Python application: This is the easy way. Everything should be self-explanatory. All following example pictures are created by this application.

## **Load Test**
![PowerAnalyzer_block_load.png](https://raw.githubusercontent.com/wagiminator/ATtiny814-Power-Analyzer/master/documentation/PowerAnalyzer_block_load.png)
- Command: "l  *maxloadcurrent[mA: 17..5000]* *minloadvoltage[mV: 0..26000]*"
- Example: "l 2500 4200"
- The Power Analyzer continuously increases the load from 17 mA up to *maxloadcurrent*. It stops automatically if the voltage drops below *minloadvoltage*. It continuously transmits the measured values via the serial interface in the format: current[mA] voltage[mV] power[mW] (seperated by the SEPERATOR string).

![PowerAnalyzer_chart_load.png](https://raw.githubusercontent.com/wagiminator/ATtiny814-Power-Analyzer/master/documentation/PowerAnalyzer_chart_load.png)

## **Voltage Regulation Test**
![PowerAnalyzer_block_regulation.png](https://raw.githubusercontent.com/wagiminator/ATtiny814-Power-Analyzer/master/documentation/PowerAnalyzer_block_regulation.png)
- Command: "g *maxloadcurrent[mA: 17..5000]*"
- Example: "g 3000"
- The Power Analyzer changes rapidly the load between 17 mA and *maxloadcurrent*. It continuously transmits the measured values via the serial interface in the format: time[ms] current[mA] voltage[mV] (seperated by the SEPERATOR string).

![PowerAnalyzer_chart_regulation.png](https://raw.githubusercontent.com/wagiminator/ATtiny814-Power-Analyzer/master/documentation/PowerAnalyzer_chart_regulation.png)

## **Efficiency Test**
![PowerAnalyzer_block_efficiency.png](https://raw.githubusercontent.com/wagiminator/ATtiny814-Power-Analyzer/master/documentation/PowerAnalyzer_block_efficiency.png)
- Command: "e  *maxloadcurrent[mA: 17..5000]* *minloadvoltage[mV: 0..26000]*"
- Example: "e 4000 2500"
- The Power Analyzer continuously increases the load from 17 mA up to *maxloadcurrent*. It stops automatically if the voltage at TEST-IN drops below *minloadvoltage*. It continuously transmits the measured values via the serial interface in the format: current[mA] voltage[mV] efficiency[% * 10] (seperated by the SEPERATOR string).

![PowerAnalyzer_chart_efficiency.png](https://raw.githubusercontent.com/wagiminator/ATtiny814-Power-Analyzer/master/documentation/PowerAnalyzer_chart_efficiency.png)

## **Battery Discharge Test**
![PowerAnalyzer_block_battery.png](https://raw.githubusercontent.com/wagiminator/ATtiny814-Power-Analyzer/master/documentation/PowerAnalyzer_block_battery.png)
- Command: "b  *maxloadcurrent[mA: 17..5000]* *minloadvoltage[mV: 0..26000]*"
- Example: "l 1000 2700"
- The Power Analyzer sets a constant current load of *maxloadcurrent*. If the voltage drops below *minloadvoltage* it constantly decreases the load to maintain *minloadvoltage*. It stops automatically if the load current drops to 0mA. It continuously transmits the measured values via the serial interface in the format: time[s] current[mA] voltage[mV] capacity[mAh] (seperated by the SEPERATOR string).

![PowerAnalyzer_chart_battery.png](https://raw.githubusercontent.com/wagiminator/ATtiny814-Power-Analyzer/master/documentation/PowerAnalyzer_chart_battery.png)

## **Long-Term Multimeter**
![PowerAnalyzer_block_multimeter.png](https://raw.githubusercontent.com/wagiminator/ATtiny814-Power-Analyzer/master/documentation/PowerAnalyzer_block_multimeter.png)
- Command: "m  *interval[ms: 2..65535]* *duration[s: 1..65535]*"
- Example: "m 18000 18000"
- The Power Analyzer measures voltage, current and power delivered to the test device at every *interval* for a total of *duration*. It continuously transmits the measured values via the serial interface in the format: time[ms] current[mA] voltage[mV] (seperated by the SEPERATOR string).

![PowerAnalyzer_chart_multimeter.png](https://raw.githubusercontent.com/wagiminator/ATtiny814-Power-Analyzer/master/documentation/PowerAnalyzer_chart_multimeter.png)

## **Commands for Direct Control**
|Command|Function|
|-|-|
|"i"|transmits indentification string ("Power Analyzer")|
|"v"|transmits firmware version number|
|"x"|terminate current test program|
|"s *loadcurrent[mA]*"|set load to a constant current of *loadcurrent*|
|"r"|reset the load to minimum|
|"t"|read current and voltage of both sensors and transmit them|

# Notes
- Use a good heatsink with a 5V fan for the MOSFET! Attach a 10K 3950B NTC thermistor to the heatsink close to the MOSFET!
- Be careful with high power loads! Make some tests to figure out what can be achieved with your cooling solution!
- Due to the limitations of the cheap OpAmp and the internal DAC the minimum load current is around 17mA. You can choose a better OpAmp if you like (must have same pinout, must be rail-to-rail and unity gain stable), but for most cases this is not necessary.
- The maximum load current is 5A, however for small voltages it might be less.
- The maximum PWR-IN/PWR-OUT current is 8A.
- Do not exceed the maximum voltage of 26V on all connectors !
- In order to make the design much simpler all connectors including USB share a common ground. Keep this in mind when making your test setup in order to avoid ground loops or shorts. Using a [USB isolator](https://github.com/wagiminator/ADuM3160-USB-Isolator) between the Analyzer and your PC is not a bad idea!
- The CH330N can be replaced with a CH340N. Windows users may need to install a [driver](http://www.wch.cn/download/CH341SER_ZIP.html). This is not necessary for linux users.
- You need a UPDI programmer for uploading the firmware. You can find one in my [projects](https://github.com/wagiminator/AVR-Programmer) or you can easily build one following this [guide](https://github.com/SpenceKonde/AVR-Guidance/blob/master/UPDI/jtag2updi.md).
- The Python skript was only tested on Linux, but it should also work on other operating systems.

![pic2.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny814-Power-Analyzer/master/documentation/PowerAnalyzer_pic2.jpg)

# License
![license.png](https://i.creativecommons.org/l/by-sa/3.0/88x31.png)

This work is licensed under Creative Commons Attribution-ShareAlike 3.0 Unported License. 
(http://creativecommons.org/licenses/by-sa/3.0/)
