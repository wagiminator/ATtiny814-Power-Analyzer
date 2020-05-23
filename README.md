# Power Supply Analyzer with USB Interface based on ATtiny814

# 1. Overview #

The Power Analyzer is a programmable electronic constant current dummy load with two high side voltage and current sensors for an automatic analysis of power supplys, DC/DC converters, voltage regulators, batteries, chargers, power consumers and others. The device can be controlled via USB serial interface using a serial monitor or the provided Python skripts. Data can be exported to spread sheet programs or directly be analyzed by the Python skript.

Video: https://youtu.be/q4-aywmeqHs

![IMG_20200418_124716_x.jpg](https://image.easyeda.com/pullimage/JiItuSNVWCwB8sDvpDRtLE4U1jW5eZ8c6Ffm1ckF.jpeg)

# 2. Working Principle #

The ATtiny814 controls the electronic dummy load with its internal digital to analog converter (DAC). All of its 5 internal reference voltages are being used in order to get the maximum accuracy and resolution of the DAC. The DAC is connected to an OpAmp which acts as a unity gain amplifier controlling the resistance of the MOSFET. Voltage and current are measured via a high side 8 mOhm shunt resistor connected to an INA219 with a resolution of 4mV/1mA. A second INA219 is connected to another 8 mOhm shunt resistor between the PWR-IN and PWR-OUT terminal. The Power Analyzer is connected via USB to a PC or a RaspberryPi. Commands to the Analyzer can be sent via a serial monitor or by the GUI-based Python skript. The Analyzer has different built-in automatic test algorithms. The collected data is sent back via the serial interface/USB to the PC/RaspberryPi. The ATtiny814 constantly measures power and temperature of the heatsink. It controls the fan and cuts off the load when the temperature gets too hot.

![analyzer.png](https://image.easyeda.com/pullimage/5JHEmxEiDpetM5mmhL0AHnsd0qrTjO99EDUd7R2A.png)

# 3. Test Algorithms #

- Using a serial monitor: Test algorithms can be started by sending the corresponding command via a serial monitor. The collected data will be displayed in the serial monitor and can be exported to a spread sheet program for further analysis.
- Using the GUI-based python application: This is the easy way. Everything should be self-explanatory. All following example pictures are created by this application.

# **Load Test** #

![PA_load.png](https://image.easyeda.com/pullimage/Tb0V0PMVHk0lj8hvDW5fH4EMr7WinR5Uau3vDT5e.png)
- Command: "l  *maxloadcurrent[mA: 17..5000]* *minloadvoltage[mV: 0..26000]*"
- Example: "l 2500 4200"
- The Power Analyzer continuously increases the load from 17 mA up to *maxloadcurrent*. It stops automatically if the voltage drops below *minloadvoltage*. It continuously transmits the measured values via the serial interface in the format: current[mA] voltage[mV] power[mW] (seperated by the SEPERATOR string).

![Powerbank_load_x.png](https://image.easyeda.com/pullimage/rnJ77VPvbiIG7qtHjKaOPW24Ux02ae1oXPcEBA0R.png)

# **Voltage Regulation Test** #

![PA_load.png](https://image.easyeda.com/pullimage/YirhE95f7BDj9JcpMuqxlahK3m5FGmSDD9HGJUSL.png)
- Command: "g *maxloadcurrent[mA: 17..5000]*"
- Example: "g 3000"
- The Power Analyzer changes rapidly the load between 17 mA and *maxloadcurrent*. It continuously transmits the measured values via the serial interface in the format: time[ms] current[mA] voltage[mV] (seperated by the SEPERATOR string).

![FP6277_regulation_x.png](https://image.easyeda.com/pullimage/HpoK9MOV3x21YkKqRrJeRTs44hCF3q4Bzjfpgy6C.png)

# **Efficiency Test** #

![PA_efficiency.png](https://image.easyeda.com/pullimage/XljSDpb6NbbVamf6RN3b3J5UD0ugtc6V27MuJUKG.png)
- Command: "e  *maxloadcurrent[mA: 17..5000]* *minloadvoltage[mV: 0..26000]*"
- Example: "e 4000 2500"
- The Power Analyzer continuously increases the load from 17 mA up to *maxloadcurrent*. It stops automatically if the voltage at TEST-IN drops below *minloadvoltage*. It continuously transmits the measured values via the serial interface in the format: current[mA] voltage[mV] efficiency[% * 10] (seperated by the SEPERATOR string).

![MP2307_efficiency_x.png](https://image.easyeda.com/pullimage/FN0OcwGXq2hCBlosAKOcNmqud5f5xymeJSkNYGTg.png)

# **Battery Discharge Test** #

![PA_battery.png](https://image.easyeda.com/pullimage/v3cs7mV414xeYIfDpTG1E8ilCKRApmQOLbznXwIh.png)
- Command: "b  *maxloadcurrent[mA: 17..5000]* *minloadvoltage[mV: 0..26000]*"
- Example: "l 1000 2700"
- The Power Analyzer sets a constant current load of *maxloadcurrent*. If the voltage drops below *minloadvoltage* it constantly decreases the load to maintain *minloadvoltage*. It stops automatically if the load current drops to 0mA. It continuously transmits the measured values via the serial interface in the format: time[s] current[mA] voltage[mV] capacity[mAh] (seperated by the SEPERATOR string).

![discharge_x.png](https://image.easyeda.com/pullimage/yhJ7WRHxEhoV2eTCi7wuxJK72kx3jNUx8ILJdYs8.png)

# **Long-Term Multimeter** #

![PA_multimeter.png](https://image.easyeda.com/pullimage/TJGxqkKJv2RRmLWgRUwXsji4feI53ui6k2R3vqqI.png)
- Command: "m  *interval[ms: 2..65535]* *duration[s: 1..65535]*"
- Example: "m 18000 18000"
- The Power Analyzer measures voltage, current and power delivered to the test device at every *interval* for a total of *duration*. It continuously transmits the measured values via the serial interface in the format: time[ms] current[mA] voltage[mV] (seperated by the SEPERATOR string).

![TP4056_charge_x.png](https://image.easyeda.com/pullimage/UNkHfqfx3ETdvMpiQEETEzMi013vV7Ha1eeUnqdU.png)

# **Commands for Direct Control** #

|Command|Function|
|-|-|
|"i"|transmits indentification string ("Power Analyzer")|
|"v"|transmits firmware version number|
|"x"|terminate current test program|
|"s *loadcurrent[mA]*"|set load to a constant current of *loadcurrent*|
|"r"|reset the load to minimum|
|"t"|read current and voltage of both sensors and transmit them|

# 4. Notes #

- Use a good heatsink with a 5V fan for the MOSFET! Attach a 10K 3950B NTC thermistor to the heatsink close to the MOSFET!
- Be careful with high power loads! Make some tests to figure out what can be achieved with your cooling solution!
- Due to the limitations of the cheap OpAmp the minimum load current is around 17mA. You can choose a better OpAmp if you like (must have same pinout, must be rail-to-rail and unity gain stable), but for most cases this is not necessary.
- The maximum load current is 5A, however for small voltages it might be less.
- The maximum PWR-IN/PWR-OUT current is 8A.
- Do not exceed the maximum voltage of 26V on all connectors !
- In order to make the design much simpler all connectors including USB share a common ground. Keep this in mind when making your test setup in order to avoid ground loops or shorts. Using a USB isolator between the Analyzer and your PC is not a bad idea!
- Windows users may need to install a driver: http://www.wch.cn/download/CH341SER_ZIP.html. This is not necessary for linux users.
- You need a UPDI programmer for uploading the firmware. You can find one in my projects (https://easyeda.com/wagiminator/y-updi-programmer) or you can use an Arduino as a jtag2updi. Further information can be found at https://github.com/SpenceKonde/megaTinyCore.
- The Python skript was only tested on Linux, but it should also work on other operating systems.
- The software is still in development. There might be some minor bugs. New functions and test algorithms will be added in the future.
