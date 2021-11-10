#!/usr/bin/env python3

# ===================================================================================
# Project:   Power Analyzer - analyzergui
# Version:   1.0
# Year:      2020
# Author:    Stefan Wagner
# Github:    https://github.com/wagiminator
# License:   http://creativecommons.org/licenses/by-sa/3.0/
# ===================================================================================
#
# Description:
# ------------
# A simple GUI based on tkinter to interface with the Power Analyzer,
# collect and analyze data.
#
# Skript is still in development, not all features are implemented yet.
# There might still be some bugs.
#
# pySerial and tkinter (8.6 or newer) need to be installed

import os
from tkinter        import *
from tkinter        import messagebox, filedialog
from tkinter.ttk    import *
from PIL            import Image
from serial         import Serial
from serial.tools.list_ports import comports

CANX = 50
CANY = 600
CANW = 1000
CANH = 500


class Analyzer(Serial):
    def __init__(self):
        super().__init__(baudrate = 115200, timeout = 1, write_timeout = 1)
        self.identify()

    def identify(self):
        pid = '1a86'
        hid = '7523'
        did = 'Power Analyzer'
        for p in comports():
            if pid and hid in p.hwid:
                self.port = p.device

                try:
                    self.open()
                except:
                    continue

                try:
                    self.sendcommand ('i')
                    data = self.getline()
                except:
                    self.close()
                    continue

                if data == did:
                    break
                else:
                    self.close()


    def sendcommand(self, cmd, argument1=0, argument2=0):
        cmd += ' {} {}'
        self.write((cmd.format(argument1, argument2) + '\n').encode())


    def getline(self):
        return self.readline().decode().rstrip('\r\n')


    def confirmation(self):
        return (self.getline() == 'DONE')


    def getversion(self):
        self.sendcommand ('v')
        version = self.getline()
        return version


def getParameters(fields, values):
    parameterWindow = Toplevel(mainWindow)
    parameterWindow.title('Enter Parameters')
    parameterWindow.resizable(width=False, height=False)
    parameterWindow.transient(mainWindow)
    parameterWindow.grab_set()
    entries = []
    for i in range(0, len(fields)):
        Label(parameterWindow, text=fields[i]).grid(row=i, sticky=W, padx=4, pady=4)
        ent = Entry(parameterWindow)
        ent.insert(10,values[i])
        ent.grid(row=i, column=1, padx=4)
        entries.append(ent)
    Button(parameterWindow, text='OK', command=parameterWindow.quit).grid(
                            row=len(fields), column=0, sticky=W, padx=4, pady=4)
    parameterWindow.mainloop()

    fetches = []
    for ent in entries:
        fetches.append(ent.get())
    parameterWindow.destroy()
    return fetches


def setupCanvas(canvas):
    canvas.create_line(CANX, CANY - CANH - 51, CANX, CANY + 5, fill='black', width=2)
    canvas.create_line(CANX - 5, CANY, CANX + CANW + 40, CANY, fill='black', width=2)
    canvas.create_line(CANX + CANW, CANY - CANH - 51, CANX + CANW, CANY + 5, fill='black', width=2)
    canvas.create_line(CANX - 1, CANY - CANH - 50, CANX + CANW + 1, CANY - CANH - 50, fill='black', width=2)

    # draw vertical lines
    for x in range(1, 10):
        x1 = CANX + CANW / 10 * x
        canvas.create_line( x1, CANY - CANH - 49, x1, CANY - 1, fill='grey', width=1)

    # draw horizontal lines
    for y in range(1, 11):
        y1 = CANY - CANH / 10 * y
        canvas.create_line( CANX + 1, y1, CANX + CANW - 1, y1, fill='grey', width=1)


def saveCanvas(canvas,fileName):
    canvas.postscript(file = fileName + '.eps', pagewidth = 4400) 
    img = Image.open(fileName + '.eps') 
    img.save(fileName + '.png', 'png')


def voltageCalibration():
    pass

def currentCalibration():
    pass


def testLoad():
    # establish connection to Power Analyzer
    analyzer = Analyzer()
    if not analyzer.is_open:
        messagebox.showerror('Error', 'Power Analyzer not found !')
        return

    # get and set parameters
    fields = 'Max Load Current [mA]: ', 'Min Load Voltage [mV]: ', 'Diagram Label: '
    values = '1000', '2900', ''
    values = getParameters(fields, values)
    maxcurrent = int(values[0])
    minvoltage = int(values[1])
    labelstring = values[2]
    cmdmaxcurrent = maxcurrent
    maxcurrent = maxcurrent + 9 // 10 * 10

    # get filename for save files
    fileName = filedialog.asksaveasfilename(title = "Select output file (without file extension)")
    if not fileName:
        return
    try:
        file = open(fileName + '.csv', 'w')
    except:
        messagebox.showerror('Error', 'Could not open file !')
        analyzer.close()
        return

    # create graph window
    contentWindow = Toplevel(mainWindow)
    contentWindow.title('Power Analyzer - Load Test')
    contentWindow.resizable(width=False, height=False)
    contentWindow.transient(mainWindow)
    contentWindow.grab_set()

    # create canvas and coordinate system
    canvas = Canvas(contentWindow, width=1100, height=660)
    canvas.pack()
    setupCanvas(canvas)
    canvas.create_text(CANX - 8, CANY - CANH - 50, text='U [V]', fill='red', anchor='ne', font=('Helvetica',12))
    canvas.create_text(CANX + CANW + 8, CANY - CANH - 50, text='P [W]', fill='blue', anchor='nw', font=('Helvetica',12))
    canvas.create_text(CANX + CANW + 20, CANY + 11, text='I [mA]', font=('Helvetica',12))
    canvas.create_text(10, 25, text='Power Analyzer: Load Test', anchor='w', font=('Helvetica',16,'bold'))
    canvas.create_text(1090, 25, text=labelstring, anchor='e', font=('Helvetica',16,'bold'))

    # label current lines
    for x in range(1, 10):
        x1 = CANX + CANW / 10 * x
        canvas.create_text( x1, CANY + 12, text=str(x * maxcurrent // 10), font=('Helvetica',12))

    # send command to power analyzer
    analyzer.sendcommand('l', cmdmaxcurrent, minvoltage)

    # get first values
    valuestring = analyzer.getline()
    if valuestring != 'DONE':
        file.write(valuestring + '\n')
        valuelist      = valuestring.split('\t')
        lastcurrent    = int(valuelist[0])
        lastvoltage    = int(valuelist[1])
        lastpower      = int(valuelist[2])
        maxvoltage     = (lastvoltage // 5300 + 1) * 5000
        maxpower       = (lastvoltage * maxcurrent // 530000 + 1) * 500
        pmaxpower      = lastpower

        # label voltage lines
        for y in range(1, 6):
            y1 = CANY - CANH / 5 * y
            canvas.create_text(CANX - 8, y1, text=str(y * maxvoltage // 5000), fill='red', anchor='e', font=('Helvetica',12))
            canvas.create_text(CANX + CANW + 8, y1, text=str(y * maxpower / 5000), fill='blue', anchor='w', font=('Helvetica',12))

        # get the rest of the values and draw the graphs
        valuestring = analyzer.getline()
        while(valuestring != 'DONE'):
            file.write(valuestring + '\n')
            valuelist  = valuestring.split('\t')
            current    = int(valuelist[0])
            voltage    = int(valuelist[1])
            power      = int(valuelist[2])
            if power > pmaxpower: pmaxpower = power
            x1 = CANX + CANW / maxcurrent * lastcurrent
            x2 = CANX + CANW / maxcurrent * current
            y1 = CANY - CANH / maxvoltage * lastvoltage
            y2 = CANY - CANH / maxvoltage * voltage
            y3 = CANY - CANH / maxpower * lastpower
            y4 = CANY - CANH / maxpower * power
            canvas.create_line(x1, y1, x2, y2, fill='red', width=2)
            canvas.create_line(x1, y3, x2, y4, fill='blue', width=2)
            contentWindow.update_idletasks()
            lastcurrent    = current
            lastvoltage    = voltage
            lastpower      = power
            valuestring = analyzer.getline()

        # draw maxpower
        t1 = 'Max Power: '   + str(pmaxpower)+ ' mW'
        canvas.create_text(CANX + CANW // 2, CANY + 40, text=t1, font=('TkFixedFont',12,'bold'))
        contentWindow.update_idletasks()

    # save and close everything
    file.close()
    analyzer.close()
    saveCanvas(canvas, fileName)
    contentWindow.mainloop()
    contentWindow.quit()


def testRegulation():
    # establish connection to Power Analyzer
    analyzer = Analyzer()
    if not analyzer.is_open:
        messagebox.showerror('Error', 'Power Analyzer not found !')
        return

    # get and set parameters
    fields = 'Max Load Current [mA]: ', 'Diagram Label: '
    values = '1000', ''
    values = getParameters(fields, values)
    maxcurrent = int(values[0])
    labelstring = values[1]
    cmdmaxcurrent = maxcurrent
    maxcurrent = (maxcurrent // 530  + 1) * 500

    # get filename for save files
    fileName = filedialog.asksaveasfilename(title = "Select output file (without file extension)")
    if not fileName:
        return
    try:
        file = open(fileName + '.csv', 'w')
    except:
        messagebox.showerror('Error', 'Could not open file !')
        analyzer.close()
        return

    # create graph window
    contentWindow = Toplevel(mainWindow)
    contentWindow.title('Power Analyzer - Voltage Regulation Test')
    contentWindow.resizable(width=False, height=False)
    contentWindow.transient(mainWindow)
    contentWindow.grab_set()

    # create canvas and coordinate system
    canvas = Canvas(contentWindow, width=1100, height=640)
    canvas.pack()
    setupCanvas(canvas)
    canvas.create_text(CANX - 8, CANY - CANH - 50, text='U [V]', fill='red', anchor='ne', font=('Helvetica',12))
    canvas.create_text(CANX + CANW + 8, CANY - CANH - 50, text='I [mA]', fill='blue', anchor='nw', font=('Helvetica',12))
    canvas.create_text(CANX + CANW + 20, CANY + 11, text='t [ms]', font=('Helvetica',12))
    canvas.create_text(10, 25, text='Power Analyzer: Voltage Regulation Test', anchor='w', font=('Helvetica',16,'bold'))
    canvas.create_text(1090, 25, text=labelstring, anchor='e', font=('Helvetica',16,'bold'))

    # label time lines
    for x in range(1, 10):
        x1 = CANX + CANW / 10 * x
        canvas.create_text( x1, CANY + 12, text=str(x * 100), font=('Helvetica',12))

    # send command to power analyzer
    analyzer.sendcommand('g', cmdmaxcurrent)

    # get first values
    valuestring = analyzer.getline()
    if valuestring != 'DONE':
        file.write(valuestring + '\n')
        valuelist      = valuestring.split('\t')
        lasttime       = int(valuelist[0])
        lastcurrent    = int(valuelist[1])
        lastvoltage    = int(valuelist[2])
        maxvoltage     = (lastvoltage // 5300 + 1) * 5000

        # label voltage lines
        for y in range(1, 6):
            y1 = CANY - CANH / 5 * y
            canvas.create_text(CANX - 8, y1, text=str(y * maxvoltage // 5000), fill='red', anchor='e', font=('Helvetica',12))
            canvas.create_text(CANX + CANW + 8, y1, text=str(y * maxcurrent // 5), fill='blue', anchor='w', font=('Helvetica',12))

        # get the rest of the values and draw the graphs
        valuestring = analyzer.getline()
        while(valuestring != 'DONE'):
            file.write(valuestring + '\n')
            valuelist  = valuestring.split('\t')
            time       = int(valuelist[0])
            current    = int(valuelist[1])
            voltage    = int(valuelist[2])
            x1 = CANX + CANW / 1000 * lasttime
            x2 = CANX + CANW / 1000 * time
            y1 = CANY - CANH / maxvoltage * lastvoltage
            y2 = CANY - CANH / maxvoltage * voltage
            y3 = CANY - CANH / maxcurrent * lastcurrent
            y4 = CANY - CANH / maxcurrent * current
            canvas.create_line(x1, y1, x2, y2, fill='red', width=2)
            canvas.create_line(x1, y3, x2, y4, fill='blue', width=2)
            contentWindow.update_idletasks()
            lastcurrent    = current
            lastvoltage    = voltage
            lasttime       = time
            valuestring = analyzer.getline()

    # save and close everything
    file.close()
    analyzer.close()
    saveCanvas(canvas, fileName)
    contentWindow.mainloop()
    contentWindow.quit()


def testEfficiency():
    # establish connection to Power Analyzer
    analyzer = Analyzer()
    if not analyzer.is_open:
        messagebox.showerror('Error', 'Power Analyzer not found !')
        return

    # get and set parameters
    fields = 'Max Load Current [mA]: ', 'Min Load Voltage [mV]: ', 'Diagram Label: ', 'Input Label: '
    values = '1000', '2900', '', ''
    values = getParameters(fields, values)
    maxcurrent = int(values[0])
    minvoltage = int(values[1])
    labelstring = values[2]
    inputstring = values[3]
    cmdmaxcurrent = maxcurrent
    maxcurrent = maxcurrent + 9 // 10 * 10

    # get filename for save files
    fileName = filedialog.asksaveasfilename(title = "Select output file (without file extension)")
    if not fileName:
        return
    try:
        file = open(fileName + '.csv', 'w')
    except:
        messagebox.showerror('Error', 'Could not open file !')
        analyzer.close()
        return

    # create graph window
    contentWindow = Toplevel(mainWindow)
    contentWindow.title('Power Analyzer - Efficiency Test')
    contentWindow.resizable(width=False, height=False)
    contentWindow.transient(mainWindow)
    contentWindow.grab_set()

    # create canvas and coordinate system
    canvas = Canvas(contentWindow, width=1100, height=660)
    canvas.pack()
    setupCanvas(canvas)
    canvas.create_text(CANX - 8, CANY - CANH - 50, text='U [V]', fill='red', anchor='ne', font=('Helvetica',12))
    canvas.create_text(CANX + CANW + 8, CANY - CANH - 50, text='Eff[%]', fill='blue', anchor='nw', font=('Helvetica',12))
    canvas.create_text(CANX + CANW + 20, CANY + 11, text='I [mA]', font=('Helvetica',12))
    canvas.create_text(10, 25, text='Power Analyzer: Efficiency Test', anchor='w', font=('Helvetica',16,'bold'))
    canvas.create_text(1090, 25, text=labelstring, anchor='e', font=('Helvetica',16,'bold'))

    # label current lines
    for x in range(1, 10):
        x1 = CANX + CANW / 10 * x
        canvas.create_text( x1, CANY + 12, text=str(x * maxcurrent // 10), font=('Helvetica',12))

    # send command to power analyzer
    analyzer.sendcommand('e', cmdmaxcurrent, minvoltage)

    # get first values
    valuestring = analyzer.getline()
    if valuestring != 'DONE':
        file.write(valuestring + '\n')
        valuelist      = valuestring.split('\t')
        lastcurrent    = int(valuelist[0])
        lastvoltage    = int(valuelist[1])
        lastefficiency = int(valuelist[2])
        maxvoltage     = (lastvoltage // 5300 + 1) * 5000
        pmaxpower      = lastvoltage * lastcurrent // 1000
        pmaxefficiency = lastefficiency

        # label voltage lines
        for y in range(1, 6):
            y1 = CANY - CANH / 5 * y
            canvas.create_text(CANX - 8, y1, text=str(y * maxvoltage // 5000), fill='red', anchor='e', font=('Helvetica',12))
            canvas.create_text(CANX + CANW + 8, y1, text=str(y * 100 // 5), fill='blue', anchor='w', font=('Helvetica',12))

        # get the rest of the values and draw the graphs
        valuestring = analyzer.getline()
        while(valuestring != 'DONE'):
            file.write(valuestring + '\n')
            valuelist  = valuestring.split('\t')
            current    = int(valuelist[0])
            voltage    = int(valuelist[1])
            efficiency = int(valuelist[2])
            power      = voltage * current // 1000
            if power > pmaxpower: pmaxpower = power
            if efficiency > pmaxefficiency: pmaxefficiency = efficiency
            x1 = CANX + CANW / maxcurrent * lastcurrent
            x2 = CANX + CANW / maxcurrent * current
            y1 = CANY - CANH / maxvoltage * lastvoltage
            y2 = CANY - CANH / maxvoltage * voltage
            y3 = CANY - CANH / 1000 * lastefficiency
            y4 = CANY - CANH / 1000 * efficiency
            canvas.create_line(x1, y1, x2, y2, fill='red', width=2)
            canvas.create_line(x1, y3, x2, y4, fill='blue', width=2)
            contentWindow.update_idletasks()
            lastcurrent    = current
            lastvoltage    = voltage
            lastefficiency = efficiency
            valuestring = analyzer.getline()

        # draw maxpower and maxefficiency
        t1 = 'Input: ' + inputstring
        t2 = 'Max Power: ' + str(pmaxpower) + ' mW'
        t3 = 'Max Efficiency: ' + str(pmaxefficiency / 10) + ' %'
        canvas.create_text(CANX, CANY + 40, text=t1, anchor='w', font=('TkFixedFont',12,'bold'))
        canvas.create_text(CANX + CANW // 2, CANY + 40, text=t2, font=('TkFixedFont',12,'bold'))
        canvas.create_text(CANX + CANW, CANY + 40, text=t3, anchor='e', font=('TkFixedFont',12,'bold'))
        contentWindow.update_idletasks()

    # save and close everything
    file.close()
    analyzer.close()
    saveCanvas(canvas, fileName)
    contentWindow.mainloop()
    contentWindow.quit()


def testRipple():
    # establish connection to Power Analyzer
    analyzer = Analyzer()
    if not analyzer.is_open:
        messagebox.showerror('Error', 'Power Analyzer not found !')
        return

    # get and set parameters
    fields = 'Max Load Current [mA]: ', 'Min Load Voltage [mV]: ', 'Diagram Label: '
    values = '1000', '2900', ''
    values = getParameters(fields, values)
    maxcurrent = int(values[0])
    minvoltage = int(values[1])
    labelstring = values[2]
    cmdmaxcurrent = maxcurrent
    maxcurrent = maxcurrent + 9 // 10 * 10

    # get filename for save files
    fileName = filedialog.asksaveasfilename(title = "Select output file (without file extension)")
    if not fileName:
        return
    try:
        file = open(fileName + '.csv', 'w')
    except:
        messagebox.showerror('Error', 'Could not open file !')
        analyzer.close()
        return

    # create graph window
    contentWindow = Toplevel(mainWindow)
    contentWindow.title('Power Analyzer - Low Frequency Ripple Test')
    contentWindow.resizable(width=False, height=False)
    contentWindow.transient(mainWindow)
    contentWindow.grab_set()

    # create canvas and coordinate system
    canvas = Canvas(contentWindow, width=1100, height=640)
    canvas.pack()
    setupCanvas(canvas)
    canvas.create_text(CANX - 8, CANY - CANH - 55, text='U(pp)', fill='red', anchor='ne', font=('Helvetica',12))
    canvas.create_text(CANX - 8, CANY - CANH - 35, text='[mV]', fill='red', anchor='ne', font=('Helvetica',12))
    canvas.create_text(CANX + CANW + 20, CANY + 11, text='I [mA]', font=('Helvetica',12))
    canvas.create_text(10, 25, text='Power Analyzer: Low Frequency Ripple Test', anchor='w', font=('Helvetica',16,'bold'))
    canvas.create_text(1090, 25, text=labelstring, anchor='e', font=('Helvetica',16,'bold'))

    # label lines
    for x in range(1, 10):
        x1 = CANX + CANW / 10 * x
        canvas.create_text( x1, CANY + 12, text=str(x * maxcurrent // 10), font=('Helvetica',12))
    for y in range(1, 6):
        y1 = CANY - CANH / 5 * y
        canvas.create_text(CANX - 8, y1, text=str(y * 50), fill='red', anchor='e', font=('Helvetica',12))

    # send command to power analyzer
    analyzer.sendcommand('f', cmdmaxcurrent, minvoltage)

    # get first values
    valuestring = analyzer.getline()
    if valuestring != 'DONE':
        file.write(valuestring + '\n')
        valuelist      = valuestring.split('\t')
        lastcurrent    = int(valuelist[0])
        lastvoltage    = int(valuelist[1])

        # get the rest of the values and draw the graphs
        valuestring = analyzer.getline()
        while(valuestring != 'DONE'):
            file.write(valuestring + '\n')
            valuelist  = valuestring.split('\t')
            current    = int(valuelist[0])
            voltage    = int(valuelist[1])
            x1 = CANX + CANW / maxcurrent * lastcurrent
            x2 = CANX + CANW / maxcurrent * current
            y1 = CANY - CANH / 250 * lastvoltage
            y2 = CANY - CANH / 250 * voltage
            canvas.create_line(x1, y1, x2, y2, fill='red', width=2)
            contentWindow.update_idletasks()
            lastcurrent    = current
            lastvoltage    = voltage
            valuestring = analyzer.getline()

    # save and close everything
    file.close()
    analyzer.close()
    saveCanvas(canvas, fileName)
    contentWindow.mainloop()
    contentWindow.quit()


def testBattery():
    # establish connection to Power Analyzer
    analyzer = Analyzer()
    if not analyzer.is_open:
        messagebox.showerror('Error', 'Power Analyzer not found !')
        return

    # get and set parameters
    fields = 'Max Load Current [mA]: ', 'Min Battery Voltage [mV]: ', 'Max expected Capacity [mAh]: ', 'Diagram Label: '
    values = '1000', '2900', '3000', ''
    values = getParameters(fields, values)
    maxcurrent  = int(values[0])
    minvoltage  = int(values[1])
    maxcapacity = int(values[2])
    labelstring = values[3]
    maxtime     = (maxcapacity * 10) // maxcurrent * 600

    # get filename for save files
    fileName = filedialog.asksaveasfilename(title = "Select output file (without file extension)")
    if not fileName:
        return
    try:
        file = open(fileName + '.csv', 'w')
    except:
        messagebox.showerror('Error', 'Could not open file !')
        analyzer.close()
        return

    # create graph window
    contentWindow = Toplevel(mainWindow)
    contentWindow.title('Power Analyzer - Battery Discharge Test')
    contentWindow.resizable(width=False, height=False)
    contentWindow.transient(mainWindow)
    contentWindow.grab_set()

    # create canvas and coordinate system
    canvas = Canvas(contentWindow, width=1100, height=660)
    canvas.pack()
    setupCanvas(canvas)
    canvas.create_text(CANX - 8, CANY - CANH - 50, text='U [V]', fill='red', anchor='ne', font=('Helvetica',12))
    canvas.create_text(CANX + CANW + 8, CANY - CANH - 50, text='I [mA]', fill='blue', anchor='nw', font=('Helvetica',12))
    canvas.create_text(CANX + CANW + 20, CANY + 11, text='t [min]', font=('Helvetica',12))
    canvas.create_text(10, 25, text='Power Analyzer: Battery Discharge Test', anchor='w', font=('Helvetica',16,'bold'))
    canvas.create_text(1090, 25, text=labelstring, anchor='e', font=('Helvetica',16,'bold'))

    # label time lines
    for x in range(1, 10):
        x1 = CANX + CANW / 10 * x
        canvas.create_text( x1, CANY + 12, text=str(x * maxtime // 600), font=('Helvetica',12))

    # send command to power analyzer
    analyzer.sendcommand('b', maxcurrent, minvoltage)

    # get first values
    valuestring = analyzer.getline()
    if valuestring != 'DONE':
        file.write(valuestring + '\n')
        valuelist      = valuestring.split('\t')
        lasttime       = int(valuelist[0])
        lastcurrent    = int(valuelist[1])
        lastvoltage    = int(valuelist[2])
        lastcapacity   = int(valuelist[3])
        maxvoltage     = lastvoltage

        # label voltage and current lines
        maxvoltage = (maxvoltage // 5300 + 1) * 5000
        maxcurrent = (maxcurrent // 530  + 1) * 500
        for y in range(1, 6):
            y1 = CANY - CANH / 5 * y
            canvas.create_text(CANX - 8, y1, text=str(y * maxvoltage // 5000), fill='red', anchor='e', font=('Helvetica',12))
            canvas.create_text(CANX + CANW + 8, y1, text=str(y * maxcurrent // 5), fill='blue', anchor='w', font=('Helvetica',12))

        # write values on canvas
        t1 = 'Capacity: ' + str(lastcapacity) + ' mAh'
        valuetext = canvas.create_text(CANX + CANW // 2, CANY + 40, text=t1, font=('TkFixedFont',12,'bold'))
        contentWindow.update_idletasks()

        # get the rest of the values and draw the graphs
        valuestring = analyzer.getline()
        while(valuestring != 'DONE'):
            file.write(valuestring + '\n')
            valuelist  = valuestring.split('\t')
            time    = int(valuelist[0])
            current = int(valuelist[1])
            voltage = int(valuelist[2])
            capacity= int(valuelist[3])

            # draw the graph
            x1 = CANX + CANW / maxtime * lasttime
            x2 = CANX + CANW / maxtime * time
            y1 = CANY - CANH / maxvoltage * lastvoltage
            y2 = CANY - CANH / maxvoltage * voltage
            y3 = CANY - CANH / maxcurrent * lastcurrent
            y4 = CANY - CANH / maxcurrent * current
            canvas.create_line(x1, y1, x2, y2, fill='red', width=2)
            canvas.create_line(x1, y3, x2, y4, fill='blue', width=2)

            # write values on canvas
            t1 = 'Capacity: ' + str(capacity) + ' mAh'
            canvas.delete(valuetext)
            valuetext = canvas.create_text(CANX + CANW // 2, CANY + 40, text=t1, font=('TkFixedFont',12,'bold'))

            # prepare next cycle
            contentWindow.update_idletasks()
            lasttime       = time
            lastcurrent    = current
            lastvoltage    = voltage
            lastcapacity   = capacity
            valuestring = analyzer.getline()

    # save and close everything
    file.close()
    analyzer.close()
    saveCanvas(canvas, fileName)
    contentWindow.mainloop()
    contentWindow.quit()


def multimeter():
    # updates values on canvas
    global valuelabel
    def updateValues():
        global valuelabel
        t1 = 'Min Voltage: ' + str(pminvolt) + ' mV'
        t2 = 'Max Voltage: ' + str(pmaxvolt) + ' mV'
        t3 = 'Min Current: ' + str(pmincurr) + ' mA'
        t4 = 'Max Current: ' + str(pmaxcurr) + ' mA'
        t5 = 'Max Power: '   + str(pmaxpower)+ ' mW'
        t6 = 'Avg Power: '   + str(pavgpower)+ ' mW'
        t7 = 'Energy:   '    + str(int(energy * 100) / 100) + ' mWh'
        t8 = 'Capacity: '    + str(int(capacity * 100) / 100) + ' mAh'
        for valuetext in valuelabel: canvas.delete(valuetext)
        valuelabel = []
        valuelabel.append(canvas.create_text(CANX      , CANY + 40, text=t1, anchor='w', font=('TkFixedFont',12,'bold')))
        valuelabel.append(canvas.create_text(CANX      , CANY + 60, text=t2, anchor='w', font=('TkFixedFont',12,'bold')))
        valuelabel.append(canvas.create_text(CANX + 250, CANY + 40, text=t3, anchor='w', font=('TkFixedFont',12,'bold')))
        valuelabel.append(canvas.create_text(CANX + 250, CANY + 60, text=t4, anchor='w', font=('TkFixedFont',12,'bold')))
        valuelabel.append(canvas.create_text(CANX + 500, CANY + 40, text=t5, anchor='w', font=('TkFixedFont',12,'bold')))
        valuelabel.append(canvas.create_text(CANX + 500, CANY + 60, text=t6, anchor='w', font=('TkFixedFont',12,'bold')))
        valuelabel.append(canvas.create_text(CANX + 750, CANY + 40, text=t7, anchor='w', font=('TkFixedFont',12,'bold')))
        valuelabel.append(canvas.create_text(CANX + 750, CANY + 60, text=t8, anchor='w', font=('TkFixedFont',12,'bold')))

    # establish connection to Power Analyzer
    analyzer = Analyzer()
    if not analyzer.is_open:
        messagebox.showerror('Error', 'Power Analyzer not found !')
        return

    # get and set parameters
    fields = 'Interval [ms]: ', 'Duration [s]: ', 'Max expected Current [mA]: ', 'Max expected Voltage [mV]:', 'Diagram Label: '
    values = '100', '20', '1000', '5000', ''
    values = getParameters(fields, values)
    interval = int(values[0])
    duration = int(values[1])
    maxcurrent = int(values[2])
    maxvoltage = int(values[3])
    labelstring = values[4]

    # get filename for save files
    fileName = filedialog.asksaveasfilename(title = "Select output file (without file extension)")
    if not fileName:
        return
    try:
        file = open(fileName + '.csv', 'w')
    except:
        messagebox.showerror('Error', 'Could not open file !')
        analyzer.close()
        return

    # create graph window
    contentWindow = Toplevel(mainWindow)
    contentWindow.title('Power Analyzer - Multimeter')
    contentWindow.resizable(width=False, height=False)
    contentWindow.transient(mainWindow)
    contentWindow.grab_set()

    # create canvas and coordinate system
    canvas = Canvas(contentWindow, width=1100, height=680)
    canvas.pack()
    setupCanvas(canvas)
    canvas.create_text(CANX - 8, CANY - CANH - 50, text='U [V]', fill='red', anchor='ne', font=('Helvetica',12))
    canvas.create_text(CANX + CANW + 8, CANY - CANH - 50, text='I [mA]', fill='blue', anchor='nw', font=('Helvetica',12))
    if duration < 600:
        canvas.create_text(CANX + CANW + 20, CANY + 11, text='t [s]', font=('Helvetica',12))
    else:
        canvas.create_text(CANX + CANW + 20, CANY + 11, text='t [min]', font=('Helvetica',12))
    canvas.create_text(10, 25, text='Power Analyzer: Long-Term Multimeter', anchor='w', font=('Helvetica',16,'bold'))
    canvas.create_text(1090, 25, text=labelstring, anchor='e', font=('Helvetica',16,'bold'))

    # label time lines
    if   duration >= 6000: duration = (duration + 599) // 600 * 600
    elif duration >= 600:  duration = (duration +  59) // 60 * 60
    for x in range(1, 10):
        x1 = CANX + CANW / 10 * x
        if duration < 600:
            canvas.create_text( x1, CANY + 12, text=str(x * duration / 10), font=('Helvetica',12))
        elif duration < 6000:
            canvas.create_text( x1, CANY + 12, text=str(x * duration / 600), font=('Helvetica',12))
        else:
            canvas.create_text( x1, CANY + 12, text=str(x * duration // 600), font=('Helvetica',12))

    # label voltage and current lines
    maxvoltage = (maxvoltage // 5300 + 1) * 5000
    maxcurrent = (maxcurrent // 530  + 1) * 500
    for y in range(1, 6):
        y1 = CANY - CANH / 5 * y
        canvas.create_text(CANX - 8, y1, text=str(y * maxvoltage // 5000), fill='red', anchor='e', font=('Helvetica',12))
        canvas.create_text(CANX + CANW + 8, y1, text=str(y * maxcurrent // 5), fill='blue', anchor='w', font=('Helvetica',12))
    contentWindow.update_idletasks()

    # send command to power analyzer
    if interval > 800: analyzer.timeout = interval // 1000 + 2
    analyzer.sendcommand('m', interval, duration)

    # get first values
    valuestring = analyzer.getline()
    if valuestring != 'DONE':
        file.write(valuestring + '\n')
        valuelist      = valuestring.split('\t')
        lasttime       = int(valuelist[0])
        lastcurrent    = int(valuelist[1])
        lastvoltage    = int(valuelist[2])
        lastpower      = lastvoltage * lastcurrent // 1000
        pmaxpower      = lastpower
        pavgpower      = lastpower
        pmaxvolt       = lastvoltage
        pminvolt       = lastvoltage
        pmaxcurr       = lastcurrent
        pmincurr       = lastcurrent
        energy         = 0
        capacity       = 0
        nextupdate     = lasttime + 1000

        # write values on canvas
        valuelabel = []
        updateValues()

        # get the rest of the values and draw the graphs
        valuestring = analyzer.getline()
        while(valuestring != 'DONE'):
            file.write(valuestring + '\n')
            valuelist  = valuestring.split('\t')
            time    = int(valuelist[0])
            current = int(valuelist[1])
            voltage = int(valuelist[2])
            power   = voltage * current // 1000

            # draw the graph
            x1 = CANX + CANW / duration * lasttime / 1000
            x2 = CANX + CANW / duration * time / 1000
            y1 = CANY - CANH / maxvoltage * lastvoltage
            y2 = CANY - CANH / maxvoltage * voltage
            y3 = CANY - CANH / maxcurrent * lastcurrent
            y4 = CANY - CANH / maxcurrent * current
            canvas.create_line(x1, y1, x2, y2, fill='red', width=2)
            canvas.create_line(x1, y3, x2, y4, fill='blue', width=2)

            # some calculations
            interval  = time - lasttime
            energy   += power * interval / 3600000
            capacity += current * interval / 3600000
            pavgpower = int (energy * 3600000 / time)
            if voltage > pmaxvolt: pmaxvolt = voltage
            if voltage < pminvolt: pminvolt = voltage
            if current > pmaxcurr: pmaxcurr = current
            if current < pmincurr: pmincurr = current
            if power   > pmaxpower: pmaxpower = power

            # update values on screen every second
            if time > nextupdate:
                updateValues()
                nextupdate = time + 1000

            # prepare next cycle
            contentWindow.update_idletasks()
            lasttime       = time
            lastcurrent    = current
            lastvoltage    = voltage
            valuestring = analyzer.getline()

        # last value update
        updateValues()
        contentWindow.update_idletasks()

    # save and close everything
    file.close()
    analyzer.close()
    saveCanvas(canvas, fileName)
    contentWindow.mainloop()
    contentWindow.quit()





mainWindow = Tk()
mainWindow.title('Power Analyzer')
mainWindow.resizable(width=False, height=False)


calibFrame = Frame(mainWindow, borderwidth = 2, relief = 'groove')
Label(calibFrame, text = 'Calibration:').pack(pady = 5)
Button(calibFrame, text = 'Calibrate Voltage Sensors', command = voltageCalibration
            ).pack(padx = 10, fill = 'x')
Button(calibFrame, text = 'Calibrate Current Sensors', command = currentCalibration,
            ).pack(padx = 10, fill = 'x')
calibFrame.pack(padx = 10, pady = 10, ipadx = 5, ipady = 5, fill = 'x')

actionFrame = Frame(mainWindow, borderwidth = 2, relief = 'groove')
Label(actionFrame, text = 'Analysis:').pack(pady = 5)
Button(actionFrame, text = 'Load Test', command = testLoad
            ).pack(padx = 10, fill = 'x')
Button(actionFrame, text = 'Voltage Regulation Test', command = testRegulation
            ).pack(padx = 10, fill = 'x')
Button(actionFrame, text = 'Efficiency Test', command = testEfficiency,
            ).pack(padx = 10, fill = 'x')
Button(actionFrame, text = 'Low Frequency Ripple Test', command = testRipple
            ).pack(padx = 10, fill = 'x')
Button(actionFrame, text = 'Battery Discharge Test', command = testBattery,
            ).pack(padx = 10, fill = 'x')
Button(actionFrame, text = 'Long-Term Multimeter', command = multimeter,
            ).pack(padx = 10, fill = 'x')
actionFrame.pack(padx = 10, pady = 10, ipadx = 5, ipady = 5, fill = 'x')

Button(mainWindow, text = 'Exit', command = mainWindow.quit).pack(pady = 10)

mainWindow.mainloop()
