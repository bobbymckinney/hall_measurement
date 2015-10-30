#! /usr/bin/python
# -*- coding: utf-8 -*-
"""
Created: 2015-03-31

@author: Bobby McKinney (bobbymckinney@gmail.com)

__Title__ : voltagepanel
Description:
Comments:
"""
import os
import sys
import wx
from wx.lib.pubsub import pub # For communicating b/w the thread and the GUI
import matplotlib
matplotlib.interactive(False)
matplotlib.use('WXAgg') # The recommended way to use wx with mpl is with WXAgg backend.

from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg
from matplotlib.figure import Figure
from matplotlib.pyplot import gcf, setp
import matplotlib.animation as animation # For plotting
import pylab
import numpy as np
import matplotlib.pyplot as plt
import serial # for communicating with Lakeshore magnet power supply
import visa # pyvisa, essential for communicating with the Keithley
from threading import Thread # For threading the processes going on behind the GUI
import time
from datetime import datetime # for getting the current date and time
# Modules for saving logs of exceptions
import exceptions
import sys
from logging_utils import setup_logging_to_file, log_exception

# for getting magnetic field value
from MagFieldInterpolator import fieldInterp

# for a fancy status bar:
import EnhancedStatusBar as ESB

#==============================================================================
# Keeps Windows from complaining that the port is already open:

version = '1.0 (2015-05-22)'

'''
Global Variables:
'''

# Naming a data file:
dataFile = 'Data_Backup.csv'
finaldataFile = 'Data.csv'
hallFile = 'Hall.csv'

thickness = '0.1' #placeholder for sample thickness in cm
poledist = '30' #placeholder for distance between magnet poles in mm

current = .04 # (A) Current that is sourced by the k2400
magcurrent = 70 # (A) Current that is supplied by the mag power supply
magrate = 30 # (A) Current ramp rate that is supplied to the mag power supply
APP_EXIT = 1 # id for File\Quit
measurement_number = 30 # number of measurements

maxCurrent = .2 # (A) Restricts the user to a max current
maxMagCurrent = 70 # (A) Restricts the user to a max current on mag power supply
maxMagRate = 40 # (A/s) Restricts the user to a max current ramp rate on mag power supply

abort_ID = 0 # Abort method

# Global placers for instruments
k2700 = ''
k2400 = ''
k2182 = ''
ls643 = ''

# placer for directory
filePath = 'global file path'

# placer for files to be created
myfile = 'global file'
processfile = 'global file'

# Placers for the GUI plots:
r_A_list = []
t_A_list = []
r_B_list = []
t_B_list = []
r_P_list = []
t_P_list = []

timecalclist = []
rAcalclist = []
rBcalclist = []
rPcalclist = []
Bcalclist = []


#ResourceManager for visa instrument control
ResourceManager = visa.ResourceManager()

###############################################################################
class Keithley_2700:
    ''' Used for the matrix card operations. '''
    #--------------------------------------------------------------------------
    def __init__(self, instr):
        self.ctrl = ResourceManager.open_resource(instr)

        self.openAllChannels()
    #end init

    #--------------------------------------------------------------------------
    def closeChannels(self, channels):
        self.ctrl.write(":ROUT:MULT:CLOS (@ %s)" %channels)
    #end def

    #--------------------------------------------------------------------------
    def openChannels(self, channels):
        self.ctrl.write(":ROUT:MULT:OPEN (@ %s)" %channels)
    #end def

    #--------------------------------------------------------------------------
    def openAllChannels(self):
        self.ctrl.write("ROUTe:OPEN:ALL")
    #end def

    #--------------------------------------------------------------------------
    def get_closedChannels(self):
        return self.ctrl.query(":ROUT:MULT:CLOS?")
    #end def

#end class
###############################################################################

###############################################################################
class Keithley_2400:
    ''' SourceMeter '''
    #--------------------------------------------------------------------------
    def __init__(self, instr):
        self.ctrl = ResourceManager.open_resource(instr)

        self.ctrl.write(":ROUT:TERM REAR") # Use the rear output terminals
        self.current_mode()
        self.set_current_range(10.5*10**(-3)) # Default
        self.set_current(float(current))
    #end init

    #--------------------------------------------------------------------------
    def turn_source_on(self):
        self.ctrl.write(":OUTPut:STATe ON")
    #end def

    #--------------------------------------------------------------------------
    def turn_source_off(self):
        self.ctrl.write(":OUTPut:STATe OFF")
    #end def

    #--------------------------------------------------------------------------
    def query_state(self):
        state = self.ctrl.query(":OUTPut:STATe?")

        if state == '1':
            amplitude = self.ctrl.query(":SOURce:CURRent:LEVel:IMMediate:AMPLitude?") + ' Amps'

            if amplitude == '0.000000E+00 Amps':
                amplitude = self.ctrl.query(":SOURce:VOLTage:LEVel:IMMediate:AMPLitude?") + ' Volts'

            return 'state: %s, amplitude: %s ' % (state, amplitude)

        else:
            return 'state: %s' % state

    #end def

    #--------------------------------------------------------------------------
    def current_mode(self):
        self.ctrl.write(":SOURce:FUNCtion:MODE CURRent")
        self.ctrl.write(":SOURce:CURRent:MODE FIXed") # Fixed current mode
    #end def

    #--------------------------------------------------------------------------
    def set_current(self, current):
        self.change_current_range(current)
        #time.sleep(5)
        self.ctrl.write(":SOURce:CURRent:LEVel:IMMediate:AMPLitude %f" % current)
    #end def

    #--------------------------------------------------------------------------
    def change_current_range(self, current):
        #self.write(":SOURce:CURRent:LEVel:IMMediate:AMPLitude 0")
        if current > 0:
            if current > 105*10**(-3):
                    self.set_current_range(1.05)
            else:
                if current > 10.5*10**(-3):
                    self.set_current_range(105*10**(-3))
                else:
                    if current > 1.05*10**(-3):
                        self.set_current_range(10.5*10**(-3))
                    else:
                        if current > 105*10**(-6):
                            self.set_current_range(1.05*10**(-3))
                        else:
                            if current > 10.5*10**(-6):
                                self.set_current_range(105*10**(-6))
                            else:
                                if current > 1.05*10**(-6):
                                    self.set_current_range(10.5*10**(-6))
                                else:
                                    self.set_current_range(1.05*10**(-6))

        elif current < 0:
            if current < -105*10**(-3):
                    self.set_current_range(-1.05)
            else:
                if current < -10.5*10**(-3):
                    self.set_current_range(-105*10**(-3))
                else:
                    if current < -1.05*10**(-3):
                        self.set_current_range(-10.5*10**(-3))
                    else:
                        if current < -105*10**(-6):
                            self.set_current_range(-1.05*10**(-3))
                        else:
                            if current < -10.5*10**(-6):
                                self.set_current_range(-105*10**(-6))
                            else:
                                if current < -1.05*10**(-6):
                                    self.set_current_range(-10.5*10**(-6))
                                else:
                                    self.set_current_range(-1.05*10**(-6))

        else:
            self.set_current_range(1.05*10**(-6))

    #end def

    #--------------------------------------------------------------------------
    def set_current_range(self, current):
        self.ctrl.write(":SOURce:CURRent:RANGe %f" % current)

    #end def

    #--------------------------------------------------------------------------
    def voltage_mode(self):
        self.ctrl.write(":SOURce:FUNCtion:MODE VOLTage")
        self.ctrl.write(":SOURce:VOLTage:MODE FIXed") # Fixed voltage mode
    #end def

    #--------------------------------------------------------------------------
    def set_voltage(self, voltage):
        self.ctrl.write(":SOURce:VOLTage:LEVel:IMMediate:AMPLitude %f" % voltage)
    #end def

#end class
###############################################################################

###############################################################################
class Keithley_2182:
    ''' NanoVoltMeter '''
    #--------------------------------------------------------------------------
    def __init__(self, instr):
        self.ctrl = ResourceManager.open_resource(instr)

        self.ctrl.write(":TRIGger:SEQuence1:COUNt 1")
        self.ctrl.write(":TRIGger:SEQuence1:DELay 0") # Set count rate
        self.ctrl.write(":SENSe:FUNCtion VOLTage")
        self.ctrl.write(":SENS:VOLT:CHAN1:RANG:AUTO ON")
        self.ctrl.write(":SENSe1:VOLTage:DC:NPLCycles 5") # Sets integration period based on frequency
    #end init

    #--------------------------------------------------------------------------
    def fetch(self):
        """
        Scan the channel and take a reading
        """
        #self.write(":ROUTe:SCAN:INTernal:CCOunt 1") # Specify number of readings on channel 1
        self.ctrl.write(":SENSe:CHANnel 1")
        data = self.ctrl.query(":SENSe:DATA:FRESh?")
        #print str(data)[0:15]
        #print data
        return str(data)[0:15] # Fetches Reading

    #end def

#end class
###############################################################################

###############################################################################
class Lakeshore_643(serial.Serial):
    '''Electromagnet Power Supply'''
    #--------------------------------------------------------------------------
    def __init__(self, port):
        serial.Serial.__init__(self, port, 57600,serial.SEVENBITS,serial.PARITY_ODD, serial.STOPBITS_ONE, None,False,False,None,False,None)

        self.term = '\r\n'
        self.delay = 2

        # clear error
        self.clearError()

        # clear and reset power supply
        self.clearInstrument()
        self.resetInstrument()

        #set current to 0
        self.setCurrent(0)
        self.setRampRate(magrate)

    #end init

    def setCurrent(self, current):
        if (-70 <= current <= 70):
            self.write('SETI ' + str(current) + self.term)
        else: pass
    #end def

    def getCurrent(self):
        self.write('RDGI? ' + self.term)
        time.sleep(self.delay)
        current = self.readline()

        current = float(current[:-2])
        return current
    #end def

    def getVoltage(self):
        self.write('RDGV? ' + self.term)
        time.sleep(self.delay)
        voltage = self.readline()
        voltage = float(voltage[:-2])
        return voltage
    #end def

    def setRampRate(self, rate):
        if (0.0001 <= rate <= 50):
            self.write('RATE ' + str(rate) + ',' + self.term)
        else: pass
    #end def

    def getRampRate(self):
        self.write('RATE? ' + self.term)
        time.sleep(self.delay)
        rate = self.readline()
        rate = float(rate[:-2])
        return rate
    #end def

    def stopRamp(self):
        self.write('STOP' + self.term)
        time.sleep(20*self.delay)
    #end def

    def setLimits(self, currentLim, rateLim):
        if (0.0001 <= rateLim <= 50 and 0 <= currentLim <= 70.1000):
            self.write('LIMIT' + str(currentLim) + ',' + str(rateLim) + self.term)
        else: pass
    #end def

    def getLimits(self):
        self.write('LIMIT?' + self.term)
        time.sleep(self.delay)
        limits = self.readline()
        limits = limits[:-2]
        limits = limits.split(',')
        currentLim = float(limits[0])
        rateLim = float(limits[1])
        return currentLim, rateLim
    #end def

    def errorTest(self):
        self.write('*TST?' + self.term)
        time.sleep(self.delay)
        error = self.readline()
        error = int(error[:-2])
        return error
    #end def

    def clearError(self):
        self.write('ERCL' + self.term)
    #end def

    def clearInstrument(self):
        self.write('*CLS' + self.term)
    #end def

    def resetInstrument(self):
        self.write('*RST' + self.term)
    #end def

    def factorySettings(self):
        self.write('DFLT 99' + self.term)
    #end def

#end class
###############################################################################

###############################################################################
class Setup:
    """
    Call this class to run the setup for the Keithley and the PID.
    """
    def __init__(self):
        """
        Prepare the Keithley to take data on the specified channels:
        """
        global k2700
        global k2400
        global k2182
        global ls643

        global maxMagCurrent
        global maxMagRate

        # Define Keithley instrument ports:
        self.k2700 = k2700 = Keithley_2700('GPIB0::2::INSTR') # MultiMeter for Matrix Card operation
        self.k2400 = k2400 = Keithley_2400('GPIB0::3::INSTR') # SourceMeter
        self.k2182 = k2182 = Keithley_2182('GPIB0::4::INSTR') # NanoVoltMeter
        # define the port for the magnet power supply
        self.ls643 = ls643 = Lakeshore_643('/dev/cu.SLAB_USBtoUART') # magnet power supply

        """
        Set Limits on Magnet Power Supply
        """
        self.ls643.clearInstrument()
        self.ls643.factorySettings()
        self.ls643.setLimits(maxMagCurrent,maxMagRate)

#end class
###############################################################################

###############################################################################
class InitialCheck:
    #--------------------------------------------------------------------------
    def __init__(self):
        global k2700
        global k2400

        self.k2700 = k2700
        self.k2400 = k2400


        self.delay = .5
        self.voltage = .1


        self.setupIV()

        self.Data = {}

        # short the matrix card
        self.k2700.closeChannels('117, 125, 126, 127, 128, 129,130')
        print(self.k2700.get_closedChannels())
        time.sleep(self.delay)

        self.measure_contacts()

        self.resetSourcemeter()
        self.create_plot()


    #end init
    #--------------------------------------------------------------------------

    #--------------------------------------------------------------------------
    def setupIV(self):
        self.k2400.ctrl.write(":SOUR:FUNC VOLT")
        self.k2400.ctrl.write(":SOUR:VOLT:MODE FIXED")
        self.k2400.ctrl.write(":SOUR:VOLT:RANG 20")
        self.k2400.ctrl.write(":SOUR:VOLT:LEV "+str(self.voltage))
        self.k2400.ctrl.write(":SENS:CURR:PROT 10E-2")
        self.k2400.ctrl.write(":SENS:FUNC CURR")
        self.k2400.ctrl.write(":SENS:CURR:RANG 10E-2")
        self.k2400.ctrl.write(":FORM:ELEM CURR")
    #end def
    #--------------------------------------------------------------------------

    #--------------------------------------------------------------------------
    def resetSourcemeter(self):
        self.k2400.current_mode()
        self.k2400.set_current_range(10.5*10**(-3)) # Default
        self.k2400.set_current(float(current))
    #end def
    #--------------------------------------------------------------------------

    #--------------------------------------------------------------------------
    def measure_contacts(self):

        # r_12
        print('measure r_12')
        self.k2700.openChannels('125, 126, 127, 128,129,130')
        print(self.k2700.get_closedChannels())
        self.r_12 = self.checkIV('A','B')
        self.k2700.closeChannels('125, 126, 127, 128,129,130')
        print(self.k2700.get_closedChannels())
        print "r12: %f Ohm" % (self.r_12)

        time.sleep(self.delay)

        # r_13
        print('measure r_13')
        self.k2700.closeChannels('119')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('117, 125, 126, 127, 128,129,130')
        print(self.k2700.get_closedChannels())
        self.r_13 = self.checkIV('A','C')
        self.k2700.closeChannels('117, 125, 126, 127, 128,129,130')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('119')
        print(self.k2700.get_closedChannels())
        print "r13: %f Ohm" % (self.r_13)

        time.sleep(self.delay)

        # r_24
        print('measure r_24')
        self.k2700.closeChannels('120')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('117, 125, 126, 127, 128,129,130')
        print(self.k2700.get_closedChannels())
        self.r_24 = self.checkIV('B','D')
        self.k2700.closeChannels('117, 125, 126, 127, 128,129,130')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('120')
        print(self.k2700.get_closedChannels())
        print "r24: %f Ohm" % (self.r_24)

        time.sleep(self.delay)

        # r_34
        print('measure r_34')
        self.k2700.closeChannels('118')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('117, 125, 126, 127, 128,129,130')
        print(self.k2700.get_closedChannels())
        self.r_34 = self.checkIV('C','D')
        self.k2700.closeChannels('117, 125, 126, 127, 128,129,130')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('118')
        print(self.k2700.get_closedChannels())
        print "r34: %f Ohm" % (self.r_34)

    #end def
    #--------------------------------------------------------------------------

    #--------------------------------------------------------------------------
    def checkIV(self,p1,p2):
        print('check IV')
        n = 6
        I = []
        V = [1000*self.voltage*(x)/n for x in range(-n,n+1)]

        for v in V:
            self.k2400.ctrl.write(":SOUR:VOLT:LEV "+str(float(v)/1000))
            self.k2400.ctrl.write(":OUTP ON")
            time.sleep(self.delay)
            i = float(self.k2400.ctrl.query(":READ?"))
            time.sleep(self.delay)
            self.k2400.ctrl.write(":OUTP OFF")
            print 'v: %f\ni: %f'%(v,i)
            I.append(i)
            time.sleep(self.delay)
        #end for


        fit = self.polyfit(V,I,1)

        self.Data[p1+p2] = fit

        self.Data[p1+p2]['current'] = I
        self.Data[p1+p2]['voltage'] = V

        r = 1/(fit['polynomial'][0])

        return r
    #end def
    #--------------------------------------------------------------------------

    #--------------------------------------------------------------------------
    def polyfit(self, x, y, degree):
        '''
        Returns the polynomial fit for x and y of degree degree along with the
        r^2 and the temperature, all in dictionary form.
        '''
        results = {}

        coeffs = np.polyfit(x, y, degree)

        # Polynomial Coefficients
        results['polynomial'] = coeffs.tolist()

        # Calculate coefficient of determination (r-squared):
        p = np.poly1d(coeffs)
        # fitted values:
        yhat = p(x)                      # or [p(z) for z in x]
        # mean of values:
        ybar = np.sum(y)/len(y)          # or sum(y)/len(y)
        # regression sum of squares:
        ssreg = np.sum((yhat-ybar)**2)   # or sum([ (yihat - ybar)**2 for yihat in yhat])
        # total sum of squares:
        sstot = np.sum((y - ybar)**2)    # or sum([ (yi - ybar)**2 for yi in y])
        results['r-squared'] = ssreg / sstot

        return results

    #end def

    #--------------------------------------------------------------------------
    def create_plot(self):
        plt.figure(num='IV Curves', figsize=(12,9),dpi=100)
        fitData = {}
        sp = 221
        for key in self.Data.keys():
            fitData[key] = {}
            i = np.poly1d(self.Data[key]['polynomial'])
            v = np.linspace(min(self.Data[key]['voltage']), max(self.Data[key]['voltage']), 500)

            fitData[key]['current'] = i(v)
            fitData[key]['voltage'] = v
            fitData[key]['equation'] = 'I = %.7f*(V) + %.7f' % (self.Data[key]['polynomial'][0], self.Data[key]['polynomial'][1])

            plt.subplot(sp)
            sp = sp + 1
            plt.plot(self.Data[key]['voltage'],self.Data[key]['current'],'r.',fitData[key]['voltage'],fitData[key]['current'],'b--')
            plt.xlabel("V (mV)")
            plt.ylabel("I (mA)")
            plt.title('IV curve - '+key)
            plt.legend(('i-v data: r^2 %.4f'%(self.Data[key]['r-squared']),'fit: '+fitData[key]['equation']),loc=4,fontsize=10)
            #plt.axis([ , , , ])
            plt.grid(True)
        #end for


        #fig.savefig('%s.png' % (plot_folder + title) , dpi=dpi)
        #plt.savefig('%s.png' % ('IV Curves') )
        plt.show()

    #end def

    #--------------------------------------------------------------------------
    def updateGUI(self, stamp, data):
        """
        Sends data to the GUI (main thread), for live updating while the process is running
        in another thread.
        """
        time.sleep(0.1)
        wx.CallAfter(pub.sendMessage, stamp, msg=data)

    #end def

#end class
###############################################################################

###############################################################################
class ProcessThreadRun(Thread):
    """
    Thread that runs the operations behind the GUI. This includes measuring
    and plotting.
    """

    #--------------------------------------------------------------------------
    def __init__(self):
        """ Init Worker Thread Class """
        Thread.__init__(self)
        self.start()

    #end init

    #--------------------------------------------------------------------------
    def run(self):
        """ Run Worker Thread """
        td = TakeData()
        #td = TakeDataTest()
    #end def

#end class
###############################################################################

###############################################################################
class TakeData:
    ''' Takes measurements and saves them to file. '''
    #--------------------------------------------------------------------------
    def __init__(self):
        global abort_ID
        global current
        global magcurrent
        global magrate
        global poledist
        global cycle
        global measurement_number

        self.k2400 = k2400
        self.k2700 = k2700
        self.k2182 = k2182
        self.ls643 = ls643

        self.delay = 1 # time for the keithley to take a steady measurement
        self.magdelay = np.abs((magcurrent / magrate) + 3) # time for magnet to stabilize

        self.current = current
        self.magcurrent = 0
        self.magrate = magrate
        self.poledist = float(poledist)
        self.thickness = float(thickness)

        self.k2400.set_current(self.current)
        self.ls643.setCurrent(0)

        self.exception_ID = 0

        self.updateGUI(stamp='Status Bar', data='Running')

        self.start = time.time()

        try:
            self.measurement='ON'
            self.updateGUI(stamp='Measurement', data=self.measurement)
            for n in range(measurement_number):
                self.safety_check()
                if abort_ID == 1: break
                # short the matrix card
                self.k2700.closeChannels('117, 125, 126, 127, 128, 129, 130')
                print(self.k2700.get_closedChannels())
                time.sleep(self.delay)

                ### RESISTIVITY MEASUREMENTS ###
                self.Resistivity_Measurement()
                if abort_ID == 1: break
                self.resistivity = self.resistivitycalc([self.r_A],[self.r_B])
                self.updateGUI(stamp = "Resistivity", data = self.resistivity*1000)

                ### HALL EFFECT MEASUREMENTS ###
                self.Hall_Measurement()
                if abort_ID == 1: break
                self.hallcoeff, self.concentration, self.mobility = self.hallcalc(self.thickness, self.bfield, self.r_P, self.resistivity)

                self.all_time = (self.t_A + self.t_B + self.t_P)/3
                self.updateGUI(stamp = "Hall Coefficient", data = self.hallcoeff)
                self.updateGUI(stamp = "Carrier Concentration", data = self.concentration)
                self.updateGUI(stamp = "Mobility", data = self.mobility)

                self.write_data_to_file()
            #end for


        #end try

        except exceptions.Exception as e:
            log_exception(e)

            abort_ID = 1

            self.exception_ID = 1

            print "Error Occurred, check error_log.log"
        #end except

        if self.exception_ID == 1:
            self.updateGUI(stamp='Status Bar', data='Exception Occurred')
        #end if
        else:
            self.updateGUI(stamp='Status Bar', data='Finished, Ready')
        #end else
        self.measurement = 'OFF'

        self.process_data()
        self.updateGUI(stamp='Measurement', data=self.measurement)
        self.save_files()

        self.k2400.turn_source_off()
        self.ls643.setCurrent(0)
        time.sleep(10)
        self.ls643.stopRamp()

        wx.CallAfter(pub.sendMessage, 'Enable Buttons')
    #end init

    #--------------------------------------------------------------------------
    def safety_check(self):
        magerror = self.ls643.errorTest()
        if magerror != 0:
            abort_ID = 1
    #end def

    #--------------------------------------------------------------------------
    def Resistivity_Measurement(self):
        ### r_A:
        # r_12,34
        print('measure r_12,34')
        self.k2700.openChannels('125, 127, 128, 129, 130')
        print(self.k2700.get_closedChannels())
        self.r_1234, self.t_1234 = self.delta_method()
        #self.r_1234 = abs(self.r_1234)
        self.k2700.closeChannels('125, 127, 128, 129, 130')
        print(self.k2700.get_closedChannels())
        print "t_r1234: %.2f s\tr1234: %f Ohm" % (self.t_1234, self.r_1234)

        time.sleep(self.delay)
        if abort_ID == 1: return
        # r_34,12
        print('measure r_34,12')
        self.k2700.closeChannels('118')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('117, 126, 127, 128, 129, 130')
        print(self.k2700.get_closedChannels())
        self.r_3412, self.t_3412 = self.delta_method()
        #self.r_3412 = abs(self.r_3412)
        self.k2700.closeChannels('117, 126, 127, 128, 129, 130')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('118')
        print(self.k2700.get_closedChannels())
        print "t_r3412: %.2f s\tr3412: %f Ohm" % (self.t_3412, self.r_3412)

        time.sleep(self.delay)
        if abort_ID == 1: return
        # Calculate r_A
        self.r_A = (self.r_1234 + self.r_3412)/2
        self.t_A = time.time()-self.start
        self.updateGUI(stamp="Time R_A", data=self.t_A)
        self.updateGUI(stamp="R_A", data=self.r_A*1000)
        print "t_rA: %.2f s\trA: %f Ohm" % (self.t_A, self.r_A)

        ### r_B:
        # r_13,24
        print('measure r_13,24')
        self.k2700.closeChannels('119')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('117, 125, 126, 127, 129, 130')
        print(self.k2700.get_closedChannels())
        self.r_1324, self.t_1324 = self.delta_method()
        #self.r_1324 = abs(self.r_1324)
        self.k2700.closeChannels('117, 125, 126, 127, 129, 130')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('119')
        print(self.k2700.get_closedChannels())
        print "t_r1324: %.2f s\tr1324: %f Ohm" % (self.t_1324, self.r_1324)

        time.sleep(self.delay)
        if abort_ID == 1: return
        # r_24,13
        print('measure r_24,13')
        self.k2700.closeChannels('120')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('117, 125, 126, 128, 129, 130')
        print(self.k2700.get_closedChannels())
        self.r_2413, self.t_2413 = self.delta_method()
        #self.r_2413 = abs(self.r_2413)
        self.k2700.closeChannels('117, 125, 126, 128, 129, 130')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('120')
        print(self.k2700.get_closedChannels())
        print "t_r2413: %.2f s\tr2413: %f Ohm" % (self.t_2413, self.r_2413)
        if abort_ID == 1: return
        # Calculate r_B
        self.r_B = (self.r_1324 + self.r_2413)/2
        self.t_B = time.time()-self.start
        self.updateGUI(stamp="Time R_B", data=self.t_B)
        self.updateGUI(stamp="R_B", data=self.r_B*1000)
        print "t_rB: %.2f s\trB: %f Ohm" % (self.t_B, self.r_B)
    #end def

    #--------------------------------------------------------------------------
    def resistivitycalc(self,Alist,Blist):
        global thickness
        delta = 0.0005 # error limit (0.05%)
        lim = 1
        rA = np.average(Alist)
        rB = np.average(Blist)

        z1 = (2*np.log(2))/(np.pi*(rA + rB))

        # Algorithm taken from http://www.nist.gov/pml/div683/hall_algorithm.cfm
        while (lim > delta):
            y = 1/np.exp(np.pi*z1*rA) + 1/np.exp(np.pi*z1*rB)

            z2 = z1 - (1/np.pi)*((1-y)/(rA/np.exp(np.pi*z1*rA) + rB/np.exp(np.pi*z1*rB)))

            lim = abs(z2 - z1)/z2

            z1 = z2
        #end while

        rho = 1/z1*float(thickness)
        return rho
    #end def

    #--------------------------------------------------------------------------
    def Hall_Measurement(self):

        # postive magnetic field
        self.magfield('pos')
        b1 = np.abs(fieldInterp(self.poledist, self.magcurrent))/10.0

        # rp_14,23
        print('measure rp_14,23')
        self.k2700.closeChannels('121')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('117, 125, 126, 127, 128, 129')
        print(self.k2700.get_closedChannels())
        time.sleep(self.delay)
        self.rp_1423, self.tp_1423 = self.delta_method()
        #self.rp_1423 = abs(self.rp_1423)
        self.k2700.closeChannels('117, 125, 126, 127, 128, 129')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('121')
        print(self.k2700.get_closedChannels())
        print "tp1423: %.2f s\trp1423: %f Ohm" % (self.tp_1423, self.rp_1423)
        if abort_ID == 1: return
        # rp_23,14
        print('measure rp_23,14')
        self.k2700.closeChannels('122')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('117, 125, 126, 127, 128, 130')
        print(self.k2700.get_closedChannels())
        time.sleep(self.delay)
        self.rp_2314, self.tp_2314 = self.delta_method()
        #self.rp_2314 = abs(self.rp_2314)
        self.k2700.closeChannels('117, 125, 126, 127, 128, 130')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('122')
        print(self.k2700.get_closedChannels())
        print "tp2314: %.2f s\trp2314: %f Ohm" % (self.tp_2314, self.rp_2314)
        if abort_ID == 1: return
        # postive magnetic field
        self.magfield('neg')
        b2 = np.abs(fieldInterp(self.poledist, self.magcurrent))/10.0

        # rp_14,23
        print('measure rn_14,23')
        self.k2700.closeChannels('121')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('117, 125, 126, 127, 128, 129')
        print(self.k2700.get_closedChannels())
        time.sleep(self.delay)
        self.rn_1423, self.tn_1423 = self.delta_method()
        #self.rn_1423 = abs(self.rn_1423)
        self.k2700.closeChannels('117, 125, 126, 127, 128, 129')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('121')
        print(self.k2700.get_closedChannels())
        print "tn1423: %.2f s\trn1423: %f Ohm" % (self.tn_1423, self.rn_1423)
        if abort_ID == 1: return
        time.sleep(self.delay)

        # rn_23,14
        print('measure rn_23,14')
        self.k2700.closeChannels('122')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('117, 125, 126, 127, 128, 130')
        print(self.k2700.get_closedChannels())
        time.sleep(self.delay)
        self.rn_2314, self.tn_2314 = self.delta_method()
        #self.rn_2314 = abs(self.rn_2314)
        self.k2700.closeChannels('117, 125, 126, 127, 128, 130')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('122')
        print(self.k2700.get_closedChannels())
        print "tn2314: %.2f s\trn2314: %f Ohm" % (self.tn_2314, self.rn_2314)
        if abort_ID == 1: return
        # turn off magnetic field & calculate r_perp
        self.magfield('off')
        self.r_1423 = (self.rp_1423-self.rn_1423)/2
        self.t_1423 = (self.tp_1423 + self.tn_1423)/2
        self.r_2314 = (self.rp_2314-self.rn_2314)/2
        self.t_2314 = (self.tp_2314 + self.tn_2314)/2
        self.r_P = (-1*self.r_1423 + self.r_2314)/2
        self.t_P = time.time()-self.start
        self.bfield = (b1 + b2)/2
        self.updateGUI(stamp="Time R_P", data=self.t_P)
        self.updateGUI(stamp="R_P", data=self.r_P)
        print "tP: %.2f s\trP: %f Ohm" % (self.t_P, self.r_P)
    #end def

    #--------------------------------------------------------------------------
    def hallcalc(self,d,B,rP,rho):
        RH = (d*rP/B)*(10**4)
        nH = 1/(RH*1.6*10**-19)/(10**18)
        uH = RH/rho
        return RH, nH, uH
    #end def

    #--------------------------------------------------------------------------
    def delta_method(self):
        print('Delta Method')
        t1 = time.time() - self.start
        # delta method:
        # positive V1:
        self.k2400.turn_source_on()
        self.k2400.set_current(float(self.current))
        self.updateGUI(stamp="Current Status", data=float(self.current)*1000)
        time.sleep(self.delay)
        v1p = float( self.k2182.fetch() )
        time.sleep(self.delay)
        # negative V:
        self.k2400.set_current(-1*float(self.current))
        self.updateGUI(stamp="Current Status", data=-1*float(self.current)*1000)
        time.sleep(self.delay)
        vn = float( self.k2182.fetch() )
        time.sleep(self.delay)
        t2 = time.time() - self.start

        # positive V2:
        self.k2400.set_current(float(self.current))
        self.updateGUI(stamp="Current Status", data=float(self.current)*1000)
        time.sleep(self.delay)
        v2p = float( self.k2182.fetch() )
        time.sleep(self.delay)
        self.k2400.turn_source_off()
        self.updateGUI(stamp="Current Status", data=0)

        t3 = time.time() - self.start

        print 'Delta Method'
        print 'i: %f Amps' % float(self.current)
        print "v: %f V, %f V, %f V" % (v1p, vn, v2p)

        r = (v1p + v2p - 2*vn)/(4*float(self.current))

        avgt = (t3 + t2 + t1)/3

        return r, avgt
    #end def

    #--------------------------------------------------------------------------
    def magfield(self,dir):
        global magcurrent
        global magrate

        if dir == 'pos':
            self.ls643.setCurrent(magcurrent)
            print "set mag current to %f\n" % (magcurrent)
            time.sleep(self.magdelay)
        #end if
        elif dir == 'neg':
            self.ls643.setCurrent(-magcurrent)
            print "set mag current to %f\n" % (-magcurrent)
            time.sleep(2*self.magdelay)
        #end elif
        else:
            self.ls643.setCurrent(0)
            print "set mag current to %f\n" % (0)
            time.sleep(self.magdelay)
        #end else
        self.magcurrent = self.ls643.getCurrent()
        self.magrate = self.ls643.getRampRate()
        b = fieldInterp(self.poledist, self.magcurrent)/10.0

        self.updateGUI(stamp="Mag Current", data=self.magcurrent)
        self.updateGUI(stamp="Mag Rate", data=self.magrate)
        self.updateGUI(stamp="Bfield", data=b)
    #end def

    #--------------------------------------------------------------------------
    def updateGUI(self, stamp, data):
        """
        Sends data to the GUI (main thread), for live updating while the process is running
        in another thread.
        """
        time.sleep(0.1)
        wx.CallAfter(pub.sendMessage, stamp, msg=data)
    #end def

    #--------------------------------------------------------------------------
    def write_data_to_file(self):
        global myfile
        global timecalclist,rAcalclist,rBcalclist,rPcalclist,Bcalclist

        print('Write data to file')
        myfile.write('%.2f,' % (self.all_time) )
        print 'time: ', self.all_time
        myfile.write('%s, %f,' % (self.thickness, self.bfield))
        print 'thickness (cm): ', self.thickness
        print 'bfield (T): ', self.bfield

        myfile.write('%.3f,%.3f,%.3f,' % (self.r_A*1000, self.r_B*1000, self.r_P*1000) )
        print 'r_A (mOhm): %.3f\nr_B (mOhm): %.3f\nr_P (mOhm):%.3f' % (self.r_A*1000, self.r_B*1000, self.r_P*1000)
        myfile.write('%.3f,%.3f,%.3f,%.3f\n' % (self.resistivity*1000,self.hallcoeff,self.concentration,self.mobility))
        print 'resistivity (mOhm*cm): %.3f' %(self.resistivity*1000)
        print 'hall coefficient (cm^3/C): %.3f' %(self.hallcoeff)
        print 'carrier concentration (10^18 cm^-3): %.3f' %(self.concentration)
        print 'hall mobility (cm^2/Vs): %.3f' %(self.mobility)


        timecalclist.append(self.all_time)
        rAcalclist.append(self.r_A)
        rBcalclist.append(self.r_B)
        rPcalclist.append(self.r_P)
        Bcalclist.append(self.bfield)
    #end def

    #--------------------------------------------------------------------------
    def process_data(self):
        global timecalclist, rAcalclist, rBcalclist, rPcalclist, Bcalclist
        global processfile

        time = np.average(timecalclist)
        B = np.average(Bcalclist)
        rP = np.average(rPcalclist)
        thickness = self.thickness

        resistivity = self.resistivitycalc(rAcalclist,rBcalclist)
        hallcoeff, concentration, mobility = self.hallcalc(thickness, B, rP, resistivity)


        processfile.write('%.1f,%.5f,%.5f,%.5f,%5f\n'%(time,resistivity*1000,hallcoeff,concentration,mobility))
    #end def

    #--------------------------------------------------------------------------
    def save_files(self):
        ''' Function saving the files after the data acquisition loop has been
            exited.
        '''

        print('Save Files')

        global dataFile
        global finaldataFile
        global myfile
        global processfile

        stop = time.time()
        end = datetime.now() # End time
        totalTime = stop - self.start # Elapsed Measurement Time (seconds)
        endStr = 'end time: %s \nelapsed measurement time: %s seconds \n \n' % (str(end), str(totalTime))

        myfile.close() # Close the file
        processfile.close()

        myfile = open(dataFile, 'r') # Opens the file for Reading
        contents = myfile.readlines() # Reads the lines of the file into python set
        myfile.close()

        # Adds elapsed measurement time to the read file list
        contents.insert(1, endStr) # Specify which line and what value to insert
        # NOTE: First line is line 0

        # Writes the elapsed measurement time to the final file
        myfinalfile = open(finaldataFile,'w')
        contents = "".join(contents)
        myfinalfile.write(contents)
        myfinalfile.close()

        # Save the GUI plots
        self.updateGUI(stamp='Save_All', data='Save')
    #end def

#end class
###############################################################################

###############################################################################
class BoundControlBox(wx.Panel):
    """ A static box with a couple of radio buttons and a text
        box. Allows to switch between an automatic mode and a
        manual mode with an associated value.
    """
    #--------------------------------------------------------------------------
    def __init__(self, parent, ID, label, initval):
        wx.Panel.__init__(self, parent, ID)

        self.value = initval

        box = wx.StaticBox(self, -1, label)
        sizer = wx.StaticBoxSizer(box, wx.VERTICAL)

        self.radio_auto = wx.RadioButton(self, -1, label="Auto", style=wx.RB_GROUP)
        self.radio_manual = wx.RadioButton(self, -1, label="Manual")
        self.manual_text = wx.TextCtrl(self, -1,
            size=(30,-1),
            value=str(initval),
            style=wx.TE_PROCESS_ENTER)

        self.Bind(wx.EVT_UPDATE_UI, self.on_update_manual_text, self.manual_text)
        self.Bind(wx.EVT_TEXT_ENTER, self.on_text_enter, self.manual_text)

        manual_box = wx.BoxSizer(wx.HORIZONTAL)
        manual_box.Add(self.radio_manual, flag=wx.ALIGN_CENTER_VERTICAL)
        manual_box.Add(self.manual_text, flag=wx.ALIGN_CENTER_VERTICAL)

        sizer.Add(self.radio_auto, 0, wx.ALL, 10)
        sizer.Add(manual_box, 0, wx.ALL, 10)

        self.SetSizer(sizer)
        sizer.Fit(self)

    #end init

    #--------------------------------------------------------------------------
    def on_update_manual_text(self, event):
        self.manual_text.Enable(self.radio_manual.GetValue())

    #end def

    #--------------------------------------------------------------------------
    def on_text_enter(self, event):
        self.value = self.manual_text.GetValue()

    #end def

    #--------------------------------------------------------------------------
    def is_auto(self):
        return self.radio_auto.GetValue()

    #end def

    #--------------------------------------------------------------------------
    def manual_value(self):
        return self.value

    #end def

#end class
###############################################################################

###############################################################################
class UserPanel(wx.Panel):
    ''' User Input Panel '''

    #--------------------------------------------------------------------------
    def __init__(self, *args, **kwargs):
        wx.Panel.__init__(self, *args, **kwargs)

        global current
        global magcurrent
        global magrate
        global thickness
        global poledist
        global measurement_number

        self.current = current*1000
        self.magcurrent = magcurrent
        self.magrate = magrate
        self.measurement_number = measurement_number

        self.create_title("User Panel") # Title

        self.font2 = wx.Font(11, wx.DEFAULT, wx.NORMAL, wx.NORMAL)

        self.current_control()
        self.mag_current_control()
        self.mag_rate_control()
        self.thickness_control()
        self.poledist_control()
        self.measurement_number_control()

        self.maxCurrent_label()
        self.maxMagCurrent_label()
        self.maxMagRate_label()

        self.linebreak1 = wx.StaticLine(self, pos=(-1,-1), size=(300,1))
        self.linebreak2 = wx.StaticLine(self, pos=(-1,-1), size=(300,1))
        self.linebreak3 = wx.StaticLine(self, pos=(-1,-1), size=(300,1))
        self.linebreak4 = wx.StaticLine(self, pos=(-1,-1), size=(600,1), style=wx.LI_HORIZONTAL)

        self.run_stop() # Run and Stop buttons

        self.create_sizer() # Set Sizer for panel

        #pub.subscribe(self.post_process_data, "Post Process")
        pub.subscribe(self.enable_buttons, "Enable Buttons")

    #end init

    #--------------------------------------------------------------------------
    def create_title(self, name):
        self.titlePanel = wx.Panel(self, -1)
        title = wx.StaticText(self.titlePanel, label=name)
        font_title = wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.BOLD)
        title.SetFont(font_title)

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add((0,-1))
        hbox.Add(title, 0, wx.LEFT, 5)

        self.titlePanel.SetSizer(hbox)
    #end def

    #--------------------------------------------------------------------------
    def run_stop(self):
        self.run_stopPanel = wx.Panel(self, -1)
        rs_sizer = wx.GridBagSizer(3, 3)


        self.btn_check = btn_check = wx.Button(self.run_stopPanel, label='check', style=0, size=(60,30)) # Run Button
        btn_check.SetBackgroundColour((0,0,255))
        caption_check = wx.StaticText(self.run_stopPanel, label='*check contacts')
        self.btn_run = btn_run = wx.Button(self.run_stopPanel, label='run', style=0, size=(60,30)) # Run Button
        btn_run.SetBackgroundColour((0,255,0))
        caption_run = wx.StaticText(self.run_stopPanel, label='*run measurement')
        self.btn_stop = btn_stop = wx.Button(self.run_stopPanel, label='stop', style=0, size=(60,30)) # Stop Button
        btn_stop.SetBackgroundColour((255,0,0))
        caption_stop = wx.StaticText(self.run_stopPanel, label = '*quit operation')

        btn_check.Bind(wx.EVT_BUTTON, self.check)
        btn_run.Bind(wx.EVT_BUTTON, self.run)
        btn_stop.Bind(wx.EVT_BUTTON, self.stop)

        controlPanel = wx.StaticText(self.run_stopPanel, label='Control Panel')
        controlPanel.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.BOLD))

        rs_sizer.Add(controlPanel,(0,0), span=(1,3),flag=wx.ALIGN_CENTER_HORIZONTAL)
        rs_sizer.Add(btn_check,(1,0),flag=wx.ALIGN_CENTER_HORIZONTAL)
        rs_sizer.Add(caption_check,(2,0),flag=wx.ALIGN_CENTER_HORIZONTAL)

        rs_sizer.Add(btn_run,(1,1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        rs_sizer.Add(caption_run,(2,1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        rs_sizer.Add(btn_stop,(1,2),flag=wx.ALIGN_CENTER_HORIZONTAL)
        rs_sizer.Add(caption_stop,(2,2),flag=wx.ALIGN_CENTER_HORIZONTAL)

        self.run_stopPanel.SetSizer(rs_sizer)

        btn_stop.Disable()

    # end def

    #--------------------------------------------------------------------------
    def run(self, event):
        global k2700, k2400, k2182, ls643
        global dataFile
        global finaldataFile
        global myfile
        global processfile
        global hallFile
        global abort_ID

        try:

            self.name_folder()

            file = dataFile # creates a data file
            myfile = open(dataFile, 'w') # opens file for writing/overwriting
            processfile = open(hallFile,'w')
            begin = datetime.now() # Current date and time
            myfile.write('Start Time: ' + str(begin) + '\n')
            processfile.write('Start Time: ' + str(begin) + '\n')

            dataheaders = ( 'time (s),thickness (cm), B-field (T),R_A (mOhm),R_B (mOhm),R_P (mOhm),Resistivity (mOhm*cm), Hall Coefficient (cm^3/C),Carrier Concentration (cm^-3),Hall Mobility (cm^2/Vs)\n' )
            myfile.write(dataheaders)

            processheaders = 'time (s),resistivity (mOhm*cm),hall coefficient (cm^3/C),carrier concentration (10^18 cm^-3),hall mobility (cm^2/Vs)\n'
            processfile.write(processheaders)

            abort_ID = 0

            #start the threading process

            thread = ProcessThreadRun()

            self.btn_current.Disable()
            self.btn_mag_current.Disable()
            self.btn_mag_rate.Disable()
            self.btn_thickness.Disable()
            self.btn_poledist.Disable()
            self.btn_measurement_number.Disable()
            self.btn_check.Disable()
            self.btn_run.Disable()
            self.btn_stop.Enable()


        #end try

        except visa.VisaIOError:
            wx.MessageBox("Not all instruments are connected!", "Error")
        #end except

    #end def

    #--------------------------------------------------------------------------
    def name_folder(self):
        question = wx.MessageDialog(None, 'The data files are saved into a folder upon ' + \
                    'completion. \nBy default, the folder will be named with a time stamp.\n\n' + \
                    'Would you like to name your own folder?', 'Question',
                    wx.YES_NO | wx.NO_DEFAULT | wx.ICON_QUESTION)
        answer = question.ShowModal()

        if answer == wx.ID_YES:
            self.folder_name = wx.GetTextFromUser('Enter the name of your folder.\n' + \
                                                'Only type in a name, NOT a file path.')
            if self.folder_name == "":
                wx.MessageBox("Canceled")
            else:
                self.choose_dir()

        #end if

        else:
            date = str(datetime.now())
            self.folder_name = 'Hall Effect Data %s.%s.%s' % (date[0:13], date[14:16], date[17:19])

            self.choose_dir()

        #end else

    #end def

    #--------------------------------------------------------------------------
    def choose_dir(self):
        found = False

        dlg = wx.DirDialog (None, "Choose the directory to save your files.", "",
                    wx.DD_DEFAULT_STYLE)

        self.run_check = dlg.ShowModal()

        if self.run_check == wx.ID_OK:
            global filePath
            filePath = dlg.GetPath()

            filePath = filePath + '/' + self.folder_name

            if not os.path.exists(filePath):
                os.makedirs(filePath)
                os.chdir(filePath)
            else:
                n = 1

                while found == False:
                    path = filePath + ' - ' + str(n)

                    if os.path.exists(path):
                        n = n + 1
                    else:
                        os.makedirs(path)
                        os.chdir(path)
                        n = 1
                        found = True

                #end while

            #end else

        #end if

        # Set the global path to the newly created path, if applicable.
        if found == True:
            filePath = path
        #end if
    #end def

    #--------------------------------------------------------------------------
    def check(self, event):

        ic = InitialCheck()


    #end def

    #--------------------------------------------------------------------------
    def stop(self, event):
        global abort_ID
        abort_ID = 1

        self.enable_buttons

    #end def

    #--------------------------------------------------------------------------
    def current_control(self):
        global current
        self.current_Panel = wx.Panel(self, -1)
        hbox = wx.BoxSizer(wx.HORIZONTAL)

        self.label_current = wx.StaticText(self, label="Current:")
        self.label_current.SetFont(self.font2)
        self.text_current = text_current = wx.StaticText(self.current_Panel, label=str(self.current) + ' mA')
        text_current.SetFont(self.font2)
        self.edit_current = edit_current = wx.TextCtrl(self.current_Panel, size=(40, -1))
        self.btn_current = btn_current = wx.Button(self.current_Panel, label="Save", size=(40, -1))
        text_guide = wx.StaticText(self.current_Panel, label="The current sourced to \nthe sample.")

        btn_current.Bind(wx.EVT_BUTTON, self.save_current)

        hbox.Add((0, -1))
        #hbox.Add(self.label_current, 0 , wx.LEFT, 5)
        hbox.Add(text_current, 0, wx.LEFT, 5)
        hbox.Add(edit_current, 0, wx.LEFT, 11)
        hbox.Add(btn_current, 0, wx.LEFT, 5)
        hbox.Add(text_guide, 0, wx.LEFT, 5)

        self.current_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def save_current(self, e):
        global current

        try:


            val = self.edit_current.GetValue()

            if float(val)/1000 > maxCurrent:
                current = str(maxCurrent)
            if float(val)/1000 < -maxCurrent:
                current = str(-maxCurrent)

            self.text_current.SetLabel(val + ' mA')

            current = float(val)/1000
            self.current = current*1000


        except ValueError:
            wx.MessageBox("Invalid input. Must be a number.", "Error")

        except visa.VisaIOError:
            wx.MessageBox("The SourceMeter is not connected!", "Error")
        #end except

    #end def

    #--------------------------------------------------------------------------
    def mag_current_control(self):
        global magcurrent
        self.mag_current_Panel = wx.Panel(self, -1)
        hbox = wx.BoxSizer(wx.HORIZONTAL)

        self.label_mag_current = wx.StaticText(self, label="Magnet Current:")
        self.label_mag_current.SetFont(self.font2)
        self.text_mag_current = text_mag_current = wx.StaticText(self.mag_current_Panel, label=str(self.magcurrent) + ' A')
        text_mag_current.SetFont(self.font2)
        self.edit_mag_current = edit_mag_current = wx.TextCtrl(self.mag_current_Panel, size=(40, -1))
        self.btn_mag_current = btn_mag_current = wx.Button(self.mag_current_Panel, label="Save", size=(40, -1))
        text_guide = wx.StaticText(self.mag_current_Panel, label="The current sourced to \nthe magnet.")

        btn_mag_current.Bind(wx.EVT_BUTTON, self.save_mag_current)

        hbox.Add((0, -1))
        #hbox.Add(self.label_current, 0 , wx.LEFT, 5)
        hbox.Add(text_mag_current, 0, wx.LEFT, 5)
        hbox.Add(edit_mag_current, 0, wx.LEFT, 11)
        hbox.Add(btn_mag_current, 0, wx.LEFT, 5)
        hbox.Add(text_guide, 0, wx.LEFT, 5)

        self.mag_current_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def save_mag_current(self, e):
        global magcurrent
        try:

            val = self.edit_mag_current.GetValue()

            if float(val) > maxMagCurrent:
                magcurrent = str(maxMagCurrent)
            if float(val) < -maxMagCurrent:
                magcurrent = str(-1*maxMagCurrent)

            self.text_mag_current.SetLabel(val + ' A')

            magcurrent = float(val)
            self.magcurrent = magcurrent


        except ValueError:
            wx.MessageBox("Invalid input. Must be a number.", "Error")

        except visa.VisaIOError:
            wx.MessageBox("The SourceMeter is not connected!", "Error")
        #end except

    #end def

    #--------------------------------------------------------------------------
    def mag_rate_control(self):
        global magrate
        self.mag_rate_Panel = wx.Panel(self, -1)
        hbox = wx.BoxSizer(wx.HORIZONTAL)

        self.label_mag_rate = wx.StaticText(self, label="Magnet Ramp Rate:")
        self.label_mag_rate.SetFont(self.font2)
        self.text_mag_rate = text_mag_rate = wx.StaticText(self.mag_rate_Panel, label=str(self.magrate) + ' A/s')
        text_mag_rate.SetFont(self.font2)
        self.edit_mag_rate = edit_mag_rate = wx.TextCtrl(self.mag_rate_Panel, size=(40, -1))
        self.btn_mag_rate = btn_mag_rate = wx.Button(self.mag_rate_Panel, label="Save", size=(40, -1))
        text_guide = wx.StaticText(self.mag_rate_Panel, label="The ramp rate of\nthe magnet.")

        btn_mag_rate.Bind(wx.EVT_BUTTON, self.save_mag_rate)

        hbox.Add((0, -1))
        #hbox.Add(self.label_current, 0 , wx.LEFT, 5)
        hbox.Add(text_mag_rate, 0, wx.LEFT, 5)
        hbox.Add(edit_mag_rate, 0, wx.LEFT, 11)
        hbox.Add(btn_mag_rate, 0, wx.LEFT, 5)
        hbox.Add(text_guide, 0, wx.LEFT, 5)

        self.mag_rate_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def save_mag_rate(self, e):
        global magrate
        try:


            val = self.edit_mag_rate.GetValue()

            if float(val) > maxMagRate:
                magrate = str(maxMagRate)

            self.text_mag_rate.SetLabel(val + ' A/s')

            magrate = float(val)
            self.magrate = magrate


        except ValueError:
            wx.MessageBox("Invalid input. Must be a number.", "Error")

        except visa.VisaIOError:
            wx.MessageBox("The SourceMeter is not connected!", "Error")
        #end except

    #end def

    #--------------------------------------------------------------------------
    def thickness_control(self):
        global thickness
        self.thickness_Panel = wx.Panel(self, -1)
        hbox = wx.BoxSizer(wx.HORIZONTAL)

        self.label_thickness = wx.StaticText(self, label="Sample Thickness:")
        self.label_thickness.SetFont(self.font2)
        self.text_thickness = text_thickness = wx.StaticText(self.thickness_Panel, label=thickness + ' cm')
        text_thickness.SetFont(self.font2)
        self.edit_thickness = edit_thickness = wx.TextCtrl(self.thickness_Panel, size=(40, -1))
        self.btn_thickness = btn_thickness = wx.Button(self.thickness_Panel, label="Save", size=(40, -1))
        text_guide = wx.StaticText(self.thickness_Panel, label="The thickness of \nthe sample.")

        btn_thickness.Bind(wx.EVT_BUTTON, self.save_thickness)

        hbox.Add((0, -1))
        #hbox.Add(self.label_current, 0 , wx.LEFT, 5)
        hbox.Add(text_thickness, 0, wx.LEFT, 5)
        hbox.Add(edit_thickness, 0, wx.LEFT, 11)
        hbox.Add(btn_thickness, 0, wx.LEFT, 5)
        hbox.Add(text_guide, 0, wx.LEFT, 5)

        self.thickness_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def save_thickness(self, e):
        global thickness
        val = self.edit_thickness.GetValue()
        self.text_thickness.SetLabel(val + ' cm')
        thickness = val
        wx.CallAfter(pub.sendMessage, "Sample Thickness", msg=thickness)

    #end def

    #--------------------------------------------------------------------------
    def poledist_control(self):
        global poledist
        self.poledist_Panel = wx.Panel(self, -1)
        hbox = wx.BoxSizer(wx.HORIZONTAL)

        self.label_poledist = wx.StaticText(self, label="Magnet Pole Separation:")
        self.label_poledist.SetFont(self.font2)
        self.text_poledist = text_poledist = wx.StaticText(self.poledist_Panel, label=poledist + ' mm')
        text_poledist.SetFont(self.font2)
        self.edit_poledist = edit_poledist = wx.TextCtrl(self.poledist_Panel, size=(40, -1))
        self.btn_poledist = btn_poledist = wx.Button(self.poledist_Panel, label="Save", size=(40, -1))
        text_guide = wx.StaticText(self.poledist_Panel, label="The separation between\nthe magnet poles.")

        btn_poledist.Bind(wx.EVT_BUTTON, self.save_poledist)

        hbox.Add((0, -1))
        #hbox.Add(self.label_current, 0 , wx.LEFT, 5)
        hbox.Add(text_poledist, 0, wx.LEFT, 5)
        hbox.Add(edit_poledist, 0, wx.LEFT, 11)
        hbox.Add(btn_poledist, 0, wx.LEFT, 5)
        hbox.Add(text_guide, 0, wx.LEFT, 5)

        self.poledist_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def save_poledist(self, e):
        global poledist
        val = self.edit_poledist.GetValue()
        self.text_poledist.SetLabel(val + ' mm')
        poledist = val
        wx.CallAfter(pub.sendMessage, "Pole Distance", msg=poledist)

    #end def

    #--------------------------------------------------------------------------
    def measurement_number_control(self):
        global measurement_number
        self.measurement_number_Panel = wx.Panel(self, -1)
        hbox = wx.BoxSizer(wx.HORIZONTAL)

        self.label_measurement_number = wx.StaticText(self,
                                            label="Measurement Number:"
                                            )
        self.label_measurement_number.SetFont(self.font2)
        self.text_measurement_number = text_measurement_number = wx.StaticText(self.measurement_number_Panel, label=str(self.measurement_number))
        text_measurement_number.SetFont(self.font2)
        self.edit_measurement_number = edit_measurement_number = wx.TextCtrl(self.measurement_number_Panel, size=(40, -1))
        self.btn_measurement_number = btn_measurement_number = wx.Button(self.measurement_number_Panel, label="Save", size=(40, -1))
        text_guide = wx.StaticText(self.measurement_number_Panel, label=('How many measurements \nto take.'
                                                                      )
                                   )

        btn_measurement_number.Bind(wx.EVT_BUTTON, self.save_measurement_number)

        hbox.Add((0, -1))
        #hbox.Add(self.label_equil_threshold, 0 , wx.LEFT, 5)
        hbox.Add(text_measurement_number, 0, wx.LEFT, 5)
        hbox.Add(edit_measurement_number, 0, wx.LEFT, 32)
        hbox.Add(btn_measurement_number, 0, wx.LEFT, 5)
        hbox.Add(text_guide, 0, wx.LEFT, 5)

        self.measurement_number_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def save_measurement_number(self, e):
        global measurement_number

        try:
            val = self.edit_measurement_number.GetValue()
            self.measurement_number = float(val)
            self.text_measurement_number.SetLabel(val)
            measurement_number = int(self.measurement_number)
        except ValueError:

            wx.MessageBox("Invalid input. Must be a number.", "Error")

    #end def

    #--------------------------------------------------------------------------
    def maxCurrent_label(self):
        self.maxCurrent_Panel = wx.Panel(self, -1)
        maxCurrent_label = wx.StaticText(self.maxCurrent_Panel, label='Max Current:')
        maxCurrent_text = wx.StaticText(self.maxCurrent_Panel, label='%s mA' % str(maxCurrent*1000))

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add((0,-1))
        hbox.Add(maxCurrent_label, 0, wx.LEFT, 5)
        hbox.Add(maxCurrent_text, 0, wx.LEFT, 5)

        self.maxCurrent_Panel.SetSizer(hbox)

    #edn def

    #--------------------------------------------------------------------------
    def maxMagCurrent_label(self):
        global maxMagCurrent
        self.maxMagCurrent_Panel = wx.Panel(self, -1)
        maxMagCurrent_label = wx.StaticText(self.maxMagCurrent_Panel, label='Max Magnet Current:')
        maxMagCurrent_text = wx.StaticText(self.maxMagCurrent_Panel, label='%s A' % str(maxMagCurrent))

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add((0,-1))
        hbox.Add(maxMagCurrent_label, 0, wx.LEFT, 5)
        hbox.Add(maxMagCurrent_text, 0, wx.LEFT, 5)

        self.maxMagCurrent_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def maxMagRate_label(self):
        global maxMagRate
        self.maxMagRate_Panel = wx.Panel(self, -1)
        maxMagRate_label = wx.StaticText(self.maxMagRate_Panel, label='Max Magnet Ramp Rate:')
        maxMagRate_text = wx.StaticText(self.maxMagRate_Panel, label='%s A/s' % str(maxMagRate))

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add((0,-1))
        hbox.Add(maxMagRate_label, 0, wx.LEFT, 5)
        hbox.Add(maxMagRate_text, 0, wx.LEFT, 5)

        self.maxMagRate_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def create_sizer(self):

        sizer = wx.GridBagSizer(12,2)

        sizer.Add(self.titlePanel, (0, 1), span=(1,2), flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_current, (1, 1))
        sizer.Add(self.current_Panel, (1, 2))
        sizer.Add(self.label_mag_current, (2, 1))
        sizer.Add(self.mag_current_Panel, (2, 2))
        sizer.Add(self.label_mag_rate, (3, 1))
        sizer.Add(self.mag_rate_Panel, (3, 2))

        sizer.Add(self.label_thickness, (4, 1))
        sizer.Add(self.thickness_Panel, (4, 2))
        sizer.Add(self.label_poledist, (5, 1))
        sizer.Add(self.poledist_Panel, (5, 2))

        sizer.Add(self.label_measurement_number, (6,1))
        sizer.Add(self.measurement_number_Panel, (6, 2))
        sizer.Add(self.maxCurrent_Panel, (7, 1), span=(1,2))
        sizer.Add(self.maxMagCurrent_Panel, (8, 1), span=(1,2))
        sizer.Add(self.maxMagRate_Panel, (9, 1), span=(1,2))

        sizer.Add(self.linebreak4, (10,1),span = (1,2))
        sizer.Add(self.run_stopPanel, (11,1),span = (1,2), flag=wx.ALIGN_CENTER_HORIZONTAL)

        self.SetSizer(sizer)

    #end def

    def enable_buttons(self):
        self.btn_current.Enable()
        self.btn_mag_current.Enable()
        self.btn_mag_rate.Enable()
        self.btn_thickness.Enable()
        self.btn_poledist.Enable()
        self.btn_measurement_number.Enable()
        self.btn_check.Enable()
        self.btn_run.Enable()
        self.btn_stop.Disable()

    #end def

#end class
###############################################################################

###############################################################################
class StatusPanel(wx.Panel):
    """
    Current Status of Measurements
    """
    #--------------------------------------------------------------------------
    def __init__(self, *args, **kwargs):
        wx.Panel.__init__(self, *args, **kwargs)

        global current
        global magcurrent
        global magrate

        self.ctime = str(datetime.now())[11:19]
        self.t='0:00:00'
        self.d=str(0.00)
        self.pd = str(0.00)
        self.rA=str(0.00)
        self.rB=str(0.00)
        self.rP=str(0.00)
        self.i = str(0.00)
        self.mi = str(0.00)
        self.mr = str(0.00)
        self.b = str(0.00)
        self.measurement = 'OFF'
        self.rho = str(0.00)
        self.RH = str(0.00)
        self.nH = str(0.00)
        self.uH = str(0.00)

        self.ohm = u"\u2126"
        self.perp = u"\u27c2"
        self.mu = u"\u00b5"

        self.create_title("Status Panel")
        self.linebreak1 = wx.StaticLine(self, pos=(-1,-1), size=(300,1))
        self.create_status()
        self.linebreak2 = wx.StaticLine(self, pos=(-1,-1), size=(300,1))

        self.linebreak3 = wx.StaticLine(self, pos=(-1,-1), size=(1,300), style=wx.LI_VERTICAL)

        # Updates from running program
        pub.subscribe(self.OnTime, "Time R_B")
        pub.subscribe(self.OnTime, "Time R_A")
        pub.subscribe(self.OnTime, "Time R_P")

        pub.subscribe(self.OnThickness, "Sample Thickness")
        pub.subscribe(self.OnPoleDist, "Pole Distance")

        pub.subscribe(self.OnR_A, "R_A")
        pub.subscribe(self.OnR_B, "R_B")
        pub.subscribe(self.OnR_P, "R_P")

        pub.subscribe(self.OnCurrent, "Current")
        pub.subscribe(self.OnMagCurrent, "Mag Current")
        pub.subscribe(self.OnMagRate, "Mag Rate")
        pub.subscribe(self.OnBfield, "Bfield")

        pub.subscribe(self.OnResistivity, "Resistivity")
        pub.subscribe(self.OnHallCoefficient, "Hall Coefficient")
        pub.subscribe(self.OnConcentration, "Carrier Concentration")
        pub.subscribe(self.OnMobility, "Mobility")

        pub.subscribe(self.OnMeasurement, "Measurement")

        #self.update_values()

        self.create_sizer()

    #end init

    #--------------------------------------------------------------------------
    def OnR_A(self, msg):
        self.rA = '%.2f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnR_B(self, msg):
        self.rB = '%.2f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnR_P(self, msg):
        self.rP = '%.2f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnCurrent(self, msg):
        self.i = '%.2f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnMagCurrent(self, msg):
        self.mi = '%.2f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnMagRate(self, msg):
        self.mr = '%.2f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnTime(self, msg):
        time = int(float(msg))

        hours = str(time/3600)
        minutes = int(time%3600/60)
        if (minutes < 10):
            minutes = '0%i'%(minutes)
        else:
            minutes = '%i'%(minutes)
        seconds = int(time%60)
        if (seconds < 10):
            seconds = '0%i'%(seconds)
        else:
            seconds = '%i'%(seconds)

        self.t = '%s:%s:%s'%(hours,minutes,seconds)
        self.ctime = str(datetime.now())[11:19]
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnThickness(self, msg):
        self.d = '%.2f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnPoleDist(self, msg):
        self.pd = '%.2f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnBfield(self, msg):
        self.b = '%.3f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnResistivity(self, msg):
        self.rho = '%.2f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnHallCoefficient(self, msg):
        self.RH = '%.2f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnConcentration(self, msg):
        self.nH = '%.2f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnMobility(self, msg):
        self.uH = '%.2f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnMeasurement(self, msg):
        self.measurement = msg
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def create_title(self, name):
        self.titlePanel = wx.Panel(self, -1)
        title = wx.StaticText(self.titlePanel, label=name)
        font_title = wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.BOLD)
        title.SetFont(font_title)

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add((0,-1))
        hbox.Add(title, 0, wx.LEFT, 5)

        self.titlePanel.SetSizer(hbox)
    #end def

    #--------------------------------------------------------------------------
    def create_status(self):
        self.label_ctime = wx.StaticText(self, label="current time:")
        self.label_ctime.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_t = wx.StaticText(self, label="run time (s):")
        self.label_t.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_d = wx.StaticText(self, label="sample thickness (cm):")
        self.label_d.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_pd = wx.StaticText(self, label="magnet pole separation (mm):")
        self.label_pd.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_rA = wx.StaticText(self, label="resistance_A (m"+self.ohm+"):")
        self.label_rA.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_rB = wx.StaticText(self, label="resistance_B (m"+self.ohm+"):")
        self.label_rB.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_rP = wx.StaticText(self, label="resistance_"+self.perp+" (m"+self.ohm+"):")
        self.label_rP.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_i = wx.StaticText(self, label="current (mA):")
        self.label_i.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_mi = wx.StaticText(self, label="mag current (A):")
        self.label_mi.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_mr = wx.StaticText(self, label="mag ramp rate (A/s):")
        self.label_mr.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_b = wx.StaticText(self, label="B-field (T):")
        self.label_b.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_measurement = wx.StaticText(self, label="measurement:")
        self.label_measurement.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_rho = wx.StaticText(self, label="resistivity (m"+self.ohm+"cm):")
        self.label_rho.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_RH = wx.StaticText(self, label="hall coefficient (cm^3/C):")
        self.label_RH.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_nH = wx.StaticText(self, label="carrier concentration (10^18 cm^-3):")
        self.label_nH.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_uH = wx.StaticText(self, label="hall mobility (cm^2/Vs):")
        self.label_uH.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))

        self.ctimecurrent = wx.StaticText(self, label=self.ctime)
        self.ctimecurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.tcurrent = wx.StaticText(self, label=self.t)
        self.tcurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.dcurrent = wx.StaticText(self, label=self.d)
        self.dcurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.pdcurrent = wx.StaticText(self, label=self.pd)
        self.pdcurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.rAcurrent = wx.StaticText(self, label=self.rA)
        self.rAcurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.rBcurrent = wx.StaticText(self, label=self.rB)
        self.rBcurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.rPcurrent = wx.StaticText(self, label=self.rP)
        self.rPcurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.icurrent = wx.StaticText(self, label=self.i)
        self.icurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.micurrent = wx.StaticText(self, label=self.mi)
        self.micurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.mrcurrent = wx.StaticText(self, label=self.mr)
        self.mrcurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.bcurrent = wx.StaticText(self, label=self.b)
        self.bcurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.measurementcurrent = wx.StaticText(self, label=self.measurement)
        self.measurementcurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.rhocurrent = wx.StaticText(self, label=self.rho)
        self.rhocurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.RHcurrent = wx.StaticText(self, label=self.RH)
        self.RHcurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.nHcurrent = wx.StaticText(self, label=self.nH)
        self.nHcurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.uHcurrent = wx.StaticText(self, label=self.uH)
        self.uHcurrent.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL))


    #end def

    #--------------------------------------------------------------------------
    def update_values(self):
        self.ctimecurrent.SetLabel(self.ctime)
        self.tcurrent.SetLabel(self.t)
        self.dcurrent.SetLabel(self.d)
        self.pdcurrent.SetLabel(self.pd)
        self.rAcurrent.SetLabel(self.rA)
        self.rBcurrent.SetLabel(self.rB)
        self.rPcurrent.SetLabel(self.rP)
        self.icurrent.SetLabel(self.i)
        self.micurrent.SetLabel(self.mi)
        self.mrcurrent.SetLabel(self.mr)
        self.bcurrent.SetLabel(self.b)
        self.measurementcurrent.SetLabel(self.measurement)
        self.rhocurrent.SetLabel(self.rho)
        self.RHcurrent.SetLabel(self.RH)
        self.nHcurrent.SetLabel(self.nH)
        self.uHcurrent.SetLabel(self.uH)

    #end def

    #--------------------------------------------------------------------------
    def create_sizer(self):
        sizer = wx.GridBagSizer(19,2)

        sizer.Add(self.titlePanel, (0, 0), span = (1,2), border=5, flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.linebreak1,(1,0), span = (1,2))

        sizer.Add(self.label_ctime, (2,0))
        sizer.Add(self.ctimecurrent, (2, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_t, (3,0))
        sizer.Add(self.tcurrent, (3, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_d, (4,0))
        sizer.Add(self.dcurrent, (4, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_pd, (5,0))
        sizer.Add(self.pdcurrent, (5, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)

        sizer.Add(self.label_rA, (6,0))
        sizer.Add(self.rAcurrent, (6, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_rB, (7,0))
        sizer.Add(self.rBcurrent, (7, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_rP, (8,0))
        sizer.Add(self.rPcurrent, (8, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)

        sizer.Add(self.label_i, (9,0))
        sizer.Add(self.icurrent, (9, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_mi, (10,0))
        sizer.Add(self.micurrent, (10, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_mr, (11,0))
        sizer.Add(self.mrcurrent, (11, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_b, (12,0))
        sizer.Add(self.bcurrent, (12, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)

        sizer.Add(self.label_rho, (13,0))
        sizer.Add(self.rhocurrent, (13, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_RH, (14,0))
        sizer.Add(self.RHcurrent, (14, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_nH, (15,0))
        sizer.Add(self.nHcurrent, (15, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_uH, (16,0))
        sizer.Add(self.uHcurrent, (16, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)

        sizer.Add(self.label_measurement, (17,0))
        sizer.Add(self.measurementcurrent, (17, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)

        sizer.Add(self.linebreak2, (18,0), span = (1,2))

        self.SetSizer(sizer)
    #end def

#end class
###############################################################################

###############################################################################
class ResistancePanel(wx.Panel):
    """
    GUI Window for plotting voltage data.
    """
    #--------------------------------------------------------------------------
    def __init__(self, *args, **kwargs):
        wx.Panel.__init__(self, *args, **kwargs)

        global filePath
        global t_A_list
        global r_A_list
        global t_B_list
        global r_B_list
        global t_P_list
        global r_P_list

        # Placers for the GUI plots:

        self.tA = 0
        self.tB = 0
        self.tP = 0
        self.rA = 1.0
        self.rB = 1.0
        self.rP = 0.1


        self.create_title("Resistance Panel")
        self.init_plot()
        self.canvas = FigureCanvasWxAgg(self, -1, self.figure)
        self.create_control_panel()
        self.create_sizer()

        pub.subscribe(self.OnR_A, "R_A")
        pub.subscribe(self.OnR_ATime, "Time R_A")
        pub.subscribe(self.OnR_B, "R_B")
        pub.subscribe(self.OnR_BTime, "Time R_B")
        pub.subscribe(self.OnR_P, "R_P")
        pub.subscribe(self.OnR_PTime, "Time R_P")

        # For saving the plots at the end of data acquisition:
        pub.subscribe(self.save_plot, "Save_All")

        self.animator = animation.FuncAnimation(self.figure, self.draw_plot, interval=2000, blit=False)
    #end init

    #--------------------------------------------------------------------------
    def create_title(self, name):
        self.titlePanel = wx.Panel(self, -1)
        title = wx.StaticText(self.titlePanel, label=name)
        font_title = wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.BOLD)
        title.SetFont(font_title)

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add((0,-1))
        hbox.Add(title, 0, wx.LEFT, 5)

        self.titlePanel.SetSizer(hbox)
    #end def

    #--------------------------------------------------------------------------
    def create_control_panel(self):

        self.xmin_control = BoundControlBox(self, -1, "t min", 0)
        self.xmax_control = BoundControlBox(self, -1, "t max", 100)
        self.ymin_control = BoundControlBox(self, -1, "R min", 10**-6)
        self.ymax_control = BoundControlBox(self, -1, "R max", 10**2)

        self.hbox1 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox1.AddSpacer(10)
        self.hbox1.Add(self.xmin_control, border=5, flag=wx.ALL)
        self.hbox1.Add(self.xmax_control, border=5, flag=wx.ALL)
        self.hbox1.AddSpacer(10)
        self.hbox1.Add(self.ymin_control, border=5, flag=wx.ALL)
        self.hbox1.Add(self.ymax_control, border=5, flag=wx.ALL)
    #end def

    #--------------------------------------------------------------------------
    def OnR_A(self, msg):
        self.rA = np.abs(float(msg))
        r_A_list.append(self.rA)
        t_A_list.append(self.tA)
    #end def

    #--------------------------------------------------------------------------
    def OnR_ATime(self, msg):
        self.tA = float(msg)

    #end def

    #--------------------------------------------------------------------------
    def OnR_B(self, msg):
        self.rB = np.abs(float(msg))
        r_B_list.append(self.rB)
        t_B_list.append(self.tB)
    #end def

    #--------------------------------------------------------------------------
    def OnR_BTime(self, msg):
        self.tB = float(msg)

    #end def

    #--------------------------------------------------------------------------
    def OnR_P(self, msg):
        self.rP = np.abs(float(msg))
        r_P_list.append(self.rP)
        t_P_list.append(self.tP)
    #end def

    #--------------------------------------------------------------------------
    def OnR_PTime(self, msg):
        self.tP = float(msg)

    #end def


    #--------------------------------------------------------------------------
    def init_plot(self):
        self.dpi = 100
        self.colorA = 'g'
        self.colorB = 'y'
        self.colorP = 'r'

        self.figure = Figure((6,3.5), dpi=self.dpi)
        self.subplot = self.figure.add_subplot(111)

        self.linerA, = self.subplot.plot(t_A_list,r_A_list, color=self.colorA, linewidth=1)
        self.linerB, = self.subplot.plot(t_B_list,r_B_list, color=self.colorB, linewidth=1)
        self.linerP, = self.subplot.plot(t_P_list,r_P_list, color=self.colorP, linewidth=1)

        self.legend = self.figure.legend( (self.linerA, self.linerB, self.linerP), (r"$R_A$", r"$R_B$", r"$R_\perp$"), (0.15,0.70),fontsize=8)

    #end def

    #--------------------------------------------------------------------------
    def draw_plot(self,i):
        self.subplot.clear()

        self.subplot.set_ylabel(r"resistance ($\Omega$)",fontsize=8)
        self.subplot.set_xlabel("time (s)", fontsize = 8)

        # Adjustable scale:
        if self.xmax_control.is_auto():
            xmax = max(t_A_list+t_B_list+t_P_list)
        else:
            xmax = float(self.xmax_control.manual_value())
        if self.xmin_control.is_auto():
            xmin = 0
        else:
            xmin = float(self.xmin_control.manual_value())
        if self.ymin_control.is_auto():
            minR = min(r_A_list+r_B_list+r_P_list)
            ymin = minR/3
        else:
            ymin = float(self.ymin_control.manual_value())
        if self.ymax_control.is_auto():
            maxR = max(r_A_list+r_B_list+r_P_list)
            ymax = maxR*3
        else:
            ymax = float(self.ymax_control.manual_value())

        self.subplot.set_yscale('log')
        self.subplot.set_xlim([xmin, xmax])
        self.subplot.set_ylim([ymin, ymax])


        pylab.setp(self.subplot.get_xticklabels(), fontsize=8)
        pylab.setp(self.subplot.get_yticklabels(), fontsize=8)

        self.linerA, = self.subplot.plot(t_A_list,r_A_list, color=self.colorA, linewidth=1)
        self.linerB, = self.subplot.plot(t_B_list,r_B_list, color=self.colorB, linewidth=1)
        self.linerP, = self.subplot.plot(t_P_list,r_P_list, color=self.colorP, linewidth=1)

        return (self.linerA, self.linerB, self.linerP)
        #return (self.subplot.plot( thighV_list, highV_list, color=self.colorH, linewidth=1),
            #self.subplot.plot( tlowV_list, lowV_list, color=self.colorL, linewidth=1))

    #end def

    #--------------------------------------------------------------------------
    def save_plot(self, msg):
        path = filePath + "/Resistance_Plot.png"
        self.canvas.print_figure(path)

    #end def

    #--------------------------------------------------------------------------
    def create_sizer(self):
        sizer = wx.GridBagSizer(3,1)
        sizer.Add(self.titlePanel, (0, 0), flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.canvas, ( 1,0), flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.hbox1, (2,0), flag=wx.ALIGN_CENTER_HORIZONTAL)

        self.SetSizer(sizer)
    #end def

#end class
###############################################################################

###############################################################################
class Frame(wx.Frame):
    """
    Main frame window in which GUI resides
    """
    #--------------------------------------------------------------------------
    def __init__(self, *args, **kwargs):
        wx.Frame.__init__(self, *args, **kwargs)
        self.init_UI()
        self.create_statusbar()
        self.create_menu()

        pub.subscribe(self.update_statusbar, "Status Bar")

    #end init

    #--------------------------------------------------------------------------
    def init_UI(self):
        self.SetBackgroundColour('#E0EBEB')
        self.userpanel = UserPanel(self, size=wx.DefaultSize)
        self.statuspanel = StatusPanel(self,size=wx.DefaultSize)
        self.resistancepanel = ResistancePanel(self, size=wx.DefaultSize)

        self.statuspanel.SetBackgroundColour('#ededed')

        sizer = wx.GridBagSizer(1, 3)
        sizer.Add(self.userpanel, (0,0),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.statuspanel, (0,2),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.resistancepanel, (0,1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Fit(self)

        self.SetSizer(sizer)
        self.SetTitle('Room Temp Hall Effect GUI')
        self.Centre()
    #end def

    #--------------------------------------------------------------------------
    def create_menu(self):
        # Menu Bar with File, Quit
        menubar = wx.MenuBar()
        fileMenu = wx.Menu()
        qmi = wx.MenuItem(fileMenu, APP_EXIT, '&Quit\tCtrl+Q')
        #qmi.SetBitmap(wx.Bitmap('exit.png'))
        fileMenu.AppendItem(qmi)

        self.Bind(wx.EVT_MENU, self.onQuit, id=APP_EXIT)

        menubar.Append(fileMenu, 'File')
        self.SetMenuBar(menubar)
    #end def

    #--------------------------------------------------------------------------
    def onQuit(self, e):
        global abort_ID

        abort_ID=1
        self.Destroy()
        self.Close()

        sys.stdout.close()
        sys.stderr.close()
    #end def

    #--------------------------------------------------------------------------
    def create_statusbar(self):
        self.statusbar = ESB.EnhancedStatusBar(self, -1)
        self.statusbar.SetSize((-1, 23))
        self.statusbar.SetFieldsCount(6)
        self.SetStatusBar(self.statusbar)

        self.space_between = 10

        ### Create Widgets for the statusbar:
        # Status:
        self.status_text = wx.StaticText(self.statusbar, -1, "Ready")
        self.width0 = 105

        # Placer 1:
        placer1 = wx.StaticText(self.statusbar, -1, " ")

        # Title:
        #measurement_text = wx.StaticText(self.statusbar, -1, "Measurement Indicators:")
        #boldFont = wx.Font(9, wx.DEFAULT, wx.NORMAL, wx.BOLD)
        #measurement_text.SetFont(boldFont)
        #self.width1 = measurement_text.GetRect().width + self.space_between

        # Measurement Time:
        measurement_number_text = wx.StaticText(self.statusbar, -1, "Time Until Measurement Complete:")
        self.width6 = measurement_number_text.GetRect().width + self.space_between

        self.indicator_measurement_number = wx.StaticText(self.statusbar, -1, "-")
        self.width7 = 40

        # Placer 2:
        placer2 = wx.StaticText(self.statusbar, -1, " ")

        # Version:
        version_label = wx.StaticText(self.statusbar, -1, "Version: %s" % version)
        self.width8 = version_label.GetRect().width + self.space_between

        # Set widths of each piece of the status bar:
        self.statusbar.SetStatusWidths([self.width0, 50, self.width6, self.width7, -1, self.width8])

        ### Add the widgets to the status bar:
        # Status:
        self.statusbar.AddWidget(self.status_text, ESB.ESB_ALIGN_CENTER_HORIZONTAL, ESB.ESB_ALIGN_CENTER_VERTICAL)

        # Placer 1:
        self.statusbar.AddWidget(placer1)

        # Title:
        #self.statusbar.AddWidget(measurement_text, ESB.ESB_ALIGN_CENTER_HORIZONTAL, ESB.ESB_ALIGN_CENTER_VERTICAL)

        # Measurement Time:
        self.statusbar.AddWidget(measurement_number_text, ESB.ESB_ALIGN_CENTER_HORIZONTAL, ESB.ESB_ALIGN_CENTER_VERTICAL)
        self.statusbar.AddWidget(self.indicator_measurement_number, ESB.ESB_ALIGN_CENTER_HORIZONTAL, ESB.ESB_ALIGN_CENTER_VERTICAL)

        # Placer 2
        self.statusbar.AddWidget(placer2)

        # Version:
        self.statusbar.AddWidget(version_label, ESB.ESB_ALIGN_CENTER_HORIZONTAL, ESB.ESB_ALIGN_CENTER_VERTICAL)

    #end def

    #--------------------------------------------------------------------------
    def update_statusbar(self, msg):
        string = msg

        # Status:
        if string == 'Running' or string == 'Checking' or string == 'Finished, Ready' or string == 'Exception Occurred':
            self.status_text.SetLabel(string)
            self.status_text.SetBackgroundColour(wx.NullColour)

            if string == 'Exception Occurred':
                self.status_text.SetBackgroundColour("RED")
            #end if

        #end if

        # Measurement Timer:
        else:
            self.indicator_measurement_number.SetLabel(string[:-3] + ' (s)')

        #end else


    #end def


#end class
###############################################################################

###############################################################################
class App(wx.App):
    """
    App for initializing program
    """
    #--------------------------------------------------------------------------
    def OnInit(self):
        self.frame = Frame(parent=None, title="Room Temp Hall Effect/Resistivity GUI", size=(1350,1350))
        self.frame.Show()

        setup = Setup()
        return True
    #end init

#end class
###############################################################################

#==============================================================================
if __name__=='__main__':
    app = App()
    app.MainLoop()

#end if
