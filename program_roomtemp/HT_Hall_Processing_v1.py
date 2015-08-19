# -*- coding: utf-8 -*-
"""
Created on 2015-05-22

@author: Bobby McKinney (rmckinne@mines.edu)

__Title__

Description:
    
    
Comments:
    
"""

import numpy as np

#--------------------------------------------------------------------------
def output_file(inFile, outFile):
    
    celsius = u"\u2103"
    ohm = u"\u2126"
    perp = u"\u27c2"
    
    data = import_Data(inFile)
    
    index = create_measurement_index(data[-1])
    #print index
    
    ### Create an indexed list of all averaged values:
    # number of measurements:
    num = len(index)/2
    # index list:
    measurement_range = [[None,None,None,None,None,None,None,None]]*num # start, stop (indexes), temp
    n = 0 # index for this list
    for i in xrange(len(index)-1):
        #if 'Start':
        if index[i][1] == 'Start':
            m1 = index[i] # Start
            m2 = index[i+1] # Stop
            low = m1[0]
            high = m2[0]
            
            avg_temp  = np.average(data[0][low:high+1])
            avg_d = np.average(data[1][low:high+1])
            avg_B = np.average(data[2][low:high+1])
            avg_r1234 = np.average(data[3][low:high+1])
            avg_r3412 = np.average(data[4][low:high+1])
            avg_r1324 = np.average(data[5][low:high+1])
            avg_r2413 = np.average(data[6][low:high+1])
            avg_r1423 = np.average(data[7][low:high+1])
            avg_r2314 = np.average(data[8][low:high+1])
            
            avg_rA = (avg_r1234 + avg_r3412)/2
            avg_rB = (avg_r1324 + avg_r2413)/2
            avg_rP = (avg_r1423 + avg_r2314)/2
            
            measurement_range[n] = [low, high, avg_temp, avg_d, avg_B, avg_rA, avg_rB, avg_rP]
            n = n + 1
    
    ### Create a file to save the data to:
    file = outFile
    myfile = open(outFile, 'w')
    myfile.write('Temperature ('+celsius+'),Thickness (cm),Magnetic Field (T),Sheet Resistance ('+ohm+'),Resistivity ('+ohm+'cm),Perpendicular Resistance ('+ohm+'),Hall Coefficient ('+ohm+'cm/T)\n')
    
    ### Get final data at each temp:
    final_data = [[None,None,None,None,None,None,None]]*len(measurement_range) # temp, sheet resistance
    for x in xrange(len(measurement_range)):
        t = measurement_range[x][2]
        d = measurement_range[x][3]
        B = measurement_range[x][4]/10
        rs = calculate_sheet_resistance(measurement_range[x][5], measurement_range[x][6])
        rho = d*rs
        rp = measurement_range[x][7]
        RH = d*rp/B
        final_data[x] = [t, d, B, rs, rho, rp, RH]
        myfile.write('%f,%f,%f,%f,%f,%f,%f\n' % (t,d,B,rs,rho,rp,RH))
    
    myfile.close()

#end def

#--------------------------------------------------------------------------
def import_Data(filePath):
    
    f = open(filePath)
    loadData = f.read()
    f.close()
    
    loadDataByLine = loadData.split('\n')
    numericData = loadDataByLine[6:]
    #print(numericData)
    
    length = len(numericData)
    d      = [None]*length
    B      = [None]*length
    temp   = [None]*length
    r_1234 = [None]*length
    r_3412 = [None]*length
    r_1324 = [None]*length
    r_2413 = [None]*length
    r_1423 = [None]*length
    r_2314 = [None]*length
    r_A    = [None]*length
    r_B    = [None]*length
    r_P    = [None]*length
    indicator = [None]*length
    
    for x in xrange(length):
        line = numericData[x].split(',')
        #print(line)
        temp[x] = float(line[1])
        d[x] = float(line[2])
        B[x] = float(line[3])
        r_1234[x] = float(line[5])
        r_3412[x] = float(line[7])
        r_1324[x] = float(line[9])
        r_2413[x] = float(line[11])
        r_1423[x] = float(line[13])
        r_2314[x] = float(line[15])
        r_A[x] = float(line[16])
        r_B[x] = float(line[17])
        r_P[x] = float(line[18])
        indicator[x] = line[19]
    #end for
    
    return temp, d, B, r_1234, r_3412, r_1324, r_2413, r_1423, r_2314, r_A, r_B, r_P, indicator
            
#end def

#--------------------------------------------------------------------------           
def create_measurement_index(indicator):
    # Extract a list of just 'Start' and 'Stop' (and 'Left' (Equilibrium) if applicable):
    h = [None]*len(indicator)
    for x in xrange(len(indicator)):
        h[x] = indicator[x][:-12]
    h = ','.join(h)
    h = ''.join(h.split())
    h = h.split(',')
    h = [x for x in h if x]
    
    # Get number of Measurements:
    num = 0
    for x in xrange(len(h)):
        if h[x] == 'Start':
            # if the indicator says stop:
            if h[x+1] == 'Stop':
                num = num + 1
        
    num = num*2 # Both start and stop
    
    
    # Create a list that records the beginning and end of each measurement:
    measurement_indicator = [[None,None]]*num # [line num, 'Start/Stop', temp]
    s = -1 # iterator for each element of h
    n = 0 # iterator to create the elements of measurement_indicator
    for x in xrange(len(indicator)):
        if indicator[x] == 'Start Measurement':
            s = s + 1 # next element of h
            if h[s] == 'Start' and h[s+1] == 'Stop':
                measurement_indicator[n] = [x,'Start']
                n = n + 1
            #end if
        #end if
        elif indicator[x] == 'Stop Measurement':
            measurement_indicator[n] = [x,'Stop']
            n = n + 1
            # goes to the next element of h:
            s = s + 1
        #end elif
        elif  indicator[x] == 'Left Equilibrium':
            # goes to the next element of h:
            s = s + 1
        #end elif
            
    #end for
            
    return measurement_indicator

#end def

#--------------------------------------------------------------------------
def calculate_sheet_resistance(rA, rB):
    delta = 0.0005 # error limit (0.05%)
    
    z1 = (2*np.log(2))/(np.pi*(rA + rB))

    condition = 'not met'
    
    # Algorithm taken from http://www.nist.gov/pml/div683/hall_algorithm.cfm
    while condition == 'not met':
        y = 1/np.exp(np.pi*z1*rA) + 1/np.exp(np.pi*z1*rB)
        
        z2 = z1 - (1/np.pi)*((1-y)/(rA/np.exp(np.pi*z1*rA) + rB/np.exp(np.pi*z1*rB)))
        
        lim = abs(z2 - z1)/z2
        
        if lim < delta:
            condition = 'met'
        else:
            z1 = z2
            
    #end while
    
    return 1/z2
    
#end def    
    
#==============================================================================
            
def main():
    #directory = 'E:\Google Drive\Toberer Lab\Resistivity System\Data\R Test 14\\'
    directory = 'C:\Users\Toberer Lab\Google Drive\rmckinney\samples\MoTe2\\'
    inFile = directory + 'Data.csv'
    outFile = directory + 'Final Data Test.csv'
    
    output_file(inFile, outFile)
    
#end def
    
if __name__ == '__main__':
    main()
#end if
