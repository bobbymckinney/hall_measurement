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
    
    ohm = u"\u2126"
    perp = u"\u27c2"
    
    data = import_Data(inFile)
    
    avg_d = np.average(data[0])
    avg_B = np.average(data[1])
    avg_r1234 = np.average(data[2])
    avg_r3412 = np.average(data[3])
    avg_r1324 = np.average(data[4])
    avg_r2413 = np.average(data[5])
    avg_rp1423 = np.average(data[6])
    avg_rn1423 = np.average(data[7])
    avg_rp2314 = np.average(data[8])
    avg_rn2314 = np.average(data[9])
    
    avg_rA = (avg_r1234 + avg_r3412)/2
    avg_rB = (avg_r1324 + avg_r2413)/2
    avg_rP1 = -1*(avg_rp1423 - avg_rn1423)/2
    avg_rP2 = (avg_rp2314 - avg_rn2314)/2
    avg_rP = (avg_rP1 + avg_rP2)/2
            
    measurement = [avg_d, avg_B, avg_rA, avg_rB, avg_rP]
    
    ### Create a file to save the data to:
    file = outFile
    myfile = open(outFile, 'w')
    myfile.write('Thickness (cm),Magnetic Field (T),Sheet Resistance (Ohm),Resistivity (Ohm*cm),Perpendicular Resistance (Ohm),Hall Coefficient (cm^3/C), Carrier Concentration (10^18 cm^-3), Hall Mobility (cm^2/Vs)\n')
    e = 1.6*10**-19
    ### Get final data:
    d = measurement[0]
    B = measurement[1]
    rs = calculate_sheet_resistance(measurement[2], measurement[3])
    rho = d*rs
    rp = measurement[4]
    RH = (d*rp/B)*(10**4)
    nH = 1/(RH*e)
    uH = RH/rho
    final_data = [d, B, rs, rho, rp, RH, nH/(10**18), uH]
    myfile.write('%f,%f,%f,%f,%f,%f,%f,%f\n' % (d,B,rs,rho,rp,RH,nH/(10**18),uH))
    print '\nthickness (cm): %.2f\nb-field (T): %.3f\nsheet resistance (Ohm): %.5f\nresistivity (Ohm*cm): %.5f\nperpendicular resistance (Ohm): %.9f\nhall coefficient (cm^3/C): %.4f\ncarrier concentration (10^18 cm^-3): %.1f\nhall mobility (cm^2/Vs): %f\n' % (d,B,rs,rho,rp,RH,nH/(10**18),uH)
    
    
    myfile.close()

#end def

#--------------------------------------------------------------------------
def import_Data(filePath):
    
    f = open(filePath)
    loadData = f.read()
    f.close()
    
    loadDataByLine = loadData.split('\n')
    numericData = loadDataByLine[5:]
    
    length = len(numericData)-1
    d      = [None]*length
    B      = [None]*length
    r_1234 = [None]*length
    r_3412 = [None]*length
    r_1324 = [None]*length
    r_2413 = [None]*length
    rp_1423 = [None]*length
    rn_1423 = [None]*length
    rp_2314 = [None]*length
    rn_2314 = [None]*length
    r_A    = [None]*length
    r_B    = [None]*length
    r_P    = [None]*length
    
    for x in xrange(length):
        line = numericData[x].split(',')
        d[x] = float(line[1])
        B[x] = float(line[2])
        r_1234[x] = float(line[4])
        r_3412[x] = float(line[6])
        r_1324[x] = float(line[8])
        r_2413[x] = float(line[10])
        rp_1423[x] = float(line[12])
        rn_1423[x] = float(line[14])
        rp_2314[x] = float(line[16])
        rn_2314[x] = float(line[18])
        r_A[x] = float(line[19])
        r_B[x] = float(line[20])
        r_P[x] = float(line[21])

    #end for
    
    return d, B, r_1234, r_3412, r_1324, r_2413, rp_1423, rn_1423, rp_2314, rn_2314, r_A, r_B, r_P
            
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
    directory = '/Users/bobbymckinney/Google Drive/rwm-tobererlab/Hall Effect Data 2015-05-27 16.36.15'
    inFile = directory + '/Data.csv'
    outFile = directory + '/Final Data.csv'
    
    output_file(inFile, outFile)
    
#end def
    
if __name__ == '__main__':
    main()
#end if
