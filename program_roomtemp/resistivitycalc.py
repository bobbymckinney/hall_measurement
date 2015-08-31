import numpy as np

def resistivitycalc(Alist,Blist):
    thickness = .0575
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

    rho = '%.2f'%(1/z1*float(thickness))
    return rho
#end def


resistivitycalc([27.276],[1.497])