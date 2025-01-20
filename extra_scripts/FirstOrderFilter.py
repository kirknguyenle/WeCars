import numpy as np
# Thank you Bryce Gaille for the suggestion of a the First Order Filter
# Derivation of FOF implementation found from: 
# Joshua A. Marshall, How to Implement a First-Order Low-Pass Filter in Discrete Time, 2021, URL: https://github.com/botprof/first-order-low-pass-filter.
def firstOrderFilter(x, y, omega, T):
    u = np.zeros(len(y))
    a = ((2-T*omega)/(2+T*omega))
    b = T*omega/(2+T*omega)
    for i in range(0, len(x)):
        dy = y[i]+y[i-1]
        u[i] = a*u[i-1]+b*dy
    return u


def postProcess(y, shift):
    u = np.zeros(len(y)-shift)
    for i in range(0, len(y)-shift):
        u[i] = y[i+shift]
    return u
            
#Table for later
def edgeComp(x, y, pp, omega, T, remd):
    u = np.zeros(len(pp))
    for i in range (0, len(y)-remd):
        u[i] = pp[i]   
   
    for i in range(len(x)-remd, len(x)):
        dydx = (pp[i-5]+pp[i-4]+pp[i-3]+pp[i-2])/4
        ext = pp[i-1]-y[i-1]
        if (dydx < 0):
            ext = ext*-1
        u[i] = y[i] + ext
    return u





















