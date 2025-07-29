import numpy as np
import math as m

def generateLine(path,length, increment):
    n = m.ceil(length/increment) 
    xf = path[0][len(path[0])-1]
    yf = path[1][len(path[0])-1]
    for i in range(n):
        path[0].append(xf + (i+1)*increment)
        path[1].append(yf)
    return path

def jumpLaneChange(path, dy, increment):
    xf = path[0][len(path[0])-1]
    yf = path[1][len(path[0])-1]
    path[0].append(xf+increment)
    path[1].append(yf+dy)

    return path

def linearLaneChange(path, theta, dy, increment):
    dx = (abs(dy)/np.tan(theta))
    n = m.ceil(dx/increment)-1
    print(dx)
    xf = path[0][len(path[0])-1]
    yf = path[1][len(path[0])-1]
    for i in range(n):
        path[0].append(xf + (i+1)*increment)
        path[1].append(yf + (i+1)*increment*np.tan(theta)*np.sign(dy)) 
    
    path[0].append(xf + (n+1)*increment)
    path[1].append(yf + dy)

    return path

def sinLaneChange(path, dx, dy, increment):
    n = m.ceil(abs(dx)/increment)
    xf = path[0][len(path[0])-1]
    yf = path[1][len(path[0])-1]
    for i in range(n):
        path[0].append(xf + (i+1)*increment)
        path[1].append(yf + ((dy/2)*(np.sin((np.pi*((i+1))/dx)-np.pi/2)+1)))
    path[0].append(xf + (n+1)*increment)
    path[1].append(yf + dy)

    return path
 
   
