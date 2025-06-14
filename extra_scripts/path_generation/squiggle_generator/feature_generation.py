import numpy as np
import math as m

def generateLine(path,length, increment):
    n = m.ceil(length/increment) 
    xf = path[0, len(path[0])-1]
    yf = path[1, len(path[0])-1]
    for i in range(n):
        path[0].append(xf + (i+1)*increment)
        path[1].append(yf)
    return path

def jumpLaneChange(path, dy, increment):
    xf = path[0, len(path[0])-1]
    yf = path[1, len(path[0])-1]
    path[0].append(xf+increment)
    path[1].append(yf+dy)

def linearLaneChange(path, theta, dy, increment):
    dx = dy/np.tan(theta)
    n = m.ceil(dx/increment)
    xf = path[0, len(path[0])-1]
    yf = path[1, len(path[0])-1]
    for i in range(n):
        path[0].append(xf + (i+1)*increment)
        path[1].append(yf + (i+1)*increment*np.tan(theta)) 
    
    path[0].append(xf + (n+1)*increment)
    path[1].append(yf + dy)

 
   
