import numpy as np
import pandas as pd
"Helper File for Standardized Tire Data Interpetation for Webots"


def simpleLinearSaturated(Vx,Stiffness, Slip, Saturation):
    S = 0
    if(Slip*Stiffness < Saturation):
        S = (np.tan(Slip))/(Vx*Stiffness*Slip)
        return S
    else:
        S = (np.tan(Slip))/(Vx*Saturation)
        return S

def simpleLinearSaturatedLong(Vw, Vc, Stiffness, Saturation):
    S = 0
    if(Vw == 0):
        return 0
    SR = (((Vw/Vc)-1)*100)
    if(SR*Stiffness < Saturation):
        S = ((Vw-Vc)/(SR*Stiffness))
        return S
    else:
        S = (Vw-Vc)/(Saturation)
        return S

def simpleMFLong(B, C, D, E, Vw, Vc):
    if(Vw == 0):
        return 0
    SR = (((Vw/Vc)-1)*100)
    f_r = D*np.sin(C*np.arctan(B*SR-E*(B*SR-np.arctan(B*SR))))
    S = ((Vw-Vc)/(f_r))
    return S

def mfzLat(B, C, D, E, Slip, Vx, fz):
    f_y = fz*D*np.sin(C*np.arctan(B*Slip-E*(B*Slip-np.arctan(B*Slip))))
    S = np.tan(Slip)/(Vx*f_y)
    if S < 0:
        return 0
    else:
        return S

def mfzLong(B, C, D, E, Vw, Vc, fz):
    SR = (((Vw/Vc)-1)*100)
    f_r = fz*D*np.sin(C*np.arctan(B*SR-E*(B*SR-np.arctan(B*SR))))
    S = ((Vw-Vc)/(f_r))
    if S < 0:
        return 0
    else:
        return S

def simpleMFLat(B, C, D,E, Slip, Vx):
    f_y = D*np.sin(C*np.arctan(B*Slip-E*(B*Slip-np.arctan(B*Slip))))
    S = np.tan(Slip)/(Vx*f_y)
    return S


def loadTireData(data, mu_max):
    table = pd.read_csv(data, header = None)
    table.mul(mu_max)
    return table

def forceDependantTableLong(Vc,Vw, Fz, table):
    SR = (((Vw/Vc)-1)*100)
    f_r = table.get(str(SR), str(Fz))
    S = ((Vw-Vc)/(f_r))
    return S

def forceDependantTableLat(Vx,Slip, Fz, table):
    f_y = table.get(str(Slip), str(Fz))
    S = np.tan(Slip)/(Vx*f_y)
    return S


def rotationMatrix3D(theta):
    m =[[np.cos(theta), np.sin(theta), 0],
         [-np.sin(theta), np.cos(theta), 0],
         [0, 0, 1]]
    return m

def velocityTransform2D(v1, tdot, r):
    cross = np.cross([0,0,tdot], [r[0],r[1],0])
    x = v1[0] + cross[0]
    y = v1[1] + cross[1]
    return [x,y]



