import numpy as np
import pandas as pd
"Helper File for Standardized Tire Data Interpetation for Webots"


def simpleLinearSaturated(Vx,Stiffness, Slip, Saturation):
    S = 0
    if(Slip < Saturation):
        S = (np.tan(Slip))/(Vx*Stiffness*Slip)
        return S
    else:
        S = (np.tan(Slip))/(Vx*Stiffness*Saturation)
        return S

def simpleLinearSaturatedLong(Vw, Vc, Stiffness, Saturation):
    S = 0
    SR = (((Vw/Vc)-1)*100)
    if(SR < Saturation):
        S = ((Vw-Vc)/(SR*Stiffness))
        return S
    else:
        S = (Vw-Vc)/(SR*Saturation)
        return S

def simpleMFLong(B, C, D,E, Vw, Vc):
    SR = (((Vw/Vc)-1)*100)
    f_r = D*np.sin(C*np.arctan(B*SR-E*(B*SR-np.arctan(B*SR))))
    S = ((Vw-Vc)/(f_r))
    return S

def simpleMFLat(B, C, D,E, Slip, Vx):
    f_y = D*np.sin(C*np.arctan(B*Slip-E*(B*Slip-np.arctan(B*Slip))))
    S = S = np.tan(Slip)/(Vx*f_y)
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



