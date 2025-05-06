import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import tireHelper as th


Fz = np.arange(2000, 8000, 150)
slip = np.arange(0, 1.57, 0.005)

vx = 10

D = 8000*2000
C = 1.4
B = 0.714
E = 0.2
#df = pd.DataFrame()
fy = []
lnsat = []


for i in range(len(slip)):
        mfl = D*np.sin(C*np.arctan(B*(slip[i])-E*(B*(slip[i])-np.arctan(B*(slip[i])))))
        #fy.append(mfl)
        #fy.append(np.tan(slip[i])/(vx*mfl))
        fy.append(th.simpleMFLat(B, C, D, E, slip[i], vx))
        if(slip[i]*B*C*D < B*C*D*0.9):
                lf = slip[i]*B*C*D
        else:
                lf = B*C*D*0.9
        #lnsat.append(lf)
        #lnsat.append(np.tan(slip[i])/(vx*lf))
        lnsat.append(th.simpleLinearSaturated(vx, B*D*C, slip[i], B*C*D))
        
        
fig = plt.figure()
axes = fig.add_subplot()
axes.plot(slip,fy, label='mf')
axes.plot(slip,lnsat, label = 'linear')
plt.legend()
plt.show()

#df.to_csv(r'C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\car_files\tireData\artificaldata6.csv')
