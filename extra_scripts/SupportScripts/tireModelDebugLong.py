import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import tireHelper as th


Fz = np.arange(2000, 8000, 150)
slip = np.arange(0, 10, 0.5)

vx = 10

D = 9000*2000
C = 1.4
B = 0.714
E = 0.2
#df = pd.DataFrame()
fy = []
lnsat = []


for i in range(len(slip)):
        #mfl = (D)*np.sin(C*np.arctan(B*(slip[i])-E*(B*(slip[i])-np.arctan(B*(slip[i])))))
        fy.append(th.simpleMFLong(B,C,D,E,slip[i],vx))
        #fy.append((((slip[i]/100)+1)/(mfl)))
        if(slip[i]*B*C*D < B*C*D*0.9):
                lf = slip[i]*B*C*D
        else:
                lf = B*C*D*0.9
        #lnsat.append(((slip[i]/100)+1)/lf)
        lnsat.append(th.simpleLinearSaturatedLong(slip[i],vx,B*C*D, B*C*D*0.9))
        
fig = plt.figure()
axes = fig.add_subplot()
axes.plot(slip,fy, label= 'mf')
axes.plot(slip,lnsat, label= 'linear')
plt.legend()
plt.show()

#df.to_csv(r'C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\car_files\tireData\artificaldata6.csv')
