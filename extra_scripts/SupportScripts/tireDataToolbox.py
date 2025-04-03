import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


Fz = np.arange(2000, 8000, 150)
slip = np.arange(-100, 100, 0.5)

D = 8000
C = 1.4
B = 0.714
E = 0.2
#df = pd.DataFrame()
fy = []
for i in range(len(slip)):
        fy.append((D)*np.sin(C*np.arctan(B*(slip[i])-E*(B*(slip[i])-np.arctan(B*(slip[i]))))))

fig = plt.figure()
axes = fig.add_subplot()
axes.plot(slip,fy)
plt.show()

#df.to_csv(r'C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\car_files\tireData\artificaldata6.csv')
