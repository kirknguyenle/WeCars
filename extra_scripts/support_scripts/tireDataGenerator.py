import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


Fz = np.arange(2000, 8000, 150)
angles = np.arange(0, 0.5, 0.0125)
D = 8000
C = 1.4
B = 0.714
E = 0.2
df = pd.DataFrame()

for i in range(len(Fz)):
    tmp = []
    for j in range(len(angles)):
        fy = (D/(8000/Fz[i]))*np.sin(C*np.arctan(B*(angles[j])-E*(B*(angles[j])-np.arctan(B*(angles[j])))))
        tmp.append(fy)
    s = pd.Series(tmp, index = angles)
    print(s)
    df[Fz[i]] = s

index = df.index
columns = df.columns
sl, fz = np.meshgrid(Fz, angles)
Fy = z = np.array([[df[c][i] for c in columns] for i in index])
fig = plt.figure()
axes = fig.add_subplot(projection='3d')
axes.plot_surface(fz, sl, Fy)
axes.set_zlim3d(bottom=0)
plt.show()

df.to_csv(r'C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\car_files\tireData\artificaldata6.csv')
