import pathlib
from numpy import *
from matplotlib.pyplot import *

t,v_x,k,slip,f_y,S,AbsVy = loadtxt(r"C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\testData\SuspGraph20-01-2025-10-50-17.txt", delimiter=',', unpack=True)
#print(slip)
print(shape(slip))
print(len(slip))


linear = []
webots = []
yvel =[]
for i in range(len(slip)):
    linear.append(f_y[i])
    webots.append(((tan(slip[i])*v_x[0])/(S[i])))
print(shape(linear))


figure, ax1 = subplots(2,2)

ax1[0,0].plot(t, f_y, label='FDS', color='blue')
ax1[0,1].plot(t, AbsVy, label ="vy", color='red')
ax1[1,0].plot(t,slip,label = 'slip',color='green')
figure.supxlabel('Slip Angle (radians)')
figure.supylabel('Lateral Force (N)')
figure.legend()
tight_layout()
show()