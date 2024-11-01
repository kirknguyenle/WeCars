import pathlib
from numpy import *
from matplotlib.pyplot import *

t,v_x,k,slip,f_y,S,AbsVy = loadtxt(r"C:\Users\quoca\OneDrive\Desktop\Motorsports\WeCars\testData\dynamicFDS_31-10-2024-20-14-45.txt", delimiter=',', unpack=True)
#print(slip)
print(shape(slip))
print(len(slip))


linear = []
webots = []
for i in range(len(slip)):
    linear.append(k[0]*slip[i])
    webots.append(((tan(slip[i])*v_x[0])/(S[i])))
print(shape(linear))


figure, ax1 = subplots(2,2)

ax1[0,0].plot(slip, f_y, label='FDS', color='blue')
ax1[0,1].plot(slip, webots, label='Webots', color='red')
ax1[1,0].plot(slip, linear, label='Linear', color='green')
figure.suptitle('Comparison of FDS, Webots, and Linear Model')
figure.supxlabel('Slip Angle (radians)')
figure.supylabel('Lateral Force (N)')
figure.legend()
tight_layout()
show()