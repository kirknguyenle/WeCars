import pathlib
from numpy import *
from matplotlib.pyplot import *
import FirstOrderFilter as fl
bmp,cmb,cst,toe,strA,stR = loadtxt(r"C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\test_data\SuspGraph20-01-2025-22-59-49.txt", delimiter=',', unpack=True)

leverArm_y = 0.275
#create arrays for bump steer
bmp1 = []
cmb1 = []
toe1 = []
#enter arrays
for i in range(280):
    bmp1.append(bmp[i]*leverArm_y*100)
    cmb1.append(np.degrees(cmb[i]))
    toe1.append(np.degrees(toe[i]))

o_cmb = 1600*(2*np.pi)
o_toe = 300*2*np.pi
filtered_cmb = fl.firstOrderFilter(bmp1, cmb1, o_cmb, 0.0000005)
filtered_toe = fl.firstOrderFilter(bmp1, toe1, o_toe, 0.0005)
pp_cmb = fl.postProcess(filtered_cmb, 2)
pp_toe = fl.postProcess(filtered_toe, 8)

#ec_cmb = fl.edgeComp(bmp1, filtered_cmb, pp_cmb, o_cmb, 0.000005, 2)
#ec_toe = fl.edgeComp(bmp1, filtered_toe, pp_toe, o_toe, 0.0005, 2)


figure, ax1 = subplots()
ax1.plot(bmp1, cmb1, label = 'sanity check', color = 'orange')
ax1.plot(bmp1[0: len(bmp1)-2],pp_cmb, label = 'Filter + PP', color = 'blue')
figure.supxlabel('Bump Travel (cm)')
figure.supylabel('Camber (Degrees)')
figure.legend()
tight_layout()

figure, ax2 = subplots(1)
ax2.plot(pp_toe,bmp1[0: len(bmp1)-8], label = 'Filter + PP', color = 'blue')
figure.supxlabel('Toe (Degrees)')
figure.supylabel('Bump Travel (cm)')
ax2.spines['bottom'].set_position('center')
figure.legend()
tight_layout()
show()