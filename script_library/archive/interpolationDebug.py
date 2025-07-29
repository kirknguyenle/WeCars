import numpy as np
from matplotlib.pyplot import *


steer_angles, rack_extension = np.loadtxt(r"C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\car_files\mr2\mr2Steer.txt", delimiter=',', unpack=True)

st1 = []
rx1 = []

print(steer_angles)
print(rack_extension)



figure, ax1 = subplots()
ax1.plot(steer_angles, rack_extension)
show()
