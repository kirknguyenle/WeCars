from controller import Supervisor,Robot, GPS, Motor, Node, TouchSensor
from datetime import date, datetime
import sys
import numpy
import math
import os.path

run = datetime.now()

runString= run.strftime("%d-%m-%Y-%H-%M-%S")

save_path = (r"C:\Users\quoca\OneDrive\Desktop\Motorsports\WeCars\testData")
filename = (r"dynamicLongitudinalVx_" + runString)

location = os.path.join(save_path, filename+".txt")
print(location)
file = open(location, "w")
file.write("#time,V_x,k,SlipAngle,LateralForce,DynamicS,AbsVy,\r\n")
TIME_STEP = 32

robot = Supervisor()  # create Robot instance
mc_node = robot.getFromDef('Contact1')
tire_node = robot.getFromDef('Wheel')

if mc_node == None:
    sys.exit(1)
    
if tire_node == None: 
    sys.exit(1)
    
coulomb_friction = mc_node.getField("coulombFriction")
sideslipConstant = mc_node.getField("forceDependentSlip")


mf_var = [0, 0, 0, 0] # Magic Formula parameters D,C,B,E


S = 0.0

coulomb_friction.setMFFloat(0,300)

sideslipConstant.setMFFloat(0, 0)
sideslipConstant.setMFFloat(1, 0)

k = 100000



i = 0
slip = 0.0

useMF = False

v_y = 1

while robot.step(TIME_STEP) != -1:
    
    i +=1
    v_C = [0, v_y, 0, 0, 0, 0]
    v_C[0] = v_C[0]+ i/1000
    
    tire_node.setVelocity(v_C)
    
    absolute_v = tire_node.getVelocity()
    
    abs_vy = absolute_v[1]
    
    tire_node.setVelocity(v_C)# roll forward
    if absolute_v[0] != 0.0:
        slip = numpy.arctan(abs_vy/absolute_v[0])
    
    print('slip \n', numpy.degrees(slip))
   
    print('abs_vy \n', abs_vy)
   
    print('lateral force \n', k*slip)
    if slip != 0.0:
        if useMF:
            f_y = mf_var[0]*numpy.sin(mf_var[1]*numpy.arctan(mf_var[2]*slip-mf_var[3]*(mf_var[2]*slip-numpy.arctan(mf_var[2]*slip))))
            S = numpy.tan(slip)/(f_y*absolute_v[0])
            sideslipConstant.setMFFloat(1, S) 
            print('MF dynamic \n',S)
        else:
            S = (numpy.tan(slip)*absolute_v[0])/(k*slip)
            sideslipConstant.setMFFloat(1, S) 
            print('FDS dynamic \n',S)
            
   
    file.write(str(i)+","+str(absolute_v[0])+","+str(k)+","+str(slip)+","+str(k*slip)+","+str(S)+","+str(absolute_v[1])+"\r\n")
    
file.close()

