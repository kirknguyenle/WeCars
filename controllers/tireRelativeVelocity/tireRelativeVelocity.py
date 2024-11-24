from controller import Supervisor,Robot, GPS, Motor, Node
from datetime import date, datetime
import sys
import numpy
import math
import os.path

sys.path.append(r"C:\Users\quoca\OneDrive\Desktop\Motorsports\WeCars\extra_scripts")

import transformations as tf

run = datetime.now()

runString= run.strftime("%d-%m-%Y-%H-%M-%S")

save_path = (r"C:\Users\quoca\OneDrive\Desktop\Motorsports\WeCars\testData")
filename = (r"tireRelativeVelocity" + runString)

location = os.path.join(save_path, filename+".txt")
print(location)
file = open(location, "w")
file.write("#time,V_x,k,SlipAngle,LateralForce,DynamicS,AbsVy,\r\n")
TIME_STEP = 32

robot = Supervisor()  # create Robot instance
mc_node = robot.getFromDef('Contact1')
tire_node = robot.getFromDef('Wheel')
p_node = robot.getFromDef('TirePhysics')
load = robot.getDevice('loadSensor')
load.enable(10)
if mc_node == None:
    sys.exit(1)
    
if tire_node == None: 
    sys.exit(1)



    
coulomb_friction = mc_node.getField("coulombFriction")
sideslipConstant = mc_node.getField("forceDependentSlip")


mu = 1


mf_var = [12.5664, 1.4, 0.714, 0.2] # Magic Formula parameters D,C,B,E




S = 0.0

coulomb_friction.setMFFloat(0,mu)

sideslipConstant.setMFFloat(0, 0)
sideslipConstant.setMFFloat(1, S)



k = 12.5664




i = 0
slip = 0.0

useMF = False

v_x = 1

rotationZ= numpy.array(tire_node.getOrientation()).reshape(3,3)
tireForces = [[0],[0],[0]]
while robot.step(TIME_STEP) != -1:
    
    v_C = [v_x, 0, 0, 0, 0, 0]
    v_C[1] = v_C[1]+ i/1000
    
    rotation = numpy.array(tire_node.getOrientation()).reshape(3,3)
    
    rotationZ[0][2] = rotation[0][2]
    rotationZ[1][2] = rotation[1][2]
    rotationZ[2][2] = rotation[2][2]
    r_i = numpy.linalg.inv(rotation)
    r_iZ = numpy.linalg.inv(rotationZ)
    velocity_raw = numpy.array(tire_node.getVelocity())
    velocity = numpy.array([[velocity_raw[0]],[velocity_raw[1]],[velocity_raw[2]]])
    velocity_local = numpy.matmul(r_iZ,velocity)

    if (velocity_local[0] != 0.0):
        slip = numpy.arctan(velocity_local[1]/velocity_local[0])
    
    #print ("velocity: \n", velocity, "\n")
    #print("rotation matrix: \n", rotationZ, "\n")
    #print("velocity local matrix: \n", velocity_local*10, "\n")
    #print("slip angle: \n", numpy.degrees(slip), "\n")
    i +=1
    tireRaw = load.getValues()
    tireForces[0] = tireRaw[0]
    tireForces[1] = tireRaw[1]
    tireForces[2] = tireRaw[2]
    localTireVect = numpy.matmul(r_i,tireForces)
    
    print("force vector: \n",localTireVect, "\n")
  
  
    
    
    
    
    tire_node.setVelocity(v_C)
    
    

    
    
    tire_node.setVelocity(v_C)# roll forward
    
        
    
    
   
    #print('abs_vy \n', abs_vy)
   
    #print('lateral force \n', k*slip)
    #print('slip \n', (slip))
    if slip != 0.0:
        if useMF:
            f_y = mf_var[0]*numpy.sin(mf_var[1]*numpy.arctan(mf_var[2]*slip-mf_var[3]*(mf_var[2]*slip-numpy.arctan(mf_var[2]*slip))))
            S = numpy.tan(slip)*velocity_local[0]/(f_y)
            sideslipConstant.setMFFloat(1, S) 
            #print('MF dynamic \n',S)
        else:
            S = (numpy.tan(slip)*velocity_local[0])/(k*slip)
            sideslipConstant.setMFFloat(1, S) 
            #print('FDS dynamic \n',S)
    
   
    file.write(str(i)+","+str(velocity_local[0])+","+str(k)+","+str(slip)+","+str(S)+","+str(velocity_local[1])+"\r\n")
    
file.close()

