from controller import Supervisor,Robot, GPS, Motor, Node
from datetime import date, datetime
import sys
import numpy
import math
import os.path
import transformations as tf


run = datetime.now()

runString= run.strftime("%d-%m-%Y-%H-%M-%S")

save_path = (r"C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\testData")
filename = (r"loadSmartTire" + runString)

location = os.path.join(save_path, filename+".txt")
print(location)
file = open(location, "w")
file.write("#time,V_x,k,SlipAngle,LateralForce,DynamicS,RelVy,\r\n")
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


mu = 2


mf_var = [10000, 1.4, 0.714, 0.2] # Magic Formula parameters D,C,B,E




S = 0.0

coulomb_friction.setMFFloat(0,mu)

sideslipConstant.setMFFloat(0, 0)
sideslipConstant.setMFFloat(1, S)



k = 10000
r = 0.08



i = 0
slip = 0.0

useMF = False

v_x = 1


tireForces = [[0],[0],[0]]
while robot.step(TIME_STEP) != -1:
    
    v_C = [0, v_x, 0, 0, 0, 0]
    v_C[0] = v_C[0]+ i/1000

    
    rotation = numpy.array(tire_node.getOrientation()).reshape(3,3)
    euler = tf.getEulerAngles(rotation)
    inv = numpy.transpose(rotation)
    velocity_raw = numpy.array(tire_node.getVelocity())
    angularVel = numpy.array([[velocity_raw[3]],[velocity_raw[4]],[velocity_raw[5]]])
    velocity = numpy.array([[velocity_raw[0]],[velocity_raw[1]],[velocity_raw[2]]])
    angularLocal = numpy.matmul(inv,angularVel)
    velocity_local = numpy.matmul(inv,velocity)
    euler = tf.getEulerAngles(rotation)
    if (numpy.abs(angularLocal[2])*r != 0.0):
        slip = numpy.arctan(velocity_local[2]/numpy.abs(angularLocal[2])*r)
    print("Slip Angle (Degrees) \n",numpy.degrees(slip))
    #print("Velocity X: \n", numpy.abs(angularLocal[2])*r,"\n")
    print("Velocity Y: \n", velocity_local[2],"\n")
    i +=1
    tireRaw = load.getValues()
    tireForces[0] = tireRaw[0]
    tireForces[1] = tireRaw[1]
    tireForces[2] = tireRaw[2]
    print("Force Z: \n",
          numpy.sqrt(numpy.power(tireRaw[0],2)+numpy.power(tireRaw[1],2)), 
          "\n")
    print("Force Y: \n",tireForces[2],"\n")

  
  
   
    
    tire_node.setVelocity(v_C)# roll forward
    
        
    
    
   
    #print('abs_vy \n', abs_vy)
   
    #print('lateral force \n', k*slip)
    #print('slip \n', (slip))
    if slip != 0.0:
        if useMF:
            f_y = mf_var[0]*numpy.sin(mf_var[1]*numpy.arctan(mf_var[2]*slip-mf_var[3]*(mf_var[2]*slip-numpy.arctan(mf_var[2]*slip))))
            S = numpy.tan(slip)*numpy.abs(angularLocal[2])*r/(f_y)
            sideslipConstant.setMFFloat(1, S) 
            #print('MF dynamic \n',S)
        else:
            
            S = (numpy.tan(slip)*numpy.abs(angularLocal[2])*r)/(k*slip)
            sideslipConstant.setMFFloat(1, S) 
            print('FDS dynamic \n',S)
    
   
    file.write(str(i)+","+str(float(numpy.abs(angularLocal[2])*r))+","+str(k)+","+str(float(slip))+","+str(float(tireForces[2]))+","+str(float(S))+","+str(float(velocity_local[2]))+"\r\n")
    
file.close()

