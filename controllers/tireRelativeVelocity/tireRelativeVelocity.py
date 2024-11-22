from controller import Supervisor,Robot, GPS, Motor, Node
from datetime import date, datetime
import sys
import numpy
import math
import os.path

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
mass = p_node.getField("mass")




mf_var = [100040, 1.4, 0.714, 0.2] # Magic Formula parameters D,C,B,E


mu = 1000

S = 0.0

coulomb_friction.setMFFloat(0,mu)

sideslipConstant.setMFFloat(0, 0)
sideslipConstant.setMFFloat(1, S)

F_n = mass.getSFFloat()*mu

k = 100040



i = 0
slip = 0.0

useMF = False

v_x = 1

while robot.step(TIME_STEP) != -1:

    v_C = [v_x, 0, 0, 0, 0, 0]
    v_C[1] = v_C[1]+ i/1000
    rotation = numpy.array(tire_node.getOrientation()).reshape(3,3)
    r_i = numpy.linalg.inv(rotation)
    i +=1
    tireForces = load.getValues()
    print("force vector: \n", tireForces[2], "\n")
  
  
    v_l = numpy.array(v_C[0:3]) # Legacy code
    v_r = numpy.dot(r_i,v_l) # Legacy Right Now
    
    
    
    
    tire_node.setVelocity(v_C)
    
    absolute_v = tire_node.getVelocity()
    
    abs_v = absolute_v
    
    
    
    tire_node.setVelocity(v_C)# roll forward
    if absolute_v[0] != 0.0:
        slip = numpy.arctan(absolute_v[1]/absolute_v[0])
    
    #print('slip \n', (slip))
   
    #print('abs_vy \n', abs_vy)
   
    #print('lateral force \n', k*slip)
    if slip != 0.0:
        if useMF:
            f_y = mf_var[0]*numpy.sin(mf_var[1]*numpy.arctan(mf_var[2]*slip-mf_var[3]*(mf_var[2]*slip-numpy.arctan(mf_var[2]*slip))))
            S = numpy.tan(slip)*absolute_v[0]/(f_y)
            sideslipConstant.setMFFloat(1, S) 
            print('MF dynamic \n',S)
        else:
            S = (numpy.tan(slip)*absolute_v[0])/(k*slip)
            sideslipConstant.setMFFloat(1, S) 
            #print('FDS dynamic \n',S)
    
   
    file.write(str(i)+","+str(absolute_v[0])+","+str(k)+","+str(slip)+","+str(F_n)+","+str(S)+","+str(absolute_v[1])+"\r\n")
    
file.close()

