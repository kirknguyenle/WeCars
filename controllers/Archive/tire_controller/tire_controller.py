from controller import Supervisor,Robot, GPS, Motor, Node, TouchSensor
import sys
import numpy
import math
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


data = robot.getDevice('gps')
data.enable(TIME_STEP)


slipAngle = 0.0



coulomb_friction.setMFFloat(0,300)

sideslipConstant.setMFFloat(0, 1)
sideslipConstant.setMFFloat(1, 1)
vI = [1, 1, 0, 0, 0, 0]  
v2 = [1, -1, 0, 0, 0, 0]  
v3 = [1, 0, 0, 0, 0, 0]  

k = 1

motor = robot.getDevice('AxleMotor')  # get the motor device

v_last = 1

if motor == None:
    sys.exit(1)

motor.setPosition(float('inf')) 
motor.setVelocity(0.0)  # set the velocity to zero
i = 0
slip = 0.0
lslip =0.0
#tire_node.setVelocity(vI)

useMF = False

while robot.step(TIME_STEP) != -1:
    
    i +=1
    v_C = vI = [1, 0, 0, 0, 0, 0]
    v_C[1] = v_C[1]+ i/1000
    
    tire_node.setVelocity(v_C)
    
    absolute_v = tire_node.getVelocity()
    
    abs_vy = absolute_v[1]
    
    tire_node.setVelocity(vI)# roll forward
    if absolute_v[0] != 0.0:
        slip = numpy.arctan(abs_vy/absolute_v[0])
    
    if absolute_v[0] != 0.0:
        v_y = numpy.tan(slipAngle)*absolute_v[0]
    
    #print('wanted Vy ',v_y)
    #print('dv ', dv)
    print('slip \n', numpy.degrees(slip))
   
    print('abs_vy \n', abs_vy)
   
    print('lateral force \n', k*slip)
     
    S = numpy.tan(slip)/(k*slip*1)
    sideslipConstant.setMFFloat(1, S) 
    print('FDS dynamic \n',S)
  
