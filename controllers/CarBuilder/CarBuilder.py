from controller import Supervisor,Robot,Node
import numpy as np
import sys as s
import os.path
import sys
import math 
import json
import carOrganizer as cO

#Instantiate Supervisor
robot = Supervisor()
car_node = robot.getFromDef('car')
children_field = car_node.getField('children')
world_node = robot.getFromDef('WorldInfo')
contact_field =  world_node.getField('contactProperties')
timestep = int(robot.getBasicTimeStep())
simtime = 0

#Read Trackwidth, WheelBase
#Read SuspensionData 
with open(r'C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\car_files\mr2.json') as f:
    data = json.load(f)

#Calculate "SingleSuspension" Width 

corner_width = data["wheel_anchor"][1]+data["tire_width"]
print(corner_width)
#Calculate "SingleSuspension" X Offset
x_offset = data["wheel_anchor"][0]
print(x_offset)
#Generate Min-Box (Box with mass of simulated car, allowing for 4 wheels to place wheels at edges)
length = data["wheelbase"]-2*x_offset
width = data["trackwidth"]-2*corner_width
height = 2/3


        
#Generate Contact Properties

contact_field.importMFNodeFromString(0,'ContactProperties { material1 "frontleft_contact"}')
contact_field.importMFNodeFromString(0,'ContactProperties { material1 "frontright_contact"}')
contact_field.importMFNodeFromString(0,'ContactProperties { material1 "rearleft_contact"}')
contact_field.importMFNodeFromString(0,'ContactProperties { material1 "rearright_contact" }')


#Generate Front Left and place

children_field.importMFNodeFromString(0, 'SingleSuspension {cornerID "frontleft"}')

frontLeft = robot.getFromDef('frontleft_corner_anchor')
if frontLeft == None:
    sys.exit(1)
FL_location = frontLeft.getField('cornerPose')
if FL_location == None:
    sys.exit(1)
FL_newLoc = [length/2, width/2, 0]
FL_location.setSFVec3f(FL_newLoc)

FL_UPF = frontLeft.getField('UpperFrontPickupPoint')
FL_UPR = frontLeft.getField('UpperRearPickupPoint')
FL_LWF = frontLeft.getField('LowerFrontPickupPoint')
FL_LWR = frontLeft.getField('LowerRearPickupPoint')
FL_TRP = frontLeft.getField('TieRodPickupPoint')
FL_TRAP = frontLeft.getField('TieRodAttachmentPoint')
FL_LBJ = frontLeft.getField('LowerBallJoint')
FL_UBJ = frontLeft.getField('UpperBallJoint')
FL_WHA = frontLeft.getField('WheelAnchor')

FL_UPF.setSFVec3f(data["upper_front_location"])
FL_UPR.setSFVec3f(data["upper_rear_location"])
FL_LWF.setSFVec3f(data["lower_front_location"])
FL_LWR.setSFVec3f(data["lower_rear_location"])
FL_TRP.setSFVec3f(data["tie_rod_pickup"])
FL_TRAP.setSFVec3f(data["tie_rod_upright_attachment"])
FL_LBJ.setSFVec3f(data["lower_balljoint"])
FL_UBJ.setSFVec3f(data["upper_balljoint"])
FL_WHA.setSFVec3f(data["wheel_anchor"])



'''while robot.step(timestep) != -1 :
    simtime += timestep/100
    if simtime < 2:'''





#Generate Front Right, Flipping Y cords of suspension, then place
children_field.importMFNodeFromString(0, 'SingleSuspension {cornerID "frontright" }')

frontRight = robot.getFromDef('frontright_corner_anchor')

FR_location = frontRight.getField('cornerPose')
FR_newLoc = [length/2, -width/2, 0]
FR_location.setSFVec3f(FR_newLoc)

FR_UPF = frontRight.getField('UpperFrontPickupPoint')
FR_UPR = frontRight.getField('UpperRearPickupPoint')
FR_LWF = frontRight.getField('LowerFrontPickupPoint')
FR_LWR = frontRight.getField('LowerRearPickupPoint')
FR_TRP = frontRight.getField('TieRodPickupPoint')
FR_TRAP = frontRight.getField('TieRodAttachmentPoint')
FR_LBJ = frontRight.getField('LowerBallJoint')
FR_UBJ = frontRight.getField('UpperBallJoint')
FR_WHA = frontRight.getField('WheelAnchor')

m_UPF = cO.mirrorY(data["upper_front_location"])
m_UPR = cO.mirrorY(data["upper_rear_location"])
m_LWF = cO.mirrorY(data["lower_front_location"])
m_LWR = cO.mirrorY(data["lower_rear_location"])
m_TRP = cO.mirrorY(data["tie_rod_pickup"])
m_TRAP = cO.mirrorY(data["tie_rod_upright_attachment"])
print(m_TRAP)
m_LBJ = cO.mirrorY(data["lower_balljoint"])
m_UBJ = cO.mirrorY(data["upper_balljoint"])
m_WHA = cO.mirrorY(data["wheel_anchor"])

FR_TRP.setSFVec3f(m_TRP)
FR_UPF.setSFVec3f(m_UPF)
FR_UPR.setSFVec3f(m_UPR)
FR_LWF.setSFVec3f(m_LWF)
FR_LWR.setSFVec3f(m_LWR)
FR_LBJ.setSFVec3f(m_LBJ)
FR_UBJ.setSFVec3f(m_UBJ)
#FR_TRAP.setSFVec3f(m_TRAP)
FR_LBJ.setSFVec3f(m_LBJ)
FR_UBJ.setSFVec3f(m_UBJ)
FR_WHA.setSFVec3f(m_WHA)


#Generate Rear Left,
'''children_field.importMFNodeFromString(0, 'SingleSuspension {cornerID "rearleft"}')

rearLeft = robot.getFromDef('rearleft_corner_anchor')
RL_location = rearLeft.getField('cornerPose')
RL_newLoc = [-length/2, width/2, 0]
RL_location.setSFVec3f(RL_newLoc)

RL_UPF = rearLeft.getField('UpperFrontPickupPoint')
RL_UPR = rearLeft.getField('UpperRearPickupPoint')
RL_LWF = rearLeft.getField('LowerFrontPickupPoint')
RL_LWR = rearLeft.getField('LowerRearPickupPoint')
RL_TRP = rearLeft.getField('TieRodPickupPoint')
RL_TRAP = rearLeft.getField('TieRodAttachmentPoint')
RL_LBJ = rearLeft.getField('LowerBallJoint')
RL_UBJ = rearLeft.getField('UpperBallJoint')
RL_WHA = rearLeft.getField('WheelAnchor')

RL_UPF.setSFVec3f(data["upper_front_location"])
RL_UPR.setSFVec3f(data["upper_rear_location"])
RL_LWF.setSFVec3f(data["lower_front_location"])
RL_LWR.setSFVec3f(data["lower_rear_location"])
RL_TRP.setSFVec3f(data["tie_rod_pickup"])
RL_TRAP.setSFVec3f(data["tie_rod_upright_attachment"])
RL_LBJ.setSFVec3f(data["lower_balljoint"])
RL_UBJ.setSFVec3f(data["upper_balljoint"])
RL_WHA.setSFVec3f(data["wheel_anchor"])


#Generate Rear Right
children_field.importMFNodeFromString(0, 'SingleSuspension {cornerID "rearright"}')

rearRight = robot.getFromDef('rearright_corner_anchor')
RR_location = rearRight.getField('cornerPose')
RR_newLoc = [-length/2, -width/2, 0]
RR_location.setSFVec3f(RR_newLoc)
RR_UPF = rearRight.getField('UpperFrontPickupPoint')
RR_UPR = rearRight.getField('UpperRearPickupPoint')
RR_LWF = rearRight.getField('LowerFrontPickupPoint')
RR_LWR = rearRight.getField('LowerRearPickupPoint')
RR_TRP = rearRight.getField('TieRodPickupPoint')
RR_TRAP = rearRight.getField('TieRodAttachmentPoint')
RR_LBJ = rearRight.getField('LowerBallJoint')
RR_UBJ = rearRight.getField('UpperBallJoint')
RR_WHA = rearRight.getField('WheelAnchor') 

RR_UPF.setSFVec3f(cO.mirrorY(data["upper_front_location"]))
RR_UPR.setSFVec3f(cO.mirrorY(data["upper_rear_location"]))
RR_LWF.setSFVec3f(cO.mirrorY(data["lower_front_location"]))
RR_LWR.setSFVec3f(cO.mirrorY(data["lower_rear_location"]))
RR_TRP.setSFVec3f(cO.mirrorY(data["tie_rod_pickup"]))
RR_TRAP.setSFVec3f(cO.mirrorY(data["tie_rod_upright_attachment"]))
RR_LBJ.setSFVec3f(cO.mirrorY(data["lower_balljoint"]))
RR_UBJ.setSFVec3f(cO.mirrorY(data["upper_balljoint"]))
RR_WHA.setSFVec3f(cO.mirrorY(data["wheel_anchor"]))

'''







'''

use_strut = data("use_mcpherson")

children_field.insertMFBool("useMcphereson", use_strut)
children_field.insertMFFloat("SpringRate", data("spring_rate"))
children_field.insertMFFloat("DampingRate", data("damping_rate"))
children_field.insertMFFloat("AntiRollBarStiffness", data("anti_rollbar_stiffness"))
children_field.insertMFFloat("AntiRollBarLeverArm", data("anti_rollbar_lever_arm"))
children_field.insertMFFloat("wheelMass", data("wheel_mass"))
children_field.insertMFFloat("tireRadius", data("tire_radius"))
children_field.insertMFFloat("tireSectionWidth", data("tire_width"))

children_field.importMFNodeFromString(0, 'car_body { dimensions ',str(length),' ',str(width),' ',str(height),' car_mass ', str(data["car_mass"]),' }')

'''
#Done! 


        












        


