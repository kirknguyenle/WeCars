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
#Calculate "SingleSuspension" X Offset
x_offset = data["wheel_anchor"][0]
#Generate Min-Box (Box with mass of simulated car, allowing for 4 wheels to place wheels at edges)
length = data["wheelbase"]-2*x_offset 
print(length)
width = data["trackwidth"]-2*corner_width
print(width)
height = 2/3

#Generate Contact Properties

contact_field.importMFNodeFromString(0,'ContactProperties { material1 "frontleft_contact"}')
contact_field.importMFNodeFromString(0,'ContactProperties { material1 "frontright_contact"}')
contact_field.importMFNodeFromString(0,'ContactProperties { material1 "rearleft_contact"}')
contact_field.importMFNodeFromString(0,'ContactProperties { material1 "rearright_contact" }')


#Generate Front Left and place
'''
children_field.importMFNodeFromString(0, 'SingleSuspension {cornerID "frontleft"}')

frontLeft = robot.getFromDef('frontleft_corner_anchor')
if frontLeft == None:
    sys.exit(1)
fl_location = frontLeft.getField('cornerPose')
if fl_location == None:
    sys.exit(1)
fl_newLoc = [length/2, width/2, 0]
fl_location.setSFVec3f(fl_newLoc)

fl_UPF = frontLeft.getField('UpperFrontPickupPoint')
fl_UPR = frontLeft.getField('UpperRearPickupPoint')
fl_LWF = frontLeft.getField('LowerFrontPickupPoint')
fl_LWR = frontLeft.getField('LowerRearPickupPoint')
fl_TRP = frontLeft.getField('TieRodPickupPoint')
fl_TRAP = frontLeft.getField('TieRodAttachmentPoint')
fl_LBJ = frontLeft.getField('LowerBallJoint')
fl_UBJ = frontLeft.getField('UpperBallJoint')
fl_WHA = frontLeft.getField('WheelAnchor')

fl_UPF.setSFVec3f(data["upper_front_location"])
fl_UPR.setSFVec3f(data["upper_rear_location"])
fl_LWF.setSFVec3f(data["lower_front_location"])
fl_LWR.setSFVec3f(data["lower_rear_location"])
fl_TRP.setSFVec3f(data["tie_rod_pickup"])
fl_TRAP.setSFVec3f(data["tie_rod_upright_attachment"])
fl_LBJ.setSFVec3f(data["lower_balljoint"])
fl_UBJ.setSFVec3f(data["upper_balljoint"])
fl_WHA.setSFVec3f(data["wheel_anchor"])
'''




#Generate Front Right, Flipping Y cords of suspension, then place
children_field.importMFNodeFromString(0, 'SingleSuspension {cornerID "frontright"}')

frontRight = robot.getFromDef('frontright_corner_anchor')
fr_location = frontRight.getField('cornerPose')
fr_newLoc = [length/2, -width/2, 0]
fr_location.setSFVec3f(fr_newLoc)

fr_UPF = frontRight.getField('UpperFrontPickupPoint')
fr_UPR = frontRight.getField('UpperRearPickupPoint')
fr_LWF = frontRight.getField('LowerFrontPickupPoint')
fr_LWR = frontRight.getField('LowerRearPickupPoint')
fr_TRP = frontRight.getField('TieRodPickupPoint')
fr_TRAP = frontRight.getField('TieRodAttachmentPoint')
fr_LBJ = frontRight.getField('LowerBallJoint')
fr_UBJ = frontRight.getField('UpperBallJoint')
fr_WHA = frontRight.getField('WheelAnchor')
#fr_TRAP.setSFVec3f([-0.1,-0.2,0.05])
'''
fr_TRAP.setSFVec3f(cO.mirrorY(data["tie_rod_upright_attachment"]))
print(cO.mirrorY(data["tie_rod_upright_attachment"]))
fr_LBJ.setSFVec3f(cO.mirrorY(data["lower_balljoint"]))
fr_UBJ.setSFVec3f(cO.mirrorY(data["upper_balljoint"]))
fr_WHA.setSFVec3f(cO.mirrorY(data["wheel_anchor"]))
fr_UPF.setSFVec3f(cO.mirrorY(data["upper_front_location"]))
fr_UPR.setSFVec3f(cO.mirrorY(data["upper_rear_location"]))
fr_LWF.setSFVec3f(cO.mirrorY(data["lower_front_location"]))
fr_LWR.setSFVec3f(cO.mirrorY(data["lower_rear_location"]))
fr_TRP.setSFVec3f(cO.mirrorY(data["tie_rod_pickup"]))

'''

#fr_LBJ.setSFVec3f(m_LBJ)
#fr_TRP.setSFVec3f(m_TRP)
#fr_UPF.setSFVec3f(m_UPF)
#fr_UPR.setSFVec3f(m_UPR)
#fr_LWF.setSFVec3f(m_LWF)
#fr_LWR.setSFVec3f(m_LWR)
#fr_UBJ.setSFVec3f(m_UBJ)
#fr_TRAP.setSFVec3f(m_TRAP)
#fr_WHA.setSFVec3f(m_WHA)


#Generate Rear Left,
'''
children_field.importMFNodeFromString(0, 'SingleSuspension {cornerID "rearleft"}')

rearLeft = robot.getFromDef('rearleft_corner_anchor')
fl_location = rearLeft.getField('cornerPose')
fl_newLoc = [-length/2, width/2, 0]
fl_location.setSFVec3f(fl_newLoc)

fl_UPF = rearLeft.getField('UpperFrontPickupPoint')
fl_UPR = rearLeft.getField('UpperRearPickupPoint')
fl_LWF = rearLeft.getField('LowerFrontPickupPoint')
fl_LWR = rearLeft.getField('LowerRearPickupPoint')
fl_TRP = rearLeft.getField('TieRodPickupPoint')
fl_TRAP = rearLeft.getField('TieRodAttachmentPoint')
fl_LBJ = rearLeft.getField('LowerBallJoint')
fl_UBJ = rearLeft.getField('UpperBallJoint')
fl_WHA = rearLeft.getField('WheelAnchor')

fl_UPF.setSFVec3f(data["upper_front_location"])
fl_UPR.setSFVec3f(data["upper_rear_location"])
fl_LWF.setSFVec3f(data["lower_front_location"])
fl_LWR.setSFVec3f(data["lower_rear_location"])
fl_TRP.setSFVec3f(data["tie_rod_pickup"])
fl_TRAP.setSFVec3f(data["tie_rod_upright_attachment"])
fl_LBJ.setSFVec3f(data["lower_balljoint"])
fl_UBJ.setSFVec3f(data["upper_balljoint"])
fl_WHA.setSFVec3f(data["wheel_anchor"])'''
'''

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


        












        


