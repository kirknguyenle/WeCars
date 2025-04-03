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
timestep = int(robot.getBasicTimeStep())
root_node = robot.getRoot()
#car_node = robot.getFromDef('car')
root_field = root_node.getField('children')
#children_field = car_node.getField('children')
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
height = 0.2

#Generate Contact Properties

contact_field.importMFNodeFromString(0,'DEF frontleft_contact ContactProperties { material1 "frontleft_contact" softCFM 0.0001 softERP 0.1 coulombFriction 8 bounce 0.1 maxContactJoints 8 forceDependentSlip [0 0]}')
contact_field.importMFNodeFromString(0,'DEF frontright_contact ContactProperties { material1 "frontright_contact" softCFM 0.0001 softERP 0.1 coulombFriction 8 bounce 0.1 maxContactJoints 8 forceDependentSlip [0 0]}')
contact_field.importMFNodeFromString(0,'DEF rearleft_contact ContactProperties { material1 "rearleft_contact" softCFM 0.0001 softERP 0.1 coulombFriction 8 bounce 0.1 maxContactJoints 8 forceDependentSlip [0 0]}')
contact_field.importMFNodeFromString(0,'DEF rearright_contact ContactProperties { material1 "rearright_contact" softCFM 0.0001 softERP 0.1 coulombFriction 8 bounce 0.1 maxContactJoints 8 forceDependentSlip [0 0]}')

mUPF = data["upper_front_location"]
mUPR = data["upper_rear_location"]
mLWF = data["lower_front_location"]
mLWR = data["lower_rear_location"]
mTRP = data["tie_rod_pickup"]
mLBJ = data["lower_balljoint"]
mUBJ = data["upper_balljoint"]
mTRAP = data["tie_rod_upright_attachment"]
mWA = data["wheel_anchor"]
mSTR = data["mcpherson_strut_pickup"]


fUPF = cO.mirrorY(mUPF)
fUPR = cO.mirrorY(mUPR)
fLWF = cO.mirrorY(mLWF)
fLWR = cO.mirrorY(mLWR)
fTRP = cO.mirrorY(mTRP)
fLBJ = cO.mirrorY(mLBJ)
fUBJ = cO.mirrorY(mUBJ)
fTRAP = cO.mirrorY(mTRAP)
fWA = cO.mirrorY(mWA)
fSTR = cO.mirrorY(mSTR)
#Generate Front Left Parameters
fl_string = ('SingleSuspension {cornerID "frontleft"'+ 
                              ' cornerPose '+ str(length/2)+' '+ str(width/2)+' '+ str(-height/2)+
                              ' steerMotorName "steer_left"'+
                              ' UpperFrontPickupPoint '+ str(mUPF[0])+' '+ str(mUPF[1])+' '+ str(mUPF[2])+
                              ' UpperRearPickupPoint '+ str(mUPR[0])+' '+str(mUPR[1])+' '+str(mUPR[2])+
                              ' LowerFrontPickupPoint '+ str(mLWF[0])+' '+str(mLWF[1])+' '+str(mLWF[2])+
                              ' LowerRearPickupPoint '+ str(mLWR[0])+' '+str(mUPR[1])+' '+str(mLWR[2])+
                              ' TieRodPickupPoint '+ str(mTRP[0])+' '+str(mTRP[1])+' '+str(mTRP[2])+
                              ' TieRodAttachmentPoint '+ str(mTRAP[0])+' '+str(mTRAP[1])+' '+str(mTRAP[2])+
                              ' LowerBallJoint '+ str(mLBJ[0])+' '+str(mLBJ[1])+' '+str(mLBJ[2])+
                              ' UpperBallJoint '+ str(mUBJ[0])+' '+str(mUBJ[1])+' '+str(mUBJ[2])+
                              ' WheelAnchor '+ str(mWA[0])+' '+str(mWA[1])+' '+str(mWA[2])+
                              ' useMcphereson '+ str(data["use_mcpherson"]).upper()+
                              ' McpheresonStrutPickupPoint '+ str(mSTR[0])+' '+str(mSTR[1])+' '+str(mSTR[2])+
                              ' AntiRollBarStiffness ' + str(data["anti_rollbar_stiffness"])+
                              ' AntiRollBarLeverArm ' + str(data["anti_rollbar_lever_arm"])+
                              ' SpringRate ' + str(data["spring_rate"])+
                              ' DampingRate ' + str(data["damping_rate"])+
                              ' tireSectionWidth ' + str(data["tire_width"])+
                              ' tireRadius ' + str(data["tire_radius"])+
                              ' wheelMass ' + str(data["wheel_mass"])+
                              '}')

#Generate Front Right Parameters
fr_string = ('SingleSuspension {cornerID "frontright"'+ 
                              ' cornerPose '+ str(length/2)+' '+ str(-width/2)+' '+ str(-height/2)+
                              ' steerMotorName "steer_right"'+
                              ' UpperFrontPickupPoint '+ str(fUPF[0])+' '+ str(fUPF[1])+' '+ str(fUPF[2])+
                              ' UpperRearPickupPoint '+ str(fUPR[0])+' '+str(fUPR[1])+' '+str(fUPR[2])+
                              ' LowerFrontPickupPoint '+ str(fLWF[0])+' '+str(fLWF[1])+' '+str(fLWF[2])+
                              ' LowerRearPickupPoint '+ str(fLWR[0])+' '+str(fLWR[1])+' '+str(fLWR[2])+
                              ' TieRodPickupPoint '+ str(fTRP[0])+' '+str(fTRP[1])+' '+str(fTRP[2])+
                              ' TieRodAttachmentPoint '+ str(fTRAP[0])+' '+str(fTRAP[1])+' '+str(fTRAP[2])+
                              ' LowerBallJoint '+ str(fLBJ[0])+' '+str(fLBJ[1])+' '+str(fLBJ[2])+
                              ' UpperBallJoint '+ str(fUBJ[0])+' '+str(fUBJ[1])+' '+str(fUBJ[2])+
                              ' WheelAnchor '+ str(fWA[0])+' '+str(fWA[1])+' '+str(fWA[2])+
                              ' useMcphereson '+ str(data["use_mcpherson"]).upper()+
                              ' McpheresonStrutPickupPoint '+ str(fSTR[0])+' '+str(fSTR[1])+' '+str(fSTR[2])+
                              ' AntiRollBarStiffness ' + str(data["anti_rollbar_stiffness"])+
                              ' AntiRollBarLeverArm ' + str(data["anti_rollbar_lever_arm"])+
                              ' SpringRate ' + str(data["spring_rate"])+
                              ' DampingRate ' + str(data["damping_rate"])+
                              ' tireSectionWidth ' + str(data["tire_width"])+
                              ' tireRadius ' + str(data["tire_radius"])+
                              ' wheelMass ' + str(data["wheel_mass"])+
                              '}')

#Generate Rear Left Parameters
rl_string = ('SingleSuspension {cornerID "rearleft"'+ 
                              ' cornerPose '+ str(-length/2)+' '+ str(width/2)+' '+ str(-height/2)+
                              ' steerMotorName "rear_static_left"'+
                              ' UpperFrontPickupPoint '+ str(mUPF[0])+' '+ str(mUPF[1])+' '+ str(mUPF[2])+
                              ' UpperRearPickupPoint '+ str(mUPR[0])+' '+str(mUPR[1])+' '+str(mUPR[2])+
                              ' LowerFrontPickupPoint '+ str(mLWF[0])+' '+str(mLWF[1])+' '+str(mLWF[2])+
                              ' LowerRearPickupPoint '+ str(mLWR[0])+' '+str(mLWR[1])+' '+str(mLWR[2])+
                              ' TieRodPickupPoint '+ str(mTRP[0])+' '+str(mTRP[1])+' '+str(mTRP[2])+
                              ' TieRodAttachmentPoint '+ str(mTRAP[0])+' '+str(mTRAP[1])+' '+str(mTRAP[2])+
                              ' LowerBallJoint '+ str(mLBJ[0])+' '+str(mLBJ[1])+' '+str(mLBJ[2])+
                              ' UpperBallJoint '+ str(mUBJ[0])+' '+str(mUBJ[1])+' '+str(mUBJ[2])+
                              ' WheelAnchor '+ str(mWA[0])+' '+str(mWA[1])+' '+str(mWA[2])+
                              ' useMcphereson '+ str(data["use_mcpherson"]).upper()+
                              ' McpheresonStrutPickupPoint '+ str(mSTR[0])+' '+str(mSTR[1])+' '+str(mSTR[2])+
                              ' AntiRollBarStiffness ' + str(data["anti_rollbar_stiffness"])+
                              ' AntiRollBarLeverArm ' + str(data["anti_rollbar_lever_arm"])+
                              ' SpringRate ' + str(data["spring_rate"])+
                              ' DampingRate ' + str(data["damping_rate"])+
                              ' tireSectionWidth ' + str(data["tire_width"])+
                              ' tireRadius ' + str(data["tire_radius"])+
                              ' wheelMass ' + str(data["wheel_mass"])+
                              '}')

#Generate Rear Right Parameters
rr_string = ('SingleSuspension {cornerID "rearright"'+ 
                              ' cornerPose '+ str(-length/2)+' '+ str(-width/2)+' '+ str(-height/2)+
                              ' steerMotorName "rear_static_right"'+
                              ' UpperFrontPickupPoint '+ str(fUPF[0])+' '+ str(fUPF[1])+' '+ str(fUPF[2])+
                              ' UpperRearPickupPoint '+ str(fUPR[0])+' '+str(fUPR[1])+' '+str(fUPR[2])+
                              ' LowerFrontPickupPoint '+ str(fLWF[0])+' '+str(fLWF[1])+' '+str(fLWF[2])+
                              ' LowerRearPickupPoint '+ str(fLWR[0])+' '+str(fLWR[1])+' '+str(fLWR[2])+
                              ' TieRodPickupPoint '+ str(fTRP[0])+' '+str(fTRP[1])+' '+str(fTRP[2])+
                              ' TieRodAttachmentPoint '+ str(fTRAP[0])+' '+str(fTRAP[1])+' '+str(fTRAP[2])+
                              ' LowerBallJoint '+ str(fLBJ[0])+' '+str(fLBJ[1])+' '+str(fLBJ[2])+
                              ' UpperBallJoint '+ str(fUBJ[0])+' '+str(fUBJ[1])+' '+str(fUBJ[2])+
                              ' WheelAnchor '+ str(fWA[0])+' '+str(fWA[1])+' '+str(fWA[2])+
                              ' useMcphereson '+ str(data["use_mcpherson"]).upper()+
                              ' McpheresonStrutPickupPoint '+ str(fSTR[0])+' '+str(fSTR[1])+' '+str(fSTR[2])+
                              ' AntiRollBarStiffness ' + str(data["anti_rollbar_stiffness"])+
                              ' AntiRollBarLeverArm ' + str(data["anti_rollbar_lever_arm"])+
                              ' SpringRate ' + str(data["spring_rate"])+
                              ' DampingRate ' + str(data["damping_rate"])+
                              ' tireSectionWidth ' + str(data["tire_width"])+
                              ' tireRadius ' + str(data["tire_radius"])+
                              ' wheelMass ' + str(data["wheel_mass"])+
                              '}')

bdy_string = ('car_body { dimensions ' + str(length) + ' ' + str(width) + ' ' + str(height) +
            ' car_mass '+ str(data["car_mass"])+
            '}')

#Generate Car Body
root_field.importMFNodeFromString(0, bdy_string)

car_node = robot.getFromDef('car')
children_field = car_node.getField('extension')



#Generate Front Left
children_field.importMFNodeFromString(0, fl_string)

#Generate Front Right, Flipping Y cords of suspension, then place
children_field.importMFNodeFromString(0, fr_string)

#Generate Rear Left
children_field.importMFNodeFromString(0, rl_string)

#Generate Rear Right
children_field.importMFNodeFromString(0, rr_string)





#bounding = car_node.getField("boundingObject")
#bounding.setSFString("car_shape")

#Done! 


        












        


