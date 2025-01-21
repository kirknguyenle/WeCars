from controller import Supervisor,Robot,Node
import numpy as np
import sys as s
import os.path
import math 
import json

#Instantiate Supervisor
robot = Supervisor()
car_node = robot.getFromDef('car')
children_field = car_node.getField('children')


#Read Trackwidth, WheelBase
#Read SuspensionData 
with open('C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\car_files\mr2.json', 'r') as f:
    data = json.load(f)

#Calculate "SingleSuspension" Width 

corner_width = data["wheel_anchor"][1]+data["tire_width"]
print(corner_width)
#Calculate "SingleSuspension" X Offset
x_offset = data["wheel_anchor"][0]
print(x_offset)
#Generate Min-Box (Box with mass of simulated car, allowing for 4 wheels to place wheels at edges)

length = data["wheel_base"]-2*x_offset
width = data["track_width"]-2*corner_width
height = 2/3

children_field.importMFNodeFromString(0, 'car_body { dimensions ',str(length),' ',str(width),' ',str(height),' car_mass', str(data["car_mass"]),' }')



#Generate Contact Properties

#Generate Front Left and place

children_field.importMFNodeFromString(0, 'SingleSuspension' )

#Generate Front Right, Flipping Y cords of suspension, then place

#Generate Rear Left,

#Generate Rear Right

#Done! 
