import numpy as np
""""Shamelessly copied from the https://alexanderallenbrown.github.io/ES302_FA24_Students/05_Coordinates_Frames/ 
    Thank you so much for this Doc"""
def create3DRotationAB(theta,axis):
    """ Create a 3D rotation matrix AB_R (R from A to B ) to express the unit vectors of coordinate system B
        in terms of the unit vectors of coordinate system B for ONE of the principal axes 0,1,2 (x,y,z)
        If you wish to do the opposite (transform unit vectors of A into B's coordinates), you can invert/transpose this matrix.
        The angle you feed this function should be in RADIANS.
        note that the matrix equation here is [xhatb,yhatb,zhatb]' = R*[xhata,yhata,zhata]' where ' represents transpose.
    """
    if(axis==0):
        R = np.array([[1,0,0],[0,np.cos(theta),np.sin(theta)],[0,-np.sin(theta),np.cos(theta)]])
    elif(axis==1):
        R = np.array([[np.cos(theta),0,-np.sin(theta)],[0,1,0],[np.sin(theta),0,np.cos(theta)]])
    elif(axis==2):
        R = np.array([[np.cos(theta),np.sin(theta),0],[-np.sin(theta),np.cos(theta),0],[0,0,1]])
    else:
        raise Exception("invalid axis. use 0 for x, 1 for y, 2 for z")
    return R

def createEulerZYXRotation(roll,pitch,yaw):
    Rz = create3DRotationAB(yaw,2)
    Ry = create3DRotationAB(pitch,1)
    Rx = create3DRotationAB(roll,0)
    return np.dot(Rx,Ry,Rz)



