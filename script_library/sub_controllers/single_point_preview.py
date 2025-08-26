import numpy as np
import math as m 


def steercmd(b, l, pos):
    ed = b[1]-pos[1]
    dist = b[0]-pos[0]+l
    steer = np.arctan(ed/dist)
    return steer