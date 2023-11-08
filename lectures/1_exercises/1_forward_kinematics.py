import numpy as np
import matplotlib as plt
from helpers import draw_planar_robot as draw_robo

# a: (fixed) length of robot joints
# q1: degree from 1st link to the base
# q2: degree from 2nd link to 1st link 
# See p20-23

def get_effector_pos(q1:float, q2:float, a1:float, a2:float):
    
    # Robot joint fixed lengths...

    # Coordinates of the end effector can be expressed as:
    # x = a1 * cos(q1) + a2 * cos(q1 + q2)
    # y = a1 * sin(q1) + a2 * sin(q1 + q2)

    x:float = a1 * np.cos(q1) + a2 * np.cos(q1 + q2)
    y:float = a1 * np.sin(q1) + a2 * np.sin(q1 + q2)

    return(x, y)

def rad_to_deg(r:float):
    return r * 180/np.pi

def get_robo_angles(x:float, y:float, a1:float, a2:float):
    
    D = np.double((np.square(x) + np.square(y) - np.square(a1) - np.square(a2))/(2*a1*a2))

    # q2 = np.arccos(D)
    

    q2 = np.arctan((np.sqrt(1 - D*D))/D)
    q3 = np.arctan2(np.sqrt(1 - D*D), D)
    print(q3)
    q2_2 = np.arctan(-np.sqrt((1 - D*D))/D)
    print(q2, q2_2)
    q1 = np.arctan2(y, x) - np.arctan((a2*np.sin(q2))/(a1 + a2*np.cos(q2)))

    return(q1, q2)

def test_fk():
    a1, a2 = 20, 30

    q1 = np.pi/4
    q2 = -0.5
    print(rad_to_deg(q1), rad_to_deg(q2))
    (x, y) = get_effector_pos(q1, q2, a1, a2)

    a = np.array([a1, a2])
    q = np.array([q1, q2])
    
    draw_robo(q, a, 1)

    (c, d) = get_robo_angles(x, y, a1, a2)
    print(rad_to_deg(c), rad_to_deg(d))
    

test_fk()
