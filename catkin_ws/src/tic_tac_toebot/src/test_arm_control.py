#!/usr/bin/env python
import rospy

from time import sleep
import keyboard
import os
import tty
import termios
import sys
import select

from tic_tac_toebot.msg import ManualMoveTo
from std_msgs.msg import Float64

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    rospy.init_node("ManualArmControl")
    
    mag = 1.0

    armPub = rospy.Publisher("ManualMoveDir", ManualMoveTo, queue_size=4)
    gripPub = rospy.Publisher("ManualGrip", Float64, queue_size=1)
    while armPub.get_num_connections() == 0:
        print("Waiting for publisher to connect")
        sleep(0.100)
    while gripPub.get_num_connections() == 0:
        print("Waiting for publisher to connect")
        sleep(0.100)
    

    done = False
    while not done:
        
        key = getKey()
        if(key == 'w' or key == 's' or key == 'a' or key == 'd' or key == 'k' or key == 'j' or key ==' '):
            dir = ManualMoveTo(x = 0, y = 0, z = 0)
            if(key == 'w'):
                dir.y = mag
            elif(key == 's'):
                dir.y = -mag
            elif(key == 'a'):
                dir.x = -mag
            elif(key == 'd'):
                dir.x = mag
            elif(key == 'k'):
                dir.z = 1.0
            elif (key == 'j'):
                dir.z = -1.0
            elif (key == ' '):
                print("launch gripper!")
                msg = Float64(data=1.0)
                gripPub.publish(msg)
            
            armPub.publish(dir)
        if(key == 'q'):
            done = True
