#!/usr/bin/env python
import sys
import tty
import select

import rospy

from std_msgs.msg import Bool

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
    print("initializing game controls")
    rospy.init_node("GameControl")
    gamePub = rospy.Publisher("GameStart", Bool, queue_size=4)
    rospy.loginfo("Press space to start game. Make sure that the table is clear first, so we can take a picture of an empty table!")
    done = False
    while not done:
        
        key = getKey()
        if(key ==' '):
            rospy.loginfo("Start game signal sent!")
            gamePub.publish(Bool(True))
        if(key == 'q'):
            rospy.loginfo("game controls exit")
            done = True

