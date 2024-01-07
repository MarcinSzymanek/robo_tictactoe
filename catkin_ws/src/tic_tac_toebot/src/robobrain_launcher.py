#!/usr/bin/env python
import rospy
import actionlib
from tic_tac_toebot.msg import ManualMoveTo
from std_msgs.msg import Bool
from time import sleep

from robobrain import Robobrain


# This is the launcher for our brain node. This is the one that reacts to explicit signals
# Fx, we can launch a game by sending a rosmsg to it... hopefully
class RoboBrainNode():
    def __init__(self):
        rospy.loginfo("Robobrain node constructor")
        self.brain = Robobrain()
        print("Robobrain activated")
        

if __name__ == "__main__":
    rospy.init_node("robobrain")
    node = RoboBrainNode()
    rospy.spin()


