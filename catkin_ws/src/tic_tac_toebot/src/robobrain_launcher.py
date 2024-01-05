#!/usr/bin/env python
import rospy
import actionlib
import math
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Bool, Float64
from tic_tac_toebot.msg import ManualMoveTo

import moveit_msgs.msg

from robobrain import Robobrain

# This is our brain node. This is the one that reacts to explicit signals
# Fx, we can launch a game by sending a rosmsg to it... hopefully
class RoboBrainNode():
    def __init__(self):
        self.brain = Robobrain()
        self.display_traj_publisher = rospy.Publisher("/move_group/display_planned_path",
                                                      moveit_msgs.msg.DisplayTrajectory,
                                                      queue_size=20)
        print("Robobrain activated")

    def onGameStart(self, val):
        self.brain.onGameStart()
        print("HAHA I START")
        rospy.loginfo("Game start signal received")

    def moveTo(self, moveToMsg):
        x, y, z = moveToMsg.x, moveToMsg.y, moveToMsg.z
        print(x, " ",y , " ", z)
        
        self.brain.moveTo([[x, y, z]])
        

if __name__ == "__main__":
    rospy.init_node("robobrain")
    node = RoboBrainNode()
    rospy.spin()


