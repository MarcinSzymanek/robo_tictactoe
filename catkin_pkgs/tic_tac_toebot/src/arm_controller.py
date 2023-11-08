#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
import math
import moveit_commander
import moveit_msgs
import geometry_msgs
from math import pi
from moveit_commander.conversions import pose_to_list

class ArmController:
    N_JOINTS = 4
    def __init__(self,server_name):
        self.client = actionlib.SimpleActionClient(server_name, FollowJointTrajectoryAction)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("ax12_arm")
        self.joint_positions = []
        self.names =["joint1",
				"joint2",
				"joint3",
				"joint4"
				]
	
    # Just sends the goal to the server and prints some debug info
    def send_positions_(self):
		self.client.wait_for_server()
		print self.goal
		self.client.send_goal(self.goal)

		self.client.wait_for_result()
		print self.client.get_result()


    # Note: This actually sets several trajectories in a list
    def set_goal(self, positions, velocities=0.5):
        self.joint_positions = []
        dur = rospy.Duration(2)
        for p in positions:
			jtp = JointTrajectoryPoint(positions=p,velocities=[velocities]*self.N_JOINTS ,time_from_start=dur)
			dur += rospy.Duration(2)
			self.joint_positions.append(jtp)
        
        self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)
        self.goal = FollowJointTrajectoryGoal( trajectory = self.jt, goal_time_tolerance=rospy.Duration(2))
        self.send_positions_()

	
