#!/usr/bin/env python
import rospy

from tic_tac_toebot.msg import StartGame

class TestNode():
    def __init__(self):
        rospy.loginfo("Test node constructor")
        self.manualMoveListener = rospy.Subscriber("ManualMoveTo", StartGame, self.onGameStart)
        print("Robobrain activated")

    def onGameStart(self, val):
        rospy.loginfo("Game start signal received")

        

if __name__ == "__main__":
    print("initializing robobrain node")
    print("Hello")
    rospy.init_node("ttttest")
    print("rospy init")
    node = TestNode()
    rospy.spin()
