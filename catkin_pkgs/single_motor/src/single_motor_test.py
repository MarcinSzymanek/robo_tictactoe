#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from rosgraph_msgs import msg


class SingleMotorNode():
    
    def __init__(self):
        # msg.Log is too many parameters, let's find a different topic to publish to!
        self.publisher_out = rospy.Publisher("rosout", msg.Log)

if __name__ == "__main__":
    rospy.init_node("single_motor_node")
    node = SingleMotorNode()
    rospy.spin()
