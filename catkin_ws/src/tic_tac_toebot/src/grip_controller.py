import rospy
import std_msgs.msg
from time import sleep

class GripController:
    def __init__(self):
        self.publisher = rospy.Publisher("gripper/command", std_msgs.msg.Float64, queue_size=10)
        # Wait until the publisher is connected to the topic
        while(self.publisher.get_num_connections() is 0):
            sleep(0.100)

    def control_(self, val):
        self.publisher.publish(data=val)
	
    def grab(self):
        # Find the precise amount of grip value to grab a piece here
        pass

    def let_go(self):
        self.control_(0)