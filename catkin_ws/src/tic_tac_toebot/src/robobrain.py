from enum import Enum
from tttgame import GameSession
from arm_controller import ArmController
from grip_controller import GripController
import traceback
from std_msgs.msg import Float64
from tic_tac_toebot.msg import ManualMoveTo
import rospy
from time import sleep

STATES = Enum('Robostate', ['wait_for_start', 'wait_turn', 'calc_move', 'placing_piece'])

class Robostate():
    def __init__(self, action, state):
        self.action = action
        self.state = state

    def act(self):
        if(self.action == None): return
        self.action()


class ArmPosition:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def printpos(self):
        print(self.x, self.y, self.z)

class Robobrain():
    START_POS = ArmPosition(0, 0, 49)
    def __init__(self):
        self.state = STATES.wait_for_start
        self.session = GameSession()
        self.arm = ArmController()
        self.gripper = GripController()
        
        self.arm.move_to_default()
        self.arm_position = Robobrain.START_POS
        self.moveGripper(1.0)
        self.gripper_val = 1.0
        sleep(1)
        self.moveGripper(0.7)
        self.gripper_val = 0.7
        sleep(1)
        self.moveGripper(1.0)
        self.gripper_val = 1.0

        self.manualMoveListener = rospy.Subscriber("ManualMoveTo", ManualMoveTo, self.moveTo)
        self.manualMoveDirListener = rospy.Subscriber("ManualMoveDir", ManualMoveTo, self.moveInDirection)
        
        self.manualGripperListener = rospy.Subscriber("ManualGrip", Float64, self.arm.move_to_default())

        # Define states and their methods
        def onWaitStart():
            # Wait for a message that tells us to start: need to define w/ rosmsg
            pass
        self.wait_start = Robostate(onWaitStart, STATES.wait_for_start)
    

    def toggleGripper(self, msg):
        if(self.gripper_val > 0.0):
            self.moveGripper(0.0)
            self.gripper_val = 0.0
        else:
            self.moveGripper(0.7)
            self.gripper_val = 0.7

    def moveGripper(self, val):
        self.gripper.control_(val)
    
    def onGameStart(self):
        print("robobrain::onGameStart()")
        self.session.start_new()
        positions = [
            [0, 0, 49]
            # [-1, 0.4, 0.3],
            # [4, 0.8, 0.1]

        ]
        self.moveTo(positions)

    def moveInDirection(self, dir):
        try:
            newpos = ArmPosition(self.arm_position.x + dir.x, self.arm_position.y + dir.y, self.arm_position.z + dir.z)
            self.moveTo(newpos)
            print "Move in direction:"
            print "new pos: "
            newpos.printpos()
            self.arm_position = newpos

        except Exception as error:
            print("Error moving the arm in dir: " + traceback.format_exc())

    def moveTo(self, p):
        print("Sending position to arm controller")
        print(p)
        print(p.x, p.y, p.z)
        try:
            self.arm.set_goal([[p.x, p.y, p.z]])
        except Exception as error:
            print("Error moving the arm: " + traceback.format_exc())

    def evaluateBoard(self):
        # Use computer vision to check if there are any changes on the board
        # If so, change state to calc_move, run calculateMove change to placing_piece
        pass

    def calculateMove(self):
        # Figure out which move to make and return the position as a tuple
        return (0, 0)
    
    def placePiece(self, pos):
        # Send a message to robot movement controller with directions to the piece. 
        # We should have a method for finding the piece itself using CV
        # And another one for moving end effector to the piece and grabbing it
        pass

    def findPiece():
        # This should go into CV module, probably
        # How to communicate directions/necessary movements?
        pass

    def moveToPos(pos):
        # Move end effector to the desired position
        pass
