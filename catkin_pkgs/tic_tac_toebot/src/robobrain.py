from enum import Enum
from tttgame import GameSession
from arm_controller import ArmController




STATES = Enum('Robostate', ['wait_for_start', 'wait_turn', 'calc_move', 'placing_piece'])

class Robostate():
    def __init__(self, action, state):
        self.action = action
        self.state = state

    def act(self):
        if(self.action == None): return
        self.action()


class Robobrain():
    def __init__(self):
        self.state = STATES.wait_for_start
        self.session = GameSession()
        self.arm = ArmController("/arm_controller/follow_joint_trajectory")
        
        # Define states and their methods
        def onWaitStart():
            # Wait for a message that tells us to start: need to define w/ rosmsg
            pass
        self.wait_start = Robostate(onWaitStart, STATES.wait_for_start)
    
    def onGameStart(self):
        print("robobrain::onGameStart()")
        self.session.start_new()
        positions = [
            [1, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 0]
        ]
        print("Sending position to arm controller")
        self.arm.set_goal(positions)

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
