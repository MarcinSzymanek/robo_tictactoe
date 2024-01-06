from enum import Enum
from tttgame import GameSession
from arm_controller import ArmController
from grip_controller import GripController
import traceback
from std_msgs.msg import Float64
from tic_tac_toebot.msg import ManualMoveTo
import rospy
from time import sleep
from vision import *

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
    BG_IMG_PATH = "session/baggrund.jpeg"
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
        
        self.gameStarted = False
        # Define states and their methods


        self.wait_start = Robostate(self.onWaitStart, STATES.wait_for_start)
        self.wait_turn = Robostate(self.onWaitTurn, STATES.wait_turn)
        self.calc_move = Robostate(self.calculateMove, STATES.calc_move)
        self.state = self.wait_start
        self.state.act()
    

    def onWaitStart(self):
        # Wait for a message that tells us to start: need to define w/ rosmsg
        # Simply put, do nothing

        # test evaluate board here

        # Check if the prints are correct and if the picture is saved 
        self.evaluateBoard()
        pass

    def onWaitTurn(self):
        your_turn = False
        while not your_turn:
            # Check the board, see if player has made a move
            self.evaluate_board()
            xs, os = self.session.count_pieces()
            # If there are more 'O' s on the board than Xs, it is our turn
            if(os > xs):
                your_turn = True
            # We check every 0.5 second by default
            sleep(0.5)
        self.state = self.calc_move
        self.state.act()

    def toggleGripper(self):
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
        self.arm.move_to_default()
        sleep(2)
        self.storeBackgroundImage()
    
    def storeBackgroundImage(self):
        try:
            bgimage = get_from_webcam()
            cv2.imwrite("baggrund.jpeg", bgimage)
        except:
            print("Erroring capturing background image!")

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
        
        image = get_from_webcam()
        image_ref = get_from_file('baggrund.jpeg')
        regionOfInterest, (xc,yc), (x_start, y_start), (x_end, y_end), theta = draw_regionOfInterest(image_ref)
        regionOfInterest2 = crop_image(image,xc,yc,x_start,y_start,x_end,y_end,theta)
        ref = regionOfInterest.astype(float)
        img = regionOfInterest2.astype(float)
        src = np.abs(img-ref).astype(np.uint8)
        pieces, o_pieces, x_pieces = find_pieces(src)
        cv2.imshow('Pieces', pieces)
        board, squares = draw_board(src)
        cv2.imshow('Board',board)
        tic_tac_toe_board = get_board_state(squares,o_pieces,x_pieces)
        
        # Keeps track of the board state
        self.session.set_board_state(tic_tac_toe_board)
        
        print(tic_tac_toe_board)
        h = src.shape[0]
        w = src.shape[1]
        print(w,h)
        print(o_pieces)
        self.x_pieces_coord = []
        self.y_pieces_coord = []
        for o_piece in o_pieces:
            x_coordinate, y_coordinate = from_image_to_coordinates(pieces,o_piece[0],o_piece[1])
            x_coordinate = x_coordinate/8.69832
            y_coordinate = y_coordinate/8.69832
            self.x_pieces_coord.append((x_coordinate, y_coordinate))
            print(x_coordinate,y_coordinate)
        print(x_pieces)
        for x_piece in x_pieces:
            x_coordinate, y_coordinate = from_image_to_coordinates(pieces,x_piece[0],x_piece[1])
            x_coordinate = x_coordinate/8.69832
            y_coordinate = y_coordinate/8.69832
            self.y_pieces_coord.append((x_coordinate, y_coordinate))
            print(x_coordinate,y_coordinate)

        

        
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
