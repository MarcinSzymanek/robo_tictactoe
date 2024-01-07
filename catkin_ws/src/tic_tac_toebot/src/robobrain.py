from enum import Enum
from tttgame import GameSession
from arm_controller import ArmController
from grip_controller import GripController
import traceback
from std_msgs.msg import Float64, Bool
from tic_tac_toebot.msg import ManualMoveTo, StartGame
import rospy
from time import sleep
from vision import *
import random

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
        print("Hello from robobrain!")
        self.state = STATES.wait_for_start
        self.session = GameSession()
        self.arm = ArmController()
        self.gripper = GripController()
        
        self.arm.move_to_default()
        self.arm_position = Robobrain.START_POS
        self.moveGripper(0.0)
        sleep(1)
        self.moveGripper(0.7)
        sleep(1)
        self.moveGripper(0.0)

        self.manualMoveListener = rospy.Subscriber("ManualMoveTo", ManualMoveTo, self.moveTo)
        self.manualMoveDirListener = rospy.Subscriber("ManualMoveDir", ManualMoveTo, self.moveInDirection)
        self.manualGripperListener = rospy.Subscriber("ManualGrip", Float64, self.toggleGripper)

        self.gameStartListener = rospy.Subscriber("GameStart", Bool, self.onGameStart)

        self.gameStarted = False
        # Define states and their methods


        self.wait_start = Robostate(self.onWaitStart, STATES.wait_for_start)
        self.wait_turn = Robostate(self.onWaitTurn, STATES.wait_turn)
        self.calc_move = Robostate(self.calculateMove, STATES.calc_move)
        self.state = self.wait_start
        self.state.act()
    

    def onWaitStart(self):
        print("Waiting to start game")
        # Wait for a message that tells us to start: need to define w/ rosmsg
        # Simply put, do nothing
        pass

    def onWaitTurn(self):
        self.arm.move_to_default()
        your_turn = False
        while not your_turn:
            # Check the board, see if player has made a move
            self.evaluateBoard()
            xs, os = self.session.count_pieces()
            # If there are more 'O' s on the board than Xs, it is our turn
            if(os > xs):
                your_turn = True
            # We check every 1 second by default
          
            sleep(1)
        self.state = self.calc_move
        self.state.act()

    def calculateMove(self):
        # Figure out which move to make and return the position as a tuple
        target_square_idx = random.randint(0, len(self.squares_coord) - 1)
        target_piece_idx = random.randint(0, len(self.x_pieces_coord) - 1)

        sleep(0.5)

        x, y = self.x_pieces_coord[target_piece_idx][0], self.x_pieces_coord[target_piece_idx][1]        

        target = ManualMoveTo(x, y, 20)        
        self.moveTo(target)
        sleep(1)
        target = ManualMoveTo(x, y, 2)
        self.moveTo(target)
        sleep(1)
        self.toggleGripper()
        sleep(1)
        self.moveTo(ManualMoveTo(x, y, 12))
        sleep(1)
        x, y = self.squares_coord[target_square_idx][0], self.squares_coord[target_square_idx][1]

        self.moveTo(ManualMoveTo(x, y, 14))
        sleep(1)
        self.moveTo(ManualMoveTo(x, y, 4))
        sleep(1)
        self.toggleGripper() 
        sleep(1.5)
        self.arm.set_goal([[0, 5.5, 55]])
        sleep(1.5)
        self.state = self.wait_turn
        self.state.act()
        #target = ManualMoveTo(x, y, 20)   
        #self.moveTo(target)


    def toggleGripper(self):
        if(self.gripper_val > 0.0):
            self.moveGripper(0.0)
            self.gripper_val = 0.0
        else:
            self.moveGripper(0.7)
            self.gripper_val = 0.7

    def moveGripper(self, val):
        self.gripper.control_(val)
        self.gripper_val = val
    
    def onGameStart(self, val):
        print("robobrain::onGameStart()")
        self.session.start_new()    
        self.arm.move_to_default()
        sleep(1)
        self.storeBackgroundImage()
        self.state = self.wait_turn
        self.state.act()
    
    def storeBackgroundImage(self):
        bgimage = get_from_webcam()
        cv2.imwrite("baggrund.jpeg", bgimage)


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
        try:
            image = get_from_webcam()
        except Exception as error:
            print("something went wrong!")
            print(error)
            return
        image_ref = get_from_file('baggrund.jpeg')
        print("saa langt saa godt")
        regionOfInterest, (xc,yc), (x_start, y_start), (x_end, y_end), theta = draw_regionOfInterest(image_ref)
        regionOfInterest2 = crop_image(image,xc,yc,x_start,y_start,x_end,y_end,theta)
        ref = regionOfInterest.astype(float)
        img = regionOfInterest2.astype(float)
        src = np.abs(img-ref).astype(np.uint8)
        pieces, o_pieces, x_pieces = find_pieces(src)
        
        board, squares = draw_board(src)

        cv2.imshow("board", board)
        cv2.waitKey(0)
        cv2.imshow("pieces", pieces)
        cv2.waitKey(0)
        tic_tac_toe_board, empty_squares_list, o_on_board_list, x_on_board_list = get_board_state(squares,o_pieces,x_pieces)
        
        
        # Keeps track of the board state
        self.session.set_board_state(tic_tac_toe_board)
        self.squares_coord = []
        self.o_pieces_coord = []
        self.x_pieces_coord = []

        print(tic_tac_toe_board)
        # h = src.shape[0]
        # w = src.shape[1]
        # print(w,h)
        # print(o_pieces)
        
        print('empty squares')
        for square in empty_squares_list:
            x_coordinate, y_coordinate = from_image_to_coordinates(board, square[0],square[1])
            x_coordinate = x_coordinate/8.69832
            y_coordinate = y_coordinate/8.69832
            self.squares_coord.append((x_coordinate, y_coordinate))
            print(x_coordinate,y_coordinate)
        print('o pieces')
        for o_piece in o_pieces:
            if(o_piece in o_on_board_list):
                continue
            x_coordinate, y_coordinate = from_image_to_coordinates(pieces,o_piece[0],o_piece[1])
            x_coordinate = x_coordinate/8.69832
            y_coordinate = y_coordinate/8.69832
            self.o_pieces_coord.append((x_coordinate, y_coordinate))
            print(x_coordinate,y_coordinate)
        print('x pieces')
        for x_piece in x_pieces:
            if(x_piece in x_on_board_list):
                continue
            x_coordinate, y_coordinate = from_image_to_coordinates(pieces,x_piece[0],x_piece[1])
            x_coordinate = x_coordinate/8.69832
            y_coordinate = y_coordinate/8.69832
            self.x_pieces_coord.append((x_coordinate, y_coordinate))
            print(x_coordinate,y_coordinate)
