import numpy as np


# This class implements the logic that understands game state
# The board state is saved as a 3x3 matrix of chars, with state being 'X', 'O' or empty ''
# It also checks whether moves are valid and whether one of the players won
class GameSession:
    class Pieces:
        X = 'X'
        O = 'O'

    def __init__(self):
        self.current_player = GameSession.Pieces.X
        
    def start_new(self, player = 1):
        if(player != 1 and player != 2):
            print("Robot player must be 1 or 2! Got: ", player)
            return
        self.current_player = GameSession.Pieces.X
        self.clear_board()


    def clear_board(self):
        self.board_state = np.array([['', '', ''], ['', '', ''], ['', '', '']])

    def set_board_state(self, board):
        self.board_state = board

    # Count number of pieces on the board and return it as a tuple (X count, O count)
    def count_pieces(self):
        xs = 0
        os = 0
        for row in self.board_state:
            for item in row:
                if item == 'X':
                    xs = xs + 1
                elif item == 'O':
                    os = os + 1
        print("xs count: ", xs)
        print("os count: ", os)
        return (xs, os)


    def update_board(self, pos, val):
        if(self.current_player != val):
            print("Player tried to place: ", val, " but it's not his turn!")
            return False
        for item in pos:
            if(item > 2 or item < 0):
                print("Cannot update position", pos, " item ", item, " is not valid")
                return False
        if(self.board_state[pos[0], pos[1]] != ''):
            print("Cannot update position ", pos, " : Position occupied")
            return False
        self.board_state[pos[0], pos[1]] = val
        print("Player ", val, " placed a piece at ", pos)
        if(self.current_player == GameSession.Pieces.X):
            self.current_player = GameSession.Pieces.O
        else:
            self.current_player = GameSession.Pieces.X
        return True

    def print_board(self):
        print(self.board_state)

    def print_game_state(self):
        winner = self.check_winner()
        if(winner != ''):
            print("Player ", winner, " WON!!!!")
            self.print_board()
            return
        print("Current player turn: ", self.current_player)
        print("Game board: ")
        self.print_board()

    def check_winner(self):
        # Check diagonals
        if(self.board_state[0][0] == self.board_state[1, 1] and 
           self.board_state[1, 1] == self.board_state[2, 2] and
           self.board_state[0][0] != ''):
            return self.board_state[0][0]
        if(self.board_state[0][2] == self.board_state[1, 1] and 
           self.board_state[1, 1] == self.board_state[2, 0] and
           self.board_state[2][0] != ''):
            return self.board_state[0][2]
        
        # Check horizontals
        for i in range(3):
            if(self.board_state[i][1] == self.board_state[i][0] and
               self.board_state[i][1] == self.board_state[i][2]):
                return self.board_state[i][1]
        
        # Check verticals
        for i in range(3):
            if(self.board_state[1][i] == self.board_state[0][i] and
               self.board_state[1][i] == self.board_state[2][i]):
                return self.board_state[1][i]
        

        # If nobody won, return empty string
        return ''
        
