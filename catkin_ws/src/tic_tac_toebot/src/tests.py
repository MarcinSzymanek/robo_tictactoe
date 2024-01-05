from tttgame import GameSession
import numpy as np
# Test that everything behaves correctly

session = GameSession()

session.start_new()

assert session.board_state.shape == (3, 3)
for i in range(3):
    for item in session.board_state[i]:
        assert item == ''

session.print_game_state()
session.update_board((1, 1), 'X')
assert session.board_state[1][1] == 'X'

session.print_game_state()

session.update_board((0, 1), 'O')

session.print_game_state()

session.update_board((1, 1), 'O')

session.update_board((1, 1), 'X')

session.update_board((0, 2), 'X')

session.print_game_state()

session.update_board((0, 0), 'O')

session.print_game_state()
session.update_board((2, 0), 'X')
session.print_game_state()

session.start_new()

def update_n_print(pos, val):
    session.update_board(pos, val)
    session.print_game_state()

session.print_game_state()
update_n_print((1, 1), 'X')
update_n_print((1, 2), 'O')
update_n_print((0, 1), 'X')
update_n_print((0, 2), 'O')
update_n_print((0, 0), 'X')
update_n_print((2, 2), 'O')