import vision
import cv2


#
# Select player and turn
#
def player_selection():
    user_input = raw_input("Which player are you? (x/o): ")
    if user_input == 'x':
        print("...You are player x.")
    elif user_input == 'o':
        print("...You are player o.")
    user_input = raw_input("Who go first? (x/o): ")
    if user_input == 'x':
        print("...x goes first.")
    elif user_input == 'o':
        print("...o goes first.")

#
# Take reference photo
#
def reference_photo():
    user_input = raw_input("Remove all items from game table. Have you removed all items? (y/n): ")
    if user_input == 'y':
        print('...You have removed all items.')
        #image_ref = vision.get_from_webcam()
        image_ref = vision.get_from_file('images/baggrund.jpeg')
        regionOfInterest, (xc,yc), (x_start, y_start), (x_end, y_end), theta = vision.draw_regionOfInterest(image_ref)
        cv2.imshow('Region of interest reference image', regionOfInterest)
        cv2.waitKey(0)
    elif user_input == 'n':
        print("...You are player o.")

#
# Move arm to the starting position
#
def reference_position():
    print('set all joints to 0')

def process(step=0):
    while step == 0:
        reference_position()
        step=1
    while step == 1:
        reference_photo()
        step=2
    while step == 2:
        player_selection()
        

process(0)