#!/usr/bin/env python
import cv2
import urllib
import numpy as np
import math

RADIUS_THRESH = (15, 30)

def get_from_webcam():
    """
    Fetches an image from the webcam
    """
    print "try fetch from webcam..."
    stream=urllib.urlopen('http://192.168.0.20/image/jpeg.cgi')
    bytes=''
    bytes+=stream.read(64500)
    a = bytes.find('\xff\xd8')
    b = bytes.find('\xff\xd9')

    if a != -1 and b != -1:
        jpg = bytes[a:b+2]
        bytes= bytes[b+2:]
        i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.IMREAD_COLOR)

        return i

def get_from_file(filename):
    print "try fetch from file..."
    return cv2.imread(filename)


def get_gray_image(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return gray_image

def from_pixel_to_cm(image, scale):

    # Calculate the new dimensions
    width = image.shape[1] / scale
    height = image.shape[0] / scale

    # Resize the image
    image_cm = height, width, scale

    return image_cm

def draw_rectangle(image,min_height,max_height,min_width,max_width):
    # Change from BGR to gray
    src = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Change from gray to BGR
    rectangle = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
    edge_image = cv2.Canny(rectangle, threshold1=50, threshold2=255)
    _, contours, _ = cv2.findContours(edge_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    bounding_boxes = []
    for contour in contours:
        rect = cv2.minAreaRect(contour) 
        box = cv2.boxPoints(rect) 
        box = np.int0(box)
        if min_width < rect[1][0] < max_width and min_height < rect[1][1] < max_height: 
            cv2.drawContours(rectangle, [box], 0, (0, 0, 255), 2)
            bounding_boxes.append(rect)
    return edge_image, rectangle, bounding_boxes

def draw_coordinates(image):
    bounding_boxes=[]
    _, ROI, bounding_boxes =draw_rectangle(image,min_height=340,max_height=360,min_width=560,max_width=590)
    for box in bounding_boxes:
        (x,y), (w,h), theta = box
    orion = (int(x),int(y+h/2))

    return ROI, orion

def draw_board(image):
    bounding_boxes=[]
    _, board, bounding_boxes =draw_rectangle(image,min_height=175,max_height=180,min_width=175,max_width=180)
    for box in bounding_boxes:
        (x,y), (w,h), theta = box
    center_of_rotation = (x,y)

    squares=[]
    scale_center = 0.31633
    scale_square = 0.25
    square_sidelength = (round(scale_square*(w+h)/2),round(scale_square*(w+h)/2))
    square_center = round(scale_center*(w+h)/2)
    
    squares.append([(x - square_center, y - square_center), square_sidelength, theta])
    squares.append([(x, y - square_center), square_sidelength, theta])
    squares.append([(x + square_center, y - square_center), square_sidelength, theta])
    squares.append([(x - square_center, y), square_sidelength, theta])
    squares.append([(x, y), square_sidelength, theta])
    squares.append([(x + square_center, y), square_sidelength, theta])
    squares.append([(x - square_center, y + square_center), square_sidelength, theta])
    squares.append([(x, y + square_center), square_sidelength, theta])
    squares.append([(x + square_center, y + square_center), square_sidelength, theta])
    
    
    
    for square in squares:
        rotated_points = []
        for point in square[0]:
            dx = point[0] - center_of_rotation[0]
            dy = point[1] - center_of_rotation[1]
            rotated_x = int(center_of_rotation[0] + dx * np.cos(theta) - dy * np.sin(theta))
            rotated_y = int(center_of_rotation[1] + dx * np.sin(theta) + dy * np.cos(theta))
            rotated_points.append([rotated_x, rotated_y])

        rotated_square = [rotated_points, square[1], square[2]]
        box = cv2.boxPoints(tuple(rotated_square))
        box = np.int0(box)
        cv2.drawContours(board, [box], 0, (0, 0, 255), 2)

    return board

def find_white_elements(image):
    src = np.copy(image)
    src_gray = get_gray_image(src)
    # Define the lower and upper bounds for white color in BGR
    lower_white = np.array([100, 140, 160], dtype=np.uint8)
    upper_white = np.array([240, 240, 240], dtype=np.uint8)

    # Create a binary mask for red pixels
    white_mask = cv2.inRange(src, lower_white, upper_white)

    # Apply the mask to the original image
    white_elements = cv2.bitwise_and(src, src, mask=white_mask)


    #retval, white_elements = cv2.threshold(src_gray, 170, 255, cv2.THRESH_BINARY)


    return white_elements



def find_white_circles(binary_image, radius_threshold):

    src = binary_image.copy()
    src_gray = get_gray_image(src)
    #kernel =  np.ones((3,3),np.uint8)
    #img_ero = cv2.erode((white_circles*255).astype('uint8'), kernel)
    #img_dil = cv2.dilate((img_ero*255).astype('uint8'), kernel)
    #white_circles = img_dil.copy()
    white_circles_BGR = cv2.cvtColor(src_gray, cv2.COLOR_GRAY2BGR)

    # contours, hierarchy = cv2.findContours(white_circles, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    _ , contours, hierarchy = cv2.findContours(src_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Iterate through the contours
    for contour in contours:
        # Calculate the center and radius of the circle
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)

        # Check for a black hole in the middle
        # Assuming the center pixel represents the hole
        center_value = src_gray[int(y), int(x)]

        if center_value <= 220 and (radius_threshold[0] < radius < radius_threshold[1]):  # Assuming black holes have pixel value 0 (adjust if needed)
            # Draw the circle on the copy of the input image
            cv2.circle(white_circles_BGR, center, radius, (0, 0, 255), 2)

    return white_circles_BGR

image = get_from_webcam().astype(float)
image2 = get_from_file('baggrund.jpeg').astype(float)
foto= np.abs(image2-image).astype(np.uint8)

#image_cm = from_pixel_to_cm(image, 8.648)
#white_elements = find_white_elements(image)

#edge_image, rectangle, bounding_boxes = draw_rectangle(image,min_height=0,max_height=480,min_width=0,max_width=640)
#ROI, orion = draw_coordinates(image)
#board = draw_board(image)
#print("Pixel", image.shape)
#print("cm", image_cm)
#print("orion", orion)

#cv2.imshow("Det original billede", image)
#cv2.imshow('edge', edge_image)
cv2.imshow("Rectangles", foto)
#cv2.imshow('ROI', ROI)
#cv2.imshow("board", board)
#cv2.imwrite('.jpeg', image)
cv2.waitKey(0)
