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

def from_pixel_to_m_coordinates(scale,x,y):
    x_coordinate_m = float(x)/(scale*100)
    y_coordinate_m = float(y)/(scale*100)
    return x_coordinate_m,y_coordinate_m


def black_out_white(image, blue_threshold=200, green_threshold=100, red_threshold=50):
    modified_image = image.astype(float)
    blue = modified_image[:, :, 0]
    green = modified_image[:, :, 1]
    red = modified_image[:, :, 2]
    mask = (blue > blue_threshold) & (green > green_threshold) & (red > red_threshold)
    modified_image[mask, 0] = 0
    modified_image[mask, 1] = 0
    modified_image[mask, 2] = 0
    modified_image = np.clip(modified_image, 0, 255)
    no_white = modified_image.astype(np.uint8)
    return no_white

def white_out_black(image):
    modified_image = image.astype(float)
    blue = modified_image[:, :, 0]
    green = modified_image[:, :, 1]
    red = modified_image[:, :, 2]
    mask = (blue != 0) & (green != 0) & (red != 0)
    modified_image[mask, 0] = 255
    modified_image[mask, 1] = 255
    modified_image[mask, 2] = 255
    modified_image = np.clip(modified_image, 0, 255)
    no_black = modified_image.astype(np.uint8)
    return no_black

def black_out_portion_of_gray(image, threshold, span):
    modified_image = image.astype(float)
    blue = modified_image[:, :, 0]
    green = modified_image[:, :, 1]
    red = modified_image[:, :, 2]
    mask = (blue > (threshold-span)) & (blue < (threshold+span)) & (green > (threshold-span)) & (green < (threshold+span)) & (red > (threshold-span)) & (red < (threshold+span))
    modified_image[mask, 0] = 0
    modified_image[mask, 1] = 0
    modified_image[mask, 2] = 0
    modified_image = np.clip(modified_image, 0, 255)
    no_gray = modified_image.astype(np.uint8)
    return no_gray

def black_out_all_grey(image,limit=30):
    no_gray = image.copy()
    color = list(range(256))
    for number in color:
        no_gray = black_out_portion_of_gray(no_gray,number,limit)
    return no_gray
    
def most_common_color(image):
    # Convert the image to a flat numpy array
    flat_image = image.reshape((-1, 3))

    # Use np.unique to get unique colors and their counts
    unique_colors, counts = np.unique(flat_image, return_counts=True)

    # Find the index of the color with the maximum count
    index_of_max_count = np.argmax(counts)

    # Retrieve the most common color
    most_common_color = unique_colors[index_of_max_count]

    return most_common_color

def get_clear_difference(image, scale=5):
    modified_image = image.astype(float)
    modified_image *= scale
    modified_image = np.clip(modified_image, 0, 255)
    clear_difference = modified_image.astype(np.uint8)
    return clear_difference

def rotate_image(image, angle, center=None):
    # Rotate the image around the center
    (h, w) = image.shape[:2]
    if center is None:
        center = (w // 2, h // 2)

    rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated_image = cv2.warpAffine(image, rotation_matrix, (w, h), flags=cv2.INTER_LINEAR)

    return rotated_image

def rotate_point(x, y, cx, cy, theta):
    x_prime = (x - cx) * math.cos(theta) - (y - cy) * math.sin(theta) + cx
    y_prime = (x - cx) * math.sin(theta) + (y - cy) * math.cos(theta) + cy
    return x_prime, y_prime

def draw_regionOfInterest(image):
    image_ref_copy = image.copy()
    gray_image = get_gray_image(image_ref)
    modified_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)  
    #cv2.imshow('gray image', gray_image)
    _, threshold = cv2.threshold(gray_image,40,255,cv2.THRESH_BINARY)
    #cv2.imshow('threshold', threshold)
    edge_image = cv2.Canny(threshold, 50, 150)
    #cv2.imshow('edges',edge_image)
    lines = cv2.HoughLines(edge_image, 1, np.pi / 180, 150)
    if lines is not None:
        for line in lines:
            rho, theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(gray_image, (x1, y1), (x2, y2), (0, 0, 0), 2)
    #cv2.imshow('lines',edge_image)
    _, threshold = cv2.threshold(gray_image,1,255,cv2.THRESH_BINARY)
    edge_image = cv2.Canny(threshold, 50, 150)
    #cv2.imshow('edges',edge_image)
    #_, _, bounding_boxes, _ = draw_rectangle(image_copy, min_height=340, max_height=360, min_width=560, max_width=590, kernel=3, threshold1=50, threshold2=150)
    _, contours, _ = cv2.findContours(edge_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    bounding_boxes = []
    for contour in contours:
        rect = cv2.minAreaRect(contour) 
        box = cv2.boxPoints(rect) 
        box = np.int0(box)
        if 560 < rect[1][0] < 590 and 340 < rect[1][1] < 360: 
            cv2.drawContours(modified_image, [box], 0, (0, 0, 255), 2)
            bounding_boxes.append(rect)
    for box in bounding_boxes:
        (xc, yc), (w, h), theta = box
        rotated_image = rotate_image(image_ref_copy, theta, (xc, yc))
        box_points = cv2.boxPoints(box) 
        box_points = np.int0(box_points)
        x_start, y_start = rotate_point(box_points[1][0], box_points[1][1], xc, yc, -np.radians(theta))
        x_start, y_start = int(math.ceil(x_start)), int(math.ceil(y_start))
        x_end, y_end = int(x_start+math.floor(w)), int(y_start+math.floor(h))
        regionOfInterest = rotated_image[y_start:y_end, x_start:x_end]

    return regionOfInterest, (xc,yc), (x_start, y_start), (x_end, y_end), theta

def crop_image(image, xc, yc, x_start, y_start, x_end, y_end, theta):

    rotated_image = rotate_image(image, theta, (xc, yc))
    cropped_image = rotated_image[y_start:y_end, x_start:x_end]

    return cropped_image


def draw_board(src):
    clear_difference = get_clear_difference(src,5)
    #cv2.imshow('gray image', gray_image)
    gray_image = get_gray_image(clear_difference)
    #cv2.imshow('gray image', gray_image)
    _, threshold = cv2.threshold(gray_image,40,255,cv2.THRESH_BINARY)
    #cv2.imshow('threshold', threshold)
    kernel = np.ones((3, 3), np.uint8)
    eroded_edges = cv2.erode(threshold, kernel, iterations=1)
    #cv2.imshow('image with erode', eroded_edges)
    dilated_edges = cv2.dilate(eroded_edges, kernel, iterations=2)
    #cv2.imshow('image with dilation', dilated_edges)

    edge_image = cv2.Canny(dilated_edges, 50, 150)
    #cv2.imshow('edge', edge_image)
    board = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR) 
    #modified_image = cv2.cvtColor(threshold, cv2.COLOR_GRAY2BGR)  
    lines = cv2.HoughLines(edge_image, 1, np.pi / 180, 50)
    if lines is not None:
        for line in lines:
            rho, theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(board, (x1, y1), (x2, y2), (0, 0, 0), 2)
    #cv2.imshow('lines',modified_image)
    _, threshold = cv2.threshold(board,1,255,cv2.THRESH_BINARY)
    edge_image = cv2.Canny(threshold, 50, 150)
    #cv2.imshow('lines',edge_image)

    _, contours, _ = cv2.findContours(edge_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    bounding_boxes = []
    for contour in contours:
        rect = cv2.minAreaRect(contour) 
        box = cv2.boxPoints(rect) 
        box = np.int0(box)
        if 170 < rect[1][0] < 185 and 170 < rect[1][1] < 185: 
            cv2.drawContours(board, [box], 0, (0, 0, 255), 2)
            bounding_boxes.append(rect)
    for box in bounding_boxes:
        (x,y), (w,h), theta = box
        print(box)

    squares=[]
    scale_center = 0.31
    scale_square = 0.255
    square_sidelength = (round(scale_square*(w+h)/2),round(scale_square*(w+h)/2))
    a = scale_center*(w+h)/2
    d = math.sqrt(a**2+a**2)
    
    squares.append([(rotate_point(x+d, y, x, y, -3*np.pi/4+np.radians(theta))), square_sidelength, theta])
    squares.append([(rotate_point(x+a, y, x, y, -np.pi/2+np.radians(theta))), square_sidelength, theta])
    squares.append([(rotate_point(x+d, y, x, y, -np.pi/4+np.radians(theta))), square_sidelength, theta])
    squares.append([(rotate_point(x+a, y, x, y, np.pi+np.radians(theta))), square_sidelength, theta])
    squares.append([(x,y), square_sidelength, theta])
    squares.append([(rotate_point(x+a, y, x, y, np.radians(theta))), square_sidelength, theta])
    squares.append([(rotate_point(x+d, y, x, y, 3*np.pi/4+np.radians(theta))), square_sidelength, theta])
    squares.append([(rotate_point(x+a, y, x, y, np.pi/2+np.radians(theta))), square_sidelength, theta])
    squares.append([(rotate_point(x+d, y, x, y, np.pi/4+np.radians(theta))), square_sidelength, theta])
    
    for square in squares:
        theta=np.degrees(theta)
        box = cv2.boxPoints(tuple(square))
        box = np.int0(box)
        cv2.drawContours(board, [box], 0, (0, 0, 255), 2)
        print(square)

    return board

def from_image_to_coordinates(image,x,y):
    h = image.shape[0]
    w = image.shape[1]
    print(h,w)
    if x < (w/2):
        y_coordinate = -(x-(w/2))
    elif x > (w/2):
        y_coordinate = -(x-(w/2))
    else:
        y_coordinate = 0
    x_coordinate = -(y-h)
    return x_coordinate, y_coordinate

#image = get_from_webcam()
#cv2.imwrite('images/onsdag4.jpeg', image)
image_ref = get_from_file('images/baggrund.jpeg')
image = get_from_file('images/onsdag2.jpeg')

regionOfInterest, (xc,yc), (x_start, y_start), (x_end, y_end), theta = draw_regionOfInterest(image_ref)
cv2.imshow('Region of interest reference image', regionOfInterest)
regionOfInterest2 = crop_image(image,xc,yc,x_start,y_start,x_end,y_end,theta)
cv2.imshow('Region of interest image', regionOfInterest2)
ref = regionOfInterest.astype(float)
img = regionOfInterest2.astype(float)
src = np.abs(img-ref).astype(np.uint8)
cv2.imshow('Image - reference image', src)
board = draw_board(src)
cv2.imshow('Board',board)
x_coordinate, y_coordinate = from_image_to_coordinates(src,60,342)
print(x_coordinate,y_coordinate)
x_coordinate_m, y_coordinate_m = from_pixel_to_m_coordinates(8,x_coordinate,y_coordinate)
print(x_coordinate_m,y_coordinate_m)
cv2.waitKey(0)
