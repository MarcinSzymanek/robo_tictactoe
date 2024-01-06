#!/usr/bin/env python
import cv2
import urllib
import numpy as np
import math

RADIUS_THRESH = (15, 30)

def get_from_webcam():
    print('try fetch from webcam...')
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
    print('try fetch from file...')
    return cv2.imread(filename)

def from_pixel_to_cm(image, scale):
    width = image.shape[1] / scale
    height = image.shape[0] / scale
    image_cm = height, width, scale
    return image_cm

def from_pixel_to_m_coordinates(scale,x,y):
    x_coordinate_m = float(x)/(scale*100)
    y_coordinate_m = float(y)/(scale*100)
    return x_coordinate_m,y_coordinate_m

def split_up_image_color(image):
    modified_image = image.astype(float)
    blue = modified_image[:, :, 0]
    green = modified_image[:, :, 1]
    red = modified_image[:, :, 2]
    return modified_image, blue, green, red

def set_new_color_parameter(modified_image, mask, blue=0, green=0, red=0):
    modified_image[mask, 0] = blue
    modified_image[mask, 1] = green
    modified_image[mask, 2] = red
    modified_image = np.clip(modified_image, 0, 255)
    return modified_image

def black_out_white(image, blue_threshold=200, green_threshold=100, red_threshold=50):
    modified_image, blue, green, red = split_up_image_color(image)
    mask = (blue > blue_threshold) & (green > green_threshold) & (red > red_threshold)
    modified_image = set_new_color_parameter(modified_image, mask, 0, 0, 0)
    no_white = modified_image.astype(np.uint8)
    return no_white

def black_out_black(image, blue_threshold=200, green_threshold=100, red_threshold=50):
    modified_image, blue, green, red = split_up_image_color(image)
    mask = (blue < blue_threshold) | (green < green_threshold) | (red < red_threshold)
    modified_image = set_new_color_parameter(modified_image, mask, 0, 0, 0)
    no_black = modified_image.astype(np.uint8)
    return no_black

def black_out_portion_of_gray(image, threshold, span):
    modified_image, blue, green, red = split_up_image_color(image)
    mask = (blue > (threshold-span)) & (blue < (threshold+span)) & (green > (threshold-span)) & (green < (threshold+span)) & (red > (threshold-span)) & (red < (threshold+span))
    modified_image = set_new_color_parameter(modified_image, mask, 0, 0, 0)
    no_gray = modified_image.astype(np.uint8)
    return no_gray

def black_out_all_grey(image,limit=30):
    no_gray = image.copy()
    color = list(range(256))
    for number in color:
        no_gray = black_out_portion_of_gray(no_gray,number,limit)
    return no_gray

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
    image_copy = image.copy()
    _, edge_image, dst_image = find_edges(image,0,40,255,0,0,50,150)
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
            cv2.line(dst_image, (x1, y1), (x2, y2), (0, 0, 0), 2)
    _, edge_image, dst_image = find_edges(dst_image,0,1,255,0,0,50,150) 
    _, contours, _ = cv2.findContours(edge_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    bounding_boxes = []
    for contour in contours:
        rect = cv2.minAreaRect(contour) 
        box = cv2.boxPoints(rect) 
        box = np.int0(box)
        if 560 < rect[1][0] < 590 and 340 < rect[1][1] < 360: 
            cv2.drawContours(dst_image, [box], 0, (0, 0, 255), 2)
            bounding_boxes.append(rect)
    for box in bounding_boxes:
        (xc, yc), (w, h), theta = box
        rotated_image = rotate_image(image_copy, theta, (xc, yc))
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

def find_edges(image,blur_kernel,threshold,color,erode_iterations,dilate_iterations,canny_thres1, canny_thres2):
    grey_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dst_image = cv2.cvtColor(grey_image, cv2.COLOR_GRAY2BGR) 
    if blur_kernel == 0:
        blur_image = grey_image
    else:
        kernel_blur = (blur_kernel,blur_kernel)
        blur_image = cv2.GaussianBlur(grey_image,kernel_blur,0)
    _, threshold = cv2.threshold(blur_image,threshold,color,cv2.THRESH_BINARY)
    kernel = np.ones((3, 3), np.uint8)
    if erode_iterations == 0:
        eroded_edges = threshold
    else: 
        eroded_edges = cv2.erode(threshold, kernel, erode_iterations)
        #cv2.imshow('eroded_edges', eroded_edges)
    if dilate_iterations == 0:
        dilated_edges = eroded_edges
    else:
        dilated_edges = cv2.dilate(eroded_edges, kernel, dilate_iterations)
        #cv2.imshow('dilated_edges', dilated_edges)
    edge_image = cv2.Canny(dilated_edges, canny_thres1, canny_thres2)
    return grey_image, edge_image, dst_image

def draw_board(src):
    clear_difference = get_clear_difference(src,5)
    #cv2.imshow('clear image', clear_difference)
    #_, edge_image, board = find_edges(clear_difference,40,255,1,2,50,150)
    gray_image = cv2.cvtColor(clear_difference, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('gray image', gray_image)
    _, threshold = cv2.threshold(gray_image,50,255,cv2.THRESH_BINARY)
    #cv2.imshow('threshold', threshold)
    kernel = np.ones((3, 3), np.uint8)
    eroded_edges = cv2.erode(threshold, kernel, iterations=1)
    #cv2.imshow('image with erode', eroded_edges)
    dilated_edges = cv2.dilate(eroded_edges, kernel, iterations=2)
    #cv2.imshow('image with dilation', dilated_edges)
    edge_image = cv2.Canny(dilated_edges, 50, 150)
    #cv2.imshow('edge', edge_image)
    board = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR) 
    board_copy = board.copy()
    #modified_image = cv2.cvtColor(threshold, cv2.COLOR_GRAY2BGR)  
    lines = cv2.HoughLines(edge_image, 1, np.pi / 180, 50)
    if lines is None:
        print('No lines found.')
    else: 
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
            cv2.line(board_copy, (x1, y1), (x2, y2), (0, 0, 0), 2)
    #cv2.imshow('lines',modified_image)
    #_, edge_image, _ = find_edges(board_copy,1,255,0,0,50,150)
    _, threshold = cv2.threshold(board_copy,1,255,cv2.THRESH_BINARY)
    edge_image = cv2.Canny(threshold, 50, 150)
    #cv2.imshow('lines',edge_image)

    _, contours, _ = cv2.findContours(edge_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    bounding_boxes = []
    squares=[]
    if not contours:
        print('No contours found.')
    else:
        for contour in contours:
            rect = cv2.minAreaRect(contour) 
            box = cv2.boxPoints(rect) 
            box = np.int0(box)
            if 170 < rect[1][0] < 185 and 170 < rect[1][1] < 185: 
                cv2.drawContours(board, [box], 0, (0, 0, 255), 2)
                bounding_boxes.append(rect)
        if not bounding_boxes:
            print('No board found.')
        else:
            for box in bounding_boxes:
                (x,y), (w,h), theta = box
            #print(box)
            
            scale_center = 0.31
            scale_square = 0.255
            square_sidelength = (round(scale_square*(w+h)/2),round(scale_square*(w+h)/2))
            a = round(scale_center*(w+h)/2)
            d = round(math.sqrt(a**2+a**2))
            
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
                theta = np.degrees(theta)
                box = cv2.boxPoints(tuple(square))
                box = np.int0(box)
                cv2.drawContours(board, [box], 0, (0, 0, 255), 2)
                #print(square)

    

    return board, squares



def find_pieces(image):
    #cv2.imshow('Image - reference image', src)
    clear_difference = get_clear_difference(image,6)
    #cv2.imshow('Image - clear', clear_difference)
    black = black_out_black(clear_difference,150,100,100)
    #cv2.imshow('Image - pieces', pieces)
    _, edge_image, dst_image = find_edges(black,3,100,255,3,3,50,150)
    x_pieces = []
    o_pieces = []
    rectangles = []
    crosses =[np.int0((0,0,0,0))]
    detected_circles = cv2.HoughCircles(edge_image, cv2.HOUGH_GRADIENT, 1.5, 10, param1 = 50, param2 = 25, minRadius = 18, maxRadius = 22) 
    if detected_circles is None:
        circles = []
        print('No O-markers are found')
    else:
        circles = np.uint16(np.around(detected_circles))
        if len(circles) > 0:
            for i in circles[0,:]:
                print(i)
                cv2.circle(dst_image,(i[0],i[1]),i[2],(0,255,0),2)
                o_pieces.append([i[0],i[1]])        
    _, contours, _ = cv2.findContours(edge_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        rect = cv2.minAreaRect(contour)
        if (30 < rect[1][0] < 50) & (60 < rect[1][1] < 100):
            rect1 = ((rect[0][0], rect[0][1] + rect[1][1]/4), (rect[1][0], rect[1][1]/2), rect[2])
            rect2 = ((rect[0][0], rect[0][1] - rect[1][1]/4), (rect[1][0], rect[1][1]/2), rect[2])
            rectangles.append(rect1)
            rectangles.append(rect2)

        elif (60 < rect[1][0] < 100) & (30 < rect[1][1] < 50):
            rect1 = ((rect[0][0] + rect[1][0]/4, rect[0][1]), (rect[1][0]/2, rect[1][1]), rect[2])
            rect2 = ((rect[0][0] - rect[1][0]/4, rect[0][1]), (rect[1][0]/2, rect[1][1]), rect[2])
            rectangles.append(rect1)
            rectangles.append(rect2)

        else:
            rectangles.append(rect)
            
    for recta in rectangles:
        if 30 < recta[1][0] < 50 and 30 < recta[1][1] < 50: 
            is_cross = False
            is_circle = False
            
            for cross in crosses:
                if (recta[0][0] >= cross[0] and recta[0][0] <= cross[1]) and (recta[0][1] >= cross[2] and recta[0][1] <= cross[3]):
                    is_cross = True
            if len(circles) > 0:
                for i in circles[0, :]:
                    

                    if ((recta[0][0] >= (i[0] - i[2]) and recta[0][0] <= (i[0] + i[2])) and (recta[0][1] >= (i[1] - i[2]) and recta[0][1] <= (i[1] + i[2]))):
                        is_circle = True
                    
            box = cv2.boxPoints(recta)
            box = np.int0(box)
            (x1, y1), (x2, y2), (x3, y3), (x4, y4) = box
            if not is_cross and not is_circle:
                x_min = np.min([x1, x2, x3, x4])
                x_max = np.max([x1, x2, x3, x4])
                y_min = np.min([y1, y2, y3, y4])
                y_max = np.max([y1, y2, y3, y4])
                crosses.append((x_min, x_max, y_min, y_max))
                cv2.drawContours(dst_image, [box], 0, (0, 0, 255), 2)
                piece = [np.int0(recta[0][0]), np.int0(recta[0][1])]

                x_pieces.append([np.int0(recta[0][0]),np.int0(recta[0][1])])
        
    print("o_pieces count: ", len(o_pieces))
    print("x_pieces count: ", len(x_pieces))
    return dst_image, o_pieces, x_pieces

def get_board_state(squares,o_pieces,x_pieces):
    tic_tac_toe_board = np.array([[' ',' ',' '],
                        [' ',' ',' '],
                        [' ',' ',' ']])
    
    empty_squares_list = []
    o_on_board_list = []
    x_on_board_list = []
    for square in squares:
        print("square:", square[0])
        empty_squares_list.append(square[0])
        for o_piece in o_pieces:
            if (o_piece[0] >= (square[0][0]-(square[1][0]//2)) and o_piece[0] <= (square[0][0]+(square[1][0]//2))) and (o_piece[1] >= (square[0][1]-(square[1][1]//2)) and o_piece[1] <= (square[0][1]+(square[1][1]//2))):
                row = squares.index(square) // 3
                col = squares.index(square) % 3
                tic_tac_toe_board[row][col] = 'O'
                o_on_board_list.append(o_piece)
                if square[0] in empty_squares_list:
                    empty_squares_list.remove(square[0])   
        for x_piece in x_pieces:
            if (x_piece[0] >= (square[0][0]-(square[1][0]//2)) and x_piece[0] <= (square[0][0]+(square[1][0]//2))) and (x_piece[1] >= (square[0][1]-(square[1][1]//2)) and x_piece[1] <= (square[0][1]+(square[1][1]//2))):
                row = squares.index(square) // 3
                col = squares.index(square) % 3
                tic_tac_toe_board[row][col] = 'X'
                x_on_board_list.append(x_piece)
                if square[0] in empty_squares_list:
                    empty_squares_list.remove(square[0])
                
    # print(tic_tac_toe_board[0])
    # print(tic_tac_toe_board[1])
    # print(tic_tac_toe_board[2])
    print(tic_tac_toe_board)
    print(empty_squares_list)
    print(o_on_board_list)
    print(x_on_board_list)
    return tic_tac_toe_board, empty_squares_list, o_on_board_list, x_on_board_list

def from_image_to_coordinates(image,x,y):
    h = image.shape[0]
    w = image.shape[1]
    x_coordinate = x-(w/2)
    y_coordinate = h-y
    return x_coordinate, y_coordinate

def test_vision():
    image_ref = get_from_webcam()
    cv2.imshow("ref", image_ref)
    cv2.waitKey(0)

    image = get_from_webcam()
    
# image_ref = get_from_file('images/baggrund2.jpeg')
# #cv2.imshow('reference image', image_ref)
# #image = get_from_file('images/game1.jpeg')
# cv2.imshow('image', image)
    regionOfInterest, (xc,yc), (x_start, y_start), (x_end, y_end), theta = draw_regionOfInterest(image_ref)
# #cv2.imshow('Region of interest reference image', regionOfInterest)
    regionOfInterest2 = crop_image(image,xc,yc,x_start,y_start,x_end,y_end,theta)
# #cv2.imshow('Region of interest image', regionOfInterest2)
    ref = regionOfInterest.astype(float)
    img = regionOfInterest2.astype(float)
    src = np.abs(img-ref).astype(np.uint8)
# #cv2.imshow('source image', src)
# #clear_difference = get_clear_difference(src,5)
# #_, edge_image, _ = find_edges(clear_difference,40,255,1,2,50,150)
# #cv2.imshow('Image - clear', edge_image)
    pieces, o_pieces, x_pieces = find_pieces(src)
    cv2.imshow('Pieces', pieces)
    cv2.waitKey(0)
# board, squares = draw_board(src)
# cv2.imshow('Board',board)
# tic_tac_toe_board = find_pieces_on_board(squares,o_pieces,x_pieces)
# h = src.shape[0]
# w = src.shape[1]
# print(w,h)
# print(o_pieces)
# for o_piece in o_pieces:
#     x_coordinate, y_coordinate = from_image_to_coordinates(pieces,o_piece[0],o_piece[1])
#     x_coordinate = x_coordinate/8.69832
#     y_coordinate = y_coordinate/8.69832
#     print(x_coordinate,y_coordinate)
# print(x_pieces)
# for x_piece in x_pieces:
#     x_coordinate, y_coordinate = from_image_to_coordinates(pieces,x_piece[0],x_piece[1])
#     x_coordinate = x_coordinate/8.69832
#     y_coordinate = y_coordinate/8.69832
#     print(x_coordinate,y_coordinate)

#for square in squares:
#    print('byte:',square[0][0],square[0][1])
#    x_coordinate, y_coordinate = from_image_to_coordinates(board,square[0][0],square[0][1])
#    print('x and y',x_coordinate,y_coordinate)

#print(x_coordinate,y_coordinate)
#x_coordinate_m, y_coordinate_m = from_pixel_to_m_coordinates(8,x_coordinate,y_coordinate)
#print(x_coordinate_m,y_coordinate_m)
