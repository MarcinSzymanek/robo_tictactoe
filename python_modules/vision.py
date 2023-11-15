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

def group_lines(lines, rho_threshold=20, theta_threshold=np.pi/180 * 15):
    # Group lines based on their rho and theta values
    grouped_lines = []
    # Fix negative angles
    num_lines = lines.shape[1]
    for i in range(0, num_lines):
        line = lines[0,i,:]
        rho = line[0]
        theta = line[1]
        if rho < 0:
            rho *= -1.0
            theta -= np.pi
            line[0] = rho
            line[1] = theta
    for line in lines:
        found_group = False
        for group in grouped_lines:
            if (abs(line[0][0] - group[0][0][0]) < rho_threshold) and (abs(line[0][1] - group[0][0][1]) < theta_threshold):
                group.append(line)
                found_group = True
                break

        if not found_group:
            grouped_lines.append([line])

    return grouped_lines


def average_line(grouped_lines, image):
    # Calculate the average line for a group of lines
    slopes = []
    intercepts = []

    for group in grouped_lines:
        for line in group:
            rho, theta = line[0], line[1]
            a, b = np.cos(theta), np.sin(theta)
            x0, y0 = a * rho, b * rho
            slope = -a / b if b != 0 else 1e9 # Avoid division by zero
            intercept = y0 - slope * x0

            slopes.append(slope)
            intercepts.append(intercept)

    avg_slope = np.mean(slopes)
    avg_intercept = np.mean(intercepts)

    # Convert the average slope and intercept back to rho and theta
    avg_rho = avg_intercept / np.sin(np.arctan(avg_slope))
    avg_theta = np.arctan(avg_slope)

    # Calculate the line coordinates
    a_avg, b_avg = np.cos(avg_theta), np.sin(avg_theta)
    x0_avg, y0_avg = a_avg * avg_rho, b_avg * avg_rho

    # Use np.clip to ensure that coordinates are within the valid range
    x1 = int(np.clip(x0_avg + 1000 * (-b_avg), 0, image.shape[1] - 1))
    y1 = int(np.clip(y0_avg + 1000 * (a_avg), 0, image.shape[0] - 1))
    x2 = int(np.clip(x0_avg - 1000 * (-b_avg), 0, image.shape[1] - 1))
    y2 = int(np.clip(y0_avg - 1000 * (a_avg), 0, image.shape[0] - 1))

    return (avg_rho, avg_theta), (x1, y1), (x2, y2)

def draw_lines(image):
    # Change from BGR to gray
    src = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Change from gray to BGR
    dst = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
    cdst = np.copy(dst)
    
    edge_image = cv2.Canny(dst, threshold1=50, threshold2=255)

    lines = cv2.HoughLines(edge_image, rho=1, theta=np.pi/180, threshold=100)

    if lines is not None:
        # Group similar lines together based on rho and theta
        grouped_lines = group_lines(lines)

        # Calculate the average line for each group
        average_lines = [average_line(group, dst) for group in grouped_lines]

        for line in lines:
            rho, theta = line[0][0], line[0][1]
            a, b = np.cos(theta), np.sin(theta)
            x0, y0 = a * rho, b * rho
            if not np.isinf(a) and not np.isinf(b) and b != 0:
                x1, y1 = int(x0 + 1000 * (-b)), int(y0 + 1000 * (a))
                x2, y2 = int(x0 - 1000 * (-b)), int(y0 - 1000 * (a))
            elif not np.isinf(a):
                x1, y1 = int(x0), 0
                x2, y2 = int(x0), dst.shape[0] - 1
            else:
                x1, y1 = 0, int(y0)
                x2, y2 = dst.shape[1] - 1, int(y0)

            cv2.line(cdst, (x1, y1), (x2, y2), (0, 0, 255), 2)

        for (avg_rho, avg_theta), (avg_x1, avg_y1), (avg_x2, avg_y2) in average_lines:
            # Use np.clip to ensure that coordinates are within the valid range
            avg_x1 = int(np.clip(avg_x1, 0, dst.shape[1] - 1))
            avg_y1 = int(np.clip(avg_y1, 0, dst.shape[0] - 1))
            avg_x2 = int(np.clip(avg_x2, 0, dst.shape[1] - 1))
            avg_y2 = int(np.clip(avg_y2, 0, dst.shape[0] - 1))

            cv2.line(dst, (avg_x1, avg_y1), (avg_x2, avg_y2), (0, 0, 255), 2)

    return dst, cdst


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

image = get_from_webcam()
#image = get_from_file('image.jpeg')
image_cm = from_pixel_to_cm(image, 8.648)
white_elements = find_white_elements(image)
#white_circles_BGR = find_white_circles(image, RADIUS_THRESH)
#AoI_result = find_area_of_interest(image)
dst, cdst = draw_lines(image)
#resized_image = resize_image(image)
print("Pixel", image.shape)
print("cm", image_cm)
#white_elements = find_white_elements(resized_image)
#yellow_elements = find_yellow_elements(resized_image)
#red_elements = find_red_elements(resized_image)
cv2.imshow("Det original billede", image)
#cv2.imshow("Bolle", white_circles_BGR)
#cv2.imwrite('image12.jpeg', image)
#cv2.imshow('black_elements', black_elements)
cv2.imshow('white_elements', white_elements)
cv2.waitKey(0)
