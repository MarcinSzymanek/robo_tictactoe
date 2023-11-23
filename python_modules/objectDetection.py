import cv2
import os
import sys
import numpy as np

class Circle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

# We should have some validation rules:
# Exclude objects out of bounds
# If we find more circles/crosses than pieces we need to exclude the
# worst entries...

class ObjectDetector:
    def __init__(self, image_path , debug = False):
        self.debug = debug
        self.load_image(image_path)
        
    def load_image(self, path):
        self.image = get_from_file(path)
        self.image_gray = get_gray_image(self.image)
        if self.debug:
            displayImg(self.image)
            displayImg(self.image_gray)

    # Find circles using cv2.HoughCircles. Fixed params for now
    def find_circles_hough(self):
        gray = self.image_gray.copy()
        canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        hough_circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 30,
        param1 = 50, param2=30, minRadius = 20, maxRadius=33 )
        if self.debug:
            print(hough_circles)
            print(hough_circles.shape)
            print(hough_circles[0][0])
        count = 0
        circles = []

        for circle in hough_circles[0]:
            count += 1
            print(circle)
            center = int(circle[0]), int(circle[1])
            radius = circle[2]
            if(18 < radius < 40):
                if self.debug:
                    cv2.circle(canvas, center, radius, (0, 255, 0), 2)
                circles.append(Circle(center[0], center[1], radius))
        if self.debug:
            print("Found: ", count, " circles.")
            displayImg(canvas)
        return circles

    def find_crosses_black(self, black_thresh = 75, passes=1):
        img = self.image.copy()
        gray = self.image_gray.copy()

        blur = cv2.medianBlur(gray, 5)
        
        results = []

        for i in range(passes):
            thresh = cv2.threshold(blur, black_thresh + i*20, 255, cv2.THRESH_BINARY_INV)[1]
            #t2 = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
            
            o = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
            #o2 = cv2.morphologyEx(t2, cv2.MORPH_OPEN, kernel, iterations=1)
            
            contours, hierarchy = cv2.findContours(o, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            contour_result = o.copy()
            contour_result = cv2.cvtColor(contour_result, cv2.COLOR_GRAY2BGR)
            i = 0

            circles = []

            # Iterate through contours and try to find crosses with appropriate radius
            for contour in contours:
                print(i)
                i += 1
                arclength = cv2.arcLength(contour, False)
                area = cv2.contourArea(contour)
                # if(area < 180): continue
                # Calculate the center and radius of the circle
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)

            
                # Check for a black hole in the middle
                # Assuming the center pixel represents the hole
                center_value = o[int(y), int(x)]
                print("arc: ", arclength, ", area: ", area, " convex: ", cv2.isContourConvex(contour))
                if (100 < arclength < 200) and (700 < area < 1400):  # Assuming black holes have pixel value 0 (adjust if needed)                    
                        # Draw the circle on the copy of the input image
                        # cv2.circle(white_circles_BGR, center, radius, (0, 0, 255), 2)
                        if self.debug:
                            # displayContour(contour)
                            cv2.circle(contour_result, center, radius, (255, 0, 0), 2)
                        circles.append(Circle(x, y, radius))
                
                results.append(circles)

            if self.debug:
                displayImg(thresh)
                displayImg(o)
                displayImg(contour_result)
                # displayImg(t2)



    def find_circles_adaptive(self):
        
        print("find adaptive")
        if self.debug:
            displayImg(self.image, killafter=0)
        gray = self.image_gray.copy()
        
        blur = cv2.medianBlur(gray, 15)
        thresh = cv2.threshold(self.image, 100, 255, cv2.THRESH_TOZERO)[1]
        t2 = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        # t3 = cv2.adaptiveThreshold(get_gray_image(thresh), 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 5, 3)
        # displayImg(t3, killafter=0)

        # displayImg(thresh, "thresholded")
        # Morph open 
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=3)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        # o2 = cv2.erode(t2, kernel)
        o2 = cv2.morphologyEx(t2, cv2.MORPH_OPEN, kernel, iterations=1)
        # t2 = cv2.erode(t2, kernel)
        #displayImg(opening, killafter=0)
        displayImg(t2, "adaptiveThresh", killafter=0)
        displayImg(o2, "adaptiveOpened", killafter=0)

        contours, hierarchy = cv2.findContours(o2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour_result = o2.copy()
        contour_result = cv2.cvtColor(contour_result, cv2.COLOR_GRAY2BGR)
        i = 0

        circles = []

        # Iterate through contours and try to find circles with appropriate radius
        for contour in contours:
            
            print(i)
            i += 1
            arclength = cv2.arcLength(contour, False)
            area = cv2.contourArea(contour)
            if(area < 180): continue
            # Calculate the center and radius of the circle
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            # displayContour(contour)
            # Check for a black hole in the middle
            # Assuming the center pixel represents the hole
            center_value = o2[int(y), int(x)]
            print("arc: ", arclength, ", area: ", area, " convex: ", cv2.isContourConvex(contour))
            if (0 < arclength < 300) and (10 < radius < 30):  # Assuming black holes have pixel value 0 (adjust if needed)
                if center_value == 255:                    
                    # Draw the circle on the copy of the input image
                    # cv2.circle(white_circles_BGR, center, radius, (0, 0, 255), 2)
                    if self.debug:
                        cv2.circle(contour_result, center, radius, (0, 0, 255), 2)
                    circles.append(Circle(x, y, radius))

        if self.debug:
            displayImg(contour_result, killafter=0)

        return circles

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

# Convert image to grayscale
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


def displayContour(cnt):
    width = 640
    height = 480
    img = np.zeros((height, width, 3), np.uint8)
    cv2.drawContours(img, cnt, -1, (0, 255, 0), 3)
    displayImg(img, killafter=0)

# Display a single image. Does not automatically kill window unless killafter
# Is set to a positive value
def displayImg(img, name = "img", destroyOtherWindows = True, killafter = 0):
    if(destroyOtherWindows): cv2.destroyAllWindows()
    cv2.namedWindow(name)        # Create a named window
    cv2.moveWindow(name, 40,30)  # Move it to (40,30)
    cv2.imshow(name, img)
    cv2.waitKey(killafter)

def showCannyEdges(img):
    gray = get_gray_image(img)
    edged = cv2.Canny(gray, 30, 200)
    displayImg(edged, killafter=0)