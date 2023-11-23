from vision import get_from_file, get_gray_image
import cv2
import os
import sys
import numpy as np

img_path = "images/"

def find_white_circles(img, bw_threshold, radius_threshold):

    src = img.copy()
    src_gray = get_gray_image(src)
    #displayImg(src_gray, killafter=0)
    #kernel =  np.ones((3,3),np.uint8)
    #img_ero = cv2.erode((white_circles*255).astype('uint8'), kernel)
    #img_dil = cv2.dilate((img_ero*255).astype('uint8'), kernel)
    #white_circles = img_dil.copy()
    white_circles_BGR = src

    # contours, hierarchy = cv2.findContours(white_circles, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
   
    thresh = cv2.threshold(src_gray, bw_threshold[0], bw_threshold[1], cv2.THRESH_BINARY)[1]
    # displayImg(thresh, "thresholded")
    # Morph open 
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=3)
    # displayImg(opening, "opening")
    # blur = cv2.medianBlur(opening, 11)
    img_contours , contours, hierarchy = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    displayImg(img_contours, killafter=0)
    # Iterate through the contours
    i = 0
    circles_pos = []
    contour_result = img_contours.copy()
    contour_result = cv2.cvtColor(contour_result, cv2.COLOR_GRAY2BGR)
    for contour in contours:
        print(i)
        i += 1
        arclength = cv2.arcLength(contour, False)
        area = cv2.contourArea(contour)
        # Calculate the center and radius of the circle
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)
        # displayContour(contour)
        # Check for a black hole in the middle
        # Assuming the center pixel represents the hole
        center_value = src_gray[int(y), int(x)]
        print("arc: ", arclength, ", area: ", area, " convex: ", cv2.isContourConvex(contour))
        if (1000 < area < 1400) and (110 < arclength < 143) and (radius_threshold[0] < radius < radius_threshold[1]):  # Assuming black holes have pixel value 0 (adjust if needed)
            print("Is circle")
            # Draw the circle on the copy of the input image
            cv2.circle(white_circles_BGR, center, radius, (0, 0, 255), 2)
            cv2.circle(contour_result, center, radius, (0, 0, 255), 2)
            
            # Store circle position
            circles_pos.append((x, y))

    displayImg(contour_result, killafter=0)
    return white_circles_BGR, circles_pos

def displayContour(cnt):
    width = 640
    height = 480
    img = np.zeros((height, width, 3), np.uint8)
    cv2.drawContours(img, cnt, -1, (0, 255, 0), 3)
    displayImg(img, killafter=0)


def displayImg(img, name = "img", destroyOtherWindows = True, killafter = 0):
    if(destroyOtherWindows): cv2.destroyAllWindows()
    cv2.namedWindow(name)        # Create a named window
    cv2.moveWindow(name, 40,30)  # Move it to (40,30)
    cv2.imshow(name, img)
    cv2.waitKey(killafter)

def find_circles_hough(image, debug=False):
    gray = get_gray_image(image)
    canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    hough_circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 30,
    param1 = 50, param2=30, minRadius = 20, maxRadius=33 )
    #print(hough_circles)
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
            if debug:
                cv2.circle(canvas, center, radius, (0, 255, 0), 2)
            circles.append((center, radius))
    if debug:
        displayImg(canvas)
    print("Found: ", count, " circles.")
    return circles



def find_circles_adaptive(image):
    img = image.copy()
    displayImg(img, killafter=0)
    gray = get_gray_image(img)
    blur = cv2.medianBlur(gray, 15)
    thresh = cv2.threshold(img, 100, 255, cv2.THRESH_TOZERO)[1]
    t2 = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
    t3 = cv2.adaptiveThreshold(get_gray_image(thresh), 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 5, 3)
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
                
                print("Is circle")
                # Draw the circle on the copy of the input image
                # cv2.circle(white_circles_BGR, center, radius, (0, 0, 255), 2)
                cv2.circle(contour_result, center, radius, (0, 0, 255), 2)
            # else:
            #     print("cross")
            #     cv2.circle(contour_result, center, radius, (255, 0, 0), 2)
            # Store circle position
            #circles_pos.append((x, y))

    displayImg(contour_result, killafter=0)


    # displayImg(o2, "adaptiveT opened", killafter=0)
    # showCannyEdges(img)
    # showCannyEdges(thresh)
 
    #displayImg(img, killafter=0)
    # circles_img, circles_pos = find_white_circles(img, bw_threshold=(black_thresh, 255), radius_threshold=(18, 28))
    # print(circles_pos)
    # displayImg(circles_img, killafter=0)

def showCannyEdges(img):
    gray = get_gray_image(img)
    edged = cv2.Canny(gray, 30, 200)
    displayImg(edged, killafter=0)

img_path = sys.argv[1]

circles = []
start_black_thresh = 140
thresh = start_black_thresh
#for i in range(5):
#    process_img(img_path, thresh)
#    thresh += 5
# process_img(img_path, thresh)

image = get_from_file(img_path)
circles = find_circles_hough(image, debug=True)
# Print coordinates of circle center
for c in circles:
    print(c[0])

# all_imgs = os.listdir(img_path)
# for img in all_imgs:
#     process_img(img_path + img)
