import sys

from objectDetection import ObjectDetector


img_path = sys.argv[1]

print("Hello")

od = ObjectDetector(image_path=img_path, debug=True)
#od.load_image(img_path)
# od.find_circles_adaptive()
# od.find_circles_hough()
od.debug = True
od.find_crosses_black(passes=3)
