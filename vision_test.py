# import cv2
# import sys
# import numpy as np

# def nothing(x):
#     pass

# # Load in image
# image = cv2.imread('test_img.png')


# # Initialize to check if HSV min/max value changes
# hMin = sMin = vMin = hMax = sMax = vMax = 0
# phMin = psMin = pvMin = phMax = psMax = pvMax = 0

# output = image
# wait_time = 33



# # Set minimum and max HSV values to display
# lower = np.array([173, 132, 0])
# upper = np.array([179, 255, 255])

# # Create HSV Image and threshold into a range.
# hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
# mask = cv2.inRange(hsv, lower, upper)
# output = cv2.bitwise_and(image,image, mask= mask)

# im = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

# cv2.imshow("Keypoints", im)
# cv2.waitKey(0)


import cv2
import numpy as np
from collections import deque

# for keeping center points of object
buffer_size = 16
pts = deque(maxlen=buffer_size)

# blue HSV
blueLower = (173, 132, 0)
blueUpper = (179, 255, 255)

imgOriginal = cv2.imread('test_img.png')

#blur
blurred = cv2.GaussianBlur(imgOriginal, (11,11), 0)

# HSV
hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
cv2.imshow("HSV IMAGE", hsv)

# mask for blue
mask = cv2.inRange(hsv, blueLower, blueUpper)

# deleting noises which are in area of mask
# mask = cv2.erode(mask, None, iterations=2)
# mask = cv2.dilate(mask, None, iterations=2)
# cv2.imshow("Mask + Erosion + Dilation", mask)

# contours
contours,_ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
center = None

if len(contours) > 0:

    # get max contour
    c = max(contours, key=cv2.contourArea)

    # return rectangle
    rect = cv2.minAreaRect(c)
    ((x,y), (width, height), rotation) = rect

    s = f"x {np.round(x)}, y: {np.round(y)}, width: {np.round(width)}, height: {np.round(height)}, rotation: {np.round(rotation)}"
    print(s)

    # box
    box = cv2.boxPoints(rect)
    box = np.int64(box)

    # moment
    M = cv2.moments(c)
    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    # draw contour
    cv2.drawContours(imgOriginal, [box], 0, (0, 255, 255), 2)

    # point in center
    cv2.circle(imgOriginal, center, 5, (255, 0, 255), -1)

    # print inform
    cv2.putText(imgOriginal, s, (25, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255), 2)


# deque
pts.appendleft(center)
for i in range(1, len(pts)):

    if pts[i - 1] is None or pts[i] is None: continue

    cv2.line(imgOriginal, pts[i - 1], pts[i], (0, 255, 0), 3)

cv2.imshow("DETECTED IMAGE", imgOriginal)


cv2.waitKey(0)

