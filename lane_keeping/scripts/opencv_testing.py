import cv2 as cv
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import sys, select, termios, tty


def getCanny(_img, _min, _max):
    return cv.Canny(_img, _min, _max)


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(3)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# Get image
__location__ = os.path.relpath(
    os.path.join(os.getcwd(), os.path.dirname(__file__)))
im = cv.imread(os.path.join(__location__, 'for_testing_camera_raised.jpg'))
height, width, ch = im.shape

# Convert to grayscale
im_gray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)

# Rectangle crop
im_crop = im_gray[288:398][0:width]
h_crop_rec, w_crop_rec = im_crop.shape

# roi_pts = roi_pts - roi_pts.min(axis=0)  # Make Mask
# mask = np.zeros(crop.shape[:2], np.uint8)
# cv.drawContours(mask, [roi_pts], -1, (255, 255, 255), -1, cv.LINE_AA)
#
# crop = cv.bitwise_and(crop, crop, mask=mask)
#
# h_crop, w_crop = crop.shape

# Trapezoid coordinates for the Perspective transform
vt_lim1 = 320
vt_lim2 = height
hz_offset = 100
pt_A = [hz_offset, vt_lim1]
pt_B = [0, vt_lim2]
pt_C = [width, vt_lim2]
pt_D = [width - hz_offset, vt_lim1]
im_crop = im_gray[vt_lim1:vt_lim2][0:width]
print(vt_lim1, vt_lim2)

# L2 norm used to derive the new height. Width remains the same
# Can use either AB or CD since we chose an isosceles trapezoid
max_ptrans_width = width
max_ptrans_height = int(np.sqrt(((pt_A[0] - pt_B[0]) ** 2) + ((pt_A[1] - pt_B[1]) ** 2)))
print("Ptrans size: ", max_ptrans_width, max_ptrans_height)

# Create set of points for the input (trapezoid) to output (rectangle)
trapezoid_pts = np.float32([pt_A, pt_B, pt_C, pt_D])
rectangle_pts = np.float32([[0, 0], [0, max_ptrans_height - 1],
                            [max_ptrans_width - 1, max_ptrans_height - 1], [max_ptrans_width - 1, 0]])

# Get the perspective transform matrix, then warp the gray image
ptrans_M = cv.getPerspectiveTransform(trapezoid_pts, rectangle_pts)
im_warped = cv.warpPerspective(im_gray, ptrans_M, (max_ptrans_width, max_ptrans_height), flags=cv.INTER_LINEAR)
im_warped2 = cv.warpPerspective(im_gray, ptrans_M, (width, height), flags=cv.INTER_LINEAR)

# IGNORE ABOVE TRANSFORMATIONS FOR NOW
# Image thresholding using Otsu's Binarization
im_blur = cv.GaussianBlur(im_crop, (5, 5), 0)
_, im_bin = cv.threshold(im_blur, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

# Canny edge detector
canny_thresholds = [100, 200]  # Through experimentation found test image max value threshold reached 1520. Robust image
im_canny = getCanny(im_bin, canny_thresholds[0], canny_thresholds[1])
im_hough = cv.cvtColor(im_canny, cv.COLOR_GRAY2BGR)     # So we can overlay color lines over canny edges

# Probabilistic Hough lines
linesP = cv.HoughLinesP(im_canny, 1, np.pi / 180, 50, None, 50, 10)
if linesP is not None:
    for i in range(0, len(linesP)):
        line = linesP[i][0]
        cv.line(im_hough, (line[0], line[1]), (line[2], line[3]), (0, 0, 255), 3, cv.LINE_AA)

# cv.imshow("Original", im)
cv.imshow("Initial ROI", im_crop)
cv.imshow("Binary", im_bin)
cv.imshow("Canny", im_canny)
cv.imshow("Hough", im_hough)
# cv.imshow("ROI", crop)
# cv.imshow("Warped", im_warped)
# cv.imshow("Warped 2", im_warped2)
cv.waitKey(200)

# settings = termios.tcgetattr(sys.stdin)
# while 1:
#     key = getKey()
#     if key == '\x1b[A':  # up
#         canny_maxVal += 20
#     elif key == '\x1b[B':  # down
#         canny_maxVal -= 20
#     elif key == '\x1b[C':  # right
#         canny_minVal += 20
#     elif key == '\x1b[D':  # left
#         canny_minVal -= 20
#     else:
#         break
#     print("Canny. Min: " + str(canny_minVal) + " Max: " + str(canny_maxVal))
#     im_canny = getCanny(im_bin, canny_minVal, canny_maxVal)
#     cv.imshow("Canny", im_canny)
#     cv.waitKey(100)

while True:
    inp = input("Cmd: ")
    if inp == 'q':
        break
