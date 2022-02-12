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
height_crop, width_crop = im_crop.shape
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
im_hough = cv.cvtColor(im_canny, cv.COLOR_GRAY2BGR)  # So we can overlay color lines over canny edges

# Probabilistic Hough lines
linesP = cv.HoughLinesP(im_canny, 1, np.pi / 180, 50, None, 50, 10)
if linesP is not None:
    intercepts_height = np.zeros(len(linesP), dtype=float)
    m = np.zeros(len(linesP), dtype=float)
    b = np.zeros(len(linesP), dtype=float)
    for i in range(0, len(linesP)):
        line = linesP[i][0]
        cv.line(im_hough, (line[0], line[1]), (line[2], line[3]), (0, 0, 255), 2, cv.LINE_AA)
        # line = np.asarray(line, dtype=float)        # Convert to float now

        # Find where each line intercepts lower edge of image (y=height)
        m[i] = (line[3] - line[1]) / (line[2] - line[0])
        b[i] = line[1] - m[i]*line[0]

        intercepts_height[i] = (height_crop - line[1]) / m[i] + line[0]
    centerpoint = int((np.min(intercepts_height) + np.max(intercepts_height)) / 2)
    cv.line(im_hough, (centerpoint, 0), (centerpoint, height_crop), (0, 255, 0), 2, cv.LINE_AA)
    print("centerline: ", centerpoint)
    print("average slope: ", round(np.average(m),2), "intercept: ", round(np.average(b),2))
    print("m:", m, "b:", b)

    # Now try to get average of left side and right side lines, then get average from those 2 lines
    left_line = [0., 0., 0]
    right_line = [0., 0., 0]
    for i in range(0, len(m)):
        if m[i] < 0.:        # Left line
            left_line[0] += m[i]
            left_line[1] += b[i]
            left_line[2] += 1       # Counter
        else:               # Right line
            right_line[0] += m[i]
            right_line[1] += b[i]
            right_line[2] += 1  # Counter
    # Average. TODO TAKE CARE OF DIVISON BY 0
    left_line[0] /= left_line[2]
    left_line[1] /= left_line[2]
    right_line[0] /= right_line[2]
    right_line[1] /= right_line[2]
    print("L: ", left_line, "R: ", right_line)

cv.line(im_hough, (0, int(left_line[1])), (width, int(left_line[0]*width + left_line[1])), (255, 255, 0), 2, cv.LINE_AA)
cv.line(im_hough, (0, int(right_line[1])), (width, int(right_line[0]*width + right_line[1])), (255, 255, 0), 2, cv.LINE_AA)

# Now get average of two lines
proj_line = [(left_line[0]+right_line[0])/2, (left_line[1]+right_line[1])/2]
print("Proj: ", proj_line)
print("y @0: ", int(proj_line[1]))
print("y @width: ", int(proj_line[0]*width + proj_line[1]))
cv.line(im_hough, (0, int(proj_line[1])), (width, int(proj_line[0]*width + proj_line[1])), (0, 255, 255), 2, cv.LINE_AA)

# cv.imshow("Original", im)
cv.imshow("Initial ROI", im_crop)
cv.imshow("Binary", im_bin)
cv.imshow("Canny", im_canny)
cv.imshow("Hough", im_hough)
# cv.imshow("ROI", crop)
# cv.imshow("Warped", im_warped)
# cv.imshow("Warped 2", im_warped2)
cv.waitKey(500)

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

a = np.array([2, 3, 2])
b = a
b[0:1] = b[2]
print(b)


while True:
    inp = input("Cmd: ")
    if inp == 'q':
        break
