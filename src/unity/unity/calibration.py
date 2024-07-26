import cv2
import numpy as np
import os
import glob



 
# Defining the dimensions of checkerboard
CHECKERBOARD = (5,7)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = [] 
 
 
# Defining the world coordinates for 3D points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None
 
# Extracting path of individual image stored in a given directory
images = glob.glob('./calibration/*.png')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    # If desired number of corners are found in the image then ret = true
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
     
    """
    If desired number of corner are detected,
    we refine the pixel coordinates and display 
    them on the images of checker board
    """
    if ret == True:
        objpoints.append(objp)
        # refining pixel coordinates for given 2d points.
        corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
         
        imgpoints.append(corners2)
 
        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
     
    # cv2.imshow('img',img)
    # cv2.waitKey(0)
 
cv2.destroyAllWindows()
 
h,w = img.shape[:2]
 
"""
Performing camera calibration by 
passing the value of known 3D points (objpoints)
and corresponding pixel coordinates of the 
detected corners (imgpoints)
"""
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
 
np.savez(
    "calibration/CalibrationMatrix_college_cpt",
    Camera_matrix=mtx,
    distCoeff=dist,
    RotationalV=rvecs,
    TranslationV=tvecs,
)

# generate the src and destination points for birds eye warp from image995.png
img = cv2.imread('./calibration/1721989141.983419.png')

# undistort the image based on previous calibration
img = cv2.undistort(img, mtx, dist, None)

# detect checkerboard corners
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
# get the perspective transform matrix
# get the outermost corners
print(corners2)
output_width = 640
output_height = 480

ow2 = output_width/8
oh2 = output_height/8

src = np.float32([corners2[0], corners2[CHECKERBOARD[0]-1], corners2[-1], corners2[-CHECKERBOARD[0]]])
dst = np.float32([[ow2*2, oh2*5], [ow2, oh2*5], [ow2, oh2*3], [ow2*2, oh2*3]])
# draw the points on the original image
for point in src:
    cv2.circle(img, (int(point[0][0]), int(point[0][1])), 5, (0, 0, 255), 1)
cv2.imshow('img', img)
cv2.waitKey(0)
print("gp")
print(src)
print("sef")
print(dst)
# print(dst)
M = cv2.getPerspectiveTransform(src, dst)
# warp the image
warped = cv2.warpPerspective(img, M, (output_width, output_height))
# resize the warped image so it fits on the screen
warped = cv2.resize(warped, (640, 480))
cv2.imshow('warped', warped)
cv2.waitKey(0)

# save the perspective transform matrix
np.savez(
    "calibration/BirdsEyeMatrix_college_cpt",
    M=M
)


