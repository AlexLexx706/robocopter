import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2.aruco as aruco
import numpy as np
import glob


rospy.init_node('computer_vision_sample')
bridge = CvBridge()
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((6 * 7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()

print("1. start read from calib images")

# camera calibration
for fname in glob.glob('calib_images/*.png'):
    print("2. read image:%s" % (fname, ))
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

print('3. create calib matrix')
# create camera calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)


def image_callback(data):
    frame = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters)
    print('image_callback corners:%s' % (corners, ))

    if np.all(ids != None):
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
            corners, 0.05, mtx, dist)


print('4. run main loop')
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

rospy.spin()
