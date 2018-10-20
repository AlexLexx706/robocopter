import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import os
import threading

calib_imgs = 'web_imgs'
detected_results = []
lock = threading.Lock()
enable_gui = False


def get_detected():
    # print('get_detected')
    with lock:
        return detected_results.copy()


def update_detected(res):
    with lock:
        detected_results.clear()
        for r in res:
            detected_results.append(r)
        # print('update_detected:%s' % (detected_results, ))


def detect_markers(device_id, hold_info_count=15):
    cap = cv2.VideoCapture(device_id)

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 7, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    print('1. start read calib imgs')
    images = glob.glob(os.path.join(calib_imgs, '*.png'))

    if not len(images):
        print("calib images not exist!!!!!!!!!")
        return

    for fname in images:
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

            # Draw and display the corners
            if enable_gui:
                img = cv2.drawChessboardCorners(img, (7, 6), corners2, ret)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)

    print('2. start detect markers')
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    update_counter = 0

    while True:
        ret, frame = cap.read()
        # operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # lists of ids and the corners beloning to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters)

        font = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text (below)

        if np.all(ids != None):
            # print(type(ids), ids)
            result = []

            for id_, corner in zip(ids, corners):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corner, 0.05, mtx, dist)
                result.append((id_[0], rvec, tvec))

            update_detected(result)
            update_counter = hold_info_count

            if enable_gui:
                # Draw Axis
                aruco.drawAxis(frame, mtx, dist, rvec[0], tvec[0], 0.1)
                # Draw A square around the markers
                aruco.drawDetectedMarkers(frame, corners)
                cv2.putText(frame, "Id: " + str(ids), (0, 64),
                            font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Display the resulting frame
        if enable_gui:
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # clear update
        if update_counter <= 0:
            update_detected([])
            update_counter = 0

        update_counter -= 1

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import time
    print('1.')
    thread = threading.Thread(target=detect_markers, args=(0,))

    print('2.')
    thread.start()
    print('2.')

    while 1:
        print(get_detected())
        time.sleep(0.1)
    print('3.')
