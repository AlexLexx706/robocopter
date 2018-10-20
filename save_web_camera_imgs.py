import cv2
import os
import shutil

cap = cv2.VideoCapture(0)
out_dir = 'web_imgs'
counter = 0

print('1. try create out dir:%s' % (out_dir, ))

shutil.rmtree(out_dir)
if not os.path.exists(out_dir):
    os.makedirs(out_dir)

objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

print('2. start save files')
while counter < 30:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)

    corners2 = None
    img = frame

    if ret:
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria)
        # Draw and display the corners
        img = cv2.drawChessboardCorners(
            cv2.copyMakeBorder(
                frame,
                0, 0, 0, 0,
                cv2.BORDER_REPLICATE), (7, 6),
            corners2,
            ret)
        print("corners2 size: %s" % (corners2.size, ))
    cv2.imshow('img', img)
    ret = cv2.waitKey(30)

    if ret != -1 and corners2 is not None and corners2.size:
        file_path = os.path.join(out_dir, 'img_%s.png' % (counter, ))
        cv2.imwrite(file_path, frame)
        print('save img to:%s' % (file_path, ))
        counter += 1
# When everything done, release the capture
cap.release()
