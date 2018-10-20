import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

out_dir = 'calib_images'
calib_counter = 0
calib_devider = 3
rospy.init_node('computer_vision_sample')
bridge = CvBridge()

# create dir for calibration imgs
print('1. try create calib dir')

if not os.path.exists(out_dir):
    print('2. create dir:%s' % (out_dir))
    os.makedirs(out_dir)
    calib_counter = 0


def image_callback(data):
    global calib_counter
    print('3. callback')
    # save imag every calib_devider
    if not calib_counter % calib_devider:
        frame = bridge.imgmsg_to_cv2(data, 'bgr8')
        file_path = os.path.join(
            out_dir, 'img%s.png' % (calib_counter, ))
        cv2.imwrite(file_path, frame)
        print('4. save file:%s' % (file_path, ))
    calib_counter += 1


print('4. run main loop')
image_sub = rospy.Subscriber(
    'main_camera/image_raw', Image, image_callback)
rospy.spin()
