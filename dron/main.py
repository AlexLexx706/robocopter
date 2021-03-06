#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from mavros_msgs.msg import RCIn
from clever import srv
from std_srvs.srv import Trigger
import dron

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
release = rospy.ServiceProxy('release', Trigger)
start = get_telemetry()

rospy.init_node('control_node')


def rc_callback(data):
    print(data)
    if data.channels[5] < 1100:
        dron.start_flag = 0
    elif data.channels[5] > 1900:
        dron.start_flag = 1


def take_off(z):
    print('take_off')
    tolerance = 0.2
    start = get_telemetry()
    print(navigate(z=z, speed=0.4, frame_id='fcu_horiz', auto_arm=True))
    while True:
        print('z', get_telemetry().z)
        if get_telemetry().z - start.z + z < tolerance:
            break
        rospy.sleep(0.2)


def translate(x, y, z, speed):
    print('translate')
    tolerance = 0.1
    frame_id = 'fcu_horiz'
    print(dron.navigate(frame_id=frame_id, x=x, y=y, z=z, speed=speed))
    while True:
        telem = dron.get_telemetry(frame_id=frame_id)
        print('telem', telem)
        if dron.get_distance(1, 2, 3, telem.x, telem.y, telem.z) < tolerance:
            break
        rospy.sleep(0.2)


def landing():
    print('landing')
    res = dron.land()
    tolerance = 0.1

    if res.success:
        print('Copter is landing')
    while True:
        print('z', dron.get_telemetry().z)
        if dron.get_telemetry().z - start.z < tolerance:
            print('Done')
            break
        rospy.sleep(0.2)


def release():
    res = dron.release()


# rospy.Subscriber('mavros/rc/in', RCIn, rc_callback)
# rospy.spin()

if __name__ == '__main__':
    take_off(1)
    translate(1, 0, 0, 0.5)
    landing()
    rospy.sleep(10)
    release()
