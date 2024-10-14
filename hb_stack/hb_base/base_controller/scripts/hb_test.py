#!/usr/bin/env python3

import math
import rospy
import tf
import orienbus as orien_left
import orienbus_2 as orien_right
from nav_msgs.msg import Odometry
from hb_msgs.msg import Speed
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

class BaseController(object):

    def __init__(self):

        self._wheel_base     = 0.58
        self._wheel_radius   = 0.1
        self._port           = '/dev/hb_wheel'
        self._port_arm       = '/dev/hb_arm'
        self._left_motor_id  = 2
        self._right_motor_id = 3
        self._arm_motor_id   = 4
        self._gear_ratio     = 100
        self.max_RPM         = 4000

        # Create orienbus object with port name
        orienbus_right = orien_right.OrienBus(self._port)
        orienbus_left = orien_left.OrienBus(self._port)

        orienbus_arm = orien_left.OrienBus(self._port_arm)

        # Initialze motors with slave addresses
        self._left_motor = orienbus_left.initialize(self._left_motor_id)
        self._right_motor = orienbus_right.initialize(self._right_motor_id)
        self._arm_motor = orienbus_arm.initialize(self._arm_motor_id)

        
        
        self.linear_x=0
        self.angular_z=0
        self.linear_y=0

        self.left_wheel_speed = 0
        self.right_wheel_speed = 0

        self.arm_motor_speed_rpm=0

        self.prev_time = rospy.Time.now()
        self.cb_timer = rospy.Time.now()
        self.prev_yaw = 0
	
            




def main():
    rospy.init_node("base_test_node", anonymous=True)

    _object = BaseController()

    rate = rospy.Rate(10)
    ctrl_c = False

    def shutdownhook():
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not rospy.is_shutdown():
        _
    #rospy.spin()

if __name__ == '__main__':
    main()
