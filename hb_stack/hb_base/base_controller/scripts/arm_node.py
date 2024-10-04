#!/usr/bin/env python2

import math
import rospy
import tf
#import orienbus as orien
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
import orienbus_2

class BaseController(object):

    def __init__(self):

	#self.odom_pub = rospy.Publisher('odom_wheel', Odometry, queue_size=10)
       # self.wheels_speed_pub = rospy.Publisher('wheels_speed', Twist, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self.cmdVelCallback)
	
        #rospy.Subscriber("/vectornav/IMU_repub", Imu, self.imuCallback)
	#rospy.Subscriber("/odometry/filtered", Odometry, self.odom_filtered_Callback)
        #self.odomBroadcaster = tf.TransformBroadcaster()
        #self.imu_msg=Imu()
	#self.odom_filtered_msg=Odometry()


        #self._wheel_base     = rospy.get_param('wheel_base', 0.14)
        #self._wheel_radius   = rospy.get_param('wheel_radius', 0.03)
        self._port           = '/dev/ttyUSB0'
        #self._left_motor_id  = rospy.get_param('left_motor_id')
        #self._right_motor_id = rospy.get_param('right_motor_id')
	self._arm_motor_id   = 4
        #self._gear_ratio     = rospy.get_param('gear_ratio')
        self.max_RPM         = rospy.get_param('max_RPM', 4000)
	self.linear_y	     =0
        # Create orienbus object with port name
        orienbus = orienbus_2.OrienBus(self._port)

        # Initialze motors with slave addresses
        #self._left_motor = orienbus.initialize(self._left_motor_id)
        #self._right_motor = orienbus.initialize(self._right_motor_id)
	self._arm_motor = orienbus.initialize(self._arm_motor_id)

        
	self.arm_motor_speed_rpm=0
        self.prev_time = rospy.Time.now()

	


    def cmdVelCallback(self, msg):

        self.linear_x = msg.linear.x
	self.linear_y = msg.linear.y
        self.angular_z = msg.angular.z

        

    def publish_odom(self):

	if self.linear_y > 0:
	    self.arm_motor_speed_rpm = 300
	elif self.linear_y < 0:
	    self.arm_motor_speed_rpm = -300
	else:
	    self.arm_motor_speed_rpm = 0

   

        #print("arm speed :",self.arm_motor_speed_rpm)
       # print("right wheel speed :",right_wheel_speed)





	self._arm_motor.writeSpeed(int(self.arm_motor_speed_rpm))


def main():
    rospy.init_node("arm_node", anonymous=True)

    _object = BaseController()

    rate = rospy.Rate(10)
    ctrl_c = False

    def shutdownhook():
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not rospy.is_shutdown():
	_object.publish_odom()
        rate.sleep()
    #rospy.spin()

if __name__ == '__main__':
    main()
