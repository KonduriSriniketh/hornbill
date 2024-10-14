#!/usr/bin/env python3

import math
import time
import rospy
import tf
import orienbus as orien
from nav_msgs.msg import Odometry
from hb_msgs.msg import Speed
from geometry_msgs.msg import Quaternion, TransformStamped
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

class BaseOdometryNode(object):

    def __init__(self):

        self._wheel_base     = rospy.get_param('wheel_base', 0.14)
        self._wheel_radius   = rospy.get_param('wheel_radius', 0.03)
        self._port           = rospy.get_param('port', '/dev/ttyUSB0')
        self._port_arm       = rospy.get_param('port_arm', '/dev/ttyUSB1')
        self._left_motor_id  = rospy.get_param('left_motor_id')
        self._right_motor_id = rospy.get_param('right_motor_id')
        self._arm_motor_id   = rospy.get_param('arm_motor_id')
        self._gear_ratio     = rospy.get_param('gear_ratio')
        self.max_RPM         = rospy.get_param('max_RPM', 4000)

        self.vx = 0
        self.vy = 0
        self.vth = 0
        self.vth_imu = 0

        self.x = 0
        self.y = 0
        self.th = 0
        self.th_imu = 0
        self.th_imu_prev = 0

        self.odom_pub = rospy.Publisher('/wheelodom', Odometry, queue_size=1)
        rospy.Subscriber("/wheels_speed", Speed, self.speedCallback)
        rospy.Subscriber("/vectornav/IMU", Imu, self.imuCallback)

        time.sleep(1)
        
        self.odomBroadcaster = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10)
    
        self.th_imu_prev = self.th_imu

        self.left_wheel_speed = 0
        self.right_wheel_speed = 0
        self.arm_motor_speed_rpm=0

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.cb_timer = rospy.Time.now()
        
        
    def imuCallback(self, msg):
        # Extract the quaternion from the IMU message
        orientation_q = msg.orientation
        angular_velocity_z = msg.angular_velocity.z

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # Calculate yaw (z-axis rotation)
        # siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        # cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        # yaw = math.atan2(siny_cosp, cosy_cosp)

        self.th_imu = yaw
        self.vth_imu = angular_velocity_z
            
        
        
    def speedCallback(self, msg):
        self.left_wheel_speed = msg.wheel_left
        self.right_wheel_speed = msg.wheel_right

        self.vx = self._wheel_radius * (self.right_wheel_speed + self.left_wheel_speed)/2.0
        self.vy = 0
        self.vth = self._wheel_radius * (self.right_wheel_speed - self.left_wheel_speed)/(self._wheel_base)

        self.cb_timer = rospy.Time.now()

    def odometry_publisher(self):

        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()

        # delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        # delta_y = (self.vx * math.sin(self.th) + self.vy * math.sin(self.th)) * dt
        delta_x = (self.vx * math.cos(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th)) * dt
        # delta_th = self.vth * dt # Calculate change in yaw using wheel feedback

        delta_th = self.th_imu_prev - self.th_imu # Calculation using IMU only
        self.vth = delta_th/dt
    
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        print("self.x = ",self.x)
        print("self.y = ",self.y)
        print("self.th = ", self.th)
        print("self.th_imu = ", self.th_imu)

        odom_quat = Quaternion()
        odom_quat = tf.transformations.quaternion_from_euler(0,0,self.th)

        # Broadcast tf
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.current_time
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation.x = odom_quat[0]
        odom_trans.transform.rotation.y = odom_quat[1]
        odom_trans.transform.rotation.z = odom_quat[2]
        odom_trans.transform.rotation.w = odom_quat[3]
        # Send the transform
        self.odomBroadcaster.sendTransform((self.x, self.y, 0),
                                       odom_quat,self.current_time, "base_link", "odom")
        odom = Odometry()
        odom.header.frame_id = "odom_wheel"
        odom.header.stamp = self.current_time
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation.x= odom_quat[0]
        odom.pose.pose.orientation.y= odom_quat[1]
        odom.pose.pose.orientation.z= odom_quat[2]
        odom.pose.pose.orientation.w= odom_quat[3]
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        self.odom_pub.publish(odom)
    
        self.last_time = self.current_time
        self.th_imu_prev = self.th_imu
        print("-------------------------")
        
    
def main():
    rospy.init_node("base_controller_node", anonymous=True)

    _object = BaseOdometryNode()

    def shutdownhook():
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not rospy.is_shutdown():
        _object.odometry_publisher()
        _object.rate.sleep()

if __name__ == '__main__':
    main()
