#!/usr/bin/env python3

import math
import rospy
import tf
import orienbus as orien
from nav_msgs.msg import Odometry
from hb_msgs.msg import Speed
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

class BaseController(object):

    def __init__(self):

        self.odom_pub = rospy.Publisher('odom_wheel', Odometry, queue_size=10)
       # self.wheels_speed_pub = rospy.Publisher('wheels_speed', Twist, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self.cmdVelCallback)

        self.odomBroadcaster = tf.TransformBroadcaster()
        self.imu_msg=Imu()


        self._wheel_base     = rospy.get_param('wheel_base', 0.14)
        self._wheel_radius   = rospy.get_param('wheel_radius', 0.03)
        self._port           = rospy.get_param('port', '/dev/ttyUSB0')
        self._port_arm       = rospy.get_param('port_arm', '/dev/ttyUSB1')
        self._left_motor_id  = rospy.get_param('left_motor_id')
        self._right_motor_id = rospy.get_param('right_motor_id')
        self._arm_motor_id   = rospy.get_param('arm_motor_id')
        self._gear_ratio     = rospy.get_param('gear_ratio')
        self.max_RPM         = rospy.get_param('max_RPM', 4000)

        # Create orienbus object with port name
        orienbus = orien.OrienBus(self._port)
        orienbus_arm = orien.OrienBus(self._port_arm)

        # Initialze motors with slave addresses
        self._left_motor = orienbus.initialize(self._left_motor_id)
        self._right_motor = orienbus.initialize(self._right_motor_id)
        self._arm_motor = orienbus_arm.initialize(self._arm_motor_id)

        self.x = 0
        self.y = 0
        self.th = 0

        self.l_speed_rpm = 0
        self.r_speed_rpm = 0
        self.linear_x=0
        self.angular_z=0
        self.linear_y=0
        self.arm_motor_speed_rpm=0
        self.prev_time = rospy.Time.now()
        self.prev_yaw = 0
       # self.wheels_speed=Twist()
	
            
    def cmdVelCallback(self, msg):

        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.angular_z = msg.angular.z

        

    def publish_odom(self):

        print("############### LOOP START ##################")

        linear_x= self.linear_x
        linear_y= self.linear_y
        angular_z=self.angular_z
        if linear_x > 0.1:
            linear_x = 0.1
        elif linear_x < -0.1:
            linear_x = -0.1

        if angular_z > 0.25:
            angular_z = 0.25
        elif angular_z < -0.25:
            angular_z = -0.25
        
        if linear_y > 0:
	        self.arm_motor_speed_rpm = 200
        elif(linear_y < 0):
             self.arm_motor_speed_rpm = -200
        else:
	        self.arm_motor_speed_rpm = 0

        right_wheel_speed = (linear_x / self._wheel_radius) + (self._wheel_base/self._wheel_radius * angular_z) # rad/s
        left_wheel_speed  = (linear_x / self._wheel_radius) - (self._wheel_base/self._wheel_radius * angular_z)

       # print("left wheel speed :",left_wheel_speed)
       # print("right wheel speed :",right_wheel_speed)

        right_wheel_speed_rpm = right_wheel_speed * 60/ (2*math.pi) * self._gear_ratio #rpm
        left_wheel_speed_rpm = left_wheel_speed * 60/ (2*math.pi) * self._gear_ratio
        print("left wheel speed rpm :", left_wheel_speed_rpm)
        print("right wheel speed rpm :", right_wheel_speed_rpm)
        print("-----")


        self._left_motor.writeSpeed(int(left_wheel_speed_rpm))
        self._right_motor.writeSpeed(int(-right_wheel_speed_rpm))
        self._arm_motor.writeSpeed(int(self.arm_motor_speed_rpm))

        self.l_speed_rpm = self._left_motor.readSpeed() / self._gear_ratio # rpm
        self.r_speed_rpm = self._right_motor.readSpeed() / self._gear_ratio
        if self.l_speed_rpm>0:
            self.l_speed_rpm=self.l_speed_rpm+1
        if self.r_speed_rpm>0:
            self.r_speed_rpm=self.r_speed_rpm+1


        print("1l_speed from motor left", self.l_speed_rpm)
        print("1r_speed from motor right", self.r_speed_rpm)

        if self.l_speed_rpm>100 or self.l_speed_rpm<-100 :
            self.l_speed_rpm=0
        if self.r_speed_rpm>100 or self.r_speed_rpm<-100 :
            self.r_speed_rpm=0

      #  self.wheels_speed.linear.x = self._left_motor.readSpeed()
      #  self.wheels_speed.linear.y = self._right_motor.readSpeed()
  #      self._left_motor.writeSpeed(0)
  #      self._right_motor.writeSpeed(0)
       # self.wheels_speed_pub.publish(self.wheels_speed)

        curr_time = rospy.Time.now()
        dt = (curr_time - self.prev_time).to_sec()
        self.prev_time = curr_time

        l_speed = self.l_speed_rpm * 2 * math.pi / 60 # rad/s
        r_speed = -self.r_speed_rpm * 2 * math.pi / 60
	
        quaternion = (
			self.imu_msg.orientation.x,
			self.imu_msg.orientation.y,
			self.imu_msg.orientation.z,
			self.imu_msg.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        delta_th = yaw - self.prev_yaw

        vx = self._wheel_radius * (r_speed + l_speed) / 2.0
        vy = 0
        #vth = self._wheel_radius * (r_speed - l_speed) / (2 * self._wheel_base)
	#vth = (yaw - self.prev_yaw) / dt
        self.prev_yaw = yaw
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt


        self.x  += delta_x
        self.y  += delta_y
        self.th += delta_th

        vth = delta_th / dt
        print("--")
        print("2l_speed send to motor left", l_speed)
        print("2r_speed send to motor right ", r_speed)

        print("roll :", roll)
        print("pitch :", pitch)
        print("yaw :", self.th)

        odom_quat = Quaternion()
        odom_quat = tf.transformations.quaternion_from_euler(0,0,self.th)

        # Create the odometry transform frame broadcaster (publish tf)
        self.odomBroadcaster.sendTransform(
                           (self.x, self.y, 0),
                           odom_quat,
                           rospy.Time.now(),
                           "base_link",
                           "odom"
                           )


        # Publish Odom
        odom = Odometry()
        odom.header.frame_id = "odom_wheel"
#        odom.child_frame_id = "base_link"
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation.x= odom_quat[0]
        odom.pose.pose.orientation.y= odom_quat[1]
        odom.pose.pose.orientation.z= odom_quat[2]
        odom.pose.pose.orientation.w= odom_quat[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

def main():
    rospy.init_node("base_controller_node", anonymous=True)

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
