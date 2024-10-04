#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import minimalmodbus
import time
import orienbus_2
import math
import hid
from time import sleep

port = '/dev/ttyUSB1' # modbus port name

class BaseController(object):

    def __init__(self):

		rospy.Subscriber("/cmd_vel", Twist, self.callback)
		self.stop_mode = rospy.Publisher('stop_mode', Float32, queue_size=100)
		self.right_wheel_speed_rpm=0
		self.left_wheel_speed_rpm=0

    def callback(self, msg):
    		self.linear_x= msg.linear.x
    		self.linear_y= msg.linear.y
   		self.angular_z= msg.angular.z
    		# desired_speed=msg.linear.x #motor right


    		right_wheel_speed = (self.linear_x / 0.075) + (0.43/0.075 * self.angular_z) # rad/s
    		left_wheel_speed  = (self.linear_x / 0.075) - (0.43/0.075 * self.angular_z)

    		self.right_wheel_speed_rpm = right_wheel_speed * 60/ (2*math.pi) * 100 #rpm
    		self.left_wheel_speed_rpm = left_wheel_speed * 60/ (2*math.pi) * 100
    		#rospy.loginfo(vel_right_rpm)
    		#rospy.loginfo(vel_left_rpm)
   		if((self.right_wheel_speed_rpm==0) and (self.left_wheel_speed_rpm==0)):
			#sleep(0.3)
    			#relay.state(1, on=False)
    			#relay.state(2, on=False)
			self.stop_mode.publish(0)
			print("turn off relay")
    			motor2.writeSpeed(-self.right_wheel_speed_rpm)
    			motor4.writeSpeed(-self.right_wheel_speed_rpm)
    			print("Right_speed", self.right_wheel_speed_rpm)
    			motor1.writeSpeed(self.left_wheel_speed_rpm) # run the motor with desire speed in RPM
    			motor3.writeSpeed(self.left_wheel_speed_rpm) # run the motor with desire speed in RPM
    			print("Left_speed", self.left_wheel_speed_rpm)
    		else:
			#sleep(0.1)
    			#relay.state(1, on=True)
    			#relay.state(2, on=True)
			#self.stop_mode.publish(1)
			#sleep(0.2)
			#self.stop_mode.publish(1)
			#sleep(0.2)
			#self.stop_mode.publish(1)
			#sleep(0.2)
			self.stop_mode.publish(1)
			print("turn on relay")
			sleep(0.1)
    			motor2.writeSpeed(-self.right_wheel_speed_rpm)
    			motor4.writeSpeed(-self.right_wheel_speed_rpm)
    			print("Right_speed", self.right_wheel_speed_rpm)
    			motor1.writeSpeed(self.left_wheel_speed_rpm) # run the motor with desire speed in RPM
    			motor3.writeSpeed(self.left_wheel_speed_rpm) # run the motor with desire speed in RPM
    			print("Left_speed", self.left_wheel_speed_rpm)
   		# print(motor1.readSpeed())
    		#print(motor2.readSpeed())




def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('some_node')
    _object = BaseController()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()




if __name__ == '__main__':
    #relay = Relay(idVendor=0x16c0, idProduct=0x05df)
    address_right = 1
    address_left = 2
    old_vel_right_rpm=9999999999999
    old_vel_left_rpm=999999999999999
    orienbus = orienbus_2.OrienBus(port)

#addres2 = 2 # slave ID for motor driver
    motor2 = orienbus.initialize(2)
    motor4 = orienbus.initialize(4)
    motor1 = orienbus.initialize(1)
    motor3 = orienbus.initialize(3)
#initialzing the motor driver
    listener()
    
    
