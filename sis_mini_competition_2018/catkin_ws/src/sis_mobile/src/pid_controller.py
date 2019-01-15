#!/usr/bin/env python

'''
  PID controller for differential-drive robot 
  Two main things to do:
   * Read data from Arduino (in format v_l v_r theta), use this data to do dead-reckoning
     and publish odom topic -> called by timer with 100Hz
   * Subscribe to twist topic, calculate desired linear velocities of two wheels and use
     PID controller to chase the setpoint, produce pwm value for motors -> called by callback
'''

import serial
import rospy
import tf

from math import sin, cos, isnan
from Adafruit_MotorHAT import Adafruit_MotorHAT
from simple_pid import PID
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Empty, EmptyResponse

WIDTH = 0.172
RADIUS = 0.032

# Check if the list is in range (low, up)
# Params:
# 	value_list:  given value list
#	low:         lower bound
# 	up:          upper bound
# Output:
#   True: if every value in list in range (low, up)
#   False: otherwise
def in_range(value_list, low, up):
	for i in range(0, len(value_list)):
		if value_list[i] > up or value_list[i] < low:
			return False
	return True

class Car_controller(object):
	def __init__(self):
		self.motorhat = Adafruit_MotorHAT(0x60)
		self.left_motor = self.motorhat.getMotor(1)
		self.right_motor = self.motorhat.getMotor(2)
		self.pid_r = PID(1500.0, 500.0, 100.0, sample_time = 0.05) # P/I/D for right wheel
		self.pid_l = PID(1500.0, 500.0, 100.0, sample_time = 0.05) # P/I/D for left wheel
		self.pid_r.output_limits = (-255, 255)
		self.pid_l.output_limits = (-255, 255) # PWM limit
		self.port = rospy.get_param("~port", '/dev/ttyACM0') # Arduino port from parameter server
		self.pub_tf = rospy.get_param("~pub_tf", False) # If true, broadcasr transform from
		# odom to car_base
		self.ard = serial.Serial(self.port, 57600)
		# Flush serial data
		for i in range(0, 20):
			_ = self.ard.readline()
		# Subscriber and publisher
		self.pub_odom = rospy.Publisher('/wheel_odom', Odometry, queue_size = 10)
		self.sub_cmd  = rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb,  queue_size = 1)
		if self.pub_tf:
			self.tf_br = tf.TransformBroadcaster()
		# Service
		self.reset_odom = rospy.Service('reset_wheel_odom', Empty, self.reset_odom)
		rospy.Timer(rospy.Duration(1/100.), self.read_data) # 100Hz
		self.v_r = None
		self.v_l  = None
		self.heading = 0
		self.x = 0
		self.y = 0
		self.time = rospy.Time.now()
		rospy.loginfo("[%s] Initialized"  %(rospy.get_name()))
	def reset_odom(self, req):
		self.x = 0
		self.y = 0
		self.heading = 0
		self.ard.write("c".encode()) # Write to the serial to make Arduino clear theta param
		print "Reset wheel odom" 
		return EmptyResponse()
	# Read data from serial, called by timer
	def read_data(self, event):
		data_str = self.ard.readline()
		data_list = data_str.split()
		try:
			data_list = [float(i)/100 for i in data_list]
		except ValueError:
			print "incorrect data"
			return # incorrect data
		if len(data_list) != 3:
			print "incorrect array size"
			return # incorrect array size
		# We use 36 RPM motor -> 36/60*2*pi*0.032 = 0.12 m/s
		# Take two times as limitation
		if not in_range(data_list[0:2], -0.24, 0.24):
			print "out of range"
			return # data not in range
		self.v_r, self.v_l, heading = data_list
		# dead reckoning
		dt = rospy.Time.now().to_sec() - self.time.to_sec() # time difference
		self.time = rospy.Time.now() # update time
		v = (self.v_r + self.v_l) / 2
		omega = (self.v_r - self.v_l) / WIDTH
		sth = sin(self.heading)
		cth = cos(self.heading)
		dth = omega * dt
		if self.v_r != self.v_l:
			R = (self.v_r + self.v_l) / (self.v_r - self.v_l) * WIDTH / 2
			A = cos(dth) -1
			B = sin(dth)
			self.x += R*(sth*A  + cth*B)
			self.y += R*(cth*-A + sth*B)
		else: # go straight
			self.x += v*dt*cth
			self.y += v*dt*sth
		self.heading = heading
		if self.pub_tf:
			# Broadcast transform from odom to car_base
			self.tf_br.sendTransform((self.x, self.y, 0),
			                         (0, 0, sin(self.heading/2), cos(self.heading/2)),
			                         rospy.Time.now(),
			                         'car_base',
			                         'odom') 
		# Publish odometry message
		odom = Odometry()
		odom.header.frame_id = 'odom'
		odom.header.stamp = rospy.Time.now()
		odom.pose.pose.orientation.z = sin(self.heading/2)
		odom.pose.pose.orientation.w = cos(self.heading/2)
		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.covariance[0] = 0.1 # X
		odom.pose.covariance[7] = 0.1 # Y
		odom.pose.covariance[35] = 0.4 # RZ
		self.pub_odom.publish(odom)
		# Visulize the path robot traversed 
	# sub_cmd callback, get two wheel desired velocity and try to complete it through PID controllers
	def cmd_cb(self, msg):
		# Reach so v = omega = 0
		if msg.linear.x == 0 and msg.angular.z == 0:
			print "reach"
			self.pid_r.auto_mode = False
			self.pid_l.auto_mode = False
			self.motor_motion(0, 0)
		# Make sure two wheel velocity not invalid values
		if not isnan(self.v_r) and not isnan(self.v_l): 
			self.pid_r.auto_mode = True
			self.pid_l.auto_mode = True
			v_d = msg.linear.x # desired velocity
			w_d = msg.angular.z # desired angular velocity
			v_d_r = v_d + WIDTH/2*w_d # desired right wheel velocity
			v_d_l = v_d - WIDTH/2*w_d # desired left wheel velocity
			# Set the setpoint of controller to desired one
			self.pid_r.setpoint = v_d_r
			self.pid_l.setpoint = v_d_l
			# Get PWM value from controller
			pwm_r = self.pid_r(self.v_r)
		  	pwm_l = self.pid_l(self.v_l)
			# Send command to motors
			self.motor_motion(pwm_r, pwm_l)
	# Send command to motors
	# pwm_r: right motor PWM value
	# pwm_l: left motor PWM value
	def motor_motion(self, pwm_r, pwm_l):
		#print self.v_r, " ", self.v_l
		if pwm_r < 0:
			right_state = Adafruit_MotorHAT.BACKWARD
			pwm_r = -pwm_r
		elif pwm_r > 0:
			right_state = Adafruit_MotorHAT.FORWARD
		else:
			right_state = Adafruit_MotorHAT.RELEASE
		if pwm_l < 0:
			left_state  = Adafruit_MotorHAT.BACKWARD
			pwm_l = -pwm_l
		elif pwm_l > 0:
			left_state = Adafruit_MotorHAT.FORWARD
		else:
			left_state = Adafruit_MotorHAT.RELEASE
		self.right_motor.setSpeed(int(pwm_r))
		self.left_motor.setSpeed(int(pwm_l))
		self.right_motor.run(right_state)
		self.left_motor.run(left_state)
		if pwm_r == 0 and pwm_l == 0:
			rospy.sleep(1.0)
	# Shutdown function, call when terminate
	def shutdown(self):
		self.sub_cmd.unregister()
		rospy.sleep(1.0)
		self.right_motor.setSpeed(0)
		self.left_motor.setSpeed(0)
		self.right_motor.run(Adafruit_MotorHAT.RELEASE)
		self.left_motor.run(Adafruit_MotorHAT.RELEASE)
		del self.motorhat	

if __name__ == '__main__':
	rospy.init_node('pid_controller_node')
	controller = Car_controller()
	rospy.on_shutdown(controller.shutdown)
	rospy.spin()
