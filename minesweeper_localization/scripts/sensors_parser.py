#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from  geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from serial import Serial, SerialException
from time import sleep
import tf
import glob
import math
from minesweeper_mapping.srv import *

x=0.0
y=0.0
th=0.0
th_offset=0.0

def callback(msg):
	global x
	global y
	global th
	global th_offset
	if msg.x>0.0 and msg.y>=0.0:
		x=msg.x
		y=msg.y
	th_offset=msg.theta-th

def handle_get_pose(req):
	global x
	global y
	global th
	#rospy.loginfo(str(th))
	resp=PoseResponse(x,y,th)
	return resp

sub=rospy.Subscriber('/minesweeper/set_pose',Pose2D,callback)
pub=rospy.Publisher('/minesweeper/pose', Pose2D, queue_size=10)
rospy.init_node('sensors_parser_node',anonymous=False)
s = rospy.Service('/minesweeper/get_pose', Pose, handle_get_pose)
rate=rospy.Rate(30)
ports=glob.glob('/dev/tty[A-Za-z]*')
for port in ports:
	try:
		ser=Serial(port,9600)
	except SerialException:
		rospy.loginfo("Unavailable port")
		continue
	rospy.loginfo("Found device at port %s",port)
	break

sq=0
line=''
data=['']
	
sleep(.5)

old_enc=0.0
old_gyro=0.0
while not rospy.is_shutdown():
	try:
		line=ser.read_until()
	except SerialException:
		rospy.loginfo("Serial port unavailable or depleted!")
		continue
	data=line.split()

	try:
                gyro=float(data[0])*math.pi/180.0
		enc=float(data[1])/100.0
	except (ValueError,IndexError):
		rospy.loginfo("Just read contaminated values!")
		continue
	msg=Pose2D()
        th=gyro+th_offset
	msg.theta=th
        diff=enc-old_enc
	x=x+diff*math.cos((gyro+old_gyro)/2.0)
	y=y+diff*math.sin((gyro+old_gyro)/2.0)

	msg.x=x
	msg.y=y

        pub.publish(msg)
	rospy.loginfo ("OK")
	old_enc=enc
	old_gyro=gyro
        rate.sleep()

