#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from  geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from serial import Serial, SerialException
from time import sleep
import tf

if __name__=='__main__':

        pub1=rospy.Publisher('/imu_data',Imu,queue_size=10)
	pub2=rospy.Publisher('/odom', Odometry, queue_size=10)

        rospy.init_node('node',anonymous=True)
        rate=rospy.Rate(30)

	try:
		ser=Serial("/dev/ttyACM2",9600)
	except:
		print "fuck"

        sq=0
	line=''
	data=['']
	
	sleep(.5)

	temp=0
	prev_time=rospy.Time.now()
        while not rospy.is_shutdown():
		try:
			line=ser.read_until()
		except SerialException:
			print "Serial Port Depleted!"
			continue
		data=line.split()
                time=rospy.Time.now()
		try:
                	gyro=float(data[0])
			enc=float(data[1])                
			acc=float(data[2])
		except (ValueError,IndexError):
			print "Just Read Contaminated Values!"
			continue
                msg1=Imu()
       
                msg1.header.stamp=time
                msg1.header.frame_id='link_chassis'
                msg1.header.seq=sq
                msg1.orientation.z =gyro
                 
                #msg1.linear_acceleration.x = acc*9.8
                #msg1.orientation_covariance=[2.6030820491461885e-07, 0.0, 0.0, 0.0, 2.6030820491461885e-07, 0.0, 0.0, 0.0, 0.0]
                msg1.angular_velocity_covariance= [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]
                #msg1.linear_acceleration_covariance = [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]
                #roll=msg1.orientation.x
                #pitch=msg1.orientation.y
                #yaw=msg1.orientation.z
                #qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
                #qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
                #qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
               # qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
                qz=msg1.orientation.z*3.141592654/180.0
                msg1.angular_velocity.z=(qz-temp)/float(str(time-prev_time))*1e+9
                temp=qz
                #msg1.orientation.x=qx
                #msg1.orientation.y=qy
                #msg1.orientation.z=qz
            
                pub1.publish(msg1)

                msg2=Odometry()
                msg2.header.seq=sq
                msg2.header.stamp = msg1.header.stamp
                msg2.header.frame_id= 'odom'
                msg2.child_frame_id='link_chassis'
                msg2.pose.pose.position.x=enc/100.0
                msg2.pose.covariance =   [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03]

               # msg2.twist.covariance =  [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03]


                pub2.publish(msg2)
		print "OK\n"
##################################################################################################
                sq=sq+1
		ser.set_input_flow_control(True)
		rate.sleep()
		#ser.set_input_flow_control(False)
   		#ang_vel=(gyro-old_gyro)*10*3.141592654/180.0
                #lin_vel=(enc-old_enc)/0.1
                #msg = Twist()
                #msg.linear.x=lin_vel
                #msg2.angular.z=ang_vel
                #old_gyro=gyro
                #old_enc=enc
                #pub2.publish(msg)
                prev_time=time
                rate.sleep();

