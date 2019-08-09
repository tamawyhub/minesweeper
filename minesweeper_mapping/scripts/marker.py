#! /usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose2D
import tf2_ros
import std_srvs.srv
from minesweeper_mapping.srv import Add_mine
from minesweeper_mapping.srv import *
import minesweeper_mapping.srv
import math
import tf_conversions
import geometry_msgs.msg
from time import sleep

rospy.init_node('marker',anonymous = False)
rospy.wait_for_service('/minesweeper/get_pose')
tfBuffer=tf2_ros.Buffer()
broadcaster = tf2_ros.TransformBroadcaster()
listener=tf2_ros.TransformListener(tfBuffer)
rate = rospy.Rate(10)
vis_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size = 10)

marker = Marker()
marker.header.frame_id = "minesweeper/odom"
marker.header.stamp = rospy.Time()
marker.ns = 'my_namespace'
marker.id = 0
marker.type = Marker.CUBE
marker.action = Marker.ADD
marker.pose.position.x = 1
marker.pose.position.y = 1
marker.pose.position.z = 0
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0
marker.scale.x = 1.0
marker.scale.y = 1.0
marker.scale.z = 0.0
marker.color.a = 1.0
marker.color.r = 0.0
marker.color.g = 0.0
marker.color.b = 1.0
markerArray = MarkerArray()

#gripper_to_base_link=.66
f= open("vectorMap.txt","w+")
def handle_add_mine(req):
	global f
	global gripper_to_base_link

	t = geometry_msgs.msg.TransformStamped()
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "minesweeper/odom"
	t.child_frame_id = 'link_chassis'
	
	get_pose = rospy.ServiceProxy('/minesweeper/get_pose', Pose)
        resp = get_pose()
	#rospy.loginfo (str(resp.theta))
	t.transform.translation.x = resp.x
	t.transform.translation.y = resp.y
	t.transform.translation.z = 0.0
	
	q = tf_conversions.transformations.quaternion_from_euler(0, 0, resp.theta)
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]

	broadcaster.sendTransform(t)
	#rospy.loginfo("transform sent")
	sleep(.2)
	trans = tfBuffer.lookup_transform('minesweeper/odom', 'metal_detector_link', rospy.Time(0))
        x= trans.transform.translation.x
	y= trans.transform.translation.y
	#th= trans.transform.rotation.z
	#x=x+gripper_to_base_link*math.cos(th)
	#y=y+gripper_to_base_link*math.sin(th)
	marker.pose.position.x = math.ceil(x)-.5
	marker.pose.position.y = math.ceil(y)-.5
	
	if req.isBurried:
		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 0.0
	else:
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0

	markerArray.markers.append( marker)
	vis_pub.publish(markerArray)
	marker.id = marker.id + 1
	coord=str(chr(int(y)+ord('A')-1))
	coord=coord+str(int(x))
	if req.isBurried:
		coord=coord+' \"Burried\"'
	else:
		coord=coord+' \"Surface\"'
	f.write(coord)
	f.write('\n')
	return minesweeper_mapping.srv._Add_mine.Add_mineResponse()

s = rospy.Service('add_mine', Add_mine, handle_add_mine)
#vis_sub = rospy.Subscriber("mine_pose", Pose2D, callback)

if __name__ == '__main__':
	#vis_pub.publish(markerArray)
	rospy.spin()
	f.close()

