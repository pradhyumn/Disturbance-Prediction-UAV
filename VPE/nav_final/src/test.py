#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
import tf
from tf.transformations import quaternion_from_euler

def callback(msg):
	
	cur_pose.header = msg.header
	
	cur_pose.pose.position.x = msg.position.x
	cur_pose.pose.position.y = msg.position.y
	cur_pose.pose.position.z = msg.position.z
	print(cur_pose.pose.position.z)

	#convert yaw to orientation msg.yaw
	roll = 0
	pitch = 0
	quaternion = tf.transformations.quaternion_from_euler(roll, pitch, msg.yaw)

	cur_pose.pose.orientation.x = quaternion[0]
	cur_pose.pose.orientation.y = quaternion[1]
	cur_pose.pose.orientation.z = quaternion[2]
	cur_pose.pose.orientation.w = quaternion[3]
	path.header = msg.header
	path.poses.append(cur_pose)
	path_pub.publish(path)
	print("-- Navigation Message Received --")


rospy.init_node('Nav_final')
path = Path()
cur_pose = PoseStamped()

path.header.frame_id = "map"


position_sub = rospy.Subscriber('/planning/pos_cmd', PositionCommand, callback)
path_pub = rospy.Publisher('/mavros/trajectory/path', Path, latch=True, queue_size=10)

if __name__ == '__main__':
	rospy.spin()
