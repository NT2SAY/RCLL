#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3

nodeName = "gripper_pos_message"
topicName = "GripperPosition"

def callBackFunction(message):
	rospy.loginfo(f"Recieving Gripper Position: x = {message.x:.2f}, y = {message.y:.2f}, z = {message.z:.2f}")
	
rospy.init_node(nodeName, anonymous=True)
rospy.Subscriber(topicName, Vector3, callBackFunction)
rospy.spin()

