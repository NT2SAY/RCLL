#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3

nodeName = "gripper_pos_message"
topicName = "/gripper/status"

def callBackFunction(message):
	rospy.loginfo(f"Gripper Status: x = {message.x}, y = {message.y}, z = {message.z}")
	
rospy.init_node(nodeName, anonymous=True)
rospy.Subscriber(topicName, Vector3, callBackFunction)
rospy.spin()
