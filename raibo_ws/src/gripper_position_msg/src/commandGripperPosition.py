#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3

def sendCommand():
	pub = rospy.Publisher("commanderGripperPosition", Vector3, queue_size=10)
	rospy.init_node("commandPublisher", anonymous=True)
