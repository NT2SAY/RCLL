#!/usr/bin/env python3
import roslib; roslib.load_manifest('laser_assembler')
import rospy; from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud2, PointCloud, Image



rospy.init_node("test_client")
rospy.wait_for_service("assemble_scans2")

pc_pub = rospy.Publisher('/pc2', PointCloud2, queue_size=10)

rate = rospy.Rate(5.0)
count = 0
while not rospy.is_shutdown():
    try:
        assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
        resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
        pc_pub.publish(resp.cloud)
        #print (resp)
    except rospy.ServiceException as e:
        print (f"Service call failed: %s"%e)
    
    print('asdfadsf')
    rate.sleep()