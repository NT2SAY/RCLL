#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Vector3
import time
midx = 107
midy = 160
z_pos = 28

template = cv2.imread(r'/home/rai/raibocup/raibo_ws/src/img_msg/src/temp.png')
(template_r,_,_) = cv2.split(template)
w,h = template_r.shape[::-1]

class getDepthImage():
    def __init__(self):
        self.image = None
        self.image2 = None
        self.getImage = False
        self.sub = None
        self.br = CvBridge()
        # self.loop_rate = rospy.Rate(1)
        
        self.save_image_rgb = None
        
        self.pub = rospy.Publisher("/gripper/command/position", Vector3, queue_size=10)
        rospy.Subscriber("/heightmap/rgb/raw", Image, self.handle_raw_image)
        
        s = rospy.Service("find/place", Empty, self.handle_place)
        
        # rospy.Subscriber("/heightmap/depth/raw", Image, self.callbackDepth)
    
    def handle_raw_image(self, msg):
        self.rawImage = msg
    
    def handle_place(self, req):
        rospy.loginfo("finding position to pick")
        rospy.wait_for_service('/capture_heightmap')
        try:
            capture_heightmap = rospy.ServiceProxy('/capture_heightmap', Empty)
            resp1 = capture_heightmap()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            # depth_img.saveImg()
        
        self.callBackRGB()
        return EmptyResponse()
        
        
    def callBackRGB(self):
        self.Image  = self.br.imgmsg_to_cv2(self.rawImage)
        
        (R,G,B) = cv2.split(self.Image)
        kernel = np.ones((5, 5), np.uint8)
        closing_R = cv2.morphologyEx(R, cv2.MORPH_CLOSE, kernel)
        closing_G = cv2.morphologyEx(G, cv2.MORPH_CLOSE, kernel)
        closing_B = cv2.morphologyEx(B, cv2.MORPH_CLOSE, kernel)
        merge =cv2.merge([closing_R, closing_G, closing_B])
        self.save_image_rgb = merge
        # cv2.imshow("detected", self.save_image_rgb)
        # merge_1 = cv2.convertScaleAbs(merge)
        merge_1 = cv2.cvtColor(merge, cv2.COLOR_RGB2GRAY)
        res = cv2.matchTemplate(closing_R,template_r,cv2.TM_CCOEFF_NORMED)
        threshold = 0.62
        loc = np.where(res >= threshold)
        
        pose_all = []
        for pt in zip(*loc[::-1]):
            if pt[1] > 90:
                poseX = (pt[0]+w/2)-320+midx
                poseY = 240-(pt[1]+h/2)+midy
                pose = [poseX, poseY]
                pose_all.append(pose)
                rospy.loginfo(f"point:{poseX}, {poseY}")
                cv2.rectangle(merge, pt, (pt[0]+w, pt[1]+h), (0,255,255), 2)
        # cv2.destroyAllWindows()
        if len(pose_all) != 0:
            sorted_pose_all = sorted(pose_all, key=lambda coord: coord[0])
            rospy.loginfo(sorted_pose_all)
            vector_msg = Vector3()
            vector_msg.x = sorted_pose_all[0][0]
            vector_msg.y = sorted_pose_all[0][1]
            vector_msg.z = z_pos
            self.pub.publish(vector_msg)
        else:
            self.callBackRGB()
            
        
        
if __name__ == '__main__':
    rospy.init_node("fine2pick", anonymous=True)
    
    
    depth_img = getDepthImage()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        rate.sleep()
