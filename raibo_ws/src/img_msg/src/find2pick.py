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
        rospy.Subscriber("/heightmap/depth/raw", Image, self.handle_raw_image)
        
        s = rospy.Service("find/pick", Empty, self.handle_pick)
        
        # rospy.Subscriber("/heightmap/depth/raw", Image, self.callbackDepth)
    
    def handle_raw_image(self, msg):
        self.rawImage = msg
    
    def handle_pick(self, req):
        rospy.loginfo("finding position to pick")
        rospy.wait_for_service('/capture_heightmap')
        try:
            capture_heightmap = rospy.ServiceProxy('/capture_heightmap', Empty)
            resp1 = capture_heightmap()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            # depth_img.saveImg()
        
        self.callbackDepth()
        return EmptyResponse()
        
        
    def callbackDepth(self):
        max_value = 0.15 * 100_000
        min_value = 0.07 * 100_000
        rospy.loginfo("Image Recieved..")
        self.image = self.br.imgmsg_to_cv2(self.rawImage)
        self.image = self.image.astype('float32')
        
        normal_img = (self.image - min_value)/(max_value - min_value)
        normal_img[normal_img<0] = 0
        normal_img[normal_img>1] = 1
        
        
        kernel = np.ones((17, 17), np.uint8)
        closing = cv2.morphologyEx(normal_img, cv2.MORPH_CLOSE, kernel)
        closing2 = closing*255
        ret,th1 = cv2.threshold(closing2,95,255,cv2.THRESH_BINARY)
        th1 = cv2.convertScaleAbs(th1)
        
        # cv2.circle(closing,(320,240), 20, (1), -1)
        contours, hierarchy = cv2.findContours(th1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(closing, contours, -1, 1, 3)
        pose_all = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 1000 < area:
                cv2.drawContours(closing, [cnt], -1, 1, 2)
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])  # X coordinate of the centroid
                    cy = int(M["m01"] / M["m00"])  # Y coordinate of the centroid
                    # Draw the centroid in the output image
                    cv2.circle(closing, (cx, cy), 2, (255, 0, 0), -1)
                    poseX = cx-320+midx
                    poseY = 240-cy+midy
                    if (0 < poseX < 220) and (0 < poseY < 220):
                        pose = [poseX, poseY]
                        pose_all.append(pose)
                    rospy.loginfo(f"postion to move: {poseX}, {poseY}")
                else:
                    print("Zero division error.")
        if len(pose_all) != 0:
            sorted_pose_all = sorted(pose_all, key=lambda coord: coord[0])
            rospy.loginfo(sorted_pose_all)
            vector_msg = Vector3()
            vector_msg.x = sorted_pose_all[0][0]
            vector_msg.y = sorted_pose_all[0][1]
            vector_msg.z = z_pos
            self.pub.publish(vector_msg)
        else:
            self.callbackDepth()
            
        
        
if __name__ == '__main__':
    rospy.init_node("fine2pick", anonymous=True)
    
    
    depth_img = getDepthImage()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        rate.sleep()
