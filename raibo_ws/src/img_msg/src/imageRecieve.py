#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
from std_srvs.srv import Empty
import time
import torch
from torchvision.transforms import functional as F
template = cv2.imread(r'/home/rai/raibocup/raibo_ws/src/img_msg/src/temp.png')
(template_r,_,_) = cv2.split(template)
w,h = template_r.shape[::-1]
midx = 107
midy = 160
class getDepthImage():
    def __init__(self):
        self.image = None
        self.image2 = None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(1)
        
        self.save_image_depth = None
        self.save_image_rgb = None
        
        # rospy.Subscriber("/heightmap/rgb/raw", Image, self.callBackRGB)
        rospy.Subscriber("/heightmap/depth/raw", Image, self.callbackDepth)
        
    
    def callbackDepth(self, msg):
        max_value = 0.15 * 100_000
        min_value = 0.07 * 100_000
        rospy.loginfo("Image Recieved..")
        self.image = self.br.imgmsg_to_cv2(msg)
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
                    rospy.loginfo(f"centroid: {cx-320+midx}, {240-cy+midy}")
                else:
                    print("Zero division error.")
        cv2.imshow("", closing)
        cv2.waitKey(30)
        # cv2.imshow("equa",closing)
        # cv2.waitKey(0)
        
        self.save_image_depth = closing.copy()
        
    def callBackRGB(self, msg):
        self.image2 = self.br.imgmsg_to_cv2(msg)
        
        (R,G,B) = cv2.split(self.image2)
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
        
        for pt in zip(*loc[::-1]):
            if pt[1] > 90:
                rospy.loginfo(f"point:{(pt[0]+w/2)-320+midx}, {240-(pt[1]+h/2)+midy}")
                cv2.rectangle(merge, pt, (pt[0]+w, pt[1]+h), (0,255,255), 2)
        
        
        cv2.imshow("",  merge)
        cv2.waitKey(0)
        # cv2.destroyAllWindows()
    
    
            
    def saveImg(self):
        # cv2.imshow("equa",self.save_image_depth)
        # cv2.waitKey(0)
        # cv2.imwrite(f"depth_{round(time.time())}.png",self.save_image_depth*255)
        cv2.imwrite(f"rgb_{round(time.time())}.png",self.save_image_rgb)
        
if __name__ == '__main__':
    rospy.init_node("equDepth", anonymous=True)
    depth_img = getDepthImage()
    rate = rospy.Rate(20) # 10hz
    counter = 0

    while not rospy.is_shutdown():
        

        if counter >= 10:
            counter = 0
            rospy.wait_for_service('/capture_heightmap')
            try:
                capture_heightmap = rospy.ServiceProxy('/capture_heightmap', Empty)
                resp1 = capture_heightmap()
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            # depth_img.saveImg()
            
        counter += 1
        rate.sleep()
        