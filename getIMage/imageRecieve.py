#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
from std_srvs.srv import Empty
import time

class getDepthImage():
    def __init__(self):
        self.image = None
        self.image2 = None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(1)
        
        self.save_image_depth = None
        self.save_image_rgb = None
        rospy.Subscriber("/heightmap/rgb/raw", Image, self.callBackRGB)
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
        
        
        kernel = np.ones((3, 3), np.uint8)
        closing = cv2.morphologyEx(normal_img, cv2.MORPH_CLOSE, kernel)
        # closing = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)
        # closing = cv2.morphologyEx(closing, cv2.MORPH_CLOSE, kernel)
        # cv2.imshow("Image", closing)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # hist,bins = np.histogram(self.image.flatten(),256,[0,256])
        # convert = cv2.convertScaleAbs(self.image)
        # rospy.loginfo(convert.dtype)
        # equ = cv2.equalizeHist(convert)
        # # cdf = hist.cumsum()
        # # cdf_normalized = cdf * float(hist.max()) / cdf.max()
        # # cdf_m = np.ma.masked_equal(cdf,0)
        # # cdf_m = (cdf_m - cdf_m.min())*255/(cdf_m.max()-cdf_m.min())
        # # cdf = np.ma.filled(cdf_m,0).astype('uint8')
        # # img2 = cdf[convert]
        # cv2.imshow("equa",closing)
        # cv2.waitKey(0)
        
        self.save_image_depth = closing.copy()
 
        # plt.plot(cdf_normalized, color = 'b')
        # plt.hist(self.image.flatten(),256,[0,256], color = 'r')
        # plt.xlim([0,256])
        # plt.legend(('cdf','histogram'), loc = 'upper left')
        # plt.show()
        
        
        # cv2.imshow("equalisation",equ)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        
    def callBackRGB(self, msg):
        self.image2 = self.br.imgmsg_to_cv2(msg)
        self.save_image_rgb = self.image2
        
    def saveImg(self):
        # cv2.imshow("equa",self.save_image_depth)
        # cv2.waitKey(0)
        cv2.imwrite(f"depth_{round(time.time())}.png",self.save_image_depth*255)
        cv2.imwrite(f"rgb_{round(time.time())}.png",self.save_image_rgb)
        
if __name__ == '__main__':
    rospy.init_node("equDepth", anonymous=True)
    depth_img = getDepthImage()
    rate = rospy.Rate(10) # 10hz
    counter = 0
    while not rospy.is_shutdown():

        if counter >= 20:
            counter = 0
            rospy.wait_for_service('/capture_heightmap')
            try:
                capture_heightmap = rospy.ServiceProxy('/capture_heightmap', Empty)
                resp1 = capture_heightmap()
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            depth_img.saveImg()
            
        counter += 1
        rate.sleep()
        