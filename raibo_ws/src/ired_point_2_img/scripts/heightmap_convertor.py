#!/usr/bin/env python3

import concurrent.futures
import time
from asyncio import as_completed

import cv2
import numpy as np
import ros_numpy
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import (Point, PointStamped, Pose, Quaternion, Twist,
                               Vector3)
from nav_msgs.msg import MapMetaData
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Empty


def createImage(meta_data, p_):
    #print(meta_data, p_)
    height = meta_data['height']
    width = meta_data['width']
    rgb_image = np.zeros((height,width,3), np.uint8) # B G R
    depth_image = np.zeros((height,width), np.float32)

    o_w = meta_data['o_w']
    o_h = meta_data['o_h']

    res = meta_data['res']
    m_width = meta_data['m_width']
    m_height = meta_data['m_height']

    for p in p_:
        #print(p)
        if np.isnan(p['x']) or np.isnan(p['y']) or np.isnan(p['z']):
            continue
        
        #print(p['x'],p['y'])
        if((p['y'] <= o_w) and (p['y'] >= (o_w - m_width + res))) and ((p['x'] <= o_h) and (p['x'] >= (o_h - m_height + res))):
            w = (o_w - p['y'])/res
            h = (o_h - p['x'])/res
            w = int(round(w))
            h = int(round(h))
            d = round(p['z'],4)
            #print(d,((d+0.10) * 10_000))
            if(d >= (0.08) and d <= (0.30)):
            
                rgb_image[h,w,0] = p['b']
                rgb_image[h,w,1] = p['g']
                rgb_image[h,w,2] = p['r']
                depth_image[h,w] = (d * 100_000)

    #print(rgb_image.all())
    return (rgb_image, depth_image)

class HEIGHTMAP_CONVERTOR:
    def __init__(self):
        rospy.Subscriber("/tf_transform_cloud/output", PointCloud2, self.pointCloudSubCallBack)
        self.pc_msg = PointCloud2

        rospy.Service('/capture_heightmap', Empty, self.captureCallBack)
        self.first_recive = False

        self.tf_listener = tf.TransformListener()
        self.base_frame = 'base_cartesian_link'
        self.tray_frame = 'pc_capture_link'

        self.m_height = 0.48 #meter
        self.m_width = 0.64 #meter
        self.res = round(0.0010,4) #meter
        self.height = round(self.m_height / self.res)
        self.width = round(self.m_width / self.res)

        self.rgb_image = np.zeros((self.height,self.width,3), np.uint8) # B G R
        self.depth_image = np.zeros((self.height,self.width), np.uint16)

        self.bridge = CvBridge()
        self.rgb_pub = rospy.Publisher('heightmap/rgb/raw', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('heightmap/depth/raw', Image, queue_size=10)

        self.map_meta_data_pub = rospy.Publisher('heightmap/map_meta_data', MapMetaData, queue_size=10)
        self.map_meta_data_msg = MapMetaData()

        self.map_meta_data_msg.height = self.height
        self.map_meta_data_msg.width = self.width
        self.map_meta_data_msg.resolution = self.res

        self.process_status_pub = rospy.Publisher('heightmap/process_status', Bool, queue_size=10)
        self.process_status_msg = Bool()
        self.process_status_msg.data = True

    def pointCloudSubCallBack(self, data):
        self.pc_msg = data
        self.first_recive = True

    def captureCallBack(self,req):
        if not self.first_recive:

            return
        print('Capture')
        start_time = time.perf_counter()
        self.process_status_msg.data = False
        (self.tray_point,self.tray_rot) = self.getTrayPoint()

        #print(self.tray_point)

        # self.o_w = self.tray_point[1] - (self.m_width/2.0)
        # self.o_h = self.tray_point[0] - (self.m_height/2.0)

        self.o_w = (self.m_width/2.0) # y axis
        self.o_h = (self.m_height/2.0) # x axis

        #print(self.o_w,self.o_h)

        self.map_meta_data_msg.origin.position.x = self.o_h
        self.map_meta_data_msg.origin.position.y = self.o_w

        pc_frame = self.pc_msg.header.frame_id

        meta_data = {'o_w':self.o_w, 'o_h':self.o_h, 'm_width':self.m_width, 'm_height':self.m_height, 'res':self.res, 'height':self.height, 'width':self.width}

        pc = ros_numpy.numpify(self.pc_msg)
        #print(pc)
        pc_rgb = ros_numpy.point_cloud2.split_rgb_field(pc)
        pc_rgb = pc_rgb.flatten()
        #print(pc_rgb) #pc_rgb['x'] pc_rgb['y'] pc_rgb['z'] pc_rgb['r'] pc_rgb['g'] pc_rgb['b']

        rgb_image = np.zeros((self.height,self.width,3), np.uint8) # B G R
        depth_image = np.zeros((self.height,self.width), np.float32)

        process_num = 20
        batch_size = int(pc_rgb.size/process_num)
        pc_rgb_list = []
        for i in range(process_num):
            start_point = i
            stop_point = i + 1
            #print(pc_rgb.size,start_point*batch_size,stop_point*batch_size)
            pc_rgb_list.append(pc_rgb[start_point*batch_size:stop_point*batch_size])
        
        with concurrent.futures.ProcessPoolExecutor() as executor:
            results = [executor.submit(createImage,meta_data,p) for p in pc_rgb_list]

            for f in concurrent.futures.as_completed(results):

                res_rgb_img,res_depth_image = f.result()
                rgb_image[np.where(rgb_image == 0)] = res_rgb_img[np.where(rgb_image == 0)]
                depth_image[np.where(depth_image == 0)] = res_depth_image[np.where(depth_image == 0)]

        self.rgb_image = rgb_image
        self.depth_image = depth_image.astype(np.uint16)
        self.process_status_msg.data = True
        finish_time = time.perf_counter()
        print(f'Done Finished in {round(finish_time-start_time,2)} second(s)')
        
        return

    def getTrayPoint(self):
        (trans,rot) = self.tf_listener.lookupTransform(self.base_frame, self.tray_frame, rospy.Time(0))
        return (trans,rot)


    def process(self):
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            
            try:
                rgb_msg = self.bridge.cv2_to_imgmsg(self.rgb_image, encoding='bgr8')
                self.rgb_pub.publish(rgb_msg)
                depth_image = self.bridge.cv2_to_imgmsg(self.depth_image, encoding='mono16')
                self.depth_pub.publish(depth_image)
            except CvBridgeError:
                print('Error')
            self.map_meta_data_msg.map_load_time = current_time
            self.map_meta_data_pub.publish(self.map_meta_data_msg)

            self.process_status_pub.publish(self.process_status_msg)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('heightmap_convertor', anonymous=True)
    heightmap_convertor = HEIGHTMAP_CONVERTOR()
    heightmap_convertor.process()