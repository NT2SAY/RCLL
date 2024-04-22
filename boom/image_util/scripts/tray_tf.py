#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_srvs.srv import Empty

import numpy as np

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class TRAY_TF:
    def __init__(self):
        rospy.Service('save_tray_pose', Empty, self.savePoseCallback)
        self.tray_tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        self.base_frame = 'base_link'
        self.tray_frame = 'tray_frame'
        self.tray_from_aruco_frame = 'tray_from_aruco'

        self.scaleweight_prot_frame = 'scaleweight_prot_plane1'

        self.pose = Pose()
        self.pose.position.x = 0
        self.pose.position.y = 0
        self.pose.position.z = 0

        self.pose.orientation.x = 0
        self.pose.orientation.y = 0
        self.pose.orientation.z = 0
        self.pose.orientation.w = 1


    def savePoseCallback(self, req):
        print('Save Pose')
        sampling_trans = []
        sampling_rot = []
        for _ in range(20):
            try:
                (trans,rot) = self.tf_listener.lookupTransform(self.base_frame, self.tray_from_aruco_frame, rospy.Time(0))
                #print(trans)
                # if 0.080 <= trans[2] <= 0.095:
                #     sampling_trans.append(trans)
                #     sampling_rot.append(rot)
                sampling_trans.append(trans)
                sampling_rot.append(rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            rospy.sleep(0.01)
        
        #get
        (s_trans,s_rot) = self.tf_listener.lookupTransform(self.base_frame, self.scaleweight_prot_frame, rospy.Time(0))

        sampling_trans = np.array(sampling_trans)
        #print(sampling_trans)
        trans = np.median(sampling_trans,axis=0)

        self.pose.position.x = trans[0]
        self.pose.position.y = trans[1]
        self.pose.position.z = s_trans[2] +0.005

        sampling_rot = np.array(sampling_rot)
        rot = np.median(sampling_rot,axis=0)

        orientation_list = [rot[0], rot[1], rot[2], rot[3]]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        roll = self.snapRotation(roll)
        pitch = self.snapRotation(pitch)
        yaw = self.snapRotation(yaw)

        quat = quaternion_from_euler (roll, pitch,yaw)
        print(trans, quat)

        self.pose.orientation.x = quat[0]
        self.pose.orientation.y = quat[1]
        self.pose.orientation.z = quat[2]
        self.pose.orientation.w = quat[3]

    def process(self):
        
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            
            self.tray_tf_broadcaster.sendTransform(
                (self.pose.position.x, self.pose.position.y, self.pose.position.z),
                (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w),
                current_time,
                self.tray_frame,
                self.base_frame
            )
            rate.sleep()

    def snapRotation(self, _angle):
        angle_array = np.array([(-4.0 / 2.0) * pi, (-3.0 / 2.0) * pi, (-2.0 / 2.0) * pi, (-1.0 / 2.0) * pi, 0, (1.0 / 2.0) * pi, 1.0 * pi, (3.0 / 2.0) * pi, (4.0 / 2.0) * pi])

        abs_array = np.absolute(angle_array-_angle)
        return angle_array[np.argmin(abs_array)]


if __name__ == '__main__':
    rospy.init_node('tf_tray', anonymous=True)
    tray_tf = TRAY_TF()
    tray_tf.process()
