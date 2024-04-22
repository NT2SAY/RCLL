#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Int16

class toptoolSeq():
    def __init__(self):
        self.statusX = False
        self.statusY = False
        self.statusZ = False
        self.isGripperMoved = False
        self.isGripperReady = False
        self.isCommanded = False
        self.isGripped = False
        self.isBack = False
        self.isReadytoGrip = False
        
        self.pick = False
        self.place = False
        self.pickNplace = False
        rospy.init_node("toptool")
        s1 = rospy.Service("toptool/pick", Empty, self.handle_pick_request)
        s2 = rospy.Service("toptool/place", Empty, self.handle_place_request)
        s3 = rospy.Service("toptool/pickNplace", Empty, self.handle_pickNplace_request)
        rospy.Subscriber("/gripper/command/position", Vector3, self.callbackPose)
        rospy.Subscriber("/gripper/status", Vector3, self.updateGripperStatus)
        self.pub = rospy.Publisher("XYZcommanderGripperPosition", Vector3, queue_size=10)
        # self.pub = rospy.Publisher("/gripper/command/x", Int16, queue_size=10)
        self.pub2 = rospy.Publisher("/gripper/command", String, queue_size=10)
        self.pub3 = rospy.Publisher("ZYXcommanderGripperPosition", Vector3, queue_size=10)
        
        rospy.wait_for_service("/cartesian/set_zero")
        try:
            service = rospy.ServiceProxy("/cartesian/set_zero", Empty)
            service()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def handle_pickNplace_request(self, req):
        self.pickNplace = True
        self.handle_pick_request()
        

    def handle_pick_request(self, req):
        self.pick = True
        rospy.loginfo("pick")
        rospy.wait_for_service("find/pick")
        try:
            service = rospy.ServiceProxy("find/pick", Empty)
            service()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        
        return EmptyResponse()
        
    def handle_place_request(self, req):
        self.place = True
        rospy.loginfo("place")
        rospy.wait_for_service("find/place")
        try:
            service = rospy.ServiceProxy("find/place", Empty)
            service()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        
        return EmptyResponse()
        
    def toptool_empty_service():

        rospy.spin()
        
    def updateGripperStatus(self, msg):
        self.statusX = msg.x
        self.statusY = msg.y
        self.statusZ = msg.z
        if(self.statusX and self.statusY and self.statusZ):
            self.isGripperReady = True
        else:
            self.isGripperReady = False
            
        if(self.isCommanded and (not self.isGripperReady) ):
            self.isGripperMoved = True
        if(self.isGripperMoved and self.isGripperReady):
            if self.isReadytoGrip:
                msg = String()
                if self.pick:
                    msg.data = "CLOSE"
                elif self.place:
                    msg.data = "OPEN"
                self.pub2.publish(msg)
                rospy.sleep(1)
                self.isGripped = True
                self.isGripperMoved = False
                self.isCommanded = False
            elif self.isBack:
                rospy.wait_for_service("/cartesian/set_zero")
                try:
                    service = rospy.ServiceProxy("/cartesian/set_zero", Empty)
                    service()
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")
                
                self.statusX = False
                self.statusY = False
                self.statusZ = False
                self.isGripperMoved = False
                self.isGripperReady = False
                self.isCommanded = False
                self.isGripped = False
                self.isBack = False
                self.isReadytoGrip = False
                
                self.pick = False
                self.place = False
                return EmptyResponse
                
                        
        
            
        
    def callbackPose(self, msg):
        rospy.loginfo('callbackPose')
        pos_msg = Vector3()
        pos_msg.x = msg.x
        pos_msg.y = msg.y
        pos_msg.z = msg.z
        self.pub.publish(pos_msg)
        
        # temp = Int16()
        # temp.data = msg.x
        # self.pub.publish(temp)
        self.isCommanded = True
        self.isReadytoGrip = True
        
        
    
if __name__ == '__main__':
    cmd = toptoolSeq()
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        if cmd.pickNplace:
            if (not cmd.pick and (not cmd.place)):
                cmd.pickNplace = False
                cmd.handle_place_request()
                
        if cmd.isGripped:
            vector_msg = Vector3()
            vector_msg.x = 107
            vector_msg.y = 50
            vector_msg.z = 50
            # cmd.pub3(vector_msg)
            cmd.pub3.publish(vector_msg)
            cmd.isReadytoGrip = False
            cmd.isBack = True
            cmd.isCommanded = True
            cmd.isGripped = False
        rate.sleep()