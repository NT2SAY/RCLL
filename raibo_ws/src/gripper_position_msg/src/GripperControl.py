#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool, Int16
from serial import Serial
from math import pow
from time import sleep as delay


nodeName = "gripper_stage_message"
topicName = "GripperState"

class GripperControl():
    currentDeg = 0
    openGripper = 70
    closeGripper = 115
    gripperDelay = 1.5
    stat = None
    
    def __init__(self, port):
        self.ser = Serial(port, 115200, timeout= 1)
        self.statCallback = rospy.Publisher("/gripper/stat", Int16, queue_size= 10)
        rospy.Subscriber("/gripper/command", String, self.commandHandle)
        self.ser.reset_input_buffer
        
        self.sendCallback(2)
        
    def sendCallback(self, stat):    
        # 0 - task done
        # 1 - activate
        # 2 - waiting
        self.stat = stat
        cmd = Int16()
        cmd.data = stat
        self.statCallback.publish(cmd)
    
    def statHandle(self, stat):
        self.stat = stat.data
        
    def commandHandle(self, cmd):
        print(cmd.data)
        if cmd.data == "OPEN":
            self.sendCallback(1)
            self._open()
            rospy.sleep(self.gripperDelay)
            self.sendCallback(0)
        elif cmd.data == "CLOSE":
            self.sendCallback(1)
            self._close()
            rospy.sleep(self.gripperDelay)
            self.sendCallback(0)
        else:
            self.sendCallback(2)

    def XORsum(self, a):
        out = not a[0]
        for i in a:
            out = out ^ i
            
        return out
    
    def degToHex(self, deg):
        steps = f'{round(deg * 267.42857):04x}'
        return f'{steps[0:2]} {steps[2:]}'
    
    def getHexbit(self, dir, deg, show= False):
        in_bits = f'7b 01 02 0{dir} 20 {self.degToHex(deg)} 00 c8'
        bits = [int(i, 16) for i in in_bits.split()]
        msg = (''.join(in_bits)) + ' ' +  f'{self.XORsum(bits):02x} 7d'
        if show: print('Hexbits:', msg)
        
        return bytes.fromhex(msg)
        

    def drive(self, dir, deg, show = False):
        self.currentDeg += pow(-1, not dir) * deg
        hexbit = self.getHexbit(dir, deg, show)
        self.ser.write(hexbit)
        
    def driveTo(self, pos, show = False):
        deg = pos - self.currentDeg
        
        if deg >= 0: hexbit = self.getHexbit(1, deg, show)
        else: hexbit = self.getHexbit(0, -1 * deg, show)
        
        self.currentDeg+= deg
        if show: print('Current Pos:', self.currentDeg)
        
        self.ser.write(hexbit)
        
    def _open(self):
        self.driveTo(self.openGripper)
    
    def _close(self):
        self.driveTo(self.closeGripper)
    
    def setHome(self):
        self.drive(1, 200)
        delay(2)
        self.drive(0, 100)
        self.currentDeg = 0
        
        
def callBackFunction(message):
    if message.data == 1:
        gripper._open()
    elif message.data == 0:
        gripper._close()
    
	

if __name__ == '__main__':
    
    
    
    rospy.init_node('gripper_control_server')
    gripper = GripperControl('/dev/gripper')
    gripper.setHome()
    delay(0.5)
    gripper.setHome()
    rospy.spin()
    
    # rospy.init_node(nodeName, anonymous=True)
    # rospy.Subscriber(topicName, Int16, callBackFunction)
    # rospy.spin()     


    # while True:
    #     _in = input('Enter: ')
    #     try:
    #         _dir, _deg = [int(i) for i in _in.split()]
    #     except:
    #         _deg = int(_in)
        
    #     # print(_dir, _deg)
    #     # gripper.drive(_dir, _deg, True)
    #     # print(gripper.currentDeg)
        
    #     # gripper.driveTo(_deg, True)
    #     # print(gripper.degToHex(70))
        
    #     if _deg == 1:
    #         gripper._open()
    #     elif _deg == 0:
    #         gripper._close()
    #     else:
    #         gripper.drive(_dir, _deg, True)          

        
        
