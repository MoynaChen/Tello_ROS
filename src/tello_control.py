#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty, UInt8, Bool,String
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from tello_driver.cfg import TelloConfig
from cv_bridge import CvBridge, CvBridgeError

import time
import math
import sys
sys.path.append('/home/tello/catkin_ws/src/TelloPy/')#/home/#your_name/catkin_ws/src/TelloPy
from tellopy._internal import tello
from tellopy._internal import error
from tellopy._internal import protocol
from tellopy._internal import logger

key_mapping = { 'w': [ 0, 1], 'x': [0, -1], 
                'a': [1, 0], 'd': [-1,  0], 
                's': [ 0, 0] }


global g_last_twist 
g_last_twist = Twist()
        

class RospyLogger(logger.Logger):
    def __init__(self, header=''):
        super(RospyLogger,self).__init__(header)

    def error(self, s):
        if self.log_level < logger.LOG_ERROR:
            return
        rospy.logerr(s)

    def warn(self, s):
        if self.log_level < logger.LOG_WARN:
            return
        rospy.logwarn(s)

    def info(self, s):
        if self.log_level < logger.LOG_INFO:
            return
        rospy.loginfo(s)

    def debug(self, s):
        if self.log_level < logger.LOG_DEBUG:
            return
        rospy.logdebug(s)


def notify_cmd_success(cmd, success):
    if success:
        rospy.loginfo('%s command executed' % cmd)
    else:
        rospy.logwarn('%s command failed' % cmd)


class TelloControlNode(tello.Tello):
    def __init__(self):
        self.local_cmd_client_port = int(
            rospy.get_param('~local_cmd_client_port', 8890))
        self.local_vid_server_port = int(
            rospy.get_param('~local_vid_server_port', 6038))
        self.tello_ip = rospy.get_param('~tello_ip', '192.168.10.1')
        self.tello_cmd_server_port = int(
            rospy.get_param('~tello_cmd_server_port', 8889))
        self.connect_timeout_sec = float(
            rospy.get_param('~connect_timeout_sec', 10.0))
        self.stream_h264_video = bool(
            rospy.get_param('~stream_h264_video', False))
        self.bridge = CvBridge()
        self.frame_thread = None

        # Connect to drone
        log = RospyLogger('Tello')
        log.set_level(self.LOG_WARN)
        super(TelloControlNode,self).__init__(
            local_cmd_client_port=self.local_cmd_client_port,
            local_vid_server_port=self.local_vid_server_port,
            tello_ip=self.tello_ip,
            tello_cmd_server_port=self.tello_cmd_server_port,
            log=log)
        rospy.loginfo('Connecting to drone @ %s:%d' % self.tello_addr)
        self.connect()
        try:
            self.wait_for_connection(timeout=self.connect_timeout_sec)
        except error.TelloError as err:
            rospy.logerr(str(err))
            rospy.signal_shutdown(str(err))
            self.quit()
            return
        rospy.loginfo('Connected to drone')
       

        # Setup topics and services
        
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist_sub = rospy.Subscriber('keys', String, self.keys_cb, self.twist_pub)
        self.sub_takeoff = rospy.Subscriber('keys', String, self.cb_takeoff)
        self.sub_land = rospy.Subscriber('keys', String, self.cb_land)
        self.sub_forward = rospy.Subscriber('keys', String, self.cb_forward)
        self.sub_forward = rospy.Subscriber('keys', String, self.cb_backward)

        rospy.loginfo('Tello driver node ready')
        
    def cb_takeoff(self, msg):
        if g_last_twist.linear.x == 1:
            print("takeoff")
            success = self.takeoff()
            print(success)
            notify_cmd_success('Takeoff', success)
       
            
    def cb_land(self, msg):
        if g_last_twist.linear.x == -1:
            print("land")
            success = self.land()
            notify_cmd_success('Land', success)
    
    def cb_forward(self,val):
        if g_last_twist.angular.z == 1:
            val=10
            success = True
            self.forward(val)
            notify_cmd_success('forward', success)
    
    def cb_backward(self,val):
        if g_last_twist.angular.z == -1:
            val=10
            success = True
            self.backward(val)
            notify_cmd_success('backward', success)


    def keys_cb(self,msg,twist_pub1):
        if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
            return # unknown key.
        vels = key_mapping[msg.data[0]]
        g_last_twist.angular.z = vels[0]
        g_last_twist.linear.x  = vels[1]
        twist_pub1.publish(g_last_twist)
        print(g_last_twist)



def main():
    rospy.init_node('tello_control_node')
    robot = TelloControlNode()
    #rate = rospy.Rate(10)
    g_last_twist = Twist() # initializes to zero
    # while not rospy.is_shutdown():
    #     rospy.publish(g_last_twist)
    #     rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
