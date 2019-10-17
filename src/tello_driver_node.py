#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty, UInt8, Bool,String
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from h264_image_transport.msg import H264Packet
from tello_driver.cfg import TelloConfig
from cv_bridge import CvBridge, CvBridgeError

import av
import math
import cv2
import numpy as np
import threading
import time
import sys
sys.path.append('/home/tello/catkin_ws/src/TelloPy/')#/home/#your_name/catkin_ws/src/TelloPy
from tellopy._internal import tello
from tellopy._internal import error
from tellopy._internal import protocol
from tellopy._internal import logger


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

key_mapping = { 'w': [ 0, 1,0], 'x': [0, -1,0], 
                'a': [1, 0,0], 'd': [-1,  0,0], 
                'z':[0,0,1], 'c':[0,0,-1],
                's': [ 0, 0,0] }


global g_last_twist 
g_last_twist = Twist()


class TelloNode(tello.Tello):
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
        super(TelloNode,self).__init__(
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
        rospy.on_shutdown(self.cb_shutdown)

        # Setup dynamic reconfigure
        self.cfg = None
        self.srv_dyncfg = Server(TelloConfig, self.cb_dyncfg)

        # Setup topics and services
        if self.stream_h264_video:
            self.pub_image_h264 = rospy.Publisher(
                'image_raw/h264', H264Packet, queue_size=10)
        else:
            self.pub_image_raw = rospy.Publisher(
                'image_raw', Image, queue_size=10)
        
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist_sub = rospy.Subscriber('/keys', String, self.keys_cb, self.twist_pub)
        self.sub_takeoff = rospy.Subscriber('/keys', String, self.cb_takeoff)
        self.sub_land = rospy.Subscriber('/keys', String, self.cb_land)
        self.sub_forward = rospy.Subscriber('/keys', String, self.cb_forward)
        self.sub_forward = rospy.Subscriber('/keys', String, self.cb_backward)
        self.sub_left = rospy.Subscriber('/keys', String, self.cb_left)
        self.sub_right = rospy.Subscriber('/keys', String, self.cb_right)


        if self.stream_h264_video:
            self.start_video()
            self.subscribe(self.EVENT_VIDEO_FRAME, self.cb_h264_frame)
        else:
            # self.frame_thread = threading.Thread(target=self.framegrabber_loop)
            # self.frame_thread.start()
            self.framegrabber_loop()

        rospy.loginfo('Tello driver node ready')

    def cb_shutdown(self):
        self.quit()
        if self.frame_thread is not None:
            self.frame_thread.join()

    def cb_h264_frame(self, event, sender, data, **args):
        frame, seq_id, frame_secs = data
        pkt_msg = H264Packet()
        pkt_msg.header.seq = seq_id
        pkt_msg.header.frame_id = rospy.get_namespace()
        pkt_msg.header.stamp = rospy.Time.from_sec(frame_secs)
        pkt_msg.data = frame
        self.pub_image_h264.publish(pkt_msg)

    def framegrabber_loop(self):
        # Repeatedly try to connect
        vs = self.get_video_stream()
        while self.state != self.STATE_QUIT:
            try:
                container = av.open(vs)
                break
            except BaseException as err:
                rospy.logerr('fgrab: pyav stream failed - %s' % str(err))
                time.sleep(1.0)
        
        # Once connected, process frames till drone/stream closes
        frame_skip = 300
        while self.state != self.STATE_QUIT:
            try:
                # vs blocks, dies on self.stop
                for frame in container.decode(video=0):
                    if 0 < frame_skip:
                        frame_skip = frame_skip - 1
                        continue
                    start_time = time.time()
                    img = np.array(frame.to_image())
                    image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                    #cv2.imshow('Original', image)
                    #cv2.imshow('Canny', cv2.Canny(image, 100, 200))
                    #cv2.waitKey(1)
                    
                    if frame.time_base < 1.0/60:
                        time_base = 1.0/60
                    else:
                        time_base = frame.time_base
                    frame_skip = int((time.time() - start_time)/time_base)
                    try:
                        img_msg = self.bridge.cv2_to_imgmsg(img, 'rgb8')
                        img_msg.header.frame_id = rospy.get_namespace()
                    except CvBridgeError as err:
                        rospy.logerr('fgrab: cv bridge failed - %s' % str(err))
                        continue
                    self.pub_image_raw.publish(img_msg)

                break
            except BaseException as err:
                rospy.logerr('fgrab: pyav decoder failed - %s' % str(err))
                





    def cb_dyncfg(self, config, level):
        update_all = False
        req_sps_pps = False
        if self.cfg is None:
            self.cfg = config
            update_all = True

        if update_all or self.cfg.fixed_video_rate != config.fixed_video_rate:
            self.set_video_encoder_rate(config.fixed_video_rate)
            req_sps_pps = True
        if update_all or self.cfg.video_req_sps_hz != config.video_req_sps_hz:
            self.set_video_req_sps_hz(config.video_req_sps_hz)
            req_sps_pps = True

        if req_sps_pps:
            self.send_req_video_sps_pps()

        self.cfg = config
        return self.cfg

    def cb_takeoff(self, msg):
        if g_last_twist.linear.z  == 1:
            print("takeoff")
            success = self.takeoff()
            print(success)
            notify_cmd_success('Takeoff', success)
       
            
    def cb_land(self, msg):
        if g_last_twist.linear.z  == -1:
            print("land")
            success = self.land()
            notify_cmd_success('Land', success)
    
    def cb_forward(self,val):
        if g_last_twist.linear.x == 1:
            val=10
            success = True
            self.forward(val)
            notify_cmd_success('forward', success)
    
    def cb_backward(self,val):
        if g_last_twist.linear.x == -1:
            val=10
            success = True
            self.backward(val)
            notify_cmd_success('backward', success)

    def cb_left(self,val):
        if g_last_twist.angular.z == 1:
            val=10
            success = True
            self.left(val)
            notify_cmd_success('left', success)

    def cb_right(self,val):
        if g_last_twist.angular.z == -1:
            val=10
            success = True
            self.right(val)
            notify_cmd_success('right', success)
    
    def keys_cb(self,msg,twist_pub1):
        if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
            return # unknown key.
        vels = key_mapping[msg.data[0]]
        g_last_twist.linear.z = vels[1] #takeoff&land
        g_last_twist.linear.x  = vels[0]#forward&backward
        g_last_twist.angular.z = vels[2]#left&right
        twist_pub1.publish(g_last_twist)
        print(g_last_twist)


def main():
    rospy.init_node('tello_node')
    robot = TelloNode()
    g_last_twist = Twist() # initializes to zero
    rospy.spin()

if __name__ == '__main__':
    main()


