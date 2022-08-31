#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class depth_clip:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.clip_meter = 20
        self.depth_pub = rospy.Publisher("/depth_clip_20", Image, queue_size=10)
        self.depth_sub = rospy.Subscriber("/airsim_node/drone_1/camera_1/DepthPlanar", Image, self.callback)
        
    
    def callback(self, data):
        depth_input = self.bridge.imgmsg_to_cv2(data, desired_encoding=data.encoding)
        depth_input = np.clip(depth_input, 0, self.clip_meter)
        
        depth_pub_msg = self.bridge.cv2_to_imgmsg(depth_input, encoding="passthrough")
        depth_pub_msg.header = data.header
        
        self.depth_pub.publish(depth_pub_msg)
        
        # cv2.imshow('depth', depth_input/self.clip_meter)
        # cv2.waitKey(1)
        # print(type(depth_input), depth_input.min(), depth_input.max(), depth_input.shape, depth_input.dtype)
        # print(depth_input)
        

def main():
    rospy.init_node('depth_clip', anonymous=True)
    dc = depth_clip()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    

if __name__ == '__main__':
    main()
    