#!/usr/bin/env python3
#import sys, os

#import yaml
import rospy
import sys
#import math
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf import transformations as tft

class PublishStaticTransforms(object):
    def __init__(self):
        rospy.init_node("publish_static_transforms")
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.broadcaster.sendTransform(self.transform())
            #self.broadcaster.sendTransform(self.transform_static_robot_head())
            #self.broadcaster.sendTransform(self.transform_uma8_array())
            rate.sleep()

    def transform(self):
        st = TransformStamped()
        st.header.stamp = rospy.Time.now()
        st.header.frame_id = 'base_link'
        st.child_frame_id = 'kinect_camera_base'

        st.transform.translation.x =  -0.25 #-0.092075
        st.transform.translation.y = 0 #0.145821
        # this value has been manually tweaked
        st.transform.translation.z = 0.25

        # 10 degrees -> 0.17 rad
        q = tft.quaternion_from_euler(0, 0, 0)
        st.transform.rotation.x = q[0]
        st.transform.rotation.y = q[1]
        st.transform.rotation.z = q[2]
        st.transform.rotation.w = q[3]
        return st
    
    def transform_static_robot_head(self):
        st = TransformStamped()
        st.header.stamp = rospy.Time.now()
        st.header.frame_id = 'base_link'
        st.child_frame_id = 'head_link'

        st.transform.translation.x =  0 #-0.092075
        st.transform.translation.y = 0 #0.145821
        # this value has been manually tweaked
        st.transform.translation.z = 0.16

        # 10 degrees -> 0.17 rad
        q = tft.quaternion_from_euler(0, 0, 0)
        st.transform.rotation.x = q[0]
        st.transform.rotation.y = q[1]
        st.transform.rotation.z = q[2]
        st.transform.rotation.w = q[3]
        return st

   
if __name__ == "__main__":
    try:
        node = PublishStaticTransforms()
    except rospy.ROSInterruptException:
        pass

