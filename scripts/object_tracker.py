#!/usr/bin/env python3
#
# Publishes joint commands for face tracking, when enabled
#
# Created 02/20/2020 by Nathan Tsoi for Shutter robot
# Adjusted 04/26/2023 by Sarah Gillet to fit Blossom

import numpy as np
import rospy
import std_msgs
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PointStamped
#from std_msgs.msg import Float64, Int16
#from sensor_msgs.msg import JointState
from blossom_ros.msg import Motor, MotorArray
from scipy.interpolate import interp1d

#import copy
import math
import tf2_ros
import tf2_geometry_msgs
#from tf import transformations as tft

# import schedule, time, threading
import time
# import random
import sys
import os

DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(DIR, '..', 'src'))

import body_markers
import trajectory_consts
import kinematics as k
import numpy as np

class KeepTrackGaze():

    def __init__(self):
        self.currently_gazed_at = False
        self.time_attended = 0
        self.started_gazing = -1
        self.time_to_look = np.random.uniform(2,8)
    
    def stopped_gazing(self):
        self.currently_gazed_at = False
        self.time_attended += time.time() - self.started_gazing
    
    def start_gazing(self):
        self.currently_gazed_at = True
        self.started_gazing = time.time()
        self.time_to_look = np.random.uniform(2,8)
    
    def get_time_attended(self):
        time_attended = self.time_attended
        if self.currently_gazed_at:
                time_attended += time.time() - self.started_gazing
        return time_attended

class ExternalPoseTracker:
    def __init__(self):
        '''body_id is used to specify the marker to follow, defaults to the first person if not specified'''
        rospy.init_node("face_tracker")

        self.angles = {"base": 0, "head": 0}
        self.body_id = None
        self.mode = 'follow'
        #self.velocities = TR_DEFAULT_VEL

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    
        # listen for kinect tracking data
        rospy.Subscriber("/kinect/body_tracking_data", MarkerArray, self.callback_body_tracking, queue_size=2)
        # listen to externally set pose of object
        rospy.Subscriber("/object_pose_tracker", PointStamped, self.callback_object_tracking, queue_size=2)
        # listen for trajectory mode, latched!
        rospy.Subscriber("/mode", std_msgs.msg.String, self.callback_trajectory_mode)
        #rospy.Subscriber("/joint_states", JointState, self.callback_joints)

        self.body_id_tracking = {}
        # TODO put message
        #self.pub_to_pose_controller = rospy.Publisher("/pose/two_angles", queue_size=1)
        self.motor_pub = rospy.Publisher('blossom/motors/goal', MotorArray, queue_size=1)
        
        self.gaze_pt = PointStamped()
        self.gaze_pt.header = None
        rospy.spin()

    def callback_trajectory_mode(self, mode):
        self.mode = mode.data
        # self.mode = 'follow'
        # reset the currently tracked person
        if self.mode not in ['follow_person', 'look_between', 'look_at_object']:
            self.body_id = None

   
    # the following function was kindly provided by chatgpt
    def rotate_point(self, x, y, angle):
        # Rotate a 2-dimensional point around the origin by a given angle in radians.
        sin_theta = math.sin(angle)
        cos_theta = math.cos(angle)
        rotated_x = x * cos_theta - y * sin_theta
        rotated_y = x * sin_theta + y * cos_theta
        return rotated_x, rotated_y


    def next_to_look_at(self,closest_id, body_ids):
        list_attended = []
        for key in self.body_id_tracking.keys():
            time_attended = self.body_id_tracking[key].get_time_attended()
            list_attended.append([key, time_attended])
        for body_id in body_ids:
            if body_id not in self.body_id_tracking.keys():
                list_attended.append([body_id, 0])
        list_attended.sort(key=lambda x:x[1])
        #print(list_attended)
        return list_attended[0][0]
        
    def callback_force_gaze(self, msg):
        self.body_id = msg.data

    def callback_object_tracking(self, msg):
        # set object point from msg
        # initate motion track position
        self.gaze_pt.header = msg.header
        self.gaze_pt.header.stamp = rospy.Time.now()

        self.gaze_pt.point.x = msg.point.x
        self.gaze_pt.point.y = msg.point.y
        self.gaze_pt.point.z = msg.point.z
        self.motion_track_position(self.gaze_pt)

    def callback_body_tracking(self, msg):
        # only run in follow mode
        if self.mode not in ['follow_person', 'look_between', ]:
            return

        # sort markers by person
        markers_by = {}
        for marker in msg.markers:
            body_id = int(marker.id/100)
            marker_id = int(str(marker.id)[-2:])
            if body_id not in markers_by:
                markers_by[body_id] = {}
            markers_by[body_id][marker_id] = marker

        body_ids = list(markers_by.keys())
        if len(body_ids) <= 0:
            # TODO: some action when there is no one to follow
            return

        # keep tracking the current person
        body_id = self.body_id
        # or find the closest person
        heads = {}
        closest = 0
        closest_id = None
        for b_id, m in markers_by.items():
            heads[b_id] = m[body_markers.BodyMakers.K4ABT_JOINT_HEAD.value]
            p = heads[b_id].pose.position
            l2 = np.linalg.norm([p.x, p.y, p.z])
            if closest_id is None or l2 < closest:
                closest = l2
                closest_id = b_id
        if body_id is None and closest_id is None:
            return

        if body_id is None or body_id not in body_ids:
            # if person we are currently tracking is not available anymore
            # look at new person and start keeping track of gaze action
            self.body_id = closest_id
            if self.mode == 'look_between' and not self.body_id in self.body_id_tracking:
                self.body_id_tracking[self.body_id] = KeepTrackGaze()
            if self.body_id in self.body_id_tracking:
                self.body_id_tracking[self.body_id].start_gazing()
        elif self.mode == 'look_between':
            # if person is still there, decide if we need to look at someone else 
            # (if there is someone else)
            if self.body_id is not None:
                if self.body_id in self.body_id_tracking and (time.time() - self.body_id_tracking[self.body_id].started_gazing) > self.body_id_tracking[self.body_id].time_to_look and \
                    len(body_ids) > 1:
                    body_id_next = self.next_to_look_at(closest_id, body_ids)
                    rospy.loginfo("Next body I am looking: ", body_id_next, "From those: ", body_ids)
                    # if gazed at person changes, keep track of attention
                    if body_id_next != self.body_id:
                        self.body_id_tracking[self.body_id].stopped_gazing()
                        self.body_id = body_id_next
                        if not self.body_id in self.body_id_tracking:
                            self.body_id_tracking[self.body_id] = KeepTrackGaze()
                        self.body_id_tracking[self.body_id].start_gazing()

        #rospy.logerr(self.body_id)
        # set eye gaze
        head_marker = heads[self.body_id]
        # self.gaze_pt = PointStamped()
        self.gaze_pt.header = head_marker.header
        self.gaze_pt.header.stamp = rospy.Time.now()

        self.gaze_pt.point.x = head_marker.pose.position.x
        self.gaze_pt.point.y = head_marker.pose.position.y
        self.gaze_pt.point.z = head_marker.pose.position.z
        
        # track the face (motion trajectory)
        self.motion_track_position(self.gaze_pt)

        if self.gaze_phase == 2: #aversion
            self.gaze_pt.point.x += self.offset
            self.gaze_pt.point.y += abs(self.offset)
            # self.gaze_pt.point.z += self.offset

        self.gaze_pt.header.stamp = rospy.Time.now()
        self.gaze_pub.publish(self.gaze_pt)

        # re-publish tracked point
        head_marker.type = 1
        head_marker.color.r = 1.0
        head_marker.color.b = 0
        head_marker.color.g = 0
        self.gaze_marker_pub.publish(head_marker)


    def motion_track_position(self, point_msg):
        ''' track a position in space
            TODO: some smoothing (kalman filter?)'''

        # transform target point into the base_link frame
        target_frame = "base_link"
        theta_z = 0
        try:
            trans = self.tfBuffer.lookup_transform(target_frame, point_msg.header.frame_id, rospy.Time.now(), rospy.Duration(0.1))
            # in the base_link frame
            pt = tf2_geometry_msgs.do_transform_point(point_msg, trans)
            # check if x-distance is negative, if so, give up
            # if pt.point.x <= 0:
            #     rospy.logwarn("x value should not be negative")
            # else:
            pt.header.frame_id = target_frame
            # compute the angle needed to move 
            theta_z = math.atan(pt.point.y/pt.point.x)
            if pt.point.x < 0:
                theta_z = math.pi - theta_z
            print(theta_z)
            # z-angle offset clipped to [-pi/2, pi/2]
            self.angles['base'] = max(-math.pi/2, min(theta_z, math.pi/2))
              
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Failed to transform point in {} frame to {}:\n{}".format(point_msg.header.frame_id, target_frame, e))
            return

        # y-angle offset, relative to the last joint
        target_frame = "head_link"
        th_y = 0
        try:
            # This transform has an intermittent error when using Time.now()
            # It seems to have something to do with the realsense driver
            # Don't just switch it to rospy.Time() though, b/c this depends on a moving joint and can cause oscillations
            # in the head if the transform timestamp is off
            # The error is:
            #    Failed to transform point in depth_camera_link frame to camera_link: Lookup would require extrapolation into the future.
            htrans = self.tfBuffer.lookup_transform(target_frame, point_msg.header.frame_id, rospy.Time.now(), rospy.Duration(0.1))
            # qoffset range clipped to 0 (can't look down) to .5 rad
            hpt = tf2_geometry_msgs.do_transform_point(point_msg, htrans)
            #print('transformed', hpt.point)
            x, y = self.rotate_point(hpt.point.x, hpt.point.y, -theta_z)
            #print(self.rotate_point(hpt.point.x, hpt.point.y, -theta_z))
            #y = hpt.point.y
            z = hpt.point.z
            print(x,y,z, point_msg.point)
            th_y = math.atan(z/x)
            #print(th_y, math.pi - th_y if abs(math.pi - th_y) < math.pi/2 else th_y-math.pi)
            #if x < 0:
            #    th_y = math.pi - th_y if abs(math.pi - th_y) < math.pi/2 else th_y-math.pi
            #print("x: {}, y: {}, z: {}, y': {}, z': {}, th: {}".format(hpt.point.y, hpt.point.y, hpt.point.z, y, z, theta_x))
            #clipped_theta_y = max(-math.pi/3, min(math.pi/3, th_y))
            self.angles['head'] = th_y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to transform point in {} frame to {}:\n{}".format(point_msg.header.frame_id, target_frame, e))

        #z = hpt.point.z
        motor_names = ["tower_1", "tower_2", "tower_3", "base"]
        clip_at = [0, 0.5]
        interpol_at = [0, 0.5]
        z_clipped = max(clip_at[0], min(pt.point.z, clip_at[1]))
        m1 = interp1d(interpol_at, [30, 0])
        z = m1(z_clipped)
        motor_pos = k.get_motor_pos([theta_z, th_y, 0, z])
        print(theta_z, th_y, z)
        #motor_pos = k.get_motor_pos([0, 0, 0, 20])
        m_array = MotorArray()
        i = 0
        for m in motor_names:
            mv = Motor()
            mv.name = m
            mv.position = motor_pos[i]
            i += 1
            mv.time_ms = 1
            m_array.motors.append(mv)

        # Publish the array
        #print("Don't publish right now for safety, goal motor pose: ", str(m_array))
        self.motor_pub.publish(m_array)


def main():
    try:
        ExternalPoseTracker()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
