#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
from blossom_ros.msg import Motor, MotorArray
from motor_driver import Robot 
import json
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf import transformations as tft

import sys
import os

DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(DIR, '..', 'src'))


import kinematics as k

class Commander():

    def __init__(self, _name):

        # Create the robot based on the config file (handled motor_driver.py)
        rospy.init_node('motor_driver', anonymous=True)
        _config_file = rospy.get_param('~path_to_config', '../config/config.json')
        self.bot = Robot(_config_file, name=_name)
        self.bot_config_file = {}
        self.config = {}

        # This seems silly to load a second time, but here we can get the config details for the motors
        with open(_config_file) as f:
            self.bot_config_file = json.load(f)
        self.config = self.generate_config_dict()

        # ROS
        rospy.Subscriber("blossom/motors/goal", MotorArray, self.motor_callback)
        self.motor_position_publisher = rospy.Publisher("blossom/motors/current", MotorArray, queue_size=1)
        

        # publish current height of robot
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.rate = rospy.Rate(10) # 10hz

    # Keep the commander alive
    def main_loop(self):

        while not rospy.is_shutdown():
            motor_positions = self.get_motor_positions()
            self.broadcaster.sendTransform(self.calculate_forward(motor_positions))
            self.rate.sleep()


    def calculate_forward(self, motor_positions):
        val = motor_positions.copy()
        for m in val:
            # Add offset to get clean value of motors
            val[m] = val[m] - self.config[m]["offset"]
        # update kinematics
        z = k.calculate_height(list(val.values()))

        # create transform to be send to the tf tree
        st = TransformStamped()
        st.header.stamp = rospy.Time.now()
        st.header.frame_id = 'base_link'
        st.child_frame_id = 'head_link'

        st.transform.translation.x =  0
        st.transform.translation.y = 0
        st.transform.translation.z = z/100.0 # cm to m

        q = tft.quaternion_from_euler(0, 0, 0)
        st.transform.rotation.x = q[0]
        st.transform.rotation.y = q[1]
        st.transform.rotation.z = q[2]
        st.transform.rotation.w = q[3]
        return st

    # Get the list of motors (from the config)
    def get_motors(self):
        m_list = []
        for m in self.bot_config_file["motors"]:
            m_list.append(m)

        self.motor_list = m_list
        return m_list

    # Get the motor limits (from the config)
    def get_limits(self, _name):
        return self.bot_config_file["motors"][_name]["angle_limit"]

    # Get the motor offset (from the config)
    def get_offset(self, _name):
        return self.bot_config_file["motors"][_name]["offset"]    

    # Generate a dict based on the config
    def generate_config_dict(self):

        m_list = self.get_motors()
        m_dict = {}
        for m in m_list:
            m_dict[m] = {   "offset": self.get_offset(m),
                            "angle_limit": self.get_limits(m) }
            
        return m_dict

    # Set the motor based on a dict
    def set_motors(self, move_dict):
        fw_kin_dict = move_dict.copy()
        # Loop through the motors and set them to the goal position, check the limits
        for m in move_dict:
            val = move_dict[m]
            # Check limits:
            if val < self.config[m]["angle_limit"][0]:
                move_dict[m] = self.config[m]["angle_limit"][0]
                rospy.loginfo(str(rospy.get_caller_id()) + "Motor "+str(m)+ " under limit")
            elif val > self.config[m]["angle_limit"][1]:
                move_dict[m] = self.config[m]["angle_limit"][1]
                rospy.loginfo(rospy.get_caller_id() + "Motor "+ str(m)+ " over limit")
            fw_kin_dict[m] = move_dict[m]
            # Check offset:
            move_dict[m] = move_dict[m] + self.config[m]["offset"]
        # Send the command to the motors, no delay, don't wait
        self.bot.goto_position(move_dict, 1, False)
        # update kinematics
        print(k.calculate_height(list(fw_kin_dict.values())))
        

    # Called on Msg, create a dict and send it to the motors:
    def motor_callback(self, data): 
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.motors)

        md = {}
        motors = data.motors
        for m in motors:
            md[m.name] = m.position

        self.set_motors(md)

    # Get the position of the motors currently and publish to a topic
    def get_motor_positions(self):

        motor_poses = self.bot.get_motor_pos()
        motor_array = MotorArray()
        for m, val in motor_poses.items():
            mv = Motor()
            mv.name = m
            mv.position = float(val)
            motor_array.motors.append(mv)

        self.motor_position_publisher.publish(motor_array)
        return motor_poses
        

if __name__ == "__main__":
    c = Commander("cherry")
    c.main_loop()


