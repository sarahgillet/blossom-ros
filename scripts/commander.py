#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
from blossom_ros.msg import Motor, MotorArray
from motor_driver import Robot 
import json

class Commander():

    def __init__(self, _config_file, _name):

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
        rospy.init_node('motor_driver', anonymous=True)

        self.rate = rospy.Rate(10) # 1hz

    def main_loop(self):

        while not rospy.is_shutdown():

            self.get_motor_positions()
            self.rate.sleep()

    def get_motors(self):
        m_list = []
        for m in self.bot_config_file["motors"]:
            m_list.append(m)

        self.motor_list = m_list
        return m_list

    def get_limits(self, _name):
        return self.bot_config_file["motors"][_name]["angle_limit"]

    def get_offset(self, _name):
        return self.bot_config_file["motors"][_name]["offset"]    

    def generate_config_dict(self):

        m_list = self.get_motors()
        m_dict = {}
        for m in m_list:
            m_dict[m] = {   "offset": self.get_offset(m),
                            "angle_limit": self.get_limits(m) }
            
        return m_dict

    def set_motors(self, move_dict):
       
        for m in move_dict:
            val = move_dict[m]
            
            # Check limits:
            if val < self.config[m]["angle_limit"][0]:
                move_dict[m] = self.config[m]["angle_limit"][0]
                rospy.loginfo(rospy.get_caller_id() + "Motor ", m, " under limit")
            elif val > self.config[m]["angle_limit"][1]:
                move_dict[m] = self.config[m]["angle_limit"][1]
                rospy.loginfo(rospy.get_caller_id() + "Motor ", m, " over limit")

            # Check offset:
            move_dict[m] = move_dict[m] + self.config[m]["offset"]
        
        self.bot.goto_position(move_dict, 0, False)

    def motor_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.motors)

        md = {}
        motors = data.motors
        for m in motors:
            md[m.name] = m.position

        self.set_motors(md)

    def get_motor_positions(self):

        motor_poses = self.bot.get_motor_pos()
        motor_array = MotorArray()
        for m, val in motor_poses.items():
            mv = Motor()
            mv.name = m
            mv.position = float(val)
            motor_array.motors.append(mv)

        self.motor_position_publisher.publish(motor_array)
        

if __name__ == "__main__":

    c = Commander("config.json", "woody")
    c.main_loop()


