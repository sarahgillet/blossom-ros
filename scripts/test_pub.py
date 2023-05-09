#!/usr/bin/env python3
import rospy
from blossom_ros.msg import Motor, MotorArray
from geometry_msgs.msg import PointStamped

def talker():
    pub = rospy.Publisher('blossom/motors/goal', MotorArray, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.2) # 1hz

    motor_names = ["tower_1", "tower_2", "tower_3", "base", "ears"]
    seq = [
        [50, 50, 50, 50, 50],
        [50, 50, 100, 100, 50],
        [50, 50, 150, 150, 50],
        [50, 50, 100, 100, 50],
        [50, 50, 50, 50, 50]
    ]
    seq_count = 0

    while not rospy.is_shutdown():

        # This is a fairly stupid way of putting the sequence and motor name together, do it better:
        i = 0
        m_array = MotorArray()
        for m in motor_names:
            mv = Motor()
            mv.name = m
            mv.position = float(seq[seq_count][i])
            mv.time_ms = 0
            i += 1
            m_array.motors.append(mv)

        # Publish the array
        pub.publish(m_array)

        # Increment (and loop) the sequence counter
        seq_count += 1
        if(seq_count >= len(seq)):
            seq_count = 0

        rate.sleep()

def talker_position_stamped():
    pub = rospy.Publisher('/object_pose_tracker', PointStamped, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    seq = [[0.05, -0.5, 0.00], [0.7, 0, 0.5], [0.07, 0.3, 0.25], [-2, -0.2, 0.5], [0.7, 0, 0.5]] #,-0.05, -0.1, -0.15, -0.1, -0.05, 0]
    #seq = [[-0.17, 0.05, 0.36]] #,-0.05, -0.1, -0.15, -0.1, -0.05, 0] [0.1, -0.8, 0.00]] #,
    seq_count = 0
    seq_count = 0
    while not rospy.is_shutdown():

        msg = PointStamped()
        msg.header.frame_id = "base_link"
        msg.point.x = seq[int(seq_count/5)][0]
        msg.point.y = seq[int(seq_count/5)][1]
        msg.point.z = seq[int(seq_count/5)][2]
        # remove base height
        msg.point.z -= 0.065
        # account for plate to "face" dist
        msg.point.z -= 0.06
        print(seq[int(seq_count/5)])
        # Publish the array
        pub.publish(msg)

        # Increment (and loop) the sequence counter
        seq_count += 1
        if(seq_count >= len(seq*5)):
            seq_count = 0

        rate.sleep()

if __name__ == '__main__':
    try:
        talker_position_stamped()
    except rospy.ROSInterruptException:
        pass

