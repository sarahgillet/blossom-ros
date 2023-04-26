#!/usr/bin/env python
import rospy
from blossom_ros.msg import Motor, MotorArray

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

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

