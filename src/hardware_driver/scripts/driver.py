#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Vector3

def talker():
    pub = rospy.Publisher('pwm_sig', Vector3, queue_size=10)
    pub1 = rospy.Publisher('mode_sig', Int16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        temp_dir_mode = input("please enter the direction mode you want 0: strait mode, 1: turning mode:")
        temp_pwm_r = input("Enter right pwm here:")
        temp_pwm_l = input("Enter left pwm here:")
        temp_heading = input("Enter the heading angle here:")

        dir_comand = int(temp_dir_mode)

        pwm_comand = Vector3()
        pwm_comand.x = int(temp_pwm_r)
        pwm_comand.y = int(temp_pwm_l)
        pwm_comand.z = int(temp_heading)
        rospy.loginfo(pwm_comand)
        pub.publish(pwm_comand)
        rospy.loginfo(dir_comand)
        pub1.publish(dir_comand)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
