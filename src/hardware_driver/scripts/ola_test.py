#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import String


def talker():
    #pub = rospy.Publisher('pwm_sig', Vector3, queue_size=10)
    pub1 = rospy.Publisher('data_to_be_sent', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        dir_comand = input("please enter the direction mode you want 0: strait mode, 1: turning mode:")
        if dir_comand == "kill" or dir_comand == "quit" or dir_comand == "end" or dir_comand == "stop" or dir_comand == "q":
            sys.exit("kill compleat")
        #temp_pwm_r = input("Enter right pwm here:")
        #temp_pwm_l = input("Enter left pwm here:")
        #temp_heading = input("Enter the heading angle here:")


        rospy.loginfo(dir_comand)
        pub1.publish(dir_comand)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
