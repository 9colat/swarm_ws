#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def talker():
    pub1 = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('driving_control', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        vel_comand = input("please enter the direction mode you want 0: strait mode, 1: turning mode:")
        omega_comand = input("please enter the direction mode you want 0: strait mode, 1: turning mode:")
        if vel_comand == "kill" or vel_comand == "quit" or vel_comand == "end" or vel_comand == "stop" or vel_comand == "q" or omega_comand == "kill" or omega_comand == "quit" or omega_comand == "end" or omega_comand == "stop" or omega_comand == "q":
            sys.exit("kill compleat")

        dir_comand = Twist()
        dir_comand.linear.x = float(vel_comand)
        dir_comand.angular.z = float(omega_comand)
        dir_comand.angular.x = 90

        pub1.publish(dir_comand)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
