#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16

def talker():
    pub = rospy.Publisher('pwm_sig', Int16, queue_size=10)
    pub1 = rospy.Publisher('dir_sig', Int16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        temp_dir_mode = input("please enter the direction mode you want 0: strait mode, 1: turning mode")
        temp_pwm = input("Enter pwm here:")
        print(type(temp_pwm))
        #if type(temp_var) == int:
            #pwm_comand = temp_var
        #if type(temp_var) == !int:
            #pwm_comand = int(temp_var)
        dir_comand = int(temp_dir_mode)
        pwm_comand = int(temp_pwm)
        rospy.loginfo(dir_comand)
        pub1.publish(dir_comand)
        rospy.loginfo(pwm_comand)
        pub.publish(pwm_comand)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
