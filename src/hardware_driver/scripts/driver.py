#!/usr/bin/env python3
import rospy
import sys
import os
import time
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

path = "/home/nicoleg/test_data/wheel_speed%s.txt"
number_of_files = 0
run = 1



def file_iterator():
    global number_of_files
    number_of_files = 0
    while os.path.exists(path % number_of_files):
        print("im running itorator")
        number_of_files = number_of_files + 1

def speed_writting(data):
    global path, number_of_files
    f = open(path % number_of_files,"a")
    f.write(str(int(data.z))+","+str(data.x)+","+str(data.y)+"\n")
    f.close()

def main():
    global path, number_of_files, run
    print("so driver.py is starting now")
    time.sleep(3)
    pub = rospy.Publisher('stat_up_done', Int16, queue_size=10)
    pub_string = rospy.Publisher('hi', String, queue_size=10)
    pub1 = rospy.Publisher('mode_sig', Int16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        temp_dir_mode = input("just give me a string")
        if temp_dir_mode == "kill" or temp_dir_mode == "quit" or temp_dir_mode == "end" or temp_dir_mode == "stop" or temp_dir_mode == "q":
            sys.exit("kill compleat")


        string_to_be_sent = String()
        string_to_be_sent.data = temp_dir_mode
        #temp_pwm_r = input("Enter right pwm here:")
        #temp_pwm_l = input("Enter left pwm here:")
        #temp_heading = input("Enter the heading angle here:")

        #dir_comand = int(temp_dir_mode)

        #pwm_comand = Int16()
        #pwm_comand.data = number_of_files
        #pwm_comand.y = int(temp_pwm_l)
        #pwm_comand.z = int(temp_heading)
        #rospy.Subscriber("collection", Vector3, speed_writting)
        #rospy.Subscriber("end_of_run", Int16, file_iterator)
        #rospy.loginfo(pwm_comand)

        #rospy.loginfo(dir_comand)
        #pub1.publish(dir_comand)
    #start_up_cmd = Int16()
    #start_up_cmd.data = 1
        pub_string.publish(string_to_be_sent)
    #rate.sleep()
    #print("so driver.py is ending now")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
