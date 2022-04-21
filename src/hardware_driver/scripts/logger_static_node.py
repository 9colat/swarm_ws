#!/usr/bin/env python3
import rospy
import sys
import os
import csv
from pathlib import Path
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from custom_msgs.msg import odom_and_imu
from custom_msgs.msg import USPS_msgs



##### this is a subscriber example that can be copy pasta later #####

## paste this as a global variable, outside of the callback function (after "number_of_files = 0")
#data_to_be_logged = [0] * 3 # remember to specify this correctly, depending on what you are trying to log

## now we specify the callback function for the specific data subscription (replace "something" with what you want)
#def callback_something(data):

#    global data_to_be_logged
#    data_to_be_logged[0] = data.x
#    data_to_be_logged[1] = data.y
#    data_to_be_logged[2] = data.z

## past the thing  below in the main loop within "while not rospy.is_shutdown():"
#    rospy.Subscriber("topic_name", Data_Type, callback_something) #(topic name, data type, callback function)

##### now you should have a working subscriber! #####



input_speed = 0
angular_speed = 0

path = Path.home().joinpath("test_data", "static_log%s.csv")
folder_path = str(Path.home().joinpath("test_data"))
isfolder = os.path.isdir(folder_path)
number_of_files = 0
lidar_array = [0] * 360
beacon_data = [0] * 2
IMU_data = [0]*6
without_lidar_data = [0] * 5
with_lidar_data = [0] * 5
lidar_label = "lidar","lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar"

fieldnames = ["with_x","with_y","with_v","with_hx","with_hy","without_x","without_y","without_v","without_hx","without_hy","acc_x","acc_y","acc_z","gyro_x","gyro_y","gyro_z","ID","distance","linear_speed","angular_speed"]
def file_iterator():
    global number_of_files
    number_of_files = 0
    #print("im finding out how many logs there are")
    while os.path.exists(str(path) % number_of_files):
        #print("im running itorator")
        number_of_files = number_of_files + 1

#def callback(data):
#    print(data.data)

def callback_distance(data):
    global beacon_data
    beacon_data = [data.ID, data.distance]

def callback_imu(data):
    global IMU_data
    IMU_data = [data.imu_acc.x, data.imu_acc.y, data.imu_acc.z, data.imu_gyro.x, data.imu_gyro.y, data.imu_gyro.z]

def callback_pose_estimator_with_lidar(data):
    global with_lidar_data
    with_lidar_data = [data.position.x,data.position.y,data.position.v,data.orientation.x,data.orientation.y]

def callback_pose_estimator_without_lidar(data):
    global without_lidar_data
    without_lidar_data = [data.position.x,data.position.y,data.position.v,data.orientation.x,data.orientation.y]

def callback_lidar(data):#####
    global lidar_array
    #print("laser data recived")
    for i in range(len(data.ranges)):
        lidar_array[i] = data.ranges[i]



def callback_input_speed(data):
    global input_speed, angular_speed
    input_speed = data.linear.x
    angular_speed = data.angular.z


def main():
    global path, folder_path, isfolder, number_of_files, seperator,input_speed, angular_speed, lidar_array,without_lidar_data, with_lidar_data, IMU_data, beacon_data, fieldnames
    if not isfolder:
        os.mkdir(folder_path)
        print("making directory")
    file_iterator()
    path = str(path) % number_of_files
    rospy.init_node('logger', anonymous=True)

    with open(path,'w') as csv_file:
        csv_writer = csv.DictWriter(csv_file,fieldnames=fieldnames)
        csv_writer.writeheader()
    #rospy.Subscriber("chatter", String, callback)
    #v = 1
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        #print(v)
        rospy.Subscriber("scan", LaserScan, callback_lidar)
        rospy.Subscriber("cmd_vel", Twist, callback_input_speed)
        rospy.Subscriber("beacon_data", USPS_msgs, callback_distance)
        rospy.Subscriber("odometry_and_IMU", odom_and_imu, callback_imu)
        rospy.Subscriber("pose_estimator_with_lidar", Pose, callback_pose_estimator_with_lidar)
        rospy.Subscriber("pose_estimator_without_lidar", Pose, callback_pose_estimator_without_lidar)

        with open(path, 'a') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            info = {
                "with_x": with_lidar_data[0],
                "with_y": with_lidar_data[1],
                "with_v": with_lidar_data[2],
                "with_hx": with_lidar_data[3],
                "with_hy": with_lidar_data[4],
                "without_x": without_lidar_data[0],
                "without_y": without_lidar_data[1],
                "without_v": without_lidar_data[2],
                "without_hx": without_lidar_data[3],
                "without_hy": without_lidar_data[4],
                "acc_x": IMU_data[0],
                "acc_y": IMU_data[1],
                "acc_z": IMU_data[2],
                "gyro_x": IMU_data[3],
                "gyro_y": IMU_data[4],
                "gyro_z": IMU_data[5],
                "ID": beacon_data[0],
                "distance": beacon_data[1],
                "linear_speed": input_speed,
                "angular_speed": angular_speed
                }

            csv_writer.writerow(info)
        rate.sleep()
    #rospy.spin()

if __name__ == '__main__':
    main()
