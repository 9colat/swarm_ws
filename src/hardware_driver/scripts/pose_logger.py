#!/usr/bin/env python3
import rospy
import os
import sys
from pathlib import Path
import csv
from geometry_msgs.msg import Pose


path = Path.home().joinpath("test_data", "pose.csv")
fieldnames = ["x - coordinate", "y - coordinate", "z - coordinate"]
pose_est = [0] * 4
seperator = ","

def callback_pose_log(data):
    global path
    write_once = True
    with open(path,'w') as csv_file:
        csv_writer = csv.DictWriter(csv_file,fieldnames=fieldnames)
        csv_writer.writeheader()
    with open(path, 'a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        if write_once:
            info = {
                "x - coordinate": data.position.x,
                "y - coordinate": data.position.y,
                "z - coordinate": data.position.z
            }

            csv_writer.writerow(info)
            write_once = False
            print(info)

def main():
    global path, seperator, fieldnames
    rospy.init_node('pose_logger', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.Subscriber("USPS_pose_est", Pose, callback_pose_log)

        rate.sleep()


if __name__ == '__main__':
    main()
