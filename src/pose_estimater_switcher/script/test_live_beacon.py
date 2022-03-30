import rospy
import numpy as np
import math
import random
import csv
from pathlib import Path
from estimator_function import Pose_Calculator
from custom_msgs.msg import USPS_msgs

PC = Pose_Calculator(-1,-1,-1)
path = Path.home().joinpath("test_data", "pose.csv")
fieldnames = ["ID","distance","x", "y", "z"]
with open(path,'w') as csv_file:
    csv_writer = csv.DictWriter(csv_file,fieldnames=fieldnames)
    csv_writer.writeheader()

def callback_distance(data):
    global PC
    if data.ID != 44540:
        pose_meas_beacon = PC.pose_estimator(data.ID, data.distance)
        with open(path, 'a') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            info = {
                "ID": data.ID,
                "distance": data.distance,
                "x": pose_meas_beacon[0],
                "y": pose_meas_beacon[1],
                "z": pose_meas_beacon[2]
                }

            csv_writer.writerow(info)
        print(pose_meas_beacon)



def main():
    rospy.init_node('pose_logger', anonymous=True)
    rospy.Subscriber("beacon_data", USPS_msgs, callback_distance)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():

        #print("hey")

        rate.sleep()





if __name__ == '__main__':
    main()
