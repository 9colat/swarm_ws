import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
import random
from estimator_function import Pose_Calculator
from custom_msgs.msg import USPS_msgs

PC = Pose_Calculator(-1,-1,-1)

def callback_distance(data):
    global PC
    pose_meas_beacon = PC.pose_estimator(data.ID, data.distance)
    print(pose_meas_beacon)



def main():
    rospy.Subscriber("beacon_data", USPS_msgs, callback_distance)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():



        rate.sleep()





if __name__ == '__main__':
    main()
