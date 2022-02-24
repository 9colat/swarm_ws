#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
from datetime import datetime
import math

old_time = datetime.now()
old_time_timestamp = datetime.timestamp(old_time)
old_pose = [16000.0, 6000.0, 300.0]

class USPS_data:
    def __init__(self):
        self.id = [42928, 42929, 44530, 44531, 44532, 44533, 44534, 44536, 44537, 44538, 44540]
        self.x = [16244, 7824, 2000, 21369, 26163, 26163, 31000, 35766, 40205, 40204, 16560]
        self.y = [10150, 5726, 4499, 6534, 9939, 3699, 6519, 3522, 11684, 4363, 3549]
        self.z = [5577, 4286, 3530, 5578, 5577, 5577, 5578, 5578, 3767, 3767, 5577]
        self.time = [0,0,0,0,0,0,0,0,0,0,0]
        self.distance = [0,0,0,0,0,0,0,0,0,0,0]
        self.RSSI = [0,0,0,0,0,0,0,0,0,0,0]

    def updating_distance(self, id, distance):
        dt = datetime.now()
        ts = datetime.timestamp(dt)
        index_of_data = self.id.index(id)
        self.distance[index_of_data] = distance
        self.time[index_of_data] = ts

    def pose_estimator_henrik_method(self):
        global old_pose
        id_array = [0] * 11
        x_array = [0] * 11
        y_array = [0] * 11
        z_array = [0] * 11
        dist_array = [0] * 11
        j = 0

        period = 1000
        dt = datetime.now()
        ts = datetime.timestamp(dt)
        for i in range(len(self.distance)):
            if self.time[i] > 0:
                if self.time[i] < ts+period:
                    id_array[j] = self.id[i]
                    x_array[j] = self.x[i]
                    y_array[j] = self.y[i]
                    z_array[j] = self.z[i]
                    dist_array[j] = self.distance[i]
                    j = j + 1

        for k in range(j+1):
            dp = math.sqrt(pow(old_pose[0] - x_array[k], 2) + pow(old_pose[1] - y_array[k], 2) + pow(old_pose[2] - z_array[k], 2))
            alpha = (dp - self.distance[k])/(dp + 10)


        #add sort array of the maybe sorted by the time elapsed since it was set


        return


    def pose_estimator_trilatertion(self):
        self.length

def callback_pose_est(data):
    data.x





def pose_estimator():
    w1 = USPS_data()
    w1.updating_distance(44533,59604)
    print(w1.distance[w1.id.index(44533)])
    #i = [1,2,3]
    #print(i[1])
    #rospy.init_node('USPS_pose_estimator', anonymous=True)

    #rospy.Subscriber("robot_position_estimate", Vector3, callback_pose_est)

    #rospy.spin()

if __name__ == '__main__':
    pose_estimator()
