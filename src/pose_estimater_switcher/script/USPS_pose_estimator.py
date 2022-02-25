#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
from datetime import datetime
import numpy as np
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
        pose_est = [0,0,0]
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
            pose_est[0] = old_pose[0] + alpha * x_array[k] - old_pose[0]
            pose_est[1] = old_pose[1] + alpha * y_array[k] - old_pose[1]
            pose_est[2] = old_pose[2] + alpha * z_array[k] - old_pose[2]

            dist_new = math.sqrt(pow(old_pose[0] - pose_est[0], 2) + pow(old_pose[1] - pose_est[1], 2) + pow(old_pose[2] - pose_est[2], 2))

            if dist_new <= 1:
                pose_est[0] = old_pose[0] + 0.2 * (x_array[k] - pose_est[0])/dp
                pose_est[1] = old_pose[1] + 0.2 * (y_array[k] - pose_est[1])/dp
                pose_est[2] = old_pose[2] + 0.2 * (z_array[k] - pose_est[2])/dp

        #add sort array of the maybe sorted by the time elapsed since it was set
        old_pose[0] = pose_est[0]
        old_pose[1] = pose_est[1]
        old_pose[2] = pose_est[2]

        return pose_est


    def pose_estimator_trilatertion(self):
        id_array = [0] * 11
        x_array = [0] * 11
        x_sort_array = [0] * 4
        y_array = [0] * 11
        y_sort_array = [0] * 4
        z_array = [0] * 11
        z_sort_array = [0] * 4
        pose_est = [0,0,0]
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

        dist_sort = sorted(dist_array, reverse=True)
        x_sort_array[0] = x_array[dist_array.index(dist_sort[0])]
        y_sort_array[0] = y_array[dist_array.index(dist_sort[0])]
        z_sort_array[0] = z_array[dist_array.index(dist_sort[0])]
        if dist_sort[0] == dist_sort[1]:
            x_sort_array[1] = x_array[dist_array.index(dist_sort[1],dist_array.index(dist_sort[0])+1, len(x_array))]
            y_sort_array[1] = y_array[dist_array.index(dist_sort[1],dist_array.index(dist_sort[0])+1, len(y_array))]
            z_sort_array[1] = z_array[dist_array.index(dist_sort[1],dist_array.index(dist_sort[0])+1, len(z_array))]
        if dist_sort[0] != dist_sort[1]:
            x_sort_array[1] = x_array[dist_array.index(dist_sort[1])]
            y_sort_array[1] = y_array[dist_array.index(dist_sort[1])]
            z_sort_array[1] = z_array[dist_array.index(dist_sort[1])]
        if dist_sort[0] == dist_sort[2]:
            x_sort_array[2] = x_array[dist_array.index(dist_sort[2],dist_array.index(dist_sort[0])+1, len(x_array))]
            y_sort_array[2] = y_array[dist_array.index(dist_sort[2],dist_array.index(dist_sort[0])+1, len(y_array))]
            z_sort_array[2] = z_array[dist_array.index(dist_sort[2],dist_array.index(dist_sort[0])+1, len(z_array))]
        if dist_sort[1] == dist_sort[2]:
            x_sort_array[2] = x_array[dist_array.index(dist_sort[2],dist_array.index(dist_sort[1])+1, len(x_array))]
            y_sort_array[2] = y_array[dist_array.index(dist_sort[2],dist_array.index(dist_sort[1])+1, len(y_array))]
            z_sort_array[2] = z_array[dist_array.index(dist_sort[2],dist_array.index(dist_sort[1])+1, len(z_array))]
        if dist_sort[0] != dist_sort[2] and dist_sort[1] != dist_sort[2]:
            x_sort_array[2] = x_array[dist_array.index(dist_sort[2])]
            y_sort_array[2] = y_array[dist_array.index(dist_sort[2])]
            z_sort_array[2] = z_array[dist_array.index(dist_sort[2])]
        if dist_sort[0] == dist_sort[3]:
            x_sort_array[3] = x_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[0])+1, len(x_array))]
            y_sort_array[3] = y_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[0])+1, len(y_array))]
            z_sort_array[3] = z_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[0])+1, len(z_array))]
        if dist_sort[1] == dist_sort[3]:
            x_sort_array[3] = x_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[1])+1, len(x_array))]
            y_sort_array[3] = y_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[1])+1, len(y_array))]
            z_sort_array[3] = z_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[1])+1, len(z_array))]
        if dist_sort[2] == dist_sort[3]:
            x_sort_array[3] = x_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[2])+1, len(x_array))]
            y_sort_array[3] = y_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[2])+1, len(y_array))]
            z_sort_array[3] = z_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[2])+1, len(z_array))]
        if dist_sort[0] != dist_sort[3] and dist_sort[1] != dist_sort[3] and dist_sort[2] != dist_sort[3]:
            x_sort_array[3] = x_array[dist_array.index(dist_sort[2])]
            y_sort_array[3] = y_array[dist_array.index(dist_sort[2])]
            z_sort_array[3] = z_array[dist_array.index(dist_sort[2])]


        print([x_sort_array[1]-x_sort_array[0],y_sort_array[1]-y_sort_array[0],z_sort_array[1]-z_sort_array[0]])
        print([x_sort_array[2]-x_sort_array[1],y_sort_array[2]-y_sort_array[1],z_sort_array[2]-z_sort_array[1]])
        print([x_sort_array[3]-x_sort_array[0],y_sort_array[3]-y_sort_array[0],z_sort_array[3]-z_sort_array[0]])
        m_1 = np.array([[x_sort_array[1]-x_sort_array[0],y_sort_array[1]-y_sort_array[0],z_sort_array[1]-z_sort_array[0]],[x_sort_array[2]-x_sort_array[1],y_sort_array[2]-y_sort_array[1],z_sort_array[2]-z_sort_array[1]],[x_sort_array[3]-x_sort_array[0],y_sort_array[3]-y_sort_array[0],z_sort_array[3]-z_sort_array[0]]])
        print(m_1)
        m_1t = m_1.transpose()
        m_2 = np.array([[pow(x_sort_array[1],2)-pow(x_sort_array[0],2)+pow(y_sort_array[1],2)-pow(y_sort_array[0],2)+pow(z_sort_array[1],2)-pow(z_sort_array[0],2)],[pow(x_sort_array[2],2)-pow(x_sort_array[1],2)+pow(y_sort_array[2],2)-pow(y_sort_array[1],2)+pow(z_sort_array[2],2)-pow(z_sort_array[1],2)],[pow(x_sort_array[3],2)-pow(x_sort_array[0],2)+pow(y_sort_array[3],2)-pow(y_sort_array[0],2)+pow(z_sort_array[3],2)-pow(z_sort_array[0],2)]])
        m_pose = (1/2) * m_1t.dot(m_2)
        print(m_pose)

w1 = USPS_data()


def callback_pose_est(data):
    global w1
    w1.updating_distance(44533,17372)
    print("im in the callback")


def pose_estimator():
    global w1
    w1.updating_distance(44531,23)
    w1.updating_distance(44532,64534)
    w1.updating_distance(44533,12312)
    w1.updating_distance(44538,3848)
    w1.pose_estimator_trilatertion()
    dist_sort = sorted(w1.distance, reverse=True)
    print(dist_sort[-2])
    print(w1.distance[-2])
    #print(w1.distance[self.id.index(44532)])
    #print(w1.pose_estimator_henrik_method())
    #i = [1,2,3]
    #print(i[1])
    #rospy.init_node('USPS_pose_estimator', anonymous=True)

    #rospy.Subscriber("robot_position_estimate", Vector3, callback_pose_est)

    #rospy.spin()

if __name__ == '__main__':
    pose_estimator()
