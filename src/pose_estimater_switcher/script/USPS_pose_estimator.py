#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
from datetime import datetime
import numpy as np
import math

old_time = datetime.now()
old_time_timestamp = datetime.timestamp(old_time)

class USPS_data:
    def __init__(self):
        self.id = [42928, 42929, 44530, 44531, 44532, 44533, 44534, 44536, 44537, 44538, 44540]
        self.x = [16244, 7824, 2000, 21369, 26163, 26163, 31000, 35766, 40205, 40204, 16560]
        self.y = [10150, 5726, 4499, 6534, 9939, 3699, 6519, 3522, 11684, 4363, 3549]
        self.z = [5577, 4286, 3530, 5578, 5577, 5577, 5578, 5578, 3767, 3767, 5577]
        self.time = [0,0,0,0,0,0,0,0,0,0,0]
        self.distance = [0,0,0,0,0,0,0,0,0,0,0]
        self.RSSI = [0,0,0,0,0,0,0,0,0,0,0]
        self.pose_est = [16000.0, 6000.0, 300.0]
        self.pose_est_stored = [[16000.0, 6000.0, 380.0],[16000.0, 6000.0, 310.0],[16000.0, 6000.0, 360.0]]
        self.pose_predict_from_pose = [0.0, 0.0, 0.0]
        self.time_i = [3.0, 2.0, 1.0]
        self.acc_meas = [0.0, 0.0, 0.0]
        self.omega = [0.0, 0.0]
        self.r = 0.04
        self.l = 0.229



    def pose_predict(self, time):
        for j in range(len(self.pose_est_stored[1])):
            self.pose_predict_from_pose[j] =  self.pose_est[j] + ((self.pose_est_stored[0][j]-self.pose_est_stored[1][j])/(self.time_i[0]-self.time_i[1])) * time + 1/2*((((self.pose_est_stored[0][j]-self.pose_est_stored[1][j])/(self.time_i[0]-self.time_i[1]))-((self.pose_est_stored[1][j]-self.pose_est_stored[2][j])/(self.time_i[1]-self.time_i[2])))/(self.time_i[0]-self.time_i[2]))*pow(time,2)
        return self.pose_predict_from_pose





    def updating_distance(self, id, distance):
        dt = datetime.now()
        ts = datetime.timestamp(dt)
        index_of_data = self.id.index(id)
        self.distance[index_of_data] = distance
        self.time[index_of_data] = ts

    def updating_acc(self, acc_x, acc_y, acc_z):
        self.acc_meas[0] = acc_x
        self.acc_meas[1] = acc_y
        self.acc_meas[2] = acc_z


    def velocity_cal(self):
        v = self.omega[0] * self.r + self.omega[1] * self.r
        return v


    def pose_estimator_henrik_method(self):
        id_array = [0] * 11
        x_array = [0] * 11
        y_array = [0] * 11
        z_array = [0] * 11
        pose_esti = [0,0,0]
        dist_array = [0] * 11
        j = 0

        period = 1000
        dt = datetime.now()
        ts = datetime.timestamp(dt)
        for i in range(len(self.distance)):
            if self.time[i] > 0:
                if self.time[i] > ts-period:
                    id_array[j] = self.id[i]
                    x_array[j] = self.x[i]
                    y_array[j] = self.y[i]
                    z_array[j] = self.z[i]
                    dist_array[j] = self.distance[i]
                    j = j + 1

        for k in range(j+1):
            dp = math.sqrt(pow(self.pose[0] - x_array[k], 2) + pow(self.pose[1] - y_array[k], 2) + pow(self.pose[2] - z_array[k], 2))
            alpha = (dp - self.distance[k])/(dp + 10)
            pose_est[0] = self.pose[0] + alpha * x_array[k] - self.pose[0]
            pose_est[1] = self.pose[1] + alpha * y_array[k] - self.pose[1]
            pose_est[2] = self.pose[2] + alpha * z_array[k] - self.pose[2]

            dist_new = math.sqrt(pow(self.pose[0] - pose_est[0], 2) + pow(self.pose[1] - pose_est[1], 2) + pow(self.pose[2] - pose_est[2], 2))

            if dist_new <= 1:
                pose_est[0] = self.pose[0] + 0.2 * (x_array[k] - pose_est[0])/dp
                pose_est[1] = self.pose[1] + 0.2 * (y_array[k] - pose_est[1])/dp
                pose_est[2] = self.pose[2] + 0.2 * (z_array[k] - pose_est[2])/dp

        #add sort array of the maybe sorted by the time elapsed since it was set
        self.pose_est[0] = pose_esti[0]
        self.pose_est[1] = pose_esti[1]
        self.pose_est[2] = pose_esti[2]

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

        m_1 = np.array([[x_sort_array[1]-x_sort_array[0],y_sort_array[1]-y_sort_array[0],z_sort_array[1]-z_sort_array[0]],[x_sort_array[2]-x_sort_array[1],y_sort_array[2]-y_sort_array[1],z_sort_array[2]-z_sort_array[1]],[x_sort_array[3]-x_sort_array[0],y_sort_array[3]-y_sort_array[0],z_sort_array[3]-z_sort_array[0]]])
        #print(m_1)
        m_1t = m_1.transpose()
        m_2 = np.array([[pow(x_sort_array[1],2)-pow(x_sort_array[0],2)+pow(y_sort_array[1],2)-pow(y_sort_array[0],2)+pow(z_sort_array[1],2)-pow(z_sort_array[0],2)],[pow(x_sort_array[2],2)-pow(x_sort_array[1],2)+pow(y_sort_array[2],2)-pow(y_sort_array[1],2)+pow(z_sort_array[2],2)-pow(z_sort_array[1],2)],[pow(x_sort_array[3],2)-pow(x_sort_array[0],2)+pow(y_sort_array[3],2)-pow(y_sort_array[0],2)+pow(z_sort_array[3],2)-pow(z_sort_array[0],2)]])
        m_pose = (1/2) * m_1t.dot(m_2)
        #self.pose[0] = m_pose[0]
        #self.pose[1] = m_pose[1]
        #self.pose[2] = m_pose[2]
        #print(m_pose)
        return m_pose

    def measured_model_imu_and_odometry(self, time):
        pose_predict_meas = [0,0,0]
        pose_predict_meas[0] = self.pose[0] + self.velocity_cal() * math.cos(((self.l*(self.omega[0] - self.omega[1])* self.r)/2)*time)*time + 1/2 * self.acc_meas[0] * pow(time,2)
        pose_predict_meas[1] = self.pose[1] + self.velocity_cal() * math.sin(((self.l*(self.omega[0] - self.omega[1])* self.r)/2)*time)*time + 1/2 * self.acc_meas[1] * pow(time,2)
        pose_predict_meas[2] = self.pose[2]
        return pose_predict_meas

w1 = USPS_data()


def callback_distance(data):
    global w1
    w1.updating_distance(data.x,data.y)
    #print("im in dist. the callback")

def callback_imu_acc(data):
    global w1
    w1.updating_acc(data.x,data.y,data.z)
    #print("im in acc the callback")


def pose_estimator():
    global w1
    w1.updating_distance(44531,23)
    w1.updating_distance(44532,64534)
    w1.updating_distance(44533,12312)
    print(w1.pose_predict(10))
    w1.pose_estimator_trilatertion()
    dist_sort = sorted(w1.distance, reverse=True)
    #print(w1.distance[self.id.index(44532)])
    #print(w1.pose_estimator_henrik_method())
    #i = [1,2,3]
    #print(i[1])
    #rospy.init_node('USPS_pose_estimator', anonymous=True)
    #rospy.Subscriber("imu_acc", Vector3, callback_imu_acc)
    #rospy.Subscriber("robot_position_estimate", Vector3, callback_distance)

    #rospy.spin()

if __name__ == '__main__':
    pose_estimator()
