#!/usr/bin/env python3
import rospy
import time
import numpy as np
from custom_msgs.msg import odom_and_imu
from custom_msgs.msg import USPS_msgs
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from ekf_usps import EKF
from laser_system import Laser_component

w_laser = Laser_component()
w1 = EKF()
w2 = EKF()
w3 = EKF()
w4 = EKF()
w5 = EKF()
w6 = EKF()
w7 = EKF()
w8 = EKF()
w9 = EKF()
w10 = EKF()
w11 = EKF()
w12 = EKF()
w0 = EKF()


global_time = time.time() # this should be already in seconds, initialised
state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# gettng data from the beacons
def callback_distance(data):
    global w_laser, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w0, global_time
    beacon_id = [42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
    if data.ID in beacon_id:
        projected_distance = w_laser.projection(data.ID, data.distance) * 1000 # w_laser.projection() output is in m and there for it need to be converted to mm
        temp_array = [0] * len(beacon_id)

        local_time = time.time()
        dT = local_time - global_time
        global_time = local_time

        if data.ID == beacon_id[0]:
            state = w2.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w3.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w4.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w5.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w6.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w7.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w8.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w9.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w10.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w11.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w12.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if data.ID == beacon_id[1]:
            state = w0.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w3.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w4.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w5.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w6.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w7.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w8.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w9.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w10.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w11.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w12.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w2.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if data.ID == beacon_id[2]:
            state = w0.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w4.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w5.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w6.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w7.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w8.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w9.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w10.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w11.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w12.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w3.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if data.ID == beacon_id[3]:
            state = w2.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w3.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w5.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w6.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w7.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w8.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w9.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w10.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w11.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w12.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w4.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if data.ID == beacon_id[4]:
            state = w2.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w3.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w4.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w6.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w7.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w8.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w9.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w10.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w11.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w12.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w5.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if data.ID == beacon_id[5]:
            state = w2.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w3.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w4.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w5.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w7.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w8.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w9.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w10.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w11.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w12.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w6.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if data.ID == beacon_id[6]:
            state = w2.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w3.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w4.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w5.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w6.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w8.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w9.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w10.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w11.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w12.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w7.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if data.ID == beacon_id[7]:
            state = w2.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w3.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w4.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w5.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w6.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w7.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w9.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w10.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w11.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w12.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w8.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if data.ID == beacon_id[8]:
            state = w2.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w3.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w4.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w5.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w6.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w7.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w8.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w10.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w11.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w12.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w9.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if data.ID == beacon_id[9]:
            state = w2.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w3.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w4.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w5.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w6.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w7.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w8.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w9.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w11.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w12.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w10.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if data.ID == beacon_id[10]:
            state = w2.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w3.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w4.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w5.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w6.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w7.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w8.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w9.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w10.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w12.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w11.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if data.ID == beacon_id[11]:
            state = w2.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w3.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w4.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w5.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w6.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w7.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w8.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w9.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w10.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w11.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w12.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if data.ID == beacon_id[12]:
            state = w2.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w3.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w4.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w5.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w6.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w7.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w8.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w9.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w10.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w11.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w0.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)

        temp_array[0] = w0.beacon_estimation_difference
        temp_array[1] = w1.beacon_estimation_difference
        temp_array[2] = w2.beacon_estimation_difference
        temp_array[3] = w3.beacon_estimation_difference
        temp_array[4] = w4.beacon_estimation_difference
        temp_array[5] = w5.beacon_estimation_difference
        temp_array[6] = w6.beacon_estimation_difference
        temp_array[7] = w7.beacon_estimation_difference
        temp_array[8] = w8.beacon_estimation_difference
        temp_array[9] = w9.beacon_estimation_difference
        temp_array[10] = w10.beacon_estimation_difference
        temp_array[11] = w11.beacon_estimation_difference
        temp_array[12] = w12.beacon_estimation_difference


        dist_sort = sorted(temp_array) # for sorting

        if temp_array.index(dist_sort[0]) == 0:
            state = w0.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if temp_array.index(dist_sort[0]) == 1:
            state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if temp_array.index(dist_sort[0]) == 2:
            state = w2.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if temp_array.index(dist_sort[0]) == 3:
            state = w3.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if temp_array.index(dist_sort[0]) == 4:
            state = w4.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if temp_array.index(dist_sort[0]) == 5:
            state = w5.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if temp_array.index(dist_sort[0]) == 6:
            state = w6.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if temp_array.index(dist_sort[0]) == 7:
            state = w7.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if temp_array.index(dist_sort[0]) == 8:
            state = w8.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if temp_array.index(dist_sort[0]) == 9:
            state = w9.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if temp_array.index(dist_sort[0]) == 10:
            state = w10.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if temp_array.index(dist_sort[0]) == 11:
            state = w11.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        if temp_array.index(dist_sort[0]) == 12:
            state = w12.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
        #print(state[0])

    # REMEMBER TO ADD UPDATED_R TO THE FUNCTIONS

def callback_terminating_signal(data):
    boll = data.data
    if boll == True:
        #print("i'ma fireing my LAZER!")
        rospy.spin(0)

# getting data from the IMU
def callback_imu(data):
    global w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w0, global_time

    local_time = time.time()
    dT = local_time - global_time
    global_time = local_time

    w1.updating_imu([[data.imu_acc.x], [data.imu_acc.y], [data.imu_acc.z]], [[data.imu_gyro.x], [data.imu_gyro.y], [data.imu_gyro.z]])
    #w1.magnetometer_measurement_updater_EKF([[data.imu_mag.x], [data.imu_mag.y], [data.imu_mag.z]], dT)

def main():
    global w_laser, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w0, global_time, state

    # initialize subscribers
    rospy.init_node('pose_estimator_without_lidar', anonymous=True) # initialize the node
    rospy.Subscriber("beacon_data", USPS_msgs, callback_distance)
    rospy.Subscriber("odometry_and_IMU", odom_and_imu, callback_imu)
    rospy.Subscriber("terminating_signal", Bool, callback_terminating_signal)
    pub = rospy.Publisher('multi_kalman', Pose, queue_size=10)
    rate = rospy.Rate(100) # 100hz
    pose_est = Pose()



    while not rospy.is_shutdown():

        # time update for ONLY the predictor function in EKF
        local_time = time.time()
        dT = local_time - global_time
        global_time = local_time
        state = w1.state_prediction(dT)
        print("State x: ",state[0][0], "y: ",state[1][0])
        pose_est.position.x = int(state[0][0])
        pose_est.position.y = int(state[1][0])
        print("ROS x: ",pose_est.position.x, "y: ",pose_est.position.y)
        #pose_est.position.z = state[2][0]
        pose_est.orientation.x = state[3][0]
        pose_est.orientation.y = state[4][0]
        pub.publish(pose_est)
        #print("without: ", state[0],state[1])
        rate.sleep()


if __name__ == '__main__':
    main()
