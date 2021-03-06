#!/usr/bin/env python3
import rospy
import time
import numpy as np
from pathlib import Path
from custom_msgs.msg import odom_and_imu
from custom_msgs.msg import USPS_msgs
#from custom_msgs.msg import Q_R
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from ekf_usps import EKF
from laser_system import Laser_component

w1 = EKF()
w2 = Laser_component()

global_time = time.time() # this should be already in seconds, initialised
state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
first_time = True
ready_to_run = False

# missing beacons
# id, x, y, z
# 44050, 7825, 9999, 4286
# 44539, 1999, 10677, 3531

# gettng data from the beacons
def callback_distance(data):
    global w1, w2, global_time, first_time
    path = str(Path.home().joinpath("my_list.txt"))
    beacon_id = [44539, 44050, 42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540] #44540 - over tbl; 44535- gone ; 44539 - not on list
    if data.ID in beacon_id and data.ID != 42867:
        if data.distance < 11000:

            projected_distance = w2.projection(data.ID, data.distance) * 1000 # w2.projection() output is in m and there for it need to be converted to mm

            local_time = time.time()
            dT = local_time - global_time
            global_time = local_time

            state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
            if not first_time:
                file1 = open(path, "a")
            if first_time:
                file1 = open(path, "w")
                first_time = False
        #s = str(data.ID)+','+ str(data.distance) + ',' + str(int(state[0]) + ',' + str(int(state[1]))+"\n"
            file1.write(str(data.ID)+','+ str(data.distance) + ',' + str(int(state[0])) + ',' + str(int(state[1]))+"\n")
            file1.close()
        #print(state[0])
    # REMEMBER TO ADD UPDATED_R TO THE FUNCTIONS

#def callback_q_r(data):
    #global ready_to_run
    #w1.q_and_r_update(data.Q.data, data.R.data)
    #ready_to_run = True


def callback_terminating_signal(data):
    boll = data.data
    if boll == True:
        #print("i'ma fireing my LAZER!")
        rospy.spin(0)

# getting data from the IMU
def callback_imu(data):
    global w1, global_time

    local_time = time.time()
    dT = local_time - global_time
    global_time = local_time

    w1.updating_imu([[data.imu_acc.x], [data.imu_acc.y], [data.imu_acc.z]], [[data.imu_gyro.x], [data.imu_gyro.y], [data.imu_gyro.z]])
    #w1.magnetometer_measurement_updater_EKF([[data.imu_mag.x], [data.imu_mag.y], [data.imu_mag.z]], dT)

def main():
    global w1, w2, global_time, state

    # initialize subscribers
    rospy.init_node('pose_estimator_without_lidar', anonymous=True) # initialize the node
    rospy.Subscriber("beacon_data", USPS_msgs, callback_distance)
    rospy.Subscriber("odometry_and_IMU", odom_and_imu, callback_imu)
    #rospy.Subscriber("q_and_r", Q_R, callback_q_r)
    rospy.Subscriber("terminating_signal", Bool, callback_terminating_signal)
    pub = rospy.Publisher('without_lidar', Pose, queue_size=10)
    rate = rospy.Rate(100) # 100hz
    pose_est = Pose()


    #if ready_to_run:
    while not rospy.is_shutdown():
            # time update for ONLY the predictor function in EKF
        local_time = time.time()
        dT = local_time - global_time
        global_time = local_time
        state = w1.state_prediction(dT)
        #print("State x: ",state[0][0], "y: ",state[1][0])
        pose_est.position.x = int(state[0][0])
        pose_est.position.y = int(state[1][0])
        #print("ROS x: ",pose_est.position.x, "y: ",pose_est.position.y)
        #pose_est.position.z = state[2][0]
        pose_est.orientation.x = state[3][0]
        pose_est.orientation.y = state[4][0]
        pub.publish(pose_est)
        #print("without: ", state[0],state[1])
        rate.sleep()


if __name__ == '__main__':
    main()
