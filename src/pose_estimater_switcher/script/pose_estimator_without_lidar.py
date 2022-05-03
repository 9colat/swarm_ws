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

w1 = EKF()
w2 = Laser_component()

global_time = time.time() # this should be already in seconds, initialised
state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# gettng data from the beacons
def callback_distance(data):
    global w1, w2, global_time
    projected_distance = w2.projection(data.ID, data.distance)

    local_time = time.time()
    dT = local_time - global_time
    global_time = local_time

    state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
    # REMEMBER TO ADD UPDATED_R TO THE FUNCTIONS

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
    rospy.Subscriber("terminating_signal", Bool, callback_terminating_signal)
    pub = rospy.Publisher('without_lidar', Pose, queue_size=10)
    rate = rospy.Rate(100) # 100hz
    pose_est = Pose()



    while not rospy.is_shutdown():

        # time update for ONLY the predictor function in EKF
        local_time = time.time()
        dT = local_time - global_time
        global_time = local_time
        state = w1.state_prediction(dT)
        pose_est.position.x = state[0]
        pose_est.position.y = state[1]
        pose_est.position.x = state[2]
        pose_est.orientation.x = state[3]
        pose_est.orientation.y = state[4]
        pub.publish(pose_est)
        #print("without: ", state[0],state[1])
        rate.sleep()


if __name__ == '__main__':
    main()
