import rospy
import time
from custom_msgs.msg import odom_and_imu
from custom_msgs.msg import USPS_msgs
from sensor_msgs.msg import LaserScan
from ekf_usps import EKF
from laser_system import Laser_component




w1 = EKF()
w2 = Laser_component()

lidar_array = [0.0]*360
global_time = time.time() # this should be already in seconds, initialised



# gettng data from the beacons
def callback_distance(data):

    global w1, w2, global_time, lidar_array
    projected_distance = w2.projection(data.ID, data.distance)
    updated_R = w2.potential_occlusion_check(lidar_array, data.ID, [w1.state_predicted[0], w1.state_predicted[1]], [w1.state_predicted[3], w1.state_predicted[4]], data.distance) # measurement 'variance/trust' updated

    local_time = time.time()
    dT = local_time - global_time
    global_time = local_time

    state = w1.beacon_measurement_updater_EKF(data.ID, projected_distance, dT)
    # REMEMBER TO ADD UPDATED_R TO THE FUNCTIONS


# getting data from the IMU
def callback_imu(data):
    global w1, global_time

    local_time = time.time()
    dT = local_time - global_time
    global_time = local_time

    w1.updating_imu([[data.imu_acc.x], [data.imu_acc.y], [data.imu_acc.z]], [[data.imu_gyro.x], [data.imu_gyro.y], [data.imu_gyro.z]])
    #w1.magnetometer_measurement_updater_EKF([[data.imu_mag.x], [data.imu_mag.y], [data.imu_mag.z]], dT)


# getting data from the LiDAR
def callback_lidar(data): #from the beacon
    global lidar_array
    for i in range(len(data.ranges)):
        lidar_array[i] = data.ranges[i]



def main():

    global w1, w2, global_time

    # initialize subscribers
    rospy.init_node('pose_estimator', anonymous=True) # initialize the node
    rospy.Subscriber("beacon_data", USPS_msgs, callback_distance)
    rospy.Subscriber("odometry_and_IMU", odom_and_imu, callback_imu)
    rospy.Subscriber("scan", LaserScan, callback_lidar)



    while not rospy.is_shutdown():

        # time update for ONLY the predictor function in EKF
        local_time = time.time()
        dT = local_time - global_time
        global_time = local_time
        w1.state_predicted(dT)

        rospy.spin()


if __name__ == '__main__':
    main()
