#!/usr/bin/env python3
import rospy
from ekf_usps import EKF
from laser_system import Laser_component




def test():
    ID = 42929
    distance = 10000
    lidar_array = [12]*360

    w1 = EKF()
    w2 = Laser_component()


    print(w1.imu_gyro[2][0])

if __name__ == '__main__':
    test()
