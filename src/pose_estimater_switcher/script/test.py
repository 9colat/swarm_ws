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

    projected_distance = w2.projection(ID, distance)
    updated_R = w2.potential_occlusion_check(lidar_array, ID, [w1.state_predicted[0][0], w1.state_predicted[0][1]], [w1.state_predicted[0][3], w1.state_predicted[0][4]], distance) # measurement 'variance/trust' updated
    #print(updated_R)
    print(w1.state_predicted[0][0],w1.state_predicted[0][1],w1.state_predicted[0][2],w1.state_predicted[0][3],w1.state_predicted[0][4])
    print(w1.state_predicted[0][-1])

if __name__ == '__main__':
    test()
