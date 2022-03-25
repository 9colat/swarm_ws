#!/usr/bin/env python3
import math
import numpy as np



class Laser_component:
    def __init__(self):
        self.beacon_id = [42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
        self.beacon =    np.array([[11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560],[5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549]])
        self.magnetometer_corection_factor = math.radians(30)

    def calculated_local_angle(self, id, robot_pose, heading):
        if len(robot_pose) != 2 and len(heading) != 2:
            print("sorry the input length was weird give that a look ;)")
            return -1
        else:
            index_of_data = self.beacon_id.index(id)
            mag_theta_for_robot = math.atan2(heading[1],heading[0])
            robot_heading_x = math.cos(mag_theta_for_robot + self.magnetometer_corection_factor)
            robot_heading_y = math.sin(mag_theta_for_robot + self.magnetometer_corection_factor)
            robot_heading_corrected = math.atan2(robot_heading_y,robot_heading_x)
            beacon_x = self.beacon[0,index_of_data]
            beacon_y = self.beacon[1,index_of_data]
            beacon_in_realation_to_robot_x = beacon_x - robot_pose[0]
            beacon_in_realation_to_robot_y = beacon_y - robot_pose[1]
            beacon_global_thata = math.atan2(beacon_in_realation_to_robot_y,beacon_in_realation_to_robot_x)
            beacon_theta = beacon_global_thata - robot_heading_corrected
            #print(beacon_theta)
            if beacon_theta < 0:
                beacon_theta = math.pi * 2 + beacon_theta
                #print(beacon_theta)
            return beacon_theta

    def potential_occlusion_check(self, lidar_array, id, robot_pose, heading):
        if len(robot_pose) != 2 and len(heading) != 2 and lidar_array != 360:
            print("sorry the input length was weird give that a look ;)")
            return -1
        else:
            index_of_data = self.beacon_id.index(id)
            theta = int(math.degrees(self.calculated_local_angle(id, robot_pose, heading)))
            dist = (math.dist(self.beacon[:,index_of_data],robot_pose))/1000
            #print(dist)
            for i in range(theta - 5, theta + 5):
                if lidar_array[i] < dist:
                    print("here i will do some stuff later ;)")
                    return 1
        #print("all good")
