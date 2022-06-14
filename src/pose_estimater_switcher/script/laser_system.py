#!/usr/bin/env python3
import rospy
import math
import numpy as np
from std_msgs.msg import Int16

# missing beacons
# id, x, y, z
# 44050, 7825, 9999, 4286
# 44539, 1999, 10677, 3531


class Laser_component:
    def __init__(self):
        self.beacon_id = [44539, 44050, 42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
        self.beacon =    np.array([[1999, 7825, 11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560],[10677, 9999, 5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549]])
        self.magnetometer_corection_factor = math.atan2(13.1,-149.4) # please remmeber to update me ;)
        self.r_h = 198 # please remmeber to update me ;)
        self.std = 0.7887537272 # please remmeber to update me ;)
        self.mean = 1.464210526

    def projection(self, id, dist):
        beacon_z =    [3531, 4286, 5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767]
        index_of_data = self.beacon_id.index(id)
        #dist = math.sqrt(pow(beacon_x[index_of_data] - pose[0],2) + pow(beacon_y[index_of_data] - pose[1],2)+pow(beacon_z[index_of_data] - pose[2],2))
        d_z = (beacon_z[index_of_data] - self.r_h)/1000
        dist = (dist/1000)
        x = math.sqrt(abs(pow(dist,2)-pow(d_z,2)))
        return x

    def bell_function(self, max_occlusion_height): #input is in meters
        t = pow(max_occlusion_height-self.mean,2)/(pow(self.std,2))
        y = (1/(self.std * math.sqrt(2*math.pi)))*pow(math.e,(1/2)*t)
        bell_procent = - math.erf((math.sqrt(2)*(-math.inf + self.mean))/(2*self.std))/2 - math.erf((math.sqrt(2)*(max_occlusion_height - self.mean))/(2*self.std))/2
        new_y = bell_procent + 0.01
        print("new y: ",new_y)
        return new_y

    def assumed_dist(self, id, robot_pose):
        index_of_data = self.beacon_id.index(id)
        beacon_x =    [11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560]    #[11700  , 16244 , 7824  , 2000  , 21369 , 26163 , 26163 , 31000 , 35766 , 35766 , 40205 , 40204 , 16560 ]beacon_
        beacon_y =    [5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549]    #[5999   , 10150 , 5726  , 4499  , 6534  , 9939  , 3699  , 6519  , 10012  , 10012 , 11684 , 4363  , 3549 ]
        beacon_z =    [5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767]
        dist = math.sqrt(pow(beacon_x[index_of_data] - robot_pose[0],2) + pow(beacon_y[index_of_data] - robot_pose[1],2)+pow(beacon_z[index_of_data] - self.r_h,2))
        return dist

    def calculated_local_angle(self, id, robot_pose, mag_heading):
        if len(robot_pose) != 2 and len(mag_heading) != 2:
            print("sorry the input length was weird give that a look ;)")
            return -1
        else:
            index_of_data = self.beacon_id.index(id)
            mag_theta_for_robot = math.atan2(mag_heading[1],mag_heading[0])
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

    def potential_occlusion_check(self, lidar_array, id, robot_pose, mag_heading, dist, indi = False):

        pub1 = rospy.Publisher('indicator_color', Int16, queue_size=10)
        #try:
            #rospy.init_node('laser_node', anonymous=True)
        #except:
            #pass

        indicator = Int16()
        first_time = True
        beacon_z =    [5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767]
        if len(robot_pose) != 2 and len(mag_heading) != 2 and lidar_array != 360:
            print("sorry the input length was weird give that a look ;)")

            return -1
        else:
            indicator.data = 255
            #pub1.publish(indicator)
            #rospy.spin()
            bell = 0.01 # chage later to make sence ;)
            index_of_data = self.beacon_id.index(id)
            theta = int(math.degrees(self.calculated_local_angle(id, robot_pose, mag_heading)))
            #dist = (math.dist(self.beacon[:,index_of_data],robot_pose))/1000
            dist_projeted = self.projection(id, dist)
            #print(dist)
            assumed_dist = self.assumed_dist(id, robot_pose)
            assumed_dist_p = assumed_dist + (assumed_dist/100)*5    # The max distance with 5% error tolerance
            assumed_dist_m = assumed_dist - (assumed_dist/100)*5    # The min distance with 5% error tolerance
            if assumed_dist_p < dist or assumed_dist_m > dist:      # This if statmant will take objects that is not connected to the foor in to acount
                #print("bell value set to a value: ",bell)
                bell = 1000 # chage later to make sence ;)
            area_of_intreast = 2
            stored_dist = [12]*(2*area_of_intreast+1)
            j = 0
            for i in range(theta - area_of_intreast, theta + area_of_intreast):
                i = i % 359
                if lidar_array[i] < dist_projeted:
                    #if first_time and indi:
                        #indicator.data = 5
                        #pub1.publish(indicator)
                        #rospy.spin()

                    stored_dist[j] = lidar_array[i]
                    j = j + 1
                    #print(bell)
                    #print("here i will do some stuff later ;)")
                if i == (theta + area_of_intreast) % 359:
                    min_laser_measure = min(stored_dist)
                    beacon_h_minus_robot = (beacon_z[index_of_data] - self.r_h)/1000
                    angle_to_beacon = math.tan(beacon_h_minus_robot/dist_projeted)
                    height_to_occlusion = min_laser_measure/math.tan(angle_to_beacon)
                    bell = self.bell_function(height_to_occlusion)*10
            return bell
        #print("all good")
