#!/usr/bin/env python3
import math
import matplotlib.pyplot as plt
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
            print(beacon_theta)
            if beacon_theta < 0:
                beacon_theta = math.pi * 2 + beacon_theta
                print(beacon_theta)
            return beacon_theta

    def potential_occlusion_check(self, lidar_array, id, robot_pose, heading):
        if len(robot_pose) != 2 and len(heading) != 2 and lidar_array != 360:
            print("sorry the input length was weird give that a look ;)")
            return -1
        else:
            index_of_data = self.beacon_id.index(id)
            theta = int(math.degrees(self.calculated_local_angle(id, robot_pose, heading)))
            dist = (math.dist(self.beacon[:,index_of_data],robot_pose))/1000
            for i in range(theta - 5, theta + 5):
                if lidar_array[i] < dist:
                    print("here i will do some stuff later ;)")
                    return 1
        print("all good")






w2 = Laser_component()
lidar_array = [0.014] * 360
theta_r = math.radians(50)
theta_m = math.radians(30)
r_x = 16234
r_y = 10140
x_m = math.cos(theta_m)
y_m = math.sin(theta_m)
m_t = math.atan2(y_m, x_m)
r_h_x = math.cos(theta_r + m_t)
r_h_y = math.sin(theta_r + m_t)

angle = np.linspace( 0 , 2 * np.pi , 150 )

radius = 1

x = (radius * np.cos( angle )) + r_x
y = (radius * np.sin( angle )) + r_y

b_x = 16244
b_y = 10150
b_id = 42928
b_xr = b_x-r_x
b_yr = b_y-r_y

b_t = math.atan2(b_yr,b_xr)
t_rb = int(b_t - math.atan2(r_h_y,r_h_x))


pt = int(math.degrees(w2.calculated_local_angle(b_id,[r_x,r_y],[math.cos(theta_r),math.sin(theta_r)])))
#print(math.degrees(math.atan2(r_h_y,r_h_x)),math.degrees(m_t),math.degrees(theta_r),math.degrees(t_rb))

b_c = w2.beacon[:,w2.beacon_id.index(b_id)]
fun = int(math.degrees(math.atan2(b_c[0]-r_x,b_c[1]-r_y)))
print(fun)
plt.style.use('fivethirtyeight')
#fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
fig = plt.figure(figsize=(10,10))
fig.suptitle('pose plot test')
ax1 = fig.add_subplot(1,1,1)
ax1.plot(x, y, c='b',zorder=5)
for i in range(fun-5, fun+5):
    print(i)
    ax1.scatter((lidar_array[i] * math.cos(math.radians(i)))*1000 + r_x,(lidar_array[i] * math.sin(math.radians(i)))* 1000 + r_y, c='r')
ax1.scatter(r_x, r_y, c='b', zorder=10)
ax1.scatter(x_m+r_x, y_m+r_y, c='r',zorder=10)
ax1.scatter(r_h_x + r_x, r_h_y + r_y, c='g', zorder=10)
ax1.scatter(b_x, b_y, c='y', zorder=10)
plt.show()
