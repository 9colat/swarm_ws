#!/usr/bin/env python3
import matplotlib.pyplot as plt
import random
import numpy as np
import math
import rospy
from custom_msgs.msg import odom_and_imu
from custom_msgs.msg import USPS_msgs
from ekf_usps import EKF #this is our thing huhu


w1 = EKF()

def rot_mat(theta):
    output = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    return output



# constants
rotated_matrix = rot_mat(math.pi/2)
mag_x_calibrated = 0.0
mag_y_calibrated = 0.0



# for plotting
#plt.ion()
#fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)

fig = plt.figure(figsize = (20, 10))
fig.suptitle('Kalman Filter Test')
ax = fig.add_subplot(1,2,1)
ax2 = fig.add_subplot(1,2,2)



def main():
    n = 2000
    test_array = [0.0]*n
    head = np.array([[1.0, 0.0]]).T
    w1.predicted_heading = head
    new_position = np.array([[1.0, 1.0]]).T
    dT = 0.1
    velocity = 1
    omega = 0.2
    w1.imu_gyro[2] = omega
    arr = [42867, 42928,  42929]
    test_arr = np.array([[1.0]*2]*n)
    test_arr_pose = np.array([[1.0]*2]*n)
    mag_measurement = np.array([[0.0, 0.0]])
    mag_measurement_normalised = np.array([[0.0, 0.0]])


    R_matrix = np.dot(rot_mat(0), np.array([[1, 0]]).T)
    #print(R_matrix.T)
    #ax.scatter(head[0],head[1],c='g',zorder=15)




    for x in range(n):
        id = w1.id.index(arr[x%3])
        new_position = new_position + head * velocity * dT
        dist = math.sqrt(pow(w1.x[id] - new_position[0], 2) + pow(w1.y[id] - new_position[1], 2)) #+ random.uniform(-1, 1)
        #print(head)
        head = head + np.dot(rotated_matrix, head) * omega * dT # true heading
        #print(head)
        #head = np.divide(head, math.sqrt(pow(head[0], 2) + pow(head[1], 2)))
        noise_x = random.uniform(-0.001, 0.001)
        noise_y = random.uniform(-0.001, 0.001)
        # with noise:
        #mag_measurement = np.array([[float(np.dot(R_matrix.T, head) + noise_x)], [float(np.dot(R_matrix.T, np.dot(rotated_matrix, head)) + noise_y)]])
        # no noise:
        mag_measurement = np.array([[float(np.dot(R_matrix.T, head))], [float(np.dot(np.dot(R_matrix.T, rotated_matrix), head))]])
        #print(R_matrix.T, head)
        #print(mag_measurement)
        mag_measurement_normalised = np.divide(mag_measurement, math.sqrt(pow(mag_measurement[0], 2) + pow(mag_measurement[1], 2)))/math.sqrt(pow(mag_measurement[0], 2) + pow(mag_measurement[1], 2))
        #mag_measurement_normalised[0][1] = mag_measurement[1] / math.sqrt(pow(mag_measurement[0], 2) + pow(mag_measurement[1], 2))
        #print("mag: ", mag_measurement)
        #print("mag: ", mag_measurement)
        #print("mag normalised: ", mag_measurement_normalised)
        #print(round(math.sqrt(pow(mag_measurement_normalised[0], 2) + pow(mag_measurement_normalised[1], 2))))


        #test_array[x] = w1.beacon_measurement_updater_EKF(arr[x%3], dist + random.uniform(-0.1, 0.1), dT)
        #w1.beacon_measurement_updater_EKF(arr[x%3], dist + random.uniform(-0.1, 0.1), dT)
        test_array[x] = w1.magnetometer_measurement_updater_EKF(mag_measurement_normalised, dT)
        #test_array[x] = w1.magnetometer_measurement_updater_EKF(mag_measurement_normalised, dT)
        test_arr_pose[x] = new_position.T
        test_arr[x] = head.T

        #ax.scatter(mag_measurement_normalised[0], mag_measurement_normalised[1], c='y',zorder=10)
        ax.scatter(R_matrix[0],R_matrix[1], c='c')
        ax.scatter(test_array[x][3],test_array[x][4],c='r',zorder=10) #predicted
        #ax2.scatter(test_array[x][0],test_array[x][1],c='r',zorder=10) #predicted
        #ax.scatter(test_array[x][0],test_array[x][1],c='r') #predicted
        ax2.scatter(x,float(w1.state_predicted[3]),c='r',zorder=15)
        ax2.scatter(x,float(w1.state_predicted[4]),c='y',zorder=15)
        #print(x,float(w1.state_predicted[3]))
        ax2.scatter(x,test_arr[x][0], c='b',zorder=10) # true
        ax2.scatter(x,test_arr[x][1], c='c',zorder=10) # true
        #print(test_array)
        #print(test_array[x][3],test_array[x][4])

    ax.scatter(test_arr[:,0],test_arr[:,1], c='b',zorder=5) # true

        #fig.canvas.draw()
        #fig.canvas.flush_events()
        #plt.waitforbuttonpress()
    plt.show()


if __name__ == '__main__':
    main()
