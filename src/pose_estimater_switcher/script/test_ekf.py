import matplotlib.pyplot as plt
import random
import numpy as np
import math
import rospy
from custom_msgs.msg import odom_and_imu
from custom_msgs.msg import USPS_msgs
from ekf_usps import EKF #this is our thing huhu


w1 = EKF()

# constants
rotated_matrix = np.array([[0.0, -1.0], [1.0, 0.0]])
mag_x_calibrated = 0.0
mag_y_calibrated = 0.0
pi = 3.141593


# for plotting
fig = plt.figure(figsize = (10, 10))
fig.suptitle('Kalman Filter Test')
ax = fig.add_subplot(1, 1, 1) # one row, 1 column, which you want to interact with

def rot_mat(theta):
    output = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    return output



def main():
    theta = 0.1
    n = 1600
    test_array = [0.0]*n
    head = np.array([[1.0, 0.0]]).T
    new_position = np.array([[1.0, 1.0]]).T
    dT = 0.01
    velocity = 5
    omega = 0.1
    arr = [42867, 42928,  42929]
    test_arr = np.array([[1.0]*2]*n)
    mag_measurement = np.array([[0.0, 0.0]])

    R_matrix = np.dot(rot_mat(3*math.pi/180), np.array([[1, 0]]).T)





    for x in range(n):

        #id = w1.id.index(arr[x%3])
        #new_position = new_position + head * velocity * dT
        #dist = math.sqrt(pow(w1.x[id] - new_position[0], 2) + pow(w1.y[id] - new_position[1], 2)) #+ random.uniform(-1, 1)

        head = head + np.dot(rotated_matrix, head) * omega * dT # true heading
        mag_angle = np.array(np.dot(theta, np.array([[1, 0]]).T))
        mag_measurement = np.array([[float(np.dot(R_matrix.T, head))], [float((np.dot(np.dot(R_matrix.T, rotated_matrix), head)))]])
        #+ random.uniform(-0.0001, 0.0001)




        #test_array[x] = (w1.beacon_measurement_updater_EKF(arr[x%3], dist + random.uniform(-0.1, 0.1), dT))
        test_array[x] = w1.magnetometer_measurement_updater_EKF(mag_measurement, dT)
        #test_arr[x] = new_position.T
        test_arr[x] = head.T
        ax.scatter(test_array[x][3],test_array[x][4],c='r') #predicted
        #ax.scatter(test_array[x][0],test_array[x][1],c='r') #predicted

        print(test_array[x])

    ax.scatter(test_arr[:,0],test_arr[:,1], c='b') # true

    plt.show()


if __name__ == '__main__':
    main()
