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



def main():
    n = 2000
    test_array = [0.0]*n
    head = np.array([[0.1, 0.0]]).T
    test_head = np.array([[0.1, 0.0]]).T
    new_position = np.array([[1.0, 1.0]]).T
    dT = 0.01
    velocity = 5
    arr = [42867, 42928,  42929]
    test_arr = np.array([[1.0]*2]*n)
    old_angle = 0.0

    for x in range(n):
        id = w1.id.index(arr[x%3])


        new_position = new_position + head * velocity * dT
        head = head + np.dot(rotated_matrix, head) * w1.imu_gyro[2] * dT
        true_angle = math.atan2(head[1], head[0])
        varied_angle = true_angle + random.uniform(-0.01, 0.01)

        test_head[0] = math.cos(varied_angle)
        test_head[1] = math.sin(varied_angle)

        dist = math.sqrt(pow(w1.x[id] - new_position[0], 2) + pow(w1.y[id] - new_position[1], 2)) #+ random.uniform(-1, 1)
        test_array[x] = (w1.beacon_measurement_updater_EKF(arr[x%3], dist + random.uniform(-0.1, 0.1), dT))
        #test_array[x] = w1.magnetometer_measurement_updater_EKF(test_head, dT)
        test_arr[x] = new_position.T
        #test_arr[x] = head.T
        #ax.scatter(test_array[x][3],test_array[x][4],c='r') #predicted
        ax.scatter(test_array[x][0],test_array[x][1],c='r') #predicted
        print(test_array[x])

    ax.scatter(test_arr[:,0],test_arr[:,1], c='b') # true

    plt.show()


if __name__ == '__main__':
    main()
