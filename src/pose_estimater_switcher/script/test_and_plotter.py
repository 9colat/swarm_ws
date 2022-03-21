import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
import random
from estimator_function import Pose_Calculator

know_pose = np.array([20,0,2])
ID = [42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
beacon = np.array([[11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560],[5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549],[5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767]])
PC = Pose_Calculator(-1,-1,-1)
fig = plt.figure(figsize=(20,10))
fig.suptitle('pose plot test')
ax1 = fig.add_subplot(1,2,1)
ax2 = fig.add_subplot(1,2,2, projection='3d')
pose_est = []
pose_est_x = []
pose_est_y = []
pose_est_z = []

for i in range(50):
    for j in range(len(ID)):
        meas = math.dist(beacon[:,j],know_pose) + random.randint(-1,1)
        pose_est = PC.pose_estimator(ID[j], meas)
        pose_est_x = np.append(pose_est_x, pose_est[0])
        pose_est_y = np.append(pose_est_y, pose_est[1])
        pose_est_z = np.append(pose_est_z, pose_est[2])
print(pose_est)

ax1.scatter(pose_est_x, pose_est_y)
ax2.scatter(pose_est_x, pose_est_y, pose_est_z, c='r')
plt.show()
