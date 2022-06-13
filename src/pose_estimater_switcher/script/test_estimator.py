import matplotlib.pyplot as plt
import numpy as np
import math

# missing beacons
# id, x, y, z
# 44050, 7825, 9999, 4286
# 44539, 1999, 10677, 3531


beacon =    np.array([[11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560],[5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549],[5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767]])
know_pose = np.array([20,0,2])
est_pose = np.array([0,0,0])

for i in range(1000):
    for j in range(len(beacon[0,:])):
        meas = math.dist(beacon[:,j],know_pose)
        #print(meas)
        diff = est_pose-beacon[:,j]
        l = math.dist(est_pose, beacon[:,j])
        diff = np.divide(diff, l)
        est_pose = diff * meas + beacon[:,j]

        print(est_pose)
