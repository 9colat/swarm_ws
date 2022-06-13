import matplotlib.pyplot as plt
import numpy as np
import math

# missing beacons
# id, x, y, z
# 44050, 7825, 9999, 4286
# 44539, 1999, 10677, 3531

class Pose_Calculator:
    def __init__(self, x=0, y=0, z=0):
        self.beacon_id = [44539, 44050, 42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
        self.beacon =    np.array([[1999, 7825, 11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560],[10677, 9999, 5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549],[3531, 4286, 5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767]])
        self.est_pose = np.array([x,y,z])


    def pose_estimator(self, id, meas):
        index_of_data = self.beacon_id.index(id)
        diff = self.est_pose-self.beacon[:,index_of_data]
        length = math.dist(self.est_pose, self.beacon[:,index_of_data])
        diff = np.divide(diff, length)
        self.est_pose = diff * meas + self.beacon[:,index_of_data]


        #print(self.est_pose)
        return self.est_pose
