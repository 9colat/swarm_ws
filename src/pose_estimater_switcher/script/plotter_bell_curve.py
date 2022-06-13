#!/usr/bin/env python3
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

def bell_function(id,d):
    r_h = 300
    std = 6500
    mean = 0
    beacon_id =   [42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
    beacon_z =    [5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767]
    index_of_data = beacon_id.index(id)
    d_z = beacon_z[index_of_data] - r_h
    x = math.sqrt(pow(d,2)-pow(d_z,2))
    t = pow(x-mean,2)/(2*pow(std,2))
    print(x, t)
    y = (1/(std * math.sqrt(2*math.pi)))*pow(math.e,t)
    return [x, y]

# missing beacons
# id, x, y, z
# 44050, 7825, 9999, 4286
# 44539, 1999, 10677, 3531


plt.style.use('fivethirtyeight')
#fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
fig = plt.figure(figsize=(10,10))
fig.suptitle('pose plot test')
ax1 = fig.add_subplot(1,1,1)
b_id =        [44539, 44050, 42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
beacon_x =    [1999, 7825, 11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560]    #[11700  , 16244 , 7824  , 2000  , 21369 , 26163 , 26163 , 31000 , 35766 , 35766 , 40205 , 40204 , 16560 ]beacon_
beacon_y =    [10677, 9999, 5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549]    #[5999   , 10150 , 5726  , 4499  , 6534  , 9939  , 3699  , 6519  , 10012  , 10012 , 11684 , 4363  , 3549 ]
beacon_z =    [3531, 4286, 5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767]
r_x = 2000
r_y = 6000
r_z = 300
for j in range(len(b_id)):
    ax1.clear()
    ax1.set_title("2d plot")
    ax1.set_xlabel("X - I have a bad feeling about this")
    ax1.set_ylabel("Y - Hello there")
    dist = math.sqrt(pow(beacon_x[j] - r_x,2) + pow(beacon_y[j] - r_y,2)+pow(beacon_z[j] - r_z,2))
    [radius, y] = bell_function(b_id[j], dist)
    angle = np.linspace( 0 , 2 * np.pi , 150 )
    x_ring = radius * np.cos( angle ) + beacon_x[j]
    y_ring = radius * np.sin( angle ) + beacon_y[j]


    ax1.plot(x_ring, y_ring, zorder=5, c='r')
    ax1.scatter(r_x, r_y, zorder=10, c='b')
    ax1.scatter(beacon_x[j], beacon_y[j], zorder=10, c='r')


    ax1.set_xlim(0,45000)
    ax1.set_ylim(0,12000)

    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.waitforbuttonpress()
