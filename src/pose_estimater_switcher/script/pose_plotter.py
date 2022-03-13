#!/usr/bin/env python3
import random
from itertools import count
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

beacon_id =   [42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
beacon_x =    [11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560]    #[11700  , 16244 , 7824  , 2000  , 21369 , 26163 , 26163 , 31000 , 35766 , 35766 , 40205 , 40204 , 16560 ]beacon_
beacon_y =    [5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549]    #[5999   , 10150 , 5726  , 4499  , 6534  , 9939  , 3699  , 6519  , 10012  , 10012 , 11684 , 4363  , 3549 ]
beacon_z =    [5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767]

plt.style.use('fivethirtyeight')
#fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
fig = plt.figure(figsize=(20,10))
fig.suptitle('pose plot test')
ax1 = fig.add_subplot(1,2,1)
ax2 = fig.add_subplot(1,2,2, projection='3d')
path = Path.home().joinpath("test_data", "pose.csv")
x_vals = []
y_vals = []

index = count()

def animate(i):
    data = pd.read_csv(path)
    x = data["x"]
    y = data["y"]
    z = data["z"]

    ax1.clear()
    ax2.clear()

    ax1.set_title("2d plot")
    ax1.set_xlabel("X - I have a bad feeling about this")
    ax1.set_ylabel("Y - Hello there")
    ax2.set_title("3d plot")
    ax2.set_xlabel("X - I have a bad feeling about this")
    ax2.set_ylabel("Y - Hello there")
    ax2.set_zlabel("z - General kenobi")
    #print('x: ',x,',y: ',y)
    ax1.scatter(x, y)
    ax2.scatter(beacon_x,beacon_y,beacon_z,c='r')
    ax2.scatter(x, y, z, c='b')
    for i in range(len(beacon_id)):
        label = '%d' % (beacon_id[i])
        ax2.text(beacon_x[i],beacon_y[i],beacon_z[i],label)
        #ax2.annotate(,())
    ax1.set_xlim(0,45000)
    ax1.set_ylim(0,12000)
    ax2.set_xlim(0,45000)
    ax2.set_ylim(0,12000)
    ax2.set_zlim(0,6000)
    #ax.tight_layout()

ani = FuncAnimation(plt.gcf(), animate, interval=500)

#plt.tight_layout()
plt.show()
