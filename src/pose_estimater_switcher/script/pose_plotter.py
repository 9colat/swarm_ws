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
    x = data["x - coordinate"]
    y = data["y - coordinate"]
    z = data["z - coordinate"]



    ax1.cla()
    ax2.cla()
    ax1.scatter(x, y)
    ax2.scatter(beacon_x,beacon_y,beacon_z,c='r')
    ax2.scatter(x, y, z, c='b')
    for i, txt in enumerate(beacon_id):
        ax2.annotate(txt,(beacon_x[i],beacon_y[i],beacon_z[i]))
    #ax.tight_layout()

ani = FuncAnimation(plt.gcf(), animate, interval=1000)

plt.tight_layout()
plt.show()
