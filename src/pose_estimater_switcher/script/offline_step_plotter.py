#!/usr/bin/env python3
import math
from itertools import count
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

beacon_id =   [42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
beacon_x =    [11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560]    #[11700  , 16244 , 7824  , 2000  , 21369 , 26163 , 26163 , 31000 , 35766 , 35766 , 40205 , 40204 , 16560 ]beacon_
beacon_y =    [5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549]    #[5999   , 10150 , 5726  , 4499  , 6534  , 9939  , 3699  , 6519  , 10012  , 10012 , 11684 , 4363  , 3549 ]
beacon_z =    [5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767]

plt.style.use('fivethirtyeight')
plt.ion()
#fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
fig = plt.figure(figsize=(20,10))
fig.suptitle('pose plot test')
ax1 = fig.add_subplot(1,2,1)
ax2 = fig.add_subplot(1,2,2, projection='3d')
path = Path.home().joinpath("test_data", "pose.csv")

index = count()

data = pd.read_csv(path)
df = pd.DataFrame(data)
id = data["ID"]
distance = data["distance"]
x = data["x"]
y = data["y"]
z = data["z"]

ax1.set_title("2d plot")
ax1.set_xlabel("X - I have a bad feeling about this")
ax1.set_ylabel("Y - Hello there")
ax2.set_title("3d plot")
ax2.set_xlabel("X - I have a bad feeling about this")
ax2.set_ylabel("Y - Hello there")
ax2.set_zlabel("z - General kenobi")
    #print('x: ',x,',y: ',y)
for j in range(len(x)):
    index_of_data = beacon_id.index(id[j])
    #ax1.clear()
    #ax2.clear()
    fig.clear()
    fig.suptitle('pose plot test')
    ax1 = fig.add_subplot(1,2,1)
    ax2 = fig.add_subplot(1,2,2, projection='3d')
    ax1.set_xlim(0,45000)
    ax1.set_ylim(0,12000)
    ax2.set_xlim(0,45000)
    ax2.set_ylim(0,12000)
    ax2.set_zlim(0,6000)
    ax1.set_title("2d plot")
    ax1.set_xlabel("X - I have a bad feeling about this")
    ax1.set_ylabel("Y - Hello there")
    ax2.set_title("3d plot")
    ax2.set_xlabel("X - I have a bad feeling about this")
    ax2.set_ylabel("Y - Hello there")
    ax2.set_zlabel("z - General kenobi")
    fig.text(0.03, 0.95,
         "measured dist: " + str(distance[j]),
         style = 'italic',
         fontsize = 12,
         color = "black")
    fig.text(0.25, 0.95,
         "calculated dist: " + str(format(math.sqrt(math.pow(beacon_x[index_of_data]-x[j],2)+math.pow(beacon_y[index_of_data]-y[j],2)+math.pow(beacon_z[index_of_data]-z[j],2)),'.2f')),
         style = 'italic',
         fontsize = 12,
         color = "black")

    ax1.scatter(x[j], y[j], c='b',zorder=10)
    label = '%d' % (beacon_id[index_of_data])
    ax1.text(beacon_x[index_of_data],beacon_y[index_of_data],label, zorder=10)
    ax1.scatter(beacon_x[index_of_data],beacon_y[index_of_data],c='r', zorder=10)
    ax1.scatter(beacon_x[4],beacon_y[4],c='r', zorder=10)

    #angle = np.linspace( 0 , 2 * np.pi , 150 )
    #radius = distance[j]
    #x_ring = radius * np.cos( angle ) + beacon_x[index_of_data]
    #y_ring = radius * np.sin( angle ) + beacon_y[index_of_data]


    #ax1.plot(x_ring, y_ring, zorder=5)
    ax2.scatter(x[j], y[j], z[j], c='b')
    label = '%d' % (beacon_id[index_of_data])
    ax2.text(beacon_x[index_of_data],beacon_y[index_of_data],beacon_z[index_of_data],label)
    ax2.scatter(beacon_x[index_of_data],beacon_y[index_of_data],beacon_z[index_of_data],c='r')

    #for i in range(len(beacon_id)):
        #label = '%d' % (beacon_id[i])
        #ax2.text(beacon_x[i],beacon_y[i],beacon_z[i],label)

    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.waitforbuttonpress()
#plt.tight_layout()
#plt.show()
