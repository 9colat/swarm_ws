import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time
import sys
import os


fig = plt.figure(figsize=(6,6))

ax = fig.add_subplot(111)#, projection='3d')

class USPS_data:
    def __init__(self):
        self.id =   [42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
        self.x =    [11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560]    #[11700  , 16244 , 7824  , 2000  , 21369 , 26163 , 26163 , 31000 , 35766 , 35766 , 40205 , 40204 , 16560 ]
        self.y =    [5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549]    #[5999   , 10150 , 5726  , 4499  , 6534  , 9939  , 3699  , 6519  , 10012  , 10012 , 11684 , 4363  , 3549 ]
        self.z =    [5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767]    #[5577   , 5577  , 4286  , 3530  , 5578  , 5577  , 5577  , 5578  , 5578  , 5578  , 3767  , 3767  , 5577  ]



w1 = USPS_data()

def plotter(x, y, z):
    print("im in the function")
    global w1
    ax.clear()
    ax.set_title("Potentiometer Reading Live Plot")
    ax.set_xlabel("X - I have a bad feeling about this")
    ax.set_ylabel("Y - Hello there")
    ax.set_zlabel("z - General kenobi")

    for i in range(len(w1.id)):
        ax.scatter(w1.x[i],w1.y[i],w1.z[i],c="red")
    ax.scatter(x,y,z,c="blue")
    plt.show()


path_test = [(0.0707, 0.0702), (0.141, 0.139), (0.212, 0.208), (0.283, 0.275), (0.354, 0.341), (0.424, 0.407), (0.495, 0.471), (0.566, 0.534), (0.636, 0.597), (0.707, 0.658), (0.778, 0.719), (0.849, 0.778), (0.919, 0.836), (0.99, 0.894), (1.06, 0.95), (1.13, 1.01), (1.2, 1.06), (1.27, 1.11), (1.34, 1.17), (1.41, 1.22), (1.48, 1.27), (1.56, 1.32), (1.63, 1.37), (1.7, 1.41), (1.77, 1.46), (1.84, 1.51), (1.91, 1.55), (1.98, 1.6), (2.05, 1.64), (2.12, 1.68), (2.19, 1.72), (2.26, 1.76), (2.33, 1.8), (2.4, 1.84), (2.47, 1.87), (2.55, 1.91), (2.62, 1.95), (2.69, 1.98), (2.76, 2.01), (2.83, 2.04), (2.9, 2.08), (2.97, 2.11), (3.04, 2.13), (3.11, 2.16), (3.18, 2.19), (3.25, 2.22), (3.32, 2.24), (3.39, 2.27), (3.46, 2.29), (3.54, 2.31), (3.61, 2.33), (3.68, 2.35), (3.75, 2.37), (3.82, 2.39), (3.89, 2.41), (3.96, 2.42), (4.03, 2.44), (4.1, 2.45), (4.17, 2.47), (4.24, 2.48), (4.31, 2.49), (4.38, 2.5), (4.45, 2.51), (4.53, 2.52), (4.6, 2.53), (4.67, 2.53), (4.74, 2.54), (4.81, 2.54), (4.88, 2.55), (4.95, 2.55), (5.02, 2.55), (5.09, 2.55), (5.16, 2.55), (5.23, 2.55), (5.3, 2.55), (5.37, 2.54), (5.44, 2.54), (5.52, 2.53), (5.59, 2.53), (5.66, 2.52), (5.73, 2.51), (5.8, 2.5), (5.87, 2.49), (5.94, 2.48), (6.01, 2.47), (6.08, 2.46), (6.15, 2.44), (6.22, 2.43), (6.29, 2.41), (6.36, 2.39), (6.43, 2.38), (6.51, 2.36), (6.58, 2.34), (6.65, 2.32), (6.72, 2.3), (6.79, 2.27), (6.86, 2.25), (6.93, 2.22), (7.0, 2.2), (7.07, 2.17), (7.14, 2.14), (7.21, 2.11), (7.28, 2.08), (7.35, 2.05), (7.42, 2.02), (7.5, 1.99), (7.57, 1.96), (7.64, 1.92), (7.71, 1.89), (7.78, 1.85), (7.85, 1.81), (7.92, 1.77), (7.99, 1.73), (8.06, 1.69), (8.13, 1.65), (8.2, 1.61), (8.27, 1.57), (8.34, 1.52), (8.41, 1.48), (8.49, 1.43), (8.56, 1.38), (8.63, 1.33), (8.7, 1.28), (8.77, 1.23), (8.84, 1.18), (8.91, 1.13), (8.98, 1.08), (9.05, 1.02), (9.12, 0.968), (9.19, 0.911), (9.26, 0.854), (9.33, 0.796), (9.4, 0.737), (9.48, 0.677), (9.55, 0.616), (9.62, 0.554), (9.69, 0.491), (9.76, 0.427), (9.83, 0.361), (9.9, 0.295), (9.97, 0.229), (10.0, 0.161), (10.1, 0.0916), (10.2, 0.0217)]

def anime(path):
    X = []
    Y = []

    for item in path:
        X.append(item[0])
        Y.append(item[1])

    plt.ion() # turn interactive mode on
    animated_plot = plt.scatter(X, Y, 'ro')[0]

    for i in range(len(X)):
        animated_plot.set_xdata(X[0:i])
        animated_plot.set_ydata(Y[0:i])
        plt.draw()
        plt.pause(0.1)




plt.style.use('fivethirtyeight')

x_values = []
y_values = []

#index = count()


def animate(data):
    x_values = data[0]
    y_values = data[1]
    ax.set_title("Potentiometer Reading Live Plot")
    ax.set_xlabel("X - I have a bad feeling about this")
    ax.set_ylabel("Y - Hello there")
    #ax.set_zlabel("z - General kenobi")
    for i in range(len(w1.id)):
        ax.scatter(w1.x[i],w1.y[i],c="red") #w1.z[i],

    plt.cla()
    plt.plot(x_values, y_values)
    plt.gcf().autofmt_xdate()
    plt.tight_layout()





def pose_estimator():
    print("main loop")
    global w1

    x = input("x: ")
    y = input("y: ")
    x_and_y = [int(x),int(y)]
    #ani = FuncAnimation(plt.gcf(), animate, fargs=x_and_y)
    animate(x_and_y)



    #plt.tight_layout()
    plt.show()
    #while True:
        #anime(path_test)
    #for i in range(100):
        #(1,1,1)

    #time.sleep(0.01)


if __name__ == '__main__':
    pose_estimator()
