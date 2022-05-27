#!/usr/bin/env python3
##### import of libs #####
import rospy
import os
import math
import csv
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

##### setting up the path for the data file and folder #####
path_NLoS = str(Path.home().joinpath("test_data/1_filtered_data", "filtered_NLoS_dyn_log%s.csv"))
path_LoS = str(Path.home().joinpath("test_data/1_filtered_data", "filtered_LoS_dyn_log%s.csv"))
path_output = str(Path.home().joinpath("test_data","1_filtered_data"))
folder_path = str(Path.home().joinpath("test_data/1_filtered_data"))
output_path = str(Path.home()) + '/' + "figure/1_filtered_data/"
fontdict_me={'size': 35}
plt.style.use('fivethirtyeight')
fig1 = plt.figure(figsize=(30,30))
fig1.suptitle('Position plot',fontsize=50)
ax1 = fig1.add_subplot(2,2,1)
ax2 = fig1.add_subplot(2,2,2)
ax3 = fig1.add_subplot(2,2,3)
ax4 = fig1.add_subplot(2,2,4)

isfolder = os.path.isdir(output_path)



def main():
    if not isfolder:
        os.mkdir(output_path)
        print("making directory")
    global path, folder_path
    number_of_files = 0
    output_file_name = 'NLoS_dyn_outputdata.csv'
    path_out = str(Path.home().joinpath("test_data/1_filtered_data", output_file_name))

    path_output_file = Path(path_out)
    print(path_output_file.is_file())
    if path_output_file.is_file():
        os.remove(path_out)
    true_x = [22653, 22673]
    true_y = [1039, 3822]


    while os.path.exists(path_NLoS % number_of_files):
        print(number_of_files)
        local_path_LoS = path_NLoS %  number_of_files
        local_path_NLoS = path_LoS %  number_of_files
        data_LoS = pd.read_csv(local_path_LoS)
        data_NLoS = pd.read_csv(local_path_NLoS)

        LoS_with_x = data_LoS["with_x"]
        LoS_with_y = data_LoS["with_y"]
        LoS_without_x = data_LoS["without_x"]
        LoS_without_y = data_LoS["without_y"]

        NLoS_with_x = data_NLoS["with_x"]
        NLoS_with_y = data_NLoS["with_y"]
        NLoS_without_x = data_NLoS["without_x"]
        NLoS_without_y = data_NLoS["without_y"]


        ax1.set_title("LoS",fontdict=fontdict_me)
        ax1.set_xlabel("X - coordinate [mm]", fontsize=30)
        ax1.set_ylabel("Y - coordinate [mm]", fontsize=30)
        ax1.text(0.05, 0.95,"Kalman",transform=ax1.transAxes, fontsize=35,verticalalignment='top')
        ax2.set_title("NLoS",fontdict=fontdict_me)
        ax2.set_xlabel("X - coordinate [mm]", fontsize=30)
        ax2.set_ylabel("Y - coordinate [mm]", fontsize=30)
        ax2.text(0.05, 0.95,"Kalman",transform=ax2.transAxes, fontsize=35,verticalalignment='top')
        ax3.set_title("LoS",fontdict=fontdict_me)
        ax3.set_xlabel("X - coordinate [mm]", fontsize=30)
        ax3.set_ylabel("Y - coordinate [mm]", fontsize=30)
        ax3.text(0.05, 0.95,"Lidar",transform=ax3.transAxes, fontsize=35,verticalalignment='top')
        ax4.set_title("NLoS",fontdict=fontdict_me)
        ax4.set_xlabel("X - coordinate [mm]", fontsize=30)
        ax4.set_ylabel("Y - coordinate [mm]", fontsize=30)
        ax4.text(0.05, 0.95,"Lidar",transform=ax4.transAxes, fontsize=35,verticalalignment='top')


        ax1.scatter(LoS_without_x, LoS_without_y, c='b')
        ax2.scatter(NLoS_without_x, NLoS_without_y, c='b')
        ax3.scatter(LoS_with_x, LoS_with_y, c='b')
        ax4.scatter(NLoS_with_x, NLoS_with_y, c='b')

        number_of_files += 1
    ax1.scatter(LoS_without_x, LoS_without_y, c='b', label='LoS kalman')
    ax1.plot(true_x, true_y, '-o', c='r', label='True path', linewidth=10)
    ax2.scatter(NLoS_without_x, NLoS_without_y, c='b', label='NLoS kalman')
    ax2.plot(true_x, true_y, '-o', c='r', label='True path', linewidth=10)
    ax3.scatter(LoS_with_x, LoS_with_y, c='b', label='LoS Lidar')
    ax3.plot(true_x, true_y, '-o', c='r', label='True path', linewidth=10)
    ax4.scatter(NLoS_with_x, NLoS_with_y, c='b', label='NLoS Lidar')
    ax4.plot(true_x, true_y, '-o', c='r', label='True path', linewidth=10)
    #ax1.legend(fontsize=20)
    ax2.legend(fontsize=30)
    #ax3.legend(fontsize=20)
    #ax4.legend(fontsize=20)

    ax1.set_xlim(20000,25500)
    ax1.set_ylim(0,6000)
    ax2.set_xlim(20000,25500)
    ax2.set_ylim(0,6000)
    ax3.set_xlim(20000,25500)
    ax3.set_ylim(0,6000)
    ax4.set_xlim(20000,25500)
    ax4.set_ylim(0,6000)
    name_of_file_1 = 'grit_pose_plot.png'


    fig1.savefig(output_path + name_of_file_1)




    print("im done baby ;)")



if __name__ == '__main__':
    main()
