#!/usr/bin/env python3
##### import of libs #####
import rospy
import os
import math
import csv
import scipy.stats
from pathlib import Path
import pandas as pd
import numpy as np

##### setting up the path for the data file and folder #####
path_NLoS = str(Path.home().joinpath("test_data", "NLoS_dyn_log%s.csv"))
path_LoS = str(Path.home().joinpath("test_data", "LoS_dyn_log%s.csv"))
path_output = str(Path.home().joinpath("test_data"))
output_path = str(Path.home()) + '/' + "figure/1_new_data/"

isfolder = os.path.isdir(output_path)



# missing beacons
# id, x, y, z
# 44050, 7825, 9999, 4286
# 44539, 1999, 10677, 3531




def main():
    if not isfolder:
        os.mkdir(output_path)
        #print("making directory")
    beacon_id =   [44539, 44050, 42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
    beacon_x =    [1999, 7825, 11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560]    #[11700  , 16244 , 7824  , 2000  , 21369 , 26163 , 26163 , 31000 , 35766 , 35766 , 40205 , 40204 , 16560 ]beacon_
    beacon_y =    [10677, 9999, 5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549]    #[5999   , 10150 , 5726  , 4499  , 6534  , 9939  , 3699  , 6519  , 10012  , 10012 , 11684 , 4363  , 3549 ]
    beacon_z =    [3531, 4286, 5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767]
    global path
    number_of_files = 21
    output_file_name = 't_test.csv'
    path_out = str(Path.home().joinpath("test_data/", output_file_name))

    path_output_file = Path(path_out)
    if path_output_file.is_file():
        os.remove(path_out)
    true_x = [22653, 22673]
    true_y = [1039, 3822]
    per_x = [22000, 100]
    per_y = [158.10276679, 0.7186489399]
    print(os.path.exists(path_NLoS % number_of_files))
    print(path_NLoS % number_of_files)

    while os.path.exists(path_NLoS % number_of_files):
    #for number_of_files in range(0,5):
        print(number_of_files)
        t_1 = 0
        t_2 = 0
        t_3 = 0
        t_4 = 0

        local_path_LoS = path_LoS %  number_of_files
        local_path_NLoS = path_NLoS %  number_of_files
        #local_path_LoS_lidar = path_LoS_lidar %  number_of_files
        #local_path_NLoS_lidar = path_NLoS_lidar %  number_of_files
        #local_path_lidar = path_LoS_lidar %  number_of_files
        LoS_data = pd.read_csv(local_path_LoS)
        NLoS_data = pd.read_csv(local_path_NLoS)
        #data_lidar_LoS = pd.read_csv(local_path_LoS_lidar)
        #print(local_path_NLoS_lidar)
        #data_lidar_NLoS = pd.read_csv(local_path_NLoS_lidar)


        #### ---- LoS ---- ####
        LoS_slam_x_time = [0] * (len(LoS_data["with_x"])-101)
        LoS_slam_y_time = [0] * (len(LoS_data["with_x"])-101)
        LoS_Xsg_x = [0] * (len(LoS_data["with_x"])-101)
        LoS_Xsg_y = [0] * (len(LoS_data["with_x"])-101)
        LoS_error_slam_kalman_x = [0] * (len(LoS_data["with_x"])-101)
        LoS_error_slam_kalman_y = [0] * (len(LoS_data["with_x"])-101)
        LoS_error_slam_lidar_x = [0] * (len(LoS_data["with_x"])-101)
        LoS_error_slam_lidar_y = [0] * (len(LoS_data["with_x"])-101)
        LoS_error_slam_kalman = [0] * (len(LoS_data["with_x"])-101)
        LoS_error_slam_lidar = [0] * (len(LoS_data["with_x"])-101)
        LoS_slam_x_time_tans = [0] * (len(LoS_data["with_x"])-101)
        LoS_slam_y_time_tans = [0] * (len(LoS_data["with_x"])-101)
        LoS_time_slam = np.linspace(1, len(LoS_data["with_x"]-101/10), num=(len(LoS_data["with_x"])-101))
        LoS_lidar_kalman_slam_x_time = [0] * (len(LoS_data["with_x"])-101)
        LoS_lidar_kalman_slam_y_time = [0] * (len(LoS_data["with_x"])-101)
        LoS_kalman_slam_x_time = [0] * (len(LoS_data["with_x"])-101)
        LoS_kalman_slam_y_time = [0] * (len(LoS_data["with_x"])-101)
        LoS_kalman_x_bank_slam = [0] * (len(LoS_data["with_x"])-101)
        LoS_kalman_y_bank_slam = [0] * (len(LoS_data["with_x"])-101)
        LoS_simple_x_data_slam = [0] * (len(LoS_data["with_x"])-101)
        LoS_simple_y_data_slam = [0] * (len(LoS_data["with_x"])-101)
        LoS_slam_delta_l = [0] * (len(LoS_data["with_x"])-102)
        LoS_kalman_delta_l = [0] * (len(LoS_data["with_x"])-102)
        LoS_lidar_delta_l = [0] * (len(LoS_data["with_x"])-102)



        #### ---- NLoS ---- ####
        NLoS_slam_x_time = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_slam_y_time = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_Xsg_x = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_Xsg_y = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_error_slam_kalman_x = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_error_slam_kalman_y = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_error_slam_lidar_x = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_error_slam_lidar_y = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_error_slam_kalman = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_error_slam_lidar = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_slam_x_time_tans = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_slam_y_time_tans = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_time_slam = np.linspace(1, len(NLoS_data["with_x"]-101/10), num=(len(NLoS_data["with_x"])-101))
        NLoS_lidar_kalman_slam_x_time = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_lidar_kalman_slam_y_time = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_kalman_slam_x_time = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_kalman_slam_y_time = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_kalman_x_bank_slam = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_kalman_y_bank_slam = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_simple_x_data_slam = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_simple_y_data_slam = [0] * (len(NLoS_data["with_x"])-101)
        NLoS_slam_delta_l = [0] * (len(NLoS_data["with_x"])-102)
        NLoS_kalman_delta_l = [0] * (len(NLoS_data["with_x"])-102)
        NLoS_lidar_delta_l = [0] * (len(NLoS_data["with_x"])-102)


        LoS_dist_s_delta_kalman = 0
        NLoS_dist_s_delta_kalman = 0
        LoS_dist_s_abs_kalman = 0
        NLoS_dist_s_abs_kalman = 0
        LoS_dist_s_abs_x_kalman = 0
        NLoS_dist_s_abs_x_kalman = 0
        LoS_dist_s_abs_y_kalman = 0
        NLoS_dist_s_abs_y_kalman = 0
        LoS_dist_s_delta_lidar = 0
        NLoS_dist_s_delta_lidar = 0
        LoS_dist_s_abs_lidar = 0
        NLoS_dist_s_abs_lidar = 0
        LoS_dist_s_abs_x_lidar = 0
        NLoS_dist_s_abs_x_lidar = 0
        LoS_dist_s_abs_y_lidar = 0
        NLoS_dist_s_abs_y_lidar = 0
        LoS_dist_s_bank = 0
        NLoS_dist_s_bank = 0


        #### ---- LoS ---- ####
        LoS_with_x = LoS_data["with_x"]
        LoS_with_y = LoS_data["with_y"]
        LoS_without_x = LoS_data["without_x"]
        LoS_without_y = LoS_data["without_y"]
        LoS_simple_x = LoS_data["simple_x"]
        LoS_simple_y = LoS_data["simple_y"]
        LoS_multi_x = LoS_data["muti_kalman_x"]
        LoS_multi_y = LoS_data["muti_kalman_y"]
        LoS_slam_x = LoS_data["slam_x"]
        LoS_slam_y = LoS_data["slam_y"]


        for i in range(len(LoS_lidar_kalman_slam_x_time)):
            if i > 100:
                LoS_lidar_kalman_slam_x_time[i-101] = LoS_with_x[i]
                LoS_lidar_kalman_slam_y_time[i-101] = LoS_with_y[i]
                LoS_kalman_slam_x_time[i-101] = LoS_without_x[i]
                LoS_kalman_slam_y_time[i-101] = LoS_without_y[i]

                LoS_kalman_x_bank_slam[i-101] = LoS_multi_x[i]
                LoS_kalman_y_bank_slam[i-101] = LoS_multi_y[i]
                LoS_simple_x_data_slam[i-101] = LoS_simple_x[i]
                LoS_simple_y_data_slam[i-101] = LoS_simple_y[i]

                LoS_slam_x_time[i-101] =  LoS_slam_x[i]
                LoS_slam_y_time[i-101] =  LoS_slam_y[i]

        LoS_sx = np.array([LoS_slam_x_time]).T
        LoS_sy = np.array([LoS_slam_y_time]).T
        LoS_gx = np.array([LoS_kalman_slam_x_time]).T
        LoS_gy = np.array([LoS_kalman_slam_y_time]).T





        LoS_A = [[float(np.dot(LoS_sx.T,LoS_sx)), float(np.dot(LoS_sx.T,LoS_sy)), float(np.sum(LoS_sx))],
            [float(np.dot(LoS_sx.T,LoS_sy)), float(np.dot(LoS_sy.T,LoS_sy)), float(np.sum(LoS_sy))],
            [float(np.sum(LoS_sy)), float(np.sum(LoS_sy)), float(len(LoS_sx))]]
        LoS_B1 = [[float(np.dot(LoS_gx.T, LoS_sx))],[float(np.dot(LoS_gx.T,LoS_sy))],[float(np.sum(LoS_gx))]]
        LoS_B2 = [[float(np.dot(LoS_gy.T, LoS_sx))],[float(np.dot(LoS_gy.T,LoS_sy))],[float(np.sum(LoS_gy))]]

        LoS_X1 = np.linalg.solve(LoS_A, LoS_B1)
        LoS_X2 = np.linalg.solve(LoS_A, LoS_B2)

        LoS_AT = [[LoS_X1[0][0], LoS_X1[1][0]],[LoS_X2[0][0], LoS_X2[1][0]]]
        LoS_BT = [[LoS_X1[2][0]],[LoS_X2[2][0]]]

        for h in range(len(LoS_sy)):
            LoS_Xsg = np.dot(LoS_AT,[[float(LoS_sx[h][0])],[float(LoS_sy[h][0])]])+LoS_BT
            LoS_Xsg_x[h] = LoS_Xsg[0][0]
            LoS_Xsg_y[h] = LoS_Xsg[1][0]
            LoS_error_slam_kalman_x[h] = LoS_kalman_slam_x_time[h] - LoS_Xsg_x[h]
            LoS_error_slam_kalman_y[h] = LoS_kalman_slam_y_time[h] - LoS_Xsg_y[h]
            LoS_error_slam_lidar_x[h] = LoS_lidar_kalman_slam_x_time[h] - LoS_Xsg_x[h]
            LoS_error_slam_lidar_y[h] = LoS_lidar_kalman_slam_y_time[h] - LoS_Xsg_y[h]

            LoS_error_slam_kalman[h] = math.sqrt(pow(LoS_error_slam_kalman_x[h],2)+pow(LoS_error_slam_kalman_y[h],2))
            LoS_error_slam_lidar[h] = math.sqrt(pow(LoS_error_slam_lidar_x[h],2)+pow(LoS_error_slam_lidar_y[h],2))

        for k in range(len(LoS_slam_x_time)-1):
            LoS_slam_delta = math.sqrt(pow(LoS_Xsg_x[k+1]-LoS_Xsg_x[k],2)+pow(LoS_Xsg_y[k+1]-LoS_Xsg_y[k],2))
            LoS_lidar_delta = math.sqrt(pow(LoS_lidar_kalman_slam_x_time[k+1]-LoS_lidar_kalman_slam_x_time[k],2)+pow(LoS_lidar_kalman_slam_y_time[k+1]-LoS_lidar_kalman_slam_y_time[k],2))
            LoS_kalman_delta = math.sqrt(pow(LoS_kalman_slam_x_time[k+1]-LoS_kalman_slam_x_time[k],2)+pow(LoS_kalman_slam_y_time[k+1]-LoS_kalman_slam_y_time[k],2))

            LoS_lidar_delta_l[k] = LoS_lidar_delta - LoS_slam_delta
            LoS_kalman_delta_l[k] = LoS_kalman_delta - LoS_slam_delta





        #### ---- NLoS ---- ####
        NLoS_with_x = NLoS_data["with_x"]
        NLoS_with_y = NLoS_data["with_y"]
        NLoS_without_x = NLoS_data["without_x"]
        NLoS_without_y = NLoS_data["without_y"]
        NLoS_simple_x = NLoS_data["simple_x"]
        NLoS_simple_y = NLoS_data["simple_y"]
        NLoS_multi_x = NLoS_data["muti_kalman_x"]
        NLoS_multi_y = NLoS_data["muti_kalman_y"]
        NLoS_slam_x = NLoS_data["slam_x"]
        NLoS_slam_y = NLoS_data["slam_y"]


        for s in range(len(NLoS_lidar_kalman_slam_x_time)):
            if s > 100:
                NLoS_lidar_kalman_slam_x_time[s-101] = NLoS_with_x[s]
                NLoS_lidar_kalman_slam_y_time[s-101] = NLoS_with_y[s]
                NLoS_kalman_slam_x_time[s-101] = NLoS_without_x[s]
                NLoS_kalman_slam_y_time[s-101] = NLoS_without_y[s]

                NLoS_kalman_x_bank_slam[s-101] = NLoS_multi_x[s]
                NLoS_kalman_y_bank_slam[s-101] = NLoS_multi_y[s]
                NLoS_simple_x_data_slam[s-101] = NLoS_simple_x[s]
                NLoS_simple_y_data_slam[s-101] = NLoS_simple_y[s]

                NLoS_slam_x_time[i-101] =  NLoS_slam_x[i]
                NLoS_slam_y_time[i-101] =  NLoS_slam_y[i]








        NLoS_sx = np.array([NLoS_slam_x_time]).T
        NLoS_sy = np.array([NLoS_slam_y_time]).T
        NLoS_gx = np.array([NLoS_kalman_slam_x_time]).T
        NLoS_gy = np.array([NLoS_kalman_slam_y_time]).T





        NLoS_A = [[float(np.dot(NLoS_sx.T,NLoS_sx)), float(np.dot(NLoS_sx.T,NLoS_sy)), float(np.sum(NLoS_sx))],
            [float(np.dot(NLoS_sx.T,NLoS_sy)), float(np.dot(NLoS_sy.T,NLoS_sy)), float(np.sum(NLoS_sy))],
            [float(np.sum(NLoS_sy)), float(np.sum(NLoS_sy)), float(len(NLoS_sx))]]
        NLoS_B1 = [[float(np.dot(NLoS_gx.T, NLoS_sx))],[float(np.dot(NLoS_gx.T,NLoS_sy))],[float(np.sum(NLoS_gx))]]
        NLoS_B2 = [[float(np.dot(NLoS_gy.T, NLoS_sx))],[float(np.dot(NLoS_gy.T,NLoS_sy))],[float(np.sum(NLoS_gy))]]

        print(NLoS_A, NLoS_B1)

        NLoS_X1 = np.linalg.solve(NLoS_A, NLoS_B1)
        NLoS_X2 = np.linalg.solve(NLoS_A, NLoS_B2)

        NLoS_AT = [[NLoS_X1[0][0], NLoS_X1[1][0]],[NLoS_X2[0][0], NLoS_X2[1][0]]]
        NLoS_BT = [[NLoS_X1[2][0]],[NLoS_X2[2][0]]]

        for c in range(len(NLoS_sy)):
            NLoS_Xsg = np.dot(NLoS_AT,[[float(NLoS_sx[c][0])],[float(NLoS_sy[c][0])]])+NLoS_BT
            NLoS_Xsg_x[c] = NLoS_Xsg[0][0]
            NLoS_Xsg_y[c] = NLoS_Xsg[1][0]
            NLoS_error_slam_kalman_x[c] = NLoS_kalman_slam_x_time[c] - NLoS_Xsg_x[c]
            NLoS_error_slam_kalman_y[c] = NLoS_kalman_slam_y_time[c] - NLoS_Xsg_y[c]
            NLoS_error_slam_lidar_x[c] = NLoS_lidar_kalman_slam_x_time[c] - NLoS_Xsg_x[c]
            NLoS_error_slam_lidar_y[c] = NLoS_lidar_kalman_slam_y_time[c] - NLoS_Xsg_y[c]

            NLoS_error_slam_kalman[c] = math.sqrt(pow(NLoS_error_slam_kalman_x[c],2)+pow(NLoS_error_slam_kalman_y[c],2))
            NLoS_error_slam_lidar[c] = math.sqrt(pow(NLoS_error_slam_lidar_x[c],2)+pow(NLoS_error_slam_lidar_y[c],2))

        for p in range(len(NLoS_slam_x_time)-1):
            NLoS_slam_delta = math.sqrt(pow(NLoS_Xsg_x[k+1]-NLoS_Xsg_x[p],2)+pow(NLoS_Xsg_y[k+1]-NLoS_Xsg_y[p],2))
            NLoS_lidar_delta = math.sqrt(pow(NLoS_lidar_kalman_slam_x_time[k+1]-NLoS_lidar_kalman_slam_x_time[p],2)+pow(NLoS_lidar_kalman_slam_y_time[k+1]-NLoS_lidar_kalman_slam_y_time[p],2))
            NLoS_kalman_delta = math.sqrt(pow(NLoS_kalman_slam_x_time[k+1]-NLoS_kalman_slam_x_time[p],2)+pow(NLoS_kalman_slam_y_time[k+1]-NLoS_kalman_slam_y_time[p],2))

            NLoS_lidar_delta_l[p] = NLoS_lidar_delta - NLoS_slam_delta
            NLoS_kalman_delta_l[p] = NLoS_kalman_delta - NLoS_slam_delta




        #### ---- T-test ---- ####
        k_delta = len(LoS_lidar_delta_l)+len(NLoS_lidar_delta_l) - 2
        k_abs = len(LoS_error_slam_lidar)+len(NLoS_error_slam_lidar) - 2
        k_abs_x = len(LoS_error_slam_lidar_x)+len(NLoS_error_slam_lidar_x) - 2
        k_abs_y = len(LoS_error_slam_lidar_y)+len(NLoS_error_slam_lidar_y) - 2

        mean_LoS_lidar_delta_l = np.mean(LoS_lidar_delta_l)
        mean_LoS_kalman_delta_l = np.mean(LoS_kalman_delta_l)
        mean_LoS_error_slam_lidar = np.mean(LoS_error_slam_lidar)
        mean_LoS_error_slam_kalman = np.mean(LoS_error_slam_kalman)
        mean_LoS_error_slam_lidar_x = np.mean(LoS_error_slam_lidar_x)
        mean_LoS_error_slam_lidar_y = np.mean(LoS_error_slam_lidar_y)
        mean_LoS_error_slam_kalman_x = np.mean(LoS_error_slam_kalman_x)
        mean_LoS_error_slam_kalman_y = np.mean(LoS_error_slam_kalman_y)

        mean_NLoS_lidar_delta_l = np.mean(NLoS_lidar_delta_l)
        mean_NLoS_kalman_delta_l = np.mean(NLoS_kalman_delta_l)
        mean_NLoS_error_slam_lidar = np.mean(NLoS_error_slam_lidar)
        mean_NLoS_error_slam_kalman = np.mean(NLoS_error_slam_kalman)
        mean_NLoS_error_slam_lidar_x = np.mean(NLoS_error_slam_lidar_x)
        mean_NLoS_error_slam_lidar_y = np.mean(NLoS_error_slam_lidar_y)
        mean_NLoS_error_slam_kalman_x = np.mean(NLoS_error_slam_kalman_x)
        mean_NLoS_error_slam_kalman_y = np.mean(NLoS_error_slam_kalman_y)

        for j in range(len(LoS_kalman_delta_l)):
            LoS_dist_s_delta_kalman += pow(LoS_kalman_delta_l[j] - mean_LoS_kalman_delta_l, 2)
            LoS_dist_s_delta_lidar += pow(LoS_lidar_delta_l[j] - mean_LoS_lidar_delta_l, 2)
            LoS_dist_s_abs_lidar += pow(LoS_error_slam_lidar[j]-mean_LoS_error_slam_lidar, 2)
            LoS_dist_s_abs_kalman += pow(LoS_error_slam_kalman[j]-mean_LoS_error_slam_kalman, 2)
            LoS_dist_s_abs_x_kalman += pow(LoS_error_slam_kalman_x[j]-mean_LoS_error_slam_kalman_x, 2)
            LoS_dist_s_abs_y_kalman += pow(LoS_error_slam_kalman_y[j]-mean_LoS_error_slam_kalman_y, 2)
            LoS_dist_s_abs_x_lidar += pow(LoS_error_slam_lidar_x[j]- mean_LoS_error_slam_lidar_x, 2)
            LoS_dist_s_abs_y_lidar += pow(LoS_error_slam_lidar_y[j]- mean_LoS_error_slam_lidar_y, 2)

        for g in range(len(NLoS_kalman_delta_l)):
            NLoS_dist_s_delta_kalman += pow(NLoS_kalman_delta_l[g] - mean_NLoS_kalman_delta_l, 2)
            NLoS_dist_s_delta_lidar += pow(NLoS_lidar_delta_l[g] - mean_NLoS_lidar_delta_l, 2)
            NLoS_dist_s_abs_lidar += pow(NLoS_error_slam_lidar[g]-mean_NLoS_error_slam_lidar, 2)
            NLoS_dist_s_abs_kalman += pow(NLoS_error_slam_kalman[g]-mean_NLoS_error_slam_kalman, 2)
            NLoS_dist_s_abs_x_kalman += pow(NLoS_error_slam_kalman_x[g]-mean_NLoS_error_slam_kalman_x, 2)
            NLoS_dist_s_abs_y_kalman += pow(NLoS_error_slam_kalman_y[g]-mean_NLoS_error_slam_kalman_y, 2)
            NLoS_dist_s_abs_x_lidar += pow(NLoS_error_slam_lidar_x[g]- mean_NLoS_error_slam_lidar_x, 2)
            NLoS_dist_s_abs_y_lidar += pow(NLoS_error_slam_lidar_y[g]- mean_NLoS_error_slam_lidar_y, 2)




        LoS_S_delta_kalman = 1/(len(LoS_kalman_delta_l)-1)*LoS_dist_s_delta_kalman
        LoS_S_abs_kalman = 1/(len(LoS_error_slam_kalman)-1)*LoS_dist_s_abs_kalman
        LoS_S_abs_x_kalman = 1/(len(LoS_error_slam_kalman_x)-1)*LoS_dist_s_abs_x_kalman
        LoS_S_abs_y_kalman = 1/(len(LoS_error_slam_kalman_y)-1)*LoS_dist_s_abs_y_kalman

        LoS_S_delta_lidar = 1/(len(LoS_lidar_delta_l)-1)*LoS_dist_s_delta_lidar
        LoS_S_abs_lidar = 1/(len(LoS_error_slam_lidar)-1)*LoS_dist_s_abs_lidar
        LoS_S_abs_x_lidar = 1/(len(LoS_error_slam_lidar_x)-1)*LoS_dist_s_abs_x_lidar
        LoS_S_abs_y_lidar = 1/(len(LoS_error_slam_lidar_y)-1)*LoS_dist_s_abs_y_lidar

        NLoS_S_delta_kalman = 1/(len(NLoS_kalman_delta_l)-1)*NLoS_dist_s_delta_kalman
        NLoS_S_abs_kalman = 1/(len(NLoS_error_slam_kalman)-1)*NLoS_dist_s_abs_kalman
        NLoS_S_abs_x_kalman = 1/(len(NLoS_error_slam_kalman_x)-1)*NLoS_dist_s_abs_x_kalman
        NLoS_S_abs_y_kalman = 1/(len(NLoS_error_slam_kalman_y)-1)*NLoS_dist_s_abs_y_kalman

        NLoS_S_delta_lidar = 1/(len(NLoS_lidar_delta_l)-1)*NLoS_dist_s_delta_lidar
        NLoS_S_abs_lidar = 1/(len(NLoS_error_slam_lidar)-1)*NLoS_dist_s_abs_lidar
        NLoS_S_abs_x_lidar = 1/(len(NLoS_error_slam_lidar_x)-1)*NLoS_dist_s_abs_x_lidar
        NLoS_S_abs_y_lidar = 1/(len(NLoS_error_slam_lidar_y)-1)*NLoS_dist_s_abs_y_lidar

        t_LoS_delta = (mean_LoS_kalman_delta_l - mean_LoS_lidar_delta_l)/math.sqrt((pow(LoS_S_delta_kalman, 2)/mean_LoS_kalman_delta_l)+(pow(LoS_S_delta_lidar, 2)/mean_LoS_lidar_delta_l))
        t_LoS_abs = (mean_LoS_kalman_abs - mean_LoS_lidar_abs)/math.sqrt((pow(LoS_S_abs_kalman, 2)/mean_LoS_kalman_abs)+(pow(LoS_S_abs_lidar, 2)/mean_LoS_lidar_abs))
        t_LoS_abs_x = (mean_LoS_kalman_abs_x - mean_LoS_lidar_abs_x)/math.sqrt((pow(LoS_S_abs_x_kalman, 2)/mean_LoS_kalman_abs_x)+(pow(LoS_S_abs_x_lidar, 2)/mean_LoS_lidar_abs_x))
        t_LoS_abs_y = (mean_LoS_kalman_abs_y - mean_LoS_lidar_abs_y)/math.sqrt((pow(LoS_S_abs_y_kalman, 2)/mean_LoS_kalman_abs_y)+(pow(LoS_S_abs_y_lidar, 2)/mean_LoS_lidar_abs_y))

        t_NLoS_delta = (mean_NLoS_kalman_delta_l - mean_NLoS_lidar_delta_l)/math.sqrt((pow(NLoS_S_delta_kalman, 2)/mean_NLoS_kalman_delta_l)+(pow(NLoS_S_delta_lidar, 2)/mean_NLoS_lidar_delta_l))
        t_NLoS_abs = (mean_NLoS_kalman_abs - mean_NLoS_lidar_abs)/math.sqrt((pow(NLoS_S_abs_kalman, 2)/mean_NLoS_kalman_abs)+(pow(NLoS_S_abs_lidar, 2)/mean_NLoS_lidar_abs))
        t_NLoS_abs_x = (mean_NLoS_kalman_abs_x - mean_NLoS_lidar_abs_x)/math.sqrt((pow(NLoS_S_abs_x_kalman, 2)/mean_NLoS_kalman_abs_x)+(pow(NLoS_S_abs_x_lidar, 2)/mean_NLoS_lidar_abs_x))
        t_NLoS_abs_y = (mean_NLoS_kalman_abs_y - mean_NLoS_lidar_abs_y)/math.sqrt((pow(NLoS_S_abs_y_kalman, 2)/mean_NLoS_kalman_abs_y)+(pow(NLoS_S_abs_y_lidar, 2)/mean_NLoS_lidar_abs_y))


        t_crit_LoS = scipy.stats.t.ppf(q=1-.05,df=k)
        t_crit_NLoS = scipy.stats.t.ppf(q=1-.05,df=k)





        fieldnames = ["t_LoS_delta", "t_NLoS_delta", "t_LoS_abs", "t_NLoS_abs", "t_LoS_abs_x", "t_NLoS_abs_x", "t_LoS_abs_y", "t_NLoS_abs_y"]

        if os.path.exists(path_out) == False:
            with open(path_out, 'w') as csv_file:
                csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                csv_writer.writeheader()
                pass

        with open(path_out, 'a') as csv_file:
            #print("opening file")
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            info = {
                "t_LoS_delta": t_LoS_delta,
                "t_NLoS_delta": t_NLoS_delta,
                "t_LoS_abs": t_LoS_abs,
                "t_NLoS_abs": t_NLoS_abs,
                "t_LoS_abs_x": t_LoS_abs_x,
                "t_NLoS_abs_x": t_NLoS_abs_x,
                "t_LoS_abs_y": t_LoS_abs_y,
                "t_NLoS_abs_y": t_NLoS_abs_y

                }
            csv_writer.writerow(info)
        #number_of_files = number_of_files + 1

    number_of_files = number_of_files + 2
    with open(path_out, 'a') as csv_file:
        #fieldnames = ["t_LoS_delta", "t_NLoS_delta", "t_LoS_abs", "t_NLoS_abs", "t_LoS_abs_x", "t_NLoS_abs_x", "t_LoS_abs_y", "t_NLoS_abs_y"]

        #print("opening file")
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        info = {
            "t_LoS_delta": "=AVERAGE(A2:A%s)" % number_of_files,
            "t_NLoS_delta": "=AVERAGE(B2:B%s)" % number_of_files,
            "t_LoS_abs": "=AVERAGE(C2:C%s)" % number_of_files,
            "t_NLoS_abs": "=AVERAGE(D2:D%s)" % number_of_files,
            "t_LoS_abs_x": "=AVERAGE(E2:E%s)" % number_of_files,
            "t_NLoS_abs_x": "=AVERAGE(F2:F%s)" % number_of_files,
            "t_LoS_abs_y": "=AVERAGE(G2:G%s)" % number_of_files,
            "t_NLoS_abs_y": "=AVERAGE(H2:H%s)" % number_of_files
            #"LoS_mean_error_lidar": "=AVERAGE(I2:I%s)" % number_of_files,
            #"LoS_std_error_lidar": "=AVERAGE(J2:J%s)" % number_of_files,
            #"NLoS_mean_error_kalman": "=AVERAGE(K2:K%s)" % number_of_files,
            #"NLoS_std_error_kalman": "=AVERAGE(L2:L%s)" % number_of_files,
            #"NLoS_mean_error_lidar": "=AVERAGE(M2:M%s)" % number_of_files,
            #"NLoS_std_error_lidar": "=AVERAGE(N2:N%s)" % number_of_files

            }
        csv_writer.writerow(info)
    print("im done Casper ¯\_(ツ)_/¯")


if __name__ == '__main__':
    main()
