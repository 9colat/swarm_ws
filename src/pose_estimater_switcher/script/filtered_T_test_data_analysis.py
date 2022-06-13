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
path_NLoS = str(Path.home().joinpath("test_data/1_filtered_data", "filtered_NLoS_dyn_log%s.csv"))
path_LoS = str(Path.home().joinpath("test_data/1_filtered_data", "filtered_LoS_dyn_log%s.csv"))
path_LoS_lidar = str(Path.home().joinpath("test_data/1_filtered_data", "filtered_LoS_lidar%s.csv"))
path_NLoS_lidar = str(Path.home().joinpath("test_data/1_filtered_data", "filtered_NLoS_lidar%s.csv"))
path_output = str(Path.home().joinpath("test_data","1_filtered_data"))
folder_path = str(Path.home().joinpath("test_data/1_filtered_data"))
output_path = str(Path.home()) + '/' + "figure/1_filtered_data/"




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
    global path, folder_path
    number_of_files = 0
    output_file_name = 't_test.csv'
    path_out = str(Path.home().joinpath("test_data/1_filtered_data", output_file_name))

    path_output_file = Path(path_out)
    if path_output_file.is_file():
        os.remove(path_out)
    true_x = [22653, 22673]
    true_y = [1039, 3822]
    per_x = [22000, 100]
    per_y = [158.10276679, 0.7186489399]


    #while os.path.exists(path_NLoS % number_of_files):
    for number_of_files in range(0,5):
        print(number_of_files)
        t_1 = 0
        t_2 = 0
        t_3 = 0
        t_4 = 0

        local_path_LoS = path_LoS %  number_of_files
        local_path_NLoS = path_NLoS %  number_of_files
        local_path_LoS_lidar = path_LoS_lidar %  number_of_files
        local_path_NLoS_lidar = path_NLoS_lidar %  number_of_files
        local_path_lidar = path_LoS_lidar %  number_of_files
        data_LoS = pd.read_csv(local_path_LoS)
        data_NLoS = pd.read_csv(local_path_NLoS)
        data_lidar_LoS = pd.read_csv(local_path_LoS_lidar)
        print(local_path_NLoS_lidar)
        data_lidar_NLoS = pd.read_csv(local_path_NLoS_lidar)
        print("hello")

        LoS_dist_from_path_kalman = [0]*len(data_lidar_LoS["dist1"])
        NLoS_dist_from_path_kalman = [0]*len(data_lidar_NLoS["dist1"])
        LoS_dist_from_path_lidar = [0]*len(data_lidar_LoS["dist1"])
        NLoS_dist_from_path_lidar = [0]*len(data_lidar_NLoS["dist1"])
        LoS_dist_from_path_bank = [0]*len(data_lidar_LoS["dist1"])
        NLoS_dist_from_path_bank = [0]*len(data_lidar_NLoS["dist1"])

        LoS_dist_from_path_kalman_per = [0]* len(data_lidar_LoS["dist1"])
        NLoS_dist_from_path_kalman_per = [0]* len(data_lidar_NLoS["dist1"])
        LoS_dist_from_path_lidar_per  = [0]* len(data_lidar_LoS["dist1"])
        NLoS_dist_from_path_lidar_per = [0]* len(data_lidar_NLoS["dist1"])


        LoS_dist_s_kalman = 0
        NLoS_dist_s_kalman = 0
        LoS_dist_s_lidar = 0
        NLoS_dist_s_lidar = 0
        LoS_dist_s_bank = 0
        NLoS_dist_s_bank = 0
        LoS_x1_k = 0
        LoS_x2_k = 0
        LoS_x1_k1 = 0
        LoS_x2_k1 = 0
        LoS_y1_k = 0
        LoS_y2_k = 0
        LoS_y1_k1 = 0
        LoS_y2_k1 = 0
        NLoS_x1_k = 0
        NLoS_x2_k = 0
        NLoS_x1_k1 = 0
        NLoS_x2_k1 = 0
        NLoS_y1_k = 0
        NLoS_y2_k = 0
        NLoS_y1_k1 = 0
        NLoS_y2_k1 = 0
        delta_lidar_LoS = 0
        delta_lidar_NLoS = 0
        LoS_kalman_error = [0]*(len(data_lidar_LoS["dist1"])-1)
        LoS_lidar_error = [0]*(len(data_lidar_LoS["dist1"])-1)
        NLoS_kalman_error = [0]*(len(data_lidar_NLoS["dist1"])-1)
        NLoS_lidar_error = [0]*(len(data_lidar_NLoS["dist1"])-1)




        LoS_with_x = data_LoS["with_x"]
        LoS_with_y = data_LoS["with_y"]
        LoS_without_x = data_LoS["without_x"]
        LoS_without_y = data_LoS["without_y"]
        LoS_simple_x = data_LoS["simple_x"]
        LoS_simple_y = data_LoS["simple_y"]
        LoS_multi_x = data_LoS["muti_kalman_x"]
        LoS_multi_y = data_LoS["muti_kalman_y"]
        LoS_with_lidar_dist_1 = data_lidar_LoS["dist1"]
        LoS_with_lidar_angle_1 = data_lidar_LoS["angle1"]
        LoS_with_lidar_dist_2 = data_lidar_LoS["dist1"]
        LoS_with_lidar_angle_2 = data_lidar_LoS["angle1"]

        NLoS_with_x = data_NLoS["with_x"]
        NLoS_with_y = data_NLoS["with_y"]
        NLoS_without_x = data_NLoS["without_x"]
        NLoS_without_y = data_NLoS["without_y"]
        NLoS_simple_x = data_NLoS["simple_x"]
        NLoS_simple_y = data_NLoS["simple_y"]
        NLoS_multi_x = data_NLoS["muti_kalman_x"]
        NLoS_multi_y = data_NLoS["muti_kalman_y"]
        NLoS_with_lidar_dist_1 = data_lidar_NLoS["dist1"]
        NLoS_with_lidar_angle_1 = data_lidar_NLoS["angle1"]
        NLoS_with_lidar_dist_2 = data_lidar_NLoS["dist1"]
        NLoS_with_lidar_angle_2 = data_lidar_NLoS["angle1"]




        for i in range(len(LoS_with_x)):
            LoS_dist_from_path_kalman[i] = abs((true_x[1]-true_x[0])*(true_y[0]-LoS_without_y[i])-(true_x[0]-LoS_without_x[i])*(true_y[1]-true_y[0]))/math.sqrt(pow(true_x[1]-true_x[0],2)+pow(true_y[1]-true_y[0],2))
            LoS_dist_from_path_lidar[i] = abs((true_x[1]-true_x[0])*(true_y[0]-LoS_with_y[i])-(true_x[0]-LoS_with_x[i])*(true_y[1]-true_y[0]))/math.sqrt(pow(true_x[1]-true_x[0],2)+pow(true_y[1]-true_y[0],2))
            LoS_dist_from_path_bank[i] = abs((true_x[1]-true_x[0])*(true_y[0]-LoS_multi_y[i])-(true_x[0]-LoS_multi_x[i])*(true_y[1]-true_y[0]))/math.sqrt(pow(true_x[1]-true_x[0],2)+pow(true_y[1]-true_y[0],2))
            LoS_dist_from_path_kalman_per[i] = abs((per_x[1]-per_x[0])*(per_y[0]-LoS_without_y[i])-(per_x[0]-LoS_without_x[i])*(per_y[1]-per_y[0]))/math.sqrt(pow(per_x[1]-per_x[0],2)+pow(per_y[1]-per_y[0],2))
            LoS_dist_from_path_lidar_per[i] = abs((per_x[1]-per_x[0])*(per_y[0]-LoS_with_y[i])-(per_x[0]-LoS_with_x[i])*(per_y[1]-per_y[0]))/math.sqrt(pow(per_x[1]-per_x[0],2)+pow(per_y[1]-per_y[0],2))


        for s in range(len(NLoS_without_y)-1):
            #print(len(NLoS_dist_from_path_kalman),len(NLoS_without_y))
            NLoS_dist_from_path_kalman[s] = abs((true_x[1]-true_x[0])*(true_y[0]-NLoS_without_y[s])-(true_x[0]-NLoS_without_x[s])*(true_y[1]-true_y[0]))/math.sqrt(pow(true_x[1]-true_x[0],2)+pow(true_y[1]-true_y[0],2))
            NLoS_dist_from_path_lidar[s] = abs((true_x[1]-true_x[0])*(true_y[0]-NLoS_with_y[s])-(true_x[0]-NLoS_with_x[s])*(true_y[1]-true_y[0]))/math.sqrt(pow(true_x[1]-true_x[0],2)+pow(true_y[1]-true_y[0],2))
            NLoS_dist_from_path_bank[s] = abs((true_x[1]-true_x[0])*(true_y[0]-NLoS_multi_y[s])-(true_x[0]-NLoS_multi_x[s])*(true_y[1]-true_y[0]))/math.sqrt(pow(true_x[1]-true_x[0],2)+pow(true_y[1]-true_y[0],2))
            NLoS_dist_from_path_kalman_per[s] = abs((per_x[1]-per_x[0])*(per_y[0]-NLoS_without_y[s])-(per_x[0]-NLoS_without_x[s])*(per_y[1]-per_y[0]))/math.sqrt(pow(per_x[1]-per_x[0],2)+pow(per_y[1]-per_y[0],2))
            NLoS_dist_from_path_lidar_per[s] = abs((per_x[1]-per_x[0])*(per_y[0]-NLoS_with_y[s])-(per_x[0]-NLoS_with_x[s])*(per_y[1]-per_y[0]))/math.sqrt(pow(per_x[1]-per_x[0],2)+pow(per_y[1]-per_y[0],2))



        for z in range(len(LoS_with_lidar_dist_1)-1):
            LoS_x1_k = LoS_with_lidar_dist_1[z]
            LoS_x2_k = LoS_with_lidar_dist_2[z]
            LoS_y1_k = LoS_with_lidar_angle_1[z]
            LoS_y2_k = LoS_with_lidar_angle_2[z]

            LoS_x1_k1 = LoS_with_lidar_dist_1[z+1]
            LoS_x2_k1 = LoS_with_lidar_dist_2[z+1]
            LoS_y1_k1 = LoS_with_lidar_angle_1[z+1]
            LoS_y2_k1 = LoS_with_lidar_angle_2[z+1]

            delta_lidar_LoS = (math.sqrt(pow(LoS_x1_k1-LoS_x1_k,2)+pow(LoS_y1_k1-LoS_y1_k,2))+math.sqrt(pow(LoS_x2_k1-LoS_x2_k,2)+pow(LoS_y2_k1-LoS_y2_k,2)))/2
            #print(len(LoS_with_lidar_dist_1)-1,len(LoS_dist_from_path_lidar_per),len(data_LoS["with_x"]))
            LoS_kalman_error[z] = delta_lidar_LoS * 1000 - LoS_dist_from_path_kalman_per[z]
            LoS_lidar_error[z] = delta_lidar_LoS * 1000 - LoS_dist_from_path_lidar_per[z]


        for p in range(len(NLoS_with_lidar_dist_1)-1):
            #print(len(NLoS_with_lidar_dist_1)-1)
            NLoS_x1_k = NLoS_with_lidar_dist_1[p]
            NLoS_x2_k = NLoS_with_lidar_dist_2[p]
            NLoS_y1_k = NLoS_with_lidar_angle_1[p]
            NLoS_y2_k = NLoS_with_lidar_angle_2[p]

            NLoS_x1_k1 = NLoS_with_lidar_dist_1[p+1]
            NLoS_x2_k1 = NLoS_with_lidar_dist_2[p+1]
            NLoS_y1_k1 = NLoS_with_lidar_angle_1[p+1]
            NLoS_y2_k1 = NLoS_with_lidar_angle_2[p+1]

            delta_lidar_NLoS = (math.sqrt(pow(NLoS_x1_k1-NLoS_x1_k,2)+pow(NLoS_y1_k1-NLoS_y1_k,2))+math.sqrt(pow(NLoS_x2_k1-NLoS_x2_k,2)+pow(NLoS_y2_k1-NLoS_y2_k,2)))/2

            NLoS_kalman_error[p] = delta_lidar_NLoS * 1000 - NLoS_dist_from_path_kalman_per[p]
            NLoS_lidar_error[p] = delta_lidar_NLoS * 1000 - NLoS_dist_from_path_lidar_per[p]


        k = len(LoS_dist_from_path_kalman)+len(NLoS_dist_from_path_kalman) - 2
        LoS_dist_from_path_kalman_mean = np.mean(LoS_dist_from_path_kalman)
        NLoS_dist_from_path_kalman_mean = np.mean(NLoS_dist_from_path_kalman)
        LoS_dist_from_path_lidar_mean = np.mean(LoS_dist_from_path_lidar)
        NLoS_dist_from_path_lidar_mean = np.mean(NLoS_dist_from_path_lidar)
        LoS_dist_from_path_bank_mean = np.mean(LoS_dist_from_path_bank)
        NLoS_dist_from_path_bank_mean = np.mean(NLoS_dist_from_path_bank)

        LoS_dist_from_path_kalman_mean_per = np.mean(LoS_dist_from_path_kalman_per)
        NLoS_dist_from_path_kalman_mean_per = np.mean(NLoS_dist_from_path_kalman_per)
        LoS_dist_from_path_lidar_mean_per = np.mean(LoS_dist_from_path_lidar_per)
        NLoS_dist_from_path_lidar_mean_per = np.mean(NLoS_dist_from_path_lidar_per)

        LoS_lidar_error_mean = np.mean(LoS_lidar_error)
        LoS_kalman_error_mean = np.mean(LoS_kalman_error)
        NLoS_lidar_error_mean = np.mean(NLoS_lidar_error)
        NLoS_kalman_error_mean = np.mean(NLoS_kalman_error)
        LoS_lidar_error_std = np.std(LoS_lidar_error)
        LoS_kalman_error_std = np.std(LoS_kalman_error)
        NLoS_lidar_error_std = np.std(NLoS_lidar_error)
        NLoS_kalman_error_std = np.std(NLoS_kalman_error)


        for j in range(len(LoS_with_x)):
            LoS_dist_s_kalman += pow(LoS_dist_from_path_kalman[j] - LoS_dist_from_path_kalman_mean, 2)
            LoS_dist_s_lidar += pow(LoS_dist_from_path_lidar[j] - LoS_dist_from_path_lidar_mean, 2)
            LoS_dist_s_bank += pow(LoS_dist_from_path_bank[j] - LoS_dist_from_path_lidar_mean, 2)

        for g in range(len(NLoS_dist_from_path_kalman)):
            NLoS_dist_s_kalman += pow(NLoS_dist_from_path_kalman[g] - NLoS_dist_from_path_kalman_mean, 2)
            NLoS_dist_s_lidar += pow(NLoS_dist_from_path_lidar[g] - NLoS_dist_from_path_lidar_mean, 2)
            NLoS_dist_s_bank += pow(NLoS_dist_from_path_bank[g] - NLoS_dist_from_path_lidar_mean, 2)

        LoS_S_kalman = 1/(len(LoS_dist_from_path_kalman)-1)*LoS_dist_s_kalman
        NLoS_S_kalman = 1/(len(NLoS_dist_from_path_kalman)-1)*NLoS_dist_s_kalman
        LoS_S_lidar = 1/(len(LoS_dist_from_path_lidar)-1)*LoS_dist_s_lidar
        NLoS_S_lidar = 1/(len(NLoS_dist_from_path_lidar)-1)*NLoS_dist_s_lidar
        LoS_S_bank = 1/(len(LoS_dist_from_path_bank)-1)*LoS_dist_s_bank
        NLoS_S_bank = 1/(len(NLoS_dist_from_path_bank)-1)*NLoS_dist_s_bank

        #print(LoS_dist_from_path_kalman_mean)

        t_LoS = (LoS_dist_from_path_kalman_mean - LoS_dist_from_path_lidar_mean)/math.sqrt((pow(LoS_S_kalman,2)/LoS_dist_from_path_kalman_mean)+(pow(LoS_S_lidar,2)/LoS_dist_from_path_lidar_mean))
        t_NLoS = (NLoS_dist_from_path_kalman_mean - NLoS_dist_from_path_lidar_mean)/math.sqrt((pow(NLoS_S_kalman,2)/NLoS_dist_from_path_kalman_mean)+(pow(NLoS_S_lidar,2)/NLoS_dist_from_path_lidar_mean))

        t_LoS_bank = (LoS_dist_from_path_kalman_mean - LoS_dist_from_path_bank_mean)/math.sqrt((pow(LoS_S_kalman,2)/LoS_dist_from_path_kalman_mean)+(pow(LoS_S_bank,2)/LoS_dist_from_path_bank_mean))
        t_NLoS_bank = (NLoS_dist_from_path_kalman_mean - NLoS_dist_from_path_bank_mean)/math.sqrt((pow(NLoS_S_kalman,2)/NLoS_dist_from_path_kalman_mean)+(pow(NLoS_S_bank,2)/NLoS_dist_from_path_bank_mean))

        t_crit_LoS = scipy.stats.t.ppf(q=1-.05,df=k)
        t_crit_NLoS = scipy.stats.t.ppf(q=1-.05,df=k)

        if t_LoS < t_crit_LoS:
            t_1 = 1
        if t_NLoS < t_crit_NLoS:
            t_2 = 1
        if t_LoS_bank < t_crit_LoS:
            t_3 = 1
        if t_NLoS_bank < t_crit_NLoS:
            t_4 = 1







        fieldnames = ["t_LoS", "t_crit_LoS","no_sig_diff_LoS","t_NLoS", "t_crit_NLoS","no_sig_diff_NLoS","LoS_mean_error_kalman","LoS_std_error_kalman","LoS_mean_error_lidar","LoS_std_error_lidar","NLoS_mean_error_kalman","NLoS_std_error_kalman","NLoS_mean_error_lidar","NLoS_std_error_lidar","t_LoS_bank", "t_crit_LoS_bank","no_sig_diff_LoS_bank","t_NLoS_bank", "t_crit_NLoS_bank","no_sig_diff_NLoS_bank"]

        if os.path.exists(path_out) == False:
            with open(path_out, 'w') as csv_file:
                csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                csv_writer.writeheader()
                pass

        with open(path_out, 'a') as csv_file:
            #print("opening file")
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            info = {
                "t_LoS": t_LoS,
                "t_crit_LoS": t_crit_LoS,
                "no_sig_diff_LoS": t_1,
                "t_NLoS": t_NLoS,
                "t_crit_NLoS": t_crit_NLoS,
                "no_sig_diff_NLoS": t_2,
                "LoS_mean_error_kalman": LoS_kalman_error_mean,
                "LoS_std_error_kalman": LoS_kalman_error_std,
                "LoS_mean_error_lidar": LoS_lidar_error_mean,
                "LoS_std_error_lidar": LoS_lidar_error_std,
                "NLoS_mean_error_kalman": NLoS_kalman_error_mean,
                "NLoS_std_error_kalman": NLoS_kalman_error_std,
                "NLoS_mean_error_lidar": NLoS_lidar_error_mean,
                "NLoS_std_error_lidar": NLoS_lidar_error_std,
                "t_LoS_bank": t_LoS_bank,
                "t_crit_LoS_bank": t_crit_LoS,
                "no_sig_diff_LoS_bank": t_3,
                "t_NLoS_bank": t_NLoS_bank,
                "t_crit_NLoS_bank": t_crit_NLoS,
                "no_sig_diff_NLoS_bank": t_4

                }
            csv_writer.writerow(info)
        #number_of_files = number_of_files + 1
    number_of_files = number_of_files + 1
    with open(path_out, 'a') as csv_file:
        #print("opening file")
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        info = {
            "t_LoS": "=AVERAGE(A2:A%s)" % number_of_files,
            "t_crit_LoS": "=AVERAGE(B2:B%s)" % number_of_files,
            "no_sig_diff_LoS": "=AVERAGE(C2:C%s)" % number_of_files,
            "t_NLoS": "=AVERAGE(D2:D%s)" % number_of_files,
            "t_crit_NLoS": "=AVERAGE(E2:E%s)" % number_of_files,
            "no_sig_diff_NLoS": "=AVERAGE(F2:F%s)" % number_of_files,
            "LoS_mean_error_kalman": "=AVERAGE(G2:G%s)" % number_of_files,
            "LoS_std_error_kalman": "=AVERAGE(H2:H%s)" % number_of_files,
            "LoS_mean_error_lidar": "=AVERAGE(I2:I%s)" % number_of_files,
            "LoS_std_error_lidar": "=AVERAGE(J2:J%s)" % number_of_files,
            "NLoS_mean_error_kalman": "=AVERAGE(K2:K%s)" % number_of_files,
            "NLoS_std_error_kalman": "=AVERAGE(L2:L%s)" % number_of_files,
            "NLoS_mean_error_lidar": "=AVERAGE(M2:M%s)" % number_of_files,
            "NLoS_std_error_lidar": "=AVERAGE(N2:N%s)" % number_of_files

            }
        csv_writer.writerow(info)
    print("im done baby ;)")

    print("LoS kalman STD: ",np.std(np.array(LoS_dist_from_path_kalman)))
    print("LoS kalman mean: ",LoS_dist_from_path_kalman_mean)
    print("NLoS kalman STD: ",np.std(np.array(NLoS_dist_from_path_kalman)))
    print("NLoS kalman mean: ",NLoS_dist_from_path_kalman_mean)
    print("LoS Lidar STD: ",np.std(np.array(LoS_dist_from_path_lidar)))
    print("LoS Lidar mean: ",LoS_dist_from_path_lidar_mean)
    print("NLoS Lidar STD: ",np.std(np.array(NLoS_dist_from_path_lidar)))
    print("NLoS Lidar mean: ",NLoS_dist_from_path_lidar_mean)




if __name__ == '__main__':
    main()
