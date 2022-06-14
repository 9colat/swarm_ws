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
#from sympy import symbols, Eq, solve

##### setting up the path for the data file and folder #####
path = str(Path.home().joinpath("test_data", "LoS_dyn_log%s.csv"))
path_output = str(Path.home().joinpath("test_data", "LoS_dyn_log_output%s.csv"))
folder_path = str(Path.home().joinpath("test_data"))
output_path = str(Path.home()) + '/' + "figure/"
plt.style.use('fivethirtyeight')
fig1 = plt.figure(figsize=(20,20))
fig1.suptitle('Position plot test')
ax1 = fig1.add_subplot(2,2,1)
ax2 = fig1.add_subplot(2,2,2)
ax10 = fig1.add_subplot(2,2,3)
ax11 = fig1.add_subplot(2,2,4)
fig2 = plt.figure(figsize=(10,10))
fig2.suptitle('Delta position')
ax3 = fig2.add_subplot(1,1,1)
fig3 = plt.figure(figsize=(10,10))
fig3.suptitle('Both position in 1 plot')
ax4 = fig3.add_subplot(1,1,1)
fig4 = plt.figure(figsize=(10,10))
fig4.suptitle('LiDAR over time')
ax5 = fig4.add_subplot(1,1,1)
fig5 = plt.figure(figsize=(20,30))
fig5.suptitle('Position coordinate delta plot')
ax6 = fig5.add_subplot(3,2,1)
ax7 = fig5.add_subplot(3,2,2)
ax8 = fig5.add_subplot(3,2,3)
ax9 = fig5.add_subplot(3,2,4)
ax12 = fig5.add_subplot(3,2,5)
ax13 = fig5.add_subplot(3,2,6)
fig6 = plt.figure(figsize=(10,10))
fig6.suptitle('velocity in 1 plot')
ax14 = fig6.add_subplot(1,1,1)
fig7 = plt.figure(figsize=(20,10))
fig7.suptitle('X and Y over time')
ax15 = fig7.add_subplot(1,2,1)
ax16 = fig7.add_subplot(1,2,2)
fig8 = plt.figure(figsize=(20,10))
fig8.suptitle('X and Y over time')
ax17 = fig8.add_subplot(1,2,1)
ax18 = fig8.add_subplot(1,2,2)
fig9 = plt.figure(figsize=(20,10))
#fig9.suptitle('X and Y over time')
ax19 = fig9.add_subplot(1,2,1)
ax20 = fig9.add_subplot(1,2,2)
fig10 = plt.figure(figsize=(20,20))
fig10.suptitle('Position plot test')
ax21 = fig10.add_subplot(2,2,1)
ax22 = fig10.add_subplot(2,2,2)
ax23 = fig10.add_subplot(2,2,3)
ax24 = fig10.add_subplot(2,2,4)
fig11 = plt.figure(figsize=(10,10))
fig11.suptitle('Position plot test')
ax25 = fig11.add_subplot(1,1,1)

isfolder = os.path.isdir(output_path)



def main():
    if not isfolder:
        os.mkdir(output_path)
        print("making directory")
    beacon_id =   [42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
    beacon_x =    [11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560]    #[11700  , 16244 , 7824  , 2000  , 21369 , 26163 , 26163 , 31000 , 35766 , 35766 , 40205 , 40204 , 16560 ]beacon_
    beacon_y =    [5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549]    #[5999   , 10150 , 5726  , 4499  , 6534  , 9939  , 3699  , 6519  , 10012  , 10012 , 11684 , 4363  , 3549 ]
    beacon_z =    [5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767]
    global path, folder_path
    number_of_files = 21
    output_file_name = 'LoS_dyn_log_outputdata.csv'
    path_out = str(Path.home().joinpath("test_data", output_file_name))

    path_output_file = Path(path_out)
    print(path_output_file.is_file())
    if path_output_file.is_file():
        os.remove(path_out)
    print(path % number_of_files)
    true_x = [22653, 22673]
    true_y = [1039, 3822]

    #a11, a12, a21, a22, b1, b2 = symbols('a11, a12, a21, a22, b1, b2')

    while os.path.exists(path % number_of_files):
        print(number_of_files)
        local_path = path %  number_of_files
        data = pd.read_csv(local_path)
        lidar_array = [[0] * len(data["lidar0"])]*360
        delta = [0] * len(data["lidar0"])
        delta_x = [0] * len(data["lidar0"])
        delta_y = [0] * len(data["lidar0"])
        delta_x_kalman_bank = [0] * len(data["lidar0"])
        delta_y_kalman_bank = [0] * len(data["lidar0"])
        delta_x_simple = [0] * len(data["lidar0"])
        delta_y_simple = [0] * len(data["lidar0"])
        vel_input = [0] * len(data["lidar0"])
        vel_with = [0] * len(data["lidar0"])
        vel_without = [0] * len(data["lidar0"])
        kalman_x = [0] * len(data["lidar0"])
        kalman_y = [0] * len(data["lidar0"])
        kalman_x_lidar = [0] * len(data["lidar0"])
        kalman_y_lidar = [0] * len(data["lidar0"])
        kalman_x_bank = [0] * len(data["lidar0"])
        kalman_y_bank = [0] * len(data["lidar0"])
        simple_x_data = [0] * len(data["lidar0"])
        simple_y_data = [0] * len(data["lidar0"])
        slam_x_time = [0] * (len(data["lidar0"])-101)
        slam_y_time = [0] * (len(data["lidar0"])-101)
        Xsg_x = [0] * (len(data["lidar0"])-101)
        Xsg_y = [0] * (len(data["lidar0"])-101)
        slam_x_time_tans = [0] * (len(data["lidar0"])-101)
        slam_y_time_tans = [0] * (len(data["lidar0"])-101)
        time_slam = np.linspace(1, len(data["lidar0"]-101/10), num=(len(data["lidar0"])-101))
        lidar_kalman_slam_x_time = [0] * (len(data["lidar0"])-101)
        lidar_kalman_slam_y_time = [0] * (len(data["lidar0"])-101)
        kalman_slam_x_time = [0] * (len(data["lidar0"])-101)
        kalman_slam_y_time = [0] * (len(data["lidar0"])-101)
        kalman_x_bank_slam = [0] * (len(data["lidar0"])-101)
        kalman_y_bank_slam = [0] * (len(data["lidar0"])-101)
        simple_x_data_slam = [0] * (len(data["lidar0"])-101)
        simple_y_data_slam = [0] * (len(data["lidar0"])-101)
        slam_delta_l = [0] * (len(data["lidar0"])-102)
        kalman_delta_l = [0] * (len(data["lidar0"])-102)
        lidar_delta_l = [0] * (len(data["lidar0"])-102)
        time_slam_l = np.linspace(1, len(data["lidar0"]-101/10), num=(len(data["lidar0"])-102))
        lidar_mean_x_array = [0]*100
        lidar_mean_y_array = [0]*100
        time_array = np.linspace(1, len(data["lidar0"]/10), num=len(data["lidar0"]))
        with_x = data["with_x"]
        with_y = data["with_y"]
        with_v = data["with_v"]
        with_hx = data["with_hx"]
        with_hy = data["with_hy"]
        without_x = data["without_x"]
        without_y = data["without_y"]
        without_v = data["without_v"]
        without_hx = data["without_hx"]
        without_hy = data["without_hy"]
        acc_x = data["acc_x"]
        acc_y = data["acc_y"]
        v = data["acc_z"]
        gyro_x = data["gyro_x"]
        gyro_y = data["gyro_y"]
        gyro_z = data["gyro_z"]
        ID = data["ID"]
        distance = data["distance"]
        linear_speed = data["linear_speed"]
        angular_speed = data["angular_speed"]
        simple_x = data["simple_x"]
        simple_y = data["simple_y"]
        multi_x = data["muti_kalman_x"]
        multi_y = data["muti_kalman_y"]
        if number_of_files > 19:
            slam_x = data["slam_x"]
            slam_y = data["slam_y"]
        good_lidar = data["lidar_good"]
        lidar_array[0] = data["lidar0"]
        lidar_array[1] = data["lidar1"]
        lidar_array[2] = data["lidar2"]
        lidar_array[3] = data["lidar3"]
        lidar_array[4] = data["lidar4"]
        lidar_array[5] = data["lidar5"]
        lidar_array[6] = data["lidar6"]
        lidar_array[7] = data["lidar7"]
        lidar_array[8] = data["lidar8"]
        lidar_array[9] = data["lidar9"]
        lidar_array[10] = data["lidar10"]
        lidar_array[11] = data["lidar11"]
        lidar_array[12] = data["lidar12"]
        lidar_array[13] = data["lidar13"]
        lidar_array[14] = data["lidar14"]
        lidar_array[15] = data["lidar15"]
        lidar_array[16] = data["lidar16"]
        lidar_array[17] = data["lidar17"]
        lidar_array[18] = data["lidar18"]
        lidar_array[19] = data["lidar19"]
        lidar_array[20] = data["lidar20"]
        lidar_array[21] = data["lidar21"]
        lidar_array[22] = data["lidar22"]
        lidar_array[23] = data["lidar23"]
        lidar_array[24] = data["lidar24"]
        lidar_array[25] = data["lidar25"]
        lidar_array[26] = data["lidar26"]
        lidar_array[27] = data["lidar27"]
        lidar_array[28] = data["lidar28"]
        lidar_array[29] = data["lidar29"]
        lidar_array[30] = data["lidar30"]
        lidar_array[31] = data["lidar31"]
        lidar_array[32] = data["lidar32"]
        lidar_array[33] = data["lidar33"]
        lidar_array[34] = data["lidar34"]
        lidar_array[35] = data["lidar35"]
        lidar_array[36] = data["lidar36"]
        lidar_array[37] = data["lidar37"]
        lidar_array[38] = data["lidar38"]
        lidar_array[39] = data["lidar39"]
        lidar_array[40] = data["lidar40"]
        lidar_array[41] = data["lidar41"]
        lidar_array[42] = data["lidar42"]
        lidar_array[43] = data["lidar43"]
        lidar_array[44] = data["lidar44"]
        lidar_array[45] = data["lidar45"]
        lidar_array[46] = data["lidar46"]
        lidar_array[47] = data["lidar47"]
        lidar_array[48] = data["lidar48"]
        lidar_array[49] = data["lidar49"]
        lidar_array[50] = data["lidar50"]
        lidar_array[51] = data["lidar51"]
        lidar_array[52] = data["lidar52"]
        lidar_array[53] = data["lidar53"]
        lidar_array[54] = data["lidar54"]
        lidar_array[55] = data["lidar55"]
        lidar_array[56] = data["lidar56"]
        lidar_array[57] = data["lidar57"]
        lidar_array[58] = data["lidar58"]
        lidar_array[59] = data["lidar59"]
        lidar_array[60] = data["lidar60"]
        lidar_array[61] = data["lidar61"]
        lidar_array[62] = data["lidar62"]
        lidar_array[63] = data["lidar63"]
        lidar_array[64] = data["lidar64"]
        lidar_array[65] = data["lidar65"]
        lidar_array[66] = data["lidar66"]
        lidar_array[67] = data["lidar67"]
        lidar_array[68] = data["lidar68"]
        lidar_array[69] = data["lidar69"] #nice
        lidar_array[70] = data["lidar70"]
        lidar_array[71] = data["lidar71"]
        lidar_array[72] = data["lidar72"]
        lidar_array[73] = data["lidar73"]
        lidar_array[74] = data["lidar74"]
        lidar_array[75] = data["lidar75"]
        lidar_array[76] = data["lidar76"]
        lidar_array[77] = data["lidar77"]
        lidar_array[78] = data["lidar78"]
        lidar_array[79] = data["lidar79"]
        lidar_array[80] = data["lidar80"]
        lidar_array[81] = data["lidar81"]
        lidar_array[82] = data["lidar82"]
        lidar_array[83] = data["lidar83"]
        lidar_array[84] = data["lidar84"]
        lidar_array[85] = data["lidar85"]
        lidar_array[86] = data["lidar86"]
        lidar_array[87] = data["lidar87"]
        lidar_array[88] = data["lidar88"]
        lidar_array[89] = data["lidar89"]
        lidar_array[90] = data["lidar90"]
        lidar_array[91] = data["lidar91"]
        lidar_array[92] = data["lidar92"]
        lidar_array[93] = data["lidar93"]
        lidar_array[94] = data["lidar94"]
        lidar_array[95] = data["lidar95"]
        lidar_array[96] = data["lidar96"]
        lidar_array[97] = data["lidar97"]
        lidar_array[98] = data["lidar98"]
        lidar_array[99] = data["lidar99"]
        lidar_array[100] = data["lidar100"]
        lidar_array[101] = data["lidar101"]
        lidar_array[102] = data["lidar102"]
        lidar_array[103] = data["lidar103"]
        lidar_array[104] = data["lidar104"]
        lidar_array[105] = data["lidar105"]
        lidar_array[106] = data["lidar106"]
        lidar_array[107] = data["lidar107"]
        lidar_array[108] = data["lidar108"]
        lidar_array[109] = data["lidar109"]
        lidar_array[110] = data["lidar110"]
        lidar_array[111] = data["lidar111"]
        lidar_array[112] = data["lidar112"]
        lidar_array[113] = data["lidar113"]
        lidar_array[114] = data["lidar114"]
        lidar_array[115] = data["lidar115"]
        lidar_array[116] = data["lidar116"]
        lidar_array[117] = data["lidar117"]
        lidar_array[118] = data["lidar118"]
        lidar_array[119] = data["lidar119"]
        lidar_array[120] = data["lidar120"]
        lidar_array[121] = data["lidar121"]
        lidar_array[122] = data["lidar122"]
        lidar_array[123] = data["lidar123"]
        lidar_array[124] = data["lidar124"]
        lidar_array[125] = data["lidar125"]
        lidar_array[126] = data["lidar126"]
        lidar_array[127] = data["lidar127"]
        lidar_array[128] = data["lidar128"]
        lidar_array[129] = data["lidar129"]
        lidar_array[130] = data["lidar130"]
        lidar_array[131] = data["lidar131"]
        lidar_array[132] = data["lidar132"]
        lidar_array[133] = data["lidar133"]
        lidar_array[134] = data["lidar134"]
        lidar_array[135] = data["lidar135"]
        lidar_array[136] = data["lidar136"]
        lidar_array[137] = data["lidar137"]
        lidar_array[138] = data["lidar138"]
        lidar_array[139] = data["lidar139"]
        lidar_array[140] = data["lidar140"]
        lidar_array[141] = data["lidar141"]
        lidar_array[142] = data["lidar142"]
        lidar_array[143] = data["lidar143"]
        lidar_array[144] = data["lidar144"]
        lidar_array[145] = data["lidar145"]
        lidar_array[146] = data["lidar146"]
        lidar_array[147] = data["lidar147"]
        lidar_array[148] = data["lidar148"]
        lidar_array[149] = data["lidar149"]
        lidar_array[150] = data["lidar150"]
        lidar_array[151] = data["lidar151"]
        lidar_array[152] = data["lidar152"]
        lidar_array[153] = data["lidar153"]
        lidar_array[154] = data["lidar154"]
        lidar_array[155] = data["lidar155"]
        lidar_array[156] = data["lidar156"]
        lidar_array[157] = data["lidar157"]
        lidar_array[158] = data["lidar158"]
        lidar_array[159] = data["lidar159"]
        lidar_array[160] = data["lidar160"]
        lidar_array[161] = data["lidar161"]
        lidar_array[162] = data["lidar162"]
        lidar_array[163] = data["lidar163"]
        lidar_array[164] = data["lidar164"]
        lidar_array[165] = data["lidar165"]
        lidar_array[166] = data["lidar166"]
        lidar_array[167] = data["lidar167"]
        lidar_array[168] = data["lidar168"]
        lidar_array[169] = data["lidar169"]
        lidar_array[170] = data["lidar170"]
        lidar_array[171] = data["lidar171"]
        lidar_array[172] = data["lidar172"]
        lidar_array[173] = data["lidar173"]
        lidar_array[174] = data["lidar174"]
        lidar_array[175] = data["lidar175"]
        lidar_array[176] = data["lidar176"]
        lidar_array[177] = data["lidar177"]
        lidar_array[178] = data["lidar178"]
        lidar_array[179] = data["lidar179"]
        lidar_array[180] = data["lidar180"]
        lidar_array[181] = data["lidar181"]
        lidar_array[182] = data["lidar182"]
        lidar_array[183] = data["lidar183"]
        lidar_array[184] = data["lidar184"]
        lidar_array[185] = data["lidar185"]
        lidar_array[186] = data["lidar186"]
        lidar_array[187] = data["lidar187"]
        lidar_array[188] = data["lidar188"]
        lidar_array[189] = data["lidar189"]
        lidar_array[190] = data["lidar190"]
        lidar_array[191] = data["lidar191"]
        lidar_array[192] = data["lidar192"]
        lidar_array[193] = data["lidar193"]
        lidar_array[194] = data["lidar194"]
        lidar_array[195] = data["lidar195"]
        lidar_array[196] = data["lidar196"]
        lidar_array[197] = data["lidar197"]
        lidar_array[198] = data["lidar198"]
        lidar_array[199] = data["lidar199"]
        lidar_array[200] = data["lidar200"]
        lidar_array[201] = data["lidar201"]
        lidar_array[202] = data["lidar202"]
        lidar_array[203] = data["lidar203"]
        lidar_array[204] = data["lidar204"]
        lidar_array[205] = data["lidar205"]
        lidar_array[206] = data["lidar206"]
        lidar_array[207] = data["lidar207"]
        lidar_array[208] = data["lidar208"]
        lidar_array[209] = data["lidar209"]
        lidar_array[210] = data["lidar210"]
        lidar_array[211] = data["lidar211"]
        lidar_array[212] = data["lidar212"]
        lidar_array[213] = data["lidar213"]
        lidar_array[214] = data["lidar214"]
        lidar_array[215] = data["lidar215"]
        lidar_array[216] = data["lidar216"]
        lidar_array[217] = data["lidar217"]
        lidar_array[218] = data["lidar218"]
        lidar_array[219] = data["lidar219"]
        lidar_array[220] = data["lidar220"]
        lidar_array[221] = data["lidar221"]
        lidar_array[222] = data["lidar222"]
        lidar_array[223] = data["lidar223"]
        lidar_array[224] = data["lidar224"]
        lidar_array[225] = data["lidar225"]
        lidar_array[226] = data["lidar226"]
        lidar_array[227] = data["lidar227"]
        lidar_array[228] = data["lidar228"]
        lidar_array[229] = data["lidar229"]
        lidar_array[230] = data["lidar230"]
        lidar_array[231] = data["lidar231"]
        lidar_array[232] = data["lidar232"]
        lidar_array[233] = data["lidar233"]
        lidar_array[234] = data["lidar234"]
        lidar_array[235] = data["lidar235"]
        lidar_array[236] = data["lidar236"]
        lidar_array[237] = data["lidar237"]
        lidar_array[238] = data["lidar238"]
        lidar_array[239] = data["lidar239"]
        lidar_array[240] = data["lidar240"]
        lidar_array[241] = data["lidar241"]
        lidar_array[242] = data["lidar242"]
        lidar_array[243] = data["lidar243"]
        lidar_array[244] = data["lidar244"]
        lidar_array[245] = data["lidar245"]
        lidar_array[246] = data["lidar246"]
        lidar_array[247] = data["lidar247"]
        lidar_array[248] = data["lidar248"]
        lidar_array[249] = data["lidar249"]
        lidar_array[250] = data["lidar250"]
        lidar_array[251] = data["lidar251"]
        lidar_array[252] = data["lidar252"]
        lidar_array[253] = data["lidar253"]
        lidar_array[254] = data["lidar254"]
        lidar_array[255] = data["lidar255"]
        lidar_array[256] = data["lidar256"]
        lidar_array[257] = data["lidar257"]
        lidar_array[258] = data["lidar258"]
        lidar_array[259] = data["lidar259"]
        lidar_array[260] = data["lidar260"]
        lidar_array[261] = data["lidar261"]
        lidar_array[262] = data["lidar262"]
        lidar_array[263] = data["lidar263"]
        lidar_array[264] = data["lidar264"]
        lidar_array[265] = data["lidar265"]
        lidar_array[266] = data["lidar266"]
        lidar_array[267] = data["lidar267"]
        lidar_array[268] = data["lidar268"]
        lidar_array[269] = data["lidar269"]
        lidar_array[270] = data["lidar270"]
        lidar_array[271] = data["lidar271"]
        lidar_array[272] = data["lidar272"]
        lidar_array[273] = data["lidar273"]
        lidar_array[274] = data["lidar274"]
        lidar_array[275] = data["lidar275"]
        lidar_array[276] = data["lidar276"]
        lidar_array[277] = data["lidar277"]
        lidar_array[278] = data["lidar278"]
        lidar_array[279] = data["lidar279"]
        lidar_array[280] = data["lidar280"]
        lidar_array[281] = data["lidar281"]
        lidar_array[282] = data["lidar282"]
        lidar_array[283] = data["lidar283"]
        lidar_array[284] = data["lidar284"]
        lidar_array[285] = data["lidar285"]
        lidar_array[286] = data["lidar286"]
        lidar_array[287] = data["lidar287"]
        lidar_array[288] = data["lidar288"]
        lidar_array[289] = data["lidar289"]
        lidar_array[290] = data["lidar290"]
        lidar_array[291] = data["lidar291"]
        lidar_array[292] = data["lidar292"]
        lidar_array[293] = data["lidar293"]
        lidar_array[294] = data["lidar294"]
        lidar_array[295] = data["lidar295"]
        lidar_array[296] = data["lidar296"]
        lidar_array[297] = data["lidar297"]
        lidar_array[298] = data["lidar298"]
        lidar_array[299] = data["lidar299"]
        lidar_array[300] = data["lidar300"]
        lidar_array[301] = data["lidar301"]
        lidar_array[302] = data["lidar302"]
        lidar_array[303] = data["lidar303"]
        lidar_array[304] = data["lidar304"]
        lidar_array[305] = data["lidar305"]
        lidar_array[306] = data["lidar306"]
        lidar_array[307] = data["lidar307"]
        lidar_array[308] = data["lidar308"]
        lidar_array[309] = data["lidar309"]
        lidar_array[310] = data["lidar310"]
        lidar_array[311] = data["lidar311"]
        lidar_array[312] = data["lidar312"]
        lidar_array[313] = data["lidar313"]
        lidar_array[314] = data["lidar314"]
        lidar_array[315] = data["lidar315"]
        lidar_array[316] = data["lidar316"]
        lidar_array[317] = data["lidar317"]
        lidar_array[318] = data["lidar318"]
        lidar_array[319] = data["lidar319"]
        lidar_array[320] = data["lidar320"]
        lidar_array[321] = data["lidar321"]
        lidar_array[322] = data["lidar322"]
        lidar_array[323] = data["lidar323"]
        lidar_array[324] = data["lidar324"]
        lidar_array[325] = data["lidar325"]
        lidar_array[326] = data["lidar326"]
        lidar_array[327] = data["lidar327"]
        lidar_array[328] = data["lidar328"]
        lidar_array[329] = data["lidar329"]
        lidar_array[330] = data["lidar330"]
        lidar_array[331] = data["lidar331"]
        lidar_array[332] = data["lidar332"]
        lidar_array[333] = data["lidar333"]
        lidar_array[334] = data["lidar334"]
        lidar_array[335] = data["lidar335"]
        lidar_array[336] = data["lidar336"]
        lidar_array[337] = data["lidar337"]
        lidar_array[338] = data["lidar338"]
        lidar_array[339] = data["lidar339"]
        lidar_array[340] = data["lidar340"]
        lidar_array[341] = data["lidar341"]
        lidar_array[342] = data["lidar342"]
        lidar_array[343] = data["lidar343"]
        lidar_array[344] = data["lidar344"]
        lidar_array[345] = data["lidar345"]
        lidar_array[346] = data["lidar346"]
        lidar_array[347] = data["lidar347"]
        lidar_array[348] = data["lidar348"]
        lidar_array[349] = data["lidar349"]
        lidar_array[350] = data["lidar350"]
        lidar_array[351] = data["lidar351"]
        lidar_array[352] = data["lidar352"]
        lidar_array[353] = data["lidar353"]
        lidar_array[354] = data["lidar354"]
        lidar_array[355] = data["lidar355"]
        lidar_array[356] = data["lidar356"]
        lidar_array[357] = data["lidar357"]
        lidar_array[358] = data["lidar358"]
        lidar_array[359] = data["lidar359"]

        ax1.clear()
        ax2.clear()
        ax3.clear()
        ax4.clear()
        ax5.clear()
        ax6.clear()
        ax7.clear()
        ax8.clear()
        ax9.clear()
        ax10.clear()
        ax11.clear()
        ax12.clear()
        ax13.clear()
        ax14.clear()
        ax15.clear()
        ax16.clear()
        ax25.clear()
        ax1.set_title("EKF Augmented with LiDAR")
        ax1.set_xlabel("X - coordinate [mm]")
        ax1.set_ylabel("Y - coordinate [mm]")
        ax2.set_title("EKF")
        ax2.set_xlabel("X - coordinate [mm]")
        ax2.set_ylabel("Y - coordinate [mm]")
        ax10.set_title("Recursive Monolateration")
        ax10.set_xlabel("X - coordinate [mm]")
        ax10.set_ylabel("Y - coordinate [mm]")
        ax11.set_title("EKF bank")
        ax11.set_xlabel("X - coordinate [mm]")
        ax11.set_ylabel("Y - coordinate [mm]")
        ax21.set_title("EKF Augmented with LiDAR")
        ax21.set_xlabel("X - coordinate [mm]")
        ax21.set_ylabel("Y - coordinate [mm]")
        ax22.set_title("EKF")
        ax22.set_xlabel("X - coordinate [mm]")
        ax22.set_ylabel("Y - coordinate [mm]")
        ax23.set_title("Recursive Monolateration")
        ax23.set_xlabel("X - coordinate [mm]")
        ax23.set_ylabel("Y - coordinate [mm]")
        ax24.set_title("EKF bank")
        ax24.set_xlabel("X - coordinate [mm]")
        ax24.set_ylabel("Y - coordinate [mm]")
        ax3.set_title("Position difference over time")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("Difference between position[mm]")
        ax4.set_title("Both positions in the same plot")
        ax4.set_xlabel("X - coordinate [mm]")
        ax4.set_ylabel("Y - coordinate [mm]")
        ax6.set_title("X coordinate difference with_lidar - without_lidar")
        ax6.set_xlabel("Time [s]")
        ax6.set_ylabel("Difference between position[mm]")
        ax7.set_title("Y coordinate difference with_lidar - without_lidar")
        ax7.set_xlabel("Time [s]")
        ax7.set_ylabel("Difference between position[mm]")
        ax8.set_title("X coordinate difference with_lidar - kalman_bank")
        ax8.set_xlabel("Time [s]")
        ax8.set_ylabel("Difference between position[mm]")
        ax9.set_title("Y coordinate difference with_lidar - kalman_bank")
        ax9.set_xlabel("Time [s]")
        ax9.set_ylabel("Difference between position[mm]")
        ax12.set_title("X coordinate difference with_lidar -")
        ax12.set_xlabel("Time [s]")
        ax12.set_ylabel("Difference between position[mm]")
        ax13.set_title("Y coordinate difference over time")
        ax13.set_xlabel("Time [s]")
        ax13.set_ylabel("Difference between position[mm]")
        ax14.set_title("velocity over time")
        ax14.set_xlabel("Time [s]")
        ax14.set_ylabel("speed [mm/s]")
        ax15.set_title("X over time")
        ax15.set_xlabel("Time [s]")
        ax15.set_ylabel("X coordinate [mm]")
        ax16.set_title("Y over time")
        ax16.set_xlabel("Time [s]")
        ax16.set_ylabel("Y coordinate [mm]")
        ax17.set_title("X")
        ax17.set_xlabel("Time [s]")
        ax17.set_ylabel("pose component[mm]")
        ax18.set_title("Y")
        ax18.set_xlabel("Time [s]")
        ax18.set_ylabel("pose component[mm]")
        ax19.set_title("Delta error between slam and other systems")
        ax19.set_xlabel("Time [s]")
        ax19.set_ylabel("Delta error[mm]")
        ax20.set_title("Delta error between slam and other systems")
        ax20.set_xlabel("Time [s]")
        ax20.set_ylabel("Delta error[mm]")
        ax21.set_title("EKF Augmented with LiDAR")
        ax21.set_xlabel("X - coordinate [mm]")
        ax21.set_ylabel("Y - coordinate [mm]")
        ax22.set_title("EKF")
        ax22.set_xlabel("X - coordinate [mm]")
        ax22.set_ylabel("Y - coordinate [mm]")
        ax23.set_title("Recursive Monolateration")
        ax23.set_xlabel("X - coordinate [mm]")
        ax23.set_ylabel("Y - coordinate [mm]")
        ax24.set_title("EKF bank")
        ax24.set_xlabel("X - coordinate [mm]")
        ax24.set_ylabel("Y - coordinate [mm]")

        ax1.scatter(with_x, with_y)
        ax1.plot(Xsg_x, Xsg_y, '-o', c='r', label='True path')
        ax2.scatter(without_x, without_y)
        ax2.plot(Xsg_x, Xsg_y, '-o', c='r', label='True path')
        ax10.scatter(simple_x, simple_y)
        ax10.plot(Xsg_x, Xsg_y, '-o', c='r', label='True path')
        ax11.scatter(multi_x, multi_y)
        ax11.plot(Xsg_x, Xsg_y, '-o', c='r', label='True path')
        for b in range(101,201):
            lidar_mean_x_array[b-101] = without_x[b]
            lidar_mean_y_array[b-101] = without_y[b]

        kalman_mean_x = np.mean(lidar_mean_x_array)
        kalman_mean_y = np.mean(lidar_mean_y_array)


        #print(kalman_mean_x)

        for i in range(len(with_x)):
            delta[i] = math.sqrt(pow(with_x[i],2)+pow(with_y[i],2))-math.sqrt(pow(without_x[i],2)+pow(without_y[i],2))
            delta_x[i] = with_x[i] - without_x[i]
            delta_y[i] = with_y[i] - without_y[i]
            delta_x_kalman_bank[i] = with_x[i] - multi_x[i]
            delta_y_kalman_bank[i] = with_y[i] - multi_y[i]
            delta_x_simple[i] = with_x[i] - multi_x[i]
            delta_y_simple[i] = with_y[i] - multi_y[i]
            vel_input[i] = linear_speed[i]
            vel_with[i] = with_v[i]
            vel_without[i] = without_v[i]
            kalman_x[i] = without_x[i]
            kalman_y[i] = without_y[i]
            kalman_x_lidar[i] = with_x[i]
            kalman_y_lidar[i] = with_y[i]
            kalman_x_bank[i] = multi_x[i]
            kalman_y_bank[i] = multi_y[i]
            simple_x_data[i] = simple_x[i]
            simple_y_data[i] = simple_y[i]
            if number_of_files > 19:
                if i > 100:
                    if kalman_mean_y < 3000:
                        slam_x_time[i-101] =  slam_x[i] #* 1000# + kalman_mean_y
                        slam_y_time[i-101] =  slam_y[i] #* 1000# + kalman_mean_x
                    if kalman_mean_y > 3000:
                        slam_x_time[i-101] =  slam_x[i] #* 1000# + kalman_mean_y
                        slam_y_time[i-101] =  slam_y[i] #* 1000# + kalman_mean_x

                    lidar_kalman_slam_x_time[i-101] = with_x[i]
                    lidar_kalman_slam_y_time[i-101] = with_y[i]
                    kalman_slam_x_time[i-101] = without_x[i]
                    kalman_slam_y_time[i-101] = without_y[i]

                    kalman_x_bank_slam[i-101] = multi_x[i]
                    kalman_y_bank_slam[i-101] = multi_y[i]
                    simple_x_data_slam[i-101] = simple_x[i]
                    simple_y_data_slam[i-101] = simple_y[i]





        #eq1 = Eq((a11*np.sum(slam_x_time)*np.sum(slam_x_time)+a12*np.sum(slam_x_time)*np.sum(slam_y_time))+b1*np.sum(slam_x_time), np.sum(kalman_slam_x_time)*np.sum(slam_x_time))
        #eq2 = Eq(a11*np.sum(slam_x_time)*np.sum(slam_y_time)+a12*np.sum(slam_y_time)*np.sum(slam_y_time)+b1*np.sum(slam_y_time),np.sum(kalman_slam_x_time)*np.sum(slam_y_time))
        #eq3 = Eq(a11*np.sum(slam_x_time)+a12*np.sum(slam_y_time)+b1*len(slam_y_time), np.sum(kalman_slam_x_time))

        #eq4 = Eq((a21*pow(np.sum(slam_x_time),2)+a22*np.sum(slam_x_time)*np.sum(slam_y_time))+b2*np.sum(slam_x_time), np.sum(kalman_slam_y_time)*np.sum(slam_x_time))
        #eq5 = Eq(a21*np.sum(slam_x_time)*np.sum(slam_y_time)+a22*pow(np.sum(slam_y_time),2)+b2*np.sum(slam_y_time),np.sum(kalman_slam_y_time)*np.sum(slam_y_time))
        #eq6 = Eq(a21*np.sum(slam_x_time)+a22*np.sum(slam_y_time)+b2*len(slam_y_time), np.sum(kalman_slam_y_time))

        #a = solve((eq1,eq2,eq3,eq4,eq5,eq6),(a11, a12, a21, a22, b1, b2))

        #a_11 =  float(a.get(a11))
        #a_12 =  float(a.get(a12))
        #a_21 =  float(a.get(a21))
        #a_22 =  float(a.get(a22))
        #b_1 =  float(a.get(b1))
        #b_2 =  float(a.get(b2))
        #print("the type is ",type(a_11))

        #for d in range(len(slam_x_time)):
            #slam_x_time_tans[d] = a_11 * slam_x_time[d] + a_12 * slam_y_time[d] + b_1
            #slam_y_time_tans[d] = a_21 * slam_x_time[d] + a_22 * slam_y_time[d] + b_2
            sx = np.array([slam_x_time]).T
            sy = np.array([slam_y_time]).T
            gx = np.array([kalman_slam_x_time]).T
            gy = np.array([kalman_slam_y_time]).T





        A = [[float(np.dot(sx.T,sx)), float(np.dot(sx.T,sy)), float(np.sum(sx))],
            [float(np.dot(sx.T,sy)), float(np.dot(sy.T,sy)), float(np.sum(sy))],
            [float(np.sum(sy)), float(np.sum(sy)), float(len(sx))]]
        B1 = [[float(np.dot(gx.T, sx))],[float(np.dot(gx.T,sy))],[float(np.sum(gx))]]
        B2 = [[float(np.dot(gy.T, sx))],[float(np.dot(gy.T,sy))],[float(np.sum(gy))]]

        X1 = np.linalg.solve(A,B1)
        X2 = np.linalg.solve(A,B2)

        AT = [[X1[0][0], X1[1][0]],[X2[0][0], X2[1][0]]]
        BT = [[X1[2][0]],[X2[2][0]]]
        for h in range(len(sy)):
            Xsg = np.dot(AT,[[float(sx[h][0])],[float(sy[h][0])]])+BT
            Xsg_x[h] = Xsg[0][0]
            Xsg_y[h] = Xsg[1][0]


        print(Xsg[1][0])


        for k in range(len(slam_x_time)-1):
            slam_delta = math.sqrt(pow(Xsg_x[k+1]-Xsg_x[k],2)+pow(Xsg_y[k+1]-Xsg_y[k],2))
            lidar_delta = math.sqrt(pow(lidar_kalman_slam_x_time[k+1]-lidar_kalman_slam_x_time[k],2)+pow(lidar_kalman_slam_y_time[k+1]-lidar_kalman_slam_y_time[k],2))
            kalman_delta = math.sqrt(pow(kalman_slam_x_time[k+1]-kalman_slam_x_time[k],2)+pow(kalman_slam_y_time[k+1]-kalman_slam_y_time[k],2))

            lidar_delta_l[k] = lidar_delta - slam_delta
            kalman_delta_l[k] = kalman_delta - slam_delta



        ax3.plot(time_array,delta)
        ax6.plot(time_array,delta_x)
        ax7.plot(time_array,delta_y)
        ax8.plot(time_array,delta_x_kalman_bank)
        ax9.plot(time_array,delta_y_kalman_bank)
        ax12.plot(time_array,delta_x_kalman_bank)
        ax13.plot(time_array,delta_y_kalman_bank)
        ax4.scatter(with_x, with_y, c='b',label='Kalman w. Lidar')
        ax4.scatter(without_x, without_y, c='g',label='Kalman')
        ax4.plot(Xsg_x, Xsg_y, '-o', c='r', label='True path')
        #ax14.plot(time_array,vel_input,c='b')
        ax14.plot(time_array, vel_with, c='g',label='Kalman w. Lidar')
        ax14.plot(time_array, vel_without, c='r',label='Kalman')
        ax15.plot(time_array, kalman_x,c='b',label='Kalman')
        ax15.plot(time_array, kalman_x_lidar, c='g',label='Kalman w. Lidar')
        #ax15.plot(time_array, kalman_x_bank, c='r', label='Kalman Bank')
        #ax15.plot(time_array, simple_x_data, c='c', label='recursive monolateration')
        ax16.plot(time_array, kalman_y,c='b',label='Kalman')
        ax16.plot(time_array, kalman_y_lidar, c='g',label='Kalman w. Lidar')
        #ax16.plot(time_array, kalman_y_bank, c='r', label='Kalman Bank')
        #ax16.plot(time_array, simple_y_data, c='c',label='recursive monolateration')


        ax4.legend()
        ax14.legend()
        ax15.legend()
        ax16.legend()
        ax1.legend()
        ax2.legend()
        ax10.legend()
        ax11.legend()


        frames_per_figure = 1
        def animate(i):
            ax5.clear()
            ax5.set_xlim(-12,12)
            ax5.set_ylim(-12,12)
            i = int(i/frames_per_figure)
            for l in range(360):
                ax5.scatter(math.cos(math.radians(l)) * lidar_array[l][i], math.sin(math.radians(l)) * lidar_array[l][i], c='r')
        ani = FuncAnimation(fig4, animate, frames = frames_per_figure * len(data["lidar0"]))

        ax1.set_xlim(0,45000)
        ax1.set_ylim(0,12000)
        ax2.set_xlim(0,45000)
        ax2.set_ylim(0,12000)
        ax4.set_xlim(0,45000)
        ax4.set_ylim(0,12000)
        ax10.set_xlim(0,45000)
        ax10.set_ylim(0,12000)
        ax11.set_xlim(0,45000)
        ax11.set_ylim(0,12000)
        if number_of_files >= 0 or number_of_files <= 19:
            name_of_file_1 = 'LoS_dyn_position_plot%s.png'
            name_of_file_2 = 'LoS_dyn_position_delta%s.png'
            name_of_file_3 = 'LoS_dyn_position_in_the_same_plot%s.png'
            name_of_file_4 = 'LoS_dyn_lidar%s.gif'
            name_of_file_5 = 'LoS_dyn_delta_coordinate_w_respect_to_with_lidar%s.png'
            name_of_file_6 = 'LoS_dyn_vel_over%s.png'
            name_of_file_7 = 'LoS_dyn_coordinate_over_time%s.png'
        if number_of_files > 19:
            name_of_file_1 = '1_new_data/LoS_dyn_position_plot%s.png'
            name_of_file_2 = '1_new_data/LoS_dyn_position_delta%s.png'
            name_of_file_3 = '1_new_data/LoS_dyn_position_in_the_same_plot%s.png'
            name_of_file_4 = '1_new_data/LoS_dyn_lidar%s.gif'
            name_of_file_5 = '1_new_data/LoS_dyn_delta_coordinate_w_respect_to_with_lidar%s.png'
            name_of_file_6 = '1_new_data/LoS_dyn_vel_over%s.png'
            name_of_file_7 = '1_new_data/LoS_dyn_coordinate_over_time%s.png'



        fig1.savefig(output_path + name_of_file_1 % number_of_files)
        #fig2.savefig(output_path + name_of_file_2 % number_of_files)
        fig3.savefig(output_path + name_of_file_3 % number_of_files)
        #fig5.savefig(output_path + name_of_file_5 % number_of_files)
        #fig6.savefig(output_path + name_of_file_6 % number_of_files)
        fig7.savefig(output_path + name_of_file_7 % number_of_files)

        if number_of_files > 19:
            print(len(time_slam), len(slam_x_time))
            ax17.clear()
            ax18.clear()
            ax19.clear()
            ax20.clear()
            ax21.clear()
            ax22.clear()
            ax23.clear()
            ax24.clear()
            ax17.plot(time_slam, Xsg_x,c='r',label='slam_y')
            ax17.plot(time_slam, kalman_slam_x_time,c='b',label='Kalman')
            ax17.plot(time_slam, lidar_kalman_slam_x_time,c='g',label='Lidar aug')
            ax18.plot(time_slam, Xsg_y, c='r',label='slam_x')
            ax18.plot(time_slam, kalman_slam_y_time, c='b',label='Kalman')
            ax18.plot(time_slam, lidar_kalman_slam_y_time, c='g',label='Lidar aug')
            ax19.plot(time_slam_l, lidar_delta_l,c='g',label='lidar')
            ax20.plot(time_slam_l, kalman_delta_l,c='b',label='kalman')
            ax21.scatter(lidar_kalman_slam_x_time, lidar_kalman_slam_y_time)
            ax21.plot(Xsg_x, Xsg_y, '-o', c='r', label='SLAM path')
            ax22.scatter(kalman_slam_x_time, kalman_slam_y_time)
            ax22.plot(Xsg_x, Xsg_y, '-o', c='r', label='SLAM path')
            ax23.scatter(simple_x_data_slam, simple_y_data_slam)
            ax23.plot(Xsg_x, Xsg_y, '-o', c='r', label='SLAM path')
            ax24.scatter(kalman_x_bank_slam, kalman_y_bank_slam)
            ax24.plot(Xsg_x, Xsg_y, '-o', c='r', label='SLAM path')
            ax25.hist(kalman_delta_l)


            ax17.legend()
            ax18.legend()
            ax19.legend()
            ax20.legend()
            ax21.legend()
            ax22.legend()
            ax23.legend()
            ax24.legend()
            name_of_file_8 = '1_new_data/LoS_slam_time%s.png'
            name_of_file_9 = '1_new_data/LoS_error_time%s.png'
            name_of_file_10 = '1_new_data/LoS_pose_with_slam_added%s.png'
            name_of_file_11 = '1_new_data/LoS_hist%s.png'

            fig8.savefig(output_path + name_of_file_8 % number_of_files)
            fig9.savefig(output_path + name_of_file_9 % number_of_files)
            fig10.savefig(output_path + name_of_file_10 % number_of_files)
            fig11.savefig(output_path + name_of_file_11 % number_of_files)
        if number_of_files == 1:
            writergif = animation.PillowWriter(fps=10)
            #ani.save(output_path + name_of_file_4 % number_of_files, writer=writergif)
        #plt.show()

        w_x_variance = np.var(with_x)
        w_x_std = np.std(with_x)
        w_x_mean = np.mean(with_x)
        w_y_variance = np.var(with_y)
        w_y_std = np.std(with_y)
        w_y_mean = np.mean(with_y)

        wo_x_variance = np.var(without_x)
        wo_x_std = np.std(without_x)
        wo_x_mean = np.mean(without_x)
        wo_y_variance = np.var(without_y)
        wo_y_std = np.std(without_y)
        wo_y_mean = np.mean(without_y)

        simple_x_variance = np.var(simple_x)
        simple_x_std = np.std(simple_x)
        simple_x_mean = np.mean(simple_x)
        simple_y_variance = np.var(simple_y)
        simple_y_std = np.std(simple_y)
        simple_y_mean = np.mean(simple_y)

        multi_x_variance = np.var(multi_x)
        multi_x_std = np.std(multi_x)
        multi_x_mean = np.mean(multi_x)
        multi_y_variance = np.var(multi_y)
        multi_y_std = np.std(multi_y)
        multi_y_mean = np.mean(multi_y)

        fieldnames = ["variance_w_x","std_w_x","mean_w_x","variance_w_y","std_w_y","mean_w_y","variance_wo_x","std_wo_x","mean_wo_x","variance_wo_y","std_wo_y","mean_wo_y","variance_simple_x","std_simple_x","mean_simple_x","variance_simple_y","std_simple_y","mean_simple_y","variance_multi_x","std_multi_x","mean_multi_x","variance_multi_y","std_multi_y","mean_multi_y"]
        if os.path.exists(path_out) == False:
            with open(path_out, 'w') as csv_file:
                csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                csv_writer.writeheader()
                pass
        with open(path_out, 'a') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            info = {
                "variance_w_x": w_x_variance,
                "std_w_x": w_x_std,
                "mean_w_x": w_x_mean,
                "variance_w_y": w_y_variance,
                "std_w_y": w_y_std,
                "mean_w_y": w_y_mean,
                "variance_wo_x": wo_x_variance,
                "std_wo_x": wo_x_std,
                "mean_wo_x": wo_x_mean,
                "variance_wo_y": wo_y_variance,
                "std_wo_y": wo_y_std,
                "mean_wo_y": wo_y_mean,
                "variance_simple_x": simple_x_variance,
                "std_simple_x": simple_x_std,
                "mean_simple_x": simple_x_mean,
                "variance_simple_y": simple_y_variance,
                "std_simple_y": simple_y_std,
                "mean_simple_y": simple_y_mean,
                "variance_multi_x": multi_x_variance,
                "std_multi_x": multi_x_std,
                "mean_multi_x": multi_x_mean,
                "variance_multi_y": multi_y_variance,
                "std_multi_y": multi_y_std,
                "mean_multi_y": multi_y_mean

                }
            csv_writer.writerow(info)

        number_of_files = number_of_files + 1
    print("im done baby ;)")



if __name__ == '__main__':
    main()
