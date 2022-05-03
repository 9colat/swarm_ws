#!/usr/bin/env python3
import rospy
import csv
import pandas as pd
from std_msgs.msg import Bool
from pathlib import Path
from custom_msgs.msg import odom_and_imu
from custom_msgs.msg import USPS_msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan









def main():
    path = str(Path.home().joinpath("test_data", "static_log%s.csv"))
    local_path = path %  1
    data = pd.read_csv(local_path)
    lidar_array = [[0] * len(data["lidar0"])]*360
    delta = [0] * len(data["lidar0"])
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
    rospy.init_node('ofline_data', anonymous=True)
    pub = rospy.Publisher('terminating_signal', Bool, queue_size=10)
    pub_laser = rospy.Publisher('scan', LaserScan, queue_size=10)
    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    pub_beacon = rospy.Publisher('beacon_data', USPS_msgs, queue_size=10)
    pub_odom = rospy.Publisher('odometry_and_IMU', odom_and_imu, queue_size=10)
    rate = rospy.Rate(5)
    i = 0
    lidar_array = [[0] * len(data["lidar0"])]*360
    while not rospy.is_shutdown():
        if i == len(lidar_array[0]):
            i = 0
        term_sig = Bool()
        laser_array = LaserScan()
        cmd_input = Twist()
        beacon = USPS_msgs()
        odom = odom_and_imu()

        #for j in range(len(lidar_array)):
        #    print(laser_array.header)
        #    lidar_array[0][0] = laser_array.ranges[0]
        #    print(laser_array.ranges[j])
        cmd_input.linear.x = linear_speed[i]
        cmd_input.angular.z = angular_speed[i]
        beacon.ID = ID[i]
        beacon.distance = distance[i]
        odom.imu_acc.x = acc_x[i]
        odom.imu_acc.y = acc_y[i]
        odom.imu_gyro.x = gyro_x[i]
        odom.imu_gyro.y = gyro_y[i]
        odom.imu_gyro.z = gyro_z[i]

        #print(acc_x)
        #pub_cmd_vel.publish(cmd_input)
        pub_beacon.publish(beacon)
        #pub_odom.publish(odom)
        i = i + 1
        rate.sleep()



if __name__ == '__main__':
    main()