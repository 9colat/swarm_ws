import math
import csv
import os
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


path_NLoS = str(Path.home().joinpath("test_data/1_filtered_data", "filtered_NLoS_dyn_log%s.csv"))
path_LoS = str(Path.home().joinpath("test_data/1_filtered_data", "filtered_LoS_dyn_log%s.csv"))
output_path = str(Path.home()) + '/' + "figure/1_filtered_data/1_lidar/"
plt.style.use('fivethirtyeight')
fig1 = plt.figure(figsize=(10,10))
fig1.suptitle('LoS Lidar plot')
ax1 = fig1.add_subplot(1,1,1)
fig2 = plt.figure(figsize=(10,10))
fig2.suptitle('NLoS Lidar plot')
ax2 = fig2.add_subplot(1,1,1)

number_of_files_LoS = 0
number_of_files_NLoS = 0

while os.path.exists(path_LoS % number_of_files_LoS):
    local_path = path_LoS %  number_of_files_LoS
    data_LoS = pd.read_csv(local_path)
    lidar_array = [[0] * len(data_LoS["lidar0"])]*360

    lidar_array[0] = data_LoS["lidar0"]
    lidar_array[1] = data_LoS["lidar1"]
    lidar_array[2] = data_LoS["lidar2"]
    lidar_array[3] = data_LoS["lidar3"]
    lidar_array[4] = data_LoS["lidar4"]
    lidar_array[5] = data_LoS["lidar5"]
    lidar_array[6] = data_LoS["lidar6"]
    lidar_array[7] = data_LoS["lidar7"]
    lidar_array[8] = data_LoS["lidar8"]
    lidar_array[9] = data_LoS["lidar9"]
    lidar_array[10] = data_LoS["lidar10"]
    lidar_array[11] = data_LoS["lidar11"]
    lidar_array[12] = data_LoS["lidar12"]
    lidar_array[13] = data_LoS["lidar13"]
    lidar_array[14] = data_LoS["lidar14"]
    lidar_array[15] = data_LoS["lidar15"]
    lidar_array[16] = data_LoS["lidar16"]
    lidar_array[17] = data_LoS["lidar17"]
    lidar_array[18] = data_LoS["lidar18"]
    lidar_array[19] = data_LoS["lidar19"]
    lidar_array[20] = data_LoS["lidar20"]
    lidar_array[21] = data_LoS["lidar21"]
    lidar_array[22] = data_LoS["lidar22"]
    lidar_array[23] = data_LoS["lidar23"]
    lidar_array[24] = data_LoS["lidar24"]
    lidar_array[25] = data_LoS["lidar25"]
    lidar_array[26] = data_LoS["lidar26"]
    lidar_array[27] = data_LoS["lidar27"]
    lidar_array[28] = data_LoS["lidar28"]
    lidar_array[29] = data_LoS["lidar29"]
    lidar_array[30] = data_LoS["lidar30"]
    lidar_array[31] = data_LoS["lidar31"]
    lidar_array[32] = data_LoS["lidar32"]
    lidar_array[33] = data_LoS["lidar33"]
    lidar_array[34] = data_LoS["lidar34"]
    lidar_array[35] = data_LoS["lidar35"]
    lidar_array[36] = data_LoS["lidar36"]
    lidar_array[37] = data_LoS["lidar37"]
    lidar_array[38] = data_LoS["lidar38"]
    lidar_array[39] = data_LoS["lidar39"]
    lidar_array[40] = data_LoS["lidar40"]
    lidar_array[41] = data_LoS["lidar41"]
    lidar_array[42] = data_LoS["lidar42"]
    lidar_array[43] = data_LoS["lidar43"]
    lidar_array[44] = data_LoS["lidar44"]
    lidar_array[45] = data_LoS["lidar45"]
    lidar_array[46] = data_LoS["lidar46"]
    lidar_array[47] = data_LoS["lidar47"]
    lidar_array[48] = data_LoS["lidar48"]
    lidar_array[49] = data_LoS["lidar49"]
    lidar_array[50] = data_LoS["lidar50"]
    lidar_array[51] = data_LoS["lidar51"]
    lidar_array[52] = data_LoS["lidar52"]
    lidar_array[53] = data_LoS["lidar53"]
    lidar_array[54] = data_LoS["lidar54"]
    lidar_array[55] = data_LoS["lidar55"]
    lidar_array[56] = data_LoS["lidar56"]
    lidar_array[57] = data_LoS["lidar57"]
    lidar_array[58] = data_LoS["lidar58"]
    lidar_array[59] = data_LoS["lidar59"]
    lidar_array[60] = data_LoS["lidar60"]
    lidar_array[61] = data_LoS["lidar61"]
    lidar_array[62] = data_LoS["lidar62"]
    lidar_array[63] = data_LoS["lidar63"]
    lidar_array[64] = data_LoS["lidar64"]
    lidar_array[65] = data_LoS["lidar65"]
    lidar_array[66] = data_LoS["lidar66"]
    lidar_array[67] = data_LoS["lidar67"]
    lidar_array[68] = data_LoS["lidar68"]
    lidar_array[69] = data_LoS["lidar69"] #nice
    lidar_array[70] = data_LoS["lidar70"]
    lidar_array[71] = data_LoS["lidar71"]
    lidar_array[72] = data_LoS["lidar72"]
    lidar_array[73] = data_LoS["lidar73"]
    lidar_array[74] = data_LoS["lidar74"]
    lidar_array[75] = data_LoS["lidar75"]
    lidar_array[76] = data_LoS["lidar76"]
    lidar_array[77] = data_LoS["lidar77"]
    lidar_array[78] = data_LoS["lidar78"]
    lidar_array[79] = data_LoS["lidar79"]
    lidar_array[80] = data_LoS["lidar80"]
    lidar_array[81] = data_LoS["lidar81"]
    lidar_array[82] = data_LoS["lidar82"]
    lidar_array[83] = data_LoS["lidar83"]
    lidar_array[84] = data_LoS["lidar84"]
    lidar_array[85] = data_LoS["lidar85"]
    lidar_array[86] = data_LoS["lidar86"]
    lidar_array[87] = data_LoS["lidar87"]
    lidar_array[88] = data_LoS["lidar88"]
    lidar_array[89] = data_LoS["lidar89"]
    lidar_array[90] = data_LoS["lidar90"]
    lidar_array[91] = data_LoS["lidar91"]
    lidar_array[92] = data_LoS["lidar92"]
    lidar_array[93] = data_LoS["lidar93"]
    lidar_array[94] = data_LoS["lidar94"]
    lidar_array[95] = data_LoS["lidar95"]
    lidar_array[96] = data_LoS["lidar96"]
    lidar_array[97] = data_LoS["lidar97"]
    lidar_array[98] = data_LoS["lidar98"]
    lidar_array[99] = data_LoS["lidar99"]
    lidar_array[100] = data_LoS["lidar100"]
    lidar_array[101] = data_LoS["lidar101"]
    lidar_array[102] = data_LoS["lidar102"]
    lidar_array[103] = data_LoS["lidar103"]
    lidar_array[104] = data_LoS["lidar104"]
    lidar_array[105] = data_LoS["lidar105"]
    lidar_array[106] = data_LoS["lidar106"]
    lidar_array[107] = data_LoS["lidar107"]
    lidar_array[108] = data_LoS["lidar108"]
    lidar_array[109] = data_LoS["lidar109"]
    lidar_array[110] = data_LoS["lidar110"]
    lidar_array[111] = data_LoS["lidar111"]
    lidar_array[112] = data_LoS["lidar112"]
    lidar_array[113] = data_LoS["lidar113"]
    lidar_array[114] = data_LoS["lidar114"]
    lidar_array[115] = data_LoS["lidar115"]
    lidar_array[116] = data_LoS["lidar116"]
    lidar_array[117] = data_LoS["lidar117"]
    lidar_array[118] = data_LoS["lidar118"]
    lidar_array[119] = data_LoS["lidar119"]
    lidar_array[120] = data_LoS["lidar120"]
    lidar_array[121] = data_LoS["lidar121"]
    lidar_array[122] = data_LoS["lidar122"]
    lidar_array[123] = data_LoS["lidar123"]
    lidar_array[124] = data_LoS["lidar124"]
    lidar_array[125] = data_LoS["lidar125"]
    lidar_array[126] = data_LoS["lidar126"]
    lidar_array[127] = data_LoS["lidar127"]
    lidar_array[128] = data_LoS["lidar128"]
    lidar_array[129] = data_LoS["lidar129"]
    lidar_array[130] = data_LoS["lidar130"]
    lidar_array[131] = data_LoS["lidar131"]
    lidar_array[132] = data_LoS["lidar132"]
    lidar_array[133] = data_LoS["lidar133"]
    lidar_array[134] = data_LoS["lidar134"]
    lidar_array[135] = data_LoS["lidar135"]
    lidar_array[136] = data_LoS["lidar136"]
    lidar_array[137] = data_LoS["lidar137"]
    lidar_array[138] = data_LoS["lidar138"]
    lidar_array[139] = data_LoS["lidar139"]
    lidar_array[140] = data_LoS["lidar140"]
    lidar_array[141] = data_LoS["lidar141"]
    lidar_array[142] = data_LoS["lidar142"]
    lidar_array[143] = data_LoS["lidar143"]
    lidar_array[144] = data_LoS["lidar144"]
    lidar_array[145] = data_LoS["lidar145"]
    lidar_array[146] = data_LoS["lidar146"]
    lidar_array[147] = data_LoS["lidar147"]
    lidar_array[148] = data_LoS["lidar148"]
    lidar_array[149] = data_LoS["lidar149"]
    lidar_array[150] = data_LoS["lidar150"]
    lidar_array[151] = data_LoS["lidar151"]
    lidar_array[152] = data_LoS["lidar152"]
    lidar_array[153] = data_LoS["lidar153"]
    lidar_array[154] = data_LoS["lidar154"]
    lidar_array[155] = data_LoS["lidar155"]
    lidar_array[156] = data_LoS["lidar156"]
    lidar_array[157] = data_LoS["lidar157"]
    lidar_array[158] = data_LoS["lidar158"]
    lidar_array[159] = data_LoS["lidar159"]
    lidar_array[160] = data_LoS["lidar160"]
    lidar_array[161] = data_LoS["lidar161"]
    lidar_array[162] = data_LoS["lidar162"]
    lidar_array[163] = data_LoS["lidar163"]
    lidar_array[164] = data_LoS["lidar164"]
    lidar_array[165] = data_LoS["lidar165"]
    lidar_array[166] = data_LoS["lidar166"]
    lidar_array[167] = data_LoS["lidar167"]
    lidar_array[168] = data_LoS["lidar168"]
    lidar_array[169] = data_LoS["lidar169"]
    lidar_array[170] = data_LoS["lidar170"]
    lidar_array[171] = data_LoS["lidar171"]
    lidar_array[172] = data_LoS["lidar172"]
    lidar_array[173] = data_LoS["lidar173"]
    lidar_array[174] = data_LoS["lidar174"]
    lidar_array[175] = data_LoS["lidar175"]
    lidar_array[176] = data_LoS["lidar176"]
    lidar_array[177] = data_LoS["lidar177"]
    lidar_array[178] = data_LoS["lidar178"]
    lidar_array[179] = data_LoS["lidar179"]
    lidar_array[180] = data_LoS["lidar180"]
    lidar_array[181] = data_LoS["lidar181"]
    lidar_array[182] = data_LoS["lidar182"]
    lidar_array[183] = data_LoS["lidar183"]
    lidar_array[184] = data_LoS["lidar184"]
    lidar_array[185] = data_LoS["lidar185"]
    lidar_array[186] = data_LoS["lidar186"]
    lidar_array[187] = data_LoS["lidar187"]
    lidar_array[188] = data_LoS["lidar188"]
    lidar_array[189] = data_LoS["lidar189"]
    lidar_array[190] = data_LoS["lidar190"]
    lidar_array[191] = data_LoS["lidar191"]
    lidar_array[192] = data_LoS["lidar192"]
    lidar_array[193] = data_LoS["lidar193"]
    lidar_array[194] = data_LoS["lidar194"]
    lidar_array[195] = data_LoS["lidar195"]
    lidar_array[196] = data_LoS["lidar196"]
    lidar_array[197] = data_LoS["lidar197"]
    lidar_array[198] = data_LoS["lidar198"]
    lidar_array[199] = data_LoS["lidar199"]
    lidar_array[200] = data_LoS["lidar200"]
    lidar_array[201] = data_LoS["lidar201"]
    lidar_array[202] = data_LoS["lidar202"]
    lidar_array[203] = data_LoS["lidar203"]
    lidar_array[204] = data_LoS["lidar204"]
    lidar_array[205] = data_LoS["lidar205"]
    lidar_array[206] = data_LoS["lidar206"]
    lidar_array[207] = data_LoS["lidar207"]
    lidar_array[208] = data_LoS["lidar208"]
    lidar_array[209] = data_LoS["lidar209"]
    lidar_array[210] = data_LoS["lidar210"]
    lidar_array[211] = data_LoS["lidar211"]
    lidar_array[212] = data_LoS["lidar212"]
    lidar_array[213] = data_LoS["lidar213"]
    lidar_array[214] = data_LoS["lidar214"]
    lidar_array[215] = data_LoS["lidar215"]
    lidar_array[216] = data_LoS["lidar216"]
    lidar_array[217] = data_LoS["lidar217"]
    lidar_array[218] = data_LoS["lidar218"]
    lidar_array[219] = data_LoS["lidar219"]
    lidar_array[220] = data_LoS["lidar220"]
    lidar_array[221] = data_LoS["lidar221"]
    lidar_array[222] = data_LoS["lidar222"]
    lidar_array[223] = data_LoS["lidar223"]
    lidar_array[224] = data_LoS["lidar224"]
    lidar_array[225] = data_LoS["lidar225"]
    lidar_array[226] = data_LoS["lidar226"]
    lidar_array[227] = data_LoS["lidar227"]
    lidar_array[228] = data_LoS["lidar228"]
    lidar_array[229] = data_LoS["lidar229"]
    lidar_array[230] = data_LoS["lidar230"]
    lidar_array[231] = data_LoS["lidar231"]
    lidar_array[232] = data_LoS["lidar232"]
    lidar_array[233] = data_LoS["lidar233"]
    lidar_array[234] = data_LoS["lidar234"]
    lidar_array[235] = data_LoS["lidar235"]
    lidar_array[236] = data_LoS["lidar236"]
    lidar_array[237] = data_LoS["lidar237"]
    lidar_array[238] = data_LoS["lidar238"]
    lidar_array[239] = data_LoS["lidar239"]
    lidar_array[240] = data_LoS["lidar240"]
    lidar_array[241] = data_LoS["lidar241"]
    lidar_array[242] = data_LoS["lidar242"]
    lidar_array[243] = data_LoS["lidar243"]
    lidar_array[244] = data_LoS["lidar244"]
    lidar_array[245] = data_LoS["lidar245"]
    lidar_array[246] = data_LoS["lidar246"]
    lidar_array[247] = data_LoS["lidar247"]
    lidar_array[248] = data_LoS["lidar248"]
    lidar_array[249] = data_LoS["lidar249"]
    lidar_array[250] = data_LoS["lidar250"]
    lidar_array[251] = data_LoS["lidar251"]
    lidar_array[252] = data_LoS["lidar252"]
    lidar_array[253] = data_LoS["lidar253"]
    lidar_array[254] = data_LoS["lidar254"]
    lidar_array[255] = data_LoS["lidar255"]
    lidar_array[256] = data_LoS["lidar256"]
    lidar_array[257] = data_LoS["lidar257"]
    lidar_array[258] = data_LoS["lidar258"]
    lidar_array[259] = data_LoS["lidar259"]
    lidar_array[260] = data_LoS["lidar260"]
    lidar_array[261] = data_LoS["lidar261"]
    lidar_array[262] = data_LoS["lidar262"]
    lidar_array[263] = data_LoS["lidar263"]
    lidar_array[264] = data_LoS["lidar264"]
    lidar_array[265] = data_LoS["lidar265"]
    lidar_array[266] = data_LoS["lidar266"]
    lidar_array[267] = data_LoS["lidar267"]
    lidar_array[268] = data_LoS["lidar268"]
    lidar_array[269] = data_LoS["lidar269"]
    lidar_array[270] = data_LoS["lidar270"]
    lidar_array[271] = data_LoS["lidar271"]
    lidar_array[272] = data_LoS["lidar272"]
    lidar_array[273] = data_LoS["lidar273"]
    lidar_array[274] = data_LoS["lidar274"]
    lidar_array[275] = data_LoS["lidar275"]
    lidar_array[276] = data_LoS["lidar276"]
    lidar_array[277] = data_LoS["lidar277"]
    lidar_array[278] = data_LoS["lidar278"]
    lidar_array[279] = data_LoS["lidar279"]
    lidar_array[280] = data_LoS["lidar280"]
    lidar_array[281] = data_LoS["lidar281"]
    lidar_array[282] = data_LoS["lidar282"]
    lidar_array[283] = data_LoS["lidar283"]
    lidar_array[284] = data_LoS["lidar284"]
    lidar_array[285] = data_LoS["lidar285"]
    lidar_array[286] = data_LoS["lidar286"]
    lidar_array[287] = data_LoS["lidar287"]
    lidar_array[288] = data_LoS["lidar288"]
    lidar_array[289] = data_LoS["lidar289"]
    lidar_array[290] = data_LoS["lidar290"]
    lidar_array[291] = data_LoS["lidar291"]
    lidar_array[292] = data_LoS["lidar292"]
    lidar_array[293] = data_LoS["lidar293"]
    lidar_array[294] = data_LoS["lidar294"]
    lidar_array[295] = data_LoS["lidar295"]
    lidar_array[296] = data_LoS["lidar296"]
    lidar_array[297] = data_LoS["lidar297"]
    lidar_array[298] = data_LoS["lidar298"]
    lidar_array[299] = data_LoS["lidar299"]
    lidar_array[300] = data_LoS["lidar300"]
    lidar_array[301] = data_LoS["lidar301"]
    lidar_array[302] = data_LoS["lidar302"]
    lidar_array[303] = data_LoS["lidar303"]
    lidar_array[304] = data_LoS["lidar304"]
    lidar_array[305] = data_LoS["lidar305"]
    lidar_array[306] = data_LoS["lidar306"]
    lidar_array[307] = data_LoS["lidar307"]
    lidar_array[308] = data_LoS["lidar308"]
    lidar_array[309] = data_LoS["lidar309"]
    lidar_array[310] = data_LoS["lidar310"]
    lidar_array[311] = data_LoS["lidar311"]
    lidar_array[312] = data_LoS["lidar312"]
    lidar_array[313] = data_LoS["lidar313"]
    lidar_array[314] = data_LoS["lidar314"]
    lidar_array[315] = data_LoS["lidar315"]
    lidar_array[316] = data_LoS["lidar316"]
    lidar_array[317] = data_LoS["lidar317"]
    lidar_array[318] = data_LoS["lidar318"]
    lidar_array[319] = data_LoS["lidar319"]
    lidar_array[320] = data_LoS["lidar320"]
    lidar_array[321] = data_LoS["lidar321"]
    lidar_array[322] = data_LoS["lidar322"]
    lidar_array[323] = data_LoS["lidar323"]
    lidar_array[324] = data_LoS["lidar324"]
    lidar_array[325] = data_LoS["lidar325"]
    lidar_array[326] = data_LoS["lidar326"]
    lidar_array[327] = data_LoS["lidar327"]
    lidar_array[328] = data_LoS["lidar328"]
    lidar_array[329] = data_LoS["lidar329"]
    lidar_array[330] = data_LoS["lidar330"]
    lidar_array[331] = data_LoS["lidar331"]
    lidar_array[332] = data_LoS["lidar332"]
    lidar_array[333] = data_LoS["lidar333"]
    lidar_array[334] = data_LoS["lidar334"]
    lidar_array[335] = data_LoS["lidar335"]
    lidar_array[336] = data_LoS["lidar336"]
    lidar_array[337] = data_LoS["lidar337"]
    lidar_array[338] = data_LoS["lidar338"]
    lidar_array[339] = data_LoS["lidar339"]
    lidar_array[340] = data_LoS["lidar340"]
    lidar_array[341] = data_LoS["lidar341"]
    lidar_array[342] = data_LoS["lidar342"]
    lidar_array[343] = data_LoS["lidar343"]
    lidar_array[344] = data_LoS["lidar344"]
    lidar_array[345] = data_LoS["lidar345"]
    lidar_array[346] = data_LoS["lidar346"]
    lidar_array[347] = data_LoS["lidar347"]
    lidar_array[348] = data_LoS["lidar348"]
    lidar_array[349] = data_LoS["lidar349"]
    lidar_array[350] = data_LoS["lidar350"]
    lidar_array[351] = data_LoS["lidar351"]
    lidar_array[352] = data_LoS["lidar352"]
    lidar_array[353] = data_LoS["lidar353"]
    lidar_array[354] = data_LoS["lidar354"]
    lidar_array[355] = data_LoS["lidar355"]
    lidar_array[356] = data_LoS["lidar356"]
    lidar_array[357] = data_LoS["lidar357"]
    lidar_array[358] = data_LoS["lidar358"]
    lidar_array[359] = data_LoS["lidar359"]



    for i in range(len(lidar_array[0])):
    #for i in range(5):
        ax1.clear()
        ax1.set_xlim(-12,12)
        ax1.set_ylim(-12,12)
        for l in range(360):
            ax1.scatter(math.cos(math.radians(l)) * lidar_array[l][i], math.sin(math.radians(l)) * lidar_array[l][i], c='r')

        name_of_file_1 = 'LoS_lidar%s_' + str(i) +'.png'

        fig1.savefig(output_path + name_of_file_1 % number_of_files_LoS)
        print("i: ",i)
    number_of_files_LoS = number_of_files_LoS + 1


while os.path.exists(path_NLoS % number_of_files_NLoS):
    local_path = path_NLoS %  number_of_files_NLoS
    data_NLoS = pd.read_csv(local_path)
    lidar_array = [[0] * len(data_NLoS["lidar0"])]*360

    lidar_array[0] = data_NLoS["lidar0"]
    lidar_array[1] = data_NLoS["lidar1"]
    lidar_array[2] = data_NLoS["lidar2"]
    lidar_array[3] = data_NLoS["lidar3"]
    lidar_array[4] = data_NLoS["lidar4"]
    lidar_array[5] = data_NLoS["lidar5"]
    lidar_array[6] = data_NLoS["lidar6"]
    lidar_array[7] = data_NLoS["lidar7"]
    lidar_array[8] = data_NLoS["lidar8"]
    lidar_array[9] = data_NLoS["lidar9"]
    lidar_array[10] = data_NLoS["lidar10"]
    lidar_array[11] = data_NLoS["lidar11"]
    lidar_array[12] = data_NLoS["lidar12"]
    lidar_array[13] = data_NLoS["lidar13"]
    lidar_array[14] = data_NLoS["lidar14"]
    lidar_array[15] = data_NLoS["lidar15"]
    lidar_array[16] = data_NLoS["lidar16"]
    lidar_array[17] = data_NLoS["lidar17"]
    lidar_array[18] = data_NLoS["lidar18"]
    lidar_array[19] = data_NLoS["lidar19"]
    lidar_array[20] = data_NLoS["lidar20"]
    lidar_array[21] = data_NLoS["lidar21"]
    lidar_array[22] = data_NLoS["lidar22"]
    lidar_array[23] = data_NLoS["lidar23"]
    lidar_array[24] = data_NLoS["lidar24"]
    lidar_array[25] = data_NLoS["lidar25"]
    lidar_array[26] = data_NLoS["lidar26"]
    lidar_array[27] = data_NLoS["lidar27"]
    lidar_array[28] = data_NLoS["lidar28"]
    lidar_array[29] = data_NLoS["lidar29"]
    lidar_array[30] = data_NLoS["lidar30"]
    lidar_array[31] = data_NLoS["lidar31"]
    lidar_array[32] = data_NLoS["lidar32"]
    lidar_array[33] = data_NLoS["lidar33"]
    lidar_array[34] = data_NLoS["lidar34"]
    lidar_array[35] = data_NLoS["lidar35"]
    lidar_array[36] = data_NLoS["lidar36"]
    lidar_array[37] = data_NLoS["lidar37"]
    lidar_array[38] = data_NLoS["lidar38"]
    lidar_array[39] = data_NLoS["lidar39"]
    lidar_array[40] = data_NLoS["lidar40"]
    lidar_array[41] = data_NLoS["lidar41"]
    lidar_array[42] = data_NLoS["lidar42"]
    lidar_array[43] = data_NLoS["lidar43"]
    lidar_array[44] = data_NLoS["lidar44"]
    lidar_array[45] = data_NLoS["lidar45"]
    lidar_array[46] = data_NLoS["lidar46"]
    lidar_array[47] = data_NLoS["lidar47"]
    lidar_array[48] = data_NLoS["lidar48"]
    lidar_array[49] = data_NLoS["lidar49"]
    lidar_array[50] = data_NLoS["lidar50"]
    lidar_array[51] = data_NLoS["lidar51"]
    lidar_array[52] = data_NLoS["lidar52"]
    lidar_array[53] = data_NLoS["lidar53"]
    lidar_array[54] = data_NLoS["lidar54"]
    lidar_array[55] = data_NLoS["lidar55"]
    lidar_array[56] = data_NLoS["lidar56"]
    lidar_array[57] = data_NLoS["lidar57"]
    lidar_array[58] = data_NLoS["lidar58"]
    lidar_array[59] = data_NLoS["lidar59"]
    lidar_array[60] = data_NLoS["lidar60"]
    lidar_array[61] = data_NLoS["lidar61"]
    lidar_array[62] = data_NLoS["lidar62"]
    lidar_array[63] = data_NLoS["lidar63"]
    lidar_array[64] = data_NLoS["lidar64"]
    lidar_array[65] = data_NLoS["lidar65"]
    lidar_array[66] = data_NLoS["lidar66"]
    lidar_array[67] = data_NLoS["lidar67"]
    lidar_array[68] = data_NLoS["lidar68"]
    lidar_array[69] = data_NLoS["lidar69"] #nice
    lidar_array[70] = data_NLoS["lidar70"]
    lidar_array[71] = data_NLoS["lidar71"]
    lidar_array[72] = data_NLoS["lidar72"]
    lidar_array[73] = data_NLoS["lidar73"]
    lidar_array[74] = data_NLoS["lidar74"]
    lidar_array[75] = data_NLoS["lidar75"]
    lidar_array[76] = data_NLoS["lidar76"]
    lidar_array[77] = data_NLoS["lidar77"]
    lidar_array[78] = data_NLoS["lidar78"]
    lidar_array[79] = data_NLoS["lidar79"]
    lidar_array[80] = data_NLoS["lidar80"]
    lidar_array[81] = data_NLoS["lidar81"]
    lidar_array[82] = data_NLoS["lidar82"]
    lidar_array[83] = data_NLoS["lidar83"]
    lidar_array[84] = data_NLoS["lidar84"]
    lidar_array[85] = data_NLoS["lidar85"]
    lidar_array[86] = data_NLoS["lidar86"]
    lidar_array[87] = data_NLoS["lidar87"]
    lidar_array[88] = data_NLoS["lidar88"]
    lidar_array[89] = data_NLoS["lidar89"]
    lidar_array[90] = data_NLoS["lidar90"]
    lidar_array[91] = data_NLoS["lidar91"]
    lidar_array[92] = data_NLoS["lidar92"]
    lidar_array[93] = data_NLoS["lidar93"]
    lidar_array[94] = data_NLoS["lidar94"]
    lidar_array[95] = data_NLoS["lidar95"]
    lidar_array[96] = data_NLoS["lidar96"]
    lidar_array[97] = data_NLoS["lidar97"]
    lidar_array[98] = data_NLoS["lidar98"]
    lidar_array[99] = data_NLoS["lidar99"]
    lidar_array[100] = data_NLoS["lidar100"]
    lidar_array[101] = data_NLoS["lidar101"]
    lidar_array[102] = data_NLoS["lidar102"]
    lidar_array[103] = data_NLoS["lidar103"]
    lidar_array[104] = data_NLoS["lidar104"]
    lidar_array[105] = data_NLoS["lidar105"]
    lidar_array[106] = data_NLoS["lidar106"]
    lidar_array[107] = data_NLoS["lidar107"]
    lidar_array[108] = data_NLoS["lidar108"]
    lidar_array[109] = data_NLoS["lidar109"]
    lidar_array[110] = data_NLoS["lidar110"]
    lidar_array[111] = data_NLoS["lidar111"]
    lidar_array[112] = data_NLoS["lidar112"]
    lidar_array[113] = data_NLoS["lidar113"]
    lidar_array[114] = data_NLoS["lidar114"]
    lidar_array[115] = data_NLoS["lidar115"]
    lidar_array[116] = data_NLoS["lidar116"]
    lidar_array[117] = data_NLoS["lidar117"]
    lidar_array[118] = data_NLoS["lidar118"]
    lidar_array[119] = data_NLoS["lidar119"]
    lidar_array[120] = data_NLoS["lidar120"]
    lidar_array[121] = data_NLoS["lidar121"]
    lidar_array[122] = data_NLoS["lidar122"]
    lidar_array[123] = data_NLoS["lidar123"]
    lidar_array[124] = data_NLoS["lidar124"]
    lidar_array[125] = data_NLoS["lidar125"]
    lidar_array[126] = data_NLoS["lidar126"]
    lidar_array[127] = data_NLoS["lidar127"]
    lidar_array[128] = data_NLoS["lidar128"]
    lidar_array[129] = data_NLoS["lidar129"]
    lidar_array[130] = data_NLoS["lidar130"]
    lidar_array[131] = data_NLoS["lidar131"]
    lidar_array[132] = data_NLoS["lidar132"]
    lidar_array[133] = data_NLoS["lidar133"]
    lidar_array[134] = data_NLoS["lidar134"]
    lidar_array[135] = data_NLoS["lidar135"]
    lidar_array[136] = data_NLoS["lidar136"]
    lidar_array[137] = data_NLoS["lidar137"]
    lidar_array[138] = data_NLoS["lidar138"]
    lidar_array[139] = data_NLoS["lidar139"]
    lidar_array[140] = data_NLoS["lidar140"]
    lidar_array[141] = data_NLoS["lidar141"]
    lidar_array[142] = data_NLoS["lidar142"]
    lidar_array[143] = data_NLoS["lidar143"]
    lidar_array[144] = data_NLoS["lidar144"]
    lidar_array[145] = data_NLoS["lidar145"]
    lidar_array[146] = data_NLoS["lidar146"]
    lidar_array[147] = data_NLoS["lidar147"]
    lidar_array[148] = data_NLoS["lidar148"]
    lidar_array[149] = data_NLoS["lidar149"]
    lidar_array[150] = data_NLoS["lidar150"]
    lidar_array[151] = data_NLoS["lidar151"]
    lidar_array[152] = data_NLoS["lidar152"]
    lidar_array[153] = data_NLoS["lidar153"]
    lidar_array[154] = data_NLoS["lidar154"]
    lidar_array[155] = data_NLoS["lidar155"]
    lidar_array[156] = data_NLoS["lidar156"]
    lidar_array[157] = data_NLoS["lidar157"]
    lidar_array[158] = data_NLoS["lidar158"]
    lidar_array[159] = data_NLoS["lidar159"]
    lidar_array[160] = data_NLoS["lidar160"]
    lidar_array[161] = data_NLoS["lidar161"]
    lidar_array[162] = data_NLoS["lidar162"]
    lidar_array[163] = data_NLoS["lidar163"]
    lidar_array[164] = data_NLoS["lidar164"]
    lidar_array[165] = data_NLoS["lidar165"]
    lidar_array[166] = data_NLoS["lidar166"]
    lidar_array[167] = data_NLoS["lidar167"]
    lidar_array[168] = data_NLoS["lidar168"]
    lidar_array[169] = data_NLoS["lidar169"]
    lidar_array[170] = data_NLoS["lidar170"]
    lidar_array[171] = data_NLoS["lidar171"]
    lidar_array[172] = data_NLoS["lidar172"]
    lidar_array[173] = data_NLoS["lidar173"]
    lidar_array[174] = data_NLoS["lidar174"]
    lidar_array[175] = data_NLoS["lidar175"]
    lidar_array[176] = data_NLoS["lidar176"]
    lidar_array[177] = data_NLoS["lidar177"]
    lidar_array[178] = data_NLoS["lidar178"]
    lidar_array[179] = data_NLoS["lidar179"]
    lidar_array[180] = data_NLoS["lidar180"]
    lidar_array[181] = data_NLoS["lidar181"]
    lidar_array[182] = data_NLoS["lidar182"]
    lidar_array[183] = data_NLoS["lidar183"]
    lidar_array[184] = data_NLoS["lidar184"]
    lidar_array[185] = data_NLoS["lidar185"]
    lidar_array[186] = data_NLoS["lidar186"]
    lidar_array[187] = data_NLoS["lidar187"]
    lidar_array[188] = data_NLoS["lidar188"]
    lidar_array[189] = data_NLoS["lidar189"]
    lidar_array[190] = data_NLoS["lidar190"]
    lidar_array[191] = data_NLoS["lidar191"]
    lidar_array[192] = data_NLoS["lidar192"]
    lidar_array[193] = data_NLoS["lidar193"]
    lidar_array[194] = data_NLoS["lidar194"]
    lidar_array[195] = data_NLoS["lidar195"]
    lidar_array[196] = data_NLoS["lidar196"]
    lidar_array[197] = data_NLoS["lidar197"]
    lidar_array[198] = data_NLoS["lidar198"]
    lidar_array[199] = data_NLoS["lidar199"]
    lidar_array[200] = data_NLoS["lidar200"]
    lidar_array[201] = data_NLoS["lidar201"]
    lidar_array[202] = data_NLoS["lidar202"]
    lidar_array[203] = data_NLoS["lidar203"]
    lidar_array[204] = data_NLoS["lidar204"]
    lidar_array[205] = data_NLoS["lidar205"]
    lidar_array[206] = data_NLoS["lidar206"]
    lidar_array[207] = data_NLoS["lidar207"]
    lidar_array[208] = data_NLoS["lidar208"]
    lidar_array[209] = data_NLoS["lidar209"]
    lidar_array[210] = data_NLoS["lidar210"]
    lidar_array[211] = data_NLoS["lidar211"]
    lidar_array[212] = data_NLoS["lidar212"]
    lidar_array[213] = data_NLoS["lidar213"]
    lidar_array[214] = data_NLoS["lidar214"]
    lidar_array[215] = data_NLoS["lidar215"]
    lidar_array[216] = data_NLoS["lidar216"]
    lidar_array[217] = data_NLoS["lidar217"]
    lidar_array[218] = data_NLoS["lidar218"]
    lidar_array[219] = data_NLoS["lidar219"]
    lidar_array[220] = data_NLoS["lidar220"]
    lidar_array[221] = data_NLoS["lidar221"]
    lidar_array[222] = data_NLoS["lidar222"]
    lidar_array[223] = data_NLoS["lidar223"]
    lidar_array[224] = data_NLoS["lidar224"]
    lidar_array[225] = data_NLoS["lidar225"]
    lidar_array[226] = data_NLoS["lidar226"]
    lidar_array[227] = data_NLoS["lidar227"]
    lidar_array[228] = data_NLoS["lidar228"]
    lidar_array[229] = data_NLoS["lidar229"]
    lidar_array[230] = data_NLoS["lidar230"]
    lidar_array[231] = data_NLoS["lidar231"]
    lidar_array[232] = data_NLoS["lidar232"]
    lidar_array[233] = data_NLoS["lidar233"]
    lidar_array[234] = data_NLoS["lidar234"]
    lidar_array[235] = data_NLoS["lidar235"]
    lidar_array[236] = data_NLoS["lidar236"]
    lidar_array[237] = data_NLoS["lidar237"]
    lidar_array[238] = data_NLoS["lidar238"]
    lidar_array[239] = data_NLoS["lidar239"]
    lidar_array[240] = data_NLoS["lidar240"]
    lidar_array[241] = data_NLoS["lidar241"]
    lidar_array[242] = data_NLoS["lidar242"]
    lidar_array[243] = data_NLoS["lidar243"]
    lidar_array[244] = data_NLoS["lidar244"]
    lidar_array[245] = data_NLoS["lidar245"]
    lidar_array[246] = data_NLoS["lidar246"]
    lidar_array[247] = data_NLoS["lidar247"]
    lidar_array[248] = data_NLoS["lidar248"]
    lidar_array[249] = data_NLoS["lidar249"]
    lidar_array[250] = data_NLoS["lidar250"]
    lidar_array[251] = data_NLoS["lidar251"]
    lidar_array[252] = data_NLoS["lidar252"]
    lidar_array[253] = data_NLoS["lidar253"]
    lidar_array[254] = data_NLoS["lidar254"]
    lidar_array[255] = data_NLoS["lidar255"]
    lidar_array[256] = data_NLoS["lidar256"]
    lidar_array[257] = data_NLoS["lidar257"]
    lidar_array[258] = data_NLoS["lidar258"]
    lidar_array[259] = data_NLoS["lidar259"]
    lidar_array[260] = data_NLoS["lidar260"]
    lidar_array[261] = data_NLoS["lidar261"]
    lidar_array[262] = data_NLoS["lidar262"]
    lidar_array[263] = data_NLoS["lidar263"]
    lidar_array[264] = data_NLoS["lidar264"]
    lidar_array[265] = data_NLoS["lidar265"]
    lidar_array[266] = data_NLoS["lidar266"]
    lidar_array[267] = data_NLoS["lidar267"]
    lidar_array[268] = data_NLoS["lidar268"]
    lidar_array[269] = data_NLoS["lidar269"]
    lidar_array[270] = data_NLoS["lidar270"]
    lidar_array[271] = data_NLoS["lidar271"]
    lidar_array[272] = data_NLoS["lidar272"]
    lidar_array[273] = data_NLoS["lidar273"]
    lidar_array[274] = data_NLoS["lidar274"]
    lidar_array[275] = data_NLoS["lidar275"]
    lidar_array[276] = data_NLoS["lidar276"]
    lidar_array[277] = data_NLoS["lidar277"]
    lidar_array[278] = data_NLoS["lidar278"]
    lidar_array[279] = data_NLoS["lidar279"]
    lidar_array[280] = data_NLoS["lidar280"]
    lidar_array[281] = data_NLoS["lidar281"]
    lidar_array[282] = data_NLoS["lidar282"]
    lidar_array[283] = data_NLoS["lidar283"]
    lidar_array[284] = data_NLoS["lidar284"]
    lidar_array[285] = data_NLoS["lidar285"]
    lidar_array[286] = data_NLoS["lidar286"]
    lidar_array[287] = data_NLoS["lidar287"]
    lidar_array[288] = data_NLoS["lidar288"]
    lidar_array[289] = data_NLoS["lidar289"]
    lidar_array[290] = data_NLoS["lidar290"]
    lidar_array[291] = data_NLoS["lidar291"]
    lidar_array[292] = data_NLoS["lidar292"]
    lidar_array[293] = data_NLoS["lidar293"]
    lidar_array[294] = data_NLoS["lidar294"]
    lidar_array[295] = data_NLoS["lidar295"]
    lidar_array[296] = data_NLoS["lidar296"]
    lidar_array[297] = data_NLoS["lidar297"]
    lidar_array[298] = data_NLoS["lidar298"]
    lidar_array[299] = data_NLoS["lidar299"]
    lidar_array[300] = data_NLoS["lidar300"]
    lidar_array[301] = data_NLoS["lidar301"]
    lidar_array[302] = data_NLoS["lidar302"]
    lidar_array[303] = data_NLoS["lidar303"]
    lidar_array[304] = data_NLoS["lidar304"]
    lidar_array[305] = data_NLoS["lidar305"]
    lidar_array[306] = data_NLoS["lidar306"]
    lidar_array[307] = data_NLoS["lidar307"]
    lidar_array[308] = data_NLoS["lidar308"]
    lidar_array[309] = data_NLoS["lidar309"]
    lidar_array[310] = data_NLoS["lidar310"]
    lidar_array[311] = data_NLoS["lidar311"]
    lidar_array[312] = data_NLoS["lidar312"]
    lidar_array[313] = data_NLoS["lidar313"]
    lidar_array[314] = data_NLoS["lidar314"]
    lidar_array[315] = data_NLoS["lidar315"]
    lidar_array[316] = data_NLoS["lidar316"]
    lidar_array[317] = data_NLoS["lidar317"]
    lidar_array[318] = data_NLoS["lidar318"]
    lidar_array[319] = data_NLoS["lidar319"]
    lidar_array[320] = data_NLoS["lidar320"]
    lidar_array[321] = data_NLoS["lidar321"]
    lidar_array[322] = data_NLoS["lidar322"]
    lidar_array[323] = data_NLoS["lidar323"]
    lidar_array[324] = data_NLoS["lidar324"]
    lidar_array[325] = data_NLoS["lidar325"]
    lidar_array[326] = data_NLoS["lidar326"]
    lidar_array[327] = data_NLoS["lidar327"]
    lidar_array[328] = data_NLoS["lidar328"]
    lidar_array[329] = data_NLoS["lidar329"]
    lidar_array[330] = data_NLoS["lidar330"]
    lidar_array[331] = data_NLoS["lidar331"]
    lidar_array[332] = data_NLoS["lidar332"]
    lidar_array[333] = data_NLoS["lidar333"]
    lidar_array[334] = data_NLoS["lidar334"]
    lidar_array[335] = data_NLoS["lidar335"]
    lidar_array[336] = data_NLoS["lidar336"]
    lidar_array[337] = data_NLoS["lidar337"]
    lidar_array[338] = data_NLoS["lidar338"]
    lidar_array[339] = data_NLoS["lidar339"]
    lidar_array[340] = data_NLoS["lidar340"]
    lidar_array[341] = data_NLoS["lidar341"]
    lidar_array[342] = data_NLoS["lidar342"]
    lidar_array[343] = data_NLoS["lidar343"]
    lidar_array[344] = data_NLoS["lidar344"]
    lidar_array[345] = data_NLoS["lidar345"]
    lidar_array[346] = data_NLoS["lidar346"]
    lidar_array[347] = data_NLoS["lidar347"]
    lidar_array[348] = data_NLoS["lidar348"]
    lidar_array[349] = data_NLoS["lidar349"]
    lidar_array[350] = data_NLoS["lidar350"]
    lidar_array[351] = data_NLoS["lidar351"]
    lidar_array[352] = data_NLoS["lidar352"]
    lidar_array[353] = data_NLoS["lidar353"]
    lidar_array[354] = data_NLoS["lidar354"]
    lidar_array[355] = data_NLoS["lidar355"]
    lidar_array[356] = data_NLoS["lidar356"]
    lidar_array[357] = data_NLoS["lidar357"]
    lidar_array[358] = data_NLoS["lidar358"]
    lidar_array[359] = data_NLoS["lidar359"]



    for i in range(len(lidar_array[0])):
    #for i in range(0):
        ax2.clear()
        ax2.set_xlim(-12,12)
        ax2.set_ylim(-12,12)
        for l in range(360):
            ax2.scatter(math.cos(math.radians(l)) * lidar_array[l][i], math.sin(math.radians(l)) * lidar_array[l][i], c='r')

        name_of_file_2 = 'NLoS_lidar%s_' + str(i) +'.png'

        fig2.savefig(output_path + name_of_file_2 % number_of_files_NLoS)

    number_of_files_NLoS = number_of_files_NLoS + 1


print("im done")
