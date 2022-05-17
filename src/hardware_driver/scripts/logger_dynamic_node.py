#!/usr/bin/env python3
import rospy
import sys
import os
import csv
import time
import math
from pathlib import Path
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from custom_msgs.msg import odom_and_imu
from custom_msgs.msg import USPS_msgs



##### this is a subscriber example that can be copy pasta later #####

## paste this as a global variable, outside of the callback function (after "number_of_files = 0")
#data_to_be_logged = [0] * 3 # remember to specify this correctly, depending on what you are trying to log

## now we specify the callback function for the specific data subscription (replace "something" with what you want)
#def callback_something(data):

#    global data_to_be_logged
#    data_to_be_logged[0] = data.x
#    data_to_be_logged[1] = data.y
#    data_to_be_logged[2] = data.z

## past the thing  below in the main loop within "while not rospy.is_shutdown():"
#    rospy.Subscriber("topic_name", Data_Type, callback_something) #(topic name, data type, callback function)

##### now you should have a working subscriber! #####



input_speed = 0
angular_speed = 0

path = Path.home().joinpath("test_data", "init_dyn_log%s.csv")
path_lidar = str(Path.home().joinpath("test_data", "dyn_lidar%s.txt"))
folder_path = str(Path.home().joinpath("test_data"))
isfolder = os.path.isdir(folder_path)
number_of_files = 0
beacon_data = [0] * 2
IMU_data = [0]*6
without_lidar_data = [0] * 5
with_lidar_data = [0] * 5
simple_pose_esti = [0] * 3
multi_kalman = [0] * 3
test_run_time = 120
terminate_time = time.time() + test_run_time
print(terminate_time)
first_time = True

fieldnames = ["with_x","with_y","with_v","with_hx","with_hy","without_x","without_y","without_v","without_hx","without_hy","acc_x","acc_y","acc_z","gyro_x","gyro_y","gyro_z","ID","distance","linear_speed","angular_speed","simple_x","simple_y","muti_kalman_x","muti_kalman_y","lidar_good","lidar0","lidar1","lidar2","lidar3","lidar4","lidar5","lidar6","lidar7","lidar8","lidar9","lidar10","lidar11","lidar12","lidar13","lidar14","lidar15","lidar16","lidar17","lidar18","lidar19","lidar20","lidar21","lidar22","lidar23","lidar24","lidar25","lidar26","lidar27","lidar28","lidar29","lidar30","lidar31","lidar32","lidar33","lidar34","lidar35","lidar36","lidar37","lidar38","lidar39","lidar40","lidar41","lidar42","lidar43","lidar44","lidar45","lidar46","lidar47","lidar48","lidar49","lidar50","lidar51","lidar52","lidar53","lidar54","lidar55","lidar56","lidar57","lidar58","lidar59","lidar60","lidar61","lidar62","lidar63","lidar64","lidar65","lidar66","lidar67","lidar68","lidar69","lidar70","lidar71","lidar72","lidar73","lidar74","lidar75","lidar76","lidar77","lidar78","lidar79","lidar80","lidar81","lidar82","lidar83","lidar84","lidar85","lidar86","lidar87","lidar88","lidar89","lidar90","lidar91","lidar92","lidar93","lidar94","lidar95","lidar96","lidar97","lidar98","lidar99","lidar100","lidar101","lidar102","lidar103","lidar104","lidar105","lidar106","lidar107","lidar108","lidar109","lidar110","lidar111","lidar112","lidar113","lidar114","lidar115","lidar116","lidar117","lidar118","lidar119","lidar120","lidar121","lidar122","lidar123","lidar124","lidar125","lidar126","lidar127","lidar128","lidar129","lidar130","lidar131","lidar132","lidar133","lidar134","lidar135","lidar136","lidar137","lidar138","lidar139","lidar140","lidar141","lidar142","lidar143","lidar144","lidar145","lidar146","lidar147","lidar148","lidar149","lidar150","lidar151","lidar152","lidar153","lidar154","lidar155","lidar156","lidar157","lidar158","lidar159","lidar160","lidar161","lidar162","lidar163","lidar164","lidar165","lidar166","lidar167","lidar168","lidar169","lidar170","lidar171","lidar172","lidar173","lidar174","lidar175","lidar176","lidar177","lidar178","lidar179","lidar180","lidar181","lidar182","lidar183","lidar184","lidar185","lidar186","lidar187","lidar188","lidar189","lidar190","lidar191","lidar192","lidar193","lidar194","lidar195","lidar196","lidar197","lidar198","lidar199","lidar200","lidar201","lidar202","lidar203","lidar204","lidar205","lidar206","lidar207","lidar208","lidar209","lidar210","lidar211","lidar212","lidar213","lidar214","lidar215","lidar216","lidar217","lidar218","lidar219","lidar220","lidar221","lidar222","lidar223","lidar224","lidar225","lidar226","lidar227","lidar228","lidar229","lidar230","lidar231","lidar232","lidar233","lidar234","lidar235","lidar236","lidar237","lidar238","lidar239","lidar240","lidar241","lidar242","lidar243","lidar244","lidar245","lidar246","lidar247","lidar248","lidar249","lidar250","lidar251","lidar252","lidar253","lidar254","lidar255","lidar256","lidar257","lidar258","lidar259","lidar260","lidar261","lidar262","lidar263","lidar264","lidar265","lidar266","lidar267","lidar268","lidar269","lidar270","lidar271","lidar272","lidar273","lidar274","lidar275","lidar276","lidar277","lidar278","lidar279","lidar280","lidar281","lidar282","lidar283","lidar284","lidar285","lidar286","lidar287","lidar288","lidar289","lidar290","lidar291","lidar292","lidar293","lidar294","lidar295","lidar296","lidar297","lidar298","lidar299","lidar300","lidar301","lidar302","lidar303","lidar304","lidar305","lidar306","lidar307","lidar308","lidar309","lidar310","lidar311","lidar312","lidar313","lidar314","lidar315","lidar316","lidar317","lidar318","lidar319","lidar320","lidar321","lidar322","lidar323","lidar324","lidar325","lidar326","lidar327","lidar328","lidar329","lidar330","lidar331","lidar332","lidar333","lidar334","lidar335","lidar336","lidar337","lidar338","lidar339","lidar340","lidar341","lidar342","lidar343","lidar344","lidar345","lidar346","lidar347","lidar348","lidar349","lidar350","lidar351","lidar352","lidar353","lidar354","lidar355","lidar356","lidar357","lidar358","lidar359"]
def file_iterator():
    global number_of_files
    number_of_files = 0
    #print("im finding out how many logs there are")
    while os.path.exists(str(path) % number_of_files):
        #print("im running itorator")
        number_of_files = number_of_files + 1
    print(number_of_files)

#def callback(data):
#    print(data.data)

def callback_distance(data):
    global beacon_data
    beacon_id = [42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
    if data.ID in beacon_id:
        beacon_data = [data.ID, data.distance]

def callback_imu(data):
    global IMU_data
    IMU_data = [data.imu_acc.x, data.imu_acc.y, data.imu_acc.z, data.imu_gyro.x, data.imu_gyro.y, data.imu_gyro.z]

def callback_pose_estimator_with_lidar(data):
    global with_lidar_data
    with_lidar_data = [data.position.x,data.position.y,data.position.z,data.orientation.x,data.orientation.y]
    #print(with_lidar_data)

def callback_simple_pose_esti(data):
    global simple_pose_esti
    simple_pose_esti = [data.position.x,data.position.y,data.position.z]

def callback_multi_kalman(data):
    global multi_kalman
    multi_kalman = [data.position.x,data.position.y,data.position.z]



def callback_pose_estimator_without_lidar(data):
    global without_lidar_data
    without_lidar_data = [data.position.x,data.position.y,data.position.z,data.orientation.x,data.orientation.y]

def callback_lidar(data):#####
    global lidar_array, first_time
    lidar_array = [0] * int(2 * math.pi/data.angle_increment)
    #print("laser data recived")
    for i in range(len(data.ranges)):
        lidar_array[i] = data.ranges[i]
    if first_time == True:
        lidar_lable = str(['lidar'] * int(2 * math.pi/data.angle_increment))
        f = open(path_lidar % number_of_files, "w")
        for k in range(len(lidar_array)-1):
            f.write('lidar' + ',')
        f.write('lidar' + "\n")
        f.close()
        first_time = False



def callback_input_speed(data):
    global input_speed, angular_speed
    input_speed = data.linear.x
    angular_speed = data.angular.z


def main():
    global path, folder_path, isfolder, number_of_files, seperator,input_speed, angular_speed, lidar_array,without_lidar_data, with_lidar_data, IMU_data, beacon_data, fieldnames
    if not isfolder:
        os.mkdir(folder_path)
        print("making directory")
    file_iterator()
    path = str(path) % number_of_files
    rospy.init_node('logger', anonymous=True)

    with open(path,'w') as csv_file:
        csv_writer = csv.DictWriter(csv_file,fieldnames=fieldnames)
        csv_writer.writeheader()
    #rospy.Subscriber("chatter", String, callback)
    #v = 1
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #print("i am boot")
        rospy.Subscriber("scan", LaserScan, callback_lidar)
        rospy.Subscriber("cmd_vel", Twist, callback_input_speed)
        rospy.Subscriber("beacon_data", USPS_msgs, callback_distance)
        rospy.Subscriber("odometry_and_IMU", odom_and_imu, callback_imu)
        rospy.Subscriber("with_lidar", Pose, callback_pose_estimator_with_lidar)
        rospy.Subscriber("without_lidar", Pose, callback_pose_estimator_without_lidar)
        rospy.Subscriber("simple_pose_esti", Pose, callback_simple_pose_esti)
        rospy.Subscriber("multi_kalman", Pose, callback_multi_kalman)
        pub = rospy.Publisher('terminating_signal', Bool, queue_size=10)
        if first_time == False:
            good_lidar = 0
            if len(lidar_array) == 360:
                good_lidar = 1
            if len(lidar_array) != 360:
                lidar_array = [float("nan")] * len(lidar_array)
            with open(path, 'a') as csv_file:
                csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                info = {
                    "with_x": with_lidar_data[0],
                    "with_y": with_lidar_data[1],
                    "with_v": with_lidar_data[2],
                    "with_hx": with_lidar_data[3],
                    "with_hy": with_lidar_data[4],
                    "without_x": without_lidar_data[0],
                    "without_y": without_lidar_data[1],
                    "without_v": without_lidar_data[2],
                    "without_hx": without_lidar_data[3],
                    "without_hy": without_lidar_data[4],
                    "acc_x": IMU_data[0],
                    "acc_y": IMU_data[1],
                    "acc_z": IMU_data[2],
                    "gyro_x": IMU_data[3],
                    "gyro_y": IMU_data[4],
                    "gyro_z": IMU_data[5],
                    "ID": beacon_data[0],
                    "distance": beacon_data[1],
                    "linear_speed": input_speed,
                    "angular_speed": angular_speed,
                    "simple_x": simple_pose_esti[0],
                    "simple_y": simple_pose_esti[1],
                    "muti_kalman_x": multi_kalman[0],
                    "muti_kalman_y": multi_kalman[1],
                    "lidar_good": good_lidar,
                    "lidar0": lidar_array[0],
                    "lidar1": lidar_array[1],
                    "lidar2": lidar_array[2],
                    "lidar3": lidar_array[3],
                    "lidar4": lidar_array[4],
                    "lidar5": lidar_array[5],
                    "lidar6": lidar_array[6],
                    "lidar7": lidar_array[7],
                    "lidar8": lidar_array[8],
                    "lidar9": lidar_array[9],
                    "lidar10": lidar_array[10],
                    "lidar11": lidar_array[11],
                    "lidar12": lidar_array[12],
                    "lidar13": lidar_array[13],
                    "lidar14": lidar_array[14],
                    "lidar15": lidar_array[15],
                    "lidar16": lidar_array[16],
                    "lidar17": lidar_array[17],
                    "lidar18": lidar_array[18],
                    "lidar19": lidar_array[19],
                    "lidar20": lidar_array[20],
                    "lidar21": lidar_array[21],
                    "lidar22": lidar_array[22],
                    "lidar23": lidar_array[23],
                    "lidar24": lidar_array[24],
                    "lidar25": lidar_array[25],
                    "lidar26": lidar_array[26],
                    "lidar27": lidar_array[27],
                    "lidar28": lidar_array[28],
                    "lidar29": lidar_array[29],
                    "lidar30": lidar_array[30],
                    "lidar31": lidar_array[31],
                    "lidar32": lidar_array[32],
                    "lidar33": lidar_array[33],
                    "lidar34": lidar_array[34],
                    "lidar35": lidar_array[35],
                    "lidar36": lidar_array[36],
                    "lidar37": lidar_array[37],
                    "lidar38": lidar_array[38],
                    "lidar39": lidar_array[39],
                    "lidar40": lidar_array[40],
                    "lidar41": lidar_array[41],
                    "lidar42": lidar_array[42],
                    "lidar43": lidar_array[43],
                    "lidar44": lidar_array[44],
                    "lidar45": lidar_array[45],
                    "lidar46": lidar_array[46],
                    "lidar47": lidar_array[47],
                    "lidar48": lidar_array[48],
                    "lidar49": lidar_array[49],
                    "lidar50": lidar_array[50],
                    "lidar51": lidar_array[51],
                    "lidar52": lidar_array[52],
                    "lidar53": lidar_array[53],
                    "lidar54": lidar_array[54],
                    "lidar55": lidar_array[55],
                    "lidar56": lidar_array[56],
                    "lidar57": lidar_array[57],
                    "lidar58": lidar_array[58],
                    "lidar59": lidar_array[59],
                    "lidar60": lidar_array[60],
                    "lidar61": lidar_array[61],
                    "lidar62": lidar_array[62],
                    "lidar63": lidar_array[63],
                    "lidar64": lidar_array[64],
                    "lidar65": lidar_array[65],
                    "lidar66": lidar_array[66],
                    "lidar67": lidar_array[67],
                    "lidar68": lidar_array[68],
                    "lidar69": lidar_array[69],
                    "lidar70": lidar_array[70],
                    "lidar71": lidar_array[71],
                    "lidar72": lidar_array[72],
                    "lidar73": lidar_array[73],
                    "lidar74": lidar_array[74],
                    "lidar75": lidar_array[75],
                    "lidar76": lidar_array[76],
                    "lidar77": lidar_array[77],
                    "lidar78": lidar_array[78],
                    "lidar79": lidar_array[79],
                    "lidar80": lidar_array[80],
                    "lidar81": lidar_array[81],
                    "lidar82": lidar_array[82],
                    "lidar83": lidar_array[83],
                    "lidar84": lidar_array[84],
                    "lidar85": lidar_array[85],
                    "lidar86": lidar_array[86],
                    "lidar87": lidar_array[87],
                    "lidar88": lidar_array[88],
                    "lidar89": lidar_array[89],
                    "lidar90": lidar_array[90],
                    "lidar91": lidar_array[91],
                    "lidar92": lidar_array[92],
                    "lidar93": lidar_array[93],
                    "lidar94": lidar_array[94],
                    "lidar95": lidar_array[95],
                    "lidar96": lidar_array[96],
                    "lidar97": lidar_array[97],
                    "lidar98": lidar_array[98],
                    "lidar99": lidar_array[99],
                    "lidar100": lidar_array[100],
                    "lidar101": lidar_array[101],
                    "lidar102": lidar_array[102],
                    "lidar103": lidar_array[103],
                    "lidar104": lidar_array[104],
                    "lidar105": lidar_array[105],
                    "lidar106": lidar_array[106],
                    "lidar107": lidar_array[107],
                    "lidar108": lidar_array[108],
                    "lidar109": lidar_array[109],
                    "lidar110": lidar_array[110],
                    "lidar111": lidar_array[111],
                    "lidar112": lidar_array[112],
                    "lidar113": lidar_array[113],
                    "lidar114": lidar_array[114],
                    "lidar115": lidar_array[115],
                    "lidar116": lidar_array[116],
                    "lidar117": lidar_array[117],
                    "lidar118": lidar_array[118],
                    "lidar119": lidar_array[119],
                    "lidar120": lidar_array[120],
                    "lidar121": lidar_array[121],
                    "lidar122": lidar_array[122],
                    "lidar123": lidar_array[123],
                    "lidar124": lidar_array[124],
                    "lidar125": lidar_array[125],
                    "lidar126": lidar_array[126],
                    "lidar127": lidar_array[127],
                    "lidar128": lidar_array[128],
                    "lidar129": lidar_array[129],
                    "lidar130": lidar_array[130],
                    "lidar131": lidar_array[131],
                    "lidar132": lidar_array[132],
                    "lidar133": lidar_array[133],
                    "lidar134": lidar_array[134],
                    "lidar135": lidar_array[135],
                    "lidar136": lidar_array[136],
                    "lidar137": lidar_array[137],
                    "lidar138": lidar_array[138],
                    "lidar139": lidar_array[139],
                    "lidar140": lidar_array[140],
                    "lidar141": lidar_array[141],
                    "lidar142": lidar_array[142],
                    "lidar143": lidar_array[143],
                    "lidar144": lidar_array[144],
                    "lidar145": lidar_array[145],
                    "lidar146": lidar_array[146],
                    "lidar147": lidar_array[147],
                    "lidar148": lidar_array[148],
                    "lidar149": lidar_array[149],
                    "lidar150": lidar_array[150],
                    "lidar151": lidar_array[151],
                    "lidar152": lidar_array[152],
                    "lidar153": lidar_array[153],
                    "lidar154": lidar_array[154],
                    "lidar155": lidar_array[155],
                    "lidar156": lidar_array[156],
                    "lidar157": lidar_array[157],
                    "lidar158": lidar_array[158],
                    "lidar159": lidar_array[159],
                    "lidar160": lidar_array[160],
                    "lidar161": lidar_array[161],
                    "lidar162": lidar_array[162],
                    "lidar163": lidar_array[163],
                    "lidar164": lidar_array[164],
                    "lidar165": lidar_array[165],
                    "lidar166": lidar_array[166],
                    "lidar167": lidar_array[167],
                    "lidar168": lidar_array[168],
                    "lidar169": lidar_array[169],
                    "lidar170": lidar_array[170],
                    "lidar171": lidar_array[171],
                    "lidar172": lidar_array[172],
                    "lidar173": lidar_array[173],
                    "lidar174": lidar_array[174],
                    "lidar175": lidar_array[175],
                    "lidar176": lidar_array[176],
                    "lidar177": lidar_array[177],
                    "lidar178": lidar_array[178],
                    "lidar179": lidar_array[179],
                    "lidar180": lidar_array[180],
                    "lidar181": lidar_array[181],
                    "lidar182": lidar_array[182],
                    "lidar183": lidar_array[183],
                    "lidar184": lidar_array[184],
                    "lidar185": lidar_array[185],
                    "lidar186": lidar_array[186],
                    "lidar187": lidar_array[187],
                    "lidar188": lidar_array[188],
                    "lidar189": lidar_array[189],
                    "lidar190": lidar_array[190],
                    "lidar191": lidar_array[191],
                    "lidar192": lidar_array[192],
                    "lidar193": lidar_array[193],
                    "lidar194": lidar_array[194],
                    "lidar195": lidar_array[195],
                    "lidar196": lidar_array[196],
                    "lidar197": lidar_array[197],
                    "lidar198": lidar_array[198],
                    "lidar199": lidar_array[199],
                    "lidar200": lidar_array[200],
                    "lidar201": lidar_array[201],
                    "lidar202": lidar_array[202],
                    "lidar203": lidar_array[203],
                    "lidar204": lidar_array[204],
                    "lidar205": lidar_array[205],
                    "lidar206": lidar_array[206],
                    "lidar207": lidar_array[207],
                    "lidar208": lidar_array[208],
                    "lidar209": lidar_array[209],
                    "lidar210": lidar_array[210],
                    "lidar211": lidar_array[211],
                    "lidar212": lidar_array[212],
                    "lidar213": lidar_array[213],
                    "lidar214": lidar_array[214],
                    "lidar215": lidar_array[215],
                    "lidar216": lidar_array[216],
                    "lidar217": lidar_array[217],
                    "lidar218": lidar_array[218],
                    "lidar219": lidar_array[219],
                    "lidar220": lidar_array[220],
                    "lidar221": lidar_array[221],
                    "lidar222": lidar_array[222],
                    "lidar223": lidar_array[223],
                    "lidar224": lidar_array[224],
                    "lidar225": lidar_array[225],
                    "lidar226": lidar_array[226],
                    "lidar227": lidar_array[227],
                    "lidar228": lidar_array[228],
                    "lidar229": lidar_array[229],
                    "lidar230": lidar_array[230],
                    "lidar231": lidar_array[231],
                    "lidar232": lidar_array[232],
                    "lidar233": lidar_array[233],
                    "lidar234": lidar_array[234],
                    "lidar235": lidar_array[235],
                    "lidar236": lidar_array[236],
                    "lidar237": lidar_array[237],
                    "lidar238": lidar_array[238],
                    "lidar239": lidar_array[239],
                    "lidar240": lidar_array[240],
                    "lidar241": lidar_array[241],
                    "lidar242": lidar_array[242],
                    "lidar243": lidar_array[243],
                    "lidar244": lidar_array[244],
                    "lidar245": lidar_array[245],
                    "lidar246": lidar_array[246],
                    "lidar247": lidar_array[247],
                    "lidar248": lidar_array[248],
                    "lidar249": lidar_array[249],
                    "lidar250": lidar_array[250],
                    "lidar251": lidar_array[251],
                    "lidar252": lidar_array[252],
                    "lidar253": lidar_array[253],
                    "lidar254": lidar_array[254],
                    "lidar255": lidar_array[255],
                    "lidar256": lidar_array[256],
                    "lidar257": lidar_array[257],
                    "lidar258": lidar_array[258],
                    "lidar259": lidar_array[259],
                    "lidar260": lidar_array[260],
                    "lidar261": lidar_array[261],
                    "lidar262": lidar_array[262],
                    "lidar263": lidar_array[263],
                    "lidar264": lidar_array[264],
                    "lidar265": lidar_array[265],
                    "lidar266": lidar_array[266],
                    "lidar267": lidar_array[267],
                    "lidar268": lidar_array[268],
                    "lidar269": lidar_array[269],
                    "lidar270": lidar_array[270],
                    "lidar271": lidar_array[271],
                    "lidar272": lidar_array[272],
                    "lidar273": lidar_array[273],
                    "lidar274": lidar_array[274],
                    "lidar275": lidar_array[275],
                    "lidar276": lidar_array[276],
                    "lidar277": lidar_array[277],
                    "lidar278": lidar_array[278],
                    "lidar279": lidar_array[279],
                    "lidar280": lidar_array[280],
                    "lidar281": lidar_array[281],
                    "lidar282": lidar_array[282],
                    "lidar283": lidar_array[283],
                    "lidar284": lidar_array[284],
                    "lidar285": lidar_array[285],
                    "lidar286": lidar_array[286],
                    "lidar287": lidar_array[287],
                    "lidar288": lidar_array[288],
                    "lidar289": lidar_array[289],
                    "lidar290": lidar_array[290],
                    "lidar291": lidar_array[291],
                    "lidar292": lidar_array[292],
                    "lidar293": lidar_array[293],
                    "lidar294": lidar_array[294],
                    "lidar295": lidar_array[295],
                    "lidar296": lidar_array[296],
                    "lidar297": lidar_array[297],
                    "lidar298": lidar_array[298],
                    "lidar299": lidar_array[299],
                    "lidar300": lidar_array[300],
                    "lidar301": lidar_array[301],
                    "lidar302": lidar_array[302],
                    "lidar303": lidar_array[303],
                    "lidar304": lidar_array[304],
                    "lidar305": lidar_array[305],
                    "lidar306": lidar_array[306],
                    "lidar307": lidar_array[307],
                    "lidar308": lidar_array[308],
                    "lidar309": lidar_array[309],
                    "lidar310": lidar_array[310],
                    "lidar311": lidar_array[311],
                    "lidar312": lidar_array[312],
                    "lidar313": lidar_array[313],
                    "lidar314": lidar_array[314],
                    "lidar315": lidar_array[315],
                    "lidar316": lidar_array[316],
                    "lidar317": lidar_array[317],
                    "lidar318": lidar_array[318],
                    "lidar319": lidar_array[319],
                    "lidar320": lidar_array[320],
                    "lidar321": lidar_array[321],
                    "lidar322": lidar_array[322],
                    "lidar323": lidar_array[323],
                    "lidar324": lidar_array[324],
                    "lidar325": lidar_array[325],
                    "lidar326": lidar_array[326],
                    "lidar327": lidar_array[327],
                    "lidar328": lidar_array[328],
                    "lidar329": lidar_array[329],
                    "lidar330": lidar_array[330],
                    "lidar331": lidar_array[331],
                    "lidar332": lidar_array[332],
                    "lidar333": lidar_array[333],
                    "lidar334": lidar_array[334],
                    "lidar335": lidar_array[335],
                    "lidar336": lidar_array[336],
                    "lidar337": lidar_array[337],
                    "lidar338": lidar_array[338],
                    "lidar339": lidar_array[339],
                    "lidar340": lidar_array[340],
                    "lidar341": lidar_array[341],
                    "lidar342": lidar_array[342],
                    "lidar343": lidar_array[343],
                    "lidar344": lidar_array[344],
                    "lidar345": lidar_array[345],
                    "lidar346": lidar_array[346],
                    "lidar347": lidar_array[347],
                    "lidar348": lidar_array[348],
                    "lidar349": lidar_array[349],
                    "lidar350": lidar_array[350],
                    "lidar351": lidar_array[351],
                    "lidar352": lidar_array[352],
                    "lidar353": lidar_array[353],
                    "lidar354": lidar_array[354],
                    "lidar355": lidar_array[355],
                    "lidar356": lidar_array[356],
                    "lidar357": lidar_array[357],
                    "lidar358": lidar_array[358],
                    "lidar359": lidar_array[359]
                    }
                csv_writer.writerow(info)

            f = open(path_lidar % number_of_files, "a")
            for k in range(len(lidar_array)-1):
                f.write(str(lidar_array[k])+',')
            f.write(str(lidar_array[-1]) + '\n')
            f.close()

        rate.sleep()
    #rospy.spin()

if __name__ == '__main__':
    main()
