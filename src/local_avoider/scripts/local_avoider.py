#! /usr/bin/env python3
import rospy

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import math
import numpy
lidar_array = [0] * 360
robot_angle = 90
robot_pose = [12,1]
goal_pose = [13,-12]
robot_point_x = 0.0
robot_point_y = 0.0
robot_point_z = 0.0
robot_ori_x = 0.0
robot_ori_y = 0.0
robot_ori_z = 0.0
robot_ori_w = 0.0
quat_heading_x = 0.0
quat_heading_y = 0.0
quat_heading_z = 0.0






def callback_robot_pose(data):
    print("i got the robot pose")
    robot_point_x = data.poses.pose.position.x
    robot_point_y = data.poses.pose.position.y
    robot_point_z = data.poses.pose.position.z
    robot_ori_x = data.poses.pose.orientation.x
    robot_ori_y = data.poses.pose.orientation.y
    robot_ori_z = data.poses.pose.orientation.z
    robot_ori_w = data.poses.pose.orientation.w
    return

def callback_goal_path(data):
    print("goal point have been aquired")
    quat_heading_x = data.poses.pose.position.x
    quat_heading_y = data.poses.pose.position.y
    quat_heading_z = data.poses.pose.position.z
    return

def callback_lidar(data):
    print("lidar is running")
    for i in range(len(data.ranges)):
        lidar_array[i] = data.ranges[i]
        if lidar_array[i] < 0.3:
            pub_signal = rospy.Publisher('danger_signal', Vector3, queue_size=10)
            pub_quat = rospy.Publisher('danger_quar', Quaternion, queue_size=10)
            j = 0
            q = quaternion_from_euler(0, 0, math.radians(i))
            quat_msg = Quaternion(q[0],q[1],q[2],q[3])
            mat = Vector3()
            mat.x = data.ranges[i]
            mat.y = i
            mat.z = j
            print("You are too close at angle: "+str(i))
            pub_signal.publish(mat)
            pub_quat.publish(quat_msg)
            print(q)
            j = j + 1
    print(" ")

def cartian_to_polar(current_pose_x, current_pose_y, quat_x, quat_y, quat_z, quat_w , goal_pose_x, goal_pose_y):
    t3 = +2.0 * (quat_w * quat_z + quat_x * quat_y)
    t4 = +1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z)
    current_yaw = math.degrees(math.atan2(t3, t4))
    current_theta = math.atan2(t3, t4)

    global_delta_x = goal_pose_x - current_pose_x
    global_delta_y = goal_pose_y - current_pose_y

    goal_theta = math.atan2(global_delta_y, global_delta_x)

    print("goal angle: ", math.degrees(goal_theta),"robot angle: ", math.degrees(current_theta) , "delta angle: ", math.degrees(current_theta-goal_theta))
    pub_heading = rospy.Publisher('heading_quat', Quaternion, queue_size=10)
    r = math.sqrt(pow(global_delta_x,2) + pow(global_delta_y,2))
    goal_heading = (current_theta - goal_theta)
    #v = ((numpy.arctan(r-4))/1)+1
    v = 50
    obstical_length = [0]*10
    obstical_angle = [0]*10
    obstical_x = [0]*10
    obstical_y = [0]*10
    array_of_obstical_angle = [0]*5
    j = 0
    for i in range(int(goal_heading) - 5, int(goal_heading) + 5):
        if lidar_array[i] < 0.6 and lidar_array[i] > 0.4:
            obstical_length[j] = lidar_array[i]
            obstical_angle[j] = i
        j = j + 1
    for x in range(5):
        if obstical_length[x] > 0 and obstical_length[9 - x] > 0:
            obstical_x[x] = obstical_length[x] * math.cos(obstical_angle[x])
            obstical_y[x] = obstical_length[x] * math.sin(obstical_angle[x])
            obstical_x[10-x] = obstical_length[10-x] * math.cos(obstical_angle[10-x])
            obstical_y[10-x] = obstical_length[10-x] * math.cos(obstical_angle[10-x])
            array_of_obstical_angle[x] = math.atan2((obstical_y[x]-obstical_y[10-x]),(obstical_x[x]-obstical_x[10-x]))
    #print("speed factor: ", math.cos(heading))
    sum_of_obstical_angle = (array_of_obstical_angle[0]+array_of_obstical_angle[1]+array_of_obstical_angle[2]+array_of_obstical_angle[3]+array_of_obstical_angle[4])/5
    if sum_of_obstical_angle != 0:
        goal_heading = sum_of_obstical_angle

    q1 = quaternion_from_euler(0, 0, goal_heading)
    quat_heading = Quaternion(q1[0],q1[1],q1[2],q1[3])
    pub_heading.publish(quat_heading)

    return


def lidar_data():
    rospy.init_node('local_avoider', anonymous=True)

    #quat = quaternion_from_euler(0, 0, math.radians(robot_angle))
    while True:
        rospy.Subscriber("scan", LaserScan, callback_lidar)
        rospy.Subscriber("goal_path", Path, callback_goal_path)
        rospy.Subscriber("robot_pose", Path, callback_robot_pose)
        cartian_to_polar(robot_point_x, robot_point_y,robot_ori_x, robot_ori_y, robot_ori_z, robot_ori_w,quat_heading_x, quat_heading_y)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    lidar_data()
