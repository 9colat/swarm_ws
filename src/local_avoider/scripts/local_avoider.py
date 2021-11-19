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


def callback_lidar(data):
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
        if lidar_array[i] > 0.3 and lidar_array[i] < 0.5:
            print("hey i meet just you")
    print(lidar_array[i])

def cartian_to_polar(current_pose_x, current_pose_y, quat_x, quat_y, quat_z, quat_w , goal_pose_x, goal_pose_y, lidar_array_temp):
    t3 = +2.0 * (quat_w * quat_z + quat_x * quat_y)
    t4 = +1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z)
    current_yaw = math.degrees(math.atan2(t3, t4))
    current_theta = math.atan2(t3, t4)

    global_delta_x = goal_pose_x - current_pose_x
    global_delta_y = goal_pose_y - current_pose_y

    goal_theta = math.atan2(global_delta_y, global_delta_x)

    print("goal angle: ", math.degrees(goal_theta),"robot angle: ", math.degrees(current_theta) , "delta angle: ", math.degrees(current_theta-goal_theta))
    r = math.sqrt(pow(global_delta_x,2) + pow(global_delta_y,2))
    goal_heading = (current_theta - goal_theta)
    #v = ((numpy.arctan(r-4))/1)+1
    v = 50
    for i in range(int(goal_heading) - 5, int(goal_heading) + 5)
        if lidar_array[i] < 0.6 and lidar_array[i] > 0.4:

    #print("speed factor: ", math.cos(heading))


    return goal_heading


def lidar_data():
    rospy.init_node('local_avoider', anonymous=True)

    quat = quaternion_from_euler(0, 0, math.radians(robot_angle))
    print(math.degrees(cartian_to_polar(robot_pose[0],robot_pose[1],quat[0],quat[1],quat[2],quat[3],goal_pose[0],goal_pose[1],lidar_array)))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    lidar_data()
