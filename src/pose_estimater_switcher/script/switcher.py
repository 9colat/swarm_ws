#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
estimator_switch = 6 ## this number is in meters.
pose_x = 0  ## just an initial coordinate in x
pose_y = 0  ## just an initial coordinate in y
pose_z = 0  ## just an initial coordinate in z
ups_pose_x = 0 ## just an initial pose estimate from ultrasound position system in x
ups_pose_y = 0 ## just an initial pose estimate from ultrasound position system in y
ups_pose_z = 0 ## just an initial pose estimate from ultrasound position system in z
lidar_pose_x = 0 ## just an initial pose estimate from lidar in x
lidar_pose_y = 0 ## just an initial pose estimate from lidar in y
lidar_pose_z = 0 ## just an initial pose estimate from lidar in z

## here we need to add the Subscriber for the lidar position estimate and the ups.

def callback_ups_pose(data):
    global ups_pose_x, ups_pose_y, ups_poze_z
    ups_pose_x = data.position.x
    ups_pose_y = data.position.y
    ups_pose_z = data.position.z
    print(ups_pose_x)


def callback_lidar_pose(data):
    global ups_lidar_x, ups_lidar_y, ups_lidar_z
    ups_lidar_x = data.position.x
    ups_lidar_y = data.position.y
    ups_lidar_z = data.position.z


def callback_lidar(data):
    global estimator_switch, pose_x, pose_y, pose_z, lidar_pose_x, lidar_pose_y, lidar_pose_z, ups_pose_x, ups_pose_y, ups_pose_z
    for i in range(len(data.ranges)):
        lidar_array[i] = data.ranges[i]
    if min(lidar_array) < estimator_switch:
        pose_x = lidar_pose_x
        pose_y = lidar_pose_y
        pose_z = lidar_pose_z
    else:
        pose_x = ups_pose_x
        pose_y = ups_pose_y
        pose_z = ups_pose_z


def switcher():
    global pose_x, pose_y, pose_z
    rospy.init_node('switcher', anonymous=True)
    pub = rospy.Publisher('position', Pose, queue_size=10)
    while True:
        rospy.Subscriber("scan", LaserScan, callback_lidar)
        rospy.Subscriber("ups_pose", Pose, callback_ups_pose)
        rospy.Subscriber("lidar_pose", Pose, callback_lidar_pose)

        data = Pose()
        data.position.x = pose_x
        data.position.y = pose_y
        data.position.z = pose_z
        pub.publish(data)
        rospy.spin()

if __name__ == '__main__':
    switcher()
