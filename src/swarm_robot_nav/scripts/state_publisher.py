#!/usr/bin/env python3
## Libraries ##
import rospy
from sensor_msgs.msg import JointState
import tf
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
import math
import numpy
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry


## Variables ##
wheel_radius = 0.04
wheel_base = 0.229
x_pose = 0
y_pose = 0
z_pose = 0.02
n = 1920
pi = 3.14159
omega_robot=0
q = quaternion_from_euler(0, 0, 0)
vel_x=0
vel_y=0
global omega_right, omega_left, tick_right, tick_left, old_tick_right, old_tick_left


def callback_odom(data):
    omega_right = data.x
    omega_left = data.y
    tick_right = data.z
    tick_left = data.w
    m_r = -(omega_right*wheel_radius)*(wheel_base/2)
    m_l = -(omega_left*wheel_radius)*(wheel_base/2)
    omega_robot = m_l + m_r
    d_r = 2*pi*wheel_radius*((tick_right-old_tick_right)/n)
    d_l = 2*pi*wheel_radius*((tick_left-old_tick_left)/n)
    d_c = (d_r+d_l)/2
    theta = (d_r+d_l)/wheel_base + theta
    x_pose = x_pose + d_c*cos(theta)
    y_pose = y_pose + d_c*sin(theta)

    vel=(omega_right*wheel_radius)+(omega_left*wheel_radius)
    vel_x=vel*cos(theta)
    vel_y=vel*sin(theta)

    q = quaternion_from_euler(0, 0, theta)
    quat_msg = Quaternion(q[0],q[1],q[2],q[3])

    odom_trans.header.stamp = rospy.Time.now()
    odom_trans.transform.translation.x = x_pose
    odom_trans.transform.translation.y = y_pose
    odom_trans.transform.translation.z = z_pose
    odom_trans.transform.rotation = quat_msg

    br = tf.TransformBroadcaster()
    br.sendTransform(odom_trans)

    old_tick_right = tick_right
    old_tick_left = tick_left

def odom_pub():
    pub_odom = rospy.Publisher('odom',Odometry,queue_size=10)
    odom_data = Odometry()
    odom_data.header.frame_id = "odom"
    odom_data.child_frame_id = "base_link"
    odom_data.header.stamp = rospy.Time.now()
    odom_data.pose.pose.position.x = x_pose
    odom_data.pose.pose.position.y = y_pose
    odom_data.pose.pose.position.z = z_pose
    odom_data.pose.pose.orientation = (Quaternion(q[0],q[1],q[2],q[3]))
    odom_data.twist.twist.linear.x = vel_x
    odom_data.twist.twist.linear.y = vel_y
    odom_data.twist.twist.linear.z = 0
    odom_data.twist.twist.angular.x = 0
    odom_data.twist.twist.angular.y = 0
    odom_data.twist.twist.angular.z=omega_robot
    pub_odom.publish(odom_data)




def main():
    print("hi")
    rospy.Subscriber("speed_and_tick",Quaternion,callback_odom)
#    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
#    rospy.init_node('state_publisher', anonymous=True)

#    joint_state = JointState()
    odom_trans = TransformStamped()
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "base_link"
    while not rospy.is_shutdown():
        print("pee pee")
        odom_pub()

#        joint_state.header.stamp = rospy.Time.now()
#        joint_state.name[0] ="base_laser"
#        ##joint_state.position[0] = swivel
#        joint_state.name[1] ="base_footprint"
#        ##joint_state.position[1] = tilt
#        joint_state.name[2] ="lidar_to_footprint"
#        ##joint_state.position[2] = height
#        rospy.Subscriber("speed_and_tick", Quaternion, speed_writting)



if __name__ == '__main__':
    rospy.init_node('state_publisher', anonymous=True)
    main()
