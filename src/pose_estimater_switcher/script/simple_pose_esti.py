#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from estimator_function import Pose_Calculator
from custom_msgs.msg import USPS_msgs

pose = np.array([0,0,0])

PC = Pose_Calculator()

# missing beacons
# id, x, y, z
# 44050, 7825, 9999, 4286
# 44539, 1999, 10677, 3531

def callback_distance(data):
    global pose
    beacon_id = [44539, 44050, 42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
    if data.ID in beacon_id:
        pose = PC.pose_estimator(data.ID, data.distance)


def main():
    global pose

    # initialize subscribers
    rospy.init_node('simple_pose_esti', anonymous=True) # initialize the node
    rospy.Subscriber("beacon_data", USPS_msgs, callback_distance)
    pub = rospy.Publisher('simple_pose_esti', Pose, queue_size=10)
    rate = rospy.Rate(100) # 100hz
    pose_est = Pose()



    while not rospy.is_shutdown():

        # time update for ONLY the predictor function in EKF
        pose_est.position.x = pose[0]
        pose_est.position.y = pose[1]
        #pose_est.position.x = pose[2]
        pub.publish(pose_est)
        #print("without: ", state[0],state[1])
        rate.sleep()


if __name__ == '__main__':
    main()
