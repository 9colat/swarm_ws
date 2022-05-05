import numpy as np
from estimator_function import Pose_Calculator

pose = np.array([0,0,0])

PC = Pose_Calculator()
PC.pose_estimator(ID[j], meas)

def callback_distance(data):
    global pose
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
        pose_est.position.x = pose[2]
        pub.publish(pose_est)
        #print("without: ", state[0],state[1])
        rate.sleep()


if __name__ == '__main__':
    main()
