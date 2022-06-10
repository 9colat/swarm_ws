#!/usr/bin/env python3
import roslaunch
import rospy
import time
import sys
from pathlib import Path
from custom_msgs.msg import Q_R

path = str(Path.home().joinpath("swarm_ws/src/swarm_robot_nav/launch","turning.launch"))

rospy.init_node('turning', anonymous=True)
pub1 = rospy.Publisher('q_and_r', Q_R, queue_size=10)
pub2 = rospy.Publisher('indicator_color', Int16, queue_size=10)
fun = Int16()
indicator = Q_R()

for r in range(0.0001, 1):
    for q in range(200, 300):
        print("boot")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
        launch.start()
        indicator.Q.data = q
        indicator.R.data = r
        time.sleep(10)
        fun.data = 1
        pub2.publish(fun)
        pub1.publish(indicator)
        rospy.spin()
        time.sleep(40)
        fun.data = 30
        pub2.publish(fun)
        launch.shutdown()
        time.sleep(10)
