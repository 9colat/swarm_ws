#!/usr/bin/env python3
import roslaunch
import rospy
import time
import sys
from pathlib import Path
from std_msgs.msg import Int16
#print("hello")
path = str(Path.home().joinpath("swarm_ws/src/swarm_robot_nav/launch","StaticTestigAndLog.launch"))

rospy.init_node('en_Mapping', anonymous=True)
pub1 = rospy.Publisher('indicator_color', Int16, queue_size=10)
indicator = Int16()

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
#print("there")
launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
#print("general")
launch.start()
#print("kenobi")
local_time = time.time()
pub_time = 10
first_time = True
desired_time = 20 * 60 # we want the test to be run for 20 min, and convert it to seconds
test_time_with_over_head_takken_to_a_count = desired_time + (desired_time/100)*25 # we add 25% time to acount for overhead.
test_time_with_over_head_takken_to_a_count = 30 * 60
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    if (time.time() - local_time) > pub_time and first_time:
        indicator.data = 1
        pub1.publish(indicator)
        first_time = False
        print("it is now time for a sleep")
    if (time.time() - local_time) > test_time_with_over_head_takken_to_a_count:
        print("im awake again")
        indicator.data = 255
        pub1.publish(indicator)
        launch.shutdown()
        rospy.spin()


    rate.sleep()
