#!/usr/bin/env python3
import roslaunch
import rospy
import time
from pathlib import Path

path = str(Path.home().joinpath("swarm_ws/src/swarm_robot_nav/launch","StaticTestigAndLog.launch"))

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
launch.start()

desired_time = 20 * 60 # we want the test to be run for 20 min, and convert it to seconds
test_time_with_over_head_takken_to_a_count = desired_time + (desired_time/100)*25 # we add 25% time to acount for overhead.
time.sleep(test_time_with_over_head_takken_to_a_count) # there is about 20% overhead on the time.
launch.shutdown()
