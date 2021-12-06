cmake_minimum_required(VERSION 2.8.3)

project(rudolph_v4)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs tf)
catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
include_directories(include ${catkin_INCLUDE_DIRS})
