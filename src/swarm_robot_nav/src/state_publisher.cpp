// Libraries //

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>

// Variabler //
const double degree = M_PI/180;
const double wheel_radius = 0.04; // the radius of the wheels, unit is in meters.
const double wheel_base = 0.229; // the length between the wheels, unit is in meters.
double x_pose = 0; // the initial x coordinate
double y_pose = 0; // the initail y coordinate
double theta = 0; // the initial heading of the robot
double omega_right; // initilazed right wheel rotation velocity
double omega_left; // initilazed left wheel rotation velocity



// Call back functions //
void chatterCallback(const std_msgs::Quaternion::ConstPtr& msg){
  omega_right = msg.x;
  omega_left = msg.y;




  odom_trans.header.stamp = ros::Time::now();
  odom_trans.transform.translation.x = cos(angle);
  odom_trans.transform.translation.y = sin(angle);
  odom_trans.transform.translation.z = 0.02;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);


  broadcaster.sendTransform(odom_trans);
}






// Main loop //
int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(3);
        joint_state.position.resize(3);
        joint_state.name[0] ="base_laser";
        //joint_state.position[0] = swivel;
        joint_state.name[1] ="base_footprint";
        //joint_state.position[1] = tilt;
        joint_state.name[2] ="lidar_to_footprint";
        //joint_state.position[2] = height;





        //send the joint state and transform
        joint_pub.publish(joint_state);



        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
