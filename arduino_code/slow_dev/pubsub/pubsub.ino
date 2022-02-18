/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

 #include <ros.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle  nh;

geometry_msgs::Vector3 estimate_xyz;
ros::Publisher robot_position_estimate("robot_position_estimate", &estimate_xyz);

int placeholder_msg = 12 ; 

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(robot_position_estimate);
}

void loop()
{
   
  estimate_xyz.x = placeholder_msg;
  robot_position_estimate.publish( &estimate_xyz );
  nh.spinOnce();
  delay(500);
}
