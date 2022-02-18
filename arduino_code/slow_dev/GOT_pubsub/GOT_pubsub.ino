#include <ros.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle  nh;

geometry_msgs::Vector3 estimate_xyz;
ros::Publisher robot_position_estimate("robot_position_estimate", &estimate_xyz);


void setup()
{
  Serial.begin(19200);
  Serial3.begin(115200);
  nh.initNode();
  nh.advertise(robot_position_estimate);
}

void loop()
{

  robot_position_estimate.publish( &estimate_xyz );

  if (Serial3.available()) {
    int inByte = Serial3.read();
    estimate_xyz.x = inByte;

  //  Serial.print(inByte, DEC);
  }

  nh.spinOnce();
  delay(500);
}
