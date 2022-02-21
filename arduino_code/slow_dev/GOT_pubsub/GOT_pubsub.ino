#include <ros.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle  nh;

geometry_msgs::Vector3 estimate_xyz;
ros::Publisher robot_position_estimate("robot_position_estimate", &estimate_xyz);

float EscapeByte = 0x10;
float StartByte = 0x02;
float StopByte = 0x03;

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
    char inByte = Serial3.read();
    estimate_xyz.x = inByte;
    if (inByte == StartByte) {
      Serial.print("Start");
    }
    if (inByte == StopByte) {
      Serial.println("|Stop");
      //      Serial.print(inByte, DEC);
    }
    else {
      Serial.print("|");
      Serial.print(inByte, DEC);
     // Serial.print("|");

    }
  }

  nh.spinOnce();
  //delay(500);
}
