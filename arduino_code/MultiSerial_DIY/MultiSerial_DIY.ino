#include <ros.h>
#include <geometry_msgs/Vector3.h>


//Ros setup initialization
//ros::NodeHandle nh;
//geometry_msgs::Vector3 estimate_xyz;
//ros::Publisher robot_position_estimate("robot_position_estimate", &estimate_xyz);


void setup() {
  // initialize both serial ports:
 // nh.initNode();
 // nh.advertise(robot_position_estimate);
  //char inByte;
  Serial.begin(19200);
  Serial3.begin(115200);
}

void loop() {
  //nh.spinOnce();
  //robot_position_estimate.publish( &estimate_xyz );
  //int placeholder = 0;
  //estimate_xyz.x = placeholder;
  //estimate_xyz.y = placeholder;
  //estimate_xyz.z = placeholder;
  //delay(1000);
  // read from port 1, send to port 0:
  char inByte = Serial3.read();
  if (Serial3.available()) {
    if (inByte == "0x02")
      Serial.print(inByte);
      Serial.print("|");
    if (inByte == "0x03")
      Serial.println("ENDBYTE");
  }
}
