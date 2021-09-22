#include <ros.h>
#include <Adafruit_BMP280.h>
#include <MPU9250_asukiaaa.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>


ros::NodeHandle nh;
//std_msgs::Int16 gyro_data;
geometry_msgs::Vector3 gyro_data;
ros::Publisher IMU_pub("mode_repeat", &gyro_data);  //here the publisher is initilazed with the publisher "name" the topic "name" and a pointer to the variable that is sent

void setup() {
  Serial.begin(9600);
  Serial.println("Start up");
  nh.initNode();
  nh.advertise(IMU_pub);
}

void loop() {
  gyro_data.x = 1;
  IMU_pub.publish(&gyro_data);
  nh.spinOnce();
}
