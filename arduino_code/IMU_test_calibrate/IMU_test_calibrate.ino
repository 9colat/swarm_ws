//-----Libraries-----//
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU9250.h"
#include <Adafruit_BMP280.h>
#include <ros.h>
#include <custom_msgs/odom_and_imu.h>

//-----Pinout setting up-----//
const int MPU_addr = 0x68; // I2C address of the MPU-6050 and mpu9250

//-----Variabels-----//
unsigned long time_now = 0;
const int period = 1000;
int16_t accel_X, accel_Y, accel_Z, tmp, gyro_X, gyro_Y, gyro_Z, mx, my, mz;
MPU9250 accelgyro;
Adafruit_BMP280 bmp; // use I2C interface
//-----ROS node handler-----//
ros::NodeHandle nh;   // here the node handler is set with the name nh

//-----Setting up publishers and there types-----//
custom_msgs::odom_and_imu data_measured_odom_and_imu = custom_msgs::odom_and_imu();
ros::Publisher odom_and_IMU_pub("odometry_and_IMU", &data_measured_odom_and_imu);


//-----IMU data colection function-----//
void imu_collection() {
  Serial.println("hello");
  accelgyro.getMotion9(&accel_X, &accel_Y, &accel_Z, &gyro_X, &gyro_Y, &gyro_Z, &mx, &my, &mz);
  Serial.println("there");
  data_measured_odom_and_imu.imu_acc.x = accel_X;
  data_measured_odom_and_imu.imu_acc.y = accel_Y;
  data_measured_odom_and_imu.imu_acc.z = accel_Z;
  Serial.println(gyro_Z);
  data_measured_odom_and_imu.imu_gyro.x = gyro_X;
  data_measured_odom_and_imu.imu_gyro.y = gyro_Y;
  data_measured_odom_and_imu.imu_gyro.z = gyro_Z;

  data_measured_odom_and_imu.imu_mag.x = mx;
  data_measured_odom_and_imu.imu_mag.y = my;
  data_measured_odom_and_imu.imu_mag.z = mz;


}


//-----Setup function, put your setup code here, to run once-----//
void setup() {
  nh.initNode();
  accelgyro.initialize();
  Serial.begin(9600);
  nh.advertise(odom_and_IMU_pub);
  pinMode(LED_BUILTIN, OUTPUT);
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  if (!status) {
    nh.loginfo("Could not find a valid BMP280 sensor, check wiring");
  }
  
}

//-----Loop function, put your main code here, to run repeatedly-----//
void loop() {
  //Serial.println("loop start");
  digitalWrite(LED_BUILTIN, LOW); 
  if(millis() > time_now + period){
    Serial.println("time bit");
    time_now = millis();
    imu_collection();
    Serial.println("time bit 2");
    //odom_and_IMU_pub.publish(&data_measured_odom_and_imu);
  }
  nh.spinOnce();
}
