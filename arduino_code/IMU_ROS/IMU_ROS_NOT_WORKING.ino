#include <MPU9250_asukiaaa.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <Adafruit_BMP280.h>
#include <geometry_msgs/Vector3.h>

Adafruit_BMP280 bme; // I2C
MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

ros::NodeHandle nh;
geometry_msgs::Vector3 gyro_data;
ros::Publisher IMU_pub("IMU_gyro_data", &gyro_data);





void setup() {
  Serial.begin(115200);
  Serial.println("start up");
  nh.initNode();
  while (!Serial);

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
#endif

  bme.begin();
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;

  nh.advertise(IMU_pub);

}

void loop() {

  if (mySensor.accelUpdate() == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
    Serial.print("accelX: " + String(aX));
    Serial.print("\taccelY: " + String(aY));
    Serial.print("\taccelZ: " + String(aZ));
    Serial.print("\taccelSqrt: " + String(aSqrt));
  }

  if (mySensor.gyroUpdate() == 0) {
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
    Serial.print("\tgyroX: " + String(gX));
    Serial.print("\tgyroY: " + String(gY));
    Serial.print("\tgyroZ: " + String(gZ));
  }

  if (mySensor.magUpdate() == 0) {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();
    Serial.print("\tmagX: " + String(mX));
    Serial.print("\tmaxY: " + String(mY));
    Serial.print("\tmagZ: " + String(mZ));
    Serial.print("\thorizontalDirection: " + String(mDirection));
  }

  Serial.println(""); // Add an empty line
  gyro_data.x=gX;
  gyro_data.y=gY;
  gyro_data.z=gZ;
  IMU_pub.publish(&gyro_data);
  }


____________________--
#include <ros.h>
#include <Adafruit_BMP280.h>
#include <MPU9250_asukiaaa.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>

Adafruit_BMP280 bme; // I2C
MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
aX=1;

ros::NodeHandle nh;
//std_msgs::Int16 gyro_data;
geometry_msgs::Vector3 gyro_data;
ros::Publisher IMU_pub("IMU_data", &gyro_data);  //here the publisher is initilazed with the publisher "name" the topic "name" and a pointer to the variable that is sent

void setup() {
  Serial.begin(9600);
  nh.initNode();
  nh.advertise(IMU_pub);
}

void loop() {
  gX = mySensor.gyroX();
  gY = mySensor.gyroY();
  gZ = mySensor.gyroZ();
  gyro_data.x=gX;
  gyro_data.y=gY;
  gyro_data.z=gZ;


  IMU_pub.publish(&gyro_data);
  nh.spinOnce();
}


1: use wire.h
2: publish each individuatly (check nic code)
