#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU9250.h"
#include <ros.h>
#include <Wire.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>



//-----pinout setting up-----//
const int MPU_addr = 0x68; // I2C address of the MPU-6050 and mpu9250
const byte right_motor_pwm = 38;    // setting the pin for the right motors PWM
const byte right_motor_inb = 37;    // setting the pin for the right motors b direction pin
const byte right_motor_ina = 39;    // setting the pin for the right motors a direction pin
const byte right_encoder_a = 33;    // setting the pin for the right motors first encoder signal pin
const byte right_encoder_b = 34;    // setting the pin for the right motors secondt encoder signal pin
const byte left_motor_pwm = 29;     // setting the pin for the left motors PWM
const byte left_motor_inb = 28;     // setting the pin for the left motors b direction pin
const byte left_motor_ina = 30;     // setting the pin for the left motors a direction pin
const byte left_encoder_a = 24;     // setting the pin for the left motors first encoder signal pin
const byte left_encoder_b = 25;     // setting the pin for the left motors secondt encoder signal pin
const byte RGB_led_green = 2;
const byte RGB_led_blue = 3;
const byte RGB_led_red = 4;
//-----variabels-----//
int counts_per_revolution = 1920.0;   // the number of counts per full wheel revulotion
const float pi = 3.141593;          // this is pi, or an aproximation, what did you expeced?
double encoder_counter_right = 0.0;   // this is the encoder counter for the right wheel
double encoder_counter_left = 0.0;    // this is the encoder counter for the left wheel
int status_of_led = HIGH;           // this can be removed later
int direction_indicator_right;  //this is a direction indicator, that can be either 0 or 1. if the variable is 0 that means that the moters are going backwards, and 1 means forwards.
int direction_indicator_left;   //this is a direction indicator, that can be either 0 or 1. if the variable is 0 that means that the moters are going backwards, and 1 means forwards.
float count_to_deg = (360.0) / counts_per_revolution; //convertion constants for degrees
double count_to_rad = (2.0 * pi) / counts_per_revolution; //convertion constants for radians
float pwm_procent_right = 0.0;        // the PWM procentage, initialed to 0 for the right motor
float pwm_procent_left = 0.0;         // the PWM procentage, initialed to 0 for the left motor
int pwm_value_right;                // initialzing the PWM value aka. turning the procentage into a 8-bit value (0-255)
int pwm_value_left;                 // initialzing the PWM value aka. turning the procentage into a 8-bit value (0-255)
float measured_angle;  // a heading angle we get from the IMU
float reference_angle; // a heading angle we want to be at - our goal angle
int mode_mode;                      // initialzing the mode of the system
double delta_time_right = 0.0;             // initialzing the the differents in time that is used to calculate the angular velocity
double old_time_right = 0.0;    // setting the initial time of the system for the right motor
double delta_time_left = 0.0;              // initialzing the the differents in time that is used to calculate the angular velocity
double old_time_left = 0.0;     // setting the initial time of the system for the left motor
long speed_array_right[500];         // initialzing the array that holds the newest angular velocity values
long speed_array_left[500];          // initialzing the array that holds the newest angular velocity values
float current_omega_right;          // initialzing the current angular velocity for the right motor
float current_omega_left;           // initialzing the current angular velocity for the left motor
float average_omega_right;
float average_omega_left;
float float_to_long_factor = 10000.0;
float robot_radius = 1.0;             // needs to be updated and use the right unit (proberbly meters)
float wheel_radius = 1.0;             // needs to be updated and use the right unit (proberbly meters)
int16_t accel_X, accel_Y, accel_Z, tmp, gyro_X, gyro_Y, gyro_Z, mx, my, mz;
long publisher_timer;
MPU9250 accelgyro;
int mag_x_cal = -20; //magnetometer callibration in x direction
int mag_y_cal = -6; //magnetometer callibration in y direction


ros::NodeHandle nh;                 // here the node handler is set with the name nh
std_msgs::Int16 mode_confurm;       // the variable is initilazed as a Int16, this is a ros type that is the type that you can sent over the ros topics
ros::Publisher mode_pub("mode_repeat", &mode_confurm);  //here the publisher is initilazed with the publisher "name" the topic "name" and a pointer to the variable that is sent
std_msgs::Float32 angle_of_wheel;
ros::Publisher ankle_pub("wheel_angle", &angle_of_wheel);
std_msgs::Vector3 wheel_speed;
ros::Publisher speed_pub("wheel_speed", &wheel_speed);


struct Quaternion
{
    double w, x, y, z;
};

Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}


void array_push(long the_input_array[], float data) {
  for (int x = sizeof(the_input_array); x > 0; x = x - 1) {
    the_input_array[x] = the_input_array[x - 1];
  }
  the_input_array[0] = data * float_to_long_factor;
}

float averaging_array(long the_input_array[]) {
  float result = 0;
  for (int x = 0; x < sizeof(the_input_array); x++) {
    result = result + (the_input_array[x] / float_to_long_factor);
  }
  result = result / sizeof(the_input_array);
  return result;
}


void encoder_count_chage_right() {
  delta_time_right =  double(micros()) / 1000000 - old_time_right;
  old_time_right = double(micros()) / 1000000;
  if (encoder_counter_right < counts_per_revolution && encoder_counter_right > -counts_per_revolution) {
    if (direction_indicator_right == 1) {
      encoder_counter_right++;
      current_omega_right = count_to_rad / delta_time_right;

    }
    if (direction_indicator_right == 0) {
      encoder_counter_right = encoder_counter_right - 1;
      current_omega_right = -count_to_rad / delta_time_right;
    }
  }
  if (encoder_counter_right == counts_per_revolution) {
    if (direction_indicator_right == 1) {
      encoder_counter_right = 0;
      current_omega_right = count_to_rad / delta_time_right;
    }
  }
  if (encoder_counter_right == -counts_per_revolution) {
    if (direction_indicator_right == 0) {
      encoder_counter_right = 0;
      current_omega_right = -count_to_rad * 1.0 / delta_time_right;
    }


  }
  if (current_omega_right < 20 && current_omega_right > -20){
    array_push(speed_array_right, current_omega_right);
  }
  average_omega_right = averaging_array(speed_array_right);
}

void encoder_count_chage_left() {
  delta_time_left = double(micros()) / 1000000 - old_time_left;
  old_time_left = double(micros()) / 1000000;
  if (encoder_counter_left < counts_per_revolution && encoder_counter_left > -counts_per_revolution) {
    if (direction_indicator_left == 1) {
      encoder_counter_left++;
      current_omega_left = count_to_rad / delta_time_left;
    }
    if (direction_indicator_left == 0) {
      encoder_counter_left = encoder_counter_left - 1;
      current_omega_left = -count_to_rad / delta_time_left;
    }
  }
  if (encoder_counter_left == counts_per_revolution) {
    if (direction_indicator_left == 1) {
      encoder_counter_left = 0;
      current_omega_left = count_to_rad / delta_time_left;
    }
  }
  if (encoder_counter_left == -counts_per_revolution) {
    if (direction_indicator_left == 0) {
      encoder_counter_left = 0;
      current_omega_left = -count_to_rad * 1.0 / delta_time_left;
    }


  }
  if (current_omega_left < 20 && current_omega_left > -20){
    array_push(speed_array_left, current_omega_left);
  }
  average_omega_left = averaging_array(speed_array_left);
}


geometry_msgs::Vector3 imu_acc = geometry_msgs::Vector3();
ros::Publisher IMU_data_acc("imu_acc", &imu_acc);
geometry_msgs::Vector3 imu_gyro = geometry_msgs::Vector3();
ros::Publisher IMU_data_gyro("imu_gyro", &imu_gyro);
geometry_msgs::Vector3 imu_mag = geometry_msgs::Vector3();
ros::Publisher IMU_data_mag("imu_mag", &imu_mag);
//std_msgs::Float64 measured_angle = std_msgs::Float64();
//ros::Publisher data_measured_angle("measured_angle", &measured_angle);
geometry_msgs::Vector3 data_measured_angle = geometry_msgs::Vector3();
ros::Publisher datadata_measured_angle("data_measured_angle", &data_measured_angle);


void setPWM(int pwm_left, int pwm_right) {
  //setting the correct direction of the motor
  digitalWrite(right_motor_ina, pwm_right >= 0);
  digitalWrite(right_motor_inb, pwm_right < 0);
  digitalWrite(left_motor_ina, pwm_left >= 0);
  digitalWrite(left_motor_inb, pwm_left < 0);
  //setting the value of the motor
  pwm_right = abs(pwm_right);
  pwm_left = abs(pwm_left);
  if (pwm_left > 255) {
    pwm_left = 255;
  }
  if (pwm_right > 255) {
    pwm_right = 255;
  }
  analogWrite(right_motor_pwm, pwm_right);
  analogWrite(left_motor_pwm, pwm_left);
}




void message_pwm(geometry_msgs::Vector3& pwm_comand) {
  pwm_procent_right = pwm_comand.x;
  //pwm_value_right = map(pwm_procent_right, 0, 100, 0, 255);
  pwm_procent_left = pwm_comand.y;
  //pwm_value_left = map(pwm_procent_left, 0, 100, 0, 255);
  reference_angle = pwm_comand.z;
  setPWM(pwm_procent_left, pwm_procent_right);
}


ros::Subscriber<geometry_msgs::Vector3> sub("pwm_sig", &message_pwm);


void imu_collection() {
  accelgyro.getMotion9(&accel_X, &accel_Y, &accel_Z, &gyro_X, &gyro_Y, &gyro_Z, &mx, &my, &mz);

  imu_acc.x = accel_X;
  imu_acc.y = accel_Y;
  imu_acc.z = accel_Z;
  IMU_data_acc.publish(&imu_acc);

  imu_gyro.x = gyro_X;
  imu_gyro.y = gyro_Y;
  imu_gyro.z = gyro_Z;
  IMU_data_gyro.publish(&imu_gyro);
  // data from the magnetometer that is calibrated and turned into a heading
  imu_mag.x = mx;
  imu_mag.y = my;
  imu_mag.z = mz;
  IMU_data_mag.publish(&imu_mag);
  measured_angle = atan2(my - mag_y_cal, mx - mag_x_cal) * 180 / pi;
  data_measured_angle.x = mx;
  data_measured_angle.y = reference_angle;
  data_measured_angle.z = measured_angle;
  datadata_measured_angle.publish(&data_measured_angle);
}

void heading_controller(float measured_angle, float reference_angle, float p_gain) { //(measured, goal, p_value)
  int heading_error = reference_angle - measured_angle;
  int control_signal = heading_error * p_gain;
  setPWM(control_signal, - control_signal);
}

double encoder_to_unit(int encoder_count, int unit_output) { //if unit_output is 1 the unit is deg, if its 2 its rad
  double output_number;
  float temp_number;
  if (unit_output == 1) {
    output_number = encoder_count * count_to_deg;
    //output_number = temp_number % float(360);
    //output_number = map(encoder_count,-counts_per_revolution, counts_per_revolution, -360, 360);
  }
  if (unit_output == 2) {
    output_number = encoder_count * count_to_rad;
    //output_number = temp_number % (2*pi);
    //output_number = map(encoder_count,-counts_per_revolution, counts_per_revolution, -2*pi, 2*pi);
  }
  return output_number;
}

void RGB_led_set(const String& color){
  if(color == "red"||color == "Red"||color == "RED"){
    digitalWrite(RGB_led_green, HIGH);
    digitalWrite(RGB_led_blue, HIGH);
    digitalWrite(RGB_led_red, LOW);
  }
  if(color == "green"||color == "Green"||color == "GREEN"){
    digitalWrite(RGB_led_green, LOW);
    digitalWrite(RGB_led_blue, HIGH);
    digitalWrite(RGB_led_red, HIGH);
  }
  if(color == "blue"||color == "Blue"||color == "BLUE"){
    digitalWrite(RGB_led_green, HIGH);
    digitalWrite(RGB_led_blue, LOW);
    digitalWrite(RGB_led_red, HIGH);
  }
  if(color == "cyan"||color == "Cyan"||color == "CYAN"){
    digitalWrite(RGB_led_green, LOW);
    digitalWrite(RGB_led_blue, LOW);
    digitalWrite(RGB_led_red, HIGH);
  }
  if(color == "purple"||color == "Purple"||color == "PURPLE"){
    digitalWrite(RGB_led_green, HIGH);
    digitalWrite(RGB_led_blue, LOW);
    digitalWrite(RGB_led_red, LOW);
  }
  if(color == "orange"||color == "Orange"||color == "ORANGE"){
    digitalWrite(RGB_led_green, LOW);
    digitalWrite(RGB_led_blue, HIGH);
    digitalWrite(RGB_led_red, LOW);
  }
  if(color == "white "||color == "White"||color == "WHITE"){
    digitalWrite(RGB_led_green, LOW);
    digitalWrite(RGB_led_blue, LOW);
    digitalWrite(RGB_led_red, LOW);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Start up");
  nh.initNode();
  pinMode(RGB_led_green, OUTPUT);
  pinMode(RGB_led_blue, OUTPUT);
  pinMode(RGB_led_red, OUTPUT);
  RGB_led_set("white");
  pinMode(right_motor_pwm, OUTPUT);
  pinMode(right_motor_ina, OUTPUT);
  pinMode(right_motor_inb, OUTPUT);
  pinMode(right_encoder_a, INPUT_PULLUP);
  pinMode(right_encoder_b, INPUT_PULLUP);
  pinMode(left_motor_pwm, OUTPUT);
  pinMode(left_motor_ina, OUTPUT);
  pinMode(left_motor_inb, OUTPUT);
  pinMode(left_encoder_a, INPUT_PULLUP);
  pinMode(left_encoder_b, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(right_encoder_a), encoder_count_chage_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder_b), encoder_count_chage_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_encoder_a), encoder_count_chage_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_encoder_b), encoder_count_chage_left, CHANGE);
  nh.subscribe(sub);
  //  nh.subscribe(sub1);
  nh.advertise(mode_pub);
  nh.advertise(ankle_pub);
  nh.advertise(speed_pub);
  nh.advertise(IMU_data_acc);
  nh.advertise(IMU_data_gyro);
  nh.advertise(IMU_data_mag);
  nh.advertise(datadata_measured_angle);

  Wire.begin();
  //Wire.beginTransmission(MPU_addr);
  //Wire.write(0x6B);  // PWR_MGMT_1 register
  //Wire.write(0);     // set to zero (wakes up the MPU-6050)
  //Wire.endTransmission(true);
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");

}

void loop() {
  RGB_led_set("green");
  //  mode_confurm.data = test;
  //  mode_pub.publish(&mode_confurm);
  //  float test = encoder_to_unit(encoder_counter_right,1);
  //  angle_of_wheel.data = encoder_to_unit(encoder_counter_right,1);



  mode_confurm.data = speed_array_left[1];
  mode_pub.publish(&mode_confurm);

  wheel_speed.x = average_omega_right;
  wheel_speed.y = average_omega_left;
  speed_pub.publish(&wheel_speed);

  //in order to improve the measured_angle reading, you can do measured=(integral(gyro))*0.9+magnetometer*0.1
  //the integral is going to drift, but the magnetometer will correct it (you need to find the correct ratio)
  //in short term, we trust gyro, but in a long term, we trust magnetometer more

  //imu_collection();
  //heading_controller(measured_angle, reference_angle, 2.0); // 2.0 to use all the range (for now - testing purposes)
  //Serial.print("Measured:");
  //Serial.println(measured_angle);
  //Serial.print("Reference:");
  //Serial.println(reference_angle);


  nh.spinOnce();
}
