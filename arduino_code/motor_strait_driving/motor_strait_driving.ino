//-----Libraries-----//
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU9250.h"
#include <ros.h>
#include <Wire.h>
#include <math.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>


double error_l;
double error_r;

//-----pinout setting up-----//
const int MPU_addr = 0x68; // I2C address of the MPU-6050 and mpu9250
const byte right_motor_pwm = 29;     // setting the pin for the left motors PWM
const byte right_motor_inb = 28;     // setting the pin for the left motors b direction pin
const byte right_motor_ina = 30;     // setting the pin for the left motors a direction pin
const byte right_encoder_a = 24;     // setting the pin for the left motors first encoder signal pin
const byte right_encoder_b = 25;     // setting the pin for the left motors secondt encoder signal pin
const byte left_motor_pwm = 38;    // setting the pin for the right motors PWM
const byte left_motor_inb = 37;    // setting the pin for the right motors b direction pin
const byte left_motor_ina = 39;    // setting the pin for the right motors a direction pin
const byte left_encoder_a = 33;    // setting the pin for the right motors first encoder signal pin
const byte left_encoder_b = 34;    // setting the pin for the right motors secondt encoder signal pin
const byte RGB_led_green = 2;
const byte RGB_led_blue = 3;
const byte RGB_led_red = 4;

//-----variabels-----//
const int counts_per_revolution = 1920.0;   // the number of counts per full wheel revulotion
const float pi = 3.141593;          // this is pi, or an aproximation, what did you expeced?
const float max_vel = 13.0;
const float max_omega = 1.0;
const float p_gain = 2.0;
const float i_gain = 0.5; //0.3 is roughly good, a little slow
const float d_gain = 0.0;
const double a = 0.1899;
const double b = -3.462;
const double c = 23.78;
const double d = -9.307;
const double a_2 = 1.72;
const double b_2 = 22.88;
const int change_over_point = 2;
const int opper_lim = 255;
bool bool_tele_op_toggel = true;
int period = 100;
unsigned long time_now = 0;
double encoder_counter_right = 0.0;   // this is the encoder counter for the right wheel
double encoder_counter_left = 0.0;    // this is the encoder counter for the left wheel
int status_of_led = HIGH;           // this can be removed later
int direction_indicator_right;  //this is a direction indicator, that can be either 0 or 1. if the variable is 0 that means that the moters are going backwards, and 1 means forwards.
int direction_indicator_left;   //this is a direction indicator, that can be either 0 or 1. if the variable is 0 that means that the moters are going backwards, and 1 means forwards.
float count_to_deg = (360.0) / counts_per_revolution; //convertion constants for degrees
double count_to_rad = (2.0 * pi) / counts_per_revolution; //convertion constants for radians
int pwm_procent_right = 0;        // the PWM procentage, initialed to 0 for the right motor
int pwm_procent_left = 0;         // the PWM procentage, initialed to 0 for the left motor
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
long current_time;
long previous_time;
double elapsed_time;
double last_error_right;
double cum_error_r;
double cum_error_l;
double rate_error_right;
double last_error_left;
double cum_error_left;
double rate_error_left;
double goal_omega_right;
double goal_omega_left;
float float_to_long_factor = 10000.0;
int right_count_tick;
int left_count_tick;
float wheel_base = 0.229;            // needs to be updated and use the right unit (proberbly meters)
float wheel_radius = 0.04;           // needs to be updated and use the right unit (proberbly meters)
int16_t accel_X, accel_Y, accel_Z, tmp, gyro_X, gyro_Y, gyro_Z, mx, my, mz;
long publisher_timer;
MPU9250 accelgyro;
int mag_x_cal = -20; //magnetometer callibration in x direction
int mag_y_cal = -6; //magnetometer callibration in y direction
int hi;

//-----ROS node handler-----//
ros::NodeHandle nh;   // here the node handler is set with the name nh

//-----Setting up publishers and there types-----//
std_msgs::Int16 right_tick;       // the variable is initilazed as a Int16, this is a ros type that is the type that you can sent over the ros topics
ros::Publisher right_tick_pub("right_tick", &right_tick);  //here the publisher is initilazed with the publisher "name" the topic "name" and a pointer to the variable that is sent
std_msgs::Int16 left_tick;       // the variable is initilazed as a Int16, this is a ros type that is the type that you can sent over the ros topics
ros::Publisher left_tick_pub("left_tick", &left_tick);  //here the publisher is initilazed with the publisher "name" the topic "name" and a pointer to the variable that is sent
std_msgs::Float32 angle_of_wheel;
ros::Publisher ankle_pub("wheel_angle", &angle_of_wheel);
geometry_msgs::Quaternion wheel_speed;
ros::Publisher speed_pub("speed_and_tick", &wheel_speed);
geometry_msgs::Vector3 imu_acc = geometry_msgs::Vector3();
ros::Publisher IMU_data_acc("imu_acc", &imu_acc);
geometry_msgs::Vector3 imu_gyro = geometry_msgs::Vector3();
ros::Publisher IMU_data_gyro("imu_gyro", &imu_gyro);
geometry_msgs::Vector3 imu_mag = geometry_msgs::Vector3();
ros::Publisher IMU_data_mag("imu_mag", &imu_mag);
geometry_msgs::Vector3 data_measured_angle = geometry_msgs::Vector3();
ros::Publisher measured_angle_pub("measured_angle", &data_measured_angle);


//-----Functions-----//
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
    //  encoder_counter_right++;
      right_count_tick += 1;
      current_omega_right = count_to_rad / delta_time_right;
    }
    if (direction_indicator_right == 0) {
    //  encoder_counter_right = encoder_counter_right - 1;
      right_count_tick += -1;
      current_omega_right = -count_to_rad / delta_time_right;
    }
  }


  if (current_omega_right < 20 && current_omega_right > -20) {
    array_push(speed_array_right, current_omega_right);
  }

  //right_tick.data = right_count_tick;
  //right_tick_pub.publish(&right_tick);
  average_omega_right = averaging_array(speed_array_right);
}

void encoder_count_chage_left() {
  delta_time_left = double(micros()) / 1000000 - old_time_left;
  old_time_left = double(micros()) / 1000000;
  if (encoder_counter_left < counts_per_revolution && encoder_counter_left > -counts_per_revolution) {
    if (direction_indicator_left == 1) {
    //  encoder_counter_left++;
      left_count_tick += 1;
      current_omega_left = count_to_rad / delta_time_left;
    }
    if (direction_indicator_left == 0) {
    //  encoder_counter_left = encoder_counter_left - 1;
      left_count_tick += -1;
      current_omega_left = -count_to_rad / delta_time_left;
    }
  }

  if (current_omega_left < 20 && current_omega_left > -20) {
    array_push(speed_array_left, current_omega_left);
  }

  //left_tick.data = left_count_tick;
  //left_tick_pub.publish(&left_tick);

  average_omega_left = averaging_array(speed_array_left);
}

int omega_to_pwm(double x){
  double pwm_cal;
  int sign;
  if(x < 0){
    sign = -1;
  }
  else{
    sign = 1;
  }
  if (abs(x) > change_over_point){
    pwm_cal = a*pow(abs(x),3)+b*pow(abs(x),2)+c*abs(x)+d;
  }
  if (abs(x) <= change_over_point && abs(x) > 0.01){
    pwm_cal = a_2*abs(x) + b_2;
  }
  if (pwm_cal>opper_lim){
    pwm_cal = opper_lim;
  }
  int pwm_int = sign * pwm_cal;

  return pwm_int;
}


void setPWM(int pwm_right, int pwm_left) {
  //setting the correct direction of the motor
  direction_indicator_right = 0;
  direction_indicator_left = 0;
  if(pwm_right >= 0){
    direction_indicator_right = 1;
  }
  if(pwm_left >= 0){
    direction_indicator_left = 1;
  }
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

void speed_PID_controller(double goal_wheel_speed_r, double current_wheel_speed_r, double last_error_r, double goal_wheel_speed_l, double current_wheel_speed_l, double last_error_l, double elapsed_time){
  //double error_r = goal_wheel_speed_r - current_wheel_speed_r;
  error_r = goal_wheel_speed_r - current_wheel_speed_r;
  //double error_l = goal_wheel_speed_l - current_wheel_speed_l;
  error_l = goal_wheel_speed_l - current_wheel_speed_l;
  cum_error_r += error_r * elapsed_time;
  cum_error_l += error_l * elapsed_time;
  double rate_error_r = (error_r - last_error_r) / elapsed_time;
  double rate_error_l = (error_l - last_error_l) / elapsed_time;
  double controller_output_r = error_r * p_gain + cum_error_r * i_gain + rate_error_r * d_gain;
  double controller_output_l = error_l * p_gain + cum_error_l * i_gain + rate_error_l * d_gain;
  double pwm_signal_r = omega_to_pwm(controller_output_r);
  double pwm_signal_l = omega_to_pwm(controller_output_l);
  setPWM(pwm_signal_r,pwm_signal_l);
}

void wheel_speed_set(double input_vel_x, double input_omega, bool tele_op){
  double vel_x_goal;
  double goal_omega;

  if (tele_op == true){
    RGB_led_set("cyan");
    // here we assume that the imput is for input_vel_x is between 0 - 1 and
    // input_omega is between 0 - 0.5
    vel_x_goal = input_vel_x * 75;
    goal_omega = input_omega * 50;
    goal_omega_right = vel_x_goal + goal_omega;
    goal_omega_left = vel_x_goal - goal_omega;
    pwm_procent_right = int(map(goal_omega_right, 0, 100, 0, 255));
    pwm_procent_left = int(map(goal_omega_left, 0, 100, 0, 255));
    setPWM(pwm_procent_right, pwm_procent_left);

  }
  else if(tele_op == false){
    RGB_led_set("green");
    // here we assume that the imput gives a goal_omega =< 15 [rad/s]
    vel_x_goal = input_vel_x;
    goal_omega = input_omega;
    goal_omega_right = (2*vel_x_goal + wheel_base*goal_omega)/(4*wheel_radius);
    goal_omega_left = (2*vel_x_goal - wheel_base*goal_omega)/(4*wheel_radius);
    double time_elapsed = double(current_time - previous_time);
    if(goal_omega_right > 15){
      goal_omega_right = 15;
    }
    if(goal_omega_left > 15){
      goal_omega_left = 15;
    }
    speed_PID_controller(goal_omega_right, average_omega_right, last_error_right, goal_omega_left, average_omega_left, last_error_left, time_elapsed);

  }
  //wheel_speed.x = vel_x_goal;
  //wheel_speed.y = goal_omega;
  //wheel_speed.z = goal_omega_right;
  //wheel_speed.w = goal_omega_left;

  //speed_pub.publish(&wheel_speed);
}

void start_up_hi(std_msgs::Int16& num){
  hi = num.data;
  if (hi == 1){
    RGB_led_set("green");
  }
  if (hi == 0){
    RGB_led_set("red");
  }
}

void cmd_velocity(geometry_msgs::Twist& cmd_goal) {
  cum_error_r = 0;
  cum_error_l = 0;
  previous_time = micros();
  double goal_vel_x = cmd_goal.linear.x;
  double goal_omega = cmd_goal.angular.z;
  if (cmd_goal.angular.x == 0){ // here if this is true that means that the robot is being teleoperated
    bool_tele_op_toggel = true;
  }
  if (cmd_goal.angular.x > 0){
    bool_tele_op_toggel = false;
  }
  //if(tele_op_toggel == 0.5 || tele_op_toggel == -0.5){
  //  bool_tele_op_toggel = !bool_tele_op_toggel;
  //}
  wheel_speed_set(goal_vel_x, goal_omega, bool_tele_op_toggel);
}





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
  measured_angle_pub.publish(&data_measured_angle);

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

void RGB_led_set(const String& color) {
  if (color == "red" || color == "Red" || color == "RED") {
    digitalWrite(RGB_led_green, HIGH);
    digitalWrite(RGB_led_blue, HIGH);
    digitalWrite(RGB_led_red, LOW);
  }
  if (color == "green" || color == "Green" || color == "GREEN") {
    digitalWrite(RGB_led_green, LOW);
    digitalWrite(RGB_led_blue, HIGH);
    digitalWrite(RGB_led_red, HIGH);
  }
  if (color == "blue" || color == "Blue" || color == "BLUE") {
    digitalWrite(RGB_led_green, HIGH);
    digitalWrite(RGB_led_blue, LOW);
    digitalWrite(RGB_led_red, HIGH);
  }
  if (color == "cyan" || color == "Cyan" || color == "CYAN") {
    digitalWrite(RGB_led_green, LOW);
    digitalWrite(RGB_led_blue, LOW);
    digitalWrite(RGB_led_red, HIGH);
  }
  if (color == "purple" || color == "Purple" || color == "PURPLE") {
    digitalWrite(RGB_led_green, HIGH);
    digitalWrite(RGB_led_blue, LOW);
    digitalWrite(RGB_led_red, LOW);
  }
  if (color == "orange" || color == "Orange" || color == "ORANGE") {
    digitalWrite(RGB_led_green, LOW);
    digitalWrite(RGB_led_blue, HIGH);
    digitalWrite(RGB_led_red, LOW);
  }
  if (color == "white " || color == "White" || color == "WHITE") {
    digitalWrite(RGB_led_green, LOW);
    digitalWrite(RGB_led_blue, LOW);
    digitalWrite(RGB_led_red, LOW);
  }
}

// Subscribers //
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &cmd_velocity);
ros::Subscriber<std_msgs::Int16> start_up("stat_up_done", &start_up_hi);

void setup() {
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
  nh.subscribe(sub_cmd_vel);
  nh.subscribe(start_up);
  nh.advertise(right_tick_pub);
  nh.advertise(left_tick_pub);
  nh.advertise(ankle_pub);
  nh.advertise(speed_pub);
  nh.advertise(IMU_data_acc);
  nh.advertise(IMU_data_gyro);
  nh.advertise(IMU_data_mag);
  nh.advertise(measured_angle_pub);


  Wire.begin();
  //Wire.beginTransmission(MPU_addr);
  //Wire.write(0x6B);  // PWR_MGMT_1 register
  //Wire.write(0);     // set to zero (wakes up the MPU-6050)
  //Wire.endTransmission(true);
  accelgyro.initialize();


}

void loop() {
  if(bool_tele_op_toggel == false){
    current_time = micros();
    double time_elapsed = double(current_time - previous_time)*pow(10,-6);
    speed_PID_controller(goal_omega_right, average_omega_right, last_error_right, goal_omega_left, average_omega_left, last_error_left, time_elapsed);

    previous_time = current_time;
    last_error_right = goal_omega_right - average_omega_right;
    last_error_left = goal_omega_left - average_omega_left;
  }

  if(millis() > time_now + period){
    time_now = millis();

    /*wheel_speed.x = average_omega_right;
    wheel_speed.y = average_omega_left;
    wheel_speed.z = right_count_tick;
    wheel_speed.w = left_count_tick;*/
    wheel_speed.x = error_r;
    wheel_speed.y = error_l;
    wheel_speed.z = cum_error_r;
    wheel_speed.w = cum_error_l;
    speed_pub.publish(&wheel_speed);
  }
  nh.spinOnce();
  //}
  //delay(100);
  }
