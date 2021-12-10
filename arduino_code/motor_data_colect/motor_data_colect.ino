
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>



//-----pinout setting up-----//
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
float reference_angle = 0.0; // a heading angle we want to be at - our goal angle
int mode_mode = 0;                      // initialzing the mode of the system
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

ros::NodeHandle nh;
geometry_msgs::Vector3 right_wheel_speed;
ros::Publisher data_pub("collection", &right_wheel_speed);
std_msgs::Int16 run_end;
ros::Publisher end_pub("end_of_run", &run_end);

void message_mode(std_msgs::Int16& mode_comand) {
  mode_mode = mode_comand.data;
}
ros::Subscriber<std_msgs::Int16> sub("mode_sig", &message_mode);

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
  int right_count_tick;
  if (encoder_counter_right < counts_per_revolution && encoder_counter_right > -counts_per_revolution) {
    if (direction_indicator_right == 1) {
      encoder_counter_right++;
      right_count_tick = 1;
      current_omega_right = count_to_rad / delta_time_right;
    }
    if (direction_indicator_right == 0) {
      encoder_counter_right = encoder_counter_right - 1;
      right_count_tick = -1;
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
  int left_count_tick;
  if (encoder_counter_left < counts_per_revolution && encoder_counter_left > -counts_per_revolution) {
    if (direction_indicator_left == 1) {
      encoder_counter_left++;
      left_count_tick = 1;
      current_omega_left = count_to_rad / delta_time_left;
    }
    if (direction_indicator_left == 0) {
      encoder_counter_left = encoder_counter_left - 1;
      left_count_tick = -1;
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
  if (current_omega_left < 20 && current_omega_left > -20) {
    array_push(speed_array_left, current_omega_left);
  }

  //left_tick.data = left_count_tick;
  //left_tick_pub.publish(&left_tick);

  average_omega_left = averaging_array(speed_array_left);
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

void setup() {
  Serial.begin(9600);
  Serial.println("Start up");
  nh.initNode();
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
  nh.advertise(data_pub);
  nh.advertise(end_pub);
  digitalWrite(right_motor_ina, HIGH);
  digitalWrite(left_motor_inb, HIGH);
}

void loop() {
  if (mode_mode == 1){
    for(int i = 0; i < 256; i++){
      for(int j = 0; j < 25; j++){
        setPWM(-i,-i);
        right_wheel_speed.x = average_omega_right;
        right_wheel_speed.y = average_omega_left;
        right_wheel_speed.z = i;
        data_pub.publish(&right_wheel_speed);
        nh.spinOnce();
        delay(8);
      }
    }
    mode_mode = 0;
    run_end.data = 0;
    end_pub.publish(&run_end);
    analogWrite(right_motor_pwm, 0);
    analogWrite(left_motor_pwm, 0);
  }
  nh.spinOnce();
}
