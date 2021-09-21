#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>


//-----pinout setting up-----//
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
//-----variabels-----//
int counts_per_revolution = 1920;   // the number of counts per full wheel revulotion
const float pi = 3.141593;          // this is pi, or an aproximation, what did you expeced?
double encoder_counter_right = 0;   // this is the encoder counter for the right wheel
double encoder_counter_left = 0;    // this is the encoder counter for the left wheel
int status_of_led = HIGH;           // this can be removed later
int direction_indicator_right = 1;  //this is a direction indicator, that can be either 0 or 1. if the variable is 0 that means that the moters are going backwards, and 1 means forwards.
int direction_indicator_left = 1;   //this is a direction indicator, that can be either 0 or 1. if the variable is 0 that means that the moters are going backwards, and 1 means forwards.
double count_to_deg = (360)/counts_per_revolution;  //convertion constants for degrees
double count_to_rad = (2*pi)/counts_per_revolution; //convertion constants for radians
float pwm_procent_right = 0;        // the PWM procentage, initialed to 0 for the right motor
float pwm_procent_left = 0;         // the PWM procentage, initialed to 0 for the left motor
int pwm_value_right;                // initialzing the PWM value aka. turning the procentage into a 8-bit value (0-255)
int pwm_value_left;                 // initialzing the PWM value aka. turning the procentage into a 8-bit value (0-255)
float heading_angle;                // a filler value as of sep. 21 it has no uses other then it being set in the subcriber ""
int mode_mode;                      // initialzing the mode of the system
float delta_time_right;             // initialzing the the differents in time that is used to calculate the angular velocity
float old_time_right = millis();    // setting the initial time of the system for the right motor
float delta_time_left;              // initialzing the the differents in time that is used to calculate the angular velocity
float old_time_left = millis();     // setting the initial time of the system for the left motor
long speed_array[10];               // initialzing the array that holds the newest angular velocity values 
float current_omega_rigth;          // initialzing the current angular velocity for the right motor
float current_omega_left;           // initialzing the current angular velocity for the left motor
int float_to_long_factor = 1000;


ros::NodeHandle nh;                 // here the node handler is set with the name nh
std_msgs::Int16 mode_confurm;       // the variable is initilazed as a Int16, this is a ros type that is the type that you can sent over the ros topics
ros::Publisher mode_pub("mode_repeat", &mode_confurm);  //here the publisher is initilazed with the publisher "name" the topic "name" and a pointer to the variable that is sent

void array_push(float data){
  for (int x = sizeof(speed_array); x > 0; x = x - 1){
    speed_array[x] = speed_array[x-1];
  }
  speed_array[0] = data*float_to_long_factor;
  }

float array_sum(){
  float result = 0;
  for (int x = 0; x < sizeof(speed_array); x++){
    result = result + (speed_array[x]/float_to_long_factor);
  }
  return result;
  }



void message_pwm(geometry_msgs::Vector3& pwm_comand){
  pwm_procent_right = pwm_comand.x;
  pwm_value_right = map(pwm_procent_right, 0, 100, 0, 255);
  pwm_procent_left = pwm_comand.y;
  pwm_value_left= map(pwm_procent_left, 0, 100, 0, 255);
  heading_angle = pwm_comand.z;
  analogWrite(right_motor_pwm, pwm_value_right);
  analogWrite(left_motor_pwm, pwm_value_left);
  //nh.loginfo(pwm_procent);
}
void message_mode(std_msgs::Int16& mode_comand){
  mode_mode = mode_comand.data;
  direction_seclection(mode_mode);
  //String string_mode = String(mode_mode);
  nh.loginfo(mode_mode);
}

ros::Subscriber<geometry_msgs::Vector3> sub("pwm_sig", &message_pwm);
ros::Subscriber<std_msgs::Int16> sub1("mode_sig", &message_mode);

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
  nh.subscribe(sub1);
  nh.advertise(mode_pub);
}

void loop() {
//  Serial.print("my mode is: ");
//  Serial.println(mode_mode);
//  Serial.print("Counter is: ");
//  Serial.print(encoder_counter_right);
//  Serial.print(" and that equals: ");
//  Serial.print(encoder_to_unit(encoder_counter_right,1));
//  Serial.println(" deg");
  mode_confurm.data = mode_mode;
  mode_pub.publish(&mode_confurm);
  nh.spinOnce();
}


double encoder_to_unit(int encoder_count,int unit_output){//if unit_output is 1 the unit is deg, if its 2 its rad
  double output_number;
  float temp_number;
  if (unit_output == 1){
    output_number = encoder_count * count_to_deg;
    //output_number = temp_number % float(360);
    //output_number = map(encoder_count,-counts_per_revolution, counts_per_revolution, -360, 360);
  }
  if (unit_output == 2){
    output_number = encoder_count * count_to_rad;
    //output_number = temp_number % (2*pi);
    //output_number = map(encoder_count,-counts_per_revolution, counts_per_revolution, -2*pi, 2*pi);
  }
  return output_number;
}


void encoder_count_chage_right(){
  delta_time_right = millis() - old_time_right;
  old_time_right = millis();
  if (encoder_counter_right < counts_per_revolution && encoder_counter_right > -counts_per_revolution){
    if (direction_indicator_right == 1){
      encoder_counter_right++;
      current_omega_rigth = count_to_rad/delta_time_right;
      array_push(current_omega_rigth);

    }
    if (direction_indicator_right == 0){
    //Serial.println("Third logic");
      encoder_counter_right = encoder_counter_right - 1;
      current_omega_rigth = -count_to_rad/delta_time_right;
      array_push(current_omega_rigth);
    }
  }
  if ((encoder_counter_right == counts_per_revolution && direction_indicator_right == 0 )||(encoder_counter_right == -counts_per_revolution && direction_indicator_right == 1)){
    encoder_counter_right = 0;
    if(direction_indicator_right == 1){
      current_omega_rigth = count_to_rad/delta_time_right;
      array_push(current_omega_rigth);
    }
    if(direction_indicator_right == 0){
      current_omega_rigth = -count_to_rad/delta_time_right;
      array_push(current_omega_rigth);
    }
  }
  }

void encoder_count_chage_left(){
  delta_time_left = millis() - old_time_left;
  old_time_left = millis();
  if (encoder_counter_left < counts_per_revolution && encoder_counter_left > -counts_per_revolution){
    if (direction_indicator_left == 1){
      encoder_counter_left++;
      current_omega_left = count_to_rad/delta_time_left;
      array_push(current_omega_left);
      //digitalWrite(led_indicator, HIGH);
      //status_of_led = !status_of_led;
    }
    if (direction_indicator_left == 1){
      encoder_counter_left = encoder_counter_left - 1;
      current_omega_left = -count_to_rad/delta_time_left;
      array_push(current_omega_left);
    }
  }
  if ((encoder_counter_left == counts_per_revolution && direction_indicator_left == 0 )||(encoder_counter_left == -counts_per_revolution && direction_indicator_left == 1)){
    encoder_counter_left = 0;
    if (direction_indicator_left == 1){
      current_omega_left = count_to_rad/delta_time_left;
      array_push(current_omega_left);
    }
    if (direction_indicator_left == 0){
      current_omega_left = -count_to_rad/delta_time_left;
      array_push(current_omega_left);
    }
  }
  }

void direction_seclection(int mode){
  if (mode == 0){// 0 means that the robot is going forward
    digitalWrite(right_motor_ina, HIGH);
    digitalWrite(right_motor_inb, LOW);
    digitalWrite(left_motor_ina, HIGH);
    digitalWrite(left_motor_inb, LOW);
  }
  if (mode == 1){// 1 means that the robot is reversing
    digitalWrite(right_motor_ina, LOW);
    digitalWrite(right_motor_inb, HIGH);
    digitalWrite(left_motor_ina, LOW);
    digitalWrite(left_motor_inb, HIGH);
  }
  if (mode == 2){// 2 means that the robot is turning left
    digitalWrite(right_motor_ina, HIGH);
    digitalWrite(right_motor_inb, LOW);
    digitalWrite(left_motor_ina, LOW);
    digitalWrite(left_motor_inb, HIGH);
  }
  if (mode == 3){// 3 means that the robot is turning right
    digitalWrite(right_motor_ina, LOW);
    digitalWrite(right_motor_inb, HIGH);
    digitalWrite(left_motor_ina, HIGH);
    digitalWrite(left_motor_inb, LOW);
  }
}  
