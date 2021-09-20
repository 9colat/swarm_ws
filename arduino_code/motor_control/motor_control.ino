#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>


//pinout setting up
const byte right_motor_pwm = 3;
const byte right_motor_inb = 2;
const byte right_motor_ina = 4;
const byte right_encoder_a = 24;
const byte right_encoder_b = 25;
const byte left_motor_pwm = 29;
const byte left_motor_inb = 28;
const byte left_motor_ina = 30;
const byte left_encoder_a = 26;
const byte left_encoder_b = 27;
//variabels  
int counts_per_revolution = 1920;
const float pi = 3.141593; // this is pi, or an aproximation, what did you expeced?
double encoder_counter_right = 0; // this is the encoder counter for the right wheel
double encoder_counter_left = 0; // this is the encoder counter for the left wheel
int status_of_led = HIGH; // this can be removed later
int direction_indicator_right = 1; //this is a direction indicator, that can be either 0 or 1. if the variable is 0 that means that the moters are going backwards, and 1 means forwards.
int direction_indicator_left = 1;
double count_to_deg = (360)/counts_per_revolution;
double count_to_rad = (2*pi)/counts_per_revolution;
float pwm_procent_right = 0;
float pwm_procent_left = 0;
int pwm_value_right;
int pwm_value_left;
float heading_angle;
int dir_mode;

ros::NodeHandle nh;
std_msgs::Int16 dir_confurm;
ros::Publisher dir_pub("dir_repeat", &dir_confurm);





void message_pwm(geometry_msgs::Vector3& pwm_comand){
  pwm_procent_right = pwm_comand.x;
  pwm_value_right = map(pwm_procent_right, 0, 100, 0, 255);
  pwm_procent_left = pwm_comand.y;
  pwm_value_left= map(pwm_procent_left, 0, 100, 0, 255);
  heading_angle = pwm_comand.z;
  //nh.loginfo(pwm_procent);
}
void message_dir(std_msgs::Int16& dir_comand){
  dir_mode = dir_comand.data;
  direction_seclection(dir_mode);
  //String string_dir = String(dir_mode);
  nh.loginfo(dir_mode);
}

ros::Subscriber<geometry_msgs::Vector3> sub("pwm_sig", &message_pwm);
ros::Subscriber<std_msgs::Int16> sub1("dir_sig", &message_dir);

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
  nh.advertise(dir_pub);
}

void loop() {
//  Serial.print("my mode is: ");
//  Serial.println(dir_mode);
//  Serial.print("Counter is: ");
//  Serial.print(encoder_counter_right);
//  Serial.print(" and that equals: ");
//  Serial.print(encoder_to_unit(encoder_counter_right,1));
//  Serial.println(" deg");
  dir_confurm.data = dir_mode;
  dir_pub.publish(&dir_confurm);
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
  if (encoder_counter_right < counts_per_revolution && encoder_counter_right > -counts_per_revolution){
  //Serial.println("First logic");
  //Serial.println(digitalRead(right_motor_ina));
    if (direction_indicator_right == 1){
    //Serial.println("Second logic");
      encoder_counter_right++;

    }
    if (direction_indicator_right == 0){
    //Serial.println("Third logic");
      encoder_counter_right = encoder_counter_right - 1;
    }
  }
  if ((encoder_counter_right == counts_per_revolution && direction_indicator_right == 0 )||(encoder_counter_right == -counts_per_revolution && direction_indicator_right == 1)){
    encoder_counter_right = 0;
  }
  else{
    //Serial.println("the encoder fucked up ples help");
    }
  }
  
void encoder_count_chage_left(){
  if (encoder_counter_left < counts_per_revolution && encoder_counter_left > -counts_per_revolution){
    if (right_motor_ina == HIGH && right_motor_inb == LOW){
      encoder_counter_left++;
      //digitalWrite(led_indicator, HIGH);
      //status_of_led = !status_of_led;
    }
    if (right_motor_ina == LOW && right_motor_inb == HIGH){
      encoder_counter_left = encoder_counter_left - 1;
    }
  }
  if ((encoder_counter_left == counts_per_revolution && right_motor_ina == HIGH && right_motor_inb == LOW )||(encoder_counter_left == -counts_per_revolution && right_motor_ina == LOW && right_motor_inb == HIGH)){
    encoder_counter_left = 0;
  }
  else{
    Serial.println("the encoder fucked up ples help");
    }
  }
  
void direction_seclection(int mode){
  if (mode == 0){// 0 means that the robot is going forward 
    digitalWrite(right_motor_ina, HIGH);
    digitalWrite(right_motor_inb, LOW);
    digitalWrite(left_motor_ina, HIGH);
    digitalWrite(left_motor_inb, LOW);
    analogWrite(right_motor_pwm, pwm_value_right);
    analogWrite(left_motor_pwm, pwm_value_left);
  }
  if (mode == 1){// 1 means that the robot is reversing
    digitalWrite(right_motor_ina, LOW);
    digitalWrite(right_motor_inb, HIGH);
    digitalWrite(left_motor_ina, LOW);
    digitalWrite(left_motor_inb, HIGH);
    analogWrite(right_motor_pwm, pwm_value_right);
    analogWrite(left_motor_pwm, pwm_value_left);
  }
  if (mode == 2){// 2 means that the robot is turning left 
    digitalWrite(right_motor_ina, HIGH);
    digitalWrite(right_motor_inb, LOW);
    digitalWrite(left_motor_ina, LOW);
    digitalWrite(left_motor_inb, HIGH);
    analogWrite(right_motor_pwm, pwm_value_right);
    analogWrite(left_motor_pwm, pwm_value_left);
  }
  if (mode == 3){// 3 means that the robot is turning right
    digitalWrite(right_motor_ina, LOW);
    digitalWrite(right_motor_inb, HIGH);
    digitalWrite(left_motor_ina, HIGH);
    digitalWrite(left_motor_inb, LOW);
    analogWrite(right_motor_pwm, pwm_value_right);
    analogWrite(left_motor_pwm, pwm_value_left);
  }
  
  
  

}
  


  
