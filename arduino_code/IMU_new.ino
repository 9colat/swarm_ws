#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>


ros::NodeHandle nh;                 // here the node handler is set with the name nh
std_msgs::Int16 mode_confurm;       // the variable is initilazed as a Int16, this is a ros type that is the type that you can sent over the ros topics
ros::Publisher mode_pub("mode_repeat", &mode_confurm);  //here the publisher is initilazed with the publisher "name" the topic "name" and a pointer to the variable that is sent

void array_push(long the_input_array[], float data){
  for (int x = sizeof(the_input_array); x > 0; x = x - 1){
    the_input_array[x] = the_input_array[x-1];
  }
  the_input_array[0] = data*float_to_long_factor;
  }

float array_sum(long the_input_array[]){
  float result = 0;
  for (int x = 0; x < sizeof(the_input_array); x++){
    result = result + (the_input_array[x]/float_to_long_factor);
  }
  return result;
  }


void setup() {
  Serial.begin(9600);
  Serial.println("Start up");
  nh.initNode();
  nh.advertise(mode_pub);
}

void loop() {
  mode_confurm.data = mode_mode;
  mode_pub.publish(&mode_confurm);
  nh.spinOnce();
}
