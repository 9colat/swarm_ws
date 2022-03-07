//-----Libraries-----//
#include <ros.h>
#include <std_msgs/Int32.h>
#include <custom_msgs/USPS_msgs.h>

int input[46] = {43, 1, 1, 0, 0, 10, 236,  6, 202, 252, 173, 0, 31, 82, 207, 250, 173, 0, 255, 255, 207, 249, 173, 0, 255, 255, 207, 248, 173, 0, 255, 255, 207, 247, 173, 0, 255, 255, 207, 246, 173, 0, 255, 255, 54, 43}; //remove later
int period = 1000;
unsigned long time_now = 0;
enum Byte_Type {Escape = 0x10, StartByte = 0x2, StopByte = 0x3};

ros::NodeHandle nh;

std_msgs::Int32 right_tick;
ros::Publisher right_tick_pub("right_tick", &right_tick);
custom_msgs::USPS_msgs beacon_data = custom_msgs::USPS_msgs();
ros::Publisher beacon_data_pub("beacon_data", &beacon_data);

int hex_to_useable_data_test(int x, int y, int z){
  int data = x + y * 256 + z * pow(256, 2);
  return data;
}

void seperator(int array_input[]){
  for(int i = 0; i < array_input[7]; i++){
    if(array_input[i*6+12] != 255 && array_input[i*6+13] != 255){
      beacon_data.ID = hex_to_useable_data_test(array_input[i*6+9],array_input[i*6+10],array_input[i*6+11]);
      beacon_data.RSSI = hex_to_useable_data_test(array_input[i*6+8],0,0);
      beacon_data.distance = 0.343 * hex_to_useable_data_test(array_input[i*6+12],array_input[i*6+13],0);
      beacon_data_pub.publish(&beacon_data);
      nh.spinOnce();
    }
  }
}



void data_colector() {
  bool esc_char = false;
  bool start_up = false;
  int inByte_array[100] = {};
  int i = 0;
  while (Serial3.available()) {
  char inByte = Serial3.read();
    if (inByte == StartByte || inByte == StopByte || inByte == Escape){
      if (inByte == Escape) {
        bool esc_char = true;
      }
      if (inByte == StartByte && esc_char == false) {
        //Serial.print("start|");
        start_up = true;
      }
      if (inByte == StopByte && esc_char == false) {
        //Serial.println("|stop");
        start_up = false;
        int output[i];
        for (int j = 0;j<i;j++){
          output[j] = inByte_array[j];
        }
        seperator(output);
      }
    }
    else if(((inByte != StartByte && inByte != StopByte) || (esc_char && (inByte == StartByte || inByte==StopByte))) && !start_up)
    {
      inByte_array[i] = int(inByte);
      i = i + 1;
    //  if ((esc_char == true && inByte != StartByte) || (esc_char == true && inByte != StopByte) ) {
      esc_char = false;
    }
  }
}





void setup() {
  Serial3.begin(115200);
  nh.initNode();
  nh.advertise(right_tick_pub);
  nh.advertise(beacon_data_pub);
}

void loop() {
data_colector();
  /*if(millis() > time_now + period){
    time_now = millis();

  }*/
}
