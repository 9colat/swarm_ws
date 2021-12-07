#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;
SoftwareSerial radio_module(4, 7); // TX 4, RX 7

char a = 33; //! = start bit
char b = 122; //z
char c = 101; //e
char d = 116; //t
char e = 38; //& = end bit
char f = 14858414; //⸮
char g = 0;
char h = 0;
char i = 0;
char j = 0;
char test_array[] = {f};



const int message_size = 178; // how many chars the message can contain
char str[message_size]; // array of chars initialised, that will contain the sent message
String message_to_send;
std_msgs::String data_to_be_received;
char inverted_question_mark = 14858414; //⸮
char end_character = 38; //& = end bit


// first we need to subscribe to data, so we can send it over the radio later (WORKS AS AN INTERRUPTER)
void data_to_be_sent (const std_msgs::String& data_msg) {
  message_to_send = data_msg.data;
  Serial.println(message_to_send);
  radio_module.write(&message_to_send); //now the subscribed data is being sent over the radio
}

ros::Subscriber<std_msgs::String> subscriber("data_to_be_sent", data_to_be_sent ); //subscriber setup
ros::Publisher publisher("data_to_be_received", &data_to_be_received); //publisher setup




// first we need to receive the data over the radio, so we can publish it later
void receiving() {

  if (radio_module.available() > 0) { // is there something on the radio? (enable the radio)

    //Serial.println("Radio module available");
    int j = 0;
    int k = 0;


    for (int i = 0; i < message_size; i++) {

      char radio_data = char(radio_module.read()); // reading data in chars, saved in radio_data char

      if (radio_data < 127 && radio_data > 32) { // inverse question mark filter - we can read symbols from 33rd to 126th in the ASCII table
        str[j] = radio_data; // radio_data char passed to the str char array
        j++;
        //delay();
        
        if (radio_data == end_character || k == 1){
          str[j] = 0;
          k = 1;
          exit(0); //exit the loop when it hits the end character
        } 
      }
      
    delay(1);
    
    }

    if (str != NULL) { //this is because we want to send the full message
      data_to_be_received.data = str;
      publisher.publish(&data_to_be_received); //now the read data is being published through ROS
      Serial.println(str);
    }
  }

}



void setup() {

  Serial.begin(57600); // terminal
  radio_module.begin(9600); // radio module
  Serial.println("Setup initialised");

  nh.initNode();
  nh.advertise(publisher);
  nh.subscribe(subscriber);

}



void loop() {

  //Serial.print(test_array);
  //String something = "hellonicetomeetyouihaveaverynicestreaktodaysoiamveryhappyandexcitedforkeepingmyhabitforsomanydaysthreehundredsixtyfivedayswoopwoop";
  //Serial.println(something);
  //radio_module.write(&something);
  //delay(1000);
  receiving();
  nh.spinOnce();
  //Serial.println(str);

}
