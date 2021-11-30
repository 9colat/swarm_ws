#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;
SoftwareSerial radio_module(4, 7); // TX 4, RX 7




const int message_size = 63; // how many chars the message can contain
char str[message_size]; // array of chars initialised, that will contain the sent message
String message_to_send;
std_msgs::String data_to_be_received;


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

    Serial.println("Radio module available");

    for (int i = 0; i < message_size; i++) {

      char radio_data = char(radio_module.read()); // reading data in chars, saved in radio_data char
      str[i] = radio_data; // radio_data char passed to the str char array

      //FILTER STUFF THAT IS DUMB AND YOU DONT WANT TO PASS IT INTO THE ARRAY
    }
    
    if (str != NULL) {
      data_to_be_received.data = str;
      publisher.publish(&data_to_be_received); //now the read data is being published through ROS
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


  receiving();
  nh.spinOnce();

}
