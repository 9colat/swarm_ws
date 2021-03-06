#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;
//SoftwareSerial radio_module(4, 7);
SoftwareSerial radio_module(10, 12); // TX -> 10, RX -> 14

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
char test_array[] = {a, b, c, d, e};
byte green = 5;
byte yellow = 6;



unsigned int message_size = 281; // how many chars the message can contain
std_msgs::String data_to_be_received;
char inverted_question_mark = 14858414; //⸮
char end_character = 38; //& = end bit


// first we need to subscribe to data, so we can send it over the radio later (WORKS AS AN INTERRUPTER)
void data_to_be_sent (const std_msgs::String& data_msg) {

  digitalWrite(green, LOW);
  digitalWrite(yellow, HIGH);

  char message_to_radio[message_size];
  String message_from_ros;
  message_from_ros = data_msg.data;
  int current_size = message_from_ros.length() + 1;
  message_from_ros.toCharArray(message_to_radio, current_size);

  
  //nh.loginfo(&message_from_ros);

  radio_module.write(message_to_radio, current_size); //now the subscribed data is being sent over the radio

}

ros::Subscriber<std_msgs::String> subscriber("data_to_be_sent", data_to_be_sent ); //subscriber setup
ros::Publisher publisher("data_to_be_received", &data_to_be_received); //publisher setup




// first we need to receive the data over the radio, so we can publish it later
void receiving() {

  char str[message_size]; // array of chars initialised, that will contain the sent message

  digitalWrite(green, HIGH);
  digitalWrite(yellow, LOW);

  //Serial.println(radio_module.available());


  if (radio_module.available() > 0) { // is there something on the radio? (enable the radio)

    //Serial.println("Radio module available");
    int j = 0;
    int k = 0;


    for (int i = 0; i < message_size; i++) {
      str[i] = 0;
      char radio_data = char(radio_module.read()); // reading data in chars, saved in radio_data char

      if (radio_data < 127 && radio_data > 32) { // inverse question mark filter - we can read symbols from 33rd to 126th in the ASCII table
        str[j] = radio_data; // radio_data char passed to the str char array
        j++;
        //delay();

        if (radio_data == end_character || k == 1) {
          str[j] = 0;
          k = 1;
          exit(0); //exit the loop when it hits the end character
        }
      }
      Serial.println(i);
      //delay(1);

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
  pinMode(green, OUTPUT);
  pinMode(yellow, OUTPUT);
  digitalWrite(green, HIGH);
  digitalWrite(yellow, HIGH);
  nh.initNode();
  nh.advertise(publisher);
  nh.subscribe(subscriber);

}



void loop() {

  //Serial.print(test_array);
  //radio_module.write("123456789011234567891112345678111123456711111234561111112345111111123411111111231111111112111111111112345678901123456789111234567811112345671111123456111111234511111112341111111123111111111211111111111234567890112345678911123456781111234567111112345611111123451111111234111111112311111111121111111111");
  //radio_module.write("punch yourself");
  //delay(1000);
  receiving();
  nh.spinOnce();
  //Serial.println(str);

}
