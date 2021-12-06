#include <ros.h>
#include <std_msgs/Int16.h>

// the setup function runs once when you press reset or power the board
const byte RGB_led_green = 2;
const byte RGB_led_blue = 3;
const byte RGB_led_red = 4;
int mode = 0;

ros::NodeHandle nh;

void message_mode(std_msgs::Int16& mode_comand) {
  mode = mode_comand.data;
}
ros::Subscriber<std_msgs::Int16> sub("mode_sig", &message_mode);

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
  pinMode(RGB_led_green, OUTPUT);
  pinMode(RGB_led_blue, OUTPUT);
  pinMode(RGB_led_red, OUTPUT);
  RGB_led_set("white");
  nh.subscribe(sub);

}

// the loop function runs over and over again forever
void loop() {
    for(int i = 0; i < 7; i++){
      if(i == 0){
        RGB_led_set("red");
      }
      if(i == 1){
        RGB_led_set("green");
      }
      if(i == 2){
        RGB_led_set("blue");
      }
      if(i == 3){
        RGB_led_set("orange");
      }
      if(i == 4){
        RGB_led_set("cyan");
      }
      if(i == 5){
        RGB_led_set("purple");
      }
      if(i == 6){
        RGB_led_set("white");
      }
      delay(1000);
    }
  nh.spinOnce();
}
