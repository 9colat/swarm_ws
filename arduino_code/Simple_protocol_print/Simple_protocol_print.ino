#include <ros.h>
#include <geometry_msgs/Vector3.h>

enum Byte_Type {Escape = 0x10, StartByte = 0x2, StopByte = 0x3};

void setup() {
  Serial.begin(19200);
  Serial3.begin(115200);
}

void loop() {
  bool esc_char = false;

  while (Serial3.available()) {
    char inByte = Serial3.read();
    if (inByte == StartByte || inByte == StopByte || inByte == Escape) {
      if (inByte == Escape) {
        bool esc_char = true;
      }
      if (inByte == StartByte && esc_char == false) {
        Serial.print("start|");
      }
      if (inByte == StopByte && esc_char == false) {
        //Serial.println("StopByte");
        Serial.print(inByte,DEC);
        Serial.println("|stop");


      }
    }
    if (inByte != StartByte && inByte != StopByte || esc_char == true && inByte == StartByte || inByte == StopByte && esc_char == true) {
      //Serial.print(inByte, DEC);
      Serial.print("|");
      if (esc_char == true && inByte != StartByte || esc_char == true && inByte != StopByte ) {
        esc_char = false;
      }
    }
  }

}
