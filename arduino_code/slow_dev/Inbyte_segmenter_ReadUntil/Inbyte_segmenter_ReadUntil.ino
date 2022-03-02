#include <ros.h>
#include <geometry_msgs/Vector3.h>

enum Byte_Type {Escape = 0x10, StartByte = 0x2, StopByte = 0x3};
bool start_byte_recieved = false;
const int BUFFER_SIZE = 100;
char buf[BUFFER_SIZE];

void setup()
{
  Serial.begin(19200);
  Serial3.begin(115200);
}
/*
void inByte_segmenter(int inByte_array)
{




}
    /*packet_length
      Serial.print("|length: ");
      Serial.print(packet_length,DEC);
      Serial.println("|");
    */
  //}


void loop()
{
  bool esc_char = false;

  while (Serial3.available()) {
    char inByte = Serial3.read();
    if (inByte!=0x3){
        int inByte_array = Serial3.readBytesUntil(StartByte, buf, BUFFER_SIZE);
        for (int i = 0; i < inByte_array; i++) {

        Serial.print("|");
        Serial.print((int)buf[i]);
      }
      Serial.println(" ");
    }

        //inByte_segmenter(inByte_array);
    if (inByte == StopByte && esc_char == false) {
      //Serial.println("|stop");
    }
    else if ((inByte != StartByte && inByte != StopByte) || (esc_char == true && inByte == StartByte || inByte == StopByte))
    {
      //Serial.print(inByte, DEC);
      //Serial.print("|");
      //  if ((esc_char == true && inByte != StartByte) || (esc_char == true && inByte != StopByte) ) {
      esc_char = false;
    }
    /*else
    {
      Serial.print("ERROR");
    }*/
  }
}
