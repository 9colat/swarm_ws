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

void inByte_segmenter(int inByte_array)
{

  for (int i = 0; i < inByte_array; i++) {
    if (i==Escape){
      Serial.println(" ");
      Serial.print("escape");
      Serial.println(" ");
    }else{
    Serial.print("|");
    Serial.print(buf[i], DEC);
}
    /*packet_length
      Serial.print("|length: ");
      Serial.print(packet_length,DEC);
      Serial.println("|");
    */
  }
}

void loop()
{
  bool esc_char = false;

  while (Serial3.available()) {
    char inByte = Serial3.read();
    if (inByte == StartByte || inByte == StopByte || inByte == Escape)
    {
      if (inByte == Escape) {
        bool esc_char = true;
      }
      if (inByte == StartByte && esc_char == false) {
        //Serial.print("start|");
        //  start_byte_recieved=true;
        //  inByte_array[]==Serial3readBytesUntil(StopByte);
        int inByte_array = Serial3.readBytesUntil('0x2', buf, BUFFER_SIZE);

        //Serial.print("length: ");
        //Serial.println(buf[0],HEX);//propebly 0 or 1
        //  Serial.println(" ");
        inByte_segmenter(inByte_array);
      }
      Serial.println(" ");
    }
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
