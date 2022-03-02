#include <ros.h>
#include <geometry_msgs/Vector3.h>

enum Byte_Type {Escape = 0x10, StartByte = 0x2, StopByte = 0x3};
bool start_byte_recieved = false;
bool stop_byte_recieved = false;
bool esc_char_recieved = false;
const int BUFFER_SIZE = 100;
char buf[BUFFER_SIZE];
char inByteArray[BUFFER_SIZE];


void setup()
{
  Serial.begin(19200);
  Serial3.begin(115200);
}

void inByte_segmenter()
{

}


void loop()
{
  //bool esc_char = false;

  while (Serial3.available()) {
    //char inByte = Serial3.read();
    //inByte_segmenter();
    char inByte = Serial3.read();
    if (inByte == StopByte || inByte == Escape || StartByte) {
      if (inByte == Escape) {
        esc_char_recieved = true;
        stop_byte_recieved = false;
        start_byte_recieved = false;
      }
      if (inByte == StopByte && esc_char_recieved == false) {
        stop_byte_recieved = true;
        start_byte_recieved = false;
      }
      if (inByte == StartByte && esc_char_recieved == false) {
        start_byte_recieved = true;
        stop_byte_recieved = false;
      }
      if ((inByte == StartByte || inByte == StopByte) && esc_char_recieved == true) {
        start_byte_recieved = false;
        stop_byte_recieved = false;
      } else {
        for (int i=0; (stop_byte_recieved == false && start_byte_recieved == true); i++) {
          inByteArray[i] = inByte;
          Serial.print("|");
          Serial.print(i, DEC);
          Serial.println(" ");

        }
      }
    }

    /*
        if (inByte == StartByte || inByte == StopByte || inByte == Escape)

        {
          if (inByte == Escape) {
            bool esc_char = true;
          }
          if (inByte == StartByte && esc_char == false) {
            int inByte_array = Serial3.readBytesUntil('0x2', buf, BUFFER_SIZE);

            inByte_segmenter(inByte_array);
          }
          Serial.println(" ");
        }*/
    //  if (inByte == StopByte && esc_char == false) {
    //  }
    //  else if ((inByte != StartByte && inByte != StopByte) || (esc_char == true && inByte == StartByte || inByte == StopByte))
    //  {
    //    esc_char = false;
    //  }
  }
}
