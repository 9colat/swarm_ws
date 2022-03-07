#include <ros.h>
#include <geometry_msgs/Vector3.h>

enum Byte_Type {Escape = 0x10, StartByte = 0x2, StopByte = 0x3};
int cnt = 0;
int Array[] = {};
int inBytes[44];
void setup() {
  Serial.begin(19200);
  Serial3.begin(115200);
}

void loop() {

  bool esc_char = false;
  while (Serial3.available()) {
    char inByte = Serial3.read();
    if (inByte == StartByte || inByte == StopByte || inByte == Escape)
    {
      if (inByte == Escape) {
        esc_char = true;
      }
      if (inByte == StartByte && esc_char == false) {
        Serial.print("start|");
      }
      if (inByte == StopByte && esc_char == false) {
        /* Array[cnt];
          for (int e; e < cnt; e++) {
           Array[e] = inByte;
          }
          Serial.println(Array);

        */
        for (int e; e < cnt; e++) {
          Serial.print(inBytes[e]);
        }
        cnt = 0;
      }
    } else if ((inByte != StartByte && inByte != StopByte) || (esc_char == true && inByte == StartByte || inByte == StopByte))
    {
      //Serial.print(inByte, DEC);
      inBytes[cnt] = inByte;
      //Serial.print("|");
      //Serial.print(cnt);
      cnt++;
      esc_char = false;
    } else
    {
      Serial.print("ERROR");
    }
  }


}
