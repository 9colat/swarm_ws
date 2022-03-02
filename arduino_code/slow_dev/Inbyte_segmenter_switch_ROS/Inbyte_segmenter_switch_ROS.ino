#include <ros.h>
#include <geometry_msgs/Vector3.h>

enum Byte_Type {Escape = 0x10, StartByte = 0x2, StopByte = 0x3};
enum State_Type {EscapeRec = 2, StartByteRec = 1, idle = 0};
enum State_Type State = idle;
int ByteCnt;
byte inBytes[25];
bool cc;


void setup() {
  Serial.begin(19200);
  Serial3.begin(115200);
}
bool compute_checksum()
{
  int i;
  long sum1 = 0;
  long sum2 = 0;
  int check1 = 0;
  int check2 = 0;
  for (i = 1; i < ByteCnt - 2; i++)
  {
    sum1 += (int)inBytes[i] & 0xFF;
    sum1 = sum1;
    sum2 += sum1;
  }
  sum1 = sum1 % 255;
  sum2 = sum2 % 255;
  check1 = 255 - ((sum1 + sum2) % 255);
  check2 = 255 - ((sum1 + check1) % 255);

  if (sum1 == (int)inBytes[ByteCnt - 2] && sum2 == (int)inBytes[ByteCnt - 1])
  {
    return (true);
  }
  else
  {
    return (false);
  }
}

void data_saver_switch() {
  char inByte = Serial3.read();
  switch (State) {
    case idle:
      Serial.print("state: idle");
      if (inByte == StartByte)
        State = StartByteRec;
      //Serial.print("state set: StartByteRec");

      break;
    case StartByteRec:
      switch (inByte) {
        case StartByte:
          Serial.print("state: Startbyte");
          ByteCnt = 0;
          State = idle;
          break;
        case StopByte:
          //We finished now publish and/or data extract
          Serial.println("Finished");
          cc = compute_checksum();
          if (cc == true)
          {
            Serial.println(" CC is true ");
          }
          if (cc == false)
          {
            Serial.println(" CC is false ");
          }
          break;

        case Escape:
          State = EscapeRec;
          //Serial.print("state: Escape");

          break;
        default:
          inBytes[ByteCnt++] = inByte;
          //Serial.print("state: default");
          Serial.print((int)inByte);

      }
      break;
    case EscapeRec:
      Serial.print("state: EscapeRec");
      State = StartByteRec;
      inBytes[ByteCnt++] = (char)(inByte - 0x20);

  }
}



void loop() {
  bool esc_char = false;
  while (Serial3.available()) {
    data_saver_switch();
    /*  char inByte = Serial3.read();
      if (inByte == StartByte || inByte == StopByte || inByte == Escape)
      {
        if (inByte == Escape) {
          bool esc_char = true;
        }
        if (inByte == StartByte && esc_char == false) {
          Serial.print("start|");
        }
        if (inByte == StopByte && esc_char == false) {
          Serial.println("|stop");
        }
      }
      else if ((inByte != StartByte && inByte != StopByte) || (esc_char == true && inByte == StartByte || inByte == StopByte))
      {
        Serial.print(inByte, DEC);
        Serial.print("|");
        //  if ((esc_char == true && inByte != StartByte) || (esc_char == true && inByte != StopByte) ) {
        esc_char = false;
      } else
      {
        Serial.print("ERROR");
      } */
  }


}
