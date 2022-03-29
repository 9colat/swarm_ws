//-----Libraries-----//
#include <ros.h>
#include <std_msgs/Int32.h>
#include <custom_msgs/USPS_msgs.h>

enum Byte_Type {Escape = 0x10, StartByte = 0x2, StopByte = 0x3};
enum State_Type {EscapeRec = 2, StartByteRec = 1, idle = 0};
enum State_Type State = idle;
int ByteCnt;
unsigned char inBytes[100];
bool cc;

ros::NodeHandle nh;

custom_msgs::USPS_msgs beacon_data = custom_msgs::USPS_msgs();
ros::Publisher beacon_data_pub("beacon_data", &beacon_data);

int hex_to_useable_data_test(int x, int y, int z){
  int data = x + y * 256 + z * pow(256, 2);
  return data;
}

void setup() {
  //Serial.begin(19200);
  Serial3.begin(115200);
  nh.initNode();
  nh.advertise(beacon_data_pub);
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

void seperator(unsigned char array_input[], int lgt) {
  for (int i = 0; i < array_input[7]; i++) {
    if (array_input[i * 6 + 12] != 255 && array_input[i * 6 + 13] != 255) {

      //Serial.println("seperator: ");
      //Serial.println(array_input[7]);
      beacon_data.ID = hex_to_useable_data_test(array_input[i*6+9],array_input[i*6+10],array_input[i*6+11]);
      beacon_data.RSSI = hex_to_useable_data_test(array_input[i*6+8],0,0);
      beacon_data.distance = 0.343 * hex_to_useable_data_test(array_input[i*6+12],array_input[i*6+13],0);
      beacon_data_pub.publish(&beacon_data);

    }
  }
}

void data_saver_switch() {
  int tmp=Serial3.read();
  unsigned char inByte;
  bool gotten_esc = false;
  if(tmp<0){
    return;
  }
  inByte=(unsigned char) tmp;


  switch (State) {
    case idle:
      // Serial.print("idle");
      if (inByte == StartByte)
        State = StartByteRec;
      //Serial.print("state set: StartByteRec");
      break;
    case StartByteRec:
      switch (inByte) {
        case StartByte:
          // Serial.print("Startbyte");
          ByteCnt = 0;
          State = StartByteRec;
          break;
        case StopByte:
          //We finished now publish and/or data extract
          Serial.println("Finished");
          cc = compute_checksum();
          if (cc == true)
          {
            unsigned char data_array[ByteCnt];
            // Serial.println(ByteCnt);
            //  Serial.println(sizeof(data_array)/sizeof(int));
            for (int j = 0; j < ByteCnt; j++) {
              data_array[j] = inBytes[j];
              //Serial.println(j);
            }
            seperator(data_array, ByteCnt);

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
          if (gotten_esc){
            inBytes[ByteCnt++] = (unsigned char)(inByte - 0x20);
            gotten_esc = false;
          }
          if (!gotten_esc){
          inBytes[ByteCnt++] = inByte;
          //Serial.print("state: default");
        }
          //Serial.print(inBytes[ByteCnt-1], DEC);
          //Serial.print("|");

      }
      break;
    case EscapeRec:
      // Serial.print("EscapeRec");
      State = StartByteRec;
      gotten_esc = true;
      //inBytes[ByteCnt++] = (unsigned char)(inByte - 0x20);

  }
}



void loop() {
  while (Serial3.available()) {
    data_saver_switch();
      nh.spinOnce();
  }


}
