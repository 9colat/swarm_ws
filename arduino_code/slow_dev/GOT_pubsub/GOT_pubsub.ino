#include <ros.h>
#include <geometry_msgs/Vector3.h>
#define NUM_BEACONS 13



//Map of beacon posittions
long int ID_POS_List[NUM_BEACONS][4] = {
  //{ID, , ,}
  //{44529, 7825, 9999, 4286},
  {42867, 11700, 5999, 5577},
  {42928, 16244, 10150, 5577},  //only in old list
  {42929, 7824, 5726, 4286},
  //{44529, 1999, 10677, 3531}, //bad quality
  {44530, 2000, 4499, 3530},
  {44531, 21369, 6534, 5578},
  {44532, 26163, 9939, 5577},
  {44533, 26163, 3699, 5577},
  {44534, 31000, 6519, 5578},
  {44535, 35766, 10012, 5578},
  {44536, 35766, 3522, 5578},
  {44537, 40205, 11684, 3767},
  {44538, 40204, 4363, 3767},
  {44540, 16560, 3549, 5577}
};

//Ros initialization
ros::NodeHandle  nh;
geometry_msgs::Vector3 estimate_xyz;
ros::Publisher robot_position_estimate("robot_position_estimate", &estimate_xyz);

//initialize parameters; These is used to mark start/end of the protocol
float EscapeByte = 0x10;
float StartByte = 0x02;
float StopByte = 0x03;

typedef struct store  {
  long int  ID;
  float  Dist;
  int  Alive;
} store_type;
store_type Stored_List[NUM_BEACONS];

typedef struct data  {
  byte  rssi;
  byte  TxID_Low;
  byte  TxID_Middle;
  byte  TxID_High;
  byte  TxID_time_Low;
  byte  TxID_time_High;
} data_type;

data_type* data_ptr;






void setup()
{
  Serial.begin(19200);
  Serial3.begin(115200);
  nh.initNode();
  nh.advertise(robot_position_estimate);
}

void loop()
{

  robot_position_estimate.publish( &estimate_xyz );

  if (Serial3.available()) {
    char inByte = Serial3.read();
    estimate_xyz.x = inByte;
    if (inByte == StartByte) {
      Serial.print("Start");
    }
    if (inByte == StopByte) {
      Serial.println("|Stop");
      //      Serial.print(inByte, DEC);
    }
    else {
      Serial.print("|");
      Serial.print(inByte, DEC);
     // Serial.print("|");

    }
  }

  nh.spinOnce();
  //delay(500);
}
