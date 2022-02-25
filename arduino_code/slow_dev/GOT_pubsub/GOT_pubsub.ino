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
enum Byte_Type {Escape = 0x10, StartByte = 0x02, StopByte = 0x03};
enum State_Type {EscapeRec = 2, start_byte_recieved = 1, Idle = 0};
enum State_Type State = Idle;
byte inBytes[25];
int ByteCnt;
int SatCnt;
int test_cnt = 0;
bool cc;


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
char* Lev_ptr;

///ADD usable parts'
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
void Extract_Data()
{
  int Length = inBytes[0];
  //  int No_Of_Data = floor((Length - NUM_BEACONS) / NUM_BEACONS);
  int i;

  //  for (i = 0; i < No_Of_Data; i++)
  //  {
  //    data_ptr = (data_type*) &inBytes[8 + i * NUM_BEACONS];
  //  }
  data_ptr = (data_type*) &inBytes[8];
  Lev_ptr = (char*) &inBytes[5];
  ByteCnt = 0;
}
void Store_distance()
{
  //Serial.println("Store");
  long int ID = (int)data_ptr->TxID_High;
  ID = 256 * ID + (int)data_ptr->TxID_Middle;
  ID = 256 * ID + (int)data_ptr->TxID_Low;


  //  Serial.print(SatCnt);
  //  Serial.print(",");

  //compute measured distance
  double meas_dist = (double)data_ptr->TxID_time_High;
  meas_dist = 256 * meas_dist + (double)data_ptr->TxID_time_Low;
  meas_dist *= 0.342; //Speed of light in mm pr uS

  //meas_dist = 10000 + random(1000) - 500; //test without receiver UNO

  int lev = (int)(*Lev_ptr);

  //run through beacons
  for (int i = 0; i < NUM_BEACONS; i++)
  {
    if (Stored_List[i].Alive > 0)
      Stored_List[i].Alive = Stored_List[i].Alive - 1; //discount old measurements
    //if (Stored_List[i].ID == ID )
    if (Stored_List[i].ID == ID && meas_dist < 20000 && lev > 2)  //test UNO <-
    {
      Stored_List[i].Alive = 10; //valid for the next 6 measurements
      Stored_List[i].Dist = meas_dist;


    }
  }

#ifdef PR
  Serial.print(ID);
  Serial.print(",");
  Serial.print(":");
  Serial.print(x_est);
  Serial.print(',');
  Serial.print(y_est);
  Serial.print(',');
  Serial.print(z_est);
  Serial.print(',');
  Serial.print(meas_dist);
  Serial.print(',');
  Serial.print(SatCnt);
  Serial.println(";");
#endif

}

void GoTFSM()
{
  //char inByte = 0;
  char inByte = Serial3.read();  //test UNO
  Serial.print(inByte);
  switch (State) {
    case Idle:
      ByteCnt = 0;
      if (inByte == StartByte)
        State = start_byte_recieved;
      break;
    case start_byte_recieved:
      switch (inByte) {
        case StartByte:
          ByteCnt = 0;
          State = Idle;
          break;
        case StopByte:
          //compute checksums and extract data
          cc = compute_checksum();
          if (cc == true)
          {
            Extract_Data();
            Store_distance();
          }
          State = Idle;
          if (Serial3.available()) {  //if stop signal goto statistics state
            inByte = Serial.read();
      //      Stat_State = 2; //start waiting for matlab to fetch results
          }
          break;
        case Escape:
          State = EscapeRec;
          break;
        default:
          inBytes[ByteCnt++] = inByte;
      }
      break;
    case EscapeRec:
      State = start_byte_recieved;
      inBytes[ByteCnt++] = (char)(inByte - 0x20);
  }
}


///




void setup()
{
  Serial.begin(19200);
  Serial3.begin(115200);
  nh.initNode();
  nh.advertise(robot_position_estimate);
}

void loop()
{
  char inByte = Serial3.read();
  /////////////

  //////////////




  robot_position_estimate.publish( &estimate_xyz );

  if (Serial3.available()) {
    //char inByte = Serial3.read();
    //estimate_xyz.x = inByte;
    if (inByte == StartByte) {
      Serial.print("Start");
      GoTFSM();
      struct store data;
      estimate_xyz.x=data.ID;
      estimate_xyz.y=data.Dist;
      estimate_xyz.z=0;



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
