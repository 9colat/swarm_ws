#define NUM_BEACONS 13
#define Sample_Size 40 //100
#define Sample_Time 87000
#define Transient 0 //time until statistics start
#include <ros.h>
#include <geometry_msgs/Vector3.h>
//#define PR

ros::NodeHandle nh;
geometry_msgs::Vector3 estimate_xyz;
ros::Publisher robot_position_estimate("robot_position_estimate",&estimate_xyz);


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
  //{44535, 35766, 10012, 5578}, //down
  {44536, 35766, 3522, 5578},
  {44537, 40205, 11684, 3767},
  {44538, 40204, 4363, 3767},
  {44540, 16560, 3549, 5577}

  //        {44530,   1949,    1380,  3531},
  //        {44529,  -4257,    1368,  3532},
  //        {42929,  -3589,    7135,  4286},
  //        //{42428,   644,    7234,  4279}, //low quality in overview
  //        //{42429,  -3742,   15629,  5585}, //only in new list
  //        {44540,   2706,   15960,  5587},
  //        {44531,  -286,   20767,  5585},
  //        {44532,  -3679,   25578,  5587},
  //        {44533,   2553,   25573,  5591},
  //        {44534,  -329,   30365,  5594},
  //        {42867,   277,   11572,  5585},
  //        {44537,  -5432,   39537,  3806},
  //        {44538,   1930,   39574,  3796},
  //        {44535,  -3783,   35166,  5588},
  //        {44536,   2705,   35189,  5591}
};


enum State_Type {EscapeRec = 2, StartByteRec = 1, Idle = 0};
enum State_Type State = Idle;
byte inBytes[25];
int ByteCnt;
int SatCnt;
int test_cnt = 0;
bool cc;

//statistics - initialized at loop start
unsigned long Tim;
int Stat_State = 0;
double SqSumX, SqSumY, SqSumZ;
double SumX, SumY, SumZ;
double MaxDist, MinDist;
int Stat_cnt;
double Stored_Pos[3][500]; //test UNO ->200

typedef struct store  {
  long int  ID;
  float  Dist;
  int  Alive;
} store_type;
store_type Stored_List[NUM_BEACONS];

double x_est = 16000.0, y_est = 6000.0, z_est = 300;
double x_est_new = 0.0, y_est_new = 0.0, z_est_new = 0.0;



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


enum Byte_Type {Escape = 0x10, StartByte = 0x02, StopByte = 0x03};

void setup() {
//  nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.advertise(robot_position_estimate);
  char inByte;
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  // initialize both serial ports:
  //Serial.begin(115200); //test UNO
  Serial.begin(19200);
  Serial3.begin(115200);
  delay(3000);
#ifdef PR
  //Serial.println("Starting");
#endif
  for (int i = 0; i < NUM_BEACONS; i++)
  {
    Stored_List[i].ID = ID_POS_List[i][0];
    Stored_List[i].Alive = 0; //-10000; //no valid measurement
    Stored_List[i].Dist = 0; //no valid measurement
  }
  ////Serial.println("Starting");
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

void Extract_Data()
{
  int Length = inBytes[0];
  int No_Of_Data = floor((Length - NUM_BEACONS) / NUM_BEACONS);
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
  ////Serial.println("Store");
  long int ID = (int)data_ptr->TxID_High;
  ID = 256 * ID + (int)data_ptr->TxID_Middle;
  ID = 256 * ID + (int)data_ptr->TxID_Low;


  //  //Serial.print(SatCnt);
  //  //Serial.print(",");

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


      //test UNO
      //    int IDNo = random(3);
      //    ID = ID_POS_List[IDNo][0];
      ////Serial.println(ID);
      // if (ID == 44529 || ID == 42867 || ID == 42928)  //test UNO
    {
      Stored_List[i].Alive = 10; //valid for the next 6 measurements
      Stored_List[i].Dist = meas_dist;
      Stat_cnt++; //valid measurement received - moved from GoTFSM

      //estimates stored for statistics - Transient removed in statistics
      if (Stat_cnt > Transient)
      {
        Stored_Pos[0][Stat_cnt] = x_est;
        Stored_Pos[1][Stat_cnt] = y_est;
        Stored_Pos[2][Stat_cnt] = z_est;
      }
    }
  }

#ifdef PR
  //Serial.print(ID);
  //Serial.print(",");
  //Serial.print(":");
  //Serial.print(x_est);
  //Serial.print(',');
  //Serial.print(y_est);
  //Serial.print(',');
  //Serial.print(z_est);
  //Serial.print(',');
  //Serial.print(Stat_cnt);
  //Serial.print(',');
  //Serial.print(meas_dist);
  //Serial.print(',');
  //Serial.print(SatCnt);
  //Serial.println(";");
#endif

  //  if (Stat_cnt > 10)
  //  {
  //    //Serial.println("............");
  //    //Serial.println(Stat_cnt);
  //    //Serial.println(x_est);
  //    Stored_Pos[0][Stat_cnt] = x_est;
  //    Stored_Pos[1][Stat_cnt] = y_est;
  //    Stored_Pos[2][Stat_cnt] = z_est;
  //    //Serial.println(Stored_Pos[0][Stat_cnt]);
  //  }

}

void Estimate_position(){
  ////Serial.println("hallo there");
  SatCnt = 0;
  for (int i = 0; i < NUM_BEACONS; i++) {
    if (Stored_List[i].Alive > 0)
      SatCnt++;
  }
  for (int i = 0; i < NUM_BEACONS; i++)
  {
    if (Stored_List[i].Alive > 0) { //if valid measurement
      //Stored_List[i].Alive = Stored_List[i].Alive - 1;
      //SatCnt++;
      double meas_dist = Stored_List[i].Dist;

      //compute distance between the so far estimated pt and transmitter
      double dp = (double)pow(x_est - (double)ID_POS_List[i][1], 2);
      dp += (double)pow(y_est - (double)ID_POS_List[i][2], 2);
      dp += (double)pow(z_est - (double)ID_POS_List[i][3], 2);
      dp = sqrt(dp);
      double lambda = 1 - dp / meas_dist;
      double alfa = (dp - meas_dist) / (dp + 10);

      //The 2 ways of projecting to circle should be the same
      //       if(abs(lambda-1)>0.01){
      //         x_est_new=x_est/(1-lambda)-(double)ID_POS_List[i][1]*lambda/(1-lambda);
      //         y_est_new=y_est/(1-lambda)-(double)ID_POS_List[i][2]*lambda/(1-lambda);
      //         z_est_new=z_est/(1-lambda)-(double)ID_POS_List[i][3]*lambda/(1-lambda);
      //       }
      if (z_est > 700)
        z_est = 700;
      if (SatCnt > 2) {
        x_est_new = x_est + alfa * ((double)ID_POS_List[i][1] - x_est);
        y_est_new = y_est + alfa * ((double)ID_POS_List[i][2] - y_est);
        z_est_new = z_est + alfa * ((double)ID_POS_List[i][3] - z_est);

        double dist_new = (double)pow(x_est - x_est_new, 2);
        dist_new += (double)pow(y_est - y_est_new, 2);
        dist_new += (double)pow(z_est - z_est_new, 2);
        if (dist_new > pow(0.2, 2))  //never step larger than 0.1 m to prevent rogue beacons
        //if (0)
        {
          x_est_new = x_est + 0.2 * ((double)ID_POS_List[i][1] - x_est)/dp;
          y_est_new = y_est + 0.2 * ((double)ID_POS_List[i][2] - y_est)/dp;
          z_est_new = z_est + 0.2 * ((double)ID_POS_List[i][3] - z_est)/dp;
        }
      }
      //xc=xp/(1-lambda)-y(:,j)*lambda/(1-lambda); //from MATLAB
      else
      {
        x_est_new = x_est;
        y_est_new = y_est;
        z_est_new = z_est;
      }
      if (z_est_new > 500)
        z_est_new = 500;

//      double dist_new = (double)pow(x_est - x_est_new, 2);
//      dist_new += (double)pow(y_est - y_est_new, 2);
//      dist_new += (double)pow(z_est - z_est_new, 2);

      //if(dist_new<2e7)
      if (1)
      {
        x_est = x_est_new;
        y_est = y_est_new;
        z_est = z_est_new;
      }
    }
  }
 //Serial.println(x_est);





}

void GoTFSM()
{
  //char inByte = 0;
  char inByte = Serial3.read();  //test UNO
  //Serial.print(inByte);
  switch (State) {
    case Idle:
      ByteCnt = 0;
      if (inByte == StartByte)
        State = StartByteRec;
      break;
    case StartByteRec:
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
            //Stat_cnt++; //one more measurement for statistics -  moved to store_distance
          }
          State = Idle;
          if (Serial.available()) {  //if stop signal goto statistics state
            inByte = Serial.read();
            ////Serial.println("stop received");
            Stat_State = 2; //start waiting for matlab to fetch results
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
      State = StartByteRec;
      inBytes[ByteCnt++] = (char)(inByte - 0x20);
  }
}

void Init_Store()
{
  int cnt;
  for (cnt = 0; cnt < 200; cnt++) //test UNO -> 200
  {
    Stored_Pos[0][cnt] = -1000;
  }
}

void Compute_Stats()
{
  int cnt, no_of_samp = 0;
  double Dist2;
  SumX = 0.0;
  SumY = 0.0;
  SumZ = 0.0;
  MaxDist = 0;
  MinDist = 1e10;

  for (cnt = 0; cnt < Sample_Size + Transient; cnt++)
  {
    //    //Serial.println(cnt);
    //    //Serial.println(Stored_Pos[0][cnt]);
    //    //Serial.println("--------------");
    if (Stored_Pos[0][cnt] > -1000)
    {
      no_of_samp++;
      SumX += Stored_Pos[0][cnt];
      SumY += Stored_Pos[1][cnt];
      SumZ += Stored_Pos[2][cnt];
    }

  }
  SumX = SumX / no_of_samp;
  SumY = SumY / no_of_samp;
  SumZ = SumZ / no_of_samp;

  SqSumX = 0.0;
  SqSumY = 0.0;
  SqSumZ = 0.0;

  for (cnt = 0; cnt < Sample_Size + Transient; cnt++)
  {
    if (Stored_Pos[0][cnt] > -1000)
    {
      SqSumX = SqSumX + pow(Stored_Pos[0][cnt] - SumX, 2);
      SqSumY = SqSumY + pow(Stored_Pos[1][cnt] - SumY, 2);
      SqSumZ = SqSumZ + pow(Stored_Pos[2][cnt] - SumZ, 2);
      Dist2 = pow(Stored_Pos[0][cnt] - SumX, 2) + pow(Stored_Pos[1][cnt] - SumY, 2);// + pow(Stored_Pos[2][cnt] - SumZ, 2);
      if (Dist2 > MaxDist)
        MaxDist = Dist2;
      if (Dist2 < MinDist)
        MinDist = Dist2;
    }
  }

  SqSumX = SqSumX / no_of_samp;
  SqSumY = SqSumY / no_of_samp;
  SqSumZ = SqSumZ / no_of_samp;
}


void loop() {
  nh.spinOnce();
  char inByte;
  switch (Stat_State) {
    case 0:
      //Serial.println("main loop 0");
      SqSumX = 0; SqSumY = 0; SqSumZ = 0;
      SumX = 0; SumY = 0; SumZ = 0;
      Stat_cnt = 0;
      //Serial.println("main loop 0 - 1");
      Init_Store();
      //Serial.println("main loop 0 - 2");
      digitalWrite(13, LOW);
      while (!Serial3.available()); //{delay(10); //Serial.println("wa");}
      //Serial.println("main loop 0 - 3");
      inByte = Serial.read();
      Tim = millis();
      Stat_State = 1;
      ////Serial.println("St0 -> St1");
      break;
    case 1:
      ////Serial.println("main loop 1");
      while (Serial3.available()) {  //test UNO
        GoTFSM();
      }
      //Store_distance();  //test UNO
      Estimate_position();

      ////Serial.println("after estimate_pos funk");
      //if (millis()-Tim > Sample_Time) {
      //if (Stat_cnt > Sample_Size + Transient) {
      //      if (Serial.available()) {
      //        //while (Serial.available()) {
      //        inByte = Serial.read();
      //        Stat_State = 2; //start waiting for matlab to fetch results
      //        ////Serial.println(Stat_cnt);
      //        ////Serial.println("St1 -> St2");
      //        //}
      //      }
      digitalWrite(13, HIGH);

      delay(1);
      break;
    case 2:
      //Serial.print("main loop 2");
      digitalWrite(13, LOW);
      //      while (!Serial.available());
      //      inByte = Serial.read();
      delay(100);
      Compute_Stats();
      estimate_xyz.x=x_est;
      estimate_xyz.y=y_est;
      estimate_xyz.z=z_est;
      robot_position_estimate.publish( &estimate_xyz );
      delay(1000);

      //Serial.print(":");
      //      //Serial.print(sqrt(SqSumX)); //std in Xdir
      //      //Serial.print(',');
      //      //Serial.print(sqrt(SqSumY)); //variance in Ydir
      //      //Serial.print(',');
      //      //Serial.print(sqrt(SqSumZ)); //variance in Zdir
      //      //Serial.print(',');
      //Serial.print(x_est); //mean in Xdir
      //Serial.print(',');
      //Serial.print(y_est); //mean in Ydir
      //Serial.print(',');
      //Serial.print(z_est); //mean in Zdir
      //      //Serial.print(',');
      //      //Serial.print(sqrt(MinDist)); //Minimum distance
      //      //Serial.print(',');
      //      //Serial.print(sqrt(MaxDist)); //Maximum distance
      //Serial.println(";");
      Stat_State = 0; //back to start state
      ////Serial.println("St2 -> St0");
  }
}
