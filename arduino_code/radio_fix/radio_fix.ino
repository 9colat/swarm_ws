#include <SoftwareSerial.h>


/* APC220 reprogramming

* AUTO VERSION WHICH USE hw serial and pin 8 for set and if seelcted pin7 for enable radio
*
* (C)CJens Dalsgaard Nielsen (AAU)
* no warranty whatsoever
* you are on your own
* find m on github.com/jdn-aau/div
* * SEE readme.ino for more info (look up in tabs)
*/

/**** IMPORTANT ***/


int shieldID = 1;                                                                                                                                                          ;

// NB NB NB NB NB
// orange radioes V4  i 435000 freq range (431-455)
// green radioer i 475000 freq range  (455-485 ca)

unsigned long startF = 435000;
unsigned long stepF = 500;  // only relevant for fastprogramming in manuel mode with command "p"
unsigned long nrStep = 0;   // ...

// programming by p command  p 2  ==  f =  439000 + 2 * 500 =  440000 kHz for radio

// and rest of parameters as below - baud rate etc setup+
char rfBaud     = '3' ;   //   1.. 4   equals 2400(1km)/4800/9600/19200bps
char rfPower    = '9';    //   0 ..9   9(max power) equals 13dBm(20mW).
char uartBaud   = '3';    //   0.. 4   equals 1200/2400/4800/9600(3)/19200/
char uartParChk = '0';    //   0/1/2   equals NoCheck(8N1)/EvenParity(8E1)/OddParity

// ex: 3 9 3 0 : 9600 in air , full power, 9600 on rs232, no parity




/* READ HERE  READ HERE
  *
  * Jumpers
  * 1 on  - connect setup pin to D8
  * J2 on - connect radio tx to arduino rx (pin 0)
  * J3 on - connect radio rx to arduino tx (pin 1)
  * J4 and J5 off - no connection to openlog
  *
  * Setting parameters prior to auto programming
  * See fields below like rfBaud, rfPower etc

  * remove jumper 4 and 5
  * remove USB
  * attach battery and start Arduino on battery
  * wait 10 seconds or if AUTOMODELIGHT is defined it is finished when blinking LED13
  * remove battery
  * remove jumper 1
  * and program your normal progrtam :-)
  * happy hacking
  * ALSO FOR OLD SHIELD !!!
  * automode can also be used for old shield
  * In addition to the setup above setup pin has to be manually connected to D8 by wire
  * Then just go for mode 4 (shieldID = 4;)
  */





/**----------------------------------------------------------------------------------*/
void (*resetF)(void) = 0x00;

const int vrs = 3141;


char radioRcvStr[25]; // for receving from radio
/**----------------------------------------------------------------------------------*/

// enable pin turn radio on(HIGH). off(LOW)
// by software serial
#define ENPIN12 12
#define ENPIN7 7

// setpin radio on(high) off(low)
#define SETPIN 8

#define RXPIN 10
#define TXPIN 11

// D13 supply radio with power
#define FIVEV 13

// auto program for boardID + AUTOOFFSET
#define AUTOOFFSET 100

int enPin;

/**----------------------------------------------------------------------------------*/

#define PARM_LGT 10
#define CMD_BUF_LGT 50

char var1[PARM_LGT], var2[PARM_LGT], var3[PARM_LGT], var4[PARM_LGT], var5[PARM_LGT], var6[PARM_LGT];

SoftwareSerial apc220(RXPIN, TXPIN); // Crt softserial port and bind tx/rx to appropriate PINS

void emptySerial()
{
   int c;
   while (Serial.available())
     c = Serial.read();
}


/**----------------------------------------------------------------------------------*/
char st[200];
void radioFastProg(int index)
{
   int i = 0;

   st[i++] = ' W';
   st[i++] = ' R';
   st[i++] = ' ' ;

   ltoa(startF + index * stepF, st + i, 10);
   i = 0;
   while ( st[i] != 0x00)
     i++;

   st[i++] = ' ';
   st[i++] = rfBaud;
   st[i++] = ' ';
   st[i++] = rfPower;
   st[i++] = ' ';
   st[i++] = uartBaud;
   st[i++] = ' ';
   st[i++] = uartParChk;
   st[i] = 0x00;

   delay(10);

   radioPrintln(st);

   delay(20); // critical bq we are using softserial - so you have to be present in read for not loosing chars

   while (radioAvailable()) {
     i = radioRead();
     if (!shieldID) {
       Serial.print((char)(i));
     }
     delay(10);
   }
   delay(10);

}

/**----------------------------------------------------------------------------------*/
void set_para(char hz[], char rf_rate[], char pwr[], char uart_rate[], char sc[])
{
   // sanity chk
   if (strlen(hz) != 6) {
     Serial.println("Freq parm not 6 digits... - legal is 418000 - 455000");
     return;
   }
   if (strlen(rf_rate) != 1 ) {
     Serial.println("RF parm is not 1 digit: legal values is 1/2/3/4");
     return;
   }
   if (strlen(pwr) != 1 ) {
     Serial.println("Power parm is not 1 digit: legal values is 1..9");
     return;
   }
   if (strlen(uart_rate) != 1 ) {
     Serial.println("Uart baudrate parm is not 1 digit: legal values is 0..6");
     return;
   }
   if (strlen(sc) != 1 ) {
     Serial.println("Parity parm is not 1 digit: legal values is 0/1/2");
     return;
   }

   Serial.println("programming");

   delay(10);
   radioPrint("WR");
   radioPrint(" ");

   radioPrint(hz);
   radioPrint(" ");

   radioPrint(rf_rate);
   radioPrint(" ");

   radioPrint(pwr);
   radioPrint(" ");

   radioPrint(uart_rate);
   radioPrint(" ");

   radioPrint(sc);

   radioPrintln("");


   //0x0D);
   //radioWriteCh(0x0A);
   delay(10);

   // read feedback from radio

   while (radioAvailable()) {
     Serial.print((char)(radioRead()));
   }

}

/**----------------------------------------------------------------------------------*/
void get_para(char s[])
{
   int i = 0;

   delay(200);  // wait a second


   radioPrintln("RD");
   delay(10); // critical !

   while (radioAvailable()) {
     s[i] = radioRead();
     i++;
     // Serial.print((char)(radioRead()));
     delay(5);
   }

   i = 0;
   while ((s[i] != 0x0a) && (s[i] != 0x0d))
     i++;
   s[i] = 0x00; //termination of string
}


boolean isRadioPresent()
{
   radioRcvStr[0] = 0x00;
   get_para(radioRcvStr);
   if ( (radioRcvStr[0] == 'P') && (radioRcvStr[1] == 'A') && (radioRcvStr[2] == 'R'))
     return true;
   else
     return false;
}


/**----------------------------------------------------------------------------------*/
void setupShield(int id)
{
   shieldID = id;
   radioConfig(shieldID);
   delay(20);
   configPinInit();
   configModeOff();
   delay(500);
}

/**----------------------------------------------------------------------------------*/
void do_cmd(void) {
   int i = 0; char c;
   char buff[CMD_BUF_LGT];

   emptySerial();

   menu();

   buff[0] = 0;
   c = 0x00;

   while (c != 0x0a && c != 0x0d && i < CMD_BUF_LGT) {
     if (Serial.available()) {
       buff[i++] = c = Serial.read();
     }
     delay(5);
   }

   buff[i] = 0x00;

   var1[0] = 0x00; // reset
   Serial.println(buff);
   if (0 ==  sscanf(buff, "%s %s %s %s %s %s", var1, var2, var3, var4, var5, var6)) {
     return; //err
   }
   configModeOn();
   switch (var1[0]) { // one letter commands :-)
     case 'r':
       {
         Serial.print(">>> Read radio:");
         if (isRadioPresent())
           Serial.println(radioRcvStr);
         else
           Serial.println(F("no radio found on pin 8-13"));
         delay(1000);
         break;
       }
     case 'w':
       {
         Serial.print("write(config) radio ");
         set_para(var2, var3, var4, var5, var6);
         delay(1000);
         break;
       }
     case 'p':
       {
         long l;
         l = 0;
         l = atol(var2);
         Serial.println(l);
         radioFastProg(l);
         break;
       }
     case 'P': {
         drawRadio();
         break;
       }
     case 'R': resetF();
       break;
     case 'b':
       beacons();
       break;
     case 'c':
       {
         int i;
         for (i = 0 ;  i < 60; i++)
           Serial.println("");
         break;
       }
     default:
       {
         Serial.println("BAD COMMAND...");
         delay(500);
       }
   }
   configModeOff();
}

void beacons()
{
   delay(1000);
   configModeOn();
   delay(20);
   isRadioPresent();
   configModeOff();
   delay(200);
   radioWrite("BEACONMODE\n");
   while (1) {
     radioWrite("ping \n");
     delay(1000);
   }
}

boolean radioOn(int p)
{
   setupShield(p); // test for radio on p8-13

   delay(200);

   for (int i = 0 ; i < 3 ; i++) {
     configModeOn();
     if (isRadioPresent())
       return true;
     configModeOff();
     delay(500); // one more trial
   }

   delay(500);
   return false;
}

/**----------------------------------------------------------------------------------*/
void setup() {
   int jj;
   delay(500);
   pinMode(13, OUTPUT);
   digitalWrite(13, LOW);

   Serial.begin(9600);

   delay(100);

   dmpProgID(); // version of program

   Serial.println(F("Test starts for presence on apc220 radios in 1 second"));

   delay(500);

   Serial.print(F("APC220 on dig pin 8-13 ?.. "));

   delay(500);
  
   if (radioOn(0)) {
     goto xx;
   }

   delay(200);
  
   Serial.println(" nope");

   delay(200);
   digitalWrite(13, LOW); // radio power off
   delay(200);
   // let us test if the radio is on a shield and connected to HW serial port

   Serial.print(F("APC220 on AAU shield (and setup pin connected to dig pin 8) ? "));
   delay(1000);
   if (radioOn(1))
     goto xx;

   delay(500);

   Serial.println(" nope - beacon mode now");

   while (1) {
     radioWrite("ping\n ");
     delay(1000);
   }

xx:
   configModeOff();
   delay(400);
   Serial.print(" YEP - configurable radio found - configured as ");
   Serial.println(radioRcvStr);

   if (shieldID) {
     Serial.println("Auto programming on HW tx/rx in 3 seconds");
     delay(3000);
   }
}

/**----------------------------------------------------------------------------------*/
void loop()
{

   if (!shieldID) {

     do_cmd();
   }
   else { /* autoprogramming */
     delay(1000);
     configModeOn();
     Serial.println("Reprogramming on Serial port now");
     delay(500);
     radioFastProg(0);
     configModeOff();
 
     digitalWrite(13, HIGH); // Finish
     beacons();
   }
}
