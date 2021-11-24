#include <SoftwareSerial.h>

SoftwareSerial apc(4,7);


void setup()
{
   Serial.begin(115200);
   apc.begin(9600);
}

void loop()
{
  /*unsigned long now = millis();
  static unsigned long timer = 0;
  unsigned long interval = 1000;
  if(now - timer >= interval)
  {
   timer = millis();
   Serial.print("Sending  ");
   Serial.println(now);
   apc.println(now);
  }
  */
  {
  apc.write(35);
  delay(10000);// send a byte with the value 45

   //int bytesSent = apc.write(“hello”); //send the string “hello” and return the length of the string.
  }
  
  if(apc.available() > 0)
  {
    
   Serial.print("Reading  ");
   Serial.println(char(apc.read()));
  }
  else {
    //Serial.println("Not receiving data");
  }
}
