#include <SoftwareSerial.h>

SoftwareSerial radio_module(4, 7); //TX 4, RX 7




void setup() {
  Serial.begin(115200);
  radio_module.begin(9600);

  char str[] = "strtok needs to be called several times to split a string";
  char delim[] = " ";

  char *ptr = strtok(str, delim);

  while (ptr != NULL) {
    Serial.println(ptr);
    ptr = strtok(NULL, delim);
  }

}



void loop() {
  radio_module.write(35); //sending data
  delay(10000);

  if (radio_module.available() > 0) {
    Serial.print("Reading  ");
    Serial.println(char(radio_module.read())); //reading data
  }

}
