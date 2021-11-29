#include <SoftwareSerial.h>

SoftwareSerial radio_module(4, 7); // TX 4, RX 7

const int message_size = 63; // how many chars the message can contain
char str[message_size]; // array of chars initialised, that will contain the sent message



void setup() {

  Serial.begin(115200); // terminal
  radio_module.begin(9600); // radio module

}



void loop() {

  Serial.println("Start the main loop");

  if (radio_module.available() > 0) { // is there something on the radio? (enable the radio)

    Serial.println("Radio module available");

    for (int i = 0; i < message_size; i++) {

      Serial.print("Reading  ");
      //Serial.println(char(radio_module.read())); //reading data
      char radio_data = char(radio_module.read()); // reading data in chars, saved in radio_data char
      str[i] = radio_data; // radio_data char passed to the str char array
      Serial.print(i);
      Serial.print(" ");
      Serial.println(str[i]);
      //delay(1000);
    }

    Serial.println(str);
    String radio_data_in_string_format = str; // radio_data that is in the str char array is now converted to a string XD

    Serial.println(radio_data_in_string_format);

  }




  delay(1000);





  //radio_module.write(35); //sending data
  //delay(10000);


}
