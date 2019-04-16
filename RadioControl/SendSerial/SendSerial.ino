#include <ArduinoJson.h>  // Using arduinoJSON version 6
StaticJsonDocument  <256> doc;
char json[] = "{\"Throttle\":\"100\",\"Steering\":125,\"Mode\":0,\"Stop\":1}"; 

void setup() {
  Serial.begin(9600); // set the baud rate
  auto error = deserializeJson(doc, json);
  if (error) {
      Serial.print(F("deserializeJson() failed with code "));
      Serial.println(error.c_str());
      return;
  }
  else{
    Serial.println("no error with json :D");
  }
}
void loop() {
 // char inByte = ' ';
  if(Serial.available()){ 
    //Serial.println(inByte); 

    //Serial.println(inByte); 
    serializeJson(doc, Serial);  
    Serial.println(""); 


  }
  delay(100); // delay for 1/10 of a second
}
