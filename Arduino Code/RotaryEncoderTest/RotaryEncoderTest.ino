#include <stdlib.h>

void setup(){
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial2.begin(9600);
}

void loop(){
  //do nothing
  while(1);
}
  
void serialEvent(){
  while(Serial1.available()){
    char a1 = (char) Serial1.read();
    Serial.print("Left PIC angle: ");
    Serial.print(a1);
    Serial.print("\n");
  }
  while(Serial2.available()){
    char a2 = (char) Serial2.read();
    Serial.print("Right PIC angle: ");
    Serial.print(a2);
    Serial.print("\n");
  }
}
