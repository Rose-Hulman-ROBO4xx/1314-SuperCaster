#include <stdlib.h>

void setup(){
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial2.begin(9600);
}

void loop(){
   int angle = 0;
   int mag = 0;
   char i = 0;
   for(i =0; i<20;i++){
      //Serial1.print(i);
      
      Serial1.print(angle);
      delay(15);
      Serial1.print('A');
      delay(15);
      Serial1.print(mag); 
      delay(15);
      Serial1.print('S');
      delay(15);
      Serial2.print(angle);
      delay(15);
      Serial2.print('A');
      delay(15);
      Serial2.print(mag); 
      delay(15);
      Serial2.print('S');
      Serial.print(angle);
      Serial.print("A ");
      Serial.print(mag); 
      Serial.print("S \n");
      delay(1000);
      angle += 18;
      mag += 50;
    }
  }
