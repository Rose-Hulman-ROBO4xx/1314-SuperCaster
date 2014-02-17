#include <stdlib.h>

int s1 = 0;
int s2 = 0;
int s0;
int ang = 0;
char a1;
char a2;
char a0;

void setup(){
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial.print("Ready\n");
}

void loop(){
  //do nothing
  /*if(s1){
    s1=0;
    Serial.print(a1);
    Serial.print('\n');
  }
  if(s2){
    s2=0;
    Serial.print(a2);
    Serial.print('\n');
  }
  if(s0){
    s0=0;
    Serial.print(a0);
    Serial.print('\n');
  }*/
  //Serial.print('\n');
  delay(100);
}
  
void serialEvent(){
  /*while(Serial.available()){
    a0 = (char)Serial.read();
    s0 = 1;
  }
  while(Serial2.available()){
    a2 = (char)Serial2.read();
    s2 = 1;    
  }*/  
  while(Serial1.available()){
     a1 = (char)Serial1.read();
     
     if(a1 == 'A'){
       Serial.print(" -> ");
       Serial.print(ang);
       Serial.print('\n');
       ang = 0;
     }else{
       ang = ang*10;
       ang = ang + a1;
       Serial.print(ang);
       Serial.print(" ");
     }
  }
  
}
